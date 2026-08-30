# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""A genuinely TENDON-DRIVEN MuJoCo model — building the robot in simulation (M42).

`mjcf.py` puts a `<position>` servo on every joint. That is a **direct-drive**
robot, and it has been the plant under every balance result since M17. The gap it
leaves is not a detail:

1. ⚠️ **A position servo can PUSH.** "A cable can only pull" is a load-bearing
   premise of [ADR-0002](../../../docs/DESIGN_DECISIONS.md) (why antagonistic pairs
   exist at all), ADR-0021 (why standing costs 76-87 % of moving for zero work) and
   ADR-0023 (why standing is the worst thermal case). **The simulation has never had
   that constraint.**
2. **The moment arm was a parameter, not a geometry.** `TendonParams.joint_moment_arm`
   was fed to the analytical map and the sim never saw a pulley.
3. **The joint coupling of ADR-0042 was absent.** A physical via-pulley couples the
   joints by its own radius per radian; a joint servo cannot.
4. **No cable compliance, no capstan friction, no spool.**

This module builds the other thing. Established by probe before anything was built
on it (MuJoCo 3.10):

- a `<spatial>` tendon routed over a cylinder `<geom>` **wraps** it, and
  `d(ten_length)/d(joint)` comes out as the cylinder radius to **0.25 %** — so the
  moment arm is *emergent from the geometry* rather than asserted;
- a `<motor tendon=...>` with `gear="-1"` shortens on positive control, and
  `ctrlrange="0 T"` makes it **physically pull-only**: commanding -600 N applies
  **+0.00 N**. `forcerange` is set too, belt-and-braces.

⚠️ **`mjcf.py` is deliberately untouched.** Every published figure from M17-M41 was
measured on it, and they have to stay reproducible while this is proven out.
"""

from __future__ import annotations

import math
import re

import numpy as np

from .params import (
    DEFAULT_FORELEG, DEFAULT_HINDLEG, DEFAULT_SPINE, DEFAULT_TENDON, GRAVITY,
)

#: ⚠️ **Pulley geoms are MASSLESS on purpose.** ADR-0041's manufacturing model
#: already apportions every sheave, via-pulley and bearing into
#: `LegParams.link_mass`, so giving the geoms their own mass double-counts. The
#: first build did, and it showed up as **4.532 kg against params' 4.3041** --
#: 0.224 kg, which is exactly 4 x the per-leg pulley masses. Caught by comparing
#: the compiled model's total against the parameter it is built from.
#: Sheave half-width (m). The groove is modelled as a plain cylinder: MuJoCo wraps
#: on the cylinder surface, which IS the cable pitch line, so the groove profile
#: would add nothing the tendon can feel.
SHEAVE_HALF_W = 0.004

#: ⚠️ **The hinge axis is `(0, -1, 0)`, and it matters.** `mjcf.py` documents why:
#: `LegModel.forward` builds the paw tip from cumulative angles with
#: `x = l cos a, z = l sin a`, so a POSITIVE joint angle must rotate +x toward +z.
#: MuJoCo's right-hand rule about +y does the opposite. The first pass here used
#: `(0, 1, 0)` and the whole leg pointed **upward** -- the foot came out at
#: z = +0.346 m, above the trunk, and the quadruped "stood" by sinking to the
#: floor with its joints held. Caught by printing the foot positions.

#: Via-pulley radius (m) — the cable's own minimum bend, 10 x Ø1.75 (ADR-0042).
#: Spool radius (m) -- `params`' own, so the wound length is `r * theta`.
SPOOL_R = float(DEFAULT_TENDON.motor_spool_radius)

#: Rotor inertia at the motor shaft (kg.m^2). `[assumed]`: the vendor does not
#: publish it and M39's spec review did not need it. It only has to be large enough
#: to keep the winding constraint well conditioned, and small enough not to dominate
#: the leg. ⚠️ Owed to the actuator story alongside the Kt question.
ROTOR_ARMATURE = 2e-5

#: Closed-loop bandwidth of the motor's own position servo (rad/s). A real motor
#: brings an encoder and a current loop; `spool_servo=True` models that as a
#: `<position>` actuator on the rotor, with gains DERIVED rather than tuned:
#: `kp = I * wn^2` and `kv = 2 * I * wn` for a critically damped rotor.
#:
#: ⚠️ **The servo's own compliance sits in SERIES with the G3 spring**, so the
#: stiffness actually delivered is `kp*k_tors/(kp + k_tors)`, not `k_tors`. ADR-0052
#: measured what that costs against ADR-0050's 150-200 kN/m band:
#:
#:     rotor bandwidth   kp (N.m/rad)   delivered (N/m)
#:     1000 rad/s               20.0            95 285   <- outside the band
#:     2000                     80.0           131 170
#:     3000                    180.0           141 004
#:     6000                    720.0           147 645
#:
#: **A firmware gain therefore sets how much of a mechanical spring you get.** 3000
#: rad/s is the default: it delivers 94 % of the specification and is a bandwidth a
#: real servo reaches.
ROTOR_BANDWIDTH = 3000.0

#: How the winding equality is solved. ⚠️ **These two are a measurement, not a
#: taste.** MuJoCo solves equalities in a normalised space, so an equality is itself
#: a spring in series with whatever it couples -- and a loose one silently softens
#: the drivetrain. Measured on a calibration rig at a known series stiffness:
#:
#:     solref          solimp                      k measured / k specified
#:     0.0005 1        0.9 0.95 0.001 0.5 2                  0.717
#:     0.0002 1        0.9 0.95 0.001 0.5 2                  0.939
#:     0.0002 1        0.99 0.9999 1e-6 0.5 2                **0.998**
#:
#: At the loose setting the equality contributed 3.8e5 N/m in series; at the tight
#: one, 8.2e7, which is negligible against the ~1.5e5 the cable and the G3 element
#: actually have. `0.0002` is also the floor MuJoCo's stability wants at this
#: timestep (2x dt).
EQ_SOLREF = "0.0002 1"
EQ_SOLIMP = "0.99 0.9999 1e-6 0.5 2"

#: ⚠️ **Without `spools=True` this model has NO SPOOL DEGREE OF FREEDOM, and M45
#: found where that bites.** A tendon's length here is purely a function of the joint angles, so a
#: motor cannot **pay cable out** -- a slack antagonist behaves as a spring instead.
#: Driving one ankle tendon at 223 N against a *zero-commanded* antagonist, the
#: antagonist stretched 2.13 mm and developed **273.8 N**, stopping the joint at
#: 8.9 deg. That is the cable's elasticity, not any property of the pair.
#:
#: Everything measured on this plant so far is unaffected, because moment arms,
#: joint stiffness and pose-holding are all small perturbations about a pose where
#: both cables are taut. ⚠️ But **the TRAVEL of an antagonistic pair cannot be
#: measured here**, and that is exactly what ADR-0002 Option A is meant to buy. See
#: ADR-0050. ✅ **M46 fixes it** -- see `spools=True`, which puts a rotor, a
#: series spring and a winding constraint behind every cable. The old plant is kept
#: as the default, because every M42-M45 measurement was taken on it.
NO_SPOOL_DOF = True

VIA_R = 0.00875

#: Cable: Ø1.75 UHMWPE. `stiffness` is the axial spring rate the analytical model
#: carries as `cable_stiffness`; `damping` is `[assumed]` and small.
CABLE_WIDTH = 0.00088
#: Settled UHMWPE tensile modulus, Pa. `[sourced: LEG_TENDON_SPEC §2]` — the fibre
#: is 100-120 GPa (SK99), a spliced settled run 50-90; 60 is the spec's own figure.
CABLE_E = 60e9

#: ⚠️ **A single `stiffness` number is wrong, and that is why the first pass NaN'd.**
#: `k = EA/L` is per-tendon and run-length dependent — LEG_TENDON_SPEC §2 says so
#: explicitly (*"kinematics should compute it from the per-tendon path length, not a
#: single constant"*), and §5.2's proposed `cable_stiffness = 3.5e5` is a single
#: constant anyway. At the routed lengths the real values span **5.5e5–1.2e6 N/m**.
#:
#: A spatial tendon's `stiffness` pulls toward `springlength`. Given ONE value it is
#: a two-sided spring — not a cable, and what fought the actuator into a NaN at
#: t = 0.168 s. Given TWO it is a deadband: slack below, elastic above, which IS a
#: cable. `single_leg_rig_elastic` builds twice to set each tendon's own deadband.
CABLE_STIFFNESS = None      # per-tendon; see `_cable_k`
CABLE_DAMPING = 0.02

#: ⚠️ **Peak tendon tension the MOTOR can produce (N) — not the cable's rating.**
#:
#: The first pass set this to 700 N from ADR-0046's 638 N land transient. That is a
#: **structural** number: what the cable, pulley and bearing must survive when the
#: GROUND hits the foot. It is not what the motor can pull. Giving a controller 700 N
#: of authority on twenty tendons is 14 kN on a 42 N robot, and the gate showed
#: exactly that: a 0.1 s contact transient saturated every tendon and **launched the
#: quadruped off the floor** (z 0.176 -> 0.834 m, airborne, ncon = 0).
#:
#: The real ceiling is `tau_motor_peak / r_spool` = 1.95 / 0.00875 = **223 N**, and
#: the CONTINUOUS one is 0.71 / 0.00875 = **81 N**. So the structure carries 2.9x
#: more than the motor can ever apply — which is right, because the land transient
#: arrives from the ground rather than from the actuator.
MOTOR_PEAK_NM = 1.95            # GIM3505-9 peak, `[sourced: motor-downselect]`
MOTOR_RATED_NM = 0.71           # its CONTINUOUS rating
TENSION_MAX = MOTOR_PEAK_NM / float(DEFAULT_TENDON.motor_spool_radius)
TENSION_CONTINUOUS = MOTOR_RATED_NM / float(DEFAULT_TENDON.motor_spool_radius)

#: Ankle return spring (ADR-0002 Option B): the ankle has ONE tendon, so the joint
#: itself carries the return. N.m/rad, from `TendonParams.spring_stiffness[2]`.
ANKLE_SPRING = float(DEFAULT_TENDON.spring_stiffness[2])

#: Rest angle of that spring (rad). `params` says **0.0**, and ⚠️ **that is 97 deg
#: away from the hind leg's stance hock angle (+1.694 rad)** -- so as specified the
#: ADR-0002 Option-B return spring does not *return* the ankle to its stance, it
#: **fights** it with a constant -0.508 N.m. ADR-0049 measured what that costs:
#: with the spring at 0 the standing allocation asks the worst tendon for the full
#: 222.9 N ceiling; referenced at the stance angle it asks 207.4 N.
#:
#: ⚠️ `mjcf_tendon` hard-coded `springref="0.0"` and never read this parameter at
#: all -- the same class of params bypass M41's fold-in existed to remove. The rigs
#: now pass the stance angle explicitly and say why; `spring_rest_angle[2]` itself
#: is a mechanical decision that ADR-0049 hands back.
ANKLE_SPRINGREF = float(DEFAULT_TENDON.spring_rest_angle[2])

#: ⚠️ **What the ADR-0002 Option-B return spring would have to be (N.m/rad), and
#: `params` specifies 0.3.** M45 measured the unloaded leg under a proper
#: non-negative allocation: at 0.3 the lone-tendon ankle sags **-14.6 deg** and the
#: worst tendon needs 58.6 N to fight it. Stiffened and referenced at the stance
#: hock (which M44 fixed), it holds:
#:
#:     k3 (N.m/rad)   0.3     1.0     2.0     4.0     8.0    16.0
#:     ankle drift  -14.62   -4.55   -2.29   -1.15   -0.58   -0.29
#:     peak tension   58.6    38.7    27.3    17.7    13.3    11.3
#:
#: **~8 N.m/rad is the knee of that curve**, and it reaches ADR-0002 Option A's
#: holding performance (0.00 deg at 12.2 N) **without a single extra motor**.
#:
#: ⚠️ **Its cost is TRAVEL, and it is severe.** Driving the lone tendon at the
#: 223 N motor peak moves the ankle **-196 deg from stance at k3 = 0.3 but only
#: -24.9 deg at k3 = 8** -- the spring eats the range of motion. ADR-0050 hands
#: mechanical the trade rather than picking for them; `params.spring_stiffness[2]`
#: is untouched.
ANKLE_SPRING_TO_HOLD = 8.0


def _rod_inertia(mass: float, length: float, radius: float) -> tuple:
    """Slender rod about its own centre — matches `mjcf.py` so the two agree."""
    ixx = mass * (3.0 * radius * radius + length * length) / 12.0
    izz = 0.5 * mass * radius * radius
    return (ixx, ixx, izz)


def _box_inertia(mass: float, hx: float, hy: float, hz: float) -> tuple:
    """Solid-box diagonal inertia about its COM — matches `mjcf.py` exactly."""
    return (
        mass * ((2 * hy) ** 2 + (2 * hz) ** 2) / 12.0,
        mass * ((2 * hx) ** 2 + (2 * hz) ** 2) / 12.0,
        mass * ((2 * hx) ** 2 + (2 * hy) ** 2) / 12.0,
    )


def _cable_k(length_m: float, dia_m: float = 1.75e-3) -> float:
    """Axial stiffness of one cable run, `EA/L` — per-tendon, as §2 requires."""
    area = math.pi * (dia_m / 2.0) ** 2
    return CABLE_E * area / max(length_m, 1e-4)


def spool_xml(tendon: str, site: str, pos, k_series: float, indent: int = 6,
              a0: float = 0.0, servo: bool = False):
    """A motor, a series spring and a winding constraint for one cable.

    Returns `(body, fixed_tendon, equality, actuator)`.

    The topology is a **series-elastic actuator**, and each piece is where it
    physically is:

        motor torque -> rotor -> torsional spring -> spool -> cable

    ⚠️ **The spool SITE does not move.** It is the point the cable leaves the
    housing, and it stays fixed while the spool turns behind it. That is not a
    simplification: MuJoCo's spatial tendons are **memoryless about winding** -- a
    cylinder is rotationally symmetric, so wrapping one and turning it changes the
    path by nothing at all, and a site carried on the rotor merely orbits. The wound
    length has to be carried **analytically**, by a `<fixed>` tendon on the spool
    angle, and tied to the geometric path by an equality. M46 built all three wrong
    versions before this one.

    The series spring is specified in N.m/rad and relates to the cable's linear
    stiffness as `k_tors = k_lin * r^2`, which the calibration in `EQ_SOLREF`
    reproduces to 0.2 %.

    ⚠️ **`a0` is not optional, and leaving it zero does not fail loudly.**
    MuJoCo takes a tendon equality's reference lengths at the model's **`qpos0`**,
    not at whatever state the caller initialises. Build the rig with the joints at
    zero and start it at the stance pose and every constraint begins **centimetres**
    out: measured 24-52 mm across the five cables, after which the solver snapped the
    leg from 97 deg to 39 deg in 5 ms and then held the wrong configuration
    perfectly. The residuals sit CONSTANT, which reads like a satisfied constraint
    until you notice what they are constant at. `a0` is the path length at the
    stance pose minus its length at `qpos0`, and the rigs measure it in a first
    pass.
    """
    pad = " " * indent
    k_tors = float(k_series) * SPOOL_R * SPOOL_R
    body = "\n".join([
        f'{pad}<body name="rotor_{tendon}" pos="{pos[0]:.5f} {pos[1]:.5f} '
        f'{pos[2]:.5f}">',
        f'{pad}  <joint name="jr_{tendon}" type="hinge" axis="0 -1 0" '
        f'damping="0.002" armature="{ROTOR_ARMATURE:.3e}"/>',
        f'{pad}  <geom name="gr_{tendon}" type="cylinder" size="0.012 0.004" '
        f'quat="0.70711 0.70711 0 0" mass="1e-9" contype="0" conaffinity="0"/>',
        f'{pad}  <body name="spool_{tendon}">',
        f'{pad}    <joint name="js_{tendon}" type="hinge" axis="0 -1 0" '
        f'damping="0.0005" armature="1e-6" stiffness="{k_tors:.6f}" '
        f'springref="0"/>',
        f'{pad}    <geom name="gs_{tendon}" type="cylinder" '
        f'size="{SPOOL_R:.5f} 0.004" quat="0.70711 0.70711 0 0" mass="1e-9" '
        f'contype="0" conaffinity="0"/>',
        f'{pad}  </body>',
        f'{pad}</body>',
    ])
    wind = (f'    <fixed name="w_{tendon}">\n'
            f'      <joint joint="jr_{tendon}" coef="{SPOOL_R:.6f}"/>\n'
            f'      <joint joint="js_{tendon}" coef="{SPOOL_R:.6f}"/>\n'
            f'    </fixed>')
    eq = (f'    <tendon tendon1="{tendon}" tendon2="w_{tendon}" '
          f'polycoef="{a0:.9f} -1 0 0 0" solref="{EQ_SOLREF}" '
          f'solimp="{EQ_SOLIMP}"/>')
    if servo:
        # ⚠️ The force range is SIGNED, and that is a correction M46 made: a motor
        # can turn either way, and "a cable can only pull" is a property of the
        # CABLE. The old plant put the actuator on the tendon, where a one-sided
        # `ctrlrange` was the right way to say it; with a spool in between, clamping
        # the motor to one sign also removes its ability to damp its own drivetrain.
        wn = ROTOR_BANDWIDTH
        act = (f'    <position name="m_{tendon}" joint="jr_{tendon}" '
               f'kp="{ROTOR_ARMATURE * wn * wn:.5f}" '
               f'kv="{2.0 * ROTOR_ARMATURE * wn:.6f}" '
               f'forcerange="-{MOTOR_PEAK_NM:.4f} {MOTOR_PEAK_NM:.4f}" '
               f'forcelimited="true"/>')
    else:
        act = (f'    <motor name="m_{tendon}" joint="jr_{tendon}" gear="1" '
               f'ctrlrange="0 {MOTOR_PEAK_NM:.4f}" ctrllimited="true" '
               f'forcerange="0 {MOTOR_PEAK_NM:.4f}" forcelimited="true"/>')
    return body, wind, eq, act


#: Which spool site each tendon leaves from. The tendon's own first site, by name.
SPOOL_OF = {
    "hip_flex": "spool_hip", "hip_ext": "spool_hip_x",
    "knee_flex": "spool_knee", "knee_ext": "spool_knee_x",
    "ankle": "spool_ankle", "ankle_ext": "spool_ankle_x",
}


def spool_positions(gx: float = 0.0, sy: float = 0.0):
    """Where each spool sits, relative to the girdle it is mounted on.

    One source for both the tendon's exit `<site>` and, under `spools=`, the rotor
    body behind it -- they have to agree exactly or the cable leaves from somewhere
    the motor is not.
    """
    return {
        "spool_hip": (gx - 0.042, sy + 0.012, 0.034),
        "spool_hip_x": (gx - 0.042, sy + 0.012, -0.034),
        "spool_knee": (gx - 0.050, sy + 0.024, 0.034),
        "spool_knee_x": (gx - 0.050, sy + 0.024, -0.030),
        "spool_ankle": (gx - 0.058, sy + 0.030, 0.034),
        "spool_ankle_x": (gx - 0.066, sy + 0.030, -0.030),
    }


#: Which spool each PAIR tendon winds on, under `pulley=True`.
#: ✅ One spool per pair, not per cable, because with a variable-radius pulley the
#: spool **is** the pulley: it takes up one cable of the pair while paying out the
#: other, so what winds on it is their difference -- exactly the `<fixed>` tendon
#: `pulley_tendons` emits. It sits at the flexor's spool position because that is
#: where ADR-0058 put the one motor.
PAIR_SPOOL_OF = {"hip": "spool_hip", "knee": "spool_knee", "ankle": "spool_ankle"}


def pair_names(prefix: str, ankle_pair: bool = True):
    """This leg's PAIR tendons, in the order `pulley_actuators` emits them."""
    names = ["hip", "knee"] + (["ankle"] if ankle_pair else [])
    return [f"{prefix}_{n}" for n in names]


def tendon_names(prefix: str, ankle_pair: bool = True, pulley: bool = False):
    """This leg's tendons, in the order the actuators are emitted."""
    if pulley:
        return pair_names(prefix, ankle_pair)
    names = ["hip_flex", "hip_ext", "knee_flex", "knee_ext", "ankle"]
    if ankle_pair:
        names.append("ankle_ext")
    return [f"{prefix}_{n}" for n in names]


def drivetrain_xml(prefix: str, ankle_pair: bool, k_series: float,
                   gx: float = 0.0, sy: float = 0.0, indent: int = 6,
                   a0: dict | None = None, servo: bool = False,
                   pulley: bool = False):
    """Every spool for one leg. Returns `(bodies, winds, equalities, actuators)`.

    `a0` maps tendon name -> its path length at the reference pose minus its length
    at `qpos0`. See `spool_xml`: without it the constraints start violated.

    ✅ **`pulley=True` is what makes G3 reachable on the shipped transmission.**
    ADR-0051 put the series-elastic element in the drivetrain, between rotor and
    spool -- so until the drivetrain could be built behind a pulley, the design goal
    had nowhere to live on the robot ADR-0058 chose. One spool per pair, six fewer
    bodies per leg, and the winding equality binds the pair's **differential**
    length, which is the quantity the motor actually commands.
    """
    pos = spool_positions(gx, sy)
    b, w, e, a = [], [], [], []
    of = PAIR_SPOOL_OF if pulley else SPOOL_OF
    for tname in tendon_names(prefix, ankle_pair, pulley):
        short = tname.split("_", 1)[1]
        site = f"{prefix}_{of[short]}"
        bb, ww, ee, aa = spool_xml(tname, site, pos[of[short]],
                                   k_series, indent=indent,
                                   a0=(a0 or {}).get(tname, 0.0), servo=servo)
        b.append(bb)
        w.append(ww)
        e.append(ee)
        a.append(aa)
    nl = chr(10)
    return nl.join(b), nl.join(w), nl.join(e), nl.join(a)


#: Which joints each antagonistic PAIR drives, and with what differential arm.
#: ⚠️ The via-pulley terms are here because the pair's two cables run on
#: **opposite** sides of each via-pulley. Run on the same side -- which is what M42
#: built and what `clamped_tendons` still emits for independent motors -- the
#: coupling leaves the differential entirely and becomes **common mode**, which a
#: pulley cannot absorb: 11.44 mm of it at the knee over the hind gait range, 20.60
#: at the ankle, worth **1716 N and 3090 N** of co-contraction swing against a 638 N
#: cable rating. Opposite sides put it back in the differential and leave the common
#: mode at exactly zero.
def pair_rows(arms, ankle_pair: bool = True):
    r_hip, r_knee, r_ankle = (float(a) for a in arms)
    v = VIA_R
    rows = {"hip": [("q1", r_hip)],
            "knee": [("q1", -v), ("q2", r_knee)]}
    if ankle_pair:
        rows["ankle"] = [("q1", -v), ("q2", -v), ("q3", r_ankle)]
    return rows


def pulley_tendons(name: str, arms, ankle_pair: bool = True) -> str:
    """One `<fixed>` tendon per antagonistic PAIR -- [ADR-0008](../../../docs/DESIGN_DECISIONS.md).

    A variable-radius pulley takes up one cable of a pair while paying out the
    other, so the motor commands their **difference**. With both cables clamped that
    difference is `sum r_j q_j` over the joints the pair crosses, which is one fixed
    tendon per pair rather than two per joint.

    ✅ **This is the transmission ADR-0008 decided and the simulation never had**:
    12 leg motors, not 20, and the mass budget closes at 4.30 kg where the
    independent-pair architecture put it at 5.36 (ADR-0056).

    ⚠️ **What it gives up is co-contraction as a control input** (ADR-0057). The
    allocation stops being redundant: three joints, three motors, no null space, so
    `T_bias` is whatever the pulley's radius profile schedules against angle. That is
    ADR-0002's *"stiffness becomes commandable"* traded for ADR-0008's mass closure,
    and it was decided rather than discovered.
    """
    rows = pair_rows(arms, ankle_pair)
    out = []
    for pair, terms in rows.items():
        out.append(f'    <fixed name="{name}_{pair}">')
        for jn, coef in terms:
            out.append(f'      <joint joint="{name}_{jn}" coef="{coef:.6f}"/>')
        out.append("    </fixed>")
    return "\n".join(out)


def pulley_actuators(name: str, ankle_pair: bool = True) -> str:
    """One BIDIRECTIONAL motor per pair.

    ✅ Bidirectional is right here and one-sided was right before: the pull-only
    constraint is a property of a **cable**, and a pair covers both directions, so
    the pulley the pair drives turns either way. The torque limit is the motor's,
    `MOTOR_PEAK_NM`, not a tension.
    """
    pairs = ["hip", "knee"] + (["ankle"] if ankle_pair else [])
    return "\n".join(
        f'    <motor name="m_{name}_{p}" tendon="{name}_{p}" gear="-1" '
        f'ctrlrange="-{TENSION_MAX:.0f} {TENSION_MAX:.0f}" ctrllimited="true" '
        f'forcerange="-{TENSION_MAX:.0f} {TENSION_MAX:.0f}" forcelimited="true"/>'
        for p in pairs)


def clamped_tendons(name: str, arms, ankle_pair: bool = True) -> str:
    """Every cable of one leg, as a CLAMPED capstan.

    ✅ [ADR-0055](../../../docs/DESIGN_DECISIONS.md) settled the construction: a
    cable **fixed to the sheave** has a moment arm of exactly `r` at every angle,
    where one that merely rests on it has a working window. The hind hip's window is
    `q1 >= -10 deg` against a gait range of -110..-35.1, and across that the resting
    arm swings **21.0 to 36.3 mm on a 28 mm specification**.

    A clamped cable's length is therefore **exactly** `sum r_j * q_j` over the joints
    it crosses, which is a `<fixed>` tendon and not a routed path.

    ⚠️ **That means the moment arm stops being emergent, and it is worth being
    plain about what that costs.** ADR-0047's headline -- the arm comes out as the
    sheave radius from the geometry alone, and ADR-0042's +/-8.75 mm/rad coupling
    with it -- was measured on the wrapped construction, at the stance pose. It is a
    real *validation* that a wrap reproduces the radius where it wraps. It is not
    the model any more: with the cable clamped, the map **is** the analytic one
    `TendonMap` has carried since M4, and the simulation no longer derives it
    independently.

    Signs are the design intent, and they match what the wrapped plant measured at
    the stance pose where its wrap is correct: the pair straddles each sheave, and
    every distal cable picks up the proximal joints at the via-pulley radius.
    """
    r_hip, r_knee, r_ankle = (float(a) for a in arms)
    v = VIA_R
    rows = {
        "hip_flex": [("q1", +r_hip)],
        "hip_ext": [("q1", -r_hip)],
        "knee_flex": [("q1", -v), ("q2", +r_knee)],
        "knee_ext": [("q1", -v), ("q2", -r_knee)],
        "ankle": [("q1", -v), ("q2", -v), ("q3", -r_ankle)],
    }
    if ankle_pair:
        rows["ankle_ext"] = [("q1", -v), ("q2", -v), ("q3", +r_ankle)]
    out = []
    for tname in tendon_names(name, ankle_pair):
        short = tname.split("_", 1)[1]
        out.append(f'    <fixed name="{tname}">')
        for jn, coef in rows[short]:
            out.append(f'      <joint joint="{name}_{jn}" coef="{coef:.6f}"/>')
        out.append("    </fixed>")
    return "\n".join(out)


def clamped_actuators(name: str, ankle_pair: bool = True) -> str:
    """Pull-only motors on the clamped cables, one per tendon.

    ⚠️ One per TENDON, which is [ADR-0002](../../../docs/DESIGN_DECISIONS.md)'s
    architecture and not [ADR-0008](../../../docs/DESIGN_DECISIONS.md)'s. ADR-0056
    found the two conflict and nobody has decided between them; see ADR-0057. This
    keeps the count the simulation has always had so that the transmission change
    is measured on its own.
    """
    return "\n".join(
        f'    <motor name="m_{t}" tendon="{t}" gear="-1" '
        f'ctrlrange="0 {TENSION_MAX:.0f}" ctrllimited="true" '
        f'forcerange="0 {TENSION_MAX:.0f}" forcelimited="true"/>'
        for t in tendon_names(name, ankle_pair))


def _ankle_anchor_deg(leg_p) -> float:
    """Where this leg's ankle cables anchor on the sheave, in degrees.

    ⚠️ **It has to be PER LEG, and three milestones of sweeping got there by
    tightening the criterion each time.**

    - M42 asked only that the cable **wrap** (45 deg).
    - M44 asked that the moment arm have the sign STANDING needs at the **stance
      pose** (300 deg). That is necessary and not sufficient.
    - M47 asks that the arm not **reverse anywhere inside the GAIT's range**, that
      the pair span both directions across the whole ROM, and that both arms stay
      near the 14 mm specification. Driving through a reversal, the arm changes sign
      under the controller and the joint runs to the opposite end stop: commanded
      122.3 deg, reached **-30**.

    Swept against the M47 criterion, no single angle serves both legs, because their
    hocks stand **81 deg apart** (hind +97.1, fore +16.4) and their gait ranges
    therefore sit in different parts of the sheave:

        leg    trot range      anchor   worst arm error
        hind   88.5-122.3      270 deg  1.57 mm
        fore    3.0- 78.8      300 deg  0.30 mm

    ⚠️ 270 deg puts a reversal at ~70-80 deg, inside the FORE range, and M47's
    first filter missed it by sampling the fore range only to 75 deg. The fore leg
    being "inherited rather than mirrored" -- flagged since M43 -- is finally forced
    here, and the fix is a number per leg rather than a mirrored construction.

    ⚠️ **M47 measured the replacement but does NOT ship it, and that is
    deliberate.** Adopting 270 deg for the hind leg breaks **14 tests across M44,
    M45 and M46** -- every ankle measurement in three milestones was taken on the
    300 deg anchor. That migration is its own piece of work with its own
    re-derivation, and M47's deliverable is the drivetrain cascade, which does not
    depend on the anchor at all. Both numbers are recorded here so the migration
    starts from a measurement rather than a re-sweep.

    So: 300 deg for both legs, and the hind ankle consequently **cannot be driven
    above ~100 deg** -- the reversal sits at ~112, inside its 88.5-122.3 deg gait
    range. That is a sharper statement of the question ADR-0050 left open: not "can
    a pair reach it" but "the pair can, once the reversal is moved out of the way".
    """
    return 300.0


def _stance_ankle(leg_p) -> float:
    """The ankle angle this leg holds in the nominal stance, from its own IK.

    ⚠️ Where the return spring should be referenced. Fore and hind differ by
    **81 deg** (+16.4 vs +97.1), so this cannot be one number -- and it is why the
    two legs' ankles behave so differently under load (ADR-0049).
    """
    from .leg import LegModel

    return float(LegModel(leg_p).inverse((0.04, -0.17, 0.0))[2])


def leg_tendon_xml(name: str, leg_p, arms, indent: int = 4,
                   elastic: dict | None = None,
                   mount=(0.0, 0.0, 0.0),
                   ankle_springref: float | None = None,
                   ankle_pair: bool = True,
                   ankle_spring: float | None = None,
                   clamped: bool = True,
                   pulley: bool = True) -> tuple[str, str, str]:
    """One tendon-driven leg. Returns (body_xml, tendon_xml, actuator_xml).

    The kinematic chain is the same four links `mjcf.py` builds. What is added:

    - a **sheave** cylinder geom at each joint, on the **DISTAL** body, radius =
      that joint's moment arm. ASSEMBLY_SPEC §2's critical rule: fix the sheave to
      the distal link or the tendon does no work.
    - a **via-pulley** cylinder concentric with each proximal joint, on the
      *proximal* body, for the tendons that have to get past it.
    - sites for the spool, the anchors, and the `sidesite` each wrap needs.
    """
    pad = " " * indent
    L = leg_p.link_lengths
    m = leg_p.link_mass
    r_hip, r_knee, r_ankle = arms

    def bone(i, tag):
        ixx = _rod_inertia(m[i], L[i], 0.005)
        return (f'{pad}  <geom name="{name}_{tag}" type="capsule" '
                f'fromto="0 0 0 {L[i]:.5f} 0 0" size="0.005" mass="{m[i]:.5f}"/>\n'
                f'{pad}  <!-- I = {ixx[0]:.3e} -->\n')

    # ------------------------------------------------------------------ bodies
    b = []
    b.append(f'{pad}<body name="{name}_femur" pos="{mount[0]:.5f} '
             f'{mount[1]:.5f} {mount[2]:.5f}">')
    b.append(f'{pad}  <joint name="{name}_q1" type="hinge" axis="0 -1 0" '
             f'range="{leg_p.q_min[0]:.4f} {leg_p.q_max[0]:.4f}" damping="0.002"/>')
    # the hip sheave rides on the FEMUR (the distal link of the hip joint)
    b.append(f'{pad}  <geom name="{name}_hip_sheave" type="cylinder" '
             f'size="{r_hip:.5f} {SHEAVE_HALF_W}" pos="0 0.012 0" '
             f'quat="0.70711 0.70711 0 0" mass="1e-9" '
             f'contype="0" conaffinity="0"/>')
    # via-pulley for the knee/ankle tendons, concentric with the HIP axis.
    # ⚠️ Every `_via_side` and `_mid` site below sits at **+z**, and that sign is
    # not cosmetic. It is set by the hinge-axis convention: with `axis="0 -1 0"`
    # the leg folds DOWNWARD, so the routed cable passes the via-pulley on the
    # opposite side from the one the first pass assumed. Built at -z the wraps
    # come apart -- the knee flexor loses its moment arm entirely (1.17 mm/rad
    # against 25) and the couplings read 11.7/36.4/41.5 instead of 8.75.
    b.append(f'{pad}  <geom name="{name}_hip_via" type="cylinder" '
             f'size="{VIA_R:.5f} 0.003" pos="0 0.024 0" '
             f'quat="0.70711 0.70711 0 0" mass="1e-9" '
             f'contype="0" conaffinity="0"/>')
    b.append(f'{pad}  <site name="{name}_hip_anchor" pos="{0.55 * r_hip:.5f} '
             f'0.012 {-(r_hip + 0.005):.5f}" size="0.0015"/>')
    b.append(f'{pad}  <site name="{name}_hip_side" pos="0 0.012 '
             f'{-(r_hip + 0.02):.5f}" size="0.001"/>')
    # ⚠️ The GATE found this: an extensor routed straight to the anchor with no
    # wrap geom gets whatever moment arm the geometry happens to give -- 19.9 mm
    # instead of 28. A sheave the tendon does not touch does no work. Each
    # antagonist needs the SAME sheave with the OPPOSITE sidesite.
    b.append(f'{pad}  <site name="{name}_hip_side_x" pos="0 0.012 '
             f'{(r_hip + 0.02):.5f}" size="0.001"/>')
    b.append(f'{pad}  <site name="{name}_hip_anchor_x" pos="{0.55 * r_hip:.5f} '
             f'0.012 {(r_hip + 0.005):.5f}" size="0.0015"/>')
    b.append(f'{pad}  <site name="{name}_hip_via_side" pos="0 0.024 '
             f'{(VIA_R + 0.02):.5f}" size="0.001"/>')
    # ⚠️ MuJoCo requires every wrap geom to be BRACKETED BY SITES -- two
    # consecutive `<geom>` entries are rejected. Physically that is right: between
    # two pulleys the cable runs free, and a site on the tangent line is how you
    # say so. These sit on the bone axis at the pulley's own lateral plane.
    b.append(f'{pad}  <site name="{name}_femur_mid" pos="{0.5 * L[0]:.5f} 0.024 '
             f'{VIA_R:.5f}" size="0.001"/>')
    b.append(bone(0, "femur").rstrip())

    b.append(f'{pad}  <body name="{name}_tibia" pos="{L[0]:.5f} 0 0">')
    b.append(f'{pad}    <joint name="{name}_q2" type="hinge" axis="0 -1 0" '
             f'range="{leg_p.q_min[1]:.4f} {leg_p.q_max[1]:.4f}" damping="0.002"/>')
    b.append(f'{pad}    <geom name="{name}_knee_sheave" type="cylinder" '
             f'size="{r_knee:.5f} {SHEAVE_HALF_W}" pos="0 0.012 0" '
             f'quat="0.70711 0.70711 0 0" mass="1e-9" '
             f'contype="0" conaffinity="0"/>')
    b.append(f'{pad}    <geom name="{name}_knee_via" type="cylinder" '
             f'size="{VIA_R:.5f} 0.003" pos="0 0.024 0" '
             f'quat="0.70711 0.70711 0 0" mass="1e-9" '
             f'contype="0" conaffinity="0"/>')
    b.append(f'{pad}    <site name="{name}_knee_anchor" pos="{0.55 * r_knee:.5f} '
             f'0.012 {-(r_knee + 0.005):.5f}" size="0.0015"/>')
    b.append(f'{pad}    <site name="{name}_knee_side" pos="0 0.012 '
             f'{-(r_knee + 0.02):.5f}" size="0.001"/>')
    b.append(f'{pad}    <site name="{name}_knee_side_x" pos="0 0.012 '
             f'{(r_knee + 0.02):.5f}" size="0.001"/>')
    b.append(f'{pad}    <site name="{name}_knee_anchor_x" '
             f'pos="{0.55 * r_knee:.5f} 0.012 {(r_knee + 0.005):.5f}" '
             f'size="0.0015"/>')
    b.append(f'{pad}    <site name="{name}_knee_via_side" pos="0 0.024 '
             f'{(VIA_R + 0.02):.5f}" size="0.001"/>')
    b.append(f'{pad}    <site name="{name}_tibia_mid" pos="{0.5 * L[1]:.5f} '
             f'0.024 {VIA_R:.5f}" size="0.001"/>')
    b.append("    " + bone(1, "tibia").strip())

    b.append(f'{pad}    <body name="{name}_meta" pos="{L[1]:.5f} 0 0">')
    _springref = (ANKLE_SPRINGREF if ankle_springref is None
                  else float(ankle_springref))
    # ⚠️ Under ADR-0002 **Option A** the return spring is REMOVED, because the
    # antagonist replaces it. Leaving both in would flatter Option A: the spring is
    # what Option B buys instead of a motor, so a comparison that keeps it is not
    # comparing the two options.
    _k3 = 0.0 if ankle_pair else (ANKLE_SPRING if ankle_spring is None
                                  else float(ankle_spring))
    b.append(f'{pad}      <joint name="{name}_q3" type="hinge" axis="0 -1 0" '
             f'range="{leg_p.q_min[2]:.4f} {leg_p.q_max[2]:.4f}" '
             f'damping="0.002" stiffness="{_k3:.4f}" '
             f'springref="{_springref:.5f}"/>')
    b.append(f'{pad}      <geom name="{name}_ankle_sheave" type="cylinder" '
             f'size="{r_ankle:.5f} {SHEAVE_HALF_W}" pos="0 0.012 0" '
             f'quat="0.70711 0.70711 0 0" mass="1e-9" '
             f'contype="0" conaffinity="0"/>')
    # ⚠️ The gate found this one too, and it is a general lesson: an anchor placed
    # where the incoming cable ALREADY clears the sheave produces no wrap, and the
    # moment arm is then whatever the straight line happens to give. The heuristic
    # was right in 2D because the incoming direction was the spool's; the ankle's
    # cable arrives from two via-pulleys instead, so it needed checking rather than
    # inheriting. (M42 read the 2-D point as a "dead spot" at ~292 deg; ⚠️ M43
    # retracted that -- it was measured on a leg folding the wrong way, and every
    # angle in fact wraps. The dead spot is on the KNEE, at 270 deg.)
    #
    # ⚠️ **M44: the angle is set by the moment arm's SIGN REVERSAL, not by the
    # wrap.** A lone tendon can only pull, so the sign of `G = -dL/dq` decides which
    # way the joint can be driven at all -- and that sign **flips partway through the
    # ROM whatever the anchor angle**. Swept 12 anchor angles x the full -30..+150
    # deg range, every one reverses somewhere between 45 and 120 deg. It has to: as
    # the metatarsus sweeps 180 deg the anchor sweeps 180 deg around the sheave, so
    # the incoming line must cross the sheave centre once.
    #
    # Standing needs a **plantarflexing** ankle (-0.68 N.m hind, -0.79 fore, one
    # sign over the whole stance sweep). At 45 deg the reversal sat at ~85 deg and
    # the hind stance pose is at **97.1 deg** -- 12 deg the wrong side of it, so the
    # hind ankle could not supply standing torque at any tension. 300 deg pushes the
    # reversal out past 105 deg, which puts both legs' stance poses on the
    # plantarflexing side. The reversal is still inside the ROM: see ADR-0049.
    #
    # ⚠️ **M47 moved it again, to 270 deg, and the reason is that M44's
    # criterion was too weak.** M44 asked only that the reversal fall outside the
    # STANCE POSE. It does at 300 deg -- and it lands at ~112 deg, which is inside
    # the range the TROT commands (the hind ankle sweeps 88.5-122.3 deg). Driving
    # through it, the moment arm changes sign under the controller and the joint runs
    # to the opposite end stop: commanded 122.3 deg, reached -30.
    #
    # Swept against the stricter criterion -- no reversal inside EITHER leg's gait
    # range, the pair spanning both directions across the whole ROM, and both arms
    # near the 14 mm specification -- 270 deg is the only candidate that passes all
    # three (1.57 mm worst arm error). 240 deg fails on the fore leg; 60/90/150 deg
    # lose 13.8 mm of arm. See ADR-0052.
    _aa = math.radians(_ankle_anchor_deg(leg_p))
    b.append(f'{pad}      <site name="{name}_ankle_anchor" '
             f'pos="{1.15 * r_ankle * math.cos(_aa):.5f} 0.012 '
             f'{1.15 * r_ankle * math.sin(_aa):.5f}" size="0.0015"/>')
    b.append(f'{pad}      <site name="{name}_ankle_side" pos="0 0.012 '
             f'{-(r_ankle + 0.02):.5f}" size="0.001"/>')
    # Option A's antagonist: the SAME sheave, the opposite sidesite, and the
    # **SAME anchor point** -- a capstan, not a mirrored pair.
    #
    # ⚠️ M45 tried the mirrored construction the hip and knee use (anchor 180 deg
    # away, and its z-mirror at 60 deg) and both fail: the two arms reverse at
    # DIFFERENT angles, leaving a **60 deg band where both tendons have the same
    # sign and the pair cannot reverse the joint at all** -- with the stance hock at
    # 97.1 deg sitting inside it. The hip and knee get away with mirroring because
    # their cable arrives from a distant spool, so the geometry really is symmetric
    # about z; the ankle's arrives from a via-pulley on the tibia and is not.
    #
    # Anchoring both at the same point makes the two wraps exact mirrors of each
    # other, so they **reverse together and stay opposite**: worst deviation from
    # the specified 14 mm arm is **1.43 mm**, against 13.83 for the mirrored pair.
    # Physically it is one cable round a pin with a motor on each end.
    b.append(f'{pad}      <site name="{name}_ankle_side_x" pos="0 0.012 '
             f'{(r_ankle + 0.02):.5f}" size="0.001"/>')
    b.append(f'{pad}      <site name="{name}_ankle_anchor_x" '
             f'pos="{1.15 * r_ankle * math.cos(_aa):.5f} 0.012 '
             f'{1.15 * r_ankle * math.sin(_aa):.5f}" size="0.0015"/>')
    b.append("      " + bone(2, "meta").strip())
    # ⚠️ The paw is a child body rotated by `euler="0 -paw_angle 0"`, matching
    # `mjcf.py`. A hand-built `fromto` with its own sin/cos is a second place to
    # get the sign convention wrong, and the first pass did exactly that.
    b.append(f'{pad}      <body name="{name}_paw" pos="{L[2]:.5f} 0 0" '
             f'euler="0 {-leg_p.paw_angle:.6f} 0">')
    b.append(f'{pad}        <geom name="{name}_pawlink" type="capsule" '
             f'fromto="0 0 0 {L[3]:.5f} 0 0" size="0.004" '
             f'mass="{m[3]:.5f}"/>')
    b.append(f'{pad}        <geom name="{name}_pad" type="sphere" size="0.006" '
             f'pos="{L[3]:.5f} 0 0" mass="0.001" '
             f'friction="0.8 0.005 0.0001"/>')
    b.append(f'{pad}        <site name="{name}_foot" pos="{L[3]:.5f} 0 0" '
             f'size="0.002"/>')
    b.append(f'{pad}      </body>')
    b.append(f'{pad}      </body>')
    b.append(f'{pad}  </body>')
    b.append(f'{pad}</body>')

    # ----------------------------------------------------------------- tendons
    def spatial(tname, chain):
        el = elastic.get(tname) if elastic else None
        springs = ""
        if el is not None:
            k, l0 = el
            springs = f'stiffness="{k:.1f}" springlength="0 {l0:.6f}" '
        out = [f'    <spatial name="{tname}" width="{CABLE_WIDTH}" '
               f'{springs}damping="{CABLE_DAMPING}" '
               f'rgba="0.76 0.48 0.23 1">']
        out += chain
        out.append("    </spatial>")
        return "\n".join(out)

    S = lambda s: f'      <site site="{s}"/>'                       # noqa: E731
    G = lambda g, sd: f'      <geom geom="{g}" sidesite="{sd}"/>'   # noqa: E731

    if clamped:
        if elastic:
            raise ValueError(
                "cable elasticity cannot live on a CLAMPED tendon. A `<fixed>` "
                "tendon's length is the commanded sum r*q, so a `stiffness` on it "
                "is a passive JOINT spring, not a stretching cable -- and adding "
                "one would pull the joints toward `springlength`. With the cable "
                "clamped, ADR-0047's G3 series element belongs in the drivetrain: "
                "build with `spools=<series_k>` (ADR-0051), where it is an exact "
                "torsional spring between rotor and spool. Asking for both used "
                "to return a silently INELASTIC plant.")
        if pulley:
            return "\n".join(b), pulley_tendons(name, arms, ankle_pair), \
                pulley_actuators(name, ankle_pair)
        return "\n".join(b), clamped_tendons(name, arms, ankle_pair), \
            clamped_actuators(name, ankle_pair)

    t = []
    # hip: antagonistic pair, both on the hip sheave, opposite sides
    t.append(spatial(f"{name}_hip_flex", [
        S(f"{name}_spool_hip"), G(f"{name}_hip_sheave", f"{name}_hip_side"),
        S(f"{name}_hip_anchor")]))
    t.append(spatial(f"{name}_hip_ext", [
        S(f"{name}_spool_hip_x"), G(f"{name}_hip_sheave", f"{name}_hip_side_x"),
        S(f"{name}_hip_anchor_x")]))
    # knee: past the hip on a concentric via, then the knee sheave
    t.append(spatial(f"{name}_knee_flex", [
        S(f"{name}_spool_knee"), G(f"{name}_hip_via", f"{name}_hip_via_side"),
        S(f"{name}_femur_mid"),
        G(f"{name}_knee_sheave", f"{name}_knee_side"), S(f"{name}_knee_anchor")]))
    t.append(spatial(f"{name}_knee_ext", [
        S(f"{name}_spool_knee_x"), G(f"{name}_hip_via", f"{name}_hip_via_side"),
        S(f"{name}_femur_mid"),
        G(f"{name}_knee_sheave", f"{name}_knee_side_x"),
        S(f"{name}_knee_anchor_x")]))
    # ankle: single tendon + the joint spring above
    t.append(spatial(f"{name}_ankle", [
        S(f"{name}_spool_ankle"), G(f"{name}_hip_via", f"{name}_hip_via_side"),
        S(f"{name}_femur_mid"),
        G(f"{name}_knee_via", f"{name}_knee_via_side"),
        S(f"{name}_tibia_mid"),
        G(f"{name}_ankle_sheave", f"{name}_ankle_side"),
        S(f"{name}_ankle_anchor")]))
    if ankle_pair:
        t.append(spatial(f"{name}_ankle_ext", [
            S(f"{name}_spool_ankle_x"),
            G(f"{name}_hip_via", f"{name}_hip_via_side"),
            S(f"{name}_femur_mid"),
            G(f"{name}_knee_via", f"{name}_knee_via_side"),
            S(f"{name}_tibia_mid"),
            G(f"{name}_ankle_sheave", f"{name}_ankle_side_x"),
            S(f"{name}_ankle_anchor_x")]))

    # --------------------------------------------------------------- actuators
    a = []
    names = [f"{name}_hip_flex", f"{name}_hip_ext", f"{name}_knee_flex",
             f"{name}_knee_ext", f"{name}_ankle"]
    if ankle_pair:
        names.append(f"{name}_ankle_ext")
    for tname in names:
        a.append(f'    <motor name="m_{tname}" tendon="{tname}" gear="-1" '
                 f'ctrlrange="0 {TENSION_MAX:.0f}" ctrllimited="true" '
                 f'forcerange="0 {TENSION_MAX:.0f}" forcelimited="true"/>')

    return "\n".join(b), "\n".join(t), "\n".join(a)


#: Lateral half-track (m) — where the limb planes sit. ASSEMBLY_SPEC §0.1.
TRACK_HALF = 0.048

#: Where each girdle sits along x, from the trunk centre (m). `[assumed]` to match
#: the packaging study's two clusters.
GIRDLE_X = 0.105


#: Capsule radius drawn for a spine segment. Cosmetic; the mass is in `<inertial>`.
SPINE_RADIUS = 0.018

#: Spine joint order per segment, with the axis and the params fields that size it.
#: ⚠️ **The sagittal axis is the one ADR-0006 is actually about** -- dorsoventral
#: arch, the "Halloween cat" curl -- and it is the one that does work against
#: gravity. `mjcf.py` has only ever emitted the LATERAL joint (ADR-0009's sway),
#: so before M57 no MuJoCo model in this project had a sagittal spine DOF at all.
#: `axis="0 -1 0"` matches the leg's hinge convention, so a positive angle means the
#: same thing on the spine as it does on a stifle.
SPINE_AXES = (
    ("p", "0 -1 0", "q_min", "q_max", "joint_moment_arm"),
    ("y", "0 0 1", "lateral_q_min", "lateral_q_max", "lateral_moment_arm"),
)


def spine_pair_names(n_segments: int = 3):
    """The spine's antagonistic pairs, in the order the actuators are emitted."""
    return [f"spine_{ax}{i + 1}"
            for i in range(n_segments) for ax, *_ in SPINE_AXES]


def spine_chain_xml(sp, indent: int, legs_front: str, spool_front: str) -> str:
    """The vertebral chain, built innermost-out, with the FRONT girdle at its end.

    Mirrors `mjcf.build_mjcf`'s rigid chain -- rear girdle -> N segments -> front
    girdle -- but gives every segment **both** of ADR-0006's sagittal and lateral
    DOF rather than the lateral one alone, and hangs the fore legs off the far end
    so a bend carries them with it.
    """
    n = sp.n_segments
    pad = " " * (indent + 2 * n)
    chain = "\n".join([
        f'{pad}<body name="front_girdle" pos="{sp.segment_lengths[-1]:.5f} 0 0">',
        f'{pad}  <geom name="front_girdle_g" type="box" '
        f'size="0.030 0.030 0.028" mass="{sp.front_girdle_mass:.5f}"/>',
        spool_front,
        legs_front,
        f'{pad}</body>',
    ])
    for i in range(n - 1, -1, -1):
        pad = " " * (indent + 2 * i)
        pos = 0.0 if i == 0 else sp.segment_lengths[i - 1]
        ln, mass = sp.segment_lengths[i], sp.segment_mass[i]
        ix, iy, iz = _box_inertia(mass, ln / 2, 0.030, 0.030)
        joints = "".join(
            f'{pad}  <joint name="spine_{ax}{i + 1}" type="hinge" axis="{axis}" '
            f'range="{getattr(sp, lo)[i]:.5f} {getattr(sp, hi)[i]:.5f}"/>\n'
            for ax, axis, lo, hi, _ in SPINE_AXES)
        chain = (
            f'{pad}<body name="spine{i + 1}" pos="{pos:.5f} 0 0">\n'
            f'{joints}'
            f'{pad}  <inertial pos="{sp.segment_com_frac[i] * ln:.5f} 0 0" '
            f'mass="{mass:.5f}" diaginertia="{ix:.9g} {iy:.9g} {iz:.9g}"/>\n'
            f'{pad}  <geom type="capsule" fromto="0 0 0 {ln:.5f} 0 0" '
            f'size="{SPINE_RADIUS}" mass="0" contype="0" conaffinity="0" '
            f'rgba="0.7 0.6 0.6 0.35"/>\n'
            f'{chain}\n'
            f'{pad}</body>'
        )
    return chain


def spine_tendons(sp) -> str:
    """One `<fixed>` tendon per spine pair -- ADR-0058's transmission, on the spine.

    ✅ Mono-articular, which is what `TendonMap.from_spine` has always modelled:
    one pair per joint, moment arm straight out of `params`. No via terms, because
    a spine tendon does not have to get past another joint's sheave the way a
    knee cable gets past the hip.

    ⚠️ **`TendonMap.from_spine`'s `pretension` has nowhere to act here**, for the
    same reason `wbc.tendon_tension`'s `t_min` does not on a pulley leg
    ([ADR-0059](../../../docs/DESIGN_DECISIONS.md)): one bidirectional motor per
    pair leaves no redundant coordinate to spend on a co-contraction floor.
    """
    out = []
    for i in range(sp.n_segments):
        for ax, _axis, _lo, _hi, arm in SPINE_AXES:
            r = float(getattr(sp, arm)[i])
            out.append(f'    <fixed name="spine_{ax}{i + 1}">')
            out.append(f'      <joint joint="spine_{ax}{i + 1}" '
                       f'coef="{r:.6f}"/>')
            out.append("    </fixed>")
    return "\n".join(out)


def spine_actuators(sp) -> str:
    """One bidirectional motor per spine pair. Six of them, ADR-0009's count."""
    return "\n".join(
        f'    <motor name="m_{nm}" tendon="{nm}" gear="-1" '
        f'ctrlrange="-{TENSION_MAX:.0f} {TENSION_MAX:.0f}" ctrllimited="true" '
        f'forcerange="-{TENSION_MAX:.0f} {TENSION_MAX:.0f}" forcelimited="true"/>'
        for nm in spine_pair_names(sp.n_segments))


def quadruped_rig(hip_height: float = 0.175, elastic: dict | None = None,
                  trunk_mass: float | None = None,
                  ankle_pair: bool = True,
                  ankle_spring: float | None = None,
                  clamped: bool = True,
                  pulley: bool = True,
                  spine: bool = False) -> str:
    """Four tendon-driven legs on a floating trunk — the whole-body stand gate.

    Twelve leg DOF, **twenty tendons, twenty actuators**, all pull-only. The spine
    is a single rigid box here on purpose: the question this rig answers is whether
    a pull-only quadruped can stand at all, and an articulated spine would add a
    second thing to get wrong (the same reason `mjsim` trots in place).

    Fore and hind legs use their own `LegParams` — different link lengths, and the
    elbow folds the opposite way to the stifle, so `q_min/q_max` differ in sign.
    """
    if trunk_mass is None:
        trunk_mass = float(DEFAULT_SPINE.trunk_mass)
    arms = tuple(float(a) for a in DEFAULT_TENDON.joint_moment_arm)

    legs = [("LF", DEFAULT_FORELEG, +GIRDLE_X, +TRACK_HALF),
            ("RF", DEFAULT_FORELEG, +GIRDLE_X, -TRACK_HALF),
            ("LR", DEFAULT_HINDLEG, -GIRDLE_X, +TRACK_HALF),
            ("RR", DEFAULT_HINDLEG, -GIRDLE_X, -TRACK_HALF)]

    bodies, tendons, acts, spools = [], [], [], []
    fore_bodies, hind_bodies, fore_spools, hind_spools = [], [], [], []
    for nm, lp, gx, ty in legs:
        # ⚠️ On a spine chain the hip sits at its GIRDLE's origin: the chain
        # already carries the fore-aft offset, so keeping GIRDLE_X here as well
        # doubles the wheelbase (0.210 -> 0.405 m, measured). Same rule as
        # `mjcf.build_mjcf`.
        mx = 0.0 if spine else gx
        b, t, a = leg_tendon_xml(nm, lp, arms, indent=6, elastic=elastic,
                                 mount=(mx, ty, 0.0),
                                 ankle_springref=_stance_ankle(lp),
                                 ankle_pair=ankle_pair,
                                 ankle_spring=ankle_spring, clamped=clamped,
                                 pulley=pulley)
        (fore_bodies if gx > 0 else hind_bodies).append(b)
        bodies.append(b)
        tendons.append(t)
        acts.append(a)
        # spools on the girdle: inboard of the limb plane, above the hip
        sy = ty - 0.030 * (1.0 if ty > 0 else -1.0)
        here = fore_spools if gx > 0 else hind_spools
        here += [
            f'      <site name="{nm}_spool_hip"     pos="{mx - 0.042:.4f} '
            f'{sy + 0.012:.4f}  0.034" size="0.002"/>',
            f'      <site name="{nm}_spool_hip_x"   pos="{mx - 0.042:.4f} '
            f'{sy + 0.012:.4f} -0.034" size="0.002"/>',
            f'      <site name="{nm}_spool_knee"    pos="{mx - 0.050:.4f} '
            f'{sy + 0.024:.4f}  0.034" size="0.002"/>',
            f'      <site name="{nm}_spool_knee_x"  pos="{mx - 0.050:.4f} '
            f'{sy + 0.024:.4f} -0.030" size="0.002"/>',
            f'      <site name="{nm}_spool_ankle_x" pos="{mx - 0.066:.4f} '
            f'{sy + 0.030:.4f} -0.030" size="0.002"/>',
            f'      <site name="{nm}_spool_ankle"   pos="{mx - 0.058:.4f} '
            f'{sy + 0.030:.4f}  0.034" size="0.002"/>',
        ]
        spools += here[-6:]

    nl = chr(10)
    if spine:
        return _spine_quadruped(hip_height, trunk_mass, nl.join(tendons),
                                nl.join(acts), fore_bodies, hind_bodies,
                                fore_spools, hind_spools)
    return f"""<mujoco model="tomcat_quadruped_tendon">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="1e-4" gravity="0 0 {-GRAVITY}" integrator="implicitfast"/>
  <default>
    <geom rgba="0.84 0.68 0.53 1"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="3 3 0.1" rgba="0.9 0.9 0.9 1"
          friction="0.8 0.005 0.0001"/>
    <body name="trunk" pos="0 0 {hip_height:.4f}">
      <freejoint name="root"/>
      <geom name="trunk_g" type="box" size="0.130 0.035 0.030"
            mass="{trunk_mass:.5f}"/>
{nl.join(spools)}
{nl.join(bodies)}
    </body>
  </worldbody>

  <tendon>
{nl.join(tendons)}
  </tendon>

  <actuator>
{nl.join(acts)}
  </actuator>
</mujoco>
"""


def _spine_quadruped(hip_height, trunk_mass, tendons, acts,
                     fore_bodies, hind_bodies, fore_spools, hind_spools) -> str:
    """The quadruped with ADR-0006's ARTICULATED spine instead of a rigid box.

    ✅ **What M43 deliberately left out, and why it is safe to add now.** The
    rigid trunk was chosen so the standing gate had *one* thing to get wrong; six
    milestones later the leg transmission is settled (ADR-0058), re-derived
    (ADR-0059) and its controller is measured (ADR-0060), so the spine is now the
    only new variable.

    ⚠️ **The girdle span changes, and it is not a free choice.** The rigid box put
    the girdles `2 * GIRDLE_X = 210 mm` apart; ADR-0006's segment lengths sum to
    **195 mm**. The chain is the sourced number, so building it shortens the
    wheelbase by **15 mm** and every standing figure measured on the box moves with
    it. Nothing reconciled those two numbers before M57.

    Legs hang off their own girdle: the hind pair on the root body, the fore pair
    on the far end of the chain, so a bend carries them the way `spine.py`'s
    girdle composition says it must.
    """
    sp = DEFAULT_SPINE
    total = float(sum(sp.segment_lengths))
    nl = chr(10)

    # keep the body centred on the origin, as the rigid box was
    pad = " " * 6
    fore = nl.join(fore_bodies)
    fspool = nl.join(fore_spools)
    chain = spine_chain_xml(sp, 6, fore, fspool)

    return f"""<mujoco model="tomcat_quadruped_tendon_spine">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="1e-4" gravity="0 0 {-GRAVITY}" integrator="implicitfast"/>
  <default>
    <geom rgba="0.84 0.68 0.53 1"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="3 3 0.1" rgba="0.9 0.9 0.9 1"
          friction="0.8 0.005 0.0001"/>
    <body name="trunk" pos="{-total / 2.0:.5f} 0 {hip_height:.4f}">
      <freejoint name="root"/>
      <geom name="rear_girdle_g" type="box" size="0.030 0.030 0.028"
            mass="{sp.rear_girdle_mass:.5f}"/>
{nl.join(hind_spools)}
{nl.join(hind_bodies)}
{chain}
    </body>
  </worldbody>

  <tendon>
{tendons}
{spine_tendons(sp)}
  </tendon>

  <actuator>
{acts}
{spine_actuators(sp)}
  </actuator>
</mujoco>
"""


def quadruped_rig_elastic(q_ref: dict | None = None,
                          series_k: float | None = None, **kw) -> str:
    """`quadruped_rig` with per-tendon cable elasticity, built in two passes.

    `q_ref` maps leg name -> its three joint angles; the tendon lengths there set
    each cable's deadband.
    """
    import mujoco

    slack = quadruped_rig(elastic=None, **kw)
    m = mujoco.MjModel.from_xml_string(slack)
    d = mujoco.MjData(m)
    if q_ref:
        for nm, q in q_ref.items():
            for i in range(3):
                j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT,
                                      f"{nm}_q{i + 1}")
                d.qpos[m.jnt_qposadr[j]] = float(q[i])
    mujoco.mj_forward(m, d)

    elastic = {}
    for i in range(m.ntendon):
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_TENDON, i)
        L = float(d.ten_length[i])
        k = _cable_k(L)
        if series_k is not None:
            k = 1.0 / (1.0 / k + 1.0 / series_k)
        elastic[nm] = (k, L)
    return quadruped_rig(elastic=elastic, **kw)


def single_leg_rig_spooled(leg_p=DEFAULT_HINDLEG, q_ref=None,
                           series_k: float = 1.5e5, **kw) -> str:
    """`single_leg_rig` with a real DRIVETRAIN behind every cable, in two passes.

    Pass 1 builds the leg without spools and reads each cable's path length at
    `qpos0` and at `q_ref`; pass 2 puts that difference into the winding equality's
    `polycoef` offset, so the constraint is satisfied exactly at the pose the rig is
    meant to start in.

    ⚠️ **The offset is what M46 got wrong first**, and the failure is quiet:
    MuJoCo references a tendon equality at `qpos0`, so without the offset every
    cable begins 24-52 mm out and the solver snaps the leg to a configuration it
    then holds perfectly. See `spool_xml`.

    `series_k` is the drivetrain's series-elastic stiffness in N/m -- design goal
    **G3**, sized at 150-200 kN/m by [ADR-0050]. It becomes a torsional spring
    between the rotor and the spool, `k_tors = series_k * r_spool^2`, which is where
    that compliance physically sits.
    """
    import mujoco

    slack = single_leg_rig(leg_p, **kw)
    m = mujoco.MjModel.from_xml_string(slack)
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_TENDON, i)
             for i in range(m.ntendon)]

    d0 = mujoco.MjData(m)
    mujoco.mj_forward(m, d0)
    at_zero = {n: float(d0.ten_length[i]) for i, n in enumerate(names)}

    d = mujoco.MjData(m)
    if q_ref is not None:
        for i, jn in enumerate(("L_q1", "L_q2", "L_q3")):
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
            d.qpos[m.jnt_qposadr[j]] = float(q_ref[i])
    mujoco.mj_forward(m, d)

    a0 = {n: float(d.ten_length[i]) - at_zero[n] for i, n in enumerate(names)}
    return single_leg_rig(leg_p, spools=series_k, spool_a0=a0, **kw)


def single_leg_rig_elastic(leg_p=DEFAULT_HINDLEG, arms=None, q_ref=None,
                           series_k: float | None = None, **kw) -> str:
    """The rig with REAL cable elasticity, built in two passes.

    Pass 1 has no springs, so each tendon's length at `q_ref` can be read; pass 2
    sets that tendon's own `stiffness = EA/L` with a `springlength="0 L_ref"`
    deadband — slack below `L_ref`, elastic above. That is a cable.

    `series_k` optionally puts a **series-elastic element** in line with each
    cable, combining as `1/k = 1/k_cable + 1/k_series`. That is design goal **G3**
    (*"passive compliance / shock absorption at each joint"*), which has never been
    sized — and the reason it now needs sizing is that the cable alone is far
    stiffer than ADR-0026 found balance can tolerate.
    """
    import mujoco

    if arms is None:
        arms = tuple(float(a) for a in DEFAULT_TENDON.joint_moment_arm)
    slack = single_leg_rig(leg_p, arms, **kw)
    m = mujoco.MjModel.from_xml_string(slack)
    d = mujoco.MjData(m)
    if q_ref is not None:
        for i, jn in enumerate(("L_q1", "L_q2", "L_q3")):
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
            d.qpos[m.jnt_dofadr[j]] = float(q_ref[i])
    mujoco.mj_forward(m, d)

    elastic = {}
    for i in range(m.ntendon):
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_TENDON, i)
        L = float(d.ten_length[i])
        k = _cable_k(L)
        if series_k is not None:
            k = 1.0 / (1.0 / k + 1.0 / series_k)
        elastic[nm] = (k, L)
    return single_leg_rig(leg_p, arms, elastic=elastic, **kw)


def single_leg_rig(leg_p=DEFAULT_HINDLEG, arms=None, hip_height: float = 0.20,
                   fixed_hip: bool = True, elastic: dict | None = None,
                   ankle_pair: bool = True,
                   ankle_spring: float | None = None,
                   spools: float | None = None,
                   spool_a0: dict | None = None,
                   spool_servo: bool = False,
                   clamped: bool = True,
                   pulley: bool = True) -> str:
    """A one-leg test rig — the gate before anything whole-body is attempted.

    `fixed_hip=True` welds the hip to the world so the question is purely *can
    pull-only tendons hold this leg's pose against gravity*. That is the M33-style
    static gate: pass it before adding a floating base.
    """
    if arms is None:
        arms = tuple(float(a) for a in DEFAULT_TENDON.joint_moment_arm)
    body, tendons, acts = leg_tendon_xml(
        "L", leg_p, arms, indent=6, elastic=elastic,
        ankle_springref=_stance_ankle(leg_p), ankle_pair=ankle_pair,
        ankle_spring=ankle_spring, clamped=clamped, pulley=pulley)

    # spool sites live on the fixed mount, i.e. the girdle
    spool_sites = chr(10).join(
        f'      <site name="L_{k}" pos="{v[0]:.5f} {v[1]:.5f} {v[2]:.5f}" '
        f'size="0.002"/>' for k, v in spool_positions().items())

    equality = ""
    joint_default = ""
    if spools is not None:
        # ⚠️ **The winding equality overpowers a default-stiffness joint
        # limit, and it does not merely overshoot -- it corrupts the answer.** Built
        # without this line, driving the ankle tendon at the motor peak sent `q3` to
        # **215 deg against a 150 deg limit** and settled there, i.e. the leg found an
        # equilibrium 65 deg outside its own range and in the WRONG DIRECTION. With
        # the limits solved as stiffly as the equality it stops exactly on the -30 deg
        # end stop, which is where a plantarflexing tendon should take it.
        joint_default = ('    <joint solreflimit="' + EQ_SOLREF
                         + '" solimplimit="' + EQ_SOLIMP + '"/>')
    if spools is not None:
        # ⚠️ With a spool behind it the cable's compliance belongs in the
        # DRIVETRAIN, not on the tendon: the winding constraint pins the path
        # length, so a `springlength` deadband on the same tendon would be fighting
        # a constraint rather than modelling a cable. The series spring carries it.
        tendons = re.sub(r'stiffness="[-0-9.eE+]+" springlength="[^"]*" ', "",
                         tendons)
        sb, sw, se, sa = drivetrain_xml("L", ankle_pair, spools, a0=spool_a0,
                                        servo=spool_servo, pulley=pulley)
        spool_sites = spool_sites + chr(10) + sb
        tendons = tendons + chr(10) + sw
        acts = sa
        equality = "  <equality>" + chr(10) + se + chr(10) + "  </equality>"

    root = ('    <body name="mount" pos="0 0 %.4f">' % hip_height) if fixed_hip \
        else ('    <body name="mount" pos="0 0 %.4f">\n'
              '      <freejoint/>\n'
              '      <geom name="trunk" type="box" size="0.05 0.03 0.03" '
              'mass="1.0"/>' % hip_height)

    return f"""<mujoco model="tomcat_leg_tendon">
  <compiler angle="radian" autolimits="true"/>
  <!-- jacobian="dense" so `d.ten_J` can be read as (ntendon, nv). MuJoCo
       defaults to sparse, and the sparsity pattern is itself the ADR-0042
       finding: the hip tendons touch 1 DOF, the knee ones 2, the ankle 3 --
       a lower-triangular coupling, emergent from the routing. -->
  <option timestep="1e-4" gravity="0 0 {-GRAVITY}" integrator="implicitfast"
          jacobian="dense"/>
  <default>
{joint_default}
    <geom rgba="0.84 0.68 0.53 1"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.9 0.9 0.9 1"
          friction="0.8 0.005 0.0001"/>
{root}
{spool_sites}
{body}
    </body>
  </worldbody>

  <tendon>
{tendons}
  </tendon>

  <actuator>
{acts}
  </actuator>
{equality}
</mujoco>
"""
