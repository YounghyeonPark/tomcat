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


def _rod_inertia(mass: float, length: float, radius: float) -> tuple:
    """Slender rod about its own centre — matches `mjcf.py` so the two agree."""
    ixx = mass * (3.0 * radius * radius + length * length) / 12.0
    izz = 0.5 * mass * radius * radius
    return (ixx, ixx, izz)


def _cable_k(length_m: float, dia_m: float = 1.75e-3) -> float:
    """Axial stiffness of one cable run, `EA/L` — per-tendon, as §2 requires."""
    area = math.pi * (dia_m / 2.0) ** 2
    return CABLE_E * area / max(length_m, 1e-4)


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
                   ankle_springref: float | None = None) -> tuple[str, str, str]:
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
    b.append(f'{pad}      <joint name="{name}_q3" type="hinge" axis="0 -1 0" '
             f'range="{leg_p.q_min[2]:.4f} {leg_p.q_max[2]:.4f}" '
             f'damping="0.002" stiffness="{ANKLE_SPRING:.4f}" '
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
    _aa = math.radians(300.0)
    b.append(f'{pad}      <site name="{name}_ankle_anchor" '
             f'pos="{1.15 * r_ankle * math.cos(_aa):.5f} 0.012 '
             f'{1.15 * r_ankle * math.sin(_aa):.5f}" size="0.0015"/>')
    b.append(f'{pad}      <site name="{name}_ankle_side" pos="0 0.012 '
             f'{-(r_ankle + 0.02):.5f}" size="0.001"/>')
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

    # --------------------------------------------------------------- actuators
    a = []
    for tname in (f"{name}_hip_flex", f"{name}_hip_ext", f"{name}_knee_flex",
                  f"{name}_knee_ext", f"{name}_ankle"):
        a.append(f'    <motor name="m_{tname}" tendon="{tname}" gear="-1" '
                 f'ctrlrange="0 {TENSION_MAX:.0f}" ctrllimited="true" '
                 f'forcerange="0 {TENSION_MAX:.0f}" forcelimited="true"/>')

    return "\n".join(b), "\n".join(t), "\n".join(a)


#: Lateral half-track (m) — where the limb planes sit. ASSEMBLY_SPEC §0.1.
TRACK_HALF = 0.048

#: Where each girdle sits along x, from the trunk centre (m). `[assumed]` to match
#: the packaging study's two clusters.
GIRDLE_X = 0.105


def quadruped_rig(hip_height: float = 0.175, elastic: dict | None = None,
                  trunk_mass: float | None = None) -> str:
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
    for nm, lp, gx, ty in legs:
        b, t, a = leg_tendon_xml(nm, lp, arms, indent=6, elastic=elastic,
                                 mount=(gx, ty, 0.0),
                                 ankle_springref=_stance_ankle(lp))
        bodies.append(b)
        tendons.append(t)
        acts.append(a)
        # spools on the girdle: inboard of the limb plane, above the hip
        sy = ty - 0.030 * (1.0 if ty > 0 else -1.0)
        spools += [
            f'      <site name="{nm}_spool_hip"     pos="{gx - 0.042:.4f} '
            f'{sy + 0.012:.4f}  0.034" size="0.002"/>',
            f'      <site name="{nm}_spool_hip_x"   pos="{gx - 0.042:.4f} '
            f'{sy + 0.012:.4f} -0.034" size="0.002"/>',
            f'      <site name="{nm}_spool_knee"    pos="{gx - 0.050:.4f} '
            f'{sy + 0.024:.4f}  0.034" size="0.002"/>',
            f'      <site name="{nm}_spool_knee_x"  pos="{gx - 0.050:.4f} '
            f'{sy + 0.024:.4f} -0.030" size="0.002"/>',
            f'      <site name="{nm}_spool_ankle"   pos="{gx - 0.058:.4f} '
            f'{sy + 0.030:.4f}  0.034" size="0.002"/>',
        ]

    nl = chr(10)
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
                   fixed_hip: bool = True, elastic: dict | None = None) -> str:
    """A one-leg test rig — the gate before anything whole-body is attempted.

    `fixed_hip=True` welds the hip to the world so the question is purely *can
    pull-only tendons hold this leg's pose against gravity*. That is the M33-style
    static gate: pass it before adding a floating base.
    """
    if arms is None:
        arms = tuple(float(a) for a in DEFAULT_TENDON.joint_moment_arm)
    body, tendons, acts = leg_tendon_xml(
        "L", leg_p, arms, indent=6, elastic=elastic,
        ankle_springref=_stance_ankle(leg_p))

    # spool sites live on the fixed mount, i.e. the girdle
    spools = "\n".join([
        f'      <site name="L_spool_hip" pos="-0.042 0.012 0.034" size="0.002"/>',
        f'      <site name="L_spool_hip_x" pos="-0.042 0.012 -0.034" size="0.002"/>',
        f'      <site name="L_spool_knee" pos="-0.050 0.024 0.034" size="0.002"/>',
        f'      <site name="L_spool_knee_x" pos="-0.050 0.024 -0.030" size="0.002"/>',
        f'      <site name="L_spool_ankle" pos="-0.058 0.030 0.034" size="0.002"/>',
    ])

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
    <geom rgba="0.84 0.68 0.53 1"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.9 0.9 0.9 1"
          friction="0.8 0.005 0.0001"/>
{root}
{spools}
{body}
    </body>
  </worldbody>

  <tendon>
{tendons}
  </tendon>

  <actuator>
{acts}
  </actuator>
</mujoco>
"""
