# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""MuJoCo model generation — an *independent* check on the reduced-order model.

Every balance number in this project (ADR-0013 onward) comes from a **Linear
Inverted Pendulum Model**: a point mass at constant height over a massless leg,
with `dH/dt = 0` assumed. `control.py` is built entirely on it.

That model is an idealisation in four specific ways, and this module exists to
find out what each one costs:

1. **Point mass.** The trunk has a real inertia tensor and can *pitch and roll*.
   LIPM has no rotational degree of freedom at all.
2. **Constant CoM height.** The real CoM rises and falls as the legs work.
3. **Massless legs.** They are 0.095–0.110 kg each and swing.
4. **`dH/dt = 0`.** ADR-0018 measured link *spin* at 3 %, and M17 measured the
   aggregate of the rest: the real divergence is ~2 % SLOWER than LIPM predicts.

⚠️ **This does not validate any [assumed] parameter.** Feeding the assumed 132 g
motor mass into a physics engine returns the assumed 132 g motor mass. Simulation
checks the *model*; only a measurement checks the *inputs* (see OPEN_RISKS R1/R2).
What it can check is whether the LIPM-derived envelope survives contact with
rigid-body dynamics it was never asked about.

Fidelity notes — where this model differs from the analytical one, deliberately:

- Links carry **slender-rod rotational inertia**. The analytical model treats them
  as point masses at ``link_com_frac``; the difference is part of what is under
  test, so it is not suppressed.
- Contact is a **finite friction cone** at a paw sphere, not the Coulomb
  hand-calculation of ADR-0019/0020.
- ``mujoco`` is an **optional** dependency. Nothing in the shipped analytical test
  suite requires it.
"""

from __future__ import annotations

import math

import numpy as np

from .params import DEFAULT_FORELEG, DEFAULT_HINDLEG, DEFAULT_SPINE, GRAVITY

# Trunk cross-section used to turn component masses into plausible box inertias.
# The analytical model carries no trunk inertia at all, so these are new numbers
# and they are `[assumed]` — half-width and half-height of the body, in metres.
TRUNK_HALF_W = 0.030
TRUNK_HALF_H = 0.030

# Paw contact sphere. The analytical model uses a POINT foot, so this is kept
# small deliberately: the sphere is centred on the analytical paw tip, which puts
# the true contact point one radius lower. `rest_height` compensates, and every
# comparison below is made against omega recomputed at the sim's ACTUAL CoM
# height rather than the nominal 0.17 — so the residual radius cannot flatter the
# result either way.
PAW_RADIUS = 0.002

LINK_RADIUS = 0.006

SPINE_RADIUS = 0.018


def _rod_inertia(mass: float, length: float, radius: float) -> tuple:
    """Slender-rod diagonal inertia about the link COM, long axis = local x."""
    ixx = 0.5 * mass * radius ** 2
    itr = mass * (3.0 * radius ** 2 + length ** 2) / 12.0
    return (max(ixx, 1e-9), max(itr, 1e-9), max(itr, 1e-9))


def _box_inertia(mass: float, hx: float, hy: float, hz: float) -> tuple:
    """Solid-box diagonal inertia about its COM."""
    return (
        mass * ((2 * hy) ** 2 + (2 * hz) ** 2) / 12.0,
        mass * ((2 * hx) ** 2 + (2 * hz) ** 2) / 12.0,
        mass * ((2 * hx) ** 2 + (2 * hy) ** 2) / 12.0,
    )


def _leg_xml(name: str, track_y: float, leg_p, indent: int) -> str:
    """One planar leg as a serial chain of hinges about the -y axis.

    The repo's `LegModel.forward` builds the paw tip from *cumulative* angles
    ``a_i = q1 + ... + qi`` with ``x = l cos a``, ``z = l sin a``. MuJoCo chains
    joints relatively, so the joint values map straight across — provided the
    hinge axis is ``(0, -1, 0)``, which makes a positive joint angle rotate +x
    toward +z exactly as the analytical convention does.

    The hip sits at the girdle origin: the spine chain already carries the x.
    """
    lens = (leg_p.l1, leg_p.l2, leg_p.l3)
    fracs = leg_p.link_com_frac
    masses = leg_p.link_mass

    out = []
    out.append(f'{" " * indent}<body name="{name}_L1" pos="0 {track_y} 0">')

    for i, (ln, m, fr) in enumerate(zip(lens, masses, fracs), start=1):
        pad = " " * (indent + 2 * i)
        if i > 1:
            out.append(f'{pad}<body name="{name}_L{i}" pos="{lens[i - 2]} 0 0">')
        out.append(
            f'{pad}  <joint name="{name}_q{i}" type="hinge" axis="0 -1 0" '
            f'range="{leg_p.q_min[i - 1]} {leg_p.q_max[i - 1]}"/>'
        )
        ix, iy, iz = _rod_inertia(m, ln, LINK_RADIUS)
        out.append(
            f'{pad}  <inertial pos="{fr * ln} 0 0" mass="{m}" '
            f'diaginertia="{ix:.9g} {iy:.9g} {iz:.9g}"/>'
        )
        out.append(
            f'{pad}  <geom type="capsule" fromto="0 0 0 {ln} 0 0" '
            f'size="{LINK_RADIUS}" mass="0" contype="0" conaffinity="0"/>'
        )

    # Paw: rigid at `paw_angle` off the metatarsus, carrying link_mass[3].
    pa = leg_p.paw_angle
    l4 = leg_p.l4
    m4 = masses[3]
    pad = " " * (indent + 2 * len(lens) + 2)
    ix, iy, iz = _rod_inertia(m4, l4, LINK_RADIUS)
    out.append(f'{pad}<body name="{name}_paw" pos="{lens[-1]} 0 0" euler="0 {-pa} 0">')
    out.append(f'{pad}  <inertial pos="{0.5 * l4} 0 0" mass="{m4}" '
               f'diaginertia="{ix:.9g} {iy:.9g} {iz:.9g}"/>')
    out.append(f'{pad}  <geom name="{name}_tip" type="sphere" pos="{l4} 0 0" '
               f'size="{PAW_RADIUS}" mass="0"/>')
    out.append(f'{pad}  <site name="{name}_site" pos="{l4} 0 0" size="0.003"/>')
    out.append(f'{pad}</body>')

    for i in range(len(lens), 0, -1):
        out.append(" " * (indent + 2 * i) + "</body>")
    return "\n".join(out)


def _girdle_block(name: str, mass: float, indent: int) -> str:
    pad = " " * indent
    ix, iy, iz = _box_inertia(mass, 0.030, TRUNK_HALF_W, TRUNK_HALF_H)
    return (
        f'{pad}<inertial pos="0 0 0" mass="{mass}" '
        f'diaginertia="{ix:.9g} {iy:.9g} {iz:.9g}"/>\n'
        f'{pad}<geom name="{name}" type="box" '
        f'size="0.030 {TRUNK_HALF_W} {TRUNK_HALF_H}" mass="0" '
        f'contype="0" conaffinity="0" rgba="0.6 0.6 0.65 0.35"/>'
    )


def build_mjcf(controller, leg_q: dict, height: float = 0.17,
               mu: float = 0.8, timestep: float = 5e-4,
               armature: float = 0.0, spine_dof: bool = False) -> str:
    """MJCF for the whole robot, generated from the live parameter set.

    Parameters
    ----------
    controller : GaitController
        Source of the body/mount geometry — nothing is hand-copied.
    leg_q : dict
        Per-leg joint angles defining the pose the model is built around.
    height : float
        Trunk-origin height. Use `rest_height` to put the stance paws on z = 0.
    mu : float
        Floor friction. ⚠️ The `[assumed]` NFR16 value is 0.70; ADR-0020 sizes the
        trot against 0.80. Neither is measured — see OPEN_RISKS R2.
    armature : float
        Reflected rotor inertia per joint. Left at 0 by default so the test
        isolates *rigid-body* effects from *drivetrain* ones.
    spine_dof : bool
        Give the three spine joints a **lateral (yaw)** degree of freedom, matching
        `SpineModel.lateral_vertebra_xy` — the same planar serial chain as a leg but
        in the horizontal plane, about the vertical axis (ADR-0009).

        M17 ran a rigid trunk, so it could only test the **feet-only** envelope.
        The spine supplies 23.6 mm of the 53.9 mm headline, which means 44 % of the
        number NFR15 is checked against sat outside the simulation entirely.

    Notes
    -----
    The body tree is a real chain — rear girdle → 3 spine segments → front girdle —
    so the fore legs ride on the spine's far end and a lateral bend carries them
    with it. That is exactly the mechanism `center_of_mass_y` describes and the
    balance authority ADR-0009 bought. With ``spine_dof=False`` the joints are
    omitted and the trunk is rigid, reproducing every M17 figure.
    """
    body = controller.body
    sp = DEFAULT_SPINE

    def legs_on(girdle_value: str, indent: int) -> str:
        p = DEFAULT_FORELEG if girdle_value == "front" else DEFAULT_HINDLEG
        return "\n".join(
            _leg_xml(nm, body.mounts[nm].track_y, p, indent)
            for nm in body.leg_names
            if body.mounts[nm].girdle.value == girdle_value
        )

    # --- spine chain, built innermost-out ------------------------------------
    n = sp.n_segments
    depth = 6 + 2 * n
    chain = "\n".join([
        f'{" " * depth}<body name="front_girdle" pos="{sp.segment_lengths[-1]} 0 0">',
        _girdle_block("front_girdle_g", sp.front_girdle_mass, depth + 2),
        legs_on("front", depth + 2),
        f'{" " * depth}</body>',
    ])
    for i in range(n - 1, -1, -1):
        pad = " " * (6 + 2 * i)
        pos = 0.0 if i == 0 else sp.segment_lengths[i - 1]
        ln, m = sp.segment_lengths[i], sp.segment_mass[i]
        ix, iy, iz = _box_inertia(m, ln / 2, TRUNK_HALF_W, TRUNK_HALF_H)
        jnt = ""
        if spine_dof:
            jnt = (f'{pad}  <joint name="spine_y{i + 1}" type="hinge" axis="0 0 1" '
                   f'range="{sp.lateral_q_min[i]} {sp.lateral_q_max[i]}"/>\n')
        chain = (
            f'{pad}<body name="spine{i + 1}" pos="{pos} 0 0">\n'
            f'{jnt}'
            f'{pad}  <inertial pos="{sp.segment_com_frac[i] * ln} 0 0" mass="{m}" '
            f'diaginertia="{ix:.9g} {iy:.9g} {iz:.9g}"/>\n'
            f'{pad}  <geom type="capsule" fromto="0 0 0 {ln} 0 0" '
            f'size="{SPINE_RADIUS}" mass="0" contype="0" conaffinity="0" '
            f'rgba="0.7 0.6 0.6 0.35"/>\n'
            f'{chain}\n'
            f'{pad}</body>'
        )

    acts = [f'    <position name="{nm}_a{i}" joint="{nm}_q{i}" kp="120" kv="4"/>'
            for nm in body.leg_names for i in (1, 2, 3)]
    if spine_dof:
        acts += [f'    <position name="spine_a{i + 1}" joint="spine_y{i + 1}" '
                 f'kp="30" kv="1.5"/>' for i in range(n)]

    return f"""<mujoco model="tomcat">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{timestep}" gravity="0 0 -{GRAVITY}" integrator="implicitfast"
          cone="elliptic" impratio="10"/>
  <default>
    <joint armature="{armature}" damping="0.01"/>
    <geom friction="{mu} 0.005 0.0001" solref="0.004 1" solimp="0.95 0.99 0.001"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.9 0.9 0.9 1"/>
    <body name="trunk" pos="0 0 {height}">
      <freejoint name="root"/>
{_girdle_block("rear_girdle_g", sp.rear_girdle_mass, 6)}
{legs_on("rear", 6)}
{chain}
    </body>
  </worldbody>
  <actuator>
{chr(10).join(acts)}
  </actuator>
</mujoco>
"""


def stance_pose(controller, phase: float = 0.0) -> dict:
    """Leg joint angles at a gait phase, as a plain dict."""
    st = controller.state(phase)
    return {nm: np.asarray(st.legs[nm].q, dtype=float) for nm in controller.body.leg_names}


def rest_height(controller, leg_q: dict, stance_legs) -> float:
    """Trunk height that puts the *contact points* of ``stance_legs`` on z = 0.

    The paw sphere is centred on the analytical tip, so its contact point is one
    ``PAW_RADIUS`` below. Without this the model starts interpenetrating and the
    solver ejects the robot — which looks exactly like a divergence.
    """
    body = controller.body
    lowest = min(
        float(body.foot_world_position(np.zeros(3), nm, leg_q[nm])[1])
        for nm in stance_legs
    )
    return -lowest + PAW_RADIUS


def support_line(controller, leg_q: dict, stance_legs) -> tuple:
    """Ground-plane unit vector along the diagonal support line, and its normal.

    The normal is the direction the body topples in — the axis every envelope in
    `control.py` is quoted along. ⚠️ The two diagonals' normals are **52.4° apart**
    (M17), which the single-axis `StepPlant` cannot express.
    """
    body = controller.body
    pts = []
    for nm in stance_legs:
        f = body.foot_world_position(np.zeros(3), nm, leg_q[nm])
        pts.append(np.array([float(f[0]), body.mounts[nm].track_y]))
    d = pts[1] - pts[0]
    d = d / np.linalg.norm(d)
    return d, np.array([-d[1], d[0]])
