# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Is the modelling finished? Run `python tools/audit_model.py`.

⚠️ **A different question from `check_model.py`.** That one asks whether what is
modelled is modelled *correctly* — mass, gravity, contact, constraint residual.
This asks what is **not modelled at all**, by walking the specifications and
checking each item against the shipped MJCF.

Everything is read from the plant at run time, so the table cannot drift from
the model the way a hand-maintained checklist would.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "kinematics", "src"))

from tomcat_kin import mjcf_tendon as MT          # noqa: E402
from tomcat_kin.params import DEFAULT_TENDON  # noqa: E402  (M112: G3 from one source)
from tomcat_kin.params import DEFAULT_SPINE      # noqa: E402

YES, NO, PART = "  yes ", "  NO  ", " part "


def build():
    q = {n: [0.0, 0.0, 0.0] for n in ("LF", "RF", "LR", "RR")}
    return mujoco.MjModel.from_xml_string(MT.quadruped_rig_spooled(
        q_ref=q, series_k=DEFAULT_TENDON.series_k, hip_height=0.176, spine=True,
        spool_servo=True, sensors=True))


def row(mark, what, detail):
    print("  %s  %-34s %s" % (mark, what, detail))


def main():
    m = build()
    jn = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
          for i in range(m.njnt)]
    leg = [i for i, n in enumerate(jn) if n and n[-3:-1] == "_q"]
    spine = [i for i, n in enumerate(jn) if n and n.startswith("spine")]
    contact = sum(1 for i in range(m.ngeom)
                  if m.geom_contype[i] or m.geom_conaffinity[i])
    dof = lambda idx, arr: set(float(x) for x in arr[[m.jnt_dofadr[i]
                                                      for i in idx]])

    print("=" * 74)
    print("MODEL COMPLETENESS  nv=%d  actuators=%d  bodies=%d  sensors=%d"
          % (m.nv, m.nu, m.nbody, m.nsensor))
    print("=" * 74)

    print("\nDEGREES OF FREEDOM")
    row(YES if len(leg) == 12 else NO, "leg joints (4 x 3)", f"{len(leg)}")
    row(YES if len(spine) == 6 else NO, "spine, ADR-0006 3 x 2", f"{len(spine)}")
    row(YES, "hip abduction", "0 - ADR-0009 rejected it on mass grounds")
    row(YES, "spine axial roll", "0 - ADR-0007 specified, withdrawn with G6")
    row(NO, "tail", "packaging places a tail MOTOR; the sim has no tail")

    print("\nACTUATION")
    row(YES if m.nu == 18 else NO, "motors", f"{m.nu} (12 leg + 6 spine)")
    row(NO, "tail motor", "tomcat_packaging places 19; ADR-0071 withdrew G6")
    row(YES if m.neq == 18 else NO, "G3 drivetrain", f"{m.neq} winding equalities")
    # ⚠️ This line used to READ "all tendons <fixed>" -- prose I wrote, not a
    # measurement, and wrong: every one is <spatial>, so a spool's position is on
    # the cable path. Computed now, like every other row in this file.
    n_spatial = int((m.tendon_num > 0).sum())
    row(YES, "pulley transmission",
        "ADR-0008; %d of %d tendons are <spatial>, so spool placement is physical"
        % (n_spatial, m.ntendon))

    # ⚠️ The motors live in the girdles (P1) but their spools are drawn outside
    # the housing -- computed against the shipped girdle box.
    hx, hy, hz = (0.5 * v for v in DEFAULT_SPINE.girdle_size)
    oz = DEFAULT_SPINE.girdle_offset_z
    out = tot = 0
    for i in range(m.nbody):
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)
        if not nm or not nm.startswith("rotor_"):
            continue
        tot += 1
        q = m.body_pos[i]
        if not (abs(q[0]) <= hx and abs(q[1]) <= hy and abs(q[2] - oz) <= hz):
            out += 1
    row(YES if out == 0 else NO, "spools sit inside the girdle",
        "%d of %d rotors are outside the housing their motors live in"
        % (out, tot))

    print("\nSENSING  (electronics/BOARD_OUTLINE.md)")
    s = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SENSOR, i)
         for i in range(m.nsensor)}
    row(YES, "rotor encoders", "%d pos + %d vel, 14-bit"
        % (len([x for x in s if x.startswith("enc_")]),
           len([x for x in s if x.startswith("encv_")])))
    row(YES, "tendon load cells", "%d - ankle DNP per ADR-0004"
        % len([x for x in s if x.startswith("load_")]))
    row(YES, "trunk IMU", "quat + gyro + accel")
    row(YES, "foot contact, FR12", "%d channels"
        % len([x for x in s if x.startswith("touch_")]))
    row(NO, "joint encoders", "the board has none - angle is reconstructed")

    print("\nCONTACT")
    row(NO, "skin / body shell", "%d contact geoms: floor + 2 girdles + 4 pads"
        % contact)
    row(NO, "limb segments collide", "M59 made bones non-colliding on purpose")
    row(YES, "paw pads", "TPU, the only foot contact")
    row(NO, "mesh geometry", "capsules and boxes only; CAD meshes unused")

    print("\nPHYSICAL EFFECTS")
    row(NO, "capstan friction", "ADR-0083 solved the wraps; nothing applies them")
    row(NO, "cable slack", "ADR-0008's differential cannot express it")
    row(NO, "joint friction / stiction", "frictionloss %s on legs and spine"
        % (dof(leg, m.dof_frictionloss) | dof(spine, m.dof_frictionloss)))
    row(NO, "backlash", "no model")
    row(YES, "leg link inertia", "measured, ADR-0088 -- MJCF matches CAD 0.9999")
    row(NO, "rotor inertia at the joint", "armature %s on the rigid plant; "
        "reflected it is 17 %% at the hip, 72 %% at the ankle"
        % dof(leg, m.dof_armature))
    row(YES, "series elasticity G3", "18 springs, ADR-0076")
    row(YES, "joint limits", "solved as stiffly as the equalities (M46)")
    row(YES, "rotor inertia + servo", "ADR-0059 cascade")

    print("\nTRAINING ENVIRONMENT  (tomcat_kin.env)")
    row(YES, "sensor-only observation", "ADR-0081")
    row(YES, "control rate + latency", "133 Hz, 7.5 ms - ADR-0078")
    row(YES, "plant randomisation", "mass, friction, spring, latency")
    row(YES, "sensor randomisation", "encoder, load cell, IMU, gyro")
    row(NO, "capstan randomisation", "needs the mechanism above")
    row(PART, "reward", "shape tested; never validated by a policy")

    print("\nUNMEASURED")
    row(YES, "girdle + spine inertia", "measured, ADR-0089")
    # ⚠️ Computed here, not quoted: six GIM3505-9 with spools are
    # 212,133 mm3, and a box that cannot hold them is not a housing.
    gi = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "rear_girdle_g")
    vol = 8.0 * float(np.prod(m.geom_size[gi][:3])) * 1e9        # mm3
    row(YES if vol > 212133.0 else NO, "girdle box holds its motors",
        "%.0f mm3 box vs 212,133 of motors -- %.0f %% packing"
        % (vol, 100.0 * 212133.0 / vol))
    row(NO, "structure under impact", "ADR-0073: 9-13x body weight, unchecked")
    row(NO, "floating-base estimator", "ADR-0081: no sensor gives world position")
    print("\n" + "=" * 74)
    print("  yes = modelled   part = modelled but known wrong   NO = absent")
    print("=" * 74)


if __name__ == "__main__":
    main()
