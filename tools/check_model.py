# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Verify the plant independently: ``python tools/check_model.py``.

Every number below is computed here and now from the shipped model. Nothing is
quoted from a document, so this can be used to check the documents rather than
to trust them.

⚠️ Written in M82 after the project owner asked whether the modelling was
actually right before the training task was made easier. It was the correct
question: the plant checked out, and the **environment's start condition** did
not — every episode began with the paws 6 mm in the air carrying 2 % of the
robot's weight.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
import mujoco

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "kinematics", "src"))
from tomcat_kin.env import TomcatEnv, LEGS, PAIRS      # noqa: E402
from tomcat_kin import mjcf_tendon as MT               # noqa: E402

OK, BAD = "  OK  ", " CHECK"


def _contact(m, d):
    f6 = np.zeros(6)
    tot = 0.0
    for i in range(d.ncon):
        mujoco.mj_contactForce(m, d, i, f6)
        tot += f6[0]
    return d.ncon, tot


def _recon(latency, steps=60):
    """Worst joint-reconstruction error, and how fast the joints are moving."""
    env = TomcatEnv(latency_s=latency)
    env.reset()
    for _ in range(steps):
        obs, *_ = env.step(np.full(18, 25.0))
    m, d = env.model, env.data
    worst, vel = 0.0, 0.0
    for nm in LEGS:
        qa = [m.jnt_qposadr[env._id(mujoco.mjtObj.mjOBJ_JOINT, f"{nm}_q{i}")]
              for i in (1, 2, 3)]
        dv = [m.jnt_dofadr[env._id(mujoco.mjtObj.mjOBJ_JOINT, f"{nm}_q{i}")]
              for i in (1, 2, 3)]
        true = np.array([float(d.qpos[a]) for a in qa])
        est = env.joint_estimate(obs)[nm]
        worst = max(worst, float(np.max(np.degrees(np.abs(est - true)))))
        vel = max(vel, float(np.max(np.abs([d.qvel[x] for x in dv]))))
    return worst, vel


def main():
    env = TomcatEnv()
    m, d = env.model, env.data
    M = float(np.sum(m.body_mass))
    W = M * 9.81
    print("=" * 70)
    print("PLANT  nv=%d  actuators=%d  equalities=%d  sensors=%d"
          % (m.nv, m.nu, m.neq, m.nsensor))
    print("=" * 70)

    print("\n1. MASS   (params.py says 4.3041 kg)")
    print("   %.4f kg -> weight %.2f N                          %s"
          % (M, W, OK if abs(M - 4.304) < 0.01 else BAD))

    print("\n2. GRAVITY   ctrl=0: a tendon robot has nothing holding its joints")
    env.reset(settle=False)
    mujoco.mj_forward(m, d)
    az = float(d.qacc[2])
    tf = float(np.max(np.abs(d.actuator_force)))
    print("   trunk acc %.2f m/s^2, tendon force %.4f N            %s"
          % (az, tf, OK if abs(az + 9.81) < 0.1 and tf < 1e-6 else BAD))

    print("\n3. RESET WITHOUT SETTLING   (the M82 defect)")
    n, tot = _contact(m, d)
    print("   contacts %d, normal %.2f N (%.0f%% of weight), trunk %.1f mm"
          % (n, tot, 100 * tot / W, 1e3 * float(d.qpos[2])))
    print("   the paws start 6 mm up, so the episode began mid-fall.")

    print("\n4. RESET WITH SETTLING   (what reset() does now)")
    obs = env.reset()
    mujoco.mj_forward(m, d)
    n, tot = _contact(m, d)
    good = n == 4 and tot > 0.5 * W and abs(float(d.qvel[2])) < 0.02
    print("   contacts %d, normal %.2f N (%.0f%% of weight), trunk %.1f mm"
          % (n, tot, 100 * tot / W, 1e3 * float(d.qpos[2])))
    print("   vertical vel %+.4f m/s, tilt %.2f deg                 %s"
          % (float(d.qvel[2]), TomcatEnv._tilt(obs), OK if good else BAD))

    print("\n5. THE WINDING EQUALITY   L_path == a0 - r*(jr + js)")
    for _ in range(60):
        obs, *_ = env.step(np.full(18, 25.0))
    mujoco.mj_forward(m, d)
    worst = 0.0
    for nm in LEGS:
        for p in PAIRS:
            t = env._id(mujoco.mjtObj.mjOBJ_TENDON, f"{nm}_{p}")
            jr = m.jnt_qposadr[env._id(mujoco.mjtObj.mjOBJ_JOINT, f"jr_{nm}_{p}")]
            js = m.jnt_qposadr[env._id(mujoco.mjtObj.mjOBJ_JOINT, f"js_{nm}_{p}")]
            pred = env.a0[f"{nm}_{p}"] - MT.SPOOL_R * (float(d.qpos[jr])
                                                       + float(d.qpos[js]))
            worst = max(worst, abs(float(d.ten_length[t]) - pred))
    print("   worst residual over 12 leg pairs: %.4f mm            %s"
          % (1e3 * worst, OK if worst < 1e-5 else BAD))

    print("\n6. CAN ANYTHING STAND OPEN LOOP?   (cap 533 steps = 4 s)")
    rng = np.random.default_rng(0)
    for label, act in (("zero", np.zeros(18)),
                       ("uniform 25 N", np.full(18, 25.0)),
                       ("random", None)):
        ks = []
        for _ in range(3):
            env.reset()
            for k in range(533):
                a = rng.uniform(-223, 223, 18) if act is None else act
                _, _, term, _ = env.step(a)
                if term:
                    break
            ks.append(k + 1)
        print("   %-14s %5.1f steps (%.3f s)"
              % (label, np.mean(ks), np.mean(ks) / env.control_hz))
    print("   nothing survives: standing is an unstable equilibrium (ADR-0082)")

    print("\n7. JOINT ANGLE THE ROBOT CANNOT MEASURE   (ADR-0077)")
    print("   the estimate is of what the SENSORS SAW, which NFR12 makes")
    print("   7.5 ms old -- comparing it to the state NOW measures the delay,")
    print("   not the estimator, so both are reported.")
    w0, vel = _recon(0.0)
    w75, _ = _recon(7.5e-3)
    print("   no delay : %.3f deg worst                            %s"
          % (w0, OK if w0 < 2.0 else BAD))
    print("   7.5 ms   : %.3f deg worst  (joints at %.1f rad/s cover"
          % (w75, vel))
    print("              %.1f deg in 7.5 ms)                        %s"
          % (math.degrees(vel * 7.5e-3), OK if w75 < w0 + 12.0 else BAD))
    print("\n" + "=" * 70)


if __name__ == "__main__":
    main()
