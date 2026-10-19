# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Is a loaded paw sliding, or is the contact creeping? (M158, ADR-0150)

    python tools/slip_diag.py POLICY.zip [--seeds 6] [--impratio 10] [--pad-solref 0.005]

Walks a policy at 0.5 m/s on the CPU plant and, for every pad carrying more
than 5 N, measures:

- the pad CENTRE's horizontal speed, which is what the walk's reward calls slip;
- the speed of the paw's material point AT the contact, which is the true slip
  (zero when the pad rolls without sliding);
- the friction it is using, |Ft| / Fn. Real sliding happens at mu; a contact
  that slides well below mu is creeping.

`--impratio` and `--pad-solref` override the model's contact settings, to
compare contact models on one policy.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "kinematics", "src"))
sys.path.insert(0, HERE)


def main():
    import mujoco
    from stable_baselines3 import PPO

    from tomcat_kin.env import LEGS
    from tomcat_kin.gym_env import TomcatWalk
    from train_walk import Stack

    ap = argparse.ArgumentParser()
    ap.add_argument("policy")
    ap.add_argument("--seeds", type=int, default=6)
    ap.add_argument("--v", type=float, default=0.5)
    ap.add_argument("--impratio", type=float, default=None, help="elliptic cone at this impratio")
    ap.add_argument("--pad-solref", type=float, default=None, help="pad and floor time constant, s")
    a = ap.parse_args()
    pol = PPO.load(a.policy, device="cpu")

    centre, contact, ratio, falls = [], [], [], 0
    for seed in range(a.seeds):
        e = TomcatWalk(v_cmd=a.v, seed=90_000 + seed, act_obs=True, episode_s=4.0)
        m, d = e.core.model, e.core.data
        geom = lambda nm: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, nm)  # noqa: E731
        pads, floor = [geom(nm + "_pad") for nm in LEGS], geom("floor")
        if a.impratio is not None:
            m.opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
            m.opt.impratio = a.impratio
        if a.pad_solref is not None:
            m.geom_solref[pads + [floor], 0] = a.pad_solref
        o, _ = e.reset()
        st = Stack(1, 4, o.shape[0])
        o = st.reset(o[None])[0]
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        f = np.zeros(6)
        n = 0
        while True:
            o, _r, te, tr, _ = e.step(pol.predict(o, deterministic=True)[0])
            o = st.step(o[None], np.array([False]))[0]
            n += 1
            if n > 40:                                   # past the start
                for i in range(d.ncon):
                    c = d.contact[i]
                    g = c.geom1 if c.geom2 == floor else (c.geom2 if c.geom1 == floor else -1)
                    if g not in pads:
                        continue
                    mujoco.mj_contactForce(m, d, i, f)
                    if f[0] < 5.0:
                        continue
                    b = m.geom_bodyid[g]
                    mujoco.mj_jac(m, d, jp, jr, c.pos, b)
                    contact.append(np.hypot(*(jp @ d.qvel)[:2]))
                    mujoco.mj_jac(m, d, jp, jr, d.geom_xpos[g], b)
                    centre.append(np.hypot(*(jp @ d.qvel)[:2]))
                    ratio.append(np.hypot(f[1], f[2]) / f[0])
            if te or tr:
                falls += int(te)
                break
    contact, ratio = np.array(contact), np.array(ratio)
    sl = contact > 0.05
    print("falls %d of %d; %d loaded-pad samples" % (falls, a.seeds, len(contact)))
    print("pad-centre speed (the reward's slip):  %.3f m/s" % np.mean(centre))
    print("contact-point speed (true slip):       %.3f m/s" % contact.mean())
    print("share sliding faster than 5 cm/s:      %.2f" % sl.mean())
    print("|Ft|/Fn: sliding %.2f, stuck %.2f (mu %.2f)"
          % (np.median(ratio[sl]) if sl.any() else float("nan"), np.median(ratio[~sl]),
             m.geom_friction[pads[0], 0]))


if __name__ == "__main__":
    main()
