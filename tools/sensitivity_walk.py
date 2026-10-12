# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""How far can each unmeasured number be off before the walk breaks? (M153)

    python tools/sensitivity_walk.py POLICY.zip --param conduit_mu --values 0.05 0.07 0.09
    python tools/sensitivity_walk.py POLICY.zip --param floor_mu --values 0.3 0.5 0.8

Holds every plant parameter at its nominal value but ONE, sweeps that one past
the range the policy trained on, and scores the walk at a fixed command on the
CPU simulator. Sensor errors are drawn per seed so the seeds differ. Where it
breaks is the tolerance the bench measurement (docs/notes/bench-plan.md) has to
come in under before the part is worth buying.

Parameters: `conduit_mu` (all legs' conduits), `floor_mu`, `series_k_scale`,
`mass_scale`, `latency_s`, `motor_peak_nm`, `motor_noload_rpm`.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "kinematics", "src"))
sys.path.insert(0, HERE)

PLANT = ("floor_mu", "series_k_scale", "mass_scale", "latency_s",
         "motor_peak_nm", "motor_noload_rpm")


def set_conduit_mu(mu):
    """Every leg's conduit friction, in the plant and in `power`'s pricing."""
    from tomcat_kin import params as P
    P.DEFAULT_TENDON = dataclasses.replace(P.DEFAULT_TENDON, conduit_mu=mu)
    P.DEFAULT_TENDON_FORE = dataclasses.replace(P.DEFAULT_TENDON_FORE, conduit_mu=mu)


def score(policy, history, v, seeds, randomize, seconds=4.0):
    from tomcat_kin.gym_env import TomcatWalk
    from train_walk import Stack

    speeds, falls, tilts = [], 0, []
    for s in range(seeds):
        e = TomcatWalk(v_cmd=v, randomize=randomize, seed=80_000 + s,
                       act_obs=history > 1, episode_s=seconds)
        o, _ = e.reset()
        st = Stack(1, history, o.shape[0]) if history > 1 else None
        if st is not None:
            o = st.reset(o[None])[0]
        x0, n, tmax = float(e.core.data.qpos[0]), 0, 0.0
        while True:
            o, _r, te, tr, info = e.step(policy.predict(o, deterministic=True)[0])
            if st is not None:
                o = st.step(o[None], np.array([False]))[0]
            n += 1
            tmax = max(tmax, info["tilt_deg"])
            if te or tr:
                break
        speeds.append((float(e.core.data.qpos[0]) - x0) / (n / e.core.control_hz))
        falls += int(te)
        tilts.append(tmax)
    return {"speed": float(np.mean(speeds)), "falls": falls, "n": seeds,
            "peak_tilt_deg": float(np.mean(tilts))}


def main():
    from stable_baselines3 import PPO

    from tomcat_kin.env import TomcatEnv

    ap = argparse.ArgumentParser()
    ap.add_argument("policy")
    ap.add_argument("--param", required=True, choices=("conduit_mu",) + PLANT)
    ap.add_argument("--values", type=float, nargs="+", required=True)
    ap.add_argument("--history", type=int, default=4)
    ap.add_argument("--v", type=float, default=0.5)
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    pol = PPO.load(a.policy, device="cpu")
    rows = []
    for x in a.values:
        dr = dict(TomcatEnv.SENSOR_RANGES)
        if a.param == "conduit_mu":
            set_conduit_mu(x)
        else:
            dr[a.param] = (x, x)
        r = score(pol, a.history, a.v, a.seeds, dr)
        r[a.param] = x
        rows.append(r)
        print("%s %.4g -> speed %.3f m/s, falls %d/%d, peak tilt %.1f deg"
              % (a.param, x, r["speed"], r["falls"], r["n"], r["peak_tilt_deg"]), flush=True)
    if a.out:
        with open(a.out, "w") as f:
            json.dump(rows, f, indent=1)


if __name__ == "__main__":
    main()
