# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Train a residual standing policy on the board's sensors (M135, ADR-0129).

    python tools/train_stand.py --steps 500000 --envs 8 --out runs/stand_m135
    python tools/train_stand.py --backend warp --envs 2048 --out runs/stand_m136

`--backend warp` (M136, ADR-0130) trains on `warp_env.WarpStand`, every robot
on the GPU at once; the scoring stays on the CPU env either way, so a policy
trained on one backend is judged on the other.

The policy adds +-`--residual` N to `stand_baseline.StandBaseline` on every
pair, with every plant and sensor range randomised. Before and after training
it is scored on the same fixed seeds against the baseline alone, so "did it
learn" is a comparison and not a curve to squint at.

⚠️ M81's run kept only `tail -30` of its log and lost the early learning
curve. Here every episode goes to `monitor.csv` and every PPO update to
`progress.csv`, in `--out`.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "kinematics", "src"))


def make(seed, residual, randomize):
    def _f():
        from stable_baselines3.common.monitor import Monitor

        from tomcat_kin.gym_env import TomcatStand
        return Monitor(TomcatStand(randomize=randomize, seed=seed, residual_n=residual))
    return _f


def score(policy, residual, randomize, seeds):
    """Mean steps survived and return over `seeds`; `policy` None = baseline."""
    from tomcat_kin.gym_env import TomcatStand
    steps, rets, full = [], [], 0
    for s in seeds:
        env = TomcatStand(randomize=randomize, seed=10_000 + s, residual_n=residual)
        o, _ = env.reset()
        n, ret = 0, 0.0
        while True:
            a = np.zeros(18) if policy is None else policy.predict(o, deterministic=True)[0]
            o, r, te, tr, _ = env.step(a)
            n += 1
            ret += r
            if te or tr:
                break
        steps.append(n)
        rets.append(ret)
        full += int(tr and not te)
    return {"mean_steps": float(np.mean(steps)), "mean_return": float(np.mean(rets)),
            "full": full, "n": len(seeds)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=500_000)
    ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--residual", type=float, default=30.0)
    ap.add_argument("--eval", type=int, default=30, help="fixed seeds scored")
    ap.add_argument("--out", default="runs/stand")
    ap.add_argument("--no-dr", action="store_true")
    ap.add_argument("--backend", choices=("cpu", "warp"), default="cpu")
    ap.add_argument("--n-steps", type=int, default=None, help="rollout per env")
    ap.add_argument("--batch", type=int, default=None, help="PPO minibatch")
    a = ap.parse_args()

    from stable_baselines3 import PPO
    from stable_baselines3.common.logger import configure
    from stable_baselines3.common.vec_env import SubprocVecEnv

    from tomcat_kin.env import TomcatEnv

    os.makedirs(a.out, exist_ok=True)
    dr = None if a.no_dr else {**TomcatEnv.RANGES, **TomcatEnv.SENSOR_RANGES}
    seeds = range(a.eval)
    t0 = time.time()
    before = score(None, a.residual, dr, seeds)
    print("baseline alone:", before, flush=True)

    from stable_baselines3.common.vec_env import VecMonitor
    if a.backend == "warp":
        from tomcat_kin.warp_env import make_vec_env
        venv = make_vec_env(a.envs, randomize=dr, residual_n=a.residual, seed=0)
        n_steps = a.n_steps or 16
        batch, device = a.batch or 2048, "cuda"
    else:
        venv = SubprocVecEnv([make(i, a.residual, dr) for i in range(a.envs)])
        n_steps = a.n_steps or 512
        batch, device = a.batch or 1024, "cpu"
    venv = VecMonitor(venv, os.path.join(a.out, "monitor"))
    model = PPO("MlpPolicy", venv, n_steps=n_steps, batch_size=batch, n_epochs=10,
                learning_rate=3e-4, gamma=0.99, gae_lambda=0.95, clip_range=0.2,
                ent_coef=0.0, policy_kwargs={"log_std_init": -1.0}, seed=0,
                verbose=1, device=device)
    model.set_logger(configure(a.out, ["stdout", "csv"]))
    model.learn(total_timesteps=a.steps)
    model.save(os.path.join(a.out, "policy"))
    venv.close()

    after = score(model, a.residual, dr, seeds)
    print("residual policy:", after, flush=True)
    summary = {"args": vars(a), "baseline": before, "policy": after,
               "minutes": (time.time() - t0) / 60.0}
    with open(os.path.join(a.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
