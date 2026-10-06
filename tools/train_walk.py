# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Train a walking policy on the GPU (M137, ADR-0131).

    python tools/train_walk.py --steps 30000000 --envs 2048 --out runs/walk

`warp_env.WarpWalk`: a residual on the standing baseline, a gait clock and a
commanded forward speed in the observation, every plant and sensor range
randomised. Scored before and after on the same commanded speeds: the mean
forward speed actually made, and how many episodes end in a fall.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "kinematics", "src"))


def score(policy, kw, speeds=(0.25, 0.5), nworld=256, seconds=4.0):
    """Per commanded speed: mean forward speed over the episodes' lives, and
    the fraction that fell. `policy` None = the baseline alone."""
    from tomcat_kin.warp_env import WarpWalk
    out = {}
    for v in speeds:
        env = WarpWalk(nworld, **{**kw, "v_range": (v, v), "seed": 12345})
        o = env.reset()
        n = int(round(seconds * env.control_hz))
        alive = np.ones(nworld, bool)
        dist = np.zeros(nworld)
        life = np.zeros(nworld)
        diag, fore, match, cnt = 0.0, 0.0, 0.0, 0
        for _ in range(n):
            a = np.zeros((nworld, 18)) if policy is None else policy.predict(o, deterministic=True)[0]
            o, _r, te, tr, _info = env.step(a)
            down = env._obs["contact"] > 1.0
            if alive.any():
                diag += float(np.mean((down[:, 0] == down[:, 3])[alive]))
                fore += float(np.mean((down[:, 0] == down[:, 1])[alive]))
                match += float(np.mean(env.trot_match(env._obs)[alive]))
                cnt += 1
            dist += np.where(alive, env.vx, 0.0) / env.control_hz
            life += alive / env.control_hz
            alive &= ~te
            if not alive.any():
                break
        out[str(v)] = {"speed": float(dist.sum() / max(life.sum(), 1e-9)),
                       "fell": float(1.0 - alive.mean()),
                       # a trot: diagonals together, the fore pair apart
                       "diag_sync": diag / max(cnt, 1), "fore_sync": fore / max(cnt, 1),
                       "trot_match": match / max(cnt, 1)}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=30_000_000)
    ap.add_argument("--envs", type=int, default=2048)
    ap.add_argument("--residual", type=float, default=80.0)
    ap.add_argument("--vmax", type=float, default=0.5)
    ap.add_argument("--n-steps", type=int, default=16)
    ap.add_argument("--batch", type=int, default=2048)
    ap.add_argument("--out", default="runs/walk")
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--target-kl", type=float, default=0.03)
    ap.add_argument("--ckpt", type=int, default=2_000_000, help="steps between checkpoints")
    ap.add_argument("--gait-w", type=float, default=0.0, help="trot term weight")
    a = ap.parse_args()

    from stable_baselines3 import PPO
    from stable_baselines3.common.logger import configure
    from stable_baselines3.common.vec_env import VecMonitor

    from tomcat_kin.env import TomcatEnv
    from tomcat_kin.warp_env import make_walk_vec_env

    os.makedirs(a.out, exist_ok=True)
    dr = {**TomcatEnv.RANGES, **TomcatEnv.SENSOR_RANGES}
    kw = {"randomize": dr, "residual_n": a.residual, "gait_w": a.gait_w}
    t0 = time.time()
    before = score(None, kw)
    print("baseline alone:", before, flush=True)

    venv = VecMonitor(make_walk_vec_env(a.envs, v_range=(0.0, a.vmax), seed=0, **kw),
                      os.path.join(a.out, "monitor"))
    # ⚠️ The first 30 M-step run walked (0.36-0.41 m/s) by 19 M and then
    # COLLAPSED twice, ending on the collapsed policy. So: the learning rate
    # decays to zero, an update stops at `target_kl`, and checkpoints are kept
    # and scored, the best one reported.
    from stable_baselines3.common.callbacks import CheckpointCallback
    model = PPO("MlpPolicy", venv, n_steps=a.n_steps, batch_size=a.batch, n_epochs=10,
                learning_rate=lambda f: a.lr * f, target_kl=a.target_kl,
                gamma=0.99, gae_lambda=0.95, clip_range=0.2,
                ent_coef=0.0, policy_kwargs={"log_std_init": -1.0,
                                             "net_arch": [256, 256]},
                seed=0, verbose=1, device="cuda")
    model.set_logger(configure(a.out, ["stdout", "csv"]))
    ck = CheckpointCallback(save_freq=max(1, a.ckpt // a.envs),
                            save_path=os.path.join(a.out, "ckpt"), name_prefix="walk")
    model.learn(total_timesteps=a.steps, callback=ck)
    model.save(os.path.join(a.out, "policy"))

    after = score(model, kw)
    print("policy (final):", after, flush=True)
    ranked = []
    for f in sorted(os.listdir(os.path.join(a.out, "ckpt"))):
        p = PPO.load(os.path.join(a.out, "ckpt", f), device="cuda")
        sc = score(p, kw)
        fell = np.mean([v["fell"] for v in sc.values()])
        err = np.mean([abs(v["speed"] - float(k)) for k, v in sc.items()])
        ranked.append((fell, err, f, sc))
        print("  ", f, sc, flush=True)
    ranked.sort(key=lambda r: (r[0], r[1]))
    best = ranked[0] if ranked else None
    summary = {"args": vars(a), "baseline": before, "policy_final": after,
               "best": None if best is None else {"file": best[2], "score": best[3]},
               "minutes": (time.time() - t0) / 60.0}
    with open(os.path.join(a.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
