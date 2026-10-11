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


class Stack:
    """The last `k` observations side by side, exactly as SB3's
    `VecFrameStack` builds them: zeros before an episode's first frame, the
    newest last, and a finished env restarted from zeros (M144)."""

    def __init__(self, n_env, k, dim):
        self.k, self.dim = int(k), int(dim)
        self.buf = np.zeros((n_env, self.k * self.dim), np.float32)

    def reset(self, obs):
        self.buf[:] = 0.0
        self.buf[:, -self.dim:] = obs
        return self.buf.copy()

    def step(self, obs, done):
        self.buf = np.roll(self.buf, -self.dim, axis=1)
        self.buf[np.asarray(done, bool)] = 0.0
        self.buf[:, -self.dim:] = obs
        return self.buf.copy()


def score(policy, kw, speeds=(0.25, 0.5), nworld=256, seconds=4.0, history=1):
    """... and with a yaw range in `kw` (M147), the commands are (v, yaw) pairs
    and each result carries the yaw rate actually made."""
    if kw.get("yaw_range") is not None:
        w = max(abs(x) for x in kw["yaw_range"])
        speeds = ((0.25, 0.0), (0.5, 0.0), (0.7, 0.0), (0.3, w), (0.3, -w))
    """Per commanded speed: mean forward speed over the episodes' lives, and
    the fraction that fell. `policy` None = the baseline alone."""
    from tomcat_kin.warp_env import WarpWalk
    out = {}
    for v in speeds:
        w_ = None
        if isinstance(v, tuple):
            v, w_ = v
        args = {**kw, "v_range": (v, v), "seed": 12345}
        if w_ is not None:
            args["yaw_range"] = (w_, w_)
        env = WarpWalk(nworld, **args)
        yaws = []
        o = env.reset()
        st = Stack(nworld, history, o.shape[1]) if history > 1 else None
        if st is not None:
            o = st.reset(o)
        n = int(round(seconds * env.control_hz))
        alive = np.ones(nworld, bool)
        dist = np.zeros(nworld)
        life = np.zeros(nworld)
        diag, fore, match, cnt = 0.0, 0.0, 0.0, 0
        slips, clears = [], []
        for _ in range(n):
            a = np.zeros((nworld, 18)) if policy is None else policy.predict(o, deterministic=True)[0]
            o, _r, te, tr, _info = env.step(a)
            if st is not None:
                o = st.step(o, te | tr)
            sl, cl = env.last_feet
            if alive.any():
                slips.append(float(np.mean(sl[alive])))
                clears.append(float(np.mean(cl[alive])))
            down = env._obs["contact"] > 1.0
            if alive.any():
                diag += float(np.mean((down[:, 0] == down[:, 3])[alive]))
                fore += float(np.mean((down[:, 0] == down[:, 1])[alive]))
                match += float(np.mean(env.trot_match(env._obs)[alive]))
                cnt += 1
            dist += np.where(alive, env.vx, 0.0) / env.control_hz
            if w_ is not None and alive.any():
                yaws.append(float(np.mean(env.wz[alive])))
            life += alive / env.control_hz
            alive &= ~te
            if not alive.any():
                break
        key = str(v) if w_ is None else "%s,%+.1f" % (v, w_)
        out[key] = {"yaw_rate": float(np.mean(yaws[20:])) if yaws else None,
                       "speed": float(dist.sum() / max(life.sum(), 1e-9)),
                       "fell": float(1.0 - alive.mean()),
                       # a trot: diagonals together, the fore pair apart
                       "diag_sync": diag / max(cnt, 1), "fore_sync": fore / max(cnt, 1),
                       "trot_match": match / max(cnt, 1),
                       "slip": float(np.mean(slips[5:])) if slips else 0.0,
                       "clearance": float(np.mean(clears)) if clears else 0.0}
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
    ap.add_argument("--init", default=None, help="continue from this policy .zip")
    ap.add_argument("--slip-w", type=float, default=0.0, help="loaded-foot slip penalty")
    ap.add_argument("--clear-w", type=float, default=0.0, help="swing clearance reward")
    ap.add_argument("--noload-rpm", type=float, default=None,
                    help="the motors' no-load speed (default: the spec motor's 665)")
    ap.add_argument("--peak-nm", type=float, default=None,
                    help="the motors' peak torque (default 1.95)")
    ap.add_argument("--history", type=int, default=1,
                    help="M144: frames stacked (with --act-obs, the last actions too)")
    ap.add_argument("--act-obs", action="store_true", help="M144: last action in the obs")
    ap.add_argument("--cu-w", type=float, default=0.0, help="M146: copper-shaped cost weight")
    ap.add_argument("--yaw", type=float, default=None, help="M147: commanded yaw rate range +-")
    ap.add_argument("--yaw-w", type=float, default=0.5)
    ap.add_argument("--yaw-sigma", type=float, default=0.3)
    ap.add_argument("--yaw-prior", type=float, nargs=3, default=None,
                    help="M148: kff kfb limit of the baseline's hip-split turn")
    ap.add_argument("--motor-dr", action="store_true",
                    help="M143: draw each world's motor from TomcatEnv.MOTOR_RANGES")
    a = ap.parse_args()

    from stable_baselines3 import PPO
    from stable_baselines3.common.logger import configure
    from stable_baselines3.common.vec_env import VecMonitor

    from tomcat_kin.env import TomcatEnv
    from tomcat_kin.warp_env import make_walk_vec_env

    os.makedirs(a.out, exist_ok=True)
    if a.noload_rpm:
        # M141 (ADR-0135): train and score under another motor's line
        import math
        from tomcat_kin import mjcf_tendon as MT
        MT.MOTOR_NOLOAD_RAD_S = a.noload_rpm * 2.0 * math.pi / 60.0
    if a.peak_nm:
        # M142 (ADR-0136): a current-limited peak, e.g. 4.19 A x the vendor's Kt
        from tomcat_kin import mjcf_tendon as MT
        MT.MOTOR_PEAK_NM = a.peak_nm
    dr = {**TomcatEnv.RANGES, **TomcatEnv.SENSOR_RANGES}
    if a.motor_dr:
        dr.update(TomcatEnv.MOTOR_RANGES)
    kw = {"randomize": dr, "residual_n": a.residual, "gait_w": a.gait_w,
          "slip_w": a.slip_w, "clear_w": a.clear_w, "act_obs": a.act_obs,
          "cu_w": a.cu_w}
    if a.yaw:
        kw["yaw_range"] = (-a.yaw, a.yaw)
        kw["yaw_w"], kw["yaw_sigma"] = a.yaw_w, a.yaw_sigma
        if a.yaw_prior:
            kw["yaw_prior"] = tuple(a.yaw_prior)
    t0 = time.time()
    before = score(None, kw, history=a.history)
    print("baseline alone:", before, flush=True)

    venv = VecMonitor(make_walk_vec_env(a.envs, v_range=(0.0, a.vmax), seed=0, **kw),
                      os.path.join(a.out, "monitor"))
    if a.history > 1:
        from stable_baselines3.common.vec_env import VecFrameStack
        venv = VecFrameStack(venv, a.history)
    # ⚠️ The first 30 M-step run walked (0.36-0.41 m/s) by 19 M and then
    # COLLAPSED twice, ending on the collapsed policy. So: the learning rate
    # decays to zero, an update stops at `target_kl`, and checkpoints are kept
    # and scored, the best one reported.
    from stable_baselines3.common.callbacks import CheckpointCallback
    if a.init:
        # M139: continue a policy; the schedule restarts from `--lr`
        model = PPO.load(a.init, env=venv, device="cuda",
                         custom_objects={"learning_rate": lambda f: a.lr * f,
                                         "lr_schedule": lambda f: a.lr * f,
                                         "target_kl": a.target_kl,
                                         "n_steps": a.n_steps, "batch_size": a.batch})
    else:
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

    after = score(model, kw, history=a.history)
    print("policy (final):", after, flush=True)
    ranked = []
    ckdir = os.path.join(a.out, "ckpt")
    for f in sorted(os.listdir(ckdir)) if os.path.isdir(ckdir) else []:
        p = PPO.load(os.path.join(a.out, "ckpt", f), device="cuda")
        sc = score(p, kw, history=a.history)
        fell = np.mean([v["fell"] for v in sc.values()])
        err = np.mean([abs(v["speed"] - float(k.split(",")[0])) for k, v in sc.items()])
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
