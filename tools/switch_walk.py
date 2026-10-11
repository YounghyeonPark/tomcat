# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Walk a command schedule by switching between two policies (M150, ADR-0144).

    python tools/switch_walk.py STRAIGHT.zip TURN.zip [--film out.webp]

ADR-0143 left two policies, each good at one thing: a straight-and-fast one
and a turning-and-slow one. Both read the same observation (the board, the
gait clock, v_cmd and w_cmd, their last action; 4 frames), so the robot can
hand over between them mid-stride. The rule: a turn command goes to the
turning policy, a straight one to the straight policy. The question is
whether the hand-over itself costs falls.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "kinematics", "src"))
sys.path.insert(0, HERE)

#: (start s, v_cmd m/s, w_cmd rad/s); a "ramp" row ramps the speed linearly
#: from the row before it to the row after it
SCHEDULES = {
    # an abrupt hand-over: speed and policy change at once
    "abrupt": ((0.0, 0.4, 0.0), (2.0, 0.1, 0.3), (4.0, 0.4, 0.0), (6.0, 0.1, -0.3),
               (8.0, 0.4, 0.0)),
    # the straight policy slows to the turner's 0.1 m/s first, hands over at
    # that speed, takes back at it and speeds up again
    "ramped": ((0.0, 0.4, 0.0), (1.5, "ramp", 0.0), (2.5, 0.1, 0.0), (3.0, 0.1, 0.3),
               (5.0, 0.1, 0.0), (5.5, "ramp", 0.0), (6.5, 0.4, 0.0), (8.0, 0.4, 0.0)),
}
SCHEDULE = SCHEDULES["abrupt"]
SECONDS = 10.0


def command(t):
    rows = SCHEDULE
    for i, row in enumerate(rows):
        nxt = rows[i + 1] if i + 1 < len(rows) else None
        if nxt is None or t < nxt[0]:
            if row[1] == "ramp":
                prev, after = rows[i - 1], rows[i + 1]
                f = (t - row[0]) / (after[0] - row[0])
                return prev[1] + f * (after[1] - prev[1]), row[2]
            return row[1], row[2]
    return rows[-1][1], rows[-1][2]


def run(straight, turn, seed, randomize, prior, film=None):
    from tomcat_kin.gym_env import TomcatWalk
    from train_walk import Stack

    env = TomcatWalk(v_cmd=SCHEDULE[0][1], w_cmd=SCHEDULE[0][2], act_obs=True,
                     randomize=randomize, seed=seed, episode_s=SECONDS, yaw_prior=prior)
    o, _ = env.reset()
    st = Stack(1, 4, o.shape[0])
    o = st.reset(o[None])[0]
    c = env.core
    log, n, fell_at = [], 0, None
    while True:
        t = n / c.control_hz
        v, w = command(t)
        env.v_cmd, env.w_cmd = v, w
        pol = turn if abs(w) > 1e-6 else straight
        a = pol.predict(o, deterministic=True)[0]
        o, _r, te, tr, _ = env.step(a)
        o = st.step(o[None], np.array([False]))[0]
        n += 1
        q = c.data.qpos[3:7]
        yaw = np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
        log.append((t, float(c.data.qpos[0]), float(c.data.qpos[1]), float(yaw)))
        if film is not None:
            film(c, t, v, w, pol is turn)
        if te:
            fell_at = t
        if te or tr:
            break
    return np.array(log), fell_at


def main():
    from stable_baselines3 import PPO

    from tomcat_kin.env import TomcatEnv

    ap = argparse.ArgumentParser()
    ap.add_argument("straight")
    ap.add_argument("turn")
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--prior", type=float, nargs=3, default=(150.0, 0.0, 40.0))
    ap.add_argument("--schedule", choices=tuple(SCHEDULES), default="abrupt")
    a = ap.parse_args()
    global SCHEDULE
    SCHEDULE = SCHEDULES[a.schedule]
    s_pol = PPO.load(a.straight, device="cpu")
    t_pol = PPO.load(a.turn, device="cpu")
    dr = {**TomcatEnv.RANGES, **TomcatEnv.SENSOR_RANGES}
    falls, segs = [], []
    for s in range(a.seeds):
        log, fell_at = run(s_pol, t_pol, 70_000 + s, dr, tuple(a.prior))
        falls.append(fell_at)
        # per schedule segment: forward speed along the heading and yaw rate
        out = []
        for i, (t0, v, w) in enumerate(SCHEDULE):
            if v == "ramp":
                out.append(None)
                continue
            t1 = SCHEDULE[i + 1][0] if i + 1 < len(SCHEDULE) else SECONDS
            m = (log[:, 0] >= t0 + 0.4) & (log[:, 0] < t1)
            if m.sum() < 10:
                out.append(None)
                continue
            seg = log[m]
            dyaw = np.unwrap(seg[:, 3])
            dt = seg[-1, 0] - seg[0, 0]
            fwd = (np.hypot(seg[-1, 1] - seg[0, 1], seg[-1, 2] - seg[0, 2])) / dt
            out.append((fwd, (dyaw[-1] - dyaw[0]) / dt))
        segs.append(out)
    print("falls: %d of %d; at t =" % (sum(f is not None for f in falls), a.seeds),
          sorted(round(f, 2) for f in falls if f is not None))
    for i, (t0, v, w) in enumerate(SCHEDULE):
        vals = [s_[i] for s_ in segs if s_[i] is not None]
        if vals and v != "ramp":
            f = np.array(vals)
            print("  t %4.1f cmd v %.1f w %+.1f -> speed %.3f, yaw %+.3f rad/s (n %d)"
                  % (t0, v, w, f[:, 0].mean(), f[:, 1].mean(), len(f)))


if __name__ == "__main__":
    main()
