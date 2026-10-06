# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Film a walking policy on the CPU simulator (M138, ADR-0132).

    python tools/film_walk.py runs/walk/policy.zip --v 0.5 --out docs/figures/walk.webp

A side camera follows the trunk. Under the robot, four lamps show which feet
the touch sensors feel down (LF RF LR RR): a trot lights the diagonals
together. The policy sees only the board; the camera is the only thing here
that looks at the simulator's state.
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
    from PIL import Image, ImageDraw
    from stable_baselines3 import PPO

    from render_episode import render_model
    from tomcat_kin.env import LEGS, TomcatEnv
    from tomcat_kin.gym_env import TomcatWalk

    ap = argparse.ArgumentParser()
    ap.add_argument("policy")
    ap.add_argument("--v", type=float, default=0.5)
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--every", type=int, default=4, help="control steps per frame")
    ap.add_argument("--dr", action="store_true", help="randomise the plant and sensors")
    ap.add_argument("--out", default="walk.webp")
    a = ap.parse_args()

    dr = {**TomcatEnv.RANGES, **TomcatEnv.SENSOR_RANGES} if a.dr else None
    env = TomcatWalk(v_cmd=a.v, randomize=dr, seed=a.seed, episode_s=a.seconds)
    pol = PPO.load(a.policy, device="cpu")
    o, _ = env.reset()
    core = env.core
    rm = render_model(core)
    rd = mujoco.MjData(rm)
    W, H = 420, 340
    r = mujoco.Renderer(rm, H, W)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth, cam.elevation, cam.distance = 90.0, -10.0, 0.85
    frames, x0, n = [], float(core.data.qpos[0]), 0
    while True:
        act = pol.predict(o, deterministic=True)[0]
        o, _r, te, tr, info = env.step(act)
        n += 1
        if n % a.every == 0 or te or tr:
            rd.qpos[:] = core.data.qpos
            mujoco.mj_forward(rm, rd)
            cam.lookat[:] = [float(core.data.qpos[0]), float(core.data.qpos[1]), 0.12]
            r.update_scene(rd, camera=cam)
            img = Image.fromarray(r.render())
            d = ImageDraw.Draw(img)
            down = np.asarray(core.observe()["contact"]) > 1.0
            for k, nm in enumerate(LEGS):
                x = 20 + 46 * k
                d.ellipse([x, H - 34, x + 20, H - 14],
                          fill=(40, 160, 70) if down[k] else (220, 220, 220),
                          outline=(80, 80, 80))
                d.text((x + 2, H - 50), nm, fill=(40, 40, 40))
            t = n / core.control_hz
            dx = float(core.data.qpos[0]) - x0
            d.text((10, 8), "v_cmd %.2f m/s   t %.2f s   mean %.2f m/s"
                   % (a.v, t, dx / max(t, 1e-9)), fill=(30, 30, 30))
            frames.append(img)
        if te or tr:
            break
    ms = int(round(1000.0 * a.every / core.control_hz))
    frames[0].save(a.out, save_all=True, append_images=frames[1:], duration=ms,
                   loop=0, quality=80)
    print("wrote %s: %d frames, %.2f s, %s" % (a.out, len(frames), n / core.control_hz,
                                               "FELL" if te else "stood"))


if __name__ == "__main__":
    main()
