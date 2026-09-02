# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Film an episode as a strip of frames: ``python tools/render_episode.py``.

⚠️ Written in M82 because the numbers were not enough. "Nothing survives 0.1 s"
is a claim; a filmstrip of the robot folding is the thing you can check with
your eyes.

Each row is one action held from the settled stance, sampled at the same wall
times, so the rows are directly comparable. The caption under each frame is the
env's own observation-derived state — tilt from the IMU, stance height from the
reconstructed joints — not privileged simulator state.
"""
from __future__ import annotations

import os
import re
import struct
import sys
import zlib

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "kinematics", "src"))

from tomcat_kin.env import TomcatEnv                      # noqa: E402
from tomcat_kin import mjcf_tendon as MT                  # noqa: E402

OUT = os.path.join(ROOT, "docs", "figures")
W, H = 420, 340
VISUAL = f"""
  <visual>
    <global offwidth="{W}" offheight="{H}"/>
    <quality shadowsize="2048" offsamples="4"/>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.45 0.45 0.45"
               specular="0.15 0.15 0.15"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" width="256"
             height="256" rgb1="0.96 0.96 0.97" rgb2="0.80 0.83 0.87"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.94 0.94 0.93"
             rgb2="0.88 0.88 0.87" width="512" height="512"/>
    <material name="gridm" texture="grid" texrepeat="14 14" reflectance="0.04"/>
  </asset>
"""


def render_model(env):
    """The same plant, with lights and a floor texture. Dynamics untouched."""
    xml = MT.quadruped_rig_spooled(
        q_ref={nm: list(v) for nm, v in env.q_ref.items()},
        series_k=env.series_k, hip_height=0.176, spine=True,
        spool_servo=True, sensors=True)
    i = xml.index(">", xml.index("<mujoco")) + 1
    xml = xml[:i] + VISUAL + xml[i:]
    xml = xml.replace('rgba="0.9 0.9 0.9 1"', 'material="gridm" rgba="1 1 1 1"')
    xml = xml.replace("  <worldbody>",
                      '  <worldbody>\n    <light name="key" pos="0.5 -0.6 1.2"'
                      ' dir="-0.35 0.45 -1" directional="true"'
                      ' diffuse="0.5 0.5 0.5" castshadow="true"/>', 1)
    return mujoco.MjModel.from_xml_string(xml)


def png(path, rgb):
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        c = tag + data
        return (struct.pack(">I", len(data)) + c
                + struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF))

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n"
                + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6))
                + chunk(b"IEND", b""))


def film(action, frames_at, label, seed=0, policy=None):
    """Run one episode, grabbing a frame at each step in `frames_at`."""
    env = TomcatEnv(rng=np.random.default_rng(seed))
    obs = env.reset()
    rm = render_model(env)
    rd = mujoco.MjData(rm)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth, cam.elevation, cam.distance = 90.0, -8.0, 0.75
    shots, caption, done_at = [], [], None
    with mujoco.Renderer(rm, H, W) as r:
        for k in range(max(frames_at) + 1):
            if k in frames_at:
                rd.qpos[:] = env.data.qpos
                mujoco.mj_forward(rm, rd)
                cam.lookat[:] = [float(rd.qpos[0]), 0.0, 0.09]
                r.update_scene(rd, camera=cam)
                shots.append(r.render().copy())
                caption.append("t=%.2fs tilt=%.0f h=%.0fmm"
                               % (env.t, TomcatEnv._tilt(obs),
                                  1e3 * env.stance_height(obs)))
            if done_at is None:
                a = policy(obs) if policy else action
                obs, _, term, _ = env.step(a)
                if term:
                    done_at = k
    return shots, caption, done_at


def strip(rows, path):
    """Tile [(frames, label)] into one image, one row per behaviour."""
    h, w, _ = rows[0][0][0].shape
    n = max(len(f) for f, _ in rows)
    canvas = np.full((h * len(rows), w * n, 3), 255, np.uint8)
    for i, (frames, _) in enumerate(rows):
        for j, fr in enumerate(frames):
            canvas[i * h:(i + 1) * h, j * w:(j + 1) * w] = fr
    png(path, canvas)
    return canvas.shape


def main():
    os.makedirs(OUT, exist_ok=True)
    at = [0, 5, 10, 20, 40]
    hold = np.zeros(18)
    for i, nm in enumerate(("LF", "RF", "LR", "RR")):
        hold[3 * i:3 * i + 3] = ((37.8, 45.5, 36.3) if nm[1] == "F"
                                 else (43.9, 14.1, 65.1))
    hold[12:] = (-145.0, 0.0, -71.3, 0.0, -23.2, 0.0)
    rng = np.random.default_rng(0)

    runs = [
        (np.zeros(18), "zero tension"),
        (hold, "gravity-compensating hold"),
        (None, "random"),
    ]
    rows = []
    for act, label in runs:
        pol = (lambda o: rng.uniform(-223, 223, 18)) if act is None else None
        frames, caps, done = film(act, at, label, policy=pol)
        rows.append((frames, label))
        print("%-28s terminates at step %s | %s"
              % (label, done, "  ".join(caps)))
    shape = strip(rows, os.path.join(OUT, "episode_strip.png"))
    print("\nwrote docs/figures/episode_strip.png %s" % (shape,))
    print("rows: %s" % ", ".join(l for _, l in runs))
    print("columns are steps %s (control period %.1f ms)"
          % (at, 1000.0 / 133.3))


if __name__ == "__main__":
    main()
