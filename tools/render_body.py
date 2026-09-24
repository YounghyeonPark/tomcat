# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Render the CURRENT whole body from the live builder.

Every geom, tendon and spool in these views comes out of ``mjcf_tendon`` at
render time, so the picture cannot drift from the model the way a saved render
can. Run:

    python tools/render_body.py

The plant is the shipped one as of M71: ADR-0006's articulated spine, ADR-0008's
pulley transmission, and G3 on all eighteen cables -- twelve leg pairs and six
spine pairs, each with a rotor, a series spring and a winding constraint
(ADR-0076). The pose is the stance pose every standing figure is measured at.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "kinematics", "src"))

from tomcat_kin import mjcf_tendon as MT                    # noqa: E402
from tomcat_kin.params import DEFAULT_TENDON  # noqa: E402  (M112: G3 from one source)
from tomcat_kin import LegModel                          # noqa: E402
from tomcat_kin.params import DEFAULT_FORELEG, DEFAULT_HINDLEG   # noqa: E402

OUT = os.path.join(ROOT, "docs", "figures")
LEGS = ("LF", "RF", "LR", "RR")
W, H = 1280, 960


def stance(foot_x=0.04, foot_z=-0.17):
    """The pose ADR-0058's standing gate and everything after it is measured at."""
    lp = {"LF": DEFAULT_FORELEG, "RF": DEFAULT_FORELEG,
          "LR": DEFAULT_HINDLEG, "RR": DEFAULT_HINDLEG}
    return {nm: np.asarray(LegModel(lp[nm]).inverse((foot_x, foot_z, 0.0)),
                           float) for nm in LEGS}


#: Rendering-only additions. They touch no dynamics -- an offscreen framebuffer
#: big enough for the requested image, a light, and a floor texture -- so the
#: model being drawn is still exactly the model the tests run.
VISUAL = f"""
  <visual>
    <global offwidth="{W}" offheight="{H}"/>
    <quality shadowsize="4096" offsamples="8"/>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.45 0.45 0.45"
               specular="0.15 0.15 0.15"/>
    <map shadowclip="0.6" shadowscale="0.7"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" width="256" height="256"
             rgb1="0.96 0.96 0.97" rgb2="0.80 0.83 0.87"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.94 0.94 0.93"
             rgb2="0.88 0.88 0.87" width="512" height="512"/>
    <material name="gridm" texture="grid" texrepeat="16 16" reflectance="0.05"/>
  </asset>
"""


def _for_rendering(xml: str) -> str:
    i = xml.index(">", xml.index("<mujoco")) + 1
    xml = xml[:i] + VISUAL + xml[i:]
    xml = xml.replace('rgba="0.9 0.9 0.9 1"',
                      'material="gridm" rgba="1 1 1 1"')
    return xml.replace(
        "  <worldbody>",
        '  <worldbody>\n'
        '    <light name="key" pos="0.5 -0.6 1.2" dir="-0.35 0.45 -1"'
        ' directional="true" diffuse="0.55 0.55 0.55"'
        ' specular="0.2 0.2 0.2" castshadow="true"/>', 1)


def build(spooled=True):
    q = stance()
    if spooled:
        xml = MT.quadruped_rig_spooled(
            q_ref={nm: list(v) for nm, v in q.items()}, series_k=DEFAULT_TENDON.series_k,
            hip_height=0.176, spine=True, spool_servo=True, spine_spools=True)
    else:
        xml = MT.quadruped_rig(hip_height=0.176, spine=True)
    m = mujoco.MjModel.from_xml_string(_for_rendering(xml))
    d = mujoco.MjData(m)
    for nm in LEGS:
        for i, jn in enumerate((f"{nm}_q1", f"{nm}_q2", f"{nm}_q3")):
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
            d.qpos[m.jnt_qposadr[j]] = q[nm][i]
    mujoco.mj_forward(m, d)
    return m, d


VIEWS = [
    # name, azimuth, elevation, distance, lookat-z
    ("three-quarter", 140.0, -14.0, 0.70, 0.085),
    ("side", 90.0, -4.0, 0.62, 0.085),
    ("front", 180.0, -8.0, 0.66, 0.095),
    # the rear quarter is the one that shows the spine drivetrain: six spools on
    # the rear girdle, which is where M71 put them (ADR-0076)
    ("rear-quarter", 25.0, -22.0, 0.62, 0.100),
    ("top", 90.0, -62.0, 0.72, 0.060),
]


def main():
    os.makedirs(OUT, exist_ok=True)
    m, d = build()
    print(f"plant: nv={m.nv} actuators={m.nu} tendons={m.ntendon} "
          f"equalities={m.neq} bodies={m.nbody}")

    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_TENDON] = True
    opt.flags[mujoco.mjtVisFlag.mjVIS_ACTUATOR] = True

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    com = np.array(d.subtree_com[0])

    with mujoco.Renderer(m, H, W) as r:
        for name, az, el, dist, z in VIEWS:
            cam.azimuth, cam.elevation, cam.distance = az, el, dist
            cam.lookat[:] = [com[0], 0.0, z]
            r.update_scene(d, camera=cam, scene_option=opt)
            img = r.render()
            path = os.path.join(OUT, f"body_{name}.png")
            _write_png(path, img)
            print("wrote", os.path.relpath(path, ROOT))


def _write_png(path, rgb):
    """PNG without a Pillow dependency -- zlib and a hand-rolled chunk writer."""
    import struct
    import zlib
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        c = tag + data
        return (struct.pack(">I", len(data)) + c
                + struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF))

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 6))
           + chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(png)


if __name__ == "__main__":
    main()
