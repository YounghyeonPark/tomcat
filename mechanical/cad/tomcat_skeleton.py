"""Parametric 3D CAD model of the T.O.M.C.A.T. skeleton (build123d).

This is the first *real geometry* pass — it turns the placeholder dimensions of
the `tomcat_kin` simulation model into an actual 3D solid, exported to STEP/STL.
It deliberately imports the SAME parameters the kinematics model uses
(`LegModel`, `DEFAULT_LEG`, `DEFAULT_SPINE`) so the CAD and the simulation cannot
drift apart — change a link length in params.py and this model follows.

Scope / honesty:
- A biomimetic *skeleton massing model*, not a manufacturing model: bones are
  capsules, joints are spheres, girdles/motors are simple prisms/cylinders.
- Sagittal leg geometry comes from the 2D model; left/right legs are placed at a
  chosen track width (the 2D model has no y). All placeholder until mechanical
  design fixes real geometry.

Run:  python mechanical/cad/tomcat_skeleton.py
Out:  tomcat_skeleton.step, tomcat_skeleton.stl, tomcat_skeleton.png (this dir)
"""

from __future__ import annotations

import os
import sys

import numpy as np
from build123d import (
    Box, Cylinder, Sphere, Compound, Vector, Plane, Pos,
    export_step, export_stl,
)

# --- pull real dimensions from the kinematics model (single source of truth) ---
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "kinematics", "src"))
from tomcat_kin import LegModel, KneeConfig  # noqa: E402
from tomcat_kin.params import DEFAULT_LEG, DEFAULT_SPINE  # noqa: E402

MM = 1000.0  # model is in metres; CAD in millimetres

# ---- placeholder body params the 2D model doesn't carry (❓ TBD) ----
TRACK = 90.0          # lateral spacing of left/right legs (mm)
BODY_R = 16.0         # spine/vertebra radius (mm)
BONE_R = 7.0          # leg bone radius (mm)
GIRDLE = (70.0, 120.0, 46.0)   # girdle housing L×W×H (mm)

# Standing stance: solve IK so each foot plants on the ground under its hip.
FOOT_Z = -0.20        # foot height below the hip (m) -> hip/body height
FOOT_PITCH = -0.25    # foot pitch (rad)
FRONT_FOOT_X = 0.02   # front feet slightly forward (m)
REAR_FOOT_X = -0.02   # rear feet slightly back (m)


def bone(p0, p1, r):
    """A capsule (cylinder + end spheres) between two 3D points."""
    p0, p1 = Vector(*p0), Vector(*p1)
    d = p1 - p0
    L = d.length
    if L < 1e-6:
        return Sphere(r)
    mid = (p0 + p1) / 2
    cyl = Plane(origin=mid, z_dir=d.normalized()).location * Cylinder(r, L)
    return Compound([cyl, Pos(*tuple(p0)) * Sphere(r),
                     Pos(*tuple(p1)) * Sphere(r)])


def leg(mount, foot_x, knee, mirror=False):
    """One digitigrade 4-link leg at `mount` (x,y,z) mm; IK plants the paw.

    `mirror` reflects the leg fore/aft about the hip so front and rear legs fold
    in opposite directions (the cat's fore-vs-hind limb geometry).
    """
    leg_model = LegModel()
    q = leg_model.inverse((foot_x, FOOT_Z, FOOT_PITCH), knee=knee)
    pts2d = leg_model.joint_positions(q) * MM  # (5,2): hip,stifle,hock,paw-base,paw-tip
    if mirror:
        pts2d = pts2d * np.array([-1.0, 1.0])
    mx, my, mz = mount
    p3 = [(mx + x, my, mz + z) for (x, z) in pts2d]  # map sagittal x,z into world
    parts = []
    for a, b in zip(p3[:-1], p3[1:]):
        parts.append(bone(a, b, BONE_R))
    # a slightly larger hip knuckle
    parts.append(Pos(*p3[0]) * Sphere(BONE_R * 1.4))
    return Compound(parts)


def girdle(x_center, z):
    """A girdle housing box with a couple of motor-spool cylinders on top."""
    L, W, H = GIRDLE
    body = Pos(x_center, 0, z) * Box(L, W, H)
    spools = []
    for sy in (-W / 4, 0, W / 4):
        spools.append(Pos(x_center, sy, z + H / 2) * Cylinder(9, 18))
    return Compound([body, *spools])


def spine(z):
    """Three tapered vertebral segments between the two girdles."""
    seg = np.asarray(DEFAULT_SPINE.segment_lengths) * MM
    xs = np.concatenate([[0.0], np.cumsum(seg)])  # vertebra x positions
    # taper the radius front-ward a touch
    radii = np.linspace(BODY_R * 1.15, BODY_R * 0.85, len(xs))
    parts = []
    for (x0, x1, r) in zip(xs[:-1], xs[1:], radii[:-1]):
        parts.append(bone((x0, 0, z), (x1, 0, z), float(r)))
    for x, r in zip(xs, radii):
        parts.append(Pos(x, 0, z) * Sphere(float(r) * 0.9))
    return Compound(parts), float(xs[-1])


def tail(z):
    """A passive multi-link tail curling up and back from the pelvic girdle."""
    # points going -x (rearward) and up, tapering
    pts = [(0, 0, z), (-70, 0, z + 20), (-125, 0, z + 55), (-160, 0, z + 100)]
    radii = [10, 8, 6, 4]
    parts = []
    for (a, b, r) in zip(pts[:-1], pts[1:], radii):
        parts.append(bone(a, b, r))
    return Compound(parts)


def build():
    H = -FOOT_Z * MM  # hip/body height so feet touch z=0 (ground)
    kp = KneeConfig.FLEXED_POSITIVE

    spine_body, front_x = spine(H)

    # Front legs mirror the rear so the limbs fold in opposite directions
    # (cat fore- vs hind-limb geometry): rear stifle points forward, front
    # elbow/carpus folds back.
    legs = Compound([
        leg((front_x, +TRACK / 2, H), FRONT_FOOT_X, kp, mirror=True),   # front-left
        leg((front_x, -TRACK / 2, H), FRONT_FOOT_X, kp, mirror=True),   # front-right
        leg((0.0, +TRACK / 2, H), REAR_FOOT_X, kp),                     # rear-left
        leg((0.0, -TRACK / 2, H), REAR_FOOT_X, kp),                     # rear-right
    ])
    girdles = Compound([girdle(0.0, H), girdle(front_x, H)])
    robot = Compound([spine_body, girdles, legs, tail(H)])
    return robot, H, front_x


def render_png(shape, path, H, front_x):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    verts, tris = shape.tessellate(0.4)
    V = np.array([[v.X, v.Y, v.Z] for v in verts])
    T = np.array(tris)
    fig = plt.figure(figsize=(11, 6))
    ax = fig.add_subplot(111, projection="3d")
    coll = Poly3DCollection(V[T], facecolor="#c98a4e", edgecolor="none", alpha=1.0)
    coll.set_zsort("average")
    ax.add_collection3d(coll)
    # ground
    gx = np.array([[-220, front_x + 120], [-220, front_x + 120]])
    gy = np.array([[-90, -90], [90, 90]])
    ax.plot_surface(gx, gy, np.zeros((2, 2)), color="#3a4656", alpha=0.25)
    ax.set_xlim(-220, front_x + 140); ax.set_ylim(-180, 180); ax.set_zlim(0, H + 140)
    ax.set_box_aspect((front_x + 360, 360, H + 140))
    ax.view_init(elev=18, azim=-62)
    ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(path, dpi=130, bbox_inches="tight", facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    here = os.path.dirname(__file__)
    robot, H, front_x = build()
    bb = robot.bounding_box()
    print(f"skeleton built. hip height H = {H:.1f} mm, front girdle x = {front_x:.1f} mm")
    print(f"bounding box (mm): "
          f"{bb.size.X:.0f} (L) x {bb.size.Y:.0f} (W) x {bb.size.Z:.0f} (H)")
    step = os.path.join(here, "tomcat_skeleton.step")
    stl = os.path.join(here, "tomcat_skeleton.stl")
    png = os.path.join(here, "tomcat_skeleton.png")
    export_step(robot, step)
    export_stl(robot, stl, tolerance=0.2, angular_tolerance=0.3)
    render_png(robot, png, H, front_x)
    for p in (step, stl, png):
        print(f"  wrote {os.path.basename(p)}  ({os.path.getsize(p)} bytes)")
