# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The tail: the part with a motor in the budget and no geometry (build123d).

⚠️ **19 motors are designed and 18 are placed.** ADR decision F bought 19 (12
leg + 3 spine pitch + 3 spine yaw + **1 tail**) and `params.py` still budgets a
"7-motor spine+tail bank". M88 redistributed the motors along the trunk and
placed **18** -- the tail's motor fell out and nothing counted it back.

⚠️ **And SPINE_TAIL_SPEC mounts the tail at `x = 0`, which is now 112 mm INSIDE
the girdle.** That was the pelvic girdle's rear face when the trunk was two
boxes; M88's rear girdle runs from x = -112 to -7 and the hip sits at its FRONT.
A tail based at x = 0 grows forward through its own body.

So this module does three things: puts the tail where the trunk now ends, gives
it mass and a section so it stops being free, and places the motor that drives
it.  Per ADR-0007 (revised) it is **one cable that tensions and loosens** -- no
precision -- and G6, the righting goal it was bought for, is withdrawn
([ADR-0071](../../docs/DESIGN_DECISIONS.md)). What is left is form and a gross
inertial contribution, so what matters is that it costs little.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
from build123d import Compound, Cylinder, Plane, Pos, Sphere

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "kinematics", "src"))

import tomcat_trunk as TT                                        # noqa: E402
import tomcat_packaging as TP                                    # noqa: E402

MM = 1000.0

#: Where the tail leaves the body: the rear girdle's rear FACE, at the spine
#: axis, because the caudal chain is the continuation of the vertebral one.
BASE_X = TT.BODIES[0][0]
BASE_Z = TT.SPINE_Z

#: Caudal vertebra count, ANATOMY.md's 19-21 taken at its middle.
N_CAUDAL = 20

#: Tail length as a fraction of the trunk. ⚠️ `[assumed]` -- ANATOMY.md gives a
#: vertebra COUNT and no proportion, and the value matters: a cat's tail is
#: roughly 0.6-0.9 of its body, while the tail `tomcat_skeleton` drew reaches
#: 0.39 of this trunk. `report()` prices the choice rather than hiding it.
LENGTH_FRAC = 0.62

#: Root and tip radii (mm). The root matches the girdle's rear face; the tip is
#: what a cable termination needs.
ROOT_R = 9.0
TIP_R = 2.5

#: Silicone over a printed core, so it is a skin-like part rather than a bone.
RHO = 1.15e-3


def spine_curve(curl=0.0, n=None):
    """Caudal vertebra centres, base to tip.

    `curl` is the single cable's action: 0 is the relaxed trail, 1 the fully
    tensioned curl. ADR-0007 (revised) asks for nothing between them.
    """
    n = n or N_CAUDAL
    span = TT.BODIES[3][1] - TT.BODIES[0][0]
    L = LENGTH_FRAC * span
    pts = []
    for i in range(n + 1):
        t = i / n
        # relaxed: trails back and droops; curled: sweeps up and over
        theta = curl * (math.pi * 0.85) * t - (1.0 - curl) * 0.30 * t
        x = BASE_X - L * (math.sin(theta + 1e-9) / max(theta, 1e-9)
                          if abs(theta) > 1e-6 else 1.0) * t * 0.0
        pts.append(t)
    # integrate the curve instead: constant-curvature arc of arclength L
    out, x, z, ang = [], BASE_X, BASE_Z, 0.0
    k = (curl * 2.4 - (1.0 - curl) * 0.35) / L        # 1/mm
    ds = L / n
    for i in range(n + 1):
        out.append((x, z))
        ang += k * ds
        x -= ds * math.cos(ang)
        z += ds * math.sin(ang)
    return out


def radius_at(t):
    """Tapered section: root to tip."""
    return ROOT_R + (TIP_R - ROOT_R) * t


def tail(curl=0.0):
    """The tail as a tapering chain of caudal segments."""
    pts = spine_curve(curl)
    parts = []
    for i, ((x0, z0), (x1, z1)) in enumerate(zip(pts[:-1], pts[1:])):
        t = i / max(len(pts) - 2, 1)
        r = radius_at(t)
        d = np.array([x1 - x0, 0.0, z1 - z0])
        L = float(np.linalg.norm(d))
        if L < 1e-6:
            continue
        parts.append(Plane(origin=((x0 + x1) / 2, 0.0, (z0 + z1) / 2),
                           z_dir=tuple(d)).location * Cylinder(r, L))
        parts.append(Pos(x1, 0.0, z1) * Sphere(r * 1.02))
    return Compound(parts)


def length():
    return LENGTH_FRAC * (TT.BODIES[3][1] - TT.BODIES[0][0])


def mass_g(curl=0.0):
    return sum(s.volume for s in tail(curl).solids()) * RHO


#: Whole-body pitch inertia about the CoM, without a tail, from the plant.
#: `[derived: mjcf_tendon.quadruped_rig]`
BODY_IYY = 4.2023e-02


def inertia_about(pt, curl=0.0, rho=None):
    """Pitch inertia the tail adds about a body point `(x, z)` in mm."""
    rho = RHO if rho is None else rho
    bx, bz = pt
    out = 0.0
    for sd in tail(curl).solids():
        c = sd.center()
        out += sd.volume * rho * 1e-3 * (((c.X - bx) * 1e-3) ** 2
                                         + ((c.Z - bz) * 1e-3) ** 2)
    return out


def report(body_com=(109.0, 15.0)):
    ok = True
    print("TAIL -- the part with a motor in the budget and no geometry\n")
    print("  base            (%.0f, %.0f)  -- the girdle's rear face, on the "
          "spine axis" % (BASE_X, BASE_Z))
    print("  length          %.0f mm = %.2f of the %.0f mm trunk  [assumed]"
          % (length(), LENGTH_FRAC, TT.BODIES[3][1] - TT.BODIES[0][0]))
    print("  caudal segments %d   root ø%.0f -> tip ø%.0f"
          % (N_CAUDAL, 2 * ROOT_R, 2 * TIP_R))

    # @W@ it must not grow through its own body
    tr = Compound([TT.rigid_body(0)])
    inter = tr.intersect(tail(0.0))
    v = 0.0
    if inter is not None:
        try:
            v = sum(x.volume for x in inter.solids())
        except Exception:
            v = 0.0
    print("\ninto the girdle %.1f mm3" % v)
    if v > 50.0:
        print("      *** the tail grows through the trunk")
        ok = False

    # @W@ and it must not drag
    lo = min(s.bounding_box().min.Z for s in tail(0.0).solids())
    print("  ground clearance %.0f mm at full droop (hip at 175)" % (175.0 + lo))
    if 175.0 + lo < 20.0:
        print("      *** the tail drags")
        ok = False

    print("\nthe inertial account, about the body CoM at (%.0f, %.0f):"
          % body_com)
    print("  %-10s %9s %11s %11s %9s" % ("material", "mass g", "adds Iyy",
                                         "curl swing", "cost/use"))
    for nm, rho in (("silicone", RHO), ("EVA foam", 0.20e-3),
                    ("hollow TPU", 0.35e-3)):
        m = sum(x.volume for x in tail().solids()) * rho
        i0 = inertia_about(body_com, 0.0, rho)
        i1 = inertia_about(body_com, 1.0, rho)
        print("  %-10s %9.1f %10.2f %% %10.2f %% %8.0fx"
              % (nm, m, 100 * i0 / BODY_IYY, 100 * abs(i0 - i1) / BODY_IYY,
                 i0 / max(abs(i0 - i1), 1e-12)))
    print("  NOTE: the curl modulates a fourteenth of what the tail costs, at any")
    print("     density -- it is all lever, so only the MASS is a design choice")

    # @W@ the motor nobody placed
    placed = sum(len(TT.ROWS[n][0]) for (_a, _b, _c, n, _r) in TT._rows())
    print("\nmotors: %d placed by the trunk, %d in the decision "
          "(12 leg + 3 pitch + 3 yaw + 1 TAIL)" % (placed, 19))
    if placed != 19:
        print("      *** the tail's motor is not in the packing")
        ok = False
    print("  %s" % ("tail checks pass" if ok else "*** SEE ABOVE ***"))
    return ok


def render_png(path, elev=8, azim=-90):
    """The tail relaxed and curled, over the rear of the trunk."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(11, 6))
    ax = fig.add_subplot(111, projection="3d")
    groups = [("#8a9bb0", 1.0, [TT.rigid_body(0)]),
              ("#c98f6a", 0.9, [tail(0.0)]),
              ("#7a9a5a", 0.5, [tail(1.0)])]
    for col, alpha, shapes in groups:
        tri = []
        for sh in shapes:
            for f in sh.faces():
                try:
                    verts, tris = f.tessellate(0.5)
                except Exception:
                    continue
                V = np.array([[v.X, v.Y, v.Z] for v in verts])
                tri.append(V[np.array(tris)])
        if not tri:
            continue
        coll = Poly3DCollection(np.concatenate(tri), facecolor=col, alpha=alpha,
                                edgecolor="#3c4a5a", linewidth=0.05)
        coll.set_zsort("average")
        ax.add_collection3d(coll)
    whole = Compound([TT.rigid_body(0), tail(0.0), tail(1.0)])
    bb = whole.bounding_box()
    ax.set_xlim(bb.min.X - 5, bb.max.X + 5)
    ax.set_ylim(bb.min.Y - 5, bb.max.Y + 5)
    ax.set_zlim(bb.min.Z - 5, bb.max.Z + 5)
    ax.set_box_aspect((bb.size.X, max(bb.size.Y, 40), bb.size.Z))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(path, dpi=140, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", path)


if __name__ == "__main__":
    report()
    render_png(os.path.join(HERE, "tomcat_tail.png"))
