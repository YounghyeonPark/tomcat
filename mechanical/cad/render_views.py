"""Multi-view renders of the T.O.M.C.A.T. CAD models.

Produces one legible figure per model (side elevation · top plan · isometric)
instead of the single cluttered iso view the build scripts emit.

    python mechanical/cad/render_views.py
    -> views_skeleton.png, views_packaging.png
"""

from __future__ import annotations

import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(__file__)
sys.path.insert(0, HERE)

import tomcat_skeleton as sk       # noqa: E402
import tomcat_packaging as pk      # noqa: E402

STYLE = {
    "bone":   ("#d7ac86", 1.00),
    "motor":  ("#37475a", 1.00),
    "tendon": ("#c1762e", 1.00),
    "pulley": ("#8a9bb0", 1.00),
    "girdle": ("#9fb0c3", 0.10),
    "flat":   ("#c49a72", 1.00),
    "joint":  ("#8a5a3c", 1.00),
    "bay":    ("#7f93a8", 0.22),
}
# matplotlib: azim=-90,elev=0 looks along +y -> the x-z SIDE view;
# elev=90 looks down -z -> top plan.
VIEWS = (("side elevation", 0, -90), ("top plan", 90, -90), ("isometric", 22, -58))


def mesh(comp, tol=0.45):
    verts, tris = comp.tessellate(tol)
    return np.array([[v.X, v.Y, v.Z] for v in verts]), np.array(tris)


def panel(ax, meshes, elev, azim, bounds, title):
    for name, (V, T) in meshes.items():
        color, alpha = STYLE.get(name, ("#999999", 1.0))
        pc = Poly3DCollection(V[T], facecolor=color, edgecolor="none", alpha=alpha)
        pc.set_zsort("average")
        ax.add_collection3d(pc)
    (x0, x1), (y0, y1), (z0, z1) = bounds
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_zlim(z0, z1)
    ax.set_box_aspect((x1 - x0, y1 - y0, z1 - z0))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    ax.set_title(title, fontsize=10, color="#33404f", pad=2)


def figure(groups, bounds, out, suptitle):
    meshes = {n: mesh(c) for n, c in groups.items()
              if c is not None and len(c.solids()) > 0}
    fig = plt.figure(figsize=(15, 4.6))
    for i, (name, elev, azim) in enumerate(VIEWS, start=1):
        panel(fig.add_subplot(1, 3, i, projection="3d"),
              meshes, elev, azim, bounds, name)
    fig.suptitle(suptitle, fontsize=12, color="#233240", y=0.99)
    fig.tight_layout()
    fig.savefig(out, dpi=135, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"  wrote {os.path.basename(out)} ({os.path.getsize(out)} bytes)")


def bounds_of(*comps, pad=25.0):
    lo = np.array([+1e9] * 3); hi = np.array([-1e9] * 3)
    for c in comps:
        if c is None or len(c.solids()) == 0:
            continue
        bb = c.bounding_box()
        lo = np.minimum(lo, [bb.min.X, bb.min.Y, bb.min.Z])
        hi = np.maximum(hi, [bb.max.X, bb.max.Y, bb.max.Z])
    lo -= pad; hi += pad
    lo[2] = min(lo[2], 0.0)          # always show the ground plane
    return tuple((lo[i], hi[i]) for i in range(3))


if __name__ == "__main__":
    skel, _, _ = sk.build()
    figure(skel, bounds_of(*skel.values()),
           os.path.join(HERE, "views_skeleton.png"),
           "T.O.M.C.A.T. — skeleton: bones, joint axes, vertebrae, scapula, ribcage")

    groups, _, _, _, _ = pk.build()
    figure(groups, bounds_of(*groups.values()),
           os.path.join(HERE, "views_packaging.png"),
           "T.O.M.C.A.T. — packaging: 19 motors (ADR-0008/0009) · tendons · joint pulleys")
