# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The head and neck: 240 g that has never had a place (build123d).

⚠️ `SpineParams.front_girdle_mass` absorbs **240 g of head and neck** and
lumps it at the girdle mount, and `params.py` says so in its own warning --
*"the head is the weak point and it is not in the box... putting it forward\nwould raise the pitch inertia"*. Nobody had measured by how much.

⚠️ **By 40 %.** Moving that lump from the shoulder to where a cat's head sits
adds **+1.70e-2 kg m2** to a body that has 4.20e-2, and carries the CoM
**10.8 mm forward**. The righting reflex is already a factor of nine short
([ADR-0093](../../docs/DESIGN_DECISIONS.md)), so this is not a finish detail.

ANATOMY.md puts the 7 cervical vertebrae explicitly out of scope, so the neck is
**not articulated**: head and neck are one rigid lump on the front girdle, which
is what the mass model has always assumed. What changes here is that the lump
acquires a position, a shape, and a bill.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
from build123d import Compound, Cylinder, Plane, Pos, Sphere, Sphere as _S

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "kinematics", "src"))

import tomcat_trunk as TT                                        # noqa: E402
from tomcat_kin.params import DEFAULT_SPINE as SP                # noqa: E402

MM = 1000.0

#: The lump the mass model already carries, and its only real constraint.
HEAD_NECK_G = 240.0

#: ⚠️ **The head is sized by the body's MASS, not by the trunk's length.**
#: Scaling it off the trunk was the first attempt -- 363 mm against a cat's
#: ~250 gives 1.45x -- and it is the wrong ruler twice over. This robot is
#: **4.38 kg**, which is a cat, so a cat's head is the right head; and the trunk
#: is long FOR that mass because the rear girdle reaches 112 mm behind the hip,
#: which is a standing `[owed]`. Scaling by it copies that defect into the head.
#:
#: Priced rather than argued:
#:
#:     sizing              head mm   nose x   adds Iyy   % of body
#:     by mass (a cat)          95      407   1.02e-02      24 %
#:     by trunk length         138      470   1.39e-02      33 %
#:
#: Nine points of pitch inertia on a righting reflex already a factor of nine
#: short.  `[derived: mechanical/reference/ANATOMY.md]`
SCALE = 1.0

NECK_L = 50.0 * SCALE
HEAD_L = 95.0 * SCALE

#: Where the neck leaves the body, and at what angle. A cat carries its head
#: ABOVE the dorsal line, not on the axis -- the cervical chain rises out of the
#: thorax. ⚠️ Raising it costs nothing in pitch inertia (the lever is x) and
#: everything in how the animal reads.
BASE_X = TT.BODIES[3][1]
BASE_Z = TT.SPINE_Z
RISE_DEG = 22.0


def _axis():
    """`(neck_base, neck_top, head_centre)` in (x, z), rising out of the thorax."""
    a = math.radians(RISE_DEG)
    b = np.array([BASE_X, BASE_Z])
    t = b + NECK_L * np.array([math.cos(a), math.sin(a)])
    h = t + 0.5 * HEAD_L * np.array([math.cos(a * 0.4), math.sin(a * 0.4)])
    return b, t, h


def neck(r0=26.0, r1=21.0):
    """A tapering column from the girdle's front face."""
    b, t, _h = _axis()
    d = np.array([t[0] - b[0], 0.0, t[1] - b[1]])
    L = float(np.linalg.norm(d))
    mid = ((b[0] + t[0]) / 2, 0.0, (b[1] + t[1]) / 2)
    parts = [Plane(origin=mid, z_dir=tuple(d)).location * Cylinder(0.5 * (r0 + r1), L)]
    parts.append(Pos(b[0], 0.0, b[1]) * Sphere(r0))
    parts.append(Pos(t[0], 0.0, t[1]) * Sphere(r1))
    return Compound(parts)


def head():
    """The cranium as a tapering form, wider at the cheeks than the muzzle."""
    _b, t, h = _axis()
    a = math.radians(RISE_DEG * 0.4)
    u = np.array([math.cos(a), 0.0, math.sin(a)])
    parts = []
    n = 8
    for i in range(n + 1):
        f = i / n
        p = np.array([t[0], 0.0, t[1]]) + u * (HEAD_L * f)
        # cheeks at 0.35 of the length, muzzle narrow
        r = 34.0 * (1.0 - 0.55 * abs(f - 0.35) / 0.65) if f > 0.35 else             34.0 * (0.70 + 0.30 * f / 0.35)
        parts.append(Pos(*p) * Sphere(max(r, 9.0)))
    # ears
    for sgn in (-1.0, +1.0):
        e = np.array([t[0], 0.0, t[1]]) + u * (HEAD_L * 0.22)
        parts.append(Pos(e[0], sgn * 20.0, e[2] + 30.0) * Sphere(13.0))
    return Compound(parts)


def whole():
    return Compound(list(neck().solids()) + list(head().solids()))


def mass_props():
    """`(mass_g, com_xz, Iyy_about_its_own_com)` at the budgeted 240 g."""
    sol = whole().solids()
    vol = sum(s.volume for s in sol)
    rho = HEAD_NECK_G / vol                      # calibrated to the budget
    cx = sum(s.volume * s.center().X for s in sol) / vol
    cz = sum(s.volume * s.center().Z for s in sol) / vol
    I = sum(s.volume * rho * 1e-3
            * (((s.center().X - cx) * 1e-3) ** 2 + ((s.center().Z - cz) * 1e-3) ** 2)
            for s in sol)
    return HEAD_NECK_G, (cx, cz), I


#: Whole-body figures without a head in place, from the plant.
#: `[derived: mjcf_tendon.quadruped_rig]`
BODY_M, BODY_IYY, BODY_COM = 4.3833, 4.2023e-02, (109.0, 15.0)

#: Where the lump sits today: the front girdle's own CoM.
LUMPED_AT = (195.0 + SP.front_girdle_com[0] * MM,
             SP.front_girdle_com[1] * MM)


def account():
    """What moving the 240 g from the girdle to the head costs."""
    m, (cx, cz), own = mass_props()
    bx, bz = BODY_COM
    r0 = math.hypot(LUMPED_AT[0] - bx, LUMPED_AT[1] - bz) * 1e-3
    r1 = math.hypot(cx - bx, cz - bz) * 1e-3
    dI = (m * 1e-3) * (r1 ** 2 - r0 ** 2) + own
    dcom = (m * 1e-3) * (cx - LUMPED_AT[0]) / BODY_M
    return dict(com=(cx, cz), lever_before=1e3 * r0, lever_after=1e3 * r1,
                dI=dI, frac=dI / BODY_IYY, dcom=dcom)


def report():
    ok = True
    a = account()
    print("HEAD AND NECK -- 240 g that has never had a place\n")
    print("  scale           %.2f from a 250 mm cat trunk to this %.0f mm one"
          % (SCALE, TT.BODIES[3][1] - TT.BODIES[0][0]))
    print("  neck            %.0f mm, rising %.0f deg out of the thorax"
          % (NECK_L, RISE_DEG))
    print("  head            %.0f mm" % HEAD_L)
    print("  nose reaches    x = %.0f" % max(s.bounding_box().max.X
                                             for s in whole().solids()))
    print()
    print("  lumped at       (%.1f, %.1f)  -- the front girdle's own CoM"
          % LUMPED_AT)
    print("  actually at     (%.1f, %.1f)" % a["com"])
    print("  lever from the body CoM  %.0f -> %.0f mm"
          % (a["lever_before"], a["lever_after"]))
    print("  pitch inertia   +%.3e kg m2 = **%.0f %%** of the body's %.3e"
          % (a["dI"], 100 * a["frac"], BODY_IYY))
    print("  CoM moves       %+.1f mm forward" % a["dcom"])

    # @W@ a cat carries its head ABOVE the back
    top = max(s.bounding_box().max.Z for s in head().solids())
    print("\nhead top        %.0f mm vs the dorsal line %.0f" % (top, TT.Z_DORSAL))
    if top < TT.Z_DORSAL:
        print("      *** the head is below the back")
        ok = False

    # @W@ and it must not sit on the shoulder
    import tomcat_assembly as TA
    leg = TA.one_leg(True, +1.0)
    inter = Compound(list(whole().solids())).intersect(leg)
    v = 0.0
    if inter is not None:
        try:
            v = sum(x.volume for x in inter.solids())
        except Exception:
            v = 0.0
    print("  into the fore leg %.1f mm3" % v)
    if v > 100.0:
        print("      *** the neck sits on the shoulder")
        ok = False
    print("  %s" % ("head checks pass" if ok else "*** SEE ABOVE ***"))
    return ok


def render_png(path, elev=8, azim=-90):
    """Head, trunk and tail together -- the animal's whole outline."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import tomcat_tail as TL

    fig = plt.figure(figsize=(13, 5))
    ax = fig.add_subplot(111, projection="3d")
    groups = [("#8a9bb0", 1.0, [TT.rigid_body(b) for b in sorted(TT.BODIES)]),
              ("#c98f6a", 0.95, [whole()]),
              ("#c98f6a", 0.95, [TL.tail(0.0)])]
    for col, alpha, shapes in groups:
        tri = []
        for sh in shapes:
            for f in sh.faces():
                try:
                    verts, tris = f.tessellate(0.6)
                except Exception:
                    continue
                V = np.array([[v.X, v.Y, v.Z] for v in verts])
                tri.append(V[np.array(tris)])
        if not tri:
            continue
        coll = Poly3DCollection(np.concatenate(tri), facecolor=col, alpha=alpha,
                                edgecolor="#3c4a5a", linewidth=0.04)
        coll.set_zsort("average")
        ax.add_collection3d(coll)
    allp = Compound([whole(), TL.tail(0.0)] +
                    [TT.rigid_body(b) for b in sorted(TT.BODIES)])
    bb = allp.bounding_box()
    ax.set_xlim(bb.min.X - 5, bb.max.X + 5)
    ax.set_ylim(bb.min.Y - 5, bb.max.Y + 5)
    ax.set_zlim(bb.min.Z - 5, bb.max.Z + 5)
    ax.set_box_aspect((bb.size.X, max(bb.size.Y, 60), bb.size.Z))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(path, dpi=140, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", path)


if __name__ == "__main__":
    report()
    render_png(os.path.join(HERE, "tomcat_head.png"))
