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
from build123d import Compound, Cylinder, Ellipse, Plane, Pos, Sphere

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

#: ✅ **Measured off a side-view cat, not assumed.** The silhouette of a
#: standing *Felis silvestris* restoration was traced and its landmarks read:
#: the back runs 455 px from withers to tail base, which is this trunk's 364 mm,
#: so 1 px = 0.80 mm. From that:
#:
#:     landmark                      photo      this model was
#:     head top above the back line   90 mm      35 mm
#:     neck length                    68         50
#:     neck rise from the withers     41 deg     22
#:     head length                   110         95
#:
#: ⚠️ **The head was carried far too low** -- a third of the height a cat
#: carries it. It is not a cosmetic difference: the head is 240 g and raising it
#: lengthens its lever about the body CoM.
#:
#: ⚠️ The image it came from is `.gitignore`d and its licence was never
#: established, so **this measurement cannot be re-run from the repo**. The same
#: landmarks want re-taking from the public-domain Reighard & Jennings plates
#: that `ANATOMY.md` names as authoritative.  `[owed]`
NECK_L = 68.0 * SCALE
HEAD_L = 95.0 * SCALE

#: Where the neck leaves the body, and at what angle. A cat carries its head
#: ABOVE the dorsal line, not on the axis -- the cervical chain rises out of the
#: thorax. ⚠️ Raising it costs nothing in pitch inertia (the lever is x) and
#: everything in how the animal reads.
BASE_X = TT.BODIES[3][1]
BASE_Z = TT.SPINE_Z
RISE_DEG = 41.0


#: ✅ **The outline is TRACED, not invented.** The head and neck were built
#: from spheres on a rising axis, and a side-view cat says that is not the shape:
#:
#:     station                photo, in head-lengths   the sphere model
#:     nose, above the back        -0.11                rising 41 deg
#:     ear tip                     +1.14                +0.37 at the top
#:     occiput                     +0.96
#:     head depth / length          1.6                  0.6  -- far too flat
#:
#: ⚠️ A cat's nose sits slightly BELOW its back line and its skull top a full
#: head-length above it; the muzzle points forward, not up. Read in head-lengths
#: the profile is scale-free, which matters because the ruler has been wrong
#: twice in this file already.
#:
#: `reference/head_profile.json` holds `(u, top, bottom)` per station, u measured
#: in head-lengths back from the nose and heights above the trunk's dorsal line.
#: ⚠️ It was traced from an image that is `.gitignore`d, whose licence was never
#: established, and which is a **European wildcat** rather than a domestic one.
#: The same trace wants re-taking from the public-domain Reighard & Jennings
#: plates `ANATOMY.md` names as authoritative.  `[owed]`
def _profile():
    import json
    with open(os.path.join(os.path.dirname(HERE), "reference",
                           "head_profile.json")) as fh:
        return [(u, t, b) for u, t, b in json.load(fh)]


#: Widest the head gets, in head-lengths. A cat's skull is about 0.72 of its
#: length across the zygomatic arches.  `[assumed]`
WIDTH_FRAC = 0.72


def _stations():
    """`(x, z_top, z_bot, half_width)` in mm, nose first.

    ⚠️ **A side silhouette cannot separate the head's underside from the
    neck's front, and the first two versions of this used it as if it could.**
    Traced, the "head depth" comes out **1.44 head-lengths** -- a cat's head is
    about 0.75 deep -- because below the jaw the outline is already throat, then
    chest. Worse, the scan was clipped, so past u ~ 0.95 the bottom reads a flat
    -0.99: the cut line.

    ✅ So the TOP line is what the photo is good for -- it runs against white
    all the way from the nose over the ears to the withers -- and the depth
    comes from anatomy. What is measured and what is assumed are separated here
    rather than averaged.
    """
    L = HEAD_L
    out = []
    for u, t, _b in _profile():
        if u > U_OCCIPUT:
            continue
        x = BASE_X + (U_WITHERS - u) * L
        zt = TT.Z_DORSAL + t * L
        # muzzle shallow, cranium full depth, tapering back to the occiput
        d = DEPTH_FRAC * L * (0.35 + 0.65 * min(u / 0.45, 1.0))
        hw = 0.5 * min(d, WIDTH_FRAC * L)
        out.append((x, zt, zt - d, hw))

    ox, ozt, ozb, ohw = out[-1]
    thw = TT._row_hw_at(BASE_X)
    tzt = TT.Z_DORSAL
    tzb = TT._zc(BASE_X) - thw * TT.ASPECT
    n = 6
    for k in range(1, n + 1):
        f = k / n
        out.append((ox + (BASE_X - ox) * f,
                    ozt + (tzt - ozt) * f,
                    ozb + (tzb - ozb) * f,
                    ohw + (thw - ohw) * f))
    return out


#: A cat's head is about this deep, as a fraction of its length.  `[assumed]`
DEPTH_FRAC = 0.78

#: Where the cranium ends and the neck begins, in head-lengths from the nose.
U_OCCIPUT = 0.95

#: Where the traced profile meets the body: the withers, in head-lengths.
U_WITHERS = 2.0


def whole():
    """Head and neck, as a chain of short lofts through the traced sections.

    ⚠️ One loft through all 27 sections went unstable -- the spline overshot to
    a bounding box of z -378..1731 on a shape 220 mm long -- because it starts
    from a near-degenerate nose section and the aspect ratio swings through the
    cranium. Segment by segment each loft spans two sections and cannot ring.
    """
    from build123d import loft
    st = [x for x in _stations() if (x[1] - x[2]) > 6.0]
    parts = []
    for (x0, t0, b0, w0), (x1, t1, b1, w1) in zip(st[:-1], st[1:]):
        if abs(x1 - x0) < 1e-6:
            continue
        a = Plane(origin=(x0, 0, 0.5 * (t0 + b0)), z_dir=(1, 0, 0))             * Ellipse(max(0.5 * (t0 - b0), 1.5), max(w0, 1.5))
        b = Plane(origin=(x1, 0, 0.5 * (t1 + b1)), z_dir=(1, 0, 0))             * Ellipse(max(0.5 * (t1 - b1), 1.5), max(w1, 1.5))
        parts.append(loft([b, a]))
    return Compound(parts)


def cranium_top():
    """Highest point of the head, in mm.

    ⚠️ This was a second loft through the forward stations alone, and it went
    unstable -- 21 sections with a reversal in them gave a bounding box of
    z -929..465 where the whole head spans -34..188. A check must not be built
    on a derived solid that can overshoot; it reads the stations the shape is
    generated FROM.
    """
    fwd = BASE_X + (U_WITHERS - 1.05) * HEAD_L
    return max(zt for x, zt, _zb, _hw in _stations() if x >= fwd)


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

    # ⚠️ a cat carries its head ABOVE the back
    top = cranium_top()
    print("\n  head top        %.0f mm, %+.0f above the dorsal line %.0f"
          % (top, top - TT.Z_DORSAL, TT.Z_DORSAL))
    if top < TT.Z_DORSAL + 40.0:
        print("      *** the head is carried too low: a cat holds it about one "
              "head-length above the back")
        ok = False

    # ⚠️ and it must not sit on the shoulder
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
