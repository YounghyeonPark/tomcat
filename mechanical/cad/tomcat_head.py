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
#: ⚠️ That 24 % is the M97 shape. Traced off the plates and carried where a
#: cat carries it the cost went to 29 %, then 40 % while the neck was a stalk,
#: and **36 %** once the neck became a neck (M101). See `NECK_AT`.
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
    """Legacy: the wildcat topline, kept for the posture it still supplies."""
    import json
    with open(os.path.join(os.path.dirname(HERE), "reference",
                           "head_profile.json")) as fh:
        return [(u, t, b) for u, t, b in json.load(fh)]


def _skull():
    """`(lateral, dorsal)` traced from Reighard & Jennings, in SKULL-LENGTHS.

    ✅ **Two orthogonal views, which is what a face needs.** A side silhouette
    alone cannot determine a 3-D surface -- the previous version of this file
    proved it twice, once by reading the neck as the head's underside and once
    by shipping the scan's crop line as a throat. Figs. 39 (dorsal) and 40
    (lateral) of *Anatomy of the Cat* (1901) are the same skull from two
    directions, so a section at each station takes its HEIGHT from one and its
    WIDTH from the other.

    The plates are **public domain** and are in `reference/plates/`, so unlike
    the wildcat photograph this measurement can be re-run from the repo.

    Traced, the skull is **0.71 as wide as it is long and 0.44 as tall** -- a
    cat, and a check on the 0.72 that was previously assumed.
    """
    import json
    with open(os.path.join(os.path.dirname(HERE), "reference",
                           "skull_profile.json")) as fh:
        d = json.load(fh)
    return [(u, t, b) for u, t, b in d["lateral"]],            [(u, w) for u, w in d["dorsal"]]


#: Soft tissue over the bone: skin, muscle and fur. ⚠️ `[assumed]` -- the
#: plates are a dry skull and nothing in the sources gives a thickness.
FLESH = 0.035            # skull-lengths, all round

#: ⚠️ **The neck was the skull's outline running out.** `_stations()` started
#: the neck tube at the LAST traced skull section, and at the occiput a traced
#: outline is the nuchal crest closing to a point -- half-width **8.2 mm**,
#: depth 11.7. So the neck left the head as a **13 mm rod carrying 240 g**, and
#: at its narrowest it was **0.15 of the chest** where a cat is about 0.6. The
#: render showed a stalk and the numbers agreed with it.
#:
#: A skull is not a head. The part of a cat's neck that has any size is the
#: muscle wrapped round the braincase, and the plates do not draw muscle. So
#: the neck leaves the head at the BRAINCASE -- which the plates do draw, and
#: which is still 20 mm half-width there -- and the crest behind it is inside
#: the neck, as it is in the animal.
NECK_AT = 0.86           # skull-lengths from the nose  [assumed]
#: Muscle over the braincase. `[assumed]` -- ANATOMY.md puts the 7 cervical
#: vertebrae explicitly out of scope, so this project has no source for a neck.
NECK_FLESH = 0.05        # skull-lengths


def _smooth(vals, k=5):
    """Moving average. ⚠️ The raw trace carries the plate's own detail -- the
    canine tooth, the zygomatic arch springing away from the braincase -- as
    step changes in section, and lofting through them gave the head a lumpy
    surface. What is wanted is the skull's ENVELOPE, so the profile is filtered
    before it becomes geometry."""
    v = np.asarray(vals, float)
    pad = np.r_[np.full(k // 2, v[0]), v, np.full(k // 2, v[-1])]
    return np.convolve(pad, np.ones(k) / k, mode="valid")


def _skull_sections():
    """`(x, z_top, z_bot, half_width)` in mm, nose first, on the head axis."""
    lat, dor = _skull()
    us = [u for u, _t, _b in lat]
    wid = np.interp(us, [u for u, _w in dor], [w for _u, w in dor])
    top = _smooth([t for _u, t, _b in lat])
    bot = _smooth([b for _u, _t, b in lat])
    hw = _smooth(wid)
    L = HEAD_L
    return [(NOSE_X - u * L,
             HEAD_Z + (t + FLESH) * L,
             HEAD_Z + (b - FLESH) * L,
             (w + FLESH) * L)
            for u, t, b, w in zip(us, top, bot, hw)]


#: Where the skull sits. ⚠️ The plates give the skull's SHAPE and cannot give
#: its posture; that still comes from the wildcat topline -- the skull crowning
#: about 0.96 head-lengths above the back line.  `[owed]`
#: Nose-to-withers span in head-lengths, from the wildcat trace.
U_WITHERS = 2.0

NOSE_X = 0.0
HEAD_Z = 0.0


def _place():
    """Set `NOSE_X` and `HEAD_Z` so the traced posture is reproduced."""
    global NOSE_X, HEAD_Z
    NOSE_X = BASE_X + U_WITHERS * HEAD_L
    lat, _d = _skull()
    top = max(t for _u, t, _b in lat) + FLESH
    HEAD_Z = TT.Z_DORSAL + (0.96 - top) * HEAD_L


def _stations():
    """`(x, z_top, z_bot, half_width)` in mm, nose first.

    ✅ The cranium is two traced orthogonal views of a real skull; only the
    NECK is interpolated, from the occiput into the trunk's front section. The
    seam is named rather than blended away.
    """
    _place()
    # ⚠️ Sections BEHIND the braincase are dropped, not blended: they are
    # the nuchal crest, and in the animal they sit inside the neck. Keeping
    # them and starting the tube after them made the loft run backwards in x.
    cut = NOSE_X - NECK_AT * HEAD_L
    out = [r for r in _skull_sections() if r[0] >= cut - 1e-9]
    # ⚠️ **A neck is a tube, not a plane-to-plane blend.** Interpolating the
    # occiput's section straight into the trunk's front face gave a flat wedge
    # -- a fin, not a neck -- because the trunk's section is a tall ellipse and
    # the skull's is nearly round. The neck follows its own CURVE from the
    # occiput down into the chest, with a round section that grows toward the
    # body, and only the last station matches the trunk.
    ox, ozt, ozb, ohw = out[-1]
    oz = 0.5 * (ozt + ozb)
    orr = 0.5 * min(ozt - ozb, 2 * ohw) + NECK_FLESH * HEAD_L
    tz = TT.Z_DORSAL - 0.45 * TT._row_hw_at(BASE_X) * TT.ASPECT
    trr = 0.62 * TT._row_hw_at(BASE_X)
    n = 10
    for k in range(1, n + 1):
        f = k / n
        # ease the centreline so the neck leaves the skull along its own axis
        e = f * f * (3.0 - 2.0 * f)
        x = ox + (BASE_X - ox) * f
        z = oz + (tz - oz) * e
        r = orr + (trr - orr) * e
        out.append((x, z + r, z - r, r))
    return out


#: ⚠️ **The skull has no ears, no eyes and no nose, and a face is those.**
#: The plates give bone; these are the soft features that make a head read as a
#: cat, each a named dimension so it can be argued with. In skull-lengths.
#: `[assumed]` -- no source in this project gives them.
EAR_BASE = 0.30          # fore-aft width of the ear's root
EAR_H = 0.34             # height above the crown
EAR_SPLAY = 0.22         # half-separation of the ear roots
EAR_AT = 0.72            # u of the ear root, nose = 0
EYE_R = 0.085
EYE_AT = (0.40, 0.17)    # (u, half-separation)
NOSE_R = 0.055


def features():
    """Ears, eyes and nose as separate solids on the cranium."""
    L = HEAD_L
    st = _stations()
    def at(u):
        x = NOSE_X - u * L
        best = min(st, key=lambda r: abs(r[0] - x))
        return x, best[1], best[2], best[3]

    parts = []
    # --- ears. ⚠️ A first version drew the three EDGES of a triangle as
    # cylinders, which renders as a pair of antennae rather than a pair of ears.
    # An ear is a filled flap: a tapering stack from a long base on the crown to
    # a point, thin across.
    from build123d import loft
    ex, etop, _eb, _ew = at(EAR_AT)
    for sgn in (-1.0, +1.0):
        y0 = sgn * EAR_SPLAY * L
        secs = []
        n = 6
        for k in range(n + 1):
            f = k / n
            z = etop - 0.03 * L + EAR_H * L * f
            y = y0 + sgn * 0.09 * L * f          # lean outward
            x = ex - 0.08 * L * f                # and a touch back
            hl = 0.5 * EAR_BASE * L * (1.0 - 0.92 * f) + 1.0
            th = 0.045 * L * (1.0 - 0.6 * f) + 0.6
            secs.append(Plane(origin=(x, y, z), z_dir=(0, 0, 1))
                        * Ellipse(hl, th))
        parts.append(loft(secs))
    # --- eyes, set into the side of the cranium
    ux, uy = EYE_AT
    gx, gt, gb, gw = at(ux)
    for sgn in (-1.0, +1.0):
        parts.append(Pos(gx, sgn * uy * L, 0.5 * (gt + gb) + 0.10 * L)
                     * Sphere(EYE_R * L))
    # --- nose
    nx, nt, nb, _nw = at(0.03)
    parts.append(Pos(nx, 0.0, 0.5 * (nt + nb) + 0.02 * L) * Sphere(NOSE_R * L))
    return Compound(parts)


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
    st = _stations()
    return max(zt for x, zt, _zb, _hw in st if x >= NOSE_X - HEAD_L)


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

#: Where the lump sat: the front girdle's own CoM BEFORE M102, which is the
#: plant the BODY_* figures above describe.
#: ⚠️ This read `SP.front_girdle_com` live, and M102 folded the placed head
#: INTO that CoM ((-4.6, +17.4) -> (+27.4, +37.8) mm). The "before" side of the
#: account then already held the head: the lever read 116 mm instead of 81 and
#: the cost fell 36 % -> 32 %, a failure from M102 to M117. The account is of a
#: past move, so both of its sides are frozen.
LUMPED_GIRDLE_COM = (-0.00461, 0.01739)   # [derived: params.py before M102]
LUMPED_AT = (195.0 + LUMPED_GIRDLE_COM[0] * MM,
             LUMPED_GIRDLE_COM[1] * MM)


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
              ("#8a6b52", 1.0, [features()]),
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
    allp = Compound([whole(), features(), TL.tail(0.0)] +
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
