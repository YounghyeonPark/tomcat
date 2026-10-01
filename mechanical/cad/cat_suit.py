# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The cat's skin: one knit suit over the whole skeleton (ADR-0119).

`tomcat_skin` (ADR-0094) sized a cover for the TRUNK from the strain the spine
puts into it -- a tube, seamed on the upper flank, with a slack belly. This is
the rest of the cat: the same knit carried over the neck, down four legs and
along the tail, in a cat's shape rather than the structure's.

✅ **Two fields, and the suit is their smooth union:**

- **the cat**: a torso lofted through a cat's sections -- a deep chest between
  the forelegs, a tucked waist, round haunches over the hind hips -- with a
  neck, a sleeve per leg (thigh to paw, tapering) and a tail sleeve;
- **the structure**: every surface of the skeleton, drive and harness, pushed
  out by `OFFSET`. It is what guarantees the cover: wherever the mechanism is
  bigger than a cat -- the 36/34/22 mm sheaves, the hollow hip's hub, the
  Bowden conduits -- the suit goes round it rather than through it.

The suit is open at the soles (the pads touch the ground).

✅ **M129 (ADR-0120): soft, round and plump.** The knit is laminated to a soft
foam (`PAD`), so the suit stands further off the structure, its blends are
wider, and the paws are round. And the suit now covers the HEAD: the skull
`tomcat_head` traced is a dry skull with no lower jaw, so the face read as a
beak. The foam gives it what bone does not -- a round crown, cheeks, two
whisker pads and a chin -- and the eyes and nose sit on that surface.

⚠️ **What the stance pose proves, and why it is enough.** Every part on a leg
is fixed to one link, and every free cable run lies between two circles fixed
to one link (the cable leaves each sheave on a tangent), so a sleeve that
clears its link's parts at the stance clears them at every pose. What changes
with pose is the fabric at the JOINTS, and that is `joint_strain`.

⚠️ It is a cover, not a structure: it carries no load. Its mass is reported
and NOT yet in `params.py` -- M130 (the spools) re-baselines the masses anyway.  `[owed]`
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "kinematics", "src"))

import tomcat_trunk as TT                                        # noqa: E402
import tomcat_skin as SK                                         # noqa: E402
import tomcat_head as HD                                         # noqa: E402
import tomcat_tail as TL                                         # noqa: E402
import tomcat_leg_detail as LD                                   # noqa: E402
from tomcat_kin import LegModel                                  # noqa: E402
from tomcat_kin.params import DEFAULT_FORELEG, DEFAULT_HINDLEG   # noqa: E402

#: Grid spacing of the field, mm.
VOXEL = 1.25
#: How far the suit stands off every structural surface, at least, mm. The
#: same 2.0 mm `tomcat_skin.CLEAR` holds, plus a voxel's worth of meshing error.
OFFSET = 2.0 + VOXEL
#: The clearance the checks hold the finished surface to, mm.
CLEAR = SK.CLEAR
#: ✅ M129: the soft layer under the knit, mm -- an open-cell polyether foam,
#: the kind a plush toy is stuffed with. It makes the cat round, and it is
#: what the face is made of.  `[assumed]`
PAD = 6.0
#: The deepest the foam is laid, mm: the face's cheeks and chin, the chest.
FOAM_MAX = 25.0
#: How much the structure's offset is blurred before the foam follows it, mm.
SMOOTH = 4.0
#: Foam densities priced, kg/m^3: the soft PU foam chosen, and a silicone foam
#: for what a squishier, heavier skin would cost.  `[assumed]`
FOAM_RHO = {"PU foam": 30.0, "silicone foam": 250.0}
#: Blend radii, mm: how softly the torso, the legs and the structure merge.
#: M129: wider (18 / 9 / 5 before) -- rounder.
K_BODY, K_LEG, K_STRUCT = 24.0, 13.0, 8.0
#: Areal mass of the knit, g/m^2 -- a nylon-elastane power mesh, ~1 mm.
#: `[assumed]`: 200-300 g/m^2 is the range such knits are sold in.
#: ⚠️ ADR-0094's 130 g priced the trunk cover as SOLID nylon 1 mm thick
#: (1150 g/m^2); a knit is mostly air.
AREAL_G_M2 = 250.0
#: Strain a knit panel carries without help. ADR-0094 read 12.6 % as inside
#: what a knit does and 34.7 % as not; this is the line between.  `[assumed]`
KNIT_WORKING = 0.15

LEGS = (("LF", True, +1.0), ("RF", True, -1.0), ("LR", False, +1.0), ("RR", False, -1.0))
HIP_X = {True: TT.FRONT_HIP_X, False: TT.REAR_HIP_X}

NECK_DIR = np.array([math.cos(math.radians(HD.RISE_DEG)), 0.0,
                     math.sin(math.radians(HD.RISE_DEG))])
NECK_BASE = np.array([HD.BASE_X, 0.0, HD.BASE_Z])

#: Where the head's mass model puts the body's CoM and its pitch inertia, for
#: pricing the foam's lever (`tomcat_tail`'s figures).
BODY_COM = (109.0, 15.0)
BODY_IYY = TL.BODY_IYY


def leg_points(fore):
    """Hip, knee, ankle, paw base, paw tip at the stance, in the leg plane (x, z) mm."""
    lg = DEFAULT_FORELEG if fore else DEFAULT_HINDLEG
    lm = LegModel(lg)
    q = lm.inverse((LD.FOOT_X, LD.FOOT_Z, LD.FOOT_PITCH))
    return lm.joint_positions(q) * 1000.0, np.asarray(q, float)


#: The pads: the sole is cut this far above the paw's base line, so they show.
SOLE_Z = float(leg_points(False)[0][3][1]) + 1.5


# --------------------------------------------------------------------- shape
#: The torso's sections, `(x, half-width, top, bottom)` mm, for a cat: the
#: chest is deepest between the forelegs, the waist tucks up behind the
#: ribs, the haunches are wide. Each is a LOWER bound -- `section()` never
#: lets the cat inside `tomcat_skin.outline`, which clears the structure.
#: M129: plumper all round -- 5-6 mm wider, 4-6 higher and deeper, and the
#: waist tucked less: a soft cat, not a lean one.
CAT = (
    (-158.0, 8.0, 66.0, 44.0),   # the rump's end, rounding into the tail
    (-134.0, 44.0, 90.0, -2.0),
    (-95.0, 60.0, 96.0, -46.0),  # the haunches
    (-55.0, 60.0, 97.0, -52.0),
    (-15.0, 54.0, 95.0, -46.0),
    (35.0, 50.0, 93.0, -42.0),   # the loin
    (90.0, 47.0, 92.0, -36.0),   # the waist, tucked a little
    (140.0, 52.0, 93.0, -54.0),
    (185.0, 58.0, 94.0, -74.0),  # the chest, deepest between the forelegs
    (230.0, 57.0, 93.0, -78.0),
    (262.0, 50.0, 91.0, -62.0),
    (298.0, 26.0, 84.0, -14.0),  # the chest's front, under the neck
)


def section(x):
    """`(half_width, top, bottom)` of the cat at station x (vectorised)."""
    cx, ca, ct, cb = (np.array(c) for c in zip(*CAT))
    a, top, bot = np.interp(x, cx, ca), np.interp(x, cx, ct), np.interp(x, cx, cb)
    inside = (x >= TT.BODIES[0][0]) & (x <= TT.BODIES[3][1])
    xs = np.clip(x, TT.BODIES[0][0], TT.BODIES[3][1])
    flo = np.vectorize(lambda v: SK.outline(float(v)), otypes=[float, float, float])
    if np.ndim(x) == 0:
        hw, t, b = SK.outline(float(xs))
    else:
        u = np.unique(xs)
        hw_u, t_u, b_u = flo(u)
        hw, t, b = (np.interp(xs, u, v) for v in (hw_u, t_u, b_u))
    a = np.where(inside, np.maximum(a, hw + 1.0), a)
    top = np.where(inside, np.maximum(top, t + 1.0), top)
    bot = np.where(inside, np.minimum(bot, b - 1.0), bot)
    return a, top, bot


#: Each link's sleeve: `(r at proximal, r at distal, lateral centre, lateral
#: half-width)`, mm, for a LEFT leg (y > 0).
#:
#: ⚠️ **The mechanism is bigger than a cat's leg, and the sleeve says so.** A
#: first pass drew a cat's limb (a 24 mm thigh tapering to 15) and it vanished
#: inside the structure's own offset: the sheaves are 36 / 34 / 22 mm in
#: radius and every cable runs on their rims, so between two joints the cables
#: stand 30-35 mm off the bone. The sleeve is therefore the HULL of the two
#: sheaves it spans, flattened to the leg's lateral stack (y 42-77 mm on the
#: thigh and shank, measured): a broad, flat thigh -- which a cat has.
SLEEVE = (
    (36.0 + OFFSET + PAD, 34.0 + OFFSET + PAD, 59.5, 20.0 + PAD),  # thigh: hip to knee sheave
    (34.0 + OFFSET + PAD, 22.0 + OFFSET + PAD, 60.5, 19.0 + PAD),  # shank: knee to ankle
    (22.0 + OFFSET + PAD, 9.0 + PAD, 53.0, 10.0 + PAD),            # metatarsus: ankle to paw
    (9.0 + PAD, 7.5 + PAD, 53.0, 11.0 + PAD),                      # paw
)

#: ✅ M129: the face the skull does not have, `(name, centre, radii)` mm in
#: trunk coordinates, mirrored for the paired ones. The traced skull is a dry
#: skull: its underside runs flat at z 118-128 from the nose back, with no
#: mandible, no cheek and no muzzle pad, so a cover drawn over it is a beak.
#: ⚠️ M129's first face was a ball -- crown, cheeks and muzzle all round, ears
#: small -- and it read as a cartoon dinosaur, not a cat. A cat's face is a
#: WEDGE: the width is at the cheekbones under the eyes, and it narrows to a
#: small muzzle and a small chin; the ears are what make the head's outline.
#: Proportions from the owner's cat, read off a frontal photograph (not in the
#: repo): ears 0.55 of the face's width tall on a base 0.45 of it, eyes 0.38
#: of it apart, the muzzle 0.35 of it wide.  `[assumed]`
FACE = (
    ("crown", (391.0, 0.0, 154.0), (40.0, 41.0, 28.0)),
    ("cheek", (403.0, 24.0, 145.0), (21.0, 18.0, 15.0)),
    ("whisker pad", (438.0, 8.0, 131.0), (9.5, 9.0, 8.5)),
    ("chin", (428.0, 0.0, 122.0), (15.0, 11.0, 9.0)),
    ("muzzle", (432.0, 0.0, 135.0), (16.0, 15.0, 12.0)),
)
#: Blend radius of the face, mm.
K_FACE = 9.0
#: The ears, `(base, tip, base radius, tip radius)` mm, thin fore-and-aft by
#: `EAR_THIN`, and CUPPED: a cat's ear is a scoop open to the front, not a
#: plate. 52 mm tall on a 42 mm base, leaning out 21 deg -- the owner's
#: cat's, scaled to this face.  `[assumed]`
EAR = ((385.0, 25.0, 176.0), (379.0, 45.0, 229.0), 21.0, 4.5)
EAR_THIN = 3.0
#: How deep the front of the ear is scooped, mm.
EAR_CUP = 2.6
#: How far each ear's opening is turned outward from straight ahead, deg.
EAR_TURN = 30.0


def _ear(P, sgn):
    """One ear, as a flattened cone with its front scooped out."""
    base, tip, rb, rt = EAR
    m = np.array([1.0, sgn, 1.0])
    b, t = np.asarray(base) * m, np.asarray(tip) * m

    # the ear is thin along the way its opening faces
    ph = math.radians(EAR_TURN)
    n = np.array([math.cos(ph), sgn * math.sin(ph), 0.0])

    def squash(X, c0, thin):
        return X + (thin - 1.0) * (((X - c0) @ n)[..., None]) * n

    def flat(c0, r0, r1, thin):
        tq = squash(t + (c0 - b), c0, thin)
        return _round_cone(squash(P, c0, thin), c0, tq, r0, r1) / thin

    outer = flat(b, rb, rt, EAR_THIN)
    cup = flat(b + (EAR_CUP + 1.0) * n + np.array([0.0, 0.0, 6.0]), rb - 4.0,
               max(rt - 2.5, 1.0), EAR_THIN)
    return np.maximum(outer, -cup)


def face_field(P):
    f = None
    for _nm, c, r in FACE:
        c = np.asarray(c, float)
        for sgn in ((-1.0, 1.0) if c[1] else (1.0,)):
            d = _ellipsoid(P, c * np.array([1.0, sgn, 1.0]), np.asarray(r, float))
            f = d if f is None else _smin(f, d, K_FACE)
    for sgn in (-1.0, 1.0):
        f = _smin(f, _ear(P, sgn), 4.0)
    return f


def sleeves(fore):
    """The cat's limb in the leg plane: `[(p0, p1, r0, r1, yc, hy)]`, leg-plane
    (x, z) mm, hip outward, one per link."""
    p, _q = leg_points(fore)
    return [(p[i], p[i + 1], *SLEEVE[i]) for i in range(4)]


# ------------------------------------------------------------------- fields
def _smin(a, b, k):
    """Polynomial smooth minimum: `min` with the corner rounded over k."""
    h = np.clip(0.5 + 0.5 * (b - a) / k, 0.0, 1.0)
    return b * (1 - h) + a * h - k * h * (1 - h)


def _round_cone(P, a, b, ra, rb):
    """Approximate distance to a cone of radius ra at a, rb at b, capped round."""
    d = b - a
    L2 = float(d @ d)
    t = np.clip(((P - a) @ d) / L2, 0.0, 1.0)
    q = P - (a + t[..., None] * d)
    return np.linalg.norm(q, axis=-1) - (ra + (rb - ra) * t)


def _ellipsoid(P, c, r):
    k0 = np.linalg.norm((P - c) / r, axis=-1)
    return (k0 - 1.0) * min(r)


def cat_field(P):
    """Signed (approximately distance) field of the cat's shape, mm; < 0 inside."""
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    a, top, bot = section(x)
    zc = 0.5 * (top + bot)
    hh = 0.5 * (top - bot)
    # a flatter-than-elliptic section (exponent 2.4): a cat's flank is full
    n = 2.4
    r = (np.abs(y / a) ** n + np.abs((z - zc) / hh) ** n) ** (1.0 / n)
    f = (r - 1.0) * np.minimum(a, hh)
    xr, xf = CAT[0][0], CAT[-1][0]
    f = np.maximum(f, np.maximum(xr - x, x - xf))
    # the neck, from the chest into the back of the head: a ruff, not a stalk
    nb = NECK_BASE + np.array([-10.0, 0.0, -2.0])
    # ⚠️ it ends BEHIND and below the face: carried into the head at 42 mm it
    # stood out of the jaw on both sides, wider than the cheeks
    ne = np.array([356.0, 0.0, 122.0])
    f = _smin(f, _round_cone(P, nb, ne, 50.0, 25.0), K_BODY)
    # the ruff at the throat, which a cat's neck disappears into
    f = _smin(f, _ellipsoid(P, np.array([292.0, 0.0, 62.0]), np.array([42.0, 40.0, 48.0])),
              K_BODY)
    # the face (M129)
    f = _smin(f, face_field(P), K_BODY)
    # the tail sleeve
    cur = np.asarray(TL.spine_curve(), float)
    pts3 = np.column_stack([cur[:, 0], np.zeros(len(cur)), cur[:, 1]])
    ts = np.linspace(0.0, 1.0, len(cur))
    tail = None
    for i in range(len(cur) - 1):
        ri = TL.radius_at(ts[i]) + 3.0 + PAD
        rj = TL.radius_at(ts[i + 1]) + 3.0 + PAD
        d = _round_cone(P, pts3[i], pts3[i + 1], ri, rj)
        tail = d if tail is None else np.minimum(tail, d)
    f = _smin(f, tail, 8.0)
    # the legs: haunch / shoulder, then a sleeve
    for _nm, fore, side in LEGS:
        hx = HIP_X[fore]
        yl = side * LD.TRACK_Y
        # ⚠️ The haunch and the shoulder are sized by the hip pair's Bowden
        # loops, not by anatomy: from the wall ports to the hub they arch out
        # to y 67 and up to z 67 within 35 mm of the hip (measured), and a
        # smaller haunch left them standing on the flank as a bracket.
        sh = _ellipsoid(P, np.array([hx + (4.0 if fore else 0.0), side * 48.0, 20.0]),
                        np.array([58.0 + PAD, 34.0 + PAD, 68.0 + PAD]))
        f = _smin(f, sh, K_BODY)
        # the ankle (hock / wrist): its sheave stack is 32 mm across (y 45-77)
        # where the metatarsus is 12, so it gets its own flattened knuckle
        pa, _q = leg_points(fore)
        ank = _ellipsoid(P, np.array([hx + pa[2][0], side * 61.0, pa[2][1]]),
                         np.array([27.0 + PAD, 21.5 + PAD, 27.0 + PAD]))
        f = _smin(f, ank, K_LEG)
        # M129: a round paw -- a cat stands on a soft ball, not a blade; the
        # sole cut leaves its pad bare
        paw = _ellipsoid(P, np.array([hx + 0.5 * (pa[3][0] + pa[4][0]), side * 53.0,
                                      pa[3][1] + 3.0]), np.array([22.0, 15.0, 13.0]))
        f = _smin(f, paw, K_LEG)
        leg = None
        for p0, p1, r0, r1, yc, hy in sleeves(fore):
            # flattened: the lateral axis scaled so the cone's mean radius
            # spans the leg's lateral half-width
            sc = 0.5 * (r0 + r1) / hy
            Q = P.copy()
            Q[..., 1] = (P[..., 1] - side * yc) * sc
            a3 = np.array([hx + p0[0], 0.0, p0[1]])
            b3 = np.array([hx + p1[0], 0.0, p1[1]])
            d = _round_cone(Q, a3, b3, r0, r1) / max(sc, 1.0)
            leg = d if leg is None else _smin(leg, d, K_LEG)
        f = _smin(f, leg, K_LEG)
    return f


def structure_solids():
    """Everything the suit must cover: trunk, four legs with their drive, the
    head and neck (`tomcat_head`'s solid, all of it since M129), the tail."""
    import tomcat_assembly as A
    bodies, legs, _soft = A.assembly(soft=False)
    return bodies + legs + [HD.whole(), TL.tail()]


def structure_points(cache=None, tol=0.8, spacing=0.6):
    """Surface samples of `structure_solids`, dense enough for `VOXEL`.

    ⚠️ The cache is stale the moment any geometry moves, and a stale one
    would fit the suit to a skeleton that no longer exists -- silently. It is
    used only while it is newer than every CAD script and `params.py`."""
    if cache and os.path.exists(cache):
        # (this file draws the suit, not the structure: its own edits do not
        # stale the samples)
        src = [os.path.join(HERE, f) for f in os.listdir(HERE)
               if f.endswith(".py") and f != os.path.basename(__file__)]
        src.append(os.path.join(os.path.dirname(HERE), "..", "kinematics", "src",
                                "tomcat_kin", "params.py"))
        if os.path.getmtime(cache) > max(os.path.getmtime(f) for f in src):
            return np.load(cache)["p"]
    rng = np.random.default_rng(0)
    out = []
    for g in structure_solids():
        for f in g.faces():
            try:
                verts, tris = f.tessellate(tol)
            except Exception:
                continue
            V = np.array([[v.X, v.Y, v.Z] for v in verts])
            T = V[np.array(tris)]
            out.append(V)
            area = 0.5 * np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1)
            n = np.ceil(area / spacing ** 2).astype(int)
            big = n > 1
            if big.any():
                idx = np.repeat(np.nonzero(big)[0], n[big])
                u = rng.random((len(idx), 2))
                flip = u.sum(1) > 1
                u[flip] = 1 - u[flip]
                Tb = T[idx]
                out.append(Tb[:, 0] + u[:, :1] * (Tb[:, 1] - Tb[:, 0]) + u[:, 1:] * (Tb[:, 2] - Tb[:, 0]))
    p = np.concatenate(out).astype(np.float32)
    if cache:
        np.savez_compressed(cache, p=p)
    return p


def grid(points):
    lo = points.min(0) - 25.0
    hi = points.max(0) + 25.0
    # the foam ears stand above everything structural (M129)
    hi[2] = max(hi[2], EAR[1][2] + EAR[3] + 10.0)
    lo[2] = SOLE_Z - 3.0 * VOXEL
    n = np.ceil((hi - lo) / VOXEL).astype(int) + 1
    return lo, n


def field(points, stats=None):
    """The suit's field on the grid: `(F, lo, n)`; F < 0 is inside the suit.

    `stats`, a dict, receives the foam: where it is laid and what it weighs."""
    from scipy import ndimage
    lo, n = grid(points)
    # the structure, pushed out by OFFSET and the foam: distance to the
    # nearest sample
    occ = np.zeros(n, bool)
    ij = np.round((points - lo) / VOXEL).astype(int)
    ok = np.all((ij >= 0) & (ij < n), axis=1)
    occ[tuple(ij[ok].T)] = True
    dist = ndimage.distance_transform_edt(~occ).astype(np.float32) * VOXEL
    del occ
    # M129: the offset is BLURRED before it is cut, so the foam reads as one
    # soft surface rather than the process tips and the sheave rims showing
    # through it. Blurring a distance field pulls it in round sharp features,
    # and the clearance check measures what that costs.
    F = ndimage.gaussian_filter(dist, SMOOTH / VOXEL) - (OFFSET + PAD)
    # the cat, one x-slab at a time
    ys = lo[1] + VOXEL * np.arange(n[1])
    zs = lo[2] + VOXEL * np.arange(n[2])
    Y, Z = np.meshgrid(ys, zs, indexing="ij")
    for i in range(n[0]):
        x = lo[0] + VOXEL * i
        P = np.stack([np.full_like(Y, x), Y, Z], axis=-1)
        c = cat_field(P).astype(np.float32)
        F[i] = _smin(c, F[i], K_STRUCT)
        # open at the soles
        F[i] = np.maximum(F[i], (SOLE_Z - Z).astype(np.float32))
    # nothing hollow inside: a void enclosed by the suit is suit
    solid = F < 0
    filled = ndimage.binary_fill_holes(solid)
    F[filled & ~solid] = -VOXEL
    if stats is not None:
        stats["dist"] = dist
    return F, lo, n


def foam(pd, dist, lo):
    """The soft layer, measured from the knit inward: at every point of the
    suit, how far in the foam goes before it meets the structure (within
    `CLEAR` of a structural surface) or `FOAM_MAX`. Volume, and its pitch
    inertia about the body's CoM -- the head's part apart, because the head is
    on the longest lever there is.

    ⚠️ A volume count (inside the suit and outside the structure) was the
    first version, and it read 6.8 L: the trunk's rigid bodies are open
    shells, so their insides were counted as foam."""
    from scipy import ndimage
    from vtk.util import numpy_support as ns
    P, T = mesh_arrays(pd)
    N = ns.vtk_to_numpy(pd.GetPointData().GetNormals()).astype(float)
    A = P[T]
    tri = 0.5 * np.linalg.norm(np.cross(A[:, 1] - A[:, 0], A[:, 2] - A[:, 0]), axis=1)
    area = np.zeros(len(P))
    for k in range(3):
        np.add.at(area, T[:, k], tri / 3.0)
    ts = np.arange(0.0, FOAM_MAX + 1e-9, 0.5)
    thick = np.empty(len(P))
    for i0 in range(0, len(P), 40000):
        p, n = P[i0:i0 + 40000], N[i0:i0 + 40000]
        s = p[:, None, :] - ts[None, :, None] * n[:, None, :]
        v = ndimage.map_coordinates(dist, ((s.reshape(-1, 3) - lo) / VOXEL).T,
                                    order=1, mode="nearest").reshape(len(p), len(ts))
        hit = v <= CLEAR
        k = np.where(hit.any(1), np.argmax(hit, 1), len(ts) - 1)
        thick[i0:i0 + 40000] = ts[k]
    head = P[:, 0] > HD.BASE_X + 0.5 * HD.NECK_L * NECK_DIR[0]
    r2k = (P[:, 0] - BODY_COM[0]) ** 2 + (P[:, 2] - BODY_COM[1]) ** 2
    out = {"knit_r2_mm4": float((area * r2k).sum())}
    # ✅ the design: a PADDED SHELL -- `PAD` of foam under the knit, with air
    # inside it, and only the face stuffed solid. Stuffing the whole cat
    # ("stuffed") is priced beside it.
    for nm, t in (("shell", np.where(head, thick, np.minimum(thick, PAD))),
                  ("stuffed", thick)):
        vol = area * t
        mid = P - 0.5 * t[:, None] * N
        r2 = (mid[:, 0] - BODY_COM[0]) ** 2 + (mid[:, 2] - BODY_COM[1]) ** 2
        out[nm] = {"foam_mm3": float(vol.sum()), "head_mm3": float(vol[head].sum()),
                   "r2_mm5": float((vol * r2).sum()),
                   "head_r2_mm5": float((vol * r2)[head].sum()),
                   "mean_mm": float(vol.sum() / area.sum())}
    return out


def suit_mesh(points=None, cache=None, out=None):
    """The suit's outer surface as `vtkPolyData`, open at the soles.

    `out`, a dict, receives the field (`F`, `lo`) and the foam's figures."""
    import vtk
    from vtk.util import numpy_support as ns
    if points is None:
        points = structure_points(cache)
    st = {} if out is None else out
    F, lo, n = field(points, stats=st)
    st["F"], st["lo"] = F, lo
    img = vtk.vtkImageData()
    img.SetDimensions(*[int(v) for v in n])
    img.SetSpacing(VOXEL, VOXEL, VOXEL)
    img.SetOrigin(*[float(v) for v in lo])
    arr = ns.numpy_to_vtk(np.ascontiguousarray(F.transpose(2, 1, 0)).ravel(), deep=True)
    img.GetPointData().SetScalars(arr)
    fe = vtk.vtkFlyingEdges3D()
    fe.SetInputData(img)
    fe.SetValue(0, 0.0)
    fe.ComputeNormalsOff()
    fe.Update()
    pd = fe.GetOutput()
    pd = _open_cuts(pd)
    con = vtk.vtkPolyDataConnectivityFilter()
    con.SetInputData(pd)
    con.SetExtractionModeToLargestRegion()
    con.Update()
    sm = vtk.vtkWindowedSincPolyDataFilter()
    sm.SetInputData(con.GetOutput())
    sm.SetNumberOfIterations(25)
    sm.SetPassBand(0.08)
    sm.BoundarySmoothingOff()
    sm.NonManifoldSmoothingOn()
    sm.NormalizeCoordinatesOn()
    sm.Update()
    cl = vtk.vtkCleanPolyData()
    cl.SetInputData(sm.GetOutput())
    cl.Update()
    nm = vtk.vtkPolyDataNormals()
    nm.SetInputData(cl.GetOutput())
    nm.SetFeatureAngle(60.0)
    nm.ConsistencyOn()
    nm.AutoOrientNormalsOn()
    nm.Update()
    return nm.GetOutput()


def _open_cuts(pd):
    """Drop the flat caps the sole plane leaves: the suit is OPEN there,
    which is what lets the pads touch the ground."""
    import vtk
    from vtk.util import numpy_support as ns
    P = ns.vtk_to_numpy(pd.GetPoints().GetData()).astype(float)
    polys = ns.vtk_to_numpy(pd.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
    C = P[polys].mean(axis=1)
    on = np.abs(C[:, 2] - SOLE_Z) < 0.35 * VOXEL
    keep = polys[~on]
    legacy = np.column_stack([np.full(len(keep), 3), keep]).astype(np.int64).ravel()
    cells = vtk.vtkCellArray()
    cells.SetCells(len(keep), ns.numpy_to_vtkIdTypeArray(legacy, deep=True))
    out = vtk.vtkPolyData()
    out.SetPoints(pd.GetPoints())
    out.SetPolys(cells)
    return out


# ------------------------------------------------------------------- checks
def mesh_arrays(pd):
    from vtk.util import numpy_support as ns
    P = ns.vtk_to_numpy(pd.GetPoints().GetData()).astype(float)
    T = ns.vtk_to_numpy(pd.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
    return P, T


def area_mm2(pd):
    P, T = mesh_arrays(pd)
    A = P[T]
    return float(0.5 * np.linalg.norm(np.cross(A[:, 1] - A[:, 0], A[:, 2] - A[:, 0]), axis=1).sum())


def clearance(pd, points):
    """`(min, where)`: the smallest distance from a covered structural point to
    the suit, mm, and whether every such point is INSIDE it."""
    import vtk
    from scipy.spatial import cKDTree
    covered = points[:, 2] > SOLE_Z + CLEAR
    Q = points[covered]
    P, _T = mesh_arrays(pd)
    d, i = cKDTree(P).query(Q, workers=-1)
    k = int(np.argmin(d))
    # inside: the suit is closed except at its cuts, so test with an
    # implicit distance -- the sign is the normal's side at the nearest vertex
    from vtk.util import numpy_support as ns
    N = ns.vtk_to_numpy(pd.GetPointData().GetNormals()).astype(float)
    side = np.einsum("ij,ij->i", Q - P[i], N[i])
    return float(d[k]), Q[k], int((side > 0).sum())


def joint_strain():
    """The fabric over each joint, through a trot and through the joint's ROM.

    A sleeve fibre on the outside of a joint, `R` from its axis, lengthens by
    `R * dtheta` as the joint turns; the fabric that shares it is the half of
    each link either side (`L`). A knit panel takes `KNIT_WORKING` of that
    and the rest is SLACK: gathered fabric that pays out, as at the belly.
    """
    from tomcat_kin import gait
    c = gait.GaitController(gait.trot_params())
    qs = {}
    for st in c.sample_cycle(64):
        for nm, ls in st.legs.items():
            if ls.q is not None:
                qs.setdefault(nm, []).append(np.asarray(ls.q, float))
    rows = []
    for nm, fore, _side in (("LF", True, 1), ("LR", False, 1)):
        lg = DEFAULT_FORELEG if fore else DEFAULT_HINDLEG
        Q = np.array(qs[nm])
        p, _q = leg_points(fore)
        L = np.linalg.norm(np.diff(p, axis=0), axis=1)
        # R: the sleeve at the joint, which is the sheave's rim plus OFFSET
        sl = sleeves(fore)
        for j, jn in enumerate(("hip", "knee", "ankle")):
            R = sl[j][2]
            span = 0.5 * L[j] + (0.5 * L[j - 1] if j > 0 else 45.0)
            d_trot = float(np.ptp(Q[:, j]))
            d_rom = float(lg.q_max[j] - lg.q_min[j])
            e = R * d_trot / span
            slack = max(0.0, R * d_rom - KNIT_WORKING * span)
            rows.append(("fore" if fore else "hind", jn, R, span,
                         math.degrees(d_trot), e, math.degrees(d_rom), slack))
    return rows


def torso_strain(pd):
    """ADR-0094's pitch strain on the SUIT's own dorsal and ventral lines."""
    P, _T = mesh_arrays(pd)
    span = TT.BODIES[3][1] - TT.BODIES[0][0]
    mid = np.abs(P[:, 1]) < 3.0
    body = mid & (P[:, 0] > TT.BODIES[0][0]) & (P[:, 0] < TT.BODIES[3][1])
    top, bot = P[body, 2].max(), P[body, 2].min()
    return [(nm, z, abs(z - SK.NEUTRAL_Z) * SK.PITCH_ROM / span)
            for nm, z in (("dorsal line", top), ("chest keel", bot))]


def report(cache=None, pd=None, points=None, st=None):
    ok = True
    if points is None:
        points = structure_points(cache)
    if pd is None:
        st = {}
        pd = suit_mesh(points, out=st)
    P, T = mesh_arrays(pd)
    print("CAT SUIT -- a plush knit over the whole skeleton (ADR-0119, ADR-0120)\n")
    bb = P.min(0), P.max(0)
    print("surface        %d triangles, nose-to-tail %.0f mm, %.0f wide, %.0f high"
          % (len(T), bb[1][0] - bb[0][0], bb[1][1] - bb[0][1], bb[1][2] - bb[0][2]))
    a = area_mm2(pd) * 1e-6
    print("area           %.3f m2 -> knit %.0f g at %.0f g/m2 ([assumed])"
          % (a, a * AREAL_G_M2, AREAL_G_M2))
    if st and "dist" in st:
        st["foam"] = foam(pd, st["dist"], st["lo"])
        Ik = AREAL_G_M2 * 1e-3 * st["foam"].pop("knit_r2_mm4") * 1e-12
        print("  the knit's pitch inertia +%.2e kg m2 = %.1f %% of the body's"
              % (Ik, 100 * Ik / BODY_IYY))
        for build, fs in st["foam"].items():
            fv, hv = fs["foam_mm3"] * 1e-3, fs["head_mm3"] * 1e-3
            print("foam, %-8s %5.0f cm3, %4.1f mm deep on average (%.0f cm3 in the "
                  "head and neck)" % (build, fv, fs["mean_mm"], hv))
            for nm, rho in FOAM_RHO.items():
                m = fv * rho * 1e-3
                I = rho * 1e-9 * fs["r2_mm5"] * 1e-6
                Ih = rho * 1e-9 * fs["head_r2_mm5"] * 1e-6
                print("  %-14s %4.0f kg/m3: %5.0f g (head %4.0f g), pitch inertia "
                      "+%.2e kg m2 = %5.1f %% of the body's (head %4.1f %%)"
                      % (nm, rho, m, hv * rho * 1e-3, I, 100 * I / BODY_IYY,
                         100 * Ih / BODY_IYY))
    dmin, where, outside = clearance(pd, points)
    print("clearance      %.2f mm to the structure, least at (%.0f, %.0f, %.0f)"
          % (dmin, *where))
    if dmin < CLEAR - 0.5 or outside:
        print("      *** the suit touches the structure (%d points outside)" % outside)
        ok = False
    print("\nthe fabric over each joint (R from the axis, shared over L)")
    print("%-5s %-6s %6s %6s %9s %8s %9s %9s" % ("leg", "joint", "R mm", "L mm",
                                                 "trot deg", "strain", "ROM deg", "slack mm"))
    for leg, jn, R, span, dt, e, dr, slack in joint_strain():
        flag = "" if e <= KNIT_WORKING else "  gathered"
        print("%-5s %-6s %6.1f %6.1f %9.1f %7.1f%% %9.0f %9.0f%s"
              % (leg, jn, R, span, dt, 100 * e, dr, slack, flag))
    print("\nthe torso in pitch, as ADR-0094 computed it, on the suit's own lines")
    for nm, z, e in torso_strain(pd):
        print("  %-12s z %6.1f   %5.1f %%" % (nm, z, 100 * e))
    print("\n  %s" % ("suit checks pass" if ok else "*** SEE ABOVE ***"))
    return ok


# --------------------------------------------------------------------- face
#: The face set on the foam: `(start inside, direction out)` mm, each feature
#: placed where its ray leaves the suit. The eyes are ALMONDS set into the
#: face looking forward -- an amber iris, a slit pupil, a dark rim -- and the
#: nose a small pink wedge with a dark edge: the owner's cat's.  `[assumed]`
EYE = ((404.0, 15.5, 150.0), (0.99, 0.08, 0.10))
#: Eye half-sizes, mm: (depth, width, height).
EYE_R = (4.5, 9.5, 7.2)
NOSE = ((430.0, 0.0, 139.0), (0.92, 0.0, 0.40))
NOSE_R = (3.5, 5.5, 3.8)


def _surface(F, lo, p0, d, reach=90.0):
    """Where the ray `p0 + t d` first leaves the suit (F crosses 0)."""
    from scipy import ndimage
    d = np.asarray(d, float) / np.linalg.norm(d)
    t = np.arange(0.0, reach, 0.2)
    pts = np.asarray(p0, float) + t[:, None] * d
    v = ndimage.map_coordinates(F, ((pts - lo) / VOXEL).T, order=1)
    k = int(np.argmax(v >= 0.0))
    return pts[k], d


def _normal(F, lo, p):
    """Outward normal of the suit at p: the field's gradient."""
    from scipy import ndimage
    h = VOXEL
    g = []
    for k in range(3):
        e = np.zeros(3)
        e[k] = h
        a = ndimage.map_coordinates(F, (((np.array([p + e, p - e]) - lo) / VOXEL).T), order=1)
        g.append(a[0] - a[1])
    g = np.array(g)
    return g / np.linalg.norm(g)


#: How far an eye stands proud of the fur, mm.
EYE_PROUD = 2.2


def face_features(F, lo):
    """`[(kind, centre, half-sizes, axis, tilt deg)]`: per eye a dark rim, the
    iris, the slit pupil and a catch-light; then the nose and its edge. Each
    is an ellipsoid whose first half-size lies along `axis`."""
    out = []
    p0, d0 = EYE
    dz, wy, hz = EYE_R
    for sgn in (-1.0, 1.0):
        m = np.array([1.0, sgn, 1.0])
        s, _ray = _surface(F, lo, np.asarray(p0) * m, np.asarray(d0) * m)
        # ⚠️ along the SURFACE's normal, not the ray: an almond 19 mm wide
        # set along a ray that meets a sloping face is buried but for a sliver
        d = _normal(F, lo, s)
        c = s - (dz - EYE_PROUD) * d
        tilt = -12.0 * sgn                      # the almond's outer corner up
        out.append(("rim", c - 0.6 * d, (dz, wy + 1.4, hz + 1.2), d, tilt))
        out.append(("iris", c, (dz, wy, hz), d, tilt))
        out.append(("pupil", c + 1.5 * d, (dz * 0.8, 1.7, hz * 0.85), d, 0.0))
        g = c + 0.55 * dz * d + np.array([0.0, -2.8 * sgn, 2.6])
        out.append(("glint", g, (1.3, 1.6, 1.6), d, 0.0))
    s, _ray = _surface(F, lo, *NOSE)
    d = _normal(F, lo, s)
    out.append(("nose rim", s - 0.6 * NOSE_R[0] * d, tuple(r + 0.8 for r in NOSE_R), d, 0.0))
    out.append(("nose", s - 0.35 * NOSE_R[0] * d, NOSE_R, d, 0.0))
    return out


# --------------------------------------------------------------------- coat
#: The owner's cat's coat: a brown-grey mackerel tabby with white -- a white
#: blaze up the nose, a white muzzle, chin, throat, chest and paws -- warm
#: orange round the eyes, pink inside the ears.  `[assumed]`
COAT = {
    "base": (0.50, 0.45, 0.40), "stripe": (0.22, 0.19, 0.17),
    "white": (0.95, 0.94, 0.92), "orange": (0.78, 0.52, 0.32),
    "pink": (0.93, 0.71, 0.68),
}


def _noise(P, scale):
    """A cheap smooth noise: a few incommensurate sines."""
    x, y, z = P[:, 0] / scale, P[:, 1] / scale, P[:, 2] / scale
    return (np.sin(1.7 * x + 2.3 * y + 0.5) * np.sin(1.3 * z - 0.9 * x + 1.1)
            + 0.5 * np.sin(3.1 * y + 2.9 * z + 0.3 * x))


def _mix(a, b, w):
    return a * (1 - w[:, None]) + np.asarray(b) * w[:, None]


def coat(P, N):
    """Per-vertex colour, 0..1, of the suit at points P with outward normals N."""
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    C = np.tile(np.asarray(COAT["base"], float), (len(P), 1))
    C *= (1.0 + 0.05 * _noise(P, 20.0))[:, None]        # mottling
    head = x > 330.0
    # mackerel stripes: across the body (dorsal to ventral), ringing the
    # legs and the tail, and the forehead's "M" lines on the crown
    leg = z < -40.0
    tail = x < TT.BODIES[0][0] - 10.0
    phase = np.where(tail, x / 17.0, np.where(leg, z / 16.0, x / 26.0))
    phase = phase + 0.15 * _noise(P, 30.0)
    crown = head & (z > 152.0) & (x < 412.0)
    phase = np.where(crown, y / 6.0 + 0.2 * _noise(P, 8.0), phase)
    band = np.clip((np.sin(2 * np.pi * phase) - 0.45) * 2.6, 0.0, 1.0)
    band *= np.clip((z + 70.0) / 60.0, 0.0, 1.0) * np.where(head & ~crown, 0.0, 1.0)
    C = _mix(C, COAT["stripe"], 0.80 * band)
    # a tabby is darker along the spine
    C = _mix(C, COAT["stripe"], 0.35 * np.clip((z - 70.0) / 40.0, 0.0, 1.0) * (~head))
    # the white: belly and chest underneath, throat and chin, the muzzle and
    # a blaze up the nose between the eyes, and the paws
    under = np.clip((-30.0 - z) / 16.0, 0.0, 1.0) * np.clip((x - 120.0) / 40.0, 0.0, 1.0) \
        * np.clip((32.0 - np.abs(y)) / 8.0, 0.0, 1.0) * np.clip((z + 95.0) / 10.0, 0.0, 1.0)
    throat = np.clip((x - 255.0) / 25.0, 0.0, 1.0) * np.clip((118.0 - z) / 16.0, 0.0, 1.0) \
        * np.clip((30.0 - np.abs(y)) / 10.0, 0.0, 1.0)
    muzzle = np.clip((x - 418.0) / 8.0, 0.0, 1.0) * np.clip((140.0 - z) / 6.0, 0.0, 1.0)
    chin = np.clip((x - 400.0) / 10.0, 0.0, 1.0) * np.clip((128.0 - z) / 6.0, 0.0, 1.0)
    blaze_w = np.clip(10.0 - 0.32 * (z - 138.0), 2.0, 10.0)      # narrowing upward
    blaze = np.clip((x - 404.0) / 6.0, 0.0, 1.0) * np.clip((blaze_w - np.abs(y)) / 3.0, 0.0, 1.0) \
        * np.clip((166.0 - z) / 5.0, 0.0, 1.0) * np.clip((N[:, 0] + 0.2) * 2.0, 0.0, 1.0)
    paws = np.clip((-155.0 - z) / 6.0, 0.0, 1.0)
    white = np.clip(np.maximum.reduce([under, throat, muzzle, chin, blaze, paws]), 0.0, 1.0)
    C = _mix(C, COAT["white"], white)
    # orange round the eyes and over the cheekbones
    for sgn in (-1.0, 1.0):
        d2 = ((x - 410.0) / 14.0) ** 2 + ((y - 24.0 * sgn) / 14.0) ** 2 + ((z - 147.0) / 12.0) ** 2
        C = _mix(C, COAT["orange"], 0.75 * np.clip(1.4 - d2, 0.0, 1.0) * (1 - white))
    # the ears: pink in the scoop that faces forward
    base, tip, _rb, _rt = EAR
    in_ear = (z > base[2] + 4.0) & (np.abs(np.abs(y) - 0.5 * (base[1] + tip[1])) < 18.0)
    ph = math.radians(EAR_TURN)
    facing = N[:, 0] * math.cos(ph) + np.abs(N[:, 1]) * math.sin(ph) * np.sign(y * N[:, 1])
    front = np.clip((facing - 0.25) * 2.5, 0.0, 1.0)
    C = _mix(C, COAT["pink"], (in_ear * front * np.clip((z - base[2] - 4.0) / 8.0, 0, 1)))
    return np.clip(C, 0.0, 1.0)


# ------------------------------------------------------------------- render
FEATURE = {"rim": (0.10, 0.08, 0.07), "iris": (0.86, 0.60, 0.22),
           "pupil": (0.04, 0.03, 0.03), "glint": (1.0, 1.0, 1.0),
           "nose rim": (0.25, 0.16, 0.15), "nose": (0.90, 0.58, 0.56)}

#: `(name, elevation, azimuth, focus, distance)`: the whole cat twice, and the
#: face from the front, as the photograph sees it.
VIEWS = (("3/4", 14, -58, None, 1300.0), ("side", 2, -90, None, 1300.0),
         ("face", 9, -16, (408.0, 0.0, 165.0), 320.0))


def _ellipsoid_actor(c, r, axis, tilt, colour, specular):
    import vtk
    import leg_drive_3d as LD3
    sp = vtk.vtkSphereSource()
    sp.SetRadius(1.0)                        # ⚠️ the default is 0.5
    sp.SetThetaResolution(48)
    sp.SetPhiResolution(48)
    tf = vtk.vtkTransform()
    tf.PostMultiply()
    tf.Scale(*r)
    tf.RotateX(tilt)                         # the almond's slant, in its own frame
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    ang = math.degrees(math.acos(np.clip(a[0], -1.0, 1.0)))
    ax = np.cross([1.0, 0.0, 0.0], a)
    if np.linalg.norm(ax) > 1e-9:
        tf.RotateWXYZ(ang, *ax)
    tf.Translate(*c)
    tp = vtk.vtkTransformPolyDataFilter()
    tp.SetInputConnection(sp.GetOutputPort())
    tp.SetTransform(tf)
    tp.Update()
    return LD3._actor(tp.GetOutput(), colour, specular=specular)


def render(pd, path, features=(), views=VIEWS, size=(1000, 760)):
    """The cat in its coat, with its face; one tile per view."""
    import vtk
    from PIL import Image
    from vtk.util import numpy_support as ns
    P, _T = mesh_arrays(pd)
    N = ns.vtk_to_numpy(pd.GetPointData().GetNormals()).astype(float)
    col = (255 * coat(P, N)).astype(np.uint8)
    arr = ns.numpy_to_vtk(col, deep=True, array_type=vtk.VTK_UNSIGNED_CHAR)
    arr.SetName("coat")
    pd.GetPointData().SetScalars(arr)
    tiles = []
    for name, elev, azim, focus, dist in views:
        ren = vtk.vtkRenderer()
        ren.SetBackground(1.0, 1.0, 1.0)
        ren.SetBackground2(0.90, 0.92, 0.95)
        ren.GradientBackgroundOn()
        mp = vtk.vtkPolyDataMapper()
        mp.SetInputData(pd)
        mp.SetColorModeToDirectScalars()
        mp.SetScalarModeToUsePointData()
        act = vtk.vtkActor()
        act.SetMapper(mp)
        pr = act.GetProperty()
        pr.SetSpecular(0.04)
        pr.SetAmbient(0.22)
        pr.SetDiffuse(0.85)
        ren.AddActor(act)
        for kind, c, r, axis, tilt in features:
            ren.AddActor(_ellipsoid_actor(c, r, axis, tilt, FEATURE[kind],
                                          0.9 if kind in ("iris", "pupil") else 0.15))
        cam = ren.GetActiveCamera()
        if focus is None:
            b = np.array(pd.GetBounds())
            c = np.array([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2])
        else:
            c = np.asarray(focus, float)
        el, az = math.radians(elev), math.radians(azim)
        cam.SetFocalPoint(*c)
        cam.SetPosition(*(c + dist * np.array([math.cos(el) * math.cos(az),
                                               math.cos(el) * math.sin(az), math.sin(el)])))
        cam.SetViewUp(0, 0, 1)
        cam.SetViewAngle(30)
        ren.ResetCameraClippingRange()
        win = vtk.vtkRenderWindow()
        win.SetOffScreenRendering(1)
        win.SetSize(2 * size[0], 2 * size[1])
        win.SetMultiSamples(8)
        win.AddRenderer(ren)
        ren.SetUseFXAA(True)
        win.Render()
        w2i = vtk.vtkWindowToImageFilter()
        w2i.SetInput(win)
        w2i.Update()
        im = w2i.GetOutput()
        W, H, _ = im.GetDimensions()
        a = ns.vtk_to_numpy(im.GetPointData().GetScalars()).reshape(H, W, -1)[::-1]
        tiles.append(Image.fromarray(a[:, :, :3].copy()).resize(size, Image.LANCZOS))
        win.Finalize()
    out = Image.new("RGB", (sum(t.width for t in tiles), size[1]), "white")
    x = 0
    for t in tiles:
        out.paste(t, (x, 0))
        x += t.width
    out.save(path)
    print("wrote", path)


def write_stl(pd, path):
    import vtk
    w = vtk.vtkSTLWriter()
    w.SetInputData(pd)
    w.SetFileName(path)
    w.SetFileTypeToBinary()
    w.Write()
    print("wrote", path)


if __name__ == "__main__":
    cache = os.path.join(HERE, "cat_suit_points.npz")
    pts = structure_points(cache)
    st = {}
    suit = suit_mesh(pts, out=st)
    report(pd=suit, points=pts, st=st)
    write_stl(suit, os.path.join(HERE, "cat_suit.stl"))
    render(suit, os.path.join(HERE, "cat_suit.png"), face_features(st["F"], st["lo"]))
