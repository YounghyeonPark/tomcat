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

The suit is open at the soles (the pads touch the ground) and at a collar
behind the head, which `tomcat_head` already gives a cat's face.

⚠️ **What the stance pose proves, and why it is enough.** Every part on a leg
is fixed to one link, and every free cable run lies between two circles fixed
to one link (the cable leaves each sheave on a tangent), so a sleeve that
clears its link's parts at the stance clears them at every pose. What changes
with pose is the fabric at the JOINTS, and that is `joint_strain`.

⚠️ It is a cover, not a structure: it carries no load. Its mass is reported
and NOT yet in `params.py` -- M129 (the spools) re-baselines the masses anyway.  `[owed]`
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
#: Blend radii, mm: how softly the torso, the legs and the structure merge.
K_BODY, K_LEG, K_STRUCT = 18.0, 9.0, 5.0
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

#: The collar: the suit ends round the neck, this far up it from its base.
COLLAR_AT = 0.80
NECK_DIR = np.array([math.cos(math.radians(HD.RISE_DEG)), 0.0,
                     math.sin(math.radians(HD.RISE_DEG))])
NECK_BASE = np.array([HD.BASE_X, 0.0, HD.BASE_Z])
COLLAR = NECK_BASE + COLLAR_AT * HD.NECK_L * NECK_DIR


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
CAT = (
    (-150.0, 6.0, 64.0, 46.0),   # the rump's end, rounding into the tail
    (-128.0, 38.0, 84.0, 4.0),
    (-95.0, 54.0, 90.0, -40.0),  # the haunches
    (-55.0, 54.0, 91.0, -46.0),
    (-15.0, 48.0, 89.0, -40.0),
    (35.0, 44.0, 87.0, -34.0),   # the loin
    (90.0, 41.0, 86.0, -27.0),   # the waist, tucked
    (140.0, 46.0, 87.0, -48.0),
    (185.0, 52.0, 88.0, -68.0),  # the chest, deepest between the forelegs
    (230.0, 51.0, 87.0, -72.0),
    (262.0, 44.0, 85.0, -56.0),
    (292.0, 22.0, 80.0, -12.0),  # the chest's front, under the neck
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
    (36.0 + OFFSET, 34.0 + OFFSET, 59.5, 20.0),   # thigh: hip sheave to knee sheave
    (34.0 + OFFSET, 22.0 + OFFSET, 60.5, 19.0),   # shank: knee to ankle
    (22.0 + OFFSET, 9.0, 53.0, 10.0),             # metatarsus: ankle to paw
    (9.0, 7.5, 53.0, 11.0),                       # paw
)


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
    # the neck, from the chest up to (and past) the collar
    nb = NECK_BASE + np.array([-10.0, 0.0, -2.0])
    ne = NECK_BASE + 1.05 * HD.NECK_L * NECK_DIR
    f = _smin(f, _round_cone(P, nb, ne, 30.0, 24.0), K_BODY)
    # the tail sleeve
    cur = np.asarray(TL.spine_curve(), float)
    pts3 = np.column_stack([cur[:, 0], np.zeros(len(cur)), cur[:, 1]])
    ts = np.linspace(0.0, 1.0, len(cur))
    tail = None
    for i in range(len(cur) - 1):
        ri = TL.radius_at(ts[i]) + 3.0
        rj = TL.radius_at(ts[i + 1]) + 3.0
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
                        np.array([58.0, 34.0, 68.0]))
        f = _smin(f, sh, K_BODY)
        # the ankle (hock / wrist): its sheave stack is 32 mm across (y 45-77)
        # where the metatarsus is 12, so it gets its own flattened knuckle
        pa, _q = leg_points(fore)
        ank = _ellipsoid(P, np.array([hx + pa[2][0], side * 61.0, pa[2][1]]),
                         np.array([27.0, 21.5, 27.0]))
        f = _smin(f, ank, K_LEG)
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
    neck (the head's solid, cut at the collar), the tail."""
    import tomcat_assembly as A
    bodies, legs, _soft = A.assembly(soft=False)
    return bodies + legs + [HD.whole(), TL.tail()]


def structure_points(cache=None, tol=0.8, spacing=0.6):
    """Surface samples of `structure_solids`, dense enough for `VOXEL`.

    ⚠️ The cache is stale the moment any geometry moves, and a stale one
    would fit the suit to a skeleton that no longer exists -- silently. It is
    used only while it is newer than every CAD script and `params.py`."""
    if cache and os.path.exists(cache):
        src = [os.path.join(HERE, f) for f in os.listdir(HERE) if f.endswith(".py")]
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


def _collar_dist(P):
    """Signed distance past the collar plane (> 0 is the head side)."""
    return (P - COLLAR) @ NECK_DIR


def grid(points):
    lo = points.min(0) - 25.0
    hi = points.max(0) + 25.0
    lo[2] = SOLE_Z - 3.0 * VOXEL
    n = np.ceil((hi - lo) / VOXEL).astype(int) + 1
    return lo, n


def field(points):
    """The suit's field on the grid: `(F, lo, n)`; F < 0 is inside the suit."""
    from scipy import ndimage
    lo, n = grid(points)
    # the structure, pushed out by OFFSET: distance to the nearest sample
    occ = np.zeros(n, bool)
    ij = np.round((points - lo) / VOXEL).astype(int)
    ok = np.all((ij >= 0) & (ij < n), axis=1)
    occ[tuple(ij[ok].T)] = True
    dist = ndimage.distance_transform_edt(~occ).astype(np.float32) * VOXEL
    F = dist - OFFSET
    del dist, occ
    # the cat, one x-slab at a time
    ys = lo[1] + VOXEL * np.arange(n[1])
    zs = lo[2] + VOXEL * np.arange(n[2])
    Y, Z = np.meshgrid(ys, zs, indexing="ij")
    for i in range(n[0]):
        x = lo[0] + VOXEL * i
        P = np.stack([np.full_like(Y, x), Y, Z], axis=-1)
        c = cat_field(P).astype(np.float32)
        F[i] = _smin(c, F[i], K_STRUCT)
        # open at the collar and at the soles
        F[i] = np.maximum(F[i], _collar_dist(P).astype(np.float32))
        F[i] = np.maximum(F[i], (SOLE_Z - Z).astype(np.float32))
    # nothing hollow inside: a void enclosed by the suit is suit
    solid = F < 0
    filled = ndimage.binary_fill_holes(solid)
    F[filled & ~solid] = -VOXEL
    return F, lo, n


def suit_mesh(points=None, cache=None):
    """The suit's outer surface as `vtkPolyData`, open at the collar and soles."""
    import vtk
    from vtk.util import numpy_support as ns
    if points is None:
        points = structure_points(cache)
    F, lo, n = field(points)
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
    """Drop the flat caps the collar and the sole planes leave: the suit is
    OPEN there, which is what lets the pads touch and the face show."""
    import vtk
    from vtk.util import numpy_support as ns
    P = ns.vtk_to_numpy(pd.GetPoints().GetData()).astype(float)
    polys = ns.vtk_to_numpy(pd.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
    C = P[polys].mean(axis=1)
    on = (np.abs(C[:, 2] - SOLE_Z) < 0.35 * VOXEL) | (np.abs(_collar_dist(C)) < 0.35 * VOXEL)
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
    covered = (points[:, 2] > SOLE_Z + CLEAR) & (_collar_dist(points) < -CLEAR)
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


def report(cache=None, pd=None, points=None):
    ok = True
    if points is None:
        points = structure_points(cache)
    if pd is None:
        pd = suit_mesh(points)
    P, T = mesh_arrays(pd)
    print("CAT SUIT -- one knit over the whole skeleton (ADR-0119)\n")
    bb = P.min(0), P.max(0)
    print("surface        %d triangles, nose-to-tail %.0f mm, %.0f wide, %.0f high"
          % (len(T), bb[1][0] - bb[0][0], bb[1][1] - bb[0][1], bb[1][2] - bb[0][2]))
    a = area_mm2(pd) * 1e-6
    print("area           %.3f m2 -> %.0f g at %.0f g/m2 (knit, [assumed])"
          % (a, a * AREAL_G_M2, AREAL_G_M2))
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


# ------------------------------------------------------------------- render
FUR = (0.80, 0.56, 0.38)


def render(pd, path, views=(("3/4", 14, -58), ("side", 2, -90)), size=(1000, 760),
           skeleton=None):
    """The suited cat, opaque, with the head and its face; one tile per view."""
    import vtk
    import leg_drive_3d as LD3
    from PIL import Image
    from vtk.util import numpy_support as ns
    head = LD3._polydata([HD.whole()], 0.5)
    fs = sorted(HD.features().solids(), key=lambda sd: -sd.volume)
    ears = LD3._polydata(fs[:2], 0.3)          # the two largest are the ears
    face = LD3._polydata(fs[2:], 0.3)          # eyes and nose
    tiles = []
    for name, elev, azim in views:
        ren = vtk.vtkRenderer()
        ren.SetBackground(1.0, 1.0, 1.0)
        ren.SetBackground2(0.90, 0.92, 0.95)
        ren.GradientBackgroundOn()
        op = 0.35 if skeleton is not None else 1.0
        ren.AddActor(LD3._actor(pd, FUR, opacity=op, specular=0.08))
        ren.AddActor(LD3._actor(head, FUR, specular=0.08))
        ren.AddActor(LD3._actor(ears, FUR, specular=0.08))
        ren.AddActor(LD3._actor(face, (0.12, 0.10, 0.09), specular=0.6))
        if skeleton is not None:
            ren.AddActor(LD3._actor(skeleton, (0.45, 0.52, 0.62)))
        if skeleton is not None:
            ren.SetUseDepthPeeling(1)
            ren.SetMaximumNumberOfPeels(8)
        cam = ren.GetActiveCamera()
        b = np.array(pd.GetBounds())
        hb = np.array(head.GetBounds())
        b[1::2] = np.maximum(b[1::2], hb[1::2])
        b[0::2] = np.minimum(b[0::2], hb[0::2])
        c = np.array([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2])
        el, az = math.radians(elev), math.radians(azim)
        d = 1300.0
        cam.SetFocalPoint(*c)
        cam.SetPosition(*(c + d * np.array([math.cos(el) * math.cos(az),
                                            math.cos(el) * math.sin(az), math.sin(el)])))
        cam.SetViewUp(0, 0, 1)
        cam.SetViewAngle(30)
        ren.ResetCameraClippingRange()
        win = vtk.vtkRenderWindow()
        win.SetOffScreenRendering(1)
        win.SetSize(2 * size[0], 2 * size[1])
        win.SetMultiSamples(0 if skeleton is not None else 8)
        if skeleton is not None:
            win.SetAlphaBitPlanes(1)
        win.AddRenderer(ren)
        ren.SetUseFXAA(True)
        win.Render()
        w2i = vtk.vtkWindowToImageFilter()
        w2i.SetInput(win)
        w2i.Update()
        im = w2i.GetOutput()
        W, H, _ = im.GetDimensions()
        arr = ns.vtk_to_numpy(im.GetPointData().GetScalars()).reshape(H, W, -1)[::-1]
        tiles.append(Image.fromarray(arr[:, :, :3].copy()).resize(size, Image.LANCZOS))
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
    suit = suit_mesh(pts)
    report(pd=suit, points=pts)
    write_stl(suit, os.path.join(HERE, "cat_suit.stl"))
    render(suit, os.path.join(HERE, "cat_suit.png"))
