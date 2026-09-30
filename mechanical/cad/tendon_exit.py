# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""How a leg cable gets from its motor to its leg -- M122, ADR-0112.

⚠️ **The drive was routed in a plane it never lived in.** M88 laid the motors
fore-aft (`tomcat_trunk`): each spool sits at the end of a row INSIDE the trunk,
at y = +/-20.2, on an axis along x, so it winds its cable in the y-z plane. Every
router, drawing and friction figure since treated it as a spool on an axis along
y, in the leg's own plane at y = 65-75 -- a 2-D projection that dropped the
45-55 mm between them, turned the spool's axis by 90 deg, and left the cable no
way out of the shell. `tomcat_trunk`'s own docstring had it `[owed]`; the
assembly's "cable ends on its spool" check compared the router's first point
with the coordinates it had been handed, so it could not fail.

⚠️ **Three ways across the hip were tried before this one** (ADR-0112):

1. *An exit idler in each cable's plane.* Over the full joint range the knee and
   ankle sheaves and the vias sweep nearly all of that plane round the hip; the
   ankle cable had one narrow window, whose spools sat inside the spine joint's
   yoke. And an idler that turns a LATERAL lead into the plane stands 8.75 mm
   out of it, into the femur's own sweep.
2. *Along the hip axis* (a hollow hip shaft, turning pulleys on the femur).
   Each 90 deg turn off the axis needs 2R = 17.5 mm of axial room inboard of its
   plane -- through the hip sheave, and through the other three turns.
3. The hip ON spine joint 0 closed both, to 0.1 mm, behind the motors packed
   there. ADR-0112 moved the joint 30 mm forward (`SpineParams.rear_hip_x`).

✅ **Bowden conduits.** Every cable leaves its spool tangentially in the
spool's x-plane (zero fleet) and runs in a conduit to a ferrule F2 aimed along
its in-plane run:

- **hip pair** -- F1 on the trunk wall, F2 on a TRUNK bracket above the hip,
  where the femur never goes, in the hip plane, aimed at the hip sheave. The
  conduit does not move.
- ✅ **knee and ankle pairs -- M123 (ADR-0114): along the hip axis.** Their
  motors sit either side of the hip at hip height, spools facing across it
  (`tomcat_trunk`), so a spool's two leads leave its top and bottom running
  straight along +y -- parallel to the hip axis, `HIP_SPOOL_DX` either side of
  it. Each enters its conduit inside the trunk (F1 at |y| = `LEAD_Y`), runs out
  through the trunk's hollow stub and the femur hub it carries, and turns to
  F2 seated in the hub's wall, `HUB_F2_R` off the axis on the ray to the
  cable's target. The femur cannot reach a conduit inside its own hub, and the
  hip only rotates F2 about the axis: the conduit twists, it is not swept.

⚠️ M122 hung F2 on the femur 42 mm out, and every knee and ankle conduit ran
through the hip sheave, the hip's boss and the femur's root -- it had to
cross the femur's own plane where the femur sweeps (ADR-0112, ADR-0113). No
routing at the hip fixed it; moving the motors did.

Friction is the conduit's SLIDING over its bend (`e^(mu_c theta)`, worst over
the hip range) times one running pulley's efficiency where there is one (the
knee via, on the ankle pair). `place()` solves the hip pair's motor end, leads
and trunk ferrules; the knee and ankle pairs' drive is fixed by the layout.
`DRIVE` pins the result.

⚠️ The conduit is modelled as a cubic Bezier between its ferrules. Its real
shape is `[owed]` to a mock-up; its bend radius, and that the knee and ankle
conduits stay inside the stub's and the hub's bores, are checked here. Four
conduits share the bore and twist with the hip `[owed]`.
"""
from __future__ import annotations

import itertools
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "kinematics", "src"))

import leg_tendons as LT  # noqa: E402
import tomcat_packaging as TP  # noqa: E402
from tomcat_kin import LegModel  # noqa: E402
from tomcat_kin.params import (  # noqa: E402
    DEFAULT_FORELEG, DEFAULT_HINDLEG, DEFAULT_SPINE,
)

TENDONS = ("hip", "knee", "ankle")
CABLE_D = LT.CABLE_D
#: Hip stations, trunk x mm; the hind hips sit `rear_hip_x` behind the spine's
#: root joint (ADR-0112).
HIP_X = {"hind": float(DEFAULT_SPINE.rear_hip_x) * 1000.0,
         "fore": (float(sum(DEFAULT_SPINE.segment_lengths))
                  + float(DEFAULT_SPINE.front_hip_x)) * 1000.0}
STANCE_FOOT = (0.04, -0.17, 0.0)
CAN_R = TP.MOTOR_D / 2.0

#: The conduit: PTFE-lined, 3 mm outside over the Ø1.75 cable. `[assumed]`
CONDUIT_OD = 3.0
#: Its minimum bend radius, mm. `[assumed]`
CONDUIT_RMIN = 15.0
#: UHMWPE sliding in a PTFE liner. `[assumed]`
CONDUIT_MU = 0.07
#: ✅ M123 (ADR-0114): the knee and ankle conduits end in the femur hub's WALL,
#: this far off the hip axis, on the ray to the cable's target, mm. Not at 15:
#: the ankle pair's two rays reach round a via of 8.75 mm about 100 mm away,
#: so they leave the hip only ~10 deg apart; at 30 their ferrules clear.
HUB_F2_R = 30.0
#: ...and begin inside the trunk: F1 where the lead reaches this |y|, mm.
LEAD_Y = 31.5
#: (F1's ferrule body, 2 mm behind F1, clears its own spool's flange from
#: |y| 31.1 -- the flange is 10.9 mm round an axis 8.75 from the lead.)
#: What a conduit keeps from the leg, mm (beyond its own radius).
LEG_MARGIN = 1.0
#: Trunk ferrules for the hip pair: this far from the hip, above it, mm.
HIP_F2_R = 55.0
#: The hip pair's two trunk ferrules keep at least this angle apart (~10 mm).
HIP_F2_SEP = math.radians(10.0)
#: Hip angles the conduit is checked over (fraction of the range).
N_HIP = 7


def _trunk():
    import tomcat_trunk as TT
    return TT


def leg(role):
    return DEFAULT_FORELEG if role == "fore" else DEFAULT_HINDLEG


def stance(role):
    return np.asarray(LegModel(leg(role)).inverse(STANCE_FOOT), float)


def plane_y(tendon: str, side: float = +1.0) -> float:
    import tomcat_leg_detail as LD
    lay = LD.plane_layout({"hip": 6.4, "knee": 6.4, "ankle": 4.4},
                          {j: LD.BEARING[j][2] for j in TENDONS})
    return side * (LD.TRACK_Y + lay["hip"]["plane"][tendon])


def motors(role: str, side: float = +1.0):
    """The three motors on this leg's side of its girdle, top first:
    [(row, x, y, z)], trunk mm."""
    TT = _trunk()
    own = []
    for (x, y, z) in TT.leg_spools(role, side, by="height"):
        row = [r for r in TT._rows() if abs(r[2] - x) < 1e-6 and r[4] == role][0]
        own.append((row[0], x, y, z))
    return own


def spool_x_of(role, m, end, side=+1.0):
    TT = _trunk()
    x = motors(role, side)[m][1]
    return x + end * (TT.row_len(role) / 2 - TP.SPOOL_L / 2)


def cans(role, side=+1.0):
    """Every motor can in the leg's girdle body as (x0, x1, y, z)."""
    TT = _trunk()
    body = TT.LEG_BODY[role]
    out = []
    for (_nm, b, x, n, ro) in TT._rows():
        if b != body:
            continue
        L = TT.row_len(ro if ro in ("hind", "fore", "spine") else "hind")
        for (y, z) in TT.ROWS[n][0]:
            out.append((x - L / 2, x + L / 2, y, TT._zc(x) + z))
    return out


#: Spool placements that put a spool inside trunk STRUCTURE (the spine joints'
#: yokes and processes at the girdle ends), (motor by height, end) -- measured
#: on the CAD, and re-measured by `test_tendon_exit`.
FORBIDDEN = {"hind": {(0, +1), (1, +1)}, "fore": {(2, -1)}}   # M123: the new rows


def _seg_clear(P, H, C, r):
    P, H, C = np.asarray(P, float), np.asarray(H, float), np.atleast_2d(C)
    d = H - P
    L = float(d @ d)
    t = np.clip(((C - P) @ d) / L, 0.0, 1.0)
    return float((np.linalg.norm(C - (P + t[:, None] * d), axis=1) - r).min())


def leg_run(role, tendon, side, q=None, start=None, senses=None):
    """The in-plane run to the anchored sheave: for the hip pair from the trunk
    ferrule `start` = (x, z) hip frame; for the knee and ankle pairs from the
    hip axis (the hub ferrule sits on that ray, `HUB_F2_R` out)."""
    lg = leg(role)
    q = stance(role) if q is None else q
    if tendon == "hip":
        return LT.route(q, tendon, side=side, leg=lg, spools={tendon: start},
                        lead_r=1e-3, senses=senses)
    return LT.route(q, tendon, side=side, leg=lg, spools={tendon: (0.0, 0.0)},
                    lead_r=1e-3, entry="axis", senses=senses)


#: Handle lengths (fractions of the span) a conduit may settle into.
HANDLES = (0.25, 0.4, 0.55, 0.7)


def bezier(P0, d0, P3, d3, n: int = 60, h0: float = 0.42, h3: float = 0.42):
    """The conduit between two ferrules: position and direction at each end,
    with handle lengths `h0`, `h3` as fractions of the span."""
    L = float(np.linalg.norm(P3 - P0))
    P1, P2 = P0 + d0 * h0 * L, P3 - d3 * h3 * L
    t = np.linspace(0.0, 1.0, n)[:, None]
    return (((1 - t) ** 3) * P0 + 3 * ((1 - t) ** 2) * t * P1
            + 3 * (1 - t) * t * t * P2 + t ** 3 * P3)


def bend(C):
    """(total turning angle rad, tightest radius mm, length mm) of a polyline."""
    d = np.diff(C, axis=0)
    seg = np.linalg.norm(d, axis=1)
    u = d / seg[:, None]
    ang = np.arccos(np.clip((u[:-1] * u[1:]).sum(1), -1.0, 1.0))
    ds = 0.5 * (seg[:-1] + seg[1:])
    rad = np.where(ang > 1e-9, ds / np.maximum(ang, 1e-12), np.inf)
    return float(ang.sum()), float(rad.min()), float(seg.sum())


_SHELL = {}
_WALL = {}


def _shell(body):
    if body not in _SHELL:
        _SHELL[body] = _trunk()._outer(body)
    return _SHELL[body]


def _wall_point(body, xs, y0, z0, dy, dz):
    """Where a ray from (y0, z0) in the plane x = xs leaves the shell's OUTER
    surface -- found on the CAD solid.

    ⚠️ An analytic ellipse at `_hw_at(x)` put every conduit ferrule 1-3 mm
    INSIDE the wall: the loft through the section stations is fuller between
    them than a linear interpolation of their widths."""
    key = (body, round(xs, 3), round(y0, 3), round(z0, 3), round(dy, 4), round(dz, 4))
    if key in _WALL:
        return _WALL[key]
    sh = _shell(body)
    p0 = np.array([xs, y0, z0])
    d = np.array([0.0, dy, dz])
    if not sh.is_inside(tuple(p0)):
        _WALL[key] = None
        return None
    lo, hi = 0.0, None
    for t in np.arange(2.0, 120.0, 2.0):
        if not sh.is_inside(tuple(p0 + t * d)):
            hi = t
            break
        lo = t
    if hi is None:
        _WALL[key] = None
        return None
    for _ in range(14):
        mid = 0.5 * (lo + hi)
        if sh.is_inside(tuple(p0 + mid * d)):
            lo = mid
        else:
            hi = mid
    out = p0 + hi * d
    _WALL[key] = out
    return out


def _outside_shell(C, pad=4.0):
    """Every point of a conduit outside the shell by about `pad` -- an analytic
    check fast enough for the search (the ellipse runs ~3 mm inside the loft,
    so `pad` = 4 keeps ~1 mm); the assembly re-checks on the CAD."""
    TT = _trunk()
    for p in C:
        hw = TT._hw_at(p[0]) + pad
        hh = TT._hw_at(p[0]) * TT.ASPECT + pad
        if (p[1] / hw) ** 2 + ((p[2] - TT._zc(p[0])) / hh) ** 2 <= 1.0:
            return False
    return True


def spool_lead(role, m, end, phi, sense, side=+1.0, to_y=None):
    """A lead leaving motor `m`'s spool at angle `phi` round it, in winding
    `sense`: (tangent point, F1, lead direction), or None if it is blocked.

    F1 is on the trunk wall (the hip pair's conduits leave the body), or --
    `to_y`, the knee and ankle pairs -- where the lead reaches |y| = `to_y`
    inside the trunk: those leads run along the hip axis."""
    _nm, x, y, z = motors(role, side)[m]
    xs = spool_x_of(role, m, end, side)
    T = np.array([y, z]) + LT.SPOOL_R * np.array([math.cos(phi), math.sin(phi)])
    dv = sense * np.array([-math.sin(phi), math.cos(phi)])
    if to_y is not None:
        if side * dv[0] < 0.99:
            return None
        F1 = np.array([xs, *(T + (side * to_y - T[0]) / dv[0] * dv)])
        return np.array([xs, T[0], T[1]]), F1, np.array([0.0, dv[0], dv[1]])
    F1 = _wall_point(_trunk().LEG_BODY[role], xs, T[0], T[1], dv[0], dv[1])
    if F1 is None or side * F1[1] < 20.0:
        return None
    for c in cans(role, side):
        if c[0] <= xs <= c[1] and not (abs(c[2] - y) < 1e-6 and abs(c[3] - z) < 1e-6):
            if _seg_clear(T, F1[1:], (c[2], c[3]), CAN_R + 1.0) < 0:
                return None
    return np.array([xs, T[0], T[1]]), F1, np.array([0.0, dv[0], dv[1]])


def hub_ferrule(role, tendon, side_c, q, lr_side=+1.0):
    """F2 in the femur hub's wall at pose `q`: `HUB_F2_R` off the hip axis in
    the cable's plane, on the ray the free cable leaves along -- fixed in the
    femur, so it turns with the hip. (position, direction), trunk mm."""
    run = leg_run(role, tendon, side_c, q)
    p = run["points"]
    u = (p[1] - p[0]) / np.linalg.norm(p[1] - p[0])
    return (np.array([HIP_X[role] + HUB_F2_R * u[0], plane_y(tendon, lr_side),
                      HUB_F2_R * u[1]]), np.array([u[0], 0.0, u[1]]))


def hip_ferrule(role, side_c, angle, q, lr_side=+1.0):
    """F2 for the hip pair: on a trunk bracket `HIP_F2_R` from the hip at
    `angle` from straight up, aimed along the run to the hip sheave."""
    xh = HIP_X[role]
    P = (xh + HIP_F2_R * math.sin(angle), HIP_F2_R * math.cos(angle))
    run = leg_run(role, "hip", side_c, q, start=(P[0] - xh, P[1]))
    p = run["points"]
    u = (p[1] - p[0]) / np.linalg.norm(p[1] - p[0])
    return np.array([P[0], plane_y("hip", lr_side), P[1]]), np.array([u[0], 0.0, u[1]])


def hip_range(role, n=N_HIP):
    lg = leg(role)
    q0 = stance(role)
    return [np.array([h, q0[1], q0[2]]) for h in np.linspace(lg.q_min[0], lg.q_max[0], n)]


def conduit_shape(F1, d1, F2, d2, ok=None, handles=HANDLES):
    """The shape a flexible conduit settles into between its ferrules, modelled
    as the Bezier, over `handles`, whose tightest bend is gentlest:
    (polyline, bend, tightest radius, length). `ok`: where it is guided -- a
    conduit in a bore takes the gentlest shape the bore lets it take."""
    best = None
    for h0 in handles:
        for h3 in handles:
            C = bezier(F1, d1, F2, d2, h0=h0, h3=h3)
            if ok is not None and not ok(C):
                continue
            b, r, L = bend(C)
            if best is None or r > best[2]:
                best = (C, b, r, L)
    if best is None:
        C = bezier(F1, d1, F2, d2)
        return C, np.inf, 0.0, bend(C)[2]
    return best


#: Handle lengths for a conduit guided through the hollow hip.
BORE_HANDLES = tuple(np.round(np.arange(0.1, 0.96, 0.05), 2))


def _leg_parts(role, q, lr_side=+1.0):
    """The leg near the hip at pose `q`, as slabs (discs across y) and
    capsules, trunk frame: [("disc", centre_xz, radius, y0, y1) |
    ("capsule", a3, b3, radius)]."""
    import tomcat_leg_detail as LD
    lg = leg(role)
    P = LT.joints(q, lg)
    xh = HIP_X[role]
    lay = LD.plane_layout({"hip": 6.4, "knee": 6.4, "ankle": 4.4},
                          {j: LD.BEARING[j][2] for j in TENDONS})
    fl = LD.FLANGE_W + LD.CABLE_D * 1.15 / 2
    half = fl + 0.5
    y0 = LD.TRACK_Y
    arms = [float(a) * 1000.0 for a in lg.q_min[:0]] or None      # unused
    hip = np.array([xh, P[0][1]])
    knee = np.array([xh + P[1][0], P[1][1]])
    ph = lay["hip"]["plane"]
    from tomcat_kin.params import DEFAULT_TENDON
    A = [float(v) * 1000.0 for v in DEFAULT_TENDON.joint_moment_arm]
    out = [
        ("disc", hip, A[0] + fl, y0 + ph["hip"] - half, y0 + ph["hip"] + half),
        ("disc", knee, A[1] + fl, y0 + ph["knee"] - half, y0 + ph["knee"] + half),
        ("disc", knee, LT.VIA_R + fl, y0 + ph["ankle"] - half, y0 + ph["ankle"] + half),
        # the hip's clevis, bearings and boss round the axis
        ("disc", hip, LD.BEARING["hip"][1] / 2 + LD.BOSS_WALL, y0 - 12.0, y0 + 12.0),
        ("capsule", np.array([hip[0], y0, hip[1]]), np.array([knee[0], y0, knee[1]]),
         LD.TUBE["femur"][0] / 2 + 2.0),
    ]
    if lr_side < 0:
        out = [(k, a, b, -d, -c) if k == "disc" else (k, a * [1, -1, 1], b * [1, -1, 1], c)
               for (k, a, b, c, d) in out]
    return out


def _leg_clear(role, C, q, lr_side=+1.0, skip_end=4, parts=None):
    """Does the conduit polyline `C` keep `LEG_MARGIN` from the leg at `q`?
    The last `skip_end` points are where it is fixed to the femur; `parts`
    picks which of `_leg_parts` to test (all by default)."""
    pts = np.asarray(C)[: len(C) - skip_end] if skip_end else np.asarray(C)
    if len(pts) == 0:
        return True
    r_c = CONDUIT_OD / 2 + LEG_MARGIN
    allp = _leg_parts(role, q, lr_side)
    for i, part in enumerate(allp):
        if parts is not None and i not in parts:
            continue
        if part[0] == "disc":
            _k, c, R, ya, yb = part
            lo, hi = min(ya, yb), max(ya, yb)
            m = (pts[:, 1] >= lo - r_c) & (pts[:, 1] <= hi + r_c)
            if m.any() and (np.hypot(pts[m, 0] - c[0], pts[m, 2] - c[1]) < R + r_c).any():
                return False
        else:
            _k, a, b, R = part
            d = b - a
            t = np.clip(((pts - a) @ d) / (d @ d), 0.0, 1.0)
            if (np.linalg.norm(pts - (a + t[:, None] * d), axis=1) < R + r_c).any():
                return False
    return True


def conduit_path(role, tendon, side_c, lead3, F2spec, q, lr_side=+1.0):
    """The conduit at pose `q`: (polyline, bend, tightest radius). The hip
    pair's runs trunk wall -> trunk ferrule (`F2spec` = its angle); the knee
    and ankle pairs' from inside the trunk, along the hip axis, to the hub."""
    _T, F1, d1 = lead3
    if tendon == "hip":
        F2, d2 = hip_ferrule(role, side_c, F2spec, stance(role), lr_side)
        C, b, r, _L = conduit_shape(F1, d1, F2, d2)
        return C, b, r
    F2, d2 = hub_ferrule(role, tendon, side_c, q, lr_side)
    C, b, r, _L = conduit_shape(F1, d1, F2, d2, ok=lambda c: in_bore(role, c, lr_side),
                                handles=BORE_HANDLES)
    return C, b, r


def in_bore(role, C, lr_side=+1.0):
    """A knee or ankle conduit stays where it may be: clear of the hip station's
    bulkhead inside the trunk, inside the stub's bore through the wall and out
    to the stub's end, inside the hub's bore beyond it, and out through the
    hub's wall only in its own plane, at its ferrule."""
    import tomcat_leg_detail as LD
    TT = _trunk()
    xh = HIP_X[role]
    rc = CONDUIT_OD / 2
    y = lr_side * C[:, 1]
    r = np.hypot(C[:, 0] - xh, C[:, 2])
    inner = TT._hw_at(xh) - TT.WALL
    stub_end = LD.TRACK_Y + LD.STUB_END
    bh = np.abs(C[:, 0] - xh) < TT.BULKHEAD_T / 2 + rc + 0.2
    if (bh & (y < inner)).any():
        return False
    stub = (y >= inner - 4.0) & (y <= stub_end)
    if (r[stub] > LD.HIP_STUB[1] / 2 - rc - 0.3).any():
        return False
    hub = y > stub_end
    wall_in = LD.HUB_BEARING[1] / 2 - rc - 0.3
    out = hub & (r > wall_in)
    # beyond the hub's bore only the ferrule's own radial run, in its plane:
    # within 1 mm of it, so the conduit (r 1.5) keeps clear of the hip sheave
    # beside the knee's plane
    return not (np.abs(y[out] - y[-1]) > 1.0).any()


def conduit_worst(role, tendon, side_c, lead3, F2spec, lr_side=+1.0):
    """(worst bend over the hip range, tightest radius) for one conduit; (inf,
    0) if it leaves where it may be. The hip pair's is fixed to the trunk and
    must stay outside the shell; the knee and ankle pairs' twist with the hip,
    inside the stub and the hub (`in_bore`)."""
    if lead3 is None:
        return np.inf, 0.0
    worst, rmin = 0.0, np.inf
    for q in ([stance(role)] if tendon == "hip" else hip_range(role)):
        C, b, r = conduit_path(role, tendon, side_c, lead3, F2spec, q, lr_side)
        if tendon == "hip" and not _outside_shell(C[4:]):
            return np.inf, 0.0            # the conduit would run back into the body
        if tendon != "hip" and not np.isfinite(b):
            return np.inf, 0.0             # no shape the bores allow
        worst, rmin = max(worst, b), min(rmin, r)
    return worst, rmin


#: The knee and ankle pairs, fixed by the layout (M123): motor by height (the
#: hip-station rows' bottom pair: 1 = the row in FRONT of the hip, 2 = behind
#: it), the spool end FACING the hip, and the leads off the spool's bottom
#: (flexor) and top (extensor), both running out along +y.
KNEE_ANKLE = {"knee": (1, -1), "ankle": (2, +1)}


def place(role: str, side=+1.0):
    """The hip pair: which end of the top motor's row its spool is on, its
    leads and its two trunk ferrules -- least conduit bend, every conduit at
    least `CONDUIT_RMIN`, the two cables leaving in OPPOSITE senses. The knee
    and ankle pairs are `KNEE_ANKLE`'s."""
    phis = np.radians(np.arange(0.0, 360.0, 6.0))
    hip_angles = np.radians(np.arange(-50.0, 51.0, 5.0))
    best = None
    for e in (+1, -1):
        if (0, e) in FORBIDDEN[role]:
            continue
        cands = {}
        for sd in (+1, -1):
            for sense in (+1.0, -1.0):
                got = {}
                for phi in phis:
                    l3 = spool_lead(role, 0, e, phi, sense, side)
                    if l3 is None:
                        continue
                    for a in hip_angles:
                        w, r = conduit_worst(role, "hip", sd, l3, a, side)
                        if r < CONDUIT_RMIN:
                            continue
                        if a not in got or w < got[a][0]:
                            got[a] = (w, float(phi), float(a), r)
                cands[(sd, sense)] = got
        for sf in (+1.0, -1.0):
            for f in cands[(+1, sf)].values():
                for x in cands[(-1, -sf)].values():
                    if abs(f[2] - x[2]) < HIP_F2_SEP - 1e-9:
                        continue
                    if best is None or f[0] + x[0] < best[0]:
                        best = (f[0] + x[0], (0, e, sf, f[1], x[1], (f[2], x[2])))
    if best is None:
        raise RuntimeError("%s: no admissible hip drive" % role)
    plan = {"hip": best[1]}
    for t, (m, e) in KNEE_ANKLE.items():
        plan[t] = (m, e, +1.0, math.radians(270.0), math.radians(90.0), None)
    return plan


#: ✅ **The drive as designed** (M122; M123 for the knee and ankle). Per tendon:
#:   (motor by height top-first, spool end (+1 = +x), flexor lead sense,
#:    flexor lead angle deg, extensor lead angle deg,
#:    hip pair only: (flexor, extensor) trunk-ferrule angle from straight up, deg)
#: A lead angle is where round the spool the cable leaves it; the extensor
#: leaves in the opposite sense. `place()` produced it; `test_tendon_exit`
#: holds the two together.
#:
#: M123 (ADR-0114): the hip pair drives off the top of the row in FRONT of
#: its hip; the knee off that row's bottom motor, the ankle off the bottom
#: motor of the row behind -- spools facing across the hip.
DRIVE = {
    "hind": {"hip": (0, -1, -1.0, 126.0, 324.0, (20.0, -20.0)),
             "knee": (1, -1, +1.0, 270.0, 90.0, None),
             "ankle": (2, +1, +1.0, 270.0, 90.0, None)},
    "fore": {"hip": (0, -1, -1.0, 126.0, 330.0, (20.0, -15.0)),
             "knee": (1, -1, +1.0, 270.0, 90.0, None),
             "ankle": (2, +1, +1.0, 270.0, 90.0, None)},
}


def _spec(role, t):
    """DRIVE's entry in radians."""
    m, e, sf, pf, pe, ha = DRIVE[role][t]
    if ha is not None:
        ha = (math.radians(ha[0]), math.radians(ha[1]))
    return m, e, sf, math.radians(pf), math.radians(pe), ha


def pulley_count(role: str = "hind"):
    """Running pulleys between each spool and its anchored sheave, per joint and
    side: only the knee via, on the ankle pair."""
    return ((0, 0), (0, 0), (1, 1))


def conduit_bend(role: str = "hind"):
    """Each conduit's worst bend over the hip range, rad, per joint and side --
    what `TendonParams.conduit_bend` carries."""
    out = []
    for t in TENDONS:
        m, e, sf, pf, pe, ha = _spec(role, t)
        row = []
        for sd, sense, phi in ((+1, sf, pf), (-1, -sf, pe)):
            l3 = spool_lead(role, m, e, phi, sense, to_y=None if t == "hip" else LEAD_Y)
            w, _r = conduit_worst(role, t, sd, l3, None if ha is None else ha[0 if sd > 0 else 1])
            row.append(w)
        out.append(tuple(row))
    return tuple(out)


def drive(role: str, side: float = +1.0, q=None, senses=None):
    """Every cable of one leg as designed, at pose `q`, in 3-D (trunk mm):
    {(tendon, +1/-1): dict} with the in-plane run (`points`, hip frame x-z,
    starting at F2), `lead` (tangent point, F1), `conduit` (polyline F1 -> F2),
    `F2`, `seat`, `spool_x`, `anchor`, `length`, `conduit_bend`."""
    own = motors(role, side)
    q = stance(role) if q is None else q
    xh = HIP_X[role]
    out = {}
    for t in TENDONS:
        m, e, sf, pf, pe, ha = _spec(role, t)
        seat = own[m][1:]
        xs = spool_x_of(role, m, e, side)
        for sd, sense, phi in ((+1, sf, pf), (-1, -sf, pe)):
            sn = None if senses is None else senses.get((t, sd))
            # the right leg mirrors the left in y: a lead at angle phi round the
            # spool becomes one at pi - phi, wound the other way
            to_y = None if t == "hip" else LEAD_Y
            if side < 0:
                T, F1, d1 = spool_lead(role, m, e, math.pi - phi, -sense, side, to_y)
            else:
                T, F1, d1 = spool_lead(role, m, e, phi, sense, side, to_y)
            F2spec = None if ha is None else ha[0 if sd > 0 else 1]
            C, b, r = conduit_path(role, t, sd, (T, F1, d1), F2spec, q, side)
            if t == "hip":
                F2, _d2 = hip_ferrule(role, sd, F2spec, stance(role), side)
                run = dict(leg_run(role, t, sd, q, start=(F2[0] - xh, F2[2]), senses=sn))
            else:
                F2, _d2 = hub_ferrule(role, t, sd, q, side)
                run = dict(leg_run(role, t, sd, q, senses=sn))
                # the run INSIDE the conduit, hip axis -> F2, is not free cable
                p = run["points"]
                k = int(np.argmax(np.linalg.norm(p - p[0], axis=1) >= HUB_F2_R))
                run["points"] = np.vstack([[F2[0] - xh, F2[2]], p[k:]])
            run.update({"seat": seat, "spool_x": xs, "lead": (T, F1), "conduit": C,
                        "F1": F1, "F2": F2, "conduit_bend": b, "conduit_rmin": r,
                        "lead_from": tuple(T), "plane_y": plane_y(t, side),
                        "free_length": run["length"]})
            out[(t, sd)] = run
    return out


def report():
    for role in ("hind", "fore"):
        own = motors(role)
        print("%s leg" % role.upper())
        cb = conduit_bend(role)
        for i, t in enumerate(TENDONS):
            m, e = DRIVE[role][t][:2]
            print("  %-5s on the %s motor (z %.1f), spool at its %s end; conduit bend "
                  "%.0f / %.0f deg worst -> friction x%.3f / x%.3f"
                  % (t, own[m][0], own[m][3], "+x" if e > 0 else "-x",
                     math.degrees(cb[i][0]), math.degrees(cb[i][1]),
                     math.exp(CONDUIT_MU * cb[i][0]) / 0.97 ** pulley_count(role)[i][0],
                     math.exp(CONDUIT_MU * cb[i][1]) / 0.97 ** pulley_count(role)[i][1]))


if __name__ == "__main__":
    for role in ("hind", "fore"):
        print(role, place(role))
