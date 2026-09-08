# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The whole trunk, motors distributed along it, in a cat's form (build123d).

⚠️ **This replaces the two-girdle architecture, and it had to.** `tomcat_girdle.py`
designed one girdle properly and the numbers said the concept does not close:

| | |
|---|---|
| girdle barrel, 6 motors clustered | 22.6 % packing |
| 18 motors + spools, solid | 636 cm³ |
| a cat-form trunk envelope | ~2100 cm³ |
| **those motors at 22.6 % packing** | **2813 cm³ — 134 % of the animal** |

Clustering into two lumps is what fails. A cat does not carry its muscle in two
boxes and neither can this: the motors go **along** the trunk, one row per rigid
body, and the packing doubles.

✅ **The row is a DIAMOND of four** -- one up, one down, one each side. That
shape suits a tall oval the way a square does not:

| section | envelope | packing in a 44.1 mm row |
|---|---|---|
| 4, diamond | **81.0 x 113.4 mm** | **44.5 %** |
| 3, symmetric triangle | 86.0 x 120.4 | 29.6 % |
| 2, over-under | 64.8 x 90.7 | 34.7 % |

Four is both *smaller* and *tighter* than three, which is not obvious and is why
the layout is solved rather than chosen.

⚠️ **A motor may not straddle a spine joint.** The trunk is four rigid bodies --
[rear girdle], [spine1], [spine2], [spine3 + front girdle] -- so the rows are
assigned per body, and `report()` checks that every motor lies inside one.

⚠️ Laying the motors fore-aft puts every spool on a bulkhead face, so a cable
reaching a lateral hip has to turn. ADR-0042/0083's wrap angles were computed for
upright motors and do not carry over.  `[owed]`
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
from build123d import Box, Compound, Cylinder, Ellipse, Plane, Pos, Rot, loft

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "kinematics", "src"))

import tomcat_packaging as TP                                    # noqa: E402
import tomcat_leg_detail as LD                                   # noqa: E402
from tomcat_kin.params import DEFAULT_SPINE as SP                # noqa: E402

MM = 1000.0
R = TP.MOTOR_D / 2.0
CLR = 3.0
WALL = 1.2
BULKHEAD_T = 1.6
ASPECT = 1.40
TRACK = LD.TRACK_Y
HIP_BORE, HIP_OD, HIP_W = LD.BEARING["hip"]
BOSS_WALL = LD.BOSS_WALL
TONGUE_T = 6.0
ROW_L = TP.MOTOR_L + TP.SPOOL_L                     # 44.1 mm
Z0 = SP.girdle_offset_z * MM

#: Solved symmetric layouts: (centres, outer half-width) for a row of N.
#: `[derived: scratch solver, mirror-symmetric, minimal 1.40-aspect ellipse]`
ROWS = {
    4: ([(0.0, 35.1), (-20.2, 0.0), (20.2, 0.0), (0.0, -35.1)], 40.5),
    2: ([(0.0, 20.3), (0.0, -20.3)], 32.4),
}

#: One row per rigid body, chest to tail. The **waist** is the two-motor row --
#: a cat is pinched between the ribcage and the pelvis, and that is where the
#: two motors that do not make a fourth diamond belong.
#: ⚠️ Rows are placed from each body's PROXIMAL joint, not centred on a
#: landmark. The first layout centred the rear row on x=0 -- which is the joint
#: -- so half of it sat in the next body, and put two rows 27.5 mm apart in one
#: body when a row is 44.1 long, so the motors interpenetrated. `report()`
#: checks both now.
_PAD = BULKHEAD_T + 0.5
LAYOUT = [
    # (name, body, x of the row centre, motors in the row)
    ("front_girdle", 3, 140.0 + 1.5 * ROW_L + 3 * _PAD, 4),
    ("spine3", 3, 140.0 + 0.5 * ROW_L + _PAD, 4),
    ("spine2", 2, 75.0 + 0.5 * ROW_L + _PAD, 2),
    ("spine1", 1, 0.0 + 0.5 * ROW_L + _PAD, 4),
    ("rear_girdle", 0, -(0.5 * ROW_L + _PAD), 4),
]

#: The three spine joints, at the proximal end of each segment.
JOINT_X = (0.0, 75.0, 140.0)
#: Rigid body extent along x, from the joints and the rows they carry.
BODIES = {
    0: (-(ROW_L + 2 * _PAD), 0.0),
    1: (0.0, 75.0),
    2: (75.0, 140.0),
    3: (140.0, 140.0 + 2 * ROW_L + 4 * _PAD),
}


def _hw(name):
    return ROWS[dict((n, k) for n, _b, _x, k in LAYOUT)[name]][1] + WALL


def _sections():
    """(x, half-width) control points, chest to tail, for the lofted skin."""
    pts = []
    for (name, _b, x, n) in LAYOUT:
        pts.append((x, ROWS[n][1] + WALL))
    pts.sort()
    lo = BODIES[0][0]
    hi = BODIES[3][1]
    pts = ([(lo, pts[0][1] * 0.86)] + pts + [(hi, pts[-1][1] * 0.90)])
    return pts


#: ✅ **The DORSAL line, and it is what makes the silhouette a cat.**
#: Scaling both semi-axes together gives an hourglass -- the back caves in at the
#: waist exactly as much as the belly rises, which no animal does. A cat's back
#: runs on and its BELLY tucks up. So the section hangs from a fixed dorsal line
#: instead of being centred: `z_centre(x) = Z_DORSAL - half_height(x)`.
#: The chest half-height sets it, so the chest is unchanged and everything
#: narrower rides up.
Z_DORSAL = Z0 + 40.5 * ASPECT


def _zc(x):
    """Vertical centre of the section at x -- hung from the dorsal line."""
    return Z_DORSAL - _hw_at(x) * ASPECT


def _ell(x, hw):
    return Plane(origin=(x, 0, _zc(x)), z_dir=(1, 0, 0)) * Ellipse(hw * ASPECT, hw)


def _outer(body: int):
    """The solid barrel for one rigid body, before hollowing."""
    x0, x1 = BODIES[body]
    pts = _sections()
    inner_x = [x for (x, _h) in pts if x0 < x < x1]
    xs = [x0] + inner_x + [x1]
    return loft([_ell(x, _hw_at(x)) for x in xs])


def body_shell(body: int):
    """One rigid body's barrel, lofted through the sections it spans."""
    x0, x1 = BODIES[body]
    pts = _sections()
    inner_x = [x for (x, _h) in pts if x0 < x < x1]
    hollow = loft([_ell(x0 - 1, _hw_at(x0) - WALL)]
                  + [_ell(x, _hw_at(x) - WALL) for x in inner_x]
                  + [_ell(x1 + 1, _hw_at(x1) - WALL)])
    return _outer(body) - hollow


def motors(body: int):
    """(x, y, z, r) of every motor bore assigned to this rigid body."""
    out = []
    for (name, b, x, n) in LAYOUT:
        if b != body:
            continue
        for (y, z) in ROWS[n][0]:
            out.append((x, y, _zc(x) + z, R))
    return out


def _hw_at(x):
    """The shell's OUTER half-width at x, interpolated between the row sections."""
    pts = _sections()
    for (a, ha), (b, hb) in zip(pts[:-1], pts[1:]):
        if a - 1e-9 <= x <= b + 1e-9:
            t = 0.0 if b == a else (x - a) / (b - a)
            return ha + (hb - ha) * t
    return pts[-1][1]


def _bulkhead_x(body: int):
    """Where the bulkheads go, deduplicated.

    ⚠️ Two rows in one body put their facing bulkheads **1 mm apart** -- two
    full sections of material doing one job. Anything closer than a bulkhead
    thickness is one bulkhead.
    """
    x0, x1 = BODIES[body]
    xs = []
    for (_name, b, x, n) in LAYOUT:
        if b != body:
            continue
        for sx in (-1.0, 1.0):
            xb = x + sx * (ROW_L / 2 + BULKHEAD_T / 2)
            xs.append((min(max(xb, x0 + BULKHEAD_T), x1 - BULKHEAD_T), n))
    xs.sort()
    keep = []
    for (xb, n) in xs:
        if keep and abs(xb - keep[-1][0]) < 2 * BULKHEAD_T:
            continue
        keep.append((xb, n))
    return keep


def bulkheads(body: int):
    """A full section behind each row -- what the motors bolt to and what ties
    the barrel across the span the hip bosses load.

    ✅ Cut as the INTERSECTION of the solid barrel with a slab, so a bulkhead
    is exactly the section at that station and cannot float. ⚠️ Sizing one from
    an interpolated half-width does not work: `loft` fits a surface through the
    control sections, it does not interpolate them linearly, so a bulkhead cut
    to the arithmetic mean sits inside the wall by a fraction of a millimetre and
    the body comes apart.
    """
    caps = []
    solid = _outer(body)
    for (xb, n) in _bulkhead_x(body):
        slab = Pos(xb, 0, Z_DORSAL) * Box(BULKHEAD_T, 400, 400)
        cap = solid & slab
        row = [r for (_nm, b, x, r) in LAYOUT
               if b == body and abs(x - xb) < ROW_L]
        for k in set(row):
            for (y, z) in ROWS[k][0]:
                cap -= (Pos(xb, y, _zc(xb) + z)
                        * (Rot(0, 90, 0)
                           * Cylinder(TP.SPOOL_D / 2 + 1.0, BULKHEAD_T + 4)))
        caps.append(cap)
    return caps


def _unused_bulkheads(body: int):
    x0, x1 = BODIES[body]
    caps = []
    for (name, b, x, n) in LAYOUT:
        if b != body:
            continue
        for sx in (-1.0, 1.0):
            xb = x + sx * (ROW_L / 2 + BULKHEAD_T / 2)
            xb = min(max(xb, x0 + BULKHEAD_T), x1 - BULKHEAD_T)
            # ⚠️ Sized to the SHELL's local inner surface, not to the row's own
            # section. The shell tapers between rows, so a bulkhead cut to the
            # row's width floats inside the barrel wherever the taper has moved
            # on -- which left the waist and the chest in three pieces each,
            # while rendering perfectly.
            hw = _hw_at(xb) - WALL / 2
            cap = loft([_ell(xb - BULKHEAD_T / 2, hw), _ell(xb + BULKHEAD_T / 2, hw)])
            for (y, z) in ROWS[n][0]:
                cap -= (Pos(xb, y, Z0 + z)
                        * (Rot(0, 90, 0)
                           * Cylinder(TP.SPOOL_D / 2 + 1.0, BULKHEAD_T + 4)))
            caps.append(cap)
    return caps


def hip_bosses(body: int):
    """Hips live on the two girdle bodies only."""
    if body not in (0, 3):
        return []
    x = 0.0 if body == 0 else 195.0
    hw = _hw("rear_girdle" if body == 0 else "front_girdle")
    out = []
    boss_r = HIP_OD / 2 + BOSS_WALL
    for side in (+1.0, -1.0):
        y_hip = side * TRACK
        inner = hw - WALL
        span = max(abs(y_hip) - inner, 1.0)
        # ⚠️ The REAR hip sits exactly on the first spine joint (x = 0), so a
        # boss centred there is half in the next rigid body. It reaches back
        # into its own girdle instead; the tongue stays at x = 0, where the
        # kinematics puts the hip.
        xa = x + (-boss_r if body == 0 else 0.0)
        arm = Pos(xa, side * (inner + span / 2), 0) * Box(2 * boss_r, span,
                                                          2 * boss_r)
        axis = Plane(origin=(x, y_hip, 0), z_dir=(0, side, 0)).location
        lug = axis * (Cylinder(boss_r, TONGUE_T)
                      - Cylinder(HIP_BORE / 2, TONGUE_T + 2))
        out += [arm, lug]
    return out


def rigid_body(body: int):
    g = body_shell(body)
    for c in bulkheads(body):
        g += c
    for h in hip_bosses(body):
        g += h
    return g


def trunk():
    return Compound([rigid_body(b) for b in sorted(BODIES)])


def report():
    mot = math.pi * R * R * TP.MOTOR_L + math.pi * (TP.SPOOL_D / 2) ** 2 * TP.SPOOL_L
    total_v = 0.0
    print("%-14s %6s %5s %9s %8s %9s" % ("rigid body", "x span", "mot",
                                         "vol cm3", "solids", "nylonCF"))
    ok = True
    for b in sorted(BODIES):
        g = rigid_body(b)
        sol = g.solids()
        v = sum(s.volume for s in sol)
        total_v += v
        n = len(motors(b))
        x0, x1 = BODIES[b]
        if len(sol) != 1:
            ok = False
        print("%-14s %6s %5d %9.1f %8s %9.1f g"
              % ("body %d" % b, "%.0f-%.0f" % (x0, x1), n, v / 1000.0,
                 "%d %s" % (len(sol), "" if len(sol) == 1 else "***"),
                 v * 1.2e-3))
    # every motor must lie inside its own body
    for b in sorted(BODIES):
        x0, x1 = BODIES[b]
        for (x, _y, _z, _r) in motors(b):
            if not (x0 - 1e-6 <= x - ROW_L / 2 and x + ROW_L / 2 <= x1 + 1e-6):
                print("  *** a motor at x=%.1f straddles body %d (%.0f..%.0f)"
                      % (x, b, x0, x1))
                ok = False

    # ⚠️ no two motors may interpenetrate
    allm = [(x, y, z) for b in BODIES for (x, y, z, _r) in motors(b)]
    for i in range(len(allm)):
        for j in range(i + 1, len(allm)):
            xi, yi, zi = allm[i]
            xj, yj, zj = allm[j]
            if abs(xi - xj) < ROW_L - 1e-6 and math.hypot(yi - yj, zi - zj) < 2 * R:
                print("  *** motors overlap: (%.0f,%.0f,%.0f) and (%.0f,%.0f,%.0f)"
                      % (xi, yi, zi, xj, yj, zj))
                ok = False

    n_mot = sum(len(motors(b)) for b in BODIES)
    lo, hi = BODIES[0][0], BODIES[3][1]
    pts = _sections()
    xs = np.linspace(lo, hi, 400)

    def hw_at(x):
        for (a, ha), (b2, hb) in zip(pts[:-1], pts[1:]):
            if a - 1e-9 <= x <= b2 + 1e-9:
                t = 0.0 if b2 == a else (x - a) / (b2 - a)
                return ha + (hb - ha) * t
        return pts[-1][1]

    env = sum(math.pi * hw_at(x) * hw_at(x) * ASPECT for x in xs) * (hi - lo) / len(xs)
    dorsal = [Z_DORSAL for x in xs]
    belly = [_zc(x) - hw_at(x) * ASPECT for x in xs]
    print()
    print("  dorsal line      flat at z = %.1f mm above the hip axis" % Z_DORSAL)
    print("  belly            %.1f mm at the chest -> %.1f at the waist  (tuck-up %.1f)"
          % (min(belly), max(belly), max(belly) - min(belly)))
    print("  motors           %d of 18" % n_mot)
    print("  trunk length     %.0f mm   chest %.1f w x %.1f h   waist %.1f x %.1f"
          % (hi - lo, 2 * _hw("front_girdle"), 2 * _hw("front_girdle") * ASPECT,
             2 * _hw("spine2"), 2 * _hw("spine2") * ASPECT))
    print("  envelope         %.0f cm3" % (env / 1000.0))
    print("  motors solid     %.0f cm3 = %.1f %% of the envelope"
          % (n_mot * mot / 1000.0, 100 * n_mot * mot / env))
    print("  structure        %.0f cm3 -> %.0f g nylon-CF"
          % (total_v / 1000.0, total_v * 1.2e-3))
    print("  %s" % ("all bodies are one part each" if ok else "*** SEE ABOVE ***"))
    return ok


def render_png(g, path, elev=16, azim=-62):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(12, 6))
    ax = fig.add_subplot(111, projection="3d")
    for i, s in enumerate(g if isinstance(g, list) else [g]):
        verts, tris = s.tessellate(0.3)
        V = np.array([[v.X, v.Y, v.Z] for v in verts])
        coll = Poly3DCollection(V[np.array(tris)],
                                facecolor=["#8a9bb0", "#9fb0c4"][i % 2],
                                edgecolor="#3c4a5a", linewidth=0.08)
        coll.set_zsort("average")
        ax.add_collection3d(coll)
    whole = Compound(g if isinstance(g, list) else [g])
    bb = whole.bounding_box()
    ax.set_xlim(bb.min.X - 5, bb.max.X + 5)
    ax.set_ylim(bb.min.Y - 5, bb.max.Y + 5)
    ax.set_zlim(bb.min.Z - 5, bb.max.Z + 5)
    ax.set_box_aspect((bb.size.X, bb.size.Y, bb.size.Z))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(path, dpi=140, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", path)


if __name__ == "__main__":
    report()
    parts = [rigid_body(b) for b in sorted(BODIES)]
    render_png(parts, os.path.join(HERE, "tomcat_trunk.png"))
    render_png(parts, os.path.join(HERE, "tomcat_trunk_side.png"), elev=2, azim=-90)
