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
import g3_flexure as G3F                                         # noqa: E402
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
#: ⚠️ **M120: G3 is in the row.** ADR-0051 put the series spring between
#: rotor and spool and ADR-0107 made it a flexure, but no row had ever been
#: longer than motor + spool. It is now, by the flexure plus its clearances --
#: 3.66 mm on a leg row, 2.43 on a spine row -- and the bodies are NOT
#: lengthened: `BODIES` below still sizes on `ROW_L`, and `report()` checks
#: the longer rows fit inside them with their bulkhead pads. The tail's motor
#: shares a hind row and gets the hind length; it has no G3 of its own.
G3_STACK = {r: G3F.stack("spine" if r == "spine" else "leg")
            for r in ("hind", "fore", "spine")}


def row_len(role: str) -> float:
    """A row's length along x: motor, G3 and spool, mm."""
    return ROW_L + G3_STACK.get(role, 0.0)
Z0 = SP.girdle_offset_z * MM

#: Solved symmetric layouts: (centres, outer half-width) for a row of N.
#: `[derived: scratch solver, mirror-symmetric, minimal 1.40-aspect ellipse]`
#: ⚠️ **A girdle row has to split left/right; a spine row does not.** The
#: diamond is the tightest four (80.9 x 113.3) but two of its members sit on the
#: centreline, so it cannot be dealt three-and-three to the two legs it drives.
#: Girdle rows are mirrored PAIRS instead, which costs 8 % of section:
#:
#:     4 diamond          80.9 x 113.3   cannot split
#:     4 mirrored pairs   87.6 x 122.7   splits 2 + 2
#:     2 mirrored pair    81.0 x 113.4   splits 1 + 1
#:     2 over-under       64.8 x  90.7   cannot split -- the waist keeps it
ROWS = {
    "diamond4": ([(0.0, 35.1), (-20.2, 0.0), (20.2, 0.0), (0.0, -35.1)], 40.5),
    "pairs4":   ([(-20.2, 20.2), (20.2, 20.2),
                  (-20.2, -20.2), (20.2, -20.2)], 43.8),
    "pair2":    ([(-20.2, 0.0), (20.2, 0.0)], 40.5),
    "stack2":   ([(0.0, 20.3), (0.0, -20.3)], 32.4),
    # ✅ **The tail's motor, and it needs no new row.** ADR decision F bought
    # **19** motors -- 12 leg, 3 spine pitch, 3 spine yaw and 1 TAIL -- and M88's
    # redistribution placed 18: the tail's fell out and nothing counted it back.
    # Body 0 is 104.6 mm long and two rows already use 88.2, so a third ROW does
    # not fit; a third POSITION does. On the centreline it must clear the pair by
    # 2R, so |z| >= 28.0, and at 28.5 it reaches 45.8 against a 56.7 half-height.
    "tri3":     ([(-20.2, 0.0), (20.2, 0.0), (0.0, 28.5)], 40.5),
    # ✅ **M123 (ADR-0114): the hip-station rows.** A knee or ankle motor sits
    # AT HIP HEIGHT -- the bottom pair on the hip axis (z = 0) -- so its spool's
    # two leads leave its top and bottom 8.75 mm either side of the axis and run
    # straight along it. `pairs4`'s bottom pair sat 3.4 mm low, which put one
    # lead 13.2 mm off the axis and bent its conduit to 12.4 mm radius on its
    # way round to the hub. Raised 3.8 mm and the top pair 0.9 (the pairs keep
    # 37.5 mm between centres, 2R + CLR), the section grows 0.3 mm. Sized on
    # the same circle `pairs4`'s 43.8 implies (r 20.30); the section hangs
    # from the dorsal line, so every hip-station row takes this half-width
    # whatever it holds: `hip2` without the top pair, `hip2top1` with the
    # tail's motor on the centreline above.
    "hip4":     ([(-20.2, 21.1), (20.2, 21.1), (-20.2, -16.4), (20.2, -16.4)], 44.09),
    "hip2":     ([(-20.2, -16.4), (20.2, -16.4)], 44.09),
    "hip2top1": ([(-20.2, -16.4), (20.2, -16.4), (0.0, 21.1)], 44.09),
}

#: ⚠️ **A row can hold motors of more than one ROLE, and the check counted
#: by row.** `tri3` carries two hind-leg motors and the TAIL's, and reading
#: the row's role for all three made the drive-train check report "7 motors
#: on body 0" for a leg that needs 6. Position index -> role, where it is not
#: the row's own.
POSITION_ROLE = {("tri3", 2): "tail", ("hip2top1", 2): "tail"}


#: One row per rigid body, chest to tail. The **waist** is the two-motor row --
#: a cat is pinched between the ribcage and the pelvis, and that is where the
#: two motors that do not make a fourth diamond belong.
#: ⚠️ Rows are placed from each body's PROXIMAL joint, not centred on a
#: landmark. The first layout centred the rear row on x=0 -- which is the joint
#: -- so half of it sat in the next body, and put two rows 27.5 mm apart in one
#: body when a row is 44.1 long, so the motors interpenetrated. `report()`
#: checks both now.
_PAD = BULKHEAD_T + 0.5

#: The three spine joints, at the proximal end of each segment.
JOINT_X = (0.0, 75.0, 140.0)

#: ⚠️ **Two barrels butt-jointed cannot bend.** At the ROM ADR-0006 specifies
#: -- +/-25 deg dorsoventral, +/-15 lateral -- a flat end face closes on its
#: neighbour by `half_height * tan(25)`, which is **23-27 mm** at full section.
#: A cat solves it the way this does: the trunk necks down where it bends.
NECK = 0.52
#: Gap between two bodies at a joint, and how far the neck reaches either side.
JOINT_GAP = 14.0
NECK_SPAN = 15.0

#: Rows sit clear of the neck zones, one per rigid body except the chest.
#: ⚠️ Rows are CENTRED in their own rigid body. Measuring them from a joint
#: instead put a 44.1 mm row into a 51 mm body starting 32 mm in, and it hung out
#: the far end -- `report()` caught it, the render did not.
#: ⚠️ **A leg's motors must be on the leg's OWN rigid body.** The first
#: distribution allocated by VOLUME alone -- 4/4/2/8 -- and that is not a drive
#: train: the hind legs need six motors and their girdle had four, while a leg
#: cable spooled on any other body crosses a spine joint, so bending the spine
#: would drive the leg. `report()` checks the roles now.
#:
#: 12 leg motors on the two girdle bodies, 6 spine motors beside the joints they
#: drive: body 1 reaches joints 0 and 1, body 2 reaches joint 2.
LAYOUT = [
    # (name, body, x fraction in that body -- or "front"/"rear" of its HIP,
    #  layout, role)
    # ✅ M123 (ADR-0114): each girdle is TWO rows straddling its hip, spools
    # facing across it. Front of the hip: the hip motors (top pair) and one of
    # knee/ankle (bottom pair); behind it: the other (bottom pair), and on the
    # rear girdle the tail's motor.
    ("fore_f", 3, "front", "hip4", "fore"),
    ("fore_r", 3, "rear", "hip2", "fore"),
    ("spine_c", 2, 0.50, "stack2", "spine"),
    ("spine_b", 1, 0.50, "diamond4", "spine"),
    ("hind_f", 0, "front", "hip4", "hind"),
    ("hind_r", 0, "rear", "hip2top1", "hind"),
]

#: Rigid body extent along x. Each end retreats from its joint by half the gap.
#: ⚠️ The rear girdle now carries TWO rows, so it reaches further behind the
#: hip than a pelvis should. That is the cost of ADR-0006 rooting the spine
#: chain at the hip station: everything driving the hind legs has to live behind
#: x = 0. Moving the first joint forward -- the sacrum-into-the-pelvis fix
#: already named in `yoke()` -- is what shortens it.  `[owed]`
#: ✅ **M122 (ADR-0112): the hind hips are 30 mm BEHIND spine joint 0.**
#: The `[owed]` above, paid: the rear girdle now reaches past its hips to the
#: first joint, so the hips sit in full section rather than on the neck, and
#: the space between the hip and the joint is free for the knee and ankle
#: cables to reach the hip axis. Body 0 grows by the offset; the hip-to-row
#: geometry behind the hip is exactly what it was (`HIP_ROW_GAP`).
#:
#: ✅ **M123 (ADR-0114): each girdle is its hip station.** Two rows straddle
#: each hip, their spools facing across it with ONE bulkhead between them at
#: the hip, so a knee or ankle spool's two leads run straight along the hip
#: axis into the hollow hip. The row in front of the hind hip must end clear
#: of spine joint 0's neck, and the row behind the fore hip start clear of
#: joint 2's: that puts the hind hips 65 mm behind joint 0 and the fore 10 mm
#: ahead of the front girdle's origin. Each body is now exactly its two rows.
REAR_HIP_X = SP.rear_hip_x * MM
FRONT_HIP_X = (float(sum(SP.segment_lengths)) + SP.front_hip_x) * MM
HIP_X_OF = {0: REAR_HIP_X, 3: FRONT_HIP_X}
#: A hip-station row's near end is this far from its hip: half the shared
#: bulkhead and a 0.5 mm gap.
HIP_ROW_NEAR = BULKHEAD_T / 2 + 0.5
#: Where a knee or ankle spool's plane sits, either side of its hip, mm.
HIP_SPOOL_DX = HIP_ROW_NEAR + TP.SPOOL_L / 2

BODIES = {
    0: (REAR_HIP_X - HIP_ROW_NEAR - row_len("hind") - _PAD, -JOINT_GAP / 2),
    1: (JOINT_GAP / 2, 75.0 - JOINT_GAP / 2),
    2: (75.0 + JOINT_GAP / 2, 140.0 - JOINT_GAP / 2),
    3: (140.0 + JOINT_GAP / 2, FRONT_HIP_X + HIP_ROW_NEAR + row_len("fore") + _PAD),
}


def _row_x(body, frac, role):
    x0, x1 = BODIES[body]
    L = row_len(role)
    return x0 + L / 2 + _PAD + frac * max((x1 - x0) - L - 2 * _PAD, 0.0)


def _rows():
    """(name, body, absolute x, motors) with the fractions resolved.

    M123: a girdle's rows are placed from its HIP ("front"/"rear"), their near
    ends `HIP_ROW_NEAR` from it."""
    out = []
    for (nm, b, f, n, r) in LAYOUT:
        if isinstance(f, str):
            sgn = 1.0 if f == "front" else -1.0
            x = HIP_X_OF[b] + sgn * (HIP_ROW_NEAR + row_len(r) / 2)
        else:
            x = _row_x(b, f, r)
        out.append((nm, b, x, n, r))
    return out


def _hw(name):
    return ROWS[dict((n, k) for n, _b, _x, k, _r in _rows())[name]][1] + WALL


def _sections():
    """(x, half-width) control points, chest to tail, for the lofted skin.

    ✅ A **neck** at every joint. Without it the barrels cannot reach the ROM,
    and with it the waist lands where a cat's waist is.
    """
    pts = [(x, ROWS[n][1] + WALL) for (_nm, _b, x, n, _r) in _rows()]
    pts.sort()
    lo, hi = BODIES[0][0], BODIES[3][1]
    pts = [(lo, pts[0][1] * 0.86)] + pts + [(hi, pts[-1][1] * 0.90)]

    def lerp(x):
        for (a2_, ha), (b2_, hb) in zip(pts[:-1], pts[1:]):
            if a2_ - 1e-9 <= x <= b2_ + 1e-9:
                t = 0.0 if b2_ == a2_ else (x - a2_) / (b2_ - a2_)
                return ha + (hb - ha) * t
        return pts[-1][1]

    for jx in JOINT_X:
        for dx in (-NECK_SPAN, 0.0, NECK_SPAN):
            f = NECK if dx == 0.0 else 0.5 * (1.0 + NECK)
            pts.append((jx + dx, lerp(jx + dx) * f))
    pts.sort()
    return pts


#: ✅ **The DORSAL line, and it is what makes the silhouette a cat.**
#: Scaling both semi-axes together gives an hourglass -- the back caves in at the
#: waist exactly as much as the belly rises, which no animal does. A cat's back
#: runs on and its BELLY tucks up. So the section hangs from a fixed dorsal line
#: instead of being centred: `z_centre(x) = Z_DORSAL - half_height(x)`.
#: The chest half-height sets it, so the chest is unchanged and everything
#: narrower rides up.
Z_DORSAL = Z0 + 40.5 * ASPECT

#: ✅ **The spine axis, one moment arm below the back.** `SpineParams` owns it
#: so the CAD and the MJCF cannot drift; the derivation is there. The tip of a
#: dorsal process therefore lands ON the dorsal line, which is what a cat's back
#: is made of.
SPINE_Z = float(SP.spine_axis_z) * MM


def _row_hw_at(x):
    """Half-width interpolated over the ROW sections only, ignoring the necks.

    ⚠️ The centreline must not follow the neck. Hanging it from the dorsal line
    with the neck included pinched the section at the joint AND dropped its
    centre, so the belly leapt 62 mm at the waist. The body's mid-line runs
    smoothly through a joint; only its RADIUS pinches.
    """
    pts = [(x2, ROWS[n][1] + WALL) for (_nm, _b, x2, n, _r) in _rows()]
    pts.sort()
    lo, hi = BODIES[0][0], BODIES[3][1]
    pts = [(lo, pts[0][1] * 0.86)] + pts + [(hi, pts[-1][1] * 0.90)]
    for (a2_, ha), (b2_, hb) in zip(pts[:-1], pts[1:]):
        if a2_ - 1e-9 <= x <= b2_ + 1e-9:
            t = 0.0 if b2_ == a2_ else (x - a2_) / (b2_ - a2_)
            return ha + (hb - ha) * t
    return pts[-1][1]


def _pinch(x):
    """0 away from a joint, 1 at one -- how far the section has necked down."""
    best = 0.0
    for jx in JOINT_X:
        d = abs(x - jx)
        if d < NECK_SPAN:
            best = max(best, 1.0 - d / NECK_SPAN)
    return best


def _zc(x):
    """Vertical centre of the section at x.

    Away from a joint it hangs from the dorsal line, which is what tucks the
    belly up. ⚠️ **At a joint it blends to the SPINE AXIS**, because that is
    where the vertebra is. Necking about the barrel's own centreline instead
    lifted the belly to z = 0.3 mm at the waist -- the joint ended up level with
    the underside of the body, and the cross would have hung outside it.

    ✅ So the structure narrows to a bead on the spine at every joint and the
    silhouette is not continuous. It is not supposed to be: a cat's back runs
    smooth over vertebrae that do exactly this, and the skin is what carries the
    form.
    """
    f = _pinch(x)
    return (Z_DORSAL - _row_hw_at(x) * ASPECT) * (1.0 - f) + SPINE_Z * f


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
    for (name, b, x, n, _r) in _rows():
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


#: Where an end bulkhead's centre is clamped: flush with the body's end.
_END_BH = BULKHEAD_T / 2 + 0.05


def _bulkhead_x(body: int):
    """Where the bulkheads go, deduplicated.

    ⚠️ Two rows in one body put their facing bulkheads **1 mm apart** -- two
    full sections of material doing one job. Anything closer than a bulkhead
    thickness is one bulkhead.
    """
    x0, x1 = BODIES[body]
    xs = []
    for (_name, b, x, n, _r) in _rows():
        if b != body:
            continue
        for sx in (-1.0, 1.0):
            xb = x + sx * (row_len(_r) / 2 + BULKHEAD_T / 2)
            # ⚠️ M122: clamped to `x0 + BULKHEAD_T` the end bulkhead spanned
            # 0.8-2.4 mm in from the body's end, and `_PAD` starts the row at
            # 2.1 -- 0.3 mm of bulkhead inside the row. Harmless while rows
            # had 8 mm to spare; G3 (M120) filled them, and the hind ankle's
            # spool (M122) sits at exactly that end. Flush with the end now,
            # 0.05 in so the slab never shares the loft's end face.
            xs.append((min(max(xb, x0 + _END_BH), x1 - _END_BH), n))
    xs.sort()
    keep = []
    for (xb, n) in xs:
        if keep and abs(xb - keep[-1][0]) < 2 * BULKHEAD_T:
            # M123: the two rows at a hip share one, centred between them
            keep[-1] = (0.5 * (xb + keep[-1][0]), keep[-1][1])
            continue
        keep.append((xb, n))
    return keep


#: Which girdle body drives which pair of legs.
LEG_BODY = {"hind": 0, "fore": 3}


def leg_spools(role: str, side: float, by: str = "height"):
    """The three spool centres driving ONE leg, as (x, y, z), hip/knee/ankle.

    ✅ This is what makes the drive train real rather than a volume argument.
    A girdle row is a set of mirrored PAIRS, so the six motors on a girdle deal
    three and three to the two legs it carries: `side > 0` takes the +y half.

    `by="height"`: dorsal first -- the order M88 dealt them, hip on top.
    `by="tendon"`: hip, knee, ankle as `tendon_exit.DRIVE` assigns them.

    ⚠️ **M122: "ordered by height" was an assignment nobody had checked.** It
    put the hip on the top spool because the hip "needs the longest cable",
    and it gave each (x, z) to a 2-D router as if the spool sat in the leg's
    plane. With the spools where they are -- inside, winding across the body
    -- the ankle cable can only reach its leg from above the hip, and only the
    top motor's spool can put an idler there (ADR-0112).
    """
    body = LEG_BODY[role]
    out = []
    for (_nm, b, x, layout, r) in _rows():
        if b != body or r != role:
            continue
        for (y, z) in ROWS[layout][0]:
            if y * side > 0.0:
                out.append((x, y, _zc(x) + z))
    if len(out) != 3:
        raise ValueError("%s side %+.0f got %d spools, not 3"
                         % (role, side, len(out)))
    out = sorted(out, key=lambda p: -p[2])
    if by == "height":
        return out
    import tendon_exit as TE
    return [out[TE.DRIVE[role][t][0]] for t in ("hip", "knee", "ankle")]


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
        row = [r for (_nm, b, x, r, _ro) in _rows()
               if b == body and abs(x - xb) < row_len(_ro)]
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
    for (name, b, x, n, _r) in _rows():
        if b != body:
            continue
        for sx in (-1.0, 1.0):
            xb = x + sx * (row_len(_r) / 2 + BULKHEAD_T / 2)
            xb = min(max(xb, x0 + _END_BH), x1 - _END_BH)
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
    """Hips live on the two girdle bodies only.

    ✅ **M123 (ADR-0113): each hip is a hollow STUB** on the hip axis, from
    inside the girdle's wall out past the femur hub's outer bearing
    (`LD.STUB_END`), with a flange where it passes the wall. The femur's hub
    turns on it; the knee and ankle conduits run out through its bore. It
    replaces M122's boss arm and the tongue a 6 mm shaft ran through."""
    if body not in (0, 3):
        return []
    x = HIP_X_OF[body]
    od, idd = LD.HIP_STUB
    out = []
    for side in (+1.0, -1.0):
        inner = _hw_at(x) - WALL
        y0, y1 = inner - 3.0, TRACK + LD.STUB_END
        axis = Plane(origin=(x, side * y0, 0), z_dir=(0, side, 0)).location
        tube = Pos(0, 0, (y1 - y0) / 2) * (Cylinder(od / 2, y1 - y0)
                                           - Cylinder(idd / 2, y1 - y0 + 2))
        # the flange: through the wall, bonded to it
        fl_t = WALL + 3.0 + 1.0
        flange = Pos(0, 0, fl_t / 2) * (Cylinder(od / 2 + 6.0, fl_t)
                                        - Cylinder(idd / 2, fl_t + 2))
        out += [axis * tube, axis * flange]
    return out


def hip_bores(body: int):
    """The stub's bore through the shell: what the conduits pass."""
    if body not in (0, 3):
        return []
    x = HIP_X_OF[body]
    out = []
    for side in (+1.0, -1.0):
        y0 = _hw_at(x) - WALL - 4.0
        L = _hw_at(x) + 4.0 - y0
        out.append(Plane(origin=(x, side * y0, 0), z_dir=(0, side, 0)).location
                   * (Pos(0, 0, L / 2) * Cylinder(LD.HIP_STUB[1] / 2, L)))
    return out


#: Cross-joint hardware. Bore and pin follow the ankle bearing the leg already
#: specs, which is the smallest the project has qualified.
PIN_D = LD.BEARING["ankle"][0]
HUB_R = 6.0
YOKE_T = 3.0
#: Cable posts: the dorsoventral pair at ADR-0006's 30 mm arm and the lateral
#: pair at ADR-0009's 20 mm. ✅ These are a vertebra's spinous and transverse
#: processes, arrived at from the moment arms rather than from the anatomy.
POST_R = 3.0
DORSAL_ARM = float(SP.joint_moment_arm[0]) * MM
LATERAL_ARM = float(SP.lateral_moment_arm[0]) * MM


def yoke(jx: float, distal: bool):
    """The fork one body presents to the cross.

    ⚠️ **The spine joint sits at z = 0, the hip axis** -- where `mjcf_tendon`
    puts it -- not on the barrel's centreline. In an animal the vertebral column
    is DORSAL and the belly hangs from it; here it runs along the underside.
    Moving it is a kinematics change (it shifts every CoM and the sway lever), so
    the hardware is built where the model says and the conflict is named.
    `[owed]`

    The proximal fork straddles in z and carries the z pin, so it yaws; the
    distal fork straddles in y and carries the y pin, so it pitches. Two
    orthogonal pins through one hub is a Cardan joint, which is the 2 DOF
    ADR-0006 asks for and nothing more.
    """
    sgn = +1.0 if distal else -1.0
    reach = JOINT_GAP / 2 + HUB_R + 2.0
    arms = []
    off = HUB_R + YOKE_T / 2 + 0.4
    for s in (-1.0, +1.0):
        c = (0.0, s * off, 0.0) if distal else (0.0, 0.0, s * off)
        size = ((reach, YOKE_T, 2 * HUB_R) if distal
                else (reach, 2 * HUB_R, YOKE_T))
        arm = Pos(jx + sgn * reach / 2, c[1], SPINE_Z + c[2]) * Box(*size)
        arms.append(arm)
    fork = arms[0] + arms[1]
    axis = (Rot(90, 0, 0) if distal else Rot(0, 0, 0))
    return fork - (Pos(jx, 0, SPINE_Z) * (axis * Cylinder(PIN_D / 2, 4 * off)))


def cross(jx: float):
    """The centre piece: a hub with two orthogonal pins."""
    off = HUB_R + YOKE_T / 2 + 0.4
    hub = Pos(jx, 0, SPINE_Z) * Box(2 * HUB_R, 2 * HUB_R, 2 * HUB_R)
    pin_y = Pos(jx, 0, SPINE_Z) * (Rot(90, 0, 0) * Cylinder(PIN_D / 2 - 0.05,
                                                            2 * (off + YOKE_T)))
    pin_z = Pos(jx, 0, SPINE_Z) * Cylinder(PIN_D / 2 - 0.05,
                                           2 * (off + YOKE_T))
    return hub + pin_y + pin_z


def processes(jx: float, distal: bool):
    """Spinous and transverse posts, where the spine cables get their arm."""
    sgn = +1.0 if distal else -1.0
    x = jx + sgn * (JOINT_GAP / 2 + POST_R)
    out = []
    # ⚠️ The stem used to end exactly ON the body's end face. Coplanar faces are
    # the classic OCC boolean failure and it did not error -- it returned a shape
    # whose volume had collapsed from 37,293 to 509 mm3, and only `report()`'s
    # solid count and the volume column showed it. The stem reaches 4 mm INTO
    # the body now, so the faces cross instead of touching.
    BITE = 4.0
    for (y, z) in ((0.0, DORSAL_ARM), (0.0, -DORSAL_ARM),
                   (LATERAL_ARM, 0.0), (-LATERAL_ARM, 0.0)):
        stem = Pos(jx + sgn * (JOINT_GAP / 4 + BITE / 2), y * 0.5,
                   SPINE_Z + z * 0.5) * Box(
            JOINT_GAP / 2 + BITE, max(2 * POST_R, abs(y)),
            max(2 * POST_R, abs(z)))
        post = Pos(x, y, SPINE_Z + z) * (Rot(90, 0, 0)
                                        * Cylinder(POST_R, 2 * POST_R))
        out += [stem, post]
    return out


def joint_parts(body: int):
    """Yokes and processes belonging to this rigid body."""
    x0, x1 = BODIES[body]
    out = []
    for jx in JOINT_X:
        if abs(jx - x1) < JOINT_GAP:            # this body is PROXIMAL to it
            out.append(yoke(jx, distal=False))
            out += processes(jx, distal=False)
        if abs(jx - x0) < JOINT_GAP:            # this body is DISTAL to it
            out.append(yoke(jx, distal=True))
            out += processes(jx, distal=True)
    return out


#: Cable clearance left around a process post, mm.
POST_CLEAR = 1.5


def _fuse(g, part, what):
    """Union, with the one invariant a union cannot break: it cannot SHRINK.

    ⚠️ **This is the check that was missing, and it cost a whole rigid body.**
    Fusing the rear joint's ventral post into body 0 took it from **114,889 mm3
    to 0.0 with zero solids** -- the barrel was annihilated -- and OCC raised
    nothing. Later parts re-fused into one small solid, so `report()`'s
    "all bodies are one part each" was TRUE of a body that no longer existed,
    and the published structure mass was computed from the wreckage.

    The trigger is a tangential boolean: the post is a 6 mm cylinder that grazes
    a 1.2 mm lofted wall, entering it by 9 % of its volume. `intersect` on the
    same pair is wrong in the other direction -- it answers 169.6 mm3, the WHOLE
    post, where point sampling says ~15. Neither call fails; both just lie.

    So the volume is checked after every fuse. A shrink is a defect, not a
    tolerance, and it stops the build.
    """
    v0 = sum(s.volume for s in g.solids())
    out = g + part
    sol = out.solids()
    v1 = sum(s.volume for s in sol)
    if v1 < v0 - 1e-6:
        raise RuntimeError(
            "fusing %s DESTROYED volume: %.1f -> %.1f mm3, %d solids. "
            "A union cannot shrink; the boolean failed silently."
            % (what, v0, v1, len(sol)))
    return out


def process_clearance(body: int):
    """The void a spine cable needs to get around each process post.

    ✅ A post is a cable-wrap cylinder standing off the joint; the cable has to
    pass BETWEEN it and the body, so the wall must be open there. Cutting that
    clearance before the post is fused also removes the grazing contact that
    `_fuse` catches -- the post then meets a void, not a tangent wall. The
    clearance is a design requirement first and a boolean fix second.
    """
    x0, x1 = BODIES[body]
    out = []
    for jx in JOINT_X:
        for distal in (False, True):
            edge = x0 if distal else x1
            if abs(jx - edge) >= JOINT_GAP:
                continue
            sgn = +1.0 if distal else -1.0
            x = jx + sgn * (JOINT_GAP / 2 + POST_R)
            for (y, z) in ((0.0, DORSAL_ARM), (0.0, -DORSAL_ARM),
                           (LATERAL_ARM, 0.0), (-LATERAL_ARM, 0.0)):
                out.append(Pos(x, y, SPINE_Z + z) * (Rot(90, 0, 0) * Cylinder(
                    POST_R + POST_CLEAR, 2 * POST_R + 2 * POST_CLEAR)))
    return out


def rigid_body(body: int):
    g = body_shell(body)
    for i, h in enumerate(hip_bosses(body)):
        g = _fuse(g, h, "body %d hip boss %d" % (body, i))
    for v in hip_bores(body):
        g = g - v
    # ⚠️ cut the cable clearance BEFORE the posts go in, or the post arrives
    # tangent to the wall and the fuse is the one that eats the body.
    clear = process_clearance(body)
    for v in clear:
        g = g - v
    for i, j in enumerate(joint_parts(body)):
        g = _fuse(g, j, "body %d joint part %d" % (body, i))
    # ⚠️ M120: the bulkheads go in LAST, each with the cable clearance already
    # cut from it. G3 lengthened body 1's row by 2.4 mm, its end bulkhead came
    # to overlap a process post by 0.36 mm, and the post's fuse destroyed the
    # body. Cut first, the bulkhead keeps 1.5 mm off the post -- and the cable
    # clearance, which a bulkhead fused after the cut would have filled again.
    for i, c in enumerate(bulkheads(body)):
        for v in clear:
            c = c - v
        g = _fuse(g, c, "body %d bulkhead %d" % (body, i))
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
    # ⚠️ M120: with its bulkhead pads, since the rows grew by G3 and the
    # bodies did not
    for (_nm, b, x, _n, ro) in _rows():
        x0, x1 = BODIES[b]
        h = row_len(ro) / 2 + _PAD
        if not (x0 - 1e-6 <= x - h and x + h <= x1 + 1e-6):
            print("  *** a row at x=%.1f straddles body %d (%.0f..%.0f)"
                  % (x, b, x0, x1))
            ok = False

    # ⚠️ no two motors may interpenetrate
    allm = [(x, y, _zc(x) + z, row_len(ro)) for (_nm, _b, x, n, ro) in _rows()
            for (y, z) in ROWS[n][0]]
    for i in range(len(allm)):
        for j in range(i + 1, len(allm)):
            xi, yi, zi, li = allm[i]
            xj, yj, zj, lj = allm[j]
            if (abs(xi - xj) < (li + lj) / 2 - 1e-6
                    and math.hypot(yi - yj, zi - zj) < 2 * R):
                print("  *** motors overlap: (%.0f,%.0f,%.0f) and (%.0f,%.0f,%.0f)"
                      % (xi, yi, zi, xj, yj, zj))
                ok = False

    # ⚠️ **The drive-train check, which volume alone cannot make.** A leg's
    # motors have to sit on the body that leg hangs from; anywhere else the cable
    # crosses a spine joint, so bending the spine drives the leg. A spine motor
    # has to be beside the joint it drives, for the same reason.
    GIRDLE = {"hind": 0, "fore": 3, "tail": 0}
    # ⚠️ Counted per POSITION, not per row: `tri3` carries two hind-leg motors
    # and the tail's, and counting by row read it as seven legs' worth.
    per_role = {}
    for (_nm, b, _x, n, role) in _rows():
        for k in range(len(ROWS[n][0])):
            per_role.setdefault(POSITION_ROLE.get((n, k), role), []).append(b)
    for role, want_body in GIRDLE.items():
        got = sum(1 for b in per_role.get(role, []) if b == want_body)
        stray = sum(1 for b in per_role.get(role, []) if b != want_body)
        need = 1 if role == "tail" else 6
        note = "" if got >= need else "   *** needs %d" % need
        print("  %-5s: %d motor%s on body %d%s"
              % (role, got, "" if got == 1 else "s", want_body, note))
        if got < need or stray:
            ok = False
        if stray:
            print("      *** %d on another body -- the cable would cross a "
                  "spine joint" % stray)
    n_spine = len(per_role.get("spine", []))
    print("  spine: %d motors on bodies %s"
          % (n_spine, sorted(set(per_role.get("spine", [])))))
    if n_spine != 6:
        print("      *** ADR-0006 needs six, three joints x two DOF")
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
    # ⚠️ **This line used to print `Z_DORSAL` and call it the dorsal line.** It
    # was the parameter, not the shape, so it read "flat at 79.8" no matter what
    # the trunk did -- and the trunk notched **52.6 mm at every joint**, 42 % of
    # the chest depth, because the spine axis was at z = 0 and each neck blended
    # the section centre down to the belly to reach it.
    #
    # ✅ **What makes a cat's back flat is the spinous process, not the barrel.**
    # The back you feel on an animal is the row of process tips with skin over
    # them; the body itself necks in between, because that is how it bends. So
    # the criterion is not "the barrel is flat" -- it never can be -- but **every
    # joint's dorsal process must reach the dorsal line**. `SpineParams.
    # spine_axis_z` is derived to make it so, and this is where that is checked.
    dorsal = [_zc(x) + hw_at(x) * ASPECT for x in xs]
    belly = [_zc(x) - hw_at(x) * ASPECT for x in xs]
    tips = [SPINE_Z + DORSAL_ARM + POST_R for _ in JOINT_X]
    print()
    print("  dorsal line      %.1f mm at the girdles, %.1f at the joint necks"
          % (max(dorsal), min(dorsal)))
    print("  process tips     %.1f mm -- the dorsal line is %.1f"
          % (tips[0], Z_DORSAL))
    if abs(tips[0] - POST_R - Z_DORSAL) > 0.5:
        print("      *** a dorsal process does not reach the back: the skin "
              "would have nothing to lie on at the joint")
        ok = False
    undulation = max(dorsal) - min(dorsal)
    print("  barrel undulates %.1f mm = %.0f %% of the %.1f mm chest depth"
          % (undulation, 100 * undulation / (2 * _hw("fore_f") * ASPECT),
             2 * _hw("fore_f") * ASPECT))
    print("  belly            %.1f mm at the chest -> %.1f at the waist  (tuck-up %.1f)"
          % (min(belly), max(belly), max(belly) - min(belly)))
    print("  motors           %d of 19  (12 leg + 3 pitch + 3 yaw + 1 tail)"
          % n_mot)
    if n_mot != 19:
        print("      *** ADR decision F bought 19")
        ok = False
    print("  trunk length     %.0f mm   chest %.1f w x %.1f h   waist %.1f x %.1f"
          % (hi - lo, 2 * _hw("fore_f"), 2 * _hw("fore_f") * ASPECT,
             2 * _hw("spine_c"), 2 * _hw("spine_c") * ASPECT))
    print("  envelope         %.0f cm3" % (env / 1000.0))
    print("  motors solid     %.0f cm3 = %.1f %% of the envelope"
          % (n_mot * mot / 1000.0, 100 * n_mot * mot / env))
    # ⚠️ **The budget lives here now, not in a sentence.** The trunk mass was
    # quoted at "221 g against a 200 g budget" in prose; neither number was in
    # the code. The 221 was measured off a body the boolean had annihilated --
    # `_fuse` explains -- and the 200 was recalled, not derived. Both are printed
    # from the model now, so neither can drift from it again.
    allow = SP.trunk_mass * MM
    motors_g = 19 * 131.7
    budget = allow - motors_g - 240.0
    struct = total_v * 1.2e-3
    print("  structure        %.0f cm3 -> %.0f g nylon-CF" % (total_v / 1000.0, struct))
    print("  budget           %.0f g  (%.0f g trunk - %.0f g motors - 240 g head/neck)"
          % (budget, allow, motors_g))
    print("  left for the un-drawn %.0f g  -- battery, electronics, cable, spine"
          % (budget - struct))
    if struct > budget:
        print("      *** the printed structure alone is over the trunk budget")
        ok = False
    print("  %s" % ("all bodies are one part each" if ok else "*** SEE ABOVE ***"))
    return ok


def render_png(g, path, elev=16, azim=-62):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(12, 6))
    ax = fig.add_subplot(111, projection="3d")
    # ⚠️ **A face is valid and will not mesh.** Body 3 carries one planar face
    # that OCC refuses to triangulate at any tolerance, though `is_valid` is True
    # and `clean()` changes nothing; it comes off the boolean where a flat stem
    # meets the curved barrel. Rendering face by face and COUNTING the skips is
    # honest; tessellating the whole body raised an AttributeError and drew
    # nothing. The same face will fail an STL export.  `[owed]`
    #
    # It was **three** faces before M91. The other two were on body 0, which the
    # tangential fuse had annihilated -- so two thirds of a standing `[owed]`
    # was a symptom of a different defect, not a limit of the mesher. 1 of 656.
    # ⚠️ One collection PER FACE ran the process out of memory -- it died inside
    # matplotlib's projection unable to allocate 750 KiB, having built thousands
    # of artists, and the shell still reported exit 0 with no file written. The
    # triangles are batched into one collection per colour now: same picture,
    # two artists.
    skipped = 0
    batch = {}
    for i, s in enumerate(g if isinstance(g, list) else [g]):
        col = ["#8a9bb0", "#9fb0c4"][i % 2]
        for f in s.faces():
            try:
                verts, tris = f.tessellate(0.3)
            except Exception:
                skipped += 1
                continue
            V = np.array([[v.X, v.Y, v.Z] for v in verts])
            batch.setdefault(col, []).append(V[np.array(tris)])
    for col, tri in batch.items():
        coll = Poly3DCollection(np.concatenate(tri), facecolor=col,
                                edgecolor="#3c4a5a", linewidth=0.08)
        coll.set_zsort("average")
        ax.add_collection3d(coll)
    if skipped:
        print("  *** " + str(skipped)
              + " face(s) would not mesh and are missing from the render")
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
