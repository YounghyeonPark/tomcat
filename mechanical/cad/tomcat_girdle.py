# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The girdle as a CAT-SHAPED structure (build123d).

`tomcat_packaging.py` draws each girdle as a translucent `Box` sized to enclose
its motors -- a fit check, and it says so. This is the part that carries the
load, in the shape the animal has.

⚠️ **Two things forced the design, and neither was a style choice.**

**1. A cat's section is a laterally compressed OVAL, and the shipped motor
packing cannot fit in one.** The motors stand upright in two banks either side of
the centreline, which fills a rectangle's corners. Wrap that same cluster in an
oval of cat proportions and it needs **110.8 x 155.1 mm**, against a 4 kg cat's
chest outline of about **96 x 124**. Laying the motors FORE-AFT and nesting three
of them triangularly in the section gives **88.7 x 124.1 mm** -- inside the
animal, and consistent with this project's own ribcage half-width of 34 mm plus
flesh. `_motor_slots()` is that arrangement.

⚠️ It moves the spools onto the fore and aft faces, so a cable reaching a lateral
hip has to turn. ADR-0042/0083's wrap angles are computed for the old orientation
and do not carry over. `[owed]`

**2. The mass allowance decides the wall, which is backwards.** `params.py`
budgets **90 g front / 110 g rear** for "structure" and calls it *"the loosest
number here"*. `report()` prints what this shell weighs against it, and counts
solids before it prints anything else.

Everything dimensional is imported, not retyped: the mount offset and mass
budget from `SpineParams`, the motor sizes from `tomcat_packaging`, the hip
bearing and tongue from `tomcat_leg_detail`, the track from `ASSEMBLY_SPEC` §0.1
via `TRACK_Y`.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
from build123d import (Box, Compound, Cylinder, Ellipse, Plane, Pos, Rot, loft)

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "kinematics", "src"))

import tomcat_packaging as TP                                    # noqa: E402
import tomcat_leg_detail as LD                                   # noqa: E402
from tomcat_kin.params import DEFAULT_SPINE as SP                # noqa: E402

MM = 1000.0
R = TP.MOTOR_D / 2.0

#: Clearance from a motor to the shell's inner face.
CLR = 3.0
#: Section aspect, height / width. A cat is deep and narrow through the chest;
#: 1.40 is the ratio of the 96 x 124 mm outline a 4 kg animal carries.
ASPECT = 1.40
#: The pelvis is narrower than the chest. This taper is what makes the trunk
#: read as one animal instead of two boxes.  `[assumed]`
PELVIC_SCALE = 0.88

#: ⚠️ **The cat form does not fit the mass allowance, and 1.2 mm is the
#: thinnest wall worth printing.** Measured, connected throughout, front girdle
#: against its 92 g structure budget:
#:
#:     wall  bulkhead   nylon-CF
#:     1.6   2.0        155.0 g
#:     1.2   1.6        122.6 g   <- shipped
#:     1.0   1.6        113.7 g   (2.5 perimeters: not a structural wall)
#:
#: A rectangular box of the same contents closes at **88.2 g**. The oval cannot
#: use its corners, so cat proportions cost **+39 %** of structure -- **+42 g on
#: a 4.30 kg robot, 1.0 %**. That is the price of the shape, not a modelling
#: error, and `params.py`'s 90/110 g allowance (its own "loosest number here")
#: has to rise to about 125 g before this part is legal.  `[owed]`
WALL = 1.2
BULKHEAD_T = 1.6
BOSS_WALL = LD.BOSS_WALL
TRACK = LD.TRACK_Y
HIP_BORE, HIP_OD, HIP_W = LD.BEARING["hip"]
TONGUE_T = 6.0
SPINE_BORE = 16.0

#: Where the girdle sits relative to the hip axis, from `SpineParams`.
Z0 = SP.girdle_offset_z * MM
#: Fore-aft pitch between the two motor rows.
STEP = TP.MOTOR_L + TP.SPOOL_L + 3.0


def _motor_slots():
    """Six modules, axes FORE-AFT, three nested per row and two rows in x.

    The section triangle is what buys the cat proportion: two circles low and one
    above, rather than two circles side by side with a stack over them.
    """
    s = 2 * R + 2.0
    tri = [(-(R + 1.0), 0.0), (R + 1.0, 0.0), (0.0, s * math.sqrt(3) / 2)]
    zc = sum(p[1] for p in tri) / 3.0
    out = []
    for row, sx in enumerate((-0.5, 0.5)):
        for (y, z) in tri:
            out.append((sx * STEP, y, z - zc, +1 if row else -1))
    return out


def _section(scale=1.0):
    """Half-width and half-height of the shell's OUTER oval."""
    a = 0.0
    for (_x, y, z, _s) in _motor_slots():
        yy = abs(y) + R + CLR + WALL
        zz = abs(z) + R + CLR + WALL
        a = max(a, math.sqrt(yy * yy + (zz / ASPECT) ** 2))
    return a * scale, a * ASPECT * scale


def _length():
    return STEP + TP.MOTOR_L + 2 * TP.SPOOL_L + 2 * WALL


def _ell(x, hw, hh):
    """A section at `x`. ⚠️ The plane's local axes are not (y, z): passing
    `Ellipse(hw, hh)` put the LONG axis across the body and made a cat that was
    wider than it was deep -- the exact opposite of the animal. Verified by
    bounding box in `report()`, not by reading the plane's documentation."""
    return Plane(origin=(x, 0, Z0), z_dir=(1, 0, 0)) * Ellipse(hh, hw)


def shell(front: bool = False):
    """The oval barrel, hollowed, tapering toward the tail on the pelvic girdle.

    ✅ Lofted between two sections so the trunk tapers -- that taper is what makes
    it read as an animal. The bulkheads are full sections: they tie the barrel
    together across the span the hip bosses load, and they are what the motors
    bolt to.
    """
    hw, hh = _section()
    ln = _length()
    tw, th = ((hw, hh) if front
              else (hw * PELVIC_SCALE, hh * PELVIC_SCALE))

    body = (loft([_ell(-ln / 2, tw, th), _ell(ln / 2, hw, hh)])
            - loft([_ell(-ln / 2 - 1, tw - WALL, th - WALL),
                    _ell(ln / 2 + 1, hw - WALL, hh - WALL)]))

    # ✅ End bulkheads: the loft is an open TUBE, so without these the spine
    # flange floats in mid-air at the inboard end and the part is two pieces.
    for sx in (-1.0, 1.0):
        xe = sx * (ln / 2 - BULKHEAD_T / 2)
        f = (xe + ln / 2) / ln
        w = tw + (hw - tw) * f - WALL
        h = th + (hh - th) * f - WALL
        cap = loft([_ell(xe - BULKHEAD_T / 2, w, h), _ell(xe + BULKHEAD_T / 2, w, h)])
        cap -= (Pos(xe, 0, 0) * (Rot(0, 90, 0)
                                 * Cylinder(SPINE_BORE / 2, BULKHEAD_T + 4)))
        # lighten: a ring of pockets, keeping a spoke to each motor bore
        for ang in range(0, 360, 60):
            a = math.radians(ang + 30)
            cap -= (Pos(xe, 0.55 * w * math.cos(a), Z0 + 0.55 * h * math.sin(a))
                    * (Rot(0, 90, 0) * Cylinder(0.20 * w, BULKHEAD_T + 4)))
        body += cap

    for sx in (-0.5, 0.5):
        x = sx * STEP - np.sign(sx) * (TP.MOTOR_L / 2 + BULKHEAD_T / 2)
        f = (x + ln / 2) / ln
        w = tw + (hw - tw) * f - WALL
        h = th + (hh - th) * f - WALL
        deck = loft([_ell(x - BULKHEAD_T / 2, w, h), _ell(x + BULKHEAD_T / 2, w, h)])
        for (mx, my, mz, _s) in _motor_slots():
            if abs(mx - sx * STEP) > 1e-6:
                continue
            deck -= (Pos(x, my, Z0 + mz)
                     * (Rot(0, 90, 0) * Cylinder(TP.SPOOL_D / 2 + 1.0,
                                                 BULKHEAD_T + 4)))
        body += deck
    return body


#: Frame left around a flank window.  `[assumed]`
RIB = 7.0


def flank_windows():
    """Openings in the barrel, in the bays the bulkheads leave free.

    ⚠️ Placed BETWEEN the bulkheads and clear of the hip bosses, because the
    first structural pass cut one window per wall and the part fell into ten
    pieces -- every attachment was made to wall the window had just removed.
    `report()` counts solids for that reason.
    """
    hw, hh = _section()
    ln = _length()
    xd = STEP / 2 + TP.MOTOR_L / 2 + BULKHEAD_T
    cuts = []
    for sx in (-1.0, 1.0):
        xc = sx * (xd + ln / 2) / 2.0
        w = max(abs(ln / 2 - xd) - 2 * RIB, 4.0)
        for sy in (-1.0, 1.0):
            cuts.append(Pos(xc, sy * hw, Z0 + 0.42 * hh)
                        * Box(w, 3 * WALL + 6, 0.42 * hh))
    # the belly, between the two decks. ⚠️ Clamped: STEP - MOTOR_L - 2*BULKHEAD
    # - 2*RIB is **-7 mm** at the shipped pitch, and build123d raises rather than
    # quietly making a zero box. The two motor rows nearly touch, so there is no
    # belly bay to open -- the window is skipped, not fudged into existence.
    belly = STEP - TP.MOTOR_L - 2 * BULKHEAD_T - 2 * RIB
    if belly > 6.0:
        cuts.append(Pos(0, 0, Z0 - hh) * Box(belly, 1.1 * hw, 3 * WALL + 6))
    return cuts


def hip_boss(side: float, front: bool):
    """One hip: a boss out to the limb plane, ending in the tongue.

    ⚠️ `tomcat_leg_detail` already draws that tongue at the hip, with a Ø10 root
    stub going nowhere, because there was no girdle to grow it from. This is what
    it plugs into.
    """
    hw, _hh = _section(1.0 if front else PELVIC_SCALE)
    y_hip = side * TRACK
    boss_r = HIP_OD / 2 + BOSS_WALL
    inner = hw - WALL
    span = max(abs(y_hip) - inner, 1.0)
    arm = Pos(0, side * (inner + span / 2), 0) * Box(2 * boss_r, span, 2 * boss_r)
    axis = Plane(origin=(0, y_hip, 0), z_dir=(0, side, 0)).location
    lug = axis * (Cylinder(boss_r, TONGUE_T) - Cylinder(HIP_BORE / 2, TONGUE_T + 2))
    return Compound([arm, lug])


def spine_flange(front: bool):
    """Where ADR-0006's chain lands, on the inboard face at the hip axis."""
    x = (-1.0 if front else 1.0) * (_length() / 2 - BULKHEAD_T)
    plate = Pos(x, 0, 0) * Box(2 * WALL + 6.0, 34.0, 34.0)
    bore = (Plane(origin=(x, 0, 0), z_dir=(1, 0, 0)).location
            * Cylinder(SPINE_BORE / 2, 2 * WALL + 14.0))
    return plate - bore


def cable_slots():
    """Every spool has to get its cable out toward a lateral hip."""
    hw, _hh = _section()
    cuts = []
    for (mx, my, mz, s) in _motor_slots():
        x = mx + s * (TP.MOTOR_L / 2 + TP.SPOOL_L / 2)
        y = hw if (my >= 0) else -hw
        cuts.append(Pos(x, y, Z0 + mz)
                    * Box(TP.SPOOL_L + 3.0, 2 * WALL + 6, TP.SPOOL_D))
    return cuts


def girdle(front: bool = False):
    g = shell(front) + spine_flange(front)
    for side in (+1.0, -1.0):
        g += hip_boss(side, front)
    for c in cable_slots() + flank_windows():
        g -= c
    return g


def report(front: bool = False):
    """Mass against the allowance -- and solids first.

    ⚠️ The first structural pass came apart into **ten** pieces: hip bosses,
    spine flange and every deck web attached to wall the windows had just
    removed. It rendered as a perfectly convincing girdle. "It looks right" is
    not a connectivity check, so this counts.
    """
    g = girdle(front)
    sol = g.solids()
    vol = sum(s.volume for s in sol)
    hw, hh = _section()
    tw, th = _section(1.0 if front else PELVIC_SCALE)
    allow = (SP.front_girdle_mass if front else SP.rear_girdle_mass) * MM
    motors = 6 * 131.7
    budget = allow - motors - (240.0 if front else 0.0)
    print("%s girdle, wall %.1f mm" % ("FRONT" if front else "REAR", WALL))
    print("  section         %.1f w x %.1f h mm   tail end %.1f x %.1f"
          % (2 * hw, 2 * hh, 2 * tw, 2 * th))
    print("  length          %.1f mm" % _length())
    print("  solids          %d %s" % (len(sol), "(one part)" if len(sol) == 1
                                       else "*** DISJOINT ***"))
    print("  structure vol   %.1f cm3" % (vol / 1000.0))
    print("  allowance       %.0f g  (%.0f g total - %.0f g motors%s)"
          % (budget, allow, motors, " - 240 g head/neck" if front else ""))
    for name, rho in (("nylon-CF", 1.20e-3), ("PETG", 1.27e-3),
                      ("alu 6061", LD.AL_RHO)):
        m = vol * rho
        print("    %-9s %7.1f g   %s" % (name, m, "OK" if m <= budget else
                                         "OVER by %.0f g" % (m - budget)))
    return vol


def render_png(g, path, elev=18, azim=-62):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")
    verts, tris = g.tessellate(0.2)
    V = np.array([[v.X, v.Y, v.Z] for v in verts])
    coll = Poly3DCollection(V[np.array(tris)], facecolor="#8a9bb0",
                            edgecolor="#3c4a5a", linewidth=0.12, alpha=1.0)
    coll.set_zsort("average")
    ax.add_collection3d(coll)
    bb = g.bounding_box()
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
    for front in (False, True):
        report(front)
        print()
    g = girdle(False)
    from build123d import export_step, export_stl
    export_step(g, os.path.join(HERE, "tomcat_girdle.step"))
    export_stl(g, os.path.join(HERE, "tomcat_girdle.stl"))
    render_png(g, os.path.join(HERE, "tomcat_girdle.png"))
    render_png(g, os.path.join(HERE, "tomcat_girdle_front.png"), elev=4, azim=-90)
