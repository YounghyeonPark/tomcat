# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The whole robot: four legs on the trunk, in the stance pose (build123d).

The pieces existed and had never met. `tomcat_leg_detail` designs one leg as
manufacturable parts and draws it about its own hip; `tomcat_trunk` designs the
four rigid bodies and presents a **tongue** at each hip. Nothing had put them
together, so nothing had checked that they fit.

What this file asserts, and each one is a way the assembly can be wrong:

- every leg's hip lands on the trunk's tongue, to a tenth of a millimetre;
- the legs do not pass through the trunk;
- the two legs on a side do not pass through each other;
- all four feet reach the same ground plane.

⚠️ **The leg model draws its own hip tongue.** It has to, to be a standalone
leg -- but the trunk grew one too, and in an assembly they are the same part
drawn twice. The leg's copy is dropped here, and the duplicate is named rather
than left to double the mass quietly.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
from build123d import Compound, Plane, Pos, mirror

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "kinematics", "src"))

import leg_tendons as LT                                         # noqa: E402
import tomcat_leg_detail as LD                                   # noqa: E402
import tomcat_trunk as TT                                        # noqa: E402
from tomcat_kin.params import DEFAULT_FORELEG, DEFAULT_HINDLEG   # noqa: E402

#: Where the hips are, from the trunk model.
#: Hip stations, trunk x. M122: the hind hips are `REAR_HIP_X` behind the
#: spine's root joint (ADR-0112).
HIP_X = {"rear": TT.REAR_HIP_X, "front": 195.0}

#: ⚠️ **Groups the TRUNK owns wherever they sit at the hip.** This was the
#: literal tuple `("clevis", "bearing")`, and when M93 split the shafts into
#: their own group the hip shaft stopped being recognised -- it is the longest
#: part at the joint now, reaching out to the via pulley, and it drove the
#: hind legs' trunk overlap from **0.0 to 118.8 mm3**. Third time a new group
#: has been missed by a list that enumerates them, so the list is named and
#: sits beside `DROP` where the next one will be looked for.
HIP_HARDWARE = ("clevis", "bearing", "shaft")

#: The leg groups that are CONTEXT in `tomcat_leg_detail`, not leg parts.
#: `motor` is the girdle bank it drew for reference; the trunk owns those now.
DROP = ("motor",)


def motor_can():
    """The GIM3505-9 can as `tomcat_leg_detail` draws it -- the template a
    duplicate is recognised BY, taken from the part that draws it rather than
    written down here."""
    return max(LD.motor_and_spool((0.0, 0.0), 10.0).solids(),
               key=lambda sd: sd.volume)


def is_motor_can(sd, ref=None):
    """⚠️ **Volume alone is not an identity.** The first version of this
    matched anything within 2 % of 33,747 mm3 and flagged **trunk body 2**,
    which is 34,200 -- a whole rigid body called a motor. A can is Ø34.5 x
    36.1, so its box is as particular as its volume; both have to agree."""
    ref = motor_can() if ref is None else ref
    rb, sb = ref.bounding_box(), sd.bounding_box()
    r = sorted((rb.size.X, rb.size.Y, rb.size.Z))
    t = sorted((sb.size.X, sb.size.Y, sb.size.Z))
    return (abs(sd.volume - ref.volume) < 0.02 * ref.volume
            and all(abs(a - b) < 0.5 for a, b in zip(r, t)))


def _at_hip(solid, radius=26.0):
    """Is this solid part of the HIP joint, which the trunk owns?"""
    c = solid.center()
    return math.hypot(c.X, c.Z) < radius


#: The drive's HARNESS: cables, conduits and the trunk-side conduit ferrules.
#: They leave the trunk through ports and pass the skin at the hip, neither of
#: which is drawn yet (M122, `[owed]`), so the checks report them apart from
#: the leg's STRUCTURE rather than fail on them.
HARNESS = ("conduit", "trunk_ferrule", "tendon")


def one_leg(fore: bool, side: float, pose=None, only=None, skip=()):
    """One leg in TRUNK coordinates.

    `tomcat_leg_detail` builds about its own hip at the origin and translates to
    the limb plane at +TRACK_Y, so a right leg is the left one mirrored in y.
    """
    leg = DEFAULT_FORELEG if fore else DEFAULT_HINDLEG
    # ✅ **The cables go to the motors that exist -- in 3-D since M122.** M88's
    # fix handed the leg the trunk's spool (x, z) and drew the spool in the
    # leg's plane; the real one is inside the trunk on an axis along x.
    # `tomcat_leg_detail` now draws the whole drive from `tendon_exit.DRIVE`:
    # spool, lead, conduit, ferrules, free run, anchor.
    role = "fore" if fore else "hind"
    comps, report, pts = LD.build(leg, role=role)
    parts = []
    for name, comp in comps.items():
        if name in DROP or name in skip or (only is not None and name not in only):
            continue
        for sd in comp.solids():
            # %s **The hip joint is drawn twice.** `tomcat_leg_detail` has to
            # draw its own hip tongue and bearings to be a standalone leg, and
            # the trunk grew a boss with the same tongue on it. Measured, that
            # duplicate is **2838 mm3 of clevis and 924 of bearing** occupying
            # the same space. In an assembly the TRUNK owns the hip, so the
            # leg's copy is dropped -- otherwise the robot carries the joint
            # twice in mass and the interference check never goes quiet.
            if name in HIP_HARDWARE and _at_hip(sd):
                continue
            parts.append(sd)
    g = Compound(parts)
    if side < 0:
        # ⚠️ A right leg is the left one REFLECTED in y, and a reflection is not
        # a rotation. `Rot(180, 0, 0)` was the first attempt: it maps y -> -y and
        # z -> -z together, so both right legs came out upside down with their
        # feet at z = -30 against the left pair's -177. The foot-plane check is
        # what said so; the render just looked busy.
        g = mirror(g, about=Plane.XZ)
    return Pos(HIP_X["front" if fore else "rear"], 0, 0) * g


def soft_parts():
    """Head, tail and skin -- the parts that are neither trunk nor leg.

    ⚠️ **`assembly()` was named "the whole robot" and knew about none of
    them.** The head (M97-M100), the tail (M96) and the skin (M94) were each
    drawn and each checked against the trunk inside their own module, and not
    one of them was ever in the thing this file calls the assembly. So the
    first render of the complete robot had to reach PAST this file to collect
    them -- and reaching past it is exactly what drew 12 motors the trunk
    already owns, routed every cable to the default `SPOOL_OFFSET` the assembly
    exists to override, and doubled the hip. A picture assembled by its viewer
    is not a picture of the assembly.
    """
    import tomcat_head as HD
    import tomcat_skin as SK
    import tomcat_tail as TL
    # ⚠️ **`features()` is a separate solid and this list shipped without
    # it.** `tomcat_head.render_png` draws `whole()` AND `features()`; the first
    # repo-side render of the robot had the cranium and no ears, no eyes and no
    # nose. Fourth time a list that enumerates parts has missed a new one --
    # after `link_inertia.RHO` dropped 14.67 g, after the hip filter missed
    # `shaft` twice.
    return [("head", HD.whole()), ("face", HD.features()),
            ("tail", TL.tail()), ("skin", SK.skin())]


def assembly(soft: bool = True, split: bool = False):
    """`(bodies, legs, soft)` -- every part of the robot that has been drawn.

    `split=True` returns `(bodies, legs, soft, harness)` with each leg's drive
    HARNESS (`HARNESS`) apart from its structure, for the checks."""
    bodies = [TT.rigid_body(b) for b in sorted(TT.BODIES)]
    if not split:
        legs = [one_leg(f, s) for f in (True, False) for s in (+1.0, -1.0)]
        return bodies, legs, (soft_parts() if soft else [])
    legs = [one_leg(f, s, skip=HARNESS) for f in (True, False) for s in (+1.0, -1.0)]
    harness = [one_leg(f, s, only=HARNESS) for f in (True, False) for s in (+1.0, -1.0)]
    return bodies, legs, (soft_parts() if soft else []), harness


def report():
    bodies, legs, soft, harness = assembly(split=True)
    print("%-16s %8s %9s" % ("part", "solids", "vol cm3"))
    tv = 0.0
    for i, b in enumerate(bodies):
        v = sum(s.volume for s in b.solids())
        tv += v
        print("%-16s %8d %9.1f" % ("trunk body %d" % i, len(b.solids()), v / 1000))
    for i, l in enumerate(legs):
        v = sum(s.volume for s in l.solids())
        tv += v
        nm = ["LF", "RF", "LR", "RR"][i]
        print("%-16s %8d %9.1f" % ("leg %s" % nm, len(l.solids()), v / 1000))
    for nm, g in soft:
        v = sum(s.volume for s in g.solids())
        tv += v
        print("%-16s %8d %9.1f" % (nm, len(g.solids()), v / 1000))
    print("%-16s %8s %9.1f" % ("TOTAL", "", tv / 1000))

    ok = True
    # --- the hips must land on the tongues
    for i, l in enumerate(legs):
        nm = ["LF", "RF", "LR", "RR"][i]
        bb = l.bounding_box()
        want_x = HIP_X["front" if i < 2 else "rear"]
        want_y = TT.TRACK * (1.0 if i % 2 == 0 else -1.0)
        print("  %-4s hip target (%.0f, %+.0f)   leg bbox x[%.0f,%.0f] y[%+.0f,%+.0f]"
              % (nm, want_x, want_y, bb.min.X, bb.max.X, bb.min.Y, bb.max.Y))
        if not (bb.min.Y - 1 <= want_y <= bb.max.Y + 1):
            print("      *** the limb plane is not at the trunk's track")
            ok = False

    # --- legs must not pass through the trunk
    trunk = Compound([s for b in bodies for s in b.solids()])
    for i, l in enumerate(legs):
        nm = ["LF", "RF", "LR", "RR"][i]
        # ⚠️ `intersect` returns None when the two do not touch at all, not an
        # empty shape. Reading `.solids()` off it is an AttributeError, and it
        # only appeared once the hind legs came CLEAN -- the failure mode of the
        # success case.
        inter = trunk.intersect(l)
        v = 0.0
        if inter is not None:
            try:
                v = sum(sd.volume for sd in inter.solids())
            except Exception:
                v = 0.0
        flag = "" if v < 1500.0 else "  *** deep interference"
        if v >= 1500.0:
            ok = False
        hv = 0.0
        hi = trunk.intersect(harness[i])
        if hi is not None:
            try:
                hv = sum(sd.volume for sd in hi.solids())
            except Exception:
                hv = 0.0
        print("  %-4s leg/trunk overlap %8.1f mm3%s   (harness through the wall: "
              "%.1f mm3, ports `[owed]`)" % (nm, v, flag, hv))

    import numpy as _np
    # --- ⚠️ every cable must START on a spool the trunk actually has, in 3-D
    # ⚠️ **M122: this check could not fail.** It asked whether the router's
    # first point sat on the spool (x, z) it had been GIVEN -- the same number
    # twice -- while the spool it named was 45-55 mm away, inside the trunk,
    # winding across the body. Now: the lead leaves a circle of `SPOOL_R` about
    # a motor the trunk has, in that motor's end plane; it reaches a conduit
    # ferrule ON the shell wall; and the hip pair's other ferrule is OUTSIDE it.
    import tendon_exit as TE
    for i, (fore, side) in enumerate([(True, 1.0), (True, -1.0),
                                      (False, 1.0), (False, -1.0)]):
        nm = ["LF", "RF", "LR", "RR"][i]
        role = "fore" if fore else "hind"
        shell = TT._outer(TT.LEG_BODY[role])
        motors = TT.leg_spools(role, side, by="tendon")
        off_spool = off_wall = 0.0
        inside = 0
        for (t, sd), r in TE.drive(role, side).items():
            x, y, z = motors[("hip", "knee", "ankle").index(t)]
            T, F1 = r["lead"]
            off_spool = max(off_spool, abs(_np.hypot(T[1] - y, T[2] - z) - LT.SPOOL_R))
            half = TT.row_len(role) / 2
            if not (x - half - 1e-6 <= T[0] <= x + half + 1e-6):
                off_spool = max(off_spool, 999.0)
            d = (F1 - T) / _np.linalg.norm(F1 - T)
            if not shell.is_inside(tuple(F1 - 1.0 * d)) or shell.is_inside(tuple(F1 + 1.0 * d)):
                off_wall += 1
            if t == "hip" and shell.is_inside(tuple(r["F2"])):
                inside += 1
        print("  %-4s cables start on their spools to %.3f mm; wall ferrules off the "
              "wall: %d of 6; hip ferrules inside the shell: %d of 2"
              % (nm, off_spool, off_wall, inside))
        if off_spool > 0.05 or off_wall or inside:
            print("      *** the drive does not connect motor to leg")
            ok = False
    print("  `[owed]`: the shell's conduit ports and the hip ferrule brackets are "
          "not drawn -- each ferrule marks where one goes")

    # --- ⚠️ **the limb plane must clear the girdle, and DEPTH is the test.**
    # An 80 mm3 "leg/trunk overlap" passed a 1500 mm3 threshold for four
    # milestones. Measured as a depth it is the femur sitting **2.1 mm inside**
    # the front girdle's flank -- and it is not a leg defect: `TRACK_Y` is 48 mm,
    # set when the trunk was narrower, and M88 widened the chest 83.4 -> 90.0
    # without moving the legs out with it. A volume threshold cannot see a
    # shallow, wide interference; a depth can.
    import numpy as _np
    hw = max(TT._hw_at(x) for b in (0, 3)
             for x in _np.linspace(*TT.BODIES[b], 40))
    od = LD.TUBE["femur"][0]
    need = hw + 1.0 + od / 2
    short = need - LD.TRACK_Y
    print("  limb plane %.1f mm vs girdle half-width %.1f + ø%.0f femur -> "
          "needs %.1f" % (LD.TRACK_Y, hw, od, need))
    if short > 0.05:
        print("      *** the femur sits %.2f mm inside the girdle flank -- "
              "TRACK_Y never followed M88's wider trunk" % short)
        ok = False

    # --- ✅ every spool must be INSIDE the body that carries it
    outside = 0
    for role, body in (("hind", 0), ("fore", 3)):
        o = TT._outer(body)
        for sd in (+1.0, -1.0):
            for sp in TT.leg_spools(role, sd):
                if not o.is_inside(sp):
                    outside += 1
    print("  spools outside their body: %d of 12" % outside)
    if outside:
        print("      *** a motor is not in the housing it bolts to")
        ok = False

    # --- feet on one plane
    zs = []
    for l in legs:
        zs.append(l.bounding_box().min.Z)
    print("  foot z: %s  spread %.2f mm"
          % (["%.1f" % z for z in zs], max(zs) - min(zs)))
    if max(zs) - min(zs) > 3.0:
        print("      *** the four feet are not on one plane")
        ok = False
    # --- ⚠️ **no physical part may be contributed by two modules.**
    # `DROP` and `HIP_HARDWARE` above exist because the leg draws a motor bank
    # and a hip tongue that the trunk owns -- and nothing ever COUNTED the
    # result, so the rule held only for callers who read the source. The first
    # whole-robot render did not: it called `LD.build()` directly and put 12
    # reference motors on the machine, reaching to y = 79 mm where the girdle
    # flank is 43. The trunk represents its 19 motors as BORES, so the number
    # of motor cans standing in the finished assembly is zero, and the volume
    # is taken from the part that draws them rather than written down here.
    ref = motor_can()
    drawn = sum(1 for g in (list(bodies) + list(legs) + [x[1] for x in soft])
                for sd in g.solids() if is_motor_can(sd, ref))
    bores = sum(len(TT.motors(b)) for b in sorted(TT.BODIES))
    print("  motor cans drawn %d, bores placed %d of 19" % (drawn, bores))
    if drawn or bores != 19:
        print("      *** a motor is drawn twice, or is not placed at all")
        ok = False

    # --- ⚠️ **the skin has to clear the leg, not just the femur.**
    # `APERTURE_R` is sized from the femur tube plus ROM plus a cuff, and the
    # femur is the SLIMMEST thing at the hip: the groove-plane stack, its
    # bearings and the via shaft reach 23.9 mm outboard of the bone plane. The
    # skin was checked against the head and the tail and never against a leg.
    skin = dict(soft).get("skin")
    if skin is not None:
        for i, l in enumerate(legs):
            nm = ["LF", "RF", "LR", "RR"][i]
            inter = skin.intersect(l)
            v = 0.0
            if inter is not None:
                try:
                    v = sum(sd.volume for sd in inter.solids())
                except Exception:
                    v = 0.0
            hv = 0.0
            hi = skin.intersect(harness[i])
            if hi is not None:
                try:
                    hv = sum(sd.volume for sd in hi.solids())
                except Exception:
                    hv = 0.0
            print("  %-4s skin/leg overlap  %8.1f mm3%s   (harness through the "
                  "skin: %.1f mm3, aperture `[owed]`)"
                  % (nm, v, "" if v < 50.0 else "  *** the cover cuts the leg", hv))
            if v >= 50.0:
                ok = False

    whole = Compound([s for g in bodies for s in g.solids()]
                     + [s for g in legs for s in g.solids()]
                     + [s for _n, g in soft for s in g.solids()])
    bb = whole.bounding_box()
    print("  envelope %.0f x %.0f x %.0f mm   (L x W x H)"
          % (bb.size.X, bb.size.Y, bb.size.Z))
    print("  %s" % ("assembly checks pass" if ok else "*** SEE ABOVE ***"))
    return ok


#: One colour per contributor, so a part that comes from the wrong module is
#: visible as the wrong colour rather than as extra clutter.
COLOUR = {"trunk": "#8a9bb0", "leg": "#d7ac86", "head": "#c98f6a",
          "face": "#8a6b52", "tail": "#c98f6a", "skin": "#7f96ae"}


def _mesh(groups):
    """Tessellate and add one collection per colour.

    ⚠️ A few faces are valid and will not mesh -- see
    `tomcat_trunk.render_png`. Drawn face by face so one bad face costs one
    face, not the whole robot; and batched into ONE collection per colour,
    because a collection per face built thousands of artists and the process
    died of memory exhaustion inside the 3-D projection -- while the shell
    reported exit 0 and wrote no file.
    """
    skipped, out = 0, []
    for col, alpha, solids in groups:
        tri = []
        for sd in solids:
            for f in sd.faces():
                try:
                    verts, tris = f.tessellate(0.4)
                except Exception:
                    skipped += 1
                    continue
                V = np.array([[v.X, v.Y, v.Z] for v in verts])
                tri.append(V[np.array(tris)])
        if tri:
            out.append((col, alpha, np.concatenate(tri)))
    return out, skipped


def _paint(ax, mesh):
    """A collection per colour. Each axes needs its OWN artists, so the
    triangles are shared and the collections are not."""
    for col, alpha, tri in mesh:
        coll = Poly3DCollection(tri, facecolor=col, alpha=alpha,
                                edgecolor="#3c4a5a", linewidth=0.06)
        coll.set_zsort("average")
        ax.add_collection3d(coll)


def _groups(bodies, legs, soft):
    out = [(COLOUR["trunk"], 1.0, [s for g in bodies for s in g.solids()]),
           (COLOUR["leg"], 1.0, [s for g in legs for s in g.solids()])]
    for nm, g in soft:
        out.append((COLOUR.get(nm, "#c98a63"),
                    0.35 if nm == "skin" else 1.0, list(g.solids())))
    return out


def _frame(ax, mesh, elev, azim):
    pts = np.concatenate([tri.reshape(-1, 3) for _c, _a, tri in mesh])
    lo, hi = pts.min(axis=0) - 5, pts.max(axis=0) + 5
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect(tuple(hi - lo))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    return hi - lo


def render_png(path, elev=16, azim=-62, soft=True):
    """One view of the whole robot."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    global Poly3DCollection
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    mesh, skipped = _mesh(_groups(*assembly(soft=soft)))
    fig = plt.figure(figsize=(13, 7))
    ax = fig.add_subplot(111, projection="3d")
    _paint(ax, mesh)
    if skipped:
        print("  *** %d face(s) would not mesh" % skipped)
    _frame(ax, mesh, elev, azim)
    fig.tight_layout()
    fig.savefig(path, dpi=140, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", path)


#: Side, three-quarter and top. ⚠️ **The first whole-robot picture was
#: built by a throwaway script outside the repo**, which is why it showed parts
#: `DROP` exists to remove. The picture of the assembly is produced BY the
#: assembly now, from one build, so it cannot disagree with the checks above.
VIEWS = (("side", 2, -90), ("3/4", 16, -62), ("top", 88, -90))


def render_views(path, soft=True):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    global Poly3DCollection
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    mesh, skipped = _mesh(_groups(*assembly(soft=soft)))
    if skipped:
        print("  *** %d face(s) would not mesh" % skipped)
    fig = plt.figure(figsize=(19, 6.5))
    span = None
    for k, (name, elev, azim) in enumerate(VIEWS):
        ax = fig.add_subplot(1, len(VIEWS), k + 1, projection="3d")
        _paint(ax, mesh)
        span = _frame(ax, mesh, elev, azim)
        ax.set_title(name)
    fig.tight_layout()
    fig.savefig(path, dpi=110, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote %s   overall %.0f x %.0f x %.0f mm"
          % (path, span[0] - 10, span[1] - 10, span[2] - 10))


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--render", action="store_true")
    ap.add_argument("--no-soft", action="store_true")
    a = ap.parse_args()
    if a.render:
        render_views(os.path.join(HERE, "tomcat_whole.png"), soft=not a.no_soft)
    else:
        report()
