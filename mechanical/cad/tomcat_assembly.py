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
HIP_X = {"rear": 0.0, "front": 195.0}

#: The leg groups that are CONTEXT in `tomcat_leg_detail`, not leg parts.
#: `motor` is the girdle bank it drew for reference; the trunk owns those now.
DROP = ("motor",)


def _at_hip(solid, radius=26.0):
    """Is this solid part of the HIP joint, which the trunk owns?"""
    c = solid.center()
    return math.hypot(c.X, c.Z) < radius


def one_leg(fore: bool, side: float, pose=None):
    """One leg in TRUNK coordinates.

    `tomcat_leg_detail` builds about its own hip at the origin and translates to
    the limb plane at +TRACK_Y, so a right leg is the left one mirrored in y.
    """
    leg = DEFAULT_FORELEG if fore else DEFAULT_HINDLEG
    # ✅ **The cables go to the motors that exist.** `tomcat_leg_detail`
    # defaults to `SPOOL_OFFSET`, a single 2-D diagonal from the hip that was
    # only ever right for the upright two-bank girdle. Measured against the real
    # centres those runs miss by **20 to 44 mm** -- the wires in the first
    # assembly render pointed at empty space. The trunk owns the spools, so the
    # trunk hands them over.
    role = "fore" if fore else "hind"
    sp3 = TT.leg_spools(role, side)
    hx = HIP_X["front" if fore else "rear"]
    spools = {t: (p3[0] - hx, p3[2])
              for t, p3 in zip(("hip", "knee", "ankle"), sp3)}
    comps, report, pts = LD.build(leg, spools=spools)
    parts = []
    for name, comp in comps.items():
        if name in DROP:
            continue
        for sd in comp.solids():
            # %s **The hip joint is drawn twice.** `tomcat_leg_detail` has to
            # draw its own hip tongue and bearings to be a standalone leg, and
            # the trunk grew a boss with the same tongue on it. Measured, that
            # duplicate is **2838 mm3 of clevis and 924 of bearing** occupying
            # the same space. In an assembly the TRUNK owns the hip, so the
            # leg's copy is dropped -- otherwise the robot carries the joint
            # twice in mass and the interference check never goes quiet.
            if name in ("clevis", "bearing") and _at_hip(sd):
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


def assembly():
    bodies = [TT.rigid_body(b) for b in sorted(TT.BODIES)]
    legs = [one_leg(f, s) for f in (True, False) for s in (+1.0, -1.0)]
    return bodies, legs


def report():
    bodies, legs = assembly()
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
        print("  %-4s leg/trunk overlap %8.1f mm3%s" % (nm, v, flag))

    # --- ⚠️ every cable must end ON a spool the trunk actually has
    for i, (fore, side) in enumerate([(True, 1.0), (True, -1.0),
                                      (False, 1.0), (False, -1.0)]):
        nm = ["LF", "RF", "LR", "RR"][i]
        role = "fore" if fore else "hind"
        hx = HIP_X["front" if fore else "rear"]
        sp3 = TT.leg_spools(role, side)
        leg = DEFAULT_FORELEG if fore else DEFAULT_HINDLEG
        q = LD.LegModel(leg).inverse((LD.FOOT_X, LD.FOOT_Z, LD.FOOT_PITCH))
        spools = {t: (p3[0] - hx, p3[2])
                  for t, p3 in zip(("hip", "knee", "ankle"), sp3)}
        worst = 0.0
        for t in ("hip", "knee", "ankle"):
            st = LT.stations(q, t, +1, leg, spools=spools)
            end = np.asarray(st[0][0], float)
            worst = max(worst, float(np.hypot(*(end - np.array(spools[t])))))
        print("  %-4s cable ends off its spool by %.3f mm" % (nm, worst))
        if worst > 0.05:
            print("      *** the run does not reach the motor")
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
    print("  %s" % ("assembly checks pass" if ok else "*** SEE ABOVE ***"))
    return ok


def render_png(path, elev=16, azim=-62):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    bodies, legs = assembly()
    fig = plt.figure(figsize=(13, 7))
    ax = fig.add_subplot(111, projection="3d")
    # ⚠️ A few faces are valid and will not mesh -- see `tomcat_trunk.render_png`.
    # Drawn face by face so one bad face costs one face, not the whole robot.
    skipped = 0
    for col, group in (("#8a9bb0", bodies), ("#d7ac86", legs)):
        for s in group:
            for f in s.faces():
                try:
                    verts, tris = f.tessellate(0.4)
                except Exception:
                    skipped += 1
                    continue
                V = np.array([[v.X, v.Y, v.Z] for v in verts])
                coll = Poly3DCollection(V[np.array(tris)], facecolor=col,
                                        edgecolor="#3c4a5a", linewidth=0.06)
                coll.set_zsort("average")
                ax.add_collection3d(coll)
    if skipped:
        print("  *** " + str(skipped) + " face(s) would not mesh")
    whole = Compound(bodies + legs)
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
    render_png(os.path.join(HERE, "tomcat_assembly.png"))
    render_png(os.path.join(HERE, "tomcat_assembly_side.png"), elev=2, azim=-90)
