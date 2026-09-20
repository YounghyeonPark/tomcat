# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The skin cover, sized by the strain the spine puts into it (build123d).

The skeleton has been the deliverable since M86 and the skin was always "after\nit". It is buildable now: `tomcat_trunk` gives a dorsal line the process tips
actually reach, `tomcat_assembly` puts four legs on it, and every rigid body is
one piece.

✅ **A skin over a bending spine has a NEUTRAL FIBRE, and finding it is the
whole design.** A fibre at height `z` changes length by `(z - spine_axis) *
theta` at each joint, so over the 75 deg of total pitch ROM:

    skin line                  z mm   r from axis    dL mm   strain
    dorsal, at the process tips 82.8        +33.0     43.2    11.9 %
    dorsal line                 79.8        +30.0     39.2    10.8 %
    FLANK, at the spine axis    49.8          0.0      0.0    0.0 %
    belly at the waist          14.7        -35.1     45.9    12.6 %
    belly at the chest         -46.1        -95.9    125.4    34.6 %

So the cover is **anchored along the flank at the spine axis** -- the one line
that does not move -- and the panels above and below carry the strain. 12 % is a
knit; 35 % is not. ⚠️ The belly panel is therefore not a stretch panel at all:
it is **slack**, gathered into a fold that pays out as the spine extends. A cat
has exactly that and it has a name, the primordial pouch, which is a good sign
for a shape arrived at from a strain table.

⚠️ **What this module does NOT do**: it is a cover, not a structure. It carries
no load, and nothing here checks that it keeps its shape under its own weight.
`[owed]`
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
from build123d import Compound, Cylinder, Ellipse, Plane, Pos, loft

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "kinematics", "src"))

import tomcat_trunk as TT                                        # noqa: E402
import tomcat_leg_detail as LD                                   # noqa: E402
from tomcat_kin.params import DEFAULT_SPINE as SP                # noqa: E402

MM = 1000.0

#: Clearance from the structure to the inside of the skin.
#:
#: ⚠️ It has to clear the PROCESS TIPS, not the barrel: the tips stand 3 mm
#: proud of the dorsal line and the barrel sits 4-12 mm under it through the
#: spine region, so a skin drawn on the barrel would be pierced at every joint.
CLEAR = 2.0

#: Skin thickness (mm). A knit over a 12 % strain panel.  `[assumed]`
THICK = 1.0

#: The flank anchor: the height at which the skin does not change length.
NEUTRAL_Z = float(SP.spine_axis_z) * MM

#: Height of a dorsal process tip -- the highest point of the skeleton, and what
#: the cover actually lies on.
TIP_Z = TT.SPINE_Z + TT.DORSAL_ARM + TT.POST_R


def outline(x):
    """`(half_width, top, belly)` of the SKIN at station x, in mm.

    ⚠️ The skin does not follow the necks. The structure pinches to 21 mm of
    half-width at a joint so it can bend; a cover that followed it would crease
    into the gap and foul the yokes. It spans the ROW sections instead, which is
    what the measured 4-12 mm of gap under the dorsal line is for.
    """
    hw = TT._row_hw_at(x) + CLEAR
    # ⚠️ **Hung from the PROCESS TIPS, not the dorsal line.** Drawn on the
    # dorsal line the cover sat at 81.8 against tips that reach 82.8 -- pierced
    # at every joint, by 1.0 mm, and the check below is what said so. The tips
    # are what a cat's back rests on and they are what this rests on too.
    top = TIP_Z + CLEAR
    zc = TT.Z_DORSAL - TT._row_hw_at(x) * TT.ASPECT
    return hw, top, zc - TT._row_hw_at(x) * TT.ASPECT - CLEAR


#: Total ROM about each axis, summed over the three joints.
PITCH_ROM = float(np.asarray(SP.q_max).sum())
YAW_ROM = float(np.asarray(SP.lateral_q_max).sum())


def fibre_strain(y, z, span):
    """`(pitch, yaw)` strain of a skin fibre at `(y, z)` over the full ROM.

    Pitch turns about the y axis AT the spine axis, so its lever is `z - axis`;
    yaw turns about the vertical through the centreline, so its lever is `y`.
    """
    return (abs(z - NEUTRAL_Z) * PITCH_ROM / span, abs(y) * YAW_ROM / span)


def anchor_at(x, span=None):
    """Where the cover is seamed at station x: `(y, z, strain)`.

    @W@ **There is no point neutral in BOTH, and the first version of this module
    assumed there was.** The flank at the spine axis takes zero in pitch -- which
    is why it was chosen -- and **9.8 %** in yaw, because yaw turns about the
    vertical and the flank is the furthest thing from it. The two neutral lines
    are perpendicular: pitch's is the flank, yaw's is the mid-sagittal plane,
    and the mid-sagittal line takes 12.6-34.7 % in pitch.

    @OK@ So the seam goes where the WORST of the two is least, and that is
    neither: the upper flank, **6.6 %** with pitch and yaw balanced, a third
    better than the flank it replaces.
    """
    if span is None:
        span = TT.BODIES[3][1] - TT.BODIES[0][0]
    hw, top, belly = outline(x)
    hh, zc = 0.5 * (top - belly), 0.5 * (top + belly)
    best = (9e9, 0.0, 0.0)
    for a in np.linspace(0.0, 0.5 * math.pi, 361):     # one quadrant; symmetric
        y, z = hw * math.cos(a), zc + hh * math.sin(a)
        w = max(fibre_strain(y, z, span))
        if w < best[0]:
            best = (w, y, z)
    return best[1], best[2], best[0]


def _sections():
    """The x stations the skin is lofted through: the ROW stations plus the ends."""
    xs = sorted({x for (_n, _b, x, _r, _ro) in TT._rows()})
    lo, hi = TT.BODIES[0][0], TT.BODIES[3][1]
    return [lo] + [x for x in xs if lo < x < hi] + [hi]


#: Radius of the hole each leg comes out of, and it is sized by the SWING.
#:
#: ✅ A leg does not leave the body through a gap, it leaves through an
#: aperture with a cuff -- 0.5 mm of flank clearance is nothing on a cover that
#: deflects. The hole has to pass the femur at every hip angle, so its radius is
#: the tube radius plus what the ROM sweeps plus a cuff.
APERTURE_R = LD.TUBE["femur"][0] / 2 + 12.0


def hip_apertures():
    """One hole per leg, on the limb plane at each hip station."""
    out = []
    for xh in (0.0, 195.0):
        zc = TT._zc(xh)
        for sgn in (-1.0, +1.0):
            out.append(Pos(xh, sgn * LD.TRACK_Y, zc)
                       * (Plane(origin=(0, 0, 0), z_dir=(0, 1, 0)).location
                          * Cylinder(APERTURE_R, 60.0)))
    return out


def skin():
    """The cover as a closed shell: an outer loft minus an inner one."""
    def shell(pad):
        secs = []
        for x in _sections():
            hw, top, belly = outline(x)
            hw += pad
            hh = 0.5 * (top - belly) + pad
            zc = 0.5 * (top + belly)
            secs.append(Plane(origin=(x, 0, zc), z_dir=(1, 0, 0))
                        * Ellipse(hh, hw))
        return loft(secs)
    g = shell(0.0) - shell(-THICK)
    for a in hip_apertures():
        g = g - a
    return g


def strain_table():
    """Length change of a skin fibre at each height, over the full pitch ROM."""
    q = float(np.asarray(SP.q_max).sum())
    span = TT.BODIES[3][1] - TT.BODIES[0][0]
    rows = []
    for name, z in (("dorsal, over the tips", outline(0.0)[1]),
                    ("flank, the ANCHOR", NEUTRAL_Z),
                    ("belly at the waist", outline(107.0)[2]),
                    ("belly at the chest", outline(217.0)[2])):
        r = z - NEUTRAL_Z
        dL = abs(r) * q
        rows.append((name, z, r, dL, dL / span))
    return rows, q, span


def report():
    ok = True
    print("SKIN COVER -- sized by the strain the spine puts into it\n")
    print("%-24s %8s %8s %8s" % ("station", "hw", "top", "belly"))
    for x in _sections():
        hw, top, belly = outline(x)
        print("%-24s %8.1f %8.1f %8.1f" % ("x = %.0f" % x, hw, top, belly))

    rows, q, span = strain_table()
    print("\nfibre length change over %.0f deg of total pitch ROM, span %.0f mm"
          % (math.degrees(q), span))
    print("%-24s %8s %10s %8s %9s" % ("line", "z mm", "r", "dL mm", "strain"))
    for name, z, r, dL, e in rows:
        print("%-24s %8.1f %10.1f %8.1f %8.1f %%" % (name, z, r, dL, 100 * e))

    print("\nthe seam, where the WORST of pitch and yaw is least")
    print("%-12s %9s %9s %9s %9s" % ("x", "y", "z", "pitch %", "yaw %"))
    worst_seam = 0.0
    for x in _sections():
        y, z, w = anchor_at(x)
        ep, ey = fibre_strain(y, z, span)
        worst_seam = max(worst_seam, w)
        print("%-12s %9.1f %9.1f %8.1f %% %8.1f %%"
              % ("x = %.0f" % x, y, z, 100 * ep, 100 * ey))
    flank_yaw = fibre_strain(outline(0.0)[0], NEUTRAL_Z, span)[1]
    print("  seam worst %.1f %% -- the flank at the spine axis would be %.1f %%"
          % (100 * worst_seam, 100 * flank_yaw))
    if worst_seam > flank_yaw:
        print("      *** the seam is worse than the plain flank")
        ok = False

    sk = skin()
    sol = sk.solids()
    vol = sum(x.volume for x in sol)
    print("\nsolids          %d %s" % (len(sol), "(one cover)" if len(sol) == 1
                                          else "*** DISJOINT ***"))
    if len(sol) != 1:
        ok = False
    for name, rho in (("knit nylon", 1.15e-3), ("TPU 85A", 1.20e-3),
                      ("silicone", 1.10e-3)):
        print("  %-14s %6.1f g at %.1f mm" % (name, vol * rho, THICK))

    # @W@ the cover must clear the PROCESS TIPS, not just the barrel
    # ⚠️ **Read from the geometry, not recomputed.** A first version of this
    # check rebuilt the skin's top from `Z_DORSAL + CLEAR`, so when `outline()`
    # was raised onto the tips the check did not move with it and went on
    # reporting the defect it had just caught. A check that recomputes what it
    # is checking is checking something else.
    gap = min(outline(x)[1] for x in _sections()) - TIP_Z
    print("\ndorsal clearance over the process tips  %+.1f mm" % gap)
    if gap < 0.5:
        print("      *** the skin would be pierced at every joint")
        ok = False
    # ⚠️ **Does it actually enclose the structure?** Clearance at the dorsal
    # line says nothing about the flanks, and the trunk's widest section is a
    # girdle, not a row the skin was lofted through.
    worst, where = 1e9, None
    for x in np.linspace(TT.BODIES[0][0], TT.BODIES[3][1], 60):
        hw, top, belly = outline(x)
        s_hw = TT._hw_at(x)
        s_top = TT._zc(x) + s_hw * TT.ASPECT
        s_belly = TT._zc(x) - s_hw * TT.ASPECT
        for name, m in (("flank", hw - s_hw),
                        ("top", top - s_top),
                        ("belly", s_belly - belly)):
            if m < worst:
                worst, where = m, (name, x)
    print("  tightest clearance to the structure %+.1f mm (%s at x = %.0f)"
          % (worst, where[0], where[1]))
    if worst < 0.5:
        print("      *** the skin is inside the skeleton there")
        ok = False

    # ⚠️ **And the legs have to come out of it.** A binary "does it foul" is the
    # wrong question for a flexible cover: it deflects, so what matters is the
    # margin. Measured at both hip stations against the femur's inner face.
    inner = LD.TRACK_Y - LD.TUBE["femur"][0] / 2
    for nm, xh in (("rear", 0.0), ("fore", 195.0)):
        print("  %s hip        femur inner face %.1f vs skin flank %.1f  -> "
              "%+.1f mm, so it leaves through an aperture"
              % (nm, inner, outline(xh)[0], inner - outline(xh)[0]))
    print("  aperture           r = %.1f mm (femur r %.1f + %.1f of cuff and sweep)"
          % (APERTURE_R, LD.TUBE["femur"][0] / 2, APERTURE_R - LD.TUBE["femur"][0] / 2))
    if APERTURE_R < LD.TUBE["femur"][0] / 2 + 5.0:
        print("      *** the hole is not bigger than the leg")
        ok = False

    print("  %s" % ("skin checks pass" if ok else "*** SEE ABOVE ***"))
    return ok


def render_png(path, elev=14, azim=-62, with_trunk=True):
    """The cover over the skeleton, drawn translucent so both read."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(12, 6))
    ax = fig.add_subplot(111, projection="3d")
    groups = []
    if with_trunk:
        groups.append(("#8a9bb0", 1.0, [TT.rigid_body(b) for b in sorted(TT.BODIES)]))
    groups.append(("#c98f6a", 0.45, [skin()]))
    skipped = 0
    for col, alpha, shapes in groups:
        tri = []
        for sh in shapes:
            for f in sh.faces():
                try:
                    verts, tris = f.tessellate(0.5)
                except Exception:
                    skipped += 1
                    continue
                V = np.array([[v.X, v.Y, v.Z] for v in verts])
                tri.append(V[np.array(tris)])
        if not tri:
            continue
        coll = Poly3DCollection(np.concatenate(tri), facecolor=col, alpha=alpha,
                                edgecolor="#3c4a5a", linewidth=0.05)
        coll.set_zsort("average")
        ax.add_collection3d(coll)
    if skipped:
        print("  *** %d face(s) would not mesh" % skipped)
    bb = Compound([skin()]).bounding_box()
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
    render_png(os.path.join(HERE, "tomcat_skin.png"))
    render_png(os.path.join(HERE, "tomcat_skin_side.png"), elev=2, azim=-90)
