# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The girdle composites, with the head and the tail in their real places.

⚠️ **`params.py` tagged these `[derived: cad/tomcat_packaging.py]` and that
module does not produce them.** It packs motors and sizes housings; it never
prints a mass, a CoM or an inertia tensor. The girdle numbers were computed
once, by hand or by a script that did not survive, and carried forward for
nine milestones. This file is the missing derivation, and it re-derives the
PUBLISHED numbers before replacing them.

What it fixes:

- **The head had no place.** `front_girdle_mass` absorbs 240 g of head and neck
  and `front_girdle_inertia` spread it uniformly through the housing box --
  `params.py` said so in its own comment and nobody had moved it. The head's
  CoM is **149.5 mm ahead of the hip and 118.3 above it**; spread through the
  housing it sat at (0, 23.1).
- **The tail was not in the model at all**, nor was the 19th motor that drives
  it (ADR-0096).

The reconstruction is checked, not assumed: strip the head-as-housing-lump out
of the published front girdle and what is left must be the rear girdle plus
20 g of structure. It is, to **0.5 % on Ixz** and exactly on Iyz.

⚠️ **This lands on an architecture M88 replaced.** The model is rear girdle +
3 segments + front girdle; the CAD is four rigid bodies with motors spread
6/4/2/6. Mapping body 3 -> front girdle and body 0 -> rear girdle is faithful
for the LEG motors and for these two appendages, and it is not the
re-apportionment `params.py` has owed since M92. These numbers will move
again.  `[owed]`
"""

from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "kinematics", "src"))

import link_inertia as LI                                        # noqa: E402
import tomcat_head as HD                                         # noqa: E402
import tomcat_tail as TL                                         # noqa: E402
import tomcat_trunk as TT                                        # noqa: E402
import g3_flexure as G3F                                         # noqa: E402
from tomcat_kin.params import DEFAULT_SPINE as SP                # noqa: E402

#: EVA foam. ⚠️ **The material is a righting decision, not a finish one**
#: (ADR-0096): the tail is all lever, so it adds 14x more pitch inertia than
#: the authority its curl buys, and the righting reflex is already a factor of
#: nine short. Silicone would cost 11.7 % of the body's pitch inertia for 0.8 %
#: of authority; foam costs 2.0 %.
#: `[assumed]` -- 0.20 g/cm3 is a structural EVA, and the cable route through
#: it is undrawn.  `[owed]`
RHO_TAIL = 2.0e-4        # g/mm3

#: SteadyWin GIM3505-9, the surveyed part, in a 34.5 x 36.1 mm can.
#:
#: ⚠️ **The repo carries two masses for the same motor.** `params.py` builds
#: both girdles from **0.132** (1.122 = 6 x 0.132 + 0.240 + 0.090 only at that
#: rounding), while ADR-0096, `mass_closure` and `thermal` all use **131.7 g**.
#: The 0.3 g matters not at all to the physics and entirely to whether the mass
#: accounting closes, so the girdle bookkeeping uses the figure the girdles were
#: built from. Reconciling the two is not this milestone's job.  `[owed]`
MOTOR_M, MOTOR_D, MOTOR_L = 0.132, 0.0345, 0.0361

#: ⚠️ **A derivation that reads its own output is not a derivation.** The first
#: version of this file took the girdle baseline from `SP` -- the live
#: `params.py` -- and produced the corrected numbers from it. Run once it was
#: right; run again, after `params.py` had been updated, it applied the
#: correction to its own result and the front girdle's Iyy went to 1.2e-02 with
#: the head placed twice. The reconstruction check caught it (`Ixz` off by
#: **713 %**), which is what that check is for, but the fix is not to notice
#: better: the baseline is FROZEN here, as the M101 published values, so this
#: file computes the same answer however many times it runs.
#:
#: These are the pre-M102 values of `SpineParams`, verbatim.
M101 = {
    "front_mass": 1.122,
    "front_com": (-0.00461, 0.01739),
    "front_inertia": (1.2577e-03, 1.1412e-03, 9.8045e-04,
                      0.0, 1.4855e-04, -1.7442e-05),
    "rear_mass": 0.902,
    "rear_com": (-0.00574, 0.01600),
    "rear_inertia": (8.9693e-04, 7.8852e-04, 7.1418e-04,
                     0.0, 1.5575e-04, -1.7442e-05),
    "segment_mass_mid": 1.354,
    "segment_inertia_mid": (1.4110e-03, 1.4110e-03, 1.1042e-03,
                            5.3457e-05, 6.3457e-05, 6.3457e-05),
}


def _full_to_mat(fi):
    ixx, iyy, izz, ixy, ixz, iyz = fi
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], float)


def _mat_to_full(I):
    return (I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2])


def _shift(I, m, d):
    """An inertia about a CoM, moved to a point the vector `d` away from it."""
    return I + m * (float(d @ d) * np.eye(3) - np.outer(d, d))


def _box_I(m, size):
    x, y, z = size
    return np.diag([m / 12 * (y * y + z * z),
                    m / 12 * (x * x + z * z),
                    m / 12 * (x * x + y * y)])


def combine(parts):
    """`[(m, com(3), I_about_own_com)]` -> `(M, com, I_about_the_new_com)`."""
    M = sum(p[0] for p in parts)
    com = sum(p[0] * p[1] for p in parts) / M
    I = np.zeros((3, 3))
    for m, c, Ic in parts:
        I += _shift(Ic, m, c - com)
    return M, com, I


def of_solids(solids, rho):
    """`(kg, com_m, I_about_com)` for build123d solids at one g/mm3 density."""
    return combine([LI._props(sd, rho) for sd in solids])


def head():
    """The head as it is DRAWN, in the front girdle's frame.

    ⚠️ Cranium **and** face: `tomcat_head` draws `whole()` and `features()`,
    and the ears sit high and back. They move the CoM 1.4 mm forward and 1.6 up
    -- small, and small is not zero.
    """
    sol = list(HD.whole().solids()) + list(HD.features().solids())
    rho = HD.HEAD_NECK_G / sum(s.volume for s in sol)   # to the 240 g budget
    m, c, I = of_solids(sol, rho)
    return m, c - np.array([0.195, 0.0, 0.0]), I        # trunk -> front girdle


def tail():
    """The tail in the rear girdle's frame, whose origin is the spine's root joint
    (the hind hips sit `rear_hip_x` behind it since M122)."""
    return of_solids(list(TL.tail(0.0).solids()), RHO_TAIL)


def tail_motor():
    """The 19th motor, where `tomcat_trunk` places it: body 0, `tri3`, on the
    centreline. A can about its own CoM, axis along y."""
    bores = [b for b in TT.motors(0) if abs(b[1]) < 1e-6 and b[2] > 0.0]
    if len(bores) != 1:
        raise RuntimeError("expected one centreline bore on body 0, got %d"
                           % len(bores))
    x, _y, z, _r = bores[0]
    r, L = MOTOR_D / 2, MOTOR_L
    I = np.diag([MOTOR_M * (3 * r * r + L * L) / 12,
                 MOTOR_M * r * r / 2,
                 MOTOR_M * (3 * r * r + L * L) / 12])
    return MOTOR_M, np.array([x / 1000.0, 0.0, z / 1000.0]), I


#: Girdle body -> the trunk x of its model frame's origin (the hip).
_GIRDLE = {"fore": (3, 0.195), "hind": (0, 0.0)}


def g3_parts(role: str):
    """The six leg G3 flexures on one girdle, as thin Ti discs (M120).

    Each sits between its motor and its spool, on the motor's axis (x).
    ✅ M122: at the END of the row its spool is on (`tendon_exit.DRIVE`), which
    the trunk now says -- M120 had it at the row's centre, `[owed]`.
    """
    import tendon_exit as TE
    import tomcat_packaging as TP
    body, x_org = _GIRDLE[role]
    m = G3F.mass_g() * 1e-3
    r = G3F.R_RIM_OUT * 1e-3
    I = np.diag([m * r * r / 2, m * r * r / 4, m * r * r / 4])
    stack = TT.G3_STACK[role]
    out = []
    for side in (+1.0, -1.0):
        own = TE.motors(role, side)
        for t in ("hip", "knee", "ankle"):
            mi, end = TE.DRIVE[role][t][:2]
            _nm, _x, y, z = own[mi]
            xs = TE.spool_x_of(role, mi, end, side)
            xg = xs - end * (TP.SPOOL_L / 2 + stack / 2)
            out.append((m, np.array([xg * 1e-3 - x_org, y * 1e-3, z * 1e-3]), I))
    return out


#: ✅ **M123 (ADR-0114): the lump IS the new layout.** The M101 girdle is a
#: housing box about its hip (CoM 5.7 mm behind it), not the motors at M88's
#: row positions -- subtracting them from there gave a negative inertia. Each
#: girdle's six leg motors now sit in two rows straddling its hip, which is
#: the box the lump always was; it moves with its hip, as in M122.
def spine_g3_kg():
    """M120: the six spine G3 flexures, kg -- they ride segment_mass[1]."""
    from tomcat_kin.params import DEFAULT_SPINE
    return 6 * G3F.mass_g(DEFAULT_SPINE.series_k) * 1e-3


def front_without_head():
    """The published front girdle with the head-as-housing-lump removed.

    This is the reconstruction the module docstring promises. The head was
    spread uniformly through `girdle_size`, centred `girdle_offset_z` above the
    mount; removing it must leave the rear girdle plus 20 g of structure.
    """
    m = M101["front_mass"]
    c = np.array([M101["front_com"][0], 0.0, M101["front_com"][1]])
    I = _full_to_mat(M101["front_inertia"])
    hm = HD.HEAD_NECK_G / 1000.0
    hc = np.array([0.0, 0.0, SP.girdle_offset_z])
    hI = _box_I(hm, np.array(SP.girdle_size, float))
    rest_m = m - hm
    rest_c = (m * c - hm * hc) / rest_m
    # everything about the girdle ORIGIN, minus the lump, back to the new CoM
    I_org = _shift(I, m, c) - _shift(hI, hm, hc)
    return rest_m, rest_c, I_org - _shift(np.zeros((3, 3)), rest_m, rest_c)


def front():
    """`(mass, com_xz, fullinertia)` for the front girdle with the head placed."""
    # M123: the lump rides the fore hip, `front_hip_x` ahead of the frame
    m, c, I = front_without_head()
    c = c + np.array([SP.front_hip_x, 0.0, 0.0])
    M, c, I = combine([(m, c, I), head()] + g3_parts("fore"))
    return M, (c[0], c[2]), _mat_to_full(I)


def rear():
    """`(mass, com_xz, fullinertia)` for the rear girdle with the tail placed.

    ⚠️ M122 (ADR-0112): the frame origin is the spine's ROOT JOINT, and the
    hind hips -- and the whole motor bank the M101 lump holds -- now sit
    `rear_hip_x` (30 mm) behind it. The lump moves with them; the tail and its
    motor come from the CAD, which moved them already."""
    m = M101["rear_mass"]
    c = np.array([M101["rear_com"][0] + SP.rear_hip_x, 0.0, M101["rear_com"][1]])
    I = _full_to_mat(M101["rear_inertia"])
    M, cc, II = combine([(m, c, I), tail_motor(), tail()] + g3_parts("hind"))
    return M, (cc[0], cc[2]), _mat_to_full(II)


def _fmt(fi):
    return ", ".join("%.4e" % (0.0 if abs(v) < 1e-9 else v) for v in fi)


def report():
    ok = True
    print("GIRDLE COMPOSITES -- the head and the tail, placed\n")

    rm, rc, rI = front_without_head()
    pub = np.array(M101["rear_inertia"], float)
    got = np.array(_mat_to_full(rI), float)
    print("  reconstruction check: strip the head lump out of the front girdle")
    print("    left with   %s" % _fmt(got))
    print("    rear says   %s" % _fmt(pub))
    # ⚠️ The DIAGONAL differs by the 20 g of structure the rear carries and the
    # front does not; the off-diagonal Ixz has no such excuse and is the real
    # test of whether the lump was reconstructed right.
    dxz = abs(got[4] - pub[4]) / abs(pub[4])
    print("    Ixz agrees to %.2f %%   (the diagonal differs by 20 g of structure)"
          % (100 * dxz))
    if dxz > 0.02:
        print("      *** the head was not spread the way params.py says it was")
        ok = False

    for nm, got_v, was_m, was_c, was_i in (
            ("front", front(), M101["front_mass"], M101["front_com"],
             M101["front_inertia"]),
            ("rear", rear(), M101["rear_mass"], M101["rear_com"],
             M101["rear_inertia"])):
        M, c, fi = got_v
        print("\n  %s girdle" % nm)
        print("    mass  %.5f -> %.5f kg" % (was_m, M))
        print("    com   (%+.5f, %+.5f) -> (%+.5f, %+.5f) m   moves (%+.1f, %+.1f) mm"
              % (was_c[0], was_c[1], c[0], c[1],
                 1000 * (c[0] - was_c[0]), 1000 * (c[1] - was_c[1])))
        print("    Iyy   %.4e -> %.4e   x%.1f" % (was_i[1], fi[1], fi[1] / was_i[1]))
        print("    fullinertia = (%s)" % _fmt(fi))
        # ⚠️ MuJoCo rejects a tensor whose principal moments break the triangle
        # inequality, and a parallel-axis sum can produce one if a sign is
        # wrong. Checked here rather than discovered as a model load failure.
        w = np.linalg.eigvalsh(_full_to_mat(fi))
        if w[0] <= 0 or w[0] + w[1] < w[2] - 1e-12:
            print("      *** principal moments %s are not a rigid body" % w)
            ok = False

    Mt, ct, _It = tail()
    mm, mc, _mI = tail_motor()
    print("\n  the tail is %.1f g of EVA foam at (%+.1f, %+.1f) mm; its motor is "
          "%.1f g at (%+.1f, %+.1f)"
          % (1000 * Mt, 1000 * ct[0], 1000 * ct[2],
             1000 * mm, 1000 * mc[0], 1000 * mc[2]))
    moved = mm + Mt
    mid = M101["segment_mass_mid"]
    g3 = spine_g3_kg()
    k = (mid - moved + g3) / mid
    print("  %.4f kg leaves segment_mass[1] and the spine's six G3 add %.4f: "
          "%.4f -> %.4f" % (moved, g3, mid, mid - moved + g3))
    print("  segment_inertia[1] scales by %.5f -> (%s)"
          % (k, ", ".join("%.4e" % (v * k) for v in M101["segment_inertia_mid"])))
    print("\n  %s" % ("girdle checks pass" if ok else "*** SEE ABOVE ***"))
    return ok


if __name__ == "__main__":
    report()
