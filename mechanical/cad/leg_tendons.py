# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The leg's tendon drive, routed and measured (M37).

Five cable runs and three motors per leg, per LEG_TENDON_SPEC §3.3 and ADR-0002/
ADR-0008:

| tendon | stations | drive |
|---|---|---|
| hip flexor / extensor | spool -> hip sheave -> anchor | one motor, variable-radius pulley |
| knee flexor / extensor | spool -> hip via -> knee sheave -> anchor | one motor |
| ankle (single) | spool -> hip via -> knee via -> ankle sheave -> anchor | one motor + return spring |

**Via-pulleys are placed CONCENTRIC with the proximal joint axis.** That is the
standard way to decouple a distal tendon from proximal motion: the centre distance
from a hip-concentric pulley to the knee axis is the femur length, which does not
change when the hip rotates, so the tangent span is invariant.

⚠️ **It is only invariant to first order, and the residual is the finding.** The
*wrap* on a via-pulley does change with the proximal angle, and an arc on a pulley
of radius `r_via` contributes `r_via * d(wrap)`. A physical via-pulley therefore
couples the joints by its own radius per radian, and `TendonMap.cable_lengths` is
a **diagonal** map — `delta = r * q`, each cable depending on its own joint only.
This module measures the off-diagonal terms the hardware actually produces.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                "kinematics", "src"))

import tendon_route as tr  # noqa: E402
from tomcat_kin import LegModel  # noqa: E402
from tomcat_kin.params import DEFAULT_LEG, DEFAULT_TENDON  # noqa: E402

MM = 1000.0

#: Spool pitch radius. §2 raised it 8.0 -> 8.75 mm for the Ø1.75 cable's minimum
#: bend; `TendonParams.motor_spool_radius` is **still 0.008** (M37 finding).
SPOOL_R = 8.75

#: Via-pulley radius. Set by the SAME minimum-bend rule as the spool, which is
#: what makes the coupling below unavoidable rather than a sizing mistake:
#: 10 x Ø1.75 cable = Ø17.5 minimum, so 8.75 mm is the smallest legal via-pulley.
VIA_R = 8.75

#: Anchor termination: a Brummel splice over a thimble (§3), modelled as a small
#: radius so it enters the tangent solve like any other station.
ANCHOR_R = 1.6

#: Spool position relative to the hip, in the pelvic girdle (P1: motors centralised).
SPOOL_OFFSET = (-42.0, 34.0)

ARMS = np.asarray(DEFAULT_TENDON.joint_moment_arm) * MM     # 28 / 25 / 14 mm
CABLE_D = 1.75


def joints(q, leg=DEFAULT_LEG):
    """Hip / knee / ankle / paw-base / paw-tip in the sagittal plane, mm."""
    return LegModel(leg).joint_positions(np.asarray(q, float)) * MM


def stations(q, tendon: str, side: int = +1, leg=DEFAULT_LEG):
    """Station list for one tendon run at joint angles `q`.

    `side` = +1 flexor, -1 extensor: the antagonist wraps its sheave the other
    way and anchors on the opposite side, which is what makes one motor drive
    both through the ADR-0008 variable-radius pulley.
    """
    p = joints(q, leg)
    hip, knee, ankle, paw = p[0], p[1], p[2], p[3]
    spool = hip + np.array(SPOOL_OFFSET)
    s = float(side)

    if tendon == "hip":
        return [(spool, SPOOL_R, s), (hip, ARMS[0], s),
                (_pin(hip, knee, ARMS[0], s), ANCHOR_R, s)]

    if tendon == "knee":
        return [(spool, SPOOL_R, s), (hip, VIA_R, s), (knee, ARMS[1], s),
                (_pin(knee, ankle, ARMS[1], s), ANCHOR_R, s)]

    if tendon == "ankle":
        return [(spool, SPOOL_R, s), (hip, VIA_R, s), (knee, VIA_R, s),
                (ankle, ARMS[2], s),
                (_pin(ankle, paw, ARMS[2], s), ANCHOR_R, s)]

    raise ValueError(tendon)


def _pin(joint, distal, arm: float, side: float):
    """Anchor-pin position on the distal link — a pin, per ASSEMBLY_SPEC §3.

    ⚠️ Placed just outside the sheave rim and offset along the link, so it reads
    as hardware bolted to the distal link rather than floating in space. Its exact
    position does **not** change the moment arm: the cable leaves the sheave
    *tangentially*, so the perpendicular distance from the joint axis to the cable
    line is the sheave radius wherever the pin sits — which is why
    `coupling_matrix` returns the arms exactly. The pin only sets how much of the
    sheave the cable wraps, and hence the capstan penalty.
    """
    joint, distal = np.asarray(joint, float), np.asarray(distal, float)
    along = (distal - joint) / max(float(np.linalg.norm(distal - joint)), 1e-9)
    return joint + along * (arm * 0.55) + _perp(along) * side * (arm + 5.0)


def _perp(v):
    v = np.asarray(v, float)
    n = float(np.linalg.norm(v))
    return np.array([-v[1], v[0]]) / (n if n > 1e-9 else 1.0)


def route(q, tendon: str, side: int = +1, leg=DEFAULT_LEG, senses=None):
    """Route one tendon, choosing the VIA-pulley wrap senses for minimum wrap.

    ⚠️ **The senses are not free-for-all and they are not arbitrary either.** The
    sense on a *sheave* is fixed — it is which way the tendon has to pull, i.e.
    flexor or extensor. The senses on the spool and the via-pulleys are a routing
    choice, and a first pass here left them all at `+1`, which sent the cable the
    long way round: **339 deg of wrap on a redirect pulley** against the 30-45 deg
    LEG_TENDON_SPEC §3.4 assumes, and a capstan penalty of 3.07x instead of 1.87x.
    That was a routing mistake being read as a physics result.

    Enumerating the free senses and taking the minimum total wrap is what a
    designer does by eye, and it is cheap: at most 2^3 combinations.
    """
    st = stations(q, tendon, side, leg)
    if senses is not None:
        st = [(c, r, sg) for (c, r, _), sg in zip(st, senses)]
        return tr.solve_path(st)

    n = len(st)
    free = [i for i in range(n) if i not in (_SHEAVE_IDX[tendon],)]
    best = None
    for bits in range(1 << len(free)):
        trial = list(st)
        for k, i in enumerate(free):
            sg = +1.0 if (bits >> k) & 1 else -1.0
            trial[i] = (trial[i][0], trial[i][1], sg)
        try:
            r = tr.solve_path(trial)
        except tr.NoTangent:
            continue
        if best is None or r["total_wrap"] < best["total_wrap"]:
            best = r
            best["senses"] = [t[2] for t in trial]
    if best is None:
        raise tr.NoTangent(f"no admissible routing for the {tendon} tendon")
    return best


#: Index of the station whose wrap sense is FIXED by the tendon's function.
_SHEAVE_IDX = {"hip": 1, "knee": 2, "ankle": 3}


def coupling_matrix(q0=None, h: float = 0.02, leg=DEFAULT_LEG):
    """d(cable length) / d(joint angle) for each tendon — the real tendon map.

    Rows are the three flexor tendons (hip, knee, ankle), columns the three joint
    angles. `TendonMap.cable_lengths` asserts this is **diagonal** with the moment
    arms on it; central differences on the routed geometry say what it is.

    Units mm/rad, so the diagonal should read 28 / 25 / 14.

    ⚠️ **The wrap senses are solved ONCE at `q0` and then held fixed.** Letting the
    minimum-wrap search re-run inside the finite difference makes it straddle a
    discontinuity where the optimiser changes its mind, and the ankle row then
    reads **678 mm/rad** — a differentiation artefact, not a coupling. Real
    hardware does not re-route itself between two nearby poses, so holding the
    senses is both the correct physics and the correct numerics.
    """
    if q0 is None:
        q0 = LegModel(leg).inverse((0.04, -0.17, 0.0))
    q0 = np.asarray(q0, float)
    fixed = {n: route(q0, n)["senses"] for n in ("hip", "knee", "ankle")}
    J = np.zeros((3, 3))
    for j, name in enumerate(("hip", "knee", "ankle")):
        for k in range(3):
            qp, qm = q0.copy(), q0.copy()
            qp[k] += h
            qm[k] -= h
            J[j, k] = (route(qp, name, senses=fixed[name])["length"]
                       - route(qm, name, senses=fixed[name])["length"]) / (2.0 * h)
    return J, q0
