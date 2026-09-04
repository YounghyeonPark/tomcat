# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Per-link inertia tensors from the manufacturing CAD, for the MJCF to use.

⚠️ **What the simulator does today.** `mjcf_tendon.py` draws each link as a
capsule and lets MuJoCo derive the inertia from that shape at uniform density.
The link *mass* is right — `params.py` carries `per_link_mass()`'s figures — but
a capsule of uniform density is not a carbon tube with an aluminium clevis at one
end and a Ø56 sheave at the other. `mass_closure.py` says so itself:

    `inertia_ratio` is used as a **first-order proxy** ... The real quantity is
    `Lambda = (J M^-1 J^T)^-1` ... which needs the per-link inertia tensors this
    ...

This computes them. `tomcat_leg_detail.build()` already places every part where
it belongs — tube, bonded insert, clevis, tongue, sheave, bearings, cable,
anchor, pad — so the tensor follows from geometry rather than assumption.

⚠️ **Apportionment follows ASSEMBLY_SPEC §2, the same rule `per_link_mass()`
uses**: a joint's sheave is fixed to its **distal** link, so each joint's
hardware is charged to the link below it. Getting this backwards would shift
mass one joint outboard and inflate the swing inertia P1 exists to protect.

⚠️ **Densities are `[assumed]` except the motor's**, and the motors are excluded
here anyway — they live in the girdle, not the leg (P1).
"""

from __future__ import annotations

import numpy as np

import body_inertia as BI
import tomcat_leg_detail as LD

MM = 1e-3

#: g/mm^3 in the detail CAD's own units, mirrored here so this file states what
#: it assumes rather than importing it silently.
#: ⚠️ `tomcat_leg_detail` defines CF, AL, TPU and STEEL but **no UHMWPE
#: density** — the cable has geometry there and no mass. 0.97 g/cm³ is the
#: published figure for Dyneema; at ~3 g for all five runs it changes little,
#: but leaving it undefined would silently drop the cable from the tensor.
UHMWPE_RHO = 0.97e-3      # g/mm^3  [sourced: Dyneema datasheet]

RHO = {"tube": LD.CF_RHO, "insert": LD.AL_RHO, "clevis": LD.AL_RHO,
       "sheave": LD.AL_RHO, "bearing": LD.STEEL_RHO, "cable": UHMWPE_RHO,
       "tendon": UHMWPE_RHO, "pad": LD.TPU_RHO, "anchor": LD.AL_RHO}

#: ASSEMBLY_SPEC §2: a joint's hardware belongs to the link BELOW it.
JOINT_LINK = {"hip": "femur", "knee": "tibia", "ankle": "meta"}
LINKS = ("femur", "tibia", "meta", "paw")


def _props(solid, rho_g_mm3):
    """Mass (kg), CoM (m) and inertia (kg m^2) for one CAD solid.

    The detail CAD works in mm with densities in g/mm^3, so the conversion is
    `g/mm^3 -> kg/m^3` is x1e6, then `mm^3 -> m^3` is x1e-9 on volume and the
    tensor picks up mm^2 -> m^2 as well.
    """
    return BI.solid_props(solid, rho_g_mm3 * 1e6)


def per_link_inertia(leg=None):
    """`{link: (mass, com_m, inertia_3x3)}` for one leg, from the placed parts.

    ⚠️ **Apportionment is `per_link_mass()`'s rule, not a geometric guess.**
    A first version assigned each solid to its nearest link segment and came out
    +105 % on the femur and -79 % on the metatarsus, because ASSEMBLY_SPEC §2
    is not a proximity rule: a joint sits BETWEEN two links, and its sheave,
    clevis and bearings all belong to the **distal** one. Proximity puts the knee
    stack on the femur, which is exactly the error that inflates swing inertia.

    Group-by-group, mirroring `tomcat_leg_detail.per_link_mass`:

    | group | goes to |
    |---|---|
    | tube, insert | its own bone |
    | clevis, sheave, bearing | the joint's **distal** link |
    | pad | paw |
    | cable | metatarsus (the ankle return run) |
    | tendon, anchor | spread over femur / tibia / meta |
    """
    comps, report, pts = LD.build(leg) if leg is not None else LD.build()
    joints = report["joints"]

    # where each joint sits, so a joint's hardware can be recognised by position
    jpos = {jn: np.asarray(d["p"], float) if "p" in d else None
            for jn, d in joints.items()}
    bone_end = {k: (np.asarray(report[k]["p0"], float),
                    np.asarray(report[k]["p1"], float)) for k in LINKS}

    def nearest_joint(p_mm):
        best, bestd = None, None
        for jn, link in JOINT_LINK.items():
            a = bone_end[link][0]          # the joint is the distal link's p0
            d = float(np.linalg.norm(p_mm - a))
            if bestd is None or d < bestd:
                best, bestd = jn, d
        return best

    def own_bone(p_mm):
        best, bestd = "femur", None
        for name, (a, b) in bone_end.items():
            ab = b - a
            t = np.clip(np.dot(p_mm - a, ab) / max(np.dot(ab, ab), 1e-9), 0, 1)
            d = float(np.linalg.norm(p_mm - (a + t * ab)))
            if bestd is None or d < bestd:
                best, bestd = name, d
        return best

    parts = {k: [] for k in LINKS}
    spread = []
    for group, solids in comps.items():
        if group == "motor":              # girdle-mounted, not in the leg (P1)
            continue
        rho = RHO.get(group)
        if rho is None:
            continue
        for sd in (s for c in solids for s in c.solids()):
            m, com, I = _props(sd, rho)
            if m <= 0:
                continue
            p_mm = com / MM
            if group in ("clevis", "sheave", "bearing"):
                parts[JOINT_LINK[nearest_joint(p_mm)]].append((m, com, I))
            elif group == "pad":
                parts["paw"].append((m, com, I))
            elif group == "cable":
                parts["meta"].append((m, com, I))
            elif group in ("tendon", "anchor"):
                spread.append((m, com, I))
            else:                          # tube, insert
                parts[own_bone(p_mm)].append((m, com, I))

    # tendons and anchors run ALONG the links, so charge each to the bone it
    # is nearest -- the spec spreads their MASS in thirds, but a tensor needs a
    # position, and the run's own centre is the honest one.
    for m, com, I in spread:
        parts[own_bone(com / MM)].append((m, com, I))

    return {k: BI.combine(parts[k]) for k in LINKS}, np.asarray(pts, float)
