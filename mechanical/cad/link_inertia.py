# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Per-link inertia tensors for the MJCF -- and the rule that makes them well-posed.

[ADR-0087](../../docs/DESIGN_DECISIONS.md) measured the whole leg about the hip
and found the capsule model **37 % high**. Closing that gap in the MJCF needs a
tensor *per link*, which needs every CAD solid charged to a link.

⚠️ **Three earlier attempts did that by proximity to a LINK and came out
+105 %, -79 % and +20 % wrong.** The question was ill-posed, not merely hard: a
joint sits physically *between* two links, so neither is "nearest" in any stable
way, and small pose changes flip the answer.

✅ **ASSEMBLY_SPEC §2 never assigns hardware to a link by position.** It assigns
it to a **joint**, and then to that joint's **distal** link. Joints are isolated
points 70-100 mm apart, so "which joint" *is* well-posed. The same rule
`per_link_mass()` already implements, applied to placed solids instead of to
volumes.

Two independent checks, both in `tests/test_link_inertia.py`:

1. the per-link **masses** must reproduce `per_link_mass()` -- same rule, so any
   drift means the assignment is not the one the mass model uses;
2. summed about the hip they must reproduce **ADR-0087's 1.244e-3 kg m²**, which
   was measured with **no assignment at all**. A wrong split can still total
   correctly, but it cannot also match the mass split from an independent route.
"""

from __future__ import annotations

import numpy as np

import mass_properties as MP
import tomcat_leg_detail as LD

#: ⚠️ `tomcat_leg_detail` defines no UHMWPE density -- without one the tendon
#: and the ankle return cable carry geometry and **no mass**. 0.97 g/cm³ is the
#: Dyneema datasheet figure.  `[sourced: Dyneema SK75 datasheet]`
UHMWPE_RHO = 0.97e-3      # g/mm^3

RHO = {"tube": LD.CF_RHO, "insert": LD.AL_RHO, "clevis": LD.AL_RHO,
       "sheave": LD.AL_RHO, "bearing": LD.STEEL_RHO, "cable": UHMWPE_RHO,
       "tendon": UHMWPE_RHO, "pad": LD.TPU_RHO, "anchor": LD.AL_RHO}

LINKS = ("femur", "tibia", "meta", "paw")

#: ASSEMBLY_SPEC §2, and the exact dict `per_link_mass()` uses.
JOINT_TO_LINK = {"hip": "femur", "knee": "tibia", "ankle": "meta"}

#: Charged to a link outright, no geometry needed to decide.
FIXED = {"cable": "meta", "pad": "paw"}

#: ⚠️ Girdle motors are **not in the leg** (P1 centralisation) -- charging them
#: to the femur is exactly the mistake the tendon drive exists to avoid.
EXCLUDE = ("motor",)

#: Solids that lie ALONG a bone. Proximity to a bone *segment* is well-posed for
#: these -- unlike joint hardware, they are not shared between two links.
ALONG_BONE = ("tube", "insert", "tendon", "anchor")


def _props(sd, rho):
    """`(kg, m, kg m²)` from a build123d solid and a g/mm³ density.

    The one place the mm/g -> m/kg conversion lives. CAD unit confusion is a
    10⁹ error in a volume and it has already cost this project one afternoon.
    """
    m, c, I = MP.properties(sd, rho * 1e6)
    return m * 1e-9, np.asarray(c, float) * 1e-3, np.asarray(I) * 1e-15


def _seg_dist(p, a, b):
    """Distance from point `p` to segment `a`->`b`."""
    ab = b - a
    t = float(np.clip((p - a) @ ab / (ab @ ab), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


#: ⚠️ `build()` translates every solid to the limb plane on its last pass
#: (`c.locate(c.location * Pos(0, TRACK_Y, 0))`) but leaves `report`'s `p0`/`p1`
#: in the pre-translation frame. Joints and solids are therefore **48 mm apart
#: in y** unless this is added back.
#:
#: ADR-0087's about-hip number cannot catch that: `I_yy = sum m(x^2 + z^2)` does
#: not contain y at all, so it agreed to four figures with the hip in the wrong
#: place, while `I_xx` and `I_zz` were both wrong. Found by printing the centre
#: of mass beside the ratio -- the habit that caught the two M85 harness bugs.
HIP_Y = LD.TRACK_Y * 1e-3


def geometry(report):
    """Joint points and bone segments, in metres, in the SOLIDS' frame."""
    off = np.array([0.0, LD.TRACK_Y, 0.0])
    joints = {"hip": np.asarray(report["femur"]["p0"], float) + off,
              "knee": np.asarray(report["tibia"]["p0"], float) + off,
              "ankle": np.asarray(report["meta"]["p0"], float) + off}
    bones = {b: (np.asarray(report[b]["p0"], float) + off,
                 np.asarray(report[b]["p1"], float) + off) for b in LINKS}
    return ({k: v * 1e-3 for k, v in joints.items()},
            {k: (a * 1e-3, b * 1e-3) for k, (a, b) in bones.items()})


def assign(group, com, joints, bones):
    """Which link this solid's mass belongs to -- `None` if it is not in the leg."""
    if group in EXCLUDE:
        return None
    if group in FIXED:
        return FIXED[group]
    if group in ALONG_BONE:
        return min(bones, key=lambda b: _seg_dist(com, *bones[b]))
    # ⚠️ joint hardware: nearest JOINT, then §2's distal link. NOT nearest link.
    jn = min(joints, key=lambda j: float(np.linalg.norm(com - joints[j])))
    return JOINT_TO_LINK[jn]


def per_link(comps=None, report=None, calibrate=True):
    """Return `{link: (mass_kg, com_m, I_3x3_about_com)}` in the hip frame.

    ⚠️ `calibrate` rescales each group so its total equals `checks()`'s mass for
    that group, because **several CAD solids are fit envelopes, not parts**.
    `bearing()` says so in its own docstring: it draws a solid Ø19x6 steel
    annulus where the catalogue bearing is 8.0 g, so its volume weighs 12.0 g --
    **50 % over**, and 15.9 g over across the leg.

    The CAD is authoritative for **where** mass is; `per_link_mass()` is
    authoritative for **how much** (ADR-0087's own conclusion). Scaling by group
    keeps each, and leaves the per-*link* split -- the thing being tested --
    entirely to the geometry.
    """
    if comps is None or report is None:
        comps, report, _ = LD.build()
    joints, bones = geometry(report)
    _, ref_mass, _ = LD.checks(comps, report)

    bucket = {b: [] for b in LINKS}
    for group, comp in comps.items():
        rho = RHO.get(group)
        if rho is None:
            continue
        placed = [_props(sd, rho) for sd in comp.solids()]
        k = 1.0
        if calibrate and group in ref_mass:
            tot = sum(p[0] for p in placed)
            if tot > 0.0:
                k = 1e-3 * ref_mass[group] / tot
        for (m, c, I) in placed:
            link = assign(group, c, joints, bones)
            if link is not None:
                bucket[link].append((m * k, c, I * k))

    out = {}
    for b, parts in bucket.items():
        M = sum(p[0] for p in parts)
        com = sum(p[0] * p[1] for p in parts) / M
        I = np.zeros((3, 3))
        for (m, c, Ic) in parts:
            r = c - com
            I += Ic + m * (float(r @ r) * np.eye(3) - np.outer(r, r))
        out[b] = (M, com, I)
    return out


def link_frames(report):
    """`{link: (origin_m, R)}` where R's columns are the MJCF body axes in CAD.

    `mjcf_tendon` draws every bone as `fromto="0 0 0 L 0 0"`, so a link's body
    frame has **+x along the link**. The hinge axis is `0 -1 0` and rotation
    about y leaves y fixed, so the body's **+y stays world +y** down the whole
    chain; +z closes the right-handed set.

    ⚠️ Not asserted from that reasoning -- `main()` and the tests check that the
    distal joint lands at exactly `(L, 0, 0)`, which is false for any wrong R.
    """
    off = np.array([0.0, LD.TRACK_Y, 0.0])
    out = {}
    for b in LINKS:
        p = (np.asarray(report[b]["p0"], float) + off) * 1e-3
        d = (np.asarray(report[b]["p1"], float) + off) * 1e-3
        e1 = (d - p) / np.linalg.norm(d - p)
        e2 = np.array([0.0, 1.0, 0.0])
        out[b] = (p, np.column_stack([e1, e2, np.cross(e1, e2)]))
    return out


def in_link_frames(links, report):
    """Re-express `per_link()` in each body's own frame -- what MJCF wants."""
    frames = link_frames(report)
    out = {}
    for b, (m, com, I) in links.items():
        p, R = frames[b]
        out[b] = (m, R.T @ (com - p), R.T @ I @ R)
    return out


def about_hip(links, hip=None):
    """Total tensor about the hip -- the quantity ADR-0087 measured directly."""
    hip = np.array([0.0, HIP_Y, 0.0]) if hip is None else np.asarray(hip, float)
    I = np.zeros((3, 3))
    for (m, com, Ic) in links.values():
        r = com - hip
        I += Ic + m * (float(r @ r) * np.eye(3) - np.outer(r, r))
    return I


def mjcf_table(leg=None):
    """`{link: (mass, com_in_body_frame, fullinertia)}` ready for `<inertial>`.

    `fullinertia` is MJCF's order: ixx iyy izz ixy ixz iyz.
    """
    comps, report, _ = LD.build() if leg is None else LD.build(leg)
    out = {}
    for b, (m, com, I) in in_link_frames(per_link(comps, report), report).items():
        out[b] = (m, com, (I[0, 0], I[1, 1], I[2, 2],
                           I[0, 1], I[0, 2], I[1, 2]))
    return out


def main():
    comps, report, _ = LD.build()
    links = per_link(comps, report)
    ref, _ = LD.per_link_mass(comps, report)

    print("%-7s %9s %9s %7s   %-28s" % ("link", "assigned", "per_link_", "delta",
                                        "com from hip (mm)"))
    for b in LINKS:
        m, com, _ = links[b]
        print("%-7s %8.1fg %8.1fg %+6.1fg   (%6.1f %5.1f %7.1f)"
              % (b, 1e3 * m, ref[b], 1e3 * m - ref[b], *(1e3 * com)))
    print("%-7s %8.1fg %8.1fg" % ("total", 1e3 * sum(v[0] for v in links.values()),
                                  sum(ref.values())))
    raw = about_hip(per_link(comps, report, calibrate=False))
    I = about_hip(links)
    print("\n  envelope masses, what ADR-0087 used: I_yy = %.4e" % raw[1, 1])
    print("I about hip diag = [%.4e %.4e %.4e]" % tuple(np.diag(I)))
    print("  I_yy = %.4e   (ADR-0087 measured 1.244e-3 with no assignment)"
          % I[1, 1])

    print("\nin each body's own frame -- MJCF <inertial>")
    frames, lf = link_frames(report), in_link_frames(links, report)
    off = np.array([0.0, LD.TRACK_Y, 0.0])
    print("%-7s %8s %-24s %-34s %s"
          % ("link", "mass g", "com in body frame (mm)", "diag(I)", "distal joint"))
    for b in LINKS:
        m, com, I = lf[b]
        p, R = frames[b]
        d = R.T @ (((np.asarray(report[b]["p1"], float) + off) * 1e-3) - p)
        print("%-7s %8.1f (%6.1f %6.1f %6.1f)   [%.3e %.3e %.3e]  (%6.1f %5.1f %5.1f) mm"
              % (b, 1e3 * m, *(1e3 * com), *np.diag(I), *(1e3 * d)))


if __name__ == "__main__":
    main()
