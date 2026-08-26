# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Tendon routing geometry — the part that makes it a tendon DRIVE (M37).

`tomcat_leg_detail.py` drew the joint hardware: sheaves, clevises, bearings,
bonded inserts. It did not draw a tendon. Without the cable, the spool, the
anchors and the antagonistic pairing it is a linkage with pulleys bolted to it —
design principle **P1** is the whole premise of this robot and it was the one
thing the manufacturing model omitted.

This module is the routing itself, solved rather than sketched.

The maths
---------
A tendon over a set of pulleys is a **belt problem**. For each station give a
centre, a radius and a wrap **sense** (+1 counter-clockwise, -1 clockwise); the
path is then the unique sequence of common tangents and arcs. Using a *signed*
radius `R = s*r`, the common tangent of two stations has unit normal `n` with

    n . (c2 - c1) = R1 - R2,     tangent points  p_i = c_i + R_i * n

Derivation, because the sign is easy to get backwards and a first pass here did:
writing the line as `{x : n.x = k}` and requiring `p_i = c_i + R_i n` to lie on
it gives `n.c_i + R_i = k`, hence `R_1 - R_2 = n.(c_2 - c_1)`. Checked against
the closed forms: equal circles same sense give a tangent equal to the centre
distance, opposite sense `sqrt(L^2 - (r1+r2)^2)`.

which handles open (same sense) and crossed (opposite sense) belts with one
formula, and fails loudly — `|R2-R1| > L` — when no tangent exists, i.e. when one
pulley swallows the other.

What it is for
--------------
Three things the sketch could not answer:

1. **Does the routing deliver the moment arm the tendon map assumes?**
   `TendonParams.joint_moment_arm` is what `tomcat_kin` converts joint angle to
   cable travel with. That is only true if `d(path length)/d(theta) = r`. Here it
   is measured off the geometry instead of asserted.
2. **Is every pulley above the cable's minimum bend radius?**
3. **What does the capstan actually cost?** LEG_TENDON_SPEC §3.4 estimates the
   ankle path at 1.87x from assumed wrap angles; the wraps are computable.
"""

from __future__ import annotations

import math

import numpy as np

#: UHMWPE minimum sheave DIAMETER as a multiple of cable diameter (§2).
MIN_SHEAVE_D_RATIO = 10.0

#: Capstan coefficient, UHMWPE on lightly-lubricated anodised aluminium (§3.4).
MU_PULLEY = 0.10


class NoTangent(ValueError):
    """Raised when two stations admit no common tangent at the given senses."""


def _rot90(v):
    return np.array([-v[1], v[0]])


def tangent(c1, r1, s1, c2, r2, s2):
    """Common tangent between two wrapped circles. Returns (p1, p2, n).

    `s` is the wrap sense: +1 counter-clockwise, -1 clockwise, in the (x, z)
    plane. The signed-radius form means an open belt and a crossed belt are the
    same computation.
    """
    c1, c2 = np.asarray(c1, float), np.asarray(c2, float)
    R1, R2 = s1 * r1, s2 * r2
    d = c2 - c1
    L = float(np.linalg.norm(d))
    if L < 1e-9:
        raise NoTangent("coincident stations")
    u = d / L
    cos_t = (R1 - R2) / L
    if abs(cos_t) > 1.0:
        raise NoTangent(
            f"|R1-R2| = {abs(R1 - R2):.2f} exceeds the {L:.2f} mm centre "
            f"distance — one pulley swallows the other")
    # two solutions; the sign of the sine picks which side the belt leaves on,
    # and the wrap sense of the FIRST station fixes it.
    sin_t = math.sqrt(max(0.0, 1.0 - cos_t * cos_t)) * (-s1)
    n = cos_t * u + sin_t * _rot90(u)
    return c1 + R1 * n, c2 + R2 * n, n


def arc_angle(c, r, s, p_in, p_out):
    """Wrap angle (rad) travelled on a station between two tangent points."""
    a_in = math.atan2(*(np.asarray(p_in, float) - c)[::-1])
    a_out = math.atan2(*(np.asarray(p_out, float) - c)[::-1])
    delta = (a_out - a_in) * s
    while delta < 0.0:
        delta += 2.0 * math.pi
    while delta > 2.0 * math.pi:
        delta -= 2.0 * math.pi
    return delta * 1.0


def solve_path(stations):
    """Route a tendon over `stations` = [(centre, radius, sense), ...].

    The first and last stations are treated as **terminations** (the spool, and
    the anchor) rather than as wraps, so their arcs are reported but the path is
    measured between the first tangent point and the last.

    Returns a dict with `points` (the polyline through tangent points),
    `length`, `wraps` (rad per station) and `capstan` (the tension ratio the
    motor must supply over what the joint receives).
    """
    pts, wraps, segs = [], [], []
    tangents = []
    for i in range(len(stations) - 1):
        (c1, r1, s1), (c2, r2, s2) = stations[i], stations[i + 1]
        p1, p2, _ = tangent(c1, r1, s1, c2, r2, s2)
        tangents.append((p1, p2))
        segs.append(float(np.linalg.norm(p2 - p1)))

    for i, (c, r, s) in enumerate(stations):
        p_in = tangents[i - 1][1] if i > 0 else None
        p_out = tangents[i][0] if i < len(tangents) else None
        if p_in is None or p_out is None:
            wraps.append(0.0)
            continue
        wraps.append(arc_angle(np.asarray(c, float), r, s, p_in, p_out))

    # polyline = tangent segment, then the arc on the next station, ...
    for i, (p1, p2) in enumerate(tangents):
        pts.append(p1)
        pts.append(p2)
        j = i + 1
        if j < len(stations) and wraps[j] > 1e-9:
            c, r, sgn = stations[j]
            c = np.asarray(c, float)
            a0 = math.atan2(*(p2 - c)[::-1])
            n_arc = max(2, int(math.degrees(wraps[j]) / 8.0))
            for k in range(1, n_arc + 1):
                a = a0 + sgn * wraps[j] * k / n_arc
                pts.append(c + r * np.array([math.cos(a), math.sin(a)]))

    arc_len = sum(r * w for (_, r, _), w in zip(stations, wraps))
    total = sum(segs) + arc_len
    theta = sum(wraps)
    return {"points": np.array(pts), "length": total, "segments": segs,
            "wraps": np.array(wraps), "arc_length": arc_len,
            "total_wrap": theta,
            "capstan": math.exp(MU_PULLEY * theta)}


def min_bend_check(stations, cable_d: float, exempt_terminations: bool = True):
    """Every RUNNING station must clear the cable's minimum sheave diameter.

    ⚠️ The last station is the anchor pin, and it is exempt by default: a spliced
    eye over a thimble (ASSEMBLY_SPEC §3) is a static termination, not a running
    bend, so the fatigue argument behind the 10x rule does not apply to it. The
    thimble still needs a sane radius — that is a `[owed]` detail, not a violation
    of this rule.
    """
    need = MIN_SHEAVE_D_RATIO * cable_d / 2.0
    n = len(stations)
    out = []
    for i, (_, r, _) in enumerate(stations):
        termination = exempt_terminations and i == n - 1
        out.append((termination or r >= need - 1e-9, r, need))
    return out


def travel_vs_moment_arm(path_fn, angles, arm: float):
    """Is `d(length)/d(theta)` actually the moment arm the tendon map assumes?

    `path_fn(theta) -> length`. Returns (measured slope, max deviation from
    `arm`), both in mm per rad. A routing whose slope is not `arm` means
    `TendonParams.joint_moment_arm` is wrong for that joint, and every torque and
    tension derived from it with it.
    """
    L = np.array([path_fn(t) for t in angles])
    slope = np.gradient(L, angles)
    return slope, float(np.max(np.abs(slope - arm)))
