# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The viable set — what is recoverable *by any controller*, computed exactly.

Every envelope figure in this project so far has been the achievement of **some
controller**: `control.py`'s 1-D deadbeat law, or M21–M27's MuJoCo harness. That
leaves the question this arc kept hitting unanswerable — when a measurement falls
short of a prediction, is the model optimistic or is the controller poor? M27 ended
with three actuators all failing to deliver credited authority, which indicts the
controller but cannot prove the model.

This module removes the controller from the question. It computes the set of
disturbances from which recovery is **possible at all**, so:

- if a disturbance is **outside** the set, no controller recovers from it, and a
  model that claims otherwise is optimistic;
- if it is **inside**, the authority exists and any shortfall is the controller's.

⚠️ This is exact *within the LIPM class* — the same class `control.py` uses — and
inherits its assumptions (constant CoM height, `dH/dt = 0`). M17 measured the cost
of those: real divergence is ~2 % SLOWER than LIPM, so the model is conservative
there. What it does NOT inherit is the 1-D collapse: this is fully two-dimensional,
which is the defect ADR-0031 identified.

The derivation
--------------
Over one stance of duration `T` with the centre of pressure free to move inside the
support set `S`, the DCM obeys `xi_dot = omega (xi - p)`. Integrating::

    xi(T) = g xi(0) - (g - 1) u,    g = e^(omega T)

where `u` is the exponentially-weighted mean of `p(t)`. Because `S` is convex, that
mean lies in `S` — so the reachable `u` is exactly `S`, with no relaxation. The set
of states from which the origin is reachable in `k` more steps is then::

    R_0 = {0},   R_(k+1) = (R_k + (g - 1) S_k) / g

a Minkowski sum of scaled convex polygons, computable in closed form. `S_k`
alternates between the two diagonals, which is what makes this see the 52.4 deg axis
split that a single-axis model cannot (ADR-0022, ADR-0031).

The support set
---------------
The legs are **planar** — ADR-0017 rejected abduction — so each foot moves only
fore-aft. Sweeping the segment between the two stance feet along `x` over the reach
range gives a **parallelogram**::

    S = conv(foot_A, foot_B) + [reach_lo, reach_hi] . x_hat

That parallelogram is the whole of this robot's balance authority in one stance:
where the feet may land, and where the load may sit between them.
"""

from __future__ import annotations

import math

import numpy as np

DIAGONALS = {"A": ("LF", "RR"), "B": ("RF", "LR")}


def _hull(points: np.ndarray) -> np.ndarray:
    """Convex hull vertices, counter-clockwise. Small-n, so a monotone chain."""
    p = np.unique(np.round(np.asarray(points, dtype=float), 12), axis=0)
    if len(p) <= 2:
        return p
    p = p[np.lexsort((p[:, 1], p[:, 0]))]

    def half(pts):
        out = []
        for q in pts:
            while len(out) >= 2:
                a, b = out[-2], out[-1]
                if (b[0] - a[0]) * (q[1] - a[1]) - (b[1] - a[1]) * (q[0] - a[0]) <= 0:
                    out.pop()
                else:
                    break
            out.append(q)
        return out

    lower, upper = half(p), half(p[::-1])
    return np.array(lower[:-1] + upper[:-1])


def minkowski(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Minkowski sum of two convex polygons: the hull of pairwise vertex sums."""
    if len(a) == 0:
        return np.asarray(b, dtype=float)
    if len(b) == 0:
        return np.asarray(a, dtype=float)
    return _hull((np.asarray(a)[:, None, :] + np.asarray(b)[None, :, :]).reshape(-1, 2))


def equilibrium(controller, leg_q: dict) -> np.ndarray:
    """Ground projection of the nominal CoM — the origin everything is measured from.

    ⚠️ Not cosmetic. The recursion's fixed point is `xi* = u*`, so the origin must be
    the nominal centre of pressure or the viable set comes out one-sided. Computed
    first in absolute coordinates, the region touched the origin and reported zero
    authority in half of all directions.
    """
    com = controller.body.center_of_mass(np.zeros(3), leg_q).total.com
    return np.array([float(com[0]), 0.0])


def support_set(controller, leg_q: dict, pair, reach, origin=None) -> np.ndarray:
    """The parallelogram of CoP positions available during one diagonal stance.

    Vertices are the two feet, each offset by the fore-aft reach limits — the feet
    may be *placed* anywhere in that range, and the load may sit anywhere between
    them once placed. Expressed relative to `origin` (default: the nominal CoM).
    """
    body = controller.body
    o = equilibrium(controller, leg_q) if origin is None else np.asarray(origin, float)
    pts = []
    for nm in pair:
        f = body.foot_world_position(np.zeros(3), nm, leg_q[nm])
        base = np.array([float(f[0]), body.mounts[nm].track_y]) - o
        pts.append(base + np.array([reach[0], 0.0]))
        pts.append(base + np.array([reach[1], 0.0]))
    return _hull(np.array(pts))


def viable_set(controller, leg_q: dict, omega: float, stance: float, reach,
               steps: int = 8, spine: float = 0.0,
               spine_axis=(0.0, 1.0)) -> np.ndarray:
    """Disturbances (as a DCM offset, metres) recoverable within `steps` steps.

    Parameters
    ----------
    spine : float
        Lateral CoM authority to credit, metres. Added as a Minkowski segment along
        ``spine_axis`` — the spine shifts the CoM directly rather than moving the
        CoP, so it widens the recoverable set along its own axis and nowhere else.
        ⚠️ Set 0 to reproduce the feet-only case. ADR-0031 found this axis is not
        the binding one; with this function that becomes checkable rather than
        arguable.
    """
    g = math.exp(omega * stance)
    o = equilibrium(controller, leg_q)
    sets = {k: support_set(controller, leg_q, v, reach, o)
            for k, v in DIAGONALS.items()}
    order = ["A", "B"]

    region = np.zeros((1, 2))          # the target: recovered, at the origin
    for k in range(steps):
        s = sets[order[k % 2]]
        region = minkowski(region, (g - 1.0) * s) / g

    if spine > 0.0:
        ax = np.asarray(spine_axis, dtype=float)
        ax = ax / np.linalg.norm(ax)
        region = minkowski(region, np.array([-spine * ax, spine * ax]))
    return region


def reach_in_direction(region: np.ndarray, direction) -> float:
    """How far the region extends from the origin along `direction`.

    Not the support function: the question is "how large a disturbance in this
    direction is still inside", so this is the ray/polygon intersection.
    """
    d = np.asarray(direction, dtype=float)
    d = d / np.linalg.norm(d)
    poly = np.asarray(region, dtype=float)
    if len(poly) < 3:
        return 0.0
    best = math.inf
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        e = b - a
        den = d[0] * e[1] - d[1] * e[0]
        if abs(den) < 1e-15:
            continue
        t = (e[1] * a[0] - e[0] * a[1]) / den          # ray parameter
        s = (d[0] * a[1] - d[1] * a[0]) / (-den)       # edge parameter
        if t >= -1e-12 and -1e-9 <= s <= 1 + 1e-9:
            best = min(best, t)
    return 0.0 if not math.isfinite(best) else max(best, 0.0)


def contains(region: np.ndarray, point) -> bool:
    """Is `point` inside the convex region?"""
    p = np.asarray(point, dtype=float)
    poly = np.asarray(region, dtype=float)
    if len(poly) < 3:
        return bool(np.allclose(poly, p).all()) if len(poly) else False
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        if (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) < -1e-12:
            return False
    return True

def project(region: np.ndarray, point) -> np.ndarray:
    """Closest point of a convex polygon to `point`. Used as a control law.

    ⚠️ This is the whole optimal controller for the LIPM class. From
    `xi_next = g xi - (g-1) u`, driving `xi_next` to the origin wants
    `u* = g/(g-1) . xi`; when that is unreachable the best available choice is its
    projection onto the reachable set. No solver — a convex polygon projection is a
    scan over edges.

    M8-M27's controller projected onto a single AXIS (the next diagonal's normal)
    instead, which is where the 1-D collapse ADR-0031 identified actually lives.
    """
    p = np.asarray(point, dtype=float)
    poly = np.asarray(region, dtype=float)
    if len(poly) == 1:
        return poly[0].copy()
    if contains(poly, p):
        return p.copy()
    best, bd = poly[0], math.inf
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        e = b - a
        L = float(e @ e)
        t = 0.0 if L < 1e-18 else float(np.clip((p - a) @ e / L, 0.0, 1.0))
        q = a + t * e
        d = float((q - p) @ (q - p))
        if d < bd:
            best, bd = q, d
    return np.asarray(best, dtype=float)


def optimal_cop(region: np.ndarray, xi, growth: float) -> np.ndarray:
    """The best reachable centre of pressure for a DCM of `xi`, both relative to the
    nominal equilibrium. Deadbeat when reachable, closest-approach when not."""
    g = float(growth)
    target = (g / (g - 1.0)) * np.asarray(xi, dtype=float)
    return project(region, target)
