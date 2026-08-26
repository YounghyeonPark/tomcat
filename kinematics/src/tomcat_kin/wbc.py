# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Whole-body contact-force allocation — the thing position control cannot do (M33).

ADR-0037 ended four attempts to add degrees of freedom to a per-step controller.
Each failed the same way, and the reason is structural rather than a tuning miss:

    **Position servos do not command force.** You command where the foot should be
    and the ground reaction is whatever the contact and the leg compliance happen to
    produce. Every "allocate the load between the two feet" scheme in M21–M32 was
    therefore commanding a *proxy* — differential leg extension — and hoping the
    force followed. It did, statically (−39.3 mm of CoP per mm, ADR-0032). In the
    loop it fought the placement it was supposed to help.

This module commands **torque**, so the contact force is a decision variable rather
than a consequence:

1. The DCM law asks for a centre of pressure `p`.
2. `allocate` finds foot forces that produce the required net wrench with the CoP at
   `p`, inside the friction cones — a 6-variable least-squares with a cone
   projection, not a solver call, because it runs every timestep.
3. `stance_torque` maps them back with `tau = -J^T f`.

⚠️ **What this does NOT change: the bound.** ADR-0033's viable set is 62.7 mm with
the spine and 29.8 mm without, and no controller beats it. This is an attempt to
*reach* it, and its own honest test is whether it beats the 25.6 mm the shipped
position controller already achieves.
"""

from __future__ import annotations

import math

import numpy as np


def cone_project(f: np.ndarray, mu: float, min_normal: float = 0.0) -> np.ndarray:
    """Nearest force inside the friction cone `|f_t| <= mu f_n`, with `f_n >= 0`.

    Closed form, because this runs at every timestep. Three cases: inside the cone
    (keep), behind the polar cone (zero), or outside (slide onto the surface).
    """
    ft = f[:2]
    fn = float(f[2])
    t = float(np.linalg.norm(ft))
    if fn < 0.0 and t <= -mu * fn:          # inside the polar cone: no contact
        return np.zeros(3)
    if t <= mu * fn:                        # already admissible
        out = f.copy()
        out[2] = max(fn, min_normal)
        return out
    # Outside: project onto the cone surface.
    scale = (t * mu + fn) / (mu * mu + 1.0)
    out = np.empty(3)
    out[2] = max(scale, min_normal)
    out[:2] = ft * (mu * out[2] / t) if t > 1e-12 else 0.0
    return out


def allocate(feet: np.ndarray, com: np.ndarray, wrench: np.ndarray, mu: float,
             reg: float = 1e-4) -> np.ndarray:
    """Foot forces producing `wrench` about the CoM, inside the friction cones.

    Parameters
    ----------
    feet : (n, 3)
        Contact points, world.
    com : (3,)
        Centre of mass, world — the point moments are taken about.
    wrench : (6,)
        Desired ``[force(3), moment(3)]``. The moment is what places the centre of
        pressure: asking for zero moment about the CoM puts the CoP under it.

    Notes
    -----
    ⚠️ Regularised least squares plus a cone projection, **not** a QP solve. With two
    point contacts the system is 6x6 and often near-singular — a diagonal trot cannot
    produce a moment about the line joining its feet at all — so the pseudo-inverse
    with `reg` is doing real work, not decoration. The projection afterwards is what
    keeps the answer physical; it is a heuristic, and where it binds the allocation is
    no longer optimal.
    """
    n = len(feet)
    a = np.zeros((6, 3 * n))
    for i, p in enumerate(feet):
        a[0:3, 3 * i:3 * i + 3] = np.eye(3)
        r = np.asarray(p, dtype=float) - np.asarray(com, dtype=float)
        a[3:6, 3 * i:3 * i + 3] = np.array([
            [0.0, -r[2], r[1]],
            [r[2], 0.0, -r[0]],
            [-r[1], r[0], 0.0],
        ])
    # (A^T A + reg I)^-1 A^T w  — stable through the singular directions.
    ata = a.T @ a + reg * np.eye(3 * n)
    f = np.linalg.solve(ata, a.T @ np.asarray(wrench, dtype=float))
    return np.array([cone_project(f[3 * i:3 * i + 3], mu) for i in range(n)])


def desired_wrench(mass: float, com: np.ndarray, com_vel: np.ndarray,
                   cop: np.ndarray, omega: float, gravity: float = 9.81,
                   damp: float = 0.0, height: float | None = None,
                   kp_z: float = 400.0, kd_z: float = 40.0) -> np.ndarray:
    """Net wrench for a LIPM tracking a commanded centre of pressure.

    `a = omega^2 (c - p)` is the LIPM's own statement. `damp` adds a velocity term,
    which the pure LIPM does not have and a real robot wants.

    ⚠️ `height` is not optional in practice. Commanding exactly `m g` vertically
    balances the weight and regulates nothing: the first run drifted **0.165 -> 0.185 m
    in 0.6 s** because nothing closed the loop on height. LIPM assumes constant CoM
    height; a torque controller has to *make* that true rather than inherit it.
    """
    c = np.asarray(com, dtype=float)
    v = np.asarray(com_vel, dtype=float)
    p3 = np.array([cop[0], cop[1], 0.0])
    acc = (omega ** 2) * (c - p3) - damp * v
    acc[2] = 0.0 if height is None else kp_z * (height - c[2]) - kd_z * v[2]
    force = mass * (acc + np.array([0.0, 0.0, gravity]))
    return np.concatenate([force, np.zeros(3)])


def stance_torque(mujoco, model, data, site_ids, forces, dof_of) -> dict:
    """Map foot forces to joint torques, `tau = -J^T f`, per leg.

    The sign is the one that trips people: `J^T f` is the torque the *ground* applies
    through the leg, so the joint must supply its negative to hold it.
    """
    out = {}
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    for nm, f in forces.items():
        mujoco.mj_jacSite(model, data, jacp, jacr, site_ids[nm])
        tau = -(jacp.T @ np.asarray(f, dtype=float))
        out[nm] = np.array([tau[d] for d in dof_of[nm]])
    return out

def nnls(a, b, max_iter: int | None = None, tol: float = 1e-12) -> np.ndarray:
    """Lawson-Hanson non-negative least squares: min ||a x - b|| subject to x >= 0.

    Written out rather than imported because the project has no scipy dependency,
    and because a tendon controller needs this in firmware where there will be no
    scipy either. Finite: the active set strictly decreases the residual, so it
    terminates. `max_iter` is a belt-and-braces cap.

    ⚠️ **Clipping an unconstrained solve is not this**, and the difference is not
    cosmetic. [ADR-0047](../../../docs/DESIGN_DECISIONS.md) measured it as about a
    degree of joint error at a co-contraction floor; ADR-0049 measured it at
    standing loads as a **1.14 N.m residual and a 20.7 deg lean**. The constraint
    has to be in the solve.
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    n = a.shape[1]
    x = np.zeros(n)
    passive = np.zeros(n, dtype=bool)
    w = a.T @ (b - a @ x)
    it = 0
    cap = 3 * n if max_iter is None else max_iter
    while (not passive.all()) and np.any(w[~passive] > tol):
        j = int(np.argmax(np.where(passive, -np.inf, w)))
        passive[j] = True
        while True:
            it += 1
            if it > cap:
                return np.maximum(x, 0.0)
            idx = np.flatnonzero(passive)
            sol = np.zeros(n)
            sol[idx] = np.linalg.lstsq(a[:, idx], b, rcond=None)[0]
            if np.all(sol[idx] > tol):
                x = sol
                break
            neg = idx[sol[idx] <= tol]
            alpha = float(np.min(x[neg] / (x[neg] - sol[neg])))
            x = x + alpha * (sol - x)
            passive &= x > tol
            if not passive.any():
                return np.maximum(x, 0.0)
        w = a.T @ (b - a @ x)
    return x


def tendon_tension(gain: np.ndarray, tau, t_min: float = 0.0,
                   t_max: float = np.inf) -> np.ndarray:
    """Pull-only tendon tensions for a desired joint torque.

    `gain[k, i]` is the torque on joint `k` per newton on tendon `i`, i.e.
    `-J_tendon^T`. Solves `min ||gain T - tau||` subject to `t_min <= T <= t_max`,
    by substituting `T = t_min + u` with `u >= 0` and running `nnls`.

    `t_min` is the co-contraction floor: a cable must stay taut to be a cable, and
    ADR-0049 found the floor also keeps the solution interior so the upper clamp is
    rarely reached.

    ⚠️ **A zero residual is not guaranteed and its absence is a design finding, not
    a solver failure.** A joint with one tendon can only be driven one way, so a
    torque of the wrong sign is unreachable at any tension -- which is exactly how
    ADR-0049 found the hind ankle unable to hold a stance. Check the residual.
    """
    gain = np.asarray(gain, dtype=float)
    n = gain.shape[1]
    floor = np.full(n, float(t_min))
    u = nnls(gain, np.asarray(tau, dtype=float) - gain @ floor)
    return np.clip(floor + u, t_min, t_max)


def actuator_torque(data, dof, stance_tau) -> np.ndarray:
    """What the ACTUATORS must supply, from MuJoCo's own equation of motion.

    `M qdd + qfrc_bias = qfrc_passive + qfrc_actuator + J_c^T f_c`, so holding a
    pose needs

        qfrc_actuator = qfrc_bias - qfrc_passive + stance_torque

    with `stance_torque = -J_c^T f_c` as `stance_torque()` returns it.

    ⚠️ **`qfrc_passive` is the term that gets forgotten, and it is not small.** Left
    out, ADR-0049's driver asked the hind ankle tendon for 0.508 N.m that the
    ADR-0002 Option-B return spring was already supplying -- 54 % of that joint's
    whole demand.
    """
    idx = list(dof)
    bias = np.array([data.qfrc_bias[k] for k in idx])
    passive = np.array([data.qfrc_passive[k] for k in idx])
    return bias - passive + np.asarray(stance_tau, dtype=float)


def realisable_cop(feet: np.ndarray, cop) -> np.ndarray:
    """Clamp a commanded centre of pressure onto what the contacts can actually make.

    ⚠️ With two point contacts the CoP is confined to the **segment between them** —
    a diagonal trot has no support polygon, only a support line. Commanding a free
    2-D point is asking for something no force allocation can deliver, and the
    regularised solve quietly returns the nearest thing instead of failing loudly.

    Clamping here makes the infeasibility visible to the caller: when the projection
    moves the command, the balance problem needs a **step**, not more force.

    ⚠️ **The three-or-more-contact branch was wrong until M44, in two ways, because
    M33 only ever ran a diagonal two-foot trot and never exercised it.**

    1. **There was no inside test.** It walked the boundary and returned the nearest
       point *on an edge*, so a perfectly feasible CoP in the middle of a four-foot
       support polygon was pushed **48 mm out to the rail**. For a standing robot
       that is not a clamp, it is a command to lean.
    2. **It assumed the caller's point order was hull order.** It is not:
       `("LF", "RF", "LR", "RR")` traverses a rectangle as a **bowtie**, so two of
       the four "edges" it measured against were diagonals. That bug partly masked
       the first one -- it moved the interior point 16.6 mm instead of 48.

    Both are fixed by taking a real convex hull and testing containment first.
    """
    p = np.asarray(cop, dtype=float)[:2]
    pts = np.asarray(feet, dtype=float)[:, :2]
    if len(pts) == 1:
        return pts[0].copy()
    if len(pts) == 2:
        a, b = pts
        e = b - a
        L = float(e @ e)
        if L < 1e-18:
            return a.copy()
        t = float(np.clip((p - a) @ e / L, 0.0, 1.0))
        return a + t * e
    # Three or more contacts: there is a real support POLYGON, so a point inside it
    # is feasible and must be returned untouched.
    hull = _convex_hull(pts)
    if len(hull) < 3:                       # collinear feet: still only a segment
        a, b = hull[0], hull[-1]
        e = b - a
        L = float(e @ e)
        if L < 1e-18:
            return a.copy()
        t = float(np.clip((p - a) @ e / L, 0.0, 1.0))
        return a + t * e
    if _inside_hull(hull, p):
        return p.copy()
    best, bd = hull[0], math.inf
    n = len(hull)
    for i in range(n):
        a, b = hull[i], hull[(i + 1) % n]
        e = b - a
        L = float(e @ e)
        t = 0.0 if L < 1e-18 else float(np.clip((p - a) @ e / L, 0.0, 1.0))
        qy = a + t * e
        dd = float((qy - p) @ (qy - p))
        if dd < bd:
            best, bd = qy, dd
    return best


def _convex_hull(pts: np.ndarray) -> np.ndarray:
    """Counter-clockwise convex hull, monotone chain. Small n, so simplicity wins.

    ⚠️ The caller's point order is NOT hull order and must not be assumed to be.
    `("LF", "RF", "LR", "RR")` walks a rectangle as a **bowtie**, whose "edges"
    include the two diagonals -- see `realisable_cop`.
    """
    q = sorted({(float(x), float(y)) for x, y in pts})
    if len(q) < 3:
        return np.array(q, dtype=float).reshape(-1, 2)

    def half(seq):
        out = []
        for r in seq:
            while len(out) >= 2:
                (ax, ay), (bx, by) = out[-2], out[-1]
                if (bx - ax) * (r[1] - ay) - (by - ay) * (r[0] - ax) > 1e-15:
                    break
                out.pop()
            out.append(r)
        return out

    lower = half(q)
    upper = half(list(reversed(q)))
    return np.array(lower[:-1] + upper[:-1], dtype=float)


def _inside_hull(hull: np.ndarray, p: np.ndarray, tol: float = 1e-12) -> bool:
    """True if `p` is inside or on a counter-clockwise convex hull."""
    n = len(hull)
    for i in range(n):
        a, b = hull[i], hull[(i + 1) % n]
        if (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) < -tol:
            return False
    return True

