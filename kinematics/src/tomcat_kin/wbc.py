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

def realisable_cop(feet: np.ndarray, cop) -> np.ndarray:
    """Clamp a commanded centre of pressure onto what the contacts can actually make.

    ⚠️ With two point contacts the CoP is confined to the **segment between them** —
    a diagonal trot has no support polygon, only a support line. Commanding a free
    2-D point is asking for something no force allocation can deliver, and the
    regularised solve quietly returns the nearest thing instead of failing loudly.

    Clamping here makes the infeasibility visible to the caller: when the projection
    moves the command, the balance problem needs a **step**, not more force.
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
    # Three or more: fall back to the convex hull's nearest point.
    best, bd = pts[0], math.inf
    n = len(pts)
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        e = b - a
        L = float(e @ e)
        t = 0.0 if L < 1e-18 else float(np.clip((p - a) @ e / L, 0.0, 1.0))
        qy = a + t * e
        dd = float((qy - p) @ (qy - p))
        if dd < bd:
            best, bd = qy, dd
    return best

