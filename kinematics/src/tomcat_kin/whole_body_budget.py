# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Combined whole-body static tendon/torque budget (spine + four legs).

This closes M1 Definition-of-Done item 5: sweep whole-body load cases (stand,
arch/extend, single-leg land) over the `WholeBody` model and report, across the
legs AND the spine:

- per-tendon peak tension (N),
- per-motor peak torque (N·m),
- TOTAL motor count, applying the ADR-0002 antagonistic factor (~2 motors per
  antagonistic DOF).

The per-leg budget is reused verbatim from `torque_budget.evaluate` (one
worst-case workspace sweep per stance leg, all legs sharing geometry).

Static spine-load model (assumptions stated explicitly)
-------------------------------------------------------
The spine joint torques come from a deliberately SIMPLE, honest static model,
appropriate to the placeholder params (M1 proves the machinery, not the final
geometry). The assumptions are:

A1. **Cantilever fixed at the rear (pelvic) girdle.** Vertebra 0 (the rear
    girdle) is taken as the grounded base — the rear legs plant it. Each spine
    joint `i` then resists the moment of every load applied OUTBOARD of it (at
    vertebrae i+1 .. N, toward the front / shoulder girdle). A consequence: a
    rear-girdle leg reaction sits at the base and loads NO spine joint. This
    upper-bounds the front-joint loads for a given front load and avoids
    modelling the statically-indeterminate both-girdles-supported closed chain
    (deferred to a later dynamic model).

A2. **REAL DISTRIBUTED MASS (rewritten in M4).** Links are rigid but no longer
    massless. Gravity is applied from the mass budget in `params.py`:

      - each spine SEGMENT's mass acts at its own CoM, computed in the ACTUAL
        bent geometry (`WholeBody.spine_com` / `mass.spine_segment_coms`), so
        arching the back really does move the moment arms;
      - each GIRDLE lump acts at its girdle vertebra (the front girdle also
        carries the head/neck mass — a big part of why a cat is front-heavy);
      - each LEG's mass is charged to the girdle it hangs from. By default it is
        lumped at that girdle vertebra; pass `leg_q` to place it at the leg's
        real posed CoM instead.

    The distribution is scaled so its total equals the load case's
    `body_mass_kg` (the *shape* comes from params, the *total* from the case),
    which keeps the budget monotonic in body mass.

    WHAT CHANGED vs. the old placeholder. M1-M3 lumped EQUAL point masses at the
    N+1 vertebrae (`body_mass * g / (N+1)` each) and admitted in this very
    docstring that real cats are ~60% front-heavy. The new distribution is
    ~60/40 fore/hind (`WholeBody.mass_budget`), which loads the REAR joints much
    harder and the front joints less: for the default 3 kg quiet-stand case the
    base (rearmost) spine joint goes from |tau| ~0.15 N·m to ~0.57 N·m (~3.9x),
    while the two forward joints drop from ~0.40 N·m to ~0.08 / ~0.11 N·m — and
    the mid joint changes SIGN. Peak spine tension for that case rises from
    ~18.5 N to ~24.0 N (into, not out of, the ~20-70 N RoboCat band). The arched
    case moves the same way (base 0.32 -> 0.55 N·m, peak tension 15.5 -> 23.4 N).
    Impact-dominated cases barely move: the single-leg land, where a 2.5x
    reaction dwarfs body weight, goes 11.33 -> 10.91 N·m at the base (peak
    tension 383 -> 369 N, still far outside the RoboCat band — that case is
    driven by the moment arm / dynamic factor, not by the mass model).

A2b. **Still quasi-static.** Mass enters ONLY as weight (m*g) and CoM geometry.
    No inertia tensors, velocities or accelerations: full Newton-Euler is a
    later milestone. The literature's Mass-Mass-Spring finding (leg mass in
    flight bending a compliant trunk) is therefore still out of reach here.

A3. **Girdle reactions are vertical point loads at the girdle vertebra.** Each
    girdle receives `n_stance_on_girdle * foot_support_force_N` upward (the legs
    push down on the ground; the ground pushes back up through the legs into the
    girdle). `foot_support_force_N` carries the load case's `dynamic_factor`
    (impact), whereas gravity (A2) is the static body weight — so an unbalanced
    impact (e.g. single-leg land) shows up as a large unopposed front reaction.

A4. **Sagittal plane only, quasi-static, frictionless cables.** No inertial /
    velocity terms; left/right legs on a girdle project onto one sagittal mount.
    The literature (leg mass bends a compliant trunk) means this static model
    will not capture spine<->leg elastic energy exchange — that is out of M1
    scope and flagged for the dynamics milestone.

Joint-torque formula (2D, all forces purely vertical)

Every load is a point force Fz applied at some x and ATTACHED to a vertebra
index a (the vertebra it hangs from / the outboard end of the segment it belongs
to). Joint i resists everything attached OUTBOARD of it:

    tau_i = sum_{loads with a > i} (x_load - x_i) * Fz_load     (i = 0 .. N-1)

Note x_load is the load's OWN x (a segment CoM, or a posed leg CoM), which is in
general NOT the attachment vertebra's x. Everything is evaluated in the ACTUAL
bent geometry from spine FK, so posture changes the moment arms. The
antagonistic tendon map then turns |tau_i| into tensions.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .leg import KneeConfig
from .spine import WholeBody, Girdle
from .mass import spine_segment_coms
from .tendon import TendonMap
from .params import WholeBodyLoadCase, GRAVITY
from . import torque_budget
from .torque_budget import BudgetResult, JOINT_NAMES


def gravity_loads(
    body: WholeBody, load: WholeBodyLoadCase, leg_q=None
) -> list[tuple[int, float, float]]:
    """The distributed weight of the body as `(attach_vertebra, x, Fz)` loads.

    Implements assumption A2 (REAL distributed mass). Each entry is a downward
    (negative Fz, newtons) point force at position `x` (m, body-ground frame),
    attached to vertebra `attach_vertebra` so the cantilever sum knows which
    joints it loads.

    The mass DISTRIBUTION comes from `params.py` (spine segment masses, girdle
    masses, per-leg link masses); the TOTAL is rescaled to the load case's
    `body_mass_kg` so a heavier case scales every load proportionally. Gravity
    carries NO `dynamic_factor` — that multiplier belongs to the impact reaction
    (A3), not to the static body weight.

    `leg_q` optionally maps leg name -> that leg's `(q1, q2, q3)`, which places
    each leg's weight at its REAL posed CoM. With `leg_q=None` a leg's mass is
    lumped at its girdle vertebra (simpler, and conservative for the front
    joints only to the extent the leg CoM sits near its hip).
    """
    spine = body.spine
    n = spine.params.n_segments
    p = spine.params
    spine_q = np.asarray(load.spine_q, dtype=float)
    xs = spine.vertebra_positions(spine_q)[:, 0]

    scale = load.body_mass_kg / body.total_mass
    loads: list[tuple[int, float, float]] = []

    # Spine segments: mass at the segment CoM (bent geometry), attached to the
    # segment's OUTBOARD vertebra i+1 so it loads joints 0..i.
    seg_pts = spine_segment_coms(
        spine.vertebra_positions(spine_q), p.segment_com_frac
    )
    for i in range(n):
        loads.append(
            (i + 1, float(seg_pts[i, 0]), -p.segment_mass[i] * scale * GRAVITY)
        )

    # Girdle lumps at their own vertebra (rear = 0 = the grounded base, so it
    # loads nothing; front = N, so it loads every joint).
    for girdle, idx in ((Girdle.REAR, 0), (Girdle.FRONT, n)):
        gc = body.girdle_com(spine_q, girdle)
        loads.append((idx, gc.x, -gc.mass * scale * GRAVITY))

    # Legs, charged to the girdle they hang from.
    for name, mount in body.mounts.items():
        idx = n if mount.girdle is Girdle.FRONT else 0
        if leg_q is not None:
            lc = body.leg_com_world(spine_q, name, leg_q[name])
            x, m = lc.x, lc.mass
        else:
            x, m = xs[idx], body.legs[name].params.mass
        loads.append((idx, float(x), -m * scale * GRAVITY))

    return loads


def spine_joint_torques(
    body: WholeBody, load: WholeBodyLoadCase, leg_q=None
) -> np.ndarray:
    """Static torque (N·m) each spine joint must resist for a whole-body case.

    Implements the cantilever model documented at the top of this module
    (assumptions A1-A4), with REAL distributed mass (A2, rewritten in M4).
    Returns a length-`n_segments` array, joint 0 = the rearmost (base) joint.

    `leg_q` is passed through to `gravity_loads` (see there): give it a per-leg
    joint-angle map to place the leg weights at their posed CoMs instead of
    lumping them at their girdles.
    """
    spine = body.spine
    n = spine.params.n_segments
    spine_q = np.asarray(load.spine_q, dtype=float)
    if spine_q.shape != (n,):
        raise ValueError(
            f"load.spine_q has shape {spine_q.shape}; expected ({n},)"
        )

    xs = spine.vertebra_positions(spine_q)[:, 0]  # (N+1,) x of each vertebra

    # A2: real distributed body weight (static; no dynamic factor).
    loads = gravity_loads(body, load, leg_q=leg_q)

    # A3: girdle reactions (upward), carrying the impact dynamic_factor.
    support = load.foot_support_force_N
    stance = set(load.stance_legs)
    front_count = sum(
        1 for name, m in body.mounts.items()
        if m.girdle is Girdle.FRONT and name in stance
    )
    rear_count = sum(
        1 for name, m in body.mounts.items()
        if m.girdle is Girdle.REAR and name in stance
    )
    loads.append((n, float(xs[n]), front_count * support))   # front girdle
    loads.append((0, float(xs[0]), rear_count * support))    # rear girdle (base)

    # A1: cantilever from the rear base. Joint i sums every load attached
    # OUTBOARD of it (attachment vertebra index > i).
    tau = np.zeros(n)
    for i in range(n):
        tau[i] = sum(
            (x - xs[i]) * fz for attach, x, fz in loads if attach > i
        )
    return tau


@dataclass
class WholeBodyBudgetResult:
    load: WholeBodyLoadCase

    # Per-leg worst-case sweep (shared leg geometry).
    leg: BudgetResult

    # Spine joint quantities, per spine joint (rear -> front).
    spine_joint_torque: np.ndarray   # N·m
    spine_tension: np.ndarray        # N (peak of the two antagonistic tendons)
    spine_motor_torque: np.ndarray   # N·m

    # Installed antagonistic motor counts (~2 per DOF, ADR-0002).
    n_leg_motors: int
    n_spine_motors: int

    # M4 mass bookkeeping: the fore/hind weight split actually used for gravity.
    mass_total_kg: float = 0.0
    mass_fore_fraction: float = 0.0

    @property
    def n_total_motors(self) -> int:
        return self.n_leg_motors + self.n_spine_motors

    @property
    def peak_leg_tension(self) -> float:
        return float(np.max(self.leg.peak_tension))

    @property
    def peak_spine_tension(self) -> float:
        return float(np.max(self.spine_tension))

    def report(self) -> str:
        lines = [
            f"Whole-body load case: {self.load.name}",
            f"  body mass {self.load.body_mass_kg} kg, x{self.load.dynamic_factor}, "
            f"stance legs {self.load.stance_legs}",
            f"  spine posture (deg): "
            f"{np.round(np.rad2deg(self.load.spine_q), 1)}",
            "  -- legs (worst case over workspace, HIND leg as representative) --",
            f"    per-leg support force: {self.load.foot_support_force_N:6.1f} N",
            f"    {'joint':<6}{'|tau| N·m':>12}{'tension N':>12}{'motor N·m':>12}",
        ]
        for i, name in enumerate(JOINT_NAMES):
            lines.append(
                f"    {name:<6}{self.leg.peak_joint_torque[i]:>12.3f}"
                f"{self.leg.peak_tension[i]:>12.1f}"
                f"{self.leg.peak_motor_torque[i]:>12.4f}"
            )
        lines.append("  -- spine (cantilever from rear girdle) --")
        lines.append(
            f"    mass model: REAL distributed, {self.mass_total_kg:.2f} kg "
            f"({self.mass_fore_fraction * 100:.1f}% fore / "
            f"{(1.0 - self.mass_fore_fraction) * 100:.1f}% hind)"
        )
        lines.append(
            f"    {'joint':<6}{'|tau| N·m':>12}{'tension N':>12}{'motor N·m':>12}"
        )
        for i in range(len(self.spine_joint_torque)):
            lines.append(
                f"    seg{i:<3}{abs(self.spine_joint_torque[i]):>12.3f}"
                f"{self.spine_tension[i]:>12.1f}"
                f"{self.spine_motor_torque[i]:>12.4f}"
            )
        lines.append(
            f"  motors: legs {self.n_leg_motors} + spine {self.n_spine_motors} "
            f"= {self.n_total_motors} total (antagonistic, ~2/DOF)"
        )
        lines.append(
            f"  peak tension: leg {self.peak_leg_tension:.1f} N, "
            f"spine {self.peak_spine_tension:.1f} N "
            f"(RoboCat sanity band ~20-70 N)"
        )
        return "\n".join(lines)


def evaluate(
    body: WholeBody,
    leg_tendons: TendonMap,
    spine_tendons: TendonMap,
    load: WholeBodyLoadCase,
    *,
    grid: int = 25,
    knee: KneeConfig | None = None,   # None -> each leg's own anatomical fold
    t_bias=None,
    leg_q=None,
) -> WholeBodyBudgetResult:
    """Evaluate the combined spine+legs static budget for one whole-body case.

    `t_bias` is the antagonistic co-contraction bias (ADR-0002), passed to BOTH
    the leg and spine tendon maps; `None` uses each map's pretension floor.

    `leg_q` (M4) optionally maps leg name -> that leg's joint angles so the leg
    weights load the spine at their REAL posed CoMs; `None` lumps each leg at its
    girdle vertebra.
    """
    # Legs: reuse the per-leg worst-case sweep with the case's per-leg load.
    # The body is now fore/hind ASYMMETRIC (fore vs. hind link proportions differ;
    # see WholeBody / params.py), but this budget still runs the sweep on ONE
    # representative leg. We deliberately use the HIND leg (DEFAULT_LEG =
    # DEFAULT_HINDLEG, the folded propulsion limb) as the worst case: it is the
    # longer-shank limb that props the body, so it bounds the fore leg here. A
    # per-leg-type split of the leg budget is deferred (noted for a later pass).
    a_leg = body.hind_leg
    leg_result = torque_budget.evaluate(
        a_leg, leg_tendons, load.leg_load, grid=grid, knee=knee, t_bias=t_bias
    )

    # Spine: static cantilever torques -> antagonistic tensions.
    tau_spine = spine_joint_torques(body, load, leg_q=leg_q)
    ssol = spine_tendons.resolve(tau_spine, t_bias=t_bias)
    spine_tension = np.maximum(ssol.tension_flexor, ssol.tension_extensor)

    # Installed antagonistic motors: ~2 per DOF (ADR-0002).
    n_legs = len(body.mounts)
    n_leg_joints = len(JOINT_NAMES)
    n_spine_joints = body.spine.params.n_segments
    n_leg_motors = n_legs * n_leg_joints * 2
    n_spine_motors = n_spine_joints * 2

    budget = body.mass_budget()
    return WholeBodyBudgetResult(
        load=load,
        leg=leg_result,
        spine_joint_torque=tau_spine,
        spine_tension=spine_tension,
        spine_motor_torque=np.abs(ssol.motor_torque),
        n_leg_motors=n_leg_motors,
        n_spine_motors=n_spine_motors,
        mass_total_kg=float(load.body_mass_kg),
        mass_fore_fraction=float(budget.fore_fraction),
    )
