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

A2. **Massless, rigid links** (inherited from the leg/spine kinematics). The
    trunk's own weight is instead lumped as equal point masses at the N+1
    vertebrae: `body_mass * g / (N+1)` downward at each. Equal split is a
    placeholder — real cats are front-heavy (~60% fore). This lumping is what
    produces the "arched-posture gravity moment": in the dorsiflexed geometry
    the forward vertebrae swing up/forward, lengthening their gravity moment
    arms about the rear joints.

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

Joint-torque formula (2D, forces purely vertical Fz at vertebra positions x_k):

    tau_i = sum_{k = i+1 .. N} (x_k - x_i) * Fz_k          (i = 0 .. N-1)

evaluated in the ACTUAL bent geometry from spine FK, so posture changes the
moment arms. The antagonistic tendon map then turns |tau_i| into tensions.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .leg import KneeConfig
from .spine import WholeBody, Girdle
from .tendon import TendonMap
from .params import WholeBodyLoadCase, GRAVITY
from . import torque_budget
from .torque_budget import BudgetResult, JOINT_NAMES


def spine_joint_torques(body: WholeBody, load: WholeBodyLoadCase) -> np.ndarray:
    """Static torque (N·m) each spine joint must resist for a whole-body case.

    Implements the cantilever model documented at the top of this module
    (assumptions A1-A4). Returns a length-`n_segments` array, joint 0 = the
    rearmost (base) joint.
    """
    spine = body.spine
    n = spine.params.n_segments
    spine_q = np.asarray(load.spine_q, dtype=float)
    if spine_q.shape != (n,):
        raise ValueError(
            f"load.spine_q has shape {spine_q.shape}; expected ({n},)"
        )

    xs = spine.vertebra_positions(spine_q)[:, 0]  # (N+1,) x of each vertebra

    # A2: lumped trunk weight, equal split, static (no dynamic factor).
    w_grav = load.body_mass_kg * GRAVITY / (n + 1)
    fz = np.full(n + 1, -w_grav)

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
    fz[-1] += front_count * support   # front girdle = vertebra N
    fz[0] += rear_count * support     # rear girdle = vertebra 0 (base)

    # A1: cantilever from the rear base. Joint i sums loads outboard (k > i).
    tau = np.zeros(n)
    for i in range(n):
        moment = 0.0
        for k in range(i + 1, n + 1):
            moment += (xs[k] - xs[i]) * fz[k]
        tau[i] = moment
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
    knee: KneeConfig = KneeConfig.FLEXED_POSITIVE,
    t_bias=None,
) -> WholeBodyBudgetResult:
    """Evaluate the combined spine+legs static budget for one whole-body case.

    `t_bias` is the antagonistic co-contraction bias (ADR-0002), passed to BOTH
    the leg and spine tendon maps; `None` uses each map's pretension floor.
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
    tau_spine = spine_joint_torques(body, load)
    ssol = spine_tendons.resolve(tau_spine, t_bias=t_bias)
    spine_tension = np.maximum(ssol.tension_flexor, ssol.tension_extensor)

    # Installed antagonistic motors: ~2 per DOF (ADR-0002).
    n_legs = len(body.mounts)
    n_leg_joints = len(JOINT_NAMES)
    n_spine_joints = body.spine.params.n_segments
    n_leg_motors = n_legs * n_leg_joints * 2
    n_spine_motors = n_spine_joints * 2

    return WholeBodyBudgetResult(
        load=load,
        leg=leg_result,
        spine_joint_torque=tau_spine,
        spine_tension=spine_tension,
        spine_motor_torque=np.abs(ssol.motor_torque),
        n_leg_motors=n_leg_motors,
        n_spine_motors=n_spine_motors,
    )
