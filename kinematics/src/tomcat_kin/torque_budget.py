"""Static torque budget for one leg.

Sweeps the reachable foot workspace for a given loading scenario and reports the
worst-case joint torque, tendon tension, and motor torque. This is the number
that feeds the actuator-selection decision (ADR-0003).

The load is modelled as a purely vertical support force at the foot (the leg
pushing down on the ground to hold the body up); extend `LoadCase` for lateral
or braking forces when needed.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .leg import LegModel, KneeConfig, UnreachableError
from .tendon import TendonMap
from .params import LoadCase


JOINT_NAMES = ("hip", "knee", "ankle")


@dataclass
class BudgetResult:
    load: LoadCase
    n_poses: int
    peak_joint_torque: np.ndarray   # N·m, per joint
    peak_tension: np.ndarray        # N, per joint
    peak_motor_torque: np.ndarray   # N·m, per joint

    def report(self) -> str:
        lines = [
            f"Load case: {self.load.name}",
            f"  foot support force : {self.load.foot_support_force_N:6.1f} N "
            f"({self.load.body_mass_kg} kg, {self.load.n_stance_legs} legs, "
            f"x{self.load.dynamic_factor})",
            f"  poses evaluated    : {self.n_poses}",
            f"  {'joint':<6}{'|tau| N·m':>12}{'tension N':>12}{'motor N·m':>12}",
        ]
        for i, name in enumerate(JOINT_NAMES):
            lines.append(
                f"  {name:<6}{self.peak_joint_torque[i]:>12.3f}"
                f"{self.peak_tension[i]:>12.1f}{self.peak_motor_torque[i]:>12.4f}"
            )
        return "\n".join(lines)


def evaluate(
    leg: LegModel,
    tendons: TendonMap,
    load: LoadCase,
    *,
    grid: int = 25,
    foot_pitch: float = 0.0,
    knee: KneeConfig = KneeConfig.FLEXED_POSITIVE,
) -> BudgetResult:
    """Sweep a grid of foot positions and return the worst-case budget.

    The workspace box is derived from the leg reach; poses that are unreachable
    or violate joint limits are skipped. `knee` defaults to the positive-flexion
    branch to match the (placeholder) knee limit sign in LegParams.
    """
    reach = leg.params.reach
    xs = np.linspace(-0.6 * reach, 0.6 * reach, grid)
    zs = np.linspace(-0.95 * reach, -0.30 * reach, grid)  # foot below the hip

    wrench = np.array([0.0, load.foot_support_force_N, 0.0])  # push up to support

    peak_tau = np.zeros(3)
    peak_tension = np.zeros(3)
    peak_motor = np.zeros(3)
    n = 0

    for x in xs:
        for z in zs:
            try:
                q = leg.inverse((x, z, foot_pitch), knee=knee)
            except UnreachableError:
                continue
            if not leg.in_limits(q):
                continue
            tau = leg.joint_torques_for_wrench(q, wrench)
            sol = tendons.resolve(tau)
            peak_tau = np.maximum(peak_tau, np.abs(tau))
            peak_tension = np.maximum(
                peak_tension,
                np.maximum(sol.tension_flexor, sol.tension_extensor),
            )
            peak_motor = np.maximum(peak_motor, np.abs(sol.motor_torque))
            n += 1

    if n == 0:
        raise ValueError(
            "no reachable, in-limit poses in the sampled workspace; "
            "check leg params / joint limits"
        )
    return BudgetResult(load, n, peak_tau, peak_tension, peak_motor)
