"""Moment-arm vs. cable-tension sensitivity tool.

For a FIXED worst-case joint torque, sweep the joint moment arm (pulley radius)
and report the resulting peak cable tension on both ends of the routing:

    JOINT-side  T_joint = T_bias + |tau| / r        (what the joint pulley feels)
    MOTOR-side  T_motor = T_joint * exp(mu * theta)  (capstan; what the motor/cable
                                                       supplies pulling against load)

This makes the ADR-0003 leg trade visible without hard-committing new moment-arm
values: small leg moment arms amplify a modest joint torque into very high cable
tension (~850-1050 N at the placeholder 10-15 mm arms), a larger pulley walks that
back toward the RoboCat ~20-70 N band, and Coulomb friction over the routing adds
further motor-side tension on top. The lead reconciles the actual mechanical spec.

Conventions match `tendon.py`: SI units, the motor-side (pulling-against-the-load)
direction is the worst case for motor/cable sizing, and `mu = 0` or `theta = 0`
makes the motor-side tension equal the joint-side tension.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

# RoboCat antagonistic tendon-tension sanity band (LITERATURE_REVIEW.md Q6).
ROBOCAT_BAND_N: tuple[float, float] = (20.0, 70.0)


@dataclass
class MomentArmSweepResult:
    """Peak cable tension vs. joint moment arm for one worst-case joint torque."""

    joint_torque: float          # N·m, the fixed |tau| driving the sweep
    t_bias: float                # N, co-contraction / pretension floor added on top
    friction_coeff: float        # mu, dimensionless
    wrap_angle: float            # theta_wrap, rad (total cable wrap)
    arms_m: np.ndarray           # m, the swept moment arms
    joint_tension_N: np.ndarray  # N, T_joint per arm
    motor_tension_N: np.ndarray  # N, T_motor per arm (capstan-amplified)
    band_N: tuple[float, float] = ROBOCAT_BAND_N

    @property
    def capstan_factor(self) -> float:
        """Motor/joint tension ratio exp(mu * theta_wrap) (1.0 when friction off)."""
        return float(np.exp(self.friction_coeff * self.wrap_angle))

    def arm_for_band_top(self, *, side: str = "joint") -> float | None:
        """Smallest swept arm whose tension falls at/below the band top.

        `side` selects which tension to test against the band: "joint" (default,
        what the joint feels) or "motor" (what the motor must supply). Returns
        None if no swept arm reaches the band on that side.
        """
        top = self.band_N[1]
        tension = self.joint_tension_N if side == "joint" else self.motor_tension_N
        ok = self.arms_m[tension <= top]
        return float(ok.min()) if ok.size else None

    def report(self) -> str:
        lines = [
            f"Moment-arm sensitivity  (|tau| = {self.joint_torque:.2f} N·m, "
            f"T_bias = {self.t_bias:.1f} N)",
            f"  capstan: mu = {self.friction_coeff:.2f}, "
            f"theta_wrap = {np.rad2deg(self.wrap_angle):.0f} deg "
            f"-> motor/joint factor = {self.capstan_factor:.3f}",
            f"  RoboCat sanity band: {self.band_N[0]:.0f}-{self.band_N[1]:.0f} N",
            f"  {'arm mm':>8}{'T_joint N':>12}{'T_motor N':>12}{'in band?':>10}",
        ]
        lo, hi = self.band_N
        for r, tj, tm in zip(self.arms_m, self.joint_tension_N, self.motor_tension_N):
            mark = "yes" if lo <= tj <= hi else ("<=hi" if tj <= hi else "high")
            lines.append(
                f"  {r * 1e3:>8.1f}{tj:>12.1f}{tm:>12.1f}{mark:>10}"
            )
        arm_j = self.arm_for_band_top(side="joint")
        arm_m = self.arm_for_band_top(side="motor")
        lines.append(
            "  smallest arm reaching band top (70 N): "
            + (f"joint-side {arm_j * 1e3:.1f} mm" if arm_j is not None
               else "joint-side none in sweep")
            + ", "
            + (f"motor-side {arm_m * 1e3:.1f} mm" if arm_m is not None
               else "motor-side none in sweep")
        )
        return "\n".join(lines)


def moment_arm_sweep(
    joint_torque: float,
    arms_m: Sequence[float] | np.ndarray,
    *,
    t_bias: float = 5.0,
    friction_coeff: float = 0.0,
    wrap_angle: float = 0.0,
    band_N: tuple[float, float] = ROBOCAT_BAND_N,
) -> MomentArmSweepResult:
    """Sweep the joint moment arm for a fixed worst-case joint torque.

    Uses the same antagonistic AIC relation as `TendonMap` (active tendon at
    `T_bias + |tau|/r`) for the joint-side tension, then applies the capstan
    factor `exp(mu * theta_wrap)` for the motor-side tension. Purely analytic, so
    it does not depend on a particular `TendonMap` instance and does not commit
    any moment-arm value to `LegParams` — it just exposes the trade.
    """
    arms = np.asarray(arms_m, dtype=float)
    if arms.size == 0 or np.any(arms <= 0.0):
        raise ValueError("arms_m must be a non-empty sequence of positive radii (m)")
    if friction_coeff < 0.0 or wrap_angle < 0.0:
        raise ValueError("friction_coeff and wrap_angle must be >= 0")

    t_joint = t_bias + np.abs(float(joint_torque)) / arms
    factor = float(np.exp(friction_coeff * wrap_angle))
    t_motor = t_joint * factor

    return MomentArmSweepResult(
        joint_torque=abs(float(joint_torque)),
        t_bias=float(t_bias),
        friction_coeff=float(friction_coeff),
        wrap_angle=float(wrap_angle),
        arms_m=arms,
        joint_tension_N=t_joint,
        motor_tension_N=t_motor,
        band_N=band_N,
    )
