"""Placeholder parameters for the TomCat single-leg model.

IMPORTANT: every numeric value here is a PLACEHOLDER (❓ TBD in
docs/REQUIREMENTS.md). They exist so the model runs end-to-end; they are not
committed design values. Swap them once mechanical design lands.

Conventions
-----------
- SI units throughout: metres, radians, kilograms, newtons, newton-metres.
- Sagittal-plane (2D) model: x forward, z up, hip at the origin.
- A planar 3R chain: hip -> thigh -> knee -> shank -> ankle -> foot.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

GRAVITY = 9.81  # m/s^2


@dataclass(frozen=True)
class LegParams:
    """Geometry and mass of one leg (sagittal-plane 3R chain)."""

    # Link lengths (m): thigh, shank, foot.  ❓ TBD
    l1: float = 0.120
    l2: float = 0.120
    l3: float = 0.050

    # Joint angle limits (rad), (min, max) per joint: hip, knee, ankle.  ❓ TBD
    q_min: tuple[float, float, float] = (-math.pi / 2, 0.0, -math.pi / 3)
    q_max: tuple[float, float, float] = (math.pi / 2, math.pi * 0.8, math.pi / 3)

    @property
    def reach(self) -> float:
        """Maximum straight-leg distance from hip to foot tip."""
        return self.l1 + self.l2 + self.l3


@dataclass(frozen=True)
class TendonParams:
    """Tendon routing and actuator parameters, per joint (hip, knee, ankle)."""

    # Joint pulley radii / moment arms (m) — converts tension to joint torque.  ❓ TBD
    joint_moment_arm: tuple[float, float, float] = (0.015, 0.012, 0.010)

    # Motor spool radius (m) — converts motor torque to cable tension.  ❓ TBD
    motor_spool_radius: float = 0.008

    # Minimum tension kept in every cable so it never goes slack (N).  ❓ TBD
    # In antagonistic mode this is the co-contraction floor on the "slack" side.
    pretension: float = 5.0

    # Spring-return mode only: torsional spring stiffness (N·m/rad) and rest
    # angle (rad) per joint.  Unused in antagonistic mode.  ❓ TBD
    spring_stiffness: tuple[float, float, float] = (0.5, 0.5, 0.3)
    spring_rest_angle: tuple[float, float, float] = (0.0, 0.4, 0.0)


@dataclass(frozen=True)
class LoadCase:
    """A static loading scenario for the torque budget."""

    name: str
    body_mass_kg: float = 3.0          # total robot mass.  ❓ TBD
    n_stance_legs: int = 2             # legs sharing the load (e.g. trot => 2).
    dynamic_factor: float = 1.5        # peak/static impact multiplier.  ❓ TBD

    @property
    def foot_support_force_N(self) -> float:
        """Vertical force one stance leg must produce to support the body."""
        return (
            self.body_mass_kg
            * GRAVITY
            * self.dynamic_factor
            / max(self.n_stance_legs, 1)
        )


# Convenience singletons used by the demo and tests.
DEFAULT_LEG = LegParams()
DEFAULT_TENDON = TendonParams()
DEFAULT_LOADS: tuple[LoadCase, ...] = (
    LoadCase("stand (4-leg)", n_stance_legs=4, dynamic_factor=1.0),
    LoadCase("trot (2-leg)", n_stance_legs=2, dynamic_factor=1.5),
    LoadCase("land (1-leg)", n_stance_legs=1, dynamic_factor=2.5),
)
