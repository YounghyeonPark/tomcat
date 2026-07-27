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
    """Tendon routing and actuator parameters, per joint.

    Defaults describe the 3-joint leg (hip, knee, ankle), but every per-joint
    tuple may be any length: the array length sets the number of joints, so the
    spine reuses this container (see `SpineParams` / `TendonMap.from_spine`).
    """

    # Joint pulley radii / moment arms (m) — converts tension to joint torque.  ❓ TBD
    joint_moment_arm: tuple[float, ...] = (0.015, 0.012, 0.010)

    # Motor spool radius (m) — converts motor torque to cable tension.  ❓ TBD
    motor_spool_radius: float = 0.008

    # Minimum tension kept in every cable so it never goes slack (N).  ❓ TBD
    # In antagonistic mode this is the co-contraction floor on the "slack" side.
    pretension: float = 5.0

    # Spring-return mode only: torsional spring stiffness (N·m/rad) and rest
    # angle (rad) per joint.  Unused in antagonistic mode.  ❓ TBD
    spring_stiffness: tuple[float, ...] = (0.5, 0.5, 0.3)
    spring_rest_angle: tuple[float, ...] = (0.0, 0.4, 0.0)


@dataclass(frozen=True)
class SpineParams:
    """Geometry / actuation of the articulated spine (sagittal-plane chain).

    The spine is modelled as a serial chain of revolute joints (one per
    inter-vertebral segment) in the SAME sagittal plane as the leg model:
    x forward, z up. Per ADR-0006 and the literature review, the seed geometry
    is 3 segments; the review recommends 2-3 segments x ~3 DOF each. This 2D
    model exposes ONLY the sagittal (dorsoventral flexion/extension) DOF of each
    segment — lateral bending and axial rotation are out of plane and NOT yet
    modelled.

    Sign convention (matches the leg): a segment's cumulative direction angle is
    measured CCW from +x (from +x toward +z). A POSITIVE joint angle rotates the
    outboard portion of the spine CCW relative to the inboard segment, i.e. it
    lifts the distal end dorsally (upward). A uniform positive bend curls the
    chain upward into a dorsiflexed / arched-back ("Halloween cat") posture.

    IMPORTANT stiffness caveat
    --------------------------
    The cat whole-spine value **53.62 N/mm is AXIAL COMPRESSIVE stiffness
    (force/length)**, NOT the per-joint ROTATIONAL stiffness (N·m/rad) an
    articulated tendon spine needs. It must NOT be dropped into `spring_stiffness`
    below. A geometry-based conversion (axial N/mm + segment lever arms ->
    per-joint N·m/rad) is still owed; `spring_stiffness` here is an unsourced
    placeholder pending that conversion.

    Directional compliance rank (cat FEA 2024), most-compliant -> stiffest:
        axial rotation  >  extension (dorsoventral)  >  lateral bending
    Only the middle axis (extension) lives in this 2D sagittal model; the rank is
    recorded here so per-axis stiffness/limits can be seeded correctly once the
    model is extended to 3D.
    """

    # Number of inter-vertebral revolute segments.  Seed = 3 (ADR-0006).
    n_segments: int = 3

    # Segment (link) lengths, base/rear -> front (m).  ❓ TBD placeholder.
    # ~0.18 m total is a rough cat-torso scale; not a committed value.
    segment_lengths: tuple[float, ...] = (0.060, 0.060, 0.060)

    # Per-segment sagittal joint-angle limits (rad), (min, max).  ❓ TBD.
    # ±25° per joint -> ~±75° whole-spine sagittal range.
    q_min: tuple[float, ...] = (-0.436, -0.436, -0.436)
    q_max: tuple[float, ...] = (0.436, 0.436, 0.436)

    # Per-segment tendon moment arm about the vertebral joint (m).  ❓ TBD.
    # Larger than the leg's because the spine tendons run further off-axis.
    joint_moment_arm: tuple[float, ...] = (0.020, 0.020, 0.020)

    # Motor spool radius for the spine tendons (m).  ❓ TBD.
    motor_spool_radius: float = 0.008

    # Minimum cable tension / antagonistic co-contraction floor (N).  ❓ TBD.
    # Lit. sanity band is ~20-70 N (RoboCat pretension ~50 N); kept at the leg's
    # 5 N placeholder for now so the two budgets are comparable.
    pretension: float = 5.0

    # Spring-return mode only: per-segment torsional stiffness (N·m/rad) and rest
    # angle (rad).  ❓ PLACEHOLDER — NOT derived from the 53.62 N/mm axial value
    # (see the stiffness caveat above); needs the axial->rotational conversion.
    spring_stiffness: tuple[float, ...] = (2.0, 2.0, 2.0)
    spring_rest_angle: tuple[float, ...] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        for name in (
            "segment_lengths",
            "q_min",
            "q_max",
            "joint_moment_arm",
            "spring_stiffness",
            "spring_rest_angle",
        ):
            got = len(getattr(self, name))
            if got != self.n_segments:
                raise ValueError(
                    f"SpineParams.{name} has {got} entries; "
                    f"expected n_segments={self.n_segments}"
                )

    @property
    def total_length(self) -> float:
        """Straight-spine distance from the rear to the front girdle mount (m)."""
        return float(sum(self.segment_lengths))


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
DEFAULT_SPINE = SpineParams()
DEFAULT_LOADS: tuple[LoadCase, ...] = (
    LoadCase("stand (4-leg)", n_stance_legs=4, dynamic_factor=1.0),
    LoadCase("trot (2-leg)", n_stance_legs=2, dynamic_factor=1.5),
    LoadCase("land (1-leg)", n_stance_legs=1, dynamic_factor=2.5),
)
