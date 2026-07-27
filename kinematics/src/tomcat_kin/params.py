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

    # --- Tendon non-idealities (ADR-0003: friction & stretch are now leg-side
    #     concerns too, not spine-only). Defaults reduce EXACTLY to the previous
    #     frictionless / inextensible behaviour, so existing budgets are unchanged.

    # Capstan (Coulomb) friction over the routing pulleys / sheaths. The motor-side
    # cable tension differs from the joint-side tension by exp(±mu * wrap_angle):
    # PULLING against the load costs exp(+mu*wrap), PAYING OUT gains exp(-mu*wrap).
    #   friction_coeff : mu, dimensionless Coulomb coefficient of the routing.  ❓ TBD
    #   wrap_angle     : theta_wrap, TOTAL cable wrap over all guides (rad).      ❓ TBD
    # mu = 0 OR wrap = 0  =>  factor = 1  =>  motor-side tension == joint-side.
    friction_coeff: float = 0.0
    wrap_angle: float = 0.0

    # Series cable compliance: model the tendon as a linear spring of stiffness
    # k_cable (N/m). Under tension T it stretches dL = T / k_cable, so the motor
    # must wind extra travel (dL / r_spool) beyond the geometric r*q to hold a
    # joint angle; if uncompensated the joint under-rotates by dL / r.  ❓ TBD.
    #   None (or a non-finite value such as inf)  =>  inextensible, no stretch.
    k_cable: float | None = None

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

    # Segment (link) lengths, base/rear -> front (m).  First-pass from
    # mechanical/SPINE_TAIL_SPEC.md: tapered (rear lumbar longer/more mobile),
    # 0.195 m total at ~3 kg cat-torso scale.  Still a placeholder, not committed.
    segment_lengths: tuple[float, ...] = (0.075, 0.065, 0.055)

    # Per-segment sagittal joint-angle limits (rad), (min, max).  ❓ TBD.
    # ±25° per joint -> ~±75° whole-spine sagittal range.
    q_min: tuple[float, ...] = (-0.436, -0.436, -0.436)
    q_max: tuple[float, ...] = (0.436, 0.436, 0.436)

    # Per-segment tendon moment arm about the vertebral joint (m).
    # Raised from the initial 0.020 m to 0.030 m per mechanical/SPINE_TAIL_SPEC.md:
    # T = tau/r, so 0.020 m amplified peak cable tension well above the ~20-70 N
    # RoboCat band; ~0.030 m brings it near the top of the band.  Still ❓ TBD.
    joint_moment_arm: tuple[float, ...] = (0.030, 0.030, 0.030)

    # Motor spool radius for the spine tendons (m).  ❓ TBD.
    motor_spool_radius: float = 0.008

    # Minimum cable tension / mechanical slack floor (N).  ❓ TBD.
    # Kept at the leg's 5 N so the two budgets are comparable.  Note: the AIC
    # *control* co-contraction bias is a separate, larger quantity — Kengoro's
    # T_bias ≈ 19.6 N (LITERATURE_REVIEW.md Seed derivation B) — passed at runtime
    # via TendonMap.resolve(..., t_bias=...), not baked in here.
    pretension: float = 5.0

    # Spring-return mode only: per-segment torsional stiffness (N·m/rad) and rest
    # angle (rad).  Seeded ~1.0 N·m/rad from the axial->rotational geometry bridge
    # in LITERATURE_REVIEW.md (Seed derivation A): the sagittal (dorsoventral
    # extension) axis this 2D model exercises.  ◐/⚠️ ORDER-OF-MAGNITUDE ONLY —
    # good to a factor of ~2-3; correct in ranking/scale, not a measured value.
    spring_stiffness: tuple[float, ...] = (1.0, 1.0, 1.0)
    spring_rest_angle: tuple[float, ...] = (0.0, 0.0, 0.0)

    # Tendon non-idealities, same meaning as TendonParams (capstan friction +
    # series cable stretch). Long spine tendons run over many vertebral guides,
    # so wrap_angle is expected to be LARGER here than at a leg once sourced.
    # Defaults reduce to the previous frictionless / inextensible behaviour.  ❓ TBD.
    friction_coeff: float = 0.0
    wrap_angle: float = 0.0
    k_cable: float | None = None

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


@dataclass(frozen=True)
class WholeBodyLoadCase:
    """A whole-body static loading scenario (spine posture + which legs bear load).

    Extends `LoadCase` to the combined budget: the spine is posed at `spine_q`
    (per-segment sagittal angles, rad) and only the legs in `stance_legs` are
    planted and share the support force. Feeds `whole_body_budget.evaluate`.

    The per-leg leg-workspace sweep reuses a plain `LoadCase` (see `.leg_load`);
    the spine joint loads are derived from the girdle reactions + a distributed
    body-weight gravity moment in the arched geometry (see whole_body_budget).
    """

    name: str
    body_mass_kg: float = 3.0
    dynamic_factor: float = 1.0
    # Spine posture for this case (rad per segment).  Length must match the
    # SpineParams used in the budget.  Straight = zeros; arch = uniform positive.
    spine_q: tuple[float, ...] = (0.0, 0.0, 0.0)
    # Which leg mounts are planted (bearing load) in this case.
    stance_legs: tuple[str, ...] = ("LF", "RF", "LR", "RR")

    @property
    def n_stance_legs(self) -> int:
        return len(self.stance_legs)

    @property
    def foot_support_force_N(self) -> float:
        """Vertical force one stance leg must produce to support the body."""
        return (
            self.body_mass_kg
            * GRAVITY
            * self.dynamic_factor
            / max(self.n_stance_legs, 1)
        )

    @property
    def leg_load(self) -> LoadCase:
        """A per-leg `LoadCase` so the existing leg budget sweep can be reused."""
        return LoadCase(
            name=self.name,
            body_mass_kg=self.body_mass_kg,
            n_stance_legs=self.n_stance_legs,
            dynamic_factor=self.dynamic_factor,
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

# Whole-body cases for the combined spine+legs budget. Spine postures assume the
# 3-segment DEFAULT_SPINE (arch = uniform +20 deg dorsiflexion).
_ARCH = (0.349, 0.349, 0.349)  # ~20 deg per segment
DEFAULT_WHOLE_BODY_LOADS: tuple[WholeBodyLoadCase, ...] = (
    # Quiet stand: straight spine, all four legs planted.
    WholeBodyLoadCase(
        "stand (4-leg, straight)",
        dynamic_factor=1.0,
        spine_q=(0.0, 0.0, 0.0),
        stance_legs=("LF", "RF", "LR", "RR"),
    ),
    # Arched "Halloween cat": same support, but the dorsiflexed geometry
    # redistributes the spine joint loads (curls the forequarters up and back
    # over the pelvis, shifting where the gravity/reaction moments land).
    WholeBodyLoadCase(
        "arch (4-leg, dorsiflexed)",
        dynamic_factor=1.0,
        spine_q=_ARCH,
        stance_legs=("LF", "RF", "LR", "RR"),
    ),
    # Single-leg front landing: one front leg takes the whole impact, so the
    # front-girdle reaction is large and unbalanced by the rear.
    WholeBodyLoadCase(
        "land (1 front leg)",
        dynamic_factor=2.5,
        spine_q=(0.0, 0.0, 0.0),
        stance_legs=("LF",),
    ),
)
