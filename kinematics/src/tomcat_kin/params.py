"""Placeholder parameters for the TomCat single-leg model.

IMPORTANT: every numeric value here is a PLACEHOLDER (❓ TBD in
docs/REQUIREMENTS.md). They exist so the model runs end-to-end; they are not
committed design values. Swap them once mechanical design lands.

Conventions
-----------
- SI units throughout: metres, radians, kilograms, newtons, newton-metres.
- Sagittal-plane (2D) model: x forward, z up, hip at the origin.
- A DIGITIGRADE 4-link chain (cats stand on their toes):
      hip -> femur -> stifle/knee -> tibia -> hock/ankle -> metatarsus -> paw.
  The first THREE joints (hip, stifle, hock) are ACTUATED; the paw is a PASSIVE
  distal link held at a fixed rest angle relative to the metatarsus. There is NO
  fourth motor: the actuated joint vector stays q = (q1, q2, q3) (see LegParams).

=====================================================================
MASS BUDGET (M4 — "real mass"): how the 3.0 kg was apportioned  ❓ ALL PLACEHOLDER
=====================================================================
Before M4 every link was MASSLESS and ``whole_body_budget`` lumped EQUAL point
masses at the vertebrae (its own assumption A2 conceded real cats are ~60%
front-heavy). The numbers below replace that with a distributed, front-heavy
budget. They are apportioned, NOT measured — every one is ❓ TBD.

Apportionment rule (documented so it can be re-derived / re-tuned):

1. TOTAL = ``LoadCase.body_mass_kg`` = **3.00 kg** (unchanged).
2. LIMBS ≈ 24% of body mass = **0.72 kg** for all four legs. Hind limbs are
   HEAVIER than fore limbs in a cat (propulsion musculature): 0.20 kg per hind
   leg, 0.16 kg per fore leg -> 2(0.20) + 2(0.16) = 0.72 kg.
3. Within a leg the mass is PROXIMAL-heavy (biology, and the ADR-0003 tendon
   drive deliberately centralises mass): femur/humerus 47.5%, tibia/radius 30%,
   metatarsus/metacarpus 15%, paw 7.5%.
4. TRUNK = 3.00 - 0.72 = **2.28 kg**, split into three spine segments
   (0.30 / 0.45 / 0.55 kg, rear->front: lumbar lighter, thoracic + ribcage +
   viscera heavier, per mechanical/reference/ANATOMY.md's 13-thoracic /
   7-lumbar formula) plus two girdles. The FRONT girdle mass (0.70 kg)
   deliberately absorbs the HEAD + NECK, which are not separate bodies in this
   model and are a large part of why a cat is front-heavy; the REAR girdle
   (pelvis, 0.28 kg) is much lighter.
5. The girdle masses were then SOLVED so the fore/hind weight split lands at
   ~60/40. "Fore share" of a distributed trunk is defined by the LEVER RULE
   (the reaction split of a straight, simply-supported trunk): a spine segment
   whose CoM sits a fraction s of the way from the rear to the front girdle
   contributes s of its mass to the forequarters and (1-s) to the hindquarters.
   See ``mass.quarter_masses``. Result: forequarters 1.798 kg (59.9%),
   hindquarters 1.202 kg (40.1%).

Why NOT the literature's 0.454 kg knee mass
-------------------------------------------
docs/LITERATURE_REVIEW.md (Q2b) records a Mass-Mass-Spring leg model with
**~0.454 kg at the knee** that produces realistic trunk bending where a massless
SLIP model gives a null bending moment. That number is from a MUCH LARGER robot
and must NOT be copied literally: 0.454 kg is ~15% of our entire 3 kg body at a
single joint. The scaled analogue here is the whole 0.20 kg hind leg (6.7% of
body mass) with 0.095 kg at the femur. What we DO take from that result is the
qualitative lesson — leg mass is not negligible and materially bends a compliant
trunk — which is why M4 distributes mass over the links instead of lumping it.

Scope limit: these masses feed a QUASI-STATIC model (gravity + CoM + support
polygon). No inertias/velocities/accelerations are modelled; rotational inertia
tensors and full Newton-Euler are deferred to the dynamics milestone.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

GRAVITY = 9.81  # m/s^2


@dataclass(frozen=True)
class LegParams:
    """Geometry of one DIGITIGRADE leg (sagittal-plane 4-link chain).

    Four links -- femur, tibia, metatarsus, paw -- but only THREE ACTUATED
    joints (hip, stifle/knee, hock/ankle). The paw is a PASSIVE distal link held
    rigidly at `paw_angle` relative to the metatarsus, modelling the largely
    passive toes / ground contact of a digitigrade stance (long near-vertical
    metatarsus, hock held high). No fourth motor exists: the actuated joint
    vector stays q = (q1, q2, q3).
    """

    # Link lengths (m): femur, tibia, metatarsus, paw.  Digitigrade cat-scale
    # placeholders; total reach ~0.28 m keeps the previous ground clearance so
    # standing/gait still reach the floor.  ❓ TBD
    l1: float = 0.090   # femur
    l2: float = 0.095   # tibia
    l3: float = 0.070   # metatarsus (long, near-vertical in stance)
    l4: float = 0.025   # paw / toes (PASSIVE distal link)

    # Fixed rest angle of the PASSIVE paw relative to the metatarsus (rad). The
    # paw direction is the metatarsus cumulative angle (q1+q2+q3) PLUS paw_angle,
    # so the paw pitch never has its own actuator. Positive = the paw rotates CCW
    # (toes lift toward horizontal) away from a downward-pointing metatarsus, i.e.
    # the digitigrade "standing on the toes" pose. Set to ~55 deg, an anatomical
    # toe-break for a proper digitigrade crouch (up from an earlier modest 30).  ❓ TBD
    paw_angle: float = math.radians(55.0)

    # Joint angle limits (rad), (min, max) per ACTUATED joint: hip, stifle, hock.
    #
    # NEGATIVE-KNEE (anatomical fold) convention — see `KneeConfig` in leg.py.
    # M4's stability check exposed that the earlier POSITIVE-knee limits
    # (`stifle >= 0`) made it geometrically impossible to plant a paw under its
    # own hip: doing so demanded a hip angle of ~+167 deg, so every foot landed
    # ~0.2 m ahead of its hip and the whole machine was fore/aft unstable. On the
    # negative branch the same pose is ordinary (hip ~-71 deg), so the stifle is
    # now restricted to a NEGATIVE range, which encodes the fold direction
    # structurally: femur angles down-and-forward, tibia folds back under it,
    # metatarsus rises to a high hock — the digitigrade Z.
    #
    # Ranges are generous placeholders around the demanded working set (at the
    # default stance both fore and hind need only hip -76..-26, stifle -113..-69,
    # hock +73..+124 deg), leaving margin for swing and deeper crouches.  ❓ TBD
    q_min: tuple[float, float, float] = (
        math.radians(-120.0),   # hip
        math.radians(-150.0),   # stifle (knee) — negative fold only
        math.radians(-30.0),    # hock (ankle)
    )
    q_max: tuple[float, float, float] = (
        math.radians(120.0),    # hip
        0.0,                    # stifle — never crosses into positive fold
        math.radians(150.0),    # hock
    )

    # --- MASS PROPERTIES (M4).  ❓ ALL PLACEHOLDER; see the module docstring for
    #     how the 3.0 kg body was apportioned.
    #
    # Per-link mass (kg), same order as the link lengths: femur, tibia,
    # metatarsus, paw.  These defaults are the HIND leg (0.200 kg total, 6.7% of
    # a 3 kg body); ``DEFAULT_FORELEG`` overrides them with a lighter 0.160 kg
    # columnar limb.  Proximal-heavy (47.5 / 30 / 15 / 7.5 %) because both feline
    # anatomy and the ADR-0003 tendon drive push mass toward the body.
    link_mass: tuple[float, float, float, float] = (0.095, 0.060, 0.030, 0.015)

    # Fraction of each link's LENGTH, measured from that link's PROXIMAL joint,
    # at which its centre of mass sits (dimensionless, 0 = proximal joint,
    # 1 = distal joint).  0.45 on the two muscled proximal links (muscle bellies
    # sit proximally), 0.50 on the mostly-bony distal links.  ❓ TBD
    link_com_frac: tuple[float, float, float, float] = (0.45, 0.45, 0.50, 0.50)

    def __post_init__(self) -> None:
        for name in ("link_mass", "link_com_frac"):
            got = len(getattr(self, name))
            if got != 4:
                raise ValueError(
                    f"LegParams.{name} has {got} entries; expected 4 "
                    "(femur, tibia, metatarsus, paw)"
                )
        if any(m < 0.0 for m in self.link_mass):
            raise ValueError("LegParams.link_mass entries must be non-negative")

    @property
    def reach(self) -> float:
        """Maximum straight-leg distance from hip to paw tip (all four links)."""
        return self.l1 + self.l2 + self.l3 + self.l4

    @property
    def link_lengths(self) -> tuple[float, float, float, float]:
        """(l1, l2, l3, l4) as a tuple, proximal -> distal."""
        return (self.l1, self.l2, self.l3, self.l4)

    @property
    def mass(self) -> float:
        """Total mass of the four links of this leg (kg)."""
        return float(sum(self.link_mass))


@dataclass(frozen=True)
class TendonParams:
    """Tendon routing and actuator parameters, per joint.

    Defaults describe the 3-joint leg (hip, knee, ankle), but every per-joint
    tuple may be any length: the array length sets the number of joints, so the
    spine reuses this container (see `SpineParams` / `TendonMap.from_spine`).
    """

    # Joint pulley radii / moment arms (m) — converts tension to joint torque.
    # Sized in mechanical/LEG_TENDON_SPEC.md (hip/knee/ankle): largest packageable
    # pulley at each joint to cut cable tension (T = tau/r).  Roughly halves the
    # land-case peak vs. the original (0.015,0.012,0.010).  Still ❓ TBD.
    joint_moment_arm: tuple[float, ...] = (0.028, 0.025, 0.014)

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
    #   friction_coeff : mu, dimensionless Coulomb coefficient of the routing.
    #   wrap_angle     : theta_wrap, TOTAL cable wrap over all guides (rad).
    # mu = 0 OR wrap = 0  =>  factor = 1  =>  motor-side tension == joint-side.
    # mechanical/LEG_TENDON_SPEC.md gives mu ~= 0.10 (low-friction idlers) and
    # PER-STATION wrap angles that differ by joint (the distal ankle path is worst,
    # ~+87% motor-side). This scalar model can't hold per-joint wrap, so wrap_angle
    # is left 0 (inert) pending a per-joint-wrap extension; set mu here so it's
    # ready, and use the sensitivity tool with explicit wrap to explore the effect.
    friction_coeff: float = 0.10
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

    # --- MASS PROPERTIES (M4).  ❓ ALL PLACEHOLDER; see the params.py module
    #     docstring for the full apportionment of the 3.0 kg body.
    #
    # Per-segment TRUNK mass (kg), rear -> front. Rear = lumbar (lighter, the
    # mobile bending region); front = thoracic/ribcage + viscera (heavier). Sums
    # to 1.30 kg.  Matches the 13-thoracic / 7-lumbar formula in
    # mechanical/reference/ANATOMY.md qualitatively, not quantitatively.
    segment_mass: tuple[float, ...] = (0.30, 0.45, 0.55)

    # Fraction along each segment (from its INBOARD/rear vertebra) at which that
    # segment's mass acts. 0.5 = uniform rod.  ❓ TBD
    segment_com_frac: tuple[float, ...] = (0.5, 0.5, 0.5)

    # Girdle (limb-girdle) masses (kg), lumped at the corresponding end vertebra.
    # FRONT = shoulder girdle; it deliberately ABSORBS the HEAD + NECK, which are
    # not separate bodies in this model and are a big part of why a cat is
    # front-heavy. REAR = pelvic girdle (much lighter). These two values were the
    # free variables solved to land the fore/hind weight split at ~60/40.  ❓ TBD
    front_girdle_mass: float = 0.70
    rear_girdle_mass: float = 0.28

    # Girdle CoM offset (x, z) in the girdle's own frame (m). (0, 0) = the mass
    # acts exactly at the girdle mount vertebra.  ❓ TBD
    front_girdle_com: tuple[float, float] = (0.0, 0.0)
    rear_girdle_com: tuple[float, float] = (0.0, 0.0)

    def __post_init__(self) -> None:
        for name in (
            "segment_lengths",
            "q_min",
            "q_max",
            "joint_moment_arm",
            "spring_stiffness",
            "spring_rest_angle",
            "segment_mass",
            "segment_com_frac",
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

    @property
    def chain_mass(self) -> float:
        """Mass of the spine SEGMENTS alone (kg), excluding the girdles."""
        return float(sum(self.segment_mass))

    @property
    def trunk_mass(self) -> float:
        """Spine segments + both girdles (kg) -- the whole body minus the legs."""
        return self.chain_mass + self.front_girdle_mass + self.rear_girdle_mass


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

# Cats are NOT fore/hind symmetric. The nominal DEFAULT_LEG doubles as the
# HIND leg (longer shank, bigger toe-break — the folded propulsion limb); the
# FORE leg is a touch more columnar (longer proximal humerus, shorter distal
# metacarpus, smaller paw toe-break). Reaches are kept ≈equal (~0.28 m) so the
# body stays roughly level. Used by the CAD to distinguish front vs rear legs;
# the actuated architecture (3 joints/leg) is identical.  ❓ placeholders.
DEFAULT_HINDLEG = DEFAULT_LEG
DEFAULT_FORELEG = LegParams(
    l1=0.100, l2=0.090, l3=0.065, l4=0.025,   # humerus, radius, metacarpus, paw
    paw_angle=math.radians(40.0),
    # 0.160 kg total vs. the hind leg's 0.200 kg: the fore limb is the lighter,
    # more columnar limb; the hind limb carries the propulsion musculature. Same
    # proximal-heavy 47.5 / 30 / 15 / 7.5 % split.  ❓ TBD
    link_mass=(0.076, 0.048, 0.024, 0.012),
)

# Total mass of the default body, for cross-checking against LoadCase.body_mass_kg.
# 2 fore legs + 2 hind legs + spine segments + both girdles == 3.00 kg by
# construction (see the module docstring's apportionment rule).
DEFAULT_BODY_MASS_KG = (
    2 * DEFAULT_FORELEG.mass + 2 * DEFAULT_HINDLEG.mass + DEFAULT_SPINE.trunk_mass
)
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
