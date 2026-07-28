"""Parameterized periodic WALK gait generation (sagittal plane, quasi-static).

This module turns the static whole-body model (``spine.py`` + ``leg.py``) into
*motion*: a periodic walk that produces per-leg joint-angle trajectories over one
gait cycle by generating a foot trajectory for each leg and running the existing
per-leg inverse kinematics (``LegModel.inverse``) on it. It is the M2 milestone
layer that sits on top of M1's kinematics (see docs/ROADMAP.md).

Frames & conventions
--------------------
- Sagittal plane only: x forward, z up (matches ``leg.py`` / ``spine.py``).
- Foot targets are expressed in each leg's OWN hip/leg frame — the same frame
  ``LegModel.forward`` / ``.inverse`` use (hip at the origin, x forward, z up).
  We never move the world; the body advancing forward shows up as the foot
  sweeping backward (-x) in the hip frame during stance.
- Angles are radians internally; joint vectors are the leg's (q1, q2, q3).
- Gait "phase" is a dimensionless fraction of the cycle in [0, 1); phase wraps at
  1.0 (phase 1.0 == phase 0.0). Time maps to phase via ``phase = (t / period) % 1``.
- Each leg has a phase OFFSET: its local phase is ``(phase - offset) % 1``. A leg
  is in STANCE for local phase in ``[0, duty_factor)`` and in SWING for local
  phase in ``[duty_factor, 1)`` (half-open, so touchdown counts as stance).

Gait timing (the walk)
----------------------
The default is a statically stable, lateral-sequence walk. With a duty factor of
0.75 each leg swings for 1/4 of the cycle, and the four swing windows are spaced
0.25 apart so they tile the cycle exactly: at every phase EXACTLY ONE leg is in
swing and the other THREE are planted (a 3-foot support polygon at all times,
the classic crawl condition for static stability).

Default per-leg phase offsets (touchdown phase of each leg):

    LF = 0.00,  RF = 0.25,  RR = 0.50,  LR = 0.75

so the liftoff (swing) sequence around the cycle is LF -> RF -> RR -> LR. (The
biological cat lateral-sequence walk LH->LF->RH->RF is the same family; only the
offset-to-leg labelling differs. Any assignment of the offset SET {0, .25, .5,
.75} keeps the 3-foot support property.)

Body speed relationship
------------------------
A planted foot sweeps ``stride_length`` backward (in the hip frame) over the
stance duration ``duty_factor * period``. For non-slipping contact the body
therefore advances forward at::

    body_speed        = stride_length / (duty_factor * period)   [m/s]
    distance_per_cycle = stride_length / duty_factor              [m/cycle]

Foot trajectory
---------------
- STANCE (fraction u in [0, 1] through stance): the foot is on the ground and
  moves backward at constant speed from +stride/2 (front, at touchdown) to
  -stride/2 (rear, at liftoff); height held at the nominal ground level::
      x = x_nom + (stride/2) * (1 - 2u)
      z = z_nom
- SWING (fraction v in [0, 1] through swing): a smooth CYCLOID returns the foot
  forward while lifting it, with zero horizontal AND vertical velocity at both
  liftoff and touchdown::
      x = x_nom - stride/2 + stride * (v - sin(2*pi*v)/(2*pi))
      z = z_nom + step_height * (1 - cos(2*pi*v)) / 2
  so lift above ground is 0 at v=0 and v=1 and peaks at ``step_height`` at v=0.5.
- Foot pitch phi is held at ``foot_pitch`` throughout.

Optional spine coupling (off by default)
-----------------------------------------
With ``spine_amplitude > 0`` the controller returns a small sinusoidal
dorsoventral spine bend coupled to the gait phase (a uniform per-segment angle
``spine_amplitude * sin(2*pi*(phase + spine_phase))``) so spine<->gait coupling
can be demonstrated. It defaults OFF (amplitude 0 => the spine is held NEUTRAL,
all-zero angles). NOTE: because foot targets live in the (moving) hip frame, the
spine bend changes where the feet are IN THE WORLD but does NOT change the per-leg
IK solution — it is a demonstration of coupling, not a whole-body foot-placement
controller. Closing that loop (placing feet in a world/ground frame through the
spine) is future work.

Modelling assumptions / limitations (flagged per engineering standards)
-----------------------------------------------------------------------
- QUASI-STATIC: no dynamics. No velocities, accelerations, inertias, or
  ground-reaction forces are modelled; the trajectory is a pure geometric
  sequence of postures. Leg mass (the literature shows it materially bends a
  compliant trunk) is ignored, so this will not capture spine-leg energy exchange
  or momentum effects.
- SAGITTAL-ONLY: no frontal-plane (roll/abduction) or yaw motion; left/right legs
  on a girdle share a mount pose in this 2D projection (inherited from spine.py).
- STABILITY: the only stability notion checked is the stance-leg COUNT (>=3 feet
  down for the default walk). There is NO ground-reaction / ZMP / support-polygon
  margin computation — a 3-foot count does not by itself guarantee the CoM lies
  inside the support triangle.
- Foot targets are in the HIP frame (see above); ground contact is idealized as
  a point at the nominal foot height with no friction/slip model.
- Rigid links, frictionless idealization inherited from leg.py / spine.py.
- All numeric defaults are ILLUSTRATIVE, built on the PLACEHOLDER params in
  params.py; they are tuned only to stay inside the placeholder joint limits.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Mapping

import numpy as np

from .leg import LegModel, KneeConfig, UnreachableError
from .spine import WholeBody, SpineModel, LegIKSolution


# Default lateral-sequence walk offsets (touchdown phase per leg). The offset SET
# {0, .25, .5, .75} with duty 0.75 keeps exactly three feet planted at all times.
DEFAULT_PHASE_OFFSETS: dict[str, float] = {
    "LF": 0.00,
    "RF": 0.25,
    "RR": 0.50,
    "LR": 0.75,
}


@dataclass(frozen=True)
class GaitParams:
    """Parameters of a periodic sagittal-plane walk.

    All lengths are metres, angles radians, times seconds (SI). Defaults are
    ILLUSTRATIVE and tuned to stay inside the placeholder leg joint limits with a
    ~11 deg margin over the whole cycle (see module docstring / demo).

    Attributes
    ----------
    period : float
        Gait cycle duration (s).
    stride_length : float
        Horizontal foot excursion during stance (m), i.e. peak-to-peak sweep.
        The body advances this far per stance (see ``body_speed``).
    step_height : float
        Peak swing lift above the nominal ground level (m).
    duty_factor : float
        Fraction of the cycle each leg spends in stance, in (0, 1). 0.75 gives the
        3-foot-support walk (one leg swinging at a time).
    nominal_foot : tuple[float, float]
        (x, z) of the mid-stride foot position in the hip frame (m). z is
        negative (foot below the hip).
    foot_pitch : float
        Constant foot pitch phi (rad) held over the trajectory.
    phase_offsets : Mapping[str, float]
        Per-leg touchdown phase in [0, 1). Keys are leg names ("LF" etc.).
    knee : KneeConfig
        Which 2R IK branch each leg uses. FLEXED_POSITIVE keeps the knee angle
        q2 >= 0, matching the placeholder ``LegParams`` knee limit [0, 144 deg].
    spine_amplitude : float
        Amplitude (rad, per segment) of the OPTIONAL dorsoventral spine
        oscillation coupled to the gait. 0.0 (default) => spine held NEUTRAL.
    spine_phase : float
        Phase lead (fraction of a cycle) of the spine oscillation relative to the
        gait phase.
    """

    period: float = 1.2
    stride_length: float = 0.06
    step_height: float = 0.03
    duty_factor: float = 0.75
    nominal_foot: tuple[float, float] = (0.22, -0.13)
    foot_pitch: float = -math.radians(20.0)
    phase_offsets: Mapping[str, float] = field(
        default_factory=lambda: dict(DEFAULT_PHASE_OFFSETS)
    )
    knee: KneeConfig = KneeConfig.FLEXED_POSITIVE
    spine_amplitude: float = 0.0
    spine_phase: float = 0.0

    def __post_init__(self) -> None:
        if not 0.0 < self.duty_factor < 1.0:
            raise ValueError(f"duty_factor must be in (0, 1); got {self.duty_factor}")
        if self.period <= 0.0:
            raise ValueError(f"period must be > 0; got {self.period}")
        for name, off in self.phase_offsets.items():
            if not 0.0 <= off < 1.0:
                raise ValueError(
                    f"phase offset for {name} must be in [0, 1); got {off}"
                )

    # ---------------------------------------------------------------- derived
    @property
    def swing_fraction(self) -> float:
        """Fraction of the cycle spent in swing (= 1 - duty_factor)."""
        return 1.0 - self.duty_factor

    @property
    def body_speed(self) -> float:
        """Forward body speed for non-slipping stance (m/s)."""
        return self.stride_length / (self.duty_factor * self.period)

    @property
    def distance_per_cycle(self) -> float:
        """Forward distance the body advances over one full cycle (m)."""
        return self.stride_length / self.duty_factor


# --------------------------------------------------------------------- results
@dataclass(frozen=True)
class LegState:
    """Kinematic state of one leg at a queried phase.

    Attributes
    ----------
    name : str
        Leg label.
    in_stance : bool
        True if the leg is planted (stance), False if swinging.
    local_phase : float
        The leg's own phase in [0, 1) after applying its offset.
    foot_target : np.ndarray
        Target foot pose (x, z, phi) in the hip frame (m, m, rad).
    q : np.ndarray | None
        Solved joint angles (q1, q2, q3) in rad, or None if the target was
        UNREACHABLE (IK failed).
    reachable : bool
        Whether IK produced a solution at all.
    within_limits : bool
        Whether the solved q is inside the leg's configured joint limits. False
        if unreachable.
    """

    name: str
    in_stance: bool
    local_phase: float
    foot_target: np.ndarray
    q: np.ndarray | None
    reachable: bool
    within_limits: bool

    @property
    def ok(self) -> bool:
        """True only if reachable AND within joint limits."""
        return self.reachable and self.within_limits


@dataclass(frozen=True)
class GaitState:
    """Whole-body kinematic state at a queried phase.

    Attributes
    ----------
    phase : float
        Wrapped gait phase in [0, 1).
    legs : dict[str, LegState]
        Per-leg state keyed by leg name.
    spine_q : np.ndarray
        Spine joint angles (rad) at this phase; all-zero unless spine coupling is
        enabled.
    """

    phase: float
    legs: dict[str, LegState]
    spine_q: np.ndarray

    @property
    def stance_legs(self) -> tuple[str, ...]:
        return tuple(n for n, s in self.legs.items() if s.in_stance)

    @property
    def swing_legs(self) -> tuple[str, ...]:
        return tuple(n for n, s in self.legs.items() if not s.in_stance)

    @property
    def stance_count(self) -> int:
        return len(self.stance_legs)

    @property
    def all_ok(self) -> bool:
        """True if every leg's pose is reachable and within limits."""
        return all(s.ok for s in self.legs.values())


# ------------------------------------------------------------------ trajectory
def foot_target(params: GaitParams, local_phase: float) -> np.ndarray:
    """Foot target pose (x, z, phi) in the hip frame for a leg's LOCAL phase.

    ``local_phase`` is the leg's own phase in [0, 1) (offset already applied):
    stance for ``[0, duty_factor)``, swing for ``[duty_factor, 1)``. See the
    module docstring for the stance line + cycloidal swing definitions.
    """
    s = float(local_phase) % 1.0
    x_nom, z_nom = params.nominal_foot
    half = params.stride_length / 2.0
    d = params.duty_factor

    if s < d:
        # Stance: linear backward sweep from +half (front) to -half (rear).
        u = s / d
        x = x_nom + half * (1.0 - 2.0 * u)
        z = z_nom
    else:
        # Swing: cycloid forward-and-up return.
        v = (s - d) / (1.0 - d)
        x = x_nom - half + params.stride_length * (
            v - math.sin(2.0 * math.pi * v) / (2.0 * math.pi)
        )
        z = z_nom + params.step_height * (1.0 - math.cos(2.0 * math.pi * v)) / 2.0
    return np.array([x, z, params.foot_pitch])


def swing_height(params: GaitParams, local_phase: float) -> float:
    """Foot lift above the nominal ground level (m) for a leg's local phase.

    0 during stance and at swing liftoff/touchdown; peaks at ``step_height``.
    """
    return float(foot_target(params, local_phase)[1] - params.nominal_foot[1])


# ------------------------------------------------------------------ controller
@dataclass
class GaitController:
    """Generates per-leg joint-angle trajectories for a walk via per-leg IK.

    Holds a ``WholeBody`` (for the legs and the spine model) and a ``GaitParams``.
    Query it with a phase in [0, 1) (``state``) or a time in seconds
    (``state_at_time``); it returns a ``GaitState`` with each foot's target,
    the IK solution, and stance/swing + reachability + joint-limit flags.
    Unreachable or out-of-limit poses are FLAGGED (q=None / within_limits=False),
    never raised, so a full cycle can always be sampled.

    Attributes
    ----------
    params : GaitParams
        The gait definition.
    body : WholeBody
        Source of the per-leg ``LegModel`` and the ``SpineModel``. Defaults to a
        fresh ``WholeBody`` (default spine + four default legs).
    """

    params: GaitParams = field(default_factory=GaitParams)
    body: WholeBody = field(default_factory=WholeBody)

    # -------------------------------------------------------------- phase math
    def local_phase(self, phase: float, leg_name: str) -> float:
        """The leg's own phase in [0, 1) (global phase minus its offset)."""
        off = self.params.phase_offsets[leg_name]
        return (float(phase) - off) % 1.0

    def is_stance(self, phase: float, leg_name: str) -> bool:
        """True if the leg is planted at ``phase`` (half-open [0, duty))."""
        return self.local_phase(phase, leg_name) < self.params.duty_factor

    def stance_count(self, phase: float) -> int:
        """Number of planted legs at ``phase``."""
        return sum(
            self.is_stance(phase, name) for name in self.params.phase_offsets
        )

    # -------------------------------------------------------------------- spine
    def spine_q(self, phase: float) -> np.ndarray:
        """Spine joint angles (rad) at ``phase``.

        All-zero (NEUTRAL) unless ``params.spine_amplitude > 0``, in which case a
        uniform dorsoventral sinusoid coupled to the gait phase is returned,
        clipped to the spine's joint limits.
        """
        n = self.body.spine.params.n_segments
        if self.params.spine_amplitude == 0.0:
            return np.zeros(n)
        ang = self.params.spine_amplitude * math.sin(
            2.0 * math.pi * (float(phase) % 1.0 + self.params.spine_phase)
        )
        q = np.full(n, ang)
        lo = np.asarray(self.body.spine.params.q_min)
        hi = np.asarray(self.body.spine.params.q_max)
        return np.clip(q, lo, hi)

    # -------------------------------------------------------------- leg solving
    def leg_state(self, phase: float, leg_name: str) -> LegState:
        """Solve one leg at ``phase`` (foot target -> IK -> flags)."""
        s = self.local_phase(phase, leg_name)
        in_stance = s < self.params.duty_factor
        target = foot_target(self.params, s)
        leg = self.body.legs[leg_name]
        try:
            q = leg.inverse(target, knee=self.params.knee)
            reachable = True
            within = bool(leg.in_limits(q))
        except UnreachableError:
            q = None
            reachable = False
            within = False
        return LegState(
            name=leg_name,
            in_stance=in_stance,
            local_phase=s,
            foot_target=target,
            q=q,
            reachable=reachable,
            within_limits=within,
        )

    def state(self, phase: float) -> GaitState:
        """Whole-body state at gait ``phase`` (phase wraps at 1.0)."""
        p = float(phase) % 1.0
        legs = {name: self.leg_state(p, name) for name in self.params.phase_offsets}
        return GaitState(phase=p, legs=legs, spine_q=self.spine_q(p))

    def state_at_time(self, t: float) -> GaitState:
        """Whole-body state at time ``t`` seconds (phase = (t/period) % 1)."""
        return self.state((t / self.params.period) % 1.0)

    def sample_cycle(self, n: int = 12) -> list[GaitState]:
        """``n`` evenly spaced states over one cycle, phase in [0, 1)."""
        return [self.state(i / n) for i in range(n)]


# ===================================================================
# Whole-body (world-frame) gait: the closed spine<->foot loop (M3)
# ===================================================================
#
# The default GaitController above places each foot target in that leg's own
# (moving) HIP frame, so a spine bend carries the feet through the world WITHOUT
# the legs compensating -- the coupling M2 could demonstrate but not CLOSE. The
# controller below closes it: foot targets live in a WORLD / body-ground frame,
# STANCE feet are held at FIXED world positions, and whole-body IK
# (``WholeBody.inverse``) is run THROUGH the moving girdle so the LEG joint
# angles absorb the spine motion and keep the planted feet put.
#
# Two frames (both x forward, z up, sagittal only)
# ------------------------------------------------
# - BODY-GROUND frame: the spine base (rear girdle, vertebra 0) sits at the
#   origin. This is the frame ``WholeBody`` FK/IK work in. It advances forward
#   with the body, so a planted foot sweeps BACKWARD in it during stance.
# - WORLD / ground frame: fixed to the ground. It is the body-ground frame
#   translated forward by ``body_offset(phase) = distance_per_cycle * phase``
#   (the body advances ``distance_per_cycle`` per cycle at ``body_speed``). A
#   foot planted on the ground is FIXED in this frame during its stance.
#
# The per-leg targets are built from the NEUTRAL-spine whole-body foot pose so
# that, at spine-neutral, this controller reproduces the hip-frame gait exactly
# (plus the body translation). Turning the spine ON then leaves the world target
# unchanged but moves the girdle, so the IK returns DIFFERENT leg angles -- the
# closed loop.


@dataclass(frozen=True)
class WholeBodyLegState:
    """Kinematic state of one leg at a queried phase, solved in the WORLD frame.

    Attributes
    ----------
    name : str
        Leg label.
    in_stance : bool
        True if the leg is planted (stance), False if swinging.
    local_phase : float
        The leg's own phase in [0, 1) after applying its offset.
    foot_target_world : np.ndarray
        Target foot pose (x, z, phi) in the fixed WORLD / ground frame (m, m,
        rad). For a stance leg this is HELD CONSTANT over the leg's stance.
    foot_target_body : np.ndarray
        The same target expressed in the body-ground frame (spine base at the
        origin), i.e. ``foot_target_world`` minus the body advance. This is the
        pose actually handed to ``WholeBody.inverse``.
    q : np.ndarray | None
        Solved joint angles (q1, q2, q3) in rad, or None if UNREACHABLE.
    reachable : bool
        Whether whole-body IK produced a solution at all.
    within_limits : bool
        Whether the solved q is inside the leg's configured joint limits.
    """

    name: str
    in_stance: bool
    local_phase: float
    foot_target_world: np.ndarray
    foot_target_body: np.ndarray
    q: np.ndarray | None
    reachable: bool
    within_limits: bool

    @property
    def ok(self) -> bool:
        """True only if reachable AND within joint limits."""
        return self.reachable and self.within_limits


@dataclass(frozen=True)
class WholeBodyGaitState:
    """Whole-body kinematic state at a queried phase, solved in the WORLD frame.

    Attributes
    ----------
    phase : float
        Wrapped gait phase in [0, 1).
    legs : dict[str, WholeBodyLegState]
        Per-leg state keyed by leg name.
    spine_q : np.ndarray
        Spine joint angles (rad) at this phase; all-zero unless spine coupling is
        enabled.
    body_offset : float
        Forward advance (m) of the body-ground origin in the world frame at this
        phase (= ``distance_per_cycle * phase``). Added to a leg's body-frame
        target to recover its world target.
    """

    phase: float
    legs: dict[str, WholeBodyLegState]
    spine_q: np.ndarray
    body_offset: float

    @property
    def stance_legs(self) -> tuple[str, ...]:
        return tuple(n for n, s in self.legs.items() if s.in_stance)

    @property
    def swing_legs(self) -> tuple[str, ...]:
        return tuple(n for n, s in self.legs.items() if not s.in_stance)

    @property
    def stance_count(self) -> int:
        return len(self.stance_legs)

    @property
    def all_ok(self) -> bool:
        """True if every leg's pose is reachable and within limits."""
        return all(s.ok for s in self.legs.values())


@dataclass
class WholeBodyGaitController:
    """Walk gait that closes the spine<->foot loop via whole-body IK (M3).

    Same gait definition and phase math as ``GaitController``, but foot targets
    are expressed in the WORLD / ground frame and solved with
    ``WholeBody.inverse`` THROUGH the (possibly bent) spine, so STANCE feet stay
    planted at fixed world positions while the spine oscillates -- the leg joint
    angles compensate. See the section banner above for the two-frame model.

    With the spine held NEUTRAL (``spine_amplitude == 0``) this reproduces the
    hip-frame ``GaitController`` per-leg angles exactly (the world target for a
    leg is its neutral-spine whole-body foot pose, so the residual hip-frame pose
    handed to the leg IK is identical). Turning the spine ON changes the girdle
    poses and hence the solved leg angles, but NOT the world foot targets.

    Assumptions (flagged): 2D sagittal, quasi-static (no dynamics / GRF), massless
    links, point foot contact, NO ZMP / support-polygon margin check -- inherited
    from ``gait``/``spine``/``leg``. Only stance-leg COUNT is a stability proxy.

    Attributes
    ----------
    params : GaitParams
        The gait definition (shared with the hip-frame controller).
    body : WholeBody
        Source of the four ``LegModel``s and the ``SpineModel``.
    """

    params: GaitParams = field(default_factory=GaitParams)
    body: WholeBody = field(default_factory=WholeBody)

    def __post_init__(self) -> None:
        # Reuse the hip-frame controller for phase math + the spine oscillation.
        self._hip = GaitController(params=self.params, body=self.body)
        # Cache each leg's neutral-spine hip world pose (x, z, theta). With the
        # spine neutral the girdle orientation is 0, so these are fixed anchors
        # the world targets are built on.
        n = self.body.spine.params.n_segments
        self._neutral = np.zeros(n)
        self._hip_anchor = {
            name: self.body.hip_world_pose(self._neutral, name)
            for name in self.params.phase_offsets
        }

    # -------------------------------------------------------------- phase math
    def local_phase(self, phase: float, leg_name: str) -> float:
        """The leg's own phase in [0, 1) (global phase minus its offset)."""
        return self._hip.local_phase(phase, leg_name)

    def is_stance(self, phase: float, leg_name: str) -> bool:
        """True if the leg is planted at ``phase`` (half-open [0, duty))."""
        return self._hip.is_stance(phase, leg_name)

    def stance_count(self, phase: float) -> int:
        """Number of planted legs at ``phase``."""
        return self._hip.stance_count(phase)

    def spine_q(self, phase: float) -> np.ndarray:
        """Spine joint angles (rad) at ``phase`` (NEUTRAL unless coupled)."""
        return self._hip.spine_q(phase)

    def body_offset(self, phase: float) -> float:
        """Forward advance (m) of the body-ground origin in the world frame.

        ``distance_per_cycle * phase`` -- the (unwrapped) phase is used so the
        advance is continuous across cycles for time-based queries.
        """
        return self.params.distance_per_cycle * float(phase)

    # -------------------------------------------------------------- targets
    def foot_target_world(self, phase: float, leg_name: str) -> np.ndarray:
        """Foot target pose (x, z, phi) in the fixed WORLD / ground frame.

        Built from the NEUTRAL-spine whole-body foot pose plus the body advance:
        the neutral hip anchor (fixed) + the hip-frame gait trajectory + the
        forward ``body_offset``. For a stance leg the backward hip-frame sweep
        exactly cancels the forward body advance, so this is CONSTANT over the
        leg's stance -- the foot is planted in the world.
        """
        s = self.local_phase(phase, leg_name)
        ax, az, ath = self._hip_anchor[leg_name]  # ath == 0 (neutral spine)
        fx, fz, fphi = foot_target(self.params, s)  # hip-frame pose
        # Neutral girdle orientation is 0, so R = I; add the body advance in +x.
        return np.array(
            [ax + fx + self.body_offset(phase), az + fz, ath + fphi]
        )

    # -------------------------------------------------------------- leg solving
    def leg_state(self, phase: float, leg_name: str) -> WholeBodyLegState:
        """Solve one leg at ``phase`` in the world frame through the spine."""
        s = self.local_phase(phase, leg_name)
        in_stance = s < self.params.duty_factor
        world = self.foot_target_world(phase, leg_name)
        # Express in the body-ground frame (spine base at origin) for the IK.
        offset = self.body_offset(phase)
        body_target = world - np.array([offset, 0.0, 0.0])
        sol: LegIKSolution = self.body.inverse(
            self.spine_q(phase), leg_name, body_target, knee=self.params.knee
        )
        return WholeBodyLegState(
            name=leg_name,
            in_stance=in_stance,
            local_phase=s,
            foot_target_world=world,
            foot_target_body=body_target,
            q=sol.q,
            reachable=sol.reachable,
            within_limits=sol.within_limits,
        )

    def state(self, phase: float) -> WholeBodyGaitState:
        """Whole-body world-frame state at gait ``phase``.

        The gait PATTERN (which legs are in stance, the spine oscillation) uses
        the wrapped phase; the body ADVANCE uses the phase as passed so it stays
        continuous across cycles for time-based queries.
        """
        p_wrapped = float(phase) % 1.0
        legs = {
            name: self.leg_state(phase, name) for name in self.params.phase_offsets
        }
        return WholeBodyGaitState(
            phase=p_wrapped,
            legs=legs,
            spine_q=self.spine_q(phase),
            body_offset=self.body_offset(phase),
        )

    def state_at_time(self, t: float) -> WholeBodyGaitState:
        """Whole-body world-frame state at time ``t`` seconds (phase = t/period)."""
        return self.state(t / self.params.period)

    def foot_world_check(self, phase: float, leg_name: str) -> np.ndarray:
        """FK-verify a leg: world foot pose from the solved q (or NaNs if unsolved).

        Runs ``WholeBody.foot_world_pose`` on the IK solution and adds the body
        advance, so it should reproduce ``foot_target_world`` for a reachable leg.
        Handy for tests/plots that want to confirm the loop actually closed.
        """
        st = self.leg_state(phase, leg_name)
        if st.q is None:
            return np.full(3, np.nan)
        body_pose = self.body.foot_world_pose(self.spine_q(phase), leg_name, st.q)
        return body_pose + np.array([self.body_offset(phase), 0.0, 0.0])
