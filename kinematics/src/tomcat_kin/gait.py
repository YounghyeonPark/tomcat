# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Parameterized periodic WALK gait generation (quasi-static).

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
The default is a statically stable, lateral-sequence walk with **duty factor
0.90** and swing windows spaced 0.25 apart. At most one leg swings at a time, so
at least THREE feet are always planted (the classic crawl condition), and because
0.90 > 0.75 the windows do NOT tile: four FOUR-FOOT windows of 15% of the cycle
each open between them.

Those four-foot windows are not incidental — they are the reason duty is 0.90 and
not the textbook 0.75. The lateral spine sway (below) must change sides, and it
can only do so while all four feet are down. At exactly 0.75 one leg touches down
at the same instant another lifts off, the support side flips discontinuously,
and the sway would have to be instantaneous. See ``GaitParams.duty_factor``.

Default per-leg phase offsets (touchdown phase of each leg):

    LF = 0.00,  RF = 0.25,  RR = 0.50,  LR = 0.75

so the liftoff (swing) sequence around the cycle is RF -> RR -> LR -> LF. (The
biological cat lateral-sequence walk LH->LF->RH->RF is the same family; only the
offset-to-leg labelling differs. Any assignment of the offset SET {0, .25, .5,
.75} keeps the >=3-foot support property, and — a measured result, see
tests/test_stability.py — all 24 assignments give the same lateral margin to
within 2 mm. Sequencing is NOT a lever for lateral stability; sway is.)

Lateral sway (M5, ADR-0009)
---------------------------
Three feet down means a SKEWED support triangle, and a mid-sagittal CoM falls
outside it: the default walk is laterally unstable at -21.6 mm however healthy
its fore-aft margin looks. The fix is the actuated lateral spine bend: hold the
spine at ``lateral_amplitude`` toward the SUPPORT side through each 3-foot phase
and traverse across the four-foot windows (``GaitController.lateral_q``). That
recovers a +10.1 mm polygon margin. See ``GaitParams.lateral_amplitude`` for
why 12.5 deg is an optimum rather than a maximum.

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
- QUASI-STATIC WITH REAL MASS (M4): still no dynamics. No velocities,
  accelerations, inertias or ground-reaction forces are modelled; the trajectory
  is a pure geometric sequence of postures. Link MASS is now real (params.py) and
  is used for gravity/CoM/stability, but the literature's Mass-Mass-Spring result
  (leg mass in flight bending a compliant trunk, and the resulting spine-leg
  elastic energy exchange) still requires full Newton-Euler and is NOT captured.
- MOSTLY SAGITTAL: the LEGS are still solved sagittally (no abduction/roll), and
  left/right legs on a girdle share a mount pose in that 2D projection. Since M5
  the SPINE has a real LATERAL bend DOF (ADR-0009) and the feet sit at their true
  lateral track offsets, so the model is 3D enough to evaluate a genuine support
  POLYGON and to command the sway that keeps the CoM inside it. Yaw and axial
  twist are still absent.
- STABILITY (updated in M5): two margins are reported. ``stability(phase)`` is
  the M4 fore-aft INTERVAL margin (necessary but not sufficient), and
  ``support_polygon(phase)`` is the TRUE ground-plane polygon margin, which can
  see lateral/roll tipping. The polygon margin is what drives the design:
  * with no sway the default walk is LATERALLY UNSTABLE at −21.6 mm, even though
    the fore-aft margin is a comfortable +40.2 mm at every phase;
  * commanding the M5 lateral spine bend recovers it to **+10.1 mm**.
  Remaining limit: still quasi-static — no ZMP, no inertia, no friction cone. The
  +10.1 mm is a STATIC margin with no dynamic allowance, and it is small. The one
  dynamic check that IS made is the sway crossover against the paw friction cone
  (``crossover_accel``), because it decides whether the margin is realisable.
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
from .spine import WholeBody, SpineModel, LegIKSolution, BodyCoM
from .stability import (StabilityMargin, sagittal_stability_margin,
                        SupportPolygon, polygon_stability_margin)


# Default lateral-sequence walk offsets (touchdown phase per leg). The offset SET
# {0, .25, .5, .75} with duty 0.80 keeps at least three feet planted at all times,
# with four-foot windows between the swings for the lateral spine crossover.
DEFAULT_PHASE_OFFSETS: dict[str, float] = {
    "LF": 0.00,
    "RF": 0.25,
    "RR": 0.50,
    "LR": 0.75,
}


# Diagonal-pair TROT offsets (M7). LF+RR strike together, then RF+LR.
TROT_PHASE_OFFSETS: dict[str, float] = {
    "LF": 0.00,
    "RR": 0.00,
    "RF": 0.50,
    "LR": 0.50,
}


def trot_params(period: float = 0.4, stride_length: float = 0.10,
                **overrides) -> "GaitParams":
    """A balanced diagonal TROT — the project's first DYNAMIC gait (M7).

    The crawl defaults are wrong for a trot in three specific ways, and this
    preset fixes all three:

    1. ``phase_offsets`` — diagonal pairs, so the support is a LINE, not a
       polygon. Static-stability checks do not apply; use
       ``dynamics.trot_sweep`` / ``line_balance`` instead of ``support_polygon``.
    2. ``nominal_foot`` x = **0.005**, not the crawl's 0.05. The crawl plants its
       feet 50 mm ahead of the hips, which puts the diagonal support line ~42 mm
       forward of the CoM. That gives a one-signed topple moment, and the roll
       rate then GROWS every cycle (the robot falls over in one stride). At the
       balanced foothold the CoM rocks symmetrically about the line, the moment
       integrates to ~zero, and the roll stays bounded.

       ⚠️ **RE-TUNED 0.005 -> 0.00214 m by M41 ([ADR-0046](../../../docs/DESIGN_DECISIONS.md)).**
       The balance point is a property of where the CoM sits relative to the
       diagonal, so it moved when the manufacturing model replaced the assumed leg
       masses: the legs got heavier (0.110 -> 0.167 kg) AND their mass shifted
       distally (the metatarsus more than doubled), which moves the whole-body CoM.
       At the old 0.005 the drift is **-0.180 rad/s per cycle** — divergent. Found
       by bisection on `_roll_drift`, the same way M7 found the original.
    3. ``step_height`` 0.02, not 0.03 — the fore hip cannot retract far enough to
       lift 30 mm at this foot placement (it overshoots its −170° limit by ~2°).

    ``swing_profile`` stays at the "matched" default, which a trot REQUIRES: on
    the legacy cycloid the foot velocity steps at liftoff and touchdown, making
    swing-leg torque impulsive (9x over-stated) and landing the paw with a
    forward scuff at the full stance speed.

    Defaults give **~50 cm/s**. Faster is possible on a good floor -- feasible and
    thermally sustainable to ~96 cm/s -- but 0.4 s is what meets **NFR15** on a
    realistic **mu = 0.8** floor.

    ⚠️ The period is set by FRICTION, not by the motors. The lateral-spine balance
    assist has to push against the ground to move the CoM, and that demand scales
    as ``1/stance^2``: at 0.3 s (67 cm/s) it needs mu ~ 1.26 on top of the gait's
    own 0.145, which no ordinary floor supplies, and the disturbance envelope falls
    to 40 mm -- under NFR15's 48 mm. At 0.4 s it clears. See ADR-0020.
    """
    kw = dict(
        period=period,
        stride_length=stride_length,
        duty_factor=0.50,
        phase_offsets=dict(TROT_PHASE_OFFSETS),
        # ⚠️ 0.00214, not 0.005 — re-tuned by M41 (ADR-0046) after the measured leg
        # masses moved the CoM. See the docstring; the old value now diverges.
        nominal_foot=(0.00214, -0.17),
        step_height=0.02,
        lateral_amplitude=0.0,     # a trot does not sway; the diagonal does the work
    )
    kw.update(overrides)
    return GaitParams(**kw)


@dataclass(frozen=True)
class GaitParams:
    """Parameters of a periodic walk (sagittal legs + lateral spine sway).

    All lengths are metres, angles radians, times seconds (SI). Defaults are
    ILLUSTRATIVE and tuned to stay inside the placeholder leg joint limits with a
    ~10 deg margin over the whole cycle for the digitigrade 4-link leg (see module
    docstring / demo). The stance holds the PAW-TIP pitch phi at 0 (paw flat on the
    ground): with the ~55 deg passive paw offset this drives the metatarsus steeply
    down (a3 = phi - paw_angle ~= -55 deg), i.e. the digitigrade "hock held high"
    crouch. The binding constraint over the cycle is now the hip (q1) approaching
    its -90 deg limit at liftoff, not the hock -- the widened +/-90 deg hock range
    is no longer what caps stance depth.

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
        Fraction of the cycle each leg spends in stance, in (0, 1). The default
        **0.80** gives a >=3-foot-support walk (at most one leg swinging) PLUS the
        four-foot windows the lateral sway needs to change sides. 0.75 is the
        degenerate tiling case — see the comment on the field itself.
    nominal_foot : tuple[float, float]
        (x, z) of the mid-stride foot position in the hip frame (m). z is
        negative (foot below the hip).
    foot_pitch : float
        Constant PAW-TIP pitch phi (rad) held over the trajectory (the passive
        paw rides at a fixed offset off the metatarsus; see leg.py).
    phase_offsets : Mapping[str, float]
        Per-leg touchdown phase in [0, 1). Keys are leg names ("LF" etc.).
    knee : KneeConfig
        Which 2R IK branch each leg uses. **None (the default) means each leg
        uses its OWN anatomical fold** (`LegModel.default_knee`): the hind leg
        folds its stifle forward (negative range) and the fore leg its elbow
        backward (positive range), exactly as in a cat. Set explicitly only to
        override. The pre-M4 walk forced the positive branch on every leg, which
        cannot place a paw under its own hip and made the gait fore/aft unstable.
    lateral_amplitude : float
        Amplitude (rad, per segment) of the LATERAL spine sway that holds the CoM
        inside the 3-foot support triangle (M5 / ADR-0009). Default 14 deg, which
        is an OPTIMUM, not a maximum. 0.0 disables the sway and reproduces the
        laterally unstable pre-M5 walk.
    spine_amplitude : float
        Amplitude (rad, per segment) of the OPTIONAL dorsoventral spine
        oscillation coupled to the gait. 0.0 (default) => spine held NEUTRAL.
    spine_phase : float
        Phase lead (fraction of a cycle) of the spine oscillation relative to the
        gait phase.
    """

    # 5.0 s, set by TIPPING -- and this is the third time this number has moved.
    # M5 set 1.4 s from a friction hand-calc. The M6 dynamics (dynamics.py) show
    # friction was never the binding constraint: what actually fails first is the
    # ZMP leaving the support polygon. Accelerating the CoM sideways to produce
    # the sway shifts the effective pressure point by (h/g)*a the OTHER way, and
    # at 1.4 s that shift is ~128 mm against a 96 mm track -- the robot tips long
    # before a paw slips (aggregate mu there is only 0.35, well inside any floor).
    # tipping binds first; slipping never binds at all., so the walk is a 1.1 cm/s crawl.
    period: float = 5.0
    stride_length: float = 0.05
    step_height: float = 0.03
    # 0.80, NOT the textbook 0.75 (M5 / ADR-0009). At exactly 0.75 the four swing
    # windows TILE the cycle: one leg touches down at the same instant another
    # lifts off, so the robot is never on four feet. That is fine for the 2D
    # fore-aft margin but fatal in 3D -- the support side flips DISCONTINUOUSLY,
    # and the lateral spine sway that holds the CoM inside the 3-foot triangle
    # would have to flip sides in zero time. A swept study found any finite ramp
    # (even 2% of the cycle) collapses the worst-case polygon margin straight back
    # to the no-sway value of -28.7 mm. Raising duty to 0.80 opens a 5%-of-cycle
    # FOUR-FOOT window at each of the four crossovers, which is when the spine
    # actually traverses. The binding case then moves off the crossover and back
    # onto the 3-foot posture, and the margin stops depending on duty at all.
    #
    # 0.90 rather than the minimum-viable 0.80 because the window must be wide in
    # SECONDS, not just non-zero: 0.80 leaves only 5% of the cycle, forcing either
    # a 9 g lateral CoM slam or a 4.05 s crawl. 0.90 gives 15% and, as a bonus,
    # a slightly BETTER polygon margin (+8.4 vs +7.3 mm).
    duty_factor: float = 0.90
    # Feet planted just ahead of / under the hips in a CROUCHED digitigrade
    # stance: hip-to-paw distance is ~63% of the leg's reach, matching a standing
    # cat (~55-65%) rather than the near-straight 70% of an earlier revision.
    # Crouching is nearly FREE here: with a vertical support force the hip torque
    # depends only on the foot's horizontal offset and the hock torque is fixed by
    # the paw geometry, so deepening the crouch loads ONLY the stifle (0.14 ->
    # 0.34 N.m at trot) -- far below the binding hip/hock. Stability margin is
    # unchanged; ~10 deg of joint-limit margin remains over the cycle.
    # Pre-M4 this was (0.20, -0.13): the positive-knee branch could not reach
    # under the hip, so every paw landed ~0.2 m forward and the support interval
    # sat entirely ahead of the trunk (statically unstable).
    nominal_foot: tuple[float, float] = (0.05, -0.17)
    foot_pitch: float = 0.0
    phase_offsets: Mapping[str, float] = field(
        default_factory=lambda: dict(DEFAULT_PHASE_OFFSETS)
    )
    knee: KneeConfig | None = None
    # LATERAL spine sway per segment (rad), ADR-0009: bend the spine toward the
    # SUPPORT side while a leg swings, which is how a real cat keeps its CoM
    # inside the 3-foot support triangle. Set to 0.0 to reproduce the pre-ADR-0009
    # (laterally unstable, -28.7 mm) behaviour.
    #
    # 12.5 deg is an OPTIMUM, not a maximum. Margin rises to +10.1 mm at 12.5 deg
    # then FALLS again: over-swaying carries the CoM out over the FAR edge of the
    # triangle. Above ~14 deg it is not merely worse but INFEASIBLE -- the sway
    # reversal exceeds what the paw friction cone can deliver (crossover_accel).
    # It sits well inside the +/-15 deg
    # per-segment ROM, so the ROM is adequate but has ~1 deg to spare -- the
    # lateral DOF is sized almost exactly right, with no slack for error.
    lateral_amplitude: float = math.radians(11.0)
    # Swing-return profile: "matched" (default, C1/C2 -- end velocities equal the
    # stance sweep so the foot lands without scuffing and the transition carries
    # no acceleration impulse) or "cycloid" (the legacy M2 profile, C0 only).
    # See ``foot_target``. The M7 dynamics showed the cycloid cannot support a
    # trot: its velocity step makes swing-leg torque impulsive.
    swing_profile: str = "matched"
    spine_amplitude: float = 0.0
    spine_phase: float = 0.0

    def __post_init__(self) -> None:
        if not 0.0 < self.duty_factor < 1.0:
            raise ValueError(f"duty_factor must be in (0, 1); got {self.duty_factor}")
        if self.period <= 0.0:
            raise ValueError(f"period must be > 0; got {self.period}")
        if self.swing_profile not in ("matched", "cycloid"):
            raise ValueError(
                f"swing_profile must be 'matched' or 'cycloid'; got {self.swing_profile!r}"
            )
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
    com : BodyCoM | None
        Whole-body + sub-assembly centres of mass at this posture, in the
        BODY-GROUND frame (M4). ``None`` only if constructed by hand.
    stability : StabilityMargin | None
        Fore-aft STATIC stability margin at this phase: the CoM ground
        projection against the interval spanned by the STANCE feet (M4). See
        ``stability.py`` — a 2D sagittal support "polygon" is a fore-aft
        interval, so a positive margin is necessary but not sufficient.
    """

    phase: float
    legs: dict[str, LegState]
    spine_q: np.ndarray
    com: BodyCoM | None = None
    stability: StabilityMargin | None = None

    @property
    def is_statically_stable(self) -> bool:
        """True if the CoM projects inside the fore-aft support interval."""
        return self.stability is not None and self.stability.is_stable

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
        # Swing: forward-and-up return.
        v = (s - d) / (1.0 - d)
        if params.swing_profile == "cycloid":
            # LEGACY C0 profile. Starts and ends at ZERO hip-frame velocity while
            # stance sweeps at -stride/(duty*period), so the foot velocity STEPS at
            # both liftoff and touchdown. Two consequences the M7 dynamics exposed:
            # the implied acceleration is impulsive (so swing-leg torque is not
            # computable), and the foot touches down moving FORWARD at the stance
            # speed, i.e. it scuffs. Harmless at the 5 s crawl, fatal in a trot.
            x = x_nom - half + params.stride_length * (
                v - math.sin(2.0 * math.pi * v) / (2.0 * math.pi)
            )
        else:
            # MATCHED C1 profile (default). Quintic Hermite whose end velocities
            # equal the stance sweep, so the foot is already travelling backward at
            # the stance rate when it touches down -- zero velocity relative to the
            # GROUND, no scuff -- and the transition carries no acceleration
            # impulse. Zero end acceleration too, so it is C2 at the joins.
            #
            #   x(v) = x0 + vs*v + (stride - vs) * S(v),  S = 10v^3 - 15v^4 + 6v^5
            #
            # where vs is the stance sweep expressed over the swing duration. The
            # price is real: the foot continues BACKWARD briefly before swinging
            # forward, so it travels further and faster than a cycloid would.
            vs = -params.stride_length * (1.0 - d) / d
            S = v * v * v * (10.0 - 15.0 * v + 6.0 * v * v)
            x = x_nom - half + vs * v + (params.stride_length - vs) * S
        z = z_nom + params.step_height * (1.0 - math.cos(2.0 * math.pi * v)) / 2.0
    return np.array([x, z, params.foot_pitch])


def swing_height(params: GaitParams, local_phase: float) -> float:
    """Foot lift above the nominal ground level (m) for a leg's local phase.

    0 during stance and at swing liftoff/touchdown; peaks at ``step_height``.
    """
    return float(foot_target(params, local_phase)[1] - params.nominal_foot[1])


# ------------------------------------------------------------------- stability
def _posture_stability(
    body: WholeBody, spine_q, leg_states
) -> tuple[BodyCoM, StabilityMargin]:
    """CoM + fore-aft static stability margin for one solved gait posture.

    ``leg_states`` maps leg name -> any object exposing ``.q`` (the solved joint
    vector, or None) and ``.in_stance``. Works for both ``LegState`` (hip-frame
    controller) and ``WholeBodyLegState`` (world-frame controller).

    Both the CoM and the stance-foot positions are evaluated by FORWARD
    kinematics from the SAME solved joint angles, in the BODY-GROUND frame
    (spine base at the origin). The margin is translation-invariant, so it is
    identical whether you work in the body-ground or the advancing world frame.

    A leg whose IK FAILED (``q is None``) is placed at all-zero joint angles so
    its mass is still counted (mass must be conserved) and is EXCLUDED from the
    support interval even if nominally in stance -- an unsolvable leg is not a
    trustworthy contact. The default gait solves every leg, so this only bites on
    deliberately broken configurations.
    """
    leg_q = {
        name: (np.zeros(3) if st.q is None else np.asarray(st.q, dtype=float))
        for name, st in leg_states.items()
    }
    com = body.center_of_mass(spine_q, leg_q)
    feet = {
        name: float(body.foot_world_position(spine_q, name, leg_q[name])[0])
        for name, st in leg_states.items()
        if st.in_stance and st.q is not None
    }
    return com, sagittal_stability_margin(com.x, feet)


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
        fresh ``WholeBody`` (default spine + the asymmetric fore/hind legs: FORE
        model on the front girdle, HIND on the rear).
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
        # Asymmetric: FORE model for front-girdle legs, HIND for rear (WholeBody
        # picks per girdle). The same foot target thus solves to different joint
        # angles on a front vs. a rear leg.
        leg = self.body.leg_model_for(leg_name)
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
        """Whole-body state at gait ``phase`` (phase wraps at 1.0).

        Also evaluates the M4 whole-body CoM and the fore-aft static stability
        margin for the solved posture (``.com`` / ``.stability``).
        """
        p = float(phase) % 1.0
        legs = {name: self.leg_state(p, name) for name in self.params.phase_offsets}
        sq = self.spine_q(p)
        com, margin = _posture_stability(self.body, sq, legs)
        return GaitState(
            phase=p, legs=legs, spine_q=sq, com=com, stability=margin
        )

    # ------------------------------------------------------------- mass (M4)
    def center_of_mass(self, phase: float) -> BodyCoM:
        """Whole-body CoM (body-ground frame) at ``phase``, from the solved pose."""
        return self.state(phase).com

    def stability(self, phase: float) -> StabilityMargin:
        """Fore-aft static stability margin at ``phase`` (see ``stability.py``)."""
        return self.state(phase).stability

    def stability_sweep(self, n: int = 48) -> list[StabilityMargin]:
        """``n`` evenly spaced stability margins over one cycle, phase in [0, 1)."""
        return [self.stability(i / n) for i in range(n)]

    # ------------------------------------------------------- lateral sway (M5)
    def _events(self) -> list[float]:
        """Sorted phases at which the STANCE SET changes (liftoffs + touchdowns).

        A leg with offset ``o`` touches down at ``o`` and lifts off at
        ``o + duty`` (both mod 1). Between two consecutive events the set of
        planted feet is constant, so these are the exact breakpoints of the
        sway law -- no numerical scanning, no missed windows.
        """
        d = self.params.duty_factor
        ev = set()
        for o in self.params.phase_offsets.values():
            ev.add(o % 1.0)
            ev.add((o + d) % 1.0)
        return sorted(ev)

    def support_side(self, phase: float) -> float:
        """Which side the sway should favour at ``phase``: +1 (left), -1, or 0.

        Returns the side AWAY from the swinging leg(s) -- i.e. toward the support
        triangle. Returns 0.0 when all four feet are planted, which is the window
        the spine uses to cross over.
        """
        swinging = [n for n in self.params.phase_offsets
                    if not self.is_stance(phase, n)]
        if not swinging:
            return 0.0
        ys = sum(self.body.mounts[n].track_y for n in swinging)
        return -float(np.sign(ys)) if ys else 0.0

    def lateral_q(self, phase: float) -> np.ndarray:
        """Commanded LATERAL spine bend at ``phase`` (rad per segment, ADR-0009).

        The law is a RAMPED square wave, not a sinusoid and not a step:

        - while one leg swings (3-foot support) the spine is held at full
          ``lateral_amplitude`` toward the support side;
        - while all four feet are down it traverses LINEARLY to the next side.

        A sinusoid was tested and is *worse than no sway at all* at every phase
        lead: it is near zero exactly at the crossovers, which is where the
        margin is decided, so the worst case never improves on -28.7 mm. The
        traverse must therefore be confined to the four-foot windows, which is
        what ``duty_factor = 0.80`` exists to provide.

        The result is clipped to the spine's lateral joint limits.
        """
        n = self.body.spine.params.n_segments
        amp = float(self.params.lateral_amplitude)
        if amp == 0.0:
            return np.zeros(n)

        p = float(phase) % 1.0
        side = self.support_side(p)
        if side == 0.0:
            # Four feet down: traverse from the side held before this window to
            # the side demanded after it, on a RAISED-COSINE (C1) profile.
            #
            # NOT linear. A linear position ramp has piecewise-constant velocity,
            # so velocity STEPS at each end of the window -- an impulse in
            # acceleration, i.e. infinite force. The M6 dynamics caught this
            # (kinematics/dynamics.py): the static checks never could, because a
            # static check never differentiates the trajectory. The raised cosine
            # starts and ends at zero velocity, so the force stays finite.
            #
            # Cost: peak acceleration is pi^2/2 = 4.93 d/w^2 rather than the
            # bang-bang 4 d/w^2 -- 23% worse, and ``crossover_accel`` reflects it.
            ev = self._events()
            start = max((e for e in ev if e <= p), default=ev[-1] - 1.0)
            end = min((e for e in ev if e > p), default=ev[0] + 1.0)
            prev = self.support_side((start - 1e-9) % 1.0)
            nxt = self.support_side((end + 1e-9) % 1.0)
            u = (p - start) / (end - start) if end > start else 0.0
            smooth = 0.5 * (1.0 - math.cos(math.pi * u))
            side = prev + (nxt - prev) * smooth

        sp = self.body.spine.params
        return np.clip(np.full(n, amp * side),
                       np.asarray(sp.lateral_q_min, dtype=float),
                       np.asarray(sp.lateral_q_max, dtype=float))

    def lateral_slew_rate(self) -> float:
        """Peak commanded lateral spine rate (rad/s per segment) for this gait.

        The traverse of ``2 * lateral_amplitude`` across the narrowest four-foot
        window. This is a hard requirement on the spine tendon drives: if they
        cannot slew this fast, the walk is NOT statically stable, however good
        the geometry looks. Returns ``inf`` when there is no four-foot window
        (duty <= 0.75), which is the degenerate case the 0.80 default avoids.
        """
        amp = float(self.params.lateral_amplitude)
        if amp == 0.0:
            return 0.0
        ev = self._events()
        widths = []
        for a, b in zip(ev, ev[1:] + [ev[0] + 1.0]):
            mid = (a + b) / 2.0
            if self.support_side(mid % 1.0) == 0.0:
                widths.append(b - a)
        if not widths:
            return math.inf
        return 2.0 * amp / (min(widths) * self.params.period)

    def crossover_window(self) -> float:
        """Duration (s) of the narrowest four-foot window — the sway's time budget."""
        ev = self._events()
        widths = [b - a for a, b in zip(ev, ev[1:] + [ev[0] + 1.0])
                  if self.support_side(((a + b) / 2.0) % 1.0) == 0.0]
        return min(widths) * self.params.period if widths else 0.0

    def crossover_accel(self) -> float:
        """Peak LATERAL CoM acceleration (m/s^2) demanded by the sway reversal.

        ⚠️ This is a HAND CHECK that steps outside the module's quasi-static
        model, and it is the number that actually constrains walk speed.

        The sway must reverse the CoM across ``2 * sway_amplitude`` within one
        four-foot window. On the RAISED-COSINE profile ``lateral_q`` actually
        commands, the peak costs ``a = (pi**2 / 2) * d / w**2`` — which grows as
        the INVERSE SQUARE of the window, so walking faster is punished hard.
        (An earlier revision quoted the bang-bang ``4 d / w**2``. That profile is
        cheaper but demands a velocity discontinuity at each end of the ramp;
        the physically realisable smooth profile costs 23 % more.) The paws can only deliver ``mu * g``
        laterally before they slide (~7.8 m/s^2 at mu = 0.8), and exceeding it
        means the static margin computed elsewhere in this module is not
        physically realisable. Compare against ``friction_accel_limit``.

        Returns 0.0 when the sway is disabled, ``inf`` when there is no window.
        """
        amp = float(self.params.lateral_amplitude)
        if amp == 0.0:
            return 0.0
        w = self.crossover_window()
        if w <= 0.0:
            return math.inf
        n = self.body.spine.params.n_segments
        d = 2.0 * abs(self.body.center_of_mass_y(np.full(n, amp)))
        return (math.pi ** 2 / 2.0) * d / (w * w)

    @staticmethod
    def friction_accel_limit(mu: float = 0.8, g: float = 9.81) -> float:
        """Lateral acceleration (m/s^2) a paw can deliver before sliding: ``mu*g``.

        Independent of mass — the friction force and the inertia both scale with
        it. ``mu = 0.8`` is a nominal rubber-on-hard-floor value ``[assumed]``.
        """
        return mu * g

    def crossover_is_feasible(self, mu: float = 0.8) -> bool:
        """Whether the commanded sway reversal stays inside the friction cone."""
        return self.crossover_accel() <= self.friction_accel_limit(mu)

    def support_polygon(self, phase: float, lateral_shift: float | None = None):
        """TRUE ground-plane support-polygon margin at ``phase`` (3D geometry).

        ``stability()`` above returns the 2D-sagittal fore-aft INTERVAL margin,
        which every prior document flagged as necessary but NOT sufficient. This
        uses the real polygon spanned by the stance feet at their lateral track
        offsets, so it can see roll/diagonal tipping.

        The CoM's lateral position comes from the **commanded lateral spine
        bend** (``GaitParams.lateral_amplitude``, ADR-0009) via
        ``WholeBody.center_of_mass_y`` — i.e. it is produced by a real actuated
        DOF, not assumed. Pass ``lateral_shift`` to override it with a raw
        offset (metres) for what-if studies.
        """
        st = self.state(phase)
        xy = self.body.foot_ground_xy(st.spine_q, {n: l.q for n, l in st.legs.items()})
        stance = {n: xy[n] for n in st.stance_legs}
        if lateral_shift is None:
            cy = self.body.center_of_mass_y(self.lateral_q(phase))
        else:
            side = float(np.sign(np.mean([stance[n][1] for n in stance])))
            cy = lateral_shift * side
        return polygon_stability_margin((st.com.x, cy), stance)

    def support_polygon_sweep(self, n: int = 48, lateral_shift: float | None = None):
        """``n`` evenly spaced TRUE polygon margins over one cycle."""
        return [self.support_polygon(i / n, lateral_shift) for i in range(n)]

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
    com : BodyCoM | None
        Whole-body + sub-assembly CoMs (M4) in the BODY-GROUND frame; add
        ``body_offset`` to x for the world frame.
    stability : StabilityMargin | None
        Fore-aft STATIC stability margin (M4), computed in the body-ground frame
        (it is translation-invariant, so it equals the world-frame value).
    """

    phase: float
    legs: dict[str, WholeBodyLegState]
    spine_q: np.ndarray
    body_offset: float
    com: BodyCoM | None = None
    stability: StabilityMargin | None = None

    @property
    def is_statically_stable(self) -> bool:
        """True if the CoM projects inside the fore-aft support interval."""
        return self.stability is not None and self.stability.is_stable

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
        sq = self.spine_q(phase)
        com, margin = _posture_stability(self.body, sq, legs)
        return WholeBodyGaitState(
            phase=p_wrapped,
            legs=legs,
            spine_q=sq,
            body_offset=self.body_offset(phase),
            com=com,
            stability=margin,
        )

    def state_at_time(self, t: float) -> WholeBodyGaitState:
        """Whole-body world-frame state at time ``t`` seconds (phase = t/period)."""
        return self.state(t / self.params.period)

    # ------------------------------------------------------------- mass (M4)
    def center_of_mass(self, phase: float) -> BodyCoM:
        """Whole-body CoM (body-ground frame) at ``phase``, from the solved pose."""
        return self.state(phase).com

    def stability(self, phase: float) -> StabilityMargin:
        """Fore-aft static stability margin at ``phase`` (see ``stability.py``).

        Computed in the body-ground frame; the margin is translation-invariant so
        it is the same number in the advancing world frame.
        """
        return self.state(phase).stability

    def stability_sweep(self, n: int = 48) -> list[StabilityMargin]:
        """``n`` evenly spaced stability margins over one cycle, phase in [0, 1)."""
        return [self.stability(i / n) for i in range(n)]

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
