"""Articulated tendon-driven spine + whole-body kinematics (sagittal plane).

This module treats the torso as a SERIAL revolute chain (ADR-0006, principle P2)
in the same 2D sagittal plane as the leg model: x forward, z up. The spine
positions and orients the base each leg hangs from, so the legs no longer swing
from a fixed frame — they hang off a moving, curving body ("whole-body
kinematics").

Frames & conventions
--------------------
- Sagittal plane only: x forward, z up. Lateral bending and axial rotation are
  out of this plane and are NOT modelled (see SpineParams for the 2D-scope note).
- The spine chain is indexed base/rear -> front. Vertebra 0 sits at the base
  (the REAR / pelvic girdle). Traversing the N segments reaches vertebra N (the
  FRONT / shoulder girdle). Anatomical "front" is +x.
- A pose is (x, z, theta): position (m) and frame orientation (rad), theta
  measured CCW from +x.
- Joint angles q = (q_1 .. q_N) are RELATIVE angles. The cumulative direction of
  segment i is theta_base + q_1 + ... + q_i. Positive q rotates CCW (from +x
  toward +z), lifting the outboard spine dorsally; a uniform positive bend curls
  the chain upward into an arched / dorsiflexed ("Halloween cat") posture. This
  matches the leg model's sign convention exactly.

Girdle -> leg composition
-------------------------
Each girdle is a moving frame. A leg's hip origin is placed at a fixed offset in
its girdle frame; the standalone LegModel then runs in that hip frame, and its
foot position is rotated/translated into the world by the girdle pose. Front
legs (shoulder pair) hang off the front girdle, rear legs (pelvic pair) off the
rear girdle.

Whole-body inverse kinematics (M3)
----------------------------------
``WholeBody.inverse`` runs that composition BACKWARDS: given a foot pose in the
WORLD frame and the spine angles, it undoes the girdle/hip transform and solves
the per-leg IK, so a foot can be commanded to a fixed world position and the leg
angles absorb the spine bend (feet placed "through" the moving spine). See its
docstring for the frame chain and the 2D transform math.

Modelling assumptions / limitations
-----------------------------------
- 2D sagittal only: the four legs' left/right lateral (y) offset is out of plane
  and ignored, so in this projection the two front legs share a mount pose (and
  likewise the two rear legs). This is a real limitation for anything involving
  roll, yaw, lateral bend, or the righting-reflex twist.
- Rigid links and a frictionless idealisation are inherited from the leg/tendon
  models.
- MASS (M4): the kinematics themselves are mass-free, but the body now carries a
  real distributed mass budget (params.py) and ``WholeBody.center_of_mass``
  reports the posed whole-body / sub-assembly CoMs. That is QUASI-STATIC ONLY —
  gravity and CoM geometry, no velocities, accelerations or inertia tensors. The
  literature's Mass-Mass-Spring result (leg mass bends a compliant trunk) is
  therefore still NOT captured: that needs the dynamics milestone.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Mapping

import numpy as np

from .params import (
    SpineParams,
    LegParams,
    DEFAULT_SPINE,
    DEFAULT_LEG,
    DEFAULT_FORELEG,
    DEFAULT_HINDLEG,
)
from .leg import LegModel, KneeConfig, UnreachableError
from .mass import (
    ComResult,
    QuarterMasses,
    combine,
    leg_com,
    quarter_masses,
    spine_chain_com,
)


def _rot(theta: float) -> np.ndarray:
    """2x2 rotation matrix for a CCW rotation by theta (rad)."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


class Girdle(Enum):
    """The two limb-bearing girdles the spine spans."""

    FRONT = "front"  # shoulder girdle (front end of the chain, +x)
    REAR = "rear"    # pelvic girdle (base / rear end of the chain)


@dataclass
class SpineModel:
    """Forward kinematics for the serial tendon-driven spine.

    Attributes
    ----------
    params : SpineParams
        Segment lengths, limits and tendon parameters.
    base_pose : tuple[float, float, float]
        World pose (x, z, theta) of the base / rear end (vertebra 0). Defaults to
        the origin pointing +x, i.e. a straight spine lies along +x.
    """

    params: SpineParams = DEFAULT_SPINE
    base_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)

    # ------------------------------------------------------------------ FK
    def vertebra_poses(self, q) -> np.ndarray:
        """(N+1, 3) array of vertebra poses (x, z, theta), base -> front.

        Row 0 is the base (rear girdle) frame; row N is the front girdle frame.
        """
        q = np.asarray(q, dtype=float)
        if q.shape != (self.params.n_segments,):
            raise ValueError(
                f"spine q has shape {q.shape}; expected ({self.params.n_segments},)"
            )
        x0, z0, th0 = (float(v) for v in self.base_pose)
        lengths = self.params.segment_lengths

        poses = [(x0, z0, th0)]
        x, z, ang = x0, z0, th0
        for i in range(self.params.n_segments):
            ang = ang + q[i]
            x = x + lengths[i] * math.cos(ang)
            z = z + lengths[i] * math.sin(ang)
            poses.append((x, z, ang))
        return np.array(poses)

    def lateral_vertebra_xy(self, lateral_q) -> np.ndarray:
        """(n+1, 2) vertebra (x, y) for a LATERAL (yaw) spine posture — ADR-0009.

        The lateral bend is the same planar serial chain as the sagittal one, but
        in the horizontal x-y plane: joint i turns the remaining chain by
        ``lateral_q[i]`` about the vertical axis. Rear girdle at the origin.

        This is what gives the body a real lateral CoM offset — the SWAY that
        review F7 showed static stability requires.
        """
        q = np.asarray(lateral_q, dtype=float)
        if q.shape != (self.params.n_segments,):
            raise ValueError(
                f"lateral_q must have {self.params.n_segments} entries, got {q.shape}"
            )
        th = 0.0
        x = y = 0.0
        out = [(0.0, 0.0)]
        for qi, L in zip(q, self.params.segment_lengths):
            th += float(qi)
            x += L * math.cos(th)
            y += L * math.sin(th)
            out.append((x, y))
        return np.array(out)

    def lateral_segment_com_y(self, lateral_q) -> np.ndarray:
        """(n,) lateral y of each spine segment's CoM, using `segment_com_frac`."""
        pts = self.lateral_vertebra_xy(lateral_q)
        f = np.asarray(self.params.segment_com_frac, dtype=float)
        return pts[:-1, 1] + f * (pts[1:, 1] - pts[:-1, 1])

    def vertebra_positions(self, q) -> np.ndarray:
        """(N+1, 2) array of vertebra XY positions, base -> front."""
        return self.vertebra_poses(q)[:, :2]

    def girdle_pose(self, q, girdle: Girdle) -> np.ndarray:
        """World pose (x, z, theta) of a girdle mount frame."""
        poses = self.vertebra_poses(q)
        if girdle is Girdle.REAR:
            return poses[0]
        return poses[-1]

    def girdle_poses(self, q) -> dict[Girdle, np.ndarray]:
        """Both girdle mount poses as {Girdle: (x, z, theta)}."""
        poses = self.vertebra_poses(q)
        return {Girdle.REAR: poses[0], Girdle.FRONT: poses[-1]}

    def hip_origin_world(
        self,
        q,
        girdle: Girdle,
        hip_offset=(0.0, 0.0),
    ) -> np.ndarray:
        """World pose (x, z, theta) of a leg's hip origin.

        The hip sits at `hip_offset` (x, z, in the girdle frame) relative to the
        girdle mount; the hip frame shares the girdle's orientation. This is the
        transform that lets a LegModel run in its (moving) hip frame while the
        spine decides where that frame is in the world.
        """
        gx, gz, gth = self.girdle_pose(q, girdle)
        offset = _rot(gth) @ np.asarray(hip_offset, dtype=float)
        return np.array([gx + offset[0], gz + offset[1], gth])

    def in_limits(self, q) -> bool:
        """True if every spine joint angle is within its configured limit."""
        return all(
            lo <= float(v) <= hi
            for v, lo, hi in zip(q, self.params.q_min, self.params.q_max)
        )


@dataclass(frozen=True)
class LegMount:
    """Placement of one leg on the body.

    name       : label, e.g. "LF" (left-front), "RR" (right-rear).
    girdle     : which girdle the leg hangs from.
    hip_offset : (x, z) of the hip origin in the girdle frame (m). The leg's
                 SOLVE is still planar-sagittal, so this stays 2D.
    track_y    : lateral (y) offset of the leg plane from the mid-sagittal plane
                 (m). Left legs positive, right negative. This does NOT add a
                 degree of freedom -- the leg does not move laterally -- it just
                 records where the leg plane actually is, which is enough to
                 build a real ground-plane SUPPORT POLYGON (see stability.py).
                 3D geometry, not 3D actuation: it costs no motors (ADR-0008).
    """

    name: str
    girdle: Girdle
    hip_offset: tuple[float, float] = (0.0, 0.0)
    track_y: float = 0.0


# Default four-leg layout: shoulder pair on the front girdle, pelvic pair on the
# rear girdle. Hip offsets are (0, 0) placeholders (hips at the girdle mounts).
# Half-track 0.048 m matches the 96 mm leg track used by the CAD.  ❓ TBD
TRACK_HALF = 0.048

DEFAULT_MOUNTS: tuple[LegMount, ...] = (
    LegMount("LF", Girdle.FRONT, track_y=+TRACK_HALF),
    LegMount("RF", Girdle.FRONT, track_y=-TRACK_HALF),
    LegMount("LR", Girdle.REAR, track_y=+TRACK_HALF),
    LegMount("RR", Girdle.REAR, track_y=-TRACK_HALF),
)


@dataclass(frozen=True)
class LegIKSolution:
    """Result of a single whole-body inverse-kinematics solve for one leg.

    Attributes
    ----------
    name : str
        Leg label.
    q : np.ndarray | None
        Solved joint angles (q1, q2, q3) in rad, or None if the WORLD foot pose
        was UNREACHABLE (per-leg IK failed).
    reachable : bool
        Whether per-leg IK produced a solution at all.
    within_limits : bool
        Whether the solved q lies inside the leg's configured joint limits.
        False when unreachable.
    foot_world : np.ndarray
        The requested foot pose (x, z, phi) in the WORLD (body-ground) frame.
    foot_hip : np.ndarray
        That same target expressed in the leg's hip frame (x, z, phi) after
        undoing the girdle/hip transform -- the pose handed to LegModel.inverse.
    """

    name: str
    q: np.ndarray | None
    reachable: bool
    within_limits: bool
    foot_world: np.ndarray
    foot_hip: np.ndarray

    @property
    def ok(self) -> bool:
        """True only if reachable AND within joint limits."""
        return self.reachable and self.within_limits


@dataclass(frozen=True)
class BodyCoM:
    """Whole-body and per-sub-assembly centres of mass at one posture (M4).

    Every CoM is in the BODY-GROUND frame: the spine base (vertebra 0 = the rear
    / pelvic girdle) at the origin, x forward, z up. Because gravity acts along
    -z, the "ground projection" used by the stability check is simply ``.x``.

    QUASI-STATIC: this is a gravity/geometry quantity only -- no velocities,
    accelerations or inertia tensors (full Newton-Euler is a later milestone).

    Attributes
    ----------
    total : ComResult
        Whole-body mass and CoM.
    legs : dict[str, ComResult]
        Per-leg mass and CoM, keyed by leg name, already transformed out of the
        hip frame into the body frame (so a spine bend moves the front legs).
    spine : ComResult
        The spine SEGMENTS (trunk chain) alone -- girdle masses excluded.
    girdles : dict[Girdle, ComResult]
        The two girdle lumps (the front one also carries the head/neck mass; see
        params.py).
    """

    total: ComResult
    legs: dict[str, ComResult]
    spine: ComResult
    girdles: dict[Girdle, ComResult]

    @property
    def mass(self) -> float:
        """Whole-body mass (kg)."""
        return self.total.mass

    @property
    def com(self) -> np.ndarray:
        """Whole-body CoM (x, z) in the body-ground frame (m)."""
        return self.total.com

    @property
    def x(self) -> float:
        """Whole-body CoM ground projection (m)."""
        return self.total.x

    @property
    def z(self) -> float:
        """Whole-body CoM height above the spine base (m)."""
        return self.total.z

    def legs_com(self) -> ComResult:
        """Combined CoM of all four legs."""
        return combine(self.legs.values())

    def report(self) -> str:
        lines = [
            f"whole body : {self.mass:.3f} kg at "
            f"(x {self.x * 1e3:+7.1f}, z {self.z * 1e3:+7.1f}) mm",
            f"  spine chain : {self.spine.mass:.3f} kg at "
            f"({self.spine.x * 1e3:+7.1f}, {self.spine.z * 1e3:+7.1f}) mm",
        ]
        for g in (Girdle.REAR, Girdle.FRONT):
            c = self.girdles[g]
            lines.append(
                f"  {g.value + ' girdle':<12}: {c.mass:.3f} kg at "
                f"({c.x * 1e3:+7.1f}, {c.z * 1e3:+7.1f}) mm"
            )
        for name in sorted(self.legs):
            c = self.legs[name]
            lines.append(
                f"  leg {name:<8}: {c.mass:.3f} kg at "
                f"({c.x * 1e3:+7.1f}, {c.z * 1e3:+7.1f}) mm"
            )
        return "\n".join(lines)


@dataclass
class WholeBody:
    """Composition of the spine and four legs into one kinematic body.

    The spine sets each girdle pose; each leg then runs in its hip frame and its
    foot is expressed in the world. Call with the spine joint vector plus a
    per-leg joint vector to get every foot's world position.

    Fore/hind ASYMMETRY (cats are NOT fore/hind symmetric; see params.py)
    --------------------------------------------------------------------
    Front-girdle legs (shoulder pair, e.g. LF/RF) use ``fore_leg`` -- the more
    columnar limb (longer humerus, shorter metacarpus, smaller ~40 deg paw
    toe-break, ``DEFAULT_FORELEG``). Rear-girdle legs (pelvic pair, e.g. LR/RR)
    use ``hind_leg`` -- the folded propulsion limb (longer shank, ~55 deg
    toe-break, ``DEFAULT_HINDLEG`` = ``DEFAULT_LEG``). Both models expose the same
    3-joint actuated API; only the link proportions and paw offset differ, so the
    SAME relative foot target lands at DIFFERENT joint angles on a front vs. rear
    leg. Everything that touches leg kinematics (``foot_world_pose``, ``inverse``,
    ``inverse_pose``, ``foot_positions``, gait) picks the correct model per leg
    via the ``self.legs`` dict / ``leg_model_for``.

    Attributes
    ----------
    fore_leg, hind_leg : LegModel
        The front- and rear-girdle leg models. Constructor-overridable (tests may
        inject their own) but default to ``LegModel(DEFAULT_FORELEG)`` /
        ``LegModel(DEFAULT_HINDLEG)``.
    legs : dict[str, LegModel]
        Per-leg model, keyed by leg name. Built in ``__post_init__`` from
        ``fore_leg`` / ``hind_leg`` by each mount's girdle unless explicitly
        supplied (an explicit dict wins, e.g. for per-leg overrides in a test).
    """

    spine: SpineModel = field(default_factory=SpineModel)
    legs: dict[str, LegModel] = field(default_factory=dict)
    mounts: dict[str, LegMount] = field(default_factory=dict)
    fore_leg: LegModel = field(default_factory=lambda: LegModel(DEFAULT_FORELEG))
    hind_leg: LegModel = field(default_factory=lambda: LegModel(DEFAULT_HINDLEG))

    def __post_init__(self) -> None:
        if not self.mounts:
            self.mounts = {m.name: m for m in DEFAULT_MOUNTS}
        if not self.legs:
            # Asymmetric: front-girdle legs get the FORE model, rear-girdle legs
            # the HIND model (cats are not fore/hind symmetric; see params.py).
            self.legs = {
                name: (self.fore_leg if m.girdle is Girdle.FRONT else self.hind_leg)
                for name, m in self.mounts.items()
            }

    @property
    def leg_names(self) -> tuple[str, ...]:
        return tuple(self.mounts.keys())

    def leg_model_for(self, name: str) -> LegModel:
        """The ``LegModel`` for one leg (FORE for front girdle, HIND for rear)."""
        return self.legs[name]

    def hip_world_pose(self, spine_q, leg_name: str) -> np.ndarray:
        """World pose (x, z, theta) of a leg's hip origin given the spine state."""
        mount = self.mounts[leg_name]
        return self.spine.hip_origin_world(spine_q, mount.girdle, mount.hip_offset)

    def foot_world_position(self, spine_q, leg_name: str, leg_q) -> np.ndarray:
        """World (x, z) of one foot given spine angles and that leg's joint angles."""
        hx, hz, hth = self.hip_world_pose(spine_q, leg_name)
        foot_hip = self.leg_model_for(leg_name).forward(leg_q)[:2]  # (x, z) hip frame
        world = np.array([hx, hz]) + _rot(hth) @ foot_hip
        return world

    def foot_world_pose(self, spine_q, leg_name: str, leg_q) -> np.ndarray:
        """World foot POSE (x, z, phi) given spine angles and that leg's joints.

        Extends ``foot_world_position`` with the foot pitch. The leg's foot pitch
        ``phi_hip`` is measured in the (rotated) hip frame; the girdle the hip
        rides on is itself tilted by ``hth`` (the girdle orientation set by the
        spine), so the WORLD foot pitch is ``hth + phi_hip``. This is the exact
        forward map that ``inverse`` inverts, so ``foot_world_pose(spine_q, leg,
        inverse(spine_q, leg, target).q) == target`` for a reachable target.
        """
        hx, hz, hth = self.hip_world_pose(spine_q, leg_name)
        fx, fz, phi_hip = self.leg_model_for(leg_name).forward(leg_q)
        world_xz = np.array([hx, hz]) + _rot(hth) @ np.array([fx, fz])
        return np.array([world_xz[0], world_xz[1], hth + phi_hip])

    def foot_ground_xy(self, spine_q, leg_q: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        """World GROUND-PLANE (x, y) of every foot — the support-polygon input.

        x comes from the planar sagittal solve; y is the leg's fixed track
        offset. The legs do not move laterally, so this adds no DOF — but it is
        what turns the fore-aft support INTERVAL into a real polygon.
        """
        return {
            name: np.array([
                float(self.foot_world_position(spine_q, name, leg_q[name])[0]),
                float(self.mounts[name].track_y),
            ])
            for name in self.mounts
        }

    def foot_positions(self, spine_q, leg_q: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        """World (x, z) of every foot.

        `leg_q` maps leg name -> that leg's (q1, q2, q3) joint vector.
        """
        return {
            name: self.foot_world_position(spine_q, name, leg_q[name])
            for name in self.mounts
        }

    # ------------------------------------------------------------ mass (M4)
    @property
    def total_mass(self) -> float:
        """Whole-body mass (kg): spine segments + both girdles + every leg.

        Summed from the PLACEHOLDER params (params.py). By construction the
        default body totals ``LoadCase.body_mass_kg`` = 4.045 kg.
        """
        legs = sum(self.legs[name].params.mass for name in self.mounts)
        return float(self.spine.params.trunk_mass + legs)

    def mass_budget(self) -> QuarterMasses:
        """Fore/hind weight split of the body (the "~60% front-heavy" number).

        Posture-INDEPENDENT bookkeeping on the straight spine, using the lever
        rule described in ``mass.QuarterMasses``: trunk mass is apportioned to
        the two girdles the way a simply-supported beam splits a distributed
        load, and each leg is charged to the girdle it hangs from. Use
        ``center_of_mass`` instead when you want the actual posed CoM.
        """
        fore = [
            self.legs[n].params.mass
            for n, m in self.mounts.items()
            if m.girdle is Girdle.FRONT
        ]
        hind = [
            self.legs[n].params.mass
            for n, m in self.mounts.items()
            if m.girdle is Girdle.REAR
        ]
        return quarter_masses(self.spine.params, fore, hind)

    def girdle_com(self, spine_q, girdle: Girdle) -> ComResult:
        """Mass + CoM of one girdle lump, in the BODY-GROUND frame.

        The girdle mass acts at ``front/rear_girdle_com`` in the girdle's own
        frame, so a spine bend both translates AND rotates it.
        """
        p = self.spine.params
        gx, gz, gth = self.spine.girdle_pose(spine_q, girdle)
        if girdle is Girdle.FRONT:
            m, local = p.front_girdle_mass, p.front_girdle_com
        else:
            m, local = p.rear_girdle_mass, p.rear_girdle_com
        xz = np.array([gx, gz]) + _rot(gth) @ np.asarray(local, dtype=float)
        return ComResult(float(m), xz)

    def spine_com(self, spine_q) -> ComResult:
        """Mass + CoM of the spine SEGMENTS alone (girdles excluded), body frame.

        Uses the ACTUAL bent geometry from spine FK, so arching the back moves
        this CoM up and rearward.
        """
        p = self.spine.params
        return spine_chain_com(
            self.spine.vertebra_positions(spine_q), p.segment_mass, p.segment_com_frac
        )

    def leg_com_world(self, spine_q, leg_name: str, leg_q) -> ComResult:
        """Mass + CoM of one leg in the BODY-GROUND frame.

        The leg's CoM is computed in its own hip frame (``mass.leg_com``) and
        then pushed through the same girdle/hip transform the feet use, so it
        respects the fore/hind asymmetry AND the spine posture.
        """
        hx, hz, hth = self.hip_world_pose(spine_q, leg_name)
        local = leg_com(self.leg_model_for(leg_name), leg_q)
        xz = np.array([hx, hz]) + _rot(hth) @ local.com
        return ComResult(local.mass, xz)

    def center_of_mass_y(self, lateral_q) -> float:
        """Lateral (y) offset of the whole-body CoM for a LATERAL spine posture.

        This is the SWAY authority ADR-0009 bought. Bending the spine sideways
        carries the FRONT girdle -- and everything mounted on it, including the
        fore legs and the head/neck lumped into it -- off the mid-sagittal plane,
        moving the CoM with it. The rear girdle and hind legs stay at the base
        (y = 0), so they anchor the other end.

        Left/right legs on a girdle sit at symmetric track offsets, so their own
        +/-y contributions cancel; what moves the CoM is the girdle they hang
        from. Returns metres, positive = toward +y (left).
        """
        sp = self.spine.params
        seg_y = self.spine.lateral_segment_com_y(lateral_q)
        tip_y = float(self.spine.lateral_vertebra_xy(lateral_q)[-1, 1])

        m_seg = np.asarray(sp.segment_mass, dtype=float)
        m_fore = sum(self.legs[n].params.mass
                     for n, m in self.mounts.items() if m.girdle is Girdle.FRONT)
        num = float((m_seg * seg_y).sum()) + (sp.front_girdle_mass + m_fore) * tip_y
        return num / self.total_mass

    def lateral_spine_loads(self, lateral_accel: float) -> list[dict]:
        """Per-joint LATERAL spine drive loads for a sideways CoM acceleration.

        Sizes the ADR-0009 lateral tendons. Unlike every other load in this model
        these are **inertial, not gravitational**: the lateral bend axis is
        VERTICAL, so gravity (acting along it) exerts no moment about it and
        holding a sway costs essentially nothing. What costs torque is *reversing*
        the sway — accelerating the forequarters sideways during the gait's
        four-foot crossover window (``GaitController.crossover_accel``).

        For each joint, everything DISTAL (toward the head) is treated as a rigid
        body accelerated at ``lateral_accel``::

            tau_joint  = m_distal * a * (com_distal - x_joint)
            T_cable    = tau_joint / lateral_moment_arm      # 20 mm, vs 30 mm
            tau_motor  = T_cable * motor_spool_radius        # dorsoventrally

        Returns one dict per joint (base first) with ``joint``, ``distal_mass``,
        ``lever``, ``joint_torque``, ``cable_tension`` and ``motor_torque`` in SI.

        ⚠️ Rigid-body approximation: the distal chain is taken as one lump at its
        combined CoM, and no spine compliance, tendon stretch or routing friction
        is included. Good for sizing, not a substitute for dynamics.
        """
        sp = self.spine.params
        L = np.asarray(sp.segment_lengths, dtype=float)
        x = np.concatenate([[0.0], np.cumsum(L)])
        m_seg = np.asarray(sp.segment_mass, dtype=float)
        seg_com = (x[:-1] + x[1:]) / 2.0
        m_fore = sum(self.legs[n].params.mass
                     for n, m in self.mounts.items() if m.girdle is Girdle.FRONT)
        m_tip = sp.front_girdle_mass + m_fore          # rides at the spine tip

        out = []
        for j in range(sp.n_segments):
            m_distal = float(m_seg[j:].sum()) + m_tip
            com = (float(np.dot(m_seg[j:], seg_com[j:])) + m_tip * x[-1]) / m_distal
            lever = com - x[j]
            tau = m_distal * float(lateral_accel) * lever
            tension = tau / sp.lateral_moment_arm[j]
            out.append({
                "joint": j,
                "distal_mass": m_distal,
                "lever": lever,
                "joint_torque": tau,
                "cable_tension": tension,
                "motor_torque": tension * sp.motor_spool_radius,
            })
        return out

    def center_of_mass(self, spine_q, leg_q=None) -> BodyCoM:
        """Whole-body centre of mass for a spine posture + per-leg joint angles.

        Parameters
        ----------
        spine_q : array-like
            Spine joint angles (rad), length ``n_segments``.
        leg_q : Mapping[str, array-like] | array-like | None
            Per-leg actuated joint vector ``(q1, q2, q3)``. A single length-3
            vector is broadcast to EVERY leg (note this gives the fore and hind
            legs different geometry, since their link lengths differ). ``None``
            means all-zero angles -- a degenerate, straight-out-forward pose that
            is only useful as a reference, not a stance.

        Returns
        -------
        BodyCoM
            Whole-body CoM plus the per-leg, spine-chain and girdle
            sub-assembly CoMs, all in the BODY-GROUND frame (spine base at the
            origin, x forward, z up).

        Notes
        -----
        QUASI-STATIC: gravity/geometry only. The 2D sagittal collapse means both
        legs of a girdle sit at the SAME x, so this CoM carries no lateral (y)
        information and cannot be used for roll balance.
        """
        if leg_q is None:
            per_leg = {name: np.zeros(3) for name in self.mounts}
        elif isinstance(leg_q, Mapping):
            missing = set(self.mounts) - set(leg_q)
            if missing:
                raise ValueError(f"leg_q is missing joint angles for {sorted(missing)}")
            per_leg = {name: np.asarray(leg_q[name], dtype=float) for name in self.mounts}
        else:
            shared = np.asarray(leg_q, dtype=float)
            if shared.shape != (3,):
                raise ValueError(
                    "leg_q must be a per-leg mapping or a single (3,) vector; "
                    f"got shape {shared.shape}"
                )
            per_leg = {name: shared for name in self.mounts}

        legs = {
            name: self.leg_com_world(spine_q, name, per_leg[name])
            for name in self.mounts
        }
        spine_part = self.spine_com(spine_q)
        girdles = {g: self.girdle_com(spine_q, g) for g in Girdle}
        total = combine([spine_part, *girdles.values(), *legs.values()])
        return BodyCoM(total=total, legs=legs, spine=spine_part, girdles=girdles)

    # ---------------------------------------------------------- inverse (IK)
    def inverse(
        self,
        spine_q,
        leg_name: str,
        foot_world,
        knee: KneeConfig | None = None,   # None -> each leg's own anatomical fold
    ) -> "LegIKSolution":
        """Whole-body INVERSE kinematics for one leg through the moving girdle.

        Given the spine joint angles and a desired foot POSE ``foot_world =
        (x, z, phi)`` in the WORLD (body-ground) frame, place the leg's joint
        angles so the foot lands on that world pose. The spine has already
        decided where the leg's hip frame is in the world, so this simply undoes
        that girdle/hip transform and hands the residual hip-frame pose to the
        per-leg ``LegModel.inverse``.

        Frame chain (world <- girdle(spine_q) <- hip <- foot)
        -----------------------------------------------------
        The forward chain is ``world = girdle(spine_q) . hip . foot``. The spine
        FK gives the hip origin's world pose ``(hx, hz, hth)`` (position + the
        girdle orientation the hip inherits). Inverting the 2D rigid transform:

            hip_xz  = R(-hth) . (world_xz - [hx, hz])      # rotate/translate back
            phi_hip = phi_world - hth                        # de-tilt foot pitch

        ``LegModel.inverse`` then solves ``(hip_xz, phi_hip)`` in the leg's own
        hip frame. Because a spine bend changes ``(hx, hz, hth)``, holding a
        WORLD foot pose FIXED while the spine moves forces the leg angles to
        change -- that is the closed spine<->foot loop M2 lacked.

        Reachability / limits are FLAGGED on the returned ``LegIKSolution`` (q is
        None, reachable=False when the pose is outside the leg workspace; a
        solved pose outside the joint limits sets within_limits=False). This
        never raises so a whole gait/posture sweep can always be sampled.

        Assumptions (2D sagittal, quasi-static, massless, point contact, no ZMP)
        are inherited from the spine + leg models; see the module docstring.
        """
        foot_world = np.asarray(foot_world, dtype=float)
        if foot_world.shape != (3,):
            raise ValueError(
                f"foot_world must be a (x, z, phi) pose; got shape {foot_world.shape}"
            )
        hx, hz, hth = self.hip_world_pose(spine_q, leg_name)
        rel = foot_world[:2] - np.array([hx, hz])
        hip_xz = _rot(-hth) @ rel
        phi_hip = foot_world[2] - hth
        hip_pose = np.array([hip_xz[0], hip_xz[1], phi_hip])

        leg = self.leg_model_for(leg_name)
        try:
            q = leg.inverse(hip_pose, knee=knee)
            reachable = True
            within = bool(leg.in_limits(q))
        except UnreachableError:
            q = None
            reachable = False
            within = False
        return LegIKSolution(
            name=leg_name,
            q=q,
            reachable=reachable,
            within_limits=within,
            foot_world=foot_world,
            foot_hip=hip_pose,
        )

    def inverse_pose(
        self,
        spine_q,
        foot_world_targets: Mapping[str, np.ndarray],
        knee: KneeConfig | None = None,   # None -> each leg's own anatomical fold
    ) -> dict[str, "LegIKSolution"]:
        """Solve every leg for a dict of WORLD foot poses at a spine posture.

        ``foot_world_targets`` maps leg name -> desired ``(x, z, phi)`` world foot
        pose. Returns leg name -> ``LegIKSolution`` (reachability / joint-limit
        flags per leg; never raises). This is the whole-body counterpart of a
        single ``inverse`` call: one spine posture, four legs solved through it.
        """
        return {
            name: self.inverse(spine_q, name, target, knee=knee)
            for name, target in foot_world_targets.items()
        }
