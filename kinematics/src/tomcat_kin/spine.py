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

Modelling assumptions / limitations
-----------------------------------
- 2D sagittal only: the four legs' left/right lateral (y) offset is out of plane
  and ignored, so in this projection the two front legs share a mount pose (and
  likewise the two rear legs). This is a real limitation for anything involving
  roll, yaw, lateral bend, or the righting-reflex twist.
- Rigid links, frictionless idealisation inherited from the leg/tendon models;
  the literature shows leg mass materially bends a compliant trunk, so this
  static, massless model will not capture spine-leg energy exchange.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math

import numpy as np

from .params import SpineParams, LegParams, DEFAULT_SPINE, DEFAULT_LEG
from .leg import LegModel


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
    hip_offset : (x, z) of the hip origin in the girdle frame (m). In this 2D
                 sagittal model the left/right (y) offset is out of plane and is
                 dropped, so left/right legs on the same girdle share a mount.
    """

    name: str
    girdle: Girdle
    hip_offset: tuple[float, float] = (0.0, 0.0)


# Default four-leg layout: shoulder pair on the front girdle, pelvic pair on the
# rear girdle. Hip offsets are (0, 0) placeholders (hips at the girdle mounts).
DEFAULT_MOUNTS: tuple[LegMount, ...] = (
    LegMount("LF", Girdle.FRONT),
    LegMount("RF", Girdle.FRONT),
    LegMount("LR", Girdle.REAR),
    LegMount("RR", Girdle.REAR),
)


@dataclass
class WholeBody:
    """Composition of the spine and four legs into one kinematic body.

    The spine sets each girdle pose; each leg then runs in its hip frame and its
    foot is expressed in the world. Call with the spine joint vector plus a
    per-leg joint vector to get every foot's world position.
    """

    spine: SpineModel = field(default_factory=SpineModel)
    legs: dict[str, LegModel] = field(default_factory=dict)
    mounts: dict[str, LegMount] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.mounts:
            self.mounts = {m.name: m for m in DEFAULT_MOUNTS}
        if not self.legs:
            # One LegModel per mount, all sharing the default leg geometry.
            self.legs = {name: LegModel() for name in self.mounts}

    @property
    def leg_names(self) -> tuple[str, ...]:
        return tuple(self.mounts.keys())

    def hip_world_pose(self, spine_q, leg_name: str) -> np.ndarray:
        """World pose (x, z, theta) of a leg's hip origin given the spine state."""
        mount = self.mounts[leg_name]
        return self.spine.hip_origin_world(spine_q, mount.girdle, mount.hip_offset)

    def foot_world_position(self, spine_q, leg_name: str, leg_q) -> np.ndarray:
        """World (x, z) of one foot given spine angles and that leg's joint angles."""
        hx, hz, hth = self.hip_world_pose(spine_q, leg_name)
        foot_hip = self.legs[leg_name].forward(leg_q)[:2]  # (x, z) in hip frame
        world = np.array([hx, hz]) + _rot(hth) @ foot_hip
        return world

    def foot_positions(self, spine_q, leg_q: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        """World (x, z) of every foot.

        `leg_q` maps leg name -> that leg's (q1, q2, q3) joint vector.
        """
        return {
            name: self.foot_world_position(spine_q, name, leg_q[name])
            for name in self.mounts
        }
