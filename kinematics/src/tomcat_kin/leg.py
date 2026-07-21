"""Planar 3R leg kinematics (sagittal plane).

Joint vector q = (q1, q2, q3) are RELATIVE joint angles (rad):
    q1 = hip, thigh angle relative to +x world axis
    q2 = knee, shank angle relative to thigh
    q3 = ankle, foot angle relative to shank

Foot pose is (x, z, phi):
    x, z = foot-tip position in the hip frame (m)
    phi  = foot pitch = q1 + q2 + q3 (rad)

Forward kinematics (cumulative angles a1=q1, a2=q1+q2, a3=q1+q2+q3):
    x   = l1 cos a1 + l2 cos a2 + l3 cos a3
    z   = l1 sin a1 + l2 sin a2 + l3 sin a3
    phi = a3
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np

from .params import LegParams, DEFAULT_LEG


class UnreachableError(ValueError):
    """Raised when a requested foot pose lies outside the leg's workspace."""


class KneeConfig(Enum):
    """Which of the two 2R IK branches to select."""

    FLEXED_POSITIVE = 1   # q2 > 0
    FLEXED_NEGATIVE = -1  # q2 < 0


@dataclass
class LegModel:
    """Forward/inverse kinematics and Jacobian for one planar 3R leg."""

    params: LegParams = DEFAULT_LEG

    # ------------------------------------------------------------------ FK
    def forward(self, q) -> np.ndarray:
        """Foot pose (x, z, phi) for joint angles q = (q1, q2, q3)."""
        q1, q2, q3 = (float(v) for v in q)
        l1, l2, l3 = self.params.l1, self.params.l2, self.params.l3
        a1, a2, a3 = q1, q1 + q2, q1 + q2 + q3
        x = l1 * math.cos(a1) + l2 * math.cos(a2) + l3 * math.cos(a3)
        z = l1 * math.sin(a1) + l2 * math.sin(a2) + l3 * math.sin(a3)
        return np.array([x, z, a3])

    def joint_positions(self, q) -> np.ndarray:
        """(4, 2) array of joint XY: hip, knee, ankle, foot-tip."""
        q1, q2, q3 = (float(v) for v in q)
        l1, l2, l3 = self.params.l1, self.params.l2, self.params.l3
        a1, a2, a3 = q1, q1 + q2, q1 + q2 + q3
        hip = np.array([0.0, 0.0])
        knee = hip + l1 * np.array([math.cos(a1), math.sin(a1)])
        ankle = knee + l2 * np.array([math.cos(a2), math.sin(a2)])
        foot = ankle + l3 * np.array([math.cos(a3), math.sin(a3)])
        return np.stack([hip, knee, ankle, foot])

    # ------------------------------------------------------------------ IK
    def inverse(
        self,
        pose,
        knee: KneeConfig = KneeConfig.FLEXED_NEGATIVE,
    ) -> np.ndarray:
        """Joint angles for a foot pose (x, z, phi).

        Solves the ankle (wrist) point from phi, does the standard planar 2R
        solution for the hip/knee, then sets the ankle to hit phi.

        Raises UnreachableError if the pose is outside the workspace.
        """
        x, z, phi = (float(v) for v in pose)
        l1, l2, l3 = self.params.l1, self.params.l2, self.params.l3

        # Ankle joint position = foot tip minus the foot link along phi.
        wx = x - l3 * math.cos(phi)
        wz = z - l3 * math.sin(phi)

        r2 = wx * wx + wz * wz
        cos_q2 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        if not -1.0 <= cos_q2 <= 1.0:
            raise UnreachableError(
                f"foot pose {pose} unreachable: |cos(q2)|={abs(cos_q2):.3f} > 1"
            )

        q2 = knee.value * math.acos(cos_q2)
        q1 = math.atan2(wz, wx) - math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))
        q3 = phi - q1 - q2
        return np.array([_wrap(q1), _wrap(q2), _wrap(q3)])

    def in_limits(self, q) -> bool:
        """True if every joint angle is within its configured limit."""
        return all(
            lo <= float(v) <= hi
            for v, lo, hi in zip(q, self.params.q_min, self.params.q_max)
        )

    # ------------------------------------------------------------ Jacobian
    def jacobian(self, q) -> np.ndarray:
        """3x3 Jacobian d(x, z, phi) / d(q1, q2, q3)."""
        q1, q2, q3 = (float(v) for v in q)
        l1, l2, l3 = self.params.l1, self.params.l2, self.params.l3
        a1, a2, a3 = q1, q1 + q2, q1 + q2 + q3
        s1, s2, s3 = math.sin(a1), math.sin(a2), math.sin(a3)
        c1, c2, c3 = math.cos(a1), math.cos(a2), math.cos(a3)

        # Each column adds the contribution of the links distal to that joint.
        t1x, t2x, t3x = -l1 * s1, -l2 * s2, -l3 * s3
        t1z, t2z, t3z = l1 * c1, l2 * c2, l3 * c3
        return np.array(
            [
                [t1x + t2x + t3x, t2x + t3x, t3x],
                [t1z + t2z + t3z, t2z + t3z, t3z],
                [1.0, 1.0, 1.0],
            ]
        )

    def joint_torques_for_wrench(self, q, wrench) -> np.ndarray:
        """Static joint torques (N·m) to exert a foot wrench (Fx, Fz, M).

        Virtual work: tau = J^T · wrench, where `wrench` is the force/moment the
        foot applies on the environment.
        """
        return self.jacobian(q).T @ np.asarray(wrench, dtype=float)


def _wrap(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
