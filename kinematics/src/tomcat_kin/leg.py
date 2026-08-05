# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Digitigrade 4-link leg kinematics (sagittal plane).

Four links -- femur (l1), tibia (l2), metatarsus (l3), paw (l4) -- but only
THREE ACTUATED joints. The paw is a PASSIVE distal link rigidly offset from the
metatarsus by the fixed `paw_angle` (see LegParams); it has NO motor, modelling
the largely passive toes / ground contact of a digitigrade stance.

Actuated joint vector q = (q1, q2, q3) are RELATIVE joint angles (rad):
    q1 = hip,          femur angle relative to +x world axis
    q2 = stifle/knee,  tibia angle relative to femur
    q3 = hock/ankle,   metatarsus angle relative to tibia

Cumulative link directions (CCW from +x):
    a1 = q1                      (femur)
    a2 = q1 + q2                 (tibia)
    a3 = q1 + q2 + q3            (metatarsus)
    a4 = a3 + paw_angle          (paw -- PASSIVE, rigid offset from a3)

Foot pose is the PAW-TIP (ground-contact) pose (x, z, phi):
    x, z = paw-tip position in the hip frame (m)
    phi  = paw-tip pitch = a4 = q1 + q2 + q3 + paw_angle (rad)

Forward kinematics:
    x   = l1 cos a1 + l2 cos a2 + l3 cos a3 + l4 cos a4
    z   = l1 sin a1 + l2 sin a2 + l3 sin a3 + l4 sin a4
    phi = a4

Modelling assumptions (flagged per engineering standards): rigid links, a
massless leg, and a paw held at EXACTLY its rest angle (no toe compliance /
ground rollover). The literature shows leg mass materially bends a compliant
trunk, so this static, massless model will not capture those effects.
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
    """Forward/inverse kinematics and Jacobian for one digitigrade 4-link leg.

    Three actuated joints (hip, stifle, hock) plus a passive paw; the public API
    (`forward`, `inverse`, `jacobian`) is in terms of the paw-tip (ground
    contact) pose and the length-3 actuated joint vector q = (q1, q2, q3).
    """

    params: LegParams = DEFAULT_LEG

    @property
    def default_knee(self) -> "KneeConfig":
        """The fold direction implied by this leg's OWN middle-joint limits.

        A cat's stifle and elbow bend in OPPOSITE directions, so the hind leg
        carries a negative stifle range and the fore leg a positive elbow range
        (see `DEFAULT_FORELEG`). Deriving the IK branch from the limits keeps the
        anatomy and the solver from drifting apart — there is no separate switch
        to set wrongly.
        """
        return (KneeConfig.FLEXED_POSITIVE if self.params.q_max[1] > 0.0
                else KneeConfig.FLEXED_NEGATIVE)

    # ------------------------------------------------------------------ FK
    def forward(self, q) -> np.ndarray:
        """Paw-tip pose (x, z, phi) for actuated joint angles q = (q1, q2, q3).

        phi is the PAW-TIP pitch a4 = q1 + q2 + q3 + paw_angle; the paw rides
        rigidly at `paw_angle` off the metatarsus.
        """
        q1, q2, q3 = (float(v) for v in q)
        p = self.params
        a1, a2, a3 = q1, q1 + q2, q1 + q2 + q3
        a4 = a3 + p.paw_angle  # passive paw direction
        x = (
            p.l1 * math.cos(a1) + p.l2 * math.cos(a2)
            + p.l3 * math.cos(a3) + p.l4 * math.cos(a4)
        )
        z = (
            p.l1 * math.sin(a1) + p.l2 * math.sin(a2)
            + p.l3 * math.sin(a3) + p.l4 * math.sin(a4)
        )
        return np.array([x, z, a4])

    def joint_positions(self, q) -> np.ndarray:
        """(5, 2) array of chain XY: hip, stifle, hock, paw-base, paw-tip.

        The paw-base is the metatarsus (hock) tip; the paw-tip (last row) is the
        ground-contact point that `forward` returns.
        """
        q1, q2, q3 = (float(v) for v in q)
        p = self.params
        a1, a2, a3 = q1, q1 + q2, q1 + q2 + q3
        a4 = a3 + p.paw_angle  # passive paw direction
        hip = np.array([0.0, 0.0])
        stifle = hip + p.l1 * np.array([math.cos(a1), math.sin(a1)])
        hock = stifle + p.l2 * np.array([math.cos(a2), math.sin(a2)])
        paw_base = hock + p.l3 * np.array([math.cos(a3), math.sin(a3)])
        paw_tip = paw_base + p.l4 * np.array([math.cos(a4), math.sin(a4)])
        return np.stack([hip, stifle, hock, paw_base, paw_tip])

    # ------------------------------------------------------------------ IK
    def inverse(
        self,
        pose,
        knee: KneeConfig | None = None,
    ) -> np.ndarray:
        """Actuated joint angles (q1, q2, q3) for a PAW-TIP pose (x, z, phi).

        `phi` is the paw-tip pitch. The passive paw is rigid, so we first back out
        the metatarsus (hock) tip and its orientation, then reuse the standard
        planar-3R solution for the actuated femur/tibia/metatarsus chain:

            a3 = phi - paw_angle                     # metatarsus orientation
            (mx, mz) = paw_tip - l4 * (cos phi, sin phi)   # metatarsus-tip = paw base
            (wx, wz) = (mx, mz) - l3 * (cos a3, sin a3)    # hock (2R wrist) point

        A standard 2R solve on (wx, wz) gives (q1, q2); q3 sets the metatarsus to
        a3. Both knee branches are kept. Raises UnreachableError if the pose is
        outside the workspace.
        """
        if knee is None:
            knee = self.default_knee
        x, z, phi = (float(v) for v in pose)
        p = self.params
        l1, l2, l3, l4 = p.l1, p.l2, p.l3, p.l4

        # Undo the rigid passive paw: metatarsus (hock) tip = paw-base, and the
        # metatarsus points along a3 = phi - paw_angle.
        mx = x - l4 * math.cos(phi)
        mz = z - l4 * math.sin(phi)
        a3 = phi - p.paw_angle

        # Hock (2R wrist) point = metatarsus tip minus the metatarsus link.
        wx = mx - l3 * math.cos(a3)
        wz = mz - l3 * math.sin(a3)

        r2 = wx * wx + wz * wz
        cos_q2 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        if not -1.0 <= cos_q2 <= 1.0:
            raise UnreachableError(
                f"paw-tip pose {pose} unreachable: |cos(q2)|={abs(cos_q2):.3f} > 1"
            )

        q2 = knee.value * math.acos(cos_q2)
        q1 = math.atan2(wz, wx) - math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))
        q3 = a3 - q1 - q2
        return np.array([_wrap(q1), _wrap(q2), _wrap(q3)])

    def in_limits(self, q) -> bool:
        """True if every joint angle is within its configured limit."""
        return all(
            lo <= float(v) <= hi
            for v, lo, hi in zip(q, self.params.q_min, self.params.q_max)
        )

    # ------------------------------------------------------------ Jacobian
    def jacobian(self, q) -> np.ndarray:
        """3x3 Jacobian d(paw-tip x, z, phi) / d(q1, q2, q3).

        The passive paw (l4 at a4 = q1+q2+q3+paw_angle) depends on ALL THREE
        actuated joints, so its position term appears in every column; phi = a4
        gives the constant [1, 1, 1] pitch row.
        """
        q1, q2, q3 = (float(v) for v in q)
        p = self.params
        l1, l2, l3, l4 = p.l1, p.l2, p.l3, p.l4
        a1, a2, a3 = q1, q1 + q2, q1 + q2 + q3
        a4 = a3 + p.paw_angle
        s1, s2, s3, s4 = math.sin(a1), math.sin(a2), math.sin(a3), math.sin(a4)
        c1, c2, c3, c4 = math.cos(a1), math.cos(a2), math.cos(a3), math.cos(a4)

        # Each column adds the contribution of the links distal to that joint;
        # the paw term (t4) is distal to all three actuated joints.
        t1x, t2x, t3x, t4x = -l1 * s1, -l2 * s2, -l3 * s3, -l4 * s4
        t1z, t2z, t3z, t4z = l1 * c1, l2 * c2, l3 * c3, l4 * c4
        return np.array(
            [
                [t1x + t2x + t3x + t4x, t2x + t3x + t4x, t3x + t4x],
                [t1z + t2z + t3z + t4z, t2z + t3z + t4z, t3z + t4z],
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
