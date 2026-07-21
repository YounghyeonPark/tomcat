"""Tendon map: the bridge between joint space and actuator space.

A cable can only PULL, never push. This module converts, per joint:

    joint angle   <->  cable length change  <->  motor angle
    joint torque  <->  tendon tension       <->  motor torque

Two actuation modes are supported (see ADR-0002):

- ANTAGONISTIC: two opposing tendons per joint (flexor + extensor). Both
  tensions stay >= `pretension`; the joint torque is the difference of the two,
  scaled by the moment arm. Co-contraction (raising both) stiffens the joint.

- SPRING_RETURN: one driven tendon plus a passive return spring. The motor can
  only pull one way; the spring supplies the restoring torque.

Sign convention: a POSITIVE joint torque is produced by the FLEXOR tendon
(the tendon whose tension increases the joint angle).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from .params import TendonParams, DEFAULT_TENDON


class ActuationMode(Enum):
    ANTAGONISTIC = "antagonistic"
    SPRING_RETURN = "spring_return"


@dataclass
class TendonSolution:
    """Result of resolving a set of joint torques into actuator commands.

    All arrays are length-3 (hip, knee, ankle) unless noted.
    """

    tension_flexor: np.ndarray     # N, >= 0
    tension_extensor: np.ndarray   # N, >= 0 (zeros in spring-return mode)
    motor_torque: np.ndarray       # N·m per driven motor
    joint_torque: np.ndarray       # N·m actually realized (for cross-check)

    @property
    def n_motors(self) -> int:
        """Motors required for this solution across all three joints."""
        two = np.count_nonzero(self.tension_extensor > 0.0)
        return int(len(self.tension_flexor) + two)


@dataclass
class TendonMap:
    """Per-joint tendon and actuator model for one leg."""

    params: TendonParams = DEFAULT_TENDON
    mode: ActuationMode = ActuationMode.ANTAGONISTIC

    def __post_init__(self) -> None:
        self._r = np.asarray(self.params.joint_moment_arm, dtype=float)
        self._k = np.asarray(self.params.spring_stiffness, dtype=float)
        self._q0 = np.asarray(self.params.spring_rest_angle, dtype=float)

    # ----------------------------------------------- geometry (angle <-> cable)
    def cable_lengths(self, q) -> np.ndarray:
        """Cable-length change from the zero pose (m), (3, 2): [flexor, extensor].

        Flexor shortens as the joint angle grows; extensor lengthens.
        """
        q = np.asarray(q, dtype=float)
        delta = self._r * q
        return np.stack([-delta, delta], axis=1)

    def motor_angles(self, q) -> np.ndarray:
        """Flexor motor angle (rad) per joint to hold joint angles q.

        Cable displacement r*q is delivered by a spool of radius r_spool, so the
        motor turns r*q / r_spool.
        """
        q = np.asarray(q, dtype=float)
        return self._r * q / self.params.motor_spool_radius

    # ------------------------------------------------ statics (torque <-> tension)
    def resolve(self, joint_torque) -> TendonSolution:
        """Resolve desired joint torques (N·m) into tendon tensions + motor torques."""
        tau = np.asarray(joint_torque, dtype=float)
        if self.mode is ActuationMode.ANTAGONISTIC:
            return self._resolve_antagonistic(tau)
        return self._resolve_spring(tau)

    def _resolve_antagonistic(self, tau: np.ndarray) -> TendonSolution:
        # tau = r * (T_flex - T_ext); keep the slack side at `pretension`.
        pre = self.params.pretension
        dtension = tau / self._r  # required T_flex - T_ext
        t_flex = np.where(dtension >= 0, pre + dtension, pre)
        t_ext = np.where(dtension >= 0, pre, pre - dtension)
        realized = self._r * (t_flex - t_ext)
        # In antagonistic mode the driven motor is whichever tendon is pulling
        # hardest; report the larger tension's motor torque.
        peak = np.maximum(t_flex, t_ext)
        return TendonSolution(
            tension_flexor=t_flex,
            tension_extensor=t_ext,
            motor_torque=peak * self.params.motor_spool_radius,
            joint_torque=realized,
        )

    def _resolve_spring(self, tau: np.ndarray, q=None) -> TendonSolution:
        # tau = r * T_flex - k*(q - q0)  =>  T_flex = (tau + k*(q-q0)) / r
        q = np.zeros(3) if q is None else np.asarray(q, dtype=float)
        spring_torque = self._k * (q - self._q0)
        t_flex = (tau + spring_torque) / self._r
        # A cable cannot push: clamp to the pretension floor.
        t_flex = np.maximum(t_flex, self.params.pretension)
        realized = self._r * t_flex - spring_torque
        return TendonSolution(
            tension_flexor=t_flex,
            tension_extensor=np.zeros_like(t_flex),
            motor_torque=t_flex * self.params.motor_spool_radius,
            joint_torque=realized,
        )
