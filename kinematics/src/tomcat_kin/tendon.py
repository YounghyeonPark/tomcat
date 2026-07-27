"""Tendon map: the bridge between joint space and actuator space.

A cable can only PULL, never push. This module converts, per joint:

    joint angle   <->  cable length change  <->  motor angle
    joint torque  <->  tendon tension       <->  motor torque

Two actuation modes are supported (see ADR-0002):

- ANTAGONISTIC: two opposing tendons per joint (flexor + extensor). Both
  tensions stay >= a per-joint co-contraction bias `T_bias`; the joint torque is
  the difference of the two, scaled by the moment arm. Co-contraction (raising
  `T_bias`) stiffens the joint WITHOUT changing net torque.

- SPRING_RETURN: one driven tendon plus a passive return spring. The motor can
  only pull one way; the spring supplies the restoring torque.

Sign convention: a POSITIVE joint torque is produced by the FLEXOR tendon
(the tendon whose tension increases the joint angle).

Commandable co-contraction bias (`T_bias`) — ADR-0002 (Accepted)
----------------------------------------------------------------
Per Kengoro's muscle-stiffness law `T_target = T_bias + max(0, k·(l - l_target))`
with Antagonist Inhibition Control (AIC), the antagonist is HELD at `T_bias`
while the agonist carries the net torque, which keeps peak tension down
(Kengoro cut peak tension 43->28 kgf; see LITERATURE_REVIEW.md Q1).

What this STATIC map implements: it exposes `T_bias` as a first-class input to
`resolve()` and realizes the AIC *split* directly — the slack (antagonist) side
is held at `T_bias` and the active (agonist) side rises to `T_bias + |tau|/r`.
Raising `T_bias` raises BOTH tensions (stiffer joint) but leaves the realized
net joint torque unchanged. The fixed-`pretension` behaviour is recovered as the
`T_bias = params.pretension` default, so it is just one operating point.

What this map does NOT do: the *dynamic* AIC gain-scheduling — the runtime
stiffness gain `k` that maps a joint-angle / cable-length error into the agonist
tension setpoint, and the moment-to-moment choice of which side is agonist — is
a firmware / real-time control concern, not part of this static kinematic map.
This module only provides the tension split for a commanded (torque, T_bias)
pair; it does not close a stiffness loop.

Tendon non-idealities (ADR-0003): friction & stretch
----------------------------------------------------
Per ADR-0003 the legs are pure tendon-driven too, so Coulomb friction and cable
stretch are first-class here, not spine-only.

- FRICTION (capstan / Eytelwein). Tension differs between the two ends of the
  routing because Coulomb friction acts over the total wrap angle `theta_wrap`
  of the cable across pulleys/sheaths:

      T_motor = T_joint * exp(+mu * theta_wrap)   (motor PULLING against the load)
      T_motor = T_joint * exp(-mu * theta_wrap)   (motor PAYING OUT / releasing)

  DIRECTION CONVENTION. `T_joint` is the tension at the JOINT end — the tension
  that actually produces joint torque (`tau = r * (T_flex - T_ext)`). `T_motor`
  is the tension the MOTOR/cable must supply at the spool. To DEVELOP or HOLD a
  joint load the motor pulls the cable in against friction, so it must supply the
  LARGER `exp(+mu*theta)` tension; this is the worst case for motor and cable
  sizing and is what `resolve()` reports as the motor-side tension and uses to
  size `motor_torque`. `mu = 0` OR `theta_wrap = 0` gives factor 1, so the map
  reduces exactly to the previous frictionless behaviour.

- STRETCH (series compliance). The cable is a linear spring of stiffness
  `k_cable` (N/m): under tension `T` it lengthens `dL = T / k_cable`. To hold a
  commanded joint angle the motor must wind that extra `dL` (an extra motor angle
  `dL / r_spool`) beyond the geometric `r*q`; if the winding is NOT compensated
  the joint under-rotates by `dL / r`. `k_cable = None` (or inf) => inextensible,
  the previous behaviour. See `cable_stretch` / `extra_motor_angle` /
  `joint_angle_error`.

Both models assume rigid links and lumped, quasi-static cable behaviour; a real
routed cable has a tension GRADIENT along its length under friction, so the true
stretch lies between the joint-side and motor-side values — the lumped spring
here takes the tension it is handed as representative.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from .params import TendonParams, SpineParams, DEFAULT_TENDON


class ActuationMode(Enum):
    ANTAGONISTIC = "antagonistic"
    SPRING_RETURN = "spring_return"


@dataclass
class TendonSolution:
    """Result of resolving a set of joint torques into actuator commands.

    All arrays are length-3 (hip, knee, ankle) unless noted.
    """

    tension_flexor: np.ndarray     # N, >= 0, JOINT-side (produces joint torque)
    tension_extensor: np.ndarray   # N, >= 0, JOINT-side (zeros in spring-return mode)
    motor_torque: np.ndarray       # N·m per driven motor (from MOTOR-side tension)
    joint_torque: np.ndarray       # N·m actually realized (for cross-check)
    t_bias: np.ndarray | None = None  # N, co-contraction bias used (antagonistic)
    # MOTOR-side tensions = joint-side * capstan factor exp(+mu*theta_wrap) — what
    # the motor/cable must supply when pulling against the load (see module doc).
    # Equal to the joint-side tensions when friction is off (mu=0 or wrap=0).
    motor_tension_flexor: np.ndarray | None = None    # N, >= 0
    motor_tension_extensor: np.ndarray | None = None  # N, >= 0 (zeros in spring-return)

    @property
    def n_motors(self) -> int:
        """Motors required for this solution across all three joints."""
        two = np.count_nonzero(self.tension_extensor > 0.0)
        return int(len(self.tension_flexor) + two)


@dataclass
class TendonMap:
    """Per-joint tendon and actuator model.

    Works for any number of joints (the moment-arm array sets the count), so the
    same class serves the 3-joint leg and the N-segment spine. `from_spine`
    builds one directly from `SpineParams`.
    """

    params: TendonParams = DEFAULT_TENDON
    mode: ActuationMode = ActuationMode.ANTAGONISTIC

    def __post_init__(self) -> None:
        self._r = np.asarray(self.params.joint_moment_arm, dtype=float)
        self._k = np.asarray(self.params.spring_stiffness, dtype=float)
        self._q0 = np.asarray(self.params.spring_rest_angle, dtype=float)

    @classmethod
    def from_spine(
        cls,
        spine_params: SpineParams,
        mode: ActuationMode = ActuationMode.ANTAGONISTIC,
    ) -> "TendonMap":
        """Build a spine tendon map from `SpineParams`.

        The spine's per-segment moment arms / spring parameters are packed into a
        `TendonParams` so spine joints get the exact same antagonistic
        torque<->tension and angle<->cable treatment as the leg joints. The
        resulting arrays have length `n_segments` instead of 3.
        """
        tp = TendonParams(
            joint_moment_arm=tuple(spine_params.joint_moment_arm),
            motor_spool_radius=spine_params.motor_spool_radius,
            pretension=spine_params.pretension,
            spring_stiffness=tuple(spine_params.spring_stiffness),
            spring_rest_angle=tuple(spine_params.spring_rest_angle),
            friction_coeff=spine_params.friction_coeff,
            wrap_angle=spine_params.wrap_angle,
            k_cable=spine_params.k_cable,
        )
        return cls(params=tp, mode=mode)

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

    # ------------------------------------------------------- friction (capstan)
    def capstan_factor(self, *, paying_out: bool = False) -> float:
        """Capstan tension ratio T_motor / T_joint over the routing (dimensionless).

        `exp(+mu*theta_wrap)` when the motor PULLS against the load (default, the
        worst case for sizing); `exp(-mu*theta_wrap)` when PAYING OUT. Returns 1.0
        when friction is off (mu = 0 or wrap = 0). See the module docstring for the
        sign/direction convention.
        """
        mu = float(self.params.friction_coeff)
        wrap = float(self.params.wrap_angle)
        sign = -1.0 if paying_out else 1.0
        return float(np.exp(sign * mu * wrap))

    # -------------------------------------------------------- stretch (compliance)
    def cable_stretch(self, tension) -> np.ndarray:
        """Cable elongation dL (m) under `tension` (N): dL = |T| / k_cable.

        Returns zeros when the cable is modelled as inextensible (`k_cable` is
        None or non-finite). `tension` is taken as the representative cable
        tension (see the module doc on the friction-induced tension gradient).
        """
        T = np.abs(np.asarray(tension, dtype=float))
        k = self.params.k_cable
        if k is None or not np.isfinite(k):
            return np.zeros_like(T)
        return T / float(k)

    def extra_motor_angle(self, tension) -> np.ndarray:
        """Extra motor rotation (rad) needed to take up cable stretch at `tension`.

        The motor must wind dL = |T|/k_cable of extra cable off a spool of radius
        r_spool, i.e. dL / r_spool beyond the geometric r*q, to hold the angle.
        """
        return self.cable_stretch(tension) / self.params.motor_spool_radius

    def joint_angle_error(self, tension) -> np.ndarray:
        """Joint-angle error (rad) if the cable stretch is NOT compensated.

        If the motor winds only the geometric r*q and ignores stretch, the cable
        elongation dL is not delivered to the joint pulley, so the joint
        under-rotates by dL / r (per-joint moment arm). Sign is the magnitude of
        the shortfall; the joint lags the commanded angle.
        """
        return self.cable_stretch(tension) / self._r

    # ------------------------------------------------ statics (torque <-> tension)
    def _bias_array(self, t_bias) -> np.ndarray:
        """Broadcast the co-contraction bias to a per-joint array (N).

        `t_bias` may be None (default to `params.pretension`), a scalar (applied
        to every joint), or a length-N array (per-joint bias).
        """
        if t_bias is None:
            t_bias = self.params.pretension
        bias = np.broadcast_to(
            np.asarray(t_bias, dtype=float), self._r.shape
        ).astype(float)
        if np.any(bias < 0.0):
            raise ValueError("t_bias (co-contraction bias) must be >= 0")
        return bias

    def resolve(self, joint_torque, t_bias=None) -> TendonSolution:
        """Resolve desired joint torques (N·m) into tendon tensions + motor torques.

        `t_bias` is the commandable co-contraction bias (N), scalar or length-N
        (ADR-0002). It is the floor both antagonistic tendons hold and sets joint
        stiffness independently of the net torque. `None` defaults to
        `params.pretension`, reproducing the fixed-pretension behaviour. Ignored
        in SPRING_RETURN mode (a single tendon has no antagonist to co-contract).
        """
        tau = np.asarray(joint_torque, dtype=float)
        if self.mode is ActuationMode.ANTAGONISTIC:
            return self._resolve_antagonistic(tau, t_bias)
        return self._resolve_spring(tau)

    def _resolve_antagonistic(self, tau: np.ndarray, t_bias=None) -> TendonSolution:
        # AIC split (ADR-0002): antagonist HELD at the co-contraction bias, agonist
        # carries the net torque.  tau = r * (T_flex - T_ext), so the pulling side
        # sits at bias + |tau|/r and the slack side stays at bias.
        bias = self._bias_array(t_bias)
        dtension = tau / self._r  # required T_flex - T_ext
        active = bias + np.abs(dtension)
        t_flex = np.where(dtension >= 0, active, bias)
        t_ext = np.where(dtension >= 0, bias, active)
        realized = self._r * (t_flex - t_ext)  # independent of bias
        # Capstan: the motor pulls the loaded cable in against friction, so the
        # motor-side tension is the joint-side tension amplified by exp(+mu*wrap).
        factor = self.capstan_factor()
        m_flex = t_flex * factor
        m_ext = t_ext * factor
        # In antagonistic mode the driven motor is whichever tendon is pulling
        # hardest; size the motor torque from the (motor-side) peak tension.
        peak_motor = np.maximum(m_flex, m_ext)
        return TendonSolution(
            tension_flexor=t_flex,
            tension_extensor=t_ext,
            motor_torque=peak_motor * self.params.motor_spool_radius,
            joint_torque=realized,
            t_bias=bias,
            motor_tension_flexor=m_flex,
            motor_tension_extensor=m_ext,
        )

    def _resolve_spring(self, tau: np.ndarray, q=None) -> TendonSolution:
        # tau = r * T_flex - k*(q - q0)  =>  T_flex = (tau + k*(q-q0)) / r
        q = np.zeros_like(self._r) if q is None else np.asarray(q, dtype=float)
        spring_torque = self._k * (q - self._q0)
        t_flex = (tau + spring_torque) / self._r
        # A cable cannot push: clamp to the pretension floor.
        t_flex = np.maximum(t_flex, self.params.pretension)
        realized = self._r * t_flex - spring_torque
        factor = self.capstan_factor()
        m_flex = t_flex * factor
        return TendonSolution(
            tension_flexor=t_flex,
            tension_extensor=np.zeros_like(t_flex),
            motor_torque=m_flex * self.params.motor_spool_radius,
            joint_torque=realized,
            motor_tension_flexor=m_flex,
            motor_tension_extensor=np.zeros_like(t_flex),
        )
