# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""A standing controller on the board's sensors alone -- the base a policy
learns a RESIDUAL on (M135, ADR-0129).

⚠️ **M81 (ADR-0086) found nothing to learn from.** Every action -- random,
zero, a constant gravity hold -- fell in 2-15 control steps, so PPO saw no
gradient. Its listed way out was a residual on a gravity-compensating
baseline. This is that baseline, built only from what `TomcatEnv.observe`
reports (ADR-0081): no `qpos`, no world position.

Four terms, each found by what failed without it:

- **The gravity hold:** the pair tensions that keep the stance pose, as a
  constant. It is solved once on the NOMINAL plant (`nominal_hold`), never the
  randomised one, so the baseline does not peek at the draw. On hardware it is
  the model's own number.
- **The spine, held to its start.** With the spine pairs at zero tension the
  pitch joints fold to their +-25 deg stops within 30 steps and the trunk
  humps. Its angle is read off the rotor and the spool (every spine pair has a
  load cell), so it is observable.
- **The legs, held softly.** A PD on the joint angle `TomcatEnv.joint_estimate`
  reconstructs. ⚠️ Kept SOFT: 6 N.m/rad. At 10 the plant goes unstable in 30
  steps, at 20 in 9. The 7.5 ms delay and the 133 Hz hold make stiff gains
  ring through the series springs. A PD on the ROTOR angle fights the inner
  tension servo instead and is worse at every gain.
- **Attitude from the IMU.** Without it the robot rolls over slowly, 0.6 ->
  32 deg in 1.2 s. It has no hip abduction (ADR-0017), so the only roll
  authority is leg length, left against right. Roll and pitch become a
  vertical foot force on each corner, through `J^T`.

Measured over 10 seeds with every plant and sensor range drawn: 7 / 10 stand
the full 4 s, a mean of 421 of 533 steps. Without randomisation it stands
throughout, peak tilt 8 deg.
"""

from __future__ import annotations

import math

import numpy as np

from . import LegModel
from . import mjcf_tendon as MT
from . import wbc
from .env import LEGS, SPINE, TomcatEnv
from .params import DEFAULT_FORELEG, DEFAULT_HINDLEG

#: Leg joint PD, N.m/rad and N.m.s/rad.
KP_LEG, KD_LEG = 6.0, 0.01
#: Spine joint PD.
KP_SPINE, KD_SPINE = 10.0, 0.01
#: Attitude: N of vertical foot force per rad of roll / pitch, and per rad/s
#: of roll rate.
K_ROLL, K_ROLL_RATE, K_PITCH = 100.0, 2.0, 50.0

_HOLD = {}


def nominal_hold(**env_kw) -> np.ndarray:
    """The 18 pair tensions that hold the stance pose on the NOMINAL plant:
    what the reset fixture applies when it lets go. Cached per env setting."""
    key = tuple(sorted(env_kw.items()))
    if key not in _HOLD:
        env = TomcatEnv(rng=np.random.default_rng(0), **env_kw)
        env.reset()
        hold = np.zeros(18)
        for k, nm in enumerate(LEGS):
            qa, dof = env._qadr(nm), env._dofadr(nm)
            G = env._leg_gain(nm, qa)
            bias = np.array([float(env.data.qfrc_bias[a] - env.data.qfrc_passive[a])
                             for a in dof])
            e = env.q_ref[nm] - np.array([float(env.data.qpos[a]) for a in qa])
            ev = -np.array([float(env.data.qvel[a]) for a in dof])
            hold[3 * k:3 * k + 3] = wbc.pair_command(G, bias + 30.0 * e + 1.5 * ev,
                                                     MT.TENSION_MAX)
        _HOLD[key] = hold
    return _HOLD[key].copy()


def _roll_pitch(q):
    w, x, y, z = (float(v) for v in q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


class StandBaseline:
    """Call it with each observation; it returns the 18 pair tensions (N).
    `reset(obs)` with the episode's first observation, which it holds."""

    def __init__(self, env: TomcatEnv, hold: np.ndarray | None = None):
        self.env = env
        self.hold = nominal_hold() if hold is None else np.asarray(hold, float)
        self.Gp = -np.asarray(env.pair_map, float).T
        # spine cable length per radian of its own joint (30 / 20 mm, pitch /
        # yaw) -- a constant of the geometry, measured once
        self.gs = self._spine_gain()
        self.si = [env.pairs.index(p) for p in SPINE]
        self.legs = {nm: LegModel(DEFAULT_FORELEG if nm[1] == "F" else DEFAULT_HINDLEG)
                     for nm in LEGS}
        self.dt = 1.0 / env.control_hz

    def _spine_gain(self):
        import mujoco
        env = self.env
        g = []
        for p in SPINE:
            t = env._id(mujoco.mjtObj.mjOBJ_TENDON, p)
            a = env.model.jnt_qposadr[env._id(mujoco.mjtObj.mjOBJ_JOINT, p)]
            L = []
            for sg in (+1, -1):
                d = mujoco.MjData(env.model)
                d.qpos[:] = env.data.qpos
                d.qpos[a] += sg * 0.002
                mujoco.mj_forward(env.model, d)
                L.append(float(d.ten_length[t]))
            g.append((L[0] - L[1]) / 0.004)
        return np.array(g)

    def spine_angle(self, obs) -> np.ndarray:
        """Spine joint angles relative to the cable lengths at reset: the rotor
        and spool angles wind the cable, `L - a0 = -r (theta_r + theta_s)`."""
        th = np.array([obs["rotor"][i] + obs["load"].get(p, 0.0)
                       for i, p in zip(self.si, SPINE)])
        return -MT.SPOOL_R * th / self.gs

    def reset(self, obs):
        self.q0 = self.env.joint_estimate(obs)
        self.q_prev = self.q0
        self.s0 = self.spine_angle(obs)
        self.s_prev = self.s0
        self.att0 = _roll_pitch(obs["imu_quat"])
        return self

    def __call__(self, obs) -> np.ndarray:
        q = self.env.joint_estimate(obs)
        s = self.spine_angle(obs)
        roll, pitch = _roll_pitch(obs["imu_quat"])
        droll, dpitch = roll - self.att0[0], pitch - self.att0[1]
        T = self.hold.copy()
        for k, nm in enumerate(LEGS):
            left = 1.0 if nm[0] == "L" else -1.0
            fore = 1.0 if nm[1] == "F" else -1.0
            fz = (left * (K_ROLL * droll + K_ROLL_RATE * float(obs["imu_gyro"][0]))
                  - fore * K_PITCH * dpitch)
            J = self.legs[nm].jacobian(q[nm])
            tau = (KP_LEG * (self.q0[nm] - q[nm])
                   - KD_LEG * (q[nm] - self.q_prev[nm]) / self.dt
                   + J.T @ np.array([0.0, fz, 0.0]))
            T[3 * k:3 * k + 3] += np.linalg.solve(self.Gp, tau)
        T[12:] = (KP_SPINE * (self.s0 - s)
                  - KD_SPINE * (s - self.s_prev) / self.dt) / (-self.gs)
        self.q_prev, self.s_prev = q, s
        if getattr(self.env, "friction", False):
            # M146 (ADR-0140): the controller knows its drive -- it asks the
            # motor for what the joint needs times the conduit's friction, at
            # the pose it estimates
            qa = np.array([q[nm] for nm in LEGS])
            T[:12] = T[:12] * self.env.capstan(qa, T[:12]).ravel()
        return T
