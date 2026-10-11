# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""A Gymnasium face for `TomcatEnv`, so a standard PPO can be pointed at it.

⚠️ **The point of training here is to validate the REWARD**, not to produce a
controller. ADR-0082 recorded that the reward's shape is tested but its
behaviour is not, and that no open-loop reference exists to check it against —
a policy is the only instrument that can. So the algorithm has to be one that is
not itself a suspect: a hand-rolled PPO that failed to learn would leave "bad
reward" and "bad PPO" indistinguishable, which is the whole question.

The observation is flattened from `TomcatEnv.observe()` and contains **nothing
the board does not measure** (ADR-0081): rotor encoders and their velocities,
the fourteen load cells that exist, the IMU, and foot contact.
"""

from __future__ import annotations

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as exc:                                   # pragma: no cover
    raise ImportError("gymnasium is needed for the training face") from exc

from . import mjcf_tendon as MT
from .env import LEGS, PAIRS, SPINE, TomcatEnv

#: The load cells that exist. ⚠️ ADR-0004 leaves the ankle DNP, so this is 14
#: and not 18, and the observation must not pretend otherwise.
LOAD_KEYS = tuple([f"{nm}_{p}" for nm in LEGS for p in PAIRS
                   if p in MT.LOAD_CELL_PAIRS] + list(SPINE))


def flatten(obs) -> np.ndarray:
    """Observation vector: 18 + 18 + 14 + 4 + 3 + 3 + 4 = 64."""
    return np.concatenate([
        np.asarray(obs["rotor"], np.float32),
        np.asarray(obs["rotor_vel"], np.float32) * 0.01,
        np.array([obs["load"][k] for k in LOAD_KEYS], np.float32) * 100.0,
        np.asarray(obs["imu_quat"], np.float32),
        np.asarray(obs["imu_gyro"], np.float32) * 0.1,
        np.asarray(obs["imu_acc"], np.float32) * 0.05,
        np.clip(np.asarray(obs["contact"], np.float32), 0.0, 50.0) * 0.02,
    ]).astype(np.float32)


class TomcatStand(gym.Env):
    """Stand up, on the board's sensors, at 133 Hz with 7.5 ms of delay.

    Action is in [-1, 1] and scales to cable tension. ⚠️ **Pull-only is a
    property of a cable**, but ADR-0008's pulley makes the pair's command a
    signed *differential*, so the action is signed too — that is the
    transmission, not a modelling shortcut.
    """

    metadata = {"render_modes": []}

    def __init__(self, *, episode_s: float = 4.0, randomize=None,
                 seed: int | None = None, residual_n: float | None = None, **kw):
        """`residual_n` (M135, ADR-0129): when set, the action is a RESIDUAL
        of +-`residual_n` newtons on `stand_baseline.StandBaseline`, the
        sensor-only standing controller -- not the whole tension."""
        super().__init__()
        rng = np.random.default_rng(seed)
        self.core = TomcatEnv(rng=rng, randomize=randomize, **kw)
        self.max_steps = int(round(episode_s * self.core.control_hz))
        self.action_space = spaces.Box(-1.0, 1.0, (18,), np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (64,), np.float32)
        self.residual_n = residual_n
        self.base = None
        if residual_n is not None:
            from .stand_baseline import StandBaseline
            self.base = StandBaseline(self.core)
        self._obs = None

    def prior(self, obs) -> np.ndarray:
        """Extra tension a task adds to the baseline (M148); none here."""
        return np.zeros(18)

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.core.rng = np.random.default_rng(seed)
        obs = self.core.reset()
        if self.base is not None:
            self.base.reset(obs)
        self._obs = obs
        return flatten(obs), {}

    def step(self, action):
        a = np.asarray(action, float)
        if self.base is None:
            tension = a * MT.TENSION_MAX
        else:
            tension = (self.base(self._obs) + self.prior(self._obs)
                       + np.clip(a, -1.0, 1.0) * self.residual_n)
        obs, reward, terminated, info = self.core.step(tension)
        self._obs = obs
        truncated = self.core.steps >= self.max_steps
        return flatten(obs), float(reward), bool(terminated), truncated, info


class TomcatWalk(TomcatStand):
    """`warp_env.WarpWalk` for one robot on the CPU (M137, ADR-0131): the
    standing task's 64 channels plus a gait clock and the commanded speed;
    the reward tracks the TRUE forward speed (a training-only quantity)."""

    def __init__(self, *, v_cmd: float = 0.5, period: float = 0.4,
                 residual_n: float = 80.0, act_obs: bool = False,
                 w_cmd=None, yaw_prior=None, **kw):
        super().__init__(residual_n=residual_n, **kw)
        self.v_cmd = float(v_cmd)
        self.period = float(period)
        self.act_obs = bool(act_obs)                      # M144, as WarpWalk
        self.w_cmd = None if w_cmd is None else float(w_cmd)   # M147
        self.yaw_prior = None if yaw_prior is None else tuple(float(x) for x in yaw_prior)
        self.last_a = np.zeros(18, np.float32)
        self.observation_space = spaces.Box(
            -np.inf, np.inf, ((85 if act_obs else 67) + (w_cmd is not None),), np.float32)

    def prior(self, obs) -> np.ndarray:
        """M148: `WarpWalk.yaw_split` for one robot."""
        T = np.zeros(18)
        if self.yaw_prior is not None and self.w_cmd is not None:
            kff, kfb, lim = self.yaw_prior
            gz = float(np.asarray(obs["imu_gyro"])[2])
            sp = -float(np.clip(kff * self.w_cmd + kfb * (self.w_cmd - gz), -lim, lim))
            T[[0, 6]] += sp
            T[[3, 9]] -= sp
        return T

    def _extra(self):
        ph = 2.0 * np.pi * ((self.core.steps / self.core.control_hz) / self.period % 1.0)
        e = np.array([np.sin(ph), np.cos(ph), self.v_cmd], np.float32)
        if self.w_cmd is not None:
            e = np.concatenate([e, np.array([self.w_cmd], np.float32)])
        return np.concatenate([e, self.last_a]) if self.act_obs else e

    def reset(self, *, seed=None, options=None):
        self.last_a = np.zeros(18, np.float32)
        o, info = super().reset(seed=seed, options=options)
        return np.concatenate([o, self._extra()]), info

    def step(self, action):
        self.last_a = np.clip(np.asarray(action, np.float32), -1.0, 1.0)
        o, _r, term, trunc, info = super().step(action)
        info["vx"] = float(self.core.data.qvel[0])
        return np.concatenate([o, self._extra()]), 0.0, term, trunc, info
