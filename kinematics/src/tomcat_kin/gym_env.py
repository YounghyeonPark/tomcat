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
                 seed: int | None = None, **kw):
        super().__init__()
        rng = np.random.default_rng(seed)
        self.core = TomcatEnv(rng=rng, randomize=randomize, **kw)
        self.max_steps = int(round(episode_s * self.core.control_hz))
        self.action_space = spaces.Box(-1.0, 1.0, (18,), np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (64,), np.float32)

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.core.rng = np.random.default_rng(seed)
        obs = self.core.reset()
        return flatten(obs), {}

    def step(self, action):
        tension = np.asarray(action, float) * MT.TENSION_MAX
        obs, reward, terminated, info = self.core.step(tension)
        truncated = self.core.steps >= self.max_steps
        return flatten(obs), float(reward), bool(terminated), truncated, info
