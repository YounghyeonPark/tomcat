"""M135 / ADR-0129: a standing controller on the board's sensors, and the
residual a policy learns on it."""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("mujoco")

from tomcat_kin.env import TomcatEnv  # noqa: E402
from tomcat_kin.stand_baseline import StandBaseline, nominal_hold  # noqa: E402

DR = {**TomcatEnv.RANGES, **TomcatEnv.SENSOR_RANGES}


def _episode(randomize=None, seed=0, seconds=4.0):
    env = TomcatEnv(rng=np.random.default_rng(seed), randomize=randomize)
    obs = env.reset()
    base = StandBaseline(env).reset(obs)
    n = int(round(seconds * env.control_hz))
    tilt = 0.0
    for i in range(n):
        obs, _r, done, info = env.step(base(obs))
        tilt = max(tilt, info["tilt_deg"])
        if done:
            return i + 1, tilt, n
    return n, tilt, n


def test_the_BASELINE_STANDS_on_sensors_alone():
    """✅ M135: the nominal plant stands the whole 4 s, under 10 deg of tilt.
    M81 found nothing standing longer than 15 steps."""
    steps, tilt, n = _episode()
    assert steps == n, f"fell at step {steps}"
    assert tilt < 10.0, f"peak tilt {tilt:.1f} deg"


def test_the_baseline_reads_NOTHING_PRIVILEGED():
    """The baseline gets the observation and nothing else: its call never
    touches the plant's state. Scramble `qpos` after the observation is taken
    and the command must not change."""
    env = TomcatEnv(rng=np.random.default_rng(0))
    obs = env.reset()
    base = StandBaseline(env).reset(obs)
    obs, *_ = env.step(base(obs))
    import copy
    b2 = copy.copy(base)
    a = base(obs)
    env.data.qpos[:] += 0.1
    env.data.qvel[:] += 1.0
    assert np.allclose(b2(obs), a)


def test_the_HOLD_is_the_NOMINAL_plant_s_not_the_draw():
    """The gravity hold is solved once on the nominal plant, so a randomised
    episode's baseline does not peek at what was drawn."""
    h = nominal_hold()
    env = TomcatEnv(rng=np.random.default_rng(3), randomize=DR)
    env.reset()
    assert np.allclose(StandBaseline(env).hold, h)
    assert np.all(np.abs(h[:12]) < 80.0) and np.all(h[12:] == 0.0)


def test_a_ZERO_residual_IS_the_baseline():
    from tomcat_kin.gym_env import TomcatStand
    g = TomcatStand(seed=0, residual_n=30.0)
    g.reset()
    n = 0
    while True:
        _o, _r, te, tr, _ = g.step(np.zeros(18))
        n += 1
        if te or tr:
            break
    assert tr and not te and n == g.max_steps


def test_the_FEET_FEEL_the_floor():
    """⚠️ M137 (ADR-0131): the touch sensors read 0 from M72 to M136 -- the
    2 mm site missed the 6 mm pad's contact point -- so the observation's
    contact channel and the reward's feet term were dead. Standing, all four
    must read, and together most of the weight. ⚠️ Measured 38.3 N against
    47.3 N, 81 %; a static stance should carry all of it, and why this
    does not is not yet run down."""
    env = TomcatEnv(rng=np.random.default_rng(0))
    obs = env.reset()
    c = np.asarray(obs["contact"])
    assert np.all(c > 1.0), c
    W = float(np.sum(env.model.body_mass)) * 9.81
    assert 0.6 * W < c.sum() < 1.2 * W, (c.sum(), W)
