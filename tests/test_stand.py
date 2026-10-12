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
    must read. Right after the reset fixture lets go they carry 81 % of the
    weight -- the robot is still settling -- and once it has stood 1.5 s on
    the baseline, all of it: 47.5 N against 47.3, equal to the contact
    solver's own normal forces (M153)."""
    env = TomcatEnv(rng=np.random.default_rng(0))
    obs = env.reset()
    c = np.asarray(obs["contact"])
    assert np.all(c > 1.0), c
    W = float(np.sum(env.model.body_mass)) * 9.81
    assert 0.6 * W < c.sum() < 1.2 * W, (c.sum(), W)
    base = StandBaseline(env).reset(obs)
    for _ in range(200):
        obs, *_ = env.step(base(obs))
    assert np.sum(obs["contact"]) == pytest.approx(W, rel=0.05), "standing carries it all"


def test_the_MOTOR_ENVELOPE_is_the_torque_speed_line_driving_and_free_braking():
    """M141 (ADR-0135): `wbc.envelope_command` holds a rotor servo to the spec
    motor's line: the full peak at stall, nothing driving at no-load, half at
    half speed -- and the full peak BRAKING at any speed."""
    from tomcat_kin import mjcf_tendon as MT
    from tomcat_kin import wbc
    kp, kv, pk, wn = 180.0, 0.12, MT.MOTOR_PEAK_NM, MT.MOTOR_NOLOAD_RAD_S
    om = np.array([0.0, wn, wn, 0.5 * wn, -0.5 * wn])
    cmd = np.array([10.0, 10.0, -10.0, 10.0, 10.0])          # all ask for too much
    f = kp * wbc.envelope_command(cmd, np.zeros(5), om, kp, kv, pk, wn) - kv * om
    assert np.allclose(f, [pk, 0.0, -pk, 0.5 * pk, pk], atol=1e-9)
    env = TomcatEnv(rng=np.random.default_rng(0))
    assert env.envelope, "the plant carries the line by default"


def test_the_CPU_env_draws_a_MOTOR_and_the_envelope_uses_it():
    """M143 (ADR-0137): a drawn motor reaches the envelope; undrawn, the
    spec motor stands."""
    from tomcat_kin import mjcf_tendon as MT
    env = TomcatEnv(rng=np.random.default_rng(2), randomize=TomcatEnv.MOTOR_RANGES)
    lo, hi = TomcatEnv.MOTOR_RANGES["motor_peak_nm"]
    assert lo <= env.motor_peak <= hi and env.motor_peak != MT.MOTOR_PEAK_NM
    assert env.drawn["motor_noload_rpm"] >= 380.0
    plain = TomcatEnv(rng=np.random.default_rng(2))
    assert plain.motor_peak == MT.MOTOR_PEAK_NM
