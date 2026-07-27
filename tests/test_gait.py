"""Tests for the periodic WALK gait generator (M2)."""

import math

import numpy as np
import pytest

from tomcat_kin import (
    GaitParams,
    GaitController,
    foot_target,
    swing_height,
    KneeConfig,
)
from tomcat_kin.gait import DEFAULT_PHASE_OFFSETS


ctrl = GaitController()
params = ctrl.params


# --------------------------------------------------------------- gait params
def test_default_is_walk_with_offset_set():
    # Four legs, offsets are exactly {0, .25, .5, .75}.
    assert set(params.phase_offsets) == {"LF", "RF", "RR", "LR"}
    assert sorted(params.phase_offsets.values()) == pytest.approx([0.0, 0.25, 0.5, 0.75])


def test_duty_and_speed_relationship():
    p = GaitParams()
    assert p.swing_fraction == pytest.approx(1.0 - p.duty_factor)
    assert p.body_speed == pytest.approx(p.stride_length / (p.duty_factor * p.period))
    assert p.distance_per_cycle == pytest.approx(p.stride_length / p.duty_factor)


def test_invalid_params_raise():
    with pytest.raises(ValueError):
        GaitParams(duty_factor=1.0)
    with pytest.raises(ValueError):
        GaitParams(period=0.0)
    with pytest.raises(ValueError):
        GaitParams(phase_offsets={"LF": 1.0})


# ------------------------------------------------------------- stance counting
@pytest.mark.parametrize("phase", np.linspace(0.0, 1.0, 41))
def test_exactly_three_legs_in_stance(phase):
    # duty 0.75 with tiled offsets => always exactly 3 planted, 1 swinging.
    st = ctrl.state(phase)
    assert st.stance_count == 3
    assert len(st.swing_legs) == 1


def test_stance_count_matches_duty_expectation():
    # Averaged over the cycle the mean stance count == 4 * duty_factor.
    counts = [ctrl.state(p).stance_count for p in np.linspace(0, 1, 200, endpoint=False)]
    assert np.mean(counts) == pytest.approx(4 * params.duty_factor, abs=1e-9)


# ---------------------------------------------------- one stance+swing per cycle
@pytest.mark.parametrize("leg", ["LF", "RF", "RR", "LR"])
def test_one_stance_and_one_swing_block_per_cycle(leg):
    n = 360
    stance = np.array([ctrl.is_stance(i / n, leg) for i in range(n)])
    # Stance fraction ~ duty_factor.
    assert stance.mean() == pytest.approx(params.duty_factor, abs=1.5 / n)
    # Exactly one contiguous swing block (one falling + one rising edge on the
    # circular sequence).
    swing = ~stance
    edges = np.sum(swing != np.roll(swing, 1))
    assert edges == 2


# --------------------------------------------------------------- foot trajectory
@pytest.mark.parametrize("leg", ["LF", "RF", "RR", "LR"])
def test_swing_height_zero_at_touchdown_and_liftoff(leg):
    off = params.phase_offsets[leg]
    d = params.duty_factor
    # Liftoff is at local phase == d, touchdown at local phase == 0 (== 1).
    liftoff_phase = (off + d) % 1.0
    touchdown_phase = off % 1.0
    assert swing_height(params, ctrl.local_phase(liftoff_phase, leg)) == pytest.approx(0.0, abs=1e-9)
    assert swing_height(params, ctrl.local_phase(touchdown_phase, leg)) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("leg", ["LF", "RF", "RR", "LR"])
def test_swing_foot_lifts_mid_swing(leg):
    off = params.phase_offsets[leg]
    d = params.duty_factor
    mid_swing_phase = (off + d + 0.5 * (1.0 - d)) % 1.0
    h = swing_height(params, ctrl.local_phase(mid_swing_phase, leg))
    assert h == pytest.approx(params.step_height, rel=1e-6)
    assert h > 0.0


def test_stance_foot_stays_on_ground_and_sweeps_backward():
    # Over stance the foot z holds at nominal and x decreases monotonically.
    d = params.duty_factor
    us = np.linspace(0.0, d, 30, endpoint=False)
    xs = [foot_target(params, s)[0] for s in us]
    zs = [foot_target(params, s)[1] for s in us]
    assert np.allclose(zs, params.nominal_foot[1])
    assert np.all(np.diff(xs) < 0)  # moving backward
    # Sweep spans the full stride, front (+half) to rear (-half).
    assert xs[0] == pytest.approx(params.nominal_foot[0] + params.stride_length / 2)


def test_swing_returns_foot_forward():
    d = params.duty_factor
    x_liftoff = foot_target(params, d)[0]
    x_touchdown = foot_target(params, 1.0 - 1e-9)[0]
    assert x_touchdown > x_liftoff  # net forward return over swing


# ------------------------------------------------------------------- IK & limits
def test_default_gait_all_reachable_and_within_limits():
    for i in range(120):
        st = ctrl.state(i / 120)
        assert st.all_ok, f"phase {i/120:.3f} not ok"
        for leg in st.legs.values():
            assert leg.q is not None
            assert ctrl.body.legs[leg.name].in_limits(leg.q)


def test_ik_solution_reproduces_foot_target():
    st = ctrl.state(0.3)
    for leg in st.legs.values():
        fk = ctrl.body.legs[leg.name].forward(leg.q)
        assert np.allclose(fk, leg.foot_target, atol=1e-9)


def test_unreachable_target_flagged_not_raised():
    # A gait whose foot targets are far outside the workspace must be flagged.
    bad = GaitParams(nominal_foot=(5.0, -0.13))
    c = GaitController(params=bad)
    st = c.state(0.1)
    assert not st.all_ok
    assert any((not lg.reachable) for lg in st.legs.values())
    # Flagged, not crashed: q is None for the unreachable legs.
    assert any(lg.q is None for lg in st.legs.values())


# ------------------------------------------------------------------- phase wrap
def test_phase_wraps_at_one():
    a = ctrl.state(0.0)
    b = ctrl.state(1.0)
    assert a.phase == pytest.approx(b.phase)
    for name in a.legs:
        assert np.allclose(a.legs[name].foot_target, b.legs[name].foot_target)
        assert a.legs[name].in_stance == b.legs[name].in_stance


def test_state_at_time_matches_phase():
    p = params
    t = 0.37 * p.period
    st_t = ctrl.state_at_time(t)
    st_p = ctrl.state(0.37)
    assert st_t.phase == pytest.approx(st_p.phase)


def test_time_beyond_period_wraps():
    st = ctrl.state_at_time(1.25 * params.period)
    assert st.phase == pytest.approx(0.25)


# ------------------------------------------------------------- spine coupling
def test_spine_neutral_by_default():
    st = ctrl.state(0.42)
    assert np.allclose(st.spine_q, 0.0)


def test_spine_oscillation_when_enabled():
    p = GaitParams(spine_amplitude=math.radians(5.0))
    c = GaitController(params=p)
    n = ctrl.body.spine.params.n_segments
    # Non-trivial and bounded by amplitude; zero-mean over the cycle.
    samples = np.array([c.spine_q(i / 100) for i in range(100)])
    assert samples.shape == (100, n)
    assert np.abs(samples).max() > 0.0
    assert np.abs(samples).max() <= math.radians(5.0) + 1e-12
    assert np.allclose(samples.mean(axis=0), 0.0, atol=1e-9)
    # And it stays within spine joint limits.
    lo = np.asarray(c.body.spine.params.q_min)
    hi = np.asarray(c.body.spine.params.q_max)
    assert np.all(samples >= lo - 1e-12) and np.all(samples <= hi + 1e-12)
