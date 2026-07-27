"""Tests for the combined whole-body static tendon/torque budget."""

import numpy as np
import pytest

from tomcat_kin import (
    WholeBody,
    SpineModel,
    TendonMap,
    ActuationMode,
    WholeBodyLoadCase,
    whole_body_budget,
    spine_joint_torques,
)
from tomcat_kin.params import DEFAULT_SPINE, DEFAULT_WHOLE_BODY_LOADS


def _body():
    return WholeBody(spine=SpineModel())


def _tendons():
    leg_t = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    spine_t = TendonMap.from_spine(DEFAULT_SPINE, mode=ActuationMode.ANTAGONISTIC)
    return leg_t, spine_t


# ------------------------------------------------------------- motor arithmetic
def test_total_motor_count_is_twice_total_dof():
    body = _body()
    leg_t, spine_t = _tendons()
    load = DEFAULT_WHOLE_BODY_LOADS[0]
    res = whole_body_budget.evaluate(body, leg_t, spine_t, load)
    # 4 legs x 3 joints x 2 = 24; spine 3 joints x 2 = 6.
    assert res.n_leg_motors == 4 * 3 * 2
    assert res.n_spine_motors == DEFAULT_SPINE.n_segments * 2
    assert res.n_total_motors == res.n_leg_motors + res.n_spine_motors
    assert res.n_total_motors == 30


def test_all_default_cases_run_and_report():
    body = _body()
    leg_t, spine_t = _tendons()
    for load in DEFAULT_WHOLE_BODY_LOADS:
        res = whole_body_budget.evaluate(body, leg_t, spine_t, load)
        txt = res.report()
        assert load.name in txt
        assert res.spine_tension.shape == (DEFAULT_SPINE.n_segments,)
        assert res.leg.peak_tension.shape == (3,)


# ------------------------------------------------------------------ spine model
def test_straight_spine_base_joint_symmetric_zero():
    # In the balanced straight stand the base joint torque works out to ~0
    # (front reaction and distributed gravity balance about the base).
    body = _body()
    load = WholeBodyLoadCase(
        "stand", dynamic_factor=1.0, spine_q=(0.0, 0.0, 0.0),
        stance_legs=("LF", "RF", "LR", "RR"),
    )
    tau = spine_joint_torques(body, load)
    assert tau[0] == pytest.approx(0.0, abs=1e-9)


def test_rear_girdle_reaction_does_not_load_spine():
    # Cantilever-from-rear: a rear-only stance (rear legs planted) applies its
    # reaction at the base vertebra, which loads no spine joint. Only the
    # distributed gravity moment remains.
    body = _body()
    rear_only = WholeBodyLoadCase(
        "rear-only", dynamic_factor=1.0, spine_q=(0.0, 0.0, 0.0),
        stance_legs=("LR", "RR"),
    )
    no_legs = WholeBodyLoadCase(
        "no-stance", dynamic_factor=1.0, spine_q=(0.0, 0.0, 0.0),
        stance_legs=(),
    )
    assert np.allclose(
        spine_joint_torques(body, rear_only),
        spine_joint_torques(body, no_legs),
    )


def test_spine_load_monotonic_in_body_mass():
    body = _body()
    leg_t, spine_t = _tendons()
    light = WholeBodyLoadCase(
        "land-light", body_mass_kg=2.0, dynamic_factor=2.5,
        spine_q=(0.0, 0.0, 0.0), stance_legs=("LF",),
    )
    heavy = WholeBodyLoadCase(
        "land-heavy", body_mass_kg=6.0, dynamic_factor=2.5,
        spine_q=(0.0, 0.0, 0.0), stance_legs=("LF",),
    )
    r_light = whole_body_budget.evaluate(body, leg_t, spine_t, light)
    r_heavy = whole_body_budget.evaluate(body, leg_t, spine_t, heavy)
    # Heavier body -> higher spine tension on every loaded joint.
    assert r_heavy.peak_spine_tension > r_light.peak_spine_tension
    assert r_heavy.peak_leg_tension > r_light.peak_leg_tension


def test_spine_load_monotonic_in_dynamic_factor():
    body = _body()
    leg_t, spine_t = _tendons()
    soft = WholeBodyLoadCase(
        "land-soft", dynamic_factor=1.5, spine_q=(0.0, 0.0, 0.0),
        stance_legs=("LF",),
    )
    hard = WholeBodyLoadCase(
        "land-hard", dynamic_factor=3.0, spine_q=(0.0, 0.0, 0.0),
        stance_legs=("LF",),
    )
    r_soft = whole_body_budget.evaluate(body, leg_t, spine_t, soft)
    r_hard = whole_body_budget.evaluate(body, leg_t, spine_t, hard)
    assert r_hard.peak_spine_tension > r_soft.peak_spine_tension


def test_t_bias_raises_spine_tension_not_torque():
    body = _body()
    leg_t, spine_t = _tendons()
    load = DEFAULT_WHOLE_BODY_LOADS[2]  # land, non-zero spine torque
    low = whole_body_budget.evaluate(body, leg_t, spine_t, load, t_bias=5.0)
    high = whole_body_budget.evaluate(body, leg_t, spine_t, load, t_bias=25.0)
    assert high.peak_spine_tension > low.peak_spine_tension
    # The underlying static joint torques are unchanged by co-contraction.
    assert np.allclose(low.spine_joint_torque, high.spine_joint_torque)


def test_wrong_spine_q_length_raises():
    body = _body()
    bad = WholeBodyLoadCase(
        "bad", spine_q=(0.0, 0.0), stance_legs=("LF",)
    )
    with pytest.raises(ValueError):
        spine_joint_torques(body, bad)
