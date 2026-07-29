"""Tests for the combined whole-body static tendon/torque budget."""

import numpy as np
import pytest

from tomcat_kin import (
    WholeBody,
    SpineModel,
    SpineParams,
    LegModel,
    LegParams,
    TendonMap,
    ActuationMode,
    WholeBodyLoadCase,
    whole_body_budget,
    spine_joint_torques,
    gravity_loads,
)
from tomcat_kin.params import (
    DEFAULT_SPINE,
    DEFAULT_WHOLE_BODY_LOADS,
    GRAVITY,
)


def _body():
    return WholeBody(spine=SpineModel())


def _symmetric_body():
    """A deliberately fore/aft-SYMMETRIC body (uniform spine, equal masses).

    Used to check the balance physics of the cantilever model in isolation: with
    a symmetric mass distribution and a symmetric 4-leg stance the base joint
    torque must cancel EXACTLY. The real default body is ~60% front-heavy.
    """
    uniform = SpineParams(
        segment_lengths=(0.06, 0.06, 0.06),
        segment_mass=(0.4, 0.4, 0.4),
        segment_com_frac=(0.5, 0.5, 0.5),
        front_girdle_mass=0.5,
        rear_girdle_mass=0.5,
    )
    sym_leg = LegModel(LegParams(link_mass=(0.05, 0.05, 0.05, 0.05)))
    return WholeBody(
        spine=SpineModel(params=uniform), fore_leg=sym_leg, hind_leg=sym_leg
    )


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
    # The exact base-joint cancellation (front reaction vs. distributed gravity
    # about the base) holds only for a body that is symmetric fore/aft: UNIFORM
    # segment spacing, EQUAL segment masses, EQUAL girdle masses and EQUAL
    # fore/hind leg masses. Build exactly that so the balance physics is checked
    # explicitly. (Since M4 the DEFAULT body is deliberately ~60% front-heavy and
    # therefore gives a decidedly non-zero base torque -- see the next test.)
    body = _symmetric_body()
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


# ------------------------------------------------- M4: real distributed mass
def test_gravity_loads_total_the_case_body_weight():
    body = _body()
    load = DEFAULT_WHOLE_BODY_LOADS[0]
    loads = gravity_loads(body, load)
    total = -sum(fz for _, _, fz in loads)
    assert total == pytest.approx(load.body_mass_kg * GRAVITY, rel=1e-12)
    # Every gravity load points DOWN and carries no dynamic factor.
    assert all(fz < 0.0 for _, _, fz in loads)


def test_gravity_load_total_scales_with_case_body_mass():
    body = _body()
    heavy = WholeBodyLoadCase("heavy", body_mass_kg=6.0, spine_q=(0.0, 0.0, 0.0))
    total = -sum(fz for _, _, fz in gravity_loads(body, heavy))
    assert total == pytest.approx(6.0 * GRAVITY, rel=1e-12)


def test_near_balanced_body_barely_loads_the_base_joint_in_quiet_stand():
    # Consequence of review finding F2. When the girdle masses were TUNED to a
    # 60/40 front-heavy split, the base joint carried a large cantilever moment
    # (>0.3 N·m). With the motors in their real clusters the body is near-balanced
    # (51/49), so in a symmetric 4-leg stand the base joint carries very little --
    # it approaches the exactly-symmetric body's zero.
    load = WholeBodyLoadCase(
        "stand", dynamic_factor=1.0, spine_q=(0.0, 0.0, 0.0),
        stance_legs=("LF", "RF", "LR", "RR"),
    )
    real = abs(spine_joint_torques(_body(), load)[0])
    symmetric = abs(spine_joint_torques(_symmetric_body(), load)[0])
    assert symmetric == pytest.approx(0.0, abs=1e-9)
    assert real < 0.15          # N·m -- small, not the 0.57 of the tuned model


def test_asymmetric_land_still_makes_the_base_joint_the_worst():
    # Balance removes the *gravity* cantilever in quiet stand, but a single-front-
    # leg landing is inherently asymmetric: the whole body hangs off the base
    # joint, which remains by far the worst spine joint.
    land = DEFAULT_WHOLE_BODY_LOADS[2]
    tau = np.abs(spine_joint_torques(_body(), land))
    assert np.argmax(tau) == 0
    assert tau[0] > 3.0 * tau[2]


def test_leg_q_places_leg_weight_at_the_posed_com():
    body = _body()
    load = DEFAULT_WHOLE_BODY_LOADS[0]
    stand = np.deg2rad([-80.0, 60.0, 10.0])
    lumped = spine_joint_torques(body, load)
    posed = spine_joint_torques(body, load, leg_q={n: stand for n in body.leg_names})
    # The placeholder stance rakes each leg FORWARD of its hip, so putting the
    # leg mass at its true CoM adds a nose-down moment at every joint.
    assert not np.allclose(lumped, posed)
    assert np.all(posed <= lumped + 1e-12)
    # Total weight is unchanged -- only its lever arms moved.
    assert -sum(fz for _, _, fz in gravity_loads(body, load)) == pytest.approx(
        -sum(fz for _, _, fz in gravity_loads(
            body, load, leg_q={n: stand for n in body.leg_names})),
        rel=1e-12,
    )


def test_report_states_the_mass_model_and_the_split():
    body = _body()
    leg_t, spine_t = _tendons()
    res = whole_body_budget.evaluate(body, leg_t, spine_t, DEFAULT_WHOLE_BODY_LOADS[0])
    txt = res.report()
    assert "REAL distributed" in txt
    assert res.mass_total_kg == pytest.approx(3.0)
    assert res.mass_fore_fraction == pytest.approx(0.51, abs=0.02)


def test_stand_case_spine_tension_is_not_excessive():
    # Sanity-check against the ~20-70 N RoboCat band. Balancing the body (F2)
    # DROPPED quiet-stand spine tension to ~12 N -- now BELOW the band. That is a
    # good outcome (less continuous tendon load), so the check is a ceiling, not
    # a window; the floor is just the pretension the cable never goes under.
    body = _body()
    leg_t, spine_t = _tendons()
    res = whole_body_budget.evaluate(body, leg_t, spine_t, DEFAULT_WHOLE_BODY_LOADS[0])
    assert res.peak_spine_tension <= 70.0
    assert res.peak_spine_tension >= spine_t.params.pretension


def test_wrong_spine_q_length_raises():
    body = _body()
    bad = WholeBodyLoadCase(
        "bad", spine_q=(0.0, 0.0), stance_legs=("LF",)
    )
    with pytest.raises(ValueError):
        spine_joint_torques(body, bad)
