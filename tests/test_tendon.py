# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Tests for the tendon map (angle<->cable and torque<->tension)."""

import numpy as np
import pytest

from tomcat_kin import TendonMap, ActuationMode
from tomcat_kin.params import TendonParams


def test_antagonistic_realizes_requested_torque():
    tmap = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    tau = np.array([0.4, -0.6, 0.1])
    sol = tmap.resolve(tau)
    assert np.allclose(sol.joint_torque, tau, atol=1e-9)


def test_antagonistic_keeps_cables_taut():
    tmap = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    sol = tmap.resolve([0.4, -0.6, 0.0])
    pre = tmap.params.pretension
    # Neither tendon ever drops below the pretension floor.
    assert np.all(sol.tension_flexor >= pre - 1e-9)
    assert np.all(sol.tension_extensor >= pre - 1e-9)
    # The slack side sits exactly at the floor for a non-zero torque.
    assert sol.tension_extensor[0] == pytest.approx(pre)  # positive tau -> flexor pulls
    assert sol.tension_flexor[1] == pytest.approx(pre)    # negative tau -> extensor pulls


def test_antagonistic_needs_two_motors_per_active_joint():
    tmap = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    sol = tmap.resolve([0.4, -0.6, 0.1])
    # 3 flexor + 3 extensor tendons all carrying load -> 6 motors.
    assert sol.n_motors == 6


def test_zero_torque_is_pure_pretension():
    tmap = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    sol = tmap.resolve([0.0, 0.0, 0.0])
    pre = tmap.params.pretension
    assert np.allclose(sol.tension_flexor, pre)
    assert np.allclose(sol.tension_extensor, pre)
    assert np.allclose(sol.joint_torque, 0.0)


def test_tension_matches_moment_arm_relation():
    # For a positive torque, flexor = pre + tau/r, extensor = pre.
    tmap = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    r = np.asarray(tmap.params.joint_moment_arm)
    tau = np.array([0.3, 0.15, 0.06])
    sol = tmap.resolve(tau)
    expected_flex = tmap.params.pretension + tau / r
    assert np.allclose(sol.tension_flexor, expected_flex)


def test_motor_angle_scales_with_joint_angle():
    tmap = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    q = np.deg2rad([10.0, 20.0, 5.0])
    phi = tmap.motor_angles(q)
    r = np.asarray(tmap.params.joint_moment_arm)
    expected = r * q / tmap.params.motor_spool_radius
    assert np.allclose(phi, expected)


def test_cable_lengths_opposing_signs():
    tmap = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    q = np.deg2rad([15.0, 15.0, 15.0])
    lengths = tmap.cable_lengths(q)
    # flexor shortens (negative), extensor lengthens (positive)
    assert np.all(lengths[:, 0] < 0)
    assert np.all(lengths[:, 1] > 0)
    assert np.allclose(lengths[:, 0], -lengths[:, 1])


# --------------------------------------------------- commandable T_bias (AIC)
def test_default_t_bias_reproduces_pretension_numbers():
    # resolve(tau) and resolve(tau, t_bias=pretension) must be identical, and
    # match the fixed-pretension formula the earlier tests pin down.
    tmap = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    tau = np.array([0.4, -0.6, 0.1])
    a = tmap.resolve(tau)
    b = tmap.resolve(tau, t_bias=tmap.params.pretension)
    assert np.allclose(a.tension_flexor, b.tension_flexor)
    assert np.allclose(a.tension_extensor, b.tension_extensor)
    r = np.asarray(tmap.params.joint_moment_arm)
    pre = tmap.params.pretension
    dt = tau / r
    assert np.allclose(a.tension_flexor, np.where(dt >= 0, pre + dt, pre))


def test_higher_t_bias_raises_both_tensions_same_torque():
    tmap = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    tau = np.array([0.4, -0.6, 0.1])
    low = tmap.resolve(tau, t_bias=5.0)
    high = tmap.resolve(tau, t_bias=20.0)
    # Both tendons rise on every joint when the co-contraction bias rises.
    assert np.all(high.tension_flexor > low.tension_flexor)
    assert np.all(high.tension_extensor > low.tension_extensor)
    # ...both sides rise by exactly the bias increment (15 N).
    assert np.allclose(high.tension_flexor - low.tension_flexor, 15.0)
    assert np.allclose(high.tension_extensor - low.tension_extensor, 15.0)
    # Net realized joint torque is UNCHANGED by co-contraction.
    assert np.allclose(low.joint_torque, tau)
    assert np.allclose(high.joint_torque, tau)


def test_higher_t_bias_raises_peak_motor_torque():
    tmap = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    tau = np.array([0.4, -0.6, 0.1])
    low = tmap.resolve(tau, t_bias=5.0)
    high = tmap.resolve(tau, t_bias=20.0)
    assert np.all(np.abs(high.motor_torque) > np.abs(low.motor_torque))


def test_active_side_equals_bias_plus_torque_over_r():
    tmap = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    r = np.asarray(tmap.params.joint_moment_arm)
    tau = np.array([0.3, -0.2, 0.06])
    bias = 12.0
    sol = tmap.resolve(tau, t_bias=bias)
    active = np.maximum(sol.tension_flexor, sol.tension_extensor)
    slack = np.minimum(sol.tension_flexor, sol.tension_extensor)
    assert np.allclose(active, bias + np.abs(tau) / r)
    assert np.allclose(slack, bias)


def test_per_joint_t_bias_array():
    tmap = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    tau = np.array([0.3, 0.3, 0.3])
    bias = np.array([5.0, 10.0, 20.0])
    sol = tmap.resolve(tau, t_bias=bias)
    slack = np.minimum(sol.tension_flexor, sol.tension_extensor)
    assert np.allclose(slack, bias)
    # Torque still realized regardless of the per-joint bias.
    assert np.allclose(sol.joint_torque, tau)


def test_negative_t_bias_rejected():
    tmap = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    with pytest.raises(ValueError):
        tmap.resolve([0.1, 0.1, 0.1], t_bias=-1.0)


def test_spring_return_uses_single_motor_and_clamps():
    tmap = TendonMap(
        params=TendonParams(pretension=2.0),
        mode=ActuationMode.SPRING_RETURN,
    )
    sol = tmap.resolve([0.0, 0.0, 0.0])
    # No extensor tendons in spring-return mode.
    assert np.all(sol.tension_extensor == 0.0)
    assert sol.n_motors == 3
    # Tension never drops below the pretension floor (cable can't push).
    assert np.all(sol.tension_flexor >= tmap.params.pretension - 1e-9)


# ------------------------------------------------------- capstan friction (ADR-0003)
def test_frictionless_default_motor_equals_joint_tension():
    # mu=0 (default): motor-side tension == joint-side tension, factor == 1.
    tmap = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    assert tmap.capstan_factor() == pytest.approx(1.0)
    sol = tmap.resolve([0.4, -0.6, 0.1])
    assert np.allclose(sol.motor_tension_flexor, sol.tension_flexor)
    assert np.allclose(sol.motor_tension_extensor, sol.tension_extensor)


def test_zero_wrap_reduces_to_frictionless():
    # A non-zero mu with zero wrap still gives factor 1 (and vice versa).
    a = TendonMap(params=TendonParams(friction_coeff=0.5, wrap_angle=0.0))
    b = TendonMap(params=TendonParams(friction_coeff=0.0, wrap_angle=np.pi))
    assert a.capstan_factor() == pytest.approx(1.0)
    assert b.capstan_factor() == pytest.approx(1.0)
    base = TendonMap(mode=ActuationMode.ANTAGONISTIC).resolve([0.3, -0.2, 0.05])
    for tmap in (a, b):
        sol = tmap.resolve([0.3, -0.2, 0.05])
        assert np.allclose(sol.motor_tension_flexor, base.tension_flexor)
        assert np.allclose(sol.motor_torque, base.motor_torque)


def test_capstan_raises_motor_side_tension_and_torque():
    tau = np.array([0.4, -0.6, 0.1])
    frictionless = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    friction = TendonMap(
        params=TendonParams(friction_coeff=0.3, wrap_angle=np.pi),
        mode=ActuationMode.ANTAGONISTIC,
    )
    f0 = frictionless.resolve(tau)
    f1 = friction.resolve(tau)
    factor = friction.capstan_factor()
    assert factor > 1.0
    # Joint-side tension is unchanged by friction; motor-side is amplified.
    assert np.allclose(f1.tension_flexor, f0.tension_flexor)
    assert np.allclose(f1.motor_tension_flexor, f0.tension_flexor * factor)
    # Motor torque is sized from the motor-side tension, so it rises too.
    assert np.all(np.abs(f1.motor_torque) > np.abs(f0.motor_torque))


def test_higher_wrap_or_mu_raises_motor_tension_monotonically():
    tau = [0.5, -0.5, 0.2]
    low = TendonMap(params=TendonParams(friction_coeff=0.2, wrap_angle=np.pi)).resolve(tau)
    hi_mu = TendonMap(params=TendonParams(friction_coeff=0.4, wrap_angle=np.pi)).resolve(tau)
    hi_wrap = TendonMap(params=TendonParams(friction_coeff=0.2, wrap_angle=2 * np.pi)).resolve(tau)
    assert np.all(hi_mu.motor_tension_flexor > low.motor_tension_flexor)
    assert np.all(hi_wrap.motor_tension_flexor > low.motor_tension_flexor)


def test_pay_out_direction_reduces_tension():
    tmap = TendonMap(params=TendonParams(friction_coeff=0.3, wrap_angle=np.pi))
    assert tmap.capstan_factor(paying_out=True) < 1.0
    assert tmap.capstan_factor(paying_out=True) == pytest.approx(
        1.0 / tmap.capstan_factor()
    )


# --------------------------------------------------------- cable stretch (ADR-0003)
def test_inextensible_default_has_no_stretch():
    tmap = TendonMap(mode=ActuationMode.ANTAGONISTIC)  # k_cable=None
    T = np.array([100.0, 200.0, 50.0])
    assert np.allclose(tmap.cable_stretch(T), 0.0)
    assert np.allclose(tmap.extra_motor_angle(T), 0.0)
    assert np.allclose(tmap.joint_angle_error(T), 0.0)


def test_infinite_stiffness_has_no_stretch():
    tmap = TendonMap(params=TendonParams(k_cable=float("inf")))
    assert np.allclose(tmap.cable_stretch([100.0, 200.0, 50.0]), 0.0)


def test_stretch_scales_linearly_with_tension():
    k = 5.0e4
    tmap = TendonMap(params=TendonParams(k_cable=k))
    T = np.array([100.0, 200.0, 400.0])
    dl = tmap.cable_stretch(T)
    assert np.allclose(dl, T / k)
    # Doubling tension doubles both stretch and the extra motor angle.
    assert dl[1] == pytest.approx(2.0 * dl[0])
    assert np.allclose(
        tmap.extra_motor_angle(T), (T / k) / tmap.params.motor_spool_radius
    )
    # Uncompensated joint-angle error = dL / r (per-joint moment arm).
    r = np.asarray(tmap.params.joint_moment_arm)
    assert np.allclose(tmap.joint_angle_error(T), (T / k) / r)
