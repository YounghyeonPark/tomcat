"""Tests for the moment-arm / cable-tension sensitivity tool (ADR-0003)."""

import numpy as np
import pytest

from tomcat_kin import moment_arm_sweep, ROBOCAT_BAND_N
from tomcat_kin.sensitivity import moment_arm_sweep as _sweep


def test_joint_tension_matches_aic_relation():
    # T_joint = T_bias + |tau| / r, same relation the TendonMap uses.
    tau = 12.6
    arms = [0.012, 0.025, 0.030]
    res = moment_arm_sweep(tau, arms, t_bias=5.0)
    expected = 5.0 + tau / np.asarray(arms)
    assert np.allclose(res.joint_tension_N, expected)


def test_tension_decreases_with_larger_arm():
    res = moment_arm_sweep(12.6, [0.010, 0.020, 0.030, 0.050], t_bias=5.0)
    # Bigger pulley -> lower peak tension, strictly monotonic.
    assert np.all(np.diff(res.joint_tension_N) < 0.0)


def test_frictionless_motor_equals_joint():
    res = moment_arm_sweep(12.6, [0.015, 0.030], t_bias=5.0)
    assert res.capstan_factor == pytest.approx(1.0)
    assert np.allclose(res.motor_tension_N, res.joint_tension_N)


def test_friction_amplifies_motor_side_only():
    arms = [0.015, 0.030, 0.050]
    tau = 12.6
    dry = moment_arm_sweep(tau, arms, t_bias=5.0)
    wet = moment_arm_sweep(tau, arms, t_bias=5.0, friction_coeff=0.3, wrap_angle=np.pi)
    factor = float(np.exp(0.3 * np.pi))
    assert wet.capstan_factor == pytest.approx(factor)
    # Joint-side unchanged; motor-side scaled by the capstan factor (> 1).
    assert np.allclose(wet.joint_tension_N, dry.joint_tension_N)
    assert np.allclose(wet.motor_tension_N, dry.joint_tension_N * factor)
    assert np.all(wet.motor_tension_N > dry.motor_tension_N)


def test_arm_for_band_top_finds_smallest_arm_in_band():
    # Only the 0.20 m arm brings the joint-side tension <= 70 N for this torque.
    arms = [0.015, 0.030, 0.050, 0.100, 0.150, 0.200]
    res = moment_arm_sweep(12.8, arms, t_bias=5.0, band_N=(20.0, 70.0))
    arm = res.arm_for_band_top(side="joint")
    assert arm == pytest.approx(0.200)
    # Small placeholder arms are far above the band top.
    assert res.joint_tension_N[0] > 70.0


def test_arm_for_band_none_when_all_too_stiff():
    res = moment_arm_sweep(12.8, [0.010, 0.015], t_bias=5.0, band_N=(20.0, 70.0))
    assert res.arm_for_band_top(side="joint") is None


def test_friction_needs_larger_arm_than_frictionless_on_motor_side():
    arms = [0.050, 0.100, 0.150, 0.200, 0.300, 0.500, 0.700, 1.000]
    tau = 12.8
    dry = moment_arm_sweep(tau, arms, t_bias=5.0)
    wet = moment_arm_sweep(tau, arms, t_bias=5.0, friction_coeff=0.3, wrap_angle=np.pi)
    # Friction pushes the motor-side band crossing out to a larger pulley.
    assert wet.arm_for_band_top(side="motor") > dry.arm_for_band_top(side="motor")


def test_report_is_stringable_and_mentions_band():
    res = moment_arm_sweep(12.8, [0.015, 0.030, 0.200], t_bias=5.0)
    txt = res.report()
    assert "RoboCat" in txt
    assert "T_joint" in txt and "T_motor" in txt


def test_default_band_is_robocat():
    res = moment_arm_sweep(12.8, [0.030])
    assert res.band_N == ROBOCAT_BAND_N == (20.0, 70.0)


def test_bad_inputs_rejected():
    with pytest.raises(ValueError):
        _sweep(12.8, [])
    with pytest.raises(ValueError):
        _sweep(12.8, [0.0, 0.03])
    with pytest.raises(ValueError):
        _sweep(12.8, [0.03], friction_coeff=-0.1)
