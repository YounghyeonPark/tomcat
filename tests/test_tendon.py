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
