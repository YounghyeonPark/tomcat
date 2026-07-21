"""Tests for the planar 3R leg kinematics."""

import math

import numpy as np
import pytest

from tomcat_kin import LegModel, KneeConfig, UnreachableError


leg = LegModel()


@pytest.mark.parametrize(
    "q_deg",
    [
        [30.0, -60.0, 15.0],
        [0.0, -90.0, 0.0],
        [-20.0, -40.0, -10.0],
        [45.0, -30.0, 5.0],
    ],
)
def test_fk_ik_roundtrip(q_deg):
    q = np.deg2rad(q_deg)
    pose = leg.forward(q)
    # Pick the branch matching the original knee sign so we recover the input.
    knee = KneeConfig.FLEXED_NEGATIVE if q[1] < 0 else KneeConfig.FLEXED_POSITIVE
    q_rec = leg.inverse(pose, knee=knee)
    assert np.allclose(leg.forward(q_rec), pose, atol=1e-9)
    assert np.allclose(q, q_rec, atol=1e-9)


def test_straight_leg_reach():
    # All angles zero -> foot tip on +x at full reach.
    pose = leg.forward([0.0, 0.0, 0.0])
    assert pose[0] == pytest.approx(leg.params.reach)
    assert pose[1] == pytest.approx(0.0, abs=1e-12)


def test_unreachable_raises():
    with pytest.raises(UnreachableError):
        leg.inverse((10.0, 0.0, 0.0))  # far beyond reach


def test_joint_positions_chain_lengths():
    pts = leg.joint_positions(np.deg2rad([20.0, -50.0, 10.0]))
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    assert seg == pytest.approx([leg.params.l1, leg.params.l2, leg.params.l3])


def test_jacobian_matches_finite_difference():
    q = np.deg2rad([25.0, -55.0, 12.0])
    J = leg.jacobian(q)
    eps = 1e-6
    J_fd = np.zeros((3, 3))
    for j in range(3):
        dq = np.zeros(3)
        dq[j] = eps
        J_fd[:, j] = (leg.forward(q + dq) - leg.forward(q - dq)) / (2 * eps)
    assert np.allclose(J, J_fd, atol=1e-6)


def test_phi_equals_angle_sum():
    q = np.deg2rad([10.0, -30.0, 20.0])
    pose = leg.forward(q)
    assert pose[2] == pytest.approx(sum(q))
