# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Tests for the digitigrade 4-link leg kinematics (3 actuated joints + paw)."""

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
    # FK returns the PAW-TIP pose; IK backs out the rigid passive paw and
    # recovers the actuated (q1, q2, q3).
    q = np.deg2rad(q_deg)
    pose = leg.forward(q)
    # Pick the branch matching the original knee sign so we recover the input.
    knee = KneeConfig.FLEXED_NEGATIVE if q[1] < 0 else KneeConfig.FLEXED_POSITIVE
    q_rec = leg.inverse(pose, knee=knee)
    assert np.allclose(leg.forward(q_rec), pose, atol=1e-9)
    assert np.allclose(q, q_rec, atol=1e-9)


def test_reach_is_sum_of_all_four_links():
    # Straight-leg reach spans all four links: femur + tibia + metatarsus + paw.
    p = leg.params
    assert leg.params.reach == pytest.approx(p.l1 + p.l2 + p.l3 + p.l4)


def test_unreachable_raises():
    with pytest.raises(UnreachableError):
        leg.inverse((10.0, 0.0, 0.0))  # far beyond reach


def test_joint_positions_chain_lengths():
    # Five points now: hip, stifle, hock, paw-base, paw-tip -> four link lengths.
    pts = leg.joint_positions(np.deg2rad([20.0, -50.0, 10.0]))
    assert pts.shape == (5, 2)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    p = leg.params
    assert seg == pytest.approx([p.l1, p.l2, p.l3, p.l4])


def test_joint_positions_last_point_is_forward_paw_tip():
    # The last chain point (paw-tip) must equal the FK ground-contact position.
    q = np.deg2rad([15.0, -45.0, 20.0])
    pts = leg.joint_positions(q)
    assert np.allclose(pts[-1], leg.forward(q)[:2])


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


def test_phi_is_paw_tip_orientation():
    # Paw-tip pitch = actuated angle sum + the fixed passive paw offset.
    q = np.deg2rad([10.0, -30.0, 20.0])
    pose = leg.forward(q)
    assert pose[2] == pytest.approx(sum(q) + leg.params.paw_angle)


@pytest.mark.parametrize(
    "q_deg",
    [
        [30.0, -60.0, 15.0],
        [-20.0, -40.0, -10.0],
        [45.0, -30.0, 5.0],
        [10.0, -80.0, 25.0],
    ],
)
def test_paw_holds_fixed_rest_angle_across_ik(q_deg):
    # The passive paw must sit at EXACTLY paw_angle relative to the metatarsus for
    # every IK solution: metatarsus direction is the actuated angle sum, paw-tip
    # pitch is that sum + paw_angle.
    q = np.deg2rad(q_deg)
    pose = leg.forward(q)
    knee = KneeConfig.FLEXED_NEGATIVE if q[1] < 0 else KneeConfig.FLEXED_POSITIVE
    q_rec = leg.inverse(pose, knee=knee)
    metatarsus_angle = float(np.sum(q_rec))
    paw_tip_pitch = leg.forward(q_rec)[2]
    assert (paw_tip_pitch - metatarsus_angle) == pytest.approx(
        leg.params.paw_angle
    )
    # ...and the paw-base -> paw-tip segment points along that paw direction.
    pts = leg.joint_positions(q_rec)
    paw_vec = pts[-1] - pts[-2]
    assert math.atan2(paw_vec[1], paw_vec[0]) == pytest.approx(
        metatarsus_angle + leg.params.paw_angle
    )
