"""Tests for the articulated spine + whole-body kinematics."""

import math

import numpy as np
import pytest

from tomcat_kin import (
    SpineModel,
    WholeBody,
    Girdle,
    LegModel,
    SpineParams,
    TendonMap,
    ActuationMode,
)
from tomcat_kin.params import DEFAULT_SPINE, DEFAULT_FORELEG


spine = SpineModel()


# --------------------------------------------------------------------- FK sanity
def test_zero_angles_straight_spine():
    q = np.zeros(DEFAULT_SPINE.n_segments)
    poses = spine.vertebra_poses(q)
    # All vertebrae on the +x axis, orientation 0.
    assert np.allclose(poses[:, 1], 0.0)   # z
    assert np.allclose(poses[:, 2], 0.0)   # theta
    # Vertebra x positions are the running sum of segment lengths.
    expected_x = np.concatenate([[0.0], np.cumsum(DEFAULT_SPINE.segment_lengths)])
    assert np.allclose(poses[:, 0], expected_x)


def test_zero_angles_girdles_at_expected_offsets():
    q = np.zeros(DEFAULT_SPINE.n_segments)
    rear = spine.girdle_pose(q, Girdle.REAR)
    front = spine.girdle_pose(q, Girdle.FRONT)
    assert np.allclose(rear, [0.0, 0.0, 0.0])
    assert np.allclose(front, [DEFAULT_SPINE.total_length, 0.0, 0.0])


def test_segment_lengths_preserved():
    q = np.deg2rad([15.0, -10.0, 20.0])
    pts = spine.vertebra_positions(q)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    assert seg == pytest.approx(DEFAULT_SPINE.segment_lengths)


def test_known_bend_matches_independent_fk():
    q = np.array([0.10, 0.20, -0.05])
    lengths = DEFAULT_SPINE.segment_lengths
    ang, x, z = 0.0, 0.0, 0.0
    expected = [(0.0, 0.0, 0.0)]
    for i in range(len(q)):
        ang += q[i]
        x += lengths[i] * math.cos(ang)
        z += lengths[i] * math.sin(ang)
        expected.append((x, z, ang))
    assert np.allclose(spine.vertebra_poses(q), np.array(expected))


def test_uniform_positive_bend_arches_dorsally():
    # Uniform positive (dorsiflexion) bend curls the front girdle up (+z) and
    # rotates its frame CCW.
    q = np.full(DEFAULT_SPINE.n_segments, np.deg2rad(20.0))
    front = spine.girdle_pose(q, Girdle.FRONT)
    assert front[1] > 0.0   # z lifted
    assert front[2] == pytest.approx(sum(q))  # orientation = angle sum


def test_in_limits():
    assert spine.in_limits(np.zeros(DEFAULT_SPINE.n_segments))
    over = np.array(DEFAULT_SPINE.q_max) + 0.1
    assert not spine.in_limits(over)


def test_wrong_q_length_raises():
    with pytest.raises(ValueError):
        spine.vertebra_poses(np.zeros(DEFAULT_SPINE.n_segments + 1))


def test_hip_offset_applied_in_girdle_frame():
    # With a bent spine, a hip offset must be rotated by the girdle orientation.
    q = np.full(DEFAULT_SPINE.n_segments, np.deg2rad(30.0))
    gx, gz, gth = spine.girdle_pose(q, Girdle.FRONT)
    offset = (0.02, -0.03)
    hip = spine.hip_origin_world(q, Girdle.FRONT, offset)
    c, s = math.cos(gth), math.sin(gth)
    exp = np.array([gx + c * offset[0] - s * offset[1],
                    gz + s * offset[0] + c * offset[1],
                    gth])
    assert np.allclose(hip, exp)


# ------------------------------------------------------------- whole-body compose
def test_straight_spine_front_foot_matches_standalone_leg():
    body = WholeBody(spine=SpineModel())
    q_spine = np.zeros(DEFAULT_SPINE.n_segments)
    leg_q = np.deg2rad([-80.0, 60.0, 10.0])
    foot = body.foot_world_position(q_spine, "LF", leg_q)
    # Front girdle is at (total_length, 0, 0) with zero orientation, hip offset 0,
    # so the world foot is just the standalone leg foot shifted forward by the
    # spine length. LF is a FRONT-girdle leg, so it uses the FORE model.
    leg_foot = LegModel(DEFAULT_FORELEG).forward(leg_q)[:2]
    assert np.allclose(foot, leg_foot + np.array([DEFAULT_SPINE.total_length, 0.0]))


def test_spine_bend_shifts_front_foot_but_not_rear_foot():
    body = WholeBody(spine=SpineModel())
    leg_q = np.deg2rad([-80.0, 60.0, 10.0])
    q0 = np.zeros(DEFAULT_SPINE.n_segments)
    q1 = np.full(DEFAULT_SPINE.n_segments, np.deg2rad(25.0))

    lf0 = body.foot_world_position(q0, "LF", leg_q)
    lf1 = body.foot_world_position(q1, "LF", leg_q)
    lr0 = body.foot_world_position(q0, "LR", leg_q)
    lr1 = body.foot_world_position(q1, "LR", leg_q)

    # Rear girdle is the fixed base, so bending the spine leaves rear feet put...
    assert np.allclose(lr0, lr1)
    # ...while the front feet move measurably.
    assert np.linalg.norm(lf1 - lf0) > 1e-3


def test_front_foot_equals_girdle_composition():
    body = WholeBody(spine=SpineModel())
    q_spine = np.full(DEFAULT_SPINE.n_segments, np.deg2rad(15.0))
    leg_q = np.deg2rad([-70.0, 50.0, 5.0])
    gx, gz, gth = body.spine.girdle_pose(q_spine, Girdle.FRONT)
    foot_hip = LegModel(DEFAULT_FORELEG).forward(leg_q)[:2]  # LF -> FORE model
    c, s = math.cos(gth), math.sin(gth)
    R = np.array([[c, -s], [s, c]])
    expected = np.array([gx, gz]) + R @ foot_hip
    assert np.allclose(body.foot_world_position(q_spine, "LF", leg_q), expected)


def test_foot_positions_covers_all_four_legs():
    body = WholeBody(spine=SpineModel())
    leg_q = {name: np.deg2rad([-80.0, 60.0, 10.0]) for name in body.leg_names}
    feet = body.foot_positions(np.zeros(DEFAULT_SPINE.n_segments), leg_q)
    assert set(feet) == {"LF", "RF", "LR", "RR"}
    for pos in feet.values():
        assert pos.shape == (2,)


# ---------------------------------------------------------- spine tendon mapping
def test_spine_tendon_map_length_matches_segments():
    stm = TendonMap.from_spine(DEFAULT_SPINE)
    sol = stm.resolve(np.zeros(DEFAULT_SPINE.n_segments))
    assert len(sol.tension_flexor) == DEFAULT_SPINE.n_segments
    assert len(sol.tension_extensor) == DEFAULT_SPINE.n_segments


def test_spine_tendon_realizes_torque_and_stays_taut():
    stm = TendonMap.from_spine(DEFAULT_SPINE, mode=ActuationMode.ANTAGONISTIC)
    tau = np.array([0.20, -0.10, 0.15])
    sol = stm.resolve(tau)
    assert np.allclose(sol.joint_torque, tau)
    pre = DEFAULT_SPINE.pretension
    assert np.all(sol.tension_flexor >= pre - 1e-9)
    assert np.all(sol.tension_extensor >= pre - 1e-9)


def test_spine_tendon_uses_spine_moment_arm():
    stm = TendonMap.from_spine(DEFAULT_SPINE)
    r = np.asarray(DEFAULT_SPINE.joint_moment_arm)
    tau = np.array([0.30, 0.15, 0.06])
    sol = stm.resolve(tau)
    expected_flex = DEFAULT_SPINE.pretension + tau / r
    assert np.allclose(sol.tension_flexor, expected_flex)


def test_from_spine_generalizes_to_two_segments():
    sp = SpineParams(
        n_segments=2,
        segment_lengths=(0.05, 0.05),
        q_min=(-0.4, -0.4),
        q_max=(0.4, 0.4),
        joint_moment_arm=(0.02, 0.02),
        spring_stiffness=(1.0, 1.0),
        spring_rest_angle=(0.0, 0.0),
        segment_mass=(0.5, 0.5),
        segment_com_frac=(0.5, 0.5),
    )
    stm = TendonMap.from_spine(sp)
    sol = stm.resolve([0.1, 0.1])
    assert len(sol.tension_flexor) == 2


def test_spine_params_rejects_length_mismatch():
    with pytest.raises(ValueError):
        SpineParams(n_segments=3, segment_lengths=(0.06, 0.06))
