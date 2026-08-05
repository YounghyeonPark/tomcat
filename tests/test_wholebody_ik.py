# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Tests for whole-body foot placement / inverse kinematics (M3).

Covers ``WholeBody.inverse`` / ``inverse_pose`` (place feet in the WORLD frame
and solve per-leg IK through the moving girdle) and the world-frame
``WholeBodyGaitController`` that closes the spine<->foot loop M2 deferred.
"""

import math

import numpy as np
import pytest

from tomcat_kin import (
    WholeBody,
    SpineModel,
    LegModel,
    KneeConfig,
    GaitParams,
    GaitController,
    WholeBodyGaitController,
)
from tomcat_kin.params import DEFAULT_SPINE, DEFAULT_FORELEG, DEFAULT_HINDLEG


N = DEFAULT_SPINE.n_segments
STRAIGHT = np.zeros(N)
BEND = np.full(N, math.radians(8.0))  # uniform dorsiflexion


# ---------------------------------------------------- world -> hip -> world
@pytest.mark.parametrize("leg", ["LF", "RF", "LR", "RR"])
@pytest.mark.parametrize(
    "spine_q",
    [np.zeros(N), np.full(N, math.radians(6.0)), np.deg2rad([10.0, -5.0, 8.0])],
)
def test_world_hip_world_roundtrip(leg, spine_q):
    # A reachable world foot pose recovered by inverse must be reproduced by the
    # whole-body forward map (inverse then forward == identity on the target).
    body = WholeBody()
    # Build a definitely-reachable world target from a known leg posture.
    leg_q = np.deg2rad([-70.0, 55.0, 5.0])
    target = body.foot_world_pose(spine_q, leg, leg_q)

    sol = body.inverse(spine_q, leg, target)
    assert sol.reachable
    assert sol.q is not None
    fk = body.foot_world_pose(spine_q, leg, sol.q)
    assert np.allclose(fk, target, atol=1e-9)


def test_neutral_spine_reduces_to_standalone_leg_ik():
    # With the spine neutral the girdle orientation is 0 and the only transform
    # is the fixed hip translation, so the whole-body solution must equal the
    # standalone LegModel IK of the world target shifted into the hip frame.
    body = WholeBody()
    hx, hz, hth = body.hip_world_pose(STRAIGHT, "LF")
    assert hth == pytest.approx(0.0)
    # Reachable hip-frame pose at the new digitigrade geometry (paw_angle 55 deg,
    # reach ~0.28 m): the old (0.22, -0.13, -20 deg) target fell outside the
    # 2R femur/tibia sub-workspace once the steep metatarsus was backed out.
    hip_pose = np.array([0.20, -0.12, math.radians(-10.0)])
    world = np.array([hip_pose[0] + hx, hip_pose[1] + hz, hip_pose[2] + hth])

    sol = body.inverse(STRAIGHT, "LF", world, knee=KneeConfig.FLEXED_POSITIVE)
    # LF is a FRONT-girdle leg -> it uses the FORE model, so the standalone check
    # must use the same fore geometry (not the default/hind LegModel).
    q_standalone = LegModel(DEFAULT_FORELEG).inverse(
        hip_pose, knee=KneeConfig.FLEXED_POSITIVE
    )
    assert np.allclose(sol.q, q_standalone)


def test_spine_bend_changes_front_leg_angles_but_holds_world_foot():
    # THE M3 property: a front foot commanded to a FIXED world pose yields
    # DIFFERENT leg angles under a spine bend than with a neutral spine (the leg
    # compensates), yet forward kinematics still lands the foot on that world
    # pose in both cases.
    body = WholeBody()
    # Reachable by the digitigrade leg both straight and bent (the shorter
    # 4-link reach ~0.28 m puts the old 0.42 m target just outside the bent hip).
    world = np.array([0.40, -0.11, math.radians(-10.0)])

    straight = body.inverse(STRAIGHT, "LF", world)
    bent = body.inverse(BEND, "LF", world)

    assert straight.reachable and bent.reachable
    # Legs compensate: the joint angles must differ measurably.
    assert not np.allclose(straight.q, bent.q, atol=1e-3)
    # ...yet both land the foot on the same fixed world pose.
    assert np.allclose(body.foot_world_pose(STRAIGHT, "LF", straight.q), world, atol=1e-9)
    assert np.allclose(body.foot_world_pose(BEND, "LF", bent.q), world, atol=1e-9)


def test_rear_leg_unaffected_by_spine_bend():
    # The rear girdle is the spine BASE (vertebra 0), so its pose is invariant to
    # the spine angles; a rear foot at a fixed world pose therefore solves to the
    # SAME leg angles straight or bent.
    body = WholeBody()
    world = np.array([0.05, -0.10, math.radians(-15.0)])
    straight = body.inverse(STRAIGHT, "LR", world)
    bent = body.inverse(BEND, "LR", world)
    assert straight.reachable and bent.reachable
    assert np.allclose(straight.q, bent.q, atol=1e-12)


# ---------------------------------------------------- fore/hind ASYMMETRY
def test_wholebody_holds_distinct_fore_and_hind_models():
    # Front-girdle legs use the FORE geometry, rear-girdle legs the HIND geometry.
    body = WholeBody()
    for front in ("LF", "RF"):
        assert body.leg_model_for(front).params is DEFAULT_FORELEG
    for rear in ("LR", "RR"):
        assert body.leg_model_for(rear).params is DEFAULT_HINDLEG
    # The two proportion sets really do differ (else the test below is vacuous).
    assert DEFAULT_FORELEG != DEFAULT_HINDLEG


def test_fore_and_hind_models_are_constructor_overridable():
    # Tests can inject their own leg models; the girdle split still applies.
    fore = LegModel(DEFAULT_HINDLEG)   # deliberately swapped for the check
    hind = LegModel(DEFAULT_FORELEG)
    body = WholeBody(fore_leg=fore, hind_leg=hind)
    assert body.leg_model_for("LF") is fore
    assert body.leg_model_for("RR") is hind
    # An explicit legs dict still wins over the fore/hind fields.
    custom = LegModel(DEFAULT_FORELEG)
    body2 = WholeBody(legs={n: custom for n in ("LF", "RF", "LR", "RR")})
    assert all(body2.leg_model_for(n) is custom for n in body2.leg_names)


def test_same_relative_foot_target_gives_different_front_vs_rear_angles():
    # THE asymmetry property. Give a FRONT leg and a REAR leg the SAME target in
    # their own hip frame (neutral spine => both girdles are unrotated, so a fixed
    # hip-frame pose maps to each leg's world target by a pure translation). The
    # fore vs. hind proportions differ, so the solved joint angles MUST differ --
    # yet each leg's FK still lands ITS OWN foot on ITS OWN world target.
    body = WholeBody()
    hip_pose = np.array([0.20, -0.12, math.radians(-10.0)])  # in the hip frame

    fx, fz, fth = body.hip_world_pose(STRAIGHT, "LF")  # front hip (fore leg)
    rx, rz, rth = body.hip_world_pose(STRAIGHT, "LR")  # rear hip (hind leg)
    assert fth == pytest.approx(0.0) and rth == pytest.approx(0.0)
    front_world = np.array([hip_pose[0] + fx, hip_pose[1] + fz, hip_pose[2] + fth])
    rear_world = np.array([hip_pose[0] + rx, hip_pose[1] + rz, hip_pose[2] + rth])

    front = body.inverse(STRAIGHT, "LF", front_world)
    rear = body.inverse(STRAIGHT, "LR", rear_world)
    assert front.reachable and rear.reachable
    # Same relative target, DIFFERENT joint angles (fore vs. hind proportions).
    assert front.foot_hip == pytest.approx(rear.foot_hip)  # identical hip-frame pose
    assert not np.allclose(front.q, rear.q, atol=1e-3)
    # ...and each leg's FK lands its own foot on its own world target.
    assert np.allclose(body.foot_world_pose(STRAIGHT, "LF", front.q), front_world, atol=1e-9)
    assert np.allclose(body.foot_world_pose(STRAIGHT, "LR", rear.q), rear_world, atol=1e-9)


# ------------------------------------------------------- flags, not exceptions
def test_unreachable_world_target_flagged_not_raised():
    body = WholeBody()
    # Far outside any leg workspace: must be FLAGGED, never raised.
    sol = body.inverse(STRAIGHT, "LF", np.array([5.0, 0.0, 0.0]))
    assert not sol.reachable
    assert sol.q is None
    assert not sol.within_limits
    assert not sol.ok


def test_out_of_limit_but_reachable_flagged():
    # A pose the 2R chain can reach but whose ankle lands outside the joint limit
    # is reachable yet not within_limits (so ok is False), still no exception.
    body = WholeBody()
    # Foot directly above the hip with a large pitch stresses the ankle limit.
    world = np.array([0.195 + 0.02, 0.18, math.radians(80.0)])
    sol = body.inverse(STRAIGHT, "LF", world)
    if sol.reachable:
        assert sol.ok == (sol.reachable and sol.within_limits)


def test_inverse_requires_full_pose():
    body = WholeBody()
    with pytest.raises(ValueError):
        body.inverse(STRAIGHT, "LF", np.array([0.3, -0.1]))  # missing phi


def test_inverse_pose_solves_all_four_legs():
    body = WholeBody()
    leg_q = np.deg2rad([-75.0, 55.0, 5.0])
    targets = {n: body.foot_world_pose(BEND, n, leg_q) for n in body.leg_names}
    sols = body.inverse_pose(BEND, targets)
    assert set(sols) == {"LF", "RF", "LR", "RR"}
    for n, sol in sols.items():
        assert sol.reachable
        assert np.allclose(body.foot_world_pose(BEND, n, sol.q), targets[n], atol=1e-9)


# ============================================================
# World-frame gait controller (closes the spine<->foot loop)
# ============================================================
def test_wholebody_gait_neutral_matches_hip_controller():
    # With the spine NEUTRAL the world-frame controller must reproduce the
    # hip-frame GaitController per-leg joint angles exactly, over the whole cycle.
    p = GaitParams()
    hip = GaitController(params=p)
    wb = WholeBodyGaitController(params=p)
    for i in range(60):
        ph = i / 60
        hs = hip.state(ph)
        ws = wb.state(ph)
        for n in p.phase_offsets:
            assert (hs.legs[n].q is None) == (ws.legs[n].q is None)
            if hs.legs[n].q is not None:
                assert np.allclose(hs.legs[n].q, ws.legs[n].q, atol=1e-12)


def test_wholebody_gait_stance_foot_fixed_in_world():
    # A contiguous-stance front leg (LF, offset 0 => stance is [0, duty) with no
    # phase wrap) must hold a CONSTANT world foot pose across its whole stance.
    wb = WholeBodyGaitController(params=GaitParams())
    d = wb.params.duty_factor
    poses = np.array(
        [wb.leg_state(p, "LF").foot_target_world for p in np.linspace(0.0, d, 40, endpoint=False)]
    )
    assert np.ptp(poses[:, 0]) < 1e-12  # world x held fixed
    assert np.ptp(poses[:, 1]) < 1e-12  # world z held fixed


def test_wholebody_gait_legs_compensate_spine_motion():
    # Spine oscillation ON vs. NEUTRAL: the STANCE front foot's fixed world pose
    # is identical, but the solved leg angles DIFFER (compensation) and forward
    # kinematics still lands the foot on the world pose.
    neu = WholeBodyGaitController(params=GaitParams())
    osc = WholeBodyGaitController(params=GaitParams(spine_amplitude=math.radians(2.0)))
    changed = 0
    for ph in (0.05, 0.2, 0.4, 0.6):
        sn = neu.leg_state(ph, "LF")
        so = osc.leg_state(ph, "LF")
        assert sn.in_stance and so.in_stance
        # Same fixed world target regardless of spine posture.
        assert np.allclose(sn.foot_target_world, so.foot_target_world, atol=1e-12)
        assert sn.reachable and so.reachable
        # Foot still lands on the world target under the moving spine.
        assert np.allclose(osc.foot_world_check(ph, "LF"), so.foot_target_world, atol=1e-9)
        if not np.allclose(sn.q, so.q, atol=1e-3):
            changed += 1
    # The legs demonstrably compensate at (most) sampled phases.
    assert changed >= 3


def test_wholebody_gait_unreachable_flagged_not_raised():
    bad = WholeBodyGaitController(params=GaitParams(nominal_foot=(5.0, -0.13)))
    st = bad.state(0.1)  # must not raise
    assert not st.all_ok
    assert any((not lg.reachable) for lg in st.legs.values())
    assert any(lg.q is None for lg in st.legs.values())


def test_wholebody_gait_body_advances_forward():
    wb = WholeBodyGaitController(params=GaitParams())
    # Body-ground origin advances distance_per_cycle over a full cycle.
    assert wb.body_offset(1.0) == pytest.approx(wb.params.distance_per_cycle)
    assert wb.body_offset(0.0) == pytest.approx(0.0)


def test_wholebody_gait_stance_count_preserved():
    # Same timing as the hip-frame walk: at least 3 feet down, at most 1 swinging
    # (duty 0.80 since M5, so four-foot windows are expected -- see test_gait).
    wb = WholeBodyGaitController(params=GaitParams())
    for i in range(41):
        assert wb.state(i / 40).stance_count in (3, 4)
