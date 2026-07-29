"""Tests for the M4 mass properties: bookkeeping, sub-assembly CoMs, whole-body CoM.

Scope reminder: everything here is QUASI-STATIC (gravity + CoM geometry). No
inertias, velocities or accelerations are modelled or tested -- full Newton-Euler
is a later milestone.
"""

import numpy as np
import pytest

from tomcat_kin import (
    WholeBody,
    SpineModel,
    SpineParams,
    LegModel,
    LegParams,
    Girdle,
    ComResult,
    combine,
    leg_com,
    leg_link_coms,
    point_masses_com,
    quarter_masses,
    spine_segment_coms,
)
from tomcat_kin.params import (
    DEFAULT_SPINE,
    DEFAULT_FORELEG,
    DEFAULT_HINDLEG,
    DEFAULT_BODY_MASS_KG,
    LoadCase,
)


STAND = np.deg2rad([-80.0, 60.0, 10.0])   # the demo's nominal standing pose
ARCH = np.full(DEFAULT_SPINE.n_segments, np.deg2rad(20.0))
STRAIGHT = np.zeros(DEFAULT_SPINE.n_segments)


def _body():
    return WholeBody(spine=SpineModel())


def _symmetric_trunk_body(leg_mass=0.0):
    """Fore/aft-SYMMETRIC body: uniform spine, equal segment + girdle masses.

    ``leg_mass`` is the mass of ONE leg (split equally over its four links); all
    four legs are identical. With ``leg_mass=0`` the CoM of a straight spine must
    land EXACTLY at mid-body, the cleanest possible CoM sanity check.
    """
    uniform = SpineParams(
        segment_lengths=(0.06, 0.06, 0.06),
        segment_mass=(0.4, 0.4, 0.4),
        segment_com_frac=(0.5, 0.5, 0.5),
        front_girdle_mass=0.5,
        rear_girdle_mass=0.5,
    )
    leg = LegModel(LegParams(link_mass=(leg_mass / 4,) * 4))
    return WholeBody(spine=SpineModel(params=uniform), fore_leg=leg, hind_leg=leg)


# =====================================================================
# 1. Mass bookkeeping
# =====================================================================
def test_default_body_totals_the_load_case_body_mass():
    # The apportionment in params.py is built to reproduce the 3.0 kg that every
    # LoadCase / WholeBodyLoadCase already assumed.
    assert DEFAULT_BODY_MASS_KG == pytest.approx(3.0, abs=1e-9)
    assert _body().total_mass == pytest.approx(LoadCase("x").body_mass_kg, abs=1e-9)


def test_leg_link_masses_sum_to_leg_mass_and_are_proximal_heavy():
    for p in (DEFAULT_FORELEG, DEFAULT_HINDLEG):
        assert p.mass == pytest.approx(sum(p.link_mass))
        # Mass decreases monotonically toward the paw (tendon drive + biology
        # both centralise mass proximally).
        assert list(p.link_mass) == sorted(p.link_mass, reverse=True)


def test_hind_leg_is_heavier_than_fore_leg():
    # The hind limb still carries more, but both are now sized BOTTOM-UP from the
    # specced hardware (review F1), not from a biological limb fraction.
    assert DEFAULT_HINDLEG.mass > DEFAULT_FORELEG.mass
    assert DEFAULT_HINDLEG.mass == pytest.approx(0.110)
    assert DEFAULT_FORELEG.mass == pytest.approx(0.095)


def test_limbs_are_light_because_the_motors_are_not_in_them():
    # REGRESSION GUARD for review finding F1. The old budget gave the limbs ~24%
    # of body mass, copied from feline biology -- but a biological limb is mostly
    # MUSCLE, and P1/ADR-0003 deliberately relocate the muscle (motors) into the
    # girdles. Applying the biological fraction double-counted the actuator.
    # A tendon-driven limb is just structure, so it must be much lighter.
    body = _body()
    legs = sum(body.legs[n].params.mass for n in body.mounts)
    assert 0.10 < legs / body.total_mass < 0.18


def test_trunk_plus_legs_equals_total():
    body = _body()
    legs = sum(body.legs[n].params.mass for n in body.mounts)
    assert body.spine.params.trunk_mass + legs == pytest.approx(body.total_mass)
    assert body.spine.params.trunk_mass == pytest.approx(
        body.spine.params.chain_mass
        + body.spine.params.front_girdle_mass
        + body.spine.params.rear_girdle_mass
    )


def test_fore_hind_split_is_near_balanced_not_sixty_forty():
    # REGRESSION GUARD for review finding F2. The old model TUNED the girdle
    # masses to hit a 60/40 front-heavy split. Once the motors sit in their real
    # clusters the pelvis carries MORE of them (19 vs 12), which largely cancels
    # the head/neck: the machine is near-balanced, not front-heavy like a cat.
    q = _body().mass_budget()
    assert q.total == pytest.approx(3.0, abs=1e-9)
    assert q.fore + q.hind == pytest.approx(q.total, abs=1e-12)
    assert q.fore_fraction == pytest.approx(0.51, abs=0.02)
    assert q.hind_fraction == pytest.approx(0.49, abs=0.02)
    # Still fore-biased, just barely -- the head/neck edges it forward.
    assert q.fore > q.hind
    assert "forequarters" in q.report()


def test_symmetric_body_splits_fifty_fifty():
    # Sanity on the lever rule itself: a fore/aft-symmetric body must split 50/50.
    q = _symmetric_trunk_body(leg_mass=0.1).mass_budget()
    assert q.fore_fraction == pytest.approx(0.5, abs=1e-12)


def test_quarter_masses_lever_rule_hand_check():
    # One segment, mass at mid-span => exactly half its mass to each girdle.
    sp = SpineParams(
        n_segments=1,
        segment_lengths=(0.10,),
        q_min=(-0.4,), q_max=(0.4,),
        joint_moment_arm=(0.03,),
        spring_stiffness=(1.0,), spring_rest_angle=(0.0,),
        segment_mass=(1.0,), segment_com_frac=(0.5,),
        front_girdle_mass=0.0, rear_girdle_mass=0.0,
    )
    q = quarter_masses(sp)
    assert q.fore == pytest.approx(0.5)
    assert q.hind == pytest.approx(0.5)
    # A CoM fraction of 0.25 sends only a quarter of the mass forward.
    sp2 = SpineParams(
        n_segments=1, segment_lengths=(0.10,), q_min=(-0.4,), q_max=(0.4,),
        joint_moment_arm=(0.03,), spring_stiffness=(1.0,), spring_rest_angle=(0.0,),
        segment_mass=(1.0,), segment_com_frac=(0.25,),
        front_girdle_mass=0.0, rear_girdle_mass=0.0,
    )
    assert quarter_masses(sp2).fore == pytest.approx(0.25)


def test_subassembly_masses_sum_to_the_total():
    body = _body()
    c = body.center_of_mass(STRAIGHT, STAND)
    parts = c.spine.mass + sum(g.mass for g in c.girdles.values()) + sum(
        l.mass for l in c.legs.values()
    )
    assert parts == pytest.approx(c.mass, abs=1e-12)
    assert c.mass == pytest.approx(body.total_mass, abs=1e-12)
    # And the sub-assembly CoMs recombine to the whole-body CoM.
    recombined = combine([c.spine, *c.girdles.values(), *c.legs.values()])
    assert np.allclose(recombined.com, c.com)


def test_params_reject_malformed_mass_tuples():
    with pytest.raises(ValueError):
        LegParams(link_mass=(0.1, 0.1, 0.1))          # 3 entries, expected 4
    with pytest.raises(ValueError):
        LegParams(link_com_frac=(0.5, 0.5))
    with pytest.raises(ValueError):
        LegParams(link_mass=(0.1, -0.1, 0.1, 0.1))    # negative mass
    with pytest.raises(ValueError):
        SpineParams(n_segments=3, segment_mass=(0.3, 0.3))


# =====================================================================
# 2. Sub-assembly CoM geometry
# =====================================================================
def test_leg_link_coms_interpolate_between_joints():
    leg = LegModel(DEFAULT_HINDLEG)
    pts = leg.joint_positions(STAND)
    coms = leg_link_coms(leg, STAND)
    assert coms.shape == (4, 2)
    for i, f in enumerate(DEFAULT_HINDLEG.link_com_frac):
        assert np.allclose(coms[i], pts[i] + f * (pts[i + 1] - pts[i]))


def test_leg_com_frac_endpoints_are_exact():
    # com_frac = 0 puts every link's mass on its PROXIMAL joint; com_frac = 1 on
    # its DISTAL joint. Both are exactly reproducible by hand.
    q = STAND
    prox = LegModel(LegParams(link_com_frac=(0.0, 0.0, 0.0, 0.0)))
    dist = LegModel(LegParams(link_com_frac=(1.0, 1.0, 1.0, 1.0)))
    pts = prox.joint_positions(q)
    m = np.asarray(prox.params.link_mass)
    assert np.allclose(leg_com(prox, q).com, (m[:, None] * pts[:-1]).sum(0) / m.sum())
    assert np.allclose(leg_com(dist, q).com, (m[:, None] * pts[1:]).sum(0) / m.sum())


def test_leg_com_inside_the_leg_bounding_box():
    for p in (DEFAULT_FORELEG, DEFAULT_HINDLEG):
        leg = LegModel(p)
        pts = leg.joint_positions(STAND)
        c = leg_com(leg, STAND)
        assert c.mass == pytest.approx(p.mass)
        assert pts[:, 0].min() - 1e-12 <= c.x <= pts[:, 0].max() + 1e-12
        assert pts[:, 1].min() - 1e-12 <= c.z <= pts[:, 1].max() + 1e-12
        # Proximal-heavy: the CoM sits nearer the hip than the paw tip.
        assert np.linalg.norm(c.com - pts[0]) < np.linalg.norm(c.com - pts[-1])


def test_spine_segment_coms_track_the_bent_geometry():
    spine = SpineModel()
    p = DEFAULT_SPINE
    straight = spine_segment_coms(spine.vertebra_positions(STRAIGHT), p.segment_com_frac)
    arched = spine_segment_coms(spine.vertebra_positions(ARCH), p.segment_com_frac)
    assert straight.shape == (p.n_segments, 2)
    assert np.allclose(straight[:, 1], 0.0)          # straight spine lies on z=0
    assert arched[-1, 1] > straight[-1, 1]           # arch lifts the front segment
    with pytest.raises(ValueError):
        spine_segment_coms(np.zeros((2, 2)), p.segment_com_frac)


def test_girdle_com_follows_the_girdle_pose():
    body = _body()
    rear = body.girdle_com(STRAIGHT, Girdle.REAR)
    front = body.girdle_com(STRAIGHT, Girdle.FRONT)
    assert rear.mass == pytest.approx(DEFAULT_SPINE.rear_girdle_mass)
    assert front.mass == pytest.approx(DEFAULT_SPINE.front_girdle_mass)
    assert np.allclose(rear.com, [0.0, 0.0])
    assert np.allclose(front.com, [DEFAULT_SPINE.total_length, 0.0])
    # Arching the back lifts the FRONT girdle but leaves the base put.
    assert body.girdle_com(ARCH, Girdle.FRONT).z > 0.0
    assert np.allclose(body.girdle_com(ARCH, Girdle.REAR).com, [0.0, 0.0])


def test_point_masses_com_and_combine():
    a = point_masses_com([1.0, 1.0], [[0.0, 0.0], [2.0, 0.0]])
    assert a.mass == pytest.approx(2.0)
    assert np.allclose(a.com, [1.0, 0.0])
    b = ComResult(2.0, np.array([3.0, 0.0]))
    assert np.allclose(combine([a, b]).com, [2.0, 0.0])
    assert combine([]).mass == 0.0
    assert point_masses_com([0.0], [[5.0, 5.0]]).mass == 0.0
    with pytest.raises(ValueError):
        point_masses_com([1.0, 2.0], [[0.0, 0.0]])


# =====================================================================
# 3. Whole-body CoM
# =====================================================================
def test_symmetric_body_com_is_exactly_mid_body():
    body = _symmetric_trunk_body(leg_mass=0.0)
    c = body.center_of_mass(np.zeros(3), STAND)
    assert c.x == pytest.approx(body.spine.params.total_length / 2.0, abs=1e-12)
    assert c.z == pytest.approx(0.0, abs=1e-12)


def test_symmetric_body_with_legs_shifts_by_exactly_the_leg_offset():
    # Adding equal legs to a symmetric trunk moves the CoM by the leg-mass
    # fraction times the (shared) hip->leg-CoM offset -- an exact hand check.
    body = _symmetric_trunk_body(leg_mass=0.1)
    bare = _symmetric_trunk_body(leg_mass=0.0)
    offset = leg_com(body.fore_leg, STAND).com   # hip -> leg CoM, shared by all 4
    frac = (4 * 0.1) / body.total_mass
    expected = bare.center_of_mass(np.zeros(3), STAND).com + frac * offset
    assert np.allclose(body.center_of_mass(np.zeros(3), STAND).com, expected)


def test_default_com_sits_forward_of_mid_body_because_the_cat_is_front_heavy():
    body = _body()
    c = body.center_of_mass(STRAIGHT, STAND)
    assert c.mass == pytest.approx(3.0)
    # Forward of the mid-spine point, but still between the two girdles.
    assert c.x > DEFAULT_SPINE.total_length / 2.0
    assert 0.0 < c.x < DEFAULT_SPINE.total_length
    # Legs hang below the spine, so the CoM is below the spine line.
    assert c.z < 0.0


def test_arching_the_spine_moves_the_com_up_and_rearward():
    body = _body()
    flat = body.center_of_mass(STRAIGHT, STAND)
    arch = body.center_of_mass(ARCH, STAND)
    assert arch.z > flat.z          # dorsiflexion curls the forequarters up
    assert arch.x < flat.x          # ...and back over the pelvis
    assert arch.mass == pytest.approx(flat.mass)


def test_ventroflexing_the_spine_moves_the_com_down():
    body = _body()
    flat = body.center_of_mass(STRAIGHT, STAND)
    sag = body.center_of_mass(-ARCH, STAND)
    assert sag.z < flat.z
    assert sag.x < flat.x           # any bend shortens the horizontal span


def test_swinging_one_leg_forward_moves_the_com_forward_by_the_right_amount():
    body = _body()
    base = {n: STAND for n in body.leg_names}
    swung = dict(base)
    swung["LF"] = np.deg2rad([-60.0, 60.0, 10.0])   # rotate the whole LF leg CCW

    c0 = body.center_of_mass(STRAIGHT, base)
    c1 = body.center_of_mass(STRAIGHT, swung)
    d_leg = c1.legs["LF"].com - c0.legs["LF"].com
    assert d_leg[0] > 0.0                            # that leg's CoM moved forward
    # Whole-body CoM moves by exactly (m_leg / M) * (leg CoM shift).
    expected = (c0.legs["LF"].mass / c0.mass) * d_leg
    assert np.allclose(c1.com - c0.com, expected, atol=1e-12)
    assert c1.x > c0.x
    # Only the moved leg changed.
    for name in ("RF", "LR", "RR"):
        assert np.allclose(c1.legs[name].com, c0.legs[name].com)


def test_spine_bend_moves_front_leg_com_but_not_rear():
    body = _body()
    flat = body.center_of_mass(STRAIGHT, STAND)
    arch = body.center_of_mass(ARCH, STAND)
    # Rear girdle is the fixed base of the chain.
    assert np.allclose(arch.legs["LR"].com, flat.legs["LR"].com)
    assert np.linalg.norm(arch.legs["LF"].com - flat.legs["LF"].com) > 1e-3


def test_leg_com_world_matches_hip_frame_com_through_the_transform():
    body = _body()
    q = np.deg2rad([-70.0, 50.0, 5.0])
    hx, hz, hth = body.hip_world_pose(ARCH, "LF")
    local = leg_com(body.leg_model_for("LF"), q)
    c, s = np.cos(hth), np.sin(hth)
    R = np.array([[c, -s], [s, c]])
    assert np.allclose(
        body.leg_com_world(ARCH, "LF", q).com, np.array([hx, hz]) + R @ local.com
    )


def test_center_of_mass_argument_forms():
    body = _body()
    shared = body.center_of_mass(STRAIGHT, STAND)
    mapped = body.center_of_mass(STRAIGHT, {n: STAND for n in body.leg_names})
    assert np.allclose(shared.com, mapped.com)
    # None => all-zero (degenerate reference) pose, still mass-consistent.
    zeroed = body.center_of_mass(STRAIGHT)
    assert zeroed.mass == pytest.approx(body.total_mass)
    assert not np.allclose(zeroed.com, shared.com)
    with pytest.raises(ValueError):
        body.center_of_mass(STRAIGHT, {"LF": STAND})          # missing legs
    with pytest.raises(ValueError):
        body.center_of_mass(STRAIGHT, np.zeros(4))            # wrong shape


def test_body_com_report_lists_every_subassembly():
    txt = _body().center_of_mass(STRAIGHT, STAND).report()
    for token in ("whole body", "spine chain", "front girdle", "rear girdle",
                  "leg LF", "leg RR"):
        assert token in txt


def test_fore_and_hind_legs_have_different_coms_for_the_same_angles():
    # The fore/hind asymmetry must survive into the mass model.
    body = _body()
    c = body.center_of_mass(STRAIGHT, STAND)
    fore_local = c.legs["LF"].com - body.hip_world_pose(STRAIGHT, "LF")[:2]
    hind_local = c.legs["LR"].com - body.hip_world_pose(STRAIGHT, "LR")[:2]
    assert not np.allclose(fore_local, hind_local)
    assert c.legs["LF"].mass < c.legs["LR"].mass
