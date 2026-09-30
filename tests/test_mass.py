# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
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
        # ⚠️ A body this fixture calls SYMMETRIC has to build symmetric girdles.
        # M87 measured `front/rear_girdle_com` at (-4.6, +17.4) and (-5.7, +16.0)
        # mm -- the motor bank stacks upward and its third module sits on the rear
        # column -- and the defaults leaked in, tilting a body whose whole purpose
        # is to be exactly balanced. Inputs a test asserts about belong to the
        # test.
        front_girdle_com=(0.0, 0.0),
        rear_girdle_com=(0.0, 0.0),
    )
    leg = LegModel(LegParams(link_mass=(leg_mass / 4,) * 4))
    return WholeBody(spine=SpineModel(params=uniform), fore_leg=leg, hind_leg=leg)


# =====================================================================
# 1. Mass bookkeeping
# =====================================================================
def test_default_body_totals_the_load_case_body_mass():
    # The apportionment in params.py is built to reproduce the 3.0 kg that every
    # LoadCase / WholeBodyLoadCase already assumed.
    assert DEFAULT_BODY_MASS_KG == pytest.approx(4.60530, abs=1e-9)  # M122 -> M123 (ADR-0114)
    assert _body().total_mass == pytest.approx(LoadCase("x").body_mass_kg, abs=1e-9)


def test_leg_link_masses_sum_to_leg_mass_and_are_proximal_heavy():
    for p in (DEFAULT_FORELEG, DEFAULT_HINDLEG):
        assert p.mass == pytest.approx(sum(p.link_mass))
        # Mass decreases monotonically toward the paw (tendon drive + biology
        # both centralise mass proximally).
        assert list(p.link_mass) == sorted(p.link_mass, reverse=True)


def test_the_fore_hind_leg_ASYMMETRY_has_vanished():
    """⚠️ **M41 (ADR-0046) overturned this test's premise, hence the rename.**

    It used to assert `hind > fore` on an ASSUMED 1.16x asymmetry — 0.110 kg against
    0.095 — justified as *"the fore limb is the lighter, more columnar limb; the hind
    limb carries the propulsion musculature"*.

    Drawn as manufacturable parts the two legs are **0.16716 and 0.16739 kg**: the
    joint hardware (sheaves, bearings, clevises) dominates at ~80 % of the leg, and
    it is the *same* hardware on both ends, so the shorter fore links barely
    register. The fore leg is in fact 0.2 g heavier — 0.14 %, which is noise in a
    model built on assumed stock and catalogue bearing masses.

    ⚠️ The honest statement is that they are **equal**, not that the ranking flipped.
    Design review F2 settled the fore/hind weight split using the old asymmetry and
    is re-checked in `test_fore_hind_split_...` below.
    """
    # M122 (ADR-0112): -13 g each -- no hip via, no return spring, no pins
    # M123 (ADR-0114): +26 g each -- the hollow hip's hub and 40 x 50 bearings
    assert DEFAULT_HINDLEG.mass == pytest.approx(0.22122, abs=5e-4)
    assert DEFAULT_FORELEG.mass == pytest.approx(0.22133, abs=5e-4)
    assert abs(DEFAULT_HINDLEG.mass - DEFAULT_FORELEG.mass) < 0.001, (
        "the legs are equal now; if a real asymmetry returns, re-derive F2's split"
    )


def test_limbs_are_light_because_the_motors_are_not_in_them():
    # REGRESSION GUARD for review finding F1. The old budget gave the limbs ~24%
    # of body mass, copied from feline biology -- but a biological limb is mostly
    # MUSCLE, and P1/ADR-0003 deliberately relocate the muscle (motors) into the
    # girdles. Applying the biological fraction double-counted the actuator.
    # A tendon-driven limb is just structure, so it must be much lighter.
    body = _body()
    legs = sum(body.legs[n].params.mass for n in body.mounts)
    # ⚠️ M111: 0.170 -> 0.187. ADR-0103's 36/34/22 sheaves put +21 g on each
    # leg, and that is the one place this guard's premise gives way: a sheave IS
    # actuator hardware, and the fore leg's load could only be reached from the
    # joint end. Still well under the ~24 % biological fraction F1 retired.
    assert 0.10 < legs / body.total_mass < 0.20
    # M122: 0.187 -> 0.173, the legs shed the hip via, spring and pins.
    # M123: -> 0.192, the hollow hip (ADR-0114). Still under the 0.20 guard.
    assert legs / body.total_mass == pytest.approx(0.192, abs=0.005)


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
    # masses to hit a 60/40 front-heavy split; the bottom-up count replaced that
    # with wherever the hardware actually sits.
    #
    # UPDATED TWICE: the ADR-0009 re-check corrected the motor mass (31 -> 72 g)
    # and moved the spine bank; the M6 motor-reality-check then replaced the 72 g
    # CLASS TARGET with the surveyed real part at 132 g, taking the body to 4.05 kg.
    # and moving the spine/tail bank out of the rear girdle into the mid-body bay
    # LIGHTENED the pelvis, so the split moved from 51/49 to ~55/45. Still nothing
    # like a real cat's 60/40, and still an OUTPUT of the hardware layout rather
    # than a tuned input.
    # ⚠️ UPDATED A THIRD TIME by M41 (ADR-0046). Folding the manufacturing model's
    # leg masses in removed the assumed fore/hind leg asymmetry (0.095/0.110 kg ->
    # 0.167 both ends) and added 0.26 kg of body, moving the split 54.2/45.8 ->
    # **54.3/45.7**. Barely, because the legs are only 15.5 % of the body and the
    # change was symmetric — which is itself the point: the split is set by the
    # girdles and the head, not by the limbs.
    q = _body().mass_budget()
    assert q.total == pytest.approx(4.60530, abs=1e-9)   # M123
    assert q.fore + q.hind == pytest.approx(q.total, abs=1e-12)
    assert q.fore_fraction == pytest.approx(0.543, abs=0.02)
    assert q.hind_fraction == pytest.approx(0.457, abs=0.02)
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
    # ⚠️ M92: the straight spine lies on the SPINE AXIS, not on the hip axis.
    assert np.allclose(straight[:, 1], p.spine_axis_z)
    assert arched[-1, 1] > straight[-1, 1]           # arch lifts the front segment
    with pytest.raises(ValueError):
        spine_segment_coms(np.zeros((2, 2)), p.segment_com_frac)


def test_girdle_com_follows_the_girdle_pose():
    body = _body()
    rear = body.girdle_com(STRAIGHT, Girdle.REAR)
    front = body.girdle_com(STRAIGHT, Girdle.FRONT)
    assert rear.mass == pytest.approx(DEFAULT_SPINE.rear_girdle_mass)
    assert front.mass == pytest.approx(DEFAULT_SPINE.front_girdle_mass)
    # ⚠️ Both were `[0, 0]` while `front/rear_girdle_com` were the (0, 0)
    # placeholder. M87 measured them from the packed motor banks: the bank stacks
    # upward, so each girdle's mass sits ~16-17 mm ABOVE its mount vertebra, and
    # a few mm behind it because the third module of a 2-per-layer bank lands on
    # the rear column. What this test is named for -- that the offset RIDES with
    # the girdle pose -- is asserted below.
    assert np.allclose(rear.com, DEFAULT_SPINE.rear_girdle_com)
    assert np.allclose(
        front.com,
        [DEFAULT_SPINE.total_length + DEFAULT_SPINE.front_girdle_com[0],
         DEFAULT_SPINE.front_girdle_com[1]])
    assert DEFAULT_SPINE.front_girdle_com[1] > 0.015, "the bank stacks UPWARD"
    # Arching the back lifts the FRONT girdle but leaves the base put.
    assert body.girdle_com(ARCH, Girdle.FRONT).z > 0.0
    assert np.allclose(body.girdle_com(ARCH, Girdle.REAR).com,
                       body.girdle_com(STRAIGHT, Girdle.REAR).com)


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
    """⚠️ M92 raised the vertebral chain, so mid-body is no longer z = 0.

    The girdles still sit ON the hip axis and the segments now sit one
    `spine_axis_z` above it, so the trunk CoM rises by the chain's mass share of
    that height -- computed from the parameters here rather than written down,
    because a number in a test drifts exactly like a number in a document.
    """
    body = _symmetric_trunk_body(leg_mass=0.0)
    sp = body.spine.params
    c = body.center_of_mass(np.zeros(3), STAND)
    want_z = sum(sp.segment_mass) * sp.spine_axis_z / sp.trunk_mass
    assert c.x == pytest.approx(sp.total_length / 2.0, abs=1e-12)
    assert c.z == pytest.approx(want_z, abs=1e-12)


def test_symmetric_body_with_legs_shifts_by_exactly_the_leg_offset():
    """⚠️ **The old "exact hand check" was only exact while the trunk CoM was
    at z = 0.** It read `bare.com + frac * offset`, which treats the legs as
    hanging from the trunk's own CoM. They hang from the HIPS. In x the two
    coincide by symmetry and in z they coincided only because the vertebral
    chain used to run along the hip axis -- so M92 raising it turned a passing
    identity into a 4 mm error. The general form is written out here: legs are
    added at the mean hip, not at the trunk CoM.
    """
    body = _symmetric_trunk_body(leg_mass=0.1)
    bare = _symmetric_trunk_body(leg_mass=0.0)
    offset = leg_com(body.fore_leg, STAND).com   # hip -> leg CoM, shared by all 4
    # mean hip -- M122: the hind hips sit `rear_hip_x` behind the spine root
    hips = np.array([(body.spine.params.total_length + body.spine.params.front_hip_x
                      + body.spine.params.rear_hip_x) / 2.0, 0.0])   # M123: both hips offset
    m_trunk = bare.total_mass
    m_legs = 4 * 0.1
    expected = ((m_trunk * bare.center_of_mass(np.zeros(3), STAND).com
                 + m_legs * (hips + offset)) / (m_trunk + m_legs))
    assert np.allclose(body.center_of_mass(np.zeros(3), STAND).com, expected)


def test_default_com_sits_forward_of_mid_body_because_the_cat_is_front_heavy():
    body = _body()
    c = body.center_of_mass(STRAIGHT, STAND)
    assert c.mass == pytest.approx(4.60530)   # M123
    # Forward of the point midway between the HIPS, but still between them.
    # (M123, ADR-0114: that was the mid-spine point while the hips sat on the
    # spine's ends; they are now 65 mm behind it and 10 mm ahead of it, and the
    # CoM is 25 mm ahead of their midpoint -- 3 mm behind the mid-spine.)
    rear = DEFAULT_SPINE.rear_hip_x
    fore = DEFAULT_SPINE.total_length + DEFAULT_SPINE.front_hip_x
    assert c.x > (rear + fore) / 2.0
    assert rear < c.x < fore
    # ⚠️ **M92 changed the SIGN of this.** The legs still hang below the hips,
    # but the trunk's 3.635 kg now sits on a vertebral chain 49.8 mm above them
    # instead of level with them, and it outweighs the 0.67 kg of leg. The whole
    # body's CoM is **above** the hip axis for the first time. Everything that
    # reads a CoM height -- omega, the DCM, every tipping margin -- moved with it.
    assert c.z > 0.0
    # ⚠️ M92 put the CoM above the hip axis (0.0167). M93's heavier legs pull it
    # back down to **0.0149** -- they hang below the hips, so 20 g per leg at
    # radius counts twice here. Still above the axis, which is the claim.
    # ⚠️ **M102 raised it again, and this is the number the whole milestone
    # is about.** The head's 240 g had never been anywhere: it was spread
    # through the girdle housing at z = 23 mm. Drawn, its CoM is **118.3 mm
    # above the hip axis**, and placing it there lifts the whole body's CoM
    # **14.9 -> 20.1 mm**. That 5.2 mm is what took the default walk from
    # +5.30 mm of ZMP margin to -0.14, and forced the lateral re-tune in
    # `gait.py`.
    # ⚠️ M111's heavier sheaves (+21 g a leg, hanging BELOW the hip axis) pull
    # it back down **20.1 -> 18.5 mm** -- the direction that helps the ZMP
    # margin M102 had to re-tune for, not the one that hurts it.
    # M122: 18.5 -> 19.1 mm -- the legs lost 13 g each below the hip axis.
    # M123: 19.1 -> 18.2 mm -- the girdles' CoM dropped (the leg motors now
    # sit at hip height, ADR-0114) and the legs gained 26 g each.
    assert c.z == pytest.approx(0.0182, abs=5e-4)


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
    """The fore/hind asymmetry must survive into the mass model.

    ⚠️ **M41 (ADR-0046): it survives in GEOMETRY but no longer in MASS.** The legs
    have different link lengths and fold the opposite way, so their CoMs differ —
    that part is unchanged. The mass asymmetry is gone: 0.16716 vs 0.16739 kg,
    because the joint hardware dominates and it is identical on both ends. The
    `mass` assertion here has been dropped rather than inverted; a 0.2 g difference
    is not a fact about feline anatomy.
    """
    body = _body()
    c = body.center_of_mass(STRAIGHT, STAND)
    fore_local = c.legs["LF"].com - body.hip_world_pose(STRAIGHT, "LF")[:2]
    hind_local = c.legs["LR"].com - body.hip_world_pose(STRAIGHT, "LR")[:2]
    assert not np.allclose(fore_local, hind_local)
    assert abs(c.legs["LF"].mass - c.legs["LR"].mass) < 0.001


def test_actuation_mass_matches_the_downselected_motor_and_count():
    # REGRESSION GUARD. The apportionment was once built on 31 CHANNELS x 36 g --
    # a count predating ADR-0008's variable-radius pulley, and a 31 g motor
    # predating the down-select. It is now 19 motors at the SURVEYED REAL mass of
    # 132 g each (motor + integrated driver), not a class target.
    from tomcat_kin.params import DEFAULT_SPINE as sp
    unit = 0.132          # SteadyWin GIM3505-9: 120 g motor + integrated driver
    # ⚠️ M120: plus six leg G3 flexures at 5.20 g (ADR-0111), placed by
    # `girdle_inertia.g3_parts` -- the head, placed, moved nothing.
    assert sp.front_girdle_mass == pytest.approx(
        6 * unit + 0.240 + 0.090 + 0.03118, abs=2e-5)
    # ⚠️ **Seven, not six.** M102 put the 19th motor on this girdle -- the one
    # ADR-0096 bought for the tail and placed on trunk body 0, which this model
    # had never heard of -- plus the 9.8 g tail it drives. Both come out of the
    # structure allowance, so the body total does not move.
    # M120: and its six leg G3 flexures, as on the front.
    # M122: the tail grew 9.8 -> 10.2 g with the 30 mm longer trunk.
    # M123: and shrank to 9.9 g with the 21 mm shorter one (ADR-0114).
    assert sp.rear_girdle_mass == pytest.approx(7 * unit + 0.110 + 0.0099
                                                + 0.03118, abs=6e-5)
    # The pelvis is now the LIGHTER girdle: the spine/tail bank left it for the
    # mid-body bay, which is where the CAD packaging actually puts those motors.
    assert sp.rear_girdle_mass < sp.front_girdle_mass
    # ...and that bank shows up in the MIDDLE spine segment, with the battery.
    assert sp.segment_mass[1] > sp.segment_mass[0] + sp.segment_mass[2]
    # ⚠️ **SIX, not seven -- the "7-motor spine+tail bank" never existed.**
    # That phrase describes the pre-M88 architecture: ADR-0007 bought 3
    # dorsoventral + 3 lateral + 1 tail, and M88 put the tail's motor on trunk
    # body 0 with the hind leg bank, where ADR-0096 found it. M102 moved it to
    # the rear girdle, which is where body 0 maps. What stays here is the
    # battery, the structure allowance and the six SPINE motors -- less the
    # 9.8 g of tail foam, which the same structure allowance pays for.
    # M120: and the six spine G3 flexures, 3.86 g each.
    # M122: the tail foam is 10.2 g now (the trunk is 30 mm longer).
    assert sp.segment_mass[1] == pytest.approx(
        0.130 - 0.0102 + 0.300 + 6 * unit + 0.0232, abs=2e-4)


def test_total_matches_the_revised_NFR5_target():
    # NFR5 was RAISED 3.0 -> 4.05 kg when a real motor was sourced: the smallest
    # buyable part meeting the torque requirement is 132 g, not the 72 g class
    # target, and 19 of them do not fit inside 3 kg. See the motor-reality-check
    # note. A domestic cat is 4-5 kg, so the new figure is if anything more
    # biomimetic -- but it was forced by hardware, not chosen.
    assert _body().mass_budget().total == pytest.approx(4.60530, abs=1e-9)   # M123
