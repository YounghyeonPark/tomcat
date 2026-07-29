"""Tests for the periodic WALK gait generator (M2) + its M4 stability reporting."""

import math

import numpy as np
import pytest

from tomcat_kin import (
    GaitParams,
    GaitController,
    WholeBodyGaitController,
    WholeBody,
    SpineModel,
    LegModel,
    LegMount,
    Girdle,
    UnreachableError,
    centering_shift,
    foot_target,
    swing_height,
    KneeConfig,
)
from tomcat_kin.gait import DEFAULT_PHASE_OFFSETS
from tomcat_kin.params import DEFAULT_FORELEG, DEFAULT_HINDLEG


ctrl = GaitController()
params = ctrl.params


# --------------------------------------------------------------- gait params
def test_default_is_walk_with_offset_set():
    # Four legs, offsets are exactly {0, .25, .5, .75}.
    assert set(params.phase_offsets) == {"LF", "RF", "RR", "LR"}
    assert sorted(params.phase_offsets.values()) == pytest.approx([0.0, 0.25, 0.5, 0.75])


def test_duty_and_speed_relationship():
    p = GaitParams()
    assert p.swing_fraction == pytest.approx(1.0 - p.duty_factor)
    assert p.body_speed == pytest.approx(p.stride_length / (p.duty_factor * p.period))
    assert p.distance_per_cycle == pytest.approx(p.stride_length / p.duty_factor)


def test_invalid_params_raise():
    with pytest.raises(ValueError):
        GaitParams(duty_factor=1.0)
    with pytest.raises(ValueError):
        GaitParams(period=0.0)
    with pytest.raises(ValueError):
        GaitParams(phase_offsets={"LF": 1.0})


# ------------------------------------------------------------- stance counting
@pytest.mark.parametrize("phase", np.linspace(0.0, 1.0, 41))
def test_exactly_three_legs_in_stance(phase):
    # duty 0.75 with tiled offsets => always exactly 3 planted, 1 swinging.
    st = ctrl.state(phase)
    assert st.stance_count == 3
    assert len(st.swing_legs) == 1


def test_stance_count_matches_duty_expectation():
    # Averaged over the cycle the mean stance count == 4 * duty_factor.
    counts = [ctrl.state(p).stance_count for p in np.linspace(0, 1, 200, endpoint=False)]
    assert np.mean(counts) == pytest.approx(4 * params.duty_factor, abs=1e-9)


# ---------------------------------------------------- one stance+swing per cycle
@pytest.mark.parametrize("leg", ["LF", "RF", "RR", "LR"])
def test_one_stance_and_one_swing_block_per_cycle(leg):
    n = 360
    stance = np.array([ctrl.is_stance(i / n, leg) for i in range(n)])
    # Stance fraction ~ duty_factor.
    assert stance.mean() == pytest.approx(params.duty_factor, abs=1.5 / n)
    # Exactly one contiguous swing block (one falling + one rising edge on the
    # circular sequence).
    swing = ~stance
    edges = np.sum(swing != np.roll(swing, 1))
    assert edges == 2


# --------------------------------------------------------------- foot trajectory
@pytest.mark.parametrize("leg", ["LF", "RF", "RR", "LR"])
def test_swing_height_zero_at_touchdown_and_liftoff(leg):
    off = params.phase_offsets[leg]
    d = params.duty_factor
    # Liftoff is at local phase == d, touchdown at local phase == 0 (== 1).
    liftoff_phase = (off + d) % 1.0
    touchdown_phase = off % 1.0
    assert swing_height(params, ctrl.local_phase(liftoff_phase, leg)) == pytest.approx(0.0, abs=1e-9)
    assert swing_height(params, ctrl.local_phase(touchdown_phase, leg)) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("leg", ["LF", "RF", "RR", "LR"])
def test_swing_foot_lifts_mid_swing(leg):
    off = params.phase_offsets[leg]
    d = params.duty_factor
    mid_swing_phase = (off + d + 0.5 * (1.0 - d)) % 1.0
    h = swing_height(params, ctrl.local_phase(mid_swing_phase, leg))
    assert h == pytest.approx(params.step_height, rel=1e-6)
    assert h > 0.0


def test_stance_foot_stays_on_ground_and_sweeps_backward():
    # Over stance the foot z holds at nominal and x decreases monotonically.
    d = params.duty_factor
    us = np.linspace(0.0, d, 30, endpoint=False)
    xs = [foot_target(params, s)[0] for s in us]
    zs = [foot_target(params, s)[1] for s in us]
    assert np.allclose(zs, params.nominal_foot[1])
    assert np.all(np.diff(xs) < 0)  # moving backward
    # Sweep spans the full stride, front (+half) to rear (-half).
    assert xs[0] == pytest.approx(params.nominal_foot[0] + params.stride_length / 2)


def test_swing_returns_foot_forward():
    d = params.duty_factor
    x_liftoff = foot_target(params, d)[0]
    x_touchdown = foot_target(params, 1.0 - 1e-9)[0]
    assert x_touchdown > x_liftoff  # net forward return over swing


# ------------------------------------------------------------------- IK & limits
def test_default_gait_all_reachable_and_within_limits():
    for i in range(120):
        st = ctrl.state(i / 120)
        assert st.all_ok, f"phase {i/120:.3f} not ok"
        for leg in st.legs.values():
            assert leg.q is not None
            assert ctrl.body.legs[leg.name].in_limits(leg.q)


def test_ik_solution_reproduces_foot_target():
    st = ctrl.state(0.3)
    for leg in st.legs.values():
        fk = ctrl.body.leg_model_for(leg.name).forward(leg.q)
        assert np.allclose(fk, leg.foot_target, atol=1e-9)


# ------------------------------------------------- fore/hind ASYMMETRY in gait
def test_gait_uses_fore_and_hind_models_per_girdle():
    # The gait controller solves front-girdle legs with the FORE model and
    # rear-girdle legs with the HIND model (via the WholeBody split).
    for front in ("LF", "RF"):
        assert ctrl.body.leg_model_for(front).params is DEFAULT_FORELEG
    for rear in ("LR", "RR"):
        assert ctrl.body.leg_model_for(rear).params is DEFAULT_HINDLEG


def test_front_and_rear_gait_angles_differ_for_same_local_phase():
    # At a phase where a front leg and a rear leg are at the SAME local phase they
    # share an identical hip-frame foot target, but the fore vs. hind proportions
    # make the solved joint angles differ -- and each leg's FK still lands its own
    # foot on that shared target.
    # LF offset 0.0, RR offset 0.5. Query each at the global phase that puts it at
    # the SAME local phase 0.3: LF at 0.3, RR at 0.8 -> both local phase 0.3.
    lf = ctrl.leg_state(0.3, "LF")   # front / fore, local phase 0.3
    rr = ctrl.leg_state(0.8, "RR")   # rear / hind, local phase 0.3
    assert lf.q is not None and rr.q is not None
    assert lf.local_phase == pytest.approx(rr.local_phase)
    assert np.allclose(lf.foot_target, rr.foot_target)  # same hip-frame target
    assert not np.allclose(lf.q, rr.q, atol=1e-3)        # different joint angles
    # FK check: each solved with its own model reproduces the shared target.
    assert np.allclose(ctrl.body.leg_model_for("LF").forward(lf.q), lf.foot_target, atol=1e-9)
    assert np.allclose(ctrl.body.leg_model_for("RR").forward(rr.q), rr.foot_target, atol=1e-9)


def test_unreachable_target_flagged_not_raised():
    # A gait whose foot targets are far outside the workspace must be flagged.
    bad = GaitParams(nominal_foot=(5.0, -0.13))
    c = GaitController(params=bad)
    st = c.state(0.1)
    assert not st.all_ok
    assert any((not lg.reachable) for lg in st.legs.values())
    # Flagged, not crashed: q is None for the unreachable legs.
    assert any(lg.q is None for lg in st.legs.values())


# ------------------------------------------------------------------- phase wrap
def test_phase_wraps_at_one():
    a = ctrl.state(0.0)
    b = ctrl.state(1.0)
    assert a.phase == pytest.approx(b.phase)
    for name in a.legs:
        assert np.allclose(a.legs[name].foot_target, b.legs[name].foot_target)
        assert a.legs[name].in_stance == b.legs[name].in_stance


def test_state_at_time_matches_phase():
    p = params
    t = 0.37 * p.period
    st_t = ctrl.state_at_time(t)
    st_p = ctrl.state(0.37)
    assert st_t.phase == pytest.approx(st_p.phase)


def test_time_beyond_period_wraps():
    st = ctrl.state_at_time(1.25 * params.period)
    assert st.phase == pytest.approx(0.25)


# ------------------------------------------------------------- spine coupling
def test_spine_neutral_by_default():
    st = ctrl.state(0.42)
    assert np.allclose(st.spine_q, 0.0)


def test_spine_oscillation_when_enabled():
    p = GaitParams(spine_amplitude=math.radians(5.0))
    c = GaitController(params=p)
    n = ctrl.body.spine.params.n_segments
    # Non-trivial and bounded by amplitude; zero-mean over the cycle.
    samples = np.array([c.spine_q(i / 100) for i in range(100)])
    assert samples.shape == (100, n)
    assert np.abs(samples).max() > 0.0
    assert np.abs(samples).max() <= math.radians(5.0) + 1e-12
    assert np.allclose(samples.mean(axis=0), 0.0, atol=1e-9)
    # And it stays within spine joint limits.
    lo = np.asarray(c.body.spine.params.q_min)
    hi = np.asarray(c.body.spine.params.q_max)
    assert np.all(samples >= lo - 1e-12) and np.all(samples <= hi + 1e-12)


# ===================================================================
# M4 — real mass: CoM + static stability margin per gait phase
# ===================================================================
#
# HONEST FINDING (do not "fix" by retuning the gait). Adding the CoM/support
# check exposes a defect in the PLACEHOLDER LEG GEOMETRY, not in the gait timing:
# with the current link lengths, joint limits and paw offset, a leg CANNOT put
# its foot under its own hip at stance height -- the reachable, in-limit foot x
# at z = -0.13 m is roughly +0.16..+0.24 m FORWARD of the hip. Every foot is
# therefore ~0.2 m ahead of its own hip, the whole support interval sits ahead of
# the body, and the CoM (which lives between the two girdles) can never be inside
# it. The tests below pin that finding down AND prove the machinery reports a
# healthy margin once the support base actually straddles the CoM.

# Mount offset that moves the hips (and hence the feet) rearward enough for the
# support base to straddle the CoM. Derived from `centering_shift` on the default
# body; used ONLY as a diagnostic what-if, it is not a committed design value.
_MISPLACED_HIP_OFFSET = (-0.22, 0.0)


def _misplaced_body():
    """WholeBody with deliberately shoved hips, so the feet do NOT land under the
    trunk. Used only to prove the stability check can still detect instability."""
    mounts = {
        name: LegMount(name, girdle, _MISPLACED_HIP_OFFSET)
        for name, girdle in (
            ("LF", Girdle.FRONT), ("RF", Girdle.FRONT),
            ("LR", Girdle.REAR), ("RR", Girdle.REAR),
        )
    }
    return WholeBody(spine=SpineModel(), mounts=mounts)


def test_gait_state_reports_com_and_stability():
    st = ctrl.state(0.3)
    assert st.com is not None and st.stability is not None
    assert st.com.mass == pytest.approx(ctrl.body.total_mass)
    assert st.com.mass == pytest.approx(3.0)
    # The state's convenience flag agrees with the margin object.
    assert st.is_statically_stable == st.stability.is_stable
    assert ctrl.stability(0.3).margin == pytest.approx(st.stability.margin)
    assert np.allclose(ctrl.center_of_mass(0.3).com, st.com.com)


def test_support_interval_uses_only_the_three_stance_feet():
    st = ctrl.state(0.3)
    assert st.stability.support.n_feet == 3
    assert set(st.stability.support.feet) == set(st.stance_legs)
    assert st.swing_legs[0] not in st.stability.support.feet


def test_leg_can_place_a_foot_under_its_own_hip():
    # REGRESSION GUARD for the M4 finding. On the old POSITIVE-knee branch a leg
    # could not plant a paw under its own hip (it demanded a ~+167 deg hip), so
    # every foot landed ~0.2 m forward and the walk was fore/aft unstable. On the
    # anatomical NEGATIVE-knee fold the paw reaches back to / behind the hip.
    z = ctrl.params.nominal_foot[1]
    for p in (DEFAULT_FORELEG, DEFAULT_HINDLEG):
        leg = LegModel(p)
        reachable_x = []
        for x in np.arange(-0.12, 0.30, 0.005):
            for knee in KneeConfig:
                try:
                    q = leg.inverse((x, z, ctrl.params.foot_pitch), knee=knee)
                except UnreachableError:
                    continue
                if leg.in_limits(q):
                    reachable_x.append(x)
                    break
        assert reachable_x, "leg has no in-limit stance pose at all"
        # The stance workspace now straddles the hip (x = 0), not just ahead of it.
        assert min(reachable_x) <= 0.0
        assert max(reachable_x) > 0.10


def test_default_walk_is_fore_aft_STATICALLY_STABLE():
    # The payoff of the negative-knee fix: with the paws under the trunk the CoM
    # stays inside the support interval for the whole cycle.
    margins = ctrl.stability_sweep(60)
    assert len(margins) == 60
    assert all(m.is_stable for m in margins)
    assert all(m.support.n_feet == 3 for m in margins)
    assert all(m.normalized_margin > 0.0 for m in margins)
    worst = min(m.margin for m in margins)
    assert worst > 0.02                     # > 20 mm of margin at every phase
    # The support base now STRADDLES the body centre of mass.
    st = ctrl.state(0.0)
    assert st.stability.support.rear < st.com.x < st.stability.support.front


def test_centering_shift_is_small_now_that_the_geometry_is_consistent():
    # centering_shift reports the fore-aft correction still needed. Pre-fix it was
    # ~170 mm (a real geometry defect); it should now be a small trim, not a
    # structural error.
    d = centering_shift(ctrl.stability(0.0))
    assert d < 0.06


def test_stability_check_still_detects_a_deliberately_misplaced_body():
    # Guard that the margin is measuring something real: shove the hips far off
    # so the feet no longer sit under the trunk and the SAME gait must go unstable.
    c = GaitController(body=_misplaced_body())
    margins = c.stability_sweep(120)
    assert any(not m.is_stable for m in margins)


def test_stability_margin_agrees_between_hip_frame_and_world_frame_controllers():
    # The margin is translation-invariant, and both controllers solve the same
    # posture at spine-neutral, so they must agree exactly.
    world = WholeBodyGaitController(params=GaitParams())
    for i in range(13):
        phase = i / 13
        assert world.stability(phase).margin == pytest.approx(
            ctrl.stability(phase).margin, abs=1e-12
        )
    st = world.state(0.4)
    assert st.com is not None
    assert st.is_statically_stable == st.stability.is_stable


def test_arching_the_spine_moves_the_gait_com_rearward():
    # Spine coupling must show up in the CoM (and therefore in the margin).
    flat = GaitController(params=GaitParams(spine_amplitude=0.0))
    arch = GaitController(params=GaitParams(spine_amplitude=math.radians(15.0)))
    p = 0.25   # sine peak => maximum dorsiflexion
    assert np.abs(arch.state(p).spine_q).max() > 0.0
    assert arch.state(p).com.x < flat.state(p).com.x
    assert arch.state(p).com.z > flat.state(p).com.z


def test_unsolvable_legs_still_conserve_mass_and_are_excluded_from_support():
    bad = GaitController(params=GaitParams(nominal_foot=(5.0, -0.13)))
    st = bad.state(0.1)
    assert not st.all_ok
    assert st.com.mass == pytest.approx(bad.body.total_mass)
    # No trustworthy contacts -> no support interval -> not stable.
    assert st.stability.support.n_feet == 0
    assert not st.is_statically_stable
