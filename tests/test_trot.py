"""Tests for the M7 diagonal TROT — the project's first dynamic gait.

A trot puts diagonal pairs down together, so the support degenerates from a
polygon to a LINE. Everything the static milestones built (support polygon, ZMP
margin) stops applying, and the inverted-pendulum picture takes over. These tests
pin the three things that make the difference between a trot and a fall.
"""

import numpy as np
import pytest

from tomcat_kin import GaitController, GaitParams
from tomcat_kin import dynamics as dyn
from tomcat_kin.gait import trot_params, TROT_PHASE_OFFSETS
from tomcat_kin.params import DEFAULT_TENDON

ARMS = np.asarray(DEFAULT_TENDON.joint_moment_arm)
SPOOL = DEFAULT_TENDON.motor_spool_radius
MOTOR_PEAK, MOTOR_RATED = 1.95, 0.71        # SteadyWin GIM3505-9, output shaft


def _roll_drift(c, n=96):
    """Roll rate gained per cycle. Non-zero => the body tips further every stride."""
    cyc = dyn.cycle(c, n)
    sg = []
    for i in range(n):
        b = dyn.line_balance(c, i / n, n, cyc=cyc)
        sg.append(np.sign(b.offset) * b.unbalanced_moment if b else 0.0)
    h = float(np.mean(cyc.com[:, 2] - cyc.ground_z))
    I = c.body.total_mass * h * h
    return float(np.sum(np.array(sg) / I) * (c.params.period / n))


# ------------------------------------------------------------------ structure
def test_trot_is_diagonal_pairs_on_a_line_support():
    c = GaitController(params=trot_params())
    assert set(TROT_PHASE_OFFSETS) == {"LF", "RF", "LR", "RR"}
    for i in range(48):
        st = c.state(i / 48)
        assert st.stance_count == 2
        # the two planted feet are always a DIAGONAL pair
        assert set(st.stance_legs) in ({"LF", "RR"}, {"RF", "LR"})


def test_static_stability_tools_correctly_refuse_a_line_support():
    # Not a bug: a line has no interior, so "is the CoM inside the polygon?" has
    # no meaning. The milestone's point is that a trot needs different physics.
    c = GaitController(params=trot_params())
    with pytest.raises(ValueError):
        c.support_polygon(0.25)
    with pytest.raises(ValueError):
        dyn.zero_moment_point(c, 0.25, 48)


def test_two_contacts_cannot_balance_the_moment_about_their_own_line():
    # The contact solve leaves a residual, and it is physical, not numerical:
    # it equals m*g*(perpendicular offset of the CoM from the support line).
    c = GaitController(params=trot_params())
    cyc = dyn.cycle(c, 96)
    # Pick the phase where the CoM is furthest off the support line.
    i = max(range(96), key=lambda k: abs(dyn.line_balance(c, k / 96, 96, cyc=cyc).offset))
    sol = dyn.contact_forces(c, i / 96, 96, cyc=cyc)
    bal = dyn.line_balance(c, i / 96, 96, cyc=cyc)
    assert sol.residual > 1e-3                       # NOT balanced -- physical
    # Same order of magnitude as m*g*offset. Not equal: the residual is the norm
    # of the whole 6-vector of unsatisfied balance equations, and the rank-5
    # least-norm solve spreads the error over its components.
    assert 0.3 < sol.residual / bal.unbalanced_moment < 3.0
    # A 3-foot posture, by contrast, IS balanced exactly.
    assert dyn.contact_forces(GaitController(), 0.2, 96).residual < 1e-9


# ----------------------------------------------------- the balance requirement
def test_trot_foot_placement_makes_the_roll_bounded():
    # THE M7 RESULT. The topple moment reverses within each stance, so it
    # integrates to ~zero over a cycle and the roll is a bounded oscillation
    # rather than a fall.
    c = GaitController(params=trot_params())
    assert abs(_roll_drift(c)) < 0.10                # rad/s gained per cycle
    r = dyn.trot_sweep(c, 96)
    assert r["crosses_zero"]                          # CoM rocks THROUGH the line
    assert r["offset_min"] < 0 < r["offset_max"]
    # roughly symmetric about zero -- that is what kills the drift
    assert abs(r["offset_min"] + r["offset_max"]) < 0.005


def test_the_CRAWL_foot_placement_would_fall_over_in_one_stride():
    # REGRESSION GUARD for why trot_params exists. The crawl plants its feet
    # 50 mm ahead of the hips, which puts the diagonal ~42 mm forward of the CoM:
    # a one-signed topple moment, so roll rate accumulates every cycle.
    bad = GaitController(params=trot_params(nominal_foot=(0.05, -0.17)))
    good = GaitController(params=trot_params())
    assert abs(_roll_drift(bad)) > 20 * abs(_roll_drift(good))
    assert abs(_roll_drift(bad)) > 1.0               # rad/s per cycle -- a fall


def test_capture_point_stays_within_reach():
    # The topple has to be caught by the NEXT diagonal. The DCM says how far that
    # foothold must reach; it must be small compared with the stride.
    c = GaitController(params=trot_params())
    r = dyn.trot_sweep(c, 96)
    assert r["dcm_abs_max"] < c.params.stride_length


# -------------------------------------------- the swing trajectory (C1 defect)
def test_matched_swing_profile_removes_the_velocity_step():
    # The legacy cycloid starts and ends swing at ZERO hip-frame velocity while
    # stance sweeps at -stride/(duty*period) -- so the foot velocity STEPS at both
    # liftoff and touchdown. That is an acceleration impulse AND a landing scuff.
    from tomcat_kin import foot_target
    n = 4000
    peak = {}
    for profile in ("cycloid", "matched"):
        p = trot_params(swing_profile=profile)
        dt = p.period / n
        x = np.array([foot_target(p, i / n)[0] for i in range(n)])
        peak[profile] = float(np.abs((np.roll(x, -1) - 2 * x + np.roll(x, 1)) / (dt * dt)).max())
    # ~8900 m/s^2 vs ~51: the cycloid's is a grid artefact of a true impulse.
    assert peak["cycloid"] > 50 * peak["matched"]


def test_matched_profile_lands_the_foot_at_the_stance_speed():
    # Zero velocity relative to the GROUND at touchdown => no scuff.
    from tomcat_kin import foot_target
    p = trot_params()
    sweep = -p.stride_length / (p.duty_factor * p.period)
    n = 20000
    dt = p.period / n
    j = n - 2                                    # just before touchdown
    v = (foot_target(p, (j + 1) / n)[0] - foot_target(p, (j - 1) / n)[0]) / (2 * dt)
    assert v == pytest.approx(sweep, rel=0.05)


def test_swing_torque_is_small_on_matched_and_inflated_on_cycloid():
    # Swing-leg torque is what caps trot speed, and it is only computable on a C1
    # trajectory. On the cycloid the impulse inflates it ~9x -- which would have
    # made the trot look infeasible for the wrong reason.
    def peak(profile, n):
        c = GaitController(params=trot_params(swing_profile=profile))
        out = 0.0
        for i in range(n):
            for nm in ("LF", "RF", "LR", "RR"):
                if c.is_stance(i / n, nm):
                    continue
                t = np.abs(dyn.swing_joint_torque(c, nm, i / n, n))
                out = max(out, float((t / ARMS * SPOOL).max()))
        return out

    # The discriminator is GRID CONVERGENCE, not a single value: a true impulse
    # has no finite peak, so the cycloid's number DOUBLES with every refinement
    # while the matched profile's settles.
    m = [peak("matched", n) for n in (48, 96, 192)]
    cy = [peak("cycloid", n) for n in (48, 96, 192)]
    assert max(m) / min(m) < 1.10                 # converged
    assert cy[-1] > 3 * cy[0]                     # diverging with the grid
    assert m[-1] < 0.3          # and small: tendon drive keeps the legs light


# ----------------------------------------------------------------- feasibility
def test_trot_is_within_the_real_motor_envelope():
    # Peak against the part's PEAK, and RMS against its CONTINUOUS rating -- the
    # correct thermal comparison. (ADR-0010 compared a quasi-static PEAK against
    # the continuous rating and wrongly concluded trot was a 2.1x overload.)
    c = GaitController(params=trot_params())
    n = 96
    cyc = dyn.cycle(c, n)
    hist = []
    for i in range(n):
        sol = dyn.contact_forces(c, i / n, n, cyc=cyc)
        st = c.state(i / n)
        for nm in ("LF", "RF", "LR", "RR"):
            q = st.legs[nm].q
            if q is None:
                continue
            if nm in sol.forces:
                F = sol.forces[nm]
                t = c.body.leg_model_for(nm).jacobian(q).T @ np.array([F[0], F[2], 0.0])
            else:
                t = dyn.swing_joint_torque(c, nm, i / n, n)
            hist.append(np.abs(t) / ARMS * SPOOL)
    hist = np.array(hist)
    assert hist.max() < MOTOR_PEAK
    assert float(np.sqrt((hist ** 2).mean())) < MOTOR_RATED     # thermally OK


def test_trot_is_far_faster_than_the_statically_stable_crawl():
    trot = GaitController(params=trot_params())
    crawl = GaitController()                       # the M6 shipped crawl
    assert trot.params.body_speed > 40 * crawl.params.body_speed
    assert trot.params.body_speed > 0.5            # m/s -- cat-like


def test_trot_stays_well_inside_the_friction_cone():
    c = GaitController(params=trot_params())
    cyc = dyn.cycle(c, 96)
    mu = max(dyn.contact_forces(c, i / 96, 96, cyc=cyc).aggregate_mu for i in range(96))
    assert mu < 0.5


# ------------------------------------ contact sensing (TACTILE_SENSING_SPEC)
def test_ground_force_is_ill_conditioned_on_the_HIND_legs_near_liftoff():
    # THE QUANTITATIVE CASE FOR A PAW SENSOR. ADR-0004's joint-end load cells can
    # in principle recover the foot force (a point contact exerts no moment, so
    # tau = J^T F has only two unknowns). On the FORE legs that inversion is well
    # conditioned; on the HIND legs it degrades ~10x just before liftoff, exactly
    # when load transfers to the other diagonal.
    c = GaitController(params=trot_params())
    obs = dyn.grf_observability_sweep(c, 96)
    assert obs["LF"]["worst"] < 5.0
    assert obs["RF"]["worst"] < 5.0
    assert obs["LR"]["worst"] > 20.0
    assert obs["RR"]["worst"] == pytest.approx(obs["LR"]["worst"], rel=1e-6)
    assert obs["LR"]["median"] > 2 * obs["LF"]["median"]


def test_paw_sensor_mass_budget_is_what_limits_it_not_the_mass_budget():
    # TACTILE_SENSING_SPEC §4: distal mass is charged against SWING TORQUE, which
    # M7 showed caps trot speed -- not against the 4.05 kg total, where 4x20 g is
    # a rounding error. Guards the "<= 20 g per paw" rule.
    import dataclasses
    from tomcat_kin import LegModel
    from tomcat_kin.spine import WholeBody, SpineModel
    from tomcat_kin.params import DEFAULT_FORELEG, DEFAULT_HINDLEG

    def swing_peak(paw_g, n=48):
        add = paw_g / 1000.0
        fo = dataclasses.replace(DEFAULT_FORELEG, link_mass=tuple(
            np.array(DEFAULT_FORELEG.link_mass) + np.array([0, 0, 0, add])))
        hi = dataclasses.replace(DEFAULT_HINDLEG, link_mass=tuple(
            np.array(DEFAULT_HINDLEG.link_mass) + np.array([0, 0, 0, add])))
        b = WholeBody(spine=SpineModel(), legs={"LF": LegModel(fo), "RF": LegModel(fo),
                                                "LR": LegModel(hi), "RR": LegModel(hi)})
        c = GaitController(params=trot_params(), body=b)
        out = 0.0
        for i in range(n):
            for nm in ("LF", "RF", "LR", "RR"):
                if c.is_stance(i / n, nm):
                    continue
                t = np.abs(dyn.swing_joint_torque(c, nm, i / n, n))
                out = max(out, float((t / ARMS * SPOOL).max()))
        return out, b.total_mass

    base, m0 = swing_peak(0)
    at20, m20 = swing_peak(20)
    # 20 g/paw costs ~40% of the swing term but only ~2% of body mass.
    assert 1.25 < at20 / base < 1.6
    assert (m20 - m0) / m0 < 0.03
