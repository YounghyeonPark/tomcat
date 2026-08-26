# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Tests for M8 closed-loop balance: step-to-step foot placement.

M7 showed the trot's nominal trajectory is dynamically consistent — i.e. that the
error STARTS at zero. These tests are about what happens when it does not.
"""

import numpy as np
import pytest

from tomcat_kin import GaitController
from tomcat_kin.gait import trot_params
from tomcat_kin import control as ctl


def _plant():
    return ctl.StepPlant.from_gait(GaitController(params=trot_params()), 96)


# ------------------------------------------------------------------- the plant
def test_plant_matches_the_full_dynamics_model():
    # The reduced model earns its keep only if its parameters come from the real
    # one, not from a textbook.
    from tomcat_kin import dynamics as dyn
    c = GaitController(params=trot_params())
    p = _plant()
    b = dyn.line_balance(c, 0.1, 96)
    assert p.omega == pytest.approx(b.omega, rel=0.02)
    assert p.stance == pytest.approx(c.params.duty_factor * c.params.period)
    assert p.growth == pytest.approx(np.exp(p.omega * p.stance))
    assert p.growth > 3.0              # ~3.2x per step: genuinely unstable


def test_open_loop_diverges():
    # THE PREMISE OF THE MILESTONE. Without foot-placement feedback a small error
    # is multiplied every step -- M7's "dynamically consistent" says nothing here.
    p = _plant()
    traj = ctl.simulate(p, 8, xi0=0.005, closed_loop=False)
    assert traj[-1] > 100 * traj[0]
    assert np.all(np.diff(np.abs(traj)) > 0)      # monotonically worse


# --------------------------------------------------------------- the control law
def test_capture_arrests_but_does_NOT_recover():
    # Placing the foot exactly AT the DCM stops the topple and leaves the body
    # permanently displaced. Kept as a distinct function because confusing it with
    # recovery is a quiet, plausible-looking bug: the robot is stable and walks
    # away sideways.
    p = _plant()
    xi = 0.02
    for _ in range(10):
        xi = p.propagate(xi, ctl.capture_placement(p, xi))
    assert xi == pytest.approx(0.02, rel=1e-9)     # held, not recovered


def test_placement_overshoots_the_dcm():
    # The recovery law asks for MORE than the DCM: (growth-beta)/(growth-1) > 1.
    p = _plant()
    xi = 0.02
    assert ctl.placement(p, xi, beta=0.0) > xi
    assert ctl.placement(p, xi, beta=0.0) == pytest.approx(
        p.growth / (p.growth - 1.0) * xi, rel=1e-9)


def test_deadbeat_recovers_in_exactly_one_step():
    p = _plant()
    traj = ctl.simulate(p, 6, xi0=0.03, beta=0.0)
    assert abs(traj[1]) < 1e-9
    assert np.all(np.abs(traj[1:]) < 1e-9)


@pytest.mark.parametrize("beta", [0.3, 0.5, 0.7])
def test_beta_sets_the_convergence_rate(beta):
    p = _plant()
    traj = ctl.simulate(p, 6, xi0=0.03, beta=beta)
    for a, b in zip(traj[1:], traj[2:]):
        assert b == pytest.approx(beta * a, rel=1e-9)


# ------------------------------------------------------------ what actually binds
def test_envelope_is_set_by_REACH_not_by_gain():
    # The physical limit is how far the leg can be put, not how hard the law
    # pushes. An earlier version ran only 12 steps and made a SLOW gain look like
    # a smaller envelope -- that was horizon-limited, a different and misleading
    # statement.
    p = _plant()
    envs = [ctl.rejection_envelope(p, beta=b) for b in (0.0, 0.3, 0.5)]
    assert max(envs) - min(envs) < 1e-3          # gain-independent
    assert all(e > 0.02 for e in envs)


def test_the_binding_direction_is_REARWARD():
    # The leg reaches further forward than back, so a disturbance needing a
    # rearward foothold is the one that limits the robot.
    p = _plant()
    assert abs(p.reach[0]) < abs(p.reach[1])
    guaranteed = ctl.rejection_envelope(p, beta=0.0)
    assert guaranteed == pytest.approx(abs(p.reach[0]), rel=0.05)


def test_one_step_envelope_is_smaller_than_the_multi_step_one():
    # Once the placement saturates the error is not nulled in one step, but a
    # clamped foot still REDUCES it, so recovery continues over several steps.
    p = _plant()
    assert ctl.one_step_envelope(p, 0.0) < ctl.rejection_envelope(p, beta=0.0)
    assert ctl.one_step_envelope(p, 0.0) == pytest.approx(
        min(abs(p.reach[0]), abs(p.reach[1])) * (p.growth - 1.0) / p.growth, rel=1e-9)


def test_large_enough_disturbance_is_unrecoverable():
    p = _plant()
    beyond = ctl.rejection_envelope(p, beta=0.0) * 3.0
    traj = ctl.simulate(p, 20, xi0=beyond, beta=0.0)
    assert abs(traj[-1]) > abs(traj[0])          # runs away


# ----------------------------------------------- the link back to paw sensing
def test_sensing_bias_becomes_a_PERMANENT_offset_amplified_by_the_growth():
    # THE REQUIREMENT ADR-0012 HAS TO MEET. A steady error in the estimated DCM
    # does not average out -- it settles into a fixed lateral offset, amplified by
    # exactly the per-step growth. The relationship holds while the placement is
    # unsaturated; past that the loop runs away entirely, which is a sharper
    # statement of the same requirement.
    p = _plant()
    for bias in (0.002, 0.005):
        traj = ctl.simulate(p, 60, xi0=0.03, beta=0.0, estimation_error=bias)
        assert traj[-1] == pytest.approx(-p.growth * bias, rel=0.02)
    # M15: the shipped 0.4 s period has growth 4.7 (up from 3.2 at 0.3 s), so a
    # bias that was merely an offset before is now unrecoverable.
    assert p.growth > 4.0
    runaway = ctl.simulate(p, 60, xi0=0.03, beta=0.0, estimation_error=0.010)
    assert abs(runaway[-1]) > 0.2


# ===================================================================
# M9 — the projection correction, the spine as a balance actuator,
#      latency, and why retiming does not help
# ===================================================================

def test_reach_is_PROJECTED_onto_the_support_line_perpendicular():
    # THE M9 CORRECTION. The DCM lives perpendicular to the diagonal, and that
    # direction is ~90% LATERAL. The legs are sagittal-only (no abduction, and the
    # track is fixed), so a fore-aft foothold shift buys perpendicular authority
    # only through its ~0.44 projection. M8 used the raw fore-aft range and so
    # OVERSTATED the disturbance envelope by ~2.3x.
    from tomcat_kin import dynamics as dyn
    c = GaitController(params=trot_params())
    p = _plant()
    assert 0.40 < p.projection < 0.50
    # cross-check the projection against the actual support-line geometry
    cyc = dyn.cycle(c, 96)
    _, d = dyn.support_line(cyc, 24)
    assert p.projection == pytest.approx(abs(-d[1]), rel=1e-6)
    # and the stored reach is the projected one, not the raw leg range
    assert abs(p.reach[0]) < 0.05          # ~33 mm, not the raw ~74 mm


def test_the_LATERAL_SPINE_roughly_doubles_the_envelope():
    # The perpendicular is ~90% lateral, which is exactly where the sagittal legs
    # are weakest -- and precisely where the ADR-0009 spine bend pushes. Hardware
    # bought for the CRAWL's static stability turns out to be the dominant
    # DYNAMIC balance actuator for the trot.
    p = _plant()
    assert p.spine > 0.015                              # ~24 mm within one stance
    feet_only = ctl.rejection_envelope(p, use_spine=False)
    with_spine = ctl.rejection_envelope(p, use_spine=True)
    assert with_spine > 1.8 * feet_only


def test_spine_assist_opposes_the_error_and_is_bounded():
    p = _plant()
    assert ctl.spine_assist(p, +0.05) == pytest.approx(-p.spine)   # saturated, opposing
    assert ctl.spine_assist(p, -0.05) == pytest.approx(+p.spine)
    small = 0.5 * p.spine
    assert ctl.spine_assist(p, small) == pytest.approx(-small)     # unsaturated


def test_latency_degrades_the_envelope_smoothly():
    # Latency is handled by predicting forward -- prediction is not the problem.
    # The costs are that estimation error is amplified by e^(omega*tau), and that
    # less time remains under the corrective placement.
    c = GaitController(params=trot_params())
    envs = [ctl.rejection_envelope(
                ctl.StepPlant.from_gait(c, latency=t), use_spine=True)
            for t in (0.0, 0.010, 0.020, 0.040)]
    assert all(a > b for a, b in zip(envs, envs[1:]))     # monotonic decay
    assert envs[2] > 0.75 * envs[0]                      # 20 ms costs < 25 %
    assert all(e > 0 for e in envs)                      # no artificial cliff


def test_retiming_speeds_recovery_but_does_NOT_extend_the_envelope():
    # With the placement saturated at reach R: xi_end = R + (e-R)*e^(wT).
    # For e < R the bracket is negative, so a LONGER stance amplifies the
    # correction -- recovery gets faster. For e > R it is positive and grows for
    # any T; as T->0 it merely holds. So timing buys speed, never range.
    import math
    p = _plant()
    R = abs(p.reach[0])
    w = p.omega
    for e in (0.5 * R, 0.9 * R):                 # inside the envelope
        T = math.log(R / (R - e)) / w
        assert T > 0
        assert R + (e - R) * math.exp(w * T) == pytest.approx(0.0, abs=1e-9)
    for e in (1.1 * R, 2.0 * R):                 # outside it
        for T in (0.01, 0.05, 0.15, 0.5):
            assert abs(R + (e - R) * math.exp(w * T)) >= e - 1e-9


# ===================================================================
# M10 — the spine is ROM-limited, not rate-limited; and both M9
#       follow-ups (abduction, faster drive) close as "not needed"
# ===================================================================

def test_spine_is_ROM_limited_not_RATE_limited():
    # M9 clamped the spine with NFR2f's 119 deg/s. That is a REQUIREMENT floor
    # sized for the ADR-0007 righting reflex, not a capability. The drive can do
    # ~912 deg/s at the joint (380 rpm motor x the 8/20 mm spool-to-arm ratio),
    # while traversing the full +/-15 deg ROM inside a 150 ms stance needs only
    # 200 deg/s. Using the requirement as the capability under-counted the spine
    # by ~40 %.
    import math
    from tomcat_kin.params import DEFAULT_SPINE as sp
    joint_dps = math.degrees(2 * math.pi * 380.0 / 60.0
                             * sp.motor_spool_radius / sp.lateral_moment_arm[0])
    assert joint_dps > 800.0                    # ~912 deg/s
    assert joint_dps > 4 * 200.0                # 4x what a full traverse needs

    p = _plant()
    rom = abs(sp.lateral_q_min[0])
    c = GaitController(params=trot_params())
    # M20: the sway must be evaluated with the fore legs where they actually are.
    # `center_of_mass_y` defaults to putting them at the spine tip, which overstates
    # it by 4 % — their CoM sits ~52 mm behind the hip and the yaw swings it back.
    mid = c.state(0.25)
    pose = {nm: mid.legs[nm].q for nm in c.body.leg_names if mid.legs[nm].q is not None}
    full = abs(c.body.center_of_mass_y(np.full(3, rom), pose))
    perp = math.sqrt(1.0 - p.projection ** 2)
    assert p.spine == pytest.approx(full * perp, rel=1e-6)   # the FULL ROM is usable

    # And the correction is real, not a rounding change.
    naive = abs(c.body.center_of_mass_y(np.full(3, rom)))
    assert naive > full
    # ⚠️ M41 (ADR-0046): **4.0 % -> 7.1 %.** ADR-0025's correction is the fore
    # legs' fore-aft CoM offset being rotated into y by the spine's yaw, so its size
    # is set by WHERE the leg mass sits. The manufacturing model moved that mass
    # distally (the metatarsus more than doubled), which lengthens the lever and
    # nearly doubles the error the naive form makes. The finding is unchanged and
    # its magnitude grew.
    assert (naive - full) / naive == pytest.approx(0.071, abs=0.006)


def test_envelope_in_physical_units_is_a_real_shove():
    # A DCM envelope is abstract; xi = c + c_dot/omega, so a pure velocity
    # disturbance v maps to xi = v/omega. That is the number to judge.
    p = _plant()
    env = ctl.rejection_envelope(p, use_spine=True)
    # ⚠️ M41: ~90 -> **73 mm**. The zero-latency with-spine figure fell with the
    # spine's own authority (42.2 -> 37.0 mm of sway, below), and this is the
    # idealised number ADR-0014/0015 quoted before latency was modelled at all.
    assert env > 0.065                           # ~73 mm
    assert env * p.omega > 0.6                   # rejects a >0.6 m/s lateral shove


def test_spine_dominates_foot_placement_for_lateral_balance():
    # The perpendicular is ~90 % lateral. The sagittal legs reach it only through
    # a 0.44 projection; the spine pushes almost straight down it. So the spine --
    # bought for the CRAWL's static stability -- outweighs foot placement here.
    p = _plant()
    feet_only = ctl.rejection_envelope(p, use_spine=False)
    with_spine = ctl.rejection_envelope(p, use_spine=True)
    assert p.spine > abs(p.reach[0])             # spine beats the binding reach
    # M20 moved this from 2.53x to 2.47x: the sway correction took 4 % off the
    # spine and none off the feet. The threshold is loosened because the FINDING
    # moved it, not to make a failure go away -- the claim under test is that the
    # spine dominates, and 2.47x says so as plainly as 2.53x did.
    assert with_spine > 2.4 * feet_only


# ===================================================================
# M11 — the latency budget solved as a fixed point
# ===================================================================

def test_actuation_time_scales_with_the_correction_and_the_projection():
    # The correction travels along the perpendicular, but sagittal legs deliver it
    # only through their 0.44 projection -- so the fore-aft foot travel, and hence
    # the time, is 2.3x the correction. Checked on the CONSTANT-VELOCITY model,
    # where the relationship is exactly linear; M12's ramp makes the shipped
    # default mildly super-linear (see the trapezoid test below).
    import math
    p = _plant()
    inf = math.inf
    t_small = ctl.actuation_time(p, 0.010, accel_limit=inf)
    t_big = ctl.actuation_time(p, 0.020, accel_limit=inf)
    assert t_big == pytest.approx(2 * t_small, rel=1e-9)
    coeff = p.growth / (p.growth - 1.0)
    expected = coeff * 0.010 / p.projection / ctl.SPARE_FOOT_SPEED
    assert t_small == pytest.approx(expected, rel=1e-9)
    # and it saturates once the placement hits the reach limit
    assert ctl.actuation_time(p, 1.0, accel_limit=inf) == pytest.approx(
        abs(p.reach[1]) / p.projection / ctl.SPARE_FOOT_SPEED, rel=1e-9)
    # the shipped (ramped) default is monotonic and always slower
    assert ctl.actuation_time(p, 0.020) > ctl.actuation_time(p, 0.010)
    assert ctl.actuation_time(p, 0.020) > t_big


def test_envelope_solved_as_a_fixed_point_is_smaller_than_the_zero_latency_one():
    # THE M11 CORRECTION. ADR-0014/0015 quoted ~90 mm, which assumed zero latency.
    # Latency is not independent: correction size sets actuation time, which IS the
    # staleness the controller commits on.
    c = GaitController(params=trot_params())
    ideal = ctl.rejection_envelope(_plant(), use_spine=True)
    real = ctl.self_consistent_envelope(c, pipeline=0.0075)
    assert ideal > 0.075                          # ~81 mm ideal (zero-latency)
    assert 0.045 < real["envelope"] < 0.065       # ~52.7 mm once latency is real
    assert real["envelope"] < 0.75 * ideal


def test_ACTUATION_dominates_the_pipeline():
    # The headline consequence: electronics and firmware are not the bottleneck.
    c = GaitController(params=trot_params())
    r = ctl.self_consistent_envelope(c, pipeline=0.0075)
    assert r["actuation"] > 4 * r["pipeline"]
    assert r["latency"] == pytest.approx(r["pipeline"] + r["actuation"], rel=1e-9)


def test_envelope_is_nearly_insensitive_to_the_electronics_pipeline():
    # 2.5 -> 20 ms of pipeline costs only ~16 % of the envelope, so chasing
    # microseconds on the bus would be effort in the wrong place. Foot speed is
    # the lever.
    c = GaitController(params=trot_params())
    fast = ctl.self_consistent_envelope(c, pipeline=0.0025)["envelope"]
    slow = ctl.self_consistent_envelope(c, pipeline=0.020)["envelope"]
    assert slow > 0.80 * fast                     # 8x the pipeline, <20 % lost


def test_faster_feet_buy_more_envelope_than_faster_electronics():
    # Quantifies the recommendation in the latency-budget note.
    c = GaitController(params=trot_params())
    base = ctl.self_consistent_envelope(c, pipeline=0.0075)["envelope"]
    no_electronics = ctl.self_consistent_envelope(c, pipeline=0.0)["envelope"]
    faster_legs = ctl.self_consistent_envelope(
        c, pipeline=0.0075, spare_speed=2 * ctl.SPARE_FOOT_SPEED)["envelope"]
    assert (faster_legs - base) > (no_electronics - base)


# ===================================================================
# M12 — the actuation ramp, and abduction on actuation-time grounds
# ===================================================================

def test_trapezoidal_actuation_costs_little_on_large_moves_and_more_on_small():
    # M11 flagged the constant-velocity assumption as optimistic. Modelling the
    # accelerate/cruise/decelerate ramp shows it was only ~10% out on a full-scale
    # correction -- but ~50% on a short one, where the move never reaches cruise.
    import math
    p = _plant()
    for d, tol in ((0.050, 0.60), (0.010, 1.0)):
        cv = ctl.actuation_time(p, d, accel_limit=math.inf)
        ramp = ctl.actuation_time(p, d, accel_limit=ctl.FOOT_ACCEL_LIMIT)
        assert ramp > cv                       # the ramp always costs something
        assert ramp < (1 + tol) * cv
    # short moves are triangular (never reach cruise), long ones trapezoidal
    v, a = ctl.SPARE_FOOT_SPEED, ctl.FOOT_ACCEL_LIMIT
    short = 0.5 * v * v / a
    assert ctl.actuation_time(p, 1e-9, accel_limit=a) < 1e-3


def test_the_ramp_barely_moves_the_envelope():
    # 52.7 vs 54.4 mm. The M11 caveat was over-cautious: the leg is light enough
    # (tendon drive) that ~107 g of foot acceleration is available, so the move is
    # SPEED limited, not acceleration limited.
    import math
    c = GaitController(params=trot_params())
    cv = ctl.self_consistent_envelope(c, accel_limit=math.inf)["envelope"]
    ramp = ctl.self_consistent_envelope(c)["envelope"]
    assert ramp < cv
    assert ramp > 0.93 * cv                    # < 7 % lost


def test_abduction_would_cut_actuation_time_roughly_in_half():
    # ADR-0015 closed abduction on AUTHORITY grounds. On ACTUATION-TIME grounds the
    # case is different: abduction points along the perpendicular (0.897) instead
    # of obliquely (0.442), so the same correction needs 2.3x less foot travel.
    import dataclasses
    p = _plant()
    abducted = dataclasses.replace(p, projection=0.897)
    t_now = ctl.actuation_time(p, 0.05)
    t_abd = ctl.actuation_time(abducted, 0.05)
    assert t_abd < 0.65 * t_now


def test_the_shipped_design_meets_the_stated_disturbance_cases():
    # NFR15. Capability had never been checked against a REQUIREMENT -- NFR13
    # recorded what the robot achieves, not what it must achieve.
    c = GaitController(params=trot_params())
    p = _plant()
    env = ctl.self_consistent_envelope(c)["envelope"]
    mass = c.body.total_mass
    # a firm 15 N push lasting 0.1 s
    dv = 15.0 * 0.1 / mass
    assert dv / p.omega < env                  # recoverable
    # a 40 mm unexpected step, and a 10 deg lateral slope
    assert 0.040 < env
    assert 0.163 * np.tan(np.radians(10)) < env
    # ...but a 30 N shove is NOT, and that is a stated limit, not a surprise
    assert (30.0 * 0.1 / mass) / p.omega > env


# ===================================================================
# M14 — the spine assist is not free: it costs ground friction
# ===================================================================

def test_spine_authority_is_ALSO_friction_limited():
    # THE M14 FINDING. Bending the spine is INTERNAL motion, and internal motion
    # cannot move the whole-body CoM -- moving it relative to the planted feet
    # needs a horizontal GROUND reaction. control.py had treated spine_assist as a
    # free DCM offset. It is the THIRD constraint on this number: M9 wrongly
    # clamped it by rate, M10 corrected that to ROM, and friction was never checked.
    c = GaitController(params=trot_params())
    rom_only = ctl.StepPlant.from_gait(c)                       # pre-M14 behaviour
    poor = ctl.StepPlant.from_gait(c, floor_mu=0.6)
    good = ctl.StepPlant.from_gait(c, floor_mu=1.5)
    assert poor.spine < rom_only.spine                          # friction binds
    assert good.spine == pytest.approx(rom_only.spine, rel=1e-9)  # ROM binds
    assert rom_only.spine == ctl.StepPlant.from_gait(c, floor_mu=None).spine
    # M15: the YAW couple roughly doubles the cost, so the floor at which ROM
    # takes over is much higher than M14's translation-only estimate suggested.
    # ⚠️ M41 (ADR-0046) INVERTED this at mu 0.8, and the crossover is the result.
    #
    # ADR-0019's claim is that ground friction, not spine ROM, is what limits the
    # sway. It was true at the old mass because the ROM-limited sway was 42.2 mm and
    # mu 0.8 clipped it to 36.6. The measured leg masses moved the whole-body CoM
    # and the ROM-limited sway fell to **37.0 mm** -- at which point mu 0.8 no longer
    # reaches it:
    #
    #   mu 0.4 -> 14.9 mm   FRICTION binds
    #   mu 0.6 -> 26.6 mm   FRICTION binds
    #   mu 0.7 -> 32.5 mm   FRICTION binds
    #   mu 0.8 -> 37.0 mm   **ROM binds**
    #
    # So the mechanism is intact and the crossover moved: friction is the limit
    # below mu ~0.8 and ROM above it. NFR16's floor of 0.70 sits just inside the
    # friction-limited region, which is the useful reading.
    assert ctl.StepPlant.from_gait(c, floor_mu=0.7).spine < rom_only.spine
    assert ctl.StepPlant.from_gait(c, floor_mu=0.8).spine == pytest.approx(
        rom_only.spine, rel=1e-9), "at mu 0.8 the ROM is what binds now"
    assert ctl.StepPlant.from_gait(c, floor_mu=0.4).spine < \
        ctl.StepPlant.from_gait(c, floor_mu=0.7).spine, "and it is monotone in mu"


def test_the_spine_costs_BOTH_translation_and_yaw_friction():
    # M14 costed only the TRANSLATION (accelerating the body sideways). M15 adds
    # the YAW couple: the spine's tip travels ~2x the CoM shift while its base
    # stays put, dumping angular momentum about the vertical into the trunk, which
    # two contacts resist with a friction COUPLE -- and a couple loads each foot
    # with the full force, not half.
    c = GaitController(params=trot_params())
    cost = ctl.spine_friction_cost(c)
    assert cost["mu_translation"] > 0.1
    assert cost["mu_yaw"] > 0.1                       # NOT negligible
    assert cost["mu_total"] == pytest.approx(
        cost["mu_translation"] + cost["mu_yaw"], rel=1e-9)
    # and it scales as 1/stance^2, which is why a slower trot is more robust
    slow = ctl.spine_friction_cost(GaitController(params=trot_params(period=0.8)))
    assert slow["mu_total"] < 0.3 * cost["mu_total"]


def test_NFR15_needs_a_floor_of_about_0_7_at_the_shipped_period():
    # The requirement (a 15 N / 0.1 s push = 48 mm) is NOT floor-independent.
    import dataclasses

    def envelope(mu):
        p = ctl.StepPlant.from_gait(c, floor_mu=mu)
        lo, hi = 0.002, 0.15
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            tau = 0.0075 + ctl.actuation_time(p, mid)
            e = ctl.rejection_envelope(dataclasses.replace(p, latency=tau), use_spine=True)
            if mid <= e:
                lo = mid
            else:
                hi = mid
        return lo

    c = GaitController(params=trot_params())
    assert envelope(0.6) < 0.048          # fails the requirement
    assert envelope(0.8) > 0.048          # meets it at the SHIPPED 0.4 s period
