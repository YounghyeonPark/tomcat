# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Tests for the M6 whole-body dynamics: contact forces, friction cone, ZMP.

These encode the milestone's central finding: the quasi-static checks (M4/M5)
were answering the wrong question. A positive support-polygon margin says the
robot would not topple if it were *standing still*; it says nothing about whether
the contacts can produce the forces the *motion* requires.
"""

import numpy as np
import pytest

from tomcat_kin import GaitController, GaitParams
from tomcat_kin import dynamics as dyn


# ------------------------------------------------------------------- soundness
def test_newton_euler_balance_is_satisfied_exactly():
    # If the residual is not ~0 the force solve is meaningless.
    c = GaitController()
    cyc = dyn.cycle(c, 60)
    for i in range(60):
        assert dyn.contact_forces(c, i / 60, 60, cyc=cyc).residual < 1e-9


def test_normal_forces_carry_the_body_weight():
    # With near-zero vertical acceleration the normals must sum to m*g.
    c = GaitController()
    cyc = dyn.cycle(c, 120)
    w = c.body.total_mass * dyn.GRAVITY
    for i in range(120):
        s = dyn.contact_forces(c, i / 120, 120, cyc=cyc)
        total_n = sum(f[2] for f in s.forces.values())
        assert total_n == pytest.approx(w + c.body.total_mass * s.accel[2], abs=1e-6)


def test_zmp_equals_the_static_projection_when_nothing_accelerates():
    # THE SANITY CONDITION for the whole module: with acceleration removed, the
    # dynamic check must reduce EXACTLY to the M5 static one. Done by zeroing the
    # acceleration in a CycleData rather than hunting for quiet phases -- the legs
    # are always swinging, so no phase is ever exactly still.
    import dataclasses
    c = GaitController()
    cyc = dyn.cycle(c, 240)
    still = dataclasses.replace(cyc, accel=np.zeros_like(cyc.accel))
    for i in range(0, 240, 7):
        z = dyn.zero_moment_point(c, i / 240, 240, cyc=still)
        assert z.excursion < 1e-12
        assert z.margin == pytest.approx(c.support_polygon(i / 240).margin, abs=1e-9)


def test_the_quiet_phases_are_nearly_static_anyway():
    # And in the shipped gait the 3-foot hold phases ARE nearly still, so the
    # static and dynamic answers agree there to well under a millimetre.
    c = GaitController()
    cyc = dyn.cycle(c, 240)
    quiet = [i for i in range(240) if np.linalg.norm(cyc.accel[i][:2]) < 0.05]
    assert len(quiet) > 100          # most of the cycle is a quiet hold
    for i in quiet:
        assert dyn.zero_moment_point(c, i / 240, 240, cyc=cyc).excursion < 1e-3


# --------------------------------------------- the C1 defect dynamics exposed
def test_sway_acceleration_is_finite_and_grid_convergent():
    # REGRESSION GUARD for a real defect M6 found in the M5 sway law. The ramp was
    # LINEAR in position, so velocity STEPPED at each end of the crossover -- an
    # impulse in acceleration, i.e. infinite force. On a discrete grid that shows
    # up as a peak that keeps growing as the grid refines. The raised-cosine ramp
    # starts and ends at zero velocity, so the peak CONVERGES.
    c = GaitController()
    peaks = [np.abs(dyn.com_acceleration(c, n)[:, 1]).max() for n in (120, 240, 480, 960)]
    assert max(peaks) / min(peaks) < 1.05          # converged, not diverging
    assert all(np.isfinite(p) for p in peaks)


def test_hand_calc_is_a_conservative_estimate_of_the_resolved_peak():
    # crossover_accel() is a lumped bang-bang-style estimate. It should sit ABOVE
    # the resolved peak (so it errs safe) but within the same ballpark.
    c = GaitController()
    cmp = dyn.compare_with_hand_calc(c, 240)
    assert cmp["hand_accel"] >= cmp["dynamic_peak_lateral_accel"]
    assert cmp["hand_accel"] < 1.5 * cmp["dynamic_peak_lateral_accel"]


# ------------------------------------------------------- the milestone finding
def test_default_gait_is_dynamically_feasible():
    c = GaitController()
    r = dyn.sweep(c, 120, mu=0.8)
    assert r["zmp_stable"]
    assert r["unilateral_ok"]                # no foot has to pull
    assert r["feasible"]
    assert r["zmp_margin_min"] > 0.004       # > 4 mm


def test_the_M5_gait_is_NOT_dynamically_feasible():
    # THE REGRESSION GUARD FOR THE WHOLE MILESTONE. M5 shipped a 1.4 s walk with a
    # +10.1 mm STATIC margin and called it stable. Resolving the dynamics shows the
    # ZMP leaves the support polygon by >100 mm during the sway crossover.
    c = GaitController(params=GaitParams(period=1.4, lateral_amplitude=np.radians(12.5)))
    assert all(p.is_stable for p in c.support_polygon_sweep(96))     # static: fine
    r = dyn.sweep(c, 120)
    assert not r["zmp_stable"]                                       # dynamic: not
    assert r["zmp_margin_min"] < -0.05
    assert not r["unilateral_ok"]                                    # feet would pull


def test_tipping_binds_before_slipping():
    # M5 concluded FRICTION set the walk speed (mu >= 0.70). It does not. Resolving
    # the per-foot forces shows the body-level friction demand stays modest even at
    # M5's speed, while the ZMP has long since left the polygon. Swept over periods
    # 0.6-6.0 s, slipping NEVER binds; tipping only clears at 3.8 s.
    fast = GaitController(params=GaitParams(period=1.4, lateral_amplitude=np.radians(11)))
    r = dyn.sweep(fast, 120)
    assert r["aggregate_mu"] < 0.8          # would NOT have slipped
    assert not r["zmp_stable"]              # but tips comfortably


def test_zmp_excursion_follows_the_height_over_gravity_law():
    # zmp = com - (h/(az+g)) * a_xy. Check the shipped gait against that directly.
    c = GaitController()
    cyc = dyn.cycle(c, 240)
    i = int(np.argmax(np.abs(cyc.accel[:, 1])))
    z = dyn.zero_moment_point(c, i / 240, 240, cyc=cyc)
    h = cyc.com[i][2] - cyc.ground_z
    expect = h / (cyc.accel[i][2] + dyn.GRAVITY) * np.linalg.norm(cyc.accel[i][:2])
    assert z.excursion == pytest.approx(expect, rel=1e-6)


def test_slowing_the_gait_recovers_dynamic_stability_quadratically():
    # a ~ 1/T^2, so the ZMP excursion should fall by ~4x when the period doubles.
    slow = GaitController(params=GaitParams(period=8.0))
    fast = GaitController(params=GaitParams(period=4.0))
    es = dyn.sweep(slow, 120)["zmp_excursion_max"]
    ef = dyn.sweep(fast, 120)["zmp_excursion_max"]
    assert ef / es == pytest.approx(4.0, rel=0.15)


# --------------------------------------------------------------- the caveats
def test_angular_momentum_assumption_is_quantified_not_just_declared():
    # dH/dt = 0 is an assumption; this reports what ignoring the swing leg's spin
    # is worth in ZMP terms, so the reader can judge it.
    c = GaitController()
    a = dyn.angular_momentum_caveat(c, 120)
    assert a["swing_leg_zmp_shift_max"] >= 0.0
    # It must be SMALL relative to the margins we are claiming, or the whole
    # ZMP result is inside the noise of its own assumption.
    assert a["swing_leg_zmp_shift_max"] < 0.004      # < 4 mm at the 5 s crawl


def test_lightly_loaded_feet_do_not_dominate_the_friction_verdict():
    # mu = |Ft|/Fn explodes as Fn -> 0. The body-level aggregate is the number
    # that decides whether the ROBOT slides.
    c = GaitController()
    cyc = dyn.cycle(c, 120)
    worst = max((dyn.contact_forces(c, i / 120, 120, cyc=cyc) for i in range(120)),
                key=lambda s: s.peak_mu)
    assert worst.aggregate_mu <= worst.peak_mu
    assert worst.aggregate_mu < 0.3


# ===================================================================
# M13 — closing the dH/dt = 0 caveat with a number
# ===================================================================

def test_angular_momentum_caveat_evaluates_a_TROT_at_all():
    # REGRESSION GUARD for a silent-zero bug. The estimator required EXACTLY ONE
    # swing leg. A trot moves diagonal pairs, so two legs are always in flight --
    # every phase was skipped and the function returned a reassuring 0.00 mm
    # having evaluated nothing. A zero that means "not measured" is worse than no
    # number at all.
    from tomcat_kin.gait import trot_params
    trot = GaitController(params=trot_params())
    a = dyn.angular_momentum_caveat(trot, 96)
    assert a["swing_leg_zmp_shift_max"] > 0.005          # ~42 mm, definitely not 0
    crawl = dyn.angular_momentum_caveat(GaitController(), 96)
    assert crawl["swing_leg_zmp_shift_max"] < 0.002      # ~1 mm, unchanged
    assert a["swing_leg_zmp_shift_max"] > 20 * crawl["swing_leg_zmp_shift_max"]


def test_dH_dt_is_large_at_trot_but_mostly_lands_on_PITCH():
    # THE M13 RESULT. dH/dt = 0 is badly violated at trot speed in MAGNITUDE, but
    # two point contacts can resist every moment except the one about the line
    # joining them -- and the swing legs' reaction is mostly pitch, which they can.
    from tomcat_kin.gait import trot_params
    c = GaitController(params=trot_params())
    cyc = dyn.cycle(c, 96)
    grav, swing = [], []
    for i in range(96):
        grav.append(dyn.line_balance(c, i / 96, 96, cyc=cyc).unbalanced_moment)
        swing.append(dyn.swing_leg_moment(c, i / 96, 96, cyc=cyc)["available"])
    ratio = max(swing) / max(grav)
    assert 0.10 < ratio < 0.40          # ~21 %: a real correction, not a reversal


def test_link_SPIN_inertia_is_negligible_because_the_legs_are_light():
    # A third P1 dividend. Slender-rod inertia goes as m*L^2/12, and tendon drive
    # keeps the legs at 95 g and short -- so the spin term is ~3 % of gravity while
    # the orbital term is ~22 %.
    from tomcat_kin.gait import trot_params
    c = GaitController(params=trot_params())
    cyc = dyn.cycle(c, 96)
    spins, orbs = [], []
    for i in range(96):
        r = dyn.swing_leg_moment(c, i / 96, 96, cyc=cyc)
        spins.append(r["spin"])
        orbs.append(r["orbital"])
    assert max(spins) < 0.25 * max(orbs)
    # and the point-mass-only path still works, for reproducing pre-M13 figures
    off = dyn.swing_leg_moment(c, 0.25, 96, cyc=cyc, include_spin=False)
    assert off["spin"] == 0.0


def test_M7_bounded_roll_SURVIVES_the_swing_leg_reaction():
    # M7 computed the trot's bounded +/-0.4 deg roll from GRAVITY ALONE. Adding the
    # swing-leg reaction must not turn it into a divergence -- if it did, the whole
    # trot result would fall over with it.
    from tomcat_kin.gait import trot_params
    c = GaitController(params=trot_params())
    n = 96
    cyc = dyn.cycle(c, n)
    h = float(np.mean(cyc.com[:, 2] - cyc.ground_z))
    I = c.body.total_mass * h * h
    dt = c.params.period / n
    sg = []
    for i in range(n):
        b = dyn.line_balance(c, i / n, n, cyc=cyc)
        r = dyn.swing_leg_moment(c, i / n, n, cyc=cyc)
        sg.append(np.sign(b.offset) * (b.unbalanced_moment - r["available"]))
    sg = np.array(sg)
    drift = float(np.sum(sg / I) * dt)
    om = np.cumsum(sg / I) * dt
    om -= om.mean()
    th = np.cumsum(om) * dt
    assert abs(drift) < 0.10                                  # still bounded
    assert np.degrees(th.max() - th.min()) < 1.0              # still under a degree
