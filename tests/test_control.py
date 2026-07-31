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
    assert all(e > 0.05 for e in envs)


def test_the_binding_direction_is_REARWARD():
    # The leg reaches +153 mm forward but only -74 mm back, so a disturbance that
    # needs a rearward foothold is the one that limits the robot.
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
    # exactly the per-step growth (3.2x). A 5 mm estimation bias is a 16 mm
    # permanent drift.
    p = _plant()
    for bias in (0.002, 0.005, 0.010):
        traj = ctl.simulate(p, 60, xi0=0.03, beta=0.0, estimation_error=bias)
        assert traj[-1] == pytest.approx(-p.growth * bias, rel=0.02)
