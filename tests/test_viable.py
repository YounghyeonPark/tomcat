# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The viable set — what any controller could recover from (M28).

⚠️ These tests settle the question the whole M17–M27 arc kept hitting: when a
measurement falls short of a prediction, is the model optimistic or the controller
poor? The viable set has no controller in it, so it answers that directly.

Pure geometry — no MuJoCo, so this runs everywhere.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from tomcat_kin import control, gait, mjcf, viable

DIRECTIONS = [(math.cos(math.radians(a)), math.sin(math.radians(a)))
              for a in range(0, 360, 15)]


@pytest.fixture(scope="module")
def setup():
    c = gait.GaitController(gait.trot_params())
    plant = control.StepPlant.from_gait(c, n=96, latency=0.0075, floor_mu=0.8)
    q = mjcf.stance_pose(c, 0.25)
    reach = (float(plant.reach[0]), float(plant.reach[1]))
    return c, plant, q, reach


def _worst(region):
    return min(viable.reach_in_direction(region, d) for d in DIRECTIONS)


def _best(region):
    return max(viable.reach_in_direction(region, d) for d in DIRECTIONS)


def test_one_step_matches_the_closed_form(setup):
    """The recursion must reproduce `R_1 = (g-1)/g . S` exactly, not approximately."""
    c, plant, q, reach = setup
    g = math.exp(plant.omega * plant.stance)
    origin = viable.equilibrium(c, q)
    s = viable.support_set(c, q, viable.DIAGONALS["A"], reach, origin)
    r1 = viable.viable_set(c, q, plant.omega, plant.stance, reach, steps=1)
    err = np.abs(np.sort(r1, axis=0) - np.sort((g - 1) / g * s, axis=0)).max()
    assert err < 1e-12, f"closed form disagrees by {err:.2e}"


def test_the_horizon_converges(setup):
    """Extra steps must stop adding authority — otherwise the number is horizon-
    limited rather than reach-limited, which is a different and misleading claim.
    `control.py`'s own docstring records making that mistake."""
    c, plant, q, reach = setup
    vals = [_worst(viable.viable_set(c, q, plant.omega, plant.stance, reach, steps=n))
            for n in (6, 12, 40)]
    assert vals[0] == pytest.approx(vals[-1], abs=1e-5)
    assert vals[1] == pytest.approx(vals[-1], abs=1e-5)


def test_the_origin_must_be_the_nominal_CoP_not_the_world_origin(setup):
    """Regression guard on a real mistake.

    The recursion's fixed point is `xi* = u*`, so the origin has to be the nominal
    centre of pressure. Built in absolute coordinates the region merely *touched*
    the origin and reported **zero** authority in half of all directions — a
    plausible-looking catastrophe.
    """
    c, plant, q, reach = setup
    good = viable.viable_set(c, q, plant.omega, plant.stance, reach, steps=8)
    assert viable.contains(good, (0.0, 0.0))
    assert _worst(good) > 0.02

    origin = viable.equilibrium(c, q)
    assert origin[0] > 0.05, "the nominal CoM is not at the world origin"


def test_the_1D_reduction_lands_on_the_worst_direction(setup):
    """⚠️ The vindication of `control.py`.

    `StepPlant` collapses balance to one axis, which ADR-0031 criticised. Compared
    against the exact 2-D viable set, its feet-only envelope (30.3 mm) sits within
    **2 %** of the true worst-direction limit (29.8 mm). The reduction is not
    optimistic — it happens to pick out the binding direction.
    """
    c, plant, q, reach = setup
    exact = _worst(viable.viable_set(c, q, plant.omega, plant.stance, reach, steps=20))
    quoted = control.rejection_envelope(plant)
    assert exact == pytest.approx(0.0298, abs=5e-4)
    assert abs(quoted - exact) / exact < 0.03


def test_the_foot_placement_controller_is_near_optimal(setup):
    """⚠️ And the vindication of the MuJoCo harness.

    M23 measured 28.9 mm and read it as "84 % of the prediction", implying a poor
    controller. Against the true limit it is **97 %**. For the feet-only problem the
    harness is close to the best any controller could do, and M27's blanket
    indictment of "the architecture" was too broad — it holds for the spine, not the
    feet.
    """
    c, plant, q, reach = setup
    exact = _worst(viable.viable_set(c, q, plant.omega, plant.stance, reach, steps=20))
    measured = 0.0289                       # ADR-0028, settled cycle, worst of 18
    assert measured / exact > 0.95


def test_the_spine_authority_is_sufficient_for_NFR15(setup):
    """⚠️ THE result. NFR15 is **achievable** — the controller is what is missing.

    Crediting the spine's 36.6 mm as a one-shot lateral CoM shift, the exact viable
    worst case is **62.7 mm** against NFR15's 48 mm. So no controller has to be
    optimistic for the requirement to be met: the authority is there, and M24–M27's
    failure to spend it is a control problem with a *proven achievable target*.

    Note also that `control.py`'s 1-D with-spine figure (52.7 mm) is **conservative**
    against the exact 62.7 mm — the opposite of the optimism M23 suspected.
    """
    c, plant, q, reach = setup
    with_spine = viable.viable_set(c, q, plant.omega, plant.stance, reach,
                                   steps=20, spine=plant.spine)
    worst = _worst(with_spine)
    assert worst == pytest.approx(0.0627, abs=1e-3)
    assert worst > 0.048, "NFR15 would be unachievable — recheck before publishing"

    quoted = control.self_consistent_envelope(c)["envelope"]
    assert quoted < worst, "the 1-D figure should be conservative, not optimistic"


def test_the_viable_set_is_direction_dependent(setup):
    """The 2-D structure M17 found, now exact rather than sampled."""
    c, plant, q, reach = setup
    region = viable.viable_set(c, q, plant.omega, plant.stance, reach, steps=20)
    assert _best(region) / _worst(region) > 2.0


def test_a_lateral_credit_helps_in_more_than_the_lateral_direction(setup):
    """⚠️ This one surprised me, and it qualifies ADR-0031.

    The spine is a Minkowski *segment* along the body's lateral axis, so along that
    axis it adds exactly its own length — 36.6 mm, to the millimetre. But the viable
    set is **slanted**, because the support parallelogram is a diagonal. Sliding a
    slanted boundary sideways moves where the fore-aft ray exits it, so the gain in
    **x is larger still (63 mm)**.

    ADR-0031 said the spine is "authority in the wrong axis". Geometrically that is
    too strong: lateral authority converts into recovery capability in directions
    that are not lateral, precisely because the trot's support is diagonal. What
    ADR-0031 established stands — the spine cannot act along the support line — but
    "only helps laterally" does not follow from it.
    """
    c, plant, q, reach = setup
    base = viable.viable_set(c, q, plant.omega, plant.stance, reach, steps=20)
    grown = viable.viable_set(c, q, plant.omega, plant.stance, reach, steps=20,
                              spine=plant.spine)

    lateral = (viable.reach_in_direction(grown, (0, 1))
               - viable.reach_in_direction(base, (0, 1)))
    fore_aft = (viable.reach_in_direction(grown, (1, 0))
                - viable.reach_in_direction(base, (1, 0)))

    # Along its own axis the segment adds exactly its length — the invariant.
    assert lateral == pytest.approx(plant.spine, rel=1e-6)
    # And the slant carries more of it into fore-aft than into lateral.
    assert fore_aft > lateral


@pytest.mark.parametrize("period,speed_cm_s", [(0.40, 50), (0.30, 67)])
def test_NFR15_is_met_from_floor_mu_0_6_at_both_trot_speeds(period, speed_cm_s):
    """⚠️ M29. Re-derives OPEN_RISKS **R2**, one of the two critical risks.

    R2's published table (40.2 / 48.1 / 53.9 mm at μ 0.5 / 0.7 / 0.9) **cannot be
    reproduced from the current code** — it predates M20's sway correction, and
    `self_consistent_envelope` takes no `floor_mu` at all, so it cannot produce a
    μ-dependent column. A stale table in a CRITICAL risk section.

    On the exact viable set, NFR15's 48 mm is met from **μ ≥ 0.6 at both speeds** —
    where R2 implied μ 0.70 was needed with no margin at all.
    """
    c = gait.GaitController(gait.trot_params(period=period))
    q = mjcf.stance_pose(c, 0.25)
    assert c.params.body_speed == pytest.approx(speed_cm_s / 100.0, abs=0.02)

    def envelope(mu):
        plant = control.StepPlant.from_gait(c, n=96, latency=0.0075, floor_mu=mu)
        reach = (float(plant.reach[0]), float(plant.reach[1]))
        region = viable.viable_set(c, q, plant.omega, plant.stance, reach,
                                   steps=20, spine=plant.spine)
        return _worst(region)

    assert envelope(0.5) < 0.048, "mu 0.5 should still fail — do not overclaim"
    assert envelope(0.6) >= 0.048, "mu 0.6 must meet NFR15"
    assert envelope(0.7) >= 0.048
    # Monotone in friction, or the spine clamp is wired backwards.
    assert envelope(0.4) < envelope(0.6) < envelope(0.8)


def test_the_ADR_0020_slowdown_is_not_required_by_NFR15():
    """⚠️ M29. ADR-0020 slowed the shipped trot **67 → 50 cm/s** because the spine's
    friction demand exceeded a realistic floor. On the exact viable set the faster
    gait **also meets NFR15** at μ ≥ 0.6 (48.1 mm), and it carries a *better* sensing
    margin besides — per-step growth is 3.21 at 0.30 s against 4.73 at 0.40 s.

    This does not reinstate 67 cm/s by itself: ADR-0020's friction accounting is
    still un-cross-checked (ADR-0025). It removes NFR15 as the reason.
    """
    fast = gait.GaitController(gait.trot_params(period=0.30))
    q = mjcf.stance_pose(fast, 0.25)
    plant = control.StepPlant.from_gait(fast, n=96, latency=0.0075, floor_mu=0.6)
    reach = (float(plant.reach[0]), float(plant.reach[1]))
    region = viable.viable_set(fast, q, plant.omega, plant.stance, reach,
                               steps=20, spine=plant.spine)
    assert _worst(region) >= 0.048

    slow = gait.GaitController(gait.trot_params(period=0.40))
    slow_plant = control.StepPlant.from_gait(slow, n=96, latency=0.0075, floor_mu=0.6)
    assert math.exp(plant.omega * plant.stance) < math.exp(
        slow_plant.omega * slow_plant.stance), "the fast gait should diverge less per step"
