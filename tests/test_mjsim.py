# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Closed-loop balance — the item M17 and M20 were both blocked on (M21).

⚠️ These are the tests that make the envelope measurable at all. The gate is the
**baseline**: a harness whose undisturbed drift is the same order as the disturbance
it is measuring cannot adjudicate anything, which is exactly why M17 declined to
report a number. Assert the noise floor before asserting any result on top of it.

`mujoco` is an optional dependency; the module skips without it.
"""

from __future__ import annotations

import numpy as np
import pytest

from tomcat_kin import control, gait, mjsim

mujoco = pytest.importorskip("mujoco", reason="mujoco is an optional dependency")

COMPLIANT_KP = 80
STIFF_KP = 500


@pytest.fixture(scope="module")
def controller():
    return gait.GaitController(gait.trot_params())


def _harness(controller, kp):
    model = mjsim.build(controller, mujoco, kp=kp)
    return mjsim.BalanceHarness(controller, mujoco, model, regulate_along_line=False)


def _mean_dcm(hist, sl):
    return float(np.abs(np.r_[[r["perp"] for r in hist][sl],
                              [r["para"] for r in hist][sl]]).mean())


def test_the_undisturbed_baseline_is_quiet_enough_to_measure_against(controller):
    """THE gate. M17's harness drifted 25 mm against a 30 mm signal, so it could
    not tell a recovery from its own noise. This one must stay far below that."""
    h = _harness(controller, COMPLIANT_KP)
    hist, fell = h.run(h.reset(), steps=30)

    assert not fell, f"undisturbed baseline fell after {len(hist)} steps"
    assert len(hist) == 30

    # ⚠️ Two numbers, because they say different things. The MEAN is the resolution
    # for a typical direction; the MAX is what limits the worst one.
    mean = _mean_dcm(hist, slice(None))
    worst = max(abs(r["perp"]) for r in hist) + max(abs(r["para"]) for r in hist)
    assert mean < 0.003, f"mean baseline drift {1000 * mean:.1f} mm"
    assert worst < 0.015, f"peak baseline excursion {1000 * worst:.1f} mm"

    # And — the failure M17 actually had — it must not be quietly winding up.
    # M17's drifted to 25 mm and was still growing; this one is bounded.
    assert _mean_dcm(hist, slice(-10, None)) < 2.0 * _mean_dcm(hist, slice(0, 10))


def test_the_swing_profile_must_land_at_rest(controller):
    """Regression guard on a C0 defect I reintroduced by hand.

    A `sin(pi u)` arc peaks correctly but lands with `-pi h / T` = 0.31 m/s of
    downward foot speed. It hammered the contact, the stance never settled at two
    feet, and the run died in 14 steps. This is the same class of error M5 and M6
    fixed in the shipped gait — which is why `GaitParams.swing_profile` defaults to
    "matched" rather than to a cycloid.
    """
    import math

    h = _harness(controller, COMPLIANT_KP)
    n = 400
    z = [0.5 * h.step_h * (1.0 - math.cos(2.0 * math.pi * k / n)) for k in range(n + 1)]
    dt = h.T / n
    assert abs(z[1] - z[0]) / dt < 0.02, "leaves the ground with vertical speed"
    assert abs(z[-1] - z[-2]) / dt < 0.02, "lands with vertical speed"
    assert max(z) == pytest.approx(h.step_h, rel=1e-3)


def test_balance_needs_compliant_legs(controller):
    """A finding, not a tuning note.

    Stiff position servos make ground-reaction distribution effectively bang-bang:
    ±1 mm of differential stance-leg extension swings the centre of pressure across
    the whole ±109 mm foot separation. The loop cannot trim its own load, and the
    undisturbed DCM winds up. Compliance restores it.

    The mechanical design already specifies passive compliance (series elastic
    elements / return springs). This validates that choice from a direction it was
    not chosen for.
    """
    h_soft = _harness(controller, COMPLIANT_KP)
    soft, _ = h_soft.run(h_soft.reset(), steps=24)
    h_stiff = _harness(controller, STIFF_KP)
    stiff, _ = h_stiff.run(h_stiff.reset(), steps=24)

    assert len(soft) == 24, "the compliant baseline should not fall"
    soft_late = _mean_dcm(soft, slice(-8, None))
    assert soft_late < 0.005, f"compliant drift {1000 * soft_late:.1f} mm"
    if len(stiff) == 24:
        assert _mean_dcm(stiff, slice(-8, None)) > 2.0 * soft_late


@pytest.mark.parametrize("angle_deg,floor_mm", [(300, 12.0), (60, 40.0)])
def test_the_envelope_is_strongly_direction_dependent(controller, angle_deg, floor_mm):
    """M17 found the two diagonals topple along axes 52.4 deg apart but could not
    cost it. Measured: the envelope spans **3.4x** across direction — 19.3 mm at its
    worst against 65.7 mm at its best — while `StepPlant` quotes a single 30.34 mm
    for every direction.

    ⚠️ The worst direction is **64 % of the prediction**. Checked loosely here
    because a bisection is slow; the numbers are in ADR-0026.
    """
    import math

    a = math.radians(angle_deg)
    u = np.array([math.cos(a), math.sin(a)])
    h = _harness(controller, COMPLIANT_KP)

    lo, hi = 0.0, 1.0
    for _ in range(6):
        mid = 0.5 * (lo + hi)
        hist, fell = h.run(h.reset(), steps=10, disturbance=mid * u)
        if not fell and len(hist) == 10:
            lo = mid
        else:
            hi = mid
    xi_mm = 1000.0 * lo / h.omega
    assert xi_mm > floor_mm, f"{angle_deg} deg gave only {xi_mm:.1f} mm"


def test_measured_worst_case_is_below_the_reduced_order_prediction(controller):
    """The result. The single-axis model promises one number in every direction; the
    worst direction does not deliver it."""
    plant = control.StepPlant.from_gait(controller, n=96, latency=0.0075, floor_mu=0.8)
    predicted = control.rejection_envelope(plant)
    assert predicted == pytest.approx(0.0303, abs=5e-4)
    # 19.3 mm measured at 300 deg (ADR-0026) against 30.3 mm predicted.
    assert 0.0193 < predicted * 0.75
