# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Tests for M16 power & runtime -- closing NFR6, blank since M1.

The headline is not the runtime. It is that a tendon-driven robot pays to STAND,
because a cable can only pull and posture is held with motor current.
"""

import numpy as np
import pytest

from tomcat_kin import GaitController, power
from tomcat_kin.gait import trot_params


def _trot():
    return GaitController(params=trot_params())



def test_standing_costs_about_what_moving_costs_for_zero_work():
    """THE M16 FINDING, and the quantified case for the ADR-0003 power-off brake.

    ✅ M107 nearly broke it with an artefact: `gait_power` bypassed the
    capstan while `standing_power` did not, and the ratio read 1.49. Fixed, it
    was 0.972 (M108).

    ⚠️ **M111 takes it to 1.04, and that is NOT standing becoming dearer than
    walking.** The two halves were never computed on the same basis:
    `standing_power` takes `torque_budget`'s WORST pose over the reachable
    workspace -- with the pretension bias -- and charges it to every joint of
    all four legs at once, while `gait_power` averages the trajectory actually
    walked, with no bias. The 36/34/22 arms cut the walked copper harder than
    the worst-pose bound, and the pretension floor does not shrink with the
    arm at all. So the ratio crossing 1 is the two methods diverging.

    `[owed]` -- price standing at the real stance pose, then this ratio means
    what M16 meant by it.
    """
    r = power.runtime(_trot())
    assert r["standing_fraction_of_trot"] == pytest.approx(1.04, abs=0.05)
    assert 0.5 < r["standing_fraction_of_trot"] < 1.2
    assert r["stand_w"] > 50.0                      # electronics included


def test_the_power_off_brake_multiplies_standing_endurance():
    # ADR-0003 specified the brake on qualitative grounds ("essential"). This is
    # what it is worth: standing hold current goes to zero and only the
    # electronics allowance remains.
    r = power.runtime(_trot())
    assert r["stand_minutes_braked"] > 4 * r["stand_minutes"]



def test_copper_loss_dominates_so_the_drive_is_inefficient():
    """Copper loss against useful work, and why it keeps moving.

    | milestone | efficiency | copper / mechanical |
    |---|---|---|
    | ADR-0021 | 39 % | 1.6x |
    | M40, three-phase copper formula | 29.6 % | 2.4x |
    | M107/M108, capstan friction counted | 16.5 % | 5.07x |
    | **M111, 36/34/22 arms** | **30.7 %** | **2.25x** |

    ✅ The arms are a reduction ratio, and copper goes as the square of the
    current it sets: raising them halved the copper while the mechanical term
    -- `tau * qd` at the joint -- did not move. The friction M107 counted is
    still all there; the arms simply ask the cable for less tension to carry it.

    ⚠️ M107's first wiring booked friction as useful work; the tell was
    efficiency falling as copper fell. The capstan sits on the current path only.
    """
    g = power.gait_power(_trot())
    assert g["copper_w"] > 2.0 * g["mechanical_w"]
    assert 0.27 < g["efficiency"] < 0.34             # ~30.7 %



def test_currents_are_back_inside_the_driver_rating():
    """✅ **M111: back inside, by the arm ratio.**

    M107 found the trot drawing 4.34 A peak against the GIM3505-9 driver's
    4.19 A and recorded it as SPEC. At the 36/34/22 arms the same joint torque
    needs a cable tension smaller by the arm ratio, so **2.79 A peak** and
    0.94 A RMS -- inside both the 4.19 A peak and the 1.60 A continuous.

    ⚠️ At Kt 0.44. The vendor's 0.35 needs 1.26x the current; see
    `test_motor_spec.py`'s thermal test, where that branch also fits now.
    """
    g = power.gait_power(_trot())
    assert g["peak_current_a"] == pytest.approx(2.79, abs=0.15)
    assert g["peak_current_a"] < 4.19               # the part's peak rating
    assert g["rms_current_a"] < 1.60                # its RATED (continuous) current



def test_NFR6_is_MET_once_the_arms_are_right():
    """✅ **M111: 21.7 min / 650 m, inside NFR6's re-stated 14-20 min on the
    optimistic Kt and above it.**

    The history of this number is the history of the corrections: ~30
    published -> 25.2 (mass + spool) -> 19.6 (three-phase copper) -> 18.85
    -> 19.39 (measured leg inertia) -> 18.81 (M93) -> 12.71 (capstan friction,
    M107/M108) -> **21.66** (36/34/22 arms, M111).

    The arms are a reduction: the same joint torque at 1/r the cable tension,
    and copper goes as its square. That recovers more than friction cost.
    """
    r = power.runtime(_trot())
    assert r["trot_minutes"] == pytest.approx(21.66, abs=0.4)
    assert r["trot_minutes"] > 14.0, "back under NFR6's floor"
    assert r["trot_range_m"] == pytest.approx(650.0, abs=15.0)
    assert r["trot_range_m"] > 420.0
    assert r["battery_wh"] == pytest.approx(
        power.BATTERY_KG * power.BATTERY_WH_PER_KG * power.BATTERY_USABLE)


def test_copper_loss_scales_with_the_squared_transmission_ratio():
    # P_cu ~ I^2 ~ tau_motor^2 ~ (r_spool / r_joint)^2, so the JOINT MOMENT ARM is
    # a runtime lever as well as a cable-tension one -- a second role
    # LEG_TENDON_SPEC never costed.
    import dataclasses
    from tomcat_kin import params as params_mod

    c = _trot()
    base = power.gait_power(c)["copper_w"]
    orig = params_mod.DEFAULT_TENDON
    bigger = dataclasses.replace(
        orig, joint_moment_arm=tuple(x * 2.0 for x in orig.joint_moment_arm))
    params_mod.DEFAULT_TENDON = bigger
    try:
        doubled = power.gait_power(c)["copper_w"]
    finally:
        params_mod.DEFAULT_TENDON = orig
    assert doubled == pytest.approx(base / 4.0, rel=0.05)   # inverse-square
