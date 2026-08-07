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


def test_standing_costs_most_of_what_moving_costs_for_zero_work():
    # THE M16 FINDING, and the quantified case for the ADR-0003 power-off brake.
    r = power.runtime(_trot())
    assert 0.5 < r["standing_fraction_of_trot"] < 1.0
    assert r["stand_w"] > 50.0                      # ~67 W including electronics


def test_the_power_off_brake_multiplies_standing_endurance():
    # ADR-0003 specified the brake on qualitative grounds ("essential"). This is
    # what it is worth: standing hold current goes to zero and only the
    # electronics allowance remains.
    r = power.runtime(_trot())
    assert r["stand_minutes_braked"] > 4 * r["stand_minutes"]


def test_copper_loss_dominates_so_the_drive_is_inefficient():
    g = power.gait_power(_trot())
    assert g["copper_w"] > g["mechanical_w"]        # ~42 W vs ~27 W
    assert 0.30 < g["efficiency"] < 0.50            # ~39 %


def test_currents_stay_inside_the_driver_rating():
    g = power.gait_power(_trot())
    assert g["peak_current_a"] < 4.19               # the part's peak rating
    assert g["rms_current_a"] < 1.60                # its RATED (continuous) current


def test_NFR6_has_an_answer_at_last():
    r = power.runtime(_trot())
    assert 20.0 < r["trot_minutes"] < 45.0          # ~30 min
    assert 600.0 < r["trot_range_m"] < 1400.0       # ~900 m
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
