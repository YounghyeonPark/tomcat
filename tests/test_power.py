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
    """THE M16 FINDING, and the quantified case for the ADR-0003 power-off brake.

    ✅ **M107: it survives friction, and it nearly did not survive my reading
    of it.** With M106's capstan live the fraction first measured **1.49** --
    standing costing MORE than trotting -- and that is a result worth having if
    it is true. It was not. `gait_power` built the motor torque straight from
    `tau / arm * spool` and never went through `TendonMap`, while
    `standing_power` goes through `torque_budget.evaluate` and does. So the
    trot was frictionless and the stand was not, and the RATIO between them --
    which is the whole finding -- was an artefact of that.

    Applied to both, the fraction is **0.972**: friction costs the trot 71 %
    and the stand 64 %, so the finding's shape is untouched and only its margin
    narrows. A ratio between two numbers is only as good as the worse-computed
    one.

    ⚠️ **M108 corrected this from 0.924.** The first fix multiplied the
    motor torque by the capstan once, before splitting it, so `p_mech` carried
    the factor too and friction was booked as useful work. See
    `test_copper_loss_dominates_so_the_drive_is_inefficient`.
    """
    r = power.runtime(_trot())
    assert 0.5 < r["standing_fraction_of_trot"] < 1.0
    assert r["standing_fraction_of_trot"] == pytest.approx(0.972, abs=0.05)
    assert r["stand_w"] > 50.0                      # electronics included


def test_the_power_off_brake_multiplies_standing_endurance():
    # ADR-0003 specified the brake on qualitative grounds ("essential"). This is
    # what it is worth: standing hold current goes to zero and only the
    # electronics allowance remains.
    r = power.runtime(_trot())
    assert r["stand_minutes_braked"] > 4 * r["stand_minutes"]


def test_copper_loss_dominates_so_the_drive_is_inefficient():
    """⚠️ M40 (ADR-0045) moved this. Copper loss was computed as `I^2 R_pp` where
    balanced three-phase is `3 I^2 R_ph` = 1.5x that, so the drive is worse than
    published: copper **63 W against 27 W** of useful work, efficiency **29.6 %**
    rather than the 39 % ADR-0021 quoted.

    Copper is now 2.4x the mechanical work, not 1.6x — which sharpens ADR-0021's
    own point that this is a property of the transmission, not of the gait.

    ⚠️ **M107 sharpens it again: 29.6 % -> 16.5 %.** Capstan friction is a
    tension multiplier, so it lands entirely on the copper and not at all on
    the useful work -- copper goes up 71 % and the mechanical term does not
    move. Every earlier efficiency figure in this project was computed with the
    wraps solved and unread.

    ⚠️ **M107 published 20.6 % and that was still too kind.** The first fix
    multiplied `mot_tau` by the capstan once, before splitting it into the
    current path and the work path, so `p_mech` carried the factor as well --
    friction booked as useful output. The tell was efficiency FALLING when
    copper fell, on the hip-spool study: copper goes as the square of the
    factor and the mechanical term as its first power, so inflating both moves
    the ratio the wrong way. A joint's useful output is `tau * qd` whatever the
    routing costs to deliver it. Copper is now **5.07x** the mechanical work.
    """
    g = power.gait_power(_trot())
    assert g["copper_w"] > 4.0 * g["mechanical_w"]
    assert 0.14 < g["efficiency"] < 0.19             # ~16.5 %


def test_the_DRIVER_CURRENT_joins_the_motor_SPEC():
    """⚠️ **M107: the peak no longer fits the proxy's driver, and that is a
    specification rather than a failure.**

    With M106's capstan live the trot draws **4.34 A peak** against the
    GIM3505-9 driver's 4.19 A rating. RMS is 1.38 A, still inside the 1.60
    continuous. The mechanism is the primary design and the motor -- driver
    included -- is a placeholder, so the current joins the torque as something
    to source or design to.

    ⚠️ `[owed]` -- 4.34 A is at a Kt of 0.44. On the vendor's own 0.35 the
    same torque needs 1.26x the current, and nothing here has chosen between
    them.
    """
    g = power.gait_power(_trot())
    assert g["peak_current_a"] == pytest.approx(4.34, abs=0.15)
    assert g["peak_current_a"] > 4.19, (
        "the peak is back inside the proxy's driver -- has friction gone off?"
    )
    assert g["rms_current_a"] < 1.60                # its RATED (continuous) current


def test_NFR6_has_an_answer_and_it_is_now_BELOW_ITS_OWN_RANGE():
    """⚠️ **M107: 18.81 -> 12.13 min, under the 14-20 NFR6 was re-stated to.**

    The history of this number is the history of the corrections: ~30
    published -> 25.2 (mass + spool) -> 19.6 (the three-phase copper formula)
    -> 18.85 -> 19.39 (the leg's measured inertia) -> 18.81 (M93's redesign)
    -> **12.71** (capstan friction, which ADR-0083 solved in M78 and nothing
    read until M106; 12.13 in M107, before M108 took the capstan back off the
    mechanical term).

    Friction is a tension multiplier and copper loss goes as current squared,
    so it lands on the battery harder than anywhere else. NFR6 now fails at
    BOTH Kt corners -- 12.13 optimistic, 8.50 pessimistic -- where before only
    the pessimistic one was marginal.
    """
    r = power.runtime(_trot())
    assert r["trot_minutes"] == pytest.approx(12.71, abs=0.4)
    assert r["trot_minutes"] < 14.0, (
        "the runtime is back inside NFR6's range -- has friction gone off?"
    )
    # ⚠️ range falls with runtime: 565 -> **381 m**, under NFR6's 420-600.
    assert r["trot_range_m"] == pytest.approx(381.0, abs=12.0)
    assert r["trot_range_m"] < 420.0, "back inside NFR6's range band"
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
