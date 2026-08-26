# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""M39 — the motor reviewed on SPEC. Findings gated.

⚠️ Several assert a **defect** (NFR6's runtime, `power.KT` unswept) so they fail
when the fix lands; that failure is the signal to re-publish, not to relax them.

This closes as much of OPEN_RISKS **R1** as can be closed without buying hardware.
Buying one and weighing it remains the only thing that settles the mass.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
import motor_spec_review as MR  # noqa: E402

from tomcat_kin import power as PW  # noqa: E402
from tomcat_kin.params import DEFAULT_TENDON  # noqa: E402


def test_the_vendors_three_numbers_disagree_by_a_quarter():
    """⚠️ THE spec finding. The same motor, three published readings:

        rated pair  0.71 N·m / 1.60 A  ->  Kt = 0.444
        peak pair   1.95 N·m / 4.19 A  ->  Kt = 0.465
        vendor Kt                          0.350

    `motor-downselect.md` took **0.44** from the current pairs and dismissed the
    quoted 0.35 as "a different reference point". The two pairs do agree with each
    other, so that is defensible — but it is also the **optimistic** branch, and
    copper loss goes as `1/Kt²`, so the spread is worth **1.61×** of dissipation.

    ADR-0021's runtime and ADR-0023/0024's thermal duty both ride on `power.KT`
    and neither swept it.
    """
    k = MR.kt_candidates()
    pair_lo = k["rated pair (0.71/1.60)"]
    pair_hi = k["peak pair (1.95/4.19)"]
    vendor = k["vendor quoted"]

    assert pair_lo == pytest.approx(0.444, abs=0.002)
    assert pair_hi == pytest.approx(0.465, abs=0.002)
    assert abs(pair_hi / pair_lo - 1.0) < 0.06, "the two current pairs agree"
    assert pair_lo / vendor == pytest.approx(1.27, abs=0.02), "vendor Kt is 27 % off"
    assert (pair_lo / vendor) ** 2 > 1.55, "which is 1.6x the copper loss"
    assert PW.KT == pytest.approx(pair_lo, abs=0.01), (
        "power.py should still be on the optimistic branch; if it moved, "
        "ADR-0021/0023/0024 need re-publishing"
    )


@pytest.fixture(scope="module")
def duty():
    return MR.torques_at(MR.BODY_M38, MR.SPOOL_SPEC, grid=15)


def test_the_motor_holds_on_TORQUE_with_less_headroom_than_recorded(duty):
    """At ADR-0043's 4.304 kg and the spool LEG_TENDON_SPEC §2 actually requires,
    the trot workspace peak is **1.71 N·m = 88 % of peak**.

    `motor-reality-check.md` recorded "1.3× peak headroom" — i.e. 77 % — at
    4.045 kg with the 8.0 mm spool. Both of those moved.
    """
    d = MR.duty_check(duty)
    assert d["trot_motor"] == pytest.approx(1.71, abs=0.05)
    assert 0.85 < d["vs_peak"] < 0.95, f"trot at {100 * d['vs_peak']:.0f} % of peak"
    assert d["vs_peak"] < 1.0, "still inside peak"
    assert d["stand_vs_rated"] < 1.0, "standing must be inside the CONTINUOUS rating"


def test_the_bigger_spool_costs_peak_margin_and_buys_foot_speed():
    """§2 raised the spool 8.0 -> 8.75 mm for the Ø1.75 cable's minimum bend.
    `tau_motor = T · r_spool` and `v_cable = omega · r_spool`, so it is a straight
    trade: **8 points of peak margin for 9.4 % of foot speed.**"""
    lo = MR.duty_check(MR.torques_at(MR.BODY_M38, MR.SPOOL_PRE_M41, grid=15))
    hi = MR.duty_check(MR.torques_at(MR.BODY_M38, MR.SPOOL_SPEC, grid=15))
    assert hi["vs_peak"] > lo["vs_peak"], "a bigger spool needs more motor torque"
    ratio = MR.SPOOL_SPEC / MR.SPOOL_PRE_M41
    assert hi["trot_motor"] / lo["trot_motor"] == pytest.approx(ratio, rel=0.02)

    s_lo = MR.speed_check(MR.SPOOL_PRE_M41)
    s_hi = MR.speed_check(MR.SPOOL_SPEC)
    assert s_hi["v_foot_sum"] / s_lo["v_foot_sum"] == pytest.approx(ratio, rel=1e-6)


def test_the_THERMAL_duty_is_fine_and_the_workspace_peak_is_not_the_duty():
    """The reassuring result, and a correction to how I first read it.

    A first pass compared the trot **workspace peak** (2.40× the continuous rating)
    to the motor's continuous rating and called it a thermal violation. It is not:
    `torque_budget` returns the worst pose in the whole reachable workspace, which
    sizes structure, not temperature. What sets temperature is the RMS over the
    trajectory actually walked:

    | Kt | RMS | vs 1.60 A rating |
    |---|---|---|
    | 0.44 | 1.03 A | 0.64× |
    | 0.35 | 1.30 A | 0.81× |

    **Both inside**, even on the pessimistic Kt. The motor is thermally adequate.
    """
    for kt, expect in ((PW.KT, 1.03), (MR.SPEC["vendor_kt"], 1.30)):
        d = MR.gait_duty(kt, MR.SPOOL_SPEC)
        assert d["rms_a"] == pytest.approx(expect, abs=0.05)
        assert d["rms_a"] < MR.SPEC["rated_current"], (
            f"RMS {d['rms_a']:.2f} A must stay inside the 1.60 A rating"
        )
    # and the peak current stays inside the peak rating
    d = MR.gait_duty(MR.SPEC["vendor_kt"], MR.SPOOL_SPEC)
    assert d["peak_a"] < MR.SPEC["peak_current"]


def test_NFR6s_runtime_does_NOT_survive(duty):
    """⚠️ What actually breaks. NFR6 publishes **~30 min / ~900 m**.

    | | runtime |
    |---|---|
    | published pre-M40 (4.045 kg, 8.0 mm spool, Kt 0.44) | 30.2 min |
    | shipped model, everything folded in (M41) | **18.85 min** |
    | ...and on the vendor's Kt | **13.62 min** |

    ⚠️ M41 folded ADR-0043's mass and §2's spool into `params.py`, so `power.py`
    now recomputes rather than being scaled. The answer came out slightly *below*
    the scaled estimate (18.85 against 19.6) because the gait poses shift with the
    spool, which a linear scale factor cannot see.

    ⚠️ **Updated by M40**, which adopted the three-phase copper-loss correction
    into `power.py`, so these are the shipped model's numbers rather than a
    hypothetical.

    So the motor survives on torque and current, and the *runtime requirement*
    does not. Asserts the defect: fails when NFR6 is re-stated.
    """
    wh = PW.battery_wh()
    opt = MR.gait_duty(PW.KT, MR.SPOOL_SPEC)
    pess = MR.gait_duty(MR.SPEC["vendor_kt"], MR.SPOOL_SPEC)
    t_opt = 60.0 * wh / opt["total_w"]
    t_pess = 60.0 * wh / pess["total_w"]

    assert t_opt == pytest.approx(18.85, abs=0.4)
    assert t_pess == pytest.approx(13.62, abs=0.4)
    assert t_opt < 30.0, "if this clears 30 min again, NFR6 was re-derived"
    assert t_pess / t_opt < 0.80, "the Kt question alone is worth >20 % of runtime"


def test_the_robot_is_more_than_half_motor_by_mass():
    """19 × 131.7 g = **2.502 kg of a 4.304 kg body = 58.1 %**, leaving 1.802 kg
    for spine, girdles, ribcage, the 300 g battery, electronics, head and tail.

    ADR-0008's amendment quotes **45.6 %** — that is 19 × 72 g of a 3.0 kg body,
    and both of those numbers are superseded.
    """
    m = MR.mass_fraction(MR.BODY_M38)
    assert m["motors_kg"] == pytest.approx(2.502, abs=0.002)
    assert m["frac"] > 0.55
    assert m["frac"] == pytest.approx(0.581, abs=0.005)
    assert m["rest_kg"] > 1.5, "there must be room left for the structure"
    stale = 19 * 0.072 / 3.0
    assert stale == pytest.approx(0.456, abs=0.002), "ADR-0008's basis, reproduced"


def test_the_down_select_re_run_now_FAILS_the_smaller_part(duty):
    """The original down-select sized to a **1.10 N·m** trot at 3.0 kg and listed
    the GIM3505-8 as meeting the requirement. At 1.71 N·m and 4.30 kg it does not:
    it needs 1.62 N·m against a **1.27 N·m peak**.

    GIM3505-9 stays the pick at 88 % of peak. **GIM4305-10 is the escape hatch** —
    59 % of peak for +158 g of motor — but it is Ø53 against Ø34.5, and the girdle
    was packaged for Ø34.5.
    """
    trot = float(duty["trot"]["motor"].max())
    rows = {r["name"]: r for r in MR.compare_alternatives(trot, MR.BODY_M38)}

    assert not rows["GIM3505-8"]["ok"], "the small part must now fail"
    assert rows["GIM3505-9"]["ok"] and rows["GIM3505-9"]["vs_peak"] > 0.8
    assert rows["GIM4305-10"]["ok"] and rows["GIM4305-10"]["vs_peak"] < 0.7
    assert rows["GIM4305-10"]["body"] > rows["GIM3505-9"]["body"], (
        "the bigger motor makes the body it has to lift heavier"
    )
    assert rows["GIM4305-10"]["dia"] / rows["GIM3505-9"]["dia"] > 1.5, (
        "and 54 % wider, which the girdle packaging study owns"
    )


def test_the_spool_radius_in_params_is_now_the_SPEC_value():
    """✅ **Closed by M41.** This test used to assert the defect — that `params.py`
    carried 0.008 m where LEG_TENDON_SPEC §2 requires **0.00875** for the Ø1.75 mm
    cable's minimum bend — and its failing is what signalled the fix had landed.

    It now guards the fix instead. The spine spool moved with it, for the same
    reason and on the same date it should have (ADR-0046).
    """
    from tomcat_kin.params import DEFAULT_SPINE

    assert float(DEFAULT_TENDON.motor_spool_radius) == pytest.approx(0.00875)
    assert float(DEFAULT_SPINE.motor_spool_radius) == pytest.approx(0.00875)
    assert 0.00875 == pytest.approx(10.0 * 1.75e-3 / 2.0), (
        "the value IS the cable's minimum bend radius, not a free choice"
    )

    assert MR.SPOOL_SPEC == pytest.approx(MR.SPOOL_PARAMS), (
        "the review's target and params now agree"
    )


# ===================================================================
# M39 follow-up — the rotor-side hypothesis, and what actually fits
# ===================================================================

def test_the_vendors_Kt_is_NOT_a_rotor_side_figure():
    """The first hypothesis worth testing, and it fails cleanly.

    If 0.35 N·m/A were referred to the **rotor**, the output constant would be
    `0.35 × 9 = 3.15 N·m/A`, and the rated 0.71 N·m would draw **0.225 A** against
    the **1.60 A** the vendor publishes — **7.1× off, and in the wrong direction.**
    Rotor-side does not reconcile the sheet; it makes the gap seven times worse.
    """
    f = MR.kt_convention_fit()
    assert f["rotor_side_output_kt"] == pytest.approx(3.15, abs=0.01)
    assert f["rotor_side_rated_A"] == pytest.approx(0.225, abs=0.005)
    assert f["rotor_side_error"] > 7.0
    assert f["rotor_side_error"] > 1.0, (
        "the error is in the direction that WIDENS the discrepancy"
    )


def test_a_drive_convention_difference_fits_to_under_half_a_percent():
    """What does reconcile it: the same shaft, two current conventions.

    The ratio to explain is **1.2679**, and **4/π = 1.2732** — the square-wave
    fundamental, i.e. six-step against sinusoidal — lands within **0.4 %**.

    ⚠️ Fitting one ratio against a list of constants is weak evidence and this
    could be coincidence. Its value is that it sharpens the question for the
    vendor: not *"which number is wrong"* but *"are the current ratings six-step or
    sinusoidal, and is Kt peak-phase or RMS"*. Under a convention difference
    **both published numbers are right** and only the driver's current-sense
    definition decides which to use.
    """
    f = MR.kt_convention_fit()
    assert f["needed"] == pytest.approx(1.2679, abs=0.002)
    best = min(f["candidates"].items(), key=lambda kv: abs(kv[1]["err"]))
    assert "4/pi" in best[0], f"best fit moved to {best[0]}"
    assert abs(best[1]["err"]) < 0.005


def test_the_three_phase_copper_loss_convention_is_a_factor_of_ONE_POINT_FIVE():
    """⚠️ Found while digging into the Kt question, and firmer than it — this half
    is circuit theory rather than a datasheet guess.

    Copper loss is `Σ I_ph,rms² R_ph`; balanced three-phase that is
    `3 I² R_ph = 1.5 I² R_pp`, because a wye winding's terminal phase-to-phase
    resistance is `2 R_ph`. `power.py` computes `I² R_pp` — **1.5× low** for
    whatever current it is handed. Its own docstring says so and nothing had priced
    it.

    ⚠️ **The two factors are entangled, not independent.** Whether the current
    `power.py` computes *is* the RMS phase current depends on the same datasheet
    ambiguity, so they bracket rather than multiply cleanly.

    ⚠️ **M40 adopted this into `power.py`**, so `THREE_PHASE_FACTOR` reversed roles:
    it now scales *down* to reproduce the pre-M40 figure. Applying it on top of the
    corrected model double-counts — which is what happened when M40 landed, and
    what these tests caught.
    """
    assert PW.PHASE_FACTOR == 1.5, "M40 adopted the correction into power.py"
    assert MR.THREE_PHASE_FACTOR == 1.5
    assert PW.R_PHASE_PHASE == pytest.approx(4.466, abs=1e-3)

    for kt in (PW.KT, MR.SPEC["vendor_kt"]):
        loose = MR.gait_duty_rigorous(kt, MR.SPOOL_SPEC, three_phase=False)
        tight = MR.gait_duty_rigorous(kt, MR.SPOOL_SPEC, three_phase=True)
        assert tight["copper_w"] / loose["copper_w"] == pytest.approx(1.5, rel=1e-9)


def test_the_runtime_bracket_is_FOURTEEN_to_NINETEEN_minutes():
    """⚠️ Tightens ADR-0044's first published range. The four corners:

    | basis | total | runtime |
    |---|---|---|
    | **Kt 0.44, everything folded in (M41)** | 133.7 W | **18.85 min** |
    | **Kt 0.35, same** | 185.0 W | **13.62 min** |

    The three-phase factor applies under any Kt reading, so the honest bracket is
    **14–19 min** against NFR6's published ~30. The history of this one number is
    the history of the corrections: 30.2 published → 25.2 (mass + spool) → 19.6
    (three-phase formula) → **18.85** (recomputed rather than scaled).
    """
    wh = PW.battery_wh()
    hi = MR.gait_duty_rigorous(PW.KT, three_phase=True)
    lo = MR.gait_duty_rigorous(MR.SPEC["vendor_kt"], three_phase=True)
    t_hi = 60.0 * wh / hi["total_w"]
    t_lo = 60.0 * wh / lo["total_w"]

    assert t_hi == pytest.approx(18.85, abs=0.4)
    assert t_lo == pytest.approx(13.62, abs=0.4)
    assert t_hi < 20.0 and t_lo > 13.0, "the 14-19 min bracket"
    assert t_hi < 30.0, "NFR6's published ~30 min does not survive either corner"
