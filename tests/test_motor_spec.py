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



def test_the_motor_holds_on_TORQUE_again_and_the_arms_are_why(duty):
    """✅ **M111: back inside the proxy on torque, because the arms grew.**

    History of the hind trot's workspace peak: 1.71 N·m (88 %) until M106
    turned capstan friction on; **2.20, 113 %** after it (M107). ADR-0103 then
    found the FORE leg binding at 2.42 and raised the shared moment arms
    28/25/14 -> **36/34/22**. Motor torque goes as `1/r` at fixed joint torque,
    so the hind is now **1.80 N·m, 92 %**, and standing 0.64 against 0.71.

    ⚠️ This test sees the HIND leg (`torques_at` uses it). The binding leg is
    the fore -- 1.825 N·m at its knee -- and `mass_closure.motor_requirement`
    is what reports the pair.

    ⚠️ The arms bought torque with SPEED: the trot now needs ~665 rpm
    no-load against the proxy's 380 (`power.gait_envelope`). See
    `test_the_trot_has_never_fitted_the_proxy_on_SPEED`.
    """
    d = MR.duty_check(duty)
    assert d["trot_motor"] == pytest.approx(1.80, abs=0.05)
    assert 0.85 < d["vs_peak"] < 1.0, f"trot at {100 * d['vs_peak']:.0f} % of peak"
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

    A first pass compared the trot **workspace peak** to the motor's continuous
    rating and called it a thermal violation. It is not: `torque_budget`
    returns the worst pose in the whole reachable workspace, which sizes
    structure, not temperature. What sets temperature is the RMS over the
    trajectory actually walked:

    | Kt | frictionless | M107 friction | **M111 36/34/22 arms** | vs 1.60 A |
    |---|---|---|---|---|
    | 0.44 | 1.03 A | 1.380 A | **0.935 A** | 0.58x |
    | 0.35 | 1.30 A | 1.735 A | **1.175 A** | 0.73x |

    ✅ **M111 put both branches back inside, with room.** M107 had split them
    across the rating -- the vendor's Kt ran hot at 1.735 A. The bigger arms
    cut the cable tension, so the current, by the arm ratio, and copper by its
    square. Peak current is back inside too: 3.51 A at Kt 0.35 against 4.19.
    """
    for kt, expect in ((PW.KT, 0.935), (MR.SPEC["vendor_kt"], 1.175)):
        d = MR.gait_duty(kt, MR.SPOOL_SPEC)
        assert d["rms_a"] == pytest.approx(expect, abs=0.05)
        assert d["rms_a"] < MR.SPEC["rated_current"], (
            f"RMS {d['rms_a']:.2f} A must stay inside the 1.60 A rating"
        )
    d = MR.gait_duty(MR.SPEC["vendor_kt"], MR.SPOOL_SPEC)
    assert d["peak_a"] == pytest.approx(3.51, abs=0.15)
    assert d["peak_a"] < MR.SPEC["peak_current"]


def test_NFR6s_runtime_SURVIVES_once_the_arms_are_right(duty):
    """✅ **M111: NFR6 is met at both Kt corners, for the first time since it
    was re-stated.**

    | | runtime |
    |---|---|
    | published pre-M40 (4.045 kg, 8.0 mm spool, Kt 0.44) | 30.2 min |
    | shipped model, everything folded in (M41) | 18.85 min |
    | redesigned leg (M93) | 18.81 min |
    | capstan friction counted (M107, corrected M108) | 12.71 / 8.78 min |
    | **36/34/22 arms, body 4.468 kg (M111)** | **21.66 / 16.04 min** |

    ✅ The arms are a REDUCTION: the same joint torque at 1/r the cable
    tension, so copper -- which goes as current squared -- roughly halves. The
    heavier sheaves (+21 g a leg) cost far less than the copper saves. Both
    corners now clear NFR6's re-stated 14 min, and the optimistic one its 20.
    """
    wh = PW.battery_wh()
    opt = MR.gait_duty(PW.KT, MR.SPOOL_SPEC)
    pess = MR.gait_duty(MR.SPEC["vendor_kt"], MR.SPOOL_SPEC)
    t_opt = 60.0 * wh / opt["total_w"]
    t_pess = 60.0 * wh / pess["total_w"]
    assert t_opt == pytest.approx(21.66, abs=0.4)
    assert t_pess == pytest.approx(16.04, abs=0.4)
    assert t_pess > 14.0, "the pessimistic corner is back inside NFR6"
    assert t_opt < 30.0, "if this clears 30 min again, NFR6 was re-derived"
    assert t_pess / t_opt < 0.80, "the Kt question alone is worth >20 % of runtime"



def test_the_robot_is_more_than_half_motor_by_mass():
    """19 × 131.7 g = **2.502 kg**, still more than half the body.

    ADR-0008's amendment quotes **45.6 %** -- 19 × 72 g of a 3.0 kg body, and
    both of those numbers are superseded.
    """
    m = MR.mass_fraction(MR.BODY_M38)
    assert m["motors_kg"] == pytest.approx(2.502, abs=0.002)
    assert m["frac"] > 0.55
    # ⚠️ M93: the structure grew but the motor count did not, so the motors'
    # share FALLS, 0.581 -> 0.571; M111's heavier sheaves take it to 0.560.
    assert m["frac"] == pytest.approx(0.560, abs=0.005)
    assert m["rest_kg"] > 1.5, "there must be room left for the structure"
    stale = 19 * 0.072 / 3.0
    assert stale == pytest.approx(0.456, abs=0.002), "ADR-0008's basis, reproduced"



def test_the_down_select_re_run_FAILS_the_smaller_part(duty):
    """The original down-select sized to a 1.10 N·m trot at 3.0 kg and listed
    the GIM3505-8. It does not survive any later requirement.

    ⚠️ M107 found the -9 failing too, at 2.20 N·m with friction live, and
    only the Ø53 GIM4305-10 left. ✅ **M111's arms put the -9 back in** -- 92 %
    of peak on the hind trot, 94 % on the binding fore knee -- so the escape
    hatch is no longer needed on TORQUE. It may still be needed on SPEED.
    """
    trot = float(duty["trot"]["motor"].max())
    rows = {r["name"]: r for r in MR.compare_alternatives(trot, MR.BODY_M38)}
    assert not rows["GIM3505-8"]["ok"], "the small part must still fail"
    assert rows["GIM3505-9"]["ok"] and rows["GIM3505-9"]["vs_peak"] > 0.85
    assert rows["GIM4305-10"]["ok"] and rows["GIM4305-10"]["vs_peak"] < 0.7
    assert rows["GIM4305-10"]["dia"] / rows["GIM3505-9"]["dia"] > 1.5, (
        "and 54 % wider, which the girdle packaging study owns"
    )


def test_the_trot_has_never_fitted_the_proxy_on_SPEED():
    """⚠️ **M111: a defect older than every milestone that touched the motor.**

    `speed_check` above sums the three joints' foot speeds as if they aligned
    (~6 m/s) and compares that with a 0.5 m/s body. The constraint is each
    joint's SWING speed, and the walked trot exceeds the proxy's 380 rpm
    no-load ceiling at the hip and knee under EVERY arm set this project has
    had: 26-27 % over at 28/25/14, 64-72 % at 36/34/22.

    Checked against the motor's torque-speed LINE along the trajectory (a swing
    is fast and light, a stance slow and heavy), a 1.95 N·m motor needs:

    | arms | no-load speed needed (hip / knee / ankle) |
    |---|---|
    | 28/25/14 | 487 / 492 / **3563** rpm |
    | **36/34/22** | **624 / 665 / 474** rpm |

    ✅ Under ADR-0100 the motor is a proxy, so this is SPEC, not failure: the
    mechanism asks ~665 rpm at the spool. The arms trade torque for speed; the
    proxy has too little of their product at any ratio.
    """
    from tomcat_kin import gait
    env = PW.gait_envelope(gait.GaitController(gait.trot_params()))
    assert not env["fits"], "the trot fits the proxy -- was the speed ceiling raised?"
    assert env["need_rpm"][1] == pytest.approx(665.0, rel=0.03)
    assert max(env["need_rpm"]) > 1.5 * PW.NO_LOAD_RPM
    # and the old scope-blind check still passes, which is the defect
    assert MR.speed_check(MR.SPOOL_SPEC)["v_foot_sum"] > 5.0


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



def test_the_runtime_bracket_is_SIXTEEN_to_TWENTY_TWO_minutes():
    """The two Kt corners, and the history of this one number.

    | basis | runtime |
    |---|---|
    | Kt 0.44, redesigned leg (M93) | 18.81 min |
    | Kt 0.35, same | 13.58 min |
    | capstan friction counted (M107/M108) | 12.71 / 8.78 min |
    | **36/34/22 arms (M111)** | **21.66 / 16.04 min** |

    ✅ Both corners now sit inside or above NFR6's re-stated 14-20 min. The
    bracket is still ~26 % wide, and that width is ADR-0044's unresolved Kt.
    """
    wh = PW.battery_wh()
    hi = MR.gait_duty_rigorous(PW.KT, three_phase=True)
    lo = MR.gait_duty_rigorous(MR.SPEC["vendor_kt"], three_phase=True)
    t_hi = 60.0 * wh / hi["total_w"]
    t_lo = 60.0 * wh / lo["total_w"]
    assert t_hi == pytest.approx(21.66, abs=0.4)
    assert t_lo == pytest.approx(16.04, abs=0.4)
    assert t_lo > 14.0, "both corners clear NFR6's re-stated floor"
    assert t_hi < 30.0, "NFR6's published ~30 min does not survive either corner"




# ===================================================================
# M119 -- the speed spec, and the one route that buys it
# ===================================================================

def test_the_speed_is_the_SWING_and_peak_torque_does_not_buy_it():
    """The walked trot's largest torque is the ankle's ~1.23 N.m; its fastest
    joint is the knee's swing at ~653 rpm, carrying almost nothing. So the
    no-load speed it needs barely moves with the motor's peak: 665 rpm at
    1.95 N.m, still ~660 at 3.5. Speed and torque are two separate axes of the
    spec, and a stronger motor is not a faster one."""
    from tomcat_kin import gait
    ctl = gait.GaitController(gait.trot_params())
    lo = PW.gait_envelope(ctl, peak_nm=1.95)
    hi = PW.gait_envelope(ctl, peak_nm=3.5)
    assert max(lo["peak_torque"]) < 1.3
    assert lo["peak_speed_rpm"][1] == pytest.approx(653.0, rel=0.03)
    assert max(hi["need_rpm"]) > 0.98 * max(lo["need_rpm"])


def test_only_a_REWIND_buys_the_speed_and_the_ratio_cannot():
    """⚠️ No-load speed is `15.83 rpm/V x V` at the -9's output, so the stock
    part is short at every pack and even at its 40 V ceiling (633 rpm). A lower
    reduction reaches the speed and loses the continuous rating in proportion
    (0.36-0.53 N.m against 0.624). A rewind -- same frame, same 9:1, Kv x1.33
    to x2 -- keeps the motor constant, so the trot's copper and the thermal
    budget stand; it costs only peak CURRENT, 5.6-8.4 A against 4.19."""
    r = MR.speed_routes(665.0, 0.624, 1.825)
    assert r["ceiling_rpm"] == pytest.approx(633.2, abs=0.5)
    assert r["ceiling_rpm"] < 665.0, "not even at the supply ceiling"
    for row in r["rows"]:
        assert row["in_range"], "every pack here charges inside 12-40 V"
        assert not row["stock_ok"]
        assert not row["ratio_ok"] and row["ratio_rated"] < 0.624
        assert row["rewind_ok"]
    six, eight, nine = r["rows"]
    assert eight["kv_out"] == pytest.approx(23.75, abs=0.05)
    assert eight["rewind_peak_a"] == pytest.approx(6.29, abs=0.05)
    assert six["rewind"] == pytest.approx(2.0, abs=0.01)
