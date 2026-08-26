# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""M39 — the motor, reviewed on SPEC at the M38 body mass.

R1 ("buy one and weigh it") is the cheapest way to settle the actuator story and it
is still open. This is what can be settled **without** buying anything: re-derive
every actuator number from the vendor's own published figures at the 4.304 kg body
ADR-0043 closed on, and check the three published numbers against each other.

⚠️ **The vendor's numbers for the GIM3505-9 are mutually inconsistent by ~27 %, and
the whole power/thermal chain rests on which one is believed.**

    rated pair   0.71 N.m / 1.60 A  ->  Kt = 0.444 N.m/A
    peak pair    1.95 N.m / 4.19 A  ->  Kt = 0.465 N.m/A
    vendor Kt                           0.35  N.m/A

`motor-downselect.md` chose **0.44** from the current pairs and dismissed the
quoted 0.35 as *"a different reference point"*. That is defensible — the two pairs
agree with each other — but it is also the **optimistic** choice, because current
is `tau/Kt` and copper loss is `I^2 R`:

    (0.44 / 0.35)^2 = 1.58x more copper loss if the vendor's Kt is the true one.

ADR-0021's runtime and ADR-0023/0024's thermal duty are both built on
`power.KT = 0.44`. Neither swept it. This module does.

Run:  python tools/motor_spec_review.py
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "kinematics", "src"))

from tomcat_kin import LegModel                                # noqa: E402
from tomcat_kin.tendon import TendonMap                          # noqa: E402
from tomcat_kin.torque_budget import evaluate as budget          # noqa: E402
from tomcat_kin.params import (                                  # noqa: E402
    DEFAULT_FORELEG, DEFAULT_HINDLEG, DEFAULT_TENDON, DEFAULT_SPINE, LoadCase,
)
from tomcat_kin import power as PW                               # noqa: E402

# ---------------------------------------------------------------- vendor sheet
#: SteadyWin GIM3505-9, as published. `[sourced: motor-downselect.md §2]`
SPEC = {
    "rated_torque": 0.71,      # N.m at the output shaft
    "peak_torque": 1.95,       # N.m
    "rated_current": 1.60,     # A
    "peak_current": 4.19,      # A
    "vendor_kt": 0.35,         # N.m/A, as quoted
    "r_phase_phase": 4.466,    # ohm
    "voltage": 24.0,           # V nominal (12-40 range)
    "speed_rpm": 380.0,        # no-load / rated output speed
    "ratio": 9.0,
    "mass_g": 131.7,           # with the integrated driver
    "dia_mm": 34.5,
    "len_mm": 36.1,
}

#: Body mass. ⚠️ **M41 folded ADR-0043's figure into `params.py`**, so this now
#: reads the live value instead of anticipating it. `BODY_PRE_M41` is kept only so
#: the superseded figures below stay reproducible.
BODY_M38 = float(DEFAULT_SPINE.trunk_mass) + 2 * sum(DEFAULT_HINDLEG.link_mass) \
    + 2 * sum(DEFAULT_FORELEG.link_mass)
BODY_PRE_M41 = 4.045

#: Spool radius. ⚠️ M41 folded §2's 8.75 mm into `params.py` too, so `SPOOL_SPEC`
#: and `SPOOL_PARAMS` are now the same number. `SPOOL_PRE_M41` keeps the old one.
SPOOL_SPEC = 0.00875
SPOOL_PARAMS = float(DEFAULT_TENDON.motor_spool_radius)
SPOOL_PRE_M41 = 0.008

N_MOTORS = 19


def kt_candidates():
    """The three mutually inconsistent readings of the same motor."""
    return {
        "rated pair (0.71/1.60)": SPEC["rated_torque"] / SPEC["rated_current"],
        "peak pair (1.95/4.19)": SPEC["peak_torque"] / SPEC["peak_current"],
        "vendor quoted": SPEC["vendor_kt"],
        "power.py in use": PW.KT,
    }


def torques_at(body_kg: float, spool_r: float, grid: int = 21):
    """Joint / motor torque per load case, at a body mass and spool radius."""
    tp = type(DEFAULT_TENDON)(
        **{**DEFAULT_TENDON.__dict__, "motor_spool_radius": spool_r})
    leg, tm = LegModel(DEFAULT_HINDLEG), TendonMap(tp)
    out = {}
    for name, legs, dyn in (("stand", 4, 1.0), ("trot", 2, 1.5), ("land", 1, 2.5)):
        r = budget(leg, tm,
                   LoadCase(name, body_mass_kg=body_kg, n_stance_legs=legs,
                            dynamic_factor=dyn), grid=grid)
        out[name] = {"joint": np.asarray(r.peak_joint_torque),
                     "motor": np.asarray(r.peak_motor_torque),
                     "T": np.asarray(r.peak_tension)}
    return out


def duty_check(t):
    """Torque duty per case.

    ⚠️ **These are WORKSPACE peaks, not duty cycles**, and a first pass here read
    them as duty. `torque_budget.evaluate` sweeps the whole reachable foot workspace
    and returns the worst pose — the right basis for *sizing a structure*, the wrong
    one for *thermal duty*. What sets temperature is the RMS over the trajectory the
    robot actually walks; see `gait_duty`, which is the quantity ADR-0021 quoted as
    0.89 A against a 1.60 A rating.
    """
    trot = float(t["trot"]["motor"].max())
    return {
        "trot_motor": trot,
        "vs_rated": trot / SPEC["rated_torque"],
        "vs_peak": trot / SPEC["peak_torque"],
        "stand_motor": float(t["stand"]["motor"].max()),
        "stand_vs_rated": float(t["stand"]["motor"].max()) / SPEC["rated_torque"],
        "land_motor": float(t["land"]["motor"].max()),
        "land_vs_peak": float(t["land"]["motor"].max()) / SPEC["peak_torque"],
    }


def electrical(t, kt: float):
    """Current and copper loss per motor at each case, for a given Kt."""
    out = {}
    for case in ("stand", "trot", "land"):
        tau = float(t[case]["motor"].max())
        i = tau / kt
        out[case] = {"tau": tau, "I": i,
                     "p_cu": i * i * SPEC["r_phase_phase"],
                     "vs_rated_A": i / SPEC["rated_current"],
                     "vs_peak_A": i / SPEC["peak_current"]}
    return out


def gait_duty(kt: float, spool_r=None):
    """RMS current and copper loss over the ACTUAL trot cycle — the thermal duty.

    `power.gait_power` integrates the gait rather than sweeping the workspace, so
    this is what the motor's continuous rating must be compared against.

    ⚠️ **M41 removed the scaling that used to live here.** `power.py` reads body
    mass and spool radius from `params.py`, and both of ADR-0043's and §2's figures
    are now folded in — so scaling on top of it double-counted, exactly the way
    ADR-0045's `THREE_PHASE_FACTOR` did when M40 landed. Second time, same shape:
    **a compensating factor outlives the thing it compensated for.**

    `spool_r` is kept in the signature and honoured only when it differs from what
    `params` carries, so the pre-M41 figures stay reproducible.
    """
    from tomcat_kin import GaitController
    from tomcat_kin.gait import trot_params

    c = GaitController(params=trot_params())
    saved = PW.KT
    try:
        PW.KT = kt
        g = PW.gait_power(c)
    finally:
        PW.KT = saved
    f = 1.0 if spool_r is None else spool_r / SPOOL_PARAMS
    copper = g["copper_w"] * f * f
    mech = g["mechanical_w"]
    return {"rms_a": g["rms_current_a"] * f, "peak_a": g["peak_current_a"] * f,
            "copper_w": copper, "mech_w": mech,
            "total_w": copper + mech + PW.ELECTRONICS_W, "scale": f}


def kt_convention_fit():
    """Which convention difference reconciles the vendor's Kt with its currents?

    ⚠️ **The rotor-side reading does not work**, and it was the first hypothesis
    worth testing. If 0.35 N.m/A were at the rotor, output Kt would be
    `0.35 x 9 = 3.15` and rated 0.71 N.m would draw **0.225 A**, not the 1.60 A the
    vendor publishes -- **7.1x off, in the wrong direction.** It makes the
    discrepancy worse, not better.

    What fits is a **current/drive-convention** difference on the same shaft. The
    ratio to explain is `0.444 / 0.350 = 1.2679`, and **4/pi = 1.2732** (the
    square-wave fundamental, i.e. six-step against sinusoidal) lands within 0.5 %.

    ⚠️ **Fitting one ratio against a list of constants is weak evidence** -- 4/pi
    at 0.4 % could be coincidence. What it buys is a sharper question for the
    vendor: not *"which of your numbers is wrong"* but *"are the current ratings
    six-step or sinusoidal, and is Kt peak-phase or RMS"* -- because under a
    convention difference **both published numbers are correct**, and only the
    driver's current-sense definition decides which to use.
    """
    kt_pair = SPEC["rated_torque"] / SPEC["rated_current"]
    rotor_out = SPEC["vendor_kt"] * SPEC["ratio"]
    cands = {
        "4/pi (square-wave fundamental)": 4.0 / math.pi,
        "pi/sqrt(6)": math.pi / math.sqrt(6.0),
        "sqrt(3/2) (RMS vs peak phase)": math.sqrt(1.5),
        "sqrt(3) (line vs phase)": math.sqrt(3.0),
        "3/2": 1.5,
    }
    return {
        "kt_pair": kt_pair,
        "rotor_side_output_kt": rotor_out,
        "rotor_side_rated_A": SPEC["rated_torque"] / rotor_out,
        "rotor_side_error": SPEC["rated_current"]
        / (SPEC["rated_torque"] / rotor_out),
        "needed": kt_pair / SPEC["vendor_kt"],
        "candidates": {k: {"factor": v, "kt": SPEC["vendor_kt"] * v,
                           "err": SPEC["vendor_kt"] * v / kt_pair - 1.0}
                       for k, v in cands.items()},
    }


#: Rigorous balanced three-phase copper loss over `power.py`'s convention.
#:
#: ⚠️ **This one is circuit theory, not a datasheet guess.** `power.py` computes
#: `I^2 * R_phase_phase`; for a wye winding the terminal phase-to-phase resistance
#: is `2 * R_phase`, and balanced copper loss is `3 * I_rms^2 * R_phase`, i.e.
#: `1.5 * I^2 * R_pp`. `power.py`'s own docstring flags the simplification -- *"a
#: rigorous three-phase treatment would use 1.5 * I_phase^2 * R_phase"* -- and
#: nothing had ever priced it.
#
# ⚠️ **ADOPTED in M40 (ADR-0045) -- `power.py` now carries it.** This constant
# therefore reversed roles: it used to scale `power.py` UP to the rigorous value,
# and it now scales the current model DOWN to reproduce the pre-M40 figure for
# comparison. Applying it on top of the corrected model double-counts -- which is
# exactly what happened when M40 landed, and the M39 tests caught it.
THREE_PHASE_FACTOR = 1.5


def gait_duty_rigorous(kt: float, spool_r=None, three_phase: bool = True):
    """Duty under the CURRENT model, or under the pre-M40 shorthand.

    ⚠️ `three_phase=True` is now a **pass-through**: `power.py` already computes the
    rigorous three-phase form. `three_phase=False` divides it back out so the
    historical figures stay reproducible.
    """
    d = gait_duty(kt, spool_r)
    f = 1.0 if three_phase else 1.0 / THREE_PHASE_FACTOR
    copper = d["copper_w"] * f
    return {**d, "copper_w": copper,
            "total_w": copper + d["mech_w"] + PW.ELECTRONICS_W}


def speed_check(spool_r: float):
    """Foot speed available from 380 rpm through the tendon ratios.

    The three joints contribute in series along the limb, so the ceiling is their
    sum when the motions align — which is the convention `control.py` uses for its
    quoted ~5.93 m/s.
    """
    omega_out = SPEC["speed_rpm"] * 2.0 * math.pi / 60.0     # rad/s at the spool
    v_cable = omega_out * spool_r                            # m/s of cable
    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm)
    L = np.asarray(DEFAULT_HINDLEG.link_lengths)
    # lever from each joint to the paw tip
    lever = np.array([L.sum(), L[1:].sum(), L[2:].sum()])
    q_dot = v_cable / arms
    return {"omega_out": omega_out, "v_cable": v_cable,
            "q_dot": q_dot, "v_foot_per_joint": q_dot * lever,
            "v_foot_sum": float((q_dot * lever).sum())}


#: The surveyed alternatives, as published. `[sourced: motor-reality-check.md §2]`
#: ⚠️ GIM4305-10's figures come from a reseller listing, not a manufacturer sheet.
ALTERNATIVES = [
    # name,            rated, peak, ratio, mass_g, dia, len
    ("GIM3505-8",       0.65, 1.27,   8.0,  120.0, 35.0, 36.0),
    ("GIM3505-9",       0.71, 1.95,   9.0,  131.7, 34.5, 36.1),
    ("GIM4305-10",      1.00, 3.00,  10.0,  140.0, 53.0, 26.0),
]


def compare_alternatives(trot_motor_Nm: float, body_kg: float):
    """Re-run the down-select against the CURRENT requirement.

    The original down-select sized to a **1.10 N.m** trot at a 3.0 kg body. The
    requirement is now `trot_motor_Nm` at `body_kg`, so the comparison is worth
    redoing rather than inherited — and each candidate's own mass feeds back into
    the body mass it has to lift.
    """
    rows = []
    for name, rated, peak, ratio, m_g, dia, ln in ALTERNATIVES:
        # its own mass changes the body, which changes the torque, linearly
        body = body_kg - N_MOTORS * (SPEC["mass_g"] - m_g) * 1e-3
        tau = trot_motor_Nm * body / body_kg
        rows.append({"name": name, "rated": rated, "peak": peak, "mass_g": m_g,
                     "body": body, "tau": tau,
                     "vs_rated": tau / rated, "vs_peak": tau / peak,
                     "ok": tau <= peak, "dia": dia, "len": ln,
                     "motors_kg": N_MOTORS * m_g * 1e-3})
    return rows


def mass_fraction(body_kg: float):
    motors = N_MOTORS * SPEC["mass_g"] * 1e-3
    return {"motors_kg": motors, "frac": motors / body_kg,
            "rest_kg": body_kg - motors}


def runtime_under(kt: float):
    """Runtime with `power.py`'s model but a different Kt — the sensitivity."""
    from tomcat_kin import GaitController
    from tomcat_kin.gait import trot_params

    c = GaitController(params=trot_params())
    saved = PW.KT
    try:
        PW.KT = kt
        g = PW.gait_power(c)
        wh = PW.battery_wh()
        total = float(g["total_w"]) if "total_w" in g else float(
            g.get("electrical_w", float("nan")))
        return {"power_w": total, "minutes": 60.0 * wh / total, "keys": list(g)}
    finally:
        PW.KT = saved


def main():
    print("MOTOR SPEC REVIEW — SteadyWin GIM3505-9 at the M38 body mass\n")

    print("1. The vendor's own numbers do not agree with each other")
    print("   %-26s %8s" % ("reading", "Kt N.m/A"))
    print("   " + "-" * 36)
    for k, v in kt_candidates().items():
        print("   %-26s %8.3f" % (k, v))
    lo, hi = SPEC["vendor_kt"], SPEC["rated_torque"] / SPEC["rated_current"]
    print("   spread %.0f %%  ->  copper loss ratio %.2fx"
          % (100 * (hi / lo - 1.0), (hi / lo) ** 2))

    print("\n2. Torque duty, at the mass ADR-0043 closed on (%.3f kg)" % BODY_M38)
    for spool, label in ((SPOOL_PARAMS, "params  r_spool 8.00 mm"),
                         (SPOOL_SPEC, "SPEC    r_spool 8.75 mm")):
        t = torques_at(BODY_M38, spool)
        d = duty_check(t)
        print("   %-26s stand %.3f (%.2fx rated)  trot %.3f (%.2fx rated, "
              "%.0f %% peak)  land %.3f (%.2fx peak)"
              % (label, d["stand_motor"], d["stand_vs_rated"],
                 d["trot_motor"], d["vs_rated"], 100 * d["vs_peak"],
                 d["land_motor"], d["land_vs_peak"]))

    print("   (WORKSPACE peaks -- right for sizing a part, wrong for duty; see 3)")
    t_spec_trot = float(torques_at(BODY_M38, SPOOL_SPEC)["trot"]["motor"].max())
    print("\n3. Electrical, under each Kt (spool 8.75 mm, %.3f kg)" % BODY_M38)
    print("   RMS over the ACTUAL trot cycle, scaled to %.3f kg and the 8.75 mm "
          "spool:" % BODY_M38)
    print("   %-14s %8s %8s %10s %9s %11s"
          % ("Kt", "RMS A", "peak A", "copper W", "total W", "runtime"))
    print("   " + "-" * 64)
    wh = PW.battery_wh()
    rows = {}
    for kt, label in ((PW.KT, "0.44 (in use)"), (SPEC["vendor_kt"], "0.35 (vendor)")):
        d = gait_duty(kt, SPOOL_SPEC)
        rows[kt] = d
        print("   %-14s %8.2f %8.2f %10.1f %9.1f %8.1f min"
              % (label, d["rms_a"], d["peak_a"], d["copper_w"], d["total_w"],
                 60.0 * wh / d["total_w"]))
    a, b = rows[PW.KT], rows[SPEC["vendor_kt"]]
    print("   RMS against the 1.60 A continuous rating: %.2fx and %.2fx"
          % (a["rms_a"] / SPEC["rated_current"],
             b["rms_a"] / SPEC["rated_current"]))
    print("   believing the vendor's Kt costs %.0f %% of runtime "
          "(%.1f -> %.1f min)"
          % (100 * (1.0 - a["total_w"] / b["total_w"]),
             60.0 * wh / a["total_w"], 60.0 * wh / b["total_w"]))

    print("")
    print("3b. Is the vendor's 0.35 a ROTOR-side figure?")
    f = kt_convention_fit()
    print("    If it were: output Kt = %.2f N.m/A, so rated 0.71 N.m would draw "
          "%.3f A" % (f["rotor_side_output_kt"], f["rotor_side_rated_A"]))
    print("    against the %.2f A published -- %.1fx off, in the WRONG direction."
          % (SPEC["rated_current"], f["rotor_side_error"]))
    print("    Rotor-side is RULED OUT. A drive-convention difference fits:")
    print("      %-38s %8s %8s %9s" % ("candidate", "factor", "Kt", "err"))
    for k, v in f["candidates"].items():
        print("      %-38s %8.4f %8.4f %+8.1f %%"
              % (k, v["factor"], v["kt"], 100 * v["err"]))
    print("    needed ratio %.4f -- 4/pi lands within 0.5 %%" % f["needed"])
    print("")
    print("3c. A FIRMER factor, same direction: the three-phase convention")
    print("    power.py uses I^2 * R_pp; balanced 3-phase is 3 I^2 R_ph = 1.5x that.")
    print("    Its own docstring flags it; nothing had priced it.")
    print("    %-22s %10s %10s %11s" % ("basis", "copper W", "total W", "runtime"))
    print("    " + "-" * 56)
    for kt, label in ((PW.KT, "Kt 0.44"), (SPEC["vendor_kt"], "Kt 0.35")):
        for tp, tag in ((False, "pre-M40 shorthand"), (True, "rigorous, shipped")):
            d = gait_duty_rigorous(kt, SPOOL_SPEC, three_phase=tp)
            print("    %-22s %10.1f %10.1f %8.1f min"
                  % ("%s, %s" % (label, tag), d["copper_w"], d["total_w"],
                     60.0 * wh / d["total_w"]))

    print("\n4. Speed — 380 rpm through the tendon ratios")
    for spool, label in ((SPOOL_PARAMS, "8.00 mm"), (SPOOL_SPEC, "8.75 mm")):
        s = speed_check(spool)
        print("   r_spool %s: cable %.3f m/s, joint rates %s rad/s, "
              "foot ceiling %.2f m/s"
              % (label, s["v_cable"], np.round(s["q_dot"], 1), s["v_foot_sum"]))
    print("   control.py quotes a 5.93 m/s ceiling and NFR14 needs 4.1 m/s spare")

    print("\n5. Mass fraction")
    for body, label in ((BODY_PRE_M41, "pre-M41 4.045 kg"),
                        (BODY_M38, "shipped %.3f kg" % BODY_M38)):
        m = mass_fraction(body)
        print("   %-18s 19 x %.1f g = %.3f kg = %.1f %% of body, %.3f kg for "
              "everything else" % (label, SPEC["mass_g"], m["motors_kg"],
                                   100 * m["frac"], m["rest_kg"]))
    print("   ADR-0008 quotes 45.6 % -- that is 19 x 72 g of a 3.0 kg body, "
          "both superseded")

    print("")
    print("6. The down-select, re-run against the CURRENT requirement")
    print("   (original basis was a 1.10 N.m trot at 3.0 kg; it is now "
          "%.2f N.m at %.2f kg)" % (t_spec_trot, BODY_M38))
    print("   %-13s %7s %7s %7s %8s %9s %8s %s"
          % ("part", "rated", "peak", "mass g", "body kg", "tau need",
             "x rated", "verdict"))
    print("   " + "-" * 78)
    for r in compare_alternatives(t_spec_trot, BODY_M38):
        print("   %-13s %7.2f %7.2f %7.1f %8.3f %9.3f %8.2f  %s"
              % (r["name"], r["rated"], r["peak"], r["mass_g"], r["body"],
                 r["tau"], r["vs_rated"],
                 ("OK, %.0f %% of peak" % (100 * r["vs_peak"])) if r["ok"]
                 else "OVER PEAK"))
    print("   Ø: 35.0 / 34.5 / 53.0 mm -- the 4305 is 54 %% wider, and the "
          "girdle was packaged for Ø34.5")


if __name__ == "__main__":
    main()
