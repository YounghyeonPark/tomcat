# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Electrical power and runtime — closing NFR6, blank since M1 (M16).

Fifteen milestones established what the robot can *do*; none asked how long it
could do it for. `NFR6 (runtime on one battery charge)` has read `TBD` throughout,
and the 300 g battery in the mass budget has never been checked against a load.

The headline is not the runtime. It is that **standing costs 76 % of what moving
costs, for zero work done** — because a tendon-driven joint holds its posture with
motor current, and current dissipates `I^2 R` whether or not anything moves. This
is the quantified case for the power-off brake ADR-0003 already specified on
qualitative grounds.

Model
-----
Per motor, from the resolved gait torques (`dynamics.contact_forces` in stance,
`dynamics.swing_joint_torque` in flight)::

    I       = tau_motor / Kt
    P_cu    = I^2 * R                (copper loss — the dominant term)
    P_mech  = |tau_motor * omega|    (useful work)

⚠️ Deliberately pessimistic in three places, all flagged rather than buried:

- **No regeneration.** Negative mechanical work is treated as dissipated, not
  recovered. A backdrivable QDD drive (ADR-0003) could recover some of it.
- ~~**`P = I^2 R` with the phase-to-phase resistance**~~ — ⚠️ **CORRECTED in M40
  ([ADR-0045](../../../docs/DESIGN_DECISIONS.md)).** This module used to compute
  `I^2 R_pp`, matching the motor down-select note "so the two agree". They agreed
  on a number **1.5x low**: balanced three-phase copper loss is `3 I^2 R_ph`, and
  `R_pp = 2 R_ph`, so the right form is `1.5 * I^2 R_pp`. See `PHASE_FACTOR`.
- **No iron/switching/driver-quiescent losses**, and no gearbox efficiency. Real
  draw will be higher than the motor terms alone; ``ELECTRONICS_W`` is a flat
  allowance for the rest.
"""

from __future__ import annotations

import numpy as np

# Motor electrical constants — SteadyWin GIM3505-9 (docs/notes/motor-downselect.md)
KT = 0.44           # N·m/A at the OUTPUT shaft
R_PHASE_PHASE = 4.466   # ohm

#: Balanced three-phase copper loss over the `I^2 R_pp` shorthand.
#:
#: ⚠️ **M40 (ADR-0045) — this used to be an implicit 1.0 and it was wrong.**
#: Copper loss is the sum over phases of `I_ph,rms^2 R_ph`. For a wye winding the
#: terminal phase-to-phase resistance is `R_pp = 2 R_ph`, so
#:
#:     P = 3 I_rms^2 R_ph = 3 I_rms^2 (R_pp / 2) = **1.5 * I_rms^2 R_pp**
#:
#: This module's docstring flagged the shorthand from the start -- *"a rigorous
#: three-phase treatment would use 1.5 * I_phase^2 * R_phase"* -- and matched the
#: motor down-select note so the two agreed. They agreed on a figure **1.5x low**.
#:
#: ⚠️ It is only exactly 1.5x if `current` here is the RMS phase current, which
#: depends on the convention behind the vendor's rated-current figure that `KT` was
#: derived from. See `docs/notes/motor-spec-review.md` §1b: the vendor's own numbers
#: disagree 27 %, rotor-side is ruled out, and a six-step-vs-sinusoidal reading fits
#: to 0.4 %. The formula correction is firm; which `KT` to feed it is not.
PHASE_FACTOR = 1.5

# Flat allowance for 19 driver boards + RT controller + SBC. [assumed]
ELECTRONICS_W = 15.0

# Battery: the 300 g in the mass budget, at a mid LiPo energy density, 80 % usable.
BATTERY_KG = 0.300
BATTERY_WH_PER_KG = 175.0
BATTERY_USABLE = 0.80


def battery_wh(kg: float = BATTERY_KG, wh_per_kg: float = BATTERY_WH_PER_KG,
               usable: float = BATTERY_USABLE) -> float:
    """Usable energy (Wh) of the pack in the mass budget. All three are `[assumed]`."""
    return kg * wh_per_kg * usable


def gait_power(controller, n: int = 96) -> dict:
    """Mean electrical power (W) of the 12 leg motors over one gait cycle.

    Returns copper, mechanical and total means plus the RMS and peak per-motor
    current, so the result can be checked against the driver's 4.19 A rating as
    well as against the battery.
    """
    from . import dynamics as dyn
    from .params import DEFAULT_TENDON

    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm, dtype=float)
    spool = DEFAULT_TENDON.motor_spool_radius
    legs = ("LF", "RF", "LR", "RR")
    dt = controller.params.period / n
    cyc = dyn.cycle(controller, n)

    q = np.zeros((n, 4, 3))
    tau = np.zeros((n, 4, 3))
    for i in range(n):
        st = controller.state(i / n)
        sol = dyn.contact_forces(controller, i / n, n, cyc=cyc)
        for j, nm in enumerate(legs):
            qq = st.legs[nm].q
            if qq is None:
                continue
            q[i, j] = qq
            if nm in sol.forces:
                f = sol.forces[nm]
                tau[i, j] = controller.body.leg_model_for(nm).jacobian(qq).T @ \
                    np.array([f[0], f[2], 0.0])
            else:
                tau[i, j] = dyn.swing_joint_torque(controller, nm, i / n, n)

    qd = (np.roll(q, -1, axis=0) - np.roll(q, 1, axis=0)) / (2.0 * dt)
    mot_tau = np.abs(tau) / arms * spool
    mot_w = np.abs(qd) * arms / spool
    current = mot_tau / KT

    p_cu = PHASE_FACTOR * (current ** 2) * R_PHASE_PHASE
    p_mech = np.abs(mot_tau * mot_w)
    cu = float(p_cu.sum(axis=(1, 2)).mean())
    mech = float(p_mech.sum(axis=(1, 2)).mean())
    return {
        "copper_w": cu,
        "mechanical_w": mech,
        "legs_w": cu + mech,
        "total_w": cu + mech + ELECTRONICS_W,
        "rms_current_a": float(np.sqrt((current ** 2).mean())),
        "peak_current_a": float(current.max()),
        "efficiency": mech / (cu + mech) if (cu + mech) > 0 else 0.0,
    }


def standing_power(n_legs: int = 4) -> dict:
    """Electrical power (W) to HOLD a stance — pure `I^2 R`, no work done.

    This is the number that matters for a tendon-driven robot. A cable can only
    pull, so posture is held by motor current, and that current burns whether or
    not the robot moves.
    """
    from . import LegModel, TendonMap, torque_budget
    from .params import DEFAULT_TENDON, DEFAULT_LOADS

    stand = [lc for lc in DEFAULT_LOADS if "stand" in lc.name][0]
    res = torque_budget.evaluate(LegModel(), TendonMap(DEFAULT_TENDON), stand)
    per_joint = np.asarray(res.peak_motor_torque, dtype=float) / KT
    per_leg = float((PHASE_FACTOR * (per_joint ** 2) * R_PHASE_PHASE).sum())
    return {"per_leg_w": per_leg, "legs_w": per_leg * n_legs,
            "total_w": per_leg * n_legs + ELECTRONICS_W}


def runtime(controller, n: int = 96) -> dict:
    """Runtime and range for the shipped battery, standing and trotting.

    The ``*_braked`` figures assume the ADR-0003 power-off brake removes the
    standing hold current entirely — which is what makes it "essential" rather
    than "an optimisation".
    """
    g = gait_power(controller, n)
    s = standing_power()
    wh = battery_wh()
    speed = controller.params.body_speed
    return {
        "battery_wh": wh,
        "trot_w": g["total_w"],
        "trot_minutes": 60.0 * wh / g["total_w"],
        "trot_range_m": wh / g["total_w"] * 3600.0 * speed,
        "stand_w": s["total_w"],
        "stand_minutes": 60.0 * wh / s["total_w"],
        "stand_minutes_braked": 60.0 * wh / ELECTRONICS_W,
        "standing_fraction_of_trot": s["legs_w"] / g["legs_w"],
    }
