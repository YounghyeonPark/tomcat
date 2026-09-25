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
    # ⚠️ **This bypassed the capstan and `standing_power` did not.** It builds
    # the motor torque straight from `tau / arm * spool` instead of going
    # through `TendonMap`, so when M106 made friction live the STAND picked it
    # up and the TROT did not -- and the ratio between them, which is the M16
    # finding, became an artefact of the inconsistency rather than physics.
    # The factor is per joint AND per side: the loaded cable is the flexor
    # where `tau >= 0` and the extensor below it, and the two sides of one pair
    # carry different wraps (the hip's are 135.6 and 239.4 deg).
    from . import TendonMap
    tmap = TendonMap(DEFAULT_TENDON)
    cap = np.where(tau >= 0.0,
                   np.asarray(tmap.capstan_factor(side=+1), dtype=float),
                   np.asarray(tmap.capstan_factor(side=-1), dtype=float))
    # ⚠️ **The capstan belongs on the CURRENT path only.** A first version of
    # this multiplied `mot_tau` once, before splitting, so `p_mech` carried the
    # factor too -- friction booked as useful work. It shows up as efficiency
    # FALLING when copper falls, because copper goes as the square and the
    # mechanical term as the first power. The joint's useful output is
    # `tau * qd` whatever the routing costs to deliver it; the motor's extra is
    # loss, and loss is what the copper term is for.
    mot_tau = np.abs(tau) / arms * spool          # joint-side, for the work
    mot_drive = mot_tau * cap                     # motor-side, for the current
    mot_w = np.abs(qd) * arms / spool
    current = mot_drive / KT

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


#: The proxy's no-load output speed at 24 V. `[sourced: motor-downselect.md]`
#: Under load it is lower, so every speed check against it is optimistic.
NO_LOAD_RPM = 380.0
#: The proxy's peak output torque, N.m -- the other end of the same line.
PEAK_NM = 1.95


def walked_trajectory(controller, n: int = 400):
    """Joint torque, joint speed and capstan factor along the walked gait.

    Arrays of shape `(n, 4 legs, 3 joints)`, legs in `LF RF LR RR` order. The
    torque is what the gait actually asks -- stance from the contact forces,
    swing from the leg's own dynamics -- not `torque_budget`'s worst reachable
    pose. The capstan is the side that carries it.
    """
    from . import dynamics as dyn
    from . import TendonMap
    from .params import DEFAULT_TENDON

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
                tau[i, j] = controller.body.leg_model_for(nm).jacobian(qq).T @                     np.array([f[0], f[2], 0.0])
            else:
                tau[i, j] = dyn.swing_joint_torque(controller, nm, i / n, n)
    qd = (np.roll(q, -1, axis=0) - np.roll(q, 1, axis=0)) / (2.0 * dt)
    tmap = TendonMap(DEFAULT_TENDON)
    cap = np.where(tau >= 0.0,
                   np.asarray(tmap.capstan_factor(side=+1), dtype=float),
                   np.asarray(tmap.capstan_factor(side=-1), dtype=float))
    return tau, qd, cap


def gait_envelope(controller, n: int = 400, arms=None,
                  peak_nm: float = PEAK_NM, no_load_rpm: float = NO_LOAD_RPM) -> dict:
    """The walked trajectory against a motor's torque-speed LINE, per joint.

    ⚠️ **M111: the trot has never fitted the proxy's speed, and nothing
    looked.** `tools/motor_spec_review.speed_check` sums the three joints' foot
    speeds as if they aligned (~6 m/s) and compares that with a 0.5 m/s body --
    the wrong scope, because the constraint is each joint's SWING speed. At the
    28/25/14 arms the hip and knee already swung 26-27 % faster than the no-load
    ceiling; the 36/34/22 arms of ADR-0103 make that 64-72 %.

    ✅ Separate peaks overstate it -- a swing is fast and light, a stance slow
    and heavy -- so this checks each instant against the linear DC line
    `T/T_peak + w/w_noload <= 1`, with the capstan on the torque exactly as
    `gait_power` puts it.

    Returns per joint: the worst envelope ratio (<= 1 reachable), the peak motor
    torque and speed the trajectory actually asks, and `need_rpm` -- the no-load
    speed a motor with `peak_nm` would need for the whole trot to fit.
    """
    from .params import DEFAULT_TENDON

    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm if arms is None else arms,
                      dtype=float)
    spool = DEFAULT_TENDON.motor_spool_radius
    tau, qd, cap = walked_trajectory(controller, n)
    t_mot = np.abs(tau) / arms * spool * cap
    w_mot = np.abs(qd) * arms / spool
    w_nl = no_load_rpm * 2.0 * np.pi / 60.0
    ratio = t_mot / peak_nm + w_mot / w_nl
    need = np.full(3, np.inf)
    for j in range(3):
        tj, wj = t_mot[:, :, j], w_mot[:, :, j]
        if np.all(tj < peak_nm):
            need[j] = float((wj / (1.0 - tj / peak_nm)).max() * 60.0 / (2.0 * np.pi))
    return {
        "worst": ratio.max(axis=(0, 1)),
        "peak_torque": t_mot.max(axis=(0, 1)),
        "peak_speed_rpm": w_mot.max(axis=(0, 1)) * 60.0 / (2.0 * np.pi),
        "need_rpm": need,
        "fits": bool(np.all(ratio <= 1.0)),
    }


def standing_power(n_legs: int = 4, foot=(0.04, -0.17)) -> dict:
    """Electrical power (W) to HOLD a stance -- pure `I^2 R`, no work done.

    This is the number that matters for a tendon-driven robot. A cable can only
    pull, so posture is held by motor current, and that current burns whether or
    not the robot moves.

    ⚠️ **M115: priced at the STANCE, not at the worst reachable pose.** Until
    here this took `torque_budget`'s standing case -- the worst pose over the
    whole reachable workspace, pretension included -- and charged it to every
    joint of every leg at once, while `gait_power` averaged the trajectory
    actually walked. M16's stand/trot ratio therefore compared two different
    bases, and M111 watched it cross 1.0 for no physical reason. Temperature is
    set by the load actually held, which M39 already said of the trot. Now: the
    body's weight split fore/hind by the CoM's lever over the feet, `J^T f` at
    the stance pose, and the same `|tau| / r * R * capstan` `gait_power` uses.
    The old basis survives as `standing_power_worst_pose`.
    """
    from . import GaitController, gait
    from . import LegModel, TendonMap
    from .params import DEFAULT_TENDON, GRAVITY

    body = GaitController(gait.trot_params()).body
    names = ("LF", "RF", "LR", "RR")
    q = {nm: np.asarray(body.leg_model_for(nm).inverse((foot[0], foot[1], 0.0)),
                        float) for nm in names}
    spine_q = np.zeros(body.spine.params.n_segments)
    com = body.center_of_mass(spine_q, q)
    feet = body.foot_positions(spine_q, q)
    x_f = 0.5 * (feet["LF"][0] + feet["RF"][0])
    x_h = 0.5 * (feet["LR"][0] + feet["RR"][0])
    frac_fore = float((com.com[0] - x_h) / (x_f - x_h))
    W = com.mass * GRAVITY

    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm, dtype=float)
    R = DEFAULT_TENDON.motor_spool_radius
    tmap = TendonMap(DEFAULT_TENDON)
    c_pos = np.asarray(tmap.capstan_factor(side=+1), dtype=float)
    c_neg = np.asarray(tmap.capstan_factor(side=-1), dtype=float)
    per_leg = {}
    motor = {}
    for nm in names:
        F = (frac_fore if nm[1] == "F" else 1.0 - frac_fore) * W / 2.0
        tau = body.leg_model_for(nm).jacobian(q[nm]).T @ np.array([0.0, F, 0.0])
        cap = np.where(tau >= 0.0, c_pos, c_neg)
        m_nm = np.abs(tau) / arms * R * cap
        motor[nm] = m_nm
        per_leg[nm] = float((PHASE_FACTOR * (m_nm / KT) ** 2 * R_PHASE_PHASE).sum())
    legs_w = float(sum(per_leg.values())) * n_legs / 4.0
    return {"per_leg_w": legs_w / n_legs, "legs_w": legs_w,
            "total_w": legs_w + ELECTRONICS_W, "frac_fore": frac_fore,
            "motor_torque": motor}


def standing_power_worst_pose(n_legs: int = 4) -> dict:
    """The pre-M115 basis: `torque_budget`'s worst reachable pose, pretension
    included, charged to every joint of every leg. An upper bound on holding
    cost, and the wrong quantity for temperature -- see `standing_power`."""
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
