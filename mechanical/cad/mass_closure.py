# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""M38 — close the mass spiral with the measured leg hardware.

ADR-0041 measured **167 g** of hind-leg hardware against `LegParams.link_mass`'s
**110 g**. That is not a bookkeeping error to note and move past: `total_mass` is
`trunk_mass + sum(leg masses)`, so heavier legs raise the body mass, which raises
every foot support force, which raises every joint torque and cable tension, which
can force bigger hardware — the ADR-0010 spiral, re-entered with better inputs.

ADR-0010 showed the spiral **converges** only because the chosen motor has
headroom. This module runs it and reports where it lands, and what it breaks on
the way.

⚠️ **M41 folded the result into `params.py`.** So this module changed role: it used
to *predict* the closure, and it now *reproduces* it and holds the comparison
against the pre-M41 values, which are named constants below. If `close()` and
`params` ever disagree again, one of them drifted.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                "kinematics", "src"))

import tomcat_leg_detail as LD  # noqa: E402
from tomcat_kin import LegModel  # noqa: E402
from tomcat_kin.tendon import TendonMap  # noqa: E402
from tomcat_kin.torque_budget import evaluate as budget  # noqa: E402
from tomcat_kin.params import (  # noqa: E402
    DEFAULT_FORELEG, DEFAULT_HINDLEG, DEFAULT_SPINE, DEFAULT_TENDON, LoadCase,
)

#: GIM3505-9 peak joint-side torque, N.m `[sourced: motor-reality-check]`
MOTOR_PEAK = 1.95

#: Ø1.75 UHMWPE breaking strength, N `[owed: confirm against a real datasheet]`
CABLE_BREAK = 3000.0

#: LEG_TENDON_SPEC §2's own safety-factor target on the land transient.
SF_TARGET = 4.0

#: What `params.py` carried before M41 folded the manufacturing model in.
#: Kept so the comparison in `main()` and the tests survives the fold-in.
PRE_M41_HIND = (0.052, 0.033, 0.017, 0.008)
PRE_M41_FORE = (0.045, 0.029, 0.014, 0.007)
PRE_M41_BODY = 4.045


def measured_leg_mass(leg_params):
    """Per-link hardware mass (kg) for one leg, from the manufacturing model."""
    comps, report, _ = LD.build(leg_params)
    per_link, _ = LD.per_link_mass(comps, report)
    order = ("femur", "tibia", "meta", "paw")
    return np.array([per_link[k] for k in order]) * 1e-3


def loads_at(body_kg: float):
    """The three standard load cases rebuilt at a given body mass."""
    return [
        LoadCase("stand", body_mass_kg=body_kg, n_stance_legs=4,
                 dynamic_factor=1.0),
        LoadCase("trot", body_mass_kg=body_kg, n_stance_legs=2,
                 dynamic_factor=1.5),
        LoadCase("land", body_mass_kg=body_kg, n_stance_legs=1,
                 dynamic_factor=2.5),
    ]


def budget_at(body_kg: float, leg_params=DEFAULT_HINDLEG, grid: int = 21):
    leg, tm = LegModel(leg_params), TendonMap(DEFAULT_TENDON)
    out = {}
    for lc in loads_at(body_kg):
        r = budget(leg, tm, lc, grid=grid)
        out[lc.name] = {"tau": np.asarray(r.peak_joint_torque),
                        "T": np.asarray(r.peak_tension),
                        "motor": np.asarray(r.peak_motor_torque)}
    return out


def close(iters: int = 6, grid: int = 21):
    """Iterate body mass -> torque -> tension until the mass stops moving.

    The hardware mass is taken as **fixed** through the loop, which is the honest
    first pass: it is set by the moment arms (pinned by the motor peak, ADR-0041)
    and by ASSEMBLY_SPEC §2's bearing C0, neither of which scales smoothly with a
    5 % body-mass change. Where a check fails, that is reported rather than
    silently growing a part.
    """
    hind = measured_leg_mass(DEFAULT_HINDLEG)
    fore = measured_leg_mass(DEFAULT_FORELEG)
    trunk = float(DEFAULT_SPINE.trunk_mass)

    rows = []
    body = trunk + 2 * hind.sum() + 2 * fore.sum()
    for k in range(iters):
        b = budget_at(body, grid=grid)
        rows.append({"iter": k, "body": body, **b})
        # the hardware does not grow with body mass in this pass, so the spiral
        # closes in one step; the loop exists to make that visible rather than
        # assumed.
        new_body = trunk + 2 * hind.sum() + 2 * fore.sum()
        if abs(new_body - body) < 1e-6:
            break
        body = new_body
    return {"hind": hind, "fore": fore, "trunk": trunk, "rows": rows,
            "body": body}


def gate(b):
    """Every design limit the new load case has to clear."""
    land, trot = b["land"], b["trot"]
    out = []
    out.append(("motor peak (trot, the ACTUATOR case)",
                float(trot["motor"].max()), MOTOR_PEAK,
                float(trot["motor"].max()) <= MOTOR_PEAK))
    sf = CABLE_BREAK / float(land["T"].max())
    out.append(("cable SF on the land transient", sf, SF_TARGET, sf >= SF_TARGET))
    c0 = 2.0 * float(land["T"].max())
    out.append(("bearing static C0 needed (2xT)", c0, 1500.0, c0 <= 1500.0))
    return out


def swing_inertia(link_mass, leg_params):
    """Leg inertia about the hip in the stance pose — the P1 metric.

    ADR-0003 accepted the tendon-drive tension burden *to buy this*. Point masses
    at each link's CoM, which is what `mass.leg_com` uses.
    """
    lm = LegModel(leg_params)
    q = lm.inverse((0.04, -0.17, 0.0))
    pts = lm.joint_positions(q)
    L = np.asarray(leg_params.link_lengths)
    frac = np.asarray(leg_params.link_com_frac)
    I = 0.0
    for i in range(4):
        p0, p1 = pts[i], pts[i + 1]
        com = p0 + (p1 - p0) * frac[i]
        I += float(link_mass[i]) * float(com @ com)
    return I


def envelope_cost(inertia_ratio: float):
    """What the heavier leg costs the BALANCE envelope — the cross-subsystem term.

    `control.FOOT_ACCEL_LIMIT = 1051` m/s^2 carries the comment *"It is this high
    because tendon drive keeps the leg at 95 g."* The leg is 167 g, and it is
    redistributed distally, so the operational-space inertia the motor has to
    accelerate is larger and the foot cannot be flung as hard.

    `self_consistent_envelope` solves the envelope as a fixed point in which the
    actuation time is a term, so a lower acceleration limit shrinks it directly.

    ⚠️ `inertia_ratio` is used as a **first-order proxy** for the operational-space
    inertia ratio. The real quantity is `Lambda = (J M^-1 J^T)^-1` minimised over
    foot-acceleration directions, which needs the per-link inertia tensors this
    model does not carry. Treat the number as an order-of-magnitude, not a result.
    """
    from tomcat_kin import GaitController, control as ctl
    from tomcat_kin.gait import trot_params

    c = GaitController(params=trot_params())
    base = ctl.self_consistent_envelope(c)
    slow = ctl.self_consistent_envelope(
        c, accel_limit=ctl.FOOT_ACCEL_LIMIT / inertia_ratio)
    return base, slow


def fore_hind_split(fore, hind):
    """The fore/hind weight split, with the measured legs.

    Design review F2 concluded the assumed 60/40 front-heavy split was wrong and
    fixed it. The legs it fixed it with were 95 g fore / 110 g hind — an assumed
    asymmetry. Measured, both legs come out ~167 g, because the joint hardware
    dominates and it is the same hardware, so the asymmetry the model carries is
    mostly gone.
    """
    return {"fore_g": 1e3 * fore.sum(), "hind_g": 1e3 * hind.sum(),
            "params_fore_g": 1e3 * sum(PRE_M41_FORE),
            "params_hind_g": 1e3 * sum(PRE_M41_HIND)}


def coupled_tensions(tau_Nm, J_mm=None):
    """Solve `tau = J^T T` with the COUPLED map, against the diagonal model.

    `TendonMap.resolve` uses a diagonal map, so it reads `T_i = tau_i / r_i`. The
    routed hardware is lower-triangular (ADR-0042), which makes `J^T` upper
    triangular and the solve a back-substitution — the distal tendons load the
    proximal joints.

    ⚠️ **The off-diagonal SIGNS are a design choice, not a given.** They come from
    the wrap senses, which `leg_tendons.route` picks for minimum wrap. Flipping a
    sense flips whether the coupling helps or hurts, so this is a lever rather than
    only a penalty — `both` returns each case.
    """
    import leg_tendons as LT

    if J_mm is None:
        J_mm, _ = LT.coupling_matrix()
    tau = np.asarray(tau_Nm, float) * 1e3               # N.mm
    arms = np.asarray(LT.ARMS, float)

    def back_sub(J):
        A = J.T
        T = np.zeros(3)
        for i in (2, 1, 0):
            T[i] = (tau[i] - A[i, i + 1:] @ T[i + 1:]) / A[i, i]
        return T

    diag = tau / arms
    return {"diagonal": diag,
            "coupled": back_sub(J_mm),
            "sign_flipped": back_sub(J_mm * np.where(np.eye(3) > 0, 1.0, -1.0))}


def main():
    r = close()
    hind, fore, trunk = r["hind"], r["fore"], r["trunk"]
    old_body = PRE_M41_BODY

    print("M38 -- mass closure with the MEASURED leg hardware\n")
    print("%-22s %10s %10s %8s" % ("", "measured", "pre-M41", "ratio"))
    print("-" * 54)
    print("%-22s %9.1f g %9.1f g %7.2fx"
          % ("hind leg", 1e3 * hind.sum(), 1e3 * sum(PRE_M41_HIND),
             hind.sum() / sum(PRE_M41_HIND)))
    print("%-22s %9.1f g %9.1f g %7.2fx"
          % ("fore leg", 1e3 * fore.sum(), 1e3 * sum(PRE_M41_FORE),
             fore.sum() / sum(PRE_M41_FORE)))
    print("%-22s %9.1f g %9.1f g" % ("trunk (incl. motors)", 1e3 * trunk,
                                     1e3 * trunk))
    print("%-22s %8.3f kg %8.3f kg %7.2fx"
          % ("BODY", r["body"], old_body, r["body"] / old_body))
    print("   NFR5 target 4.05 kg -> %s by %.1f %%"
          % ("EXCEEDED" if r["body"] > 4.05 else "met",
             100 * (r["body"] / 4.05 - 1.0)))

    print("\nload cases at the new body mass:")
    b = r["rows"][-1]
    print("%-8s %26s %26s" % ("case", "joint torque N.m", "cable tension N"))
    print("-" * 64)
    for case in ("stand", "trot", "land"):
        print("%-8s %26s %26s"
              % (case, np.round(b[case]["tau"], 2), np.round(b[case]["T"], 1)))

    print("\ndesign gates at the new mass:")
    for name, got, limit, ok in gate(b):
        print("   %-4s %-38s %9.2f vs %8.2f"
              % ("PASS" if ok else "FAIL", name, got, limit))

    print("\nP1 check -- leg swing inertia about the hip (kg.m^2):")
    i_old = swing_inertia(PRE_M41_HIND, DEFAULT_HINDLEG)
    i_new = swing_inertia(hind, DEFAULT_HINDLEG)
    print("   pre-M41 %.6f" % i_old)
    print("   measured %.6f   (%+.1f %%)" % (i_new, 100 * (i_new / i_old - 1.0)))
    share_old = np.asarray(PRE_M41_HIND) / sum(PRE_M41_HIND)
    share_new = hind / hind.sum()
    print("   mass share proximal->distal")
    print("     pre-M41  %s" % np.round(100 * share_old, 1))
    print("     measured %s" % np.round(100 * share_new, 1))

    ratio = i_new / i_old
    base, slow = envelope_cost(ratio)
    print("")
    print("what that costs the BALANCE envelope (first-order proxy):")
    print("   accel limit    1051 -> %.0f m/s^2" % (1051.0 / ratio))
    print("   envelope     %.1f -> %.1f mm   (%+.1f %%)"
          % (1e3 * base["envelope"], 1e3 * slow["envelope"],
             100 * (slow["envelope"] / base["envelope"] - 1.0)))
    print("   actuation    %.1f -> %.1f ms of the loop"
          % (1e3 * base["actuation"], 1e3 * slow["actuation"]))
    print("   NFR15 needs 48 mm: %s"
          % ("still met" if slow["envelope"] > 0.048 else "NO LONGER MET"))

    print("")
    print("the COUPLED tendon map (ADR-0042) against the diagonal model,")
    print("solving tau = J^T T at the new land case:")
    ct = coupled_tensions(b["land"]["tau"])
    print("%-14s %10s %10s %10s %9s" % ("tendon", "diagonal", "coupled",
                                        "sign-flip", "delta"))
    print("-" * 58)
    for i, nm in enumerate(("hip", "knee", "ankle")):
        d, c, f = ct["diagonal"][i], ct["coupled"][i], ct["sign_flipped"][i]
        print("%-14s %10.1f %10.1f %10.1f %8.1f %%"
              % (nm, d, c, f, 100 * (c / d - 1.0)))
    worst = max(ct["coupled"])
    print("   worst coupled tension %.0f N -> cable SF %.2f (target %.1f)"
          % (worst, CABLE_BREAK / worst, SF_TARGET))
    print("   best sign choice worst %.0f N -> SF %.2f"
          % (max(ct["sign_flipped"]), CABLE_BREAK / max(ct["sign_flipped"])))

    sp = fore_hind_split(fore, hind)
    print("")
    print("fore/hind leg asymmetry:")
    print("   params   fore %.0f g / hind %.0f g  (%.2fx)"
          % (sp["params_fore_g"], sp["params_hind_g"],
             sp["params_hind_g"] / sp["params_fore_g"]))
    print("   measured fore %.0f g / hind %.0f g  (%.2fx)"
          % (sp["fore_g"], sp["hind_g"], sp["hind_g"] / sp["fore_g"]))


if __name__ == "__main__":
    main()
