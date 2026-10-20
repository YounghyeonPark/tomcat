# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""ADR-0151's slim knee and ankle, priced through the robot's own budgets (M160).

    python tools/slim_drive.py --g 2.0 --stage-g 25 --eta 0.95

`tools/sheave_trade.py` sized the option: move a torque factor `g` of the knee
and ankle reduction from the joint sheave to a gear stage at the motor. This
prices it with the same machinery that sized the robot, by re-running it on
patched parameters in a fresh process:

- each leg rebuilt in CAD with the knee and ankle sheaves at g^(-2/3)
  (`tomcat_leg_detail`), and its links re-weighed (`per_link_mass`); the
  difference goes onto `LegParams.link_mass`;
- 8 gear stages (`--stage-g` each, `[assumed]`: no part is chosen) onto the two
  girdles that carry the knee and ankle motors;
- the motor requirement (`mass_closure.motor_requirement`), the trot's runtime
  (`power.runtime`) and the body mass, at the new masses.

What stays by construction: the torque ratio from motor to joint, so the motor
torque moves only with mass. What the stage costs besides its mass: at
efficiency `eta` its ratio is g / eta, so the knee and ankle motors turn
1 / eta faster and lose (1 / eta - 1) of their mechanical power in it. That
loss is bounded here by charging it on every leg motor.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CAD = os.path.join(ROOT, "mechanical", "cad")
sys.path.insert(0, os.path.join(ROOT, "kinematics", "src"))
sys.path.insert(0, CAD)

LINKS = ("femur", "tibia", "meta", "paw")


def _scaled_tendon(P, g):
    """Knee and ankle arms at g^(-2/3); the hip is the hollow hip's hub."""
    a = np.asarray(P.DEFAULT_TENDON.joint_moment_arm, float)
    floor = P.DEFAULT_TENDON.motor_spool_radius * g ** (1.0 / 3.0)
    arms = (a[0],) + tuple(np.maximum(a[1:] * g ** (-2.0 / 3.0), floor))
    return arms


def cad_masses(g):
    """`{role: per-link hardware kg}` for both legs at `g`. Run in its own
    process: the CAD modules read `DEFAULT_TENDON` when they import."""
    from tomcat_kin import params as P
    if g != 1.0:
        arms = _scaled_tendon(P, g)
        P.DEFAULT_TENDON = dataclasses.replace(P.DEFAULT_TENDON, joint_moment_arm=arms)
        P.DEFAULT_TENDON_FORE = dataclasses.replace(P.DEFAULT_TENDON_FORE, joint_moment_arm=arms)
    import tomcat_leg_detail as LD
    LD.DEFAULT_TENDON = P.DEFAULT_TENDON
    out = {}
    for role, leg in (("hind", P.DEFAULT_HINDLEG), ("fore", P.DEFAULT_FORELEG)):
        comps, report, _ = LD.build(leg, role=role)
        per, _ = LD.per_link_mass(comps, report)
        out[role] = [per[k] * 1e-3 for k in LINKS]
    return out


def _subprocess(args):
    r = subprocess.run([sys.executable, __file__] + args, capture_output=True, text=True,
                       cwd=CAD, check=True)
    return json.loads(r.stdout.strip().splitlines()[-1])


def price(delta, stage_kg, kt=None):
    """The budgets at patched masses. `delta`: `{role: per-link kg}` added to
    `link_mass`; `stage_kg` added to each girdle four times.

    ⚠️ The patch has to land BEFORE the package imports: `spine.py` and others
    copy `DEFAULT_*` into class defaults when they load, so a patch made after
    `import tomcat_kin` reached the torque budget and not the gait (the first
    run priced the runtime at today's mass). `params` is loaded on its own
    first, patched, and registered, and the package then imports that copy."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "tomcat_kin.params", os.path.join(ROOT, "kinematics", "src", "tomcat_kin", "params.py"))
    P = importlib.util.module_from_spec(spec)
    sys.modules["tomcat_kin.params"] = P
    spec.loader.exec_module(P)
    hind = dataclasses.replace(P.DEFAULT_HINDLEG, link_mass=tuple(
        np.asarray(P.DEFAULT_HINDLEG.link_mass) + delta["hind"]))
    fore = dataclasses.replace(P.DEFAULT_FORELEG, link_mass=tuple(
        np.asarray(P.DEFAULT_FORELEG.link_mass) + delta["fore"]))
    spine = dataclasses.replace(P.DEFAULT_SPINE,
                                front_girdle_mass=P.DEFAULT_SPINE.front_girdle_mass + 4 * stage_kg,
                                rear_girdle_mass=P.DEFAULT_SPINE.rear_girdle_mass + 4 * stage_kg)
    P.DEFAULT_HINDLEG = P.DEFAULT_LEG = hind
    P.DEFAULT_FORELEG = fore
    P.DEFAULT_SPINE = spine
    P.DEFAULT_BODY_MASS_KG = 2 * fore.mass + 2 * hind.mass + spine.trunk_mass

    import mass_closure as MC
    from tomcat_kin import gait, power
    if kt is not None:
        power.KT = kt                            # e.g. the vendor's 0.35 (ADR-0136)
    MC.DEFAULT_HINDLEG, MC.DEFAULT_FORELEG, MC.DEFAULT_SPINE = hind, fore, spine
    body = P.DEFAULT_BODY_MASS_KG
    b = {"body": body, **MC.budget_at(body, leg_params=hind)}
    req = MC.motor_requirement(b)
    ctl = gait.GaitController(gait.trot_params())
    g = power.gait_power(ctl)
    rt = power.runtime(ctl)
    return {"body_kg": body, "peak_nm": req["peak"], "continuous_nm": req["continuous"],
            "binding_leg": req["binding_leg"], "need_rpm": req["need_rpm"],
            "need_rpm_per_joint": np.asarray(req["need_rpm_per_joint"]).tolist(),
            "trot_minutes": rt["trot_minutes"], "mechanical_w": g["mechanical_w"],
            "total_w": g["total_w"]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--g", type=float, default=2.0)
    ap.add_argument("--stage-g", type=float, nargs="+", default=[15.0, 25.0, 40.0],
                    help="mass of one gear stage, g [assumed]")
    ap.add_argument("--eta", type=float, default=0.95, help="its efficiency [assumed]")
    ap.add_argument("--kt", type=float, default=None,
                    help="torque constant for the runtime, N.m/A (vendor: 0.35)")
    ap.add_argument("--cad", type=float, default=None, help=argparse.SUPPRESS)
    ap.add_argument("--price", default=None, help=argparse.SUPPRESS)
    a = ap.parse_args()
    if a.cad is not None:                        # child: CAD masses at one g
        print(json.dumps(cad_masses(a.cad)))
        return
    if a.price is not None:                      # child: budgets at one patch
        d = json.loads(a.price)
        print(json.dumps(price({k: np.asarray(v) for k, v in d["delta"].items()}, d["stage"],
                               d.get("kt"))))
        return

    base = _subprocess(["--cad", "1.0"])
    slim = _subprocess(["--cad", str(a.g)])
    delta = {r: (np.asarray(slim[r]) - np.asarray(base[r])).tolist() for r in base}
    print("leg hardware change per link (g), femur/tibia/meta/paw:")
    for r in ("hind", "fore"):
        print("  %-4s %s  total %+.1f g" % (r, np.round(1e3 * np.asarray(delta[r]), 1),
                                           1e3 * sum(delta[r])))
    today = _subprocess(["--price", json.dumps({"delta": {"hind": [0] * 4, "fore": [0] * 4},
                                                "stage": 0.0, "kt": a.kt})])
    rows = [("today", today)]
    for m in a.stage_g:
        rows.append(("g %.1f, %2.0f g stage" % (a.g, m),
                     _subprocess(["--price", json.dumps({"delta": delta, "stage": m * 1e-3,
                                                         "kt": a.kt})])))
    loss = 1.0 / a.eta - 1.0
    print("\n%-20s %8s %8s %8s %9s %9s" % ("", "body kg", "peak", "cont.", "need rpm", "trot min"))
    for name, r in rows:
        stage = name != "today"
        rpm = r["need_rpm"] / (a.eta if stage else 1.0)
        # the stage's loss, charged on every leg motor's mechanical power (a bound)
        w = r["total_w"] + (loss * r["mechanical_w"] if stage else 0.0)
        minutes = r["trot_minutes"] * r["total_w"] / w
        print("%-20s %8.3f %8.3f %8.3f %9.0f %9.2f" % (name, r["body_kg"], r["peak_nm"],
                                                       r["continuous_nm"], rpm, minutes))


if __name__ == "__main__":
    main()
