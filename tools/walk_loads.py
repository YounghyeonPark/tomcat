# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The learned walk as a LOAD CASE (M140, ADR-0134).

    python tools/walk_loads.py runs/walk/policy.zip

Every motor-sizing and runtime number in this project came from the analytic
trot (`power.gait_power`: a planned gait, quasi-static stance forces, a swing
model). This runs the trained policy on the CPU simulator and measures what
the motors would actually be asked for, the same way `power` prices it:

- cable tension from the series spring's deflection (`-k theta_spool / r`);
- times the conduit friction at the pose (`TendonMap.capstan_factor(q=...)`,
  each leg its own drive) -- the simulator itself has no friction;
- motor torque = tension x spool radius, motor speed = the rotor's own;
- copper `1.5 (tau / Kt)^2 R_pp`, mechanical `|tau omega|`, no regeneration.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "kinematics", "src"))


def run(policy, v, seed, randomize, skip_s=0.5):
    from tomcat_kin import TendonMap
    from tomcat_kin import mjcf_tendon as MT
    from tomcat_kin.env import LEGS
    from tomcat_kin.gym_env import TomcatWalk
    from tomcat_kin.params import tendon_for

    env = TomcatWalk(v_cmd=v, randomize=randomize, seed=seed)
    o, _ = env.reset()
    c = env.core
    import mujoco
    jr_dof = [int(c.model.jnt_dofadr[c._id(mujoco.mjtObj.mjOBJ_JOINT, "jr_" + p)])
              for p in c.pairs]
    maps = {nm: TendonMap(tendon_for(nm)) for nm in LEGS}
    rows = []
    n = 0
    fell = False
    while True:
        a = policy.predict(o, deterministic=True)[0]
        o, _r, te, tr, _info = env.step(a)
        n += 1
        if n / c.control_hz >= skip_s:
            th_s = np.array([c.data.qpos[i] for i in c._js])
            T = -c.k_tors * th_s / MT.SPOOL_R                  # cable, joint side
            q = {nm: np.array([c.data.qpos[i] for i in c._qadr(nm)]) for nm in LEGS}
            cap = np.ones(18)
            for k, nm in enumerate(LEGS):
                tm = maps[nm]
                sl = slice(3 * k, 3 * k + 3)
                cap[sl] = np.where(T[sl] >= 0.0,
                                   tm.capstan_factor(side=+1, q=q[nm]),
                                   tm.capstan_factor(side=-1, q=q[nm]))
            w = np.array([c.data.qvel[d] for d in jr_dof])     # rotor = output shaft
            rows.append(np.concatenate([np.abs(T), np.abs(T) * cap, w]))
        if te or tr:
            fell = bool(te)
            break
    return np.array(rows), fell


def summarise(rows, kt):
    from tomcat_kin import mjcf_tendon as MT
    from tomcat_kin import power as PW

    T, Tm, w = rows[:, :18], rows[:, 18:36], np.abs(rows[:, 36:])
    tau = Tm * MT.SPOOL_R                                     # motor torque, N.m
    cu = PW.PHASE_FACTOR * (tau / kt) ** 2 * PW.R_PHASE_PHASE
    mech = tau * w
    legs, spine = slice(0, 12), slice(12, 18)
    rpm = w * 60.0 / (2.0 * np.pi)
    # the no-load speed a motor of the proxy's peak torque would need for
    # every sample to sit under its torque-speed line
    with np.errstate(divide="ignore", invalid="ignore"):
        need = np.where(tau < PW.PEAK_NM, rpm / (1.0 - tau / PW.PEAK_NM), np.inf)
    by_joint = {}
    for j, name in enumerate(("hip", "knee", "ankle")):
        cols = [j, j + 3, j + 6, j + 9]
        by_joint[name] = {
            "peak_nm": float(tau[:, cols].max()),
            "rms_nm": float(np.sqrt((tau[:, cols] ** 2).mean(0)).max()),
            "peak_rpm": float(rpm[:, cols].max()),
            "need_rpm_p99": float(np.percentile(need[:, cols], 99)),
            "peak_cable_n": float(T[:, cols].max()),
            "rms_cable_n": float(np.sqrt((T[:, cols] ** 2).mean(0)).max()),
        }
    return {
        "legs_copper_w": float(cu[:, legs].sum(1).mean()),
        "legs_mech_w": float(mech[:, legs].sum(1).mean()),
        "spine_copper_w": float(cu[:, spine].sum(1).mean()),
        "spine_mech_w": float(mech[:, spine].sum(1).mean()),
        "spine_peak_nm": float(tau[:, spine].max()),
        "by_joint": by_joint,
        "saturated_frac": float(np.mean(T >= MT.TENSION_MAX * 0.999)),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("policy")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--dr", action="store_true")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    from stable_baselines3 import PPO

    from tomcat_kin import power as PW
    from tomcat_kin.env import TomcatEnv
    import motor_spec_review as MR

    pol = PPO.load(a.policy, device="cpu")
    dr = {**TomcatEnv.RANGES, **TomcatEnv.SENSOR_RANGES} if a.dr else None
    out = {}
    for v in (0.25, 0.5):
        rows, falls = [], 0
        for s in range(a.seeds):
            r, fell = run(pol, v, 40_000 + s, dr)
            rows.append(r)
            falls += int(fell)
        rows = np.concatenate(rows)
        res = {}
        for label, kt in (("surveyed_kt", PW.KT), ("vendor_kt", MR.SPEC["vendor_kt"])):
            s_ = summarise(rows, kt)
            total = s_["legs_copper_w"] + s_["legs_mech_w"] + PW.ELECTRONICS_W
            s_["runtime_min_legs"] = 60.0 * PW.battery_wh() / total
            s_["runtime_min_with_spine"] = 60.0 * PW.battery_wh() / (
                total + s_["spine_copper_w"] + s_["spine_mech_w"])
            res[label] = s_
        res["falls"] = falls
        res["samples"] = len(rows)
        out[str(v)] = res
        print(json.dumps({v: res}, indent=1), flush=True)
    if a.out:
        with open(a.out, "w") as f:
            json.dump(out, f, indent=1)


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(__file__))
    main()
