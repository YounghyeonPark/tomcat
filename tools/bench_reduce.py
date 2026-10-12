# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Reduce bench data to the numbers the model carries (M152).

    python tools/bench_reduce.py kt      stall.csv [--backemf spin.csv]
    python tools/bench_reduce.py kv      noload.csv
    python tools/bench_reduce.py heat    heating.csv
    python tools/bench_reduce.py capstan conduit.csv
    python tools/bench_reduce.py flexure g3.csv
    python tools/bench_reduce.py pad     pad.csv
    python tools/bench_reduce.py template <kind>     # prints the CSV header

Each command fits its measurement, then says which constants in the repo the
result replaces and what `docs/notes/bench-plan.md`'s decision rule makes of
it. Nothing is written to the repo: the numbers go in by hand, so that the
suite says exactly which assumptions they move.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys

import numpy as np

#: CSV columns per measurement (`template` prints them).
COLUMNS = {
    "kt": ["current_A", "torque_Nm"],
    "backemf": ["speed_rpm", "vll_rms_V"],
    "kv": ["volts_V", "speed_rpm"],
    "heat": ["time_s", "temp_C"],
    "capstan": ["bend_deg", "t_in_N", "t_out_N"],
    "flexure": ["load_N", "deflection_mm", "direction"],
    "pad": ["floor", "kind", "normal_N", "pull_N"],
}

#: The model's current assumptions, for the comparison each command prints.
ASSUMED = {
    "kt_surveyed": 0.44, "kt_vendor": 0.35, "rated_A": 1.60, "peak_A": 4.19,
    "ankle_rms_nm": (0.60, 0.72), "kv_rpm_per_V": 15.83, "pack_floor_V": 28.0,
    "conduit_mu": 0.07, "series_k_leg": 1.25e5, "pad_mu_nfr15": 0.6,
}


def read(path, kind):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    need = COLUMNS[kind]
    missing = [c for c in need if c not in (rows[0] if rows else {})]
    if missing:
        raise SystemExit(f"{path}: missing columns {missing}; expected {need}")
    return rows


def col(rows, name):
    return np.array([float(r[name]) for r in rows])


# -------------------------------------------------------------------- motor
def fit_kt(current, torque):
    """Kt through the origin (a motor makes no torque at no current), and
    the intercept fit as a check on friction / zero error."""
    current, torque = np.asarray(current, float), np.asarray(torque, float)
    kt0 = float(current @ torque / (current @ current))
    slope, icept = np.polyfit(current, torque, 1)
    resid = torque - kt0 * current
    r2 = 1.0 - float(resid @ resid) / float(((torque - torque.mean()) ** 2).sum())
    return {"kt_origin": kt0, "kt_slope": float(slope), "intercept_nm": float(icept), "r2": r2}


def kt_from_backemf(speed_rpm, vll_rms):
    """A sinusoidal PMSM's torque constant from its line-to-line RMS back-EMF
    at the OUTPUT shaft: Ke = V_ll,rms per rad/s. Then Kt per PEAK phase amp is
    sqrt(3/2) Ke, and per RMS phase amp sqrt(3) Ke. Which one the driver's
    "current" is decides which to compare with the stall fit (ADR-0044)."""
    w = np.asarray(speed_rpm, float) * 2.0 * math.pi / 60.0
    ke = float(w @ np.asarray(vll_rms, float) / (w @ w))
    return {"ke_vll_rms_per_rad_s": ke, "kt_per_peak_A": math.sqrt(1.5) * ke,
            "kt_per_rms_A": math.sqrt(3.0) * ke}


def verdict_kt(kt):
    lo, hi = ASSUMED["ankle_rms_nm"]
    amps = (lo / kt, hi / kt)
    if kt >= 0.44:
        v = "stock part passes thermally: buy 19"
    elif kt >= 0.38:
        v = "marginal: needs forced air (h >= 25) and a measured heating curve"
    else:
        v = "over rating: a stronger frame, or larger ankle moment arms"
    return {"ankle_rms_A": amps, "peak_nm_at_driver_limit": ASSUMED["peak_A"] * kt, "verdict": v}


def fit_kv(volts, speed):
    volts, speed = np.asarray(volts, float), np.asarray(speed, float)
    kv = float(volts @ speed / (volts @ volts))
    return {"kv_rpm_per_V": kv, "noload_rpm_at_8S_floor": kv * ASSUMED["pack_floor_V"]}


def fit_heat(t, temp):
    """First-order rise T(t) = T0 + dT (1 - exp(-t / tau)), fit by scanning
    tau and solving dT linearly at each; returns the settled rise."""
    t, temp = np.asarray(t, float), np.asarray(temp, float)
    t0 = float(temp[0])
    best = None
    for tau in np.geomspace(5.0, 2.0e4, 400):
        g = 1.0 - np.exp(-t / tau)
        dT = float(g @ (temp - t0) / (g @ g))
        err = float(((t0 + dT * g - temp) ** 2).sum())
        if best is None or err < best[0]:
            best = (err, tau, dT)
    _e, tau, dT = best
    return {"ambient_C": t0, "rise_C": dT, "settled_C": t0 + dT, "tau_s": float(tau),
            "verdict": "inside NFR18's 80 C" if t0 + dT <= 80.0 else "over 80 C"}


# ---------------------------------------------------------------- conduit
def fit_capstan(bend_deg, t_in, t_out):
    """ln(T_in / T_out) = mu theta + c, over the bends: mu is the slope per
    radian and c the ends' loss (the 0-degree case), `pulley_efficiency`'s
    analogue as exp(-c)."""
    th = np.radians(np.asarray(bend_deg, float))
    y = np.log(np.asarray(t_in, float) / np.asarray(t_out, float))
    mu, c = np.polyfit(th, y, 1)
    mu = float(mu)
    if mu <= 0.07:
        v = "keeps the computed runtime margin"
    elif mu <= 0.09:
        v = "re-run walk_loads; consider a lower-friction liner"
    else:
        v = "re-route the ankle conduit (its knee crossing is the largest bend)"
    return {"conduit_mu": mu, "end_loss": float(c), "end_efficiency": math.exp(-float(c)),
            "ankle_factor_at_310deg": math.exp(mu * math.radians(310.0)), "verdict": v}


# --------------------------------------------------------------- flexure
def fit_flexure(load, defl_mm, direction=None):
    load = np.asarray(load, float)
    x = np.asarray(defl_mm, float) / 1000.0
    k = float(x @ load / (x @ x))
    out = {"k_N_per_m": k, "vs_spec": k / ASSUMED["series_k_leg"]}
    if direction is not None:
        d = np.asarray(direction)
        up, dn = d == "load", d == "unload"
        if up.any() and dn.any():
            gap = np.interp(x[dn], np.sort(x[up]), load[up][np.argsort(x[up])]) - load[dn]
            out["hysteresis_N"] = float(np.max(np.abs(gap)))
    return out


# ------------------------------------------------------------------- pad
def fit_pad(rows):
    out = {}
    for r in rows:
        key = (r["floor"], r["kind"])
        out.setdefault(key, []).append(float(r["pull_N"]) / float(r["normal_N"]))
    res = {f"{f}/{k}": float(np.mean(v)) for (f, k), v in out.items()}
    kin = [v for (f, k), vs in out.items() if k == "kinetic" for v in vs]
    res["verdict"] = ("NFR15 holds on every floor" if kin and min(kin) >= ASSUMED["pad_mu_nfr15"]
                      else "a floor under mu 0.6: revisit NFR15, weight the slip penalty")
    return res


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("kind", choices=("kt", "kv", "heat", "capstan", "flexure", "pad", "template"))
    ap.add_argument("path", nargs="?")
    ap.add_argument("--backemf", default=None)
    a = ap.parse_args(argv)
    if a.kind == "template":
        print(",".join(COLUMNS[a.path]))
        return
    rows = read(a.path, a.kind)
    if a.kind == "kt":
        r = fit_kt(col(rows, "current_A"), col(rows, "torque_Nm"))
        r.update(verdict_kt(r["kt_origin"]))
        if a.backemf:
            b = read(a.backemf, "backemf")
            r["backemf"] = kt_from_backemf(col(b, "speed_rpm"), col(b, "vll_rms_V"))
        r["updates"] = ["power.KT", "motor_spec_review.SPEC['vendor_kt']",
                        "mjcf_tendon.MOTOR_PEAK_NM (= peak_A x Kt)", "TomcatEnv.MOTOR_RANGES"]
    elif a.kind == "kv":
        r = fit_kv(col(rows, "volts_V"), col(rows, "speed_rpm"))
        r["updates"] = ["mjcf_tendon.MOTOR_NOLOAD_RAD_S (= Kv x pack floor)",
                        "motor_spec_review.SPEC['speed_rpm']"]
    elif a.kind == "heat":
        r = fit_heat(col(rows, "time_s"), col(rows, "temp_C"))
        r["updates"] = ["thermal/src/lib.rs (h, the part's rise)"]
    elif a.kind == "capstan":
        r = fit_capstan(col(rows, "bend_deg"), col(rows, "t_in_N"), col(rows, "t_out_N"))
        r["updates"] = ["params.TendonParams.conduit_mu", "params.TendonParams.pulley_efficiency"]
    elif a.kind == "flexure":
        r = fit_flexure(col(rows, "load_N"), col(rows, "deflection_mm"),
                        [row["direction"] for row in rows])
        r["updates"] = ["params.TendonParams.series_k", "TomcatEnv.RANGES['series_k_scale']"]
    else:
        r = fit_pad(rows)
        r["updates"] = ["TomcatEnv.RANGES['floor_mu']", "mjcf_tendon pad friction (0.8)"]
    for k, v in r.items():
        print(f"{k}: {v}")


if __name__ == "__main__":
    sys.exit(main())
