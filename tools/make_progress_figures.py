"""Generate the README progress figures from the LIVE model.

Every number in these figures is computed from ``tomcat_kin`` at render time, so a
figure can never drift from the code the way a pasted number can. Run:

    python tools/make_progress_figures.py

Palette and mark conventions follow the project's data-viz rules: one axis per
panel (never a dual-axis chart -- differing units become small multiples), thin
marks, recessive grid, direct labels on every series, and a light surface that
stays legible in both GitHub themes.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "kinematics", "src"))

from tomcat_kin import GaitController, GaitParams          # noqa: E402
from tomcat_kin.gait import trot_params                     # noqa: E402
from tomcat_kin import dynamics as dyn, control as ctl      # noqa: E402

OUT = os.path.join(ROOT, "docs", "figures")

# --- validated palette (see the project's data-viz reference) ----------------
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#dedddb"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
BAD = "#e34948"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "text.color": INK, "axes.labelcolor": INK2, "axes.edgecolor": GRID,
    "xtick.color": INK2, "ytick.color": INK2,
    "font.size": 10, "axes.titlesize": 12, "axes.titleweight": "bold",
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False,
    "lines.linewidth": 2.0,
})


def _save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("  wrote", os.path.relpath(path, ROOT))


# ---------------------------------------------------------------- figure 1
def fig_speed():
    """Headline: what the two gaits actually achieve."""
    crawl = GaitController()
    trot = GaitController(params=trot_params())
    fast = GaitController(params=trot_params(period=0.25, stride_length=0.12))
    names = ["Static crawl\n(M6)", "Trot\n(M7)", "Trot, sustained max\n(M7)"]
    vals = [crawl.params.body_speed * 100, trot.params.body_speed * 100,
            fast.params.body_speed * 100]
    notes = ["limited by TIPPING", "60x the crawl", "RMS motor torque at its\ncontinuous rating"]

    fig, ax = plt.subplots(figsize=(8, 3.2))
    y = np.arange(len(vals))[::-1]
    ax.barh(y, vals, height=0.55, color=[ORANGE, BLUE, BLUE], zorder=3)
    for yi, v, n in zip(y, vals, notes):
        ax.text(v + 2, yi + 0.04, f"{v:.1f} cm/s", va="bottom", ha="left",
                color=INK, fontweight="bold")
        ax.text(v + 2, yi - 0.06, n, va="top", ha="left", color=INK2, fontsize=8)
    ax.set_yticks(y, names)
    ax.set_xlabel("body speed (cm/s)")
    ax.set_xlim(0, max(vals) * 1.55)
    ax.set_title("Locomotion capability")
    ax.grid(axis="y", visible=False)
    _save(fig, "01_speed.png")


# ---------------------------------------------------------------- figure 2
def fig_tipping_vs_slipping():
    """The M6 finding: tipping binds, slipping never does.

    Two panels, NOT a dual axis -- the units differ, so they get their own axes.
    """
    periods = np.arange(1.2, 6.01, 0.4)
    zmp, mu = [], []
    for T in periods:
        c = GaitController(params=GaitParams(period=float(T)))
        r = dyn.sweep(c, 72)
        zmp.append(r["zmp_margin_min"] * 1e3)
        mu.append(r["aggregate_mu"])

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5.0), sharex=True,
                                 gridspec_kw={"hspace": 0.18})
    a1.axhline(0, color=INK2, lw=1.0, zorder=2)
    a1.plot(periods, zmp, color=ORANGE, zorder=3)
    a1.fill_between(periods, zmp, 0, where=np.array(zmp) < 0,
                    color=ORANGE, alpha=0.12, zorder=1)
    a1.set_ylabel("ZMP margin (mm)")
    a1.set_title("What limits the statically stable crawl: tipping, not friction")
    a1.annotate("TIPS below zero", xy=(4.3, min(zmp) * 0.72), color=ORANGE,
                fontsize=9, fontweight="bold")

    a2.axhline(0.8, color=BAD, lw=1.2, ls="--", zorder=2)
    a2.plot(periods, mu, color=AQUA, zorder=3)
    a2.set_ylabel("friction demand  $\\mu$")
    a2.set_xlabel("gait period (s)")
    a2.set_ylim(0, 1.0)
    a2.annotate("floor can supply $\\mu\\approx0.8-1.2$", xy=(3.6, 0.84),
                color=BAD, fontsize=9)
    a2.annotate("demand never reaches it", xy=(1.35, 0.55), color=AQUA,
                fontsize=9, fontweight="bold")
    _save(fig, "02_tipping_vs_slipping.png")


# ---------------------------------------------------------------- figure 3
def fig_trot_placement():
    """The M7 finding: trot foot placement is a balance condition."""
    xs = np.arange(0.0, 0.052, 0.004)
    drift = []
    for xn in xs:
        c = GaitController(params=trot_params(nominal_foot=(float(xn), -0.17)))
        n = 64
        cyc = dyn.cycle(c, n)
        sg = []
        for i in range(n):
            b = dyn.line_balance(c, i / n, n, cyc=cyc)
            sg.append(np.sign(b.offset) * b.unbalanced_moment if b else 0.0)
        h = float(np.mean(cyc.com[:, 2] - cyc.ground_z))
        I = c.body.total_mass * h * h
        drift.append(float(np.sum(np.array(sg) / I) * (c.params.period / n)))

    fig, ax = plt.subplots(figsize=(8, 3.4))
    ax.axhline(0, color=INK2, lw=1.0, zorder=2)
    ax.plot(xs * 1e3, drift, color=BLUE, zorder=3)
    ax.plot([5.0], [np.interp(5.0, xs * 1e3, drift)], "o", ms=9, color=BLUE, zorder=4)
    ax.annotate("trot_params()  +5 mm\nroll BOUNDED",
                xy=(5.0, -0.06), xytext=(9, -1.30), color=BLUE, fontsize=9,
                fontweight="bold", ha="left",
                arrowprops=dict(arrowstyle="-", color=GRID, lw=1.0))
    ax.annotate("the CRAWL's placement  +50 mm\nFALLS in one stride",
                xy=(50, drift[-1]), xytext=(24, -0.55), color=ORANGE,
                fontsize=9, fontweight="bold", ha="left",
                arrowprops=dict(arrowstyle="-", color=GRID, lw=1.0))
    ax.plot([50], [drift[-1]], "o", ms=9, color=ORANGE, zorder=4)
    ax.set_xlim(-3, 58)
    ax.set_xlabel("nominal foothold, forward of the hip (mm)")
    ax.set_ylabel("roll rate gained\nper cycle (rad/s)")
    ax.set_title("Trot foot placement is a balance condition, not a preference")
    _save(fig, "03_trot_placement.png")


# ---------------------------------------------------------------- figure 4
def fig_closed_loop():
    """M8-M10: the trot is unstable open loop and recoverable closed loop."""
    P = ctl.StepPlant.from_gait(GaitController(params=trot_params()), 96)
    steps = 7
    d = 0.02
    op = ctl.simulate(P, steps, xi0=d, closed_loop=False)
    cl = ctl.simulate(P, steps, xi0=d, beta=0.0, use_spine=True)
    cap = [d]
    xi = d
    for _ in range(steps):
        xi = P.propagate(xi, ctl.capture_placement(P, xi))
        cap.append(xi)

    fig, ax = plt.subplots(figsize=(8, 3.6))
    k = np.arange(steps + 1)
    ax.semilogy(k, np.abs(op) * 1e3, color=ORANGE, zorder=3)
    ax.semilogy(k, np.abs(cap) * 1e3, color=AQUA, zorder=3)
    ax.semilogy(k, np.clip(np.abs(cl), 1e-4, None) * 1e3, color=BLUE,
                marker="o", ms=6, zorder=4)
    ax.text(steps - 0.15, abs(op[-1]) * 1e3 * 0.30, "open loop - falls",
            color=ORANGE, ha="right", va="top", fontsize=9, fontweight="bold")
    ax.text(steps - 0.1, abs(cap[-1]) * 1e3 * 1.5, "capture only - stops falling,\nstays displaced",
            color=AQUA, ha="right", va="bottom", fontsize=9, fontweight="bold")
    ax.text(1.15, 0.35, "closed loop - recovers\nin one step", color=BLUE,
            ha="left", fontsize=9, fontweight="bold")
    ax.set_xlabel("step")
    ax.set_ylabel("|DCM| error (mm, log)")
    ax.set_title("Closed-loop balance: response to a 20 mm disturbance")
    ax.set_ylim(0.05, max(np.abs(op)) * 1e3 * 3)
    _save(fig, "04_closed_loop.png")


# ---------------------------------------------------------------- figure 5
def fig_authority():
    """M9-M10: where the disturbance-rejection authority comes from."""
    P = ctl.StepPlant.from_gait(GaitController(params=trot_params()), 96)
    feet = ctl.rejection_envelope(P, use_spine=False) * 1e3
    both = ctl.rejection_envelope(P, use_spine=True) * 1e3

    fig, ax = plt.subplots(figsize=(8, 3.1))
    y = [1, 0]
    ax.barh(y, [feet, both], height=0.5, color=[ORANGE, BLUE], zorder=3)
    ax.text(feet + 1.5, 1.05, f"{feet:.0f} mm", va="bottom", color=INK, fontweight="bold")
    ax.text(feet + 1.5, 0.98, "sagittal legs reach the perpendicular\nonly through a 0.44 projection",
            va="top", color=INK2, fontsize=8)
    ax.text(both + 1.5, 0.05, f"{both:.0f} mm", va="bottom", color=INK, fontweight="bold")
    ax.text(both + 1.5, -0.02, f"rejects a {both / 1e3 * P.omega:.2f} m/s\nlateral shove",
            va="top", color=INK2, fontsize=8)
    ax.set_yticks(y, ["Foot placement\nalone", "+ LATERAL SPINE\n(already fitted)"])
    ax.set_xlabel("disturbance rejection envelope (mm of DCM error)")
    ax.set_xlim(0, both * 1.62)
    ax.set_title("The spine, bought for the crawl, is the trot's main balance actuator")
    ax.grid(axis="y", visible=False)
    _save(fig, "05_balance_authority.png")


if __name__ == "__main__":
    print("generating README progress figures from the live model...")
    fig_speed()
    fig_tipping_vs_slipping()
    fig_trot_placement()
    fig_closed_loop()
    fig_authority()
    print("done.")
