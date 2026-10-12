"""M152: the bench reducers recover known parameters from synthetic data."""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
import bench_reduce as BR  # noqa: E402

RNG = np.random.default_rng(0)


def test_KT_from_a_stall_sweep_and_the_verdicts():
    i = np.array([0.5, 1.0, 1.6, 2.5, 4.0])
    for kt, word in ((0.46, "passes"), (0.40, "marginal"), (0.35, "over")):
        r = BR.fit_kt(i, kt * i + RNG.normal(0, 0.005, i.size))
        assert r["kt_origin"] == pytest.approx(kt, abs=0.005)
        assert r["r2"] > 0.99
        assert word in BR.verdict_kt(r["kt_origin"])["verdict"]


def test_KT_from_BACK_EMF_matches_a_sinusoidal_machine():
    """A machine with Kt 0.44 per PEAK phase amp shows V_ll,rms = Kt / sqrt(1.5)
    per rad/s at the output."""
    rpm = np.array([100.0, 200.0, 300.0])
    w = rpm * 2 * math.pi / 60
    r = BR.kt_from_backemf(rpm, 0.44 / math.sqrt(1.5) * w)
    assert r["kt_per_peak_A"] == pytest.approx(0.44, rel=1e-9)
    assert r["kt_per_rms_A"] == pytest.approx(0.44 * math.sqrt(2), rel=1e-9)


def test_KV_and_the_8S_floor():
    v = np.array([12.0, 24.0, 28.0])
    r = BR.fit_kv(v, 15.83 * v)
    assert r["kv_rpm_per_V"] == pytest.approx(15.83)
    assert r["noload_rpm_at_8S_floor"] == pytest.approx(443.24)


def test_HEATING_curve_recovers_the_rise_and_time_constant():
    t = np.linspace(0, 3000, 60)
    temp = 22.0 + 48.0 * (1 - np.exp(-t / 600.0)) + RNG.normal(0, 0.2, t.size)
    r = BR.fit_heat(t, temp)
    assert r["rise_C"] == pytest.approx(48.0, abs=1.5)
    assert r["tau_s"] == pytest.approx(600.0, rel=0.08)
    assert "inside" in r["verdict"]


def test_CAPSTAN_separates_mu_from_the_ends():
    bend = np.array([0.0, 90.0, 180.0, 270.0, 360.0])
    t_out = 10.0
    t_in = t_out * np.exp(0.08 * np.radians(bend) + 0.03)
    r = BR.fit_capstan(bend, t_in, np.full(bend.size, t_out))
    assert r["conduit_mu"] == pytest.approx(0.08, abs=1e-9)
    assert r["end_efficiency"] == pytest.approx(math.exp(-0.03))
    assert "walk_loads" in r["verdict"]


def test_FLEXURE_rate_and_hysteresis():
    x = np.array([0.0, 0.2, 0.4, 0.6, 0.8])               # mm
    load = 1.25e5 * x / 1000.0
    r = BR.fit_flexure(np.r_[load, load[::-1] - 2.0], np.r_[x, x[::-1]],
                       ["load"] * 5 + ["unload"] * 5)
    assert r["vs_spec"] == pytest.approx(1.0, abs=0.03)
    assert r["hysteresis_N"] == pytest.approx(2.0, abs=1e-6)


def test_PAD_per_floor_and_the_NFR15_verdict():
    rows = [{"floor": "tile", "kind": "kinetic", "normal_N": "12", "pull_N": "8.4"},
            {"floor": "wood", "kind": "kinetic", "normal_N": "12", "pull_N": "6.0"}]
    r = BR.fit_pad(rows)
    assert r["tile/kinetic"] == pytest.approx(0.7)
    assert "revisit NFR15" in r["verdict"]


def test_a_TEMPLATE_names_every_column(capsys):
    BR.main(["template", "capstan"])
    assert capsys.readouterr().out.strip() == "bend_deg,t_in_N,t_out_N"
