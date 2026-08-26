# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Guards the Python -> Rust handoff for the M18 thermal model.

`thermal/src/lib.rs` hard-codes per-motor dissipation and runtimes taken from
`tomcat_kin.power`. Rust cannot import Python, so those numbers are copied — and a
copied number goes stale silently, which is exactly the failure this project keeps
catching. These tests fail the moment `power.py` moves away from them.

⚠️ If one of these fails, the fix is to update BOTH: the constant in
`thermal/src/lib.rs::from_power_py` and the expected value here. Do not relax the
tolerance — the thermal conclusions are sensitive to these watts.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from tomcat_kin import gait, power

LIB_RS = Path(__file__).resolve().parents[1] / "thermal" / "src" / "lib.rs"


@pytest.fixture(scope="module")
def live():
    c = gait.GaitController(gait.trot_params())
    g = power.gait_power(c)
    s = power.standing_power()
    r = power.runtime(c)
    return {
        "TROT_W": g["copper_w"] / 12.0,
        "STAND_W": s["legs_w"] / 12.0,
        "TROT_RUNTIME_MIN": r["trot_minutes"],
        "STAND_RUNTIME_MIN": r["stand_minutes"],
        # ⚠️ M40: TOTAL_W was a bare const inside two Rust functions, outside this
        # guard, and it went stale exactly as this file's docstring predicts.
        "TOTAL_W": g["total_w"],
    }


@pytest.fixture(scope="module")
def declared():
    src = LIB_RS.read_text(encoding="utf-8")
    found = dict(re.findall(r"pub const (\w+): f64 = ([0-9.]+);", src))
    return {k: float(v) for k, v in found.items()}


@pytest.mark.parametrize(
    "name",
    ["TROT_W", "STAND_W", "TROT_RUNTIME_MIN", "STAND_RUNTIME_MIN", "TOTAL_W"],
)
def test_rust_constants_match_the_python_model(name, live, declared):
    assert name in declared, f"{name} missing from {LIB_RS.name}"
    assert declared[name] == pytest.approx(live[name], rel=1e-3), (
        f"{name}: thermal/src/lib.rs says {declared[name]}, "
        f"power.py now gives {live[name]:.4f} — update both"
    )


def test_standing_still_costs_more_per_motor_than_trotting(live):
    """The premise of M18's worst case, and of ADR-0021's brake.

    A cable can only pull, so a tendon-driven joint holds posture with current.
    If this ever inverts, the thermal worst case moves and the brake argument
    weakens — so it is asserted rather than assumed.
    """
    assert live["STAND_W"] > live["TROT_W"]
