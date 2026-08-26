# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""M38 — the mass spiral closed with the measured leg hardware. Findings gated.

⚠️ Several of these assert a **defect** (NFR5 exceeded, the diagonal tendon map)
so they fail when the fix lands. That failure is the signal to re-run the budget,
not to relax the test.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

pytest.importorskip("build123d", reason="build123d is an optional dependency")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mechanical", "cad"))
import mass_closure as MC  # noqa: E402

from tomcat_kin.params import (  # noqa: E402
    DEFAULT_FORELEG, DEFAULT_HINDLEG, DEFAULT_SPINE,
)

GRID = 15


@pytest.fixture(scope="module")
def closed():
    return MC.close(grid=GRID)


def test_the_body_mass_closes_ABOVE_NFR5(closed):
    """⚠️ 4.045 -> **4.304 kg**, NFR5's 4.05 kg exceeded by 6.3 %.

    `WholeBody.total_mass` is `trunk_mass + sum(leg masses)`, so ADR-0041's 167 g
    leg propagates straight into the body mass. Every mass-derived result in the
    project sits downstream: the torque budget, the thermal duty, the runtime.

    Asserts the defect. Fails when `params.py` is updated, at which point the
    whole-body figures need re-publishing.
    """
    # ⚠️ M41 folded the measured masses into `params.py`, so the pre-fold body mass
    # is now a named historical constant rather than something params can be asked.
    old = MC.PRE_M41_BODY
    assert old == pytest.approx(4.045, abs=1e-3)
    assert closed["body"] > 4.05, "the closure is above the OLD NFR5"
    assert closed["body"] == pytest.approx(
        float(DEFAULT_SPINE.trunk_mass) + 2 * sum(DEFAULT_HINDLEG.link_mass)
        + 2 * sum(DEFAULT_FORELEG.link_mass), abs=1e-4), (
        "and params now agrees with the CAD -- if these diverge, one drifted"
    )
    assert closed["body"] == pytest.approx(4.304, abs=0.02)
    assert closed["body"] / old == pytest.approx(1.064, abs=0.01)


def test_the_spiral_still_CONVERGES_and_every_design_gate_holds(closed):
    """The good news, and it is ADR-0010's argument holding up.

    ADR-0010 warned the mass spiral converges *only* because the chosen motor has
    headroom. At 4.304 kg it still does: the trot case sits at **80 %** of the
    GIM3505-9's 1.95 N·m peak, the cable keeps **SF 4.70** against a target of 4,
    and the bearing needs **1277 N** of static C0 against the 1500 N specified.

    So the mass overrun costs margin, not viability.
    """
    b = closed["rows"][-1]
    gates = MC.gate(b)
    for name, got, limit, ok in gates:
        assert ok, f"{name}: {got:.2f} against {limit:.2f}"
    motor = float(b["trot"]["motor"].max())
    assert 0.7 < motor / MC.MOTOR_PEAK < 0.9, f"motor at {motor:.2f} N.m"


def test_the_joint_hardware_gives_BACK_the_P1_inertia_saving(closed):
    """⚠️ THE M38 finding. Leg swing inertia about the hip: **+61.7 %**.

    ADR-0003 accepted the whole tendon-drive cable-tension burden in order to buy
    low limb inertia. The joint hardware — sheaves, bearings, clevises — is
    distributed *along* the limb, and it shifts the mass distally:

    | share, proximal -> distal | femur | tibia | meta | paw |
    |---|---|---|---|---|
    | params (assumed) | 47.3 | 30.0 | 15.5 | 7.3 |
    | measured | 39.5 | 35.3 | 20.7 | 4.5 |

    `LegParams.link_mass` justifies its distribution as *"proximal-heavy because
    both feline anatomy and the ADR-0003 tendon drive push mass toward the body"*.
    The tendon drive pushes the **motors** toward the body. It does not push the
    **pulleys** there, and the metatarsus more than doubles.
    """
    hind = closed["hind"]
    i_new = MC.swing_inertia(hind, DEFAULT_HINDLEG)
    i_old = MC.swing_inertia(MC.PRE_M41_HIND, DEFAULT_HINDLEG)
    assert i_new / i_old > 1.5, f"inertia ratio {i_new / i_old:.2f}"

    share_new = hind / hind.sum()
    share_old = np.asarray(MC.PRE_M41_HIND) / sum(MC.PRE_M41_HIND)
    assert share_new[0] < share_old[0], "the femur's share must FALL"
    assert share_new[2] > 1.3 * share_old[2], "the metatarsus share must rise"


def test_but_the_balance_envelope_barely_moves(closed):
    """And this is why the +62 % does not cascade: the swing is **speed**-limited,
    not acceleration-limited.

    `test_the_ramp_barely_moves_the_envelope` already established that modelling
    the trapezoid costs under 7 % of the envelope, i.e. the acceleration limit is
    not the binding term. So a 1.6x heavier operational-space inertia costs only
    **52.7 -> 51.9 mm, −1.6 %**, and NFR15's 48 mm still clears.

    ⚠️ The inertia ratio is a first-order proxy — the real quantity needs per-link
    inertia tensors the model does not carry.
    """
    hind = closed["hind"]
    ratio = MC.swing_inertia(hind, DEFAULT_HINDLEG) \
        / MC.swing_inertia(DEFAULT_HINDLEG.link_mass, DEFAULT_HINDLEG)
    base, slow = MC.envelope_cost(ratio)
    assert slow["envelope"] < base["envelope"], "heavier must not help"
    assert slow["envelope"] > 0.95 * base["envelope"], "and it costs under 5 %"
    assert slow["envelope"] > 0.048, "NFR15's 48 mm must still clear"
    assert slow["actuation"] > base["actuation"]


def test_the_fore_hind_leg_ASYMMETRY_essentially_disappears(closed):
    """`params.py` carries 95 g fore against 110 g hind — an assumed 1.16x.

    Measured, both legs are ~167 g: the joint hardware dominates and it is the
    *same* hardware on both, so the shorter fore links barely register. Design
    review F2 settled the fore/hind weight split using the assumed asymmetry.
    """
    sp = MC.fore_hind_split(closed["fore"], closed["hind"])
    assert sp["params_hind_g"] / sp["params_fore_g"] == pytest.approx(1.158, abs=0.01)
    assert sp["hind_g"] / sp["fore_g"] == pytest.approx(1.0, abs=0.03)


def test_the_coupled_map_raises_the_KNEE_tension_by_forty_percent(closed):
    """ADR-0042's coupling, priced. `tau = J^T T` with `J` lower-triangular makes
    `J^T` upper-triangular, so distal tendons load proximal joints:

    | tendon | diagonal model | coupled | delta |
    |---|---|---|---|
    | hip | 633 N | 597 N | −5.7 % |
    | **knee** | 435 N | **607 N** | **+39.5 %** |
    | ankle | 491 N | 491 N | 0 |

    SF on the worst coupled tension is **4.94**, so §2's target of 4 still clears —
    the coupling costs margin, not the design.
    """
    b = closed["rows"][-1]
    ct = MC.coupled_tensions(b["land"]["tau"])
    assert ct["coupled"][1] / ct["diagonal"][1] > 1.3, "knee must rise sharply"
    assert ct["coupled"][0] < ct["diagonal"][0], "hip falls slightly"
    assert ct["coupled"][2] == pytest.approx(ct["diagonal"][2], rel=1e-6), (
        "the ankle is distal-most, so nothing couples into it"
    )
    worst = float(max(ct["coupled"]))
    assert MC.CABLE_BREAK / worst > MC.SF_TARGET, f"SF {MC.CABLE_BREAK / worst:.2f}"


def test_the_wrap_SENSES_are_a_load_lever_not_just_a_wrap_lever(closed):
    """⚠️ The actionable half of ADR-0042, and it is free margin.

    The off-diagonal **signs** come from the wrap senses, which `route()` currently
    picks for minimum *wrap*. Choosing them for minimum *load* instead moves the
    worst tension **607 -> 562 N**, i.e. cable SF **4.94 -> 5.34** — 8 % of margin
    for a routing decision that costs nothing.

    So the routing objective should be the load, or a trade against wrap, rather
    than wrap alone.
    """
    b = closed["rows"][-1]
    ct = MC.coupled_tensions(b["land"]["tau"])
    worst_now = float(max(ct["coupled"]))
    worst_alt = float(max(ct["sign_flipped"]))
    assert worst_alt < worst_now, "if the flip no longer helps, re-derive the senses"
    assert MC.CABLE_BREAK / worst_alt > MC.CABLE_BREAK / worst_now
    assert (worst_now - worst_alt) / worst_now > 0.05, "worth at least 5 %"
