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
    """⚠️ 4.045 -> 4.304 kg (M87) -> **4.383 kg** (M93), NFR5's 4.05 kg
    exceeded by **8.4 %**.

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
    assert closed["body"] == pytest.approx(4.383, abs=0.02)
    # ⚠️ NFR5 (4.045 kg) is exceeded by 8.4 % now, was 6.3.
    assert closed["body"] / old == pytest.approx(1.084, abs=0.01)


def test_the_spiral_still_CONVERGES_and_every_design_gate_holds(closed):
    """The gates that are set by SOURCED PARTS, and they hold.

    ADR-0010 warned the mass spiral converges *only* because the chosen motor
    has headroom. At 4.304 kg the cable keeps **SF 4.70** against a target of 4
    and the bearing needs **1277 N** of static C0 against the 1500 specified.

    ⚠️ **M107 took the motor out of this list, because it is a PROXY.** M106
    turned capstan friction on -- ADR-0083 had solved every wrap in M78 and
    nothing read them -- and the trot went 1.71 -> **2.20 N.m** against the
    GIM3505-9's 1.95 peak, so the gate failed. The mechanism is the primary
    design here and the motor is a placeholder whose torque is an OUTPUT; a
    proxy cannot gate a design, a sourced part can. What remains are the cable
    and the bearing, which really were chosen. See
    `test_the_MECHANISM_emits_a_MOTOR_SPEC_rather_than_fitting_one`.
    """
    b = closed["rows"][-1]
    gates = MC.gate(b)
    assert gates, "the gate list must not be empty"
    for name, got, limit, ok in gates:
        assert ok, f"{name}: {got:.2f} against {limit:.2f}"
    assert not any("motor" in name for name, *_ in gates), (
        "the motor is a proxy; it must not gate the mechanism"
    )


def test_the_MECHANISM_emits_a_MOTOR_SPEC_rather_than_fitting_one(closed):
    """✅ **M107: the requirement is published, not clipped.**

    The linkage is what is being designed; the actuator is sourced or designed
    to suit it afterwards. So the number this records is a specification.

        peak (trot, the ACTUATOR case)   2.20 N.m
        continuous (stand)               0.77 N.m

    ADR-0008 fixes the trot as the actuator sizing case and puts the x2.5
    single-leg landing explicitly OUTSIDE that envelope, so the 7.21 N.m land
    transient is not in the spec -- it sizes cable, pulley and bearing instead.

    ⚠️ The GIM3505-9 proxy is short by **13 % on peak and 9 % on
    continuous**, which is a slightly larger part rather than a different
    class. ⚠️ What is NOT settled is whether such a part keeps the proxy's
    Ø34.5 x 36.1 mm envelope and 131.7 g -- 19 of those are half the body, so
    if it cannot, the mass budget moves again.  `[owed]`
    """
    req = MC.motor_requirement(closed["rows"][-1])
    assert req["peak"] == pytest.approx(2.20, abs=0.05)
    assert req["continuous"] == pytest.approx(0.77, abs=0.03)
    # the hip is the sizing joint, and it is the one whose wrap is avoidable
    peak_joint = max(req["per_joint"], key=lambda k: req["per_joint"][k][1])
    assert peak_joint == "hip", f"the trot is sized by {peak_joint}"
    # ⚠️ and it exceeds the proxy -- if this ever passes silently again, the
    # friction has been switched back off.
    assert req["peak"] > req["proxy_peak"], (
        f"the requirement {req['peak']:.2f} no longer exceeds the proxy "
        f"{req['proxy_peak']:.2f} -- has the capstan gone back to 1.0?"
    )


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

    # ⚠️ **This asserted the SHARE and the prose claims the MASS.** They are
    # not the same test once the whole leg gets heavier, and the share version
    # was passing on a defect: `per_link_mass` divided the clevis mass three ways
    # equally, over-charging the metatarsus by 5.9 g. M93 fixed that and
    # redesigned the leg, and the two effects nearly cancelled -- the mass ratio
    # is what the finding is about, so it is what is asserted.
    assert hind[2] / MC.PRE_M41_HIND[2] > 1.8, (
        "the metatarsus mass is %.1f g against %.1f -- the finding is that the "
        "pulleys stay distal" % (1e3 * hind[2], 1e3 * MC.PRE_M41_HIND[2]))
    share_new = hind / hind.sum()
    share_old = np.asarray(MC.PRE_M41_HIND) / sum(MC.PRE_M41_HIND)
    assert share_new[0] < share_old[0], "the femur's share must FALL"
    assert share_new[2] > share_old[2], "the metatarsus share must rise"


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
