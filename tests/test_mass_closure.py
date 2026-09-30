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
    # ⚠️ M111: 4.383 -> **4.468 kg**, the 36/34/22 sheaves (+21 g a leg).
    # ⚠️ M120: -> 4.554 kg, the eighteen G3 flexures (ADR-0111).
    # ⚠️ M122: -> 4.501 kg, the leg drive redrawn (ADR-0112).
    # ⚠️ M123: -> 4.605 kg, the hollow hip (ADR-0114); M124: 4.589; M126: **4.579**.
    assert closed["body"] == pytest.approx(4.579, abs=0.02)
    # ⚠️ NFR5 (4.045 kg) is exceeded by 13.2 % now (13.9 at M123, 11.3 at M122).
    assert closed["body"] / old == pytest.approx(1.132, abs=0.01)


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
    """✅ **The requirement is published, not clipped** (M107), and since M111 it
    covers both legs and has a speed axis.

    | milestone | peak | continuous | speed | binding |
    |---|---|---|---|---|
    | M107, friction on | 2.20 N·m | 0.77 | -- | hind hip |
    | M109, the fore leg found | 2.42 | 0.845 | -- | FORE knee |
    | **M111, 36/34/22 arms** | **1.825** | **0.646** | **665 rpm** | fore knee |

    ✅ Inside the GIM3505-9 proxy on torque, with 6.4 % and 9 % margin.
    ⚠️ Not on speed: the walked trot needs ~665 rpm no-load at the spool
    against the proxy's 380. The arms bought torque with speed, and the proxy
    has too little of their product at any ratio (`power.gait_envelope`).

    ⚠️ `[owed]` -- whether a part meeting 1.83 N·m / 665 rpm keeps the proxy's
    Ø34.5 x 36.1 mm and 131.7 g.
    """
    req = MC.motor_requirement(closed["rows"][-1])
    # M120: 1.825 -> 1.859 under G3's 85.6 g -- 4.7 % under the proxy.
    # ✅ M122 (ADR-0112): -> **1.623**, 17 % under it -- friction is the
    # conduit's slide now, not a capstan on anchored and running pulleys.
    # M123 (ADR-0114): -> 1.656 under the hollow hip's +26 g a leg, the hips
    # 270 mm apart, and the hip pair's conduits from their new row.
    assert req["peak"] == pytest.approx(1.656, abs=0.03)
    # ⚠️ M115: 0.646 (worst pose) -> 0.624 N.m -- the larger of the stance hold
    # (0.495) and the walked trot's RMS (0.624, the fore ankle). Thermal is the
    # load actually carried.
    # M122: 0.624 -> 0.575 N.m under the conduit friction model.
    # M123: 0.575 -> 0.570 (the trot re-tuned, ADR-0114).
    assert req["continuous"] == pytest.approx(0.570, abs=0.02)   # after the M123 trot re-tune
    assert req["trot_rms"] > req["stance_hold"], "the trot, not standing, sets it"
    # ⚠️ M123: a TIE. Both hips now drive off the same kind of row with the
    # same conduit (122 / 140 deg), so the hind hip's trot peak equals the
    # fore's to 0.1 %, and which one "binds" is rounding.
    assert req["binding_leg"] in ("fore", "hind")
    assert req["per_joint"]["hip"][1] == pytest.approx(
        req["fore_per_joint"]["hip"][1], rel=0.01)
    fore_peak = max(req["fore_per_joint"], key=lambda k: req["fore_per_joint"][k][1])
    # M122: the fore HIP (1.623) edges the knee (1.618) -- 0.3 % apart; the
    # knee's capstan share was the larger, and it is gone.
    assert fore_peak in ("hip", "knee"), f"the fore trot is sized by {fore_peak}"
    assert req["fore_per_joint"]["hip"][1] == pytest.approx(
        req["fore_per_joint"]["knee"][1], rel=0.01)
    assert req["peak"] < req["proxy_peak"] and req["continuous"] < req["proxy_rated"]
    # ⚠️ and the axis the proxy fails on now
    # M122: 665 -> 629 rpm -- the swing's torque fell with the friction, and
    # the need is read off the torque-speed LINE.
    assert req["need_rpm"] == pytest.approx(629.0, rel=0.03)
    assert req["need_rpm"] > 1.5 * req["proxy_rpm"]


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



def test_the_coupled_map_raises_the_KNEE_tension(closed):
    """ADR-0042's coupling, priced. `tau = J^T T` with `J` lower-triangular makes
    `J^T` upper-triangular, so distal tendons load proximal joints:

    | tendon | diagonal | coupled | delta, M93 -> **M111** |
    |---|---|---|---|
    | hip | 511 N | 489 N | -5.7 % -> **-4.3 %** |
    | knee | 332 N | **416 N** | +39.5 % -> **+25.1 %** |
    | ankle | 325 N | 325 N | 0 |

    ⚠️ M111's 36/34/22 arms shrank the coupling's share: the off-diagonals are
    the fixed via radius and the diagonals grew. Worst coupled SF 4.94 -> 6.13.
    """
    b = closed["rows"][-1]
    ct = MC.coupled_tensions(b["land"]["tau"])
    assert ct["coupled"][1] / ct["diagonal"][1] == pytest.approx(1.251, abs=0.02)
    assert ct["coupled"][0] < ct["diagonal"][0], "hip falls slightly"
    assert ct["coupled"][2] == pytest.approx(ct["diagonal"][2], rel=1e-6), (
        "the ankle is distal-most, so nothing couples into it"
    )
    worst = float(max(ct["coupled"]))
    assert MC.CABLE_BREAK / worst > MC.SF_TARGET, f"SF {MC.CABLE_BREAK / worst:.2f}"



def test_the_wrap_SENSES_are_a_load_lever_not_just_a_wrap_lever(closed):
    """The actionable half of ADR-0042 -- and M111 moved WHICH cable it is for.

    The off-diagonal signs come from the wrap senses, which `route()` picks for
    minimum wrap. Flipped for minimum load, the knee's coupled tension falls
    **416 -> 249 N, -40 %**. Before M111 that was the WORST cable, so the flip
    bought the whole system 8 % of cable margin for free.

    ⚠️ **At the 36/34/22 arms the worst cable is the HIP (489 N), which the
    flip does not reach -- it is proximal-most, nothing couples into its
    own row.** So the lever is still real and still free at the knee, and no
    longer moves the system's SF. It becomes a knee-cable and bearing-life
    choice rather than a margin one.
    """
    b = closed["rows"][-1]
    ct = MC.coupled_tensions(b["land"]["tau"])
    knee_now, knee_alt = float(ct["coupled"][1]), float(ct["sign_flipped"][1])
    assert (knee_now - knee_alt) / knee_now > 0.35, "the knee lever is still large"
    worst_now = float(max(ct["coupled"]))
    worst_alt = float(max(ct["sign_flipped"]))
    assert int(np.argmax(ct["coupled"])) == 0, "the hip is the worst cable now"
    assert abs(worst_alt - worst_now) / worst_now < 0.02, (
        "and the flip no longer moves the worst tension"
    )


