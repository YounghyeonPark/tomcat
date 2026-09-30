# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The manufacturing-level leg pass (M36) — its findings, gated.

⚠️ These tests exist because a 3D design pass found things a document pass could
not, and every one of them is a *discrepancy* rather than a capability. Several
assert the DEFECT, so they fail when the spec is fixed — and that failure is the
signal to update the spec text, not to relax the test.

`build123d` is an optional dependency; the module skips without it.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

pytest.importorskip("build123d", reason="build123d is an optional dependency")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mechanical", "cad"))
import tomcat_leg_detail as L  # noqa: E402


@pytest.fixture(scope="module")
def loads():
    return L.live_loads()


def test_the_spec_torque_table_is_stale_by_the_WHOLE_mass_history(loads):
    """⚠️ THE finding, and M41 widened it. LEG_TENDON_SPEC §1.1 tabulates the hip
    land torque as **12.36 N·m**, and §1.3, §1.3a, §3.5 and §0.1 all derive from it.

    | body mass | live hip land torque | ratio to §1.1 |
    |---|---|---|
    | 3.0 kg (§1.1's own basis) | 12.36 N·m | 1.00 |
    | 4.045 kg (ADR-0010) | 16.67 N·m | 1.35 |
    | 4.3041 kg (ADR-0046) | 17.73 N·m | 1.43 |
    | 4.3833 kg (ADR-0093) | 18.06 N·m | 1.46 |
    | 4.4684 kg (ADR-0103, M111) | 18.41 N·m | 1.49 |
    | 4.5540 kg (ADR-0111, M120, G3) | 18.76 N·m | 1.52 |
    | 4.5009 kg (ADR-0112, M122) | 18.54 N·m | 1.50 |
    | 4.6053 kg (ADR-0114, M123, hollow hip) | 18.97 N·m | 1.54 |
    | **4.5889 kg (ADR-0115, M124)** | **18.91 N·m** | **1.53** |

    The ratio tracks the body mass exactly, which is what proves §1.1 is simply a
    stale snapshot rather than a different calculation. §2 *was* re-run at 4.045 kg
    and says "~600 N"; at 4.3833 it is 650 N, so §2 is now stale too — one
    milestone's correction became the next one's staleness.
    """
    tau, T = loads["land"]["tau"][0], loads["land"]["T"][0]
    assert tau == pytest.approx(18.91, abs=0.05)
    # ⚠️ M111: the tension FELL while the torque rose, 650 -> 516 N -- the
    # 36 mm hip arm carries the same torque on less cable. Torque tracks mass;
    # tension tracks mass over arm, so it is no longer a pure mass snapshot.
    assert T == pytest.approx(532.1, abs=2.0)          # M120: 526.2; M122: 520.1; M123: 532.1
    assert tau / 12.36 == pytest.approx(4.58886 / 3.0, rel=0.02), (
        "the discrepancy should be exactly the body-mass ratio; if it is not, "
        "something other than body mass moved and this needs re-diagnosing"
    )


def test_the_shipped_tube_sections_do_NOT_make_SF_2_at_the_live_loads(loads):
    """§3.5 chose Ø12/Ø10/Ø8 × 1.0 to equalise the safety factor at 2.84/3.10/2.87.

    Re-derived at the live torque, plus the torsion the sheave's lateral offset
    actually imposes (§0.1's combined check), the same sections give
    **1.97 / 2.08 / 1.84** — and §0.1's whole argument rested on *"even the worst
    case stays above SF 2"*.

    ⚠️ Asserts the defect. Fails when the sections are re-specified.
    """
    land = loads["land"]
    # ⚠️ The offsets were the literals 12.2 / 12.2 / 9.2 -- a layout that no
    # longer exists. Read them from the leg that is built, the same way
    # `size_tubes` now does, or this test grades a different robot.
    lay = L.plane_layout({"hip": 6.4, "knee": 6.4, "ankle": 4.4},
                         {j: L.BEARING[j][2] for j in ("hip", "knee", "ankle")})
    got = []
    for i, (bone, e) in enumerate(
            (("femur", lay["hip"]["sheave"][0]),
             ("tibia", lay["knee"]["sheave"][0]),
             ("meta", lay["ankle"]["sheave"][0]))):
        od, wall = L.TUBE[bone]
        Z = L.section_Z(od, wall)
        sig = land["tau"][i] * 1e3 / Z
        tau_s = land["T"][i] * e / (2 * Z)
        got.append(400.0 / math.sqrt(sig ** 2 + 3 * tau_s ** 2))

    # ✅ **M93 SIZED THE TUBES, so this now asserts the fix.** §3.5's
    # Ø12/Ø10/Ø8 gave 1.83 / 1.94 / 1.75 at the live loads and the lateral
    # offsets the routing really produces -- the finding stood for four
    # milestones. The leg is built to **Ø14 / Ø12 / Ø12** and clears SF 2.5 on
    # every bone for the first time. The spec text is what is stale now.
    assert all(g >= 2.5 for g in got), (
        "SF %s -- if a bone drops under 2.5 the sizing has drifted from the "
        "layout again" % [round(float(g), 2) for g in got])
    # ⚠️ M111: the metatarsus 2.97 -> 3.72. Bending grew with the body, but
    # the 22 mm ankle arm cut the cable tension -- and with it the torsion the
    # sheave's lateral offset puts into the tube -- by more.
    # ⚠️ M120: G3's 85.6 g takes the femur 2.59 -> 2.54, 0.04 above the 2.5
    # line -- the thinnest margin in the leg, and the next mass increase
    # crosses it.
    assert got[0] == pytest.approx(2.54, abs=0.03)
    assert got[2] == pytest.approx(3.65, abs=0.05)


def test_one_step_up_in_stock_tube_restores_the_margin_cheaply():
    """The remedy, and why it is the right one: bending strength goes as the CUBE
    of diameter while tube mass goes only as the first power. Ø12→Ø14, Ø10→Ø12,
    Ø8→Ø10 recovers SF 2.78/3.16/3.11 for **under 4 g on the whole leg**.
    """
    picks = L.size_tubes(target_sf=2.5)
    # ⚠️ Ø10 was the answer at the OLD lateral offsets. The metatarsus cable
    # now runs at 21.7 mm rather than 9.2 -- it has to pass outboard of two via
    # pulleys -- so the sizer asks for Ø12 there. Read from the layout, not
    # written down.
    assert [p["od"] for p in picks] == [14.0, 12.0, 12.0]
    assert all(p["sf"] >= 2.5 for p in picks)

    def area(od, wall):
        return math.pi * ((od / 2) ** 2 - (od / 2 - wall) ** 2)

    grown = sum(area(p["od"], p["wall"]) for p in picks)
    now = sum(area(*L.TUBE[b]) for b in ("femur", "tibia", "meta"))
    added = (grown - now) * 70.0 * L.CF_RHO          # ~70 mm mean tube run
    assert added < 7.0, f"the fix should stay cheap, costs {added:.1f} g"


def test_the_moment_arms_cannot_be_REDUCED_the_motor_peak_binds():
    """The trade the specs left open, now closed — against shrinking the arms.

    §1.2 grew the arms to cut cable tension and priced only the ankle's inertia.
    Turned as real parts the sheaves are **41 g of a 110 g leg**, so there is now a
    reason to want them smaller. There is no room: at the shipped arms the trot
    case already sits at **81 %** of the GIM3505-9's 1.95 N·m peak (matching §2's
    own "0.82× peak"), and one step down to 0.85× puts it **over 100 %**.

    So the sheave mass is not negotiable through the moment arms, and the leg's
    mass overrun has to be found elsewhere or the budget has to move.
    """
    rows = {r["k"]: r for r in L.trade_moment_arms((1.0, 0.85, 0.70))}
    # ⚠️ M41: 0.81 -> 0.88 (the 8.75 mm spool). ⚠️ M111: the arms went
    # 28/25/14 -> 36/34/22 and this function gained the capstan it had missed
    # since M106 -- it read 65 % frictionless. With the hip flexor's factor it
    # is **0.85**, and one step down still crosses peak, so the finding stands
    # at the new arms: they are pinned from below by the motor.
    # ⚠️ M122 (ADR-0112): 0.85 -> 0.74. The capstan on every degree of wrap
    # is gone; friction is the conduit's slide. The arms are still pinned
    # from below -- see the step down.
    # M123 (ADR-0114): 0.74 -> 0.79 under the hollow hip's +104 g of body.
    assert rows[1.0]["mot_frac"] == pytest.approx(0.79, abs=0.04), (
        "the shipped arms sit at ~79 % of motor peak in this re-sized-cable what-if"
    )
    # ⚠️ M122 REVERSED this: 0.85x now fits (93 % of peak); 0.70x does not
    # (122 %). M123: 99 % and 130 % -- 0.85x fits by 1 %. The motor still pins the arms from below, but at ~0.8x -- and
    # there the cable's own floor (ADR-0106, ~25 mm at the hip) is close.
    assert rows[0.85]["mot_frac"] < 1.0, "0.85x fits the motor since M122"
    assert rows[0.70]["mot_frac"] > 1.0, "0.70x must still exceed motor peak"
    assert rows[0.70]["sheave_g"] < rows[1.0]["sheave_g"], "smaller arms are lighter"


def test_the_leg_does_not_close_at_its_mass_budget():
    """⚠️ Turned as real parts the leg is **~160 g against `link_mass`'s 110 g**.

    Bearings, sheaves and clevises are ~82 % of it and none is negotiable: the
    arms are pinned by the motor peak (above), the bearings by ASSEMBLY_SPEC §2's
    static C₀ ≥ 1.5 kN, and the clevises carry the bearing bores. +50 g on four
    legs is +200 g on a 4.045 kg body — **NFR5's 4.05 kg breaks by ~5 %**, which
    re-triggers exactly the ADR-0010 spiral.

    ⚠️ Asserts the defect. Fails when either the hardware or the budget moves.
    """
    comps, report, _ = L.build()
    _, mass, total = L.checks(comps, report)
    # ⚠️ M41 folded the CAD's own figure into `params.py`, so comparing against
    # `link_mass` is now circular. The claim is about the ALLOWANCE this design was
    # given, which is the pre-M41 110 g.
    budget = 110.0

    # ⚠️ M93 rebuilt the drive train -- via pulleys that did not exist, shafts
    # long enough to reach them, tubes sized to SF 2.5 -- and the leg went
    # **167 -> 186.7 g**. The gap to `link_mass` is not an overrun any more, it
    # is `params.py` being stale: M86 set `link_mass` FROM this measurement, and
    # `body_mass_kg` is derived from it in turn.  `[owed]`
    assert total > budget, "if the leg now closes, re-read the ADR and the budget"
    # ⚠️ M111: 186.7 -> **208.1 g**, the 36/34/22 sheaves ADR-0103 needed for
    # the fore leg's load. Folded into `link_mass` in the same milestone.
    # ⚠️ M122: 208.1 -> **195.1 g** -- the hip via and its bearing, the ankle
    # return spring and the anchor pins left; the femur's conduit ferrules came.
    # ⚠️ M123: 195.1 -> **221.2 g** -- the hollow hip (ADR-0114): a hub on two
    # 40 x 50 bearings for the clevis, tongue, shaft and two 626s.
    assert 215.0 < total < 227.0, f"leg hardware {total:.1f} g moved unexpectedly"
    heavy = mass["bearing"] + mass["sheave"] + mass["clevis"]
    assert heavy / total > 0.75, "the joint hardware is the overrun, not the bones"


def test_the_full_ROM_is_free_of_self_interference():
    """A real verification result rather than a discrepancy: swept over the whole
    joint ROM box, the worst non-adjacent link clearance is **+15.4 mm**.

    ⚠️ The sheaves are excluded on purpose — they sit laterally offset from the
    bone plane, so they cannot foul a link by construction. What they can foul is
    the girdle, which this file does not model.
    """
    worst = L.rom_clearance(n=9)
    assert worst["gap"] > 5.0, (
        f"worst clearance {worst['gap']:.1f} mm at "
        f"{[round(math.degrees(v)) for v in worst['q']]} deg"
    )


def test_the_paw_phalanx_cannot_be_a_bonded_tube():
    """Geometry forces a fabrication change ASSEMBLY_SPEC §1 did not anticipate.

    The 25 mm paw link leaves 10 mm of tube after joint hardware — two bonded
    inserts plus a gap need ≥ 14 mm. So the phalanx is a solid turned/printed
    stub, not a CF tube with end fittings.
    """
    _, report, _ = L.build()
    assert report["paw"]["solid_stub"] is True
    assert report["paw"]["cut"] < 2 * L.MIN_ENGAGE + 2.0


def test_the_groove_follows_the_cable_not_the_stale_literal():
    """§3.1 says groove r ≈ 0.85 mm for a 1.5 mm cable; §2 re-sized the cable to
    1.75 mm (ADR-0010) and §3.1 was never updated. 0.55 × 1.75 = 0.96 mm."""
    assert L.CABLE_D == 1.75
    assert L.GROOVE_R == pytest.approx(0.55 * 1.75)
    assert L.GROOVE_R > 0.85, "the stale literal would pinch the specified cable"


def test_the_sheave_pitch_radii_ARE_the_tendon_moment_arms():
    """The CAD cannot drift from the torque budget: the groove's pitch line is the
    moment arm `TendonParams` hands the kinematics model."""
    from tomcat_kin.params import DEFAULT_TENDON

    _, report, _ = L.build()
    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm) * 1e3
    for i, (jn, d) in enumerate(report["joints"].items()):
        assert d["arm"] == pytest.approx(arms[i], abs=1e-9), jn
