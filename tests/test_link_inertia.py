# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Per-link inertia: does the assignment rule survive checks it could fail?

⚠️ These tests exist because three earlier attempts at this split came out
+105 %, -79 % and +20 % wrong and **nothing caught them** -- each was checked
against intuition alone. A split can be wrong in a way that still totals
correctly, so the total is not a test. Three that can actually fail:

- `test_sheave_mass_lands_on_the_distal_link` -- exact, no tolerance. Both sides
  measure the *same solids*, so any disagreement is the assignment itself.
- `test_masses_match_per_link_mass` -- the split, within the two equal-share
  approximations `per_link_mass()` documents.
- `test_hip_inertia_matches_the_assignment_free_measurement` -- fails if mass is
  charged to the right link but placed in the wrong spot.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAD = os.path.join(ROOT, "mechanical", "cad")
for p in (CAD, os.path.join(ROOT, "kinematics", "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

pytest.importorskip("build123d", reason="CAD stack (build123d/OCP) not installed")

import link_inertia as LI                                        # noqa: E402
import tomcat_leg_detail as LD                                   # noqa: E402

#: ADR-0087 corrected: the leg about the hip with **catalogue** bearing mass.
#: Its published 1.244e-3 used `bearing()`'s fit envelope at steel density.
#:
#: ⚠️ **M93 moved it again, and upward: 1.1753e-3 -> 1.3816e-3, +17.6 %.** The
#: leg went 167 -> 186.7 g -- via pulleys that did not exist, shafts long enough
#: to reach them, and tube stock sized to SF 2.5 at the real lateral offsets --
#: and swing inertia is where that shows up. ADR-0088 measured the capsule plant
#: carrying 45 % too MUCH swing inertia; this gives a third of that back.
#:
#: ⚠️ **M111: 1.3816e-3 -> 1.5398e-3, +11.4 %, and this is the price ADR-0103
#: left `[owed]`.** The 36/34/22 arms that bring the fore leg inside the motor
#: proxy put 21 g of sheave on each leg, 15 g of it at the knee and ankle --
#: exactly the distal mass P1 exists to avoid. Swing inertia is where it lands.
I_YY_ABOUT_HIP = 1.5398e-3


@pytest.fixture(scope="module")
def cad():
    comps, report, _ = LD.build()
    return comps, report


@pytest.fixture(scope="module")
def links(cad):
    return LI.per_link(*cad)


def test_every_link_gets_mass(links):
    """A link with 0 g means the assignment dropped it -- attempt 1's failure."""
    for name, (m, _, _) in links.items():
        assert m > 1e-3, f"{name} came out {1e3 * m:.1f} g"


def test_sheave_mass_lands_on_the_distal_link(cad):
    """⚠️ The sharp one: exact, because both sides measure the SAME solids.

    `per_link_mass()` sizes each joint's sheave from that joint's moment arm
    (hip 28 mm > knee 25 > ankle 14), then charges it to the joint's **distal**
    link. If `assign()` used proximity to a link instead, the ankle sheave would
    land on the tibia and this fails by ~8 g with no tolerance to hide in.
    """
    comps, report = cad
    joints, bones = LI.geometry(report)
    got = {b: 0.0 for b in LI.LINKS}
    for sd in comps["sheave"].solids():
        m, c = MP_props(sd)
        got[LI.assign("sheave", c, joints, bones)] += m

    # ⚠️ **M93: the sheave group is no longer three discs plus an orphan.** It
    # was `sheave(arm)` per joint plus the "root idler" -- a part 9.6 mm from the
    # nearest cable that turned nothing and whose 2.9 g was charged to the femur
    # anyway. The idler is gone; what the group holds now is one MULTI-GROOVE
    # sheave per joint plus the VIA pulleys the routing always needed and never
    # had. Built from the layout, so the expectation cannot drift from it.
    want = {}
    for jn, d in report["joints"].items():
        link = LI.JOINT_TO_LINK[jn]
        # M123: the hip's sheave rides the hollow hip's HUB, not a shaft
        bore = d["hub"][0] if "hub" in d else d["bearing"][0]
        _, sh = LD.grooved(d["arm"], d["planes"], bore=bore)
        want[link] = want.get(link, 0.0) + sum(
            x.volume for x in sh.solids()) * LD.AL_RHO
        if d.get("via"):
            _, vp = LD.grooved(LD.VIA_R, d["via"]["planes"],
                               bore=d["bearing"][0], lighten=False)
            want[link] += sum(x.volume for x in vp.solids()) * LD.AL_RHO

    for link, w in want.items():
        assert got[link] == pytest.approx(w, rel=1e-9), (
            f"{link}: assigned {got[link]:.2f} g of sheave, "
            f"per_link_mass charges it {w:.2f} g")
    assert got["paw"] == 0.0, "the paw carries no joint of its own"


def MP_props(sd):
    """Grams and metres, for the one test that needs raw solids."""
    import mass_properties as MP
    m, c, _ = MP.properties(sd, LD.AL_RHO * 1e6)
    return m * 1e-6, np.asarray(c, float) * 1e-3


def test_masses_match_per_link_mass(cad, links):
    """The split, within what `per_link_mass()` admits it approximates.

    ⚠️ The tolerance is 10 g and it is **not** slack for the assignment -- it is
    two equal-share terms `per_link_mass()` takes deliberately:

    - the clevis mass is divided **equally by 3**, but the ankle clevis carries a
      Ø10 bearing against the hip's Ø19, so geometry puts ~6 g less on the meta;
    - tendons and anchors are spread in equal thirds, which its own docstring
      calls out as over-charging the proximal links.

    Both make the *geometric* split the better one, so this test pins agreement
    in shape, not a claim that `per_link_mass()` is exact.
    """
    ref, _ = LD.per_link_mass(*cad)
    for name in LI.LINKS:
        got = 1e3 * links[name][0]
        assert abs(got - ref[name]) < 10.0, (
            f"{name}: assigned {got:.1f} g vs per_link_mass {ref[name]:.1f} g")


def test_total_mass_is_the_calibrated_one_not_the_envelope_one(cad, links):
    """⚠️ Asserts the correction: envelopes are not parts.

    `bearing()`'s docstring says "envelope" -- it draws a solid Ø19x6 steel
    annulus where the catalogue bearing is 8.0 g. Reading its volume gives
    12.0 g, **50 % over**, and 15.9 g over across the leg. ADR-0087's 182.2 g
    and its 1.244e-3 both carried it, and its "masses agree to 8 %" was that
    error, not agreement.
    """
    _, ref_mass, total = LD.checks(*cad)
    got = 1e3 * sum(m for (m, _, _) in links.values())
    assert got == pytest.approx(total, abs=0.1)
    raw = LI.per_link(*cad, calibrate=False)
    assert 1e3 * sum(m for (m, _, _) in raw.values()) > total + 10.0


def test_hip_inertia_matches_the_assignment_free_measurement(links):
    """ADR-0087's route: measured about the hip with NO split at all.

    Independent of the mass checks -- those constrain *which link*, this one
    constrains *where the mass sits*.
    """
    got = float(LI.about_hip(links)[1, 1])
    assert got == pytest.approx(I_YY_ABOUT_HIP, rel=0.02), (
        f"about-hip I_yy came out {got:.4e}, expected {I_YY_ABOUT_HIP:.4e}")


def test_THE_LIMB_PLANE_OFFSET_IS_APPLIED(cad):
    """⚠️ Asserts a bug ADR-0087's number is structurally unable to catch.

    `build()` moves every solid to y = TRACK_Y on its last pass but leaves
    `report`'s `p0`/`p1` behind. `I_yy = sum m(x^2 + z^2)` contains no y, so the
    about-hip figure agreed to four figures with the hip 48 mm out of plane --
    while `I_xx` and `I_zz` were both wrong.
    """
    _, report = cad
    joints, _ = LI.geometry(report)
    assert joints["hip"][1] == pytest.approx(LD.TRACK_Y * 1e-3)
    assert LI.HIP_Y == pytest.approx(LD.TRACK_Y * 1e-3)


def test_tensors_are_physically_possible(links):
    """Positive definite, and no principal moment above the other two summed.

    ⚠️ A parallel-axis shift applied to a tensor OCP already reports about the
    centre of mass once produced a **negative** moment here; MuJoCo would have
    taken it and simulated nonsense.
    """
    for name, (_, _, I) in links.items():
        w = np.linalg.eigvalsh(I)
        assert w[0] > 0.0, f"{name} has a non-positive principal moment: {w}"
        assert w[2] <= w[0] + w[1] + 1e-12, f"{name} violates the triangle rule: {w}"


def test_girdle_motors_are_not_charged_to_the_leg(links):
    """P1 centralisation: the motors are on the body, so the leg must not carry them.

    Six 131.7 g motors against a 167 g leg would be a 5x error, and it would
    silently reverse the tendon drive's whole argument.
    """
    total = 1e3 * sum(m for (m, _, _) in links.values())
    assert total < 250.0, f"leg totals {total:.0f} g -- motor mass leaked in"


def test_THE_CAPSULE_MODEL_IS_STILL_WHAT_THE_MJCF_SHIPS(links):
    """⚠️ Asserts the DEFECT: the gap is measured, not closed.

    Delete this test in the milestone that feeds `per_link()` into
    `mjcf_tendon`; until then it stops the gap being quietly forgotten.
    """
    mujoco = pytest.importorskip("mujoco")
    from tomcat_kin import mjcf_tendon as MT

    m = mujoco.MjModel.from_xml_string(
        MT.quadruped_rig(hip_height=0.176, spine=True))
    capsule = {b: float(m.body_mass[mujoco.mj_name2id(
        m, mujoco.mjtObj.mjOBJ_BODY, f"LF_{b}")]) for b in LI.LINKS}
    assert capsule != pytest.approx([links[b][0] for b in LI.LINKS]), (
        "the MJCF now carries the CAD masses -- if the tensors went in too, "
        "delete this test and update ADR-0087")
