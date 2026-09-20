# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The skin cover (M94) -- the deliverable that waited for a skeleton to cover.

✅ The design is not drawn and then checked; it is DERIVED from one table. A
skin fibre at height z changes length by `(z - spine_axis) * theta` at each
joint, so over 75 deg of total pitch ROM the dorsal line takes 12.6 % and the
belly at the chest **34.7 %** -- and the flank at the spine axis takes exactly
**zero**. That neutral fibre is where the cover is anchored, a knit carries the
dorsal panel, and the belly cannot be a stretch panel at all: it is slack,
gathered into a fold that pays out. A cat has one and it is called the
primordial pouch.

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
import tomcat_skin as SK  # noqa: E402
import tomcat_trunk as TT  # noqa: E402
import tomcat_leg_detail as LD  # noqa: E402


def test_NO_fibre_is_neutral_in_BOTH_pitch_and_yaw():
    """⚠️ **The first version of this module assumed one was.** The flank at the
    spine axis takes exactly zero in pitch, which is why it was chosen -- and
    **9.8 %** in yaw, because yaw turns about the vertical and the flank is the
    furthest thing from it. The two neutral lines are perpendicular:

        point on the cover        pitch    yaw    worst
        dorsal midline            12.6 %   0.0 %  12.6 %
        flank at the spine axis    0.0 %   9.8 %   9.8 %
        belly midline             34.7 %   0.0 %  34.7 %

    So "the one line that does not move" was true of one DOF and the cover has
    two. Asserting the absence keeps the claim from coming back.
    """
    span = TT.BODIES[3][1] - TT.BODIES[0][0]
    hw = SK.outline(0.0)[0]
    pitch0 = SK.fibre_strain(hw, SK.NEUTRAL_Z, span)
    assert pitch0[0] == pytest.approx(0.0, abs=1e-12)
    assert pitch0[1] == pytest.approx(0.098, abs=0.005)
    yaw0 = SK.fibre_strain(0.0, SK.outline(0.0)[1], span)
    assert yaw0[1] == pytest.approx(0.0, abs=1e-12)
    assert yaw0[0] == pytest.approx(0.126, abs=0.005)


def test_the_SEAM_goes_where_the_WORST_of_the_two_is_least():
    """✅ Neither neutral line, but the upper flank between them -- pitch and
    yaw balanced at **6.6 %**, a third better than the 9.8 % the flank alone
    would take. Scanned over the section rather than argued.
    """
    span = TT.BODIES[3][1] - TT.BODIES[0][0]
    flank_only = SK.fibre_strain(SK.outline(0.0)[0], SK.NEUTRAL_Z, span)[1]
    worst = 0.0
    for x in SK._sections():
        y, z, w = SK.anchor_at(x)
        ep, ey = SK.fibre_strain(y, z, span)
        # the optimum balances the two; if it did not, one could be traded down
        assert ep == pytest.approx(ey, abs=0.004), (
            "seam at x=%.0f is %.1f %% pitch against %.1f %% yaw"
            % (x, 100 * ep, 100 * ey)
        )
        assert 0.0 < y < SK.outline(x)[0], "the seam is on the upper flank"
        assert z > SK.NEUTRAL_Z, "above the spine axis, not on it"
        worst = max(worst, w)
    assert worst < flank_only, (
        "the seam is %.1f %% where the plain flank is %.1f %%"
        % (100 * worst, 100 * flank_only)
    )
    assert worst == pytest.approx(0.067, abs=0.005)


def test_the_BELLY_cannot_be_a_stretch_panel():
    """⚠️ 34.7 % is not a knit, and that is why the belly is a FOLD.

    Asserts the two numbers that force the decision apart: the dorsal panel is
    inside what a knit does and the ventral one is not, by a factor of about
    three. If they ever come together the fold could go and the cover could be
    one material -- which is exactly the change this test should catch.
    """
    rows, _q, _span = SK.strain_table()
    dorsal = [r for r in rows if r[0].startswith("dorsal")][0]
    belly = [r for r in rows if "chest" in r[0]][0]
    assert dorsal[4] == pytest.approx(0.126, abs=0.01)
    assert belly[4] == pytest.approx(0.347, abs=0.01)
    assert belly[4] > 2.5 * dorsal[4], (
        "the belly is %.0f %% against the dorsal %.0f %% -- if the ratio closes, "
        "re-read the fold" % (100 * belly[4], 100 * dorsal[4])
    )


def test_the_cover_is_hung_from_the_PROCESS_TIPS_not_the_dorsal_line():
    """⚠️ Asserts a defect this module's own check caught on its first run.

    Drawn on `Z_DORSAL` the cover sat at 81.8 against process tips that reach
    82.8 -- **pierced at every joint, by 1.0 mm**. The tips are the highest part
    of the skeleton and a cat's back rests on them; so does this.
    """
    tip = TT.SPINE_Z + TT.DORSAL_ARM + TT.POST_R
    assert SK.TIP_Z == pytest.approx(tip)
    tops = [SK.outline(x)[1] for x in SK._sections()]
    assert min(tops) - tip == pytest.approx(SK.CLEAR, abs=1e-9)
    assert min(tops) > TT.Z_DORSAL, (
        "the cover must sit above the dorsal line, not on it"
    )


def test_the_cover_encloses_the_skeleton_everywhere():
    """⚠️ Dorsal clearance says nothing about the flanks, and the widest section
    is a girdle rather than a row the cover was lofted through."""
    worst = 1e9
    for x in np.linspace(TT.BODIES[0][0], TT.BODIES[3][1], 60):
        hw, top, belly = SK.outline(x)
        s_hw = TT._hw_at(x)
        s_top = TT._zc(x) + s_hw * TT.ASPECT
        s_belly = TT._zc(x) - s_hw * TT.ASPECT
        worst = min(worst, hw - s_hw, top - s_top, s_belly - belly)
    assert worst >= 0.5, "the cover is %.2f mm inside the skeleton" % worst


def test_the_legs_leave_through_an_APERTURE_not_a_gap():
    """⚠️ The flank clears the femur by **0.5 mm**, which is not clearance on a
    cover that deflects -- so the check that merely asked "does it foul" was the
    wrong question. Each leg leaves through a hole with a cuff."""
    inner = LD.TRACK_Y - LD.TUBE["femur"][0] / 2
    assert 0.0 < inner - SK.outline(0.0)[0] < 3.0, (
        "the rear flank clears the femur by %.1f mm -- if that has become real "
        "clearance the aperture may be reconsidered"
        % (inner - SK.outline(0.0)[0])
    )
    assert SK.APERTURE_R > LD.TUBE["femur"][0] / 2 + 5.0
    assert len(SK.hip_apertures()) == 4


def test_the_cover_is_one_piece():
    """✅ Four holes cut in a closed shell must leave one cover, not five."""
    sol = SK.skin().solids()
    assert len(sol) == 1, "the cover came apart into %d pieces" % len(sol)
    grams = sum(s.volume for s in sol) * 1.15e-3
    assert 100.0 < grams < 200.0, "knit nylon cover is %.0f g" % grams
