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


def test_the_flank_at_the_spine_axis_is_the_NEUTRAL_FIBRE():
    """✅ The one line on the cover that does not change length when the spine
    bends, and therefore the only place it can be anchored rigidly.

    This is exact, not approximate: the fibre is ON the axis, so its radius is
    zero and so is its length change, whatever the ROM.
    """
    rows, q, span = SK.strain_table()
    flank = [r for r in rows if "ANCHOR" in r[0]]
    assert len(flank) == 1
    _name, z, r, dL, strain = flank[0]
    assert z == pytest.approx(SK.NEUTRAL_Z)
    assert r == pytest.approx(0.0, abs=1e-9)
    assert dL == pytest.approx(0.0, abs=1e-9)


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
