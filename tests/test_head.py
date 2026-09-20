# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The head and neck (M97) -- 240 g that had a budget and no place.

⚠️ `params.py` carried the warning itself: *"the head is the weak point and it
is not in the box... putting it forward would raise the pitch inertia"*. Nobody
had measured by how much. **24 %** -- +1.02e-2 kg m2 on a body with 4.20e-2 --
and the CoM moves 7.1 mm forward.

⚠️ With the tail's 11.7 % (ADR-0096) the plant has been missing about a third
of its own pitch inertia, on a righting reflex already a factor of nine short.

`build123d` is an optional dependency; the module skips without it.
"""

from __future__ import annotations

import math
import os
import sys

import pytest

pytest.importorskip("build123d", reason="build123d is an optional dependency")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mechanical", "cad"))
import tomcat_head as HD  # noqa: E402
import tomcat_trunk as TT  # noqa: E402


def test_the_head_is_sized_by_MASS_not_by_the_trunks_length():
    """⚠️ The first attempt scaled it off the trunk -- 363 mm against a cat's
    250 -- and that is the wrong ruler twice.

    This robot is 4.38 kg, which IS a cat, so a cat's head is the right head.
    And the trunk is long for that mass because the rear girdle reaches 112 mm
    behind the hip, a standing `[owed]`: scaling by it copies the defect into
    the head, and costs nine points of pitch inertia to do so (24 % -> 33 %).
    """
    assert HD.SCALE == pytest.approx(1.0), (
        "the head must not be scaled off a trunk that is long for its mass"
    )
    assert HD.HEAD_L == pytest.approx(95.0, abs=1.0)
    trunk_scale = (TT.BODIES[3][1] - TT.BODIES[0][0]) / 250.0
    assert trunk_scale > 1.4, (
        "if the trunk has come back to cat proportions the two rulers agree "
        "and this test has nothing left to say"
    )


def test_placing_the_240_g_costs_a_THIRD_of_the_pitch_inertia():
    """⚠️ THE number, and `params.py` predicted its sign and not its size.

    It moved once more when the posture came from the photo: carrying the head
    where a cat carries it -- a head-length above the back rather than the 35 mm
    first drawn -- lengthens the z lever and takes the cost **24 % -> 29 %**.
    Getting the shape right made the account worse, which is the usual direction
    in this project and not a reason to prefer the wrong shape.
    """
    a = HD.account()
    assert a["frac"] == pytest.approx(0.29, abs=0.02)
    assert a["dcom"] == pytest.approx(7.5, abs=0.6)
    assert a["lever_after"] > 2.5 * a["lever_before"], (
        "lever %.0f -> %.0f mm" % (a["lever_before"], a["lever_after"])
    )


def test_the_lump_is_where_params_says_it_is_TODAY():
    """✅ The before side of the account has to be the model's own, not a
    convenient zero: the 240 g rides the front girdle's measured CoM."""
    from tomcat_kin.params import DEFAULT_SPINE as SP
    assert HD.LUMPED_AT[0] == pytest.approx(195.0 + SP.front_girdle_com[0] * 1e3)
    assert HD.LUMPED_AT[1] == pytest.approx(SP.front_girdle_com[1] * 1e3)


def test_the_head_is_carried_a_HEAD_LENGTH_above_the_back():
    """⚠️ Measured off a side-view cat, and it was the biggest error in the
    first version: the head sat **+35 mm** where the photo puts it **+108**, a
    third of the height a cat carries it. The topline ratios are scale-free --
    read in head-lengths above the back line -- which matters because the ruler
    in this file has been wrong twice."""
    top = HD.cranium_top()
    # ⚠️ 1.14 head-lengths was the EAR tip; the skull plates have no ears, so
    # the crown is placed at the traced 0.96 and the ears are owed as features.
    assert top - TT.Z_DORSAL == pytest.approx(0.96 * HD.HEAD_L, rel=0.06), (
        "the skull tops out %.0f mm above the back; the trace says %.0f"
        % (top - TT.Z_DORSAL, 0.96 * HD.HEAD_L)
    )
    nose = max(x for x, _t, _b, _w in HD._stations())
    assert nose == pytest.approx(TT.BODIES[3][1] + HD.U_WITHERS * HD.HEAD_L,
                                 abs=1.0)


def test_the_CRANIUM_comes_from_two_orthogonal_PUBLIC_DOMAIN_views():
    """✅ A side silhouette cannot determine a 3-D surface, and this file
    proved it twice -- once by reading the neck as the head's underside (a
    traced "head depth" of 1.44 head-lengths where a cat's is 0.78) and once by
    shipping the scan's crop line as a throat.

    Figs. 39 and 40 of Reighard & Jennings (1901) are the same skull from two
    directions, so each section takes its HEIGHT from one and its WIDTH from the
    other. They are public domain and in `reference/plates/`, so unlike the
    wildcat photograph this measurement re-runs from the repo.
    """
    lat, dor = HD._skull()
    assert len(lat) > 20 and len(dor) > 20
    tall = max(t - b for _u, t, b in lat)
    wide = 2.0 * max(w for _u, w in dor)
    assert tall == pytest.approx(0.48, abs=0.03), "skull height/length %.2f" % tall
    assert wide == pytest.approx(0.71, abs=0.04), "skull width/length %.2f" % wide
    for fn in ("reighard_fig39_skull_dorsal.jpg", "reighard_fig40_skull_lateral.jpg"):
        assert os.path.exists(os.path.join(
            os.path.dirname(__file__), "..", "mechanical", "reference",
            "plates", fn)), "%s must ship with the repo" % fn


def _retired_test_what_the_PHOTO_can_say_is_separated_from_what_it_cannot():
    """⚠️ A side silhouette cannot separate the head's underside from the
    neck's front: traced, the "head depth" comes out **1.44 head-lengths** where
    a cat's head is about 0.75, because below the jaw the outline is already
    throat and then chest. The scan was clipped as well, so past u ~ 0.95 the
    bottom reads a flat -0.99 -- the cut line, shipped as a shape in the first
    two attempts.

    ✅ So the TOP line is taken from the photo and the depth from anatomy, and
    the two are not averaged together.
    """
    prof = HD._profile()
    clipped = [b for u, _t, b in prof if u > 1.0]
    assert clipped and min(clipped) == pytest.approx(max(clipped), abs=0.02), (
        "the bottom past the occiput should be the flat crop line, which is "
        "exactly why it is not used"
    )
    assert max(u for u, _t, _b in prof if u <= HD.U_OCCIPUT) < 1.0, (
        "no station past the crop may reach the stations list"
    )
    assert 0.6 < HD.DEPTH_FRAC < 0.9, "the depth is anatomical, not traced"


def test_the_mass_is_the_BUDGET_not_a_density_guess():
    """✅ The shape is free; the mass is not. 240 g is what
    `front_girdle_mass` already absorbs, so the form is calibrated to it rather
    than weighed from a made-up density."""
    m, _com, _I = HD.mass_props()
    assert m == pytest.approx(HD.HEAD_NECK_G)
    assert HD.HEAD_NECK_G == pytest.approx(240.0)
