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


def test_placing_the_240_g_costs_a_QUARTER_of_the_pitch_inertia():
    """⚠️ THE number, and `params.py` predicted its sign and not its size."""
    a = HD.account()
    assert a["frac"] == pytest.approx(0.24, abs=0.02)
    assert a["dcom"] == pytest.approx(7.1, abs=0.5)
    assert a["lever_after"] > 2.5 * a["lever_before"], (
        "lever %.0f -> %.0f mm" % (a["lever_before"], a["lever_after"])
    )


def test_the_lump_is_where_params_says_it_is_TODAY():
    """✅ The before side of the account has to be the model's own, not a
    convenient zero: the 240 g rides the front girdle's measured CoM."""
    from tomcat_kin.params import DEFAULT_SPINE as SP
    assert HD.LUMPED_AT[0] == pytest.approx(195.0 + SP.front_girdle_com[0] * 1e3)
    assert HD.LUMPED_AT[1] == pytest.approx(SP.front_girdle_com[1] * 1e3)


def test_the_head_is_carried_ABOVE_the_back_and_off_the_shoulder():
    top = max(s.bounding_box().max.Z for s in HD.head().solids())
    assert top > TT.Z_DORSAL + 20.0, (
        "a cat carries its head above the dorsal line; this is at %.0f vs %.0f"
        % (top, TT.Z_DORSAL)
    )
    base = HD._axis()[0]
    assert base[0] == pytest.approx(TT.BODIES[3][1]), (
        "the neck leaves from the trunk's front face"
    )


def test_the_mass_is_the_BUDGET_not_a_density_guess():
    """✅ The shape is free; the mass is not. 240 g is what
    `front_girdle_mass` already absorbs, so the form is calibrated to it rather
    than weighed from a made-up density."""
    m, _com, _I = HD.mass_props()
    assert m == pytest.approx(HD.HEAD_NECK_G)
    assert HD.HEAD_NECK_G == pytest.approx(240.0)
