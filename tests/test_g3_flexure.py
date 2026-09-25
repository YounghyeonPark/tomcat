# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""G3's flexure, drawn -- M117, ADR-0107."""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mechanical", "cad"))

import g3_flexure as F  # noqa: E402
from tomcat_kin.params import DEFAULT_SPINE  # noqa: E402


def test_the_leg_flexure_holds_BOTH_stresses_at_the_shipped_rate():
    d = F.design()
    assert d["k_rad"] / 1e3 == pytest.approx(9.57, abs=0.05)
    assert d["stop_deg"] == pytest.approx(13.4, abs=0.2)
    assert d["sigma_stop"] <= d["sigma_stop_limit"]
    assert d["sigma_fatigue"] <= d["sigma_fatigue_limit"]
    assert d["arm_gap"] > 1.0, "the arms must not touch as they wind up"


def test_the_STOP_leaves_exactly_the_free_play_each_way():
    """Hub and rim lugs clear each other by `(180 - hub - rim)/2` each way."""
    for k in (None, DEFAULT_SPINE.series_k):
        d = F.design(k)
        assert (180.0 - d["hub_lug_deg"] - F.RIM_LUG_DEG) / 2 == pytest.approx(d["stop_deg"])
        assert d["hub_lug_deg"] > 0.0


def test_the_SPINE_variant_is_the_same_part_thicker():
    leg, sp = F.design(), F.design(DEFAULT_SPINE.series_k)
    assert sp["T"] > leg["T"]
    assert sp["sigma_stop"] <= sp["sigma_stop_limit"]
    assert sp["sigma_fatigue"] <= sp["sigma_fatigue_limit"]


def test_the_part_is_ONE_valid_solid_inside_the_can_at_about_8_g():
    pytest.importorskip("build123d", reason="drawing the part needs CAD")
    part, d = F.build()
    assert part.is_valid
    bb = part.bounding_box()
    reach = max(bb.max.X, bb.max.Y, -bb.min.X, -bb.min.Y)
    assert reach <= 34.5 / 2, "the flexure must fit the 34.5 mm motor can"
    assert part.volume * F.RHO_TI == pytest.approx(8.26, abs=0.3)
