# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""G3's flexure, drawn (M117) and put in the trunk (M120) -- ADR-0107/0111."""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mechanical", "cad"))

import g3_flexure as F  # noqa: E402
from tomcat_kin.params import DEFAULT_SPINE  # noqa: E402


@pytest.mark.parametrize("k", [None, DEFAULT_SPINE.series_k])
def test_both_flexures_hold_BOTH_stresses_at_their_rates(k):
    d = F.design(k)
    assert d["sigma_stop"] == pytest.approx(F.SIGMA_USE * d["sigma_stop_limit"], rel=1e-3)
    assert d["sigma_fatigue"] <= d["sigma_fatigue_limit"]
    assert d["arm_gap"] > 0.8, "the arms must not touch as they wind up"


def test_the_leg_rate_and_stop_are_ADR0107s():
    d = F.design()
    assert d["k_rad"] / 1e3 == pytest.approx(9.57, abs=0.05)
    assert d["stop_deg"] == pytest.approx(13.4, abs=0.2)
    assert F.design(DEFAULT_SPINE.series_k)["stop_deg"] == pytest.approx(11.2, abs=0.2)


def test_the_ACTIVE_volume_is_the_energy_and_the_shape_only_trades_it():
    """At a fixed stop stress the active volume is set by the stored energy;
    width against thickness is free, so the width is spent to buy thickness."""
    leg, sp = F.design(), F.design(DEFAULT_SPINE.series_k)
    assert leg["active_mm3"] == pytest.approx(714.0, rel=0.02)
    assert sp["active_mm3"] == pytest.approx(595.0, rel=0.02)
    assert leg["T"] == pytest.approx(3.06, abs=0.05)
    assert sp["T"] == pytest.approx(1.83, abs=0.05)


def test_the_STOP_DOWELS_take_the_whole_landing():
    """The stop moved out of the part into two Ø2 dowels riding arc slots in
    the spool: the landing's full spool torque, 516 N x 8.75 mm, on two pins."""
    d = F.design()
    assert d["pin_N"] == pytest.approx(602.0, abs=2.0)
    assert d["pin_shear_MPa"] < 0.3 * 1000.0, "hardened dowel, ~1 GPa shear"
    sp = F.design(DEFAULT_SPINE.series_k)
    assert sp["pin_bearing_MPa"] < 0.6 * F.YIELD_TI


def test_every_row_FITS_its_body_with_G3_and_the_bodies_did_not_grow():
    """⚠️ ADR-0107 said 16.4 / 6.9 mm spare; with the bulkhead pads it is 8.0
    per girdle body and 2.7 in spine body 2. The M117 part (5.24 / 5.98 mm)
    fitted neither."""
    pytest.importorskip("build123d", reason="the trunk needs CAD")
    import tomcat_trunk as TT
    assert TT.G3_STACK["hind"] == pytest.approx(3.66, abs=0.05)
    assert TT.G3_STACK["spine"] == pytest.approx(2.43, abs=0.05)
    # M122 (ADR-0112) did lengthen body 0, by the 30 mm between the hind hip
    # and spine joint 0 -- not G3's doing. M123 (ADR-0114) made it exactly its
    # two hip-station rows and pads: 109.2 mm.
    assert TT.BODIES[0][1] - TT.BODIES[0][0] == pytest.approx(109.16, abs=0.05)
    assert TT.BODIES[2][1] - TT.BODIES[2][0] == pytest.approx(51.0, abs=0.05)
    for (_nm, b, x, _n, ro) in TT._rows():
        x0, x1 = TT.BODIES[b]
        h = TT.row_len(ro) / 2 + TT._PAD
        assert x0 - 1e-6 <= x - h and x + h <= x1 + 1e-6, (_nm, x - h - x0, x1 - x - h)


def test_the_parts_are_ONE_valid_solid_each_inside_the_can():
    pytest.importorskip("build123d", reason="drawing the part needs CAD")
    for k, g in ((None, 5.20), (DEFAULT_SPINE.series_k, 3.86)):
        part, _d = F.build(k)
        assert part.is_valid
        bb = part.bounding_box()
        reach = max(bb.max.X, bb.max.Y, -bb.min.X, -bb.min.Y)
        assert reach <= 34.5 / 2, "the flexure must fit the 34.5 mm motor can"
        assert part.volume * F.RHO_TI == pytest.approx(g, abs=0.1)
