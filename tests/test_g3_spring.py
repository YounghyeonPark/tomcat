# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""G3's series-elastic element as a part -- M114, ADR-0107."""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mechanical", "cad"))

import g3_spring as G  # noqa: E402
from tomcat_kin.params import DEFAULT_SPINE, DEFAULT_TENDON  # noqa: E402


def test_the_HARD_STOP_is_what_makes_the_spring_small():
    """Without a stop the spring absorbs the landing: 1.065 J at 125 kN/m, four
    times what it holds at a stop just past the motor's peak."""
    c = G.load_cases()
    assert c["landing"]["J"] == pytest.approx(1.065, abs=0.01)
    assert c["stop"]["J"] == pytest.approx(0.263, abs=0.005)
    assert c["landing"]["J"] > 4.0 * c["stop"]["J"]


def test_a_spring_IN_THE_CABLE_LINE_cannot_be_G3():
    """125 N/mm over 2 mm of travel wants a spring too stiff to wind: at least
    three active coils forces `d >= 0.037 C^3`, a >4.6 mm wire. None fits even
    at a 40 mm outside diameter. The spring has to be ROTATIONAL, at the spool,
    which is where ADR-0051's model already put it."""
    assert G.size_compression_spring(od_max=40.0) is None


def test_the_STOP_ANGLES_follow_from_the_rate():
    t = G.size_torsion_spring()
    assert t["working_deg"] == pytest.approx(11.7, abs=0.2)
    assert t["stop_deg"] == pytest.approx(13.4, abs=0.2)
    s = G.size_torsion_spring(DEFAULT_SPINE.series_k)
    assert s["stop_deg"] == pytest.approx(11.2, abs=0.2)


def test_the_spring_is_BIDIRECTIONAL_and_a_flexure_does_it_in_six_grams():
    """One spool drives a joint's pair (ADR-0008), so the torque behind it
    reverses. A torsion coil is weak unwinding; a planar flexure is symmetric.
    In titanium or maraging steel the active material is ~3 g, ~6 g as a part;
    17-4PH needs twice that."""
    ti = G.size_flexure(material="Ti-6Al-4V")
    c300 = G.size_flexure(material="maraging C300")
    ph = G.size_flexure(material="17-4PH H1025")
    assert ti["active_g"] == pytest.approx(2.9, abs=0.2)
    assert c300["active_g"] == pytest.approx(3.0, abs=0.2)
    assert ph["active_g"] > 2.0 * ti["active_g"]
    assert c300["binding"] == "fatigue", "maraging is limited by the walked trot"
    # the leg spring is softer than the spine's and stores more at its stop
    sp = G.size_flexure(DEFAULT_SPINE.series_k, material="Ti-6Al-4V")
    assert DEFAULT_TENDON.series_k < DEFAULT_SPINE.series_k
    assert ti["active_g"] > sp["active_g"]
