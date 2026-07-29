"""Tests for the M4 quasi-static stability margin (sagittal fore-aft interval).

Reminder of the honest scope being tested: the 2D "support polygon" is really a
FORE-AFT INTERVAL, so a positive margin is necessary but NOT sufficient for real
static stability -- lateral/roll tipping needs the 3D model. These tests check
the fore-aft arithmetic only.
"""

import numpy as np
import pytest

from tomcat_kin import (
    SupportInterval,
    StabilityMargin,
    sagittal_stability_margin,
    centering_shift,
)


FEET = {"LR": 0.00, "RR": 0.00, "LF": 0.20, "RF": 0.20}


# ------------------------------------------------------------ support interval
def test_interval_from_mapping_takes_extremes():
    s = SupportInterval.from_feet(FEET)
    assert s.rear == pytest.approx(0.0)
    assert s.front == pytest.approx(0.20)
    assert s.width == pytest.approx(0.20)
    assert s.center == pytest.approx(0.10)
    assert s.n_feet == 4
    assert s.feet == ("LF", "LR", "RF", "RR")   # sorted for stable reporting


def test_interval_from_bare_sequence():
    s = SupportInterval.from_feet([0.3, -0.1, 0.05])
    assert (s.rear, s.front) == (pytest.approx(-0.1), pytest.approx(0.3))
    assert s.n_feet == 3


def test_empty_interval_is_degenerate():
    s = SupportInterval.from_feet({})
    assert s.n_feet == 0
    assert s.width == 0.0
    assert np.isnan(s.center) and np.isnan(s.rear) and np.isnan(s.front)


# -------------------------------------------------------------------- margins
def test_centred_stance_is_stable_with_positive_margin():
    m = sagittal_stability_margin(0.10, FEET)
    assert m.is_stable
    assert m.margin == pytest.approx(0.10)
    assert m.margin_front == pytest.approx(0.10)
    assert m.margin_rear == pytest.approx(0.10)
    assert m.normalized_margin == pytest.approx(1.0)   # exactly centred
    assert "STABLE" in m.report()


def test_margin_is_distance_to_the_nearest_edge():
    m = sagittal_stability_margin(0.16, FEET)
    assert m.margin_front == pytest.approx(0.04)
    assert m.margin_rear == pytest.approx(0.16)
    assert m.margin == pytest.approx(0.04)
    assert m.tipping_edge == "front"
    assert m.is_stable
    assert m.normalized_margin == pytest.approx(0.04 / 0.10)


def test_pushing_the_com_beyond_the_front_foot_goes_unstable():
    inside = sagittal_stability_margin(0.19, FEET)
    outside = sagittal_stability_margin(0.21, FEET)
    assert inside.is_stable and not outside.is_stable
    assert outside.margin == pytest.approx(-0.01)
    assert outside.tipping_edge == "front"
    assert outside.normalized_margin < 0.0
    assert "UNSTABLE" in outside.report()


def test_pushing_the_com_behind_the_rear_foot_goes_unstable():
    m = sagittal_stability_margin(-0.05, FEET)
    assert not m.is_stable
    assert m.margin == pytest.approx(-0.05)
    assert m.tipping_edge == "rear"


def test_com_exactly_on_an_edge_is_not_strictly_stable():
    # Balanced on the tipping edge: zero margin, so not STRICTLY stable.
    m = sagittal_stability_margin(0.20, FEET)
    assert m.margin == pytest.approx(0.0)
    assert not m.is_stable


def test_no_stance_feet_is_never_stable():
    m = sagittal_stability_margin(0.10, {})
    assert not m.is_stable
    assert m.margin == float("-inf")
    assert m.tipping_edge == "none"
    assert "no stance feet" in m.report()


def test_single_contact_gives_a_zero_width_interval_and_is_never_stable():
    # In 2D a lone point contact (or all feet at one sagittal x) cannot resist
    # pitching, so no CoM position is strictly stable.
    m = sagittal_stability_margin(0.20, {"LF": 0.20})
    assert m.support.width == 0.0
    assert m.margin == pytest.approx(0.0)
    assert not m.is_stable
    assert np.isnan(m.normalized_margin)


def test_left_right_feet_collapse_onto_one_sagittal_x():
    # The documented 2D limitation: a 3-foot and a 4-foot stance can produce an
    # IDENTICAL fore-aft interval, which is exactly why this margin is necessary
    # but not sufficient.
    four = sagittal_stability_margin(0.10, FEET)
    three = sagittal_stability_margin(0.10, {"LR": 0.0, "RR": 0.0, "LF": 0.20})
    assert (four.support.rear, four.support.front) == (
        pytest.approx(three.support.rear), pytest.approx(three.support.front)
    )
    assert four.margin == pytest.approx(three.margin)


@pytest.mark.parametrize("shift", [-1.5, -0.03, 0.0, 0.42, 7.0])
def test_margin_is_translation_invariant(shift):
    base = sagittal_stability_margin(0.13, FEET)
    moved = sagittal_stability_margin(
        0.13 + shift, {k: v + shift for k, v in FEET.items()}
    )
    assert moved.margin == pytest.approx(base.margin)
    assert moved.is_stable == base.is_stable


def test_centering_shift_reports_the_correction_needed():
    m = sagittal_stability_margin(0.02, FEET)          # too far rearward
    d = centering_shift(m)
    assert d == pytest.approx(0.08)                     # move CoM +80 mm forward
    fixed = sagittal_stability_margin(m.com_x + d, FEET)
    assert fixed.normalized_margin == pytest.approx(1.0)
    assert np.isnan(centering_shift(sagittal_stability_margin(0.0, {})))


def test_margin_dataclass_is_constructible_directly():
    m = StabilityMargin(com_x=0.05, support=SupportInterval(0.0, 0.10, ("a", "b")))
    assert m.is_stable and m.margin == pytest.approx(0.05)
