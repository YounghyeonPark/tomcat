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


# ------------------------------------------------ 3D: the TRUE support polygon
def test_polygon_margin_basic_geometry():
    from tomcat_kin.stability import polygon_stability_margin
    sq = {"a": (0, 0), "b": (1, 0), "c": (1, 1), "d": (0, 1)}
    assert polygon_stability_margin((0.5, 0.5), sq).margin == pytest.approx(0.5)
    out = polygon_stability_margin((1.5, 0.5), sq)
    assert out.margin == pytest.approx(-0.5)
    assert not out.is_stable


def test_polygon_needs_at_least_three_feet():
    from tomcat_kin.stability import polygon_stability_margin
    with pytest.raises(ValueError):
        polygon_stability_margin((0, 0), {"a": (0, 0), "b": (1, 0)})


def test_feet_sit_at_their_real_lateral_track_offsets():
    # 3D GEOMETRY (no new DOF): left legs +y, right legs -y.
    from tomcat_kin import GaitController
    c = GaitController()
    st = c.state(0.0)
    xy = c.body.foot_ground_xy(st.spine_q, {n: l.q for n, l in st.legs.items()})
    assert xy["LF"][1] > 0 and xy["LR"][1] > 0
    assert xy["RF"][1] < 0 and xy["RR"][1] < 0
    assert xy["LF"][1] == pytest.approx(-xy["RF"][1])


def test_default_walk_is_LATERALLY_unstable_despite_a_positive_fore_aft_margin():
    # THE 3D FINDING. The sagittal interval says stable at every phase; the real
    # support polygon says otherwise, because with three feet down the triangle
    # is skewed and the mid-sagittal CoM falls outside it. Recorded as a fact
    # about the current 16-motor build, not fudged away.
    from tomcat_kin import GaitController
    c = GaitController()
    assert all(m.is_stable for m in c.stability_sweep(48))        # 2D says fine
    poly = c.support_polygon_sweep(48)
    assert any(not p.is_stable for p in poly)                     # 3D disagrees
    assert min(p.margin for p in poly) < -0.02                    # by > 20 mm


def test_lateral_body_sway_recovers_polygon_stability():
    # Sway toward the support side is what a real cat does in a crawl. It needs
    # a lateral DOF the 16-motor build lacks (ADR-0006/0008), so it is modelled
    # as a parameter rather than actuated.
    from tomcat_kin import GaitController
    c = GaitController()
    assert min(p.margin for p in c.support_polygon_sweep(48, lateral_shift=0.040)) > 0.0
