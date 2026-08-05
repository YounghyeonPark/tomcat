# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
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


def test_walk_WITHOUT_lateral_sway_is_laterally_unstable():
    # THE 3D FINDING that motivated ADR-0009, kept as a REGRESSION GUARD. With the
    # sway switched off the sagittal interval still says "stable" at every phase,
    # but the real support polygon disagrees: with three feet down the triangle is
    # skewed and a mid-sagittal CoM falls outside it.
    from tomcat_kin import GaitController, GaitParams
    c = GaitController(params=GaitParams(lateral_amplitude=0.0))
    assert all(m.is_stable for m in c.stability_sweep(48))        # 2D says fine
    poly = c.support_polygon_sweep(48)
    assert any(not p.is_stable for p in poly)                     # 3D disagrees
    assert min(p.margin for p in poly) < -0.02                    # by > 20 mm


def test_default_walk_IS_laterally_stable_via_the_actuated_spine():
    # The M5 payoff: the sway is now produced by a REAL actuated lateral DOF
    # (ADR-0009), not assumed as a parameter, and it closes the 3D margin.
    from tomcat_kin import GaitController
    c = GaitController()
    poly = c.support_polygon_sweep(200)
    assert all(p.is_stable for p in poly)
    assert min(p.margin for p in poly) > 0.005                    # > 5 mm
    assert all(m.is_stable for m in c.stability_sweep(200))       # 2D still fine


def test_sway_is_a_ramped_square_wave_confined_to_four_foot_windows():
    # The law must hold full amplitude through each 3-foot phase and traverse ONLY
    # while all four feet are planted -- a sinusoid is near zero exactly at the
    # crossovers, which is where the margin is decided.
    from tomcat_kin import GaitController
    import numpy as np
    c = GaitController()
    amp = c.params.lateral_amplitude
    for i in range(200):
        p = i / 200
        q = c.lateral_q(p)
        assert np.allclose(q, q[0])                       # uniform per segment
        if c.stance_count(p) == 3:
            assert abs(abs(q[0]) - amp) < 1e-9            # saturated on 3 feet
        assert abs(q[0]) <= amp + 1e-12                   # never overshoots


def test_sway_bends_AWAY_from_the_swinging_leg():
    from tomcat_kin import GaitController
    c = GaitController()
    for i in range(200):
        p = i / 200
        st = c.state(p)
        if len(st.swing_legs) != 1:
            continue
        swing = st.swing_legs[0]
        # spine leans opposite to the lifted foot's side => toward the support
        assert c.lateral_q(p)[0] * c.body.mounts[swing].track_y < 0


def test_over_swaying_is_WORSE_than_the_optimum():
    # The amplitude is an OPTIMUM, not a maximum: too much sway carries the CoM
    # out over the FAR edge of the support triangle. Guards against someone
    # "improving" stability by turning the number up.
    from tomcat_kin import GaitController, GaitParams
    import numpy as np
    # NOTE the optimum moved 13.5 -> 12.5 -> 11 deg as the mass model and then the
    # DYNAMICS were corrected; above ~11 deg the ZMP degrades even though the
    # static margin keeps improving (see test_dynamics.py).
    best = min(p.margin for p in GaitController().support_polygon_sweep(200))
    for deg in (8.0, 18.0, 20.0):
        worse = GaitController(params=GaitParams(lateral_amplitude=np.radians(deg)))
        assert min(p.margin for p in worse.support_polygon_sweep(200)) < best


def test_sequencing_is_not_the_lever_for_lateral_stability():
    # Reordering the swings changes WHEN postures occur, not WHICH postures occur,
    # so across all 24 assignments of the offset SET {0,.25,.5,.75} the worst-case
    # margin barely moves (~1.5 mm on ~7 mm) and every one is stable. It is the
    # SWAY that fixes lateral stability, not the sequence -- retiring "just
    # re-sequence the gait" as a lever.
    from tomcat_kin import GaitController, GaitParams
    import itertools
    legs = ["LF", "RF", "RR", "LR"]
    worsts = []
    for perm in itertools.permutations([0.0, 0.25, 0.5, 0.75]):
        c = GaitController(params=GaitParams(phase_offsets=dict(zip(legs, perm))))
        worsts.append(min(p.margin for p in c.support_polygon_sweep(96)))
    assert all(w > 0.004 for w in worsts)          # every sequence is stable
    assert max(worsts) - min(worsts) < 0.008       # and they differ by < 8 mm

    # Without sway, NO sequence is stable -- the same sweep, one parameter changed.
    for perm in itertools.permutations([0.0, 0.25, 0.5, 0.75]):
        c = GaitController(params=GaitParams(phase_offsets=dict(zip(legs, perm)),
                                             lateral_amplitude=0.0))
        assert min(p.margin for p in c.support_polygon_sweep(96)) < 0.0


def test_lateral_slew_rate_is_reported_and_finite():
    # The sway imposes a real SPEED requirement on the spine drives; if it cannot
    # be met the walk is not stable however good the geometry looks.
    from tomcat_kin import GaitController, GaitParams
    import numpy as np
    c = GaitController()
    slew = np.degrees(c.lateral_slew_rate())
    assert 20.0 < slew < 45.0                       # ~29 deg/s at the 5.0 s crawl
    # duty 0.75 leaves NO four-foot window -> the traverse would be instantaneous
    tiled = GaitController(params=GaitParams(duty_factor=0.75))
    assert tiled.lateral_slew_rate() == float("inf")
    assert GaitController(params=GaitParams(lateral_amplitude=0.0)).lateral_slew_rate() == 0.0


def test_sway_stays_inside_the_spine_lateral_joint_limits():
    from tomcat_kin import GaitController, GaitParams
    import numpy as np
    sp = GaitController().body.spine.params
    greedy = GaitController(params=GaitParams(lateral_amplitude=np.radians(40)))
    for i in range(100):
        q = greedy.lateral_q(i / 100)
        assert np.all(q >= np.asarray(sp.lateral_q_min) - 1e-12)
        assert np.all(q <= np.asarray(sp.lateral_q_max) + 1e-12)


def test_crossover_stays_inside_the_friction_cone():
    # THE DYNAMIC REALITY CHECK. The quasi-static polygon margin is only
    # meaningful if the sway reversal is physically achievable: the paws can
    # deliver ~mu*g laterally before they slide. The 1.2 s / duty-0.80 gait the
    # sway was first tuned on demanded 9.1 g and was NOT achievable; the shipped
    # default trades period and duty to get inside the cone.
    from tomcat_kin import GaitController, GaitParams
    c = GaitController()
    assert c.crossover_is_feasible(mu=0.8)
    assert c.crossover_accel() < c.friction_accel_limit(0.8)
    assert c.crossover_window() > 0.15                    # > 150 ms to cross

    # And the guard bites: the original fast/tight gait is correctly rejected.
    fast = GaitController(params=GaitParams(period=1.2, duty_factor=0.80))
    assert not fast.crossover_is_feasible(mu=0.8)
    assert fast.crossover_accel() > 8 * 9.81              # ~9 g


def test_crossover_accel_scales_as_the_inverse_square_of_the_window():
    # Why walking faster is punished so hard: halving the window quadruples the
    # demand. This is the relationship that caps the statically stable walk speed.
    from tomcat_kin import GaitController, GaitParams
    slow = GaitController(params=GaitParams(period=2.8))
    fast = GaitController(params=GaitParams(period=1.4))
    assert fast.crossover_accel() == pytest.approx(4 * slow.crossover_accel(), rel=1e-9)


def test_no_sway_means_no_crossover_cost():
    from tomcat_kin import GaitController, GaitParams
    c = GaitController(params=GaitParams(lateral_amplitude=0.0))
    assert c.crossover_accel() == 0.0
    assert c.crossover_is_feasible()


# --------------------------------- lateral spine DRIVE sizing (ADR-0009 f/u)
def test_lateral_spine_loads_are_inertial_and_peak_at_the_base():
    # The lateral bend axis is VERTICAL, so gravity exerts no moment about it --
    # holding a sway is nearly free. What costs torque is REVERSING it. The base
    # joint is worst because it must swing the entire forequarters.
    from tomcat_kin import GaitController
    c = GaitController()
    loads = c.body.lateral_spine_loads(c.crossover_accel())
    assert len(loads) == 3
    tq = [r["joint_torque"] for r in loads]
    assert tq[0] > tq[1] > tq[2] > 0.0                 # monotonic, base worst
    assert loads[0]["distal_mass"] > loads[2]["distal_mass"]
    # Zero acceleration => zero load, i.e. gravity really is absent from this axis.
    assert all(r["joint_torque"] == 0.0 for r in c.body.lateral_spine_loads(0.0))


# The M6 walk is a 1.1 cm/s crawl, so its own sway barely loads the lateral drive
# (~0.10 N.m). Sizing the drive to THAT would under-build it: the same motors must
# also serve the ADR-0007 righting reflex and any future dynamic gait. So the
# lateral drive is sized against a REFERENCE FAST MANOEUVRE -- the M5-era 1.4 s
# crossover, retained purely as a sizing case.
REFERENCE_FAST_ACCEL = 6.87        # m/s^2, the 1.4 s crossover


def test_the_crawl_itself_barely_loads_the_lateral_drive():
    # Recorded so nobody "optimises" the lateral drive down to the crawl's needs.
    from tomcat_kin import GaitController
    c = GaitController()
    worst = max(r["motor_torque"] for r in c.body.lateral_spine_loads(c.crossover_accel()))
    assert worst < 0.2                        # trivial at 1.1 cm/s


def test_lateral_drive_fits_inside_the_real_motor_at_the_fast_case():
    # The surveyed part (GIM3505-9) peaks at 1.95 N.m at the output shaft.
    from tomcat_kin import GaitController
    c = GaitController()
    worst = max(r["motor_torque"]
                for r in c.body.lateral_spine_loads(REFERENCE_FAST_ACCEL))
    assert worst < 1.95                       # inside the REAL part's peak
    assert worst > 0.5                        # ...and genuinely loaded


def test_the_lateral_moment_arm_buys_margin_but_is_no_longer_load_critical():
    # HONEST DOWNGRADE. The 20 mm milled lateral post was justified against a
    # 1.10 N.m CLASS TARGET motor: at the bare 15 mm transverse-process width the
    # base joint exceeded it. The surveyed REAL part peaks at 1.95 N.m, so 15 mm
    # would now fit too. 20 mm is retained because it is nearly free and buys
    # ~25 % margin -- but it is an OPTIMISATION now, not a necessity.
    import dataclasses
    from tomcat_kin import GaitController
    from tomcat_kin.spine import WholeBody, SpineModel
    c = GaitController()
    bare = WholeBody(spine=SpineModel(
        dataclasses.replace(c.body.spine.params, lateral_moment_arm=(0.015,) * 3)))
    t15 = max(r["motor_torque"] for r in bare.lateral_spine_loads(REFERENCE_FAST_ACCEL))
    t20 = max(r["motor_torque"] for r in c.body.lateral_spine_loads(REFERENCE_FAST_ACCEL))
    assert t20 < t15                       # the post still helps
    assert t15 > 1.10                      # ...it WOULD have busted the class target
    assert t15 < 1.95                      # ...but the real part swallows it anyway
    assert t20 / t15 == pytest.approx(0.015 / 0.020, rel=1e-6)   # T = tau/r, exactly


def test_cable_tension_stays_inside_the_spec_band():
    # The lateral tendon is the same 1.5 mm UHMWPE as everything else
    # (LEG_TENDON_SPEC, ~465 N design load).
    from tomcat_kin import GaitController
    c = GaitController()
    assert max(r["cable_tension"]
               for r in c.body.lateral_spine_loads(c.crossover_accel())) < 465.0
