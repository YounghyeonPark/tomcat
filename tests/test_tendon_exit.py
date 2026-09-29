# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The leg drive, motor to anchor, in 3-D -- M122, ADR-0112."""
from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

pytest.importorskip("build123d", reason="the trunk's motors and shell need CAD")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mechanical", "cad"))

import tendon_exit as TE  # noqa: E402
import tomcat_trunk as TT  # noqa: E402
from tomcat_kin.params import DEFAULT_TENDON  # noqa: E402


@pytest.fixture(scope="module", params=["hind", "fore"])
def role(request):
    return request.param


@pytest.fixture(scope="module")
def runs(role):
    return TE.drive(role)


def test_every_cable_STARTS_on_a_spool_the_trunk_has(role, runs):
    """⚠️ The check this replaces compared the router's first point with the
    spool (x, z) it had been handed -- the same number twice -- while the real
    spool was 45-55 mm away inside the trunk on an axis along x."""
    own = TT.leg_spools(role, +1.0, by="tendon")
    for (t, _sd), r in runs.items():
        x, y, z = own[TE.TENDONS.index(t)]
        T, F1 = r["lead"]
        assert math.hypot(T[1] - y, T[2] - z) == pytest.approx(TE.LT.SPOOL_R, abs=1e-6)
        assert abs(T[0] - x) <= TT.row_len(role) / 2
        # the lead is in the spool's x-plane: no fleet on the spool
        assert F1[0] == pytest.approx(T[0], abs=1e-9)


def test_the_two_cables_of_a_spool_WIND_IT_OPPOSITE_WAYS(role):
    for t in TE.TENDONS:
        sf = TE.DRIVE[role][t][2]
        # the extensor leaves in -sf by construction; both leads exist
        m, e = TE.DRIVE[role][t][:2]
        pf, pe = (math.radians(a) for a in TE.DRIVE[role][t][3:5])
        assert TE.spool_lead(role, m, e, pf, sf) is not None
        assert TE.spool_lead(role, m, e, pe, -sf) is not None


def test_no_spool_sits_in_trunk_structure(role):
    """The spine joints' yokes and processes at the girdle ends: a spool at the
    wrong end of its row sat inside them (157-203 mm3 before ADR-0112 moved
    joint 0; the fore girdle's front row still has one such end)."""
    from build123d import Compound  # noqa: F401
    import tomcat_leg_detail as LD
    import tomcat_packaging as TP
    body = TT.rigid_body(TT.LEG_BODY[role])
    bad = set()
    for side in (+1.0, -1.0):
        for m, (_nm, x, y, z) in enumerate(TE.motors(role, side)):
            for end in (+1, -1):
                xs = TE.spool_x_of(role, m, end, side)
                sp = LD.xaxis_can(xs - TP.SPOOL_L / 2, xs + TP.SPOOL_L / 2, y, z,
                                  TE.LT.SPOOL_R + LD.GROOVE_R + LD.FLANGE_H)
                inter = body.intersect(sp)
                v = 0.0
                if inter is not None:
                    try:
                        v = sum(s.volume for s in inter.solids())
                    except Exception:
                        v = 0.0
                if v > 0.5:
                    bad.add((m, end))
    assert bad == TE.FORBIDDEN[role]
    for t in TE.TENDONS:
        assert tuple(TE.DRIVE[role][t][:2]) not in bad


def test_every_conduit_bends_gently_and_stays_outside_the_body(role, runs):
    for (t, sd), r in runs.items():
        assert r["conduit_rmin"] >= TE.CONDUIT_RMIN - 1e-6, (t, sd, r["conduit_rmin"])
        assert TE._outside_shell(r["conduit"][4:]), (t, sd)
    # and through the hip range, the ones that ride the femur
    for t in ("knee", "ankle"):
        m, e, sf, pf, pe, _ha = TE._spec(role, t)
        for sd, sense, phi in ((+1, sf, pf), (-1, -sf, pe)):
            l3 = TE.spool_lead(role, m, e, phi, sense)
            w, r = TE.conduit_worst(role, t, sd, l3, None)
            assert r >= TE.CONDUIT_RMIN - 1e-6 and math.isfinite(w)


def test_the_hip_pair_ferrules_are_apart_and_outside_the_body(role, runs):
    a, b = TE.DRIVE[role]["hip"][5]
    assert abs(a - b) >= math.degrees(TE.HIP_F2_SEP) - 1e-9
    shell = TT._outer(TT.LEG_BODY[role])
    for sd in (+1, -1):
        assert not shell.is_inside(tuple(runs[("hip", sd)]["F2"]))


def test_the_HIP_no_longer_drives_the_knee_and_ankle_cables(role):
    """The hip via coupled 8.75 mm/rad into both; in a conduit whose far end
    rides the femur, the free run is fixed in the femur's frame."""
    q0 = TE.stance(role)
    lo, hi = TE.leg(role).q_min[0], TE.leg(role).q_max[0]
    for t in ("knee", "ankle"):
        for sd in (+1, -1):
            base = TE.leg_run(role, t, sd, q0)["length"]
            for h in (lo * 0.9, hi * 0.9):
                q = q0.copy()
                q[0] = h
                r = TE.leg_run(role, t, sd, q, senses=None)
                assert r["length"] == pytest.approx(base, abs=1e-6)


def test_the_right_leg_mirrors_the_left(role):
    left, right = TE.drive(role, +1.0), TE.drive(role, -1.0)
    for k in left:
        assert right[k]["conduit_bend"] == pytest.approx(left[k]["conduit_bend"], abs=1e-9)
        assert right[k]["F1"][1] == pytest.approx(-left[k]["F1"][1], abs=1e-6)


def test_params_carry_the_hind_drive():
    assert np.allclose(DEFAULT_TENDON.conduit_bend, TE.conduit_bend("hind"), atol=1e-3)
    assert tuple(DEFAULT_TENDON.pulley_count) == TE.pulley_count("hind")
    assert DEFAULT_TENDON.friction_model == "drive"


def test_friction_is_now_a_TENTH_to_a_FIFTH_not_a_half():
    """⚠️ M107-M121 priced a capstan on every degree of wrap -- on sheaves the
    cable is anchored to and on pulleys that turn on bearings -- up to x1.56.
    Sliding happens in the conduit; x1.12-1.22 with one pulley's loss on the
    ankle pair."""
    import dataclasses
    from tomcat_kin.tendon import TendonMap
    now = np.r_[TendonMap(DEFAULT_TENDON).capstan_factor(side=+1),
                TendonMap(DEFAULT_TENDON).capstan_factor(side=-1)]
    old = TendonMap(dataclasses.replace(DEFAULT_TENDON, friction_model="capstan"))
    was = np.r_[old.capstan_factor(side=+1), old.capstan_factor(side=-1)]
    assert now.min() > 1.10 and now.max() < 1.25
    assert was.max() > 1.5


def test_KNOWN_DEFECT_the_femur_conduits_cut_the_hip_sheave(role, runs):
    """⚠️ Asserts the defect (ADR-0112). Reaching a femur ferrule just past the
    hip sheave's rim, every knee and ankle conduit crosses the sheave's plane
    inside the rim. Between the trunk wall and the femur the hip's boss, the
    femur's root and the sheave leave no path at all -- a hip packaging
    problem. ADR-0113's hollow hip could not be fed from the trunk; the fix
    moves the knee and ankle spools (`[owed]`). This fails when it lands."""
    q0 = TE.stance(role)
    cut = [k for k, r in runs.items()
           if k[0] != "hip" and not TE._leg_clear(role, r["conduit"], q0, skip_end=4, parts=(0,))]
    assert len(cut) == 4, cut
