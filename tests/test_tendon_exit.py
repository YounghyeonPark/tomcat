# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The leg drive, motor to anchor, in 3-D -- M122, ADR-0112; the knee and
ankle along the hip axis -- M123, ADR-0114."""
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
        to_y = None if t == "hip" else TE.LEAD_Y
        assert TE.spool_lead(role, m, e, pf, sf, to_y=to_y) is not None
        assert TE.spool_lead(role, m, e, pe, -sf, to_y=to_y) is not None


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


def test_every_conduit_bends_gently_and_stays_where_it_may(role, runs):
    """The hip pair's conduits outside the body; the knee and ankle pairs'
    inside the stub's and the hub's bores (M123) -- through the hip range,
    since they twist with it."""
    for (t, sd), r in runs.items():
        assert r["conduit_rmin"] >= TE.CONDUIT_RMIN - 1e-6, (t, sd, r["conduit_rmin"])
        if t == "hip":
            assert TE._outside_shell(r["conduit"][4:]), (t, sd)
        else:
            assert TE.in_bore(role, r["conduit"]), (t, sd)
    for t in ("knee", "ankle"):
        m, e, sf, pf, pe, _ha = TE._spec(role, t)
        for sd, sense, phi in ((+1, sf, pf), (-1, -sf, pe)):
            l3 = TE.spool_lead(role, m, e, phi, sense, to_y=TE.LEAD_Y)
            w, r = TE.conduit_worst(role, t, sd, l3, None)
            assert r >= TE.CONDUIT_RMIN - 1e-6 and math.isfinite(w), (t, sd, r)


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


def test_the_knee_and_ankle_leads_run_ALONG_THE_HIP_AXIS(role, runs):
    """✅ M123 (ADR-0114): their spools face across the hip at hip height, so
    both leads leave the top and bottom of the spool running straight out
    along +y, `HIP_SPOOL_DX` either side of the hip and 8.75 mm above and
    below it -- inside the stub's bore from the start."""
    xh = TE.HIP_X[role]
    for t in ("knee", "ankle"):
        for sd in (+1, -1):
            T, F1 = runs[(t, sd)]["lead"]
            d = (F1 - T) / np.linalg.norm(F1 - T)
            assert d[1] == pytest.approx(1.0, abs=1e-9)
            assert abs(T[0] - xh) == pytest.approx(TT.HIP_SPOOL_DX, abs=1e-6)
            assert abs(T[2]) == pytest.approx(TE.LT.SPOOL_R, abs=0.05)


def test_the_M122_DEFECT_is_gone_no_conduit_crosses_the_femur_or_the_hip_sheave(role):
    """ADR-0112 shipped every knee and ankle conduit through the hip sheave,
    the hip's boss and the femur's root; ADR-0113 measured that no routing AT
    the hip could fix it. The hollow hip (ADR-0114) keeps them inside the hub
    until their own planes: outside the hub's rim, over the whole hip range,
    no conduit comes within `LEG_MARGIN` of the femur, and none is within the
    hip sheave's slab."""
    import tomcat_leg_detail as LD
    xh = TE.HIP_X[role]
    y_sheave = TE.plane_y("hip")
    half = LD.FLANGE_W + LD.CABLE_D * 1.15 / 2
    for q in TE.hip_range(role):
        for k, r in TE.drive(role, q=q).items():
            if k[0] == "hip":
                continue
            C = r["conduit"]
            rr = np.hypot(C[:, 0] - xh, C[:, 2])
            out = C[rr > LD.HUB_OD / 2]
            assert TE._leg_clear(role, out, q, skip_end=0, parts=(4,)), (k, q)
            # the sheave is an annulus ON the hub: the conduits pass its plane
            # inside the hub's bore, never beside it
            near = ((np.abs(C[:, 1] - y_sheave) < half + TE.CONDUIT_OD / 2 + 0.5)
                    & (rr > LD.HUB_OD / 2 - TE.CONDUIT_OD / 2))
            assert not near.any(), (k, float(np.abs(C[:, 1] - y_sheave).min()))
