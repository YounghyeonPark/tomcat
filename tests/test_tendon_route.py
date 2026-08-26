# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The tendon drive, routed and measured (M37) — its findings, gated.

⚠️ The headline is a **correction to `TendonMap`**: the physical routing couples
the joints and the model's map is diagonal. These tests assert the coupling that
exists, so they fail if either the routing or the tendon map changes — which is
the point.

`build123d` is not needed here (this is geometry, not solids), but the module
lives with the CAD, so the path is added rather than imported as a package.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mechanical", "cad"))
import leg_tendons as LT  # noqa: E402
import tendon_route as tr  # noqa: E402

from tomcat_kin.params import DEFAULT_TENDON  # noqa: E402
from tomcat_kin.tendon import TendonMap  # noqa: E402


# ===================================================================
# the routing maths, against closed forms
# ===================================================================

@pytest.mark.parametrize("r1,r2,s2,expect", [
    (10.0, 10.0, +1, 100.0),                          # open belt, equal radii
    (10.0, 10.0, -1, math.sqrt(100 ** 2 - 20 ** 2)),  # crossed belt
    (28.0, 8.75, +1, math.sqrt(100 ** 2 - (28 - 8.75) ** 2)),
    (28.0, 8.75, -1, math.sqrt(100 ** 2 - (28 + 8.75) ** 2)),
])
def test_the_common_tangent_matches_the_closed_form(r1, r2, s2, expect):
    """The sign in `n.(c2-c1) = R1-R2` is easy to get backwards, and a first pass
    here did — the crossed belt read 105.83 mm where the closed form is 97.98."""
    p1, p2, _ = tr.tangent((0.0, 0.0), r1, +1, (100.0, 0.0), r2, s2)
    assert float(np.linalg.norm(p2 - p1)) == pytest.approx(expect, abs=1e-6)


def test_a_pulley_that_swallows_another_has_no_tangent():
    """Fail loudly rather than returning a nonsense path."""
    with pytest.raises(tr.NoTangent):
        tr.tangent((0.0, 0.0), 60.0, +1, (20.0, 0.0), 5.0, -1)


# ===================================================================
# M37 — what the routing does that the tendon map does not
# ===================================================================

@pytest.fixture(scope="module")
def coupling():
    return LT.coupling_matrix()


def test_the_routing_delivers_the_moment_arms_exactly(coupling):
    """The diagonal is the good news, and it is not a coincidence.

    `TendonParams.joint_moment_arm` is what `tomcat_kin` converts joint angle to
    cable travel with, and it is only true if `d(length)/d(theta) = r`. Measured
    off the routed geometry it is exact, because the cable leaves each sheave
    tangentially — so the sheave design and the torque budget agree.
    """
    J, _ = coupling
    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm) * 1e3
    for i in range(3):
        assert J[i, i] == pytest.approx(arms[i], abs=0.02)


def test_the_via_pulleys_COUPLE_the_joints_and_TendonMap_says_they_do_not(coupling):
    """⚠️ THE M37 finding.

    A distal tendon has to get past the proximal joints, and the standard fix is a
    via-pulley concentric with the proximal axis — the centre distance to the next
    joint is then the link length, which does not change when the proximal joint
    moves. That kills the *tangent* term. It does not kill the *arc* term: the wrap
    on the via-pulley changes with the proximal angle, and an arc on a pulley of
    radius `r_via` contributes `r_via` per radian.

    Measured, the off-diagonals are **exactly ±r_via = 8.75 mm/rad**:

        [[ 28.00    0       0   ]
         [  8.75   25.00    0   ]
         [ -8.75   -8.75   14.00]]

    As a fraction of each tendon's own arm that is **35 %** for the knee and
    **62.5 %** for the ankle, twice. `TendonMap.cable_lengths` is `delta = r * q`
    — a diagonal map — so none of it is modelled.

    ⚠️ And it cannot be designed away by shrinking the via-pulley: 8.75 mm is the
    cable's own minimum bend radius (10 x Ø1.75), the same rule that forced the
    spool from 8.0 to 8.75 mm in LEG_TENDON_SPEC §2.
    """
    J, q0 = coupling
    assert abs(J[1, 0]) == pytest.approx(LT.VIA_R, abs=0.02)
    assert abs(J[2, 0]) == pytest.approx(LT.VIA_R, abs=0.02)
    assert abs(J[2, 1]) == pytest.approx(LT.VIA_R, abs=0.02)
    assert J[0, 1] == pytest.approx(0.0, abs=0.02), "hip is proximal-most"
    assert J[0, 2] == pytest.approx(0.0, abs=0.02)

    # and the model disagrees
    tm = TendonMap(DEFAULT_TENDON)
    d = tm.cable_lengths(q0)
    assert d.shape == (3, 2)
    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm)
    assert d[:, 0] == pytest.approx(-arms * np.asarray(q0)), (
        "if cable_lengths has stopped being diagonal, M37's correction landed and "
        "this test should be replaced by one asserting the coupled map"
    )


def test_the_cross_coupling_eats_the_cable_travel_budget(coupling):
    """LEG_TENDON_SPEC §1.4 sized spool travel as `r x ROM` **per joint**, giving
    117 / 65 / 44 mm and calling the hip "the sizing case".

    With the coupling included the real worst-case travels are **117 / 102 / 104 mm**
    — the knee understated by 56 %, the ankle by 135 %, and the ankle's parasitic
    travel (59.6 mm) larger than its own (44.0 mm).

    ⚠️ The hip *does* remain the sizing case, 117 against 104. A first draft of this
    test asserted the ankle overtook it, which is false and was caught here: the
    finding is that all three spools land in the same class, not that the ranking
    flips. §1.4 implies they differ by 2.7x; they differ by 13 %.
    """
    J, _ = coupling
    from tomcat_kin.params import DEFAULT_LEG

    rom = np.abs(np.asarray(DEFAULT_LEG.q_max) - np.asarray(DEFAULT_LEG.q_min))
    own = np.array([abs(J[i, i]) * rom[i] for i in range(3)])
    para = np.array([sum(abs(J[i, k]) * rom[k] for k in range(3) if k != i)
                     for i in range(3)])
    total = own + para

    assert para[0] == pytest.approx(0.0, abs=0.1)      # hip is proximal-most
    assert para[1] / own[1] > 0.5, "knee parasitic travel > 50 % of its own"
    assert para[2] / own[2] > 1.0, "ankle parasitic travel exceeds its own"
    assert own[0] == pytest.approx(total.max(), abs=0.5), "hip stays the sizing case"
    assert total.max() / total.min() < 1.25, (
        f"all three spools should now be one class, got {total.round(1)}"
    )
    assert own.max() / own.min() > 2.5, "which §1.4's per-joint numbers do not say"


def test_every_running_pulley_clears_the_cable_minimum_bend_radius():
    """§2: minimum sheave DIAMETER is 10x the cable, so Ø1.75 needs r >= 8.75 mm.

    ⚠️ `tomcat_leg_detail.idler` shipped at **5.0 mm** in its first pass — 43 %
    under the cable's own limit. The anchor pin is exempt: a spliced eye over a
    thimble is a static termination, not a running bend.
    """
    _, q0 = LT.coupling_matrix()
    for tendon in ("hip", "knee", "ankle"):
        res = tr.min_bend_check(LT.stations(q0, tendon), LT.CABLE_D)
        for ok, r, need in res:
            assert ok, f"{tendon}: r = {r:.2f} mm against a {need:.2f} mm minimum"


def test_the_capstan_penalty_is_SMALLER_than_the_spec_assumed():
    """§3.4 worked the ankle path out at **1.87x** from assumed wrap angles
    summing to 360 deg. Solved, the wraps sum to ~108 deg and the penalty is
    ~1.21x — the routing is *better* than the spec feared, and the motor-side
    tension margin it was inflating can come back.
    """
    _, q0 = LT.coupling_matrix()
    ankle = LT.route(q0, "ankle")
    assert ankle["capstan"] < 1.87
    assert ankle["capstan"] > 1.0
    assert math.degrees(ankle["total_wrap"]) < 360.0


def test_the_run_lengths_are_close_to_the_spec_estimates():
    """§3.3 estimated ~0.10 / 0.22 / 0.30 m. Solved: within ~35 % on every run,
    which matters because `cable_stiffness = EA/L` was computed at a single
    0.30 m and the spec itself said it should be per-tendon."""
    _, q0 = LT.coupling_matrix()
    for tendon, spec_mm in (("hip", 100.0), ("knee", 220.0), ("ankle", 300.0)):
        got = LT.route(q0, tendon)["length"]
        assert 0.6 * spec_mm < got < 1.45 * spec_mm, f"{tendon}: {got:.0f} mm"


def test_the_spool_radius_in_params_is_now_the_SPEC_value():
    """✅ **Closed by M41.** This test used to assert the defect — that `params.py`
    carried 0.008 m where LEG_TENDON_SPEC §2 requires **0.00875** for the Ø1.75 mm
    cable's minimum bend — and its failing is what signalled the fix had landed.

    It now guards the fix instead. The spine spool moved with it, for the same
    reason and on the same date it should have (ADR-0046).
    """
    from tomcat_kin.params import DEFAULT_SPINE

    assert float(DEFAULT_TENDON.motor_spool_radius) == pytest.approx(0.00875)
    assert float(DEFAULT_SPINE.motor_spool_radius) == pytest.approx(0.00875)
    assert 0.00875 == pytest.approx(10.0 * 1.75e-3 / 2.0), (
        "the value IS the cable's minimum bend radius, not a free choice"
    )

    assert LT.SPOOL_R == pytest.approx(8.75), "the CAD agreed all along"
