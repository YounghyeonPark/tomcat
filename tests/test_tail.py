# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The tail (M96) -- a motor in the budget with nothing on the end of it.

⚠️ ADR decision F bought **19** motors (12 leg + 3 spine pitch + 3 spine yaw +
1 TAIL) and M88's redistribution placed **18**. The tail's motor fell out, no
count caught it, and the tail itself had no length, no mass and no section
anywhere in the model.

✅ The useful result is not the geometry, it is the account: the tail adds
**11.7 %** to the body's pitch inertia and its curl modulates **0.8 %**. It
costs fourteen times what it can use, at any density -- so it is not an inertial
device, which is consistent with G6 being withdrawn (ADR-0071) and ADR-0007
calling it coarse. Only its MASS is a design choice.

`build123d` is an optional dependency; the module skips without it.
"""

from __future__ import annotations

import os
import sys

import pytest

pytest.importorskip("build123d", reason="build123d is an optional dependency")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mechanical", "cad"))
import tomcat_tail as TL  # noqa: E402
import tomcat_trunk as TT  # noqa: E402


def test_the_NINETEENTH_motor_is_placed():
    """⚠️ Asserts the count that nothing was making.

    `params.py` budgets a "7-motor spine+tail bank" and ADR decision F bought 19
    in total; the trunk packed 18 for five milestones. It does not need a new
    row -- body 0 is 104.6 mm and two rows use 88.2 -- but it does need a third
    POSITION, and the geometry allows exactly one.
    """
    placed = sum(len(TT.ROWS[n][0]) for (_a, _b, _c, n, _r) in TT._rows())
    assert placed == 19, "%d motors placed against the decision's 19" % placed
    assert ("tri3", 2) in TT.POSITION_ROLE
    assert TT.POSITION_ROLE[("tri3", 2)] == "tail"


def test_a_row_can_hold_more_than_one_ROLE():
    """⚠️ The drive-train check counted by ROW and read `tri3` as seven legs'
    worth on body 0, for a leg that needs six. Roles are per position."""
    roles = {}
    for (_nm, b, _x, n, role) in TT._rows():
        for k in range(len(TT.ROWS[n][0])):
            roles.setdefault(TT.POSITION_ROLE.get((n, k), role), []).append(b)
    assert len(roles["hind"]) == 6
    assert len(roles["fore"]) == 6
    assert len(roles["spine"]) == 6
    assert len(roles["tail"]) == 1
    assert roles["tail"] == [0], "the tail motor belongs to the pelvic girdle"


def test_the_tail_leaves_the_body_where_the_body_ENDS():
    """⚠️ SPINE_TAIL_SPEC mounts it at `x = 0`, which was the pelvic girdle's
    rear face when the trunk was two boxes. M88's rear girdle runs to x = -112
    and the hip sits at its FRONT, so a tail based at 0 grows **112 mm forward
    through its own body**."""
    assert TL.BASE_X == pytest.approx(TT.BODIES[0][0])
    assert TL.BASE_X < 0.0
    assert TL.BASE_Z == pytest.approx(TT.SPINE_Z), (
        "the caudal chain continues the vertebral one, so it leaves on its axis"
    )
    for curl in (0.0, 1.0):
        xs = [x for x, _z in TL.spine_curve(curl)]
        assert max(xs) <= TL.BASE_X + 1e-9, "the tail grows BACKWARD"


def test_the_curl_is_worth_a_fourteenth_of_what_the_tail_COSTS():
    """✅ THE result, and it does not depend on the material.

    The tail is all lever, so density scales cost and authority together: the
    ratio is invariant and the only design choice left is how light to make it.
    """
    com = (109.0, 15.0)
    for rho in (TL.RHO, 0.20e-3, 0.35e-3):
        i0 = TL.inertia_about(com, 0.0, rho)
        i1 = TL.inertia_about(com, 1.0, rho)
        ratio = i0 / abs(i0 - i1)
        assert ratio == pytest.approx(14.0, abs=1.5), (
            "cost/authority is %.0fx at rho=%.2e" % (ratio, rho)
        )
    i0 = TL.inertia_about(com, 0.0)
    # M122 (ADR-0112): 0.117 -> 0.154. The tail's base went 30 mm further back
    # with body 0, and the tail, sized to the trunk, grew with it.
    assert i0 / TL.BODY_IYY == pytest.approx(0.154, abs=0.01)


def test_it_neither_grows_through_the_trunk_nor_drags():
    from build123d import Compound
    inter = Compound([TT.rigid_body(0)]).intersect(TL.tail(0.0))
    v = 0.0
    if inter is not None:
        try:
            v = sum(x.volume for x in inter.solids())
        except Exception:
            v = 0.0
    assert v < 50.0, "the tail is %.0f mm3 inside the girdle" % v
    lo = min(s.bounding_box().min.Z for s in TL.tail(0.0).solids())
    assert 175.0 + lo > 20.0, "the tail drags at %.0f mm" % (175.0 + lo)
