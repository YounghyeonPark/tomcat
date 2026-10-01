"""M128 / ADR-0119: the cat's skin, one knit suit over the whole skeleton.

The full build -- every surface of the robot sampled, a field on a 1.25 mm grid,
the clearance measured on the mesh -- takes minutes and ~7 GB, so it runs as
`python mechanical/cad/cat_suit.py`, which prints its checks. These pin the
design rules it is built from.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest

pytest.importorskip("build123d", reason="build123d is an optional dependency")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mechanical", "cad"))

import cat_suit as CS  # noqa: E402
import tomcat_skin as SK  # noqa: E402
import tomcat_trunk as TT  # noqa: E402
from tomcat_kin.params import DEFAULT_TENDON  # noqa: E402


def test_the_cat_is_never_INSIDE_the_trunk_cover():
    """The cat's sections are lower bounds on ADR-0094's cover, which clears
    the structure: the cat may be bigger, never smaller."""
    xs = np.linspace(TT.BODIES[0][0], TT.BODIES[3][1], 80)
    a, top, bot = CS.section(xs)
    for x, ai, ti, bi in zip(xs, a, top, bot):
        hw, t, b = SK.outline(float(x))
        assert ai >= hw + 1.0 - 1e-9 and ti >= t + 1.0 - 1e-9 and bi <= b - 1.0 + 1e-9


def test_the_cat_has_a_CHEST_a_WAIST_and_HAUNCHES():
    a, top, bot = CS.section(np.array([-55.0, 90.0, 230.0]))
    haunch, waist, chest = bot
    assert chest < haunch < waist, "deepest at the chest, tucked at the waist"
    assert a[0] > a[1] and a[2] > a[1], "wider at the haunch and shoulder"


def test_each_sleeve_SPANS_THE_SHEAVES_inside_it():
    """⚠️ A cat's 24 mm thigh vanished inside the structure's offset: the
    sheaves are 36 / 34 / 22 mm and the cables run on their rims."""
    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm) * 1e3
    for fore in (False, True):
        sl = CS.sleeves(fore)
        for j in range(3):
            assert sl[j][2] >= arms[j] + SK.CLEAR, "proximal end clears its sheave"
        for j in range(2):
            assert sl[j][3] >= arms[j + 1] + SK.CLEAR, "distal end clears the next"


def test_the_suit_is_OPEN_at_the_soles_and_the_collar():
    p, _q = CS.leg_points(False)
    assert p[3][1] < CS.SOLE_Z < p[2][1], "the pads show; the ankle is covered"
    assert CS.COLLAR[0] > TT.BODIES[3][1], "the collar is up the neck, past the trunk"


def test_every_JOINT_needs_GATHERED_fabric():
    """✅ The finding: no joint's ROM fits a knit panel taut. A fibre on the
    outside of a joint travels R * dtheta, and with the sheaves setting R at
    25-39 mm, even a trot asks 21-55 % of the fabric round it. So every joint
    carries gathered slack, as the belly does (ADR-0094) and as a cat does."""
    rows = CS.joint_strain()
    assert len(rows) == 6
    for leg, jn, R, span, dt, e, dr, slack in rows:
        assert slack > 50.0, (leg, jn, slack)
        assert e > CS.KNIT_WORKING, (leg, jn, e)
    hind_hip = [r for r in rows if r[0] == "hind" and r[1] == "hip"][0]
    assert hind_hip[5] == pytest.approx(0.554, abs=0.02)
