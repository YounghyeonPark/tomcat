"""M128 / ADR-0119, M129 / ADR-0120: the cat's skin -- a plush knit suit
over the whole skeleton.

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


def test_the_suit_is_OPEN_at_the_soles():
    p, _q = CS.leg_points(False)
    assert p[3][1] < CS.SOLE_Z < p[2][1], "the pads show; the ankle is covered"


def test_the_HEAD_is_a_cat_s_not_the_skull_s():
    """✅ M129 (ADR-0120): the head is drawn as a cat's head and the skull only
    has to fit inside it.

    ⚠️ Two faces drawn OVER the traced skull -- a dry skull with no mandible --
    came out too small and the wrong shape (one round, one a narrow wedge),
    because a cover follows what is under it. In the head the structure's
    offset is faded out and `FACE` alone makes the shape: bigger than the
    skull all round, widest at the cheeks, a small muzzle and chin, big ears."""
    import tomcat_head as HD
    st = HD._stations()
    skull = [r for r in st if r[0] >= HD.NOSE_X - 0.85 * HD.HEAD_L]
    under = min(r[2] for r in skull)
    wide = max(r[3] for r in skull)
    face = {nm: (np.asarray(c), np.asarray(r)) for nm, c, r in CS.FACE}
    hc, hr = face["head"]
    assert hr[1] > wide + 15.0, "the head is well wider than the skull"
    assert hc[2] + hr[2] > HD.cranium_top() + 25.0, "a round forehead above it"
    c, r = face["chin"]
    assert c[2] - r[2] < under, "a chin below the bone"
    cheek = face["cheek"][0][1] + face["cheek"][1][1]
    assert face["chin"][1][1] < face["muzzle"][1][1] < hr[1] < cheek, (
        "widest at the cheeks, a small muzzle and a smaller chin")
    base, tip, rb, _rt = CS.EAR
    assert tip[2] - base[2] > 50.0 and 2 * rb > 45.0, "big ears"
    assert CS.HEAD_FREE_X > HD.BASE_X, "the skull stops shaping it only in the head"


def test_the_EYES_look_FORWARD():
    p0, d0 = CS.EYE
    d = np.asarray(d0) / np.linalg.norm(d0)
    assert d[0] > 0.95, "a cat's eyes face forward, not out to the side"
    dz, wy, hz = CS.EYE_R
    assert wy > hz > dz, "an almond, wider than tall"


def test_the_foam_is_a_SHELL_and_silicone_is_not_an_option():
    """The stuffed cat was priced and rejected: the foam is `PAD` under the
    knit with air inside, and only the face is solid. At silicone foam's
    density even the shell would weigh most of a kilogram."""
    assert CS.PAD == pytest.approx(6.0)
    assert CS.FOAM_RHO["PU foam"] < 50.0 < CS.FOAM_RHO["silicone foam"]


def test_every_JOINT_needs_GATHERED_fabric():
    """✅ The finding: no joint's ROM fits a knit panel taut. A fibre on the
    outside of a joint travels R * dtheta, and with the sheaves setting R at
    31-45 mm with the foam (M129), even a trot asks 26-64 % of the fabric round it. So every joint
    carries gathered slack, as the belly does (ADR-0094) and as a cat does."""
    rows = CS.joint_strain()
    assert len(rows) == 6
    for leg, jn, R, span, dt, e, dr, slack in rows:
        assert slack > 50.0, (leg, jn, slack)
        assert e > CS.KNIT_WORKING, (leg, jn, e)
    hind_hip = [r for r in rows if r[0] == "hind" and r[1] == "hip"][0]
    # M129: 0.554 -> 0.638, the foam puts the knit 6 mm further out
    assert hind_hip[5] == pytest.approx(0.638, abs=0.02)


def test_the_FACE_is_FIRM_where_it_holds_its_shape():
    """✅ M129 (ADR-0122): the face's outer `FIRM_DEPTH` is a firm foam, the
    rest of the skin soft. The chin, muzzle and cheeks stand out of the head
    with nothing behind them, and the knit stretched over them squashes a soft
    foam; it is the outer layer that holds a shape, so only that is firm --
    132 g of foam and +15.0 % of the body's pitch inertia, where a face firm
    all through is 152 g and +18.4 %."""
    name, rho = CS.FOAM_FIRM
    assert 45.0 < rho < 70.0, "a high-resilience foam, not a solid"
    assert rho > CS.FOAM_RHO["PU foam"]
    assert 5.0 <= CS.FIRM_DEPTH <= 15.0, "a skin, not the whole face"


def test_the_SKIN_is_IN_the_plant():
    """✅ M130 (ADR-0124): the skin's mass, dealt to the bodies it rides on
    (`skin_mass.json`), is in `params.py` -- the front girdle carries the head's
    foam, each leg its sleeves -- and the body total is the sum."""
    import skin_mass as SM
    from tomcat_kin.params import DEFAULT_BODY_MASS_KG, DEFAULT_FORELEG, DEFAULT_HINDLEG
    data = SM.load()
    total = sum(v["mass"] for k, v in data.items() if "." not in k) \
        + 2 * sum(v["mass"] for k, v in data.items() if "." in k)
    assert total == pytest.approx(0.2443, abs=0.003)
    assert data["front"]["mass"] > data["rear"]["mass"], "the head's foam is the most"
    for role, lg in (("hind", DEFAULT_HINDLEG), ("fore", DEFAULT_FORELEG)):
        assert sum(SM.leg_link_masses(role)) < 0.2 * sum(lg.link_mass)
    assert DEFAULT_BODY_MASS_KG == pytest.approx(4.81358, abs=1e-5)
