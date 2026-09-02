# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""M84: the CAD→rigid-body split, and the arithmetic that combines them.

⚠️ These pin the *machinery*, not the conclusions drawn from it. The conclusions
from the first pass were mostly wrong — see `body_inertia`'s docstring — and the
common cause was using a number without checking what it measured.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pytest

pytest.importorskip("build123d", reason="build123d is an optional dependency")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "mechanical", "cad"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "kinematics", "src"))

import mujoco                                    # noqa: E402
from build123d import Box, Pos                   # noqa: E402
import body_inertia as BI                        # noqa: E402
import rigid_bodies as RB                        # noqa: E402
from tomcat_kin import mjcf_tendon as MT         # noqa: E402


def test_the_CAD_SPLITS_INTO_EXACTLY_THE_MJCF_BODIES():
    """✅ **21 rigid bodies, and the names line up with the simulator's.**

    `tomcat_skeleton.build()` returns four *render* groups — `bone`, `flat`,
    `joint`, `bay` — because it draws a picture. A simulator needs one solid per
    independently-moving body. The split is a regrouping rather than a
    re-modelling: `limb()` already assembles four bones in a list before
    `Compound`-ing them, and `spine_detail()` builds every vertebra separately
    and knows where the three actuated joints sit.

    ⚠️ The joint-axis markers are dropped on purpose. They are drawing aids, and
    giving them mass would be inventing it.
    """
    bodies, meta = RB.build_bodies()
    want = {"trunk", "spine1", "spine2", "spine3", "front_girdle"} | {
        f"{leg}_{link}" for leg in ("LF", "RF", "LR", "RR")
        for link in ("femur", "tibia", "meta", "paw")}
    assert set(bodies) == want, (
        f"extra {sorted(set(bodies) - want)}, missing {sorted(want - set(bodies))}"
    )

    m = mujoco.MjModel.from_xml_string(
        MT.quadruped_rig(hip_height=0.176, spine=True))
    sim = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)
           for i in range(m.nbody)} - {"world"}
    assert want <= sim, f"MJCF lacks {sorted(want - sim)}"

    # ✅ left and right must be mirror images, which catches a mis-sided limb
    for link in ("femur", "tibia", "meta", "paw"):
        left = BI.solid_props(bodies[f"LF_{link}"], 1000.0)
        right = BI.solid_props(bodies[f"RF_{link}"], 1000.0)
        assert left[0] == pytest.approx(right[0], rel=1e-9)
        assert left[1][1] == pytest.approx(-right[1][1], abs=1e-9), (
            f"{link}: left and right must sit on opposite sides"
        )


def test_COMBINE_uses_the_PARALLEL_AXIS_THEOREM_correctly():
    """✅ **Merging solids needs the shift; reading one solid does not.**

    `mass_properties.properties` deliberately applies **no** parallel-axis term:
    OCP already reports each solid's tensor about its own centre of mass, and a
    first version that shifted anyway returned **negative** inertia for anything
    off the origin. `combine` is the opposite case — several solids becoming one
    body — where the shift is required.

    Both are checked against answers that are known in closed form.
    """
    # two point masses at ±100 mm: I_yy = I_zz = 2·m·d², I_xx = 0
    zero = np.zeros((3, 3))
    M, com, I = BI.combine([(1.0, np.array([0.1, 0.0, 0.0]), zero),
                            (1.0, np.array([-0.1, 0.0, 0.0]), zero)])
    assert M == pytest.approx(2.0)
    assert np.allclose(com, 0.0, atol=1e-12)
    assert I[0, 0] == pytest.approx(0.0, abs=1e-12)
    assert I[1, 1] == pytest.approx(0.02, rel=1e-9)
    assert I[2, 2] == pytest.approx(0.02, rel=1e-9)

    # ✅ and a solid cut in half must reassemble into itself
    whole = BI.solid_props(Box(60, 40, 20), 2700.0)
    halves = BI.combine([BI.solid_props(Pos(-15, 0, 0) * Box(30, 40, 20), 2700.0),
                         BI.solid_props(Pos(15, 0, 0) * Box(30, 40, 20), 2700.0)])
    assert halves[0] == pytest.approx(whole[0], rel=1e-12)
    assert np.allclose(np.diag(halves[2]), np.diag(whole[2]), rtol=1e-9)


def test_THE_GIRDLE_DENSITY_IS_SANE_AGAINST_ITS_OWN_GEOMETRY():
    """⚠️ **The claim this test exists to stop being made again.**

    A first pass divided MJCF girdle masses by *skeleton* CAD volumes and
    reported 21 800 and 26 800 kg/m³ — denser than tungsten — as evidence the
    mass distribution was impossible. The denominator was wrong: the skeleton's
    `trunk` is pelvis blades and a tail, while the MJCF's `trunk` is a
    60 × 60 × 56 mm girdle **box**. Same name, different object.

    | girdle | MJCF box | mass | density |
    |---|---|---|---|
    | rear (`trunk`) | 201.6 cm³ | 902 g | **4 474** |
    | front | 201.6 cm³ | 1122 g | **5 565** |

    ✅ Which is what a box packed with motors should look like: six per girdle
    at Ø34.5 × 36.1 mm is **202.5 cm³** — the whole box — and **790 g**, and the
    motor's own back-solved density is 3 903 kg/m³.
    """
    m = mujoco.MjModel.from_xml_string(
        MT.quadruped_rig(hip_height=0.176, spine=True))
    for geom, body in (("rear_girdle_g", "trunk"),
                       ("front_girdle_g", "front_girdle")):
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, geom)
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
        vol = 8.0 * float(np.prod(m.geom_size[g][:3]))
        rho = float(m.body_mass[b]) / vol
        assert vol == pytest.approx(2.016e-4, rel=1e-3), "60 x 60 x 56 mm"
        assert 3000.0 < rho < 7000.0, (
            f"{body}: {rho:.0f} kg/m3 against its OWN geometry"
        )

    # ✅ and the motors alone account for it: six fill the box
    motor_vol = 6 * np.pi * (0.0345 / 2) ** 2 * 0.0361
    assert motor_vol == pytest.approx(2.025e-4, rel=1e-2)
    assert 6 * 0.1317 / motor_vol == pytest.approx(3903.0, rel=0.01)
