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


def test_the_mjcf_now_swings_like_the_cad_leg():
    """✅ **Closed in M86.** The capsules put leg swing inertia **45 % too high**;
    the MJCF now carries an explicit `<inertial>` per link and matches the CAD.

    `mjcf_tendon` draws each link as a capsule and lets MuJoCo derive the
    inertia at uniform density. `mass_closure.py` flags the consequence itself:

        `inertia_ratio` is used as a **first-order proxy** ... The real quantity
        is `Lambda = (J M^-1 J^T)^-1` ... which needs the per-link inertia
        tensors this ...

    Measured about the hip, in the same stance pose, on the same leg:

    | | mass | I_yy about hip |
    |---|---|---|
    | CAD, bearing ENVELOPES | 182.2 g | 1.244e-3 kg m× |
    | CAD, catalogue bearings | 167.2 g | **1.175e-3** |
    | MJCF capsules, before M86 | 168.2 g | **1.701e-3** = **+45 %** |
    | MJCF `<inertial>`, now | 167.2 g | **1.175e-3** = ratio **0.9999** |

    ⚠️ **ADR-0087 published 1.244e-3 and it was 6 % high.** `bearing()` says
    "envelope" in its own docstring: it draws a solid Ø19x6 steel annulus where
    the catalogue bearing is 8.0 g, so its volume weighs 12.0 g -- 50 % over,
    15.9 g over the leg. ADR-0087 read that as agreement ("the masses agree to
    8 %, so this is not a mass error"); the 8 % *was* the error. Same class of
    mistake as sizing a girdle from its fit box.

    ✅ **This comparison is deliberately assignment-independent.** Three
    earlier attempts tried to reproduce `per_link_mass()`'s part-to-link rule and
    came out +105 %, -79 % and +20 % wrong. Whole-leg inertia about the hip does
    not care which link a part is charged to, only where the mass physically is,
    so the question can be answered without that rule at all.

    ⚠️ **The cause is the opposite of the intuition.** The joint hardware
    — clevis, sheave, bearings — is **148.4 g of the 182.2**, and it
    clusters at the joints, i.e. near the axis it swings about. The CF tube is
    only **9 g**. A uniform-density capsule spreads that same mass along the
    link and therefore further out.

    ⚠️ **What it cost.** Swing inertia is the P1 metric, and
    [ADR-0043](../docs/DESIGN_DECISIONS.md) moved it **+62 %** by redistributing
    link mass alone. Every simulation result that depends on leg swing and
    predates M86 understates the tendon drive's central argument — motors on
    the body so the leg stays light — by **45 %**.
    """
    import sys as _sys
    import tomcat_leg_detail as LD
    import link_inertia as LI
    from tomcat_kin import LegModel
    from tomcat_kin.params import DEFAULT_HINDLEG as HL

    comps, report, pts = LD.build()

    # ---- the two must be in the SAME pose, or the comparison means nothing
    q = LegModel(HL).inverse((LD.FOOT_X, LD.FOOT_Z, LD.FOOT_PITCH))
    kin = 1e3 * np.asarray(LegModel(HL).joint_positions(q))
    assert np.allclose(np.asarray(pts), kin, atol=0.5), (
        "the detail CAD and the kinematics disagree about the stance pose"
    )

    # ---- CAD: every leg part about the hip, however it is apportioned
    hip = np.asarray(report["femur"]["p0"], float) * 1e-3
    parts = []
    for group, solids in comps.items():
        if group == "motor":            # girdle-mounted, not in the leg (P1)
            continue
        rho = LI.RHO.get(group)
        if rho is None:
            continue
        for sd in (s for c in solids for s in c.solids()):
            mm, com, I = LI._props(sd, rho)
            if mm > 0:
                parts.append((mm, com, I))
    M_cad, com_cad, I_cad = BI.combine(parts)
    d = com_cad - hip
    I_cad_hip = I_cad + M_cad * (np.dot(d, d) * np.eye(3) - np.outer(d, d))

    # ---- MJCF: the same leg, the same hip
    m = mujoco.MjModel.from_xml_string(
        MT.quadruped_rig(hip_height=0.176, spine=False))
    dd = mujoco.MjData(m)
    for i, jn in enumerate(("LR_q1", "LR_q2", "LR_q3")):
        dd.qpos[m.jnt_qposadr[mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_JOINT, jn)]] = q[i]
    mujoco.mj_forward(m, dd)
    # ⚠️ `xanchor` is indexed by JOINT id. Indexing it by dof address put the
    # hip 167 mm away and inflated the parallel-axis term into a fake 5x gap.
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "LR_q1")
    hip_w = np.array(dd.xanchor[jid])
    mparts = []
    for nm in ("LR_femur", "LR_tibia", "LR_meta", "LR_paw"):
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, nm)
        R = dd.ximat[b].reshape(3, 3)
        mparts.append((float(m.body_mass[b]), np.array(dd.xipos[b]),
                       R @ np.diag(m.body_inertia[b]) @ R.T))
    M_mj, com_mj, I_mj = BI.combine(mparts)
    d2 = com_mj - hip_w
    I_mj_hip = I_mj + M_mj * (np.dot(d2, d2) * np.eye(3) - np.outer(d2, d2))

    # ⚠️ The CAD side here still uses the ENVELOPE masses, so it reads 8 % heavy
    # against the MJCF's catalogue-calibrated ones. That gap is the finding above,
    # not slack: `test_link_inertia.py` pins the calibrated pair to 2 %.
    assert M_cad == pytest.approx(M_mj, rel=0.12), (
        f"CAD {1e3 * M_cad:.1f} g vs MJCF {1e3 * M_mj:.1f} g"
    )

    # ✅ the swing inertia now agrees; before M86 this ratio was 0.73
    ratio = I_cad_hip[1, 1] / I_mj_hip[1, 1]
    assert 1.0 < ratio < 1.12, (
        f"CAD(envelope)/MJCF swing inertia {ratio:.3f} "
        f"({I_cad_hip[1, 1]:.3e} vs {I_mj_hip[1, 1]:.3e})"
    )
    # ✅ and against the calibrated CAD number it is exact
    assert I_mj_hip[1, 1] == pytest.approx(1.1753e-3, rel=0.02), (
        f"MJCF swing inertia {I_mj_hip[1, 1]:.4e}, CAD says 1.1753e-3"
    )

    # ⚠️ because 80 % of the leg is joint hardware sitting AT the joints
    hw = 0.0
    for group in ("clevis", "sheave", "bearing"):
        for sd in (s for c in comps[group] for s in c.solids()):
            hw += LI._props(sd, LI.RHO[group])[0]
    assert hw / M_cad > 0.75, f"joint hardware is {100 * hw / M_cad:.0f} % of the leg"
