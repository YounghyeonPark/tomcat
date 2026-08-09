# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The MuJoCo model must reproduce the analytical one before it can check it.

⚠️ These tests are the *gate* on the M17 cross-check. A physics engine that has
drifted from the analytical parameter set would produce confident, wrong numbers
— which is exactly the failure mode this project has hit repeatedly. So the
agreement asserted here is exact-to-tolerance, not approximate.

`mujoco` is an optional dependency; the whole module skips without it.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from tomcat_kin import gait, mjcf

mujoco = pytest.importorskip("mujoco", reason="mujoco is an optional dependency")

STANCE = ("LF", "RR")


@pytest.fixture(scope="module")
def rig():
    c = gait.GaitController(gait.trot_params())
    q = mjcf.stance_pose(c, 0.25)
    h = mjcf.rest_height(c, q, STANCE)
    m = mujoco.MjModel.from_xml_string(mjcf.build_mjcf(c, q, height=h))
    d = mujoco.MjData(m)
    for nm in c.body.leg_names:
        for i, v in enumerate(q[nm], start=1):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{nm}_q{i}")
            d.qpos[m.jnt_qposadr[jid]] = v
    mujoco.mj_forward(m, d)
    return c, q, m, d, h


def test_total_mass_matches_analytical(rig):
    c, _, m, _, _ = rig
    assert float(mujoco.mj_getTotalmass(m)) == pytest.approx(c.body.total_mass, abs=1e-9)


def test_paw_tips_match_analytical_forward_kinematics(rig):
    """The MJCF hinge convention must reproduce `LegModel.forward` exactly.

    Regression guard: an earlier version set BOTH the joint `ref` and `qpos`,
    which cancelled and left every leg straight out forward — a 287 mm error
    that still produced a plausible-looking simulation.
    """
    c, q, m, d, h = rig
    for nm in c.body.leg_names:
        ana = c.body.foot_world_position(np.zeros(3), nm, q[nm])
        sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"{nm}_site")
        sim = d.site_xpos[sid]
        assert sim[0] == pytest.approx(float(ana[0]), abs=1e-9)
        assert sim[2] - h == pytest.approx(float(ana[1]), abs=1e-9)
        assert sim[1] == pytest.approx(c.body.mounts[nm].track_y, abs=1e-9)


def test_centre_of_mass_matches_analytical(rig):
    c, q, m, d, h = rig
    ana = c.body.center_of_mass(np.zeros(3), q).total.com
    mujoco.mj_comPos(m, d)
    sim = d.subtree_com[0]
    assert sim[0] == pytest.approx(float(ana[0]), abs=1e-9)
    assert sim[2] - h == pytest.approx(float(ana[1]), abs=1e-9)


def test_paw_contact_starts_resting_not_interpenetrating(rig):
    """`rest_height` must place the contact points exactly on the floor.

    Regression guard: with the paw sphere centred on the analytical tip and no
    radius compensation, the model began 8 mm through the floor and the solver
    launched the robot upward — which reads as a 67 % divergence-rate error.
    """
    c, q, _, _, h = rig
    xml = mjcf.build_mjcf(c, q, height=h, timestep=1e-4)
    xml = xml.replace('kp="120" kv="4"', 'kp="400" kv="12"')
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    for nm in c.body.leg_names:
        for i, v in enumerate(q[nm], start=1):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{nm}_q{i}")
            d.qpos[m.jnt_qposadr[jid]] = v
    d.ctrl[:] = np.concatenate([q[nm] for nm in c.body.leg_names])
    mujoco.mj_forward(m, d)
    mujoco.mj_comPos(m, d)
    h0 = float(d.subtree_com[0][2])

    for _ in range(1000):
        mujoco.mj_step(m, d)
    mujoco.mj_comPos(m, d)
    h1 = float(d.subtree_com[0][2])

    assert d.ncon == 2, f"expected the diagonal pair in contact, got {d.ncon}"
    # The launch signature was a RISE of tens of mm. Settling may sink slightly.
    assert h1 - h0 < 1e-3, f"CoM rose {1000*(h1-h0):.2f} mm — solver is ejecting the model"
    assert h0 - h1 < 5e-3, f"CoM sank {1000*(h0-h1):.2f} mm — contact is too soft"


def test_lipm_divergence_rate_is_not_optimistic():
    """M17's core result: the real CoM must not topple FASTER than LIPM predicts.

    `control.py` sizes every envelope on xi growing as e^(omega t). If the true
    rigid-body divergence were faster, every envelope would be optimistic. It is
    measured ~2 % SLOWER (distributed inertia resists the topple), so the
    reduced-order model is conservative — but the assertion is one-sided on
    purpose: only the optimistic direction is a design error.
    """
    from tomcat_kin import control

    c = gait.GaitController(gait.trot_params())
    q = mjcf.stance_pose(c, 0.25)
    h = mjcf.rest_height(c, q, STANCE)
    _, perp = mjcf.support_line(c, q, STANCE)
    xml = mjcf.build_mjcf(c, q, height=h, timestep=1e-4)
    xml = xml.replace('kp="120" kv="4"', 'kp="400" kv="12"')
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    for nm in c.body.leg_names:
        for i, v in enumerate(q[nm], start=1):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{nm}_q{i}")
            d.qpos[m.jnt_qposadr[jid]] = v
    d.ctrl[:] = np.concatenate([q[nm] for nm in c.body.leg_names])
    mujoco.mj_forward(m, d)
    for _ in range(300):
        mujoco.mj_step(m, d)

    mujoco.mj_comPos(m, d)
    c0 = d.subtree_com[0].copy()
    omega = math.sqrt(9.81 / float(c0[2]))
    d.qvel[0] += 0.02 * perp[0]
    d.qvel[1] += 0.02 * perp[1]

    ts, xis = [], []
    for k in range(2500):
        mujoco.mj_step(m, d)
        mujoco.mj_comPos(m, d)
        com, vel = d.subtree_com[0], d.cvel[1][3:6]
        ts.append(k * m.opt.timestep)
        xis.append(float((com[:2] - c0[:2]) @ perp) + float(vel[:2] @ perp) / omega)
    ts, xis = np.array(ts), np.array(xis)
    sel = (ts > 0.03) & (xis > 1e-5)
    rate = np.polyfit(ts[sel], np.log(xis[sel]), 1)[0]

    assert rate == pytest.approx(omega, rel=0.06)
    assert rate <= omega * 1.01, (
        f"rigid-body divergence {rate:.4f} exceeds LIPM {omega:.4f} — "
        "every envelope in control.py would be optimistic"
    )


def test_the_two_diagonals_topple_along_different_axes():
    """A structural limit of the reduced-order model, found by the sim.

    `StepPlant` collapses balance onto ONE axis with a fixed `projection`. The
    two diagonal support lines are not parallel, so consecutive steps are
    unstable along axes ~52 deg apart. The magnitudes agree with `projection`,
    which is why the 1-D reduction works at all — but the directions do not.
    """
    from tomcat_kin import control

    c = gait.GaitController(gait.trot_params())
    q = mjcf.stance_pose(c, 0.25)
    _, pa = mjcf.support_line(c, q, ("LF", "RR"))
    _, pb = mjcf.support_line(c, q, ("RF", "LR"))

    pl = control.StepPlant.from_gait(c, n=96, latency=0.0075, floor_mu=0.8)
    assert abs(pa[0]) == pytest.approx(pl.projection, rel=1e-3)
    assert abs(pb[0]) == pytest.approx(pl.projection, rel=1e-3)

    angle = math.degrees(math.acos(abs(float(pa @ pb))))
    assert angle == pytest.approx(52.4, abs=0.5)


def test_lateral_spine_sway_matches_the_analytical_model(rig):
    """M20's validation gate: the spine is the balance actuator, so it must agree.

    ⚠️ This is the test that found the bug. `center_of_mass_y` used to place the
    fore legs at the spine tip, arguing that left/right track offsets cancel. They
    do — but the **fore-aft** offset of a leg's CoM does not: the yaw rotates it
    into y and both fore legs contribute the same sign. In a trot stance that CoM
    sits ~52 mm behind the hip, so the analytical sway was **4 % optimistic**.

    Passing the real pose closes it to sub-micron. The naive form is checked too,
    so the size of the error stays recorded rather than becoming folklore.
    """
    c, q, _, _, _ = rig
    h = mjcf.rest_height(c, q, STANCE)
    m = mujoco.MjModel.from_xml_string(mjcf.build_mjcf(c, q, height=h, spine_dof=True))
    d = mujoco.MjData(m)

    def adr(name):
        return m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)]

    worst_fixed = worst_naive = 0.0
    for lat in ([0.0, 0.0, 0.0], [0.262, 0.0, 0.0], [0.0, 0.262, 0.0],
                [0.262] * 3, [-0.262] * 3, [0.15, -0.10, 0.20]):
        for nm in c.body.leg_names:
            for i, v in enumerate(q[nm], start=1):
                d.qpos[adr(f"{nm}_q{i}")] = v
        for i, v in enumerate(lat, start=1):
            d.qpos[adr(f"spine_y{i}")] = v
        mujoco.mj_forward(m, d)
        mujoco.mj_comPos(m, d)
        sim = float(d.subtree_com[0][1])
        worst_fixed = max(worst_fixed, abs(sim - c.body.center_of_mass_y(np.array(lat), q)))
        worst_naive = max(worst_naive, abs(sim - c.body.center_of_mass_y(np.array(lat))))

    assert worst_fixed < 1e-5, f"corrected form off by {1000 * worst_fixed:.4f} mm"
    assert worst_naive > 1e-3, "the naive form's 4 % error has vanished — check why"


def test_spine_dof_does_not_disturb_the_rigid_model(rig):
    """`spine_dof=True` must add freedom, not change the body it is added to."""
    c, q, _, _, h = rig
    rigid = mujoco.MjModel.from_xml_string(mjcf.build_mjcf(c, q, height=h))
    flex = mujoco.MjModel.from_xml_string(mjcf.build_mjcf(c, q, height=h, spine_dof=True))
    assert float(mujoco.mj_getTotalmass(flex)) == pytest.approx(
        float(mujoco.mj_getTotalmass(rigid)), abs=1e-12)
    assert flex.nq == rigid.nq + 3
    assert flex.nu == rigid.nu + 3
