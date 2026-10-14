# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""M155: the CAD drawn on the simulator's bodies lands where the simulator's joints are.

A mesh in the wrong frame still renders, and a leg with its knee 10 mm off
still looks like a leg. So the frames are checked against the MJCF's own
numbers, not by eye: every joint the CAD places must be where the plant puts
the next body, in the plant's frame.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAD = os.path.join(ROOT, "mechanical", "cad")
for p in (CAD, os.path.join(ROOT, "kinematics", "src"), os.path.join(ROOT, "tools")):
    if p not in sys.path:
        sys.path.insert(0, p)

pytest.importorskip("build123d", reason="CAD stack (build123d/OCP) not installed")
mujoco = pytest.importorskip("mujoco")

import link_inertia as LI                                        # noqa: E402
import sim_meshes as SM                                          # noqa: E402
import tomcat_leg_detail as LD                                   # noqa: E402
import tomcat_trunk as TT                                        # noqa: E402
from tomcat_kin.params import DEFAULT_FORELEG, DEFAULT_HINDLEG, DEFAULT_SPINE  # noqa: E402

LEG = {"fore": ("LF", DEFAULT_FORELEG), "hind": ("LR", DEFAULT_HINDLEG)}


@pytest.fixture(scope="module")
def plant():
    from tomcat_kin.env import TomcatEnv
    return TomcatEnv().model


@pytest.fixture(scope="module")
def reports():
    return {role: LD.build(leg, role=role)[1] for role, (_n, leg) in LEG.items()}


def _pos(m, name):
    return np.array(m.body(name).pos)


@pytest.mark.parametrize("role", ("fore", "hind"))
def test_each_LEG_JOINT_lands_on_the_next_MJCF_body(plant, reports, role):
    """The CAD's knee, ankle and paw base, in the parent link's MJCF frame,
    are the child body's `pos`. A wrong rotation or a missing limb-plane
    offset moves them by millimetres to tens of millimetres."""
    frames = LI.link_frames(reports[role])
    leg = LEG[role][0]
    for a, b in zip(LI.LINKS[:-1], LI.LINKS[1:]):
        pa, Ra = frames[a]
        pb, _Rb = frames[b]
        got = Ra.T @ (pb - pa)
        want = _pos(plant, "%s_%s" % (leg, b))
        assert got == pytest.approx(want, abs=5e-4), (a, b, got * 1e3, want * 1e3)


@pytest.mark.parametrize("role", ("fore", "hind"))
def test_the_PAW_ANGLE_is_the_one_the_plant_fixes(plant, reports, role):
    """The paw has no joint: the MJCF fixes it at an angle to the metatarsus.
    The CAD's stance pose must have the same angle, or the paw is drawn
    turned against the pad the plant stands on."""
    frames = LI.link_frames(reports[role])
    rel = frames["meta"][1].T @ frames["paw"][1]
    q = np.array(plant.body("%s_paw" % LEG[role][0]).quat)
    want = np.zeros(9)
    mujoco.mju_quat2Mat(want, q)
    assert rel == pytest.approx(want.reshape(3, 3), abs=2e-3)


def test_the_HIPS_sit_on_the_trunk_bodies_where_the_plant_hangs_the_legs(plant):
    """CAD trunk coordinates, moved into each MJCF body's frame by
    `trunk_body_origins`, put the hips where the femurs hang."""
    o = {n: p for n, p in SM.trunk_body_origins().values()}
    y = LD.TRACK_Y * 1e-3
    hind = np.array([TT.REAR_HIP_X * 1e-3, y, 0.0]) - o["trunk"]
    assert hind == pytest.approx(_pos(plant, "LR_femur"), abs=1e-6)
    # the front girdle is rigid on spine3
    front = np.array([TT.FRONT_HIP_X * 1e-3, y, 0.0]) - o["spine3"]
    want = _pos(plant, "front_girdle") + _pos(plant, "LF_femur")
    assert front == pytest.approx(want, abs=1e-6)


def test_the_CAD_spine_joints_are_the_plant_s(plant):
    """CAD bodies 1-3 start at the joints the MJCF's spine bodies hinge on."""
    for (b, (name, origin)) in SM.trunk_body_origins().items():
        if b == 0:
            continue
        assert TT.JOINT_X[b - 1] * 1e-3 == pytest.approx(origin[0], abs=1e-9)
        parent = plant.body(plant.body_parentid[plant.body(name).id]).name
        po = 0.0 if parent == "trunk" else SM.trunk_body_origins()[b - 1][1][0]
        assert origin[0] - po == pytest.approx(_pos(plant, name)[0], abs=1e-9)
    assert SM.trunk_body_origins()[1][1][2] == pytest.approx(DEFAULT_SPINE.spine_axis_z)


def test_a_MIRRORED_mesh_keeps_its_faces_outward():
    """A reflection turns a mesh inside out unless the faces are re-wound:
    the signed volume of a closed mesh must stay positive."""
    V = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], float)
    F = np.array([[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]])

    def vol(V, F):
        t = V[F]
        return float(np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])).sum()) / 6.0
    assert vol(V, F) > 0
    assert vol(*SM.mirror_y(V, F)) == pytest.approx(vol(V, F))


def test_the_CAD_model_is_the_SAME_PLANT():
    """Visual only: adding the meshes changes no mass, no inertia and no
    contact, so the replay is of the plant the policy walked on."""
    from render_episode import CAD_MESHES, render_model
    from tomcat_kin.env import TomcatEnv
    if not os.path.isdir(CAD_MESHES):
        pytest.skip("run mechanical/cad/sim_meshes.py first")
    e = TomcatEnv()
    plain, cad = render_model(e), render_model(e, cad=True)
    assert cad.nbody == plain.nbody and cad.nq == plain.nq
    assert np.array_equal(cad.body_mass, plain.body_mass)
    assert np.array_equal(cad.body_inertia, plain.body_inertia)
    mesh = cad.geom_type == mujoco.mjtGeom.mjGEOM_MESH
    assert mesh.sum() == cad.nmesh > 0
    assert not cad.geom_contype[mesh].any() and not cad.geom_conaffinity[mesh].any()
