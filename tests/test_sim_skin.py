# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""M157: the skinned suit is bound to the plant's bodies the way it was built.

A skin bound in the wrong frame still renders, only somewhere else, or
sheared. So the binding is checked against the poses it must reproduce: at the
bind pose the suit is exactly the suit that was built, and a rigid motion of
the whole robot moves it rigidly.
"""

from __future__ import annotations

import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAD = os.path.join(ROOT, "mechanical", "cad")
for p in (CAD, os.path.join(ROOT, "kinematics", "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

pytest.importorskip("build123d", reason="CAD stack (build123d/OCP) not installed")
mujoco = pytest.importorskip("mujoco")

import sim_skin as SS                                            # noqa: E402

if not os.path.exists(SS.OUT):
    pytest.skip("run mechanical/cad/sim_skin.py first", allow_module_level=True)


@pytest.fixture(scope="module")
def setup():
    from tomcat_kin.env import TomcatEnv
    model = TomcatEnv().model
    skin = SS.Skin()
    binds = SS.bind_frames()
    return model, skin, binds


def _data(model, frames):
    xpos = np.zeros((model.nbody, 3))
    xmat = np.tile(np.eye(3).ravel(), (model.nbody, 1))
    for b, (p, R) in frames.items():
        i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b)
        xpos[i], xmat[i] = p, R.ravel()
    return SimpleNamespace(xpos=xpos, xmat=xmat)


def test_the_weights_are_a_partition_of_one(setup):
    _m, skin, _b = setup
    assert np.allclose(skin.w.sum(1), 1.0)
    assert (skin.w >= 0).all()


def test_at_the_BIND_POSE_the_suit_is_the_one_that_was_built(setup):
    model, skin, binds = setup
    V, N = skin.pose(model, _data(model, binds))
    assert np.abs(V - skin.V0).max() < 1e-9
    assert np.allclose(np.linalg.norm(N, axis=1), 1.0)


def test_a_RIGID_motion_of_the_robot_moves_the_suit_rigidly(setup):
    """Every body turned 30 deg about z and moved: no vertex stretches."""
    model, skin, binds = setup
    c, s = np.cos(np.radians(30)), np.sin(np.radians(30))
    Q = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    t = np.array([0.3, -0.1, 0.17])
    moved = {b: (Q @ p + t, Q @ R) for b, (p, R) in binds.items()}
    V, _N = skin.pose(model, _data(model, moved))
    assert np.abs(V - (skin.V0 @ Q.T + t)).max() < 1e-9


def test_every_body_the_suit_rides_is_a_plant_body(setup):
    model, skin, _b = setup
    for b in skin.names:
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b) >= 0, b
