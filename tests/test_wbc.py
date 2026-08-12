# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Whole-body contact-force allocation (M33).

⚠️ The gate is the **static case**. If a robot standing still cannot be allocated
forces that sum to its weight with the centre of pressure where it was asked for,
nothing built on top means anything. Pure numpy — no MuJoCo, so this runs anywhere.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from tomcat_kin import gait, wbc

MU = 0.9


def test_cone_projection_is_a_projection(setup=None):
    """Inside stays put, outside lands exactly on the surface, pulling gives zero."""
    inside = np.array([5.0, 0.0, 10.0])
    assert np.allclose(wbc.cone_project(inside, MU), inside)

    outside = np.array([30.0, 0.0, 10.0])
    out = wbc.cone_project(outside, MU)
    assert np.linalg.norm(out[:2]) / out[2] == pytest.approx(MU, rel=1e-9)
    assert out[2] > 0.0

    # A contact cannot pull. Deep in the polar cone the answer is no force at all.
    assert np.allclose(wbc.cone_project(np.array([0.0, 0.0, -3.0]), MU), 0.0)


def test_a_static_stance_allocates_to_body_weight_with_the_CoP_under_the_CoM():
    """THE gate. Two feet, weight only, no moment asked for."""
    controller = gait.GaitController(gait.trot_params())
    weight = controller.body.total_mass * 9.81
    com = np.array([0.1031, 0.0, 0.1649])
    feet = np.array([[0.200, +0.048, 0.0], [0.005, -0.048, 0.0]])

    f = wbc.allocate(feet, com, np.concatenate([[0, 0, weight], np.zeros(3)]), MU)

    assert f[:, 2].sum() == pytest.approx(weight, rel=2e-3)
    assert abs(f[:, 0].sum()) < 0.05 and abs(f[:, 1].sum()) < 0.05
    moment = sum(np.cross(feet[i] - com, f[i]) for i in range(2))
    assert np.linalg.norm(moment[:2]) < 0.05

    cop = (f[:, 2] @ feet[:, :2]) / f[:, 2].sum()
    assert np.allclose(cop, com[:2], atol=1e-3)


def test_height_must_be_regulated_or_the_robot_floats():
    """⚠️ Commanding exactly `m g` balances the weight and regulates nothing.

    The first torque-control run drifted **0.165 → 0.185 m in 0.6 s** with no
    disturbance. LIPM *assumes* constant CoM height; a torque controller has to
    **make** that true. `height=None` reproduces the bug on purpose.
    """
    mass, omega = 4.045, 7.7
    com = np.array([0.0, 0.0, 0.150])          # 15 mm below the 0.165 target
    vel = np.zeros(3)

    loose = wbc.desired_wrench(mass, com, vel, com[:2], omega)
    tight = wbc.desired_wrench(mass, com, vel, com[:2], omega, height=0.165)

    assert loose[2] == pytest.approx(mass * 9.81, rel=1e-9)   # weight, and nothing more
    assert tight[2] > loose[2] * 1.1, "a low CoM must be pushed back up"


def test_the_CoP_is_confined_to_the_SEGMENT_between_two_feet():
    """⚠️ The structural finding of M33.

    A diagonal trot has **no support polygon — only a support line**. A DCM law that
    commands a free 2-D centre of pressure is asking for something no allocation can
    deliver, and the regularised solve returns the nearest thing rather than failing
    loudly. Clamping makes the infeasibility measurable, and measured it says the
    balance problem needs a **step**, not more force: the standing test's command ran
    0.3 mm off the segment, then 104 mm, then 591 mm before it fell.
    """
    feet = np.array([[0.200, +0.048, 0.0], [0.005, -0.048, 0.0]])

    inside = np.array([0.1025, 0.0])
    assert np.allclose(wbc.realisable_cop(feet, inside), inside, atol=1e-6)

    # Far off the line: clamped, and the residual is the "you must step" signal.
    far = np.array([0.1025, 0.35])
    got = wbc.realisable_cop(feet, far)
    assert np.linalg.norm(got - far) > 0.2
    # ... and what it clamps to really is on the segment.
    a, b = feet[0][:2], feet[1][:2]
    t = float((got - a) @ (b - a) / ((b - a) @ (b - a)))
    assert -1e-9 <= t <= 1 + 1e-9
    assert np.allclose(got, a + t * (b - a), atol=1e-9)

    # Beyond an end it clamps to the foot itself, not past it.
    beyond = np.array([0.6, 0.3])
    assert np.allclose(wbc.realisable_cop(feet, beyond), feet[0][:2], atol=1e-6)


def test_allocation_respects_friction_when_the_demand_is_impossible():
    """A sideways demand no floor could supply must come back inside the cone."""
    controller = gait.GaitController(gait.trot_params())
    weight = controller.body.total_mass * 9.81
    com = np.array([0.1031, 0.0, 0.1649])
    feet = np.array([[0.200, +0.048, 0.0], [0.005, -0.048, 0.0]])

    wrench = np.concatenate([[3.0 * weight, 0.0, weight], np.zeros(3)])
    f = wbc.allocate(feet, com, wrench, mu=0.3)
    for row in f:
        if row[2] > 1e-9:
            assert np.linalg.norm(row[:2]) / row[2] <= 0.3 + 1e-6
