# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The trunk's booleans, which failed silently and took a rigid body with them.

⚠️ M91 found the rear girdle reporting **2.9 cm3** where its own shell alone is
47.0. Fusing the rear joint's ventral process took the body from 114,889 mm3 to
**0.0 with zero solids**, and OCC raised nothing. Later parts re-fused into one
small solid, so the existing guard -- "all bodies are one part each" -- passed on
a body that had been annihilated, and the published structure mass was computed
from what was left.

The trigger is a tangential boolean: a 6 mm post grazing a 1.2 mm lofted wall,
entering it by 9 % of its volume. `intersect` on the same pair is wrong the other
way, answering the post's WHOLE volume where point sampling says a tenth of it.

These tests assert the invariant that catches it. A union cannot shrink.

`build123d` is an optional dependency; the module skips without it.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest

pytest.importorskip("build123d", reason="build123d is an optional dependency")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mechanical", "cad"))
import tomcat_trunk as T  # noqa: E402


def _vol(g):
    return sum(s.volume for s in g.solids())


def test_a_grazing_post_does_not_annihilate_the_shell():
    """⚠️ The minimal reproduction, and it needs no assembly.

    Body 0's shell plus the four posts of the rear joint. Every fuse must leave
    the volume at least where it was; the defect drove it to exactly zero.
    """
    g = T.body_shell(0)
    v = _vol(g)
    assert v > 40e3, "the rear shell should be ~47 cm3, got %.1f" % v
    for v_cut in T.process_clearance(0):
        g = g - v_cut
    for i, part in enumerate(T.joint_parts(0)):
        g = g + part
        now = _vol(g)
        assert now >= v - 1e-6, (
            "fusing joint part %d shrank the body %.1f -> %.1f mm3" % (i, v, now))
        v = now


def test_the_fuse_guard_refuses_a_shrinking_union():
    """✅ The guard has to FAIL, or it is decoration.

    A real union cannot shrink, which is exactly why the guard is worth having
    and exactly why it cannot be provoked with real solids. The boolean engine is
    stubbed to do what OCC actually did -- return a smaller shape, quietly -- and
    `_fuse` must raise on it rather than pass it on.
    """
    class _Shape:
        def __init__(self, vol, result=None):
            self._vol, self._result = vol, result

        def solids(self):
            return [type("S", (), {"volume": self._vol})()]

        def __add__(self, other):
            return self._result

    annihilated = _Shape(0.0)
    body = _Shape(114889.3, annihilated)
    with pytest.raises(RuntimeError, match="cannot shrink"):
        T._fuse(body, object(), "body 0 joint part 4")

    grew = _Shape(115058.9)
    assert T._fuse(_Shape(114889.3, grew), object(), "a real fuse") is grew


def test_every_rigid_body_is_at_least_its_own_shell():
    """⚠️ The check the solid count could not make.

    A body is its shell PLUS bulkheads, bosses and joint hardware, so its volume
    can never be less than the shell's. Body 0 measured 2.9 cm3 against a 47.0
    cm3 shell and the report called it one clean part.
    """
    for b in sorted(T.BODIES):
        shell = _vol(T.body_shell(b))
        whole = _vol(T.rigid_body(b))
        assert whole >= shell, (
            "body %d is %.1f cm3, less than its own %.1f cm3 shell"
            % (b, whole / 1e3, shell / 1e3))


def test_intersect_is_not_trustworthy_on_the_grazing_post():
    """⚠️ Asserts the DEFECT, in the library rather than in this repo.

    `intersect` reports the ventral post is entirely inside body 0's shell.
    Point sampling says under a fifth of it is. The test pins the discrepancy so
    that a future OCP that fixes it announces itself by failing here.
    """
    shell = T.body_shell(0)
    post = T.joint_parts(0)[4]
    pv = _vol(post)
    inter = shell.intersect(post)
    claimed = 0.0 if inter is None else _vol(inter)

    sol = shell.solids()[0]
    bb = post.bounding_box()
    cx = (bb.min.X + bb.max.X) / 2
    cz = (bb.min.Z + bb.max.Z) / 2
    rng = np.random.default_rng(0)
    inside = n = 0
    while n < 400:
        dx, dz = rng.uniform(-T.POST_R, T.POST_R, 2)
        if dx * dx + dz * dz > T.POST_R ** 2:
            continue
        n += 1
        if sol.is_inside((cx + dx, rng.uniform(bb.min.Y, bb.max.Y), cz + dz)):
            inside += 1
    sampled = pv * inside / n

    assert claimed == pytest.approx(pv, rel=1e-6), (
        "intersect no longer claims the whole post (%.1f of %.1f)" % (claimed, pv))
    assert sampled < 0.2 * pv, (
        "sampling now agrees with intersect: %.1f of %.1f mm3" % (sampled, pv))


def test_the_structure_mass_is_inside_the_budget_the_model_implies():
    """⚠️ "221 g against a 200 g budget" was PROSE. Neither number was derived.

    The 221 came off the annihilated body; the 200 was recalled. Both are
    computed here from `SpineParams`, so neither can drift from the model again.
    """
    struct = sum(_vol(T.rigid_body(b)) for b in sorted(T.BODIES)) * 1.2e-3
    budget = T.SP.trunk_mass * T.MM - 18 * 131.7 - 240.0
    assert budget == pytest.approx(1024.4, abs=0.5)
    assert struct < budget, "structure %.0f g over the %.0f g budget" % (struct, budget)
    assert struct > 300.0, (
        "structure fell to %.0f g -- a body has probably collapsed again" % struct)
