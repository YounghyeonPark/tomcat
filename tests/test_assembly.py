# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The assembly's ownership rules, which were comments and not checks.

`tomcat_assembly` drops the leg module's motor bank and its duplicate hip
because the trunk owns both. That rule lived in `DROP` and `HIP_HARDWARE` as
source-code comments, and this file had **no pytest at all** -- its checks ran
only when a person ran the module by hand.

⚠️ So the first render of the complete robot broke the rule without noticing.
It called `tomcat_leg_detail.build()` directly, and put **12 reference motors**
on the machine reaching to y = 79 mm where the girdle flank is 43, with every
cable routed to the default `SPOOL_OFFSET` the assembly exists to override. The
picture looked busy; nothing said it was wrong.

⚠️ And `assembly()` -- the function named "the whole robot" -- knew about
neither the head, nor the tail, nor the skin, each of which had been drawn and
checked inside its own module and never placed with the rest.

These tests are cheap on purpose: one leg, and the soft parts. The full
four-leg report stays a command, not a test, because the CAD builds are the
heaviest thing this repo runs.

`build123d` is an optional dependency; the module skips without it.
"""

from __future__ import annotations

import os
import sys

import pytest

pytest.importorskip("build123d", reason="build123d is an optional dependency")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mechanical", "cad"))
import tomcat_assembly as TA  # noqa: E402
import tomcat_leg_detail as LD  # noqa: E402
import tomcat_trunk as TT  # noqa: E402


def test_a_rigid_body_is_not_a_motor(): 
    """⚠️ The check that found the defect had a defect of its own.

    Matching a motor by volume alone -- within 2 % of 33,747 mm3 -- flagged
    **trunk body 2**, which is 34,202. A whole rigid body was reported as a
    duplicated motor, and the assembly failed for a reason that was not true.
    A can is ø34.5 x 36.1; its BOX is as particular as its volume.
    """
    ref = TA.motor_can()
    assert TA.is_motor_can(ref, ref)
    for b in sorted(TT.BODIES):
        for sd in TT.rigid_body(b).solids():
            assert not TA.is_motor_can(sd, ref),                 "trunk body %d (%.0f mm3) called a motor" % (b, sd.volume)


def test_the_trunk_places_every_motor_the_budget_bought():
    """19, not 18. The trunk's own line read `%d of 18` for five milestones, and
    a check that counts against a literal cannot notice the literal is wrong."""
    per_body = {b: len(TT.motors(b)) for b in sorted(TT.BODIES)}
    assert sum(per_body.values()) == 19, per_body


def test_a_leg_in_the_assembly_carries_no_motor():
    """⚠️ The defect the whole-robot render shipped.

    `tomcat_leg_detail` draws three motors per leg for context. In an assembly
    the trunk owns them -- as BORES, so the number of cans standing on the
    finished robot is zero. Four legs' worth of the leg module's copy is 12.
    """
    ref = TA.motor_can()
    leg = TA.one_leg(True, +1.0)
    cans = [s.volume for s in leg.solids() if TA.is_motor_can(s, ref)]
    assert not cans, "%d motor can(s) on a leg: %s" % (len(cans), cans)


def test_the_leg_module_does_draw_them_so_the_drop_is_load_bearing():
    """The other half of the pair: if `tomcat_leg_detail` stopped drawing the
    bank, the test above would pass for the wrong reason forever."""
    comps, _report, _pts = LD.build()
    ref = TA.motor_can()
    cans = [s.volume for s in comps["motor"].solids() if TA.is_motor_can(s, ref)]
    assert len(cans) == 3, "the leg module draws %d cans, not 3" % len(cans)


def test_the_soft_parts_lose_nothing_their_modules_draw():
    """⚠️ It knew about none of them, and then it knew about the cranium
    and not the face.

    Each of the head, the tail and the skin was drawn and checked inside its
    own module, and none was in the thing called the assembly. The first fix
    listed `HD.whole()` and forgot `HD.features()` -- so the first repo-side
    render of the robot had no ears, no eyes and no nose. This names every
    solid its module draws and compares the volume that reached the assembly.
    """
    import tomcat_head as HD
    import tomcat_skin as SK
    import tomcat_tail as TL
    drawn = {n: sum(s.volume for s in g.solids()) for n, g in TA.soft_parts()}
    for name, want in (("head", HD.whole()), ("face", HD.features()),
                       ("tail", TL.tail()), ("skin", SK.skin())):
        assert name in drawn, "%s is not in the assembly: %s" % (name, list(drawn))
        v = sum(s.volume for s in want.solids())
        assert abs(drawn[name] - v) < 1.0, "%s: %.1f in the assembly, %.1f drawn" % (
            name, drawn[name], v)


def test_every_soft_part_has_volume():
    """A loft that silently returns an empty compound is this repo's oldest
    failure mode -- `_fuse` exists because a union once returned zero solids."""
    for name, g in TA.soft_parts():
        v = sum(s.volume for s in g.solids())
        assert v > 1000.0, "%s is %.1f mm3" % (name, v)
