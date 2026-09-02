# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""M83: the mass-property extractor, checked against shapes with known answers.

⚠️ Written because the first version of `properties()` returned **negative
inertia** for anything off the origin, and negative inertia is not a subtle
error — MuJoCo either refuses the model or simulates nonsense. It passed the
only case tried at first (a cylinder at the origin) because there the mistake
cancels.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pytest

pytest.importorskip("build123d", reason="build123d is an optional dependency")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "mechanical", "cad"))
from build123d import Box, Cylinder, Pos          # noqa: E402
import mass_properties as MP                      # noqa: E402


def test_the_MOTOR_reproduces_its_SURVEYED_MASS():
    """✅ **131.7 g out of a Ø34.5 × 36.1 mm cylinder.**

    The motor is the one part with a surveyed mass
    (`docs/notes/motor-reality-check.md`), so its density is the one that can be
    back-solved rather than assumed — and doing so caught the table's guess of
    **2660 kg/m³** being **46 % low**. Every other density in `MP.DENSITY` is
    still `[assumed]` and has no such check available.
    """
    r, length = 0.0345 / 2, 0.0361
    mass, com, I = MP.properties(Cylinder(r, length), MP.DENSITY["motor"])
    assert mass == pytest.approx(0.1317, abs=5e-4)
    assert np.allclose(com, 0.0, atol=1e-12)

    # a solid cylinder's inertia is analytic, so the tensor is checkable too
    want = np.array([mass * (3 * r * r + length * length) / 12,
                     mass * (3 * r * r + length * length) / 12,
                     mass * r * r / 2])
    assert np.allclose(np.diag(I), want, rtol=1e-6)


def test_INERTIA_IS_ABOUT_THE_CENTRE_OF_MASS_and_never_negative():
    """⚠️ **OCP already reports about the CoM. Shifting it again goes negative.**

    The first implementation assumed OCP returned the inertia about the origin
    and applied the parallel-axis theorem to move it to the centre of mass. It
    does not, so that subtracted a term that was never added:

    | box 60 × 40 × 20 mm | I_yy | I_zz |
    |---|---|---|
    | analytic, about CoM | 4.32e-5 | 5.62e-5 |
    | first version, box at x=100 mm | **−1.25e-3** | **−1.24e-3** |

    ✅ The tensor is invariant to where the solid sits, checked at four offsets,
    so the correct treatment is to apply the density and nothing else.
    """
    rho = MP.DENSITY["alu"]
    size = (0.06, 0.04, 0.02)
    mass_want = rho * size[0] * size[1] * size[2]
    want = mass_want * np.array([size[1] ** 2 + size[2] ** 2,
                                 size[0] ** 2 + size[2] ** 2,
                                 size[0] ** 2 + size[1] ** 2]) / 12.0

    for off in ((0, 0, 0), (0.1, 0, 0), (0, 0.2, 0), (0.05, 0.05, 0.05)):
        mass, com, I = MP.properties(Pos(*off) * Box(*size), rho)
        assert mass == pytest.approx(mass_want, rel=1e-9)
        assert np.allclose(com, np.array(off), atol=1e-9), (
            f"centre of mass must track the offset {off}"
        )
        # ⚠️ the assertion the first version failed
        assert np.all(np.diag(I) > 0.0), f"negative inertia at offset {off}"
        assert np.allclose(np.diag(I), want, rtol=1e-6), (
            f"the tensor must not move with the solid: offset {off}"
        )


def test_DENSITY_is_ASSUMED_for_everything_but_the_motor():
    """⚠️ **The weakest link in the CAD-to-MJCF path, stated plainly.**

    The skeleton CAD is *form*, not material: no part carries an alloy or a
    print setting, so every density but the motor's is a judgement with nothing
    to check it against. This test does not validate them — it pins the fact
    that they are assumptions, so the number of unvalidated ones cannot grow
    quietly.
    """
    assumed = set(MP.DENSITY) - {"motor"}
    assert assumed == {"alu", "pla", "petg", "nylon_cf", "steel"}, (
        "a density was added or removed -- is it measured or assumed?"
    )
    assert MP.DENSITY["motor"] == pytest.approx(3903.0, rel=1e-3), (
        "the motor's is back-solved from a surveyed 131.7 g, not guessed"
    )
    for k in assumed:
        assert 900.0 < MP.DENSITY[k] < 9000.0
