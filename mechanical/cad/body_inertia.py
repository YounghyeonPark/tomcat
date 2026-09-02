# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Per-body mass, centre of mass and inertia, from skeleton + packaging CAD.

Combines the two CADs so a body can carry what is actually in it: skeleton form
plus the motors `tomcat_packaging.py` places at their surveyed size
(Ø34.5 × 36.1 mm, 131.7 g each).

⚠️ **What this is NOT for, learned by getting it wrong.** A first pass used
it to divide MJCF body masses by *skeleton* volumes and concluded the girdles
implied 21 800 and 26 800 kg/m³ -- denser than tungsten. That was a wrong
denominator, not a finding: the skeleton CAD's `trunk` is pelvis blades and
tail, while the MJCF's `trunk` is a 60×60×56 mm girdle **box**. Same name,
different object. Against its own geometry the rear girdle is
**4 474 kg/m³** and the front **5 565**, which is what a box packed to 100 %
with motors (202.5 cm³ and 790 g of them per girdle, at 3 903 kg/m³) should
look like.

The same pass also "discovered" that `params.py` omitted a 20 g paw pad. It does
not: ADR-0012's 20 g is a **cap**, `tomcat_leg_detail.per_link_mass()` computes
the pad at a few grams from its actual 22×26×9 mm cast form, and `params.py`
carries that. And `leg_hardware.py`, written here to add "missing" joint
hardware, duplicated `per_link_mass()` outright -- which `params.py` names as
its source in a comment that went unread. It has been deleted.

✅ **What survives is the machinery**, which is checked: `combine()` merges
solids into one rigid body with the parallel-axis theorem -- the step
`mass_properties.properties` deliberately omits, because OCP already reports
about each solid's own centre of mass and the shift only belongs where several
solids become one.

⚠️ **Densities other than the motor's are `[assumed]`** -- the skeleton CAD
is form, not material. The motor's is back-solved from its surveyed mass and is
the only one with a check behind it.
"""

from __future__ import annotations

import numpy as np

import mass_properties as MP

MM3 = 1e-9          # the CAD works in mm; volumes come back in mm^3
MM = 1e-3

#: 131.7 g in a Ø34.5 × 36.1 mm cylinder. The one measured density.
MOTOR_MASS = 0.1317


def combine(parts):
    """Merge `[(mass, com, I_about_com)]` into one rigid body.

    Parallel-axis each part's tensor out to the combined centre of mass. This is
    the step `mass_properties.properties` deliberately does *not* do — OCP
    already reports about each solid's own CoM, and the shift only belongs here,
    where several solids become one body.
    """
    parts = [p for p in parts if p[0] > 0.0]
    if not parts:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    M = sum(p[0] for p in parts)
    com = sum(p[0] * p[1] for p in parts) / M
    I = np.zeros((3, 3))
    for m, c, Ic in parts:
        d = c - com
        I = I + Ic + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return M, com, I


def solid_props(solid, density):
    """`mass_properties.properties` in metres, from a mm-unit CAD solid."""
    m, com, I = MP.properties(solid, density)
    # volume scales mm^3 -> m^3, lengths mm -> m, so inertia mm^5 -> m^5
    return m * MM3, com * MM, I * MM3 * MM * MM


def motor_as_solid(centre_mm, axis="z"):
    """One motor at a packaging cluster point, as (mass, com, inertia).

    Analytic rather than meshed: a solid cylinder's tensor is exact, and the
    packaging CAD already fixes the size and the axis.
    """
    import tomcat_packaging as TP
    r = TP.MOTOR_D / 2.0 * MM
    L = TP.MOTOR_L * MM
    m = MOTOR_MASS
    ir = m * (3 * r * r + L * L) / 12.0
    ia = m * r * r / 2.0
    I = np.diag([ir, ir, ia] if axis == "z" else [ir, ia, ir])
    return m, np.asarray(centre_mm, float) * MM, I


def report(bodies, motors_by_body, density_of):
    """Per-body totals plus the density each implies, for sanity-checking.

    A body whose implied density lands outside 500-4000 kg/m^3 is the signal
    that mass and volume still disagree — that was the whole finding here.
    """
    rows = []
    for name, solid in sorted(bodies.items()):
        parts = [solid_props(solid, density_of(name))]
        parts += motors_by_body.get(name, [])
        M, com, I = combine(parts)
        vol = solid_props(solid, 1.0)[0] + sum(
            33.75e-6 for _ in motors_by_body.get(name, []))
        rows.append(dict(name=name, mass=M, com=com, inertia=I, volume=vol,
                         density=(M / vol if vol > 0 else 0.0)))
    return rows
