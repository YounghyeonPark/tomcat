# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Mass, centre of mass and inertia tensor for a CAD solid.

⚠️ **This is the thing `mass_closure.py` says it does not have.** Its own note:

    `inertia_ratio` is used as a **first-order proxy** for the operational-space
    inertia ratio. The real quantity is `Lambda = (J M^-1 J^T)^-1` ... which
    needs the per-link inertia tensors this ...

and `mjcf_tendon.py` builds every link as a capsule with `_box_inertia`-style
uniform density. So the simulated robot's inertia is a hand-written
approximation of a CAD model that has been sitting next to it.

OCP (the OpenCascade binding under build123d) computes all three exactly. What
it cannot supply is **density**: the skeleton CAD is form, not material, so a
density has to be asserted per part and that assertion is the honest weak point.
"""

from __future__ import annotations

import numpy as np

try:
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
except ImportError as exc:                                   # pragma: no cover
    raise ImportError("OCP is needed; it ships with build123d") from exc

#: kg/m^3. `[assumed]` until `mechanical/` fixes a material per part.
DENSITY = {
    "alu": 2700.0,        # 6061 bracket / girdle plate
    "pla": 1240.0,        # printed form parts
    "petg": 1270.0,
    "nylon_cf": 1200.0,   # printed structural
    "steel": 7850.0,      # shafts, fasteners
    "motor": 3903.0,      # 131.7 g in a 34.5x36.1 cylinder -> back-solved.
                          # ⚠️ was 2660 by guess, 46 % low, until the
                          # verification back-solved it from the surveyed mass.
}


def properties(solid, density: float):
    """Return `(mass_kg, com_xyz_m, inertia_3x3_about_com)`.

    ⚠️ **OCP already reports about the CENTRE OF MASS**, and a first version
    of this assumed the origin and subtracted a parallel-axis term that was
    never added. A box 100 mm off-origin came back with **negative** inertia --
    physically impossible, and MuJoCo would either reject it or simulate
    nonsense. Verified at four offsets: the tensor does not move, so no shift is
    applied. Only the density scaling is ours.
    """
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(
        solid.wrapped if hasattr(solid, "wrapped") else solid, p)
    vol = float(p.Mass())                       # OCP calls volume "Mass"
    c = p.CentreOfMass()
    com = np.array([c.X(), c.Y(), c.Z()], float)
    m = np.asarray(p.MatrixOfInertia().__class__ and
                   [[p.MatrixOfInertia().Value(i, j) for j in (1, 2, 3)]
                    for i in (1, 2, 3)], float)
    return vol * float(density), com, m * float(density)


def principal(I):
    """Diagonal inertia and the quaternion MJCF wants, from a 3x3 tensor."""
    w, V = np.linalg.eigh(I)
    if np.linalg.det(V) < 0:
        V[:, 0] = -V[:, 0]
    q = np.zeros(4)
    t = np.trace(V)
    if t > 0:
        s = math_sqrt(t + 1.0) * 2.0
        q[:] = [0.25 * s, (V[2, 1] - V[1, 2]) / s,
                (V[0, 2] - V[2, 0]) / s, (V[1, 0] - V[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(V)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math_sqrt(1.0 + V[i, i] - V[j, j] - V[k, k]) * 2.0
        q[0] = (V[k, j] - V[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (V[j, i] + V[i, j]) / s
        q[1 + k] = (V[k, i] + V[i, k]) / s
    return w, q / np.linalg.norm(q)


def math_sqrt(x):
    import math
    return math.sqrt(max(x, 1e-300))


def report(named_solids, density_of):
    """`[(name, solid)]` -> a table plus the totals, for eyeballing."""
    rows, M, first = [], 0.0, np.zeros(3)
    for name, solid in named_solids:
        rho = density_of(name) if callable(density_of) else density_of
        m, com, I = properties(solid, rho)
        rows.append((name, m, com, I))
        M += m
        first += m * com
    return rows, M, (first / M if M else first)
