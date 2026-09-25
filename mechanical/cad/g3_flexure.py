# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""G3 drawn: the cables' series-elastic flexure.

ADR-0107 chose the form -- a BIDIRECTIONAL planar torsional flexure between
the motor rotor and the spool, in Ti-6Al-4V, with a hard stop just past the
motor's peak -- and `g3_spring.py` bounded its size by energy. This draws it.

The part, in one piece: a HUB (bolted to the rotor), spiral ARMS, a RIM
(bolted to the spool). Nothing else -- M120 moved the stop out of it.

⚠️ **M120: the stop left the flexure, because the trunk had no room for it.**
M117 drew the stop as its own layer above the arms (three arms of 0.75 turn
sweep every angle, so it cannot share their plane), 1.5 mm of the part's 5.24.
Measured against the trunk WITH its bulkhead pads, a girdle body has 8.0 mm of
row length to spare for two rows and spine body 2 has **2.7 mm** for one --
ADR-0107's "16.4 / 6.9 mm" left the pads out. The spine part at 5.98 mm could
not go in at all. Now two hardened dowels pressed through the HUB stand up into
arc slots in the SPOOL's face, inside the spool's own 8 mm: the stop costs no
length. The spool, its slots and its own bearing on the rotor axis are the
spool drawing's, `[owed]`; this part carries torque only.

⚠️ **And the arm WIDTH was a free choice, left at 1.6 mm with stress to
spare.** The rate goes as `T b^3` and the stop stress as `b`, so at the stress
limit (`SIGMA_USE` of it) every extra tenth of width takes a cube off the
thickness. The ACTIVE VOLUME does not move -- it is the energy, 714 mm^3 on the
leg, 595 on the spine -- so the shape only trades width for thickness, and
the trunk wants thickness.

Sizing is beam theory, not FEA: each arm is a curved beam of length `l`,
in-plane width `b` and axial thickness `T` under a near-uniform moment, so

    k_tors = N E T b^3 / (12 l)          sigma = E b theta / (2 l)

`b` is set by the stop stress, `T` by the rate. ⚠️ Clamped-clamped arms on a
rigid hub carry some tension and shear this ignores, which stiffens the real
part; the rate is `[owed]` to an FEA or a bench test before it is cut.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "kinematics", "src"))

import g3_spring as G  # noqa: E402
from tomcat_kin.params import DEFAULT_TENDON  # noqa: E402

#: Ti-6Al-4V (ADR-0107). [sourced: ASM data sheet, round figures]
E_TI, YIELD_TI, SE_TI, RHO_TI = G.FLEXURE_MATERIALS["Ti-6Al-4V"]

#: (arms, turns) per cable group. The leg keeps M117's three arms; the spine,
#: with 2.7 mm to live in, takes two arms of a full turn -- longer arms, thinner
#: part (`T` goes as `1/l^2` at a fixed stress).
VARIANTS = {"leg": (3, 0.75), "spine": (2, 1.0)}
R_BORE = 2.0            # mm, rotor output pilot
R_HUB = 5.0             # mm, hub outer radius -- the arms start here
R_RIM_IN = 14.0         # mm, rim inner radius -- the arms end here
R_RIM_OUT = 15.0        # mm, the rim is a thin RING ...
EAR = (3, 15.3, 1.7)    # ... with three bolt ears: count, centre radius, ear radius
#: ⚠️ A full-width rim was two thirds of the part's mass (6.8 of 12.5 g): a
#: ring only has to hold the arm ends, and only the bolts need the width. Ears
#: put it where the bolts are. Outer reach 17.0 mm, a 34 mm envelope inside the
#: 34.5 mm can.
ROOT = 0.4              # mm, how far each arm is buried in hub and rim
#: Stop stress as a fraction of its bound (0.6 yield); the arm width is solved
#: to land here.
SIGMA_USE = 0.95
#: Axial clearance each side of the part: rotor face below, spool face above.
STACK_CLEAR = 0.3
BOLT_HUB = (2, 3.75, 0.9)  # count, circle radius, hole radius (M1.6)
BOLT_RIM = (3, 15.3, 0.9)  # a first draft put 2.2 mm holes in a 2 mm rim
#: The stop dowels: count, circle radius, hole radius (Ø2 hardened pin), on the
#: hub bolts' circle at 90 deg to them -- a 5 mm hub has room for four holes.
#: Pressed into the rotor flange, they also locate the hub on it.
STOP_PIN = (2, 3.75, 1.0)


def _arm_centreline(turns, b, n=240):
    """(phi, r) along one arm, hub to rim, as an Archimedean spiral."""
    # the arm runs 0.4 mm INTO the hub and the rim: started tangent to them,
    # the union shares only a face and OpenCascade returns a null solid
    r0 = R_HUB + 0.5 * b - ROOT
    r1 = R_RIM_IN - 0.5 * b + ROOT
    phi = np.linspace(0.0, 2.0 * math.pi * turns, n)
    return phi, r0 + (r1 - r0) * phi / phi[-1]


def arm_length(turns, b):
    """Arc length of one arm's centreline, mm."""
    phi, r = _arm_centreline(turns, b, 2000)
    dr = np.gradient(r, phi)
    return float(np.trapezoid(np.sqrt(r * r + dr * dr), phi))


def _variant(k):
    k = float(DEFAULT_TENDON.series_k if k is None else k)
    return k, ("leg" if k == float(DEFAULT_TENDON.series_k) else "spine")


def design(k=None):
    """Width from the stop stress, thickness from the rate, and what follows."""
    k, name = _variant(k)
    n, turns = VARIANTS[name]
    R = float(DEFAULT_TENDON.motor_spool_radius) * 1e3                # mm
    k_rad = k * 1e-3 * R * R                                          # N.mm/rad
    th_stop = math.radians(G.size_torsion_spring(k)["stop_deg"])
    th_fat = th_stop * G.F_FATIGUE / (G.STOP_MARGIN * G.F_WORKING)
    target = SIGMA_USE * 0.6 * YIELD_TI
    b = 1.6
    for _ in range(20):                   # sigma = E b theta / 2l, and l(b) weakly
        b = 2.0 * arm_length(turns, b) * target / (E_TI * th_stop)
    l = arm_length(turns, b)
    T = 12.0 * l * k_rad / (n * E_TI * b ** 3)
    sig = lambda th: E_TI * b * th / (2.0 * l)                        # noqa: E731
    phi, r = _arm_centreline(turns, b)
    pitch = (r[-1] - r[0]) / (n * turns)
    # the stop pins take the landing's whole spool torque, shared
    f_pin = G.F_LANDING * R / (STOP_PIN[0] * STOP_PIN[1])
    return {"variant": name, "arms": n, "turns": turns, "b": b,
            "k_rad": k_rad, "arm_length": l, "T": T,
            "stack": T + 2.0 * STACK_CLEAR,
            "stop_deg": math.degrees(th_stop),
            "sigma_stop": sig(th_stop), "sigma_fatigue": sig(th_fat),
            "sigma_stop_limit": 0.6 * YIELD_TI,
            "sigma_fatigue_limit": G.SURFACE * SE_TI,
            "arm_gap": pitch - b, "active_mm3": n * b * T * l,
            "pin_N": f_pin,
            "pin_shear_MPa": f_pin / (math.pi * STOP_PIN[2] ** 2),
            "pin_bearing_MPa": f_pin / (2.0 * STOP_PIN[2] * T)}


def stack(role):
    """Axial length G3 adds to a row of `role`'s actuators, mm."""
    from tomcat_kin.params import DEFAULT_SPINE
    return design(DEFAULT_SPINE.series_k if role == "spine" else None)["stack"]


def build(k=None):
    """The part, as a build123d solid (mm)."""
    from build123d import Cylinder, Polygon, Pos, extrude

    d = design(k)
    T, b, n = d["T"], d["b"], d["arms"]

    hub = Pos(0, 0, T / 2) * Cylinder(R_HUB, T) - Cylinder(R_BORE, 3 * T)
    rim = (Pos(0, 0, T / 2) * Cylinder(R_RIM_OUT, T)
           - Cylinder(R_RIM_IN, 3 * T))
    for j in range(EAR[0]):
        a = 2.0 * math.pi * j / EAR[0] + math.pi / EAR[0]
        rim += Pos(EAR[1] * math.cos(a), EAR[1] * math.sin(a), T / 2) * Cylinder(EAR[2], T)
    part = hub + rim

    phi, r = _arm_centreline(d["turns"], b)
    for i in range(n):
        a0 = 2.0 * math.pi * i / n
        outer = [((rr + b / 2) * math.cos(a0 + p), (rr + b / 2) * math.sin(a0 + p))
                 for p, rr in zip(phi, r)]
        inner = [((rr - b / 2) * math.cos(a0 + p), (rr - b / 2) * math.sin(a0 + p))
                 for p, rr in zip(phi, r)]
        # overlap into hub and rim so the arm is fused, not touching
        face = Polygon(*(outer + inner[::-1]), align=None)
        part = part + extrude(face, amount=T)
        assert part.is_valid, "arm %d did not fuse" % i

    # hub bolts at 0/180 deg, the stop dowels at 90/270, rim bolts in the ears
    for (count, rc, rh), off in ((BOLT_HUB, 0.0), (STOP_PIN, math.pi / 2),
                                 (BOLT_RIM, math.pi / 3)):
        for j in range(count):
            a = 2.0 * math.pi * j / count + off
            part -= Pos(rc * math.cos(a), rc * math.sin(a), T / 2) * Cylinder(rh, 3 * T)
    assert part.is_valid
    return part, d


def report(k=None):
    part, d = build(k)
    vol = part.volume
    print("G3 %s FLEXURE, Ti-6Al-4V, k = %.3g N/m -> %.2f N.m/rad"
          % (d["variant"].upper(), _variant(k)[0], d["k_rad"] / 1e3))
    print("   disc  OD %.0f mm, hub OD %.0f mm, %d arms x %.2f turn, arm %.2f mm wide x %.2f mm thick"
          % (2 * R_RIM_OUT, 2 * R_HUB, d["arms"], d["turns"], d["b"], d["T"]))
    print("   arm length %.1f mm, radial gap between arms %.2f mm, active %.0f mm^3"
          % (d["arm_length"], d["arm_gap"], d["active_mm3"]))
    print("   stress at the stop    %.0f MPa  (limit %.0f, 0.6 yield)"
          % (d["sigma_stop"], d["sigma_stop_limit"]))
    print("   stress, walked trot   %.0f MPa  (limit %.0f, surface-corrected endurance)"
          % (d["sigma_fatigue"], d["sigma_fatigue_limit"]))
    print("   hard stop +/-%.1f deg on %d dowels: %.0f N each, shear %.0f / bearing %.0f MPa"
          % (d["stop_deg"], STOP_PIN[0], d["pin_N"], d["pin_shear_MPa"],
             d["pin_bearing_MPa"]))
    print("   in the row %.2f mm, mass %.2f g" % (d["stack"], vol * RHO_TI))
    return part, d


def mass_g(k=None):
    """The drawn part's mass, g (needs CAD)."""
    return build(k)[0].volume * RHO_TI


if __name__ == "__main__":
    from build123d import export_step, export_stl
    from tomcat_girdle import render_png
    from tomcat_kin.params import DEFAULT_SPINE
    for tag, k in (("", None), ("_spine", DEFAULT_SPINE.series_k)):
        part, _ = report(k)
        export_step(part, os.path.join(HERE, "g3_flexure%s.step" % tag))
        export_stl(part, os.path.join(HERE, "g3_flexure%s.stl" % tag))
        render_png(part, os.path.join(HERE, "g3_flexure%s.png" % tag),
                   elev=55, azim=-60)
    render_png(build()[0], os.path.join(HERE, "g3_flexure_top.png"),
               elev=90, azim=-90)
