# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""G3 drawn: the leg cables' series-elastic flexure, with its hard stop.

ADR-0107 chose the form -- a BIDIRECTIONAL planar torsional flexure between
the motor rotor and the spool, in Ti-6Al-4V, with a hard stop just past the
motor's peak -- and `g3_spring.py` bounded its size by energy. This draws it.

The part, in one piece:

    z = 0 .. T          the flexure: a HUB (bolted to the rotor), three spiral
                        ARMS, a RIM (bolted to the spool)
    z = T+g .. T+g+S    the hard stop, a layer above the arms: two lugs on the
                        hub, two on the rim, clearing each other by the stop
                        angle each way

⚠️ **The stop cannot share the arms' plane.** Three arms of 0.75 turn each
sweep every angle between hub and rim two or three times over, so there is no
angle left where a hub feature can meet a rim feature. The stop sits in its own
layer, carried up by the hub and the rim, which the arms never enter.

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

N_ARMS = 3
R_BORE = 2.0            # mm, rotor output pilot
R_HUB = 5.0             # mm, hub outer radius -- the arms start here
R_RIM_IN = 14.0         # mm, rim inner radius -- the arms end here
R_RIM_OUT = 15.0        # mm, the rim is a thin RING ...
EAR = (3, 15.3, 1.7)    # ... with three bolt ears: count, centre radius, ear radius
#: ⚠️ A full-width rim was two thirds of the part's mass (6.8 of 12.5 g): a
#: ring only has to hold the arm ends and the rim lugs, and only the bolts need
#: the width. Ears put it where the bolts are. Outer reach 17.0 mm, a 34 mm
#: envelope inside the 34.5 mm can.
TURNS = 0.75            # each arm's sweep
ROOT = 0.4              # mm, how far each arm is buried in hub and rim
ARM_B = 1.6             # mm, in-plane width, set by the stop stress
STOP_GAP = 0.3          # mm, axial clearance between arms and the stop layer
STOP_T = 1.2            # mm, stop layer thickness
RIM_LUG_DEG = 30.0      # angular width of each rim lug -- narrow, at large radius
#: ⚠️ The wide lugs go on the HUB. The free play is fixed by the two widths
#: together, `(180 - hub - rim) / 2`, so the arc can sit on either part; at the
#: hub's small radius it costs a third of the material it does at the rim's.
HUB_LUG_R = 9.0         # mm, hub lugs reach out to here
RIM_LUG_R = 7.5         # mm, rim lugs reach in to here
BOLT_HUB = (3, 3.75, 0.9)  # count, circle radius, hole radius (M1.6 / dowel)
BOLT_RIM = (3, 15.3, 0.9)  # a first draft put 2.2 mm holes in a 2 mm rim


def _arm_centreline(n=240):
    """(phi, r) along one arm, hub to rim, as an Archimedean spiral."""
    # the arm runs 0.4 mm INTO the hub and the rim: started tangent to them,
    # the union shares only a face and OpenCascade returns a null solid
    r0 = R_HUB + 0.5 * ARM_B - ROOT
    r1 = R_RIM_IN - 0.5 * ARM_B + ROOT
    phi = np.linspace(0.0, 2.0 * math.pi * TURNS, n)
    return phi, r0 + (r1 - r0) * phi / phi[-1]


def arm_length():
    """Arc length of one arm's centreline, mm."""
    phi, r = _arm_centreline(2000)
    dr = np.gradient(r, phi)
    return float(np.trapezoid(np.sqrt(r * r + dr * dr), phi))


def design(k=None):
    """Thickness for the target rate, and the stresses that follow."""
    k = float(DEFAULT_TENDON.series_k if k is None else k)
    R = float(DEFAULT_TENDON.motor_spool_radius) * 1e3                # mm
    k_rad = k * 1e-3 * R * R                                          # N.mm/rad
    l = arm_length()
    T = 12.0 * l * k_rad / (N_ARMS * E_TI * ARM_B ** 3)
    th_stop = math.radians(G.size_torsion_spring(k)["stop_deg"])
    th_fat = th_stop * G.F_FATIGUE / (G.STOP_MARGIN * G.F_WORKING)
    sig = lambda th: E_TI * ARM_B * th / (2.0 * l)                    # noqa: E731
    phi, r = _arm_centreline()
    pitch = (r[-1] - r[0]) / (N_ARMS * TURNS)
    # the stop's own geometry: free play each way = (180 - hub lug - rim lug)/2
    hub_lug = 180.0 - RIM_LUG_DEG - 2.0 * math.degrees(th_stop)
    return {"k_rad": k_rad, "arm_length": l, "T": T,
            "stop_deg": math.degrees(th_stop), "hub_lug_deg": hub_lug,
            "sigma_stop": sig(th_stop), "sigma_fatigue": sig(th_fat),
            "sigma_stop_limit": 0.6 * YIELD_TI,
            "sigma_fatigue_limit": G.SURFACE * SE_TI,
            "arm_gap": pitch - ARM_B}


def build(k=None):
    """The part, as a build123d solid (mm)."""
    from build123d import Cylinder, Polygon, Pos, Rot, extrude

    d = design(k)
    T = d["T"]
    top = T + STOP_GAP + STOP_T

    hub = Pos(0, 0, top / 2) * Cylinder(R_HUB, top) - Cylinder(R_BORE, 3 * top)
    rim = (Pos(0, 0, top / 2) * Cylinder(R_RIM_OUT, top)
           - Cylinder(R_RIM_IN, 3 * top))
    for j in range(EAR[0]):
        a = 2.0 * math.pi * j / EAR[0] + math.pi / EAR[0]
        rim += Pos(EAR[1] * math.cos(a), EAR[1] * math.sin(a), top / 2) * Cylinder(EAR[2], top)
    part = hub + rim

    phi, r = _arm_centreline()
    for i in range(N_ARMS):
        a0 = 2.0 * math.pi * i / N_ARMS
        outer = [((rr + ARM_B / 2) * math.cos(a0 + p), (rr + ARM_B / 2) * math.sin(a0 + p))
                 for p, rr in zip(phi, r)]
        inner = [((rr - ARM_B / 2) * math.cos(a0 + p), (rr - ARM_B / 2) * math.sin(a0 + p))
                 for p, rr in zip(phi, r)]
        # overlap into hub and rim so the arm is fused, not touching
        face = Polygon(*(outer + inner[::-1]), align=None)
        part = part + extrude(face, amount=T)
        assert part.is_valid, "arm %d did not fuse" % i

    z0 = T + STOP_GAP
    def sector(r_in, r_out, width_deg, centre_deg):
        n = 40
        a = np.radians(np.linspace(centre_deg - width_deg / 2,
                                   centre_deg + width_deg / 2, n))
        pts = ([(r_out * math.cos(t), r_out * math.sin(t)) for t in a]
               + [(r_in * math.cos(t), r_in * math.sin(t)) for t in a[::-1]])
        return Pos(0, 0, z0) * extrude(Polygon(*pts, align=None), amount=STOP_T)

    for c in (0.0, 180.0):                      # hub lugs, turn with the rotor
        part += sector(R_HUB - 0.5, HUB_LUG_R, d["hub_lug_deg"], c)
    for c in (90.0, 270.0):                     # rim lugs, turn with the spool
        part += sector(RIM_LUG_R, R_RIM_IN + 0.5, RIM_LUG_DEG, c)

    for count, rc, rh in (BOLT_HUB, BOLT_RIM):
        for j in range(count):
            a = 2.0 * math.pi * j / count + (math.pi / count if rc > R_HUB else 0.0)
            part -= Pos(rc * math.cos(a), rc * math.sin(a), top / 2) * Cylinder(rh, 3 * top)
    return part, d


def report(k=None):
    part, d = build(k)
    vol = part.volume
    print("G3 LEG FLEXURE, Ti-6Al-4V, k = %.3g N/m -> %.2f N.m/rad"
          % (DEFAULT_TENDON.series_k if k is None else k, d["k_rad"] / 1e3))
    print("   disc  OD %.0f mm, hub OD %.0f mm, %d arms x %.2f turn, arm %.2f mm wide x %.2f mm thick"
          % (2 * R_RIM_OUT, 2 * R_HUB, N_ARMS, TURNS, ARM_B, d["T"]))
    print("   arm length %.1f mm, radial gap between arms %.2f mm"
          % (d["arm_length"], d["arm_gap"]))
    print("   stress at the stop    %.0f MPa  (limit %.0f, 0.6 yield)"
          % (d["sigma_stop"], d["sigma_stop_limit"]))
    print("   stress, walked trot   %.0f MPa  (limit %.0f, surface-corrected endurance)"
          % (d["sigma_fatigue"], d["sigma_fatigue_limit"]))
    print("   hard stop +/-%.1f deg: hub lugs %.1f deg, rim lugs %.0f deg"
          % (d["stop_deg"], d["hub_lug_deg"], RIM_LUG_DEG))
    print("   axial %.2f mm, volume %.0f mm^3, mass %.2f g"
          % (d["T"] + STOP_GAP + STOP_T, vol, vol * RHO_TI))
    return part, d


if __name__ == "__main__":
    part, _ = report()
    from build123d import export_step, export_stl
    from tomcat_girdle import render_png
    export_step(part, os.path.join(HERE, "g3_flexure.step"))
    export_stl(part, os.path.join(HERE, "g3_flexure.stl"))
    render_png(part, os.path.join(HERE, "g3_flexure.png"), elev=55, azim=-60)
    render_png(part, os.path.join(HERE, "g3_flexure_top.png"), elev=90, azim=-90)
