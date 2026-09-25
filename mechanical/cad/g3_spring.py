# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""G3's series-elastic element, sized as a PART rather than a stiffness.

ADR-0105 fixed the rates -- **125 kN/m on every leg cable, 150 kN/m on the
spine** -- as the joint stiffness ADR-0026 asks for. This turns a rate into
something that can be drawn: which FORM the spring takes, how big it is, and
where the hard stop goes.

The argument is by ENERGY. A spring must store `F^2 / 2k` at the load it is
designed to, and every form has a ceiling on how much energy a cubic millimetre
of its material can hold without yielding or fatiguing. The volume that follows
is the size and the mass, before any drawing.

Load cases, from the plant (`power.walked_trajectory`, `mjcf_tendon`):

    fatigue   0 -> 140 N every step    the walked trot's worst cable (ankle)
    working   222.9 N                  the motor's peak, `TENSION_MAX`
    landing   516 N                    the hind hip's land transient (M111)

⚠️ **The hard stop is the largest single design lever here.** Without one the
spring has to absorb the landing: 1.065 J at 125 kN/m, 5.3x what the working
load stores. With a stop just past the motor's peak the spring never sees the
landing at all; the housing does. So the stop is not a safety feature added
to a spring -- it is what makes the spring small enough to build.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "kinematics", "src"))

from tomcat_kin import mjcf_tendon as MT  # noqa: E402
from tomcat_kin.params import DEFAULT_SPINE, DEFAULT_TENDON  # noqa: E402

#: The walked trot's worst motor-side cable tension (the hind ankle), N. The
#: load every step repeats, so the one that sets fatigue. [derived: M114,
#: `power.walked_trajectory` at the 36/34/22 arms]
F_FATIGUE = 140.4
#: The motor's peak, the most the drivetrain can pull. [derived: MT.TENSION_MAX]
F_WORKING = float(MT.TENSION_MAX)
#: Where the hard stop engages, as a multiple of the working load. The spring
#: must reach the motor's peak without touching it, and a stop too close costs
#: the compliance G3 exists for. [assumed]
STOP_MARGIN = 1.15
#: The hind hip's land-case cable tension, N -- the load the STOP carries.
#: [derived: `tomcat_leg_detail.live_loads`, M111]
F_LANDING = 516.0

#: ASTM A228 music wire. S_ut = A / d^m. [sourced: Shigley 10e Table 10-4]
MUSIC_A, MUSIC_M = 2211.0, 0.145          # MPa.mm^m, -
MUSIC_G, MUSIC_E = 81.7e3, 203.4e3        # MPa
STEEL_RHO = 7.85e-3                        # g/mm^3
#: 17-4PH H1025, machined: yield and the design stress a flexure is held to.
#: [sourced: AK Steel 17-4PH data sheet; 0.6 of yield is the design rule here]
PH17_E, PH17_YIELD = 197e3, 1000.0

#: Fatigue and static limits on the TORSIONAL stress in a helical compression
#: spring, as fractions of S_ut. 0.35 for 0-to-peak infinite life unpeened,
#: 0.45 at the stop with set removed. [sourced: Shigley 10e sec. 10-6, 10-9]
TAU_FATIGUE, TAU_STOP = 0.35, 0.45


def load_cases(k=None):
    """Deflection (mm) and stored energy (J) at each load for a rate `k` (N/m)."""
    k = float(DEFAULT_TENDON.series_k if k is None else k)
    f_stop = STOP_MARGIN * F_WORKING
    out = {}
    for name, F in (("fatigue", F_FATIGUE), ("working", F_WORKING),
                    ("stop", f_stop), ("landing", F_LANDING)):
        out[name] = {"F": F, "mm": F / k * 1e3, "J": 0.5 * F * F / k}
    return out


def size_compression_spring(k=None, od_max=12.0):
    """The lightest helical compression spring in music wire that meets G3.

    Rate `k` (N/m), torsional stress within `TAU_FATIGUE * S_ut` at the fatigue
    load and `TAU_STOP * S_ut` at the stop, at least three active coils, outside
    diameter no more than `od_max` (mm). Closed ends. Returns the design as a
    dict, or None.
    """
    k_mm = float(DEFAULT_TENDON.series_k if k is None else k) * 1e-3   # N/mm
    f_stop = STOP_MARGIN * F_WORKING
    best = None
    for d in np.arange(0.60, 2.51, 0.05):
        s_ut = MUSIC_A / d ** MUSIC_M
        for C in np.arange(5.0, 12.01, 0.25):
            D = C * d
            if D + d > od_max:
                continue
            n_a = MUSIC_G * d ** 4 / (8.0 * D ** 3 * k_mm)
            if n_a < 3.0:
                continue
            kb = (4.0 * C + 2.0) / (4.0 * C - 3.0)
            tau = lambda F: kb * 8.0 * F * D / (math.pi * d ** 3)   # noqa: E731
            if tau(F_FATIGUE) > TAU_FATIGUE * s_ut or tau(f_stop) > TAU_STOP * s_ut:
                continue
            n_t = n_a + 2.0
            mass = STEEL_RHO * math.pi * d * d / 4.0 * math.pi * D * n_t
            if best is None or mass < best["mass_g"]:
                solid = d * n_t
                travel = f_stop / k_mm
                best = {"d": float(d), "D": float(D), "OD": float(D + d),
                        "C": float(C), "n_active": float(n_a),
                        "mass_g": float(mass), "solid_mm": float(solid),
                        "stop_travel_mm": float(travel),
                        "free_mm": float(solid + 1.15 * travel),
                        "tau_fatigue_frac": float(tau(F_FATIGUE) / s_ut),
                        "tau_stop_frac": float(tau(f_stop) / s_ut)}
    return best


def flexure_volume(k=None, sigma=None, E=PH17_E):
    """Active material a machined planar flexure needs, mm^3, and its mass, g.

    A spiral arm is close to uniform moment, so its energy density is
    `sigma^2 / 6E`; it must hold the stop energy at `sigma`, by default 0.6 of
    17-4PH's yield. An upper bound on performance: real arms taper, and hub and
    rim add mass that stores nothing.
    """
    sigma = 0.6 * PH17_YIELD if sigma is None else sigma
    U = load_cases(k)["stop"]["J"] * 1e3                        # N.mm
    u = sigma * sigma / (6.0 * E)                               # N.mm / mm^3
    v = U / u
    return {"volume_mm3": v, "mass_g": v * STEEL_RHO,
            "energy_density": u}


def torsion_coil_volume(k=None):
    """The same bound for a helical TORSION spring, wire in bending (`sigma^2/8E`
    for a round section), at 0.6 S_ut for a 1.4 mm wire."""
    s_ut = MUSIC_A / 1.4 ** MUSIC_M
    sigma = 0.6 * s_ut
    U = load_cases(k)["stop"]["J"] * 1e3
    u = sigma * sigma / (8.0 * MUSIC_E)
    v = U / u
    return {"volume_mm3": v, "mass_g": v * STEEL_RHO, "energy_density": u}


#: Bending-stress limits on a helical TORSION spring, fractions of S_ut:
#: 0.50 at the fatigue load (0-to-peak, infinite life, unpeened) and 0.75 at the
#: stop. [sourced: Shigley 10e sec. 10-12, Table 10-6]
SIG_FATIGUE, SIG_STOP = 0.50, 0.75


def size_torsion_spring(k=None, R=None, od_max=24.0, d_range=(0.8, 3.0)):
    """The lightest helical torsion spring, coaxial with the spool, that is G3.

    The rate is rotational, `k_tors = k * R^2`, and so is the travel: the stop
    load `STOP_MARGIN * F_WORKING` is `STOP_MARGIN * T_pk` at the spool, a few
    tens of degrees. Rate `d^4 E / (64 D N_a)` per radian; bending stress
    `K_i 32 M / (pi d^3)`, `K_i = (4C^2 - C - 1) / (4C (C - 1))` on the inner
    fibre. Returns the design as a dict, or None.
    """
    k = float(DEFAULT_TENDON.series_k if k is None else k)
    R = float(DEFAULT_TENDON.motor_spool_radius if R is None else R) * 1e3  # mm
    k_rad = k * 1e-3 * R * R                                   # N.mm / rad
    M_f, M_s = F_FATIGUE * R, STOP_MARGIN * F_WORKING * R      # N.mm
    best = None
    for d in np.arange(d_range[0], d_range[1] + 1e-9, 0.05):
        s_ut = MUSIC_A / d ** MUSIC_M
        for D in np.arange(6.0, od_max, 0.25):
            C = D / d
            if C < 4.0 or D + d > od_max:
                continue
            n_a = d ** 4 * MUSIC_E / (64.0 * D * k_rad)
            if n_a < 1.5:
                continue
            ki = (4.0 * C * C - C - 1.0) / (4.0 * C * (C - 1.0))
            sig = lambda M: ki * 32.0 * M / (math.pi * d ** 3)   # noqa: E731
            if sig(M_f) > SIG_FATIGUE * s_ut or sig(M_s) > SIG_STOP * s_ut:
                continue
            legs = 2.0 * D                                       # two tangent legs
            mass = STEEL_RHO * math.pi * d * d / 4.0 * (math.pi * D * n_a + legs)
            if best is None or mass < best["mass_g"]:
                best = {"d": float(d), "D": float(D), "OD": float(D + d),
                        "C": float(C), "n_active": float(n_a),
                        "body_mm": float(d * (n_a + 1.0)),
                        "mass_g": float(mass), "k_rad": float(k_rad),
                        "stop_deg": float(math.degrees(M_s / k_rad)),
                        "working_deg": float(math.degrees(F_WORKING * R / k_rad)),
                        "sig_fatigue_frac": float(sig(M_f) / s_ut),
                        "sig_stop_frac": float(sig(M_s) / s_ut)}
    return best


#: Flexure materials: (E MPa, yield MPa, fully-reversed endurance MPa, density
#: g/mm^3). Endurance is the smooth rotating-bending figure; `SURFACE` knocks it
#: down for a machined or wire-EDM edge. [sourced: AK Steel 17-4PH, ATI C300,
#: ASM Ti-6Al-4V data sheets -- round figures]
FLEXURE_MATERIALS = {
    "17-4PH H1025":     (197e3, 1000.0, 480.0, 7.80e-3),
    "maraging C300":    (190e3, 2000.0, 700.0, 8.00e-3),
    "Ti-6Al-4V":        (114e3,  880.0, 510.0, 4.43e-3),
}
SURFACE = 0.7


def size_flexure(k=None, material="maraging C300", overhead=2.0):
    """A BIDIRECTIONAL planar torsional flexure for G3, sized by energy.

    ⚠️ **Bidirectional is the requirement, not a preference.** The shipped
    transmission drives a joint's two cables from ONE spool (ADR-0008's motor
    count, `test_ONE_SPOOL_PER_PAIR`), so the spring behind that spool sees the
    pair's torque in both senses. A helical torsion spring is strong winding and
    weak unwinding; a planar flexure is symmetric by construction.

    The stress at the stop is the smaller of 0.6 yield and what keeps the
    FULLY-REVERSED fatigue load (the walked trot's worst, `F_FATIGUE`) under the
    surface-corrected endurance limit. Active volume is the stop energy over
    `sigma^2 / 6E`; `overhead` adds the hub and rim that store nothing.
    """
    E, yld, se, rho = FLEXURE_MATERIALS[material]
    f_stop = STOP_MARGIN * F_WORKING
    sig = min(0.6 * yld, SURFACE * se * f_stop / F_FATIGUE)
    U = load_cases(k)["stop"]["J"] * 1e3
    u = sig * sig / (6.0 * E)
    v = U / u
    return {"material": material, "sigma_stop": sig,
            "binding": "yield" if sig == 0.6 * yld else "fatigue",
            "active_mm3": v, "active_g": v * rho, "part_g": v * rho * overhead}


def report():
    for label, k in (("LEG  ", DEFAULT_TENDON.series_k),
                     ("SPINE", DEFAULT_SPINE.series_k)):
        print("%s G3, %.3g N/m" % (label, k))
        for name, c in load_cases(k).items():
            print("   %-8s %6.1f N  %5.2f mm  %6.3f J" % (name, c["F"], c["mm"], c["J"]))
        t = size_torsion_spring(k)
        print("   torsion spring: d %.2f  D %.2f  OD %.2f mm, %.2f active turns,"
              " body %.1f mm, %.2f g; working %.1f deg, STOP at %.1f deg"
              % (t["d"], t["D"], t["OD"], t["n_active"], t["body_mm"],
                 t["mass_g"], t["working_deg"], t["stop_deg"]))
        c = size_compression_spring(k, od_max=40.0)
        print("   compression spring in the cable line: %s"
              % ("NONE fits -- 2 mm of travel at this rate needs a >4.6 mm wire"
                 if c is None else "d %.2f OD %.1f, %.1f g" % (c["d"], c["OD"], c["mass_g"])))
        for mat in FLEXURE_MATERIALS:
            fl = size_flexure(k, mat)
            print("   BIDIRECTIONAL flexure, %-15s sigma %4.0f MPa (%s-bound): %5.0f mm^3,"
                  " %4.1f g active, ~%4.1f g part"
                  % (mat, fl["sigma_stop"], fl["binding"], fl["active_mm3"],
                     fl["active_g"], fl["part_g"]))
        f, t = flexure_volume(k), torsion_coil_volume(k)
        print("   planar flexure (17-4PH) >= %.0f mm^3, %.1f g active;"
              "  torsion coil >= %.0f mm^3, %.1f g"
              % (f["volume_mm3"], f["mass_g"], t["volume_mm3"], t["mass_g"]))


if __name__ == "__main__":
    report()
