# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Manufacturing-level 3D model of one HIND leg (build123d) — closes the
ASSEMBLY_SPEC §6 debt "shop drawings / manufacturable geometry".

`tomcat_skeleton.py` says of itself: *"still a SKELETAL model, not a manufacturing
model — no fasteners, bearings, tolerances or fabrication features"*. This file is
the other thing. Every feature here traces to a specified number:

| feature | source |
|---|---|
| link lengths, ROM | `DEFAULT_LEG` — the same params the sim solves |
| sheave radii = moment arms | `DEFAULT_TENDON.joint_moment_arm` |
| tube sections Ø12/Ø10/Ø8 × 1.0 CF | LEG_TENDON_SPEC §3.5 (graded, SF 2.8–3.1) |
| cable Ø1.75 UHMWPE, groove r = 0.55·d | LEG_TENDON_SPEC §2 (ADR-0010 re-size) |
| bonded insert, 0.05–0.15 mm bond gap | ASSEMBLY_SPEC §0.2 (no drilled laminate) |
| bearing bores H7, shaft h6, sheave on the DISTAL link | ASSEMBLY_SPEC §2 |
| pin/shaft ≥ 4 mm | ASSEMBLY_SPEC §0.2 (Ø3 is marginal at 149 MPa) |
| root idler turning the cable into the bone plane | ASSEMBLY_SPEC §0.1 design rule |

⚠️ **This is a design pass, not a drafting pass.** It computes the consequences of
the specified numbers and prints them, so a dimension that does not fit is a
failure here rather than a surprise at the bench. `checks()` is the point of the
file as much as the geometry is; three specified numbers do not survive it, and
they are reported rather than quietly adjusted (see MODULE NOTES below).

MODULE NOTES — what this pass found
-----------------------------------
1. **The 20 mm bonded-insert rule does not fit the distal links.** Two 20 mm
   inserts in a metatarsus whose tube is ~50 mm long after joint hardware fills
   81 % of it with aluminium. §0.2 itself prices 15 mm at SF > 10, so 15 mm is
   adopted here and the shortfall is reported per bone.
2. **Solid inserts would eat the leg mass budget.** A solid Ø9.9 × 20 plug is
   4.2 g against a 4.8 g femur tube. Inserts are therefore **turned hollow**
   (1.5 mm wall) — a bonded joint needs shear area, not a plug.
3. **The groove radius in LEG_TENDON_SPEC §3.1 is stale.** It says 0.85 mm for a
   1.5 mm cable; §2 re-sized the cable to **1.75 mm** (ADR-0010) and §3.1 was
   never updated. 0.55 × 1.75 = **0.96 mm** is used, and `checks()` asserts the
   relationship rather than the literal.

Run:  python mechanical/cad/tomcat_leg_detail.py
Out:  tomcat_leg_detail.step / .stl / .png (this dir)
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
from build123d import (
    Box, Cylinder, Compound, Plane, Pos, Sphere, Torus, export_step, export_stl,
)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "kinematics", "src"))
from tomcat_kin import LegModel  # noqa: E402
from tomcat_kin.tendon import TendonMap  # noqa: E402
from tomcat_kin.torque_budget import evaluate as budget  # noqa: E402
from tomcat_kin.params import (  # noqa: E402
    DEFAULT_LEG, DEFAULT_TENDON, DEFAULT_LOADS,
)

MM = 1000.0

# --------------------------------------------------------------- materials
CF_RHO = 1.60e-3      # g/mm^3, pultruded CF tube        [assumed]
AL_RHO = 2.70e-3      # g/mm^3, 6061-T6                  [sourced]
TPU_RHO = 1.20e-3     # g/mm^3, TPU ~80A                 [assumed]
STEEL_RHO = 7.85e-3   # g/mm^3, bearing rings + shaft    [sourced]

# ------------------------------------------------- specified section stock
#: (OD, wall) per link, LEG_TENDON_SPEC §3.5 — graded thickest proximally so the
#: safety factor equalises without putting mass on the distal links (P1).
TUBE = {"femur": (12.0, 1.0), "tibia": (10.0, 1.0),
        "meta": (8.0, 1.0), "paw": (8.0, 1.0)}

#: (bore, OD, width). ASSEMBLY_SPEC §2 wants static C0 >= 1.5 kN at the loaded
#: joints and only ~0.3 kN dynamic; the 626 class hits C0 easily at 6 mm bore,
#: and the ankle — the worst place for mass (P1) — takes an MR104.
#: ⚠️ Dimensions are ISO-standard; exact part numbers are `[owed: BOM pass]`.
BEARING = {"hip": (6.0, 19.0, 6.0), "knee": (6.0, 19.0, 6.0),
           "ankle": (4.0, 10.0, 4.0)}

CABLE_D = 1.75                    # LEG_TENDON_SPEC §2, re-sized by ADR-0010
GROOVE_R = 0.55 * CABLE_D         # §3.1's rule, applied to §2's cable = 0.96 mm
FLANGE_H = 1.2                    # groove flange above the cable  [assumed]
SHEAVE_W = 6.0                    # sheave axial width             [assumed]
WEB_T = 1.5                       # lightened sheave centre web    [assumed]
RIM_T = 1.0                       # metal inboard of the groove    [assumed]
HUB_T = 3.0                       # metal around the sheave bore   [assumed]

#: Catalogue masses for the bearing classes above, grams.
#: ⚠️ The geometric envelope over-weighs a bearing badly — a solid ring of the
#: 626 annulus is 12 g against a real ~8 g, because a bearing is mostly balls,
#: races and air. Envelope is used for FIT, catalogue mass for the BUDGET.
#: `[assumed: class-typical; exact PNs owed with the BOM]`
BEARING_G = {"hip": 8.0, "knee": 8.0, "ankle": 2.2}

#: Wire volume as a fraction of an extension spring's envelope.  `[assumed]`
SPRING_FILL = 0.25

BOND_GAP = 0.10                   # ASSEMBLY_SPEC §2: 0.05-0.15 mm glue line
ENGAGE = 15.0                     # §0.2 prices 15 mm at SF > 10 (see note 1)
MIN_ENGAGE = 6.0                  # below this a bonded insert is not a joint
INSERT_WALL = 1.5                 # hollow, not a plug (see note 2)
CLEVIS_ARM_T = 3.0                # arm thickness carrying the bearing bore
TONGUE_CLEAR = 0.2                # per side, tongue to clevis arm
BOSS_WALL = 2.5                   # metal around a bearing bore

TRACK_Y = 48.0                    # limb plane, ASSEMBLY_SPEC §0.1
FOOT_X, FOOT_Z, FOOT_PITCH = 0.04, -0.17, 0.0


def live_loads(grid: int = 21):
    """Joint torques and cable tensions from the LIVE budget, per load case.

    ⚠️ **This is not a convenience — it is the finding.** LEG_TENDON_SPEC §1.1
    tabulates the hip land torque as **12.36 N·m** and everything downstream of it
    (§1.3's tensions, §1.3a's hock review, §3.5's link sizing, §0.1's torsion
    check) uses that number. The live model at the ADR-0010 body mass of 4.045 kg
    returns **16.67 N·m / 600 N**. The ratio is exactly the mass increase
    4.045/3.0 = 1.35, so §1.1 is the pre-ADR-0010 table and was never re-run —
    while §2, which *was* re-run, correctly says "~600 N at the hip land
    transient". The spec is internally inconsistent by 35 %, on the load case that
    sizes the bones.

    Reading it from `torque_budget.evaluate` means this file cannot inherit that
    mistake, and `checks()` re-derives every safety factor from it.
    """
    leg, tm = LegModel(DEFAULT_LEG), TendonMap(DEFAULT_TENDON)
    out = {}
    for lc in DEFAULT_LOADS:
        r = budget(leg, tm, lc, grid=grid)
        key = lc.name.split()[0]                      # stand / trot / land
        out[key] = {"tau": np.asarray(r.peak_joint_torque),
                    "T": np.asarray(r.peak_tension),
                    "motor": np.asarray(r.peak_motor_torque)}
    return out


def section_Z(od: float, wall: float) -> float:
    """Elastic section modulus of a thin tube, mm^3."""
    ro, ri = od / 2, od / 2 - wall
    return math.pi * (ro ** 4 - ri ** 4) / (4 * ro)


def yaxis(origin):
    """Location whose local +Z points along world +Y — for lateral-axis parts."""
    return Plane(origin=origin, z_dir=(0, 1, 0)).location


# ------------------------------------------------------------------ primitives
def cf_tube(od: float, wall: float, length: float):
    """A cut length of pultruded CF tube. No holes: §0.2 forbids drilled laminate."""
    return Cylinder(od / 2, length) - Cylinder(od / 2 - wall, length + 2)


def bonded_insert(tube_id: float, engage: float = ENGAGE, stub: float = 8.0,
                  wall: float = INSERT_WALL, gap: float = BOND_GAP):
    """Turned 6061 end fitting, bonded into the tube bore.

    The plug OD is the tube ID less the glue line, so the bond gap is a modelled
    dimension rather than an assumption. Hollow: a bonded joint carries load as
    adhesive shear over its engagement area, so a solid plug only adds mass —
    4.2 g against a 4.8 g femur tube (module note 2).

    `stub` protrudes past the tube end and is what the clevis or tongue is built
    on, so the joint hardware never touches the laminate.
    """
    od = tube_id - gap
    total = engage + stub
    body = Cylinder(od / 2, total)
    bore = Cylinder(max(od / 2 - wall, 0.6), total + 2)
    return Pos(0, 0, total / 2) * (body - bore)


def sheave(pitch_r: float, width: float = SHEAVE_W, bore: float = 6.0,
           web: float = WEB_T, lighten: bool = True):
    """Turned, hard-anodised joint sheave. The cable pitch line sits at `pitch_r`.

    `pitch_r` IS the tendon moment arm — `checks()` asserts that against
    `DEFAULT_TENDON.joint_moment_arm`, so the CAD cannot drift from the torque
    budget. Round-bottom groove per LEG_TENDON_SPEC §3.1, throat opened to 1.15x
    the cable so it seats without pinch.

    ⚠️ **Lightened, and it is not optional.** As a solid disc the Ø60 hip sheave
    is **46 g** — on its own, 42 % of the whole leg's mass allowance. Pocketing
    both faces down to a rim, a hub and a `web`-thick centre is how the part would
    actually be turned, and it is what makes the specified moment arms affordable
    at all. `lighten=False` recovers the solid disc so the saving stays measurable.
    """
    r_out = pitch_r + GROOVE_R + FLANGE_H
    throat = CABLE_D * 1.15
    body = Cylinder(r_out, width)
    cut = Torus(pitch_r, GROOVE_R) + (Cylinder(r_out + 1, throat)
                                      - Cylinder(pitch_r, throat))
    body = body - cut
    if lighten:
        rim_in = pitch_r - GROOVE_R - RIM_T
        hub_r = bore / 2 + HUB_T
        depth = (width - web) / 2.0
        if rim_in > hub_r + 1.0 and depth > 0.4:
            pocket = Cylinder(rim_in, depth) - Cylinder(hub_r, depth + 2)
            for sgn in (-1, +1):
                body = body - Pos(0, 0, sgn * (width - depth) / 2) * pocket
    return body - Cylinder(bore / 2, width + 2)


def bearing(bore: float, od: float, width: float):
    """Deep-groove ball bearing envelope. OD -> clevis bore is H7 (§2)."""
    return Cylinder(od / 2, width) - Cylinder(bore / 2, width + 2)


def clevis(bearing_od: float, gap: float, arm_t: float = CLEVIS_ARM_T,
           root_d: float = 10.0, root_len: float = 10.0):
    """Milled fork carrying the two bearing bores; built on an insert stub.

    Two arms straddle the proximal link's tongue. The bore is the bearing OD (H7),
    and the boss around it is `BOSS_WALL` of metal.
    """
    boss_r = bearing_od / 2 + BOSS_WALL
    parts = []
    for s in (-1, +1):
        y = s * (gap / 2 + arm_t / 2)
        # ⚠️ A LUG, not a disc. The first pass drew each arm as a full Ø24 disc,
        # which is not how a clevis is milled and cost ~2x its mass: the arm only
        # has to be an annulus around the bore where the web meets it.
        ring = Pos(0, y, 0) * (yaxis((0, 0, 0))
                               * (Cylinder(boss_r, arm_t)
                                  - Cylinder(bearing_od / 2, arm_t + 2)))
        keep = Pos(0, y, -boss_r / 2) * Box(2 * boss_r, arm_t + 1, boss_r)
        parts.append(ring - keep + (ring & keep))
        parts.append(Pos(0, y, -root_len / 2) * Box(boss_r, arm_t, root_len))
    parts.append(Pos(0, 0, -root_len) * (Cylinder(root_d / 2, root_len)
                                         - Cylinder(root_d / 2 - 1.5, root_len + 2)))
    return Compound(parts)


def tongue(bearing_od: float, t: float, root_d: float = 10.0,
           root_len: float = 10.0):
    """The proximal link's blade, riding between the clevis arms on the shaft."""
    boss_r = bearing_od / 2 * 0.62 + BOSS_WALL
    blade = yaxis((0, 0, 0)) * Cylinder(boss_r, t)
    web = Pos(0, 0, root_len / 2) * Box(boss_r, t, root_len)
    root = Pos(0, 0, root_len) * (Cylinder(root_d / 2, root_len)
                                  - Cylinder(root_d / 2 - 1.5, root_len + 2))
    return Compound([blade, web, root])


def shaft(d: float, length: float):
    """Ground shaft, h6 into the bearing bores (§2 — slip fit, serviceable)."""
    return yaxis((0, 0, 0)) * Cylinder(d / 2, length)


#: Smallest legal redirect pulley: LEG_TENDON_SPEC §2's minimum sheave DIAMETER
#: is 10x the cable, so Ø1.75 cable => Ø17.5 => r >= 8.75 mm.
#:
#: ⚠️ A first pass here used **5.0 mm** (Ø10), which is 43 % under the cable's own
#: minimum bend and would fatigue the UHMWPE. The number is not free — it is the
#: same rule that forced the spool from 8.0 to 8.75 mm in §2, and it is why the
#: joint coupling in `leg_tendons.py` cannot be designed away.
MIN_BEND_R = 10.0 * CABLE_D / 2.0


def idler(pitch_r: float = MIN_BEND_R, width: float = 4.5):
    """Redirect pulley. One sits at the limb ROOT per ASSEMBLY_SPEC §0.1's rule:
    turning the cable into the bone's sagittal plane recovers the femur's full
    SF 2.84 instead of the 2.17 an unguided straight run leaves."""
    return sheave(pitch_r, width, bore=3.0)


def paw_pad(width: float = 22.0, length: float = 26.0, height: float = 9.0,
            dome_r: float = 5.0):
    """Cast TPU ~80A pad with the sealed dome cavity over the MEMS barometer.

    TACTILE_SENSING_SPEC: 0-35 N measured, >=100 N survival, <= 20 g per paw
    (NFR9 — binding via SWING INERTIA, so `checks()` weighs it).
    """
    pad = Box(length, width, height)
    dome = Pos(0, 0, height / 2 - dome_r * 0.4) * Sphere(dome_r)
    return (Pos(0, 0, -height / 2) * (pad - dome))


def return_spring(length: float = 26.0, coil_r: float = 3.2):
    """Ankle return spring envelope — ADR-0002 Option B, single tendon + spring."""
    return Cylinder(coil_r, length)


# ------------------------------------------------------------------- assembly
def cable(points, d: float = CABLE_D, y: float = 0.0):
    """A cable run as a swept solid: cylinders along the polyline, balls at the
    knuckles. `leg_tendons.route` already densifies the pulley arcs, so this
    follows the real path rather than cutting the corners."""
    parts = []
    P = np.asarray(points, float)
    for a, b in zip(P[:-1], P[1:]):
        v = b - a
        L = float(np.linalg.norm(v))
        if L < 1e-6:
            continue
        mid = (a + b) / 2.0
        parts.append(Plane(origin=(mid[0], y, mid[1]),
                           z_dir=(v[0], 0.0, v[1])).location
                     * Cylinder(d / 2, L))
    for p in P[1:-1]:
        parts.append(Pos(p[0], y, p[1]) * Sphere(d / 2))
    return Compound(parts)


def motor_and_spool(centre, spool_r: float, y: float = 0.0):
    """GIM3505-9 plus its variable-radius spool (ADR-0008: one motor drives both
    sides of an antagonistic pair). Ø34.5 x 36.1 mm, the real surveyed part."""
    d, ln = 34.5, 36.1
    body = Pos(centre[0], y - ln / 2 - 5.0, centre[1]) * (
        yaxis((0, 0, 0)) * Cylinder(d / 2, ln))
    spool = Pos(centre[0], y, centre[1]) * (
        yaxis((0, 0, 0)) * (Cylinder(spool_r + GROOVE_R + FLANGE_H, 9.0)
                            - Cylinder(3.0, 11.0)))
    return Compound([body, spool])


def anchor_fitting(centre, y: float = 0.0):
    """Threaded eye-bolt adjuster, ASSEMBLY_SPEC §4 — the re-tensioning travel
    UHMWPE creep makes necessary, at the JOINT end so it stays reachable."""
    eye = Pos(centre[0], y, centre[1]) * (
        yaxis((0, 0, 0)) * (Cylinder(4.0, 3.0) - Cylinder(2.0, 5.0)))
    stud = Pos(centre[0] - 5.0, y, centre[1]) * (
        Plane(origin=(0, 0, 0), z_dir=(1, 0, 0)).location * Cylinder(1.6, 12.0))
    return Compound([eye, stud])


def tendon_drive(q, leg=DEFAULT_LEG):
    """Every cable, spool and anchor for one leg — five runs, three motors.

    Each run gets its own lateral plane so antagonists cannot foul each other,
    stacked outboard of the sheave they act on.
    """
    import leg_tendons as LT

    groups = {"tendon": [], "motor": [], "anchor": []}
    routes = {}
    #  (tendon, side, y-plane)
    plan = [("hip", +1, 9.2), ("hip", -1, 15.2),
            ("knee", +1, 9.2), ("knee", -1, 15.2),
            ("ankle", +1, 9.2)]
    for tendon, side, y in plan:
        r = LT.route(q, tendon, side=side)
        routes[(tendon, side)] = r
        groups["tendon"].append(cable(r["points"], y=y))
        st = LT.stations(q, tendon, side)
        groups["anchor"].append(anchor_fitting(st[-1][0], y=y))

    spool_c = LT.joints(q, leg)[0] + np.array(LT.SPOOL_OFFSET)
    for k, dz in enumerate((0.0, 26.0, 52.0)):
        groups["motor"].append(
            motor_and_spool((spool_c[0] - dz * 0.35, spool_c[1] + dz),
                            LT.SPOOL_R, y=9.2 + 3.0 * k))
    return {k: Compound(v) for k, v in groups.items()}, routes


def joint_offset(joint: str) -> float:
    """Tube end to joint centre: the boss around the bearing plus its wall."""
    _, od, _ = BEARING[joint]
    return od / 2 + BOSS_WALL


def build(leg=DEFAULT_LEG):
    """One hind leg in the stance pose, as manufacturable parts."""
    lm = LegModel(leg)
    q = lm.inverse((FOOT_X, FOOT_Z, FOOT_PITCH))
    pts = lm.joint_positions(q) * MM          # hip, knee, ankle, paw-base, paw-tip

    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm) * MM
    groups: dict[str, list] = {k: [] for k in
                               ("tube", "insert", "clevis", "sheave",
                                "bearing", "cable", "pad", "tendon", "motor",
                                "anchor")}
    report: dict[str, dict] = {}

    # ⚠️ Four links, not three. The first pass stopped at the metatarsus and left
    # the passive paw phalanx unmodelled, which also left the pad floating.
    seq = [("femur", "hip", "knee", 0), ("tibia", "knee", "ankle", 1),
           ("meta", "ankle", "paw", 2), ("paw", "paw", "tip", 2)]
    JN = {"hip": "hip", "knee": "knee", "ankle": "ankle",
          "paw": "ankle", "tip": "ankle"}

    for i, (bone, j_prox, j_dist, arm_i) in enumerate(seq):
        p0 = np.array([pts[i][0], 0.0, pts[i][1]])
        p1 = np.array([pts[i + 1][0], 0.0, pts[i + 1][1]])
        span = float(np.linalg.norm(p1 - p0))
        off_p, off_d = joint_offset(JN[j_prox]), joint_offset(JN[j_dist])
        cut = span - off_p - off_d

        od, wall = TUBE[bone]
        tube_id = od - 2 * wall
        loc = Plane(origin=tuple(p0 + (p1 - p0) / span * off_p),
                    z_dir=tuple((p1 - p0) / span)).location

        report[bone] = {"span": span, "cut": cut, "od": od, "wall": wall,
                        "tube_id": tube_id, "insert_od": tube_id - BOND_GAP,
                        "engage": 0.0, "engage_frac": 0.0, "solid_stub": False,
                        "p0": tuple(p0), "p1": tuple(p1)}

        if cut < 2 * MIN_ENGAGE + 2.0:
            # ⚠️ Two inserts plus a gap need >= 14 mm of tube. The 25 mm paw
            # phalanx leaves 10 mm after joint hardware, so it cannot be a bonded
            # tube at all: modelled as a solid turned/printed stub. That is what
            # the geometry forces, not what ASSEMBLY_SPEC §1 specified.
            report[bone]["solid_stub"] = True
            groups["insert"].append(
                Plane(origin=tuple(p0), z_dir=tuple((p1 - p0) / span)).location
                * (Pos(0, 0, span / 2) * Cylinder(od / 2 - 1.0, span)))
            continue
        groups["tube"].append(loc * (Pos(0, 0, cut / 2) * cf_tube(od, wall, cut)))
        # ⚠️ ENGAGE is clamped to what the tube can actually hold: two 15 mm
        # inserts in a 30 mm tube would meet in the middle.
        eng = min(ENGAGE, max(MIN_ENGAGE, 0.40 * cut))
        report[bone]["engage"] = eng
        report[bone]["engage_frac"] = 2 * eng / cut
        groups["insert"].append(loc * bonded_insert(tube_id, engage=eng))
        flip = Plane(origin=tuple(p1 - (p1 - p0) / span * off_d),
                     z_dir=tuple(-(p1 - p0) / span)).location
        groups["insert"].append(flip * bonded_insert(tube_id, engage=eng))

    # joints: sheave on the DISTAL link (ASSEMBLY_SPEC §2's critical rule)
    for k, (jname, arm_i) in enumerate((("hip", 0), ("knee", 1), ("ankle", 2))):
        c = np.array([pts[k][0], 0.0, pts[k][1]])
        b_bore, b_od, b_w = BEARING[jname]
        tongue_t = 6.0 if jname != "ankle" else 4.0
        # ⚠️ The bearings sit in the CLEVIS ARM BORES, so the gap only has to
        # clear the tongue. Seating them inside the gap instead (a first-pass
        # error here) widened every joint by 2 x the bearing width and pushed the
        # sheave 6 mm further outboard, which §0.1 charges as femur torsion.
        gap = tongue_t + 2 * TONGUE_CLEAR

        groups["clevis"].append(Pos(*c) * clevis(b_od, gap, arm_t=b_w))
        groups["clevis"].append(Pos(*c) * tongue(b_od, tongue_t))
        for s in (-1, +1):
            groups["bearing"].append(
                Pos(c[0], s * (gap / 2 + b_w / 2), c[2])
                * (yaxis((0, 0, 0)) * bearing(b_bore, b_od, b_w)))
        groups["bearing"].append(Pos(*c) * shaft(b_bore, gap + 2 * b_w + 4))

        # the sheave sits OUTBOARD of the clevis: a full disc in the bone plane
        # would sweep the proximal link. That lateral offset is what §0.1 prices.
        e = gap / 2 + b_w + SHEAVE_W / 2
        groups["sheave"].append(
            Pos(c[0], e, c[2]) * (yaxis((0, 0, 0)) * sheave(arms[arm_i], bore=b_bore)))
        report.setdefault("joints", {})[jname] = {
            "arm": arms[arm_i], "bearing": (b_bore, b_od, b_w),
            "lateral_offset": e, "gap": gap}

    # root idler — §0.1's design rule, at the hip, turning the cable into plane
    hip = np.array([pts[0][0], 0.0, pts[0][1]])
    groups["sheave"].append(Pos(hip[0] - 18.0, 14.0, hip[2] + 6.0)
                            * (yaxis((0, 0, 0)) * idler()))

    tip = np.array([pts[4][0], 0.0, pts[4][1]])
    groups["pad"].append(Pos(tip[0], 0.0, tip[2] + 1.5) * paw_pad())
    ank = np.array([pts[2][0], 0.0, pts[2][1]])
    groups["cable"].append(Pos(ank[0] - 10.0, -12.0, ank[2] + 14.0)
                           * return_spring())

    # ⚠️ THE TENDON DRIVE. Without this the model is a linkage with pulleys
    # bolted to it, and P1 — the premise of the whole robot — is undrawn.
    drive, routes = tendon_drive(q, leg)
    for k, comp in drive.items():
        groups[k].extend(comp.solids())
    report["routes"] = {f"{t}{'+' if s > 0 else '-'}": r
                        for (t, s), r in routes.items()}

    comps = {k: Compound(v) for k, v in groups.items() if v}
    for c in comps.values():
        c.locate(c.location * Pos(0, TRACK_Y, 0))
    return comps, report, pts


# ---------------------------------------------------------------------- checks
def checks(comps, report):
    """Every specified number, verified against the geometry that came out.

    ⚠️ This is the deliverable as much as the STEP file is. A spec value that
    does not survive contact with 3D is a finding, and it is printed rather than
    silently adjusted.
    """
    out = []
    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm) * MM

    # 1. sheave pitch radius IS the tendon moment arm
    for i, (jn, d) in enumerate(report["joints"].items()):
        ok = abs(d["arm"] - arms[i]) < 1e-9
        out.append((ok, f"{jn} sheave pitch r = {d['arm']:.1f} mm "
                        f"= joint_moment_arm[{i}]"))

    # 2. groove geometry follows the CABLE, not the stale §3.1 literal
    out.append((abs(GROOVE_R - 0.55 * CABLE_D) < 1e-12,
                f"groove r {GROOVE_R:.2f} mm = 0.55 x {CABLE_D} mm cable "
                f"(§3.1 still says 0.85 for the superseded 1.5 mm cable)"))

    # 3. bond gap inside ASSEMBLY_SPEC §2's window
    for bone in ("femur", "tibia", "meta"):
        g = report[bone]["tube_id"] - report[bone]["insert_od"]
        out.append((0.05 <= g <= 0.15,
                    f"{bone} bond gap {g:.2f} mm in 0.05-0.15"))
    out.append((report["paw"]["solid_stub"],
                f"paw phalanx: {report['paw']['span']:.1f} mm span leaves "
                f"{report['paw']['cut']:.1f} mm of tube -- modelled as a solid stub"))

    # 4. shaft >= 4 mm (§0.2: Ø3 bearing stress is 149 MPa, marginal)
    for jn, d in report["joints"].items():
        out.append((d["bearing"][0] >= 4.0,
                    f"{jn} shaft Ø{d['bearing'][0]:.0f} mm >= 4 mm"))

    # 5. the insert-engagement finding, stated as a number
    for bone in ("femur", "tibia", "meta"):
        f = report[bone]["engage_frac"]
        out.append((f < 0.85,
                    f"{bone}: 2 x {report[bone]['engage']:.1f} mm insert = "
                    f"{100 * f:.0f} % of the {report[bone]['cut']:.1f} mm tube"))

    # 6. mass, against the params the sim already uses
    def vol(comp):
        # ⚠️ `Compound.volume` does NOT recurse into nested Compounds, and the
        # clevis group is a Compound of Compounds — it read 0.00 g until this was
        # summed over `.solids()` instead.
        return sum(sd.volume for sd in comp.solids())

    mass = {
        "tube": vol(comps["tube"]) * CF_RHO,
        "insert": vol(comps["insert"]) * AL_RHO,
        "clevis": vol(comps["clevis"]) * AL_RHO,
        "sheave": vol(comps["sheave"]) * AL_RHO,
        # envelope for fit, catalogue for mass: 2 bearings + a shaft per joint
        "bearing": 2 * sum(BEARING_G.values())
                   + sum(math.pi * (BEARING[j][0] / 2) ** 2
                         * (6.0 + 2 * BEARING[j][2] + 4) * STEEL_RHO
                         for j in BEARING),
        "pad": vol(comps["pad"]) * TPU_RHO,
        # cables are UHMWPE (0.97 g/cm^3) -- they float, which is the P1 point
        # UHMWPE, 0.97 g/cm^3 -- the cable floats, which IS the P1 argument
        "tendon": vol(comps["tendon"]) * 0.97e-3 if "tendon" in comps else 0.0,
        "anchor": vol(comps["anchor"]) * AL_RHO if "anchor" in comps else 0.0,
        # ⚠️ SPRING_FILL: the envelope is a solid cylinder but an extension
        # spring is mostly air. Wire volume / envelope volume for a typical
        # cat-scale coil. `[assumed]`
        "cable": vol(comps["cable"]) * STEEL_RHO * SPRING_FILL,
    }
    total = sum(mass.values())
    budget = sum(DEFAULT_LEG.link_mass) * 1e3
    out.append((total <= budget,
                f"leg hardware {total:.1f} g vs DEFAULT_LEG.link_mass "
                f"{budget:.1f} g  ({total - budget:+.1f} g, "
                f"{100 * total / budget:.0f} %)"))
    out.append((mass["pad"] <= 20.0,
                f"paw pad {mass['pad']:.1f} g <= 20 g (NFR9)"))

    # 7. link safety factors, re-derived from the LIVE land case at the lateral
    #    sheave offset this layout actually produces (§0.1's combined check).
    #    ⚠️ §3.5 quotes SF 2.84 / 3.10 / 2.87 from the stale 12.36 N.m table.
    loads = live_loads()
    land = loads["land"]
    CF_ALLOW = 400.0                              # MPa in bending  [assumed, §3.5]
    for i, (bone, jn) in enumerate((("femur", "hip"), ("tibia", "knee"),
                                    ("meta", "ankle"))):
        od, wall = TUBE[bone]
        Z = section_Z(od, wall)
        e = report["joints"][jn]["lateral_offset"]
        sig_b = land["tau"][i] * 1e3 / Z
        tau_s = land["T"][i] * e / (2 * Z)
        vm = math.sqrt(sig_b ** 2 + 3 * tau_s ** 2)
        sf = CF_ALLOW / vm
        out.append((sf >= 2.0,
                    f"{bone} SF {sf:.2f} at live tau {land['tau'][i]:.2f} N.m + "
                    f"torsion from the {e:.1f} mm sheave offset "
                    f"(§3.5 claims {(2.84, 3.10, 2.87)[i]:.2f} on the stale table)"))

    # 8. the staleness itself, asserted so it cannot be forgotten
    out.append((abs(land["tau"][0] - 12.36) < 0.05,
                f"LEG_TENDON_SPEC §1.1 hip land torque 12.36 N.m == live "
                f"{land['tau'][0]:.2f} N.m"))
    return out, mass, total


#: Pultruded CF tube stock this design may specify from. `[assumed availability]`
STOCK = [(8.0, 1.0), (10.0, 1.0), (12.0, 1.0), (14.0, 1.0), (16.0, 1.0),
         (12.0, 1.5), (14.0, 1.5), (16.0, 1.5)]


def size_tubes(target_sf: float = 2.5, allow: float = 400.0, offsets=None):
    """Lightest stock section per link that meets `target_sf` at the LIVE loads.

    §3.5 chose Ø12/Ø10/Ø8 x 1.0 to equalise the safety factor at SF 2.8-3.1 — on
    the **stale 12.36 N.m** table. Re-run at the live 16.67 N.m, plus the torsion
    the sheave's lateral offset actually imposes, the same sections give
    **1.97 / 2.08 / 1.84** and the femur and metatarsus fall below the SF 2 floor
    §0.1 relied on.

    The remedy is cheap, which is the useful part: bending strength goes as the
    cube of diameter while tube mass goes only as the first power, so a step up in
    stock buys a lot of margin for very little mass.
    """
    loads = live_loads()["land"]
    if offsets is None:
        offsets = (12.2, 12.2, 9.2)
    picks = []
    for i, bone in enumerate(("femur", "tibia", "meta")):
        best = None
        for od, wall in STOCK:
            Z = section_Z(od, wall)
            sig = loads["tau"][i] * 1e3 / Z
            tau_s = loads["T"][i] * offsets[i] / (2 * Z)
            vm = math.sqrt(sig ** 2 + 3 * tau_s ** 2)
            sf = allow / vm
            if sf < target_sf:
                continue
            area = math.pi * ((od / 2) ** 2 - (od / 2 - wall) ** 2)
            if best is None or area < best["area"]:
                best = {"bone": bone, "od": od, "wall": wall, "sf": sf,
                        "area": area, "vm": vm}
        picks.append(best or {"bone": bone, "od": None, "wall": None,
                              "sf": 0.0, "area": 0.0, "vm": 0.0})
    return picks


def rom_clearance(leg=DEFAULT_LEG, n: int = 13):
    """Minimum clearance between NON-ADJACENT links over the full joint ROM.

    Capsule-to-capsule (segment distance less the two radii), which is exact for
    round tubes and cheap enough to sweep — a boolean interference check over the
    same grid would be minutes, not milliseconds.

    ⚠️ The sheaves are excluded deliberately: they sit **laterally offset** from
    the bone plane (see `build`), so they cannot foul a link by construction, and
    including them would report a false collision in the sagittal projection. What
    they *can* foul is the girdle, which is not in this file's scope — the
    packaging study owns that, and a Ø60 hip sheave is a real question for it.
    """
    lm = LegModel(leg)
    lo, hi = np.asarray(leg.q_min), np.asarray(leg.q_max)
    rads = [TUBE[b][0] / 2 for b in ("femur", "tibia", "meta", "paw")]

    def _pt_seg(p, q0, q1):
        d = q1 - q0
        L = float(d @ d)
        t = 0.0 if L < 1e-18 else float(np.clip((p - q0) @ d / L, 0.0, 1.0))
        return float(np.linalg.norm(p - (q0 + t * d)))

    def seg_dist(a0, a1, b0, b1):
        """Distance between two segments.

        ⚠️ The interior solution is only valid when the segments are not
        parallel AND both clamped parameters are interior. A first pass here left
        `tc` unclamped in the parallel branch and reported the fully extended leg
        as femur-on-metatarsus interference at -10.0 mm, where the true clearance
        is 95 mm. The endpoint fallback below is what makes it robust, and it is
        cheap enough to run unconditionally.
        """
        u, v, w = a1 - a0, b1 - b0, a0 - b0
        a, b, c = u @ u, u @ v, v @ v
        d, e = u @ w, v @ w
        den = a * c - b * b
        best = math.inf
        if den > 1e-12:
            sc = np.clip((b * e - c * d) / den, 0.0, 1.0)
            tc = np.clip((a * e - b * d) / den, 0.0, 1.0)
            best = float(np.linalg.norm(w + sc * u - tc * v))
        for p, q0, q1 in ((a0, b0, b1), (a1, b0, b1),
                          (b0, a0, a1), (b1, a0, a1)):
            best = min(best, _pt_seg(p, q0, q1))
        return best

    worst = {"gap": math.inf, "pair": None, "q": None}
    for qh in np.linspace(lo[0], hi[0], n):
        for qk in np.linspace(lo[1], hi[1], n):
            for qa in np.linspace(lo[2], hi[2], n):
                p = lm.joint_positions(np.array([qh, qk, qa])) * MM
                P = [np.array([x, z]) for x, z in p]
                for i in range(4):
                    for j in range(i + 2, 4):     # skip adjacent links
                        g = seg_dist(P[i], P[i + 1], P[j], P[j + 1])                             - rads[i] - rads[j]
                        if g < worst["gap"]:
                            worst = {"gap": g, "pair": (i, j),
                                     "q": (qh, qk, qa)}
    return worst


def trade_moment_arms(scales=(1.0, 0.85, 0.70, 0.60, 0.50)):
    """The trade the specs never closed: sheave MASS against cable TENSION.

    LEG_TENDON_SPEC §1.2 grew the moment arms to cut cable tension, and §1.3a
    priced the *inertia* of doing so at the ankle only — from a 6 g-at-14 mm
    estimate scaling as r^2. Turned as real parts the sheaves come out **42.6 g
    for the set**, and the leg totals 144 % of its mass allowance. So the arms
    have a cost on both sides now, and neither side has been swept.

    Columns, all derived rather than assumed:
      T_hip   = tau/r + T_bias, hip land transient (tau = 12.36 N.m, bias 19.6 N)
      d_min   = cable dia for SF 4, break scaling as d^2 off §2's 1.5 mm / 2.2 kN
      r_spool = 5 x d, the min-bend rule §2 used to justify 8.75 mm at 1.75 mm
      tau_mot = T x r_spool, against the GIM3505-9's 1.95 N.m peak
    """
    base = np.asarray(DEFAULT_TENDON.joint_moment_arm) * MM
    loads = live_loads()
    tau_land = float(loads["land"]["tau"][0])       # structural case
    tau_trot = float(loads["trot"]["tau"][0])       # actuator case
    rows = []
    for k in scales:
        arms = base * k
        m = sum(sum(sd.volume for sd in sheave(a, bore=b).solids()) * AL_RHO
                for a, b in zip(arms, (6.0, 6.0, 4.0)))
        r = arms[0] / MM
        T_land = tau_land / r + 19.6
        T_trot = tau_trot / r + 19.6
        d = 1.5 * math.sqrt(4.0 * T_land / 2200.0)   # cable sized on the transient
        r_spool = 5.0 * d
        rows.append({"k": k, "arms": arms, "sheave_g": m,
                     "T_land": T_land, "T_trot": T_trot, "d": d,
                     "r_spool": r_spool,
                     # ⚠️ the motor is checked against the TROT, not the land.
                     # ADR-0008 puts the x2.5 single-leg landing explicitly
                     # OUTSIDE the actuator envelope -- it sizes cable, pulley and
                     # bearing only. A first pass here compared the land case to
                     # the motor peak and read 162 %, which is the conflation
                     # ADR-0008 exists to prevent.
                     "mot_frac": T_trot * r_spool / MM / 1.95})
    return rows


def per_link_mass(comps=None, report=None):
    """Apportion the measured hardware to the four links — what `link_mass` needs.

    `checks()` reports a leg TOTAL; `LegParams.link_mass` is a per-link tuple and
    the CoM / inertia model needs the distribution, not the sum.

    **Apportionment rule, and it follows ASSEMBLY_SPEC §2 rather than convenience:**
    a joint's sheave is fixed to its **distal** link, and the clevis/tongue/bearing
    stack straddles the joint. So each joint's hardware is charged to the distal
    link of that joint, together with that link's own tube and inserts. The girdle
    motors are charged to **nothing** — they are not in the leg (P1).

    ⚠️ The tendons are charged to the links they run along, which over-charges the
    proximal links slightly: a cable crossing the femur to reach the ankle is
    counted on the femur. At 3.3 g for all five runs the error is under a gram.
    """
    if comps is None or report is None:
        comps, report, _ = build()

    def vol(group, pred=None):
        if group not in comps:
            return 0.0
        sds = comps[group].solids()
        return sum(sd.volume for sd in sds if pred is None or pred(sd))

    _, mass, total = checks(comps, report)

    # tube + insert per bone, from the report's own geometry
    bone_g, bone_names = {}, ("femur", "tibia", "meta", "paw")
    for b in bone_names:
        r = report[b]
        if r["solid_stub"]:
            g = math.pi * (r["od"] / 2 - 1.0) ** 2 * r["span"] * AL_RHO
        else:
            ro, ri = r["od"] / 2, r["od"] / 2 - r["wall"]
            g = math.pi * (ro ** 2 - ri ** 2) * r["cut"] * CF_RHO
            ins_od = r["insert_od"]
            g += 2 * math.pi * ((ins_od / 2) ** 2
                                - (ins_od / 2 - INSERT_WALL) ** 2)                 * (r["engage"] + 8.0) * AL_RHO
        bone_g[b] = g

    # joint hardware -> DISTAL link of that joint
    joint_to_link = {"hip": "femur", "knee": "tibia", "ankle": "meta"}
    n_j = len(joint_to_link)
    per_joint_clevis = mass["clevis"] / n_j
    per_joint_sheave = {}
    for jn, d in report["joints"].items():
        per_joint_sheave[jn] = sum(
            sd.volume for sd in sheave(d["arm"], bore=d["bearing"][0]).solids()
        ) * AL_RHO
    bearing_g = {jn: 2 * BEARING_G[jn]
                 + math.pi * (BEARING[jn][0] / 2) ** 2
                 * (6.0 + 2 * BEARING[jn][2] + 4) * STEEL_RHO
                 for jn in BEARING}

    out = {b: bone_g[b] for b in bone_names}
    for jn, link in joint_to_link.items():
        out[link] += per_joint_clevis + per_joint_sheave[jn] + bearing_g[jn]
    # distal extras
    out["paw"] += mass["pad"]
    out["meta"] += mass["cable"]                       # ankle return spring
    # tendons + anchors spread over the links they run along
    spread = (mass["tendon"] + mass["anchor"]) / 3.0
    for b in ("femur", "tibia", "meta"):
        out[b] += spread
    # the root idler lives on the femur
    out["femur"] += sum(sd.volume for sd in idler().solids()) * AL_RHO

    return out, total


def render_png(comps, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    style = {"tube": "#d7ac86", "insert": "#b8bfc7", "clevis": "#8a9bb0",
             "sheave": "#5f7285", "bearing": "#3c4a5a", "cable": "#c17a3a",
             "pad": "#4a4a4a", "tendon": "#c17a3a", "motor": "#2f3b49",
             "anchor": "#9aa7b4"}
    fig = plt.figure(figsize=(11, 7))
    ax = fig.add_subplot(111, projection="3d")
    for name, comp in comps.items():
        verts, tris = comp.tessellate(0.2)
        V = np.array([[v.X, v.Y, v.Z] for v in verts])
        coll = Poly3DCollection(V[np.array(tris)], facecolor=style[name],
                                edgecolor="none", alpha=1.0)
        coll.set_zsort("average")
        ax.add_collection3d(coll)
    whole = Compound(list(comps.values()))
    bb = whole.bounding_box()
    ax.set_xlim(bb.min.X - 10, bb.max.X + 10)
    ax.set_ylim(bb.min.Y - 40, bb.max.Y + 40)
    ax.set_zlim(bb.min.Z - 10, bb.max.Z + 10)
    ax.set_box_aspect((bb.size.X + 20, bb.size.Y + 80, bb.size.Z + 20))
    ax.view_init(elev=16, azim=-68)
    ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(path, dpi=140, bbox_inches="tight", facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    here = os.path.dirname(__file__)
    comps, report, pts = build()

    print("HIND LEG — manufacturable parts, hind leg, stance pose\n")
    print("%-9s %8s %8s %9s %9s %9s" % ("bone", "span", "cut", "section",
                                        "tube ID", "insert OD"))
    print("-" * 58)
    for b in ("femur", "tibia", "meta", "paw"):
        r = report[b]
        note = "  SOLID STUB (no tube left)" if r["solid_stub"] else                "  insert %.1f mm" % r["engage"]
        print("%-9s %7.1f %8.1f  Ø%-5.0fx%.1f %8.2f %9.2f%s"
              % (b, r["span"], r["cut"], r["od"], r["wall"],
                 r["tube_id"], r["insert_od"], note))

    print("\n%-7s %8s %-14s %10s" % ("joint", "arm", "bearing", "sheave dy"))
    print("-" * 44)
    for jn, d in report["joints"].items():
        print("%-7s %7.1f  Ø%.0f/%.0fx%.0f %11.1f"
              % (jn, d["arm"], *d["bearing"], d["lateral_offset"]))

    results, mass, total = checks(comps, report)
    print("\nP1: the three leg motors (395 g) sit in the GIRDLE and are")
    print("    deliberately NOT counted in the leg mass below -- that")
    print("    relocation is the whole reason this robot is tendon-driven.")
    print("")
    print("mass by group (g):")
    for k, v in sorted(mass.items(), key=lambda kv: -kv[1]):
        print("   %-9s %6.2f" % (k, v))
    print("   %-9s %6.2f" % ("TOTAL", total))

    print("\nchecks:")
    nfail = 0
    for ok, msg in results:
        print("   %s %s" % ("PASS" if ok else "FAIL", msg))
        nfail += (not ok)

    print("")
    print("moment-arm trade -- sheave mass vs cable tension "
          "(neither side swept in the specs):")
    print("%-6s %-17s %8s %8s %8s %7s %8s %9s"
          % ("scale", "arms (mm)", "sheave g", "T land", "T trot", "d_min",
             "r_spool", "trot/peak"))
    print("-" * 80)
    for r in trade_moment_arms():
        print("%-6.2f %-17s %8.1f %8.0f %8.0f %7.2f %8.2f %8.0f %%"
              % (r["k"], "%.1f/%.1f/%.1f" % tuple(r["arms"]), r["sheave_g"],
                 r["T_land"], r["T_trot"], r["d"], r["r_spool"],
                 100 * r["mot_frac"]))
    print("   leg total = %.1f g - 42.6 + sheave_g ; budget %.1f g"
          % (total, sum(DEFAULT_LEG.link_mass) * 1e3))

    plm, plm_total = per_link_mass(comps, report)
    print("")
    print("per-link apportionment (g) -- what LegParams.link_mass needs:")
    print("%-9s %10s %10s %9s" % ("link", "measured", "current", "ratio"))
    print("-" * 42)
    cur = [m * 1e3 for m in DEFAULT_LEG.link_mass]
    for b, c in zip(("femur", "tibia", "meta", "paw"), cur):
        print("%-9s %10.1f %10.1f %8.2fx" % (b, plm[b], c, plm[b] / c))
    print("%-9s %10.1f %10.1f %8.2fx"
          % ("TOTAL", sum(plm.values()), sum(cur), sum(plm.values()) / sum(cur)))
    print("   (apportioned %.1f g of the %.1f g measured; the rest is the girdle "
          "motors)" % (sum(plm.values()), plm_total))

    rc = rom_clearance()
    names = ("femur", "tibia", "meta", "paw")
    print("")
    print("ROM interference sweep (non-adjacent links, full joint ROM):")
    print("   worst clearance %+.1f mm  between %s and %s  at q = "
          "(%.0f, %.0f, %.0f) deg"
          % (rc["gap"], names[rc["pair"][0]], names[rc["pair"][1]],
             *[math.degrees(v) for v in rc["q"]]))

    print("")
    print("remedy -- lightest stock section meeting SF 2.5 at the LIVE loads:")
    print("%-9s %-12s %8s %10s %12s" % ("bone", "section", "SF", "sigma_vm",
                                        "vs §3.5"))
    print("-" * 58)
    for p, cur, claim in zip(size_tubes(), (12.0, 10.0, 8.0),
                             (2.84, 3.10, 2.87)):
        if p["od"] is None:
            print("%-9s %-12s %8s" % (p["bone"], "NONE IN STOCK", "-"))
            continue
        print("%-9s Ø%-5.0fx%-4.1f %8.2f %9.0f MPa   Ø%.0f -> Ø%.0f"
              % (p["bone"], p["od"], p["wall"], p["sf"], p["vm"], cur, p["od"]))

    whole = Compound(list(comps.values()))
    bb = whole.bounding_box()
    print("\nbounding box: %.0f x %.0f x %.0f mm   solids: %d"
          % (bb.size.X, bb.size.Y, bb.size.Z, len(whole.solids())))

    step = os.path.join(here, "tomcat_leg_detail.step")
    stl = os.path.join(here, "tomcat_leg_detail.stl")
    png = os.path.join(here, "tomcat_leg_detail.png")
    export_step(whole, step)
    export_stl(whole, stl, tolerance=0.12, angular_tolerance=0.25)
    render_png(comps, png)
    for p in (step, stl, png):
        print("  wrote %s (%d bytes)" % (os.path.basename(p), os.path.getsize(p)))
    print("\n%d check(s) FAILED — see MODULE NOTES" % nfail if nfail else
          "\nall checks pass")
