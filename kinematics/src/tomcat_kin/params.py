# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Placeholder parameters for the TomCat single-leg model.

IMPORTANT: every numeric value here is a PLACEHOLDER (❓ TBD in
docs/REQUIREMENTS.md). They exist so the model runs end-to-end; they are not
committed design values. Swap them once mechanical design lands.

Conventions
-----------
- SI units throughout: metres, radians, kilograms, newtons, newton-metres.
- Sagittal-plane (2D) model: x forward, z up, hip at the origin.
- A DIGITIGRADE 4-link chain (cats stand on their toes):
      hip -> femur -> stifle/knee -> tibia -> hock/ankle -> metatarsus -> paw.
  The first THREE joints (hip, stifle, hock) are ACTUATED; the paw is a PASSIVE
  distal link held at a fixed rest angle relative to the metatarsus. There is NO
  fourth motor: the actuated joint vector stays q = (q1, q2, q3) (see LegParams).

=====================================================================
MASS BUDGET (M4 — "real mass"): how the 3.0 kg was apportioned  ❓ ALL PLACEHOLDER
=====================================================================
Before M4 every link was MASSLESS and ``whole_body_budget`` lumped EQUAL point
masses at the vertebrae (its own assumption A2 conceded real cats are ~60%
front-heavy). The numbers below replace that with a distributed, front-heavy
budget. They are apportioned, NOT measured — every one is ❓ TBD.

Apportionment rule — REBUILT BOTTOM-UP (review findings F1 + F2, see
mechanical/DESIGN_REVIEW.md). The first version of this budget copied feline
biology (limbs ~24% of body mass) and then TUNED the girdle masses to hit a
60/40 front-heavy split. Both were wrong for this machine:

  * F1 — a biological limb's mass is mostly MUSCLE, and P1/ADR-0003 deliberately
    relocate the muscle (motors) into the girdles. Charging the limbs a
    biological fraction DOUBLE-COUNTS the actuator. A bottom-up count of the
    specced hardware (Al sheaves + CF-tube bones + miniature bearings + cable)
    gives ~80 g/leg bare, ~110 g with margin -- not 200 g.
  * F2 — the 60/40 split was an INPUT tuned into the girdle masses. It is
    properly an OUTPUT of where the hardware actually sits, and the motors do
    NOT sit forward: per ADR-0005 the pelvis carries 19 of them and the shoulder
    only 12.

1. TOTAL = **4.05 kg** -- raised from 3.00 kg once a real motor was sourced
   (see step 3 and docs/notes/motor-reality-check.md). A domestic cat is 4-5 kg,
   so this is if anything MORE biomimetic than the original target; but it was
   forced by hardware, not chosen.
2. LEGS, bottom-up: **0.110 kg** hind / **0.095 kg** fore -> 0.410 kg for all
   four = **13.7%** of body (was 24%). Still proximal-heavy within the leg
   (47.5 / 30 / 15 / 7.5 %) because tendon drive centralises mass.
3. GIRDLES carry their real contents -- motors + one driver board each:
       front  = 6 leg motors x 132 g + head/neck 0.240 + structure 0.090 = **1.122 kg**
       rear   = 7 motors x 132 g + structure 0.110 + tail 0.0098         = **1.0438 kg**
   ⚠️ The rear girdle's SEVENTH motor is the tail's (M102); the head in the
   front girdle is no longer lumped at the mount but placed where it is drawn.
   where 132 g = the SURVEYED REAL PART (SteadyWin GIM3505-9: 120 g motor +
   integrated driver = 131.7 g), NOT the 72 g class target it replaced.
4. SPINE segments = 1.469 kg: 0.130 / 1.2122 / 0.127 kg rear->front.
   The MIDDLE segment dominates because it carries both the ~0.300 kg battery
   AND the SIX spine motors (3 dorsoventral + 3 lateral), which the CAD
   packaging puts in the mid-body bay between the girdles.

   ⚠️ **This said "the 7-motor spine+tail bank" for nine milestones and the
   seventh was the TAIL's.** ADR-0007 bought it; M88 placed it on trunk body 0
   beside the hind leg bank; ADR-0096 found that the trunk's own check counted
   `%d of 18` and could not see it. M102 moved it to the rear girdle, where
   body 0 maps, with the 9.8 g tail it drives -- 141.8 g out of this segment
   and onto that girdle, so the body total does not move.
5. The fore/hind split is a RESULT, not a target. See ``mass.quarter_masses``.

   ⚠️ CORRECTED (ADR-0009 follow-up). The previous apportionment was wrong twice,
   in opposite directions, and the errors did not cancel:
     * it charged **31 CHANNELS x 36 g**, a count that predates ADR-0008's
       variable-radius pulley (which halves motors to one per DOF) -- ~432 g too
       much; and
     * it used a **~31 g motor** from an assumed O16x28 mm envelope, while the
       motor down-select then landed on a **~72 g** O36 class -- ~779 g too little.
   Net: the model was **~347 g LIGHT on actuation**, ~12% of the whole budget.
   It also parked the spine/tail motors in the REAR GIRDLE while the CAD packs
   them mid-body, which mislocated ~0.5 kg by ~100 mm.
   Correcting both moved the body CoM forward (+100 -> +108 mm), IMPROVED the
   fore-aft margin (+32.7 -> +40.2 mm) and slightly raised lateral sway
   authority, so the M5 stability conclusions survive -- see docs/notes/
   mass-budget-recheck.md.

Consequences, cumulative over BOTH passes (all verified in the model). The F1/F2
rebuild moved the body CoM rearward from +130 mm to +102 mm and cut the fore-aft
margin from +46.5 mm to +27.4 mm; the ADR-0009 correction above then moved it
forward again to **+107.5 mm**, recovering the margin to **+40.2 mm**. Quiet-stand
spine load remains far below the pre-F2 model (the base joint no longer
cantilevers a front-heavy body) though it roughly doubled from the F1/F2 figure,
0.13 -> 0.29 N.m; an asymmetric single-leg LANDING still makes the base joint the
worst spine joint.

Estimate quality: the motor mass is now the DOWN-SELECTED ~72 g (O36 pancake
class, docs/notes/motor-downselect.md), no longer a guess. Still ❓ ASSUMED: the
0.300 kg battery, the 5 g driver board, and the per-girdle structure allowances
(0.090 / 0.110 kg) -- the last of these is the loosest number here.

Why NOT the literature's 0.454 kg knee mass
-------------------------------------------
docs/LITERATURE_REVIEW.md (Q2b) records a Mass-Mass-Spring leg model with
**~0.454 kg at the knee** that produces realistic trunk bending where a massless
SLIP model gives a null bending moment. That number is from a MUCH LARGER robot
and must NOT be copied literally: 0.454 kg is ~15% of our entire 3 kg body at a
single joint. The scaled analogue here is the whole 0.110 kg hind leg (3.7% of
body mass) with 0.052 kg at the femur. What we DO take from that result is the
qualitative lesson — leg mass is not negligible and materially bends a compliant
trunk — which is why M4 distributes mass over the links instead of lumping it.

Scope limit: these masses feed a QUASI-STATIC model (gravity + CoM + support
polygon). No inertias/velocities/accelerations are modelled; rotational inertia
tensors and full Newton-Euler are deferred to the dynamics milestone.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

GRAVITY = 9.81  # m/s^2


@dataclass(frozen=True)
class LegParams:
    """Geometry of one DIGITIGRADE leg (sagittal-plane 4-link chain).

    Four links -- femur, tibia, metatarsus, paw -- but only THREE ACTUATED
    joints (hip, stifle/knee, hock/ankle). The paw is a PASSIVE distal link held
    rigidly at `paw_angle` relative to the metatarsus, modelling the largely
    passive toes / ground contact of a digitigrade stance (long near-vertical
    metatarsus, hock held high). No fourth motor exists: the actuated joint
    vector stays q = (q1, q2, q3).
    """

    # Link lengths (m): femur, tibia, metatarsus, paw.  Digitigrade cat-scale
    # placeholders; total reach ~0.28 m keeps the previous ground clearance so
    # standing/gait still reach the floor.  ❓ TBD
    l1: float = 0.090   # femur
    l2: float = 0.095   # tibia
    l3: float = 0.070   # metatarsus (long, near-vertical in stance)
    l4: float = 0.025   # paw / toes (PASSIVE distal link)

    # Fixed rest angle of the PASSIVE paw relative to the metatarsus (rad). The
    # paw direction is the metatarsus cumulative angle (q1+q2+q3) PLUS paw_angle,
    # so the paw pitch never has its own actuator. Positive = the paw rotates CCW
    # (toes lift toward horizontal) away from a downward-pointing metatarsus, i.e.
    # the digitigrade "standing on the toes" pose. Set to ~55 deg, an anatomical
    # toe-break for a proper digitigrade crouch (up from an earlier modest 30).  ❓ TBD
    paw_angle: float = math.radians(55.0)

    # Joint angle limits (rad), (min, max) per ACTUATED joint: hip, stifle, hock.
    #
    # NEGATIVE-KNEE (anatomical fold) convention — see `KneeConfig` in leg.py.
    # M4's stability check exposed that the earlier POSITIVE-knee limits
    # (`stifle >= 0`) made it geometrically impossible to plant a paw under its
    # own hip: doing so demanded a hip angle of ~+167 deg, so every foot landed
    # ~0.2 m ahead of its hip and the whole machine was fore/aft unstable. On the
    # negative branch the same pose is ordinary (hip ~-71 deg), so the stifle is
    # now restricted to a NEGATIVE range, which encodes the fold direction
    # structurally: femur angles down-and-forward, tibia folds back under it,
    # metatarsus rises to a high hock — the digitigrade Z.
    #
    # Ranges are generous placeholders around the demanded working set (at the
    # default stance both fore and hind need only hip -76..-26, stifle -113..-69,
    # hock +73..+124 deg), leaving margin for swing and deeper crouches.  ❓ TBD
    q_min: tuple[float, float, float] = (
        math.radians(-120.0),   # hip
        math.radians(-150.0),   # stifle (knee) — negative fold only
        math.radians(-30.0),    # hock (ankle)
    )
    q_max: tuple[float, float, float] = (
        math.radians(120.0),    # hip
        0.0,                    # stifle — never crosses into positive fold
        math.radians(150.0),    # hock
    )

    # --- MASS PROPERTIES.  M4 apportioned these top-down; M41 replaced them
    #     with the manufacturing model's bottom-up figures (ADR-0043/0046).
    #
    # Per-link mass (kg), same order as the link lengths: femur, tibia,
    # metatarsus, paw.  These defaults are the HIND leg; ``DEFAULT_FORELEG``
    # overrides them.
    #
    # `[derived: mechanical/cad/tomcat_leg_detail.py per_link_mass()]` — NOT
    # measured hardware.  Bonded inserts, clevises, turned sheaves, bearings
    # (catalogue masses), the tendons and the tactile pad, apportioned to the
    # DISTAL link of each joint per ASSEMBLY_SPEC §2.  Girdle motors excluded:
    # they are not in the leg (P1).
    #
    # ⚠️ The old values were (0.052, 0.033, 0.017, 0.008) = 0.110 kg, justified as
    # "proximal-heavy (47.5 / 30 / 15 / 7.5 %) because both feline anatomy and the
    # ADR-0003 tendon drive push mass toward the body".  **That rationale was wrong
    # as written**: the tendon drive pushes the MOTORS toward the body, not the
    # PULLEYS.  Drawn as parts the split is 39.5 / 35.3 / 20.7 / 4.5 — the
    # metatarsus more than DOUBLES — and leg swing inertia about the hip rises
    # **+62 %** (ADR-0043).
    #
    # ⚠️ M86 moved the SPLIT again (the total is unchanged at 0.1672 kg).
    # `per_link_mass()` divides the clevis mass equally by three, but the ankle
    # clevis carries a Ø10 bearing against the hip's Ø19 and is physically the
    # smaller part: placed, it is 7.6 g where equal thirds charge it 13.5.
    # `[derived: cad/link_inertia.py]`, and re-derived by `test_link_inertia.py`.
    #
    # ⚠️ **M93 (ADR-0093) redesigned the leg and it grew 167.2 -> 186.7 g.** Via
    # pulleys that had never been drawn, shafts long enough to reach them, and
    # tube sections sized to SF 2.5 at the lateral offsets the routing really
    # produces (Ø14/Ø12/Ø12 against §3.5's Ø12/Ø10/Ø8, which measured
    # 1.83/1.94/1.75). The metatarsus grows most, 25.1 -> 32.5 g, because its
    # cable has to pass outboard of two vias.
    # ⚠️ M111: re-measured at the 36/34/22 sheaves (ADR-0103), +21 g per leg.
    # ⚠️ M122 (ADR-0112): -13 g per leg. The hip via and its bearing are gone
    # (the knee and ankle cables arrive by conduit), so are the ankle return
    # spring (the ankle is a PAIR) and the anchor pins (cables anchor on their
    # sheaves); the femur gains its two conduit ferrules.
    # ⚠️ M123 (ADR-0114): +26 g, all on the femur. The hip is HOLLOW: a hub on
    # two 40 x 50 thin-section bearings (22 g each `[assumed]`) replaces the
    # clevis, tongue, 6 mm shaft and two 626s; the conduit ferrules sit in the
    # hub's wall. Lightening the hub is `[owed]`.
    link_mass: tuple[float, float, float, float] = (0.10279, 0.07472, 0.03612,
                                                    0.00759)

    # Fraction of each link's LENGTH, measured from that link's PROXIMAL joint,
    # at which its centre of mass sits (dimensionless, 0 = proximal joint,
    # 1 = distal joint).
    #
    # ⚠️ Was (0.45, 0.45, 0.50, 0.50) and marked ❓ TBD -- "muscle bellies sit
    # proximally" as a hand-waved 45 %. **Measured, the three proximal links sit
    # at 6-8 %**: the mass is joint hardware sitting ON the proximal joint, not
    # a belly part-way down a bone. The paw is the opposite (87 %) because the
    # pad is at its tip.  `[derived: cad/link_inertia.py]`
    link_com_frac: tuple[float, float, float, float] = (0.0663, 0.0695, 0.1834, 0.8743)

    #: Centre of mass in the link's OWN body frame (m), +x along the link from
    #: its proximal joint. The full vector `link_com_frac` cannot carry: the
    #: sheaves stand ~4 mm off the limb plane in +y.  `[derived: cad/link_inertia.py]`
    link_com: tuple[tuple[float, float, float], ...] = (
        (0.005965, 0.005699, 0.000291),       # femur
        (0.006604, 0.007078, 0.000091),       # tibia
        (0.012836, 0.006910, -0.000181),      # meta
        (0.021858, 0.000000, -0.002353),      # paw
    )

    #: MJCF `fullinertia` about each link's CoM, in its body frame
    #: (ixx iyy izz ixy ixz iyz, kg m²).
    #:
    #: ⚠️ Until M86 `mjcf_tendon` derived these from a uniform-density capsule,
    #: which overstated leg swing inertia about the hip by **45 %**
    #: ([ADR-0087](../../../docs/DESIGN_DECISIONS.md)). Four fifths of a link is
    #: joint hardware sitting at its joints; a capsule spreads it down the bone.
    #: `[derived: cad/link_inertia.py]`
    link_inertia: tuple[tuple[float, ...], ...] = (
        (3.8041e-05, 9.3432e-05, 6.9108e-05, 1.6316e-06, -7.1882e-08, -9.4651e-08),
        (1.6209e-05, 4.5039e-05, 4.3403e-05, 2.7332e-06, 1.7834e-07, 1.4016e-07),
        (5.9258e-06, 2.1280e-05, 2.3210e-05, 3.1287e-06, 9.1186e-08, 3.3537e-08),
        (3.0802e-07, 7.2523e-07, 9.1886e-07, 0.0, 5.6145e-08, 0.0),
    )

    def __post_init__(self) -> None:
        for name in ("link_mass", "link_com_frac"):
            got = len(getattr(self, name))
            if got != 4:
                raise ValueError(
                    f"LegParams.{name} has {got} entries; expected 4 "
                    "(femur, tibia, metatarsus, paw)"
                )
        if any(m < 0.0 for m in self.link_mass):
            raise ValueError("LegParams.link_mass entries must be non-negative")

    @property
    def reach(self) -> float:
        """Maximum straight-leg distance from hip to paw tip (all four links)."""
        return self.l1 + self.l2 + self.l3 + self.l4

    @property
    def link_lengths(self) -> tuple[float, float, float, float]:
        """(l1, l2, l3, l4) as a tuple, proximal -> distal."""
        return (self.l1, self.l2, self.l3, self.l4)

    @property
    def mass(self) -> float:
        """Total mass of the four links of this leg (kg)."""
        return float(sum(self.link_mass))


@dataclass(frozen=True)
class TendonParams:
    """Tendon routing and actuator parameters, per joint.

    Defaults describe the 3-joint leg (hip, knee, ankle), but every per-joint
    tuple may be any length: the array length sets the number of joints, so the
    spine reuses this container (see `SpineParams` / `TendonMap.from_spine`).
    """

    # Joint pulley radii / moment arms (m) — converts tension to joint torque.
    # Sized in mechanical/LEG_TENDON_SPEC.md (hip/knee/ankle): largest packageable
    # pulley at each joint to cut cable tension (T = tau/r).  Roughly halves the
    # land-case peak vs. the original (0.015,0.012,0.010).
    #
    # ⚠️ **M111 (ADR-0103): 28/25/14 -> 36/34/22, and the FORE leg is why.**
    # The actuator budget had only ever been run on the hind leg -- `DEFAULT_LEG`
    # equals `DEFAULT_HINDLEG` -- and with capstan friction live the fore knee
    # needed 2.421 N.m against the 1.95 proxy, 24 % over, and 0.845 standing
    # against 0.71. That overrun is LOAD, not routing: re-routing and spool
    # re-assignment move it only 24 % -> 22 %. Motor torque goes as 1/r at
    # fixed joint torque, so the arm is the lever that reaches it. At 36/34/22
    # both legs sit inside peak and continuous with the mass spiral closed --
    # 6.3 % and 9.0 % margin -- for 23.7 g of sheave per leg.
    # ⚠️ The cost is distal mass: +9.2 g per knee, +5.8 g per ankle on moving
    # links, and the swing-inertia price is `[owed]`.
    joint_moment_arm: tuple[float, ...] = (0.036, 0.034, 0.022)

    # Motor spool radius (m) — converts motor torque to cable tension.
    #
    # ⚠️ M41: 0.008 -> 0.00875.  LEG_TENDON_SPEC §2 raised it when ADR-0010's mass
    # increase forced the cable from Ø1.5 to Ø1.75 mm, whose minimum bend DIAMETER
    # is 10x the cable = Ø17.5, i.e. r >= 8.75 mm.  The spec said so and `params`
    # was never updated, so every motor torque and motor angle the model published
    # was **9 % low** for ten milestones (ADR-0042/0046).
    #
    # It is a trade, not a loss: `tau_motor = T * r_spool` costs 9.4 % of peak
    # margin, `v_cable = omega * r_spool` buys the same 9.4 % of foot speed.
    motor_spool_radius: float = 0.00875

    #: **G3 -- the series-elastic element on every LEG cable** (N/m), physically a
    #: torsional spring between the motor rotor and the spool:
    #: `k_tors = series_k * motor_spool_radius^2` (ADR-0051).
    #:
    #: ⚠️ **M112 (ADR-0105): 1.5e5 -> 1.25e5, because ADR-0103 grew the arms.** The
    #: spring appears at a JOINT as `k r^2`, and ADR-0026 specifies the joint:
    #: 80-150 N.m/rad for balance compliance. At 36/34/22 mm the old 1.5e5 gave
    #: hip 171 / knee 154, above the window; 1.25e5 gives **145 / 132**, inside it.
    #: 1.0e5 (the window's centre) was tried and REJECTED: the cascade's hind-ankle
    #: tracking reached 6.1 deg off and the env's joint-angle reconstruction 3.4 deg,
    #: both from the larger spring deflection. The spine keeps its own rate
    #: (`SpineParams.series_k`), because its arm did not move.
    #:
    #: The part this specifies, at 1.25e5 and the 8.75 mm spool:
    #:
    #:     torsional rate          9.57 N.m/rad
    #:     working deflection     +/-11.7 deg at the 1.95 N.m motor peak
    #:     delivered through the 3000 rad/s rotor servo   ~118 kN/m (95 %)
    #:     back-driven landing    516 N cable -> 4.5 N.m -> 27 deg: needs a HARD STOP
    series_k: float = 1.25e5

    # Minimum tension kept in every cable so it never goes slack (N).  ❓ TBD
    # In antagonistic mode this is the co-contraction floor on the "slack" side.
    pretension: float = 5.0

    # --- Tendon non-idealities (ADR-0003: friction & stretch are now leg-side
    #     concerns too, not spine-only). Defaults reduce EXACTLY to the previous
    #     frictionless / inextensible behaviour, so existing budgets are unchanged.

    # Capstan (Coulomb) friction over the routing pulleys / sheaths. The motor-side
    # cable tension differs from the joint-side tension by exp(±mu * wrap_angle):
    # PULLING against the load costs exp(+mu*wrap), PAYING OUT gains exp(-mu*wrap).
    #   friction_coeff : mu, dimensionless Coulomb coefficient of the routing.
    #   wrap_angle     : theta_wrap, TOTAL cable wrap over all guides (rad).
    # mu = 0 OR wrap = 0  =>  factor = 1  =>  motor-side tension == joint-side.
    # mechanical/LEG_TENDON_SPEC.md gives mu ~= 0.10 (low-friction idlers) and
    # PER-STATION wrap angles that differ by joint (the distal ankle path is worst,
    # ~+87% motor-side). This scalar model can't hold per-joint wrap, so wrap_angle
    # is left 0 (inert) pending a per-joint-wrap extension; set mu here so it's
    # ready, and use the sensitivity tool with explicit wrap to explore the effect.
    friction_coeff: float = 0.10
    # ⚠️ **M106: this was 0.0 and that is why the motor has never been sized
    # against friction.** Its own comment called it "inert pending a
    # per-joint-wrap extension"; ADR-0083 was that extension and nothing read
    # it. `None` now means "use the routed `pair_wrap`", which is per joint AND
    # per side -- the hip's two cables are 135.6 and 239.4 deg, so a scalar
    # could never have carried them. Set it to a number, including 0.0, to
    # override and get the old frictionless behaviour back exactly.
    wrap_angle: float | None = None

    # ✅ M78: the per-joint-wrap extension the comment above was waiting for,
    # SOLVED from station geometry by `mechanical/cad/leg_tendons.route()`
    # rather than assumed. `(flexor, extensor)` total wrap in radians per pair,
    # at the stance pose. `[solved]` — `test_the_EXTENSOR_SIDE_was_never_solved`
    # re-derives these from the router, so they cannot drift from the geometry.
    #
    # ⚠️ **ADR-0042's retraction covered the FLEXOR only.** It cites the
    # ankle at ~108 deg / 1.21x, which is `side=+1`; nothing in this project had
    # ever called `route(side=-1)`. The extensor solves to **392.9 deg / 1.985x**,
    # back at the 1.87x ADR-0042 called an over-estimate.
    #
    # ⚠️ **M106: EVERY ONE OF THOSE NUMBERS WAS FOR A CABLE THAT RUNS TO
    # WHERE THE MOTORS USED TO BE.** `leg_tendons.route()` falls back to
    # `SPOOL_OFFSET`, a single 2-D diagonal that only ever suited the pre-M88
    # upright girdle, and `tomcat_assembly` overrides it precisely because the
    # defaults miss the real motors by 20 to 44 mm. `route(spools=)` and
    # `tomcat_trunk.leg_spools` both arrived in **M91**; ADR-0083 is **M78**, so
    # the table was solved thirteen milestones before the cables were connected
    # and nobody re-ran it. Re-solved on the spools the trunk actually has:
    #
    #     pair     flexor  was -> now        extensor  was -> now
    #     hip      122.1 -> 135.6 deg          7.9 -> 239.4 deg
    #     knee     158.6 -> 141.0              124.7 -> 150.0
    #     ankle    107.6 -> 152.8              392.9 -> 239.2
    #
    # ⚠️ **ADR-0083's headline inverts.** The worst case is not the ankle
    # extensor at 1.985x; it is the hip and ankle extensors tied at ~1.52 -- and
    # the hip extensor is the one ADR-0083 recorded as essentially frictionless
    # at **1.014x**. This parameter understated that cable by 50 %.
    #
    # ⚠️ The guard named above could not see it: it re-derived from the
    # router **with the same missing `spools=`**, so it reproduced the defect it
    # exists to prevent while asserting the values "cannot drift".
    #
    # ⚠️ **M111: re-solved at the 36/34/22 arms.** A bigger sheave changes the
    # tangent points, so the wraps move with the arm: hip 135.6/239.4 ->
    # 148.3/255.6, ankle 152.8/239.2 -> 147.0/243.9. ADR-0103's 1.827 N.m was
    # computed on these, not on the 28/25/14 ones.
    #
    # ⚠️ **M120: re-solved again, because G3 lengthened the rows** and moved
    # every spool centre ~1.8 mm along x (ADR-0111). Hip 148.3 -> 145.7,
    # ankle extensor 243.9 -> 243.5: a little LESS wrap on five of the six
    # cables (the knee flexor gains 0.05 deg).
    pair_wrap: tuple = ((2.5432, 4.4336),     # hip   145.7 / 254.0 deg
                        (2.5035, 2.5657),     # knee  143.4 / 147.0 deg
                        (2.5603, 4.2500))     # ankle 146.7 / 243.5 deg

    #: ✅ **M122 (ADR-0112): friction is counted where the cable SLIDES.**
    #: M107-M121 charged `exp(mu * theta)` for every degree of wrap on every
    #: pulley, including the joint sheaves the cable is ANCHORED to and the vias
    #: that turn on bearings -- a capstan on surfaces the cable does not slide
    #: over. LEG_TENDON_SPEC §3.4 asked for "open pulleys" and then priced them
    #: as fixed guides; that was 71 % of the trot's copper in M107. Now:
    #:   - `"drive"`: the cable slides only in its Bowden CONDUIT, over the
    #:     conduit's bend (`conduit_bend`, worst over the hip range,
    #:     `mechanical/cad/tendon_exit.py`) at `conduit_mu`; every RUNNING pulley
    #:     costs `pulley_efficiency` (bearing and bending loss); anchored sheaves
    #:     and the spool cost nothing.
    #:   - `"capstan"`: the M107-M121 convention on `pair_wrap`, kept for the
    #:     record and for the studies that published on it.
    #: A caller that sets `wrap_angle` still gets exactly `exp(mu * wrap_angle)`.
    friction_model: str = "drive"
    #: UHMWPE sliding in a PTFE-lined conduit. `[assumed]`
    conduit_mu: float = 0.07
    #: One ball-bearing pulley, bearing plus bending hysteresis. `[assumed]`
    pulley_efficiency: float = 0.97
    #: Worst conduit bend over the hip range, (flexor, extensor) per joint, rad
    #: -- the HIND leg's; the fore leg's is `tendon_exit.conduit_bend("fore")`.
    #: `[derived: mechanical/cad/tendon_exit.py]`
    #: ✅ M123 (ADR-0114): the knee and ankle conduits run along the hip axis
    #: to the hub; the hip pair's from the new hip-station row.
    conduit_bend: tuple = ((2.1221, 2.4463),  # hip   122 / 140 deg
                           (1.6911, 1.6908),  # knee   97 /  97 deg
                           (1.7122, 1.7058))  # ankle  98 /  98 deg
    #: Running pulleys between spool and anchored sheave: the knee via, on the
    #: ankle pair, and nothing else (ADR-0112).
    pulley_count: tuple = ((0, 0), (0, 0), (1, 1))

    # Series cable compliance: model the tendon as a linear spring of stiffness
    # k_cable (N/m). Under tension T it stretches dL = T / k_cable, so the motor
    # must wind extra travel (dL / r_spool) beyond the geometric r*q to hold a
    # joint angle; if uncompensated the joint under-rotates by dL / r.  ❓ TBD.
    #   None (or a non-finite value such as inf)  =>  inextensible, no stretch.
    k_cable: float | None = None

    # Spring-return mode only: torsional spring stiffness (N·m/rad) and rest
    # angle (rad) per joint.  Unused in antagonistic mode.  ❓ TBD
    spring_stiffness: tuple[float, ...] = (0.5, 0.5, 0.3)
    spring_rest_angle: tuple[float, ...] = (0.0, 0.4, 0.0)


@dataclass(frozen=True)
class SpineParams:
    """Geometry / actuation of the articulated spine (sagittal-plane chain).

    The spine is modelled as a serial chain of revolute joints (one per
    inter-vertebral segment) in the SAME sagittal plane as the leg model:
    x forward, z up. Per ADR-0006 and the literature review, the seed geometry
    is 3 segments; the review recommends 2-3 segments x ~3 DOF each. This 2D
    model exposes ONLY the sagittal (dorsoventral flexion/extension) DOF of each
    segment — lateral bending and axial rotation are out of plane and NOT yet
    modelled.

    Sign convention (matches the leg): a segment's cumulative direction angle is
    measured CCW from +x (from +x toward +z). A POSITIVE joint angle rotates the
    outboard portion of the spine CCW relative to the inboard segment, i.e. it
    lifts the distal end dorsally (upward). A uniform positive bend curls the
    chain upward into a dorsiflexed / arched-back ("Halloween cat") posture.

    IMPORTANT stiffness caveat
    --------------------------
    The cat whole-spine value **53.62 N/mm is AXIAL COMPRESSIVE stiffness
    (force/length)**, NOT the per-joint ROTATIONAL stiffness (N·m/rad) an
    articulated tendon spine needs. It must NOT be dropped into `spring_stiffness`
    below. A geometry-based conversion (axial N/mm + segment lever arms ->
    per-joint N·m/rad) is still owed; `spring_stiffness` here is an unsourced
    placeholder pending that conversion.

    Directional compliance rank (cat FEA 2024), most-compliant -> stiffest:
        axial rotation  >  extension (dorsoventral)  >  lateral bending
    Only the middle axis (extension) lives in this 2D sagittal model; the rank is
    recorded here so per-axis stiffness/limits can be seeded correctly once the
    model is extended to 3D.
    """

    # Number of inter-vertebral revolute segments.  Seed = 3 (ADR-0006).
    n_segments: int = 3

    # Segment (link) lengths, base/rear -> front (m).  First-pass from
    # mechanical/SPINE_TAIL_SPEC.md: tapered (rear lumbar longer/more mobile),
    # 0.195 m total at ~3 kg cat-torso scale.  Still a placeholder, not committed.
    segment_lengths: tuple[float, ...] = (0.075, 0.065, 0.055)

    #: Where the HIND hips sit in the rear girdle frame, whose origin is the
    #: spine's root joint (m, x forward).
    #:
    #: ⚠️ **M122 (ADR-0112): the first spine joint moved 30 mm FORWARD of the
    #: hind hips.** ADR-0006 rooted the chain at the hip station, so the rear
    #: hip and spine joint 0 were one point: the hip boss hung 27 mm off the
    #: section the trunk pinches to bend, and the hind knee and ankle cables had
    #: no way to reach the hip axis -- the motors packed right behind it close
    #: the only path to 0.1 mm. `tomcat_trunk` had it `[owed]` as the sacrum
    #: that belongs INSIDE the pelvis. The segment lengths are unchanged, so
    #: the hips are now 195 + 30 = 225 mm apart and the trunk 30 mm longer.
    #:
    #: ✅ **M123 (ADR-0114): 65 mm.** The knee and ankle motors now sit either
    #: side of their hip, spools facing across it, so their cables run straight
    #: along the hip axis into a hollow hip (`tendon_exit`). The rear girdle's
    #: motors are two rows straddling the hip, and the one in FRONT of it has to
    #: end clear of spine joint 0's neck -- which puts the hip 65 mm behind it.
    rear_hip_x: float = -0.065
    #: Where the FORE hips sit ahead of the front girdle frame's origin (the
    #: spine's last joint station, `sum(segment_lengths)`), m. M123 (ADR-0114):
    #: the row BEHIND the fore hip has to start clear of spine joint 2's neck.
    #: The hips are 195 + 65 + 10 = 270 mm apart.
    front_hip_x: float = 0.010

    # Per-segment sagittal joint-angle limits (rad), (min, max).  ❓ TBD.
    # ±25° per joint -> ~±75° whole-spine sagittal range.
    q_min: tuple[float, ...] = (-0.436, -0.436, -0.436)
    q_max: tuple[float, ...] = (0.436, 0.436, 0.436)

    # LATERAL (yaw) joint limits per segment, rad — ADR-0009. The sagittal
    # q_min/q_max above drive dorsoventral arch; these drive the side-to-side
    # bend that produces the body SWAY static stability needs. ADR-0006 always
    # specified this axis; ADR-0009 finally budgets motors for it.  ❓ TBD
    lateral_q_min: tuple[float, ...] = (-0.262, -0.262, -0.262)   # -15 deg
    lateral_q_max: tuple[float, ...] = (0.262, 0.262, 0.262)      # +15 deg

    # Per-segment tendon moment arm about the vertebral joint (m).
    # Raised from the initial 0.020 m to 0.030 m per mechanical/SPINE_TAIL_SPEC.md:
    # T = tau/r, so 0.020 m amplified peak cable tension well above the ~20-70 N
    # RoboCat band; ~0.030 m brings it near the top of the band.  Still ❓ TBD.
    joint_moment_arm: tuple[float, ...] = (0.030, 0.030, 0.030)

    # LATERAL tendon moment arm per segment (m) — ADR-0009 follow-up.
    # The dorsoventral tendon rides over the tall SPINOUS process (30 mm); the
    # lateral tendon has only the much shorter TRANSVERSE process. Since
    # T = tau/r, a short lateral arm directly amplifies cable tension AND the
    # motor torque behind it.
    #
    # SPINE_TAIL_SPEC §1.5 originally assumed the bare transverse-process width,
    # 0.015 m. That does not work: sizing the lateral drive against the M5
    # crossover inertia (``WholeBody.lateral_spine_loads``) puts the BASE joint at
    # 1.18 N.m of motor torque -- 1.07x ADR-0008's 1.10 N.m trot sizing point,
    # i.e. OVER peak, for the one joint that must swing the whole forequarters.
    # 0.020 m brings it to 0.88 N.m (0.80x) with margin.
    #
    # 0.020 m is bought with a dedicated lateral pulley post on each vertebra --
    # exactly the trick ASSEMBLY_SPEC already uses to realise the 30 mm
    # dorsoventral arm via a milled spinous-process post, rather than trusting
    # bone geometry. +/-20 mm sits well inside the +/-34 mm thoracic rib cavity.
    lateral_moment_arm: tuple[float, ...] = (0.020, 0.020, 0.020)

    # Motor spool radius for the spine tendons (m).
    # FLOORED by the UHMWPE cable's minimum bend diameter (10x the cable) —
    # torque cannot be bought back by shrinking it.
    #
    # ⚠️ M41: 0.008 -> 0.00875.  The floor quoted here was ~0.0075 m for a Ø1.5 mm
    # cable; ADR-0010 re-sized the cable to Ø1.75 mm, which floors it at
    # **0.00875**.  The leg spool moved for the same reason and on the same date it
    # should have.
    motor_spool_radius: float = 0.00875

    #: G3's series-elastic element on the SPINE cables (N/m). ⚠️ M112: unchanged at
    #: 1.5e5 -- the spine's 30 mm arm did not move, so neither does the joint
    #: stiffness this sets (`k r^2`). ADR-0104 measured what softening it costs:
    #: the spooled righting goes 2.62 s at 1.5e5 -> 4.72 at 1.25e5 -> fails at 1.0e5.
    series_k: float = 1.5e5

    # Minimum cable tension / mechanical slack floor (N).  ❓ TBD.
    # Kept at the leg's 5 N so the two budgets are comparable.  Note: the AIC
    # *control* co-contraction bias is a separate, larger quantity — Kengoro's
    # T_bias ≈ 19.6 N (LITERATURE_REVIEW.md Seed derivation B) — passed at runtime
    # via TendonMap.resolve(..., t_bias=...), not baked in here.
    pretension: float = 5.0

    # Spring-return mode only: per-segment torsional stiffness (N·m/rad) and rest
    # angle (rad).  Seeded ~1.0 N·m/rad from the axial->rotational geometry bridge
    # in LITERATURE_REVIEW.md (Seed derivation A): the sagittal (dorsoventral
    # extension) axis this 2D model exercises.  ◐/⚠️ ORDER-OF-MAGNITUDE ONLY —
    # good to a factor of ~2-3; correct in ranking/scale, not a measured value.
    spring_stiffness: tuple[float, ...] = (1.0, 1.0, 1.0)
    spring_rest_angle: tuple[float, ...] = (0.0, 0.0, 0.0)

    # Tendon non-idealities, same meaning as TendonParams (capstan friction +
    # series cable stretch). Long spine tendons run over many vertebral guides,
    # so wrap_angle is expected to be LARGER here than at a leg once sourced.
    # Defaults reduce to the previous frictionless / inextensible behaviour.  ❓ TBD.
    friction_coeff: float = 0.0
    wrap_angle: float = 0.0
    k_cable: float | None = None

    # --- MASS PROPERTIES (M4).  ❓ ALL PLACEHOLDER; see the params.py module
    #     docstring for the full apportionment of the 3.0 kg body.
    #
    # Per-segment TRUNK mass (kg), rear -> front. Rear = lumbar (lighter, the
    # mobile bending region); front = thoracic/ribcage + viscera (heavier). Sums
    # to 1.30 kg.  Matches the 13-thoracic / 7-lumbar formula in
    # mechanical/reference/ANATOMY.md qualitatively, not quantitatively.
    # Rear -> front. The MIDDLE segment dominates because it physically carries
    # both the ~0.300 kg battery and the 7-motor spine+tail bank, which the CAD
    # packaging places in the mid-body bay between the girdles (NOT in the rear
    # girdle, as the pre-ADR-0009 apportionment assumed).
    #
    # ⚠️ **141.8 g left the middle segment in M102** -- the 19th motor (131.7 g)
    # (132.0 g) and the tail it drives (9.8 g of EVA foam). Both were bought:
    # ADR-0096 pays for them out of the structure allowance, which is what this
    # segment carries, so the body total does not move. They now sit on the
    # REAR girdle, where `tomcat_trunk` actually places them.
    #
    # ⚠️ **M120: the spine's six G3 flexures ride it too**, 6 x 3.86 g = 23.2 g,
    # which ADR-0107 priced and nothing carried. `[derived: cad/girdle_inertia.py]`
    segment_mass: tuple[float, ...] = (0.130, 1.2349, 0.127)

    # Fraction along each segment (from its INBOARD/rear vertebra) at which that
    # segment's mass acts. 0.5 = uniform rod.  ❓ TBD
    segment_com_frac: tuple[float, ...] = (0.5, 0.5, 0.5)

    # Girdle (limb-girdle) masses (kg), lumped at the corresponding end vertebra.
    # FRONT = shoulder girdle; it deliberately ABSORBS the HEAD + NECK, which are
    # not separate bodies in this model. REAR = pelvic girdle. Both are now a
    # bottom-up COUNT of what each girdle actually holds, not a free variable:
    #     front = 6 leg motors x 132 g + head/neck 0.240 + structure 0.090
    #     rear  = 6 leg motors x 132 g + structure 0.110
    # where 132 g = the SURVEYED part (SteadyWin GIM3505-9: 120 g motor + driver),
    # which is what the values below are built from -- this comment said 77 g
    # (the superseded 72 g class + 5 g board) until M57 checked the arithmetic.
    # The rear girdle is the LIGHTER one: the spine/tail bank moved to the mid-body.
    #
    # ⚠️ **The REAR girdle grew 141.8 g in M102.** It holds the tail's motor --
    # the 19th, which ADR-0096 placed on trunk body 0 and this model had never
    # heard of -- and the tail itself. The front girdle's mass does not move:
    # the head was always inside it, just never anywhere in particular.
    #: ⚠️ **M120: each girdle carries its six leg G3 flexures**, 6 x 5.20 g,
    #: between motor and spool (ADR-0107, drawn in `g3_flexure.py`).
    front_girdle_mass: float = 1.15318
    rear_girdle_mass: float = 1.07512     # M123: the tail shrank with the rear body

    #: Girdle housing, full extents (m), and the height of its centre above the
    #: girdle mount vertebra.
    #:
    #: ⚠️ **The MJCF drew a 60x60x56 mm box that its own motors do not fit in.**
    #: Six GIM3505-9 modules with their spools are **212,133 mm3** against that
    #: box's **201,600 -- 105 %**, a packing fraction no arrangement of cylinders
    #: can reach. `tomcat_packaging.girdle_box()` sizes the housing from the
    #: motors' bounding box plus 5 mm of clearance, and gets **82 x 86.5 x
    #: 108.2 mm**, 3.8x the volume.
    #:
    #: The motors STACK upward, so the housing is not centred on the hip axis:
    #: it runs 31 mm below and **77 mm above** it. Belly clearance barely moves
    #: (148 -> 145 mm); what grows is the back and the flanks.
    #: `[derived: mechanical/cad/tomcat_packaging.py]`
    girdle_size: tuple[float, float, float] = (0.0820, 0.0865, 0.1082)
    girdle_offset_z: float = 0.0231

    #: Height of the spine joint axis above the hip axis (m).
    #:
    #: ⚠️ **This was 0 -- the vertebral column ran along the belly.** Every
    #: spine body sat at hip height, so each joint neck had to blend the trunk
    #: section down to the underside of the body, and the back notched **52.6 mm
    #: at each of three joints**, 42 % of the chest depth. The requirement the
    #: trunk was shaped to is a flat back over a tucked belly, and it was never
    #: met; `tomcat_trunk.report()` printed the parameter `Z_DORSAL` and called
    #: it the dorsal line, so nothing said so.
    #:
    #: ✅ **The value is the spinous process, not a guess.** In a cat the back
    #: you feel IS the row of spinous process tips, with skin over them. The
    #: dorsal cable's moment arm is what sets how far a process stands off the
    #: axis, so putting the axis one moment arm below the dorsal line makes the
    #: tips land exactly on it:
    #:
    #:     z = Z_DORSAL - joint_moment_arm = 79.8 - 30.0 = 49.8 mm
    #:
    #: The barrel still necks at each joint -- it has to, to bend -- and the gap
    #: between the process tips is what the skin spans. That is a cat.
    #: `[derived: cad/tomcat_trunk.py Z_DORSAL, joint_moment_arm]`
    spine_axis_z: float = 0.0498

    #: ⚠️ **Raising the axis does NOT fix where the trunk's mass is.** It was
    #: tempting to claim it did; measured, it is a wash:
    #:
    #:     CAD, structure + motors, 2729 g drawn   21.1 mm above the hip axis
    #:     model, ventral axis (was)                9.3 mm   error -11.8
    #:     model, dorsal axis (now)                31.4 mm   error +10.3
    #:
    #: The sign flips and the magnitude does not. The cause is not the axis: it
    #: is that this mass model still describes the **two-girdle** architecture
    #: M88 replaced. `segment_mass` is (130, 1354, 127) g -- one heavy middle
    #: carrying "the 7-motor spine and tail bank" -- while the CAD distributes
    #: 6/4/2/6 motors along the trunk and measures 582 g on the first middle
    #: body and 304 g on the second. `mjcf_tendon` still hangs the spine spools
    #: on the rear girdle as well. Re-apportioning is a milestone of its own and
    #: it moves every balance result again.  `[owed]`
    #:
    #: ✅ **M102 did the part that could be done without restructuring.** The
    #: head and the tail are placed, the 19th motor is on the girdle body 0
    #: maps to, and the phrase "the 7-motor spine+tail bank" is gone -- the
    #: seventh was the tail's. What remains owed is the ARCHITECTURE: four CAD
    #: bodies against three model segments plus two girdles, and `mjcf_tendon`
    #: still hanging the spine spools on the rear girdle. See
    #: [ADR-0099](../../../docs/DESIGN_DECISIONS.md#adr-0099).

    # Girdle CoM offset (x, z) in the girdle's own frame (m). (0, 0) = the mass
    # acts exactly at the girdle mount vertebra.
    #
    # ⚠️ Both were (0, 0) and marked TBD. Measured from the packed motors, the
    # mass sits **17 mm above** the mount and a few mm behind it -- the bank is
    # 2-per-layer and the third motor stacks on the inboard column, which is
    # where the -4.6 / -5.7 mm of x comes from.
    #
    # ⚠️ **`mjcf.py` hard-coded `pos="0 0 0"` for the same body**, and agreed
    # with this parameter only because the parameter was (0, 0). The same shape
    # of defect as the paw's `0.5 * l4`, found the same way -- by giving the
    # parameter a real value.  `[derived: cad/tomcat_packaging.py]`
    #
    # ⚠️ **M102 put the head and the tail where they are.** The head's 240 g
    # sat at the housing centre because nothing had ever placed it; its CoM is
    # **149.5 mm ahead of the hip and 118.3 above it**, and moving it there
    # carries the whole front girdle **+32.0 mm forward and +20.4 up**. The
    # rear girdle moves **-12.0 and +4.5** under the tail and its motor.
    # `[derived: cad/girdle_inertia.py]`
    # M120 adds the leg G3 at the motor bank and moves both a fraction of a mm.
    # M122: the hind hips and the rear motor bank sit 30 mm behind the spine
    # root now (ADR-0112), and the G3 flexures at their spools' row ends.
    # M123 (ADR-0114): each girdle's motor bank rides its hip, now 65 mm
    # behind the spine root and 10 mm ahead of the front girdle's origin.
    front_girdle_com: tuple[float, float] = (0.03556, 0.03703)
    rear_girdle_com: tuple[float, float] = (-0.07392, 0.01879)

    #: MJCF `fullinertia` about each girdle's CoM, in its own frame
    #: (ixx iyy izz ixy ixz iyz, kg m²).
    #:
    #: ⚠️ A uniform box at the housing size would be **2.9x** too high -- the six
    #: motors are only 28 % of that volume. These place the motors where the CAD
    #: packs them and spread the remainder (structure, and the head+neck the
    #: front girdle absorbs) through the housing.
    #:
    #: ✅ **M102 placed the head, and "would raise the pitch inertia" was the
    #: understatement.** This comment used to read *"the head is the weak point
    #: and it is not in the box... lumped in the housing here... putting it
    #: forward would raise the pitch inertia"* -- a defect correctly described
    #: and left in place for nine milestones. Measured, the front girdle's
    #: **Iyy goes 1.1412e-03 -> 8.2412e-03, a factor of 7.2**, and `Ixz`
    #: changes SIGN (+1.4855e-04 -> -3.2340e-03) because the head is high and
    #: forward where the motor bank is low and behind.
    #:
    #: The rear girdle gains the tail and the 19th motor: **Iyy x2.5**.
    #:
    #: ⚠️ The tag used to say `[derived: cad/tomcat_packaging.py]` and that
    #: module does not produce these -- it packs motors and sizes housings and
    #: never prints a tensor. `girdle_inertia.py` is the missing derivation,
    #: and it RE-DERIVES the published numbers before replacing them: strip the
    #: head-as-housing-lump out of the old front girdle and what is left is the
    #: rear girdle plus 20 g of structure, agreeing to 0.5 % on Ixz.
    #: `[derived: cad/girdle_inertia.py]`
    #: M120: with the leg G3 flexures (`girdle_inertia.g3_parts`).
    front_girdle_inertia: tuple[float, ...] = (
        3.2362e-03, 7.9601e-03, 5.6473e-03, 0.0, -3.1408e-03, -1.7442e-05)
    rear_girdle_inertia: tuple[float, ...] = (
        1.0084e-03, 1.0870e-03, 9.5915e-04, 0.0, 2.2677e-04, -1.7442e-05)

    #: Per-segment `fullinertia`, or `None` to derive it from the segment box.
    #:
    #: ⚠️ Only the MIDDLE segment is measured: it carries the 7-motor spine and
    #: tail bank plus the battery in an 82 x 82 x 100 mm mid-body bay, which the
    #: 60 x 60 mm cross-section the others assume understates by **1.6x**. The
    #: outer two are bone and structure and remain `[assumed]`.
    #: `[derived: cad/tomcat_packaging.py]`
    #:
    #: ⚠️ Scaled by **0.89527** in M102, the fraction of the middle segment's
    #: mass that stays after the 19th motor and the tail leave it. The bay's
    #: geometry did not change, so the tensor scales with the mass it holds.
    segment_inertia: tuple[tuple[float, ...] | None, ...] = (
        None,
        (1.2869e-03, 1.2869e-03, 1.0071e-03, 4.8756e-05, 5.7877e-05, 5.7877e-05),
        None,
    )

    def __post_init__(self) -> None:
        for name in (
            "segment_lengths",
            "q_min",
            "q_max",
            "joint_moment_arm",
            "spring_stiffness",
            "spring_rest_angle",
            "segment_mass",
            "segment_com_frac",
        ):
            got = len(getattr(self, name))
            if got != self.n_segments:
                raise ValueError(
                    f"SpineParams.{name} has {got} entries; "
                    f"expected n_segments={self.n_segments}"
                )

    @property
    def total_length(self) -> float:
        """Straight-spine distance from the rear to the front girdle mount (m)."""
        return float(sum(self.segment_lengths))

    @property
    def chain_mass(self) -> float:
        """Mass of the spine SEGMENTS alone (kg), excluding the girdles."""
        return float(sum(self.segment_mass))

    @property
    def trunk_mass(self) -> float:
        """Spine segments + both girdles (kg) -- the whole body minus the legs."""
        return self.chain_mass + self.front_girdle_mass + self.rear_girdle_mass


@dataclass(frozen=True)
class LoadCase:
    """A static loading scenario for the torque budget."""

    name: str
    # Total robot mass. History: 3.0 (M1 placeholder) -> 4.045 (ADR-0010, once a
     # real 132 g motor was sourced) -> 4.3041 (ADR-0046/M41, once the leg was
     # drawn as manufacturable parts and came out 167 g rather than 110)
     # -> **4.3833** (ADR-0093/M93, once the leg was drawn with the via pulleys
     # its own routing needs, shafts that reach them, and tubes that make SF 2.5
     # -- 186.7 g) -> **4.4684** (ADR-0103/M111, the 36/34/22 sheaves the
     # fore leg's load needed: +21 g per leg, 208 g). Kept in sync with
     # DEFAULT_BODY_MASS_KG by test_mass.py.
    # -> 4.55397 (ADR-0111/M120, the eighteen G3 flexures: 85.6 g)
    # -> 4.50092 (ADR-0112/M122: the legs drop the hip via, the return
    # spring and the anchor pins, -52.4 g for four; the trunk is 30 mm longer)
    # -> **4.60530** (ADR-0114/M123: the hollow hip, +26 g per leg; the rear
    # body is 26 mm shorter and the tail with it).
    body_mass_kg: float = 4.60530
    n_stance_legs: int = 2             # legs sharing the load (e.g. trot => 2).
    dynamic_factor: float = 1.5        # peak/static impact multiplier.  ❓ TBD

    @property
    def foot_support_force_N(self) -> float:
        """Vertical force one stance leg must produce to support the body."""
        return (
            self.body_mass_kg
            * GRAVITY
            * self.dynamic_factor
            / max(self.n_stance_legs, 1)
        )


@dataclass(frozen=True)
class WholeBodyLoadCase:
    """A whole-body static loading scenario (spine posture + which legs bear load).

    Extends `LoadCase` to the combined budget: the spine is posed at `spine_q`
    (per-segment sagittal angles, rad) and only the legs in `stance_legs` are
    planted and share the support force. Feeds `whole_body_budget.evaluate`.

    The per-leg leg-workspace sweep reuses a plain `LoadCase` (see `.leg_load`);
    the spine joint loads are derived from the girdle reactions + a distributed
    body-weight gravity moment in the arched geometry (see whole_body_budget).
    """

    name: str
    body_mass_kg: float = 4.3041      # see LoadCase; ADR-0046 raised it from 4.045
    dynamic_factor: float = 1.0
    # Spine posture for this case (rad per segment).  Length must match the
    # SpineParams used in the budget.  Straight = zeros; arch = uniform positive.
    spine_q: tuple[float, ...] = (0.0, 0.0, 0.0)
    # Which leg mounts are planted (bearing load) in this case.
    stance_legs: tuple[str, ...] = ("LF", "RF", "LR", "RR")

    @property
    def n_stance_legs(self) -> int:
        return len(self.stance_legs)

    @property
    def foot_support_force_N(self) -> float:
        """Vertical force one stance leg must produce to support the body."""
        return (
            self.body_mass_kg
            * GRAVITY
            * self.dynamic_factor
            / max(self.n_stance_legs, 1)
        )

    @property
    def leg_load(self) -> LoadCase:
        """A per-leg `LoadCase` so the existing leg budget sweep can be reused."""
        return LoadCase(
            name=self.name,
            body_mass_kg=self.body_mass_kg,
            n_stance_legs=self.n_stance_legs,
            dynamic_factor=self.dynamic_factor,
        )


# Convenience singletons used by the demo and tests.
DEFAULT_LEG = LegParams()
DEFAULT_TENDON = TendonParams()
DEFAULT_SPINE = SpineParams()

# Cats are NOT fore/hind symmetric. The nominal DEFAULT_LEG doubles as the
# HIND leg (longer shank, bigger toe-break — the folded propulsion limb); the
# FORE leg is a touch more columnar (longer proximal humerus, shorter distal
# metacarpus, smaller paw toe-break). Reaches are kept ≈equal (~0.28 m) so the
# body stays roughly level. Used by the CAD to distinguish front vs rear legs;
# the actuated architecture (3 joints/leg) is identical.  ❓ placeholders.
DEFAULT_HINDLEG = DEFAULT_LEG
DEFAULT_FORELEG = LegParams(
    l1=0.100, l2=0.090, l3=0.065, l4=0.025,   # humerus, radius, metacarpus, paw
    paw_angle=math.radians(40.0),
    # The forelimb folds the OPPOSITE way to the hindlimb: a cat's ELBOW points
    # backward while the STIFLE points forward, so the middle joint's range is
    # POSITIVE here and negative on the hind leg. Both paws still point FORWARD —
    # the fore/hind difference is the fold direction, NOT a mirror of the whole
    # limb (mirroring would point the front paws at the tail).
    # Demanded working set over the gait cycle: shoulder -157..-126,
    # elbow +69..+103, carpus -6..+32 deg.  ❓ TBD placeholders with margin.
    q_min=(math.radians(-170.0), 0.0, math.radians(-30.0)),
    q_max=(math.radians(30.0), math.radians(150.0), math.radians(150.0)),
    # `[derived: cad/tomcat_leg_detail.py]` at the FORE link lengths.
    #
    # ⚠️ M41: was (0.045, 0.029, 0.014, 0.007) = 0.095 kg, an ASSUMED 1.16x
    # fore/hind asymmetry ("the fore limb is the lighter, more columnar limb").
    # Drawn as parts the asymmetry is **1.00x** — 0.167 kg both ends — because the
    # joint hardware dominates and it is the SAME hardware on both, so the shorter
    # fore links barely register.  Design review F2 settled the fore/hind weight
    # split using the assumed asymmetry and needs re-checking (ADR-0043).
    # ⚠️ M111: re-measured at the 36/34/22 sheaves (ADR-0103), +21 g per leg.
    # ⚠️ M122: -13 g, as the hind leg (ADR-0112).
    # ⚠️ M123: +26 g on the humerus, the hollow hip (as the hind leg).
    link_mass=(0.10336, 0.07479, 0.03559, 0.00759),
    # ⚠️ Re-derived at the FORE link lengths, not copied from the hind leg:
    # the same hardware on shorter links moves the fractions.
    link_com_frac=(0.0674, 0.0705, 0.1799, 0.8743),
    link_com=(
        (0.006743, 0.005655, -0.000115),      # humerus
        (0.006343, 0.007171, -0.000090),      # radius
        (0.011696, 0.006862, -0.000109),      # metacarpus
        (0.021858, 0.000000, -0.002353),      # paw
    ),
    link_inertia=(
        (3.8063e-05, 1.0172e-04, 7.7418e-05, 2.1022e-06, -2.3574e-07, -1.1053e-07),
        (1.6151e-05, 4.1863e-05, 4.0395e-05, 2.3790e-06, 3.4343e-09, -4.9869e-08),
        (5.8584e-06, 1.8346e-05, 2.0201e-05, 2.8569e-06, 9.1982e-08, -2.6682e-08),
        (3.0802e-07, 7.2523e-07, 9.1886e-07, 0.0, 5.6145e-08, 0.0),
    ),
)

# Total mass of the default body, for cross-checking against LoadCase.body_mass_kg.
# 2 fore legs + 2 hind legs + spine segments + both girdles == 3.00 kg by
# construction (see the module docstring's apportionment rule).
DEFAULT_BODY_MASS_KG = (
    2 * DEFAULT_FORELEG.mass + 2 * DEFAULT_HINDLEG.mass + DEFAULT_SPINE.trunk_mass
)
DEFAULT_LOADS: tuple[LoadCase, ...] = (
    LoadCase("stand (4-leg)", n_stance_legs=4, dynamic_factor=1.0),
    LoadCase("trot (2-leg)", n_stance_legs=2, dynamic_factor=1.5),
    LoadCase("land (1-leg)", n_stance_legs=1, dynamic_factor=2.5),
)

# Whole-body cases for the combined spine+legs budget. Spine postures assume the
# 3-segment DEFAULT_SPINE (arch = uniform +20 deg dorsiflexion).
_ARCH = (0.349, 0.349, 0.349)  # ~20 deg per segment
DEFAULT_WHOLE_BODY_LOADS: tuple[WholeBodyLoadCase, ...] = (
    # Quiet stand: straight spine, all four legs planted.
    WholeBodyLoadCase(
        "stand (4-leg, straight)",
        dynamic_factor=1.0,
        spine_q=(0.0, 0.0, 0.0),
        stance_legs=("LF", "RF", "LR", "RR"),
    ),
    # Arched "Halloween cat": same support, but the dorsiflexed geometry
    # redistributes the spine joint loads (curls the forequarters up and back
    # over the pelvis, shifting where the gravity/reaction moments land).
    WholeBodyLoadCase(
        "arch (4-leg, dorsiflexed)",
        dynamic_factor=1.0,
        spine_q=_ARCH,
        stance_legs=("LF", "RF", "LR", "RR"),
    ),
    # Single-leg front landing: one front leg takes the whole impact, so the
    # front-girdle reaction is large and unbalanced by the rear.
    WholeBodyLoadCase(
        "land (1 front leg)",
        dynamic_factor=2.5,
        spine_q=(0.0, 0.0, 0.0),
        stance_legs=("LF",),
    ),
)
