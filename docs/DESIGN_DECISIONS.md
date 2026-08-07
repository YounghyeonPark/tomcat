# TomCat — Design Decisions (ADR log)

Lightweight Architecture Decision Records. Each entry captures a decision, its
context, and consequences. Status is one of: **Proposed**, **Accepted**,
**Superseded**.

---

## ADR-0001: Tendon-driven actuation with centralized motors
- **Status:** Accepted
- **Context:** Direct-drive joints add mass and rotational inertia to the limbs,
  reducing agility and shock tolerance.
- **Decision:** Relocate motors into the torso and drive joints via synthetic
  cables (tendons) over pulleys.
- **Consequences:** Lower limb inertia and better compliance; higher control
  complexity due to coupled cables, tendon friction, and stretch that must be
  modeled and compensated.

## ADR-0002: Antagonistic actuation vs. return spring
- **Status:** Accepted
- **Context:** A cable can only pull, not push. Each DOF needs a way to move in
  both directions.
- **Options:**
  - **A. Two antagonistic tendons** (two motors per DOF): full active control of
    both position and stiffness; ~2× motors, wiring, and mass.
  - **B. One tendon + passive return spring:** fewer motors; stiffness fixed by
    the spring, and the return direction is not actively driven.
- **Decision:** **Antagonistic tendon pairs are the baseline for joints whose
  stiffness must vary (all spine joints and the proximal leg joints), with
  co-contraction bias `T_bias` exposed as a first-class control input and an
  Antagonist Inhibition Control (AIC) rule (agonist gain `k`, antagonist held at
  `T_bias`) to keep peak tension down. Reserve Option B (single tendon + passive
  return spring) for distal, low-DOF joints (e.g. ankle) to save motors.**
  Grounded in Kengoro AIC, which cut peak tendon tension 43→28 kgf, and in the
  efficiency data showing spine stiffness should be *tunable* to gait speed
  ([LITERATURE_REVIEW.md](LITERATURE_REVIEW.md) Q1, Q2b).
- **Consequences:** The tendon map must accept a per-joint `T_bias` and implement
  the AIC split (M1 task K1). Stiffness becomes commandable rather than fixed by
  hardware.
  - **Amended by [ADR-0008](#adr-0008-actuator-sizing-basis-and-motor-count-the-mass-closure-decision):**
    the naive "~2 channels per antagonistic DOF" is **no longer affordable** — the
    mass budget forced the RoboCat **variable-radius pulley**, so one motor now
    drives *both* sides of a pair (**1 channel per DOF**, 16 total). The pair is
    still antagonistic and `T_bias` still applies, but co-contraction range is set
    by the pulley profile rather than commanded freely between two motors.

## ADR-0003: Actuator technology
- **Status:** Accepted
- **Context:** Need backdrivable, controllable rotary actuators, consistent with
  Principle **P1** (tendon-driven, limbs *and* spine).
- **Options:** BLDC + FOC (best backdrivability & control, most complex);
  geared DC (simpler, more friction/backlash); integrated servo modules
  (fastest to build, least tunable).
- **Decision:** **Tendon-driven actuation at every joint — legs and spine alike
  — honoring P1 (project-owner decision).** Motors are **BLDC + FOC**, chosen
  for backdrivability and high-bandwidth current control (needed for tension
  control; geared DC is ruled out because high gearing destroys backdrivability).
  - *This decision knowingly diverges from the leg-actuator trade study*
    ([leg-actuator-tradeoff.md](notes/leg-actuator-tradeoff.md)), which scored a
    backdrivable quasi-direct-drive (QDD) leg higher (4.35 vs. 3.45) — mainly
    because small leg moment arms drive very high cable tension. **P1 governs:**
    tendon-drive is the project's defining identity, and the trade study's
    numbers become the *engineering burden to manage*, not a reason to switch.
  - **Not fully sensorless:** a static high-torque hold cannot run reliably
    encoderless — baseline a rotor sensor per motor (ADR-0004) and evaluate a
    non-backdrivable reduction or **brake/latch** to offload the electrical DC
    hold, treated as a thermally-derated continuous-torque point
    ([sensorless-FOC note](notes/sensorless-foc-stance-hold.md)).
- **Consequences / burden accepted** (mitigation now spec'd — [LEG_TENDON_SPEC.md](../mechanical/LEG_TENDON_SPEC.md)):
  - **High cable tension** addressed by moment-arm sizing to (0.028, 0.025,
    0.014) m: this brings **continuous stand/trot tension into the ~20–70 N
    band (~55 N)**, leaving a **~500 N residual only in the ×2.5 single-leg land
    transient**. That residual is a **structural** design load (1.5 mm UHMWPE
    cable, SF ~4.5; bearing static rating), **not** a continuous fatigue/thermal
    duty — so the pure-tendon leg is buildable. Geometry alone cannot reach the
    band at the land transient (would need a ~200 mm pulley); the **stance brake**
    offloads the sustained hold.
  - **Tendon friction & stretch** are modeled (capstan + series compliance) in
    the tendon map (no longer a spine-only concern); per-joint routing wrap is a
    model TODO.
  - Per-tendon **tension sensing (ADR-0004) now applies to the legs too.**

## ADR-0004: Tension & position sensing method
- **Status:** Accepted
- **Options (tension):** in-line load cell per tendon (accurate, adds
  parts/space); motor current estimate (cheap, no extra parts, but
  friction-corrupted); series elastic element + displacement sensor (robust,
  adds compliance & size).
- **Decision:**
  - **Rotor position sensor on every motor is required — not sensorless.** A
    quasi-static high-torque hold cannot be run reliably encoderless: back-EMF
    sensorless loses observability below ~10–20 % of rated speed (no signal at
    standstill), and the only standstill-capable method (HF injection) needs a
    salient IPMSM and degrades under high load ([sensorless-FOC note](notes/sensorless-foc-stance-hold.md)).
    Baseline: **absolute encoder preferred, Hall sensors as the floor.**
  - Keep the **cable/joint-state sensor distinct from the rotor sensor** — they
    serve different loops (commutation/position vs. tendon coordination).
  - **Tension sensing: hybrid dual front-end** ([tension-sensing note](notes/tension-sensing.md)).
    (a) **Motor-current (`I_q`) estimate on every tendon** — always-on, kHz, no
    added parts — for the Tier-A over-tension/over-current latch (ADR-0005),
    slack/backdrive detection, and coarse feedforward. (b) **In-line load cell at
    the JOINT (output) end** on the stiffness-critical antagonistic joints —
    **spine joints + proximal leg joints (hip, knee)**; the spring-return ankle
    (ADR-0002) gets current-estimate only.
  - **Placement rule (the crux):** friction sits *between* motor and joint
    (`T_motor = T_joint·exp(±μ·θ_wrap)` — ~1.9× developing / ~0.5× releasing at
    the knee, μ≈0.10), so a girdle/motor-side sensor **cannot** read the tension
    the joint feels, nor regulate the ~20 N `T_bias`/AIC co-contraction stiffness
    (ADR-0002). MIT Cheetah's current-based force control does **not** transfer
    (it is direct-drive with no friction path). Any tension sensor that must know
    `T_joint` is placed at the **joint/output end**, downstream of the wrap.
  - **SEA rejected as the sensing baseline** — its compliance is redundant with
    ADR-0002 active co-contraction (and UHMWPE cable already gives some series
    give), at worse size/bandwidth for the same joint-end placement need.
- **Follow-up (blocks the error budget):** μ and per-tendon wrap are unmeasured
  placeholders — **bench-identify routed μ** (tomcat-mechanical/kinematics)
  before finalizing which joints truly need the load cell.

## ADR-0005: Compute & motor-drive topology
- **Status:** Accepted
- **Context:** **19 tendon motors** (12 leg DOF + 6 spine + 1 tail — was 16
  before [ADR-0009](#adr-0009-add-spine-lateral-bend--the-lateral-dof-static-stability-requires)
  added the lateral spine DOF; one per
  antagonistic pair via the variable-radius pulley, [ADR-0008](#adr-0008-actuator-sizing-basis-and-motor-count-the-mass-closure-decision);
  was ~31 before that decision) — each needing FOC, closed-loop tension, a rotor
  sensor, and a tension signal;
  plus ≥1 kHz motor loops (NFR3) and ≥100 Hz planning/righting (NFR4).
- **Options:** single MCU (rejected — cannot host ~30 FOC loops *and* planning);
  centralized multi-axis controller; **distributed smart drivers on a real-time
  bus** + a compute split.
- **Decision:** **Distributed FOC smart drivers — one per tendon motor —
  clustered in the shoulder & pelvic girdles and a **mid-body bay** (falls out of
  P1), on a CAN-FD real-time bus (~5 Mbit/s, split across ~6–8 segments), under a
  two-tier compute split.** A real-time controller (PREEMPT_RT core or dedicated
  MCU) owns the ≥1 kHz aggregation + tendon-map/AIC fast path + safety
  supervision; a separate SBC (ROS 2) runs ≥100 Hz planning/righting + host
  comms — hard control is never co-scheduled with planning. **Safety in three
  independent tiers:** (A) per-driver over-current/over-tension/thermal latches;
  (B) a **hardware e-stop** that cuts motor-bus power / forces zero-torque limp
  independent of SBC and RT controller; (C) an RT-supervisor watchdog. The SBC is
  never in the safety-critical path. Field-proven pattern (Kengoro's 116 per-muscle
  modules; mjbots/moteus & Mini-Cheetah per-joint FOC drivers) — see
  [compute-topology.md](notes/compute-topology.md). EtherCAT is the documented
  upgrade path if CAN-FD determinism becomes limiting.
- **Consequences:** electronics owns a per-motor FOC smart-driver board (rotor
  sensor + CAN-FD + ADR-0004 tension front-end), **three** backplanes, a
  multi-CAN-FD bridge, and the hardware e-stop; firmware owns driver FOC
  + tension + Tier-A safety, RT-tier aggregation + Tier-C watchdog, and SBC ROS 2
  nodes. ⚠️ The ~6–8 CAN-FD segment count is an extrapolation from the mjbots
  12-axis result — **bench-verify ≥1 kHz per segment** at final axis count.
  - **Cluster layout amended (ADR-0009 follow-up).** The earlier "two girdle
    backplanes + a tail stub" no longer matches where the motors physically are.
    The CAD packaging puts the whole spine+tail bank in a **mid-body bay**, so
    there are **three real nodes**, and the pelvis is no longer the big one:

    | Node | Motors | Contents |
    |---|---|---|
    | Shoulder girdle | 6 | fore-leg DOF |
    | Pelvic girdle | 6 | hind-leg DOF |
    | **Mid-body bay** | **7** | 3 spine dorsoventral + 3 spine lateral + 1 tail |

    The tail is a **stub off the mid-body node**, not its own node — one motor
    does not justify a backplane. This also matters for mass: the bay carries
    ~0.5 kg of actuation ~100 mm forward of the pelvis, which the old
    apportionment had charged to the rear girdle
    ([mass re-check](notes/mass-budget-recheck.md)).

## ADR-0006: Articulated tendon-driven spine (whole-body curvature)
- **Status:** Accepted
- **Context:** Principle P2 requires the body to curve like a real cat's
  (arching, lateral bend, righting-reflex twist). A rigid torso cannot do this.
- **Decision:** Model the torso as a serial chain of tendon-driven spine segments
  rather than a single rigid link. Long tendons run along the column from motors
  in the shoulder/pelvic girdles; antagonistic routing bends the chain (see
  ADR-0002). **Baseline sizing: seed the chain at 3 segments with up to 3 DOF
  each (dorsoventral pitch, lateral yaw, axial roll), i.e. a target of ~9 spine
  DOF, bracketing Laika's 6-DOF tensegrity spine and the bio-inspired prototypes
  ([LITERATURE_REVIEW.md](LITERATURE_REVIEW.md) Q2). The M1 kinematic model
  exercises the dorsoventral DOF in the sagittal plane first and leaves
  lateral/axial parameterized for later. Tensegrity remains the higher-ceiling
  research alternative, not the baseline.**
- **Consequences:** Significantly higher DOF and control coupling — many joints
  share tendons. The kinematic model must treat the body as a moving, curving
  base for the legs (whole-body kinematics), not a fixed frame. Adds a spine
  torque/tension budget alongside the leg budget.

## ADR-0007: Mid-air righting — spine + legs primary, coarse tail assist
- **Status:** Accepted (revised — tail simplified per project-owner)
- **Context:** Mid-air righting (landing feet-first) is an in-scope goal (G6).
  Reorientation conserves angular momentum via shape change; the question is
  which appendage provides it. **Design directive:** the tail does not need
  precise/accurate control — it is just a cable that **tensions up and loosens**.
- **Options:** (a) **spine axial twist** — cat-like, reuses the spine; (b)
  **leg/limb shape-change** — no new hardware, proven in sim for roll+pitch;
  (c) **precise inertial (morphable) tail** — highest authority but needs
  accurate multi-DOF control.
- **Decision:** **Righting authority is primary in the spine axial-twist DOF +
  leg shape-change** — rotary-actuator-only 180° reorientation is proven without
  reaction wheels/thrusters ([LITERATURE_REVIEW.md](LITERATURE_REVIEW.md) Q4).
  The **tail is a simple single-tendon appendage — tension to curl/raise, loosen
  to relax (passive return); no precision required** — providing only a coarse
  inertial assist, not controlled reorientation. This supersedes the earlier
  "precise morphable tail" decision: the literature's tail-is-best result assumed
  an *accurately controlled* tail, which we are deliberately not building.
- **Consequences:** Tail subsystem shrinks to **~1 motor + a passive return** (no
  telescoping, no accuracy budget) — cheaper, lighter, and P1-pure (cable pull).
  Righting control now lives in the **spine + legs**, so ADR-0006's axial-twist
  spine DOF becomes load-bearing for this goal (not merely complementary). The
  righting milestone plans a spine/leg reorientation law with the tail as a
  gross bias term.

## ADR-0008: Actuator sizing basis and motor count (the mass-closure decision)
- **Status:** Accepted
- **Context:** The [motor down-select](notes/motor-downselect.md) replaced the
  assumed ~31 g/motor with a real QDD module (~132 g). At the counts the
  architecture called for, **the motors alone exceeded the whole 3 kg body**
  (24 motors = 105 %, 31 = 136 %). Review finding F6. The design did not close.
- **Options considered:**
  - **A. Scale the robot up** to 6–9 kg so motors are a smaller fraction.
  - **B. Size actuators to a lesser load case** than the ×2.5 single-leg landing.
  - **C. Cut motor count** via the RoboCat variable-radius pulley (one motor per
    antagonistic pair) and/or more spring-return joints.
  - **D. Cut torque demand** with a thinner cable → smaller spool.
- **Decision: B + C. Keep the 3 kg body; size actuators to TROT; adopt the
  variable-radius pulley to run 16 motors.**
  > ⚠️ **Amended by [ADR-0009](#adr-0009-add-spine-lateral-bend--the-lateral-dof-static-stability-requires):**
  > 3D geometry later showed static stability requires a lateral DOF, taking the
  > count to **19 motors (45.6 % of body, 19.6 % left for structure)**. The
  > sizing basis (trot) and the variable-radius pulley are unchanged.
  - **Option A is rejected on physics, not preference.** At fixed geometry,
    body mass and required motor mass scale *together*, so the motor-mass
    **fraction is invariant with scale** (191 % at 1.5 kg, at 3 kg and at 9 kg
    alike); and under geometric scaling torque grows as `m^{4/3}` while mass
    grows as `m`, so scaling up is *actively worse*. A bigger robot does not buy
    its way out of this.
  - **Sizing basis = trot (2-leg, ×1.5).** This is the single biggest lever
    (24 motors: 191 % → 57 % of body). The ×2.5 single-leg landing is explicitly
    **outside the v1 actuator envelope**; it remains the *structural* design case
    for cable/pulley/bearing (unchanged), but not the *actuator* case.
  - **16 motors** = 12 leg DOF + 3 spine + 1 tail, one per antagonistic pair via
    the variable-radius pulley. **Full articulation is retained** — this is a
    change of transmission, not of DOF.
  - **Resulting motor spec: ~1.1 N·m peak, ≤80 g, ~24 V.** The surveyed
    GIM3505-9 (1.95 N·m, 132 g) is ~1.8× larger — a headroom option, not the
    target part.
- **Consequences:**
  - ⚠️ **Amended by [ADR-0010](#adr-0010-mass-target-30--405-kg-and-the-walk-is-limited-by-tipping-not-friction):**
    the ~72 g motor this closure rests on **does not exist**. The lightest real
    part meeting the torque is ~120 g (131.7 g with driver), and NFR5 rose to
    **4.05 kg**. The *sizing basis* below (trot, not landing) still governs and is
    unchanged; only the mass arithmetic moved.
  - Budget closes at 3.0 kg: motors 38 %, drivers 3 %, legs 14 %, battery 10 %,
    leaving **~35 % for structure**.
  - **Hard landings are out of scope for v1**, which touches ADR-0007 (righting
    is a *flight-phase* behaviour; the landing that follows it is now
    envelope-limited). Revisit if righting becomes a near-term goal.
  - The variable-radius pulley moves from "nice reduction" to **required**, which
    firms up an ADR-0002 option that was previously optional. Independent
    stiffness control is preserved (the pair still opposes), but both sides of a
    pair now share one motor, so co-contraction range is set by the pulley
    profile rather than commanded freely.
  - Motor count drops 24 → 16, which **invalidates the F4 girdle packing** (done
    for 24) — it should re-pack easily with fewer, though the real motor envelope
    differs from the Ø16 × 28 mm placeholder and must be re-checked.
  - ⚠️ Torque density (14.8 N·m/kg peak) is extrapolated from one datasheet; the
    whole closure rides on it. Confirm against a real candidate before committing.

## ADR-0009: Add spine LATERAL bend — the lateral DOF static stability requires
- **Status:** Accepted
- **Context:** 3D geometry (real lateral foot positions) exposed that the walk is
  **not statically stable** — the true support polygon gives a worst margin of
  **−28.7 mm** where the 2D sagittal interval had claimed +24.6 mm
  ([review F7](../mechanical/DESIGN_REVIEW.md)). With three feet down the support
  triangle is skewed while the CoM sits mid-sagittal, so it falls outside for
  about half the cycle. Recovering static stability needs **lateral body sway**,
  which the 16-motor sagittal-only build cannot produce.
- **Options considered** (all measured, see F7):
  - **A. Re-order the leg sequence** — all 24 permutations tried, best −22.7 mm.
    **Does not work.**
  - **B. Widen the track** — 96 → 260 mm makes it **worse** (−47.7 mm): the
    critical edge is the far-front-to-near-rear *diagonal*, and widening only
    rotates it further from the CoM. **Does not work.**
  - **C. Forward mass bias alone** — best −8.3 mm at +40 mm. **Not sufficient**,
    and a +30 mm shift is hard to realise anyway (moving the battery front-ward
    buys ~10 mm, moving the spine/tail motor bank ~10 mm).
  - **D. Leg abduction** (+4 motors) — 1:1 sway authority, but puts mass in the
    LIMBS, fighting P1's low-limb-inertia rationale, and leaves only 17 % for
    structure.
  - **E. Accept dynamic walking** (0 motors) — drop static stability, as many
    quadrupeds do. Cheapest in hardware, most expensive in control.
  - **F. Spine lateral bend** (+3 motors).
- **Decision: F — add one LATERAL bend DOF per spine segment (16 → 19 motors).**
  - **This is not scope creep; it is finally implementing P2.** Principle P2 says
    *all curvature of the body is allowed*, and [ADR-0006](#adr-0006-articulated-tendon-driven-spine-whole-body-curvature)
    already specifies lateral yaw as one of the ~3 DOF per segment. What we built
    was the sagittal subset. This closes the gap the founding principle assumed.
  - **Authority is ample and was checked, not assumed.** At ADR-0006's specified
    ±15°/segment lateral ROM the CoM sways **42.8 mm** — more than the ~40 mm
    needed by sway alone; even ±10°/segment gives 29.4 mm.
  - **Static stability is preserved**, so the controller stays quasi-static. That
    matters because the dynamics milestone (M5) does not exist yet — option E
    would move the entire burden onto an unbuilt controller.
  - **The motors are dual-use**: the same lateral/axial spine authority serves
    ADR-0007 righting, so they are not single-purpose mass.
- **Consequences:**
  - **Motor count 16 → 19**, amending [ADR-0008](#adr-0008-actuator-sizing-basis-and-motor-count-the-mass-closure-decision).
    Motors go 38.4 % → **45.6 %** of body; structure margin falls
    **27.3 % → 19.6 % (587 g)**. It still closes, but ⚠️ **tightly** — the
    structure budget should be re-checked against real printed-part masses
    before this is treated as settled.
  - `SpineModel` must gain a lateral DOF (the model is sagittal-only today), and
    with it a genuinely 3D `WholeBody`. That is the next milestone, not a patch.
  - Forward mass bias is retained as a **free secondary trim** — it reduces the
    lateral ROM the gait must command, buying margin.
  - NFR2c total actuated DOF **16 → 19**.
- **Implementation outcome (M5) — decision upheld, two claims corrected.**
  The lateral DOF is now built (`SpineModel.lateral_vertebra_xy`,
  `WholeBody.center_of_mass_y`, `GaitController.lateral_q`) and the default walk
  measures **+10.1 mm** polygon margin, up from **−21.6 mm** with sway disabled.
  Building it changed four things the ADR had not anticipated:
  1. ⚠️ **"Authority is ample" was too optimistic.** The 42.8 mm sway figure is
     right, but sway is not a monotonic good: margin peaks at **12.5°/segment
     (+10.1 mm)** and then *falls* — over-swaying carries the CoM out over the
     **far** edge of the triangle (at 18° the margin is negative again). The ±15°
     ROM is therefore **adequate with ~2.5° to spare, not ample**. Optimum sway
     and track width are coupled (a ±55 mm track would want 16.5°, beyond the
     ROM), so the existing ±48 mm track is well matched and should not be widened
     without also widening the ROM.
  2. **Duty factor had to rise 0.75 → 0.80** (a gait change, not a hardware one).
     At 0.75 the swing windows tile exactly, so the support side flips
     discontinuously and the sway would have to be instantaneous — a swept study
     found *any* finite ramp, even 2 % of the cycle, collapses the margin right
     back to the no-sway value. Duty must exceed 0.75 to open **four-foot
     windows** (each `duty − 0.75` of the cycle) for the spine to cross over in.
     The shipped default is **0.90**, not the minimum-viable 0.80 — see (4).
  3. **The sway law must be a ramped square wave**, not a sinusoid. A sinusoid is
     *worse than no sway at all* at every phase lead tested, because it is near
     zero exactly at the crossovers, which is where the margin is decided.
  4. ⚠️ **The binding constraint turned out to be FRICTION, not geometry — and it
     sets the walk speed.** The sway must reverse a ~78 mm CoM traverse inside
     each four-foot window, costing `a = 4d/w²` — the *inverse square* of the
     window, so speed is punished hard. A paw delivers only `μg ≈ 7.8 m/s²`
     laterally before it slides. The gait the sway was first tuned on (1.2 s,
     duty 0.80, 60 ms window) demanded **9.1 g / 268 N** and was **not physically
     realisable**, even though its quasi-static margin looked fine. The default
     was therefore retuned to **period 1.4 s, duty 0.90** → a 210 ms window,
     6.87 m/s² demanded vs 7.85 available. It closes with only **14 % margin**
     and needs **μ ≥ 0.70**. `GaitController.crossover_accel()` /
     `.crossover_is_feasible()` make this checkable.
  - ⚠️ **SUPERSEDED by [ADR-0010](#adr-0010-mass-target-30--405-kg-and-the-walk-is-limited-by-tipping-not-friction).**
    Item (4) above and the μ ≥ 0.70 requirement are **wrong**. Resolving the
    per-foot ground-reaction forces (M6 dynamics) shows friction was never the
    binding constraint — the body-level demand at this very gait is only μ ≈ 0.35.
    What actually fails is **tipping**: the ZMP leaves the support polygon by
    ~128 mm. The walk speed cap is **1.1 cm/s**, not 4 cm/s, and the sway law
    below needed a C¹ fix before it was even physically realisable.
  - ~~**Consequence — static stability caps this walk at a crawl:** ~**4 cm/s**.~~
     That is inherent, not a tuning failure: a statically stable gait must stop
     and shift its weight between steps. Anything faster must be **dynamic**,
     which is what ADR-0008 already sized the motors for (trot). Option E in this
     ADR is thus not avoided, only deferred.
  - **New requirement on the spine drives:** a lateral slew of **≈119 °/s per
    segment** at the shipped default (`GaitController.lateral_slew_rate()`).
    Modest — the *speed* was never the problem, the *acceleration of the body*
    was. Motor side: ~0.07 m/s of cable, ~80 rpm at an 8 mm spool.
  - Option **A is now retired for good**: with sway commanded, all 24 sequence
    permutations land within **2 mm** of each other and all are stable. Sequencing
    changes *when* postures occur, not *which* — it was never the lever.
  - ⚠️ The +10.1 mm is a **static** margin with no dynamic allowance, and it is
    small. It does not survive contact with inertia, and nothing here models that.
- **Follow-up (done): the lateral DRIVE was sized, and the mass model corrected.**
  ADR-0009 added three motors without checking what they must actually pull.
  - **The lateral load is INERTIAL, not gravitational.** The lateral bend axis is
    vertical, so gravity exerts no moment about it — *holding* a sway is nearly
    free. What costs torque is *reversing* it during the crossover
    (`WholeBody.lateral_spine_loads`). The **base** joint is worst, swinging the
    whole forequarters: **2.21 N·m**, 110 N of cable, **0.88 N·m at the motor
    shaft = 0.80×** ADR-0008's 1.10 N·m trot sizing point. **The +3 motors are
    the same class as the leg motors**, so ADR-0009's mass arithmetic holds.
  - ⚠️ **But only because the lateral moment arm was raised 15 → 20 mm.**
    SPINE_TAIL_SPEC §1.5 assumed the bare transverse-process width (15 mm); at
    that arm the base joint needs **1.13 N·m — over the motor's peak**. 20 mm is
    bought with a milled lateral pulley post per vertebra, the same trick
    ASSEMBLY_SPEC already uses for the 30 mm dorsoventral arm. ±20 mm fits inside
    the ±34 mm rib cavity. **This post is load-bearing, not detail.**
  - ⚠️ **The mass model was wrong by ~347 g of actuation** — it charged 31
    *channels* (a pre-variable-radius-pulley count) at a *31 g* motor, while the
    build is 19 motors at the down-selected *72 g*; the two errors had opposite
    signs and hid each other. It also parked the spine/tail bank in the rear
    girdle when the CAD packs it mid-body. Corrected in `params.py`; the pelvis
    is now the *lighter* girdle and the fore/hind split moved 51/49 → **55/45**.
    Net effect on this ADR is **favourable** — margin +8.4 → **+10.1 mm**,
    friction margin 11 % → **14 %** — but review finding F2's "quiet stand barely
    loads the base joint" is partly walked back (0.13 → 0.29 N·m). Full accounting
    in [notes/mass-budget-recheck.md](notes/mass-budget-recheck.md).
  - **ADR-0009's ⚠️ on the structure budget is downgraded, not closed.** The first
    estimate from real CAD geometry gives **~296 g of printed structure against a
    587 g allowance (~2× headroom)** — but that is a massing model with solid
    bones, so it is directional only.

## ADR-0010: Mass target 3.0 → 4.05 kg, and the walk is limited by TIPPING not friction
- **Status:** Accepted
- **Context:** Two long-standing ❓ items were closed at once, and each overturned
  a published conclusion.
  1. **The motor.** ADR-0008's mass closure rode on a **~72 g** motor at
     ~1.1 N·m. A survey of the real market
     ([motor-reality-check](notes/motor-reality-check.md)) finds the lightest
     purchasable part meeting that torque is **~120 g** (131.7 g with driver).
     Torque *density* was not the bad assumption — the real part beats the
     implied 15.3 N·m/kg — but **motors come in discrete sizes**, and the
     smallest one clearing the bar is ~2× the capability needed.
  2. **The dynamics.** M4 and M5 were quasi-static: they asked only whether the
     CoM projects inside the feet. The new `kinematics/dynamics.py` asks whether
     the **contacts can produce the forces the motion requires** (per-foot
     ground-reaction solve, friction cones, ZMP).
- **Decision A: raise NFR5 to 4.05 kg.**
  - 19 × 131.7 g of actuation alone is 2.5 kg; with legs, battery, head/neck and
    structure the body lands at **4.04 kg**. 3.0 kg is not achievable with real
    hardware at 19 motors.
  - This is **more biomimetic, not less** — a domestic cat is 4–5 kg, and 3.0 kg
    was a placeholder never derived from the animal.
  - It **converges rather than spiralling**: the heavier body needs 1.48 N·m at
    the trot hip against the part's 1.95 N·m peak, so there is still 1.3× headroom.
  - ~~⚠️ **Sustained trot is thermally limited**: 1.48 N·m against a 0.71 N·m
    *continuous* rating is a 2.1× overload.~~ **CORRECTED by ADR-0011.** That
    compared a quasi-static *peak* against a *continuous* rating, which is the
    wrong comparison — thermal limits are set by **RMS** over the cycle. Resolving
    the real trot gives peak 1.26 N·m (0.65× the part's peak) and **RMS 0.40 N·m
    = 0.56× the continuous rating**. Sustained trot is thermally FINE up to
    ~96 cm/s. The stance brake keeps its value for static holds, not for trot.
- **Decision B: retune the walk to period 5.0 s / sway 11°, and record that the
  binding constraint is TIPPING.**
  - ⚠️ **M5's headline was wrong.** M5 concluded *"friction sets the walk speed,
    needs μ ≥ 0.70"*. Resolving the per-foot forces shows the body-level friction
    demand at M5's own 1.4 s gait is only **μ ≈ 0.35** — it would never have
    slipped. What fails is the **ZMP leaving the support polygon**: accelerating
    the CoM sideways to produce the sway shifts the effective pressure point by
    `(h/g)·a` the *other* way, ~**128 mm** against a 96 mm track. **Slipping never becomes the binding
    constraint at all** — swept over periods 0.6–6.0 s the aggregate friction
    demand never reaches even μ = 0.8, while tipping only clears at **3.8 s**.
  - ⚠️ **M5's sway law was not physically realisable.** Its crossover ramp was
    *linear in position*, so velocity **stepped** at each end — an impulse in
    acceleration, i.e. infinite force. No static check could have seen this,
    because a static check never differentiates the trajectory. Replaced with a
    **raised-cosine (C¹)** ramp, which costs 23 % more peak acceleration
    (`π²/2 · d/w²` rather than `4 d/w²`) but is finite.
  - Consequence: the statically stable walk is a **1.1 cm/s crawl**, not 4 cm/s.
    Both margins are small — **+6.5 mm static, +6.4 mm ZMP**.
- **Consequences:**
  - Every mass-derived number rescaled: load cases, girdle masses, spine loads.
    Fore/hind split 55/45 → **54.2/45.8**.
  - ⚠️ **The cable had to be re-sized.** Tendon loads scale with body mass, so the
    hip land transient went **465 → 600 N**, dropping the 1.5 mm UHMWPE to
    **SF 3.67 — below LEG_TENDON_SPEC's own ≥ 4 target**. Upsized to **1.75 mm**
    (SF ≈ 5.0), which forces the spool radius **8.0 → 8.75 mm** and so costs ~9 %
    more motor torque (trot hip 1.47 → 1.61 N·m, still 0.82× the real part's
    peak). 2.0 mm was rejected: it would trade a cable margin for a *tighter*
    actuator margin (0.94× peak).
  - ⚠️ **The girdles grew.** The real motor is Ø34.5 × **36.1 mm** — nearly the
    same diameter as the placeholder but **39 % longer**, and the banks stack
    vertically, so girdle height goes **88 → 108 mm**. Width (86 mm) still clears
    the 96 mm track, so nothing collides.
  - **Paw friction is no longer a stability requirement.** NFR2g (μ ≥ 0.70) is
    withdrawn: the resolved aggregate demand is **μ ≈ 0.055** at the shipped gait.
    Published TPU/PU-on-concrete values are 0.8–1.2, so this was never close.
  - The lateral spine drive is now **trivially loaded by the crawl** (0.10 N·m),
    so it must be sized against a *faster* reference manoeuvre instead — the
    ADR-0007 righting reflex and any future dynamic gait. The 20 mm lateral post
    (ADR-0009 follow-up) is consequently **an optimisation, not a necessity**:
    against the real part's 1.95 N·m peak, even the bare 15 mm transverse process
    would fit.
  - ⚠️ **The case for a dynamic gait is now decisive, not preferential.** A 1.1 cm/s
    crawl is not cat-like by any measure. Static stability is a demonstration
    mode; the operating mode has to be dynamic.
  - Remaining modelling debt: `dH/dt = 0` (classical ZMP form). Quantified rather
    than merely declared — the swing leg's neglected spin is worth **~1.0 mm** of
    ZMP shift against the **6.4 mm** margin, a ~6× ratio, so the result holds *at
    this crawl speed*. ⚠️ It scales with leg acceleration and would **not** hold
    for a fast or dynamic gait, which needs full rigid-body dynamics.

## ADR-0011: The trot works — a dynamic gait reaches cat-like speed within the actuator envelope
- **Status:** Accepted
- **Context:** [ADR-0010](#adr-0010-mass-target-30--405-kg-and-the-walk-is-limited-by-tipping-not-friction)
  ended with the statically stable crawl capped at **1.1 cm/s** and concluded a
  dynamic gait was no longer optional. This is that gait: a diagonal **trot**,
  evaluated with the M6 dynamics.
- **Decision: adopt a diagonal trot as the locomotion mode**
  (`gait.trot_params()`), and accept that its stability is *dynamic*, not static.
  - Diagonal pairs (LF+RR, then RF+LR) at duty 0.50. The support degenerates from
    a polygon to a **LINE**, so `support_polygon` / `zero_moment_point` correctly
    **refuse to evaluate** — a line has no interior. The governing physics is the
    inverted pendulum about that line (`dynamics.line_balance`).
  - **Default ~67 cm/s.** Feasible and thermally sustainable to **~96 cm/s**; at
    ~120 cm/s the RMS motor torque reaches the continuous rating. A domestic cat
    trots at roughly 1 m/s, so this is genuinely cat-like — **60× the crawl.**
- **What the trot required, and what it revealed:**
  1. **Foot placement is a balance condition, not a comfort setting.** Two
     contacts cannot produce a moment about the line joining them, so the CoM's
     perpendicular offset from that line is an *unbalanceable* topple moment. The
     crawl plants feet 50 mm ahead of the hips, putting the diagonal ~42 mm
     forward of the CoM: a one-signed moment, and the roll rate then accumulates
     **−4.1 rad/s every cycle — the robot falls over in one stride.** Moving the
     nominal foot to **x = 0.005 m** makes the CoM rock symmetrically ±11 mm
     *through* the line, the moment integrates to ~zero, and the roll becomes a
     **bounded ±0.4°** oscillation. That rocking *is* the gait.
  2. ⚠️ **A second C¹ defect, in the M2 foot trajectory itself.** The cycloidal
     swing starts and ends at ZERO hip-frame velocity while stance sweeps at
     `-stride/(duty·period)` — so the foot velocity **steps** at both liftoff and
     touchdown. Two consequences: swing-leg torque is impulsive (it *doubles with
     every grid refinement*, so it is not a number at all), and the paw lands
     moving forward at the full stance speed — a **scuff** on every step. Replaced
     with a velocity-**matched** quintic (`GaitParams.swing_profile`), which lands
     the foot at zero velocity relative to the ground. This is the same class of
     error as the M5 sway law, found the same way: by differentiating a trajectory
     that only static checks had ever looked at.
  3. **P1 is vindicated quantitatively.** Swing-leg torque is what caps trot
     speed, and it is only **0.11 N·m** at 67 cm/s — because tendon drive keeps
     the legs at 95–110 g. A conventional leg with motors at the joints would pay
     this many times over. This is the first hard number supporting the founding
     principle rather than merely restating it.
- **Consequences:**
  - `swing_profile="matched"` is the **default** for all gaits. The crawl is
    unaffected (its margins move by 0.04 mm) — at 5 s the defect was invisible,
    which is exactly why it survived five milestones.
  - Contact-force residual is now a **meaningful output**, not an error: with two
    contacts it is the physically unbalanceable moment.
  - The **critical joint at trot is the hock**, not the hip. The quasi-static
    sizing case identified the hip because the *land* transient drives it.
  - ⚠️ Still unmodelled: touchdown **impact** (the matched profile removes the
    tangential scuff but not the vertical impulse), contact compliance, and per-link
    **spin** inertia — `swing_joint_torque` treats links as point masses, so it
    under-estimates. Flight phases (duty < 0.5) are generated but not analysed.

## ADR-0012: Tactile sensing at the paw — a barometric dome, capped at 20 g
- **Status:** Accepted
- **Context:** [ADR-0011](#adr-0011-the-trot-works--a-dynamic-gait-reaches-cat-like-speed-within-the-actuator-envelope)
  left closed-loop balance as the next milestone, and it is an *open-loop*
  trajectory check today: it shows the prescribed motion is dynamically
  consistent, not that a controller can stabilise it. Closing that loop needs
  measurement at the contact. The project had reserved a *"tactile sensor
  pocket"* in the paw pad since the assembly spec, but never specified a sensor.
- **Decision: a MEMS barometer under a sealed, moulded TPU dome in each paw,
  ≤ 20 g per paw**, reporting normal force and a hardware contact flag at ≥1 kHz;
  tangential force comes from the existing ADR-0004 joint-end load cells and is
  **fused** with the paw's normal. Full spec:
  [TACTILE_SENSING_SPEC](../mechanical/TACTILE_SENSING_SPEC.md).
- **Why not rely on the sensing we already own** (this was the real question —
  ADR-0004 already buys joint-end load cells, and a point contact exerts no
  moment, so `tau = J^T F` is solvable from two clean torque measurements):
  1. **The inversion is badly conditioned on the hind legs.**
     `dynamics.grf_observability` measures the error amplification: **3.2–3.4 on
     the fore legs but a median 7.5 and worst 36 on the hind legs**, degrading
     sharply *just before liftoff* — exactly when load transfers to the other
     diagonal. A 1 % tension error becomes a 36 % force error there.
  2. **Joint torque cannot detect contact at all.** It cannot distinguish
     "pressing on the ground" from "limb accelerating" without a dynamics model in
     the loop. For the single measurement closed-loop balance most depends on —
     *when did the foot actually land* — the joint route is not imprecise but
     **categorically unable**. Only a sensor at the contact can answer it.
- **Why ≤ 20 g, and why that is the binding constraint:** a paw sensor sits at the
  most distal point of the limb — precisely what tendon drive exists to keep
  light. M7 showed swing-leg torque caps trot speed, so the trade is measurable
  rather than rhetorical: **20 g/paw costs ~41 % of the swing term and drops the
  sustainable top speed 120 → 96 cm/s; 40 g/paw pushes the worst motor past its
  continuous rating even at 96 cm/s**, i.e. the sensor starts taking away usable
  gait. The 4 × 20 g of *mass* is ~2 % of the 4.05 kg budget and irrelevant — it
  is **swing inertia**, not mass, that limits this. P1, quantified.
- **Consequences:**
  - Requirements derived from the robot's own dynamics, not a catalogue: **0–35 N**
    measurement, **≥100 N survival** (the ×2.5 land transient), **≤0.4 N**
    resolution, **≥1 kHz** (trot stance is 150 ms).
  - No new bus — the paw joins its limb's existing CAN-FD node (ADR-0005).
  - **FR5** (detect and recover from a foot slip) becomes achievable; it has been
    a "Should" with no sensing behind it since M1.
  - Shin, body-shell and whisker sensing are **deferred** — recorded so the
    structure carries provision rather than being retrofitted, but none is on the
    balance critical path.
  - ⚠️ Owed: dome geometry (the range is set by cavity volume, and needs FEA or a
    moulding trial), a **drop test** for the real touchdown impulse, barometer part
    selection against the 100 N overpressure case, TPU **creep** under the crawl's
    4.5 s sustained load, and the fusion estimator itself.

## ADR-0013: Closed-loop balance by step-to-step foot placement
- **Status:** Accepted
- **Context:** [ADR-0011](#adr-0011-the-trot-works--a-dynamic-gait-reaches-cat-like-speed-within-the-actuator-envelope)
  closed with an explicit caveat: the trot was an **open-loop** check, showing the
  prescribed motion is dynamically consistent -- i.e. that the error *starts* at
  zero, not that it stays there. The trot is an inverted pendulum about its
  diagonal support line, so on the shipped gait `omega*T = 1.17` and any deviation
  is multiplied by **3.2 every step, 339 over five**. Nothing in the model arrested
  it.
- **Decision: stabilise by choosing where to put the next foot, not by tracking
  the trajectory harder** (`kinematics/src/tomcat_kin/control.py`). Working in the
  **DCM** (divergent component of motion, `xi = c + c_dot/omega`) collapses the
  problem to first order -- `xi_dot = omega(xi - p)` -- so the entire control law
  is one placement per touchdown:

  > `p = nominal + (growth - beta)/(growth - 1) * (xi - nominal)`

  `beta` is the residual error per step: 0 gives one-step deadbeat, 0.5 halves it
  each step for a smaller foothold excursion.
- ⚠️ **The coefficient is greater than one (1.45 here) -- the foot goes BEYOND
  the DCM, not under it.** Placing it *at* the DCM (`capture_placement`) arrests
  the topple but leaves the body permanently displaced: the robot is perfectly
  stable and walks away sideways. This is kept as a separate function because it
  is a quiet, plausible-looking error -- it was made in the first draft of this
  module and only simulating it caught the difference.
- **What actually limits it -- REACH, not gain:**

  | | value |
  |---|---|
  | One-step envelope (before the placement saturates) | ~~51 mm~~ **22 mm** (beta=0) |
  | **Guaranteed rejection envelope** | ~~74 mm~~ **33 mm** (feet alone) |
  | Binding direction | **rearward** -- the leg reaches +153 mm forward but only -74 mm back |

  ⚠️ **CORRECTED by [ADR-0014](#adr-0014-the-lateral-spine-is-the-trots-main-balance-actuator).**
  The figures above used the leg's raw FORE-AFT range, but the DCM lives
  *perpendicular* to the diagonal and that direction is ~90 % **lateral**. With
  sagittal-only legs a fore-aft foothold shift buys perpendicular authority only
  through its **0.44 projection**, so the true feet-alone envelope is **33 mm**,
  not 74. ADR-0014 then recovers it to 68 mm using the spine.

  The full envelope is **independent of `beta`**: gain sets how *fast* recovery
  happens, reach sets whether it happens at all. (An earlier version of the
  measurement ran only 12 steps and made a slow gain look like a *smaller*
  envelope -- that was horizon-limited, a different and misleading statement.)
- **A hard requirement lands back on [ADR-0012](#adr-0012-tactile-sensing-at-the-paw--a-barometric-dome-capped-at-20-g):**
  a steady bias in the *estimated* DCM does not average out. It settles into a
  **permanent lateral offset, amplified by exactly the per-step growth (3.2x)** --
  a 5 mm estimation error becomes a 16 mm drift. Paw contact timing and the state
  estimator must therefore be good to a few millimetres of equivalent DCM, which
  is a far sharper specification than "detect contact".
- **Consequences:**
  - The asymmetric leg reach is now a **balance** parameter, not just a workspace
    one. Anything that trims rearward reach directly cuts the disturbance envelope.
  - ⚠️ **Reduced-order model.** One perpendicular coordinate, constant CoM
    height, point feet, instantaneous support transfer. It is a controller-design
    tool, not a replacement for `dynamics.py`; its `omega` and stance are taken
    *from* the full model. It is also **not** a simulation of the robot: no swing
    dynamics, no actuator limits in the loop, no double support.
  - ⚠️ Still open: **sensing latency** (only a static bias is modelled), step
    retiming (the controller places feet but does not retime them), and the fact
    that a real disturbance perturbs the full 3D state, not one scalar.

## ADR-0014: The lateral spine is the trot's main balance actuator
- **Status:** Accepted
- **Context:** [ADR-0013](#adr-0013-closed-loop-balance-by-step-to-step-foot-placement)
  put the trot's disturbance envelope at 74 mm and named two gaps: sensing
  **latency** and step **retiming**. Chasing them turned up a mistake in the
  envelope itself, and then a much better actuator than the one being tuned.
- ⚠️ **First, a correction.** The DCM is measured **perpendicular to the
  diagonal support line**, and that direction is ~**90 % lateral**. ADR-0013 used
  the leg's raw *fore-aft* reach as placement authority. But the legs are
  **sagittal-only** -- abduction was rejected in ADR-0009 and the track is fixed
  at +/-48 mm -- so a fore-aft foothold shift only buys perpendicular authority
  through its **0.44 projection**. The envelope was therefore overstated by
  **2.3x**: it is **33 mm**, not 74 mm (one-step: 22 mm, not 51 mm).
- **Decision: use the ADR-0009 LATERAL SPINE as a balance actuator during the
  trot.** It moves the CoM rather than the support line, so it *adds* to foot
  placement instead of competing with it -- and it pushes almost exactly along the
  perpendicular, which is precisely where the sagittal legs are weakest.

  | actuator | perpendicular authority | note |
  |---|---|---|
  | Foot placement | **33 mm** | 0.44 projection of a generous fore-aft range |
  | **Lateral spine** | ~~24 mm~~ **39 mm** | full +/-15 deg ROM -- see the correction below |
  | **Combined** | ~~68 mm~~ **90 mm** | **+175 %** over feet alone |

  ⚠️ **CORRECTED by [ADR-0015](#adr-0015-both-m9-follow-ups-close-no-change-needed).**
  The 24 mm figure clamped the spine with **NFR2f's 119 deg/s**, which is a
  *requirement floor* sized for the righting reflex, not a capability. The drive
  actually manages ~**912 deg/s** at the joint, while a full-ROM traverse inside a
  stance needs only 200 deg/s. The spine is **ROM-limited, not rate-limited**, so
  the full +/-15 deg is available: **39 mm**, and the envelope is **90 mm**.

  The spine was bought for the CRAWL's *static* stability (ADR-0009, 16 -> 19
  motors) and the trot preset had it switched off. It turns out to be the
  dominant **dynamic** balance actuator. That is a real dividend from a decision
  taken for an unrelated reason, and it is the strongest argument yet for the
  articulated spine as more than a biomimetic flourish.
- **Latency: costs the envelope roughly linearly, no cliff.** Handled by
  predicting the DCM forward -- prediction itself is not the problem. What remains
  is that estimation error is amplified by `e^(omega*tau)` and less time is left
  under the corrective placement:

  | latency | envelope (with spine) |
  |---|---|
  | 0 ms | 68.0 mm |
  | 10 ms | 61.8 mm (-9 %) |
  | **20 ms** | **56.1 mm (-18 %)** |
  | 40 ms | 45.9 mm (-32 %) |

  **Budget 20 ms** end-to-end (contact detect + estimate + bus + swing tracking).
  NFR3's >=1 kHz loop makes the compute path nearly free; the risk is filtering
  and swing tracking.
- **Retiming: speeds recovery, does NOT extend the envelope.** With the placement
  saturated at reach `R`, `xi_end = R + (e-R)e^(omega*T)`. For `e < R` the bracket
  is negative so a *longer* stance amplifies the correction -- one-step recovery
  becomes possible. For `e > R` it is positive and grows for any `T`; as `T -> 0`
  it merely holds. **Timing buys speed, never range.** A useful negative result: it
  removes retiming from the critical path.
- **Consequences:**
  - `trot_params()` still ships with `lateral_amplitude = 0` -- the spine is used
    for *balance*, not for a nominal sway, so it stays centred until disturbed.
    The gait definition does not change; the controller gains an input.
  - **Leg abduction is worth revisiting.** ADR-0009 rejected it on mass grounds
    when the question was static stability. The dynamic case is different: abduction
    would point straight down the perpendicular. Not reopened here, but the reason
    it was rejected no longer covers this use.
  - ⚠️ The spine figure is rate-limited by **NFR2f (119 deg/s)**, which was
    sized for the righting reflex, not for balance. A faster spine drive would buy
    envelope directly -- ~39 mm if the full +/-15 deg were reachable within a stance.
  - ⚠️ Unchanged reduced-order caveats from ADR-0013, plus: the spine is
    modelled as an instantaneous bounded CoM offset, ignoring its own dynamics and
    the reaction torque it puts into the trunk.

## ADR-0015: Both M9 follow-ups close -- no change needed
- **Status:** Accepted
- **Context:** [ADR-0014](#adr-0014-the-lateral-spine-is-the-trots-main-balance-actuator)
  left two open questions: **(a)** revisit leg **abduction**, since ADR-0009
  rejected it against a *static*-stability requirement that no longer applies, and
  **(b)** consider a **faster spine drive**, since NFR2f was sized for righting
  rather than balance. Both were expected to cost motors. Neither does.
- ⚠️ **First, a correction to ADR-0014.** Its spine figure clamped the
  authority with **NFR2f's 119 deg/s**. That is a *requirement floor* set by the
  ADR-0007 righting reflex, not a capability. The real joint rate is
  `380 rpm x (8 mm spool / 20 mm arm)` = **~912 deg/s**, while traversing the full
  +/-15 deg ROM inside a 150 ms stance needs only **200 deg/s**. The spine is
  **ROM-limited, not rate-limited**, and ADR-0014 under-counted it by ~40 %:

  | | ADR-0014 | corrected |
  |---|---|---|
  | Spine authority | 24 mm | **39 mm** |
  | Combined envelope | 68 mm | **90 mm** |

- **(b) A faster spine drive is NOT needed** -- the capability is already **8x**
  the requirement, and 4.6x what a full-ROM traverse needs. NFR2f stands as a
  righting spec; balance is not asking for more.
- **(a) Leg abduction is NOT needed either.** The corrected envelope of **90 mm**
  corresponds, via `xi = c + c_dot/omega`, to rejecting a **0.70 m/s lateral
  shove** -- a substantial disturbance. Abduction would add direct lateral
  placement (authority = travel x 0.897, so ~40 mm at +/-45 mm of foot travel) at
  a cost of **+4 motors = 528 g, 13 % of the 4.05 kg budget**. That buys authority
  the robot does not currently need.
  - ADR-0009's *original* objection ("puts mass in the LIMBS") was in fact weak
    for a tendon-driven design -- the motor would sit in the girdle. But the
    conclusion survives on the stronger ground that **the requirement is already
    met**, so the mass is simply unspent.
- **Widening the spine ROM is available but deliberately NOT taken.** It scales
  well (+/-25 deg would give 119 mm) and costs no motors -- but **lateral is the
  STIFFEST spine axis**: SPINE_TAIL_SPEC ranks compliance
  *axial > dorsoventral > lateral*, so +/-15 deg is already the narrowest axis by
  design. Widening it fights the biomechanics the spine geometry came from. Held
  at +/-15 deg unless a future requirement demands otherwise; recorded here so the
  lever is known rather than rediscovered.
- **Consequences:**
  - Motor count stays at **19**. Mass stays at **4.05 kg**. No spec changes.
  - NFR10 rises **68 -> 90 mm**; a new NFR records the equivalent **0.70 m/s**
    shove, which is a more meaningful acceptance criterion than a DCM figure.
  - ⚠️ The 0.70 m/s figure inherits every reduced-order caveat from
    ADR-0013/0014, and assumes the spine's full ROM is free for balance. In the
    trot it is (`lateral_amplitude = 0`); if a future gait uses lateral sway for
    something else, the balance authority is spent and this closes differently.

## ADR-0016: Latency is not an independent parameter -- and the electronics is not the bottleneck
- **Status:** Accepted
- **Context:** [ADR-0014](#adr-0014-the-lateral-spine-is-the-trots-main-balance-actuator)
  measured the disturbance-rejection envelope against an *assumed* latency and set
  **NFR12 at <=20 ms**, unallocated -- so no subsystem owned it. Allocating it
  turns the calculation inside out.
- **The structural finding: the loop is a fixed point.** A bigger disturbance needs
  a bigger foothold correction; a bigger correction takes the leg **longer to
  execute**; and that time *is* the staleness of the information the controller
  committed on. Correction size sets latency, which sets the envelope, which bounds
  the correction. `control.self_consistent_envelope()` solves it.
- **Decision A: re-cast NFR12 as a PIPELINE budget of <=7.5 ms**, not a whole-loop
  budget. The pipeline is the part anyone can design to:

  | Stage | Budget | Basis |
  |---|---|---|
  | Contact detection (paw -> flag) | **1.0 ms** | NFR8's >=1 kHz; the flag is a comparator, no filter delay |
  | **State estimation** (contact + IMU + kinematics -> DCM) | **5.0 ms** | ⚠️ the loosest number, and the real constraint on firmware -- caps estimator group delay at roughly a >=30 Hz corner |
  | Bus transport (both directions) | **1.0 ms** | ~60 us/frame CAN-FD; 3-6 nodes/segment at 1 kHz is 18-36 % utilisation |
  | Control computation | **0.5 ms** | one multiply-add inside NFR3's >=1 kHz loop |

  The whole-loop latency is then **~45 ms**, of which **37 ms is the leg moving**.
- **Decision B: correct NFR10 from 90 mm to ~59 mm.** The 90 mm figure assumed
  **zero latency**. Solved as a fixed point at the allocated pipeline the envelope
  is **59 mm** -- still a **0.46 m/s** lateral shove, but a third smaller than
  published. This is the third correction to this number
  ([ADR-0013](#adr-0013-closed-loop-balance-by-step-to-step-foot-placement) 74 mm →
  [ADR-0014](#adr-0014-the-lateral-spine-is-the-trots-main-balance-actuator) 33 →
  [ADR-0015](#adr-0015-both-m9-follow-ups-close-no-change-needed) 90 → **59**), and
  the pattern each time was the same: a term that was assumed rather than measured.
- **The useful surprise: the envelope is nearly INSENSITIVE to the pipeline.**
  Going 2.5 → 20 ms costs 62 → 52 mm, about 16 %. Three consequences:
  - **Electronics and firmware have a comfortable budget.** Even a sloppy 20 ms
    pipeline costs little. Chasing microseconds on the CAN-FD bus would be effort
    in the wrong place -- ADR-0005's architecture is *not* the constraint. This is
    a useful negative result for two subsystems that had an unowned requirement.
  - **Foot speed is the lever.** Doubling the leg's spare foot speed buys more
    envelope than eliminating the entire electronics pipeline. The ceiling is
    5.93 m/s (motor free speed through the tendon ratios) against a nominal swing
    peak of 1.83 m/s, so the headroom exists -- the question is using it.
  - **Or attack the 0.44 projection.** The correction is inflated 2.3x because
    sagittal legs push obliquely to the support-line perpendicular. That projection
    has now cost the design something three times over, and this is the strongest
    remaining argument for **revisiting leg abduction** --
    [ADR-0015](#adr-0015-both-m9-follow-ups-close-no-change-needed) closed it on
    *authority* grounds, which does not address *actuation time*.
- **Consequences:**
  - Full allocation and workings: [notes/latency-budget.md](notes/latency-budget.md).
  - ⚠️ The 37 ms actuation term is **optimistic** -- constant-velocity
    correction, ignoring the accelerate/decelerate ramp and torque limits during
    it. It is now the dominant term in the whole budget, so this is the single most
    valuable thing left to measure on real hardware.
  - `self_consistent_envelope` builds its plant once rather than per bisection
    step; doing otherwise re-ran a full IK sweep 34 times and dominated the test
    suite.

## ADR-0017: State the disturbance requirement -- and abduction closes for the third time, properly
- **Status:** Accepted
- **Context:** [ADR-0016](#adr-0016-latency-is-not-an-independent-parameter--and-the-electronics-is-not-the-bottleneck)
  left two follow-ups: bench the actuation ramp (its dominant term rested on a
  constant-velocity idealisation), and take a third look at **leg abduction** on
  *actuation-time* grounds. Doing both exposed a gap underneath them: **the project
  has never stated what disturbance the robot must survive.**
- **The ramp: modelled, and it barely matters.** Replacing constant velocity with a
  trapezoidal accelerate/cruise/decelerate profile moves the envelope only
  **59.2 → 57.0 mm** (-4 %). ADR-0016's caveat was over-cautious. The reason is
  another P1 dividend: at 95 g the leg has ~**107 g of foot acceleration**
  available, so the move is **speed**-limited, not acceleration-limited. The ramp
  costs ~10 % on a full-scale correction but ~50 % on a short one, where the move
  never reaches cruise.
  - ⚠️ **Do not compute the acceleration limit by driving every joint at
    peak torque at once.** The distal joint's inertia is tiny, so that route reports
    ~665 g -- an artefact of a near-singular direction, not a usable acceleration.
    The operational-space form (`tau = J^T Lambda a`, minimised over directions)
    gives the defensible ~107 g.
- **Abduction, quantified at last.** It points along the support-line perpendicular
  (0.897) instead of obliquely (0.442), so the same correction needs **2.3x less
  foot travel** -- actuation drops **41 → 24 ms**:

  | abduction | perpendicular reach | self-consistent envelope |
  |---|---|---|
  | none (shipped) | 33 mm | **57.0 mm** |
  | ±10° | 59 mm | 80.7 mm (+42 %) |
  | ±15° | 72 mm | 85.7 mm (+50 %) |
  | ±20° | 85 mm | 89.9 mm (+58 %) |

  Cost unchanged: **+4 motors = 528 g, 13 % of the 4.05 kg budget.**
- ⚠️ **The gap this exposed.** ADR-0015 rejected abduction because
  "the requirement is already met" -- but **NFR13 was recording a *capability*, not
  a *requirement*.** That capability has since been corrected three times
  (74 → 33 → 90 → 59 → 57 mm), so the rejection was resting on a moving number with
  nothing behind it. The decision was unanswerable as posed.
- **Decision A: state the requirement (new NFR15).** Derived from cases the robot
  will actually meet:

  | case | DCM error | within 57 mm? |
  |---|---|---|
  | Human nudge, 5 N for 0.1 s | 16 mm | yes |
  | **Firm push, 15 N for 0.1 s** | **48 mm** | **yes** (~19 % margin) |
  | Unexpected 40 mm step | 40 mm | yes |
  | 10° lateral slope (steady bias) | 29 mm | yes |
  | Hard shove, 30 N for 0.1 s | 96 mm | **no -- a stated limit** |

- **Decision B: abduction stays REJECTED, now on solid ground.** The shipped
  57 mm meets NFR15 with margin. Abduction remains a **costed, quantified option**
  (+42-58 % for 528 g) rather than an open question -- if a future requirement
  demands rejecting a hard shove or rough terrain, this is the lever and its price
  is known.
- **Consequences:**
  - The pattern behind three corrections in a row is now named: **a capability was
    being used where a requirement belonged.** NFR13 is re-labelled as *measured
    capability*; NFR15 carries the requirement.
  - `actuation_time` takes an `accel_limit`; passing `inf` recovers the ADR-0016
    model, so the older figures remain reproducible rather than lost.
  - ⚠️ NFR15's cases are `[assumed]` engineering scenarios, not measured
    ones. They are a *stated* basis, which is the improvement -- not a validated
    one. A real disturbance test would supersede them.

## ADR-0018: Close the dH/dt = 0 caveat -- large, but on an axis the contacts can resist
- **Status:** Accepted
- **Context:** Every dynamics milestone since M6 has carried the same warning:
  the moment balance assumes **`dH/dt = 0`** (the classical ZMP form), and M6
  explicitly noted it "would **not** hold for a fast or dynamic gait". The trot is
  exactly that gait, and the warning had never been converted into a number.
- ⚠️ **A silent-zero bug, found on the way.** `angular_momentum_caveat`
  required **exactly one** swing leg. A trot moves *diagonal pairs*, so two legs
  are always in flight -- every phase was skipped and the function returned a
  reassuring **0.00 mm** having evaluated nothing. A zero that means "not measured"
  is worse than no number at all; it is now summed over all legs in flight, and a
  test asserts the trot figure is non-zero.
- **The magnitude: the assumption IS badly violated at trot.**

  | gait | swing-leg ZMP-equivalent shift |
  |---|---|
  | Crawl (5.0 s) | **1.0 mm** -- negligible, as M6 assumed |
  | **Trot (0.3 s)** | **42.5 mm** -- 41x larger, comparable to the whole 57 mm envelope |

- **The resolution: it lands mostly on an axis the contacts can resist.** Two point
  contacts can balance *every* moment except the one about the line joining them.
  The swing legs move mostly fore-aft, so their reaction is mostly **pitch** -- and
  the diagonal support line is mostly fore-aft too, so only its small perpendicular
  component couples in:

  | term about the diagonal | max | vs gravity |
  |---|---|---|
  | Gravitational topple | 0.907 N·m | -- |
  | Swing **orbital** (`m r x a`) | 0.197 N·m | 22 % |
  | Swing **spin** (`I alpha`, added in M13) | 0.026 N·m | **3 %** |
  | Swing total | 0.189 N·m | **21 %** |

- **Decision: the caveat is CLOSED as quantified, not as eliminated.** `dH/dt` is a
  ~21 % correction to the trot's roll balance -- a real term, now modelled, not a
  reversal. **M7's bounded roll survives**: including it moves the oscillation
  0.39 → 0.31 deg peak-to-peak and leaves the per-cycle drift small.
  `swing_leg_moment(..., include_spin=False)` reproduces the pre-M13 figures.
- **A third P1 dividend.** Slender-rod inertia goes as `m L^2 / 12`, and tendon
  drive keeps the legs at **95 g and short** -- so the spin term is only 3 %. On a
  robot with motors at the joints it would be a first-order effect. P1 has now paid
  off measurably three times: swing torque (ADR-0011), foot acceleration
  (ADR-0017), and link spin here.
- **Consequences:**
  - The polygon/ZMP path (used by the **crawl**) keeps the classical form, which is
    justified there by the 1 mm figure rather than by assertion.
  - ⚠️ Still not modelled: **trunk and spine** angular momentum -- only the
    LEGS are resolved. The spine is 40 % of body mass and does move laterally
    during balance, so this is the natural remaining gap.
  - ⚠️ Links are slender rods about their own CoM; no products of inertia,
    no off-axis terms. Adequate for a planar leg, not for the righting reflex.

---

### How to add an ADR
Copy the block below, bump the number, and fill it in.

```
## ADR-NNNN: <short title>
- **Status:** Proposed
- **Context:** <why a decision is needed>
- **Options:** <alternatives considered>
- **Decision:** <what was chosen>
- **Consequences:** <trade-offs, follow-ups>
```
