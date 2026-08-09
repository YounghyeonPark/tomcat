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

## ADR-0019: The spine assist is not free -- it costs ground friction, and that reinstates a withdrawn requirement
- **Status:** Accepted
- **Context:** [ADR-0014](#adr-0014-the-lateral-spine-is-the-trots-main-balance-actuator)
  made the lateral spine the trot's dominant balance actuator, and `control.py`
  modelled it as a bounded **offset added straight to the DCM** -- free of charge.
  It is not free.
- **The physics that was missed.** Bending the spine is **internal motion**, and
  internal motion cannot move the whole-body CoM. Moving the CoM *relative to the
  planted feet* requires a horizontal **ground reaction** -- i.e. friction.
  Shifting by `d` within a stance `t` needs `a ~ 4d/t^2`, hence

  > `mu_spine >= 4d / (t^2 g)` **on top of** whatever the gait already spends.

  For the shipped trot: the full 39.4 mm of spine authority needs
  **mu = 0.71**, and the gait itself already spends **0.145** -- so
  **mu = 0.86 total**, against a floor that supplies 0.8-1.2.
- **Decision: the spine authority is clamped by friction as well as ROM.**
  `StepPlant.from_gait(..., floor_mu=...)` takes the floor's coefficient and uses
  the smaller of the ROM-limited and friction-limited values. This is the **third**
  constraint on this one number, and the history is worth recording:
  [ADR-0014](#adr-0014-the-lateral-spine-is-the-trots-main-balance-actuator)
  clamped it by **rate** (wrongly -- that was a requirement floor, not a
  capability), [ADR-0015](#adr-0015-both-m9-follow-ups-close-no-change-needed)
  corrected it to **ROM**, and friction was never checked at all.

  | floor mu | spine authority | envelope | binds on |
  |---|---|---|---|
  | 0.5 | 19.6 mm | 39.5 mm | friction |
  | 0.6 | 25.1 mm | 43.7 mm | friction |
  | **0.7** | **30.6 mm** | **48.2 mm** | friction |
  | 0.8 | 36.1 mm | 53.7 mm | friction |
  | **>= 1.0** | **39.4 mm** | **57.0 mm** | **ROM** |

- ⚠️ **This reinstates a requirement that [ADR-0010](#adr-0010-mass-target-30--405-kg-and-the-walk-is-limited-by-tipping-not-friction)
  withdrew -- at almost exactly the same number, for a completely different
  mechanism.** ADR-0010 struck out NFR2g (mu >= 0.70) on the grounds that friction
  was never the binding constraint for the **crawl's sway crossover**. That was
  correct *for that mechanism*. But the **trot's spine balance action** is a
  different use of the same actuator, and it needs **mu >= 0.70** for NFR15 to be
  met. The agreement is not a coincidence: both are "shift the CoM laterally by
  tens of mm inside one stance".
- **Consequences:**
  - **New NFR16: floor friction mu >= 0.70** for NFR15 to hold. Below it the
    envelope falls under the 48 mm push case -- 43.7 mm at mu 0.6, 39.5 at 0.5.
  - **The paw-pad handoff to mechanical is back on.** ADR-0010 cancelled it;
    [TACTILE_SENSING_SPEC](../mechanical/TACTILE_SENSING_SPEC.md) and
    [ASSEMBLY_SPEC](../mechanical/ASSEMBLY_SPEC.md) must again show TPU ~80A
    delivers mu >= 0.70 on the intended floor. Published PU-on-concrete is
    0.8-1.2, so it should pass -- but it is now load-bearing, not incidental.
  - `floor_mu=None` keeps the ROM-only behaviour and reproduces every pre-M14
    figure, which stays correct for a floor of mu >= 1.0.
  - ⚠️ The spine's own **reaction torque on the trunk** is still not
    modelled -- this ADR covers the *translational* cost of the assist, not the
    yaw couple its lateral swing puts into the body. That remains open.

## ADR-0020: The spine's YAW couple doubles its friction cost -- the trot slows to 50 cm/s
- **Status:** Accepted
- **Context:** [ADR-0019](#adr-0019-the-spine-assist-is-not-free--it-costs-ground-friction-and-that-reinstates-a-withdrawn-requirement)
  found that the lateral-spine balance assist costs ground friction, because
  internal motion cannot move the CoM without a ground reaction. It costed only
  the **translation**. The spine's **reaction torque on the trunk** was left as the
  open item. It is not a footnote.
- **The second cost.** The spine's swing is **asymmetric** -- its tip travels
  ~**91 mm** while its base stays put, roughly twice the 44 mm CoM shift. That
  dumps angular momentum about the **vertical** axis into the trunk. Two contacts
  resist it with a friction **couple** over their separation, and a couple loads
  each foot with the **full** force rather than half:

  | cost at full ROM | mu |
  |---|---|
  | Translation (ADR-0019) | 0.98 |
  | **Yaw couple** (this ADR) | **0.27** |
  | Gait's own demand | 0.145 |
  | **Total** | **~1.4** |

  Peak `dHz/dt` is **2.61 N·m** (grid-converged; cross-checked by hand against the
  front-girdle term alone at 1.32 N·m). No ordinary floor supplies mu 1.4.
- ⚠️ **Profile shaping does NOT help -- measured, not assumed.** The spine
  has three independent lateral joints, and an S-bend command is genuinely more
  yaw-efficient per degree (up to **7.5x** better shift-per-yaw). But it loses more
  CoM shift than it saves in friction: swept over all profiles, the **uniform**
  command gives the most usable shift for a given friction budget. A tempting lever
  that turns out to be a dead end.
- **Decision: slow the trot from 0.30 s to 0.40 s -- 67 -> 50 cm/s.** Both friction
  costs scale as **1/stance^2**, so a longer stance buys robustness fast:

  | period | speed | spine mu at ROM | usable spine @ mu 0.8 | envelope | NFR15? |
  |---|---|---|---|---|---|
  | 0.30 s | 66.7 cm/s | 1.26 | 20.5 mm | 40.2 mm | **NO** |
  | **0.40 s** | **50.0 cm/s** | **0.71** | **36.5 mm** | **51.8 mm** | **yes** |
  | 0.50 s | 40.0 cm/s | 0.45 | 39.4 mm (ROM) | 53.5 mm | yes |

  The shipped default must meet its own stated requirement on a realistic floor.
  This is the same call M6 made when dynamics showed the 1.4 s crawl infeasible.
- **Consequences:**
  - **Speed and disturbance robustness trade against each other through FRICTION**,
    at a steep `1/t^2` exchange rate. That is the governing relationship for this
    gait, and it was invisible while the spine assist was modelled as free.
  - NFR16 (mu >= 0.70) **stands** -- it is met at the new period. Faster running
    remains available on a better floor; it is a *floor-dependent* capability now,
    not a fixed one.
  - ⚠️ **The sensing requirement tightened.** A longer stance means more
    time to topple: per-step growth rises **3.21 -> 4.73**. A DCM estimation bias
    that was merely a standing offset at 0.3 s now runs the loop away past
    **6.9 mm**. NFR11 asks for <= 3 mm, so there is ~2.3x margin -- but it is a
    smaller margin than before, and it moved for a reason unrelated to sensing.
  - The self-consistent envelope is **53.9 mm** at the shipped period (vs 57.0 mm
    quoted at 0.3 s with the spine assumed free).
  - `spine_friction_cost()` reports both terms; `StepPlant.from_gait(floor_mu=...)`
    applies them. `floor_mu=None` reproduces every pre-M14 figure.

## ADR-0021: Power and runtime -- standing costs 76 % of moving, for zero work
- **Status:** Accepted
- **Context:** Fifteen milestones established what the robot can *do*. None asked
  how long it could do it for. **NFR6 (runtime) had read `TBD` since M1**, and the
  300 g battery in the mass budget had never been checked against a load.
  `kinematics/src/tomcat_kin/power.py` now computes it from the resolved gait
  torques rather than an assumption.
- **NFR6, answered:**

  | | power | endurance |
  |---|---|---|
  | **Trotting** (50 cm/s) | **83.6 W** | **30 min, ~900 m** |
  | Standing | 67.2 W | 37 min |
  | Standing **with the ADR-0003 brake** | ~15 W (electronics only) | **168 min** |

- ⚠️ **The finding: standing costs 76 % of what moving costs, and does no
  work.** A cable can only pull, so a tendon-driven joint holds its posture with
  **motor current**, and that current burns `I^2 R` whether or not anything moves.
  ADR-0003 called the power-off brake "essential" on qualitative grounds; this is
  what it is worth -- **4.5x standing endurance**. It is not an optimisation.
- ⚠️ **The drive is only 39 % efficient.** Copper loss (42 W) exceeds the
  useful mechanical work (27 W) at the trot operating point. That is a property of
  the transmission, not of the gait.
- **A lever this exposes: the joint moment arm sets EFFICIENCY, not just cable
  tension.** Motor torque is `tau_joint * r_spool / r_joint`, so copper loss goes
  as the **square** of that ratio:

  | moment arms | copper | total | trot endurance | hip sheave |
  |---|---|---|---|---|
  | **1.00x** (shipped) | 42.0 W | 83.6 W | 30 min | 56 mm |
  | **1.25x** | 26.9 W | 68.4 W | **37 min (+23 %)** | 70 mm |
  | 1.50x | 18.7 W | 60.2 W | 42 min (+40 %) | 84 mm |
  | 2.00x | 10.5 W | 52.0 W | 48 min (+60 %) | 112 mm |

  [LEG_TENDON_SPEC](../mechanical/LEG_TENDON_SPEC.md) sized these arms for **cable
  tension**. They have a second role it never costed. **1.25x is recorded as a
  costed option, not adopted** -- a 70 mm hip sheave is large for a cat leg and the
  change ripples through mass, packaging and inertia. 2.0x (112 mm) is clearly out.
- **Consequences:**
  - **NFR6 set to ~30 min / ~900 m** trotting, ~168 min standing *with* the brake.
  - The brake moves from "specified" to "budgeted": without it the robot cannot
    idle usefully, and idling is most of what a pet robot does.
  - Peak current **2.79 A** against the driver's 4.19 A rating, RMS **0.89 A**
    against 1.60 A rated -- comfortable, and it confirms the ADR-0008 sizing from a
    direction (thermal/electrical) that had not been checked.
  - ⚠️ Deliberately pessimistic and flagged: **no regeneration** (negative
    work is treated as dissipated, though a backdrivable QDD drive could recover
    some), `I^2 R` on the phase-to-phase resistance (matching the down-select
    note's convention so the two agree), and **no iron, switching or gearbox
    losses**. Real draw will be *higher* than the motor terms; the 15 W electronics
    allowance is `[assumed]`.
  - ⚠️ Battery energy density (175 Wh/kg) and usable fraction (80 %) are
    `[assumed]`. A real cell selection could move the runtime +/-25 % on its own.

## ADR-0022: An independent physics engine says LIPM is CONSERVATIVE -- and finds a blind spot it cannot see
- **Status:** Accepted
- **Context:** Every balance number since [ADR-0013](#adr-0013) rests on a **Linear
  Inverted Pendulum Model**: a point mass at constant height on a massless leg with
  `dH/dt = 0`. `control.py` is built entirely on it, and
  [OPEN_RISKS](OPEN_RISKS.md) SS6 still listed the trunk/dorsoventral angular-momentum
  terms as *"expected small ... but that is an expectation, and this project has been
  wrong about exactly that kind of expectation four times."* A MuJoCo model
  (`kinematics/src/tomcat_kin/mjcf.py`) now checks it from outside.
- **The validation gate comes first.** A physics model that had drifted from the
  parameter set would produce confident, wrong numbers. The MJCF is *generated from
  the live parameters*, and reproduces the analytical model exactly:

  | check | agreement |
  |---|---|
  | Total mass | **0.00000 g** |
  | All four paw tips vs `LegModel.forward` | **0.000000 mm** |
  | Whole-body CoM (x, z) | **0.0000 mm** |

- **Result 1 -- the divergence rate is right, and errs SAFE.** Released near-neutral
  on the diagonal support line with no balance control, the measured divergence in
  the small-perturbation limit is **7.55 rad/s against LIPM's 7.71 -- about 2 %
  SLOWER**. Distributed inertia resists the topple that a point mass cannot. Per
  stance that is a growth of 4.53 vs 4.68.
  **This closes the SS6 `dH/dt` item** -- not by computing each term, but by measuring
  their aggregate effect on the only quantity they could change. The reduced-order
  model is conservative, which is the direction a design may safely be wrong in.
- **Result 2 -- constant CoM height holds.** LIPM assumes it; the real CoM drops
  **0.5-4.5 mm** while toppling, on a 162 mm height.
- ⚠️ **Result 3 (new) -- the trot has TWO topple axes, 52.4 deg apart.** `StepPlant`
  collapses balance onto a single axis with a fixed `projection = 0.4417`. The two
  diagonal support lines are **not parallel**: LF-RR and RF-LR give perpendiculars
  52.4 deg apart (`p1 . p2 = 0.61`). The *magnitudes* match `projection` exactly for
  both -- which is why the 1-D reduction works at all -- but consecutive steps
  correct along **different directions**, and a disturbance corrected on one
  diagonal retains a 0.61 component on the next. The 1-D model cannot express this,
  and cannot express **direction-dependence of the envelope**.
- **Result 4 -- the M8 capture-vs-recover correction is confirmed from outside.**
  M8 caught, in simulation, that placing the foot *at* the DCM (`p = xi`) arrests
  motion but leaves the body displaced. In MuJoCo the capture law **fell at every
  disturbance tested**, including 6.5 mm; the recover law survived. An error the
  project found by reasoning is now reproduced by an independent engine.
- ⚠️ **What this does NOT settle: the envelope magnitude.** The closed-loop harness
  recovers to ~13 mm against a predicted feet-only **30.34 mm**, but its own
  *undisturbed* baseline drifts up to 25 mm over ten steps -- the same order as the
  quantity being measured. **The shortfall is therefore not reportable as a finding.**
  A harness whose noise floor matches its signal cannot adjudicate; saying otherwise
  would repeat this project's own recurring error. Closing this needs a controller
  that also regulates the along-line component.
- ⚠️ **This validates the MODEL, not the INPUTS.** No physics engine knows what a
  GIM3505-9 weighs or how grippy a TPU pad is. OPEN_RISKS **R1 and R2 are untouched**
  by any of this and still need a scale and a drag test.
- **Consequences:**
  - `omega` and the per-step growth stand as published, now with ~2 % conservatism
    measured rather than assumed.
  - The single-axis reduction is recorded as a **known structural limit** rather than
    an unexamined assumption -- the honest status for something not yet costed.
  - `mujoco` is an **optional** dependency. The 321 analytical tests stand alone;
    6 more run when it is present and skip when it is not.

## ADR-0023: The battery is the thermal protection -- and that is a coincidence, not a design
- **Status:** Accepted
- **Context:** [ADR-0021](#adr-0021-power-and-runtime--standing-costs-76--of-moving-for-zero-work)
  checked the motor **electrically** -- 2.79 A peak against a 4.19 A rating, 0.89 A
  RMS against 1.60 A -- and called it comfortable. That is not the thermal question.
  The thermal question is whether the heat can *leave*.
  [OPEN_RISKS R5](OPEN_RISKS.md) parked it as *"gated on having the motor"*.
  **It was not gated on hardware.** A lumped-capacitance model
  (`thermal/`, on the `dualis-thermal` crate) answers it.
- **The result, at the boundary that actually decides it.** P1 centralises the
  motors, so the six in a girdle do not each have free air -- the assembly skin is
  what rejects the heat:

  | front girdle, 6 motors, 21 W | continuous | one battery |
  |---|---|---|
  | trot, polished | **113.7 C** | 67.1 C |
  | trot, anodised | 74.9 C | 59.7 C |
  | stand w/o brake, polished | **134.1 C** | 85.7 C |
  | stand w/o brake, anodised | 85.4 C | 72.5 C |

- **The finding: the battery is the thermal protection, by coincidence.** The bare
  girdle takes **~47 min** to get most of the way up; the pack can only feed it for
  **30 min** (ADR-0021). The robot runs out of energy before it overheats. **Tether
  it, or hot-swap the pack, and that protection disappears** -- continuous trotting on
  a bare girdle settles near **114 C**, past NdFeB's comfortable range. Nothing in the
  design put that margin there; it fell out of two unrelated numbers.
- ⚠️ **Correction (see the amendment): the 53 min first published here was
  `LumpedMass::time_constant`, which is `C/(hA)` -- convection only.** It returns the
  same number whatever the emissivity, and radiation is the same order as still-air
  convection here. Measured from the transient the real figure is **46.6 min
  polished, 25.6 min anodised**. The temperatures were always computed with radiation
  and are unaffected; it was the *mechanism* that was misstated.
- **Decision: anodise the girdles. It is worth ~39 K** (113.7 -> 74.9 C continuous).
  Radiation is the *same order* as still-air convection at these temperatures, so
  emissivity **0.09 -> 0.90** is the cheapest thermal lever available and it needs no
  mass, volume or power. **Surface finish is a thermal parameter here, not a
  cosmetic one.**
- ⚠️ **The first place P1 CHARGES rather than pays.** Centralising six motors costs
  **38 % of the heat-rejection area** (486 -> 302 cm2). P1 has paid off measurably
  four times -- swing torque, foot acceleration, link spin, light legs. This is the
  bill, and it had not been costed.
- **Standing without the brake is the worst thermal case**, not trotting -- 4.35 W
  per motor against 3.50 W, because a cable can only pull and posture is held with
  current. ADR-0021 called the brake essential from *runtime*; this reaches the same
  place from *heat*, independently.
- **R1's mass uncertainty is a heat-CAPACITY question, not a heat-REJECTION one.**
  132 -> 200 g moves the time constant 17.4 -> 26.5 min and leaves the equilibrium
  **exactly unchanged**. A clean decoupling: a heavier motor buys time, never a
  lower final temperature.
- **Consequences:**
  - **NFR18 added:** girdles anodised (or otherwise high-emissivity), and
    **continuous/tethered operation is out of spec** unless airflow is added.
    At `h = 15` the continuous case falls to 57.5 C, so a small fan would reopen it.
  - R5 moves from *"gated on hardware"* to **partly closed**: the bench test now has
    a prediction to falsify rather than a blank.
  - ⚠️ These are **assembly-SKIN** temperatures. A lumped mass has one temperature;
    the winding runs hotter and the winding is what fails. Copper loss is the only
    source modelled (inherited from ADR-0021), so reality is worse than this.
    Girdle envelope, `h` and both emissivities are `[assumed]` and swept.
  - `thermal/` is a **leaf**: nothing depends on it, the Python suite needs no Rust
    toolchain, and `tests/test_thermal_constants.py` fails if `power.py` drifts from
    the constants copied into it.

### Amendment (dualis 0.2.0): the books are now audited, and anodising is worth more than 39 K

The first pass compared two numbers **by hand** -- "the girdle's 53 min time constant
beats the 30 min runtime" -- which is a *claim about* the coupling rather than the
coupling itself. On dualis 0.2 the pack is a real domain on the same bus, so
`Simulation::advance` runs the kernel's **conservation audit** every step.

- **The upgrade changed no number.** 0.2 swaps `Exchange::take` for
  `take_share(HEAT, dt)`, which falls back to `take` when no scheduler interval is
  set -- so the hand-stepped results were already right. Verified by re-running, not
  assumed from reading the diff.
- **The runtime is now emergent.** Nothing tells the simulation how long to run: it
  stops when 42 Wh at 83.6 W is gone, landing at **30.17 min** against `power.py`'s
  30.16 -- an independent cross-check of ADR-0021 that did not exist before.
- ⚠️ **The audit has teeth, and that is tested.** A deliberately leaky pack that
  publishes heat without debiting itself is **refused**. An audit that cannot fail is
  decoration, and this project has been burned by exactly that shape of reassurance
  (the silent-zero in `angular_momentum_caveat`).
- **The finding the hand-comparison missed.** Anodising does not merely lower the
  temperature -- it shrinks the *dependence on the coincidence*:

  | | at the flat pack | continuous | gap |
  |---|---|---|---|
  | polished | 67.1 C | 113.7 C | **47 K** |
  | **anodised** | 59.6 C | 74.9 C | **15 K** |

  A bare girdle is only survivable because the battery dies first (it reaches 47 % of
  its settled rise). An anodised one reaches 69 % -- it is close to its own
  equilibrium, so **tethering it is no longer a cliff**. That is a robustness
  argument for NFR18 that the 39 K figure alone did not make.
- ⚠️ **And a self-caught error, of this project's oldest kind.** The `53 min`
  above came from `LumpedMass::time_constant`, a **convection-only** convenience --
  `C/(hA)`, no radiation, identical for every emissivity. Measured properly it is
  **46.6 min polished and 25.6 min anodised**, so for the anodised girdle the
  time constant is *shorter* than the runtime and the "pack dies first" mechanism
  **does not apply to it at all**.

  Correcting it sharpens the conclusion rather than weakening it:

  | | effective tau | vs 30 min runtime | why it is safe |
  |---|---|---|---|
  | polished | 46.6 min | outlasts it | **only because the pack dies first** |
  | **anodised** | 25.6 min | shorter | **on its own merits** -- it nearly reaches a 75 C equilibrium |

  This is the same failure this project has hit five times before -- a **nominal
  figure standing where a measured one belonged** -- arriving through a dependency
  this time rather than through our own model. Reported upstream.

## ADR-0024: The winding runs 7.7 K above the skin -- the caveat, answered
- **Status:** Accepted
- **Context:** Every temperature in [ADR-0023](#adr-0023) carried the same warning:

  > ⚠️ A lumped mass has ONE temperature. The real winding is hotter than the skin
  > these numbers describe, and **the winding is what fails**.

  That was a limitation of the tool, not a judgement: a `LumpedMass` could not be
  joined to another by a conductance, so **winding -> stator -> housing -> girdle**
  was not expressible. Reported upstream
  ([dualis#2](https://github.com/YounghyeonPark/dualis/issues/2)); `ThermalNetwork`
  shipped in dualis-thermal 0.3. **A warning is not an answer, and now there is one.**
- **The answer:**

  | | winding | stator | housing | skin (what ADR-0023 published) |
  |---|---|---|---|---|
  | polished, continuous | **121.4 C** | 117.5 | 116.1 | 113.7 |
  | anodised, continuous | **82.6 C** | 78.7 | 77.2 | 74.9 |
  | anodised, one battery | **62.0 C** | -- | -- | 55.4 |

- **The gradient is +7.7 K, and it is the SAME for both finishes.** That is not a
  coincidence: the skin finish sets where the whole stack sits, the joints set how far
  it spreads. **Two independent levers, and only the second is uncertain.**
- ⚠️ **Which is why the sweep is the result and a single number would be false
  precision.** The joint conductances are `[assumed]` -- slot insulation, an
  interference fit, a bolted mount -- and the gradient is roughly `P/UA` in each:

  | joints | winding | above skin |
  |---|---|---|
  | 0.25x | 105.7 C | **30.7 K** |
  | 1.00x (nominal) | 82.6 C | 7.7 K |
  | 4.00x | 76.8 C | 1.9 K |

- **The verdict does not move, which is the useful part.** Anodised, the winding sits
  at **82.6 C** continuous -- comfortably inside class F (155 C) -- and the *stator*
  at 78.7 C, which matters because the rotor magnets sit against it. Polished, the
  stator reaches **117.5 C**, past where ordinary NdFeB grades are happy. **NFR18
  strengthens: anodising was a 39 K saving, and it is also what keeps the magnets in
  range.**
- **A near-coincidence worth naming, so nobody reads meaning into it.** ADR-0023's
  published battery-case skin figure (67.1 C) is almost exactly the new *winding*
  figure (67.0 C, polished). Those are different quantities that happen to land
  together for this geometry. **It is luck, not a check** -- but it does mean the
  published numbers were never misleading in practice.
- **Consequences:**
  - The ADR-0023 caveat is **discharged**, not merely restated: +7.7 K nominal,
    1.9-30.7 K across plausible joints.
  - The network is cross-checked against the single lump: same mass, same skin area,
    same emissivity, and the settled skin agrees within 2 K. If that drifts, the
    network has stopped modelling the same girdle.
  - ⚠️ **`biot_number` returns `None` for an interior node** in 0.3, deliberately.
    ADR-0023 leaned on a reassuring 5e-4 for the whole assembly -- which is the Biot
    number of a *solid block*, not of a structure with motors and air gaps in it.
    Upstream called that the sharpest thing in the report and documented it.
  - ⚠️ Still copper loss only (ADR-0021). Iron loss lands in the **stator**, so
    adding it would redistribute this gradient as well as raise it.

## ADR-0025: The sway was 4 % optimistic -- and the friction cost cannot be measured without a balance controller
- **Status:** Accepted
- **Context:** M17 ran a **rigid trunk**, so it could only test the feet-only
  envelope. The spine supplies ~23 mm of the ~53 mm headline, so **44 % of the number
  NFR15 is checked against sat outside the simulation entirely.** M20 added the three
  lateral spine joints to close that gap. It found a bug on the way in, and a wall on
  the way out.
- **Finding 1 -- the fore legs are not at the spine tip.** `center_of_mass_y` argued
  that *"left/right legs sit at symmetric track offsets, so their own +/-y
  contributions cancel"*. The **track** offsets do cancel. The **fore-aft** offset of
  a leg's CoM does not: the spine's yaw rotates it into y, and **both** fore legs
  contribute the same sign. In a trot stance that CoM sits ~52 mm *behind* the hip.

  | | full-ROM sway |
  |---|---|
  | as published | 43.97 mm |
  | **corrected** | **42.22 mm** |
  | MuJoCo, independent | 42.219 mm |

  **4.0 % optimistic**, and the corrected form agrees with the independent model to
  **0.0005 mm**. Discriminated rather than guessed: folding the legs straight down
  (CoM under the hip) collapses the gap to 0.02 mm.
- **Consequence:** self-consistent envelope **53.90 -> 52.72 mm**. NFR15 needs 48 mm,
  so it still **passes with 4.72 mm** -- the conclusion holds, the margin shrinks.
  `p.spine` at `floor_mu=0.8` is unchanged, because friction binds first.
- **Finding 2 -- the STATIC premise of ADR-0009/0019 is exact.** Holding a full-ROM
  sway against a real friction cone needs **mu 0.006**, three orders below NFR16's
  0.70, and the contacts carry exactly body weight. `lateral_spine_loads` says
  holding a sway costs essentially nothing; measured, it does.
- ⚠️ **Finding 3 -- ADR-0019/0020's DYNAMIC costs are NOT testable this way, and
  that is a fact about the mechanism rather than about the harness.** Three attempts,
  all rejected:
  1. **Free root, sweep the spine.** The robot topples. A diagonal stance diverges at
     `e^(7.77 t)`, so within one 0.2 s stance the contacts unload and it leaves the
     ground -- contact is lost for 57 % of a full-ROM sweep, and for **13 % even at
     quarter amplitude over three times the duration**. It is gravity, not the spine.
  2. **Lock roll and pitch to remove toppling.** This *breaks the mechanism*. The
     legs are **planar** -- ADR-0017 rejected abduction -- so a body sway over
     planted feet needs either foot slip or body roll. Locking roll leaves neither,
     and the model levers itself off the ground.
  3. **Read the required mu during a sweep anyway.** Every configuration slides at
     the cone limit, and "foot slip" *rises* with friction -- the signature of
     measuring a fall.

  **So the translation cost (0.98) and the yaw couple (0.27) that together slowed the
  shipped trot from 67 to 50 cm/s remain un-cross-checked.** They are not refuted;
  they are untested, and the test needs a closed-loop balance controller in the sim.
- **What this sharpens.** M17 left the envelope magnitude open and blamed harness
  drift, suggesting the fix was *"regulate the along-line component"*. That was too
  small a diagnosis. **Both** the envelope and the friction costs are gated on the
  same missing piece: a controller that keeps the robot up while the measurement is
  taken. Recorded as the single blocking item rather than two vague ones.
- **Consequences:**
  - `build_mjcf(spine_dof=True)` adds the lateral chain, validated against
    `center_of_mass_y` across eight postures to **< 1e-5 m**, with the naive form
    asserted still wrong so the size of the error stays recorded.
  - `build_mjcf(planar_root=True)` exists for **static** questions only, and its
    docstring says why using it on a moving spine is invalid.
  - ⚠️ A free-root diagonal stance has **no settled state to measure** -- contact
    forces swing between 0.74x and 1.57x body weight indefinitely. Any future test
    that quietly "settles" one is measuring a fall.

## ADR-0026: The envelope, measured -- it is direction-dependent, and balance needs compliant legs
- **Status:** Accepted; **numbers superseded by [ADR-0028](#adr-0028)** -- they were measured before the robot entered its limit cycle and are pessimistic. Worst direction is **25.3 mm**, not 19.3; spread **2.6x**, not 3.4. Conclusions unchanged.
- **Context:** [ADR-0025](#adr-0025) named one blocking item: a closed-loop balance
  controller in simulation, without which neither the **envelope magnitude** nor
  ADR-0019/0020's **friction costs** could be measured. `mjsim.BalanceHarness` is
  that controller. Building it corrected my own diagnosis twice.
- **Correction 1 -- the along-line component was NOT the main problem.** M17 blamed
  its drift on the unregulated along-line DCM, and instrumenting confirmed that
  component ran +1.7 -> +22 -> +43 -> +90 mm. But the cause was upstream: **my swing
  profile landed the foot at 0.31 m/s.** A `sin(pi u)` arc peaks correctly and has a
  non-zero slope at touchdown; it hammered the contact so the stance never settled
  at two feet. Replacing it with `(1 - cos(2 pi u))/2` -- zero vertical speed at both
  ends -- took the run from 14 steps to 40 with **no along-line regulation at all**.

  ⚠️ This is the same C0 defect **M5 and M6 already fixed** in the shipped gait,
  reintroduced by hand in a new harness. It is why `GaitParams.swing_profile`
  defaults to `"matched"`.
- **Correction 2 -- explicit CoP regulation is not available, and that is a finding.**
  Differential stance-leg extension was supposed to steer the centre of pressure. It
  cannot, with stiff position servos: **+/-1 mm of differential swings the CoP across
  the entire +/-109 mm foot separation**, and past ~2 mm the light foot simply
  unloads. The authority is effectively bang-bang, and switching it on made things
  *worse*.
- **The enabling result: balance needs COMPLIANT legs.**

  | leg `kp` | steps survived | mean \|DCM\| first 10 -> last 10 |
  |---|---|---|
  | **80** | **40, never fell** | 1.99 -> **1.52 mm** |
  | **150** | **40, never fell** | 1.88 -> 2.63 mm |
  | 250 | 23 | 6.50 -> 45.8 (diverging) |
  | 500 | 24 | 8.36 -> 28.1 (diverging) |
  | 900 | 7 | -- |

  The mechanical design already specifies passive compliance (series elastic
  elements / return springs). **This validates that choice from a direction it was
  never chosen for** -- it was bought for impact tolerance, and it turns out the
  balance loop does not close without it.
- **The result: the envelope is strongly DIRECTION-DEPENDENT.** `StepPlant` quotes a
  single **30.34 mm** (feet only) for every direction. Measured at `kp = 80`:

  | disturbance | envelope |
  |---|---|
  | 60 deg | **65.7 mm** (best) |
  | 180 deg | 44.6 mm |
  | 0 deg | 42.8 mm |
  | 240 deg | 37.4 mm |
  | 120 deg | 22.3 mm |
  | **300 deg** | **19.3 mm** (worst) |

  A **3.4x spread**, and the **worst direction is 64 % of the prediction**. M17 found
  the two diagonals topple along axes 52.4 deg apart but could not cost it. This is
  the cost: the single-axis reduction does not merely lose direction information, it
  **over-promises in the direction that matters**.
- ⚠️ **What this does NOT settle.** The peak baseline excursion is ~11 mm against a
  19.3 mm worst-direction envelope -- only **1.75x**. Comfortable for the mid and
  high directions, **marginal at the worst one**, so the 19.3 mm figure carries real
  uncertainty. It is a large improvement on M17 (25 mm of *growing* drift against a
  30 mm signal) but it is not a tight measurement.
- ⚠️ **And it is feet-only.** The trunk is rigid here, so the spine's ~22 mm share
  is not available to the controller. Whether **NFR15's 48 mm** survives the
  direction dependence depends on the spine, which acts most strongly in the lateral
  directions where the feet are weakest. **That is the next question, and it is not
  answered.** Do not read a requirement verdict out of this ADR.
- **Consequences:**
  - `mjsim.BalanceHarness` ships with `regulate_along_line=False` by default and the
    docstring says why the option exists and why it is off.
  - The friction costs of ADR-0019/0020 are now *reachable* -- the harness holds the
    robot up long enough to read contact forces -- but were not measured here.
  - Six tests, gated on the baseline: a harness whose noise matches its signal cannot
    adjudicate, so the noise floor is asserted before any result built on it.

## ADR-0027: The spine assist is not the free offset the plant credits -- and NFR15 is not demonstrated
- **Status:** Accepted; **numbers superseded by [ADR-0028](#adr-0028)**. Worst direction with the spine is **28.9 mm**, not 22.5, and the NFR15 shortfall is **1.66x**, not 2.3x. Conclusions unchanged, and the gap is now localised to the spine term.
- **Context:** [ADR-0026](#adr-0026) measured the envelope with a **rigid trunk**, so
  the spine's ~22 mm share was outside the loop and the NFR15 question stayed open.
  This puts it in. The answer is not the one the reduced-order model promises.
- **First, a tuning finding that is really a design one: the legs and the spine want
  OPPOSITE gains.** ADR-0026 established that balance needs *compliant* legs. The
  lateral spine is the reverse -- it carries the whole forequarters, and at the leg's
  compliant gain it wobbles hard enough to fell an otherwise-clean baseline in **10
  steps**. Stiffened to `kp = 1000` the baseline is quiet again (2.1 mm mean over 25
  steps). **A single "servo gain" would have hidden this**, and the two groups are
  not interchangeable.
- **The result, with the spine finally in the loop:**

  | spine gain | worst direction | best direction |
  |---|---|---|
  | 0.0 (present, unused) | 19.7 mm | 39.4 mm |
  | **0.2** | **22.5 mm** | **50.7 mm** |
  | 0.4 | **0 mm** -- falls at the smallest disturbance | 50.7 mm |
  | 0.7 | 0 mm | 25.3 mm |

- ⚠️ **The finding: the spine's authority is not a static offset.** `control.py`
  books `plant.spine = 36.6 mm` of DCM authority as if it were free and always
  available. In dynamics it has a **narrow usable window**: a gentle assist helps
  (19.7 -> 22.5 mm worst case, +14 %), and by gain 0.4 the robot falls at the
  *smallest* disturbance tested. The sway swings the entire forequarters and the
  reaction destabilises. **A static credit cannot express a stability boundary.**
- ⚠️ **NFR15 is NOT demonstrated.** The requirement is 48 mm; the best measured
  worst-direction figure is **22.5 mm**, against a predicted 52.7 mm. That is a
  **2.3x shortfall**, and it is the number a requirement should be judged on because
  a disturbance does not choose a convenient direction.
- **What that does and does not mean.** `control.py`'s envelope assumes an *optimal*
  controller using the full authority; this is a **proportional foot-placement law
  plus a proportional spine assist**, which is a long way from optimal. The gap is
  therefore an upper bound on the model's optimism and a lower bound on what better
  control could recover -- **the two cannot be separated with this harness.** What it
  does establish is that the margin is **not free**: a straightforward implementation
  gets less than half the promised envelope in its worst direction.
- **Consequences:**
  - `mjsim` defaults: legs `kp = 80` (compliant), spine `kp = 1000` (stiff),
    `SPINE_GAIN = 0.2`. Each is a measurement with the sweep behind it, and the
    docstrings say what breaks on either side.
  - ⚠️ **NFR15's status changes from "met with 4.72 mm margin" to "met in the
    reduced-order model, not demonstrated in simulation".** The requirement is not
    withdrawn and the model is not refuted -- but the margin quoted against it is a
    single-axis, optimal-control figure, and neither qualifier was ever attached.
  - The open question is now sharp and answerable: **does a better controller close
    the gap, or is the reduced-order envelope optimistic?** A whole-body QP or an MPC
    over the step would separate them.

## ADR-0028: Correcting M21/M22 -- I measured before the robot was trotting, and the model's optimism is in the SPINE term
- **Status:** Accepted. **Supersedes the numbers in [ADR-0026](#adr-0026) and
  [ADR-0027](#adr-0027); their conclusions stand.**
- **The error.** Every envelope in M21 and M22 was measured by disturbing the robot
  at `t = 0`, one settle after being placed. That is not a trotting robot -- it has
  not entered its limit cycle. Disturbing after 2, 4 or 6 undisturbed steps instead
  gives **systematically larger** figures, and the +0 column is the lowest in every
  direction tested. My numbers were pessimistic by construction.

  I found this while trying to explain the direction dependence, not by re-checking
  the result. The tell was that opposite directions on the same axis disagreed
  wildly (0 deg gave 41.8 mm, 180 deg gave 19.3 mm), which no property of the robot
  explains but an unsettled initial condition does.
- **The corrected measurements** (settled cycle, worst over 3 phases x 6 directions):

  | | published | **corrected** |
  |---|---|---|
  | worst direction, feet only | 19.3 mm | **25.3 mm** |
  | worst direction, spine at 0.2 | 22.5 mm | **28.9 mm** |
  | direction spread | 3.4x | **2.6x** |
  | NFR15 shortfall | 2.3x | **1.66x** |

- **Every qualitative conclusion of ADR-0026/0027 survives**, which is the reason
  they are corrected rather than withdrawn: the envelope is direction-dependent, the
  worst direction falls short of the prediction, the spine helps modestly (**+14 %**,
  25.3 -> 28.9 mm), its usable gain window is narrow, and **NFR15 is not
  demonstrated**.
- **And the correction sharpens the finding, which is the useful part.** Split by
  term, the model is not uniformly optimistic:

  | | predicted | measured worst | achieved |
  |---|---|---|---|
  | **feet only** | 30.3 mm | 25.3 mm | **84 %** |
  | **with spine** | 52.7 mm | 28.9 mm | **55 %** |

  ⚠️ **The foot-placement model is nearly right. The spine credit is what does not
  materialise.** `control.py` books `plant.spine = 36.6 mm` as a static DCM offset;
  measured, the spine buys **3.6 mm** of worst-case envelope. That localises the gap
  to one term instead of leaving it spread across the whole model, and it is
  consistent with ADR-0027's independent finding that the assist has a narrow stable
  gain window.
- **Consequences:**
  - **Envelopes must be measured on a settled cycle.** The harness makes this easy to
    get wrong -- `run(disturbance=...)` applies it immediately -- so the tests now
    pre-run before disturbing, and the docstring says why.
  - NFR15 remains **not demonstrated**, at 28.9 mm against 48 mm required.
  - The next question is unchanged but better aimed: the gap is **in the spine term**,
    so a better controller should be judged on whether it can extract more than
    3.6 mm from a 36.6 mm credit.
  - ⚠️ This is the second time in this project that a measurement harness, not the
    model, produced the wrong number -- after M17's drift. **A harness is an
    experiment and needs its own controls.**

## ADR-0029: The proportional spine assist has unity loop gain -- it is harmful, and M22/M23's "+14 %" is withdrawn
- **Status:** Accepted. **Retracts the spine benefit claimed in
  [ADR-0027](#adr-0027) and [ADR-0028](#adr-0028).**
- **Context:** ADR-0028 localised the envelope gap to the spine term and asked
  whether a better controller could extract more than 3.6 mm from `control.py`'s
  36.6 mm credit. Before building one, I measured why the existing assist did so
  little. It does **worse than little**.
- **The finding, and it is derivable rather than empirical.** The law is
  `q = -gain * e / SPINE_SWAY_PER_RAD`, and a sway of `q` moves the CoM by
  `SPINE_SWAY_PER_RAD * q = -gain * e`. **The loop gain is `gain` exactly, by
  construction** -- the actuator sits directly in the position feedback path with no
  attenuation. With any lag it is marginal near 1. Measured on the *undisturbed*
  baseline:

  | spine gain | mean \|DCM\| over 20 steps | |
  |---|---|---|
  | **0.0** | **2.15 mm** | clean |
  | 0.2 | **11.43 mm** | **5x worse, with no disturbance at all** |
  | 0.5 | -- | falls at step 6 |
  | 1.0 | -- | falls at step 4 |

- ⚠️ **So the "+14 % worst case from the spine" reported in ADR-0027/0028 is
  withdrawn.** It was measured inside the noise the assist itself created. On a
  settled cycle the worst direction is **28.9 mm with the assist and 28.9 mm
  without** -- the spine contributes nothing to the worst case, and degrades every
  other measurement's resolution.
- **Two things this does NOT show, and the distinction matters:**
  - **The motor is not the limit.** An open-loop ramp to full ROM survives at
    **300 deg/s**, against `control.py`'s 912 deg/s capability and the ~200 deg/s a
    full traverse needs. ADR-0019's "ROM-limited, not rate-limited" stands as far as
    the *drive* is concerned.
  - **Slew-limiting does not rescue it.** Adding a 3 rad/s rate limit to break the
    chatter left gain 0.5 and above still collapsing in every direction. This is a
    loop-gain problem, not a bandwidth one.
- ⚠️ **And one thing I could NOT measure, recorded as a non-result.** I tried to
  show the 36.6 mm credit is physically realisable by holding a full-ROM sway while
  trotting and reading the CoM offset from the support line. **Two runs disagreed:
  44.0 mm and 16.5 mm.** The cause is that the offset is not steady -- it oscillates
  through zero and drifts (+8, +19, -2.8, -10, -26, -71 mm over 14 steps), so
  averaging its magnitude reads a drift as a bias. **How much offset the spine can
  hold against planted feet remains unmeasured**, and the first attempt's answer was
  an artefact of the statistic, not a result.
- **Consequences:**
  - `SPINE_GAIN` defaults to **0.0**. The assist is off, and the docstring gives the
    unity-loop-gain derivation so it is not switched back on hopefully.
  - The M24 question is unchanged but its premise is corrected: the spine credit is
    **not being spent at all**, rather than being spent inefficiently. A planned or
    feedforward deployment is the next thing to try -- reactive proportional control
    is structurally the wrong shape for an actuator that sits in its own feedback
    path.
  - **NFR15's status is unaffected**: still not demonstrated, at 28.9 mm against
    48 mm. Only the attribution changes.

## ADR-0030: Planned deployment fixes the spine's stability -- and it still buys nothing
- **Status:** Accepted
- **Context:** [ADR-0029](#adr-0029) showed the reactive spine assist has **unity
  loop gain by construction** and is harmful. That left the question open in the
  best possible way: was the spine credit unreachable, or merely unreachable *by
  that control structure*? M25 changes the structure.
- **The fix, and it is structural rather than a tuning.** The spine target is now
  decided **once per stance**, at the same instant the foot placement is committed,
  and executed **open-loop** as a C1 ramp across the stance. The loop closes at the
  step rate, exactly like the foot placement -- the one control structure in this
  harness that demonstrably works.

  | spine gain | reactive baseline | **planned baseline** |
  |---|---|---|
  | 0.0 | 2.15 mm | 2.15 mm |
  | 0.2 | 11.43 mm | **1.64 mm** |
  | 0.5 | falls at step 6 | **1.38 mm** |
  | 1.0 | falls at step 4 | **1.55 mm** |
  | 1.5 | falls at step 3 | 7.30 mm |

  **Stable to gain 1.0, and it slightly IMPROVES the undisturbed baseline.** So
  ADR-0029's instability was the control structure, not the actuator -- which is the
  cleanest possible confirmation of that diagnosis.
- ⚠️ **And it still buys nothing.** Measured at **0.23 mm** resolution (10 bisection
  steps; earlier sweeps ran at ~3.6 mm and were quantising the answer):

  | direction | gain 0.0 | gain 0.5 | gain 1.0 | best gain |
  |---|---|---|---|---|
  | **120 deg (worst)** | 29.62 mm | 29.85 | 28.26 | **+0.23 mm** |
  | 300 deg | 35.27 | 33.92 | 37.31 | +2.04 |
  | 0 deg | 54.95 | 56.08 | 59.02 | +4.07 |
  | 180 deg | 65.35 | 63.31 | **53.14** | 0.00 -- gain 1.0 *costs* 12 mm |

  **A stable implementation of the model's own mechanism, driven at full authority,
  adds 0.23 mm to the worst case against a credited 36.6 mm.**
- **What that finally settles.** M23 could not attribute the gap; M24 could not
  either, because the assist was unstable and an unstable controller proves nothing.
  This one is stable, uses the authority the way `control.py` describes it (a DCM
  offset), and still does not deliver. **That is evidence the credit is wrong, not
  merely unreached.** `plant.spine = 36.6 mm` should be treated as unsupported until
  something demonstrates otherwise.
- ⚠️ **The honest residue.** This controller is still not optimal, and "a stable
  proportional feedforward finds 0.23 mm" is not a proof that no controller can find
  more. But the burden has moved: the credit is a *modelling* claim with no
  supporting measurement, sitting in the middle of the NFR15 margin.
- **Consequences:**
  - `spine_mode` defaults to `"planned"`; `"reactive"` is kept only so the ADR-0029
    test can demonstrate the failure it describes.
  - `SPINE_GAIN = 0.5`, chosen for the quietest baseline -- **explicitly not for
    envelope**, and the docstring says so.
  - **NFR15 is unchanged and now better supported as a concern**: 28.9 mm worst-case
    against 48 mm required, with the spine term measured at ~0 rather than assumed
    to be recoverable by better control.
  - ⚠️ Earlier envelope sweeps ran a 6-step bisection over a 1.8 m/s bracket --
    **3.6 mm of quantisation**, enough to hide the whole effect being argued about.
    Resolution is now stated with every envelope figure.

## ADR-0031: The spine credit is authority in the WRONG AXIS -- the binding mode is along-line
- **Status:** Accepted. Resolves the question left open by [ADR-0030](#adr-0030).
- **Context:** ADR-0030 established that a *stable* implementation of the spine
  assist, at full authority, adds **0.23 mm** to the worst-case envelope against a
  credited **36.6 mm** -- and left `plant.spine` to be justified or withdrawn. Two
  candidate explanations were open: the credit is double-spent across a multi-step
  recovery, or it is unreachable. **Both are wrong.**
- **Not double-spent.** Instrumented, `simulate` invokes `spine_assist` on exactly
  **1 of 400 steps** of a recovery at the envelope limit -- the deadbeat placement
  nulls the error immediately afterwards, so the assist is never asked for again.
  Cumulative demand is **1.0x** full ROM. The arithmetic is internally consistent and
  I was wrong to suspect it.
- **The actual reason, from the failure mode.** At the envelope limit in the worst
  direction, logging every step of a failing recovery:

  | step | perp | **para** | foot dx | contacts |
  |---|---|---|---|---|
  | 0 | -19.0 mm | **+71.4 mm** | 35.6 mm | 1 |
  | 1 | -63.2 | **-115.2** | **saturated** | 1 |
  | 2 | -134.2 | **+114.3** | **saturated** | 1 |

  ⚠️ **The along-line component is consistently 2-4x the perpendicular one, and
  nothing controls it.** Foot placement moves both feet fore-aft, so it acts on the
  line's position; the spine acts **laterally**. Neither addresses motion *along* the
  support line. `plant.spine` is real lateral authority credited against a failure
  mode that is not lateral.
- ⚠️ **And the support is barely a line.** `ncon = 1` through most of a recovery --
  the robot is on **one foot**, not two, so the support-line geometry the whole
  reduced-order model rests on does not hold while it is actually recovering.
- **This also corrects M21.** [ADR-0026](#adr-0026) concluded that along-line
  regulation was unnecessary once the C1 swing profile was fixed. That was measured
  on the **undisturbed baseline**, where it is true. Under a **disturbance** the
  along-line component is exactly what runs away. I conflated "the baseline is quiet"
  with "the axis is controlled", and they are different claims.
- **Decision: `plant.spine = 36.6 mm` is NOT withdrawn, but it is re-scoped.** The
  number is a correct statement about lateral CoM authority. What is unsupported is
  **adding it to a single-axis envelope as though the binding constraint were
  perpendicular**. `StepPlant` has one axis and cannot distinguish the two; that is
  the defect, not the spine figure.
- **Consequences:**
  - The arc M17 -> M26 resolves into one statement: **the trot's balance problem is
    two-dimensional with two uncontrolled-to-different-degrees axes, and the
    reduced-order model collapses it to one.** M17 found the 52.4 deg axis split,
    M21 mis-scoped it, M25 showed the spine cannot close it, M26 says why.
  - `NFR15` remains **not demonstrated** (28.9 mm vs 48 mm), and the reason is now
    named rather than attributed to controller quality.
  - ⚠️ The next honest step is **not** a better controller. It is deciding whether
    the robot needs an actuator with along-line authority at all -- which is what
    ADR-0017's rejected **leg abduction** would have supplied, at +4 motors and 528 g.
    That decision was taken on the basis that NFR15 was already met.

## ADR-0032: Three actuators, three failures to deliver -- the architecture is the limit, so do NOT buy abduction yet
- **Status:** Accepted
- **Context:** [ADR-0031](#adr-0031) found the binding failure mode is the
  **along-line** DCM component, which neither fore-aft foot placement nor the lateral
  spine can address, and pointed at ADR-0017's rejected **leg abduction** (+4 motors,
  **528 g**, 13 % of the mass budget) as the only costed option that would supply it.
  Before recommending that, the free option had to be exhausted.
- **The free option exists, and M21 wrote it off wrongly.** Differential stance-leg
  extension shifts the centre of pressure **along** the support line -- exactly the
  missing axis. M21 measured it at `kp = 500` and found it bang-bang (1 of 7 points
  kept two contacts), then discovered compliance is what makes balance work at all
  and **never came back to it**. Re-measured at the shipped `kp = 80`:

  | | stiff (`kp` 500) | **compliant (`kp` 80)** |
  |---|---|---|
  | points keeping 2 contacts | 1 of 7 | **5 of 7** |
  | normal force | collapses to 1.5-11 N | **39.67 N throughout** |
  | CoP response | saturated at a foot | **linear, -39.3 mm/mm** |

  The along-line actuator is real, proportional, costs nothing, and covers most of
  the +/-109 mm foot separation.
- ⚠️ **And it buys no envelope either.** Swept across both signs and four gains at
  the worst direction, the best result was **+1.8 mm** -- about two bisection steps --
  while degrading the undisturbed baseline from **1.38 mm to 5.53 mm**.
- **The pattern is the finding.** Three independent actuators, three different axes,
  three ways of failing to deliver credited authority:

  | actuator | credited | delivered | how it failed |
  |---|---|---|---|
  | Lateral spine | 36.6 mm | **+0.23 mm** | wrong axis (ADR-0031) |
  | CoP / weight shift | +/-109 mm of CoP | **+1.8 mm** | degrades the baseline |
  | Foot placement | 30.3 mm | 25.3 mm (84 %) | the one that mostly works |

  **Only the actuator my controller was designed around delivers.** That is not three
  coincidences about three actuators; it is one fact about the controller.
- **Decision: do NOT reopen leg abduction on these grounds.** Adding 528 g and four
  motors of authority to a controller that cannot exploit the authority it already
  has would be **mass spent on a problem it does not solve**. ADR-0017's rejection
  stands for now -- but on new reasoning, since its original basis ("NFR15 already
  met") no longer holds.
- **The prerequisite is a whole-body controller.** A QP or step-MPC that allocates
  across placement, CoP and spine *simultaneously* is now the only way to answer
  whether the authority is unusable or merely unused by me. Until then, both "the
  reduced-order model is optimistic" and "the robot needs abduction" are unsupported.
- **Consequences:**
  - `regulate_along_line` stays **off** by default, with the measurement recorded so
    it is not rediscovered a third time.
  - ⚠️ **NFR15 remains not demonstrated at 28.9 mm against 48 mm**, and the honest
    statement is now: *the simulation cannot yet demonstrate it, and the limiting
    factor is known to be the controller architecture.*
  - The modelling has reached the end of what this control structure can settle.

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
