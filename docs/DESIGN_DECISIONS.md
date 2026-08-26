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
> ⚠️ **Numbers superseded by [ADR-0045](#adr-0045) (M40).** The copper-loss formula
> was `I^2 R_pp` where balanced three-phase is `1.5x` that. Corrected: copper
> **42.0 -> 63.1 W**, trot draw **83.6 -> 104.6 W**, efficiency **38.7 -> 29.6 %**,
> runtime **30.2 -> 24.1 min**, range **~905 -> 723 m**. Standing now costs **87 %**
> of moving rather than 76 %, so this ADR's brake argument gets *stronger*. Every
> qualitative finding stands; the magnitudes were low.
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
> ⚠️ **Numbers superseded, and one CONCLUSION OVERTURNED, by
> [ADR-0045](#adr-0045) (M40).** `power.py` computed copper loss as `I^2 R_pp`
> where balanced three-phase is `3 I^2 R_ph = 1.5x` that, so every temperature here
> is low. Anodised continuous goes **74.9 -> 96.1 C**, which breaks this ADR's
> headline that *"anodised, it is safe because its own equilibrium is ~75 C"*.
> Anodising is worth **more** (59 K, not 39) and is **no longer enough**; forced air
> becomes required rather than optional. The mechanism arguments all stand.
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
> ⚠️ **Superseded by [ADR-0045](#adr-0045) (M40): the gradient is 11.5 K, not
> 7.7 K**, and the whole stack sits higher -- anodised continuous winding
> **107.6 C** (was 82.6), polished **166.7 C** (was 121.4). The gradient scales with
> dissipation and M40 raised it 1.5x. The *finding* -- the finish sets where the
> stack sits, the joints set the spread -- is unchanged.
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

## ADR-0033: The viable set, computed exactly -- NFR15 is ACHIEVABLE, and the model was never the problem
- **Status:** Accepted. **Settles the M17-M27 arc. Corrects
  [ADR-0028](#adr-0028), [ADR-0031](#adr-0031) and [ADR-0032](#adr-0032).**
- **Context:** Every envelope figure in this project has been the achievement of
  *some controller*. That made the recurring question unanswerable: when a
  measurement falls short of a prediction, is the model optimistic or the controller
  poor? M27 ended with three actuators failing to deliver credited authority --
  suggestive, but not proof. `viable.py` removes the controller from the question.
- **It is exact, not optimised.** Over one stance with the CoP free inside the
  support set `S`, `xi(T) = g xi(0) - (g-1) u` with `u` the exponentially-weighted
  mean of the CoP -- and since `S` is convex, `u` ranges over exactly `S`. So the
  recoverable set follows in closed form:

      R_0 = {0},   R_(k+1) = (R_k + (g-1) S_k) / g

  a Minkowski sum of scaled polygons. `S_k` is the segment between the stance feet
  swept along `x` by the reach range -- a **parallelogram**, because the legs are
  planar (ADR-0017). Converges by 6 steps; the 1-step case matches the closed form
  to **9e-14**.
- **The result, and it reverses three conclusions:**

  | | worst direction |
  |---|---|
  | **Viable set, feet only (exact, ANY controller)** | **29.8 mm** |
  | `control.py` feet-only, 1-D | 30.3 mm |
  | MuJoCo harness measured (ADR-0028) | 28.9 mm |
  | **Viable set, + spine (exact)** | **62.7 mm** |
  | `control.py` + spine, 1-D | 52.7 mm |
  | **NFR15 requires** | **48.0 mm** |

- ⚠️ **1. The reduced-order model was never optimistic.** Its feet-only envelope is
  within **2 %** of the exact worst-direction limit, and its with-spine figure is
  **conservative** (52.7 against 62.7). ADR-0028 concluded "the foot-placement model
  is nearly right; the spine credit is what does not materialise" -- the first half
  holds, the second is **wrong in sign**.
- ⚠️ **2. The foot-placement controller is near-OPTIMAL.** 28.9 mm measured against
  a 29.8 mm true limit is **97 %**, not the "84 % of prediction" ADR-0028 read as a
  shortfall. ADR-0032's blanket indictment of "the control architecture" holds for
  the spine and **not for the feet**.
- ⚠️ **3. NFR15 is ACHIEVABLE.** 62.7 mm viable against 48 mm required. The
  authority exists; M24-M27's failure to spend it is a control problem **with a
  proven target**. Building the whole-body controller is now justified work rather
  than a hope.
- **And a geometric correction to ADR-0031.** It called the spine "authority in the
  wrong axis". Along its own axis the credit adds exactly its length (36.6 mm, to the
  millimetre). But the viable set is **slanted**, because the trot's support is a
  diagonal -- so sliding that boundary sideways moves where the fore-aft ray exits,
  and the gain in **x is larger still (63 mm)**. ADR-0031's mechanism stands (the
  spine cannot act *along* the support line); "it only helps laterally" does not
  follow from it.
- **Consequences:**
  - **Leg abduction stays rejected, now on solid ground.** ADR-0032 said "do not buy
    it because the controller cannot use what it has"; the stronger statement is that
    **the existing authority is sufficient for the requirement**. ADR-0017's original
    conclusion was right, though its stated reason had lapsed.
  - **NFR15**: achievable, **not yet demonstrated**. Those are different claims and
    the requirement table should carry both.
  - ⚠️ Still LIPM-class: constant CoM height, `dH/dt = 0`. M17 measured that as
    ~2 % **conservative**, so the bound is if anything slightly pessimistic -- but it
    is a bound within a model class, not a theorem about the robot.
  - The one thing this does not give is a controller. It gives the target to build one
    against, which is what every prior milestone lacked.

## ADR-0034: R2's critical table was stale -- NFR15 is met from mu 0.6, and NFR15 is no longer the reason for the 50 cm/s trot
- **Status:** Accepted. **Supersedes the R2 table in [OPEN_RISKS](OPEN_RISKS.md) and
  removes NFR15 as the justification in [ADR-0020](#adr-0020).**
- **Context:** [ADR-0033](#adr-0033) made the viable set computable exactly and
  instantly. The first thing worth re-deriving with it is **R2 (paw friction)** --
  one of only two CRITICAL risks, and the one whose table drove both the NFR16
  friction floor and ADR-0020's trot slowdown.
- ⚠️ **First finding: the published R2 table cannot be reproduced.** It quotes
  40.2 / 48.1 / 53.9 mm at mu 0.5 / 0.7 / 0.9. Neither `rejection_envelope(use_spine)`
  (54.1 / 67.4 / 75.7) nor `self_consistent_envelope` reproduces it -- and the latter
  **takes no `floor_mu` at all**, so it cannot produce a mu-dependent column. The
  table predates M20's 4 % sway correction and has been stale in a CRITICAL risk
  section since. **Stale numbers in the risk register are worse than missing ones:
  they are load-bearing and they look checked.**
- **Re-derived on the exact viable set** (worst over 24 directions, converged horizon):

  | stance | speed | mu 0.4 | 0.5 | **0.6** | 0.7 | 0.8 |
  |---|---|---|---|---|---|---|
  | 0.40 s | 50 cm/s | 42.6 | 47.6 | **52.6** | 57.7 | 62.7 |
  | **0.30 s** | **67 cm/s** | 42.5 | 45.3 | **48.1** | 50.9 | 53.7 |

  **NFR15's 48 mm is met from mu >= 0.6 at BOTH speeds** -- where R2 implied mu 0.70
  was needed and met "with no margin at all". At the NFR16 floor of 0.70 the margin
  is **20 %** at 50 cm/s, not zero.
- ⚠️ **Second finding: NFR15 no longer justifies the 50 cm/s trot.** ADR-0020 slowed
  the shipped gait from **67 to 50 cm/s** because the spine's friction demand
  exceeded a realistic floor and the envelope fell short. On the exact set the fast
  gait **also meets NFR15** at mu >= 0.6 -- and it is *better* on the other axis
  ADR-0020 flagged, since per-step growth is **3.21 at 0.30 s against 4.73 at
  0.40 s**, which widens the DCM-estimation margin rather than narrowing it.
- **Decision: NFR15 is removed as a reason for the slowdown; the slowdown is NOT yet
  reversed.** ADR-0020 rested on two things, and only one is now answered:
  - *the envelope falls short* -- **no longer true** on the exact model;
  - *the spine's friction cost is ~1.4 mu at full ROM* -- **still un-cross-checked**
    (ADR-0025 could not measure it without a balance controller, and ADR-0033's
    viable set inherits the same Coulomb accounting rather than testing it).

  Reinstating 67 cm/s on half an argument would repeat exactly the error this ADR is
  correcting. **The speed decision is now blocked on one specific measurement**, which
  is a better place than "blocked on a model".
- **Consequences:**
  - **R2 is downgraded from CRITICAL to SIGNIFICANT.** The drag test is still worth
    doing, but the failure threshold moved from "mu 0.70, no margin" to "mu 0.6", and
    a typical dry floor clears that comfortably. It is no longer a design-breaker.
  - **NFR16 (mu >= 0.70) is now conservative rather than exact.** Not lowered -- the
    friction accounting behind it is the thing ADR-0025 could not verify -- but it is
    no longer the razor's edge the register described.
  - ⚠️ Everything here inherits ADR-0033's LIPM class **and** ADR-0019/0020's
    friction accounting. It re-derives the *envelope* exactly; it does not
    re-derive the *friction cost*, which remains the single un-cross-checked block.

## ADR-0035: The spine's friction cost is REAL but far smaller than claimed -- and it took five failed measurements to see
- **Status:** Accepted. Partially answers the item [ADR-0025](#adr-0025) and
  [ADR-0034](#adr-0034) left as the single blocking measurement.
- **Context:** ADR-0034 removed NFR15 as a reason for the 50 cm/s trot and left the
  speed decision blocked on one thing: **is ADR-0019/0020's friction accounting
  right?** M20 could not measure it because the robot fell. The M21 harness holds a
  settled trot, so it should now be readable.
- ⚠️ **Four designs failed, and the failures are the useful part:**

  | design | what it read | why it is wrong |
  |---|---|---|
  | Per-contact `|f_t|/f_n` | pinned at the cone limit every time | a foot with 1.5 N at touchdown saturates any ratio |
  | Aggregate `|sum f_t|/sum f_n` | **3.238** | tangential at 3x normal is impossible under gravity -- impact transients |
  | Foot slip while loaded | 0.4-2.5 mm, no trend | the spine's share sits inside a ~1 mm floor from contact-point migration |
  | CoM shift, unpaired | mean 0.5-6 mm, **sd 10-15 mm** | the effect is a few mm; averaging 5 trials showed nothing |

  **Force is the wrong observable for a legged robot in contact.** Impacts dominate
  every ratio, and no threshold separates them cleanly.
- **What works: a PAIRED design.** The simulator is deterministic, so the same
  deployment phase run at two frictions differs *only* by the friction. That cancels
  the phase-to-phase variance that swamped everything else:

  | floor mu | CoM shift lost vs mu 5.0 | t (n = 11) |
  |---|---|---|
  | 0.20 | **-9.64 mm** | -2.23 |
  | 0.40 | -8.01 mm | -1.83 |
  | 0.70 | -5.71 mm | -1.32 |
  | 1.20 | -1.11 mm | -0.22 |

- **Finding 1 -- the mechanism is confirmed.** Monotone across five conditions, with
  exactly the sign ADR-0019 predicts: less friction, less achieved CoM shift.
  Internal motion really does need a ground reaction.
- ⚠️ **Finding 2 -- the cost is far smaller than ADR-0019/0020 claim.** At mu 0.70
  the loss is **5.7 mm of a 42.2 mm sway, i.e. 14 %**. ADR-0020's accounting implies
  the spine needs mu ~0.71 just to deliver its authority, i.e. near-total loss below
  that. Measured, even mu 0.20 costs only ~23 %.
- **This corroborates ADR-0034 independently.** That milestone found NFR15 met from
  mu 0.6 rather than "0.70 with no margin", from the envelope side. This finds the
  friction penalty overstated, from the mechanism side. Two different routes, same
  direction.
- ⚠️ **Significance is marginal and the decision does not move.** Only mu 0.20
  reaches `|t| > 2.1`, and `n` is capped at **11** because low-friction runs fall
  before the later phases can be sampled. The monotone ordering across five
  conditions is supporting evidence, not a substitute for power.

  **So ADR-0020's 50 cm/s stands.** ADR-0034 said reinstating 67 cm/s on half an
  argument would repeat the error it was correcting; doing it on a marginal
  `t = -2.23` would be the same mistake wearing a statistic.
- **Consequences:**
  - The friction accounting is **not refuted** -- its direction is confirmed and its
    magnitude is doubted. NFR16's `mu >= 0.70` remains as a conservative floor.
  - What would settle it: more samples at low friction, which needs the harness to
    survive longer there -- i.e. **it is now gated on controller quality again**,
    the same wall as ADR-0032.
  - ⚠️ Recorded as a method note, because it will recur: **in contact-rich
    simulation, measure displacements, and pair the trials.** Four of five designs
    here failed on impact transients or run-to-run variance, not on physics.

## ADR-0036: My envelopes were horizon-limited -- and the 2-D optimal law is not adoptable yet
- **Status:** Accepted. **Corrects the precision of every envelope figure in
  [ADR-0026](#adr-0026) through [ADR-0035](#adr-0035).**
- **Context:** ADR-0033's derivation hands over the optimal LIPM policy for free:
  `xi_next = g xi - (g-1) u` wants `u* = g/(g-1) . xi`, so when that is unreachable
  the best choice is its **projection onto the reachable set**. Every controller from
  M8 to M30 projected onto a single **axis** instead. Implementing the real thing was
  meant to close the gap to the 29.8 mm bound. It did something more useful first.
- ⚠️ **The methodological finding: the measured envelope is HORIZON-LIMITED.** The
  viable set asks *can the robot recover*; a simulation asks *does it survive N more
  steps*. Those are different questions, and the second depends on N:

  | survival horizon | measured envelope |
  |---|---|
  | 4, 6, 8 steps | 39.2 mm |
  | 12 | 34.7 mm |
  | **16, 24** | **28.6 mm** (converged) |

  **M21-M30 all used an 8-step horizon.** `control.py`'s own docstring records making
  exactly this mistake with `steps=12` in `rejection_envelope` -- *"the result was
  horizon-limited, not reach-limited, which is a different and misleading
  statement"* -- and I repeated it in the simulation without noticing.
- **What that changes, and what it does not.** Converged, the shipped controller's
  worst direction is **25.6 mm = 86 %** of the viable bound, against the **97 %**
  ADR-0033 claimed from an 8-step measurement. Still near-optimal; the number moves.
  Every converged figure sits **below** the bound, as it must -- which is a mutual
  check on the harness and the bound rather than a coincidence.
- **The 2-D projection law: better in some directions, worse where it counts.**

  | controller (16-step horizon) | 120 deg | 300 deg | worst, vs viable |
  |---|---|---|---|
  | axis (M8-M30) | 28.6 mm | 25.6 mm | **86 %** |
  | projected 2-D | 22.6 mm | **36.2 mm** | 76 % |

  **+41 % at 300 deg and -21 % at 120 deg.** Not adopted: the worst case is what a
  requirement is judged on.
- **And the reason is specific, not a tuning failure.** The projection assumes both
  degrees of freedom of the support parallelogram are available -- the fore-aft
  placement `dx` **and** where the load sits along the support line. **Only `dx` is
  actuated.** Solving a 2-DOF problem and realising 1 DOF mis-allocates: it gives up
  perpendicular authority the deadbeat law was using well, in exchange for along-line
  correction it cannot deliver.
- **Consequences:**
  - `placement_mode="projected"` ships but defaults **off**, with the decomposition
    (`dx`, `lam`) exposed so a caller that can actuate the load split may use it.
  - ⚠️ **Envelope figures now carry their horizon.** A regression test asserts a
    longer horizon is a *harder* test and that the converged value stays under the
    viability bound -- the two ways this can silently go wrong.
  - The unlock is unchanged and now sharper: **realise `lam`.** M27 measured that
    authority as available on compliant legs (linear, -39.3 mm/mm) but could not
    close a loop on it. It is the missing degree of freedom, not a missing actuator.

## ADR-0037: Four degrees of freedom, four failures -- the controller is at 86 % of optimal and the gap is NOT the feet
- **Status:** Accepted. Closes the line of work opened by [ADR-0026](#adr-0026).
- **Context:** [ADR-0036](#adr-0036) named the load split along the support line
  (`lam`) as *the* missing degree of freedom: the 2-D projection solves for it, and
  ADR-0032 measured the authority as available on compliant legs (linear,
  -39.3 mm/mm). This realises it -- planned once per stance, executed open-loop, the
  structure that fixed the spine in ADR-0030.
- ⚠️ **It makes the controller much worse.**

  | 300 deg, converged horizon | envelope |
  |---|---|
  | axis (shipped) | **25.6 mm** |
  | projected + `lam` | **0.8 mm** |

  And it took two horizon-limited readings to see. At 4 mm of differential the worst
  direction collapsed to 6.0 mm; at 1 mm it read **33.2 mm and looked like a win**,
  until the horizon was converged and it fell to 24.1 mm at 120 deg and 0.8 mm at
  300 deg. **ADR-0036's lesson, applied to ADR-0036's own successor.**
- **The pattern, stated plainly.** Four degrees of freedom have now been measured as
  physically available and then engaged in the loop:

  | DOF | static authority | effect on the loop |
  |---|---|---|
  | Spine, reactive | 36.6 mm | baseline 5x worse; falls at gain 0.5 |
  | Spine, planned | 36.6 mm | stable, **+0.23 mm** of envelope |
  | CoP, reactive | +/-109 mm | **+1.8 mm**, baseline 4x worse |
  | Load split `lam`, planned | +/-109 mm | **worst case 25.6 -> 0.8 mm** |
  | **Foot placement** | 30.3 mm | **the one that works: 86 % of the bound** |

  **Only the actuator the controller was designed around delivers.** Four
  independent attempts, four different mechanisms, one common factor.
- **And the useful reframing: 86 % is a good controller.** The feet reach 25.6 mm
  against a 29.8 mm feet-only bound. The remaining 4.2 mm is not where NFR15's gap
  lives. **The gap is entirely the spine credit** -- 62.7 mm viable *with* the spine
  against 25.6 mm achieved -- and four attempts say a hand-designed per-step
  controller does not reach it.
- **Decision: stop adding degrees of freedom to this controller.** The next honest
  step is a genuine simultaneous optimisation (whole-body MPC over the step horizon,
  with contact and friction constraints), or accepting the feet-only capability and
  revisiting NFR15. **Incremental additions have been tried four times and the result
  has been the same each time.**
- **Consequences:**
  - `placement_mode="projected"` and `realise_lambda` ship, both **off**, so the
    finding is reproducible rather than folklore. A test asserts `lam` still hurts,
    and says to reopen this ADR if it ever stops.
  - ⚠️ **The modelling arc M17-M32 is complete for this architecture.** What it
    established: the reduced-order model is **sound** (ADR-0033), the feet-only
    controller is **near-optimal**, NFR15 is **achievable but needs the spine**, and
    the spine is **not reachable by this class of controller**.

## ADR-0038: Torque control makes the contact force a decision -- and names why a diagonal stance cannot be held
- **Status:** Accepted. First step of the whole-body controller
  [ADR-0037](#adr-0037) called for.
- **Context:** ADR-0037 ended four attempts to give a per-step position controller
  extra degrees of freedom. The common cause is structural, not tuning:
  **position servos do not command force.** You command where the foot goes and the
  ground reaction is whatever the contact and leg compliance produce. Every "allocate
  the load between the feet" scheme in M21-M32 commanded a *proxy* -- differential
  leg extension -- and hoped the force followed. It did statically (-39.3 mm of CoP
  per mm, ADR-0032); in the loop it fought the placement it was meant to help.
- **`wbc.py` makes the force a decision variable.** The DCM law asks for a centre of
  pressure; `allocate` finds foot forces producing the required net wrench inside the
  friction cones; `stance_torque` maps them back with `tau = -J^T f`. Six variables,
  a regularised least-squares and a closed-form cone projection -- **not** a solver
  call, because it runs every timestep.
- **Gate passed, on the static case.** Standing on a diagonal pair, CoP commanded
  under the CoM: forces sum to **39.6795 N against a 39.681 N** weight, net moment
  under 0.01 N.m, and the resulting CoP lands at **(0.1030, 0.0002)** against a CoM
  at (0.1031, 0). Torque control then holds the stance with **sub-millimetre CoP
  error** for about a second.
- **Two things had to be added, and both are worth recording:**
  - ⚠️ **Height must be regulated.** Commanding exactly `m g` vertically balances
    the weight and regulates nothing -- the first run drifted **0.165 -> 0.185 m in
    0.6 s** with no disturbance. LIPM *assumes* constant CoM height; a torque
    controller has to **make** it true.
  - ⚠️ **`p = c` is a neutral command, not a balance law.** With the CoP under the
    CoM, `xi_dot = c_dot` -- the DCM simply runs. The law has to be
    `p = xi + k (xi - ref)`.
- ⚠️ **And the finding: a diagonal stance is not holdable, and now that is
  measurable.** Two point contacts confine the CoP to the **segment between them** --
  a trot has no support polygon, only a support **line**. A DCM law commanding a free
  2-D point asks for something no allocation can deliver, and the regularised solve
  quietly returns the nearest thing instead of failing. `realisable_cop` clamps it,
  and the residual is the signal:

  | t | CoP demanded off the segment |
  |---|---|
  | 0.25 s | 0.5 mm |
  | 0.75 s | 17.5 mm |
  | 1.00 s | **104.6 mm** |
  | 1.25 s | **591 mm** |

  **That residual is a "you must step now" measure**, and it is the quantity M20 and
  M30 were missing when they tried to hold a stance open-loop. The robot does not
  fall because the force allocation is poor; it falls because it is being asked for a
  centre of pressure that does not exist.
- **Consequences:**
  - `wbc.py` ships with five tests gating the static case, the cone projection, the
    height requirement, and the segment confinement.
  - **Next is integration**, not more allocation: drive the existing gait from the
    infeasibility residual so a step is taken when the CoP demand leaves the segment.
    That is the whole-body controller ADR-0037 asked for, and the allocation half of
    it is now built and gated.
  - ⚠️ Nothing here moves the bound. ADR-0033's viable set (29.8 mm feet-only,
    62.7 mm with the spine) stands, and the honest test of this work remains whether
    it beats the **25.6 mm** the shipped position controller already achieves.

## ADR-0039: Step timing is the fifth degree of freedom to fail -- and the first where the HARNESS is what fails

- **Status:** Accepted. Closes the M34 item [ADR-0038](#adr-0038) opened.
  **`adapt_timing` is built, gated, and NOT adopted.**
- **Context:** ADR-0038 built a whole-body force allocation and produced, as a
  by-product, the quantity every earlier attempt lacked: the distance by which the
  demanded centre of pressure falls **outside the support segment** a diagonal stance
  actually has. It named the integration as next -- drive a step from that residual.
  This is that work.
- **What was built** (`mjsim.py`):
  - `cop_residual` -- ADR-0038's residual, read from live contact geometry through
    `wbc.realisable_cop` rather than predicted.
  - `plan_stance_time` -- a **closed form**, not a fit. The segment has zero extent
    across itself, so the perpendicular DCM offset is the part no force allocation can
    balance; with the CoP pinned on the segment it grows as `e0 exp(omega t)` and the
    law demands `(1 + k)` times it, giving
    `T* = ln( tol / ((1 + k) |e0|) ) / omega`. That is ADR-0038's
    0.5 -> 104.6 -> 591 mm table, in closed form.
  - `swing_time_floor` -- so a shorter stance has to be one the **leg** can swing
    through, rather than a free win the simulator hands out.
  - `run(..., until=<seconds>)` -- a **time**-terminated horizon.
- ⚠️ **The methodological point, and it had to come first.** Sixteen re-timed stances
  are less time on the floor than sixteen nominal ones, so a step-count horizon
  rewards a controller for **stepping faster rather than balancing better**. That is
  [ADR-0036](#adr-0036)'s horizon error wearing different clothes, and measuring
  variable timing against a step count would have manufactured the result. Every
  figure below is at an equal **3.2 s**.
- **First reading -- it looks like the first success in five attempts:**

  | controller (equal 3.2 s) | 120 deg | 300 deg | T_mean |
  |---|---|---|---|
  | axis, fixed timing (shipped) | 28.6 | **25.6** | 0.200 |
  | + residual timing, tol 5 mm | 28.6 | 28.6 | 0.116 |
  | + residual timing, tol 10 mm | 33.2 | **31.7** | 0.117 |
  | + residual timing, tol 20 mm | 34.7 | 31.7 | 0.138 |

  The baseline reproduces ADR-0037's published 28.6 / 25.6 mm exactly, so the harness
  and the new horizon are sound. Worst direction **25.6 -> 31.7 mm, +24 %**.
- ⚠️ **Finding 1 -- the trigger is not what produces it.** `T_mean` sits at 0.117 s
  against a 0.100 s floor, so the planner saturates almost immediately; consistent
  with that, the tolerance barely moves the answer across a 4x sweep. The control is
  a **fixed** stance at the same duration, no trigger at all:

  | fixed stance | 120 deg | 300 deg | worst | viable bound | undisturbed drift |
  |---|---|---|---|---|---|
  | **0.200 s** (shipped) | 28.6 | 25.6 | **25.6** | 29.8 | **4.99 mm** |
  | 0.140 s | 37.7 | 39.2 | **37.7** | 36.5 | 5.12 mm |
  | 0.117 s | 19.6 | 60.3 | **19.6** | 39.5 | **9.34 mm** |
  | 0.100 s | 37.7 | 40.7 | **37.7** | 41.9 | **9.17 mm** |
  | residual timing | 33.2 | 31.7 | **31.7** | 39.5 | 9.34 mm |

  **Doing nothing clever beats it: 37.7 mm against 31.7.** The gain is the smaller
  per-step growth of a faster trot -- `e^(omega T)` falls **4.73 -> 2.48** between
  0.200 and 0.117 s -- which every controller gets for free. The residual logic
  contributes nothing on top, and costs 6 mm.
- ⚠️ **Finding 2 -- and the re-timed numbers are not trustworthy either.** The worst
  direction reads **25.6 -> 37.7 -> 19.6 -> 37.7 mm** across stance. The mechanism is
  monotone in stance; the measurement is not. The reason is in the last column: the
  **undisturbed drift nearly doubles**, from 4.99 mm at the shipped stance to
  9.3 mm below 0.117 s. M21 set the gate for this project in its own words -- *a
  harness whose undisturbed drift is the same order as the disturbance it is
  measuring cannot adjudicate anything* -- and short-stance measurement fails it.
- **What is NOT claimed.** The 0.140 s row reads 37.7 mm against a 36.5 mm exact
  viability bound. That looks like an impossibility, and it is not one: seven
  bisections on a 1.5 m/s bracket resolve to ~1.5 mm, so a 1.2 mm excess is inside
  one step. Recorded because it would have been an attractive headline.
- ⚠️ **Finding 3 -- the cost side settles it regardless.**
  `control.spine_friction_cost` scales as `1/stance^2`:

  | stance | mu demanded, full ROM |
  |---|---|
  | 0.200 s (shipped) | **0.71** |
  | 0.140 s | 1.44 |
  | 0.117 s | **2.07** |

  mu 2.07 is not a floor. A shorter stance is the **most expensive** currency this
  robot has for buying balance, in exactly the coin [ADR-0020](#adr-0020) slowed the
  trot from 67 to 50 cm/s to protect. ([ADR-0035](#adr-0035) doubts that accounting's
  magnitude by ~7x, so this overstates the level -- not the direction.) Foot speed,
  the constraint one would expect to bind, does **not**: the swing needs 1.20 m/s
  mean against 4.10 m/s spare, which is why `swing_time_floor` is in the planner and
  is never the binding clamp.
- **Decision: do not adopt. `adapt_timing` defaults to False**, alongside
  `placement_mode="projected"` and `realise_lambda`, so the finding stays
  reproducible rather than becoming folklore.
- **Consequences:**
  - Five degrees of freedom have now been added to this controller and five have
    failed: reactive spine, planned spine, reactive CoP, load split `lam`, and step
    timing. ⚠️ **But the fifth failed differently.** The first four were measured
    cleanly and were genuinely worse. This one **cannot be measured** in the harness
    as built -- which makes "the architecture is the limit"
    ([ADR-0032](#adr-0032)) an unsafe thing to keep repeating.
  - ⚠️ **The next honest step is the harness, not the controller.** Its noise floor
    is a function of the gait parameters, and nothing in M21-M34 checked that. Until
    it is flat across stance, no re-timed gait can be evaluated at all.
  - `run(until=...)` is now the correct way to measure anything with variable timing,
    and the step-count form should be treated as valid only at a fixed stance.
  - Gated by `test_the_noise_floor_doubles_at_a_short_stance`, which is deliberately
    written to **fail if the harness improves** -- at which point M34 should be re-run
    rather than trusted.
  - ⚠️ Nothing here moves ADR-0033's bound. NFR15 remains achievable (62.7 mm) and
    undemonstrated (25.6 mm), and the shipped controller is unchanged.

## ADR-0040: The harness measures SURVIVAL, not recovery -- and the bound it was checked against measures recovery

- **Status:** Accepted. **Corrects the interpretation of every simulation envelope
  from [ADR-0026](#adr-0026) (M21) through [ADR-0039](#adr-0039) (M34).** The figures
  are not withdrawn; what they *mean* is.
- **Context:** ADR-0039 set M35 as an instrument milestone: the undisturbed drift
  doubles at short stances, so flatten it before evaluating any re-timed gait. That
  work succeeded and then found something larger on the way out.
- **Finding 1 -- the floor is loop gain, not plant.** Disabling the placement
  correction entirely and re-running says the plant is not what degrades. Sweeping
  the gain says what does:

  | stance | deadbeat | x0.5 | x0.75 | x1.0 | x1.25 |
  |---|---|---|---|---|---|
  | 0.200 s | 1.268 | 3.53 | 3.70 | **4.99** | 5.00 |
  | 0.117 s | 1.675 | **3.96** | 7.38 | **9.34** | 14.03 |

  A deadbeat law has no phase margin to spare -- it asks for the whole correction in
  one step, so any lag beyond the one step it models is uncompensated. The lag is
  fixed (7.5 ms pipeline plus the `kp = 80` servo); the stance is not. As the stance
  shortens the lag grows as a **fraction** of it and the loop chatters.
- **Finding 2 -- and the shipped controller is over-geared at its own stance.** A
  constant `placement_gain = 0.5` flattens the floor across 0.100-0.200 s *and*
  improves the nominal stance, **4.99 -> 3.53 mm**. So M35's stated goal was
  reachable.
- ⚠️ **Finding 3 -- but flattening it produced an impossibility, and that is the
  milestone.** Detuned at a 0.117 s stance the harness certifies **42.2 mm against a
  39.5 mm exact viable bound** -- 6.8 % over, well outside the ~1.5 mm bisection
  resolution. No controller beats the viable set, so the **measurement** is wrong.
- ⚠️ **The cause is the success criterion, and it is not new.** `run` passes a trial
  when the CoM never drops below 0.11 m inside the horizon: **did not fall**.
  `viable.py` computes what the robot can **recover** from. Those are different
  quantities and this project has been comparing them to each other since M21 --
  including in the `measured <= bound * 1.02` consistency check, which held only
  because at the shipped configuration the two happen not to cross.

  Probed at its own certified envelope, every configuration but one is still
  displaced when the horizon ends:

  | configuration | kick | settled DCM | own floor | ratio |
  |---|---|---|---|---|
  | **shipped**, 300 deg | 25.6 mm | **26.2 mm** | 3.92 mm | **6.7x** |
  | **shipped**, 120 deg | 28.6 mm | **83.6 mm** | 3.92 mm | 21.3x |
  | detuned nominal | 15.1 mm | 51.1 mm | 3.13 mm | 16.3x |
  | 0.140 s, shipped gain | 37.7 mm | 11.1 mm | 4.51 mm | 2.5x |
  | 0.117 s, detuned | 42.2 mm | 5.3 mm | 3.79 mm | **1.4x -- a real recovery** |

  **The shipped controller ends its certified 25.6 mm trial 26.2 mm off its
  support.** It did not recover; it did not fall.
- **Finding 4 -- re-measured on a like-for-like basis, the envelope collapses.**
  `measure_envelope(recover=True)` requires the trial to return to within 2x the
  configuration's **own** undisturbed drift:

  | configuration | survival | **recovery** | bound |
  |---|---|---|---|
  | shipped (0.200, gain 1.0) | 25.6 | **1.5** | 29.8 |
  | detuned (0.200, gain 0.5) | 15.1 | 3.0 | 29.8 |
  | 0.140 s, shipped gain | 37.7 | 0.0 | 36.5 |
  | 0.117 s, detuned | 42.2 | **42.2** | 39.5 |

  1.5 mm is **one bisection quantum** -- effectively zero.
- **The mechanism is steady-state error.** The placement law arrests a topple but
  carries no term that removes a *persistent* DCM offset, so it settles into a biased
  limit cycle. That is precisely the failure this project already documents for
  at-DCM placement -- *"stable, and walking away sideways"* ([ADR-0013](#adr-0013)) --
  and the shipped deadbeat law has it too, smaller and therefore unnoticed.
- ⚠️ **Consequence for the headline claim.** ADR-0037's *"the controller is at 86 % of
  optimal"* and ADR-0033's *"97 %"* compare a **survival** measurement against a
  **recovery** bound. On a like-for-like basis the shipped controller is nowhere near
  the bound. **The four-DOF indictment of ADR-0032 also weakens**: those DOFs were
  compared to each other on the survival criterion, which is self-consistent, but
  "only foot placement delivers" was never tested against recovery at all.
- ⚠️ **An open contradiction, recorded rather than resolved.** The 0.117 s detuned
  configuration recovers -- genuinely, 1.4x its floor -- from **42.2 mm against a
  39.5 mm feet-only bound**. Under the recovery criterion those are now the same
  quantity, so one is wrong. Candidates: the bound reuses the nominal plant's `reach`
  at a stance where the real reach differs; or the LIPM basis fails there.
  [ADR-0022](#adr-0022) put LIPM/MuJoCo agreement at ~2 %, and this is 6.8 %.
  **This is M36 and it is the highest-value item in the arc**, because whichever way
  it resolves, something load-bearing is wrong.
- **Consequences:**
  - `measure_envelope(harness, angle, recover=...)` and `undisturbed_drift` ship in
    `mjsim`. `recover=False` reproduces the historical criterion, so every published
    figure stays reproducible; `recover=True` is what should be compared to
    `viable.py`.
  - `placement_gain` ships, default **1.0 -- the shipped controller is unchanged.**
    Detuning is not adopted: it flattens the floor but costs survival envelope at the
    nominal stance (25.6 -> 15.1 mm), and M35 is not the milestone to trade that on.
  - ⚠️ **Nothing here is a reason to trust the reduced-order model less.** `control.py`
    maps `placement_gain = 0.5` to `beta > 1` and predicts a **zero** envelope for it;
    the sim recovers from 42.2 mm at that gain. That disagreement is part of the M36
    contradiction, not a separate one.
  - Gated by `test_the_envelope_measures_SURVIVAL_not_recovery`, written to **fail
    once the controller gains integral action** -- at which point re-measure.
  - ⚠️ NFR15 is unchanged in requirement and worse in status: **not demonstrated**
    now means not demonstrated by a wider margin than recorded.

## ADR-0041: Drawn as real parts, the leg does not close -- and the spec has contradicted itself since ADR-0010

- **Status:** Accepted. First **manufacturing-level** geometry in the project;
  partly closes the ASSEMBLY_SPEC §6 debt. **Three spec sections corrected, one
  requirement at risk.**
- **Context:** `tomcat_skeleton.py` says of itself *"still a SKELETAL model, not a
  manufacturing model -- no fasteners, bearings, tolerances or fabrication
  features"*, and ASSEMBLY_SPEC §6 owed *"shop drawings / manufacturable
  geometry"*. Thirty-five milestones of modelling had not produced a part anyone
  could make. `cad/tomcat_leg_detail.py` is one hind leg drawn as parts: bonded
  inserts with a modelled glue line, clevis/tongue joints with H7 bearing bores and
  h6 shafts, turned sheaves whose groove pitch line **is** the tendon moment arm,
  the §0.1 root idler, the ankle return spring, the tactile pad. It exports
  STEP/STL and, more usefully, **checks its own dimensions**.
- ⚠️ **Finding 1 -- LEG_TENDON_SPEC §1.1 has been stale since ADR-0010, and it is
  the table that sizes the bones.**

  | | §1.1 as written | live `torque_budget` at 4.045 kg |
  |---|---|---|
  | hip land | 12.36 N.m | **16.67 N.m** |
  | stifle land | 7.49 N.m | **10.23 N.m** |
  | hock land | 4.79 N.m | **6.46 N.m** |
  | hip tension | ~447 N | **600 N** |

  The ratio is exactly the ADR-0010 mass increase, **4.045 / 3.0 = 1.35**. §2 *was*
  re-run -- it says "~600 N at the hip land transient", which is 16.67 / 0.028 --
  so the document has contradicted itself for ten milestones. §1.3, §1.3a, §3.5 and
  ASSEMBLY_SPEC §0.1 all derive from §1.1 and inherit the error.
- ⚠️ **Finding 2 -- so the link sizing is not what it claims.** §3.5 chose
  Ø12/Ø10/Ø8 x 1.0 to equalise the safety factor at **2.84 / 3.10 / 2.87**.
  Re-derived from the live torques *and* the torsion the sheave's real lateral
  offset imposes (12.2 / 12.2 / 9.2 mm, which the 3D layout produces rather than
  assumes), the same sections give **1.97 / 2.08 / 1.84** -- the femur and
  metatarsus below the **SF 2 floor** §0.1's argument explicitly rested on.
- **The remedy is nearly free, which is the useful part.** Bending strength goes as
  the *cube* of diameter, tube mass only as the first power: **Ø12→Ø14, Ø10→Ø12,
  Ø8→Ø10** restores **SF 2.78 / 3.16 / 3.11** for **under 4 g** on the whole leg.
- ⚠️ **Finding 3 -- the moment-arm trade closes SHUT.** §1.2 grew the arms to cut
  cable tension and §1.3a priced only the *ankle's inertia*, from a 6 g-at-14 mm
  estimate scaling as `r^2`. Turned as real parts the three sheaves are **41 g of a
  110 g leg** -- so there is now a reason to want them smaller, and no room:

  | arm scale | sheave set | T land | T trot | trot / motor peak |
  |---|---|---|---|---|
  | **1.00** (shipped) | **41.3 g** | 615 N | 198 N | **81 %** |
  | 0.85 | 32.9 g | 720 N | 230 N | **101 %** |
  | 0.70 | 25.4 g | 870 N | 275 N | 133 % |

  81 % agrees with §2's independently derived "0.82x peak", which is what makes the
  model credible. **The arms are pinned by the actuator**, so the sheave mass cannot
  be traded away.
- ⚠️ **Finding 4 -- and therefore the leg does not close. NFR5 is at risk.** Drawn
  as parts the leg is **~160 g against `DEFAULT_LEG.link_mass`'s 110 g (146 %)**.
  Bearings (48 g), sheaves (41 g) and clevises (41 g) are **82 %** of it and none is
  negotiable -- the arms by the motor peak above, the bearings by ASSEMBLY_SPEC §2's
  static C0 >= 1.5 kN, the clevises because they carry the bores. **+50 g x 4 legs =
  +200 g on a 4.045 kg body, so NFR5's 4.05 kg breaks by ~5 %** -- and a heavier
  body raises every torque, which is exactly the ADR-0010 spiral.
- **Finding 5 -- two fabrication rules did not survive contact with the geometry.**
  The **20 mm bonded-insert rule** fills 81 % of a metatarsus with aluminium (§0.2's
  own 15 mm / SF > 10 point is used instead), and the **paw phalanx cannot be a
  bonded tube at all** -- 25 mm of span leaves 10 mm after joint hardware where two
  inserts plus a gap need >= 14 mm, so it is a solid turned or printed stub, a
  fabrication method §1's table does not list. Inserts must also be turned
  **hollow**: a solid Ø9.9 x 20 plug is 4.2 g against a 4.8 g femur tube.
- **What passed.** The full **ROM sweep is clean** -- worst non-adjacent link
  clearance **+15.4 mm**, so the joint ranges need no mechanical hard stop for
  self-interference. Bond gaps land in §2's 0.05-0.15 mm window by construction,
  every shaft is >= 4 mm, and the sheave pitch radii **are** the moment arms the
  kinematics model uses, so the CAD cannot drift from the torque budget.
- ⚠️ **Two of this pass's own findings were its own errors, corrected here rather
  than shipped:** the first layout seated the joint bearings *inside* the clevis gap
  instead of in the arm bores, widening every joint by 2x a bearing width and
  pushing the sheave 6 mm further outboard; and the trade table first compared the
  **land** transient to the motor peak, reading 162 %, which is precisely the
  conflation ADR-0008 exists to prevent (the x2.5 single-leg landing is outside the
  actuator envelope and sizes cable, pulley and bearing only).
- **Decision: adopt the geometry, correct the specs, and escalate the mass.** The
  section increase and the fabrication changes are cheap and are taken. **The 50 g
  leg overrun is not a CAD problem and is not fixed here** -- it is a budget
  decision that belongs with the whole-body mass model.
- **Consequences:**
  - `mechanical/cad/tomcat_leg_detail.py` ships with STEP/STL and a self-check;
    `tests/test_leg_detail.py` gates all nine findings. **Several assert the
    defect**, so they fail when the spec text is fixed -- that failure is the
    signal to update the spec, not to relax the test.
  - LEG_TENDON_SPEC §1.1 / §1.2 / §3.1 / §3.5 and ASSEMBLY_SPEC §6 carry
    correction banners rather than edited-away numbers.
  - ⚠️ **NFR5 (4.05 kg) is flagged at risk**, pending a whole-body re-run with the
    real leg hardware mass. Every mass-derived result -- the torque budget, the
    thermal duty, the runtime -- sits downstream of it.
  - ⚠️ **A Ø60 hip sheave is a packaging question this file cannot answer.** It
    excludes the sheaves from its interference sweep because they sit laterally
    offboard of the bone plane by construction; what they may foul is the
    **girdle**, and that belongs to the packaging study.
  - The next mechanical step is the **BOM with real part numbers**, which is now
    the only thing between this and a quotable leg.

## ADR-0042: The tendon drive, routed -- and the tendon map is COUPLED where the model says it is diagonal

- **Status:** Accepted. **Corrects `TendonMap.cable_lengths`, LEG_TENDON_SPEC §1.4
  and §3.4, and a minimum-bend violation shipped by M36.**
- **Context:** [ADR-0041](#adr-0041) drew the leg as manufacturable parts -- sheaves,
  clevises, bearings, bonded inserts -- and **did not draw a tendon.** No cable, no
  spool, no anchor, no antagonistic pairing. Design principle **P1** is the premise
  of the whole robot and it was the one thing the manufacturing model omitted; what
  M36 produced was a linkage with pulleys bolted to it. `tendon_route.py` +
  `leg_tendons.py` are the routing, solved rather than sketched: five cable runs and
  three girdle motors per leg (ADR-0002/ADR-0008), each tendon a **belt problem**
  over signed-radius common tangents and arcs.
- ⚠️ **THE FINDING -- via-pulleys couple the joints, and the model's map is
  diagonal.** A distal tendon has to get past the proximal joints. The standard fix
  is a via-pulley **concentric with the proximal axis**: the centre distance to the
  next joint is then the link length, which does not change when the proximal joint
  rotates, so the *tangent* term is invariant. It does not kill the *arc* term -- the
  **wrap** on the via-pulley changes with the proximal angle, and an arc on a pulley
  of radius `r_via` contributes exactly `r_via` per radian.

  Measured off the routed geometry by central differences, `d(cable)/d(joint)` in
  mm/rad:

  | | hip joint | knee joint | ankle joint |
  |---|---|---|---|
  | **hip tendon** | **28.00** | 0 | 0 |
  | **knee tendon** | **8.75** | **25.00** | 0 |
  | **ankle tendon** | **-8.75** | **-8.75** | **14.00** |

  The diagonal is exact -- the sheaves deliver the moment arms the torque budget
  assumes, because the cable leaves each sheave tangentially. **The off-diagonals
  are exactly the via-pulley radius.** As a fraction of each tendon's own arm:
  **35 %** for the knee, and **62.5 % twice** for the ankle.
  `TendonMap.cable_lengths` is `delta = r * q` -- a diagonal map -- so none of it is
  modelled.
- ⚠️ **And it cannot be designed away.** 8.75 mm is not a sizing choice: it is the
  cable's own minimum bend radius, 10 x Ø1.75 (§2) -- the same rule that forced the
  spool from 8.0 to 8.75 mm. A smaller via-pulley would fatigue the UHMWPE. The
  coupling is a property of routing a tendon past a joint at all, and the honest
  options are to **model it** (the map becomes lower-triangular) or to accept a
  standing disturbance of the size above.
- **Consequence 1 -- torque resolution.** `tau = -J^T T`, and with `J`
  lower-triangular `J^T` is upper-triangular: the knee and ankle tendon tensions
  both produce **hip** torque. At the land-case peaks (600 / 414 / 467 N) that term
  is `8.75 x (414 + 467) = 7.7 N.m` against the hip's own 16.67 -- a **~46 %
  perturbation**, helping or opposing depending on the routing senses. `resolve()`
  puts it at zero.
- **Consequence 2 -- §1.4's spool travel is wrong for two of three joints.** It
  sized travel as `r x ROM` per joint, giving 117 / 65 / 44 mm and calling the hip
  the sizing case. With coupling the worst-case travels are **117 / 102 / 104 mm**:
  the knee understated by 56 %, the ankle by 135 %, and the ankle's parasitic travel
  (59.6 mm) larger than its own (44.0 mm). The hip *does* remain the sizing case --
  but §1.4 implies the three spools differ by 2.7x and they differ by 13 %.
- **Consequence 3 -- §3.4 over-estimates the capstan penalty.** It worked the ankle
  path out at **1.87x** from assumed wraps summing to 360 deg. Solved, the wraps sum
  to ~108 deg and the penalty is **~1.21x**. The routing is *better* than the spec
  feared, and the motor-side tension margin it was inflating can come back. Run
  lengths land within ~35 % of §3.3's estimates (135 / 192 / 270 against
  100 / 220 / 300).
- ⚠️ **Consequence 4 -- two params are still the superseded values.**
  `motor_spool_radius` is **0.008** where §2 requires **0.00875**, so every motor
  angle and motor torque the model computes is off by 9 %. And `cable_diameter`,
  `cable_break_strength`, `cable_stiffness` -- proposed in §5.2 -- were never added
  to `TendonParams` at all.
- **A minimum-bend violation M36 shipped, found and fixed.** `idler()` was
  `pitch_r = 5.0` (Ø10) against the cable's Ø17.5 minimum -- **43 % under**, and it
  would fatigue the cable at the one station that sees full tension on every step.
- **What this makes concrete about P1.** Drawn as parts, the three leg motors are
  **395 g and sit in the girdle**; the tendon that carries their 600 N into the limb
  is **3.3 g of UHMWPE**. That ratio is the entire argument for tendon drive, and it
  is now a measured number in the model rather than a claim in a trade study.
- ⚠️ **Four of this pass's own errors, corrected rather than shipped:**
  - the common-tangent **sign** (`n.(c2-c1) = R1-R2`, not `R2-R1`) -- the crossed
    belt read 105.83 mm where the closed form is 97.98;
  - leaving every wrap **sense** at `+1`, which sent cables the long way round:
    **339 deg** of wrap on a redirect pulley and a capstan of **3.07x**, read as a
    physics result when it was a routing mistake;
  - letting the minimum-wrap search re-run **inside** the finite difference, so it
    straddled a discontinuity and the ankle row read **678 mm/rad**;
  - a mass **double-count** (tendons weighed as both steel spring and UHMWPE), and a
    claim that the ankle overtakes the hip as the spool sizing case, which its own
    test refuted -- 103.5 against 117.3 mm.
- **Decision: adopt the routing; correct the specs; do NOT silently change
  `TendonMap`.** Making the map lower-triangular changes every tension, torque and
  motor angle in the project, so it is a milestone with a re-run attached, not an
  edit.
- **Consequences:**
  - `mechanical/cad/tendon_route.py` (the belt solver, closed-form verified) and
    `leg_tendons.py` (this leg's five runs) ship; the cables, spools, motors and
    anchor pins are in `tomcat_leg_detail.py`'s STEP.
  - `tests/test_tendon_route.py` gates twelve findings. **Several assert the
    defect** -- the diagonal map, the 0.008 spool -- so they fail when the fix
    lands, which is the signal to re-run the budget.
  - LEG_TENDON_SPEC §1.4 / §3.4 / §5.2 carry correction banners.
  - ⚠️ **The next mechanical step is unchanged and now better justified:** re-run the
    whole-body budget. It needs ADR-0041's 160 g leg *and* this coupling, and both
    move tension.

## ADR-0043: The mass spiral closes at 4.30 kg -- NFR5 breaks, nothing else does, and the tendon drive gives back 62 % of its own inertia argument

- **Status:** Accepted. **NFR5 must move 4.05 -> 4.31 kg.** Closes the M38 item
  [ADR-0041](#adr-0041) and [ADR-0042](#adr-0042) both pointed at.
- **Context:** ADR-0041 measured 167 g of hind-leg hardware against
  `LegParams.link_mass`'s 110 g; ADR-0042 measured a tendon map that is coupled
  where the model is diagonal. `total_mass = trunk_mass + sum(leg masses)`, so the
  first propagates straight into body mass, and body mass drives every foot support
  force, joint torque and cable tension. ADR-0010 warned this spiral converges
  **only because the chosen motor has headroom.** This is the spiral, re-run with
  measured inputs.
- **Finding 1 -- it closes, and NFR5 is what breaks.**

  | | measured | params | ratio |
  |---|---|---|---|
  | hind leg | **167.2 g** | 110.0 g | 1.52x |
  | fore leg | **167.4 g** | 95.0 g | 1.76x |
  | trunk (incl. 19 motors) | 3635 g | 3635 g | — |
  | **BODY** | **4.304 kg** | 4.045 kg | **1.064x** |

  **NFR5's 4.05 kg is exceeded by 6.3 %.** A domestic cat is 4-5 kg, so the target
  is not physically wrong -- it is simply no longer the number.
- **Finding 2 -- ADR-0010's argument holds: every design gate still passes.**

  | gate | at 4.304 kg | limit | |
  |---|---|---|---|
  | motor peak, **trot** (the actuator case) | 1.56 N.m | 1.95 | **80 %** |
  | cable SF on the land transient | 4.70 | >= 4.0 | pass |
  | bearing static C0 needed (2xT) | 1277 N | <= 1500 | pass |

  The overrun costs **margin, not viability** -- which is precisely the headroom
  ADR-0010 said the spiral depends on, being spent.
- ⚠️ **Finding 3 -- the joint hardware gives back 62 % of the P1 inertia saving.**
  Leg swing inertia about the hip rises **+61.7 %**, because the hardware is
  distributed *along* the limb rather than centralised:

  | mass share, proximal -> distal | femur | tibia | meta | paw |
  |---|---|---|---|---|
  | params (assumed) | 47.3 | 30.0 | 15.5 | 7.3 |
  | **measured** | **39.5** | **35.3** | **20.7** | 4.5 |

  `link_mass` justifies its distribution as *"proximal-heavy because both feline
  anatomy and the ADR-0003 tendon drive push mass toward the body"*. **The tendon
  drive pushes the MOTORS toward the body. It does not push the PULLEYS there.**
  The metatarsus more than doubles. ADR-0003 accepted the entire cable-tension
  burden to buy low limb inertia, and the sheaves take most of it back.
- **Finding 4 -- and it does NOT cascade, for a reason already in the record.** The
  balance envelope moves only **52.7 -> 51.9 mm (-1.6 %)**, actuation 40.8 ->
  42.6 ms, and NFR15's 48 mm still clears. The swing is **speed**-limited, not
  acceleration-limited -- exactly what
  `test_the_ramp_barely_moves_the_envelope` established in M12. So the +62 %
  inertia is real and its downstream cost is small.
  ⚠️ The inertia ratio is a first-order proxy: the real term is
  `Lambda = (J M^-1 J^T)^-1` minimised over foot-acceleration directions, which
  needs per-link inertia tensors the model does not carry.
- **Finding 5 -- the fore/hind leg asymmetry essentially disappears.** `params.py`
  carries 95 g fore against 110 g hind, an assumed **1.16x**. Measured, both are
  ~167 g (**1.00x**): the joint hardware dominates and it is the *same* hardware on
  both, so the shorter fore links barely register. Design review **F2** settled the
  fore/hind weight split using that assumed asymmetry.
- **Finding 6 -- ADR-0042's coupling, priced.** With `J` lower-triangular, `J^T` is
  upper-triangular and the distal tendons load the proximal joints:

  | tendon | diagonal model | **coupled** | delta |
  |---|---|---|---|
  | hip | 633 N | 597 N | -5.7 % |
  | **knee** | 435 N | **607 N** | **+39.5 %** |
  | ankle | 491 N | 491 N | 0 |

  Cable SF on the worst coupled tension is **4.94**, so §2's target of 4 still
  clears. Again: margin, not viability.
- ✅ **Finding 7 -- and the wrap senses are a LOAD lever, which is free margin.**
  The off-diagonal *signs* come from the wrap senses, and `leg_tendons.route`
  currently picks them for minimum **wrap**. Picking them for minimum **load**
  instead moves the worst tension **607 -> 562 N**, i.e. cable SF **4.94 -> 5.34**.
  Eight per cent of margin for a routing decision that costs nothing. **The routing
  objective should be load, or a trade against wrap -- not wrap alone.**
- **Decision: raise NFR5 to 4.31 kg and re-publish downstream, adopt the measured
  link masses, and re-target the routing objective.** The three design gates
  passing is what makes this a bookkeeping update rather than a redesign.
- **Consequences:**
  - **NFR5: 4.05 -> 4.31 kg.** ⚠️ Everything mass-derived must be re-run:
    ADR-0021's power and runtime (83.6 W, ~30 min), ADR-0023/0024's thermal duty,
    the whole-body budget's spine torques, and every envelope figure whose plant
    carries `body.total_mass`.
  - `LegParams.link_mass` should become the measured per-link tuple, and the
    "proximal-heavy" rationale in its docstring is **wrong as written** -- it
    describes where the motors go, not where the pulleys go.
  - `LegParams` fore/hind asymmetry is now ~1.0, and **F2's split needs re-checking**.
  - ⚠️ **`TendonMap.cable_lengths` and `resolve` still need the coupled map.** This
    ADR prices the consequence; it does not fold it in, because doing so moves every
    tension in the project and belongs with the re-publish above.
  - Gated by `tests/test_mass_closure.py`. **Several tests assert the defect** --
    NFR5 exceeded, the diagonal map -- so they fail when the fix lands, which is the
    signal to re-run rather than to relax them.
  - ⚠️ Nothing here is measured hardware. It is a manufacturing model of assumed
    stock, assumed catalogue bearing masses and an assumed CF density. **R1 -- buy
    one motor and weigh it -- is still the cheapest way to find out whether any of
    this is real**, and it is still open.

## ADR-0044: The motor holds on spec -- NFR6's runtime does not, and the vendor's own numbers disagree by 27 %

- **Status:** Accepted. **NFR6 must be re-stated. The GIM3505-9 stays selected.**
  Full working in [motor-spec-review](notes/motor-spec-review.md).
- **Context:** ADR-0043 closed the body at 4.304 kg and flagged everything
  mass-derived as needing a re-run. The actuator is the first of those, and it can
  be reviewed **on spec** without waiting on OPEN_RISKS R1 (*buy one and weigh
  it*), which remains open and remains the cheapest high-leverage action available.
- ⚠️ **Finding 1 -- the vendor publishes three mutually inconsistent numbers.**

  | reading of the same motor | Kt (N.m/A) |
  |---|---|
  | rated pair, 0.71 N.m / 1.60 A | **0.444** |
  | peak pair, 1.95 N.m / 4.19 A | **0.465** |
  | vendor quoted | **0.350** |

  [motor-downselect](notes/motor-downselect.md) took 0.44 from the current pairs
  and dismissed the quoted 0.35 as *"a different reference point"*. The two pairs
  agree with each other to 5 %, so that is defensible -- **but it is the optimistic
  branch and nothing had swept it.** Current is `tau/Kt` and copper loss is `I^2 R`,
  so the 27 % spread is worth **1.61x of dissipation**, and ADR-0021's runtime plus
  ADR-0023/0024's thermal duty both ride on `power.KT`.
- **Finding 2 -- torque holds, with less headroom than the record says.** At
  4.304 kg and the **8.75 mm** spool LEG_TENDON_SPEC §2 requires, the trot
  workspace peak is **1.706 N.m = 88 % of peak**. `motor-reality-check` records
  *"1.3x peak headroom"* -- 77 % -- at the old mass and the 8.0 mm spool. **The
  spool change alone costs 8 points**, and buys the same 9.4 % of foot speed: a
  trade nobody had priced.
- ✅ **Finding 3 -- the thermal duty is comfortable, and this answers the sharpest
  open item in the actuator story.** ⚠️ It also corrects how I first read it: a
  workspace peak is **not** a duty cycle. `torque_budget` returns the worst pose in
  the reachable workspace, which sizes structure, not temperature. Integrated over
  the trajectory actually walked:

  | Kt | RMS current | vs the 1.60 A continuous rating |
  |---|---|---|
  | 0.44 | 1.03 A | **0.64x** |
  | 0.35 | 1.30 A | **0.81x** |

  Both branches sit inside the rating. `motor-reality-check §5` left *"thermal test
  at the trot duty -- the sharpest open risk in the whole actuator story"* as owed;
  on spec, it passes. ⚠️ On the pessimistic branch *peak* current is **0.98x** the
  4.19 A rating -- no margin, and that is a driver note as much as a motor one.
- ⚠️ **Finding 1a -- the rotor-side reading is RULED OUT.** The obvious hypothesis
  is that 0.35 N.m/A is quoted before the 9:1 planetary. Then the output constant
  would be 3.15 N.m/A and rated 0.71 N.m would draw **0.225 A** against the 1.60 A
  published -- **7.1x off, in the wrong direction.** It makes the discrepancy seven
  times worse rather than explaining it.
- **Finding 1b -- what fits is a drive/current CONVENTION, and then both numbers are
  right.** The ratio to explain is `0.444/0.350 = 1.2679`, and **4/pi = 1.2732** --
  the square-wave fundamental, six-step against sinusoidal -- lands within **0.4 %**
  (pi/sqrt(6) +1.2 %, sqrt(3/2) -3.4 %). ⚠️ Fitting one ratio against a list of
  constants is **weak evidence** and could be coincidence; what it buys is a sharper
  question for the vendor: *are the 1.60/4.19 A ratings six-step or sinusoidal, and
  is the 0.35 peak-phase or RMS?* Under a convention difference nothing on the sheet
  is wrong, and only the driver's current-sense definition decides which Kt to use.
- ⚠️ **Finding 1c -- a FIRMER factor, found while digging, in the same direction.**
  Copper loss is `sum I_ph,rms^2 R_ph`; balanced three-phase with a wye winding's
  terminal `R_pp = 2 R_ph`, that is `3 I^2 R_ph = **1.5 x I^2 R_pp**`. `power.py`
  computes `I^2 R_pp` -- **1.5x low** for whatever current it is handed. Its own
  docstring flags the simplification and **nothing had ever priced it.** This half
  needs no vendor and no purchase: it is arithmetic.
  ⚠️ The two are **entangled, not independent** -- whether `power.py`'s current *is*
  the RMS phase current depends on the same ambiguity as 1b -- so they bracket
  rather than multiply cleanly.
- ⚠️ **Finding 4 -- NFR6 is what breaks, and by more than my first pass said.**

  | basis | copper | total | runtime |
  |---|---|---|---|
  | published (4.045 kg, 8.0 mm, Kt 0.44) | 42.0 W | 83.6 W | **30.2 min** |
  | at 4.304 kg + the 8.75 mm spool | 56.9 W | 100.2 W | 25.2 min |
  | **+ the x1.5 three-phase correction** | 85.4 W | 128.6 W | **19.6 min** |
  | on the vendor's Kt, as modelled | 90.0 W | 133.2 W | 18.9 min |
  | **on the vendor's Kt, x1.5** | 135.0 W | 178.2 W | **14.1 min** |

  **The honest bracket is 14-20 min**, the two rows carrying Finding 1c, since that
  correction applies under either Kt reading. ⚠️ This ADR first published **19-25
  min** by leaving the copper-loss formula uncorrected.
- ⚠️ **Finding 5 -- the robot is 58 % motor by mass.** 19 x 131.7 g = **2.502 kg of
  4.304 kg**, leaving 1.802 kg for spine, girdles, ribcage, the 300 g battery, 19
  drivers + controller + SBC, head/neck and tail. **ADR-0008's amendment quotes
  45.6 %**, which is 19 x 72 g of a 3.0 kg body -- a class target that does not
  exist, at a superseded mass. Both halves of that figure are stale.
- **Finding 6 -- speed is not a constraint anywhere.** 380 rpm through the ratios
  gives a **7.8-8.5 m/s** foot ceiling; `control.py` quotes 5.93 m/s on a safer
  convention and NFR14 needs 4.1 m/s of spare.
- ⚠️ **Finding 7 -- the down-select, re-run, loses a candidate.** The original sized
  to 1.10 N.m at 3.0 kg; it is now 1.71 N.m at 4.30 kg, and each candidate's own
  mass feeds back into the body it lifts:

  | part | peak | mass | body it makes | needs | verdict |
  |---|---|---|---|---|---|
  | GIM3505-8 | 1.27 | 120 g | 4.082 kg | 1.618 | ❌ **over peak** |
  | **GIM3505-9** | 1.95 | 131.7 g | 4.304 kg | 1.706 | ✅ 88 % |
  | GIM4305-10 | 3.00 | 140 g | 4.462 kg | 1.769 | ✅ 59 % |

  `motor-reality-check §2` lists the GIM3505-8 as *"meets 1.10 N.m"*. **The mass
  growth removed it.** GIM4305-10 is the escape hatch -- 59 % of peak for +158 g --
  but it is **Ø53 against Ø34.5, 54 % wider**, and the girdles were packaged around
  Ø34.5x36.1. That is a repackage, not a part swap.
- **Decision: keep the GIM3505-9, and re-state NFR6 as a range.** 88 % of peak on a
  workspace worst-pose, with the thermal duty at 0.64-0.81x of the continuous
  rating, is an acceptable place to be. Price the GIM4305-10 only if the girdle has
  to be repackaged for another reason.
- **Consequences:**
  - **NFR6: "~30 min / ~900 m" -> "14-20 min / 420-600 m"**, the spread being the
    Finding 1b convention question. The 17 % from mass and spool and the x1.5 from
    Finding 1c are **corrections, not uncertainty**.
  - ⚠️ **`power.py`'s copper-loss formula should be the rigorous three-phase form.**
    It is arithmetic, it needs nothing bought or asked, and it moves ADR-0021's
    runtime *and* ADR-0023/0024's thermal duty.
  - **ADR-0008's "45.6 % of body" -> 58.1 %**, and the sentence should say which
    mass and which motor.
  - ⚠️ **`power.KT` is not swept anywhere in the model.** It should carry both
    branches, or `Kt` should become an explicit sensitivity like the battery
    numbers already are.
  - ✅ `motor-reality-check §5`'s thermal `[owed]` **closes on spec** -- with the
    caveat that peak current on the pessimistic branch has no driver margin.
  - `[owed]` **Ask the vendor Finding 1b's question** -- six-step or sinusoidal
    current ratings, peak-phase or RMS Kt. Cheaper than R1, complementary to it,
    and rotor-side is already eliminated so the question is now specific.
  - `[owed]` Bottom-up check of the 1.802 kg non-motor remainder.
  - Gated by `tests/test_motor_spec.py`; the NFR6 and `power.KT` tests **assert the
    defect** and fail when it is fixed.

## ADR-0045: The copper-loss formula was 1.5x low -- and correcting it overturns ADR-0023's headline

- **Status:** Accepted. **Adopted into `power.py`.** Corrects the magnitudes of
  [ADR-0021](#adr-0021), [ADR-0023](#adr-0023) and [ADR-0024](#adr-0024), and
  **overturns one of ADR-0023's conclusions.**
- **Context:** [ADR-0044](#adr-0044) went looking for why the vendor's three
  published motor numbers disagree by 27 %. Rotor-side was ruled out; a
  six-step-vs-sinusoidal convention fits to 0.4 %. **The useful find was next to
  it:** `power.py` computed copper loss as `I^2 R_pp`, and balanced three-phase
  copper loss is `sum I_ph,rms^2 R_ph = 3 I^2 R_ph`. With a wye winding's terminal
  `R_pp = 2 R_ph` that is **`1.5 x I^2 R_pp`**.

  The module's own docstring had flagged the shorthand since M16 -- *"a rigorous
  three-phase treatment would use 1.5 * I_phase^2 * R_phase"* -- and justified it as
  *"matching the convention in the motor down-select note so the two agree"*. **They
  agreed on a figure 1.5x low.** Unlike the Kt question this needs no vendor and no
  purchase: it is arithmetic.
- **Finding 1 -- the power chain.** At the model's own basis (4.045 kg, 8.0 mm spool,
  Kt 0.44):

  | | was | now |
  |---|---|---|
  | copper loss | 42.0 W | **63.1 W** |
  | trot draw | 83.6 W | **104.6 W** |
  | drive efficiency | 38.7 % | **29.6 %** |
  | trot runtime | 30.2 min | **24.1 min** |
  | trot range | ~905 m | **723 m** |
  | standing runtime, brake off | 37.5 min | **27.0 min** |
  | standing / moving | 0.76 | **0.87** |

  ✅ **ADR-0021's arguments get stronger, not weaker.** Copper loss is now **2.4x**
  the useful mechanical work rather than 1.6x, which sharpens its point that the
  inefficiency is a property of the transmission and not of the gait; and standing
  costs 87 % of moving rather than 76 %, which strengthens the case for the
  ADR-0003 power-off brake.
- ⚠️ **Finding 2 -- ADR-0023's HEADLINE IS OVERTURNED.** Front girdle, 6 motors:

  | | continuous | one battery |
  |---|---|---|
  | trot, polished | **155.2 C** (was 113.7) | 78.3 (was 67.1) |
  | trot, anodised | **96.1 C** (was 74.9) | **70.2** (was 59.7) |
  | stand no brake, polished | 183.9 (was 134.1) | 97.2 (was 85.7) |
  | stand no brake, anodised | 110.2 (was 85.4) | 84.5 (was 72.5) |

  ADR-0023 concluded *"anodised, the girdle does not outlast the pack at all -- it is
  safe because its own equilibrium is ~75 C"*. **That equilibrium is 96.1 C.**
  Anodising is worth **more** than before (59 K, not 39 -- radiation goes as `T^4`
  and the operating point rose) and is **no longer sufficient**. Both halves matter:
  the lever improved, the problem outgrew it.

  ⚠️ **The battery-limited case is now marginal rather than comfortable: 70.2 C**
  against a 70 C line it used to clear by 10 K.

  ✅ **Forced air recovers it, so it stops being optional.** `h = 15` gives
  **72.7 C**, `h = 25` gives 58.2 C. NFR18's *"forced air would reopen it"* becomes
  *"forced air is required for continuous operation"*.
- **Finding 3 -- the winding gradient scales too.** ADR-0024's **+7.7 K** is
  **+11.5 K**; anodised continuous winding **107.6 C** (was 82.6), polished
  **166.7 C** (was 121.4). Its *finding* -- the finish sets where the stack sits, the
  joints set the spread -- is unchanged.
- ⚠️ **Finding 4 -- a third stale copy of the same constant, outside the guard.**
  `test_thermal_constants.py` exists precisely because Rust cannot import Python and
  *"a copied number goes stale silently"*. It guarded four constants. `TOTAL_W`
  (83.5607) was a bare `const` inside **three** separate Rust functions -- `lib.rs`,
  `main.rs`, `examples/winding.rs` -- and therefore outside the guard. It went stale
  exactly as predicted, and only the emergent-runtime cross-check caught it. It is
  now in `from_power_py` and in the pytest parametrise list.
- ⚠️ **Finding 5 -- and the correction immediately double-counted itself.**
  `tools/motor_spec_review.py` had `THREE_PHASE_FACTOR` as a *hypothetical* 1.5x on
  top of the then-uncorrected model. Once `power.py` carried it, applying it again
  gave 14.7 min where the answer is 19.6. **ADR-0044's own tests caught it**, which
  is the argument for writing defect-asserting tests: the constant now scales *down*
  to reproduce the pre-M40 figure.
- **Decision: adopt the rigorous form and re-publish.** `power.PHASE_FACTOR = 1.5`
  ships, with the derivation and the caveat in place.
- **Consequences:**
  - **NFR6: 14-20 min stands** -- ADR-0044 had already anticipated this correction,
    so the requirement does not move again. What moved is that **19.6 min is now the
    model's own answer rather than a hypothetical**, and only the 14-vs-20 spread is
    still hostage to the Kt convention question.
  - **NFR18** re-stated: continuous trot is out of spec **at any finish** in still
    air, the battery-limited case is marginal at 70.2 C, and **forced air is
    required**, not an option.
  - `thermal/src/lib.rs`'s handoff constants all moved; two Rust conclusions were
    renamed rather than relaxed --
    `anodised_is_NOT_safe_on_its_own_merits_any_more` carries the overturning, and
    the time-constant tolerance widened 1.25 -> 1.30 with the linearisation reason
    recorded.
  - ⚠️ **These thermal figures are still at the params body mass of 4.045 kg.**
    ADR-0043's 4.304 kg is not folded in, so they will move again -- upward. The
    combined re-publish is still owed.
  - ⚠️ **The Kt question is untouched by this.** If the vendor's 0.35 N.m/A turns
    out to be the right output-side constant, every temperature above rises a
    further 1.61x and no finish or airflow in the sweep saves a continuous trot.
    That email is now the highest-value open item in the actuator story.

## ADR-0046: The fold-in -- 4.30 kg is now the model, and it cost five findings

- **Status:** Accepted. **Folded into `params.py`.** Re-publishes
  [ADR-0021](#adr-0021), [ADR-0023](#adr-0023), [ADR-0024](#adr-0024) and
  [ADR-0043](#adr-0043); **suspends five earlier findings pending re-measurement.**
- **Context:** M36-M40 established what the numbers should be and deliberately did
  not change them, because *"every mass-derived published figure moves with it"*.
  This is that move. Six parameters changed:

  | | was | now | why |
  |---|---|---|---|
  | `LegParams.link_mass` (hind) | 0.110 kg | **0.1672 kg** | ADR-0041's manufacturing model |
  | `DEFAULT_FORELEG.link_mass` | 0.095 kg | **0.1674 kg** | same; the asymmetry was assumed |
  | `TendonParams.motor_spool_radius` | 0.008 m | **0.00875 m** | LEG_TENDON_SPEC §2, owed since ADR-0010 |
  | `SpineParams.motor_spool_radius` | 0.008 m | **0.00875 m** | same rule, same date it should have moved |
  | `LoadCase.body_mass_kg` | 4.045 kg | **4.3041 kg** | falls out of the above |
  | `trot_params().nominal_foot` x | 0.005 m | **0.00214 m** | re-tuned; see finding 2 |

- **Finding 1 -- the published chain, re-derived.**

  | | pre-M40 | now |
  |---|---|---|
  | body mass | 4.045 kg | **4.3041 kg** |
  | trot draw | 83.6 W | **134.2 W** |
  | drive efficiency | 38.7 % | **25.8 %** |
  | trot runtime | 30.2 min | **18.78 min** |
  | trot range | ~905 m | **563 m** |
  | hip land torque | 16.67 N.m | **17.73 N.m** |
  | hip land tension | 600 N | **638 N** |

  ⚠️ **LEG_TENDON_SPEC §2's "~600 N" is now stale too** -- one milestone's
  correction became the next one's staleness, which is the third time this document
  has done that.
- ⚠️ **Finding 2 -- the trot foothold had to be RE-TUNED, and that is a real design
  change.** NFR2k's balanced foothold is a property of where the CoM sits relative
  to the diagonal, so it moved when the leg masses did: at the old `x = 0.005` the
  roll drift is **-0.180 rad/s per cycle** -- divergent, the robot falls inside a
  stride. Re-bisected on `_roll_drift` the way M7 found the original, the balance
  point is **0.00214 m**. Nothing else in the gait needed touching.
- ⚠️ **Finding 3 -- the thermal conclusions escalated AGAIN, past what forced air at
  h = 15 can fix.**

  | front girdle, 6 motors | continuous | one battery |
  |---|---|---|
  | trot, polished | **202.2 C** | 86.4 C |
  | trot, anodised | **119.0 C** | **78.6 C** |

  ADR-0045 already overturned ADR-0023's *"anodised is safe on its own ~75 C
  equilibrium"*; at the folded-in mass that equilibrium is **119 C**. And the
  airflow that recovered it does not any more: **h = 15 gives 90.1 C**, only
  h = 25 brings it under 80. The winding gradient is now **+16.2 K** (ADR-0024's
  7.7, then M40's 11.5), so an anodised continuous winding sits at **135.1 C**.

  ⚠️ **And the M18 asymmetry INVERTED.** It used to be *"a bare girdle outlasts the
  pack, an anodised one does not"*. The runtime has fallen faster than the time
  constants, so **both** finishes now outlast the pack -- and that is not
  reassurance: reaching only 57 % of the settled rise still lands the anodised
  girdle at 78.6 C. **The protection still exists and no longer protects.**
- **Finding 4 -- two mechanism claims moved, both intact, both re-derived.**
  - ADR-0025's sway correction is **7.1 %, not 4.0 %**. Its size is set by where the
    leg mass sits, and the manufacturing model moved that mass distally, lengthening
    the lever.
  - ⚠️ **ADR-0019's friction limit no longer binds at mu 0.8.** The ROM-limited sway
    fell 42.2 -> **37.0 mm**, and mu 0.8 no longer reaches it. Friction binds
    **below** mu ~0.8 (14.9 mm at 0.4, 32.5 at 0.7) and ROM binds above. The
    mechanism is intact, the crossover moved, and **NFR16's 0.70 floor now sits just
    inside the friction-limited region** -- which is the useful reading.
- ⚠️ **Finding 5 -- FIVE earlier findings are suspended, not retuned.** The
  closed-loop survival measurement went **degenerate**: 37.17 mm at *both* 120 and
  300 deg, above the **29.15 mm** exact feet-only viable bound. That is
  [ADR-0040](#adr-0040)'s finding arriving -- survival was always the wrong quantity,
  and at the heavier mass it has visibly detached from recovery. Four tests read that
  measurement and are marked `xfail(strict=True)` rather than given new thresholds:

  - the measured worst case being below the reduced-order prediction (ADR-0028),
  - the envelope being horizon-limited (ADR-0036),
  - the load split making it worse (ADR-0037),
  - survival not being recovery (ADR-0040's own probe).

  A fifth is suspended for a different reason: ⚠️ **ADR-0029's proportional spine
  assist finding INVERTED in direction.** Spine-off is 6.69 mm and a 0.2 reactive
  assist gives 5.73 -- the assist now slightly *helps* where ADR-0029 measured a 5x
  degradation.

  **Fitting new thresholds to an instrument this milestone just showed to be broken
  would be exactly the M35 mistake.** They are marked, with reasons, and re-deriving
  them is M42.
- **Finding 6 -- what survived unchanged, which is worth saying.** Every design gate
  still passes (motor 88 % of peak at the spec spool, cable SF, bearing C0); the
  reduced-order model's 2 % agreement with the exact viable set **survived** the mass
  change (29.15 against a 29.22 mm bound); compliant legs still beat stiff ones; a
  soft spine still fells the baseline and a stiff one does not; and the paw sensor's
  *marginal* cost fell 1.4x -> 1.25x because the leg it is added to is 52 % heavier.
- **Decision: ship it.** The model now says what the hardware says.
- **Consequences:**
  - **NFR5 4.05 -> 4.31 kg. NFR6 ~30 min -> 18.8 min** (13.6 on the pessimistic Kt
    branch, ADR-0044). **NFR18** now requires **h ~ 25** forced air, not h ~ 15.
    **NFR2k**'s foothold is 0.00214 m.
  - ⚠️ `LEG_TENDON_SPEC` §1.1 *and* §2 are both stale again (17.73 N.m / 638 N).
  - **M42 is fixed in advance:** re-measure the balance arc on
    `measure_envelope(recover=True)`, and re-derive ADR-0029 on it. The five
    `xfail(strict=True)` marks fail loudly if either resolves on its own, which is
    the point of `strict`.
  - ⚠️ **The remaining fold-in is the coupled tendon map** (ADR-0042). It was left
    out deliberately: it changes `cable_lengths` and `resolve` semantics rather than
    a constant, and this milestone was already large enough to bury a mistake in.
  - ⚠️ **None of this is measured hardware.** It is a manufacturing model of assumed
    stock, catalogue bearing masses and an assumed CF density, and the whole chain
    still rests on a vendor sheet that disagrees with itself by 27 %. **R1 -- buy one
    motor and weigh it -- has not moved.**

## ADR-0047: Built as a tendon drive in simulation -- the cable is 5x too stiff, and G3 finally has a number

> ⚠️ **CORRECTED by [ADR-0048](#adr-0048) (M43).** This ADR was measured on a leg
> whose hinge axis was `(0, 1, 0)`, which folds the leg **upward**. Correcting it
> left the cable on the wrong side of every via-pulley, and repairing that moved
> four numbers published below. Corrected values, in place:
>
> | this ADR says | corrected | where |
> |---|---|---|
> | hip 1269 / knee 560 N.m/rad | **1304 / 638** | Finding 2 |
> | 1.72x / 2.21x asymmetry | **1.60x / 1.77x**, and the two hip runs swap | Finding 5 |
> | ankle 39.7 N.m/rad | **53.9** | Finding 4 |
> | the ankle anchor dead spot at ~292 deg | ⚠️ **RETRACTED** -- the ankle has no
> dead spot at any angle; the knee does, 2.02 mm at 270 deg | third bullet under
> "own errors" |
>
> ✅ **What did NOT move: the +/-8.750 coupling column (Finding 1), and G3's
> ~175 kN/m (Finding 3).** Those are the two conclusions this ADR is cited for.
>
> ⚠️ **[ADR-0049](#adr-0049) (M44) moved the G3 band once more**, to **150-200
> kN/m** (from 125-175): re-routing the ankle changed that cable's run length and
> every stiffness with it. **175 kN/m is still the point value**, and ADR-0049
> confirms the element from an entirely independent direction -- force control
> rather than balance compliance.

- **Status:** Accepted, with the corrections above. `mjcf_tendon.py` ships alongside `mjcf.py`, which is
  deliberately untouched. **Sizes design goal G3 for the first time. Confirms
  [ADR-0042](#adr-0042) independently. Puts a hardware requirement under
  [ADR-0026](#adr-0026)'s "compliant legs".**
- **Context:** the plan is to build the robot in simulation before hardware. The
  first thing that needed establishing is that the simulation **is not the robot**:
  `mjcf.py` puts a `<position>` servo on every joint, so the plant under every
  balance result since M17 has been a **direct-drive** machine. Four gaps follow,
  and the first is not a detail:
  - ⚠️ **a position servo can PUSH.** *"A cable can only pull"* is a load-bearing
    premise of [ADR-0002](#adr-0002) (why antagonistic pairs exist at all),
    [ADR-0021](#adr-0021) (why standing costs 76-87 % of moving for zero work) and
    [ADR-0023](#adr-0023) (why standing is the worst thermal case). **The simulation
    has never had that constraint;**
  - the moment arm was a *parameter*, never a geometry;
  - ADR-0042's joint coupling was absent, because a joint servo has no pulley;
  - no cable compliance, no spool.
- **What was built and how it was established.** Five spatial tendons per leg over
  cylinder sheaves and concentric via-pulleys, driven by `<motor tendon=...>` with
  `gear="-1"` and `ctrlrange="0 T"`. Probed *before* anything was built on it: a
  tendon over a cylinder **wraps**, `d(length)/d(angle)` comes out as the cylinder
  radius to **0.25 %**, and commanding **-500 N applies +0.00 N**.
- ✅ **Finding 1 -- ADR-0042's coupling is EMERGENT, and it matches to three
  decimal places.** ADR-0042 derived the via-pulley coupling by hand as **+/-8.75
  mm/rad**, exactly the pulley radius, and said the simulation could not show it:

  | tendon | hip | knee | ankle |
  |---|---|---|---|
  | hip flexor | **+28.000** | 0 | 0 |
  | hip extensor | **-28.000** | 0 | 0 |
  | knee flexor | **-8.750** | +25.097 | 0 |
  | knee extensor | **-8.750** | -24.845 | 0 |
  | ankle | **-8.738** | **-8.750** | -13.935 |

  (Signs and diagonals as corrected by ADR-0048. As first published the diagonals
  carried the opposite sign and the knee flexor and extensor sat in each other's
  rows. **The coupling column is unchanged**, which is the point of the finding and
  is argued properly in ADR-0048.)

  An independent physics engine, from the routing alone, agreeing with a hand
  derivation. MuJoCo even stores `ten_J` sparsely with **1, 1, 2, 2, 3** nonzeros --
  the lower-triangular structure, visible in the memory layout.
  ⚠️ `TendonMap.cable_lengths` is still diagonal.
- ⚠️ **Finding 2 -- THE headline: the cable is far stiffer than balance can
  tolerate.** [ADR-0026](#adr-0026) measured that balance needs **compliant** legs,
  servo `kp` 80-150 N.m/rad, and that **kp >= 250 winds up and falls**. In the servo
  sim that compliance was a gain. In a tendon drive it has to come from the cable,
  and `k = EA/L` at the routed run lengths gives:

  | joint | restoring stiffness | vs the kp = 250 that FELL |
  |---|---|---|
  | hip | ~~1269~~ **1304 N.m/rad** | **5.2x** |
  | knee | ~~560~~ **638** | 2.6x |

  **ADR-0026's "balance needs compliant legs" was a requirement on hardware that was
  never turned into hardware.** `kp = 80` was standing in for a compliance the
  machine does not have.
- ✅ **Finding 3 -- so G3 finally has a number.** Design goal **G3** ("passive
  compliance / shock absorption at each joint") has been a goal since M1 with
  nothing attached. A series-elastic element in line with each cable combines as
  `1/k = 1/k_cable + 1/k_series`; swept, **~175 kN/m puts both the hip and the knee
  inside ADR-0026's 80-150 window** (~~128 and 91~~ **136 and 107** N.m/rad). That
  is a real spring to hand to mechanical, and ADR-0048 widened it to a **125-175
  kN/m band**.
- ⚠️ **Finding 4 -- and the ankle fails the OTHER way. A note on ADR-0002 Option B.**
  A cable always pulls the same direction, so a joint driven by **one** tendon has
  no restoring stiffness from it at all -- perturb either way and the pull does not
  reverse. Measured ~~39.7~~ **53.9 N.m/rad**, of which the Option-B return spring
  contributes **0.3**. Option B buys a motor per leg; what it costs is the joint's
  stiffness, and that had not been priced. The series spring does not help here
  (10.6 N.m/rad) -- the ankle needs the opposite treatment.
- ⚠️ **Finding 5 -- a pair's stiffness is DIRECTION-DEPENDENT.** `k = EA/L` and the
  two runs are not the same length: hip flexor ~~0.121~~ **0.073 m**, extensor
  ~~0.073~~ **0.121 m**, so the **flexor** is 1.7x stiffer. One-sided the hip reads
  **1604 against 1003** N.m/rad depending on which way it is pushed -- a ~~1.72x~~
  **1.60x** asymmetry, ~~2.21x~~ **1.77x** at the knee. Equalising run lengths is a
  routing choice nobody has had to make yet.
  ⚠️ ADR-0048's routing repair swapped which member of each pair takes the short
  route; that the asymmetry exists at all is the finding, and it is unchanged.
- ⚠️ **Finding 6 -- gravity feedforward cannot hold a pose, and an outer position
  loop can.** Allocating tension to cancel the measured gravity term every timestep
  diverges at every co-contraction level (0, 5, 19.6 N) -- it is feedforward with no
  error feedback on an unstable equilibrium. `kp` 10 N.m/rad with `kd` 0.2 holds the
  hip and knee to **0.00 deg**. That is why FR1 specifies *closed-loop* position
  control, and the servo sim could not show it because a position servo **is** the
  loop.
  - **And the allocator matters, which is a firmware note.** Solving the
    non-negative least-squares properly holds the hip to 0.00 deg; taking the
    unconstrained solution and **clipping** it at zero leaves **1.22 deg**. Clipping
    is not respecting the constraint.
- ⚠️ **Three of this milestone's own errors, corrected rather than shipped:**
  - **two sign errors** in the tension allocation (`qfrc_actuator` must supply
    **+**`qfrc_bias`), which produced a tidy and completely false conclusion --
    *"constant moment arms mean co-contraction adds no stiffness"* -- before
    measuring `tau = -J^T T` directly caught it;
  - an **anchor in a dead spot** -- ⚠️ **this EXAMPLE is RETRACTED by ADR-0048,
    though the effect is real.** As published: the ankle anchor, placed by the same
    2-D heuristic the analytical routing used, landed ~292 deg around its sheave
    where the incoming cable already clears it, so the moment arm was 2.6 mm instead
    of 14. That was the mirrored fold. Corrected, **the ankle wraps at every anchor
    angle** (13.69-13.94 mm swept at 10 deg steps) and the heuristic point reads
    13.86 mm. The effect re-establishes on the **knee**: 2.02 mm at 270 deg, **8 %
    of the specified 25**. A sheave the cable never touches does no work -- and
    ADR-0048 found the dead band **moves between joints** when the hinge convention
    changes, which makes the anchor sweep a build step;
  - a **single `stiffness` constant**, which NaN'd at t = 0.168 s. A spatial
    tendon's `stiffness` pulls toward `springlength`; given one value it is a
    two-sided spring rather than a cable. Two values make a deadband, which is a
    cable -- and the stiffness has to be per-tendon anyway, as §2 always said.
- **Consequences:**
  - **G3 gets a target: ~175 kN/m series-elastic element at the hip and knee.**
    ⚠️ Not at the ankle, which is already too soft.
  - ⚠️ **ADR-0026's compliance finding is re-classified**: it is a *hardware*
    requirement, not a controller setting, and until the series spring exists the
    tendon-driven leg sits 5x past the stiffness at which the servo sim fell.
  - ⚠️ **ADR-0002 Option B needs re-examining.** It was chosen on motor count. The
    ankle having no restoring stiffness is a cost it never counted.
  - `mjcf.py` and every M17-M41 figure measured on it stay untouched and
    reproducible. This plant is not yet the one the arc is measured on.
  - **Next: whole-body, 19 DOF**, and then M41's five suspended findings re-derived
    on a plant that is actually a tendon drive.

## ADR-0048: The whole-body tendon plant leans rather than collapses -- and a sign in the via-routing had been inflating every M42 number

> ⚠️ **Its HEADLINE is RETRACTED by [ADR-0049](#adr-0049) (M44).** This ADR
> concluded that a per-leg joint controller *cannot* make the quadruped stand,
> measuring a 14.5 deg diagonal lean. **Two more routing defects were inflating
> that**, both found in M44: the ankle anchor sat past its moment-arm **sign
> reversal**, and the ADR-0002 return spring was referenced 97 deg away from the
> hind stance hock. Corrected, the same controller reaches **2.4 deg**.
>
> The comparison that survives is **2.4 deg (joint angles) against 0.006 deg
> (foot forces)** -- a 400x attitude improvement, plus the joint controller still
> inverting at kp 400 where the foot-force one does not. ✅ The *diagnosis* below
> was right and is what M44 acted on; the *measurement* was not.
>
> ⚠️ Also corrected here: the welded-trunk figures (0.37 / 1.8-2.3 deg) and the
> stand table both move again. See ADR-0049 for the current numbers.

- **Status:** Accepted. `quadruped_rig` / `quadruped_rig_elastic` ship in
  `mjcf_tendon.py`. **Corrects [ADR-0047](#adr-0047) in four places and retracts one
  of its findings. Confirms [ADR-0042](#adr-0042) a second time, harder. Confirms
  [ADR-0038](#adr-0038)'s whole-body controller is the missing piece, and
  [ADR-0033](#adr-0033)'s diagonal-stance argument from a new direction.**
- **Context:** M42 gated a single tendon-driven leg. This scales it to the robot:
  four legs on a floating trunk over a floor, **18 DOF, 20 spatial tendons, 20
  pull-only actuators, 4.3081 kg** against `params`' 4.3041. The question is whether
  a pull-only quadruped can stand.

### The finding that had to come first

- ⚠️ **The M42 leg was built on the wrong hinge axis, and correcting it broke the
  routing silently.** `LegModel.forward` builds the tip with `x = l cos a,
  z = l sin a`, so a positive joint angle must rotate +x toward **+z**; MuJoCo's
  right-hand rule about +y does the opposite, so the axis has to be `(0, -1, 0)`.
  `mjcf.py` documents this. `mjcf_tendon.py` used `(0, 1, 0)`, so **the whole leg
  pointed up** -- feet at z = +0.346 above a trunk at 0.176 -- and the quadruped
  "stood" by sinking to the floor with its joints dutifully held. Printing the foot
  positions is what caught it.
- ⚠️ **Correcting the axis left every via-pulley site on the wrong side, and
  nothing complained.** The tendons still routed, still pulled, still reported
  lengths:

  | | knee flexor arm | the four couplings |
  |---|---|---|
  | via sites at -z (as M42 shipped) | **1.17 mm/rad** | 11.73, 36.40, 14.00, 41.54 |
  | via sites at +z (repaired) | **25.10** | **8.75, 8.75, 8.74, 8.75** |

  A wrap that does not happen is not an error condition. It is a moment arm of the
  wrong size, and only differentiating the tendon length finds it.
- ⚠️ **And the test harness is what let it hide.** `_hold` had the tendon Jacobian
  **written down as a literal**, copied from M42's measurement. After the axis fix
  the hip pair's signs swapped, the frozen matrix kept commanding the wrong
  antagonist, and the leg collapsed **102 deg while the routing itself was fine**.
  A controller that measures its own plant survives a change to the plant; one that
  quotes a number from a previous milestone does not. `_hold` measures it now.

### What the repair cost, and what it did not

- ⚠️ **Four of ADR-0047's published numbers moved**, because re-routing re-cut
  every cable run and `k = EA/L`: hip/knee stiffness **1269/560 -> 1304/638
  N.m/rad**; pair asymmetry **1.72x/2.21x -> 1.60x/1.77x**, with the two hip runs
  swapping which is short (flexor 0.073 m now, extensor 0.121); the lone-tendon
  ankle **39.7 -> 53.9**. None of the conclusions drawn from them changed.
- ⚠️ **One finding is RETRACTED.** ADR-0047 reported the ankle anchor landing in a
  dead spot at ~292 deg, moment arm 2.6 mm instead of 14. That was the mirrored
  fold. Corrected, the ankle wraps at **every** anchor angle -- 13.69 to 13.94 mm
  swept at 10 deg steps -- and the 2-D heuristic point M42 called dead reads
  **13.86 mm**.
- ✅ **The general lesson survives, and the knee shows it far more sharply.** The
  knee flexor's arm collapses to **2.02 mm at 270 deg -- 8 % of the specified 25**,
  against 25.10 where it ships. ⚠️ The dead band **moved from one joint to another
  under a change of hinge convention**, which makes the anchor sweep a build step,
  not a one-off.
- ✅ **What the repair could not move: the coupling column.** It read **-8.750
  before the repair and -8.750 after**, while the diagonal signs flipped, the knee
  flexor and extensor swapped rows, and every cable length changed. A number that
  comes from the pulley radius does not care which way the leg folds; a number that
  comes from a routing accident does. **That is a stronger confirmation of
  ADR-0042 than M42's agreement was**, because it is an invariance rather than a
  coincidence.
- ✅ **G3's ~175 kN/m survived, and gained a band.** Re-swept: 175 kN/m gives
  136/107 N.m/rad (was 128/91), still inside ADR-0026's 80-150 window, and
  **1.25e5-1.75e5 N/m keeps both joints inside it**. A range is more useful to hand
  to mechanical than a point value.

### The whole-body plant, and two things it cost to build

- ⚠️ **`TENSION_MAX` is the MOTOR's limit, not the cable's -- and getting that
  wrong launched the robot.** The first pass took 700 N from ADR-0046's 638 N land
  transient. That is a **structural** number: what the cable, pulley and bearing
  must survive when the *ground* hits the foot. Given 700 N of authority on twenty
  tendons, a 0.1 s contact transient saturated all of them and **threw the 4.3 kg
  quadruped off the floor** (z 0.176 -> 0.834, `ncon = 0`). Twenty times 700 N is
  14 kN on a 42 N robot. The real ceiling is `tau_motor / r_spool`: **223 N peak,
  81 N continuous**. The structure carries 2.9x more than the actuator can ever
  apply, which is correct -- the transient arrives from the ground.
- ⚠️ **The pulley geoms had to be made massless.** The plant compiled at 4.532 kg
  against `params`' 4.3041, and the 0.224 kg gap was exactly 4x the per-leg pulley
  geom masses. [ADR-0041](#adr-0041)'s manufacturing model **already apportions
  every sheave and bearing into `link_mass`**, so giving the geoms their own mass
  double-counts it. The residual 4 g is the four paw pads, which `link_mass` does
  not carry.

### The gate: it does not stand, and the failure is a LEAN

- ✅ **Welded to the world, the same per-leg controller holds every leg** -- hind
  to **0.37 deg**, fore to **1.8-2.3 deg**. So neither the tendon routing nor the
  pull-only allocation is what fails. (Before the via repair these read 3.4 and
  14.5-19 deg; both improved by roughly an order, which is how much of M43's first
  fore/hind story was really a routing bug.)
- ⚠️ **Floating, it settles into a diagonal lean rather than collapsing:**

  | kp | T_bias | trunk z | tilt | min contacts |
  |---|---|---|---|---|
  | 25 | 5 N | 0.155 | 18.3 deg | 2 |
  | 50 | 5 N | 0.030 | **180 deg** | 0 |
  | 100 | 19.6 N | 0.150 | 14.5 deg | 2 |
  | 200 | 19.6 N | 0.150 | 14.5 deg | 2 |
  | 400 | 19.6 N | 0.150 | 14.1 deg | 0 |

  **85 % of the target height, tilted 14.5 deg, on two feet of four** -- while every
  leg holds its commanded angles, the hind pair to 0.42 deg. One gain setting
  (kp 50, 5 N) flips it completely over.
- ⚠️ **Which is exactly the diagonal-stance problem [ADR-0033](#adr-0033) named.**
  A per-leg joint controller has no term for trunk attitude, so the trunk finds its
  own equilibrium and the answer is a diagonal lean. **Commanding joint ANGLES
  cannot express "put 10 N more through the left front foot"; commanding foot FORCES
  can.** `wbc.py` from [ADR-0038](#adr-0038) does exactly that allocation, was built
  in M33, and has never been driven against a tendon plant.
- ⚠️ **The first pass read this failure as a 98 deg collapse**, which was the via
  routing inflating it. The corrected failure is both smaller and better posed: not
  *"the legs cannot hold"* but *"nothing in the loop has an opinion about the
  trunk"*.

### A new finding, from the corrected plant

- ✅ **Co-contraction buys back the clipped allocator, which is a firmware note.**
  ADR-0047 found that clipping an unconstrained least-squares allocation at zero
  costs about a degree of joint error against solving the non-negative problem
  properly. Repaired, that holds at a 5 N co-contraction floor (**1.2-1.4 deg**) --
  and at [ADR-0021](#adr-0021)'s standing tension of **19.6 N the same clipped
  allocator holds to 0.00 deg**, because the base tension keeps the solution
  interior so nothing clips at all. **Clipping is only wrong when it is reached, and
  co-contraction is what keeps it out of reach.** That is a second, previously
  unpriced reason to pay for co-contraction, alongside [ADR-0002](#adr-0002)'s.

### Consequences

- **ADR-0047 is corrected in place** with a banner, not rewritten; its two cited
  conclusions (the coupling, and G3) both survived.
- **G3's target becomes a band: 125-175 kN/m**, with 175 the current point value.
- ⚠️ **The anchor sweep is a build step.** Any change to link geometry, hinge
  convention or via placement has to re-run it, because a dead spot moves.
- ⚠️ **A frozen Jacobian in a test harness is a latent failure.** Everything that
  drives this plant measures its own map now.
- ⚠️ **The fore leg is still not mirrored** -- `DEFAULT_FORELEG` folds the opposite
  way, so its sidesites and anchor angles were inherited rather than reflected. At a
  5x gap but only 1.8 deg absolute, it no longer gates anything.
- **Next: drive this plant with `wbc.py`'s foot-force allocation.** Then the
  articulated spine and tail, to reach the full 19 DOF.

## ADR-0049: The pull-only quadruped stands on foot-force allocation -- and a lone tendon's moment arm reverses inside its own ROM

- **Status:** Accepted. `wbc.nnls`, `wbc.tendon_tension` and `wbc.actuator_torque`
  ship. **Closes the stand gate [ADR-0048](#adr-0048) left open. Fixes a defect in
  [ADR-0038](#adr-0038)'s own module. Retracts ADR-0048's headline. Corrects
  [ADR-0047](#adr-0047)'s G3 band and confirms G3 independently. Adds a second
  unpriced cost to [ADR-0002](#adr-0002) Option B.**
- **Context:** ADR-0048 left the whole-body plant leaning on a diagonal and gave an
  unambiguous diagnosis: nothing in a per-leg loop has an opinion about the trunk,
  so the controller has to command foot **forces**. `wbc.py` from ADR-0038 does
  exactly that allocation, was built in M33, and had never been driven against
  anything but a position-servo plant.

### The result

- ✅ **It stands.** Trunk height **0.17600 -> 0.17579 m over 3 s (0.21 mm)**, trunk
  tilt **0.006 deg**, all four feet down throughout, allocation residual ~0.02 N.m
  -- the commanded tensions really do produce the torques asked of them.

That took **one missing link** and **three fixes**, and the fixes are the findings.

### The missing link: joint torque -> non-negative tendon tension

- ADR-0038's chain ends at `stance_torque`, which is where a direct-drive robot
  stops. A pull-only tendon robot needs one more step, and it is a constrained
  problem: `min ||G T - tau||` subject to `T >= T_min`. `wbc.nnls` is Lawson-Hanson,
  written out rather than imported because the project has no scipy and **firmware
  will not have one either**.
- ⚠️ **Clipping escalated from "about a degree" to "loses the leg".** ADR-0047
  priced clipping an unconstrained allocation at ~1 deg of joint error. At standing
  loads:

  | | single leg, unloaded | quadruped |
  |---|---|---|
  | clipped least squares | **197 deg hip drift** | 20.7 deg lean, 1.14 N.m residual |
  | `wbc.tendon_tension` | **0.00 deg** | 0.006 deg lean, ~0 residual |

  The constraint has to be **in** the solve.

### Fix 1 -- `realisable_cop` had never been exercised on a support POLYGON

- ⚠️ M33 only ever ran a **diagonal two-foot trot**, where the CoP genuinely is
  confined to a line and that branch is exact. The three-or-more branch was written
  and never run, and it was wrong twice:
  1. **No inside test.** It walked the boundary and returned the nearest point on an
     edge, so a feasible CoP in the middle of a four-foot polygon was pushed **48 mm
     out to the rail**. For a standing robot that is not a clamp, it is a *command
     to lean*.
  2. **It assumed the caller's point order was hull order.** It is not:
     `("LF", "RF", "LR", "RR")` traverses a rectangle as a **bowtie**, so two of the
     four "edges" it measured against were diagonals. That bug partly **masked** the
     first -- it moved the interior point 16.6 mm instead of 48.
- ⚠️ **And fixing it did not make the robot stand: 14.5 deg of lean before, 14.5
  after.** A real defect that turns out not to be the cause is still worth fixing,
  and worth recording as not-the-cause.

### Fix 2 -- the torque bookkeeping omitted `qfrc_passive`

- MuJoCo's own equation of motion makes the actuator term
  `qfrc_bias - qfrc_passive + stance_torque`. Leaving `qfrc_passive` out asked the
  tendons to supply what the springs were already supplying: ⚠️ at the hind stance
  pose the **ADR-0002 Option-B return spring alone is 0.508 N.m**, which is **54 %**
  of that joint's whole demand, requested twice. `wbc.actuator_torque` does it now.

### Fix 3 -- ⚠️ a LONE TENDON's moment arm REVERSES SIGN inside its own ROM

- **This is the structural finding, and it is a sharper statement of what ADR-0002
  Option B costs than ADR-0047's was.** ADR-0047 found a lone-tendon joint has no
  restoring *stiffness*. This is stronger: **the one direction it can pull is not a
  fixed direction in joint space.**
- Swept **12 anchor angles x the full -30...+150 deg ankle range: all 12 reverse
  somewhere between 45 and 120 deg.** No anchor avoids it, and it cannot: as the
  metatarsus sweeps 180 deg the anchor sweeps 180 deg around the sheave, so the
  incoming cable line must cross the sheave centre exactly once.
- ⚠️ **The hind stance pose was on the wrong side of it.** The hind hock holds
  **+97.1 deg** in stance; at M42/M43's 45 deg anchor the reversal sat at ~85 deg.
  So the hind ankle could not supply standing torque **at any tension** -- the
  non-negative allocation left a **0.714 N.m residual**, which is infeasibility, not
  a solver miss. Moving the anchor to **300 deg** pushes the reversal past 105 deg
  and makes all four legs feasible at residual **0.000000**.
- (The fore hock holds **+16.4 deg**, comfortably inside. The two legs behaved
  completely differently under load for that reason and no other -- not, as ADR-0048
  supposed, because the fore leg's routing was un-mirrored.)

### And the consequence: Option B cannot serve BOTH stance and swing

- The ankle needs **opposite** torques loaded and unloaded. Loaded, the ground pushes
  the toe up and the joint needs **plantarflexion** (-0.68 N.m hind, -0.79 fore, one
  sign across the whole stance sweep). Unloaded, the return spring is the only thing
  acting and referenced at 0 it pulls the +97 deg hock **plantarflexing as well**, so
  the tendon must **dorsiflex** to hold the pose. **The spring and the stance load
  pull the same way**, and a lone tendon has one direction:

  | anchor | unloaded ankle | quadruped |
  |---|---|---|
  | 45 deg (M42/M43) | **0.00 deg** | ⚠️ **inverts** |
  | 300 deg (M44) | -14.6 deg | ✅ **stands** |

- **M44 ships 300 deg**, because closing the stand gate is the milestone, and
  references the spring at **each leg's own stance angle** -- which also drops the
  worst tendon from the 222.9 N ceiling to **207.4 N**. The **-14.6 deg unloaded
  ankle is the price.**
- ⚠️ **ADR-0002 Option A is now a live decision with numbers on both sides.** An
  antagonistic pair at the ankle costs **four more motors** and removes the conflict
  entirely. Option B was chosen on motor count; this is the **second** cost it never
  counted, after ADR-0047's.
- ⚠️ **A params bypass, of exactly the class [ADR-0046](#adr-0046)'s fold-in
  existed to remove:** `mjcf_tendon` hard-coded `springref="0.0"` and never read
  `spring_rest_angle` at all. `spring_rest_angle[2] = 0.0` is **97 deg from the hind
  stance hock**, and the fore and hind stance angles are **81 deg apart**, so one
  number cannot serve both. The rigs now derive it per leg from the stance pose and
  say why; `params` still owes mechanical a decision.

### G3, confirmed a second time from a different direction

- ✅ ADR-0047 sized G3's series-elastic element at ~175 kN/m from a
  **balance-compliance** argument (ADR-0026's controller falls at `kp >= 250` and the
  bare cable is 5x that). M44 arrives at the same element from **force control**:

  | cable | outcome under foot-force control |
  |---|---|
  | **series-elastic, 175 kN/m** | ✅ **stands**, tilt 0.006 deg |
  | bare cable (5x stiffer) | ⚠️ **inverts**, tilt 180 deg |
  | no cable elasticity at all | leans 14.6 deg |

  Two independent arguments, two different failure modes, the same part. That is the
  strongest form this project has for a component nobody has bought yet.
- **Band corrected to 150-200 kN/m** (was 125-175): re-routing the ankle changed
  that cable's run length and every stiffness moved a per cent or two with it.
  **175 kN/m remains the point value** and is still near the centre.

### It stands, but not indefinitely

- ⚠️ Per-tendon tension while standing, against a motor rated **81 N continuous**
  and **223 N peak**:

  | leg | worst tendon | mean | peak |
  |---|---|---|---|
  | fore | knee flexor | 74 N | 87 N |
  | **hind** | **hip extensor** | **~205 N** | **207 N** |
  | hind | knee flexor | 129 N | 138 N |

  The hind hip extensor runs **~2.5x the continuous rating just to stand still**.
  ⚠️ [ADR-0023](#adr-0023) made standing the worst thermal case at the **nominal
  19.6 N** co-contraction tension; this is an order above that on one tendon of
  twenty, and the thermal model has never been run on it. The fore legs are
  comfortable, which is the CoM sitting behind the middle: the hind feet carry
  17.3 N against the fore pair's 10.4 and 3.8.

### Two smaller findings

- ⚠️ **`desired_wrench` has no attitude term.** It returns a zero desired moment,
  which places the CoP under the CoM -- right for M33's in-place trot, but standing
  needs the trunk's attitude regulated. M44 adds a small angular PD *outside* the
  function; folding it in is owed.
- ⚠️ **The quadruped's girdle spool placement degrades the hip moment arms** from
  +/-28.000 mm to **25.875 / -27.173**, and makes the antagonistic pair asymmetric.
  The spools were placed by the packaging study, not by routing.

### Consequences

- **ADR-0048's gate is closed. The pull-only tendon quadruped stands.**
- `wbc.nnls`, `wbc.tendon_tension` and `wbc.actuator_torque` are the reusable
  pieces, and **firmware needs all three** -- particularly the first, which is why
  it is written out rather than imported.
- ⚠️ **ADR-0002 Option A vs B for the ankle is a live decision**, costed: four
  motors against a -14.6 deg unloaded ankle and a moment arm that reverses mid-ROM.
- ⚠️ **The anchor sweep must check the SIGN across the ROM, not just the wrap.**
  ADR-0048 already made it a build step; this says what it has to measure.
- **G3: 150-200 kN/m, 175 the point value**, now supported by two independent
  arguments.
- ⚠️ **Next: the thermal case for a 205 N standing tendon** (ADR-0023 does not
  cover it), fold the attitude term into `desired_wrench`, mirror the fore leg, and
  add the spine and tail to reach 19 DOF.

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
