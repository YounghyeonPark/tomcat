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
  - **Consequence — static stability caps this walk at a crawl:** ~**4 cm/s**.
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
