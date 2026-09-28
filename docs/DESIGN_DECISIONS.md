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

> ✅ **The ANKLE question is now SETTLED, by [ADR-0050](#adr-0050) (M45): Option
> A, an antagonistic pair, capstan construction.** Option B was chosen here on motor
> count, and four milestones then priced it: no restoring stiffness
> ([ADR-0047](#adr-0047)), a moment arm that reverses inside its own ROM
> ([ADR-0049](#adr-0049)), and finally the one cost nobody had looked at —
> **kinematic reach**. The trot commands the ankle **+62.4° above its reference
> during SWING**, and under Option B nothing pulls that way at all. Cost: four
> motors, **+528 g**, 19 → 23 actuators.

> ✅ **THE ADR-0008 CONFLICT IS RESOLVED, by [ADR-0058](#adr-0058) (M53), in
> ADR-0008's favour.** The paragraph below stands as the statement of what the
> conflict was; what it costs is now decided and priced. Antagonistic pairs remain,
> and `T_bias` becomes a pulley radius profile rather than a control input.

> ⚠️ **The conflict, as [ADR-0057](#adr-0057) (M52) found it.** This ADR makes
> co-contraction `T_bias` a
> **first-class control input**. ADR-0008 runs **one motor per antagonistic pair**
> and calls it *"a change of transmission, not of DOF"* -- true of the joint angles,
> **false of the stiffness**. Co-contraction is the redundant coordinate, and one
> motor per pair removes it: measured, the paired leg's allocation has **three**
> co-contraction directions and ADR-0008's would have **none**. The project has been
> running this ADR in simulation and ADR-0008 in the mass budget.
> ✅ Since M53 it runs ADR-0008 in both.

- **Status:** Accepted; the ankle case revised by ADR-0050; ⚠️ **its
  commandable-stiffness clause is SUPERSEDED by [ADR-0058](#adr-0058) (M53)**, which
  resolved the ADR-0008 conflict in ADR-0008's favour. Antagonistic pairs remain;
  `T_bias` stops being a runtime input and becomes a pulley profile.
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

> ⚠️ **SUPERSEDED: G6 IS WITHDRAWN. [ADR-0071](#adr-0071) (M66).** This ADR
> exists to serve a goal the project no longer has. It is kept for the record and
> for the measurements ADR-0068 to ADR-0070 hung on it.

> ⚠️ **THE MECHANISM THIS ADR NAMES DOES NOT EXIST, AND THE SUBSTITUTE IS A
> FACTOR OF FIVE SHORT. [ADR-0068](#adr-0068) (M63).** This ADR puts righting
> authority in the **spine axial-twist DOF**. [ADR-0006](#adr-0006) targeted three
> DOF per segment (pitch, yaw, **axial roll**) and deferred the last two;
> [ADR-0009](#adr-0009) then bought the **lateral** DOF and cited righting as dual
> use, writing *"lateral/axial"* as one thing. **Axial roll is in no budget and no
> model.** ✅ Pitch and yaw alone *can* right a body -- a precessing bend needs no
> twist -- but measured at the joint limits it gives **53 deg/s** against the
> **282-730 deg/s** a 2.0 m to 0.3 m fall allows.

- **Status:** Accepted (revised — tail simplified per project-owner)
  ⚠️ **its righting mechanism unbuilt and its substitute unmet — see
  [ADR-0068](#adr-0068)**
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

> ✅ **IMPLEMENTED AND CHOSEN, [ADR-0058](#adr-0058) (M53).** `pulley=True` emits
> one bidirectional motor per pair: **12 leg motors**, ADR-0042's map unchanged, the
> body at **4.3041 kg**. The [ADR-0002](#adr-0002) conflict was resolved here, in
> this ADR's favour, and the standing tension saturation [ADR-0056](#adr-0056)
> measured went with it. ⚠️ **Two things this ADR did not say:** the pair must be
> **split across opposite sides of each via-pulley** or the coupling becomes common
> mode the pulley cannot absorb (1716-3090 N against a 638 N cable), and *"full
> articulation is retained"* is true of the joint angles but **false of the
> stiffness** -- that is what was traded away.

> ⚠️ **Superseded banner, kept for the record -- THE SIMULATION NEVER IMPLEMENTED
> THIS. [ADR-0056](#adr-0056) (M51).** The
> decision below is one motor per antagonistic pair via the variable-radius pulley
> -- 12 leg motors, and it is what `params.py`'s `trunk_mass` is built on.
> `mjcf_tendon.py` emits an independent motor per **tendon**: **20** leg motors, or
> **24** with ADR-0050's ankle pair. That is **1.054 kg** of unbudgeted actuator, or
> **1.580 kg**, putting the body at **5.36** or **5.89 kg** against the 4-5 kg band
> NFR5 is anchored to. Eight milestones of tendon simulation (M42-M50) rest on the
> architecture this ADR **rejected, on these exact grounds**.

- **Status:** Accepted, and ✅ **implemented and chosen in M53 --
  see [ADR-0058](#adr-0058)**
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
> ⚠️ **THIS ADR'S PREMISE DOES NOT HOLD ON THE SHIPPED PLANT.
> [ADR-0067](#adr-0067) (M62).** Re-run this ADR's own margin calculation with the
> sway the actuated robot delivers (+-1.30 mm, [ADR-0066](#adr-0066)) and the worst
> margin is **-21.47 mm** against **-22.59** with no sway at all: the three lateral
> motors recover **3.9 %** of what they were bought for, and the robot spends the
> same **19.8 %** of each cycle outside its support polygon either way. Reaching a
> merely **zero** margin needs **25.90 mm** of sway. **Static stability is already
> gone**; option E below is not a choice, it is the situation.

> ⚠️ **THE SWAY DOES NOT COME OUT ON THE ACTUATED PLANT.
> [ADR-0063](#adr-0063) (M58).** Everything below is analytic geometry and a
> quasi-static margin. Run the law on M57's tendon-driven spine and the body
> gets **4.1 mm** of the designed **66.7 mm**, while every paw slides further
> than the CoM moves. The mechanism is this ADR against
> [ADR-0017](#adr-0017): a lateral bend swings the front girdle, the fore legs
> hang off it, and no planar leg can move a foot sideways.

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
> ⚠️ **But Finding 1 needs qualifying, by [ADR-0055](#adr-0055) (M50).** *"The
> moment arm is emergent from the geometry"* holds **where the cable actually
> wraps**, and this ADR measured it only at the stance pose. For the hip that window
> is `q1 >= -10 deg`; across the range the trot commands (-110..-35.1) the same
> construction gives **21.0 to 36.3 mm on a 28 mm specification, a 1.73x swing**. As
> a *validation* that a wrap reproduces the sheave radius, this finding stands. As
> the *model* for a wide-ROM joint it does not.
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
>
> ⚠️ **And the fore/hind gap this ADR read as un-mirrored sidesites is worse than
> that.** [ADR-0053](#adr-0053) measured the fore leg's Jacobian -- which no
> milestone had done -- and its **hip pair does not oppose at all**: +11.636 and
> +35.885 mm/rad against a specification of +/-28. This ADR treated the symptom as a
> tuning difference. It is a broken routing, and every fore-leg figure from M43 on
> rests on it.

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
  double-counts it. ~~The residual 4 g is the four paw pads, which `link_mass`
  does not carry.~~ ⚠️ **Wrong, corrected in [ADR-0088](#adr-0088):
  `per_link_mass()` adds the 5.68 g pad to the paw on its own line, so
  `link_mass` DOES carry it and the 1 g pad geom was a double count.**

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

> ⚠️ **THE BINDING TENDON IS THE FORE KNEE FLEXOR, NOT THE HIND HIP EXTENSOR.
> [ADR-0064](#adr-0064) (M59).** The table below was measured while the fore legs
> rested partly on their metatarsals, 8.3 mm behind the contact site, which
> unloaded them. On a point-foot contact: **fore knee flexor 100.5 N (1.24x
> continuous)**, hind ankle 67.8, and the **hind hip extensor 63.4 N -- inside its
> rating**. The overrun is real, smaller, and at the other end of the robot.

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

## ADR-0050: The ankle takes an ANTAGONISTIC PAIR (Option A) -- because the gait commands 62 deg the spring cannot reach

- **Status:** Accepted, and it **settles [ADR-0002](#adr-0002)'s open ankle
  question** after four milestones of accumulating costs against Option B.
  ⚠️ **Its MASS COSTING is withdrawn by [ADR-0056](#adr-0056)**: "+4 motors,
  +528 g, 4.83 kg" assumed the baseline was the simulation's 20 leg motors and that
  `params` carried them. `params` carries **12**, per ADR-0008's variable-radius
  pulley, so the delta from the budget is **+1.58 kg**. The ankle *decision* is
  unaffected -- it was made on kinematic reach -- but the number is not the cost.
  `ankle_pair` and `ankle_spring` build options ship in `mjcf_tendon.py`.
  **Raises NFR5 to a projected 4.83 kg. Supersedes ADR-0049's "live decision".**
- **Context:** ADR-0002 chose Option B for the ankle -- **one tendon plus a torsion
  return spring** -- on motor count, and nothing had priced it. Two milestones then
  did: [ADR-0047](#adr-0047) found a lone-tendon joint has **no restoring
  stiffness**, and [ADR-0049](#adr-0049) found its **moment arm reverses sign inside
  its own ROM**, so the one direction it can pull is not a fixed direction in joint
  space. This milestone set out to settle A vs B by measurement.

### First: "add an antagonist" is not sufficient

- ⚠️ The hip and knee build their pairs by **mirroring** -- same sheave, opposite
  sidesite, anchor reflected across z. Swept across the ankle's whole -30...+150 deg
  range, the three candidate antagonist anchors behave completely differently:

  | antagonist anchor | spans both directions? | worst arm error |
  |---|---|---|
  | 120 deg (the naive 180-deg-away mirror) | ⚠️ **no** -- a 60 deg dead band, with the stance hock at 97.1 deg inside it | |
  | 60 deg (reflected across z, as the hip and knee do it) | yes | ⚠️ **13.83 mm** on a 14 mm arm -- one member all but vanishes |
  | **300 deg, i.e. the SAME anchor as the primary** | ✅ **yes** | ✅ **1.43 mm** |

- ✅ **Option A must be built as a CAPSTAN** -- both cables to the same anchor
  point, wrapping opposite sides of the sheave; physically one cable round a pin
  with a motor on each end. That makes the two wraps exact mirrors of each other, so
  they **reverse together and stay opposite**, which turns ADR-0049's reversal from
  a fatal property into a harmless one.
- The hip and knee get away with mirroring because their cable arrives from a
  distant spool, so the geometry really is symmetric about z. The ankle's arrives
  from a via-pulley on the tibia and is not. **A construction that works at one
  joint is not a construction that works at every joint.**

### Then: most of the evidence favoured a STIFFER SPRING, not Option A

- `params` specifies the return spring at **0.3 N.m/rad**. Stiffened, and referenced
  at the stance hock (which ADR-0049 fixed), it holds the unloaded leg:

  | k3 (N.m/rad) | 0.3 | 1.0 | 2.0 | 4.0 | **8.0** | 16.0 |
  |---|---|---|---|---|---|---|
  | ankle drift | -14.62 deg | -4.55 | -2.29 | -1.15 | **-0.58** | -0.29 |
  | peak tension | 58.6 N | 38.7 | 27.3 | 17.7 | **13.3** | 11.3 |

- ✅ **~8 N.m/rad reaches Option A's holding performance (0.00 deg at 12.2 N)
  without a single extra motor.** Call it **Option B'**.
- ⚠️ Its apparent cost is travel: at the 223 N motor peak the lone tendon moves the
  ankle **-196 deg from stance at k3 = 0.3 but only -24.9 deg at k3 = 8.**
- ✅ **But that turned out not to bind.** The trot only demands **8.6 deg (hind)
  and 13.4 deg (fore)** in the tendon's own pull direction, so even k3 = 8 leaves
  **2.9x** margin. On every measurement taken so far, B' was the better buy.

### And then the gait was asked, and Option B cannot do it

- ⚠️ **THE deciding measurement.** Over one trot cycle the ankle is commanded,
  relative to the stance hock:

  | leg | swing, below | swing, **ABOVE** | stance, above |
  |---|---|---|---|
  | fore | 13.4 deg | **+62.4 deg** | +54.8 deg |
  | hind | 8.6 deg | **+25.2 deg** | +14.1 deg |

- **Under Option B nothing drives the ankle above its reference.** The lone tendon
  pulls it *down* -- that is the moment-arm sign ADR-0049 had to choose to make
  standing possible at all -- and the spring only pulls it *toward* the reference.
  The above-reference excursion during **stance** is plausibly the ground
  dorsiflexing a loaded foot, which the tendon merely resists. ⚠️ **But in SWING
  the foot is unloaded and there is nothing left to do it.**
- And the spring **reference** is not a free parameter either: moving it up to the
  swing extreme makes the trajectory reachable and gives back ADR-0049's stance
  saving, which is what dropped the worst tendon from 222.9 N to 207.4.
- ✅ **Option A resolves it directly**, because its antagonist pulls the ankle up.
  It is the only option measured here that can command the gait this project already
  publishes.

### Decision, and what it costs

- **Adopt ADR-0002 Option A at the ankle, capstan construction.**
- ⚠️ **Mass: four more motors at 132 g = +528 g on ADR-0046's 4.3041 kg, so a
  projected 4.83 kg (+12.3 %)**, before spools, cables and drivers. NFR5's history
  becomes 3.0 -> 4.05 ([ADR-0010](#adr-0010)) -> 4.31 ([ADR-0043](#adr-0043)) ->
  **4.83**. A domestic cat is 4-5 kg, so it is inside the band, at the top of it.
  Actuator count **19 -> 23**.
- ⚠️ **The compiled plant's mass does NOT move**, because the motors live in
  `trunk_mass` and the sheaves are massless by [ADR-0048](#adr-0048). This cost has
  to be carried in the budget by hand, and there is a test asserting exactly that so
  nobody reads the unchanged 4.3081 kg as Option A being free.
- ✅ **What it buys, besides the gait:** the unloaded ankle holds to **0.00 deg at
  12.2 N** against Option B's -14.6 deg at 58.6 N; ankle restoring stiffness doubles
  (11.4 -> 22.5 N.m/rad); and ADR-0049's moment-arm reversal stops mattering.
- `params.spring_stiffness[2]` is left at 0.3 and **not** folded in. Option A removes
  the spring; `ANKLE_SPRING_TO_HOLD = 8.0` records what Option B' would have needed,
  because a rejected option is worth keeping costed.

### Two limits of this analysis, stated rather than buried

- ⚠️ **This plant has NO SPOOL DEGREE OF FREEDOM, so the pair's TRAVEL cannot be
  measured here.** A tendon's length is purely a function of the joint angles, so a
  motor cannot pay cable out and a slack antagonist acts as a spring: driving one
  ankle tendon at 223 N against a *zero-commanded* antagonist, the antagonist
  stretched **2.13 mm and developed 273.8 N** -- more than the 222.9 N driving it --
  and the joint stalled at 8.9 deg.
  ✅ Nothing measured on this plant so far is affected, because moment arms, joint
  stiffness and pose-holding are all small perturbations about a pose where both
  cables are taut. **And the argument above is kinematic**, so it does not depend on
  travel either. But a dynamic swing test does, and that is the next prerequisite.
- ⚠️ **The standing-tension comparison is confounded**, and is not used above. The
  worst tendon climbs 207 -> 223 N under **both** options, because the ADR-0049
  driver has **no posture task**: with the foot pinned by contact a 3-joint leg has
  one internal DOF and nothing controls it, so the ankle sags ~5 deg and the required
  tension grows with it. A null-space posture term cut the sag (-6.4 -> -1.7 deg) but
  pushed the peak to the ceiling and destabilised above kp 10. That is its own work
  item, not an A/B discriminator.

### Consequences

- **ADR-0002's ankle question is closed after four milestones.** Option B was chosen
  on motor count; it lost on **kinematic reach**, which is the one cost nobody had
  looked at.
- ⚠️ **NFR5 rises to a projected 4.83 kg** and needs the same fold-in treatment
  ADR-0046 gave 4.30: the CAD, the mass closure and `params` all still say 4.3041.
- ⚠️ **The electronics grow by four drivers**, which nothing in `electronics/` has
  seen.
- ⚠️ **Next: spool DOFs**, so pay-out is representable and a swing-phase test can
  run; then the **posture task**; then the thermal case for a 205 N standing tendon
  that ADR-0049 left open.

## ADR-0051: A spool behind every cable -- the drivetrain is exact, and adding it exposed a controller the project does not have

- **Status:** Accepted, and **opt-in**: `spools=` is off by default, because every
  M42-M45 measurement was taken without it. **Removes the limitation
  [ADR-0050](#adr-0050) stopped on. Does NOT yet re-open ADR-0050's travel question,
  and the reason is a finding in its own right.**
- **Context:** ADR-0050 could not measure an antagonistic pair's **travel**, which
  is the thing Option A is meant to buy. A tendon's length was a pure function of
  the joint angles, so a motor could not **pay cable out** and a slack antagonist
  behaved as a spring: it stretched 2.13 mm, developed **273.8 N against the
  222.9 N driving it**, and stalled the ankle at 8.9 deg.

### What was built

- Each cable gains a **series-elastic drivetrain**, with every piece where it
  physically is:

      motor torque -> rotor -> torsional spring -> spool -> cable

  The actuator moves from the tendon to the **rotor joint**, so it is commanded in
  **N.m** rather than newtons, and the plant grows **two DOF per cable**. The series
  spring is `k_tors = k_series * r_spool^2`, so [ADR-0047](#adr-0047)'s G3 element
  is now specified in exact units and sits where it belongs -- between motor and
  cable -- rather than being a `springlength` deadband on the tendon.
- ✅ **The statics are exact.** Pin every leg joint and the spring must carry the
  motor's whole torque: measured deflection matches `-tau / k_tors` to **five
  decimal places** across three cables and two torques, and the tension lands on
  **222.9 N at the 1.95 N.m peak** -- ADR-0048's ceiling, now *arrived at* through a
  drivetrain instead of asserted.
- ✅ **And pay-out works.** A slack antagonist unwinds instead of resisting: the
  cable lengthens by exactly `r * theta` (**51.5 mm at -336 deg**, matching to the
  millimetre), and the driven tendon takes the ankle all the way to its **-30 deg
  end stop, 127 deg of travel**, against ADR-0050's 8.9 deg stall.

### Four traps, all of them quiet

- ⚠️ **MuJoCo's spatial tendons are MEMORYLESS ABOUT WINDING.** A cylinder is
  rotationally symmetric, so wrapping one and turning it changes the path by
  **nothing at all**, and a site carried on the rotor merely orbits. The wound
  length has to be carried **analytically**, by a `<fixed>` tendon on the spool
  angle tied to the geometric path by an equality. Three wrong constructions were
  built before this one, and each looked plausible.
- ⚠️ **An equality is itself a spring in series with whatever it couples**, because
  MuJoCo solves constraints in a normalised space. A loose one silently softens the
  drivetrain. Calibrated against a known series stiffness:

  | solref | solimp | k measured / k specified |
  |---|---|---|
  | 0.0005 1 | 0.9 0.95 0.001 0.5 2 | 0.717 |
  | 0.0002 1 | 0.9 0.95 0.001 0.5 2 | 0.939 |
  | **0.0002 1** | **0.99 0.9999 1e-6 0.5 2** | **0.998** |

  At the loose setting the equality contributed 3.8e5 N/m in series; at the tight
  one, 8.2e7, against the ~1.5e5 the cable actually has.
- ⚠️ **A stiff equality then OVERPOWERS a default-stiffness joint limit, and it
  does not overshoot -- it corrupts the answer.** Driving the ankle tendon at the
  motor peak sent `q3` to **215 deg against a 150 deg limit** and settled there:
  65 deg outside its own range and in the **wrong direction**. Solved as stiffly as
  the equality, it stops exactly on the -30 deg end stop.
- ⚠️ **A tendon equality is referenced at `qpos0`, not at the state the caller
  sets.** Build the rig with the joints at zero, start it at the stance pose, and
  every constraint begins **24-52 mm** out; the solver snaps the leg from 97 deg to
  39 deg in **5 ms** and then holds the wrong configuration perfectly, with the
  residuals sitting **constant** -- which reads exactly like a satisfied constraint
  until you notice what they are constant *at*. `single_leg_rig_spooled` measures
  the offset in a first pass and puts it in `polycoef`'s `a0`; the residual is then
  **1e-10**.

### What it exposed, and why M46 stops here

- ⚠️ **Adding the missing degree of freedom exposed a MISSING CONTROLLER.** With
  the actuator on the rotor rather than the tendon, commanding a tension is no
  longer instantaneous: it arrives through a series-elastic mode at
  `sqrt(k_tors / I_rotor)` = **758 rad/s, about 120 Hz**. **Every controller this
  project has commands tension directly** -- [ADR-0047](#adr-0047)'s position loop,
  [ADR-0049](#adr-0049)'s whole-body allocation, ADR-0050's comparisons -- and none
  of them can drive this plant. On the old plant an outer position loop holds the
  hip and knee to **0.00 deg**; here the same loop with a hand-tuned inner tension
  loop leaves **5-10 deg**.
- ⚠️ **And it is not numerical.** Refining the timestep **20x** (1e-4 -> 5e-6)
  changes the answer by under 2 %, and the `implicit` integrator by less. A motor
  commanded to zero torque, on a spool with almost no inertia, dragged by a cable,
  really does spin at hundreds of rad/s. **Open-loop torque on one motor with the
  rest at zero is not an experiment a tendon robot can perform**, and that is the
  honest reason no travel figure beyond the end-stop case is published here.
- ⚠️ **A false lead, recorded because it was checked and is worth not re-checking:**
  the ringing looked at first like the **capstan** closing a kinematic loop
  (ADR-0050's construction anchors both ankle cables at one point). It is not -- the
  **mirrored** hip pair rings harder, at 3862 rad/s against the capstan's 294. Any
  antagonistic pair with an unactuated antagonist has an undamped co-contraction
  mode; two motors, two cables and one joint leave one redundant coordinate, and
  nothing damps it unless a controller does.

### Consequences

- **`spools=` is opt-in and the default plant is untouched**, so ADR-0047 through
  ADR-0050 remain reproducible exactly as published.
- ⚠️ **ADR-0050's ankle decision is NOT revisited.** Its argument was **kinematic**
  -- the trot commands the ankle 62.4 deg above its reference in swing and Option B
  has nothing that pulls that way -- and nothing here touches that. What M46 was
  meant to add was the dynamic confirmation, and that now waits on the controller.
- ⚠️ **Next: the cascade the plant requires** -- a tension loop inside, a joint loop
  outside, separated by the 120 Hz series-elastic mode. That is a well-posed control
  design task, and it is the first one this project has had that is genuinely about
  the *drivetrain* rather than the body.
- ⚠️ **`ROTOR_ARMATURE` is `[assumed]` at 2e-5 kg.m^2** and now matters: it sets
  that mode. It joins the vendor's Kt reference point as an actuator number the
  project owes itself.
- The quadruped is not spooled yet. M46's question was a single-leg question, the
  way M42 gated one leg before M43 took the body.

## ADR-0052: The drivetrain cascade -- derived, not tuned; and a firmware gain sets how much of a mechanical spring you get

- **Status:** Accepted. `spool_servo=` ships in `mjcf_tendon.py` and
  `wbc.rotor_command` in `wbc.py`. **Closes the gap [ADR-0051](#adr-0051) stopped
  on. Answers [ADR-0050](#adr-0050)'s open question, qualified. Puts a firmware
  number under [ADR-0047](#adr-0047)'s G3 specification.**
- **Context:** ADR-0051 built a spool behind every cable and then could not drive
  it: with the actuator on the rotor rather than the tendon, commanding a tension is
  no longer instantaneous but arrives through a series-elastic mode at **758 rad/s
  (~120 Hz)**, and every controller in the project commands tension **directly**. A
  hand-tuned attempt left 5-10 deg of joint drift where the old plant held 0.00.
  ADR-0051 stopped rather than keep guessing gains.

### The cascade, in closed form

- **Inner loop: a rotor POSITION servo.** A real motor brings an encoder and a
  current loop, so the natural inner loop is position, not torque. For a critically
  damped rotor of inertia `I` at bandwidth `wn`, `kp = I*wn^2` and `kv = 2*I*wn`;
  at the default 3000 rad/s that is **kp 180 N.m/rad, kv 0.12**. Nothing tuned.
- **The command law.** The winding constraint makes `r*(theta_r + theta_s)` the
  wound length and the series spring acts on `theta_s`, so
  `T = (k_tors/r)*(theta_r - theta_r_at_zero_tension)` -- and the zero-tension rotor
  angle reads **straight off the state** as `theta_r + theta_s`. Hence

      theta_r_desired = (theta_r + theta_s) + T_desired * r / k_tors

  ✅ **No reference offset is needed at all**, which is a pleasant contrast with
  the `qpos0` trap ADR-0051 had to work around for the constraint itself.
- **Outer loop:** unchanged -- the joint PD and non-negative tension allocation the
  project already had ([ADR-0049](#adr-0049)'s `wbc.tendon_tension`).
- ✅ **It holds the stance pose to 0.00 deg** at every joint gain from 10 to 50,
  where ADR-0051's hand-tuned attempt left 5-10.

### ⚠️ A firmware gain sets how much of a mechanical spring you get

- The servo has finite stiffness `kp`, and it sits in **series** with the G3 spring.
  Command a rotor angle for a target tension and the tension comes out low by
  exactly `kp/(kp + k_tors)`:

  | rotor bandwidth | kp (N.m/rad) | raw error | delivered series stiffness |
  |---|---|---|---|
  | 1000 rad/s | 20.0 | **-36.5 %** | **95 285 N/m** -- ⚠️ outside ADR-0050's band |
  | 2000 | 80.0 | -12.6 % | 131 170 |
  | 3000 | 180.0 | -6.0 % | 141 004 |
  | 6000 | 720.0 | -1.6 % | 147 645 |

- ✅ **The droop compensates exactly**: multiplying the command by
  `(kp + k_tors)/kp` brings the tension to **-0.1 % at every gain tried**, including
  the one that was 36 % out. `wbc.rotor_command` takes `servo_kp` for this.
- ⚠️ **But the DELIVERED stiffness cannot be compensated away.** ADR-0047 sized G3
  at ~175 kN/m and ADR-0050 banded it at 150-200. At a **1000 rad/s** rotor loop the
  drivetrain delivers **95 kN/m** -- outside that band, from a firmware gain alone.
  **G3 cannot be specified without the servo bandwidth beside it**, and that is a
  new coupling between the mechanical and firmware sides of this project.

### ADR-0050's open question, answered and qualified

- ADR-0050 decided the ankle on **kinematic reach** and could not confirm it
  dynamically. With the cascade it can be asked, and the answer splits on
  [ADR-0049](#adr-0049)'s **moment-arm reversal**.
- ⚠️ On the shipped 300 deg anchor the hind reversal sits at **~112 deg, INSIDE**
  the hind ankle's 88.5-122.3 deg gait range, and the cascade cannot cross it:
  commanded 110 deg it reaches 97.4; commanded 122.3 it reaches 85.6. ⚠️ And with a
  `G` frozen at the target pose instead of refreshed, 122.3 deg sends the joint to
  the **opposite end stop at -30 deg** -- the arm changes sign under the controller.
- ✅ **Move the anchor to 270 deg and the whole range tracks**, worst error
  **3.48 deg**. So an antagonistic pair *can* do what ADR-0050 claimed; the reversal
  has to be moved out of the gait range first, and ADR-0049/ADR-0050's criterion --
  reversal outside the **stance pose** -- was too weak. The criterion is: **outside
  the whole GAIT range, for each leg.**
- ⚠️ **And no single angle serves both legs.** Their hocks stand **81 deg apart**
  (hind +97.1, fore +16.4), so their gait ranges sit in different parts of the
  sheave. Swept against the stricter criterion: **hind 270 deg** (1.57 mm worst arm
  error), **fore 300 deg** (0.30 mm). The fore leg being "inherited rather than
  mirrored", flagged since [ADR-0048](#adr-0048), is finally forced -- and the fix is
  a number per leg, not a mirrored construction.
- ⚠️ **M47 measures the migration and does not ship it.** Adopting 270 deg for the
  hind breaks **14 tests across M44, M45 and M46**: every ankle measurement in three
  milestones was taken on the 300 deg anchor. That is its own piece of work with its
  own re-derivation, and M47's deliverable -- the cascade -- does not depend on the
  anchor at all. Both numbers are recorded in `MT._ankle_anchor_deg` so the
  migration starts from a measurement rather than a re-sweep.

### Two smaller things

- ⚠️ **`G` must be refreshed as the pose moves.** Not a preference: with the
  reversal inside the range, a `G` measured once at the target sends the joint to the
  wrong end stop. ADR-0049's whole-body driver already refreshes every 25 steps; this
  says why it has to.
- ⚠️ **The fore leg tracks loosely**, worst error **11.8 deg** against the hind's
  3.5, on the same gains. Different link lengths and a stance hock 81 deg away make
  it a different plant; per-leg gains are owed and are not in this milestone.

### Consequences

- **The spooled plant is driveable**, and the three pieces that make it so all have
  closed forms. `wbc.rotor_command` is what firmware implements.
- ⚠️ **G3's specification now needs a servo bandwidth attached**: 175 kN/m of
  spring delivers 141 kN/m behind a 3000 rad/s rotor loop and 95 behind a 1000 one.
- ⚠️ **Next: the ANKLE ANCHOR MIGRATION** -- hind 270 deg, fore 300 -- and the 14
  tests it re-derives. It is the last thing between ADR-0050's decision and its
  dynamic confirmation.
- ⚠️ Then per-leg outer gains, and the quadruped, neither of which M47 touched.

## ADR-0053: The fore leg's routing was never mirrored -- and it reorders the ankle-anchor migration

> ⚠️ **NARROWED TOO FAR. [ADR-0054](#adr-0054) (M49) has the general statement.**
> This ADR read the defect as *"the fore leg was inherited rather than mirrored"*.
> The fore leg is the worse of the two, but the real finding is that **the hip and
> knee routings were never validated across the gait range on EITHER leg**, and both
> fail. The fore leg merely has the bad luck that its stance pose sits **inside** its
> own failure band while the hind's sits outside -- which is exactly why five
> milestones of stance-pose checks saw nothing. Everything measured below stands;
> the diagnosis is the part that was too narrow.

- **Status:** Accepted as a **finding and a re-ordering**, with the diagnosis
  narrowed by ADR-0054. No geometry ships. **The
  fore-leg defect it records invalidates fore-leg figures in
  [ADR-0048](#adr-0048) through [ADR-0052](#adr-0052). It defers ADR-0052's ankle
  anchor migration, with every number for it measured.**
- **Context:** ADR-0052 established the criterion for the ankle anchor -- the moment
  arm must not reverse anywhere inside a leg's **gait** range -- and measured the
  angles that satisfy it: **hind 270 deg, fore 300** against the shipped 300 for
  both. M48 is that migration. Applying it and then measuring the result **on both
  legs** turned up something larger, because every tendon Jacobian since
  [ADR-0047](#adr-0047) had been taken on the **hind** leg alone.

### ⚠️ The finding: the fore leg has no hip antagonist

- | tendon | hind hip | fore hip | spec |
  |---|---|---|---|
  | hip flexor | **+28.000** | **+11.636** | +/-28 |
  | hip extensor | **-28.000** | **+35.885** | +/-28 |

  **Both fore arms are positive**, so the two cables pull the joint the same way and
  there is no antagonist. The rest of the fore rows are as wrong: `knee_ext` puts
  **+44.067** on the hip where the via-pulley says -8.75, and `ankle` **-39.909** on
  the knee where it says -8.75. The hind leg is exact.
- ⚠️ **This has been true since M42.** The hip's moment arms cannot depend on the
  ankle anchor, and the same numbers come out of the committed M47 tree. It is the
  thing [ADR-0048](#adr-0048) flagged and nothing acted on: `DEFAULT_FORELEG` folds
  the **opposite** way, and the fore leg's sidesites and anchor angles were
  **inherited from the hind leg rather than mirrored**. ADR-0048 measured the symptom
  as a fore/hind drift gap and read it as a tuning difference. It is not.
- **What rests on it:** every fore-leg figure in M43-M47 -- the welded and floating
  drift, ADR-0049's *"the binding tendon is the hind hip extensor"* (with the fore
  leg fixed it may not be), ADR-0052's *"the fore leg tracks to 11.8 deg against the
  hind's 3.5"*. ✅ **ADR-0050's ankle decision does not**: its deciding argument
  came from `gait.py`'s joint trajectories, which never touch the MJCF routing.
- There is a test asserting the defect, so it fails the moment the fore leg is
  mirrored properly.

### What the migration measured, on the hind leg, before it was deferred

- | | shipped 300 deg | migrated 270 deg |
  |---|---|---|
  | reversal vs the 88.5-122.3 deg gait range | ⚠️ inside (~112 deg) | ✅ outside |
  | cascade tracks 122.3 deg | ⚠️ reaches 85.6 | ✅ **121.6** |
  | lone ankle tendon at the stance pose | plantarflexes | ⚠️ **dorsiflexes** |
  | quadruped, lone ankle | stands, 0.01 deg | ⚠️ **inverts, 179.75 deg** |
  | quadruped, ankle PAIR | stands | ✅ **stands, 0.01 deg** |

- ✅ **The migration FORCES [ADR-0050](#adr-0050)'s Option A rather than merely
  preferring it.** At the migrated anchor a lone ankle tendon reaches the gait range
  and **cannot stand at all**. ADR-0050 decided Option A on kinematic reach; the
  geometry that reach requires makes it structural.
- ✅ **And the ankle finally clears [ADR-0026](#adr-0026)'s compliance floor**:
  **86.3 N.m/rad** with the pair against 55.1 with one tendon -- the first time this
  joint has met the 80 the balance work has wanted since M20, and the answer to
  ADR-0047's *"a lone-tendon joint has no restoring stiffness"*.

### ⚠️ Three control findings retract on the migrated plant

- All three were consequences of an **under-actuated ankle**, not of a control law.
  With a full antagonistic set every joint has both directions and the non-negative
  allocation is never tight:

  | finding | as published | on Option A, migrated |
  |---|---|---|
  | ADR-0047: gravity feedforward cannot hold a pose | diverges at every co-contraction level | **0.0001 deg** |
  | ADR-0049: clipping instead of solving loses the leg | 197 deg of hip drift | **0.0002 deg** |
  | ADR-0049: the bare cable inverts the robot, so G3 is what makes it stand | tilt 180 deg | **stands, 0.07 deg** |

- ⚠️ The third is the one that matters most, because ADR-0049 used it as an
  **independent confirmation of design goal G3**. That confirmation does not
  survive: with an antagonistic ankle the 5x-stiffer bare cable stands. G3's
  *balance-compliance* argument (ADR-0047, from ADR-0026's `kp` window) is untouched
  and remains the reason to specify it -- but it is one argument again, not two.
- ⚠️ **And a caution the other way.** On Option B at the migrated anchor, **clipping
  beats NNLS** (0.0001 deg against 46.6): with a lone tendon whose sign is wrong for
  the load, the honest minimum-residual solution drives the ankle away while clipping
  happens not to. **A controller comparison on an infeasible plant measures the
  plant.** ADR-0049's clipping result was such a comparison.

### Decision

- **Nothing ships.** The migration re-derives **17 tests across M42-M47**, and the
  fore leg those tests measure has never been mirrored. Re-deriving on a
  known-broken leg would bake the wrong numbers in.
- **The order is: mirror the fore leg, then migrate the anchor, then re-derive
  once.** Both anchor angles stay recorded in `MT._ankle_anchor_deg`, and the
  measurements above stay here, so neither step starts from a re-sweep.
- ⚠️ **G3's second argument is withdrawn now, not later**, because it is a claim
  about a design target and does not depend on the migration landing.
- ✅ **The ankle meeting 86.3 N.m/rad is recorded now too**, for the same reason,
  even though it takes effect only with Option A and the migration.

### Consequences

- ⚠️ **Mirroring the fore leg is the next milestone**, and it is larger than it
  looks: sidesites, anchor angles and the ankle anchor all have to be re-derived for
  a leg that folds the other way, against ADR-0052's gait-range criterion.
- ⚠️ **Every fore-leg number in ADR-0048 through ADR-0052 carries a caveat** until
  then. The hind-leg numbers, which are most of what those ADRs argue from, do not.
- ⚠️ **A process finding:** five milestones measured one leg and generalised. The
  Jacobian check that found this is three lines and had never been run on the fore
  leg. It is now a test.

## ADR-0054: Only the ankles were ever validated across the gait -- the hip and knee fail on both legs

- **Status:** Accepted as a **finding, with three of four fixes measured**. No
  geometry ships. **Narrows [ADR-0053](#adr-0053)'s diagnosis. Applies
  [ADR-0052](#adr-0052)'s criterion to every joint for the first time. Blocks the
  ankle-anchor migration behind itself.**
- **Context:** ADR-0053 found the fore leg's hip pair does not oppose and read it as
  an un-mirrored routing. M49 set out to mirror it, and started by applying
  ADR-0052's criterion -- the pair must stay **opposing**, with both arms near
  specification, **everywhere in the range the gait commands** -- to all six
  routings. Nobody had done that for the hip or the knee.

### ⚠️ The audit

  | leg | joint | gait range | same-sign points | worst arm error |
  |---|---|---|---|---|
  | hind | hip | -110.0..-35.1 deg | ⚠️ **6 of 13** | 8.38 mm |
  | hind | knee | -114.7..-54.7 deg | none | ⚠️ **9.91 mm** on 25 |
  | hind | ankle | 88.5..122.3 deg | none | ✅ **0.32 mm** |
  | fore | hip | -168.8..-141.2 deg | ⚠️ **5 of 13** | ⚠️ **26.06 mm** on 28 |
  | fore | knee | 33.4..103.8 deg | none | ⚠️ **23.81 mm** |
  | fore | ankle | 3.0..78.8 deg | none | ✅ **0.28 mm** |

- ✅ **Only the ankles pass** -- and the ankles are the only joints anyone ever
  swept against a range criterion, across M42, M44 and M45. The hip and knee anchors
  were placed by M42's 2-D heuristic and checked **at the stance pose only**.
- ⚠️ **So ADR-0053's diagnosis was too narrow.** The fore leg is worse, but the
  statement is *"the hip and knee were never validated across the gait, on either
  leg"*. The fore leg merely has the bad luck that its hip stance pose (**-147.8
  deg**) sits **inside** its own failure band while the hind's (**-49.2 deg**) sits
  outside. **That is why five milestones of stance-pose checks saw nothing**, and it
  is the reusable lesson: a check at one operating point is not a check.

### Three of four have a measured fix

- Applying [ADR-0050](#adr-0050)'s **capstan** construction and sweeping the anchor:

  | leg | joint | anchor | worst arm error |
  |---|---|---|---|
  | hind | knee | **75 deg** | 0.28 mm |
  | fore | hip | **135 deg** | **0.00 mm** |
  | fore | knee | **285 deg** | 0.27 mm |
  | hind | hip | ⚠️ **none exists** | -- |

- ⚠️ **The hind hip has no solution.** No capstan angle works; no **mirrored** angle
  works either (every one leaves 3-12 same-sign points of 13); and no girdle-spool
  offset swept helps (+/-40 mm in x, +/-20 in z; the best left 1 of 13 same-sign at
  12.26 mm). Its gait range is **74.9 deg wide, 2.7x the fore hip's 27.6**, and the
  sheave construction's working window is narrower than that.
- The lever it needs is a different **construction**, not a different number: a
  larger hip sheave, a via-pulley at the girdle so the incoming direction turns with
  the leg, or a narrower hip excursion from the gait. ⚠️ [ADR-0052](#adr-0052)
  already flagged the girdle spools as *"placed by the packaging study, not by
  routing"*; this is the same lever and it is not enough on its own.

### Decision

- **Nothing ships**, for the reason ADR-0053 gave and this ADR strengthens: the
  re-derivation touches 17 tests across M42-M47, and it must happen **once**, after
  the geometry is settled. The hind hip is not settled.
- **The three measured anchors stay recorded here**, so the milestone that lands them
  starts from a measurement rather than a re-sweep -- as do ADR-0053's two ankle
  angles.
- There is a test asserting the audit, so the moment any routing is re-derived it
  fails and the table has to be re-run.

### Consequences

- ⚠️ **The hind hip is now the critical path.** Everything else -- the ankle-anchor
  migration, the fore-leg re-derivation, ADR-0050's dynamic confirmation -- queues
  behind a construction decision nobody has had to make.
- ⚠️ **Every hip and knee figure in M42-M47 carries a caveat**, on both legs, not
  just the fore. That includes the emergent-moment-arm table this project has cited
  since M42: it is exact **at the stance pose**, which is where it was measured, and
  not across the gait.
- ✅ **The ankles are sound**, on both legs, and that is not luck -- it is the one
  place a range criterion was applied. The lesson generalises: **sweep against the
  range the thing actually operates over, not the pose you happen to model.**

## ADR-0055: A resting wrap has a working window; a clamped capstan does not

- **Status:** Accepted as the **construction decision** [ADR-0054](#adr-0054) said
  was needed. No geometry ships yet -- the re-derivation still happens once -- but
  the question of *what construction* is settled. **Qualifies
  [ADR-0047](#adr-0047)'s emergent-moment-arm finding. Removes the cause behind
  ADR-0054's audit, [ADR-0052](#adr-0052)'s anchor sweeps and
  [ADR-0049](#adr-0049)'s moment-arm reversal.**
- **Context:** ADR-0054 found the hip and knee fail across the gait on both legs,
  and that the **hind hip has no anchor angle that works** at any sweep tried. It
  called for a different construction. This is that construction, and it explains
  every routing finding since M42.

### The mechanism

- A cable that merely **rests** on a sheave gives the sheave's radius only while it
  actually wraps. Outside that the arm is whatever the straight line happens to
  give:

  | q1 | resting wrap | clamped capstan |
  |---|---|---|
  | -120 deg (past the gait range) | 12.956 mm | 28.000 |
  | **-110 deg** (its far edge) | **21.034** | 28.000 |
  | -90 deg | 31.979 | 28.000 |
  | **-60 deg** | **36.332** | 28.000 |
  | -10 deg and above | 28.000 | 28.000 |

- Across the hind hip's **-110..-35.1 deg** gait range the resting arm swings
  **21.03 to 36.33 mm -- 1.73x, on a 28 mm specification** (0.75x spec at one end,
  1.30x at the other). ✅ **The clamped one is exactly 28.000 at every angle**,
  because the cable is *fixed to the sheave* rather than resting on it, so the arm
  is a property of the construction and not of contact.
- ✅ **And the wrapped length it costs is trivial**: `r * dq` over the hind hip's
  range is **36.7 mm** against a **176 mm** circumference -- **21 % of one turn**.

### What it explains

- ✅ **ADR-0054's audit.** The hip and knee fail across the gait because a resting
  wrap has a window and their windows are narrower than their gait ranges. The
  ankles pass because M42-M45 swept them *into* their windows -- which is why the
  one criterion anyone applied happened to work.
- ✅ **ADR-0049's moment-arm reversal.** A clamped cable cannot reverse; the
  reversal is a property of resting contact. So is the "dead spot" ADR-0047 found
  and ADR-0048 retracted.
- ✅ **The whole anchor-sweep programme** -- ADR-0052's criterion, ADR-0053's two
  ankle angles, ADR-0054's three -- is a search for the corner of a window. With
  clamped capstans there is no window and no sweep.
- ⚠️ **And it qualifies ADR-0047's headline.** *"The moment arm is emergent from the
  geometry"* is true **where the cable wraps**, which for the hip is a fraction of
  its range. As a validation that a wrap reproduces the sheave radius, that finding
  stands and is worth having. As the model for a wide-ROM joint it does not, and
  five milestones of stance-pose checks never noticed because the stance pose sits
  inside the window.

### The two alternatives, priced and rejected

- **A larger hip sheave.** Swept against the real gait range with the anchor swept at
  5 deg, only **r = 50 mm** comes out clean (0 same-sign points, 0.00 mm error).
  ✅ It would also cut ADR-0052's standing tension from **205 N to 110 N**, a real
  secondary benefit. ⚠️ But that is a **100 mm diameter sheave on a 90 mm femur --
  1.11x the segment it sits on**, and the largest manufacturable size swept
  (r = 36 mm) still leaves 1 of 17 points same-sign.
- **A narrower hip excursion.** The window ends near **-95 deg** and the trot
  commands **-110**, so the gait would give up ~15 deg of hip extension: at a 90 mm
  femur, **35 mm of stride against a 100 mm `stride_length`** -- ⚠️ **35 %**, and at
  fixed cadence 35 % of the speed.
- Clamping costs neither. What it costs is a **cable fixed to each sheave** -- a
  groove and a ferrule or set screw -- which `mechanical/` does not currently draw.

### Decision

- **Every antagonistic joint becomes a CLAMPED capstan.** The cable is terminated on
  the sheave, not routed over it, and the moment arm is `r` by construction.
- ⚠️ **In the model this is an analytic `r * q` term rather than a wrap geom**, the
  same way M46's spool carries its wound length. That is a real modelling change and
  it trades emergence for exactness -- deliberately, because the emergent version is
  only correct inside a window that the gait leaves.
- **Nothing ships in this ADR.** ADR-0053's and ADR-0054's rule stands: the
  re-derivation touches 17 tests across M42-M47 and happens **once**. What M50 adds
  is that it now has a construction to re-derive *to*.

### Consequences

- ⚠️ **`mechanical/` owes a cable termination at every sheave.** ADR-0043's leg was
  drawn with cables resting in grooves; they now have to be clamped, which is a
  fitting, a groove profile and an assembly step per joint.
- ✅ **The anchor angles ADR-0053 and ADR-0054 measured become unnecessary** for
  any joint that clamps. They stay recorded, because a joint that stays a resting
  wrap still needs them.
- ⚠️ **The 205 N standing tendon is untouched** by this. The larger sheave would
  have halved it; clamping does not. That remains ADR-0052's open thermal item.
- ✅ **And the reusable lesson from ADR-0054 gets its mechanism**: a check at one
  operating point is not a check, *because* contact-dependent geometry has windows,
  and a window is invisible from inside it.

## ADR-0056: The simulation never implemented ADR-0008's variable-radius pulley -- and the standing tension never converges

> ✅ **BOTH BLOCKING FINDINGS ARE CLOSED by [ADR-0058](#adr-0058) (M53).** The
> pulley is implemented, so the 5.36/5.89 kg projections below are **withdrawn** and
> the body stays at 4.3041 kg. And the tension wind-up **was co-contraction growing
> with the uncontrolled drift**: on the pulley transmission the same gate converges
> at **68.2 N** and holds it for 12 s, inside the 81 N continuous rating, where the
> independent-pair plant pinned at the 222.9 N ceiling by 9 s. ⚠️ The thermal item
> is therefore unblocked by the **transmission**, not by the posture task this ADR
> expected -- and the trunk sags **3.1 mm against 0.2**, the preload the
> co-contraction floor had been supplying.

- **Status:** Accepted as **three findings from a housekeeping pass**, two of which
  blocked the item they were meant to close. No geometry or parameter ships.
  **Withdraws [ADR-0050](#adr-0050)'s mass costing. Corrects
  [ADR-0049](#adr-0049)'s 205 N. Puts a banner on [ADR-0008](#adr-0008).**
- **Context:** three items had accumulated that were independent of the routing
  work: the suite had grown to 12 minutes, ADR-0050's 4.83 kg was never folded in,
  and ADR-0049's 205 N standing tendon had no thermal case. M51 is that pass. One
  closed; two turned out to be blocked, and the blockers are the findings.

### ⚠️ The simulation never implemented ADR-0008's variable-radius pulley

- ADR-0008 is **the mass-closure decision**. At the counts the architecture called
  for, the motors alone exceeded the whole body (24 = 105 %, 31 = 136 % of 3 kg) and
  **the design did not close**. Its decision was **one motor per antagonistic pair**
  via a variable-radius pulley -- 16 motors, amended to **19** by
  [ADR-0009](#adr-0009). `params.py` builds `trunk_mass` on exactly that: *"6 leg
  motors x 132 g"* per girdle, **12 leg motors**, one per DOF.
- ⚠️ **`mjcf_tendon.py` emits one motor per TENDON.**

  | | leg motors | unbudgeted | body |
  |---|---|---|---|
  | ADR-0008 budget, as `params` carries it | **12** | -- | 4.304 kg |
  | the simulation, lone ankle | **20** | **1.054 kg** | **5.358 kg** |
  | the simulation, ankle pair (ADR-0050) | **24** | **1.580 kg** | **5.885 kg** |

- So **M42 through M50 were built on the architecture ADR-0008 rejected**, and
  rejected on mass grounds specifically. A domestic cat is 4-5 kg, the band NFR5 has
  been anchored to since [ADR-0010](#adr-0010); **5.36 and 5.89 are outside it**.
- ⚠️ **And it makes ADR-0050's costing wrong twice.** *"+4 motors, +528 g, 4.83
  kg"* assumed the baseline was the simulation's 20 and that `params` carried them.
  It carries 12, so the delta from the budget is **+1.58 kg**. ✅ The ankle
  *decision* is unaffected -- it was made on kinematic reach, not mass -- but the
  number attached to it is not the cost.
- **This is why 4.83 kg was not folded in.** The fold-in is not an addition, it is a
  reconciliation of two architectures, and it has to be decided first: implement the
  variable-radius pulley (which ADR-0008 says, and which M46's spool machinery can
  express -- one rotor, two spools), or re-open ADR-0008 at 5.36 kg.

### ⚠️ The standing tension never converges -- it saturates

- ADR-0049 reported the hind hip extensor at *"~205 N mean, 2.5x continuous"* and
  left the thermal case open. **That number is not a steady state.** Run the same
  gate longer:

  | t | peak tension | trunk z | tilt |
  |---|---|---|---|
  | 1 s | 125.5 N | 0.17589 | 0.005 deg |
  | 4 s | 164.8 | 0.17586 | 0.005 |
  | 6 s | 194.3 | 0.17582 | 0.006 |
  | **9 s** | **222.9** -- the motor ceiling | 0.17578 | 0.007 |
  | 12 s | 222.9 | 0.17573 | 0.006 |

- **It ramps monotonically to the actuator's peak rating and pins there.** ADR-0049's
  205 N was where the ramp had reached inside its 1-3 s window.
- ✅ **The cause is measurable and is the one [ADR-0052](#adr-0052) named**: the
  tension tracks an uncontrolled joint drift. Hind-right drift against peak tension:
  **2.9 deg / 125 N** at 1 s, **3.8 / 179** at 5 s, **4.3 / 217** at 7.5 s, then both
  stop when the motor saturates. The trunk holds to 0.007 deg throughout, so this is
  a **wind-up, not a fall**.
- **So the thermal item is blocked behind the posture task**, and the honest
  statement is not *"2.5x the continuous rating"* but *"the tension does not settle;
  it saturates the actuator"*.

### ✅ The suite is three times faster, and nothing was loosened

- The dominant cost was not the simulated horizon but **re-measuring each leg's
  tendon Jacobian every 25 steps** -- 24 forward passes per refresh per leg.
  Measured on the standing gate, where the pose barely moves:

  | refresh | wall | z_end | tilt | residual |
  |---|---|---|---|---|
  | 25 | 36.3 s | 0.17579 | 0.006 deg | 0.0170 |
  | 250 | **18.4 s** | 0.17587 | 0.005 | 0.0175 |
  | 500 | 17.9 s | 0.17588 | 0.005 | 0.0145 |

- ⚠️ **It is not a free constant.** A test that *moves* the pose must keep 25:
  ADR-0052 measured a Jacobian frozen across the ankle's moment-arm reversal driving
  the joint to the opposite end stop. So the code carries **two** constants with the
  reason attached, `REFRESH_STATIC` and `REFRESH_MOVING`, not one number tuned down.
- `test_mjcf_tendon.py` **306 s -> 95 s**; the whole suite **12:21 -> 4:41**. No test
  shortened, no assertion loosened.

### Consequences

- ⚠️ **The motor-count reconciliation is now the second thing on the critical
  path**, beside M50's clamped-capstan rebuild -- and they touch the same file. The
  variable-radius pulley is a transmission, so it belongs in the same rebuild.
- ⚠️ **NFR5 is unresolved, in the opposite direction from ADR-0050's projection.**
  Not 4.83 kg but either 4.30 (pulley implemented) or 5.36-5.89 (not).
- ⚠️ **Every power, thermal and runtime figure rests on the 12-motor budget** while
  every simulation result rests on 20. Neither is wrong on its own terms; they are
  not the same robot.
- ✅ **A process note.** Both blocked items were blocked by something the pass was
  not looking for, and both were found by asking *"does this number converge?"* and
  *"where does this number come from?"* -- the same two questions that found
  ADR-0054's audit.

## ADR-0057: The clamped transmission, built -- and ADR-0002 and ADR-0008 want different robots

- **Status:** Accepted. The clamped-capstan transmission ships as `clamped=True` in
  `mjcf_tendon.py`. **Implements [ADR-0055](#adr-0055). Passes
  [ADR-0054](#adr-0054)'s audit outright. Sharpens [ADR-0056](#adr-0056)'s
  motor-count finding into a conflict between two accepted decisions, and does not
  resolve it.**
- **Context:** ADR-0055 settled that every antagonistic joint should be a **clamped
  capstan** rather than a resting wrap, because a resting wrap's moment arm is the
  sheave radius only inside a window the gait leaves. ADR-0056 then found the
  simulation runs 20 leg motors where the mass budget carries 12. M52 builds the
  first and takes the measure of the second.

### ✅ The clamped transmission is exact

- Clamped, a cable's length is exactly `sum r_j * q_j` over the joints it crosses --
  a `<fixed>` tendon, not a routed path. Built:

  | | hip pair | knee pair | ankle pair | couplings |
  |---|---|---|---|---|
  | hind, stance | +/-28.000 | +/-25.000 | +/-14.000 | -8.750 |
  | hind, q1 = -110 deg | +/-28.000 | +/-25.000 | +/-14.000 | -8.750 |
  | fore, stance | +/-28.000 | +/-25.000 | +/-14.000 | -8.750 |
  | fore, q1 = -168 deg | +/-28.000 | +/-25.000 | +/-14.000 | -8.750 |

  Against the wrapped construction ADR-0054 audited: hind hip same-sign at 6 of 13
  gait samples, fore hip at 5, fore knee **23.81 mm out on a 25 mm specification**.
- ✅ **And it holds on gravity feedforward alone** -- 0.00 deg at `kp = 0`, on a
  co-contraction floor, and at every joint gain tried. With every pair exact the
  non-negative allocation is never tight, which is the same reason
  [ADR-0053](#adr-0053) retracted ADR-0047's *"feedforward cannot hold a pose"*.
- ⚠️ **The arm is no longer emergent, and that is the price.** ADR-0047's headline
  was that the sheave radius comes out of the routing on its own, with ADR-0042's
  +/-8.75 mm/rad coupling alongside it. Clamped, the map **is** the analytic one
  `TendonMap` has carried since M4; the simulation no longer derives it
  independently. ADR-0055 made that trade knowingly -- the emergent version is only
  correct inside a window -- but it should be said plainly rather than left for a
  reader to notice.
- ⚠️ **A silent failure closed on the way.** `single_leg_rig_elastic(clamped=True)`
  used to return a plant with **stiffness 0.0 on every tendon** while the caller
  believed it had asked for 175 kN/m: a `<fixed>` tendon's length is the commanded
  `sum r*q`, so a `stiffness` on it is a passive **joint** spring, not a stretching
  cable. It raises now, and points at where the G3 element belongs with a clamped
  cable -- the drivetrain, `spools=<series_k>` from [ADR-0051](#adr-0051).

### ⚠️ ADR-0002 and ADR-0008 want different robots

- ADR-0056 called this *"the simulation never implemented the variable-radius
  pulley"*. It is worse than an omission. Read the two decisions side by side:
  - [ADR-0002](#adr-0002): antagonistic pairs *"for joints whose **stiffness must
    vary**, with co-contraction bias `T_bias` exposed as a **first-class control
    input**"*, closing *"Stiffness becomes commandable."*
  - [ADR-0008](#adr-0008): *"one motor per antagonistic pair via the variable-radius
    pulley"*, and *"**Full articulation is retained** -- this is a change of
    transmission, not of DOF."*
- ⚠️ **That last sentence is true of the joint angles and false of the stiffness.**
  Co-contraction is not a position DOF, it is the **redundant coordinate**. Measured
  on the clamped paired leg, the allocation `G` is 3x6 with a **three-dimensional
  null space** -- one co-contraction direction per pair -- and the same joint torque
  is reachable at a 5 N floor and at a 40 N floor. One motor per pair leaves
  **none**: a variable radius lets co-contraction be *scheduled* against joint angle,
  baked into the pulley, not commanded at runtime. **ADR-0008 never mentions
  stiffness or co-contraction.**
- So the choice is not budget-versus-simulation. It is:
  - **Keep ADR-0002.** Co-contraction stays commandable; the leg motor count is 20,
    and the body is **5.36 kg** (5.89 with ADR-0050's ankle pair) against the 4-5 kg
    band NFR5 has been anchored to since [ADR-0010](#adr-0010).
  - **Keep ADR-0008.** The budget closes at 4.30 kg, and the project gives up what
    [ADR-0021](#adr-0021) priced standing on, what ADR-0002's AIC rule is built
    around, and what M43 measured as the thing that keeps the clipped allocator
    exact.
- **M52 does not decide it.** It is a requirements-level trade between mass and
  controllability, and both sides have accepted ADRs behind them.

### Consequences

- ✅ **`clamped=True` is available and exact**, and it is what the re-derivation
  should target. It is not yet the default, because the default carries M42-M51's
  published numbers and the re-derivation happens **once**.
- ⚠️ **The motor-count decision now blocks the re-derivation**, because it sets how
  many actuators the re-derived plant has. That is the next milestone and it is a
  decision, not a measurement.
- ⚠️ **NFR5 has no single answer until it is made**: 4.30, 5.36 or 5.89 kg.
- ✅ **ADR-0054's audit would pass on the clamped plant**, on both legs, at every
  angle -- so the routing programme that ran from ADR-0052 to ADR-0055 is finished,
  and what remains is bookkeeping plus one decision.

## ADR-0058: ADR-0008 wins -- one motor per pair, and it cures the tension saturation as a side effect

- **Status:** Accepted, and it **resolves the [ADR-0002](#adr-0002) /
  [ADR-0008](#adr-0008) conflict [ADR-0057](#adr-0057) named**, in ADR-0008's
  favour. `pulley=True` ships in `mjcf_tendon.py`. **Supersedes ADR-0002's
  commandable-stiffness clause. Unblocks [ADR-0056](#adr-0056)'s thermal item by
  removing its cause. Keeps NFR5 at 4.3041 kg.**
- **Context:** ADR-0057 showed the two decisions want different robots:
  co-contraction commandable at **5.36 kg and 66 % motor by mass**, or the budget
  closed at **4.30 kg and 58 %** with stiffness scheduled by pulley geometry. It
  declined to choose. This ADR chooses, and then measures what the choice does.

### Why ADR-0008

- ⚠️ **Mass is this project's hardest constraint and it is already marginal.**
  ADR-0008 exists because the design *did not close* on motor mass. Independent
  pairs put the robot at **5.36 kg / 66.4 %** motor, or **5.89 kg / 69.4 %** with
  ADR-0050's ankle pair -- and that variant lands on **31 motors, the exact count
  ADR-0008 was written to escape**. Both are outside the 4-5 kg band NFR5 has been
  anchored to since [ADR-0010](#adr-0010).
- ⚠️ **The downstream constraints are already at their limits.**
  [ADR-0044](#adr-0044) has NFR6 at 14-20 min; [ADR-0045](#adr-0045) has NFR18
  out of spec for continuous trotting in still air at any finish. Copper loss goes
  as mass squared.
- ✅ **And this project has never measured a need for commandable co-contraction.**
  ADR-0026's compliance requirement was met by **G3, a mechanical spring**
  ([ADR-0047](#adr-0047)), not by co-contraction. ADR-0049's *"co-contraction buys
  back the clipped allocator"* was **retracted by [ADR-0053](#adr-0053)** -- with
  correct routing, clipping holds to 0.0002 deg anyway. [ADR-0021](#adr-0021)'s
  standing cost is a *cost* of co-contraction, not a benefit of commanding it. The
  case rests on Kengoro's AIC peak-tension result and on the literature's tunable
  spine, neither tested here.
- ⚠️ **What is given up, plainly:** ADR-0002's *"stiffness becomes commandable"*.
  Co-contraction becomes whatever the pulley's radius profile schedules against
  joint angle. Kengoro's AIC is itself a schedule, so the peak-tension benefit may
  be recoverable in the profile -- that is a design task, not a loss, and it is not
  done here.

### ⚠️ The prerequisite nobody had noticed

- A pulley transmits the **difference** of its pair's two cable lengths. Anything
  **common** to both must be absorbed by cable stretch, which the pulley cannot
  relieve. M42 routed both cables of a pair over the **same side** of each
  via-pulley, so ADR-0042's `-v*q1` lands on both equally:

  | routing | differential (the motor) | common (stretch) |
  |---|---|---|
  | same side, as M42 built | `[0, r_knee]` | `[-v, 0]` |
  | **opposite sides** | `[-v, r_knee]` | **`[0, 0]`** |

- Same-side, the common mode is **11.44 mm** at the knee across the hind gait range
  and **20.60 mm** at the ankle -- **1716 N and 3090 N** of co-contraction swing
  against a **638 N** cable rating. The cables break. ⚠️ It is also *falsely
  decoupling*: the differential loses the hip term, so the pulley would read the
  knee as independent of the hip while the coupling appeared as tension.
- ✅ **Opposite sides leave the common mode at exactly zero** and keep ADR-0042's
  coupling in the differential, where the motor can act on it. **ADR-0008 is only
  buildable with the pair split across each via-pulley**, and nothing had said so.

### What it measures

- ✅ **Twelve leg motors, and the map is unchanged.** Three `<fixed>` tendons and
  three **bidirectional** motors per leg -- bidirectional because pull-only is a
  property of a *cable* and a pair covers both directions. On both legs the map is
  ADR-0042's exactly: hip **28.000**, knee **-8.750 / 25.000**, ankle
  **-8.750 / -8.750 / 14.000**. `params.trunk_mass` needs no change; the body stays
  **4.3041 kg**.
- ✅ **The unloaded leg holds to 0.00 deg at a peak cable force of 1.7 N**, against
  the independent-pair plant's 205 N standing figure.
> ⚠️ **EVERY FIGURE IN THIS SECTION WAS RE-DERIVED BY [ADR-0064](#adr-0064)
> (M59), and the finding got stronger.** These were measured on a contact model
> where the body sagged 3.1 mm onto the metatarsals before anything carried load.
> On a point foot the standing force **decays** rather than converging: 60.0 N at
> 1 s, 35.2 at 4 s, **33.6 N at 12 s**, with the trunk sagging **0.18 mm** rather
> than 3.1. ✅ No saturation, at **41 %** of the published figure and a twentieth
> of the sag. ⚠️ The claim that the force *"converges and holds"* is withdrawn --
> it falls.

- ✅ **And it cures ADR-0056's tension saturation, which was not expected.** That
  ADR measured the standing force ramping to the **222.9 N motor ceiling by 9 s** and
  pinning there. On the pulley transmission the same gate runs 12 s with the force
  **converged at 68.2 N**:

  | | independent pairs | pulley |
  |---|---|---|
  | peak force at 1 s | 125.5 N | 68.2 N |
  | at 9 s | **222.9 N** (ceiling) | **68.2 N** |
  | at 12 s | 222.9 N | **68.2 N** |
  | trunk sag | 0.2 mm | 3.1 mm |
  | tilt | 0.006 deg | 0.007 deg |

  **The wind-up was co-contraction growing with the uncontrolled joint drift.**
  Remove co-contraction as a state and there is nothing to wind up. 68.2 N is inside
  the motor's **81 N continuous** rating, so ADR-0056's thermal item is unblocked by
  the *transmission* rather than by the posture task.
- ⚠️ **The cost line beside it:** the trunk sags **3.1 mm against 0.2**, which is
  the preload the co-contraction floor used to supply.

### Consequences

- **NFR5 stays at 4.3041 kg**, and ADR-0056's 5.36/5.89 kg projections are withdrawn
  along with ADR-0050's 4.83.
- ⚠️ **`wbc.tendon_tension`'s `t_min` has nowhere to act on a pulley leg.** The
  allocation is a square solve: three joints, three motors, no null space. Every
  co-contraction-floor result from M42-M47 is inapplicable to the shipped
  transmission, which is part of the re-derivation now queued.
- ⚠️ **`mechanical/` owes two things**: the cable **clamped** at each sheave
  ([ADR-0055](#adr-0055)), and each pair **split across its via-pulleys**. Neither is
  drawn.
- ⚠️ **The variable-radius PROFILE is undesigned.** This ADR models a constant
  ratio, which schedules zero co-contraction. Recovering Kengoro's AIC benefit means
  designing `r_a(theta)` and `r_b(theta)`, and that is a mechanical task with a
  measurable target.

  > ✅ **CLOSED by [ADR-0061](#adr-0061) (M56): do not design it.** The sentence
  > above names the wrong mechanism -- AIC is a control rule measured against a
  > fixed high co-contraction, and a pulley that cannot co-contract is already at
  > that optimum. The mechanism that *does* apply, a gear ratio varying with joint
  > angle, was measured and yields **exactly zero** on the only pair that needs
  > help, because the gait holds the paw flat and that demand has no shape.
- ✅ **The re-derivation is now unblocked and has one target**: clamped capstans,
  pulley transmission, opposite-side vias, 12 leg motors, 4.3041 kg.
  **Done in M54 -- [ADR-0059](#adr-0059)**, which also found that the drivetrain,
  and therefore G3, could not be built behind a pulley at all.

## ADR-0059: The re-derivation -- the shipped transmission becomes the default, and eight tests had been quietly disarmed

- **Status:** Accepted. `clamped`, `pulley` and `ankle_pair` now default to the
  shipped configuration in `mjcf_tendon.py`. **Adds `wbc.pair_command`. Retracts
  every co-contraction-floor result from M42-M47. Closes the routing findings in
  [ADR-0053](#adr-0053) and [ADR-0054](#adr-0054) by construction. Gives
  [G3](#adr-0051) a home on the shipped robot, which it did not have.**
- **Context:** [ADR-0058](#adr-0058) chose the transmission; the simulation still
  handed back the **rejected** one whenever a test asked for a leg without saying
  otherwise. Eleven milestones of results had accumulated against that default.

### What was actually done

- ✅ **The default is the shipped machine.** `single_leg_rig()` with no keywords
  is now three bidirectional motors; `quadruped_rig()` is **twelve**. The legacy
  plant is still reachable and still measured -- 39 call sites moved behind seven
  named wrappers (`_legacy_*` for the wrapped per-cable build, `_clamped_*` for
  M52's half-step) so that **which machine a test measures is part of its text**.
  The pinning was behaviour-preserving: no measured number moved.

### ⚠️ The part that mattered: eight tests had stopped being able to fail

- This project writes tests that **assert a defect**, so they fail when the defect
  is fixed. Eight such tests were live. Pinning them to the legacy plant -- the
  obvious, apparently conservative move -- would have left all eight asserting
  defects on a machine nobody builds, where they would have passed **forever**.
- Each was rewritten to keep its legacy measurement *and* name where the shipped
  plant's guarantee is asserted. The defect stays recorded against the build that
  had it; the regression guard moves to the build that ships.

### ✅ One measurement retires four routing defects

- On the shipped leg the tendon Jacobian is the constant matrix `pair_rows` emits.
  Sampled at five poses spanning each joint's **entire ROM**, on **both** legs, the
  spread is **exactly zero** -- hip 28.000, knee -8.750 / 25.000, ankle
  -8.750 / -8.750 / 14.000.
- That single fact closes: ADR-0049's **ankle sign reversal** (the arm is +14.000
  everywhere), ADR-0053's **fore leg was never mirrored** (the fore map is
  *identical*, not mirrored), ADR-0053's **anchor migration** (a fix for a wrap that
  no longer exists), and ADR-0054's **"only the ankles were validated across the
  gait"** (every joint is now validated across its whole range).

### ⚠️ And a capability that did not exist: G3 had nowhere to live

- [ADR-0051](#adr-0051) put the series-elastic element **in the drivetrain**. The
  drivetrain named its spools per *cable* (`L_hip_flex`); the shipped plant's
  tendons are the three *pairs* (`L_hip`). It did not build -- it failed with
  MuJoCo's `unknown element 'L_hip_flex'`, which names the symptom, not the cause.
- So M46's exact statics, M47's derived cascade and the rotor servo all stood on a
  transmission ADR-0058 had already replaced, and **design goal G3 had no home on
  the robot being built**. This was not a number needing re-derivation; it was a
  hole.
- ✅ **Fixed: one spool per pair.** With a variable-radius pulley the spool *is*
  the pulley -- it takes up one cable while paying out the other, so what winds on
  it is their difference, which is exactly the `<fixed>` tendon. Half the bodies
  (12 vs 18), half the constraints (3 vs 6), and M46's two-pass `a0` still lands the
  winding equality on its reference pose to **4e-10 m**.

### ⚠️ The retraction: the pull-only allocator drops whole joints

- `wbc.tendon_tension` enforces `T >= t_min` by NNLS. On the shipped plant that
  constraint is **not physical** -- the pair's motor drives either way -- and the
  cost is not a rounding error. Given the exact gravity torque at the stance pose:

  | | needed | NNLS delivers | |
  |---|---|---|---|
  | hind ankle | 0.0114 N*m | **0** | the whole joint |
  | fore knee | 0.0492 N*m | 0.0076 | **15 %** |

  It clamps to zero a command the motor could deliver, and the shortfall appears
  only in a residual nobody was checking.
- ✅ **`wbc.pair_command`** is the square, signed solve: three joints, three
  motors, unique, **zero residual**. `tendon_tension` stays correct for the legacy
  per-cable plant and now says so.
- ⚠️ **`t_min` has nowhere left to act.** The co-contraction floor was a coordinate
  of the *redundant* plant and ADR-0058 spent it. Every M42-M47 result that rested
  on choosing a floor describes a machine this project is no longer building.

### ⚠️ What this milestone did NOT do

- **The cascade and the rotor servo were not moved.** Now that the drivetrain
  exists behind the pulley they *can* be, but the gains must be re-derived rather
  than re-pointed: one spool per pair changes the reflected inertia and therefore
  the bandwidth separation ADR-0052 derived. It is a measurement, and this project
  has learned to do a re-derivation once.

  > ⚠️ **CORRECTED by [ADR-0060](#adr-0060) (M55): the conclusion above is wrong.**
  > The inertia claim holds -- the lowest drivetrain mode does halve, 54.9 -> 27.4
  > Hz -- but M47 put its outer loop far below **both** plants' modes, so the gains
  > transfer **unchanged** and hold to 0.00 deg. What actually had to change was the
  > **allocator**, which this ADR had already found and did not connect to the
  > cascade. The re-derivation confirms the gains rather than replacing them.
- **M52's six clamped tests still sit on the half-step** -- clamped capstans with a
  motor per cable, a configuration that does not ship. They are kept because the
  three findings they carry are the evidence ADR-0058 was decided on, and each is a
  statement about a plant with independent motors.
- **The variable-radius PROFILE is still undesigned**, so the shipped robot still
  schedules zero co-contraction.

## ADR-0060: The cascade transfers unchanged -- what the pulley cost was headroom, not gains

- **Status:** Accepted. No gain changes ship. **Corrects [ADR-0059](#adr-0059)'s
  prediction that the cascade gains would need re-deriving. Retires
  [ADR-0049](#adr-0049)'s ankle reversal bound and replaces it with a joint limit.
  ⚠️ Withdraws both numbers from M47's ankle tracking test, which was saturated.**
- **Context:** ADR-0059 built the drivetrain behind the pulley and then declined to
  move M46-M47's control results onto it, predicting the gains would have to be
  re-derived because one spool per pair changes the reflected inertia. This ADR
  does the measurement.

### ✅ The gains transfer unchanged

- M47's `kp = 50, kd = 1.0` holds the stance pose to **0.00 deg** on the pulley
  drivetrain at a peak commanded force of **3.2 N**, with the motor never asked
  past its rating.
- The inner loop needed no thought: `kp = I*wn^2` is a property of the **rotor**,
  and every motor still has exactly one rotor. `SERVO_KP` is 180 N*m/rad on both.
- ⚠️ **So ADR-0059's prediction was wrong**, and it is worth saying why it was
  plausible: the inertia argument was correct, and the mode really did move. It
  just moved from far above the outer loop to less far above it.

### ⚠️ What the pulley did cost: headroom

- Linearising both plants about the stance pose:

  | | lowest mode | usable outer `kp` |
  |---|---|---|
  | legacy, one spool per cable | 54.9 Hz | holds to **600** |
  | **shipped, one spool per pair** | **27.4 Hz** | holds to **200** |

- The lowest drivetrain mode **halves** and the outer loop's usable range falls with
  it: M47's chosen gain sat 12x below the edge, and now sits **4x** below it.
  ✅ Still headroom, not a wall. ⚠️ But anything wanting a stiffer joint loop --
  landing, disturbance rejection -- has a third of the room it had, and **this had
  not been measured** when ADR-0058 was decided on mass.

### ✅ The allocator was the real change

- Same plant, same gains, same reference; only the allocator differs:

  | allocator | hold error | peak force |
  |---|---|---|
  | `pair_command`, signed | **0.00 / 0.00 / 0.00 deg** | 3.2 N |
  | `tendon_tension`, pull-only | **-70.8 / -47.2 / -127.1 deg** | 32.5 N |

- ADR-0059 measured the pull-only allocator dropping whole joints and called it a
  residual. Dynamically it is a **fall**. The two findings are the same finding.

### ⚠️ M47's ankle test was measuring saturation

- M47 reported *"the cascade tracks the hind ankle but the reversal still bounds
  it"* from a **step** command. Logging the force before the clamp, that step
  demands **3700-4300 N** against a 222.9 N motor and saturates **98-100 %** of
  every timestep, on both plants. It was measuring where a saturated bang-bang
  controller comes to rest. **Both of its numbers are withdrawn.**
- ✅ **Ramped instead** -- which is what a gait commands -- the shipped cascade
  tracks the ankle exactly:

  | commanded | final error | peak force | saturation |
  |---|---|---|---|
  | +20 deg | **0.00** | 3.1 N | none |
  | +40 deg | **0.00** | 3.1 N | none |
  | +52 deg | **0.00** | 3.1 N | none |
  | +55 deg | -2.06 | 130 N | none |

  +55 misses by **exactly** its overshoot of the 150 deg end stop: the stance ankle
  sits at 97.06 deg, so the headroom is **52.94 deg**. ✅ **The bound is the joint
  limit now, not the moment arm** -- ADR-0049's reversal is gone and what replaces
  it is a number out of `params`.
- ✅ The lag is first-order with no saturation: the worst error during the ramp
  falls **0.81 -> 0.41 -> 0.20 -> 0.10 deg** as the ramp is stretched 0.5 -> 1 -> 2
  -> 4 s. ⚠️ The legacy plant cannot do it at all -- the same 2 s ramp to +20 deg
  leaves it **36.3 deg** out, saturated 44 % of the time.

### Consequences

- ✅ **M42-M47 is now entirely re-derived onto the shipped transmission.** Nothing
  in the control story still rests on a plant that is not being built.
- ⚠️ **A step command is not a test.** Two published numbers survived four
  milestones because nothing logged the pre-clamp force. Any future tracking claim
  has to report saturation alongside error.
- ⚠️ Still open and untouched by this ADR: the **variable-radius profile**, the two
  routing features `mechanical/` owes, and the null-space posture task -- which the
  shipped transmission removed the symptom of, not the need for.

## ADR-0061: The variable-radius profile is priced and declined -- the binding joint has no shape to exploit

- **Status:** Accepted as a **decision not to build**. No geometry ships. **Closes
  the profile item [ADR-0058](#adr-0058) left open. Retires
  [ADR-0002](#adr-0002)'s AIC rationale as inapplicable rather than unmet.
  ⚠️ Names one real thermal overrun and hands it to `mechanical/`.**
- **Context:** ADR-0058 chose one motor per pair and left the radius profile
  undesigned, speculating that Kengoro's peak-tension benefit *"may be recoverable
  in the radius profile"*. This milestone measures what such a profile could buy
  before designing one.

### ⚠️ Two mechanisms were being conflated

- **Kengoro's AIC is a control rule** -- hold the antagonist at `T_bias` while the
  agonist works -- and its 43 -> 28 kgf is measured **against a fixed high
  co-contraction**. The shipped pulley cannot co-contract at all, so it is already
  at that optimum. There is nothing for an AIC-like schedule to remove.
- **A variable radius is a transmission** -- a gear ratio that varies with joint
  angle, trading speed for force where the demand peaks. That mechanism is real and
  available. It is not the one ADR-0002 cited, and it is the one that was measured.

### ⚠️ The load split had to be settled first, and it points at the other leg

- A trot puts one fore and one hind foot down, each carrying its girdle's share.
  Measured on the shipped quadruped at the stance pose: **30.2 % fore, 69.8 %
  hind** -- 12.75 N on the fore foot, 29.47 N on the hind.
- ✅ `params.py` review finding **F2 had already said so**: the original budget
  *tuned girdle masses to hit a 60/40 front-heavy split*, and F2 retracted it,
  because the split is an **output** of where the hardware sits and ADR-0005 puts
  more motors on the pelvis than the shoulder.
- ⚠️ Assuming 50/50 makes the survey name the **fore knee**; assuming the
  discredited 60/40 fore-bias makes it name the fore knee and ankle. Both point at
  the wrong leg. The measured split names the **hind ankle**.

### What the trot actually demands

Quasi-static motor force over one cycle at the measured split:

| | peak | RMS | RMS / 81.1 N | peak / 222.9 N |
|---|---|---|---|---|
| hind hip | 106.1 | 43.0 | 0.53 | 0.48 |
| hind knee | 32.6 | 15.5 | 0.19 | 0.15 |
| **hind ankle** | 136.3 | **96.4** | **1.19** | 0.61 |
| fore hip | 74.4 | 35.4 | 0.44 | 0.33 |
| fore knee | 89.0 | 49.9 | 0.62 | 0.40 |
| fore ankle | 67.1 | 47.5 | 0.59 | 0.30 |

- ✅ **Structurally there is no case at all**: the worst peak is **61 %** of the
  motor's peak rating.
- ⚠️ **Thermally there is exactly one**: the hind ankle at **1.19x** continuous.
  RMS, not peak, is what heats a motor.

### ⚠️ And the profile's yield on that pair is exactly zero

- Through the whole stance phase the gait holds `q1 + q2 + q3` at **-55.0000 deg**,
  span **8.5e-14**. That is the paw's *absolute* orientation, and holding it fixed
  is what keeps the foot flat while the body passes over it.
- So the ankle's demand is a **constant**: 136.338 N at every sample, standard
  deviation **4e-12 N**, peak/mean **1.0000**. A profile can only exploit a demand
  that varies with angle. **There is no shape to remove.**
- The pairs that *are* peaked have margin already -- the hind hip is the most
  peaked at 1.99, and it runs at 0.53x its rating. ⚠️ And RMS is dominated by the
  **mean**, which a profile does not change: flattening the most peaked pair in the
  survey moves its RMS by under 2 %.

### Consequences

- ✅ **The profile is not designed, and the reason is recorded rather than
  deferred.** This item has been open since ADR-0002 and is now closed by
  measurement.
- ⚠️ **The hind ankle's 1.19x is handed to `mechanical/`.** The fix is a constant:
  **14.0 -> 16.6 mm**. It is left unspent here because the arms are shared between
  legs and the hind leg's other pairs have 5x margin, so the right change is a
  **per-leg** arm -- a decision that needs a sheave that fits, not a simulation.
- ⚠️ **This survey is quasi-static**: gravity plus a vertical foot load, no
  inertial or horizontal terms. It bounds the *shape* of the demand, which is what
  the profile question needs. It is not a duty-cycle model, and NFR18 already holds
  that continuous trotting is out of spec in still air.
- ✅ ADR-0002's antagonistic pairs survive; only its AIC rationale is retired.

## ADR-0062: The articulated spine ships -- 18 of NFR2c's 19 DOF, and it stands

- **Status:** Accepted. `quadruped_rig(spine=True)` emits ADR-0006's vertebral
  chain with **both** its sagittal and lateral DOF; `wbc.chain_reaction` ships with
  it. **Closes the deferral [ADR-0043](#adr-0043) took when it made the trunk a
  rigid box. ⚠️ Puts a banner on NFR2c: 18 built, the tail still owed.
  ⚠️ Re-opens [ADR-0061](#adr-0061)'s load split for re-checking.**
- **Context:** NFR2c has claimed **19 actuated DOF** since ADR-0009. Nothing in the
  repository had more than 15, and the shipped tendon plant had 12.

### ⚠️ What the census found

| model | actuated | leg | spine | tail |
|---|---|---|---|---|
| `mjcf`, rigid trunk | 12 | 12 | 0 | 0 |
| `mjcf`, `spine_dof=True` | 15 | 12 | **3, lateral only** | 0 |
| `mjcf_tendon`, the shipped plant | 12 | 12 | 0 | 0 |

- ⚠️ **The sagittal axis was in no MuJoCo model at all.** It is the axis ADR-0006
  is about -- dorsoventral arch, whole-body curvature -- and the only spine axis
  that does work against gravity. What existed was ADR-0009's lateral sway, and
  only in the rigid, position-servo model. **The requirement was being checked
  against a plant that could not meet it.**
- ⚠️ **The tail is nowhere.** It has a motor in the mass budget (the 7-motor
  spine+tail bank) and no `TailParams`, no joint, no body. It cannot be modelled
  without inventing its geometry.

### ✅ What was built

- Three segments, each with a sagittal and a lateral joint: **six pairs, six
  bidirectional motors**, ADR-0058's transmission applied unchanged to the spine.
  Mono-articular, moment arms straight from `params` (30 mm sagittal, 20 mm
  lateral). **18 actuated DOF.**
- ✅ **Mass cost is exactly zero**: 4.3081 kg either way. The girdles and three
  segments sum to precisely the `trunk_mass` the box carried in one lump.
- ⚠️ **It settles two numbers nobody had reconciled.** The rigid trunk put the
  girdles `2 * GIRDLE_X = 210 mm` apart; ADR-0006's segment lengths sum to
  **195 mm**. The chain is the sourced number, so the wheelbase shortens by 15 mm
  and the fore load share moves **30.2 % -> 33.0 %**.
- ⚠️ **`TendonMap.from_spine`'s `pretension` has nowhere to act** on a pulley
  spine, for the same reason `wbc.tendon_tension`'s `t_min` does not on a pulley
  leg ([ADR-0059](#adr-0059)).

### ✅ It stands -- and one term is the difference between standing and falling

| | sag | tilt | leg peak | spine peak |
|---|---|---|---|---|
| gravity compensation only | 89.7 mm | **77.0 deg** | 222.9 N | 222.9 N |
| **+ `wbc.chain_reaction`** | **2.82 mm** | **0.006 deg** | 65.8 N | **39.3 N** |
| rigid box, for reference | 3.08 mm | 0.006 deg | 68.2 N | -- |

- The missing term is what a foot force does to **every joint between that foot and
  the root**. On a rigid box there were none; on a chain it is the whole spine.
  ✅ It is [ADR-0044](#adr-0044)'s omission one level up -- the legs needed the
  stance term in `actuator_torque` for exactly the same reason. Derived, not tuned.
- ✅ **The articulated body is slightly BETTER than the box**: 2.82 mm of sag
  against 3.08, 65.8 N of leg force against 68.2, while holding the spine to
  **0.001 deg** at 39.3 N -- inside the 81.1 N continuous rating.

### ✅ The difficulty is the six DOF, not the geometry

- Built with the joints removed -- same segment geometry, same distributed mass,
  zero spine DOF -- the welded chain stands **better** than the box it replaces:
  **2.95 mm** of sag and **65.4 N** of leg force. The shorter wheelbase and the
  redistributed mass are free.
- ⚠️ What is not free is the six degrees of freedom, and what they need is a
  **control term**, not a stiffer body. That is precisely what ADR-0043 suspected
  when it made the trunk rigid so the gate had *"one thing to get wrong"*.

### Consequences

- ⚠️ **NFR2c is 18 of 19.** The tail is owed and cannot be built from what exists.
- ⚠️ **[ADR-0061](#adr-0061)'s verdict wants re-checking on this body.** It named
  the hind ankle as the one pair over its thermal rating using the **30.2 %** fore
  share; on the chain that is **33.0 %**. The finding may well survive -- the
  profile's yield was *zero*, not marginal -- but the tension survey itself was run
  on the box.
- ⚠️ **A moving spine is not tested.** This gate is quasi-static standing. Every
  gait result in the project still assumes a rigid trunk, and `mjcf.py` already
  warns that a swaying spine over planted feet needs foot slip or body roll.
- ✅ A stale `params.py` comment was corrected on the way: the girdle masses are
  computed from the surveyed **132 g** motor, while the comment beside them still
  said the superseded 77 g.

## ADR-0063: The sway does not come out -- ADR-0009 and ADR-0017 want different robots

- **Status:** Accepted as a **measurement and a named conflict. It decides
  nothing.** No geometry or gain ships. **Puts a banner on
  [ADR-0009](#adr-0009). ⚠️ Suspends its "+10.1 mm polygon margin" as an
  actuated result.**
- **Context:** ADR-0009 bought three lateral spine motors so the CoM could sway
  over the support triangle, and M5 designed the law with care -- a raised-cosine
  traverse confined to the four-foot windows, after finding a sinusoid *worse than
  no sway at all*, and after M6 caught a linear ramp's acceleration impulse. All of
  that is **analytic geometry and a quasi-static margin**. [ADR-0062](#adr-0062)
  built the actuators. Nothing had ever run the law on them.

### ⚠️ What the actuated plant does

On the shipped walk (period 5 s, duty 0.90, +-11 deg/segment), at the only gain
that keeps the six spine motors inside their rating:

| | analytic | measured |
|---|---|---|
| CoM sway | **66.7 mm** peak-to-peak | **4.1 mm** |
| foot slip | -- (the model has no ground) | **7.3-15.8 mm, all four feet** |
| spine force | -- | **76.8 N** of 81.1 N continuous |

- ⚠️ **The slip is larger than the sway.** The body gets about **6 %** of the
  designed CoM shift and pays for it by sliding every paw further than the CoM
  moves.
- ⚠️ **The spine is already at 95 % of its continuous rating** delivering that
  6 %. The lateral moment arm is **20 mm** against the sagittal 30, and `params.py`
  already warned that a short lateral arm *"directly amplifies cable tension"*.

### ⚠️ More gain does not help; it skates

> ⚠️ **RE-DERIVED BY [ADR-0064](#adr-0064) (M59): on a point foot it does not
> skate, it DIVERGES.** The table below was measured while the fore legs rested on
> their metatarsals, where more gain looked like it bought tracking in return for
> scrub. It does not: at `kp = 30` the tracking error is **65.7 deg**, five times
> worse than at `kp = 8`, and the raw demand reaches **31 750 N -- 142x the
> motor's peak**. `kp = 8` is the only gain tried that stays inside the rating,
> and there the sway is **2.6 mm of the designed 66.7**, less than this ADR
> reported. The conclusion is unchanged and the reason is different: it is not
> that friction eats the sway, it is that the loop does not close above that gain.

Logging the force **before** the clamp, as [ADR-0060](#adr-0060) requires:

| `kp` | track err | CoM "sway" | spine force | foot slip |
|---|---|---|---|---|
| 8 | 12.95 deg | 4.1 mm | 76.8 N | 25.7 mm |
| 30 | 9.07 deg | 12.1 mm | **222.9 N, saturated** | 122 mm |
| 100 | 5.69 deg | 98.9 mm | saturated | **484 mm** |
| 300 | 3.80 deg | 22.8 mm | saturated | 280 mm |

⚠️ The 98.9 mm at `kp = 100` is **not sway** -- it is the robot sliding across the
floor, which is what the 484 mm of scrub beside it says. Every gain that improves
tracking pins all six motors and buys the improvement in scrub.

### ✅ The mechanism, and it was already written down

`mjcf.py` carries this warning about a moving spine: the legs are planar because
[ADR-0017](#adr-0017) rejected abduction, so *"a sway over planted feet needs foot
slip or body roll, and locking roll leaves neither"*. It was written of the rigid,
planar-root model. **It holds on the actuated, free-root body as well.**

- Held at +-11 deg/segment with the root pinned, the CoM moves **31.7 mm** while
  the fore feet are carried **82.7 and 98.1 mm** -- the paws must travel about
  **three times further than the CoM does**.
- ⚠️ **That fixed-root figure overstates the fore share.** With a free root the
  body counter-rotates and the hind feet move too: measured **2.1x** fore-to-hind
  at the holding gain, not 3x with the hind at zero. What does not change is that
  **all four paws slide**.

### The conflict, stated plainly

- **ADR-0009** buys three lateral spine motors so the CoM can sway.
- **ADR-0017** rejects leg abduction, so no leg joint can move a foot sideways.

A lateral bend swings the front girdle and the fore legs hang off it. The sway is
therefore only realisable if the paws can translate laterally, and ADR-0017 removed
the only joint that could deliver it without scrubbing. This is a
requirements-level trade with an accepted ADR on each side, of exactly the kind
[ADR-0057](#adr-0057) named for ADR-0002 vs ADR-0008. **This ADR does not decide
it.**

### Consequences

- ⚠️ **ADR-0009's "+10.1 mm polygon margin" is suspended as an actuated result.**
  It is a quasi-static geometric margin computed from a sway the plant does not
  produce. It may still be reachable -- with abduction, with a longer lateral arm,
  or by accepting scrub -- but it is not currently demonstrated.
- ⚠️ **Scope: this is sway in place, with the legs planted.** It isolates the
  question ADR-0009 raised. A walking gait adds swing legs and contact transitions
  and is not tested here.
- Options a future milestone would have to price: **accept paw scrub** (and size
  it against ADR-0009's friction limit), **add abduction** (re-opening ADR-0017,
  +4 motors against a mass budget ADR-0058 only just closed), **lengthen the
  lateral moment arm** (mechanical, and it trades against tension), or **drop
  static stability** for dynamic walking -- which ADR-0009 listed as option E and
  rejected because the dynamics milestone did not exist. It exists now.

## ADR-0064: The foot was not a foot -- a millimetre of contact geometry moved five published results

- **Status:** Accepted. Bones no longer collide; the pad is the only leg geom that
  touches the ground. **Corrects [ADR-0049](#adr-0049)'s standing tension survey,
  [ADR-0058](#adr-0058)'s standing force and sag, and [ADR-0063](#adr-0063)'s
  gain sweep. ✅ Answers the low-friction-foot question that prompted it.**
- **Context:** A design suggestion -- *put something low-friction on the foot so
  the paw can follow the sway* -- needed a friction study. Setting the friction
  turned out to change nothing, and chasing why exposed the contact model.

### ⚠️ The defect

- The distal limb carries three geoms within two millimetres of each other: the
  **pad** sphere at the toe (the contact the model intends), the **paw capsule**
  2 mm above it, and the fore **metatarsal** 1 mm above it.
- Standing sagged the trunk **3.06 mm**, which is more than either clearance, so
  all three took load. Measured during a controlled stand: pad 200 samples in 200,
  hind paw capsule 196, **fore metatarsal 91**.
- ⚠️ **The consequence: the fore legs' effective contact sat 8.3 mm BEHIND the
  `_foot` site** every controller and every support-polygon calculation uses.
  On a 210 mm wheelbase, against the +-16 to 30 mm margins ADR-0009 argues over,
  that is half the quantity in dispute -- and it was silent.

### ✅ The fix, and what it bought

Bones do not collide. This is not a convenience: **the model now matches its own
stated abstraction**, a point foot at `_foot`.

| | before | after |
|---|---|---|
| contact-vs-site offset, fore | 8.3 mm | **0.0 mm** |
| geoms touching | pad + paw capsule + metatarsal | **pad only** |
| standing sag | +3.06 mm | **-0.57 mm** |

### ⚠️ Five published results moved

- **[ADR-0049](#adr-0049)'s standing survey inverts.** The binding tendon is the
  **fore knee flexor at 100.5 N (1.24x continuous)**; the **hind hip extensor** the
  headline named is at **63.4 N, inside its rating**. The old survey's *"the fore
  legs are comfortable"* was the fore legs resting on their metatarsals.
- **[ADR-0058](#adr-0058)'s standing force.** Not 68.2 N converged, but **60.0 N
  decaying to 33.6 N**, with **0.18 mm** of sag rather than 3.1. The conclusion is
  stronger; the numbers were all wrong.
- **[ADR-0063](#adr-0063)'s gain sweep.** Not skating but **divergence**: 65.7 deg
  of tracking error at `kp = 30` and a raw demand of **31 750 N**.
- Standing sag itself, and the contact offset.

⚠️ Only **3 of 484** tests failed on the change. That is because most of the suite
is quasi-static or kinematic, not because the defect was small.

### ✅ And the question that started it, answered

Sway in place through a crossover, at the only spine gain that stays inside the
motor rating:

| foot | CoM sway | max foot slip | spine raw demand |
|---|---|---|---|
| as built (mu 0.8) | 2.60 mm | 17.0 mm | 76.8 N |
| isotropic 0.10 | 9.17 mm | **115.9 mm** | 76.8 N |
| **anisotropic 0.03 lateral / 1.0 fore-aft** | **8.26 mm** | **13.4 mm** | 136.5 N |

- ✅ **The anisotropic foot works in the direction intended**: it roughly
  **triples** the sway while *reducing* foot slip, which is exactly the trade a
  directional pad is meant to buy.
- ⚠️ **Isotropic does not.** It reaches a similar sway only by letting the whole
  robot skate **116 mm**. Fore-aft grip is what separates the two, and it is also
  what propulsion needs.
- ⚠️ **But it does not close the gap.** 8.26 mm is **12 %** of ADR-0009's designed
  66.7 mm, and it costs **136.5 N** of spine force -- **1.68x** the continuous
  rating. ~~**The binding constraint moves from friction to the spine's own torque
  capacity**, through the 20 mm lateral moment arm `params.py` already warns
  amplifies tension.~~

  > ⚠️ **CORRECTED by [ADR-0065](#adr-0065) (M60): torque capacity was never the
  > constraint.** Nothing was clipped -- the demand was delivered in full. Triple
  > the lateral moment arm and the sway does not move **at all** (2.60 and 8.26 mm,
  > identical to three significant figures), because the controller commands a
  > *torque* and the arm only sets what that torque costs. What limits the sway is
  > the **control law**. ⚠️ And the 136.5 N is on the **sagittal** pairs, not the
  > lateral ones, so the lateral arm does not pay for it either.
- So the anisotropic foot is **necessary but not sufficient**. Whether it plus a
  longer lateral moment arm reaches the designed sway is not measured.
- ⚠️ **Confidence:** one 0.9 s window per configuration, and these sweeps have been
  jumpy. The direction is clear; treat the magnitudes as provisional.

### Consequences

- ⚠️ **Anything measured against foot contact before M59 is suspect** -- load
  splits, centre of pressure, support polygons, per-tendon standing loads. The
  three findings above are the ones the suite caught.
- ⚠️ [ADR-0061](#adr-0061)'s tension survey is quasi-static and did not move, but
  it uses a **load split** computed from CoM geometry; that is unaffected, while
  anything using measured contact forces would not be.
- ✅ A regression guard ships with the fix: only the pads may collide, and a
  controlled stand must put every contact within 1 mm of a `_foot` site.

## ADR-0065: The lateral arm buys cost, not sway -- and the pad's bill goes to the sagittal spine

- **Status:** Accepted as a **measurement**. No geometry ships. **Corrects
  [ADR-0064](#adr-0064)'s attribution of the constraint.**
- **Context:** ADR-0064 measured a directional pad tripling the sway and then
  concluded the binding constraint had *"moved to the spine's own torque
  capacity"* through the 20 mm lateral moment arm. The obvious follow-up is to
  lengthen that arm. This ADR does it.

### ⚠️ The arm does not change the sway. At all.

| foot | lateral arm | CoM sway | max slip |
|---|---|---|---|
| as built | 20 mm | 2.60 mm | 17.0 mm |
| as built | 60 mm | **2.60 mm** | 17.0 mm |
| anisotropic 0.03 | 20 mm | 8.26 mm | 13.4 mm |
| anisotropic 0.03 | 60 mm | **8.26 mm** | 13.4 mm |

- Identical to three significant figures across a **3x** change in the arm.
- ✅ **The reason is elementary once stated.** The controller commands a
  **torque**, `kp*e`; the arm only sets what that torque costs in cable force,
  `f = tau/r`. Same gain, same error, same torque, same motion. The arm is a price
  list, not a capability.

### ⚠️ So ADR-0064 named the wrong constraint

- **Torque capacity was never binding.** Nothing was ever clipped -- 76.8 N and
  136.5 N are both inside the 222.9 N peak, and the commanded value was delivered
  in full every step.
- **What limits the sway is the control law.** `kp = 8` asks for what it asks for,
  and [ADR-0063](#adr-0063) (as re-derived by ADR-0064) shows every higher gain
  **diverges**: 65.7 deg of tracking error at `kp = 30`, a raw demand of 31 750 N.
- ⚠️ Neither the foot nor the arm moves that wall. **A different control
  structure would be required, and none is proposed here.**

  > ⚠️ **[ADR-0066](#adr-0066) (M61) tried the obvious one and it does not
  > exist.** Retuning the attitude term to give the spine room fails, because that
  > term is doing **two** jobs: it is what converts a spine bend into CoM
  > *translation*, and it is what keeps the robot upright. Lower it and the sway
  > collapses **and** the robot falls. There is one working point, it is the
  > shipped one, and it delivers **4 %**.

### ✅ What the arm does buy is real, and it is thermal

- The **lateral** demand falls in exact proportion: **76.8 N at 20 mm, 25.6 N at
  60** -- precisely 20/60, from **95 %** of the continuous rating to **32 %**.
- That is worth having. It is simply not the thing ADR-0009 wanted.

### ⚠️ And the pad's bill goes somewhere the arm cannot pay it

Peak pre-clamp demand per spine pair; `p` sagittal (30 mm arm, unchanged), `y`
lateral:

| foot | arm | p1 | y1 | p2 | y2 | p3 | y3 |
|---|---|---|---|---|---|---|---|
| as built | 20 | 36.5 | **76.8** | 17.6 | **76.8** | 22.8 | **76.8** |
| as built | 60 | 36.5 | **25.6** | 17.6 | **25.6** | 22.8 | **25.6** |
| anisotropic | 20 | **130.6** | 76.8 | **136.5** | 76.8 | **127.8** | 76.8 |

- On a plain foot the **lateral** pairs bind, and lengthening their arm fixes it
  exactly.
- ⚠️ **With the directional pad the binding pairs are SAGITTAL**, at ~136 N and
  **1.68x** continuous, up from ~36 N. Letting the feet slide laterally lets the
  body move more, and holding it up is the sagittal spine's job: **the pad buys
  lateral motion and charges it to the sagittal pairs.**
- So the intuitive follow-up -- lengthen the lateral arm to pay for the pad --
  reduces a demand that is no longer the binding one.

### Consequences

- ✅ **The directional pad remains the one intervention that moved the sway**:
  3x, with *less* foot slip. That result stands.
- ⚠️ **The sway remains at 12 % of ADR-0009's design**, and the remaining gap is a
  **control** problem, not a mechanical one. Anything proposed for it should be
  measured against `kp = 8`, which is the only gain shown to be stable.
- ⚠️ If a longer arm is pursued for thermal reasons, the one to lengthen with a
  pad fitted is the **sagittal** 30 mm, not the lateral 20 mm. Both cost cable
  travel, spool capacity and a bigger vertebral post; none of that is priced here.
- ⚠️ **Confidence:** 0.9 s windows, one per configuration. The insensitivity to
  the arm is exact and structural, so that conclusion is firm; the magnitudes
  carry ADR-0064's provisional label.

## ADR-0066: There is no frontier -- one gain pair stands, and it buys 4 % of the sway

- **Status:** Accepted as a **measurement**. Nothing ships. **Closes the retuning
  option [ADR-0065](#adr-0065) left open. ⚠️ Supplies the number
  [ADR-0009](#adr-0009) vs [ADR-0017](#adr-0017) has to be decided on.**
- **Context:** ADR-0063 found the sway does not come out; ADR-0064 re-derived it on
  a corrected foot; ADR-0065 showed neither the pad nor the moment arm moves it and
  named the **control law**. The obvious next move is to retune. The WBC's attitude
  term regulates the **root** body, and a lateral spine bend rotates the front
  girdle relative to it, so the two fight. Lower the attitude gain, give the spine
  room. This measures that.

### ⚠️ It needed a column nobody was reading

Across attitude gain x spine gain, **with the trunk tilt reported**:

| attitude `kp` | spine `kp` | CoM sway | spine raw | tilt | |
|---|---|---|---|---|---|
| **40** | **8** | **2.60 mm** | **76.8 N** | **2.03 deg** | ✅ **stands** |
| 40 | 30 | 101.74 mm | 31 750 N | **179.91 deg** | upside down |
| 40 | 60 | 79.66 mm | 576 N | 24.22 deg | falling |
| 20 | 30 | 15.08 mm | 288 N | 14.96 deg | falling |
| 10 | 30 | 8.23 mm | 288 N | 14.68 deg | falling |
| 10 | 8 | 1.88 mm | 76.8 N | 6.26 deg | falling |
| 5 | 8 | 1.51 mm | 76.8 N | 9.62 deg | falling |
| 0 | 8 | 0.31 mm | 77.8 N | 30.23 deg | falling |

- ⚠️ **Every apparent improvement is the robot on its way to the floor.** The
  8.23 mm at attitude 10 reads like the trade working; the tilt says 14.68 deg.
  This ADR's first pass reported that row as *"stable, and tracking seven times
  better"* -- it was neither. It was falling, and the spine tracked its reference
  freely **because** the body was tumbling.

### ✅ The attitude term is doing two jobs at once

- It is what **converts a spine bend into CoM translation**. At attitude 0 the
  trunk simply counter-rotates: the sway collapses to **0.31 mm** even though the
  spine tracks its reference **better than anywhere else in the sweep** (2.82 deg
  against the shipped point's 12.94).
- It is also what **keeps the robot upright**: at attitude 0 the tilt is **30 deg**.
- ⚠️ So it cannot be lowered to make room for the spine loop. The room and the
  standing are the same quantity.

### Consequences

- **There is one working point**: attitude 40/4, spine `kp = 8`. It delivers
  **2.60 mm** of CoM sway -- **4 %** of ADR-0009's designed 66.7 mm -- at 76.8 N,
  inside the continuous rating, at 2.03 deg of tilt.
- ⚠️ **Even that point is disturbed.** Commanding the sway takes the tilt from
  ADR-0062's undisturbed **0.006 deg** to **2.03**, some 300x.
- ⚠️ **This is the number ADR-0009 vs ADR-0017 has to be decided on.** ADR-0009
  bought three lateral spine motors to sway the CoM ~42.8 mm; on the shipped plant,
  with the shipped controller, at the only configuration that stands, they deliver
  **2.60 mm**. Whether that is worth three motors is a requirements decision, and
  it is not made here.
- ⚠️ **What is NOT ruled out**: a controller that does not regulate root attitude
  against the spine -- one that treats the bend as commanded rather than as
  disturbance, or regulates a whole-body attitude instead of the root's. That is a
  design task with a measurable target (beat 2.60 mm while standing), and nothing
  in this project proposes one yet.
- ⚠️ **Confidence:** 0.9 s windows. A tilt of 6 deg inside 0.9 s is a fall in
  progress rather than a settled state, but the separation from the working point's
  2.03 deg is wide and the direction is unambiguous.

## ADR-0067: Static stability is already gone -- the achieved sway recovers 3.9 % of the margin

- **Status:** Accepted as a **measurement on ADR-0009's own arithmetic**. Nothing
  ships. **Puts a premise banner on [ADR-0009](#adr-0009). ⚠️ Makes its option E
  the situation rather than a choice.**
- **Context:** ADR-0009 bought three lateral spine motors because the walk is not
  statically stable without body sway. [ADR-0063](#adr-0063) through
  [ADR-0066](#adr-0066) measured what the actuated plant delivers: **2.60 mm**
  peak-to-peak at the only gain pair that stands. This puts that number back into
  the calculation the ADR was decided on.

### The margin, same gait, same function

| sway amplitude | worst margin | cycle OUTSIDE the polygon |
|---|---|---|
| designed (11 deg/segment) | **+6.33 mm** | **0.0 %** |
| none at all | -22.59 mm | **19.8 %** |
| **achieved, +-1.30 mm** | **-21.47 mm** | **19.8 %** |

- ⚠️ **The achieved sway is indistinguishable from no sway.** Same fraction of the
  cycle outside, the same worst phase, and **3.9 %** of the margin recovered.
- ⚠️ **The gap is a factor of twenty, not a near miss.** A merely **zero** worst
  margin needs **25.90 mm** of sway; the plant delivers **1.30**.
- One reconciliation: ADR-0009 reported **+10.1 mm** for the designed case and this
  measures **+6.33**. Its own follow-up records the margin peaking at
  **12.5 deg/segment** while the shipped default is **11.0**, so the two agree at
  different amplitudes.

### Consequences

- ⚠️ **Static stability is not a decision left to make. It is already gone.**
  ADR-0009 listed *"accept dynamic walking"* as option E and rejected it because
  the dynamics milestone did not exist. It exists now
  ([ADR-0052](#adr-0052), [ADR-0060](#adr-0060)), and the shipped robot is in
  option E whether or not anyone chooses it.
- ⚠️ **What the three lateral motors still earn is [ADR-0007](#adr-0007)'s
  righting reflex**, the dual use ADR-0009 itself cited. That is now their whole
  justification, and it has never been measured. Reclaiming them would return
  ~396 g (9 % of the mass budget) and give up righting.
- ⚠️ **The decision that remains is narrower than ADR-0063 framed it**: not
  *"sway or abduction"*, but *"can this robot walk dynamically, and is the
  righting reflex worth three motors"*. Neither is answered here.
- ✅ **What is not affected:** standing. The quadruped stands with 2.03 deg of
  tilt under a commanded sway and 0.006 deg without it. This is a **walking**
  finding.
- ⚠️ **Confidence:** the margin calculation is quasi-static and kinematic, which is
  what ADR-0009 used; the 2.60 mm input is a dynamic measurement over a 0.9 s
  window. Mixing them is deliberate -- it puts the measured capability into the
  original argument -- but neither half is a duty-cycle model.

## ADR-0068: Righting is a factor of five short, and the DOF it was specified on does not exist

- **Status:** Accepted as a **measurement**. Nothing ships. **Banners
  [ADR-0007](#adr-0007). ⚠️ Removes the last justification
  [ADR-0067](#adr-0067) left for the three lateral spine motors, without
  proposing what to do about it.**
- **Context:** ADR-0067 showed the lateral spine motors recover 3.9 % of the
  static-stability margin they were bought for, leaving ADR-0007's righting reflex
  as their whole remaining case. It had never been measured.

### ⚠️ First: the specified DOF is not in the robot

- **ADR-0006** targets three DOF per segment -- dorsoventral pitch, lateral yaw,
  **axial roll** -- and says the first pass exercises pitch, leaving
  *"lateral/axial parameterized for later"*.
- **ADR-0007** makes **axial twist** the primary righting authority:
  *"ADR-0006's axial-twist spine DOF becomes load-bearing for this goal"*.
- **ADR-0009** bought the **lateral** DOF for sway and justified it partly as
  *"the same lateral/axial spine authority serves ADR-0007 righting"* -- eliding
  the two.
- **NFR2c's 19 motors** are 12 leg + 6 spine (3 pitch + 3 yaw) + 1 tail.
  ⚠️ **Axial roll has no motors, and [ADR-0062](#adr-0062)'s census found it in no
  model.**

### ✅ The substitute mechanism is real

Driving pitch and yaw 90 deg out of phase makes the bend *direction* precess, and
a body with zero angular momentum then rotates about its own long axis. That is
the falling cat's bend-without-twist and it needs no axial joint.

- Measured in free fall (floor and gravity removed) the angular momentum stays at
  **2.4e-4 kg*m^2/s**, so the rotation is shape change, not a leak.
- The control confirms it: a **pitch-only** oscillation gives **+0.02 deg per
  cycle**.

### ⚠️ But the rate is short by five to fourteen times

At the joint limits (+-25 deg sagittal, +-15 lateral), command clipped to the real
motor:

| commanded | roll rate | achieved pitch / yaw |
|---|---|---|
| **25 / 15 deg @ 0.4 s** | **-52.7 deg/s** | 22.1 / 13.1 |
| 15 / 15 deg @ 0.4 s | -41.9 deg/s | 14.1 / 13.0 |
| 10 / 10 deg @ 0.4 s | -12.3 deg/s | 8.7 / 8.8 |

- Righting 180 deg needs **730 deg/s** from a cat-like 0.3 m drop, **398** from
  1.0 m, **282** from 2.0 m. At 53 deg/s the robot needs **3.4 s** -- a fall from
  **57 m**.
- ⚠️ **Range of motion is not where the missing factor lives.** Raising the lateral
  amplitude from 15 to 25 deg -- past the +-15 limit, so not even legal -- buys
  **1.6x**, not 5x.
- ⚠️ **The legs contribute 1.8 deg/s** on their own with the manoeuvre tried,
  though ADR-0007 names them as a co-equal mechanism.

### ⚠️ What this does not settle

These are naive sinusoidal shape cycles, not a designed righting law. The roll rate
**changes sign with the cycle period** (-66 deg/s at 0.4 s, +44 at 0.8), so the
result depends on the dynamics rather than on quasi-static geometry alone, and an
optimised manoeuvre is genuinely untested. The measured gap is **5-14x**; the
burden is on a manoeuvre that closes it.

> ⚠️ **[ADR-0069](#adr-0069) (M64) took that burden up. A designed manoeuvre
> gives +49 %, not 5x.** Adding the cat's own fore/hind anti-phase leg tuck raises
> the rate from 52.7 to **78.4 deg/s** -- still **3.6x** short from 2 m and 9.3x
> from a cat-like 0.3 m, and 180 deg would take a fall from **25.8 m**.

### Consequences

- ⚠️ **The three lateral spine motors now have no measured justification.**
  ADR-0067 removed the static-stability case; this removes the righting case as
  demonstrated. Reclaiming them returns **~396 g, 9 %** of the mass budget.
  **This ADR does not recommend that** -- it records that nothing currently earns
  them.

  > ⚠️ **CORRECTED by [ADR-0071](#adr-0071) (M66): G5 earns them, independently.**
  > This ADR read only ADR-0009's justifications -- sway and righting -- and both
  > did fall. But **G5** asks for a spine *"so the body can arch, bend laterally,
  > and twist like a real cat"*: lateral bend is a **capability the goal names**,
  > not a stability outcome, and the spine delivers it. What G6's withdrawal
  > actually frees is the **tail** motor.
- ⚠️ **G6 (land feet-first) is not met and has no route to being met** that has
  been measured. ✅ **[ADR-0071](#adr-0071) (M66) withdrew the goal.** Either an optimised manoeuvre closes a 5x gap, or the axial DOF
  ADR-0007 specified gets built and budgeted -- three more motors on a budget
  ADR-0058 only just closed.
- ✅ The tail, per ADR-0007, is *"a coarse inertial assist, not controlled
  reorientation"*. It is still unbuilt ([ADR-0062](#adr-0062)) and is not a
  candidate for closing this gap.

## ADR-0069: The designed righting manoeuvre gives +49 %, not the factor of five

- **Status:** Accepted as a **measurement**. Nothing ships. **Closes the
  "optimised manoeuvre" option [ADR-0068](#adr-0068) left open.**
- **Context:** ADR-0068 measured a naive precessing bend at 52.7 deg/s against the
  282-730 deg/s a fall allows, and said the burden was on a designed manoeuvre.
  This is that attempt.

### ✅ It works, and the biology is the optimum

Add the cat's own trick: tuck the fore legs while the hind pair extends, so the
front half's inertia about the roll axis falls while the rear's rises and the same
bend buys more body rotation.

| manoeuvre | roll rate |
|---|---|
| spine only (ADR-0068) | 52.7 deg/s |
| + symmetric tuck | 68.2 deg/s |
| **+ anti-phase tuck, 30 deg, 0.30 s** | **78.4 deg/s** |

- Swept over tuck mode, phase and frequency, the best is **fore/hind anti-phase at
  the precession frequency** -- exactly the pattern a falling cat uses.
- All 24 configurations kept the same sign, so the mechanism is robust rather than
  numerical. ✅ The phase sweep spans only 56-75 deg/s, which says the manoeuvre's
  *shape* matters at the tens-of-percent level.

### ⚠️ But +49 % is not 5x

- **78.4 deg/s** against **282** needed from 2.0 m: **3.6x short**. Against 398
  from 1.0 m: 5.1x. Against 730 from a cat-like 0.3 m: **9.3x**.
- 180 deg takes **2.30 s** -- a fall from **25.8 m**.
- ⚠️ **Pushing past the joint limits makes it unreliable, not stronger.** 30 deg of
  knee tuck is the largest legal amplitude (the hind knee sits at -102.8 deg in a
  -150..0 range). At 60 deg, outside the ROM, a 0.1 s change of period flips the
  roll **direction**: -95.9 deg/s at 0.30 s, **+82.8** at 0.40. A manoeuvre whose
  direction turns on the period that finely is not a reflex.

### Why the shape of the trajectory is not the dominant variable

Rotation from a shape cycle scales with the **area enclosed in shape space**, and
that area is bounded by the range of motion: +-25 deg sagittal, +-15 lateral,
+-30 of legal knee tuck. ADR-0068 already measured that opening the lateral
amplitude 15 -> 25 deg buys **1.6x**. Trajectory design moved it **1.49x**. Neither
is the missing **5x**, and they multiply to about 2.4 at best -- while requiring a
ROM that does not exist.

### Consequences

- ⚠️ **Option 1 is closed.** A designed manoeuvre does not reach G6. What remains
  is the **axial DOF ADR-0007 actually specified** (+3 motors on a budget
  [ADR-0058](#adr-0058) only just closed) or **dropping G6**.

  > ⚠️ **[ADR-0070](#adr-0070) (M65) priced the axial DOF and it makes righting
  > WORSE.** Closed loop, one of sixteen drive configurations rights at all, and
  > it takes **3.30 s** against the shipped spine's **2.14 s**. ✅ The same ADR
  > closed the loop and the robot **does** right -- in 2.14 s, which is a fall
  > from **22.5 m**.
- ✅ **One thing is now known that was not**: the mechanism is real, the cat's own
  pattern is the optimum, and the rate is **78.4 deg/s**. A future axial-DOF
  proposal has a number to beat and a manoeuvre to start from.
- ⚠️ **Confidence:** free-fall, zero gravity, angular momentum conserved to
  2.4e-4. The manoeuvre space was swept on six axes (bend amplitude, bend period,
  tuck amplitude, tuck phase, tuck frequency, fore/hind pattern), not optimised
  globally. A better trajectory may exist; a 5x better one is not consistent with
  the ROM bound above.

## ADR-0070: The robot rights itself, from 22.5 m -- and the axial DOF makes it worse

> ⚠️ **QUALIFIED by [ADR-0075](#adr-0075) (M70).** The 2.14 s below is a
> **rigid-spine** figure, and it does not report that the spine motors are
> commanded **5002.9 N** against a 222.9 N rating — saturated for the whole
> manoeuvre. Give the spine the drivetrain [ADR-0026](#adr-0026) requires and the
> same manoeuvre takes **10.10 s**.

- **Status:** Accepted as a **measurement**. Nothing ships. **Closes the axial-DOF
  option [ADR-0069](#adr-0069) left open. ✅ First demonstration that the robot
  rights at all.**
- **Context:** ADR-0068 and ADR-0069 measured rotation *rates* from open-loop shape
  cycles, where the direction proved unpredictable across parameters. Two questions
  were left: does closing the loop help, and would ADR-0007's specified axial DOF
  be worth three motors?

### ✅ Closing the loop: the robot rights

Read the roll error, run the shape cycle in whichever direction reduces it, stop
inside a 10 deg deadband. The sign becomes a control decision instead of an
accident, and only the magnitude is left to measure.

- ✅ **From fully inverted the robot reaches upright in 2.14 s** and settles at
  5.4 deg. This is the first time it has righted rather than merely rotated.
- ⚠️ **2.14 s is a fall from 22.5 m.** A cat rights in ~0.3 s from ~0.3 m. The
  windows here are 0.247 s from 0.3 m, 0.639 s from 2.0 m, 1.010 s from 5.0 m --
  so even from five metres it is **2.1x** short.
- ✅ **Feedback buys direction, not speed**, as expected: the effective rate is
  180/2.14 = **84 deg/s** against ADR-0069's open-loop **78.4**. The magnitude
  comes from the area a shape cycle encloses and the ROM bounds that; no control
  law enlarges it.
- ⚠️ The wrong cycle direction simply fails: it ends where it started.

### ⚠️ The axial DOF, priced before buying

ADR-0068 found ADR-0007's specified mechanism -- spine **axial twist** -- in no
budget and no model. Added to the model and swept over eight drive phases and both
cycle directions:

| spine | rights? | time |
|---|---|---|
| **shipped (pitch + yaw)** | ✅ | **2.14 s** |
| + axial, best of 16 | ✅ | **3.30 s** |
| + axial, the other 15 | ⚠️ no | -- |

- ⚠️ **One of sixteen configurations rights at all, and it is slower than having
  no axial DOF.**
- **As driven**, and the qualifier is real: the drive is a sinusoid at a fixed
  phase offset locked to the bend cycle, not the cat's two-phase
  bend/twist/unbend/untwist sequence. But the phase was **swept rather than
  guessed**, the best result is worse than the baseline, and 15 of 16 fail.

### Consequences

- ⚠️ **G6 is not met and the remaining route does not work.** The robot rights
  from **22.5 m**; nothing in the project suggests a fall that long is in scope.
  ✅ **[ADR-0071](#adr-0071) (M66) withdrew the goal on exactly this figure.**
- ⚠️ **The three lateral spine motors still have no measured justification**
  ([ADR-0067](#adr-0067), [ADR-0068](#adr-0068)), and **three more axial motors
  would not supply one**. Reclaiming the lateral three returns ~396 g, 9 % of the
  mass budget. As before, this ADR records rather than recommends.
- ✅ **What is now demonstrated**: the mechanism, the closed-loop law, the
  effective rate (**84 deg/s**), and the fact that feedback changes direction and
  termination but not magnitude. A future proposal has all four to beat.
- ⚠️ **Untested**: the cat's actual two-phase sequence with the axial DOF, and any
  manoeuvre found by optimisation rather than by sweeping a sinusoid family.

## ADR-0071: G6 is withdrawn -- mid-air righting leaves the project

- **Status:** Accepted. **A project-owner decision, taken on measurement.**
  **Withdraws G6, FR11 and G5's twist clause. Supersedes [ADR-0007](#adr-0007).
  Takes NFR2c from 19 actuated DOF to 18, which is what is built, and makes G5
  MET. ⚠️ Corrects [ADR-0068](#adr-0068)'s claim that nothing earns the lateral
  spine motors.**
- **Context:** [ADR-0067](#adr-0067) through [ADR-0070](#adr-0070) measured what
  the robot can actually do in the air. The decision was put to the project owner
  with those numbers and the answer was that 22.5 m is not realistic.

### What was measured

| | |
|---|---|
| time to right from fully inverted | **2.14 s** |
| the fall that allows | **22.5 m** |
| a cat | ~0.3 s from ~0.3 m |
| a 5 m drop allows | 1.010 s — still **2.1×** short |

Every route was measured and closed:

- **Trajectory design** ([ADR-0069](#adr-0069)): +49 %, and the cat's own
  fore/hind anti-phase pattern is the optimum. Not 5×.
- **The axial DOF the goal itself names** ([ADR-0070](#adr-0070)): **negative** --
  one of sixteen drive configurations rights at all, taking 3.30 s against 2.14.
- **Feedback** ([ADR-0070](#adr-0070)): buys direction and termination, not speed.
  84 deg/s against the open loop's 78.4.
- **Range of motion** ([ADR-0068](#adr-0068)): opening the lateral amplitude past
  its own limit buys 1.6×.

✅ The physics is not in doubt and neither is the mechanism: a precessing bend
rights a zero-momentum body with no axial joint, and the robot **does** right. It
is the *rate* that a spine of this ROM, on motors of this rating, cannot supply.

### What goes with G6

- **FR11** (*"detect a fall and reorient to land feet-first"*) is withdrawn with
  the goal it serves.
- **The tail motor.** [ADR-0007](#adr-0007) gave the tail exactly one purpose --
  *"a coarse inertial assist"* for righting -- and it was never built
  ([ADR-0062](#adr-0062): no parameters, no joint, no body). ✅ **NFR2c goes from
  19 actuated DOF to 18, and 18 is what M57 built**, so the requirement is now met
  rather than owed.
- **ADR-0007** is superseded. It is kept because ADR-0068 to ADR-0070 hang their
  measurements on it.

### ⚠️ What does NOT go with G6

- **The three lateral spine motors stay.** ADR-0068 said nothing earned them; that
  was wrong. It read only ADR-0009's justifications -- static-stability sway and
  righting -- and both did fall. But **G5** asks for a spine *"so the body can
  arch, bend laterally, and twist like a real cat"*. Lateral bend is a
  **capability the goal names**, not a stability outcome, and the spine delivers
  it (+-11 deg/segment, measured).
- ~~⚠️ **G5 is itself only partly met**: it also asks the body to *twist*, and
  the axial DOF is still unbuilt.~~ ✅ **Closed in the same session: the owner
  withdrew G5's twist clause too. See "The twist clause" below.**

### The twist clause, withdrawn with it

Asked what the *twist* in G5 was for, the documents had no answer. Axial roll is
specified in [ADR-0006](#adr-0006)'s three-DOF-per-segment target and deferred
there (*"lateral/axial parameterized for later"*); `params.py` records it as the
**most compliant** axis in a real cat; and [ADR-0007](#adr-0007) made it the
primary righting authority. **That was the only use ever written down**, and it
left the project with G6.

- ✅ **The project owner withdrew the clause.** G5 now reads *"arch and bend
  laterally"*, and both are built and measured -- ±25 deg sagittal, ±15
  lateral, three segments ([ADR-0062](#adr-0062)).
- ✅ **So the spine is complete at 6 DOF**, and **G5 is met**. Axial roll is not
  deferred any more; it is out of scope.
- ⚠️ **What this costs, stated plainly**: a real cat's most compliant spinal axis
  is not in this robot. If a future capability wants it -- turning in place,
  squeezing through a gap, setting a landing attitude -- it re-opens as a new
  requirement with a use attached, which is what was missing this time. The
  measurement [ADR-0070](#adr-0070) made says only that axial twist is *harmful to
  righting*; it says nothing about any other use.

### Consequences

- ⚠️ **The robot cannot right itself in any realistic fall.** Anything that could
  drop it -- a bench, a table, a stumble on stairs -- lands it on whatever face it
  was falling on. If that matters mechanically, it is now a **structural** problem
  (impact tolerance, ADR-0026's compliance) rather than a control one.
- ✅ **NFR2c is met**: 18 actuated DOF, 18 built.
- ⚠️ **Reopening G6 has a defined entry price**: beat **84 deg/s** while standing
  up the measurements in ADR-0068 to ADR-0070, which name the mechanism, the
  optimum manoeuvre family and the closed-loop law. The one route never tried is a
  manoeuvre found by **optimisation** rather than by sweeping a sinusoid family.

## ADR-0072: The impact question cannot be answered -- the whole-body model has no G3

- **Status:** Accepted as a **gap finding**. Nothing ships. **⚠️ Blocks the impact
  assessment [ADR-0071](#adr-0071) created. Puts a scope banner on
  [ADR-0026](#adr-0026) and [ADR-0051](#adr-0051).**
- **Context:** ADR-0071 withdrew G6. That does not remove the risk of falling; it
  moves it from **control** to **structure**. The obvious next question is what a
  fall costs. The model cannot say, and the reasons are two absences.

### ⚠️ G3 is not in the whole body

- [ADR-0026](#adr-0026) requires passive compliance; [ADR-0051](#adr-0051) put it
  in the **drivetrain**, and M45/M50 sized it at **175 kN/m**.
- `single_leg_rig(spools=...)` has it: three torsional springs at
  **11.484 N*m/rad**. ⚠️ **`quadruped_rig` has no `spools` parameter at all**, so
  the quadruped and the spine quadruped have **none**, and no winding constraints
  either.
- ⚠️ ~~**Every whole-body result in this project was measured on rigid tendons**
  -- ADR-0058's standing, ADR-0063 to ADR-0067's sway, ADR-0068 to ADR-0070's
  righting. Those are quasi-static or free-fall problems where compliance is
  unlikely to dominate, so this is a **scope note, not a retraction**. Impact is
  the case where it would dominate.~~

  > ✅ **HALF RIGHT, MEASURED by [ADR-0075](#adr-0075) (M70).** The sway does
  > not care (2.60 — 2.81 mm) and neither does standing (ADR-0074). ⚠️ **The
  > righting does**: with the drivetrain on the spine as well it takes **10.10 s
  > against 2.14**, a factor of 4.7. The discriminator is saturation — the
  > sway asks the spine for 76.8 N of its 222.9 N rating, the righting for
  > **5002.9 N**.

### ⚠️ And in an uncontrolled fall the cables carry nothing

The shipped tendons are pure actuators. At `ctrl = 0` they apply **0.0000 N**, so
the joints are free and the fall loads the **joint stops and the structure**. A
real robot's motors hold position and its G3 spring takes the shock. Neither is
modelled, so the load path a falling robot actually sees does not exist here.

### What the model can still say

| drop, onto the side | peak contact | x body weight |
|---|---|---|
| 0.05 m | 381 N | **9.0** |
| 0.10 m | 426 N | 10.1 |
| 0.30 m | 571 N | 13.5 |
| 1.00 m | 888 N | **21.0** |

- These are an **upper bound**: no compliance anywhere in the path.
- ⚠️ **The first thing a fall does is blow through the spine's joint stops.** A
  0.30 m side drop drives `spine_y2` **27.3 deg past a +-15 deg limit** -- nearly
  three times its range -- with `spine_y1` 10.2 deg over. In hardware that is the
  joint or its tendon, not a soft limit.
- ⚠️ **Do not compare these with the 638 N cable rating.** Contact force and cable
  tension are different quantities, and this ADR measured only the first.

### Consequences

- ✅ **UNBLOCKED by [ADR-0073](#adr-0073) (M68)**: `quadruped_rig(spools=)`
  exists, G3 is on the whole body, and the landing is priced there.
- ⚠️ **The impact assessment is blocked on a capability, not on effort**: the
  whole body needs its drivetrain before a landing can be priced. That is the same
  shape as [ADR-0059](#adr-0059)'s finding that the drivetrain had no home behind
  the pulley -- and the fix there was one spool per pair.
- ⚠️ **The spine's lateral joint stops are the first thing to check in hardware**,
  whatever the model gains later. +-15 deg is the narrowest axis on the robot and a
  0.30 m fall triples it.

  > ⚠️ **QUALIFIED by [ADR-0073](#adr-0073) (M68): that was an UNPOWERED fall.**
  > With the motors holding the stance pose the joint overshoot is **0.0 deg**,
  > with or without the drivetrain. The finding stands for a robot that has lost
  > power; it is not a statement about missing compliance.
- ✅ **What is not in doubt**: a fall costs **9x body weight at 50 mm** and
  **21x at a metre**, before any compliance is credited.

## ADR-0073: The whole body gets its drivetrain -- G3 takes the shock out of the cable, not the ground

- **Status:** Accepted. `quadruped_rig(spools=)` and `quadruped_rig_spooled` ship.
  **Unblocks the impact assessment [ADR-0072](#adr-0072) named. ✅ First
  demonstration of [ADR-0026](#adr-0026)'s compliance requirement on a whole body.
  ⚠️ Qualifies ADR-0072's joint-stop finding.**
- **Context:** ADR-0072 found G3 -- the compliance ADR-0026 requires and
  [ADR-0051](#adr-0051) put in the drivetrain -- absent from every whole-body
  model, so a landing could not be priced.

### ✅ Built: twelve spools, twelve equalities, twelve springs

Three pairs on each of four legs, each with a rotor, a spool, a winding equality
and a G3 torsional spring at **11.484 N*m/rad**. It works on the spine quadruped
too.

⚠️ **Both of [ADR-0052](#adr-0052)'s traps reproduced on the whole body, in
order.** The winding equality is referenced at `qpos0`, so without a two-pass
offset every cable starts violated -- **72.2 mm** at the stance pose, against the
24-52 mm the single leg showed. `quadruped_rig_spooled` does the two passes and
the residual is **2.2e-9 m**. And the equality **overpowers a default-stiffness
joint limit**, so the limits are solved as stiffly as the equality. Both were
already written down from the single leg, which is why they were expected rather
than discovered.

### ✅ G3 does its job, and the job is narrower than it sounds

Dropped on its side with the **motors holding** the stance pose -- the only
condition under which G3 loads at all, because at `ctrl = 0` the rotor spins free
and the cable pays out:

| drop | rigid tendons: cable | with G3: cable | contact, either |
|---|---|---|---|
| 0.05 m | **223 N, saturated** | **84 N** | ~385 N |
| 0.10 m | **223 N, saturated** | 95 N | ~430 N |
| 0.30 m | **223 N, saturated** | 127 N | ~575 N |

- ⚠️ **On rigid tendons the motor saturates on every impact tested**, including a
  50 mm drop. 223 N is `TENSION_MAX`, so that column is a **floor** on the real
  demand, not a measurement of it.
- ✅ **With the drivetrain the peak falls to 84-127 N** -- inside the 222.9 N
  peak rating throughout, and near the 81.1 N continuous rating at 50 mm.
- ⚠️ **The contact force is unchanged** (381 vs 388 N at 50 mm). **G3 protects the
  drivetrain, not the ground reaction**: the floor still takes 9x body weight at
  50 mm and 13x at 0.30 m. Anything that has to survive the *impulse* -- structure,
  bearings, the girdles -- gets no help from it.

### ⚠️ And it qualifies ADR-0072

That ADR found a 0.30 m fall driving `spine_y2` **27.3 deg past its +-15 deg
limit**. That was an **unpowered** drop. With the motors holding, the overshoot is
**0.0 deg with or without the drivetrain**. The finding stands for a robot that
has lost power; it is not a statement about missing compliance, and this ADR does
not withdraw it -- an unpowered fall is a real case.

### Consequences

- ✅ ~~**A landing can now be priced**, and the first figure is that a powered
  robot's cables stay inside their rating from 50 mm to 0.30 m.~~

  > ⚠️ **ON A RIGID TRUNK ONLY — [ADR-0076](#adr-0076) (M71).** Repeat this drop
  > on ADR-0006's articulated spine, same controller and same drivetrain, and the
  > leg cable is **saturated at 222.9 N at every height**. The 84–127 N margin
  > was bought by a trunk that does not move. The spine's own cables run at
  > **2.4–4.1 kN** against the same 222.9 N rating.
- ⚠️ **Every whole-body result before M68 was measured on rigid tendons** and can
  now be re-checked with compliance. ADR-0072 argued those are quasi-static or
  free-fall problems where compliance should not dominate; that argument is now
  testable rather than assumed.

  > ✅ **TESTED by [ADR-0075](#adr-0075) (M70), and it splits.** The sway does
  > not care; the righting takes **4.7× as long** once the spine has a
  > drivetrain too — which the whole-body build does **not** give it.
  > `quadruped_rig(spools=)` fits the twelve leg pairs and no others.
- ⚠️ **Untested**: heights above 0.30 m, landing on a corner or a single leg, and
  what the 9-13x contact impulse does to the structure. The cable is safe; nothing
  else has been checked.
- ⚠️ ~~**[ADR-0060](#adr-0060)'s headroom applies here too**: a drivetrain halves
  the lowest mode. The whole-body control margin has not been re-measured with
  spools fitted.~~

  > ✅ **MEASURED by [ADR-0074](#adr-0074) (M69), and it goes the other way.**
  > The mode does drop -- 12.6 -> 4.6 Hz on the plain quadruped -- but the
  > **practical margin improves**: on rigid tendons any joint PD saturates the
  > motor at `kp = 50`, while with the drivetrain the same gain costs **74.4 N**
  > and holds the trunk twice as level.

## ADR-0074: The whole-body drivetrain lowers the mode and improves the margin

- **Status:** Accepted as a **measurement**. Nothing ships. **Closes the margin
  item [ADR-0073](#adr-0073) left open, in the opposite direction to the one it
  expected.**
- **Context:** [ADR-0060](#adr-0060) found a drivetrain halving the lowest mode on
  one leg (54.9 -> 27.4 Hz) and taking the usable outer gain from 600 to 200.
  ADR-0073 fitted a drivetrain to the whole body and flagged that nobody had
  re-measured.

### ⚠️ The mode drops, as expected

| plant | nv | lowest mode |
|---|---|---|
| quadruped, rigid | 18 | **12.6 Hz** |
| quadruped, with drivetrain | 42 | **4.6 Hz** |

A 2.7x reduction, the same direction ADR-0060 measured on a leg.

⚠️ **One comparison that is not apples to apples**: the *spine* quadruped's lowest
mode is **1.5 Hz rigid** against 3.5 spooled -- lower **without** the drivetrain.
That is the spine's own mode, not a drivetrain mode, so "lowest mode" does not
compare those two plants and is not used here.

### ✅ The practical margin goes the other way

Standing with a joint-space PD added on top of the force allocation:

| plant | `kp` | tilt | peak cable |
|---|---|---|---|
| rigid | 0 | 0.006 deg | 68.2 N |
| rigid | 50 | 0.76 deg | **222.9 N, saturated** |
| rigid | 200 | 0.63 deg | **222.9 N, saturated** |
| rigid | 600 | 0.87 deg | **222.9 N, saturated** |
| **drivetrain** | 50 | **0.39 deg** | **74.4 N** |
| **drivetrain** | 200 | 0.44 deg | **122.0 N** |
| drivetrain | 600 | 8.58 deg | 222.9 N |

- ⚠️ **On rigid tendons any joint PD at all saturates the motor**, at `kp = 50`
  already, and the tilt gets *worse* than the pure force allocation (0.76 deg
  against 0.006).
- ✅ **With the drivetrain the same gains stay inside the rating** -- 74.4 N at
  `kp = 50`, 122.0 at 200 -- and the trunk is held **twice as level**. The series
  spring absorbs a stiff command instead of transmitting it as a force spike,
  which is what a series-elastic element is for.
- ✅ **The `kp = 0` baseline is undisturbed**: 68.2 N rigid, 71.7 N spooled, tilt
  0.006 deg on both. Fitting the drivetrain does not move [ADR-0058](#adr-0058)'s
  standing result.
- ⚠️ There is still an edge: at `kp = 600` the spooled plant saturates and the
  tilt reaches **8.58 deg**.

### Consequences

- ✅ **ADR-0073's warning is withdrawn.** The mode halves and the margin
  improves; the two are not the same quantity, and on a leg ADR-0060 measured the
  first while assuming the second.
- ✅ ~~**A joint-space posture term is now affordable.** Every previous whole-body
  controller used pure force allocation because a PD saturated the motors. That
  constraint was the missing compliance, not the gain.~~

  > ⚠️ **AFFORDABLE IN FORCE, NOT IN DYNAMICS —
  > [ADR-0078](#adr-0078) (M73).** The 74.4 N was measured with the balance loop
  > running at **10 kHz**. At any rate a real controller runs at, the same term
  > saturates the cable and tips the robot: **12.9° at 1 kHz** and
  > **180° at 133 Hz**, drivetrain or no drivetrain. Pure force allocation
  > is unaffected at every rate (0.01° / 72 N).
- ⚠️ **This does not re-check the results themselves.** Standing, sway and
  righting were measured on rigid tendons; this shows the drivetrain does not
  disturb the standing baseline, which is one of the three.

## ADR-0075: The compliance scope note discharged — the sway does not care, the righting does

> ⚠️ **CORRECTED by [ADR-0076](#adr-0076) (M71).** The legs+spine figures below
> were measured with the six spine spools mounted in `<worldbody>`. On the rear
> girdle, where a motor sits, the righting takes **10.10 s, not 7.69** — the
> conclusion holds and gets larger (**4.7×**, not 3.6), but the number and the
> spring sweep were both artefacts of the mounting. The sway rows are unaffected.

- **Status:** Accepted — M70, corrected M71
- **Supersedes nothing. ✅ Discharges the scope note
  [ADR-0072](#adr-0072) opened and [ADR-0073](#adr-0073) said was assumed rather
  than tested. ⚠️ Corrects [ADR-0070](#adr-0070).**
- **Context:** ADR-0072 found that every whole-body result in this project had
  been measured on rigid tendons, and argued the sway and the righting are
  quasi-static or free-fall problems where compliance is *unlikely to dominate*
  — a scope note, not a retraction. M68 built the whole-body drivetrain and
  M69 showed it does not disturb the standing baseline, which is one of the
  three results. The other two had still never been re-measured.

### ⚠️ First, the drivetrain the whole body has is only half a drivetrain

`quadruped_rig(spools=)` fits the **twelve leg pairs** and nothing else. The six
spine pairs — the actuator that performs both the sway and the righting —
are still rigid cables in the spooled build: **24 spool joints, none of them on
the spine.** So `_spine_spools` fits the other six by post-processing the XML,
the way `_with_axial` prices the axial DOF: a study, nothing that ships.

✅ **The instrument was checked before the measurement was read.** Commanded
against delivered on all six spine pairs: **-0.1 %** at 19.6, 100.0 and 222.9 N,
with the winding constraints inside a micron — the same figure the leg
drivetrain hits. M68's `18× worse impact` was an artefact of the plant rather
than a result, and that lesson is now a test.

### ✅ The sway does not care, at any level of compliance

| compliance | nv | CoM sway | tilt | spine demand |
|---|---|---|---|---|
| none (rigid) | 24 | **2.60 mm** | 2.033 deg | 76.8 N |
| legs (shipped) | 48 | **2.81 mm** | 2.020 deg | 76.8 N |
| legs + spine | 60 | **2.81 mm** | 2.013 deg | 76.8 N |

+8 %, in the helpful direction — [ADR-0067](#adr-0067) measured 2.60 mm
against [ADR-0009](#adr-0009)'s designed 66.7, so more sway is more of the margin
back. The tilt is flat to a hundredth of a degree.

### ⚠️ The righting does, once the spine has it

| compliance | t to right | closest | effective rate |
|---|---|---|---|
| none — [ADR-0070](#adr-0070) | **2.14 s** | 5.4 deg | 84 deg/s |
| legs (shipped) | **2.17 s** | 1.5 deg | 83 deg/s |
| legs + spine | **10.10 s** | 2.7 deg | **17.8 deg/s** |

✅ **The shipped drivetrain costs 1.4 %.** On the legs, ADR-0072's argument
holds as well as it does for the sway.

⚠️ **Put the same drivetrain behind the spine and righting takes 4.7× as
long.** The legs modulate inertia; the spine does the work. ADR-0072's claim was
about the whole body, and on the half of it that matters it is wrong.

### ⚠️ The discriminator is saturation, and ADR-0070 did not report it

The sway asks the spine for **76.8 N** of its 222.9 N rating. The righting asks
for **5002.9 N** — **22× the rating** — so the spine runs hard against
its stop for the entire cycle. A task inside the rating cannot be changed by the
transmission's stiffness; a task 22× outside it is changed by nothing else,
because what the series spring alters is how fast the stop is reached.

⚠️ **[ADR-0070](#adr-0070) published 2.14 s without publishing that the actuator
was saturated throughout.** The number stands on the plant it was measured on;
what it omits is that the manoeuvre has no force margin at all.

### ⚠️ And a short window read the compliant plant as a failure

M70's first pass ran 4 s and then 6 s and recorded **no righting** for the
legs+spine plant. It rights at **10.10 s**. [ADR-0064](#adr-0064) made the same
mistake about a different plant, and it was caught here only because a stiffness
sweep produced a result too strange to accept.

### Decision

Report the sway result as compliance-insensitive and the righting result as
compliance-dependent through the spine, and treat the spine transmission's
stiffness as an open design variable rather than a settled one.

### Consequences

- ✅ **[ADR-0072](#adr-0072)'s scope note is discharged for the sway and for
  the legs**, and it was right about both.
- ⚠️ **It is withdrawn for the spine.** [ADR-0070](#adr-0070)'s 2.14 s is a
  rigid-spine figure; with G3 on all eighteen cables the robot rights in 10.10 s,
  which is a fall from **500 m** against 22.5. ✅ That only strengthens M66's
  withdrawal of **G6** — the goal was already 9× short on the faster plant.
- ~~⚠️ **What fixes it is not established.** Time to right is **non-monotonic**
  in the spine spring: 7.69 s at the specified 150 kN/m, **2.68 at 500**, 9.40 at
  926, 10.82 at 3000 and 3.48 at 10 000 kN/m. A stiffer spring is not simply
  better, and five points are not a design curve.~~

  > ✅ **THE SWEEP WAS THE PLANT, not the spring** — [ADR-0076](#adr-0076)
  > (M71). Re-run with the spools on the girdle it is orderly: **10.10 s at 150
  > kN/m, 3.85 at 500, 4.11 at 926, 3.81 at 3000**. A stiffer spring helps and
  > stops helping above ~500 kN/m, which is 2.5–3.3×
  > [ADR-0050](#adr-0050)'s band.
- ⚠️ **The manoeuvre is as sensitive to the spine GAIN as to the spring.** On
  the rigid plant `kp = 300` rights in 2.14 s, `kp = 100` in **10.88**, and
  `kp = 48.6` not within 15 s at all. Nothing about this manoeuvre has margin.
- ⚠️ **The spine has no `spools` support in the builder.** `_spine_spools`
  post-processes XML; if the spine drivetrain is to be a design position rather
  than a study, `quadruped_rig` has to grow it.
- ⚠️ **Still untested**: whether the standing and impact results move when the
  spine has compliance — M69 and ADR-0073 measured those on the leg-only
  drivetrain too.

## ADR-0076: The spine's drivetrain, in the builder — and what it costs a landing

- **Status:** Accepted — M71
- **✅ Closes the builder gap [ADR-0075](#adr-0075) named. ⚠️ Corrects
  ADR-0075's righting figure. ⚠️ Qualifies [ADR-0073](#adr-0073).**
- **Context:** ADR-0075 measured the spine drivetrain through a post-processed
  XML and left two things open: `quadruped_rig` had no way to build one, and
  the standing and impact results had been measured on the leg-only drivetrain.

### ✅ `quadruped_rig(spine_spools=)` — G3 on all eighteen cables

| plant | nv | actuators | winding equalities | G3 springs |
|---|---|---|---|---|
| spine, rigid | 24 | 18 | 0 | 0 |
| spine, legs spooled | 48 | 18 | 12 | 12 |
| spine, all 18 spooled | 60 | 18 | **18** | **18** |

The spools ride on the **rear girdle**, where [ADR-0006](#adr-0006) puts the
spine motors, and M46's `a0` trap stays shut with six more cables in the model
(residual < 1e-6 m). `spine_spools=False` still builds the middle row, because
ADR-0075's comparison is between the two.

### ⚠️ And that mounting corrects ADR-0075's number

ADR-0075 hung the six spine spools in `<worldbody>`. On the girdle the same
manoeuvre gives:

| compliance | ADR-0075 (world) | M71 (girdle) |
|---|---|---|
| none | 2.14 s | **2.14 s** |
| legs | 2.17 s | **2.17 s** |
| legs + spine | 7.69 s | **10.10 s** |

The rigid and leg-only figures are untouched — their spools were always on
the girdle. The compliant-spine figure moves by **2.4 s**, and it moves the
wrong way for the robot: **4.7×** the rigid time, not 3.6, an effective
**17.8 deg/s** against 84, and a fall from **500 m** against 22.5.

✅ **It is not a momentum leak.** The obvious suspicion — that six motors
bolted to an inertial frame dump their reaction into the world — was tested
directly: in free fall with no contact the total angular momentum drifts
**1.16e-3** on the world-mounted plant and **9.71e-4** on the girdle-mounted
one. Same order, both from the tendon solver.

⚠️ **What it is, is that the manoeuvre has no margin anywhere.** `kp = 300`
rights in 2.14 s where `kp = 100` takes **10.88** and `kp = 48.6` never finishes
inside 15 s; a 4 s window read the compliant plant as a failure when it needed
10.10; and now a massless body's mounting point is worth **2.4 s**. A result this
sensitive to things that should not matter is a result to hold loosely.

✅ **Re-swept on the corrected plant, though, the spring stops being
mysterious.** ADR-0075 reported time to right as **non-monotonic** in the spine
spring and declined to recommend anything. That non-monotonicity was the
world-mounted plant:

| spine spring | cap `k*r^2` | ADR-0075 (world) | M71 (girdle) |
|---|---|---|---|
| 150 kN/m (specified) | 48.6 N*m/rad | 7.69 s | **10.10 s** |
| 500 | 162.0 | 2.68 | **3.85 s** |
| 926 | 300.0 | 9.40 | **4.11 s** |
| 3000 | 972.0 | 10.82 | **3.81 s** |

A stiffer spring **helps, and stops helping above ~500 kN/m**. So the spine
wants roughly **500 kN/m**, 2.5–3.3× [ADR-0050](#adr-0050)'s 150–200
band — which was set for the leg and the impact case, and was never asked
about a spine.

### ⚠️ ADR-0073's cable margin was bought by a rigid trunk

| plant | 0.05 m | 0.10 m | 0.30 m |
|---|---|---|---|
| rigid box, rigid cables | 222.9 N sat. | 222.9 sat. | 222.9 sat. |
| rigid box, legs G3 | **84.4 N** | 94.9 | 127.5 |
| **articulated spine, legs G3** | **222.9 sat.** | **222.9 sat.** | **222.9 sat.** |

The first two rows reproduce ADR-0073 exactly (it published 84 / 95 / 127). The
third is the same drop, the same controller and the same drivetrain, with the
trunk the robot is actually going to have. ⚠️ **The margin G3 bought is gone**,
and it is the articulation that takes it, not the way the spine is held: the leg
cable saturates at spine hold gains of 0, 50 and 300 alike.

⚠️ **The spine's own cables have never been measured in a fall, and they are
worse off**: **2.4–4.1 kN** of demand against a 222.9 N rating, **11–18×**.

### ⚠️ With G3 on the spine as well, the landing stops being measurable

| drop | spine demand | contact |
|---|---|---|
| 0.05 m | 26 607 N | 4831 N |
| 0.10 m | 26 273 N | 986 N |
| 0.30 m | 24 957 N | 11 040 N |

Contact is **not monotonic in drop height**, and 11 kN on a 4.30 kg robot is
**260× body weight**. The leg-only plant lands at 498–671 N over the same
heights, monotonically. These numbers are **named, not published**: what is
diagnosable is that the spine loop runs at **>20× its rating** where the
rigid-spine plant runs at 11–18×, which is ADR-0075's finding again — a
`kp = 300` spine loop is not realisable through a transmission that can present
`k*r^2 = 48.6 N*m/rad`.

### Decision

Ship the spine drivetrain in the builder. Correct ADR-0075's righting figure to
10.10 s. Withdraw ADR-0073's cable margin as a whole-body claim and record it as
a rigid-trunk one. Record the compliant-spine landing as **not yet answerable**
rather than publishing a number.

### Consequences

- ✅ **The whole robot can carry G3 now**, and the topology check is a test.
- ⚠️ **[ADR-0073](#adr-0073)'s headline needs its qualifier**: G3 takes the
  shock out of the cable *on a rigid trunk*. On ADR-0006's spine it does not.
- ⚠️ **The spine needs a controller that lives inside its transmission** before
  any landing question can be asked of the compliant plant. That is the same
  requirement ADR-0075 raised for the righting, now blocking a second result.
- ⚠️ **Standing with the spine spooled is still unmeasured.** M69's margin was
  taken on a plant with **no spine at all**, and M71 has not re-run it.
- ⚠️ **A modelling detail worth 2.4 s is a warning about every free-fall number
  in this project**, not just this one.

## ADR-0077: The plant an agent would train in has no senses

- **Status:** Accepted — M72
- **✅ Answers the tooling question directly (MJX is viable). ⚠️ Finds that
  every controller in this project reads state the robot cannot measure.**
- **Context:** The project owner set the goal: **maximally realistic modelling
  for learning in simulation**. That changes the target from *accurate* to
  *randomisable and fast*, and it makes one question decisive — what can the
  robot actually observe?

### ✅ First, the tooling question: MJX is viable, no blocker

Measured rather than assumed, in a throwaway venv so nothing shipped moved:

| check | result |
|---|---|
| feature parity | `TENDON` equality, `<fixed>` tendons, tendon transmission, `IMPLICITFAST`, condim 4/6 — all supported |
| `mjx.put_model` on the shipped plant | **`neq = 18`, `eq_type = [mjEQ_TENDON]`** — the drivetrain survives |
| MJX against C MuJoCo, 400 steps | **0.029 deg** worst joint |
| `vmap` at batch 32 | finite, compiles, runs |
| MuJoCo 3.10 → 3.12 (which `mujoco-mjx` requires) | **bit-identical** across 61 qpos × 400 steps |

✅ **The failure worth guarding against did not happen**: MJX does not
silently drop the winding constraints, which would have handed a policy a robot
with no G3 to train against. ~~⚠️ Throughput is still unanswered — JAX has no
CUDA build on Windows (`CpuDevice`), so the speedup case needs Linux/WSL2 and a
GPU.~~

> ✅ **WRONG ABOUT THE HARDWARE — [ADR-0079](#adr-0079) (M74).** JAX has no
> Windows CUDA wheel, but **Warp does**, and this machine has an **RTX 4090**.
> `mujoco-warp` reaches it, keeps all 18 equalities, agrees with C to **2e-6
> rad**, and runs **262,678 steps/s** at 8,192 worlds. No new hardware was
> needed. Per-env MJX on CPU is 1,374 steps/s against C's 25,320, which is expected
and is not the point of MJX.

### ⚠️ A timestep trap, found on the way

| timestep | x realtime | winding-equality residual |
|---|---|---|
| 1e-4 (shipped) | 1.60 | **0.5 um** |
| 2e-4 | 3.16 | 0.7 um |
| 5e-4 | 7.38 | 124 um |
| 1e-3 | 8.05 | **16 mm** |
| 2e-3 | 15.0 | 204 mm |

⚠️ **At 1e-3 the drivetrain constraint is out by 16 mm and nothing warns.** The
sim does not crash; G3 simply stops being modelled. Anyone speeding the plant up
for training without asserting that residual trains against a different robot.
2e-4 is a free 2×; past 5e-4 the model is no longer the model.

### ⚠️ And the finding that matters most: `nsensor = 0`

`electronics/BOARD_OUTLINE.md` puts this on the robot:

| the board carries | count |
|---|---|
| rotor absolute encoder (AS5047/MA-class, [ADR-0004](#adr-0004)) | 18 |
| phase-current sense | 18 |
| tendon load cell, in-amp front-end | **14** — spine+hip/knee only, DNP on ankle/tail |
| IMU on the trunk | 1 |
| per-foot contact and normal force ([FR12](REQUIREMENTS.md)) | 4 |
| **joint encoder** | **0** |

⚠️ **There is no joint encoder, and every controller in this project reads
`d.qpos[joint]`.** The standing gate, the sway, the righting, the landing —
all of them run on privileged state that hardware cannot supply. The plant had
**no sensors at all** until this milestone.

⚠️ `firmware/README.md` lists *"joint angle"* among the sampled signals. The
board has no such channel. That inconsistency is now recorded rather than
carried.

### ✅ Joint angle IS recoverable — and the ankle load cell is what pays

Two facts line up: the winding equality gives `L = a0 - r*(theta_r + theta_s)`,
and the pair map is **lower-triangular** (`L = C q`), so `C` inverts. Hence
`q = C^-1 (a0 - r*(theta_r + theta_s))`.

| what the robot knows | worst joint error |
|---|---|
| perfect encoder + all 18 load cells | **0.004 deg** |
| 14-bit encoder + all 18 load cells | **0.010 deg** |
| 14-bit encoder, **no ankle load cell** | **1.09 deg** |
| perfect encoder, no ankle load cell | 1.08 deg |

✅ **The encoder is not the limit.** A 14-bit absolute encoder costs
0.006 deg. ⚠️ **The missing ankle load cell costs 111× that**, and the last
row proves the attribution: a perfect encoder without the cell is no better.

⚠️ **It scales with tension**, because what is being ignored is G3's own spring
deflection:

| ankle tension | ankle angle error |
|---|---|
| 25 N | 0.68 deg |
| 81.1 N (continuous rating) | **2.21 deg** |
| 222.9 N (peak — where [ADR-0076](#adr-0076) found the leg runs in a landing) | **6.08 deg** |

### Decision

Emit the board's sensor suite from the builder (`sensors=`, default off so no
measurement moves), ship the reconstruction in `wbc.joint_from_encoders`, and
treat any observation space built for learning as a function of **that** suite
and not of `qpos`.

### Consequences

- ✅ **`electronics/BOARD_OUTLINE.md`'s open G-Tens item now has a number.**
  "Which driver boards populate the load-cell front-end" is decided by 0.010 deg
  against 2.21–6.08 deg of ankle-angle knowledge.
- ⚠️ **Every whole-body result in this project used privileged state.** Nothing
  is retracted — they are plant measurements, not controller proposals —
  but none of them is a controller that could run on the robot.
- ⚠️ **Latency is still zero.** [NFR12](REQUIREMENTS.md) budgets **7.5 ms**
  (contact 1.0 + estimation 5.0 + transport 1.0 + compute 0.5), which is **75
  steps** at the shipped timestep. A policy trained at zero delay will exploit
  it. That is the next gap, and it is bigger than any of the fidelity items.
- ⚠️ **Capstan friction still cannot be randomised** because the mechanism does
  not exist (`wrap_angle = 0`, [ADR-0003](#adr-0003)). For learning, the reason
  to build it is not accuracy — it is that a parameter absent from the model
  cannot be randomised over.
- ✅ **Do not switch simulators.** Discrete cables cost **14.3×** on this
  plant (nv 60 → 1032) and buy what randomising the lumped parameters buys.

## ADR-0078: It is the joint PD that cannot take a real control rate — and NFR12 holds

- **Status:** Accepted — M73
- **✅ Validates [NFR12](REQUIREMENTS.md) on the plant for the first time.
  ⚠️ Qualifies [ADR-0074](#adr-0074).**
- **Context:** [ADR-0077](#adr-0077) found the plant had no senses and no delay.
  It also turned up something neither ADR-0014 nor M11 had noticed: **every
  controller in this project recomputes every physics step**, so the balance
  loop has always run at **10 kHz**. `firmware/README.md` specifies >=1 kHz for
  the *motor* loops; NFR12's 7.5 ms pipeline implies ~**133 Hz** for balance.

### ✅ The force allocation does not care about the rate. The PD does.

Standing, spooled plant, zero latency throughout:

| control rate | `kp = 0` | `kp = 25` | `kp = 50` |
|---|---|---|---|
| 10 kHz (as shipped) | 0.01 deg / 72 N | 0.34 / 73 | 0.39 / 74 |
| 1 kHz | **0.01 deg / 72 N** | 12.9 / 223 | 36.4 / 223 |
| 500 Hz | **0.01 deg / 72 N** | 9.9 / 223 | 18.5 / 223 |
| 133 Hz (NFR12) | **0.01 deg / 72 N** | 180.0 / 223 | 94.5 / 223 |

✅ **[ADR-0058](#adr-0058)'s standing gate is rate-insensitive** — a
hundredth of a degree at 72 N whether it runs at 10 kHz or 133 Hz. The
controller doing the actual work needs none of the rate it was given.

⚠️ **[ADR-0074](#adr-0074)'s posture term is what breaks.** M69 measured it at
**74.4 N** with the drivetrain against a saturated 222.9 N without, and called
it *affordable*. That is true of its **force cost** and false of its
**dynamics**: below 10 kHz it saturates the cable and tips the robot at every
gain tried, drivetrain or not.

### ✅ And NFR12's 7.5 ms budget holds, measured rather than assumed

Pure force allocation — the configuration the rate study leaves standing:

| rate | 0 ms | 5 ms | **7.5 ms** | 10 ms | 15 ms | 20 ms |
|---|---|---|---|---|---|---|
| 1 kHz | 0.01 deg / 72 N | 0.01 / 73 | **0.01 deg / 74 N** | 0.31 / 105 | 1.81 / 158 | 3.59 / 223 |
| 133 Hz | 0.01 deg / 73 N | 0.01 / 73 | **0.91 deg / 119 N** | 0.43 / 138 | 0.15 / 192 | 8.08 / 223 |

✅ **At 1 kHz the budget costs 2 N** (74 against 72). At 133 Hz it costs
**47 N** and 0.91 deg — standing, with less room. ⚠️ **The failure boundary
is 15–20 ms**, where the cable saturates. NFR12 was re-cast from a whole-loop
<=20 ms; on this evidence 20 ms is exactly the edge, and the re-cast is what
buys the margin.

⚠️ **Read the cable column, not the tilt.** Tilt is non-monotonic at 133 Hz
(0.91 → 0.43 → 0.15 → 8.08) because a robot on the edge falls
whichever way it happens to lean. Peak cable is monotonic at both rates.

### ⚠️ Two of this milestone's own sweeps were wrong first

Recorded because the pattern keeps recurring and the corrections are the useful
part:

1. The first sweep reported **1 ms of latency knocks the robot over** (116.9 deg
   tilt). It measured a **10 kHz** loop with dead time — a rate no controller
   runs at. The non-monotonic tilts were the tell.
2. The second added rate decimation and reported the controller failing at 1 kHz
   with **zero** latency. It decimated `wbc.rotor_command` along with the balance
   loop — but that is the motor's **inner** loop closing on its own shaft
   encoder ([ADR-0059](#adr-0059)'s cascade). Starving it of its own feedback is
   a property of the harness, not the robot.

Both would have condemned a controller that is fine.

### Decision

Ship `wbc.SensorDelay` and `_stand_at_gain(control_hz=, latency_s=)`, defaults
inert. Mark NFR12 **MET**. Qualify ADR-0074's affordability claim as a 10 kHz
result rather than withdrawing it.

### Consequences

- ✅ **NFR12 is met with margin at 1 kHz** and met with less at 133 Hz. The
  first requirement in this project validated against the plant rather than an
  analytical envelope.
- ⚠️ **The null-space posture task ADR-0074 unblocked is blocked again**, for a
  different reason. ADR-0052 named it, ADR-0058 kept it open, ADR-0074 made it
  affordable in force — and it still needs a formulation that survives a
  realistic rate.
- ✅ **The whole-body controller is implementable at 133 Hz**, which the
  hardware can comfortably do. That was not previously known.
- ⚠️ **Only standing has been tested this way.** The sway, the righting and the
  landing all still run at 10 kHz on true instantaneous state.
- ⚠️ **Latency is still absent from the plant itself** — this is a harness
  facility. An RL environment wants it as a wrapper, together with the sensor
  suite ADR-0077 added.

## ADR-0079: The throughput question, answered on the machine that was already here

- **Status:** Accepted — M74
- **✅ Closes the throughput item [ADR-0077](#adr-0077) left open.
  ⚠️ Corrects ADR-0077's claim that it needed different hardware.**
- **Context:** ADR-0077 verified MJX-JAX preserves the drivetrain but could not
  measure throughput, and recorded that the answer *"needs Linux/WSL2 and a
  GPU"*. That was reasoned from JAX — which genuinely has no Windows CUDA
  wheel, and reported `CpuDevice` — and then generalised to the machine
  without checking the machine.

### ⚠️ The correction: there was a GPU all along

`warp.init()` on this Windows box reports **NVIDIA GeForce RTX 4090 Laptop GPU,
16 GiB, sm_89, CUDA Toolkit 12.9**. NVIDIA's Warp ships CUDA support on Windows
where JAX does not, so `mujoco-warp` reaches the GPU that `mujoco-mjx` could not.
No new hardware was required for any of this.

### ✅ MJX-Warp keeps the drivetrain, and is far tighter than MJX-JAX

| check | MJX-JAX (ADR-0077) | **MJX-Warp** |
|---|---|---|
| `put_model` equalities | 18/18 `mjEQ_TENDON` | **18/18 `mjEQ_TENDON`** |
| agreement with C MuJoCo | 0.029 deg / 400 steps | **0.0001 deg** (2e-6 rad) / 200 steps |
| device | CPU only, on Windows | **RTX 4090** |

✅ **300× tighter agreement**, and the failure this project keeps guarding
against — constraints dropped silently, leaving a robot with no G3 — does
not occur on either backend.

### ✅ Throughput, on the shipped 60-DOF plant

| worlds | steps/s total | × realtime | vs C MuJoCo | per world |
|---|---|---|---|---|
| 1 | 168 | 0.02 | 0.006× | 168 |
| 64 | 10,231 | 1 | 0.3× | 160 |
| 512 | 70,452 | 7 | 2× | 138 |
| 2,048 | 201,037 | 20 | 7× | 98 |
| 8,192 | **262,678** | **26** | **9×** | 32 |

⚠️ **One world is 168 steps/s against C's 29,619** — 176× *slower*. That is
not a defect and it is not the number to quote: a single 60-DOF world cannot fill
a 4090 and kernel-launch overhead dominates. GPU physics is a batch instrument.

✅ **Scaling saturates by ~2,048.** Going 2,048 → 8,192 costs 4× the
worlds for 1.3× the throughput, and per-world efficiency falls 98 → 32.
**2,048 is the efficient operating point**; 8,192 is the ceiling.

✅ **The worlds are really simulating.** At 2,048 and 8,192 every `qpos` is
finite, all worlds agree with each other to **1.2e-6 rad**, and world 0 agrees
with C MuJoCo to **1.6e-6**. Checked because a fast wrong answer is the failure
mode this project has hit twice (ADR-0072's `18× impact`, ADR-0076's mounting).

### What that buys

At 262,678 physics steps/s: **1e8 steps in 6.3 minutes, 1e9 in 63 minutes.**
Against C MuJoCo's 29,619 the same work is 56 minutes and 9.4 hours. Training is
affordable on hardware already on the desk.

⚠️ **Measured with a CONSTANT control**, so nothing round-trips to the host. A
policy in the loop adds observation and action traffic; keeping the policy on the
GPU (Warp interops zero-copy with Torch) is what preserves this. The first
attempt at this benchmark wrote control through `dw.ctrl.numpy()` — a **host
copy** — so it never reached the device: the robot sagged under zero control
and reported a 0.19 rad "divergence" that was entirely the harness.

### Decision

Use **MJX-Warp** as the training backend at **~2,048 worlds**. Keep C MuJoCo as
the reference for the test suite. Do not migrate simulators.

### Consequences

- ✅ **The tooling question is closed.** Isaac/PhysX has tendons (fixed and
  spatial) and is a real alternative, but nothing here needs it: the drivetrain
  is preserved, the agreement is 2e-6 rad, and the throughput is sufficient.
- ⚠️ **Training needs MuJoCo >= 3.12**, which the suite does not pin.
  ✅ **GATE RUN by [ADR-0080](#adr-0080) (M75) and it passes**: every stable
  result is bit-identical, and the three failures were one assertion read after
  the robot had fallen (now removed) plus two of M41's degenerate strict xfails.
- ✅ **A parity guard is now a test.** The suite cannot run MJX or Warp, so it
  asserts the precondition instead: every equality type, transmission,
  integrator, solver and condim the plant uses is on the parity list. An
  unsupported feature is added silently, and this catches it.
- ⚠️ **Throughput is not the remaining blocker; the OBSERVATION SPACE is.**
  ADR-0077's sensor suite and ADR-0078's rate and delay have to become an
  environment wrapper before any of this capacity is usable.

## ADR-0080: The 3.12 gate passes — what it caught was an assertion read after the fall

- **Status:** Accepted — M75
- **✅ Closes the migration gate [ADR-0079](#adr-0079) left open. ⚠️ Corrects
  a fragile assertion in this suite.**
- **Context:** Training needs MuJoCo >= 3.12 (`mujoco-warp`'s floor) and the
  suite pins 3.10. [ADR-0077](#adr-0077) called the two **bit-identical** on a
  400-step rollout; ADR-0079 recorded that the real gate — the whole suite
  under 3.12 — had not been run. It has now.

### ✅ The versions agree where it matters

| | 3.10 | 3.12 |
|---|---|---|
| sway, `kp = 8` (the shipped gain) | 12.94 deg / 2.033 deg / 76.8 N / 17.00 mm | **identical, every digit** |
| sway, `kp = 100` (fallen) | 27.19 / 170.68 / 5574 N / 349 mm | 9.82 / 179.93 / 5876 N / 279 mm |

✅ **Every stable result is bit-identical.** The only divergence is between
two robots that have already fallen over: 170.68 deg against 179.93 deg of tilt
is flat on its back either way, with the cable 25× saturated and the feet
sliding 280–350 mm. Chaotic divergence after a fall is not a regression.

### ⚠️ And it caught this project's own lesson, inside this project's own test

`test_RAISING_THE_SPINE_GAIN_makes_it_FALL_OVER` asserted
`high["track"] > low["track"]` — comparing the spine's **tracking error**
between the standing run and one where the robot is **inverted**. That number is
meaningless post-fall, and the assertion held only because both versions
happened to fall the same way. 3.12 falls differently and it broke.

That is [ADR-0067](#adr-0067)'s finding landing on the suite that recorded it:
*a "better" number is often the robot on its way to the floor.* M61 learned it
about the sway; the test written afterwards still contained an instance.

✅ **The assertion is removed rather than retuned.** What the milestone claims
is already asserted on quantities that survive a fall — tilt (`> 15 deg`),
force (`> 10× the rating`) and slip (`> 5×`). The test now passes on both
versions.

### ⚠️ Two remain, and they are older debt

`test_the_proportional_spine_assist_has_unity_loop_gain_and_is_harmful` and
`test_the_envelope_measures_SURVIVAL_not_recovery` are `xfail(strict=True)` and
flip to **XPASS** under 3.12. Their own recorded reasons say the measurements are
degenerate — *"the envelope went degenerate: 37.17 mm at BOTH 120 and 300
deg"*, *"inverted this finding's DIRECTION"*. They are two of M41's five strict
xfails, already on the roadmap.

⚠️ **Deliberately untouched.** Flipping them to `strict=False` would silence a
marker without doing the re-derivation they are waiting for.

### Decision

3.12 is cleared for training. Keep the suite on 3.10 until M41's xfails are
re-derived, at which point the pin can move.

### Consequences

- ✅ **The training plant and the test plant can be the same plant.**
- ⚠️ **Two tests block a full 3.12 pin.** Neither is a physics defect; both
  need M41's re-derivation.
- ✅ **Final state:** 3.10 gives **508 passed + 5 xfailed**; 3.12 gives
  **492 passed + 3 xfailed + 2 XPASS**, the 16-test difference being
  `build123d`-gated CAD tests the sandbox venv cannot import and that never
  construct an `MjModel`.
- ⚠️ **A gate is worth more than a spot check.** ADR-0077's 400-step
  held-stance rollout said bit-identical and was used to withdraw a migration
  warning. It exercised almost nothing: the sway is where a solver change shows.

## ADR-0081: The environment exists, and it cannot tell the robot where it is

- **Status:** Accepted — M76
- **✅ Builds the training environment [ADR-0077](#adr-0077) and
  [ADR-0078](#adr-0078) supplied the parts for. ⚠️ Names the floating-base
  estimator this project has never had.**
- **Context:** ADR-0077 put the board's sensors on the plant and ADR-0078 added
  the control rate and the transport delay, but both stayed inside the test
  harness. An agent needs them behind a `step()`.

### ✅ `tomcat_kin.env.TomcatEnv`

The observation is assembled from `d.sensordata` and nothing else, so privileged
state cannot be read by accident:

| channel | count | source |
|---|---|---|
| rotor encoder, position + velocity | 18 + 18 | AS5047-class, **14-bit** |
| tendon load cell | **14** | [ADR-0004](#adr-0004), DNP on the ankle |
| IMU quat / gyro / accel | 4 + 3 + 3 | trunk |
| foot contact | 4 | [FR12](REQUIREMENTS.md) |

with **133.3 Hz** control (ADR-0078's NFR12 rate), **7.5 ms** of sensor delay,
and the cascade kept intact: the action is a **tension**, and the rotor servo
closes on its own shaft encoder every physics step rather than at the policy
rate.

### ✅ Joint angle survives the interface

Through the delay, the quantisation and the missing ankle load cell,
`env.joint_estimate` recovers every joint to **0.68 deg** worst. That is
ADR-0077's ~1 deg ankle figure arriving through a real interface instead of a
bench rig, and it is deliberately **not** exact — the missing cell is in
there, because it is in the robot.

### ⚠️ And the validation this milestone planned cannot be run

The plan was to drive the force allocation through the env and confirm it still
stands at 0.01 deg / 72 N ([ADR-0078](#adr-0078)). It cannot:

| the controller needs | the board gives |
|---|---|
| joint angle | ✅ reconstructed, 0.68 deg |
| trunk orientation | ✅ `imu_quat` |
| trunk **position** | ⚠️ **nothing** |
| CoM, CoM velocity, foot positions | ⚠️ all need the above |

`wbc.allocate` and `wbc.desired_wrench` want world-frame quantities. The IMU
gives orientation, not location. **No sensor on this robot measures where it
is.**

⚠️ **That is a real gap, not a wrapper defect.** Legged robots close it with a
**floating-base estimator** — contact-aided IMU integration, or an invariant
EKF — using the fact that a stance foot is stationary. This project has never
had one, and every whole-body result it has published took the base pose from
the simulator.

✅ **It does not block learning.** A policy consumes the observation directly.
It is the hand-written controller that needs the world frame, so this blocks the
*validation route*, not the goal.

### Decision

Ship the environment with a sensor-only observation. Do not synthesise a base
pose from privileged state to make the WBC run — that would hide the gap in
exactly the place it matters.

### Consequences

- ✅ **An agent can be trained against this today.** Observation, action,
  rate, delay and quantisation are all the robot's.
- ⚠️ **The WBC has no path to hardware without a floating-base estimator.**
  It is now the top item, ahead of capstan friction and randomisation.
- ⚠️ **Every whole-body result in this project used a simulator-supplied base
  pose.** Nothing is retracted — they are plant measurements — but none of
  them is a controller that could run on the robot, which is the same
  qualification ADR-0077 made about joint angle.
- ⚠️ **No reward, no randomisation, no episode termination yet.** This is the
  interface, not the task.

## ADR-0082: The standing task — no constant action can do it, and tilt alone cannot judge it

- **Status:** Accepted — M77
- **✅ Gives [ADR-0081](#adr-0081)'s environment a task. ⚠️ Records two
  measured mistakes in defining it.**
- **Context:** ADR-0081 built the interface but not the task: no reward, no
  termination, no reset randomisation. The plan was to validate the reward with
  a known-good open-loop action.

### ⚠️ There is no known-good open-loop action

Three constants, 1.5 s each, from the stance pose:

| action | max tilt | trunk height |
|---|---|---|
| zero | 22.1 deg | 176 → **27.9 mm** |
| uniform 25 N | 66.1 deg | 176 → 40.7 mm |
| gravity-compensating hold | 38.7 deg | 176 → **27.8 mm** |

Even the tension that exactly balances gravity **at the stance pose** collapses.
A fixed tension is a fixed torque, and the torque the pose needs changes as it
tips. ✅ **Standing is an unstable equilibrium and feedback is the task** —
which is why [ADR-0078](#adr-0078)'s force allocation holds 0.01 deg only
because it is closed loop.

### ⚠️ And a tilt-only termination scored a collapsed robot 192

The first version terminated on `tilt > 45 deg`. A zero-action episode ran its
full 200 steps and returned **192.1** while the trunk fell from 176 mm to
**27.9 mm**. The robot had belly-flopped, level, and the criterion never saw it.

✅ **The fix stays observable.** ADR-0081 established the robot cannot know
its height above the *ground*. It can know its height above its own *feet*:
reconstruct the joints from the rotor encoders and load cells, run the leg
forward kinematics, read the paw drop. `env.stance_height` is that, and it needs
no world frame.

| action | terminates | tilt | stance height | caught by |
|---|---|---|---|---|
| zero | 0.12 s | 10.7 deg | **117 mm** | collapse |
| uniform 25 N | 0.08 s | **47.9 deg** | 152 mm | tip |

### The task

* **Action** — 18 cable tensions (N), held for one control period.
* **Reward** — `upright + 0.5*feet_down + tall - 0.05*spin - 0.2*effort`,
  every term from the observation.
* **Termination** — tilt > 45 deg **or** stance height < 120 mm.
* **Reset** — stance pose with optional per-joint jitter.

✅ **Every term is computable on hardware.** A reward that needed `d.qpos`
could not be used to fine-tune on the real robot, and it would hide the gap
ADR-0081 found in exactly the place it matters.

### Consequences

- ✅ **The environment is trainable.** Observation, action, rate, delay,
  quantisation, reward and termination are all the robot's.
- ⚠️ **The reward is unvalidated as a reward.** Its *shape* is tested; whether
  it produces good standing is not knowable without a policy. No open-loop
  reference exists to check it against, which is this ADR's first finding.

  > ⚠️ **STILL UNVALIDATED after a training run — [ADR-0086](#adr-0086)
  > (M81).** 200k PPO steps learned nothing, but not because of the reward: the
  > termination criterion this ADR introduced had two holes of its own, and with
  > them fixed **no known behaviour survives 0.11 s**. The task never produced an
  > episode long enough to reward.
- ⚠️ **Dynamics randomisation is still absent** — only initial-pose jitter.
  Mass, friction, `series_k`, latency and the capstan (ADR-0003's inert
  `wrap_angle`) all want ranges before a policy is trusted off this plant.
- ⚠️ **No baseline to beat.** The WBC cannot run through this interface without
  the floating-base estimator ADR-0081 named, so a trained policy will have
  nothing to be compared against.

## ADR-0083: The extensor side of every pair was never solved

- **Status:** Accepted — M78
- **⚠️ Corrects [ADR-0042](#adr-0042)'s retraction, which covered one cable of
  two. ✅ Makes the capstan wrap a `[solved]` parameter instead of an inert
  one.**
- **Context:** [ADR-0003](#adr-0003) specified capstan friction
  (`T_out = T_in * e^{mu*theta}`) and `params.py` has carried
  `friction_coeff = 0.10` live with `wrap_angle = 0.0` **inert** ever since,
  the comment saying it waits on *"a per-joint-wrap extension"*. M37 then solved
  the paths from station geometry. This is that extension — and solving the
  other side of each pair changes the answer.

### ⚠️ ADR-0042 retracted the penalty on the FLEXOR only

M37's retraction reads: *"the ankle path sums to ~108 deg, not 360, so its
capstan penalty is ~1.21x, not 1.87x. Good news — the motor-side tension
margin this was inflating can come back."* That is `side=+1`.
`route(side=-1)` — the **extensor**, the other cable of the same
antagonistic pair — had never been called anywhere in this project.

| pair | flexor | extensor |
|---|---|---|
| hip | 122.1 deg / 1.237x | 7.9 deg / 1.014x |
| knee | 158.6 deg / 1.319x | 124.7 deg / 1.243x |
| ankle | 107.6 deg / 1.207x | **392.9 deg / 1.985x** |

⚠️ **The ankle extensor is at 1.985x** — essentially the 1.87x ADR-0042
called an over-estimate and handed back as recovered margin. The margin was
recovered on one cable of two.

### ⚠️ And the cause is the defect M37 diagnosed, still present

M37's own words: *"339 deg of wrap on a redirect pulley against the 30-45 deg
LEG_TENDON_SPEC 3.4 assumes ... a routing mistake being read as a physics
result."*

Per station, the ankle extensor is **hip via 198 deg**, knee via 80, ankle
sheave 115. The knee flexor puts **140 deg** on that same hip via. Both are
**redirects**, which 3.4 budgets at **30-45 deg**.

⚠️ `route()` already enumerates the free wrap senses and takes the minimum, so
this is not an unmade sense choice. It is the station **geometry**.

### Decision

Carry the solved wraps as `TendonParams.pair_wrap`, `(flexor, extensor)` per
pair, re-derived from the router by test so they cannot drift. Do **not** apply
them to the torque budgets yet — the routing is a `mechanical/` defect, and
budgeting against a geometry that is about to change would bake it in.

### Consequences

- ✅ **Capstan friction is now expressible**, which is what
  [ADR-0082](#adr-0082) needs before `mu` can be domain-randomised. A parameter
  absent from the model cannot be randomised over.
- ⚠️ **`mechanical/` owes a routing fix**: two redirect stations at 140 and
  198 deg against a 30-45 deg budget. Bearing-mounted sheaves would cut
  effective `mu` to ~0.01-0.03 and collapse the whole term, at a mass cost.
- ⚠️ **ADR-0061's hind-ankle margin is unrecalculated.** It has the joint at
  **1.19x** its continuous rating over a trot on a **frictionless** model, and
  the ankle is the worst-wrap path in the machine.
- ⚠️ **Every tension in this project is still frictionless** — 84 N, 127 N,
  the 222.9 N saturation. This ADR makes the correction computable; it does not
  apply it.
- ⚠️ **Only the hind leg, only the stance pose.** Wrap varies over the ROM and
  the fore leg has its own geometry.

## ADR-0084: Domain randomisation — and a knob that did not turn

- **Status:** Accepted — M79
- **✅ Supplies the randomisation [ADR-0082](#adr-0082) named as missing.
  ⚠️ Repeats, and fixes, [ADR-0063](#adr-0063)'s M59 mistake inside the
  randomiser.**
- **Context:** ADR-0082 shipped a trainable environment with **fixed** dynamics:
  mass, friction, spring and delay were all nominal, so a policy would learn one
  particular robot. Under [ADR-0077](#adr-0077)'s framing — randomisable and
  fast beats accurate — this is the piece that makes the plant a distribution.

### ✅ Four parameters, and every one of them verified to bite

`TomcatEnv.RANGES`, drawn per episode:

| parameter | range | why |
|---|---|---|
| `mass_scale` | 0.85 – 1.15 | build tolerance, unmodelled cabling |
| `floor_mu` | 0.5 – 1.1 | [ADR-0058](#adr-0058) ships 0.8 |
| `series_k_scale` | 0.7 – 1.4 | [ADR-0050](#adr-0050)'s 150-200 kN/m band, widened |
| `latency_s` | 3 – 12 ms | [NFR12](REQUIREMENTS.md) budgets 7.5 |

Each is moved to both ends against one fixed action and the episode has to come
out different:

| parameter | low | high |
|---|---|---|
| `mass_scale` | 51.4 deg | 46.3 deg |
| `floor_mu` | 49.0 deg / 156.3 mm | 48.6 deg / 158.8 mm |
| `series_k_scale` | 10 steps / 150.1 mm | 8 steps / 162.9 mm |
| `latency_s` | 8 steps / 46.2 deg | 10 steps / 51.1 deg |

### ⚠️ And `floor_mu` first came back byte-identical

Setting the floor's friction to **0.5** produced an episode identical to nominal
to the decimal. MuJoCo combines contact friction as the **elementwise maximum**
of the two geoms, and the paw pads ship at **0.8** exactly like the floor — so
`max(0.5, 0.8)` is still 0.8, and lowering the floor could only ever raise
friction, never lower it.

⚠️ That is M59's mistake — friction written where the contact does not read
it — repeated in the randomiser by the same project that recorded it. It was
caught only because every knob was checked for bite rather than assumed.
Fixed by writing both surfaces.

### ⚠️ What is deliberately NOT randomised

`mu_capstan` is absent. [ADR-0083](#adr-0083) found the wraps it would scale
come from a routing `mechanical/` still owes — **198 deg on a redirect**
against a 30-45 deg budget. Randomising around a geometry known to be wrong
buys nothing, and would bake the defect into a policy.

### ✅ The controller is never told what was drawn

`self.k_tors` stays nominal through every draw. The rotor servo on the real
robot does not know which spring it got; updating the plant and the controller's
belief together would train a policy against an error that cancels, which is the
opposite of the point.

### Consequences

- ✅ **The environment is a distribution, not a robot.** Mass, friction,
  spring and delay all move per episode, and none of it is cumulative — each
  reset restores nominal first.
- ⚠️ **Capstan friction is still not covered**, and it is the largest single
  unmodelled effect: 1.24× to 1.99× on motor-side tension. It waits on the
  routing fix.
- ⚠️ **The ranges are engineering judgement, not measurement.** `mass_scale`
  and `series_k_scale` are plausible bands, not surveyed tolerances. The
  hardware has not been built, so there is nothing to survey yet.
- ⚠️ **Still nothing randomises the SENSORS** — encoder bits, IMU noise and
  load-cell scale are all exact. ADR-0077 measured the ankle's missing load cell
  at 1.09-6.08 deg; that error is present but not varied.

## ADR-0085: Sensor randomisation — and a bite test that had to change shape

- **Status:** Accepted — M80
- **✅ Closes the gap [ADR-0084](#adr-0084) left: the sensors were exact.**
- **Context:** ADR-0084 made the **plant** a distribution and recorded that the
  **sensors** were not: encoder, IMU and load cell were all perfect, so a policy
  would learn to trust instruments the robot does not have.

### ✅ Five errors, drawn per episode

| knob | range | what it is |
|---|---|---|
| `enc_offset_rad` | 0 – 0.004 | absolute-encoder mounting / zeroing |
| `enc_noise_rad` | 0 – 0.0008 | ~2 LSB of a 14-bit encoder |
| `load_scale` | 0.90 – 1.10 | load-cell calibration, which [ADR-0004](#adr-0004) still owes |
| `imu_tilt_deg` | 0 – 1.5 | IMU mounting misalignment |
| `gyro_noise` | 0 – 0.02 | rad/s |

### ⚠️ And ADR-0084's bite test would have called every one of them inert

ADR-0084 checked each knob by running an episode and requiring it to differ.
That cannot work here. Sensor error changes the **observation**; with a fixed
action the plant never reads the observation, so the trajectory is
**bit-identical by construction** — asserted, because it is the whole point.

The question has to be asked of the observation and of `env.joint_estimate`:

| knob | joint estimate | IMU tilt | gyro |
|---|---|---|---|
| (clean) | 0.684 deg | 64.962 deg | 0.0289 |
| `enc_offset_rad` | **0.784 deg** | = | = |
| `enc_noise_rad` | **0.668 deg** | = | = |
| `load_scale` | **0.631 deg** | = | = |
| `imu_tilt_deg` | = | **65.415 deg** | = |
| `gyro_noise` | = | = | **0.0323** |

✅ **Nothing leaks.** The three transmission-side errors move the joint
estimate and leave the IMU untouched; the two IMU errors do the reverse. That
separation is the evidence the wiring is right, and it is what the test asserts.

⚠️ **Two of them happen to REDUCE the error** (0.684 → 0.668 and 0.631).
That is one draw landing against the standing bias, not a benefit. The test
asserts only that the value **moves** — asserting a direction would encode a
coincidence, which is the mistake [ADR-0065](#adr-0065) made from a single
phase.

✅ **The encoder offset is a constant, not noise.** It is where the magnet
sits: drawn once per episode, and it does not average away over a rollout, which
is precisely why a policy has to be robust to it.

### Consequences

- ✅ **Both halves of the plant are now distributions** — dynamics
  (ADR-0084) and instruments (here).
- ⚠️ **Sensor error still does not reach the plant.** It only matters through a
  closed loop, so its real effect is unmeasured until a policy exists.
- ⚠️ **The ranges are judgement, not calibration.** `load_scale` at +/-10 % is
  a guess at a front-end ADR-0004 has not specified, and the hardware does not
  exist to measure.
- ⚠️ **The ankle still has no load cell at all**, and that error
  ([ADR-0077](#adr-0077): 1.09 deg rising to 6.08 at the peak rating) is present
  but not varied — it is a missing channel, not a noisy one.
- ⚠️ **Contact sensing is not randomised.** [FR12](REQUIREMENTS.md) wants
  per-foot normal force; the observation reports it exactly.

## ADR-0086: Training ran, learned nothing, and the referee was the reason

- **Status:** Accepted — M81
- **⚠️ Corrects [ADR-0082](#adr-0082)'s termination twice more. ✅ Answers
  what a first training run was for.**
- **Context:** [ADR-0084](#adr-0084) and [ADR-0085](#adr-0085) finished the
  randomisation, so the environment was trainable. ADR-0082 had recorded that
  the **reward was unvalidated as a reward** and that only a policy could
  validate it. This is that run.

### ⚠️ 200k steps, and nothing moved

`stable-baselines3` PPO, 8 parallel envs, all nine ranges on. Chosen over a
hand-rolled PPO deliberately: a homemade one that failed to learn would leave
*"bad reward"* and *"bad PPO"* indistinguishable, which is the entire question.

**`ep_len_mean` sat at 21 steps and never moved** across the last 40k. 588
steps/s, 5.7 minutes.

⚠️ **And the log was truncated to the last 20 %** by a `tail -30` in the
harness, so whether anything improved early is now unknowable. A learning curve
is the one artefact a training run exists to produce.

### ⚠️ The referee had two more holes

ADR-0082 caught a tilt-only termination scoring a collapsed robot 192 and added
`stance_height`. That fix was never itself checked, and it had two holes:

1. **The leg frame is not the world.** `LegModel.forward` returns the paw in the
   leg's own sagittal frame. Splay the legs and lie down and the number
   *grows*: under the gravity-compensating hold the trunk sank to **27.8 mm**
   while `stance_height` read **264 mm and rising**.
2. **`max` let one leg vouch for the robot.** With the IMU projection added, the
   hind pair folded to **32 mm** with the rear on the floor while the fore pair
   stretched to **250 mm**, and `max` returned 250 and passed it.

| action | before | after |
|---|---|---|
| zero | 15 steps | 15 |
| uniform 25 N | 7 | 7 |
| **gravity hold** | **533 (full episode)** | **10** |
| random | 2 | 2 |

✅ The corrected figure agrees with what ADR-0082 measured on that same action
all along — it tips to 38.7 deg and collapses. **The referee was wrong, not
the measurement.**

### ⚠️ And with the referee fixed, the diagnosis inverts

Nothing this project knows how to do survives a tenth of a second:

| action | steps | seconds |
|---|---|---|
| random | 2 | 0.015 |
| uniform 25 N | 7 | 0.05 |
| gravity hold | 10 | 0.075 |
| zero | 15 | 0.11 |

An episode ending in 2 steps carries almost no gradient toward standing, so PPO
had nothing to climb. ⚠️ **This is a task-design problem, not a reward
problem** — and it is ADR-0082's own finding from the other side: no constant
action stands, because standing is an unstable equilibrium.

⚠️ **The action scale compounds it.** `action = 1` maps to the full 222.9 N
rating while the gravity hold needs **14-145 N**, so most of the action space is
past anything usable and a random draw saturates the cable.

### Decision

Fix the referee; do not touch the reward. The reward remains **unvalidated**:
this run could not test it, because the task never produced an episode long
enough to reward.

### Consequences

- ✅ **Three corrections to one criterion**, each closing a real hole and
  exposing the next. The lesson is not any single fix — it is that ADR-0082
  patched tilt and then never checked the patch.
- ⚠️ **The tilt term is now nearly redundant.** The corrected `stance_height`
  fires first on every case tested, including the 25 N tip it used to miss.
  Kept as a backstop because these failure modes are not an enumerated set.
- ⚠️ **The task needs redesign before training means anything.** Candidates:
  scale the action to the usable band; start episodes from a supported pose; or
  make the policy a **residual** on a gravity-compensating baseline, which is
  standard for legged RL and matches ADR-0082's finding exactly.
- ⚠️ **Keep the whole training log.** Truncating it cost the early curve, which
  was the one thing that would have separated "never learned" from "learned then
  plateaued".

## ADR-0087: The capsule model overstates leg swing inertia by a third

- **Status:** Accepted — M85
- **⚠️ CORRECTED by [ADR-0088](#adr-0088) (M86): the figure below is
  **1.244e-3 with bearing ENVELOPES**; with catalogue bearings the leg is
  **1.175e-3**, so the capsule model was **+45 %**, not +37 %. The "masses agree
  to 8 %" line below is not a check that passed — the 8 % is the error. The
  gap is now CLOSED in the MJCF.**
- **⚠️ Qualifies every result that depends on leg swing.
  ✅ Answers the question [ADR-0043](#adr-0043) and `mass_closure.py` both
  left open.**
- **Context:** `mjcf_tendon` draws each link as a capsule and lets MuJoCo derive
  the inertia at uniform density. `mass_closure.py` says what that costs in its
  own comment: *"`inertia_ratio` is used as a first-order proxy ... the real
  quantity is `Lambda = (J M^-1 J^T)^-1` ... which needs the per-link inertia
  tensors this ..."*. `tomcat_leg_detail.py` has placed every part since M41, so
  the tensors were always computable.

### ⚠️ Measured about the hip, same leg, same stance pose

| | mass | I_yy about hip |
|---|---|---|
| CAD placed parts (18 solids) | 182.2 g | **1.244e-3 kg m^2** |
| MJCF capsules | 168.2 g | **1.701e-3** |
| | +8 % | **MJCF +37 %** |

The masses agree to 8 %, so this is not a mass error — it is **where the
mass sits**.

### ⚠️ And the cause is the opposite of the intuition

| part | mass | z from hip |
|---|---|---|
| bearings | 63.8 g | -41 mm |
| sheaves | 44.1 g | -42 mm |
| clevises | 40.5 g | -50 mm |
| **joint hardware** | **148.4 g — 81 %** | **at the joints** |
| CF tube | 9.0 g | -79 mm |
| bonded inserts | 13.2 g | -91 mm |
| paw pad | 5.7 g | -173 mm |

**Four fifths of the leg is joint hardware sitting at the joints** — that is,
near the axis it swings about — and the carbon tube between them is **9 g**.
A uniform-density capsule spreads the same mass along the link and so places it
further out.

⚠️ **The prediction going in was the reverse**: mass at the ends, therefore
higher inertia. The hardware is at the *joints*, not the ends, and the hip is
the reference.

### ✅ How it was measured, after three failures

Three attempts tried to reproduce `per_link_mass()`'s part-to-link rule so each
link could carry its own tensor. They came out **+105 %**, **-79 %** and
**+20 %** wrong, because ASSEMBLY_SPEC 2 is not a proximity rule: a joint sits
*between* two links and its hardware belongs to the **distal** one, which no
geometric heuristic recovers.

**The question did not need that rule.** Whole-leg inertia about the hip depends
only on where the mass physically is, not on which link it is charged to. Asking
it that way answered it in one measurement and is immune to the error that had
blocked three.

⚠️ Two harness bugs were caught before the result was reported, both by
printing the centre of mass and the pose beside the ratio rather than the ratio
alone: `d.xanchor` indexed by **dof address** instead of joint id, which put the
hip 167 mm away and manufactured a fake **5×** gap; and `FOOT_X` read as mm
when it is metres.

### Consequences

- ⚠️ **Every result that depends on leg swing is affected.** Swing inertia is
  the P1 metric; [ADR-0043](#adr-0043) moved it **+62 %** by redistributing link
  mass alone, and this is a further third in the other direction.
- ⚠️ **The tendon drive's central argument is understated.** Motors on the body
  so the leg stays light — the real leg swings **a third easier** than every
  simulation has assumed.
- ⚠️ **Not yet fixed in the MJCF.** This measures the gap; it does not close
  it. Closing it needs per-link tensors, and the honest route is to take
  `per_link_mass()`'s masses as authoritative rather than re-derive the
  apportionment that failed three times.
- ⚠️ **The girdles and spine are unmeasured** — this is the leg only.

## ADR-0088: The MJCF carries measured per-link inertia, and two numbers it corrects

- **Status:** Accepted — M86
- **✅ Closes [ADR-0087](#adr-0087)'s gap in the plant.**
  **⚠️ Corrects ADR-0087's published figure and [ADR-0073](#adr-0073)'s paw-pad
  claim.**
- **Context:** ADR-0087 measured the gap and said closing it "needs per-link
  tensors, and the honest route is to take `per_link_mass()`'s masses as
  authoritative rather than re-derive the apportionment that failed three
  times." That is what this does.

### ✅ The apportionment was not hard, it was ill-posed

Three attempts assigned CAD solids to a **link** by proximity: +105 %, -79 %,
+20 % wrong. A joint sits physically *between* two links, so neither is
"nearest" in any stable way.

ASSEMBLY_SPEC ²2 never assigns hardware to a link by position. It assigns it
to a **joint**, then to that joint's **distal** link. Joints are isolated points
70-100 mm apart, so *which joint* is well-posed. `link_inertia.assign()` applies
exactly that, and `test_sheave_mass_lands_on_the_distal_link` pins it at
`rel=1e-9` — both sides measure the same solids, so there is no tolerance to
hide in.

### ⚠️ ADR-0087's 1.244e-3 was 6 % high, and its agreement check was the error

`bearing()` says **"envelope"** in its own docstring: it draws a solid
Ø19×6 steel annulus where the catalogue bearing is 8.0 g. Its volume weighs
**12.0 g — 50 % over**, and 15.9 g over the leg.

ADR-0087 reported *"the masses agree to 8 %, so this is not a mass error"*. The
8 % **was** that error. Same class of mistake as reading a girdle's fit box as
a mass model — **a drawing made to prove clearance, read as if it were a
part.**

| | mass | I_yy about hip |
|---|---|---|
| CAD, bearing envelopes — *ADR-0087 as published* | 182.2 g | 1.244e-3 kg m² |
| CAD, catalogue bearings | 167.2 g | **1.175e-3** |
| MJCF capsules, before M86 | 168.2 g | 1.701e-3 — **+45 %**, not +37 % |
| MJCF `<inertial>`, now | 167.2 g | **1.175e-3** — ratio **0.9999** |

The last row is measured **in MuJoCo**, not derived: build the plant, pose the
hind leg, sum the four bodies about `xanchor`. A wrong frame conversion would
not survive it.

### ✅ Where the mass actually sits, and a —— TBD that had an answer

| | was | measured |
|---|---|---|
| `link_com_frac` | 0.45 / 0.45 / 0.50 / 0.50 —— TBD | **0.065 / 0.072 / 0.076 / 0.874** |

The old value was justified as *"muscle bellies sit proximally"*, a hand-waved
45 %. The femur's centre of mass is **5.9 mm** from the hip on a **90 mm**
link. It is not a belly part-way down a bone — it is joint hardware sitting
**on** the joint. The paw is the mirror image (87 %) because the pad is its tip.

`link_mass` moved too, total unchanged at 0.1672 kg: `per_link_mass()` divides
the clevis mass equally by three, but the ankle clevis carries a Ø10 bearing
against the hip's Ø19 and is the smaller part — 7.6 g placed against 13.5 g
by thirds.

### ⚠️ A bug ADR-0087's number was structurally unable to catch

`build()` moves every solid to the limb plane on its last pass and leaves
`report`'s joint coordinates behind, so joints and solids sat **48 mm apart in
y**. `I_yy = sum m(x² + z²)` contains no y, so the published figure agreed
to four figures with the hip out of plane, while `I_xx` and `I_zz` were both
wrong. Found by printing the centre of mass beside the ratio.

### ⚠️ ADR-0073's paw pads are double counted

ADR-0073 explains a 4 g residual as *"the four paw pads, which `link_mass` does
not carry"*. It does: `per_link_mass()` adds the 5.68 g pad to the paw on its
own line. The 1 g pad geom was 1 g of double count per leg. Moot now — an
explicit `<inertial>` makes MuJoCo ignore geom mass — but the geom no longer
claims it either.

### ⚠️ What it re-baselined: 23 published results, and four that did not survive

Correcting the inertia broke **36 tests**. Every one was re-measured rather than
re-fitted; the pattern is that **anything resting on the leg being hard to swing
got weaker, and anything resting on it being heavy to hold did not move.**

| result | published | measured | |
|---|---|---|---|
| trot power / runtime | 7.36 W / 18.85 min | **7.16 W / 19.39 min** | cheaper |
| righting, rigid spine | 2.14 s | **1.74 s** | faster |
| righting, all-18 spooled | 10.10 s | **7.15 s** | ratio 4.7× → 4.1× |
| lowest drivetrain mode | 54.9 / 27.4 Hz | **69.6 / 35.8 Hz** | ratio 0.51, unchanged |
| spine command in righting | 5003 N, 22× | **6778 N, 30×** | worse |
| landing contact, all-18 | 4.8 / 1.0 / 11.0 kN | **0.76 / 0.89 / 2.93 kN** | 4× lower |
| NFR12 at 7.5 ms | 0.01° / 74 N | **0.01° / 76.7 N** | still MET |
| swing-leg roll moment | 0.0736 N.m | **0.0508 N.m** | **-31 %** |

#### ✅ Three of M41's five `xfail(strict=True)` came back on their own

M41 apportioned leg mass from a manufacturing **model**; the survival envelope
went degenerate (37.17 mm at both 120° and 300°) and four tests were
marked rather than retuned. With the **measured** tensors three of them pass
unaided, including [ADR-0029](#adr-0029)'s *"the proportional spine assist is
harmful"*, whose **direction** M41 had inverted. **The instrument was wrong, not
the conclusions.**

#### ⚠️ Four findings did not survive, and they are named

- **[ADR-0046](#adr-0046)/M59's ranking inversion is undone.** M43 measured the
  worst standing tendon as the hind hip extensor at ~2.5× continuous; M59's
  point-foot fix moved it to the fore knee flexor at 100.5 N. On the measured
  inertia the **hind hip extensor binds again at 164.8 N rms, 2.03×**, peak on
  the 222.9 N ceiling. The inversion was the capsule surplus, not the foot.
- **M44's "clipping loses the leg" (197° of hip drift) does not reproduce**
  — it is **17.3°**. The ratio against the proper allocator survives at
  ~1700×, and that is what the firmware note rests on.
- **M61's "raise the spine gain and it goes over" does not reproduce.** At spine
  `kp` 30 the tilt was **179.91°**; it is **11.94°**. The conclusion
  holds for a different reason: those gains command **288 N, 1.3× the motor's
  PEAK**, and get the clamp.
- **M55's spring reference no longer keeps the worst tendon off the ceiling.**
  It dropped 222.9 → 207.4 N on the capsule plant; the hind hip extensor is
  back at 222.9. The ankle mechanism is intact; the whole-robot headline is not.

#### ⚠️ And one guard flipped from safe to unsafe

[M17](#adr-0014) measured the rigid-body divergence **2 % SLOWER** than the LIPM
`omega = sqrt(g/z)` every envelope in `control.py` is sized on, and concluded the
reduced-order model was conservative. That held only for the capsule mass
distribution. Measured, the robot diverges **6.8 % FASTER**: less inertia far
from the pivot means less resisting the topple. **Every envelope in `control.py`
is optimistic by that much**, and re-sizing them is owed.

#### ⚠️ The rotor inertia the capsules were standing in for

The rigid plant's leg joints carry `armature = 0`. Through a spool of radius
`SPOOL_R` driving a sheave of radius `r`, the rotor appears at the joint as
`I_rotor (r/R)²`: **2.05e-4 at the hip (17 % of the whole leg's swing
inertia)** and **5.12e-5 at the ankle (72 % of the metatarsus and paw about it)**.
Removing the capsules' surplus without adding this made the plant *less* physical
in one respect — the lowest oscillatory mode stopped oscillating and that
metric jumped 12.6 → 1365 Hz, a contact mode.

`leg_tendon_xml(rotor_armature=True)` computes it and is **default OFF**, and
neither reason is that zero is right. `ROTOR_ARMATURE` is `[assumed]` — the
vendor does not publish it — so switching it on trades one set of re-baselined
results for another; and most findings here compare the rigid plant against the
spooled one, where adding it to the rigid side alone moves the comparison rather
than a number. It is a milestone of its own. **This one has a single cause.**

### Consequences

- ⚠️ **Every result that depends on leg swing and predates M86 is understated
  by 45 %.** Swing inertia is the P1 metric.
- ⚠️ **[ADR-0089](#adr-0089) (M87) found the other half.** Correcting the legs
  alone inverted M17's LIPM guard; the girdles restored it. A mass model's
  errors can be cancelling, and half a correction was worse than none.
- ⚠️ **The girdles are measured and NOT yet fixed.** The MJCF girdle box is
  60×60×56 mm; six motors and their spools are **212,133 mm² against its
  201,600** — **the box is smaller than the parts it houses (105 %)**, where
  `tomcat_packaging` sizes the real one at 82×86.5×108.2 mm, **3.8×** the
  volume. At equal mass the motor cluster's inertia is **1.34-1.56×** the
  box's. So the trunk is too *easy* to rotate while the legs were too *hard* to
  swing.
- ⚠️ **Still no skin.** 7 contact geoms: floor, two girdles, four pads.

## ADR-0089: The girdle box could not hold its own motors, and three findings rested on it

- **Status:** Accepted — M87, **two conclusions reversed by
  [ADR-0092](#adr-0092)**
- **⚠️ NARROWS [ADR-0073](#adr-0073)'s landing alarm into a requirement on the
  spine loop, WITHDRAWS [ADR-0075](#adr-0075)'s compliance penalty and M29's
  NFR15 argument. ✅ Restores M17's LIPM guard
  that [ADR-0088](#adr-0088) had just inverted.**
- **⚠️ CORRECTION (M92, [ADR-0092](#adr-0092)).** Two of those did not survive
  the next correction to the same quantity:
  - ~~M29's NFR15 argument is withdrawn~~ — **restored**. The 67 cm/s envelope
    read 48.1 (M29), 47.5 (here), **48.440** (M92, once the vertebral chain moved
    off the belly). The reasoning below is right — M29's margin was inside the
    error — and the error was larger than this ADR knew. The margin is still
    only 0.9 %.
  - ~~ADR-0073's alarm requires a stiff spine loop~~ — **narrower still**. The
    leg cable saturated at spine gain 0 and 50 here, clearing only at 300; on the
    dorsal axis gain 50 gives 64.9 N against the 222.9 rating. The requirement is
    only that the spine not be **limp**.
  - What stands unchanged: the girdle box could not hold its own motors, and
    ADR-0075's compliance penalty stays withdrawn.
- **Context:** ADR-0088 measured the legs and left the girdles as the other half.
  They are **2.02 kg of a 4.30 kg robot** — 47 % of it — drawn as a
  60×60×56 mm box.

### ⚠️ The box was smaller than the parts inside it

| | volume |
|---|---|
| MJCF girdle box, 60×60×56 mm | 201,600 mm³ |
| six GIM3505-9 with their spools | **212,133 mm³** |
| | **105 %** |

No arrangement of cylinders reaches 105 % packing. `tomcat_packaging.girdle_box()`
sizes the housing from the motors' own bounding box plus 5 mm of clearance and
gets **82 × 86.5 × 108.2 mm**, **3.8×** the volume at **28 %** packing. The
bank stacks upward, so the housing sits **23.1 mm above** the hip axis rather
than centred on it: 31 mm below, 77 above. Belly clearance barely moves
(148 → 145 mm); the back and the flanks are what grow.

⚠️ **[ADR-0084](#adr-0084) already had this box in its hands.** It corrected the
"denser than tungsten" claim by finding the right denominator and reported
4,474 and 5,565 kg/m³ as *"what a box packed with motors should look like:
six per girdle ... is 202.5 cm³ — the whole box"*. **The whole box** was the
finding, printed and read as reassurance.

### ⚠️ Three published findings were artefacts of it

| finding | rested on | measured with the real girdle |
|---|---|---|
| [ADR-0073](#adr-0073): the cable margin was bought by a **rigid trunk** | the leg cable saturates at **every** spine hold gain | saturates at gain 0 and 50, **64.7 N at the shipped 300** |
| [ADR-0075](#adr-0075): spine compliance costs the righting **4.7×** | 10.10 s against 2.14 | **2.35 s against 2.28 — 3 %** |
| M29: ADR-0020's slowdown is **not required by NFR15** | 48.1 mm against a 48 mm requirement | **47.5 mm — 1 % short** |

A girdle with the inertia its motors actually have is not whipped around by the
spine, so the leg loops are not chasing a violently moving hip and the series
spring has almost nothing left to absorb. ⚠️ **The mechanism ADR-0075 named
— saturation — is still there; what has gone is its cost.** The spine's own
cables still run **1.1-1.7 kN against a 222.9 N rating**, and that was always the
larger number.

⚠️ **ADR-0073's alarm does not go away, it gets a condition.** Its own gain
sweep — *"the leg cable saturates at every spine hold gain, so this is the
articulation, not the way the spine is held"* — was the evidence, and it was
read backwards: the girdle box was small enough to swamp the gain, so the sweep
looked flat. On the real girdle the gains separate:

| spine hold gain, 0.05 m drop | leg cable |
|---|---|
| 0, limp | **222.9 N, saturated** |
| 50, softly held | **222.9 N, saturated** |
| **300, the shipped loop** | **64.7 N** |

So **holding the spine is what protects the leg cable**, and a margin conditional
on a loop gain is weaker than a structural one — particularly one
[ADR-0067](#adr-0067) shows there is no room to raise.

✅ **The landing contact became believable on the way.** 4831 / 986 / 11,040 N
— non-monotonic, 260× body weight — became **437-535 N, 10-13×**, which
is the 9-13× ADR-0073 expects an impact of this kind to deliver.

### ✅ Correcting HALF a mass model was worse than correcting neither

| plant | LIPM omega | rigid-body rate | ratio |
|---|---|---|---|
| capsule legs, 60 mm box | — | ~2 % slower | **0.98** |
| **measured legs, 60 mm box (M86)** | 7.743 | 8.266 | **1.068** |
| measured legs, measured girdles (M87) | 7.562 | 7.169 | **0.948** |

M17 measured the rigid body toppling ~2 % SLOWER than the LIPM every envelope in
`control.py` is sized on, and called the reduced-order model conservative.
ADR-0088 replaced the legs' capsule inertia with measured tensors — a strict
improvement — and the guard **inverted**: the robot diverged 6.8 % FASTER than
the model. Finishing the other half restores the margin **wider than M17's**.

⚠️ **The lesson is not "measure more", it is that a mass model's errors can be
cancelling.** ADR-0088's leg correction was right and made one published guard
wrong, for one milestone, and nothing but the girdle work would have found it.

### ⚠️ The same defect, twice more

[ADR-0088](#adr-0088) found `mjcf.py` hard-coding the paw's centre of mass at
`0.5 * l4` while `mass.py` read `link_com_frac[3]`; they agreed only because the
parameter happened to BE 0.50. Two more of exactly that:

- `mjcf.py`'s `_girdle_block` wrote `pos="0 0 0"` while `spine.girdle_com` read
  `front/rear_girdle_com`;
- `spine.center_of_mass_y` rotated the fore LEGS' fore-aft offset into y under
  yaw — M20's own correction — but not the **girdle's**, which had none to
  rotate.

Both stayed silent for exactly as long as the parameter kept its `(0, 0)`
placeholder. **A hard-coded copy of a parameter is not a duplicate, it is a
second source of truth**, and giving the parameter a real value is what finds it.

### ⚠️ And the survival criterion is not an envelope

Probing the balance harness across disturbance magnitude at 300°:

| push (mm) | 8 | 12 | 14 | 16 | 18 | 20 | 22 | 24 |
|---|---|---|---|---|---|---|---|---|
| fell | no | **yes** | no | **yes** | no | no | no | yes |
| settled (mm) | 45.6 | 122.6 | 39.8 | 117.2 | 12.6 | 9.6 | 22.9 | 107.7 |

A smaller push fells the robot where a larger one does not, twice. [ADR-0040](#adr-0040)
argued survival was the wrong quantity; this shows it **does not even order**, so
no threshold fitted to it means anything. And "survived" is not recovered: the
smallest push that survives settles **45.6 mm** off support, 18× the noise
floor. `test_the_SURVIVAL_criterion_is_NOT_AN_ENVELOPE` asserts the
non-monotonicity directly, replacing a test that probed one moving corner.

### Consequences

- ✅ **The plant's contact shell is right for the first time.** 82×86.5×108.2
  mm at 28 % packing, with the motors' measured inertia rather than a uniform
  box's (which would be **2.9×** too high).
- ⚠️ **Every balance result moved**, and the lateral stability margin with them:
  5.0 → **4.58 mm**. The walk is still stable; the headroom is under 5 mm.
- ⚠️ **NFR15 is a reason to slow down again.** The 67 cm/s trot misses by 1 %.
- ⚠️ **Still no skin, and no tail.** 7 contact geoms.
- ⚠️ **The head is still not placed.** The front girdle absorbs 240 g of head and
  neck and `front_girdle_com` lumps it in the housing; putting it forward would
  raise the pitch inertia. `[owed]`

## ADR-0091: a union that shrank, and the guard that now forbids it

- **Status:** Accepted
- **Date:** 2026-09-08 (M91)

### Context

The wiring fix passed every check the assembly had. Reading the volume table
afterwards, the rear girdle -- `body 0`, 105 mm long, carrying six motors and
both hind hips -- reported **2.9 cm3**. Its own shell alone is 47.0.

Fusing the parts one at a time located it exactly:

| step | volume mm3 | solids |
|---|---|---|
| shell + 4 bulkheads + 4 hip bosses | 110,803.9 | 1 |
| + joint parts 0..3 | 114,889.3 | 1 |
| **+ joint part 4** | **0.0** | **0** |
| + joint parts 5..8 | 2,859.1 | 1 |

Part 4 is the rear joint's ventral process post: a 6 mm cylinder grazing the
1.2 mm lofted wall. OCC raised nothing. The later parts re-fused into exactly
one small solid, so `report()`'s guard -- *"all bodies are one part each"* --
was **true of a body that no longer existed**, and the trunk's published
structure mass was computed from the wreckage.

⚠️ `intersect` is wrong on the same pair in the OTHER direction: it answers
**169.6 mm3, the post's whole volume**, where point sampling puts 9 % of it in
the wall. Two calls, two wrong answers, no error from either.

### Options

1. Move the post clear of the wall -- but its position IS the spine cable's
   moment arm, from `SpineParams`. Moving it changes the kinematics.
2. Fuse in a different order, or with a tolerance. Both hide the failure rather
   than detect it, and the next tangency finds it again.
3. Remove the tangency, and check the invariant a union cannot break.

### Decision

**Three.** A union cannot shrink, so `_fuse` measures before and after and
raises on a decrease -- not a warning, a stop. And the clearance a spine cable
needs to pass between the post and the body is cut BEFORE the post is fused, so
the post arrives in a void instead of tangent to a wall. That clearance is a
design requirement first and a boolean fix second.

`tests/test_trunk.py` holds five: the minimal reproduction on the shell alone,
a stubbed boolean proving the guard actually raises, *every body is at least its
own shell*, the `intersect` discrepancy pinned so a fixed OCP announces itself,
and the mass against the budget.

### Consequences

- ✅ **The rear girdle exists again: 2.9 -> 117.0 cm3**, one solid, and no
  fuse in the trunk shrinks.
- ⚠️ **The structure mass was wrong, and so was the budget it was judged
  against.** "221 g against a 200 g budget" was prose -- the 221 came off the
  annihilated body and the 200 was recalled, not derived. Both are computed now:
  **357 g against 1024 g** (`trunk_mass` 3635 - 2371 motors - 240 head/neck),
  leaving **667 g** for the battery, electronics, cable and spine hardware that
  are not drawn. The structure is comfortably inside; the earlier alarm was mine.
- ⚠️ **The hind legs' clean interference result was checked, not assumed.**
  `intersect` had just been caught lying, so leg-vs-trunk overlap was re-measured
  by point sampling: fore 2-4 of 900 points, hind **0 of 600**. The 83.9 mm3 and
  0.0 mm3 stand.
- ✅ **Two thirds of a standing `[owed]` was the same defect.** The trunk
  carried *"3 valid faces OCC will not mesh, which also breaks STL export"*.
  Two of the three were on body 0. With the body restored it is **1 of 656**,
  on body 3. The debt was smaller than it was written down as.
- ⚠️ **A solid count is not a volume check**, and this is the second time
  (ADR-0089 was the coplanar collapse from 37,293 to 509 mm3). The lesson is the
  same one this project keeps paying for: *a number has to be asked what it
  measures*. Here it was asked of `intersect` and `+` themselves.

---

## ADR-0092: the vertebral column was running along the belly

- **Status:** Accepted — M92, **the balance-harness diagnosis reversed by
  [ADR-0093](#adr-0093)**
- **Date:** 2026-09-08 (M92)
- **⚠️ CORRECTION (M93).** ~~The harness has no robust operating point at the
  corrected CoM~~ — it was the TRACK, not the CoM height. The legs sat 5 mm
  inside the front girdle's flank because `TRACK_Y` never followed M88's wider
  trunk; with it corrected the baseline reads **0.69 mm mean / 3.08 peak at the
  shipped gain** and five of the seven tests marked here pass untouched. Two
  things were wrong at once and this ADR corrected one, then read the wreckage
  as a property of the plant. Everything else below stands.

### Context

`tomcat_trunk.report()` printed *"dorsal line flat at z = 79.8"*. It was
printing `Z_DORSAL`, the parameter, not the shape. Measured, the back **notches
52.6 mm at each of three joints** -- 42 % of the 126 mm chest depth -- because
the spine joint axis sits at z = 0, the hip axis, and each neck has to blend the
trunk section down to the underside of the body to reach it. The trunk was
shaped to a requirement, a flat back over a tucked belly, that it never met.

⚠️ This is the standing `[owed]` -- *"the spine axis is ventral, not dorsal, and
moving it is a kinematics change"* -- arriving as a visible defect.

### Options

1. Keep the axis and hang the joints on pylons under a flat back. CAD-only, no
   re-baselining -- and the joint ends up on a stalk, loaded in bending.
2. Move the axis to where a cat's is, and re-baseline the dynamics.

### Decision

**Two.** `SpineParams.spine_axis_z = 0.0498 m`, and it is DERIVED, not chosen:
in a cat the back you feel is the row of spinous process tips with skin over
them, and the dorsal cable's moment arm is what sets how far a process stands
off the axis. One moment arm below the dorsal line puts the tips exactly on it:

    z = Z_DORSAL - joint_moment_arm = 79.8 - 30.0 = 49.8 mm

Four places read that one parameter -- the CAD, `mjcf.py`, `mjcf_tendon.py` and
`spine.py`'s analytic chain. The analytic model and MuJoCo agree to **0.0000 mm
under bend**, which is the check that would have caught it had only one moved.

### Consequences

- ✅ **The back is flat where it must be.** The notch falls **52.6 -> 16.0 mm**
  (42 % -> 13 % of chest depth) and every dorsal process tip lands on 79.8, so
  the skin has something to lie on at each joint. The assembly is untouched:
  cables 0.000 mm off their spools, 12 of 12 spools inside their bodies, four
  feet on one plane.
- ⚠️ **Raising the axis does NOT fix where the mass is.** Against the CAD's
  measured trunk CoM of 21.1 mm the model reads **9.3 mm before and 31.4 after**
  -- the sign flips, the magnitude does not. The cause is that the mass model
  still describes the two-girdle architecture M88 replaced. `[owed]`
- ✅ **NFR15 clears again, and ADR-0089's withdrawal is itself withdrawn.**
  omega falls 7.5985 -> 7.2132, divergence slows, and the 67 cm/s envelope reads
  **48.440 mm** against 47.510. ADR-0089's *reasoning* was right -- M29's margin
  was inside the error -- and the error was bigger than it knew. ⚠️ The margin
  is 0.9 % and has now crossed 48 in both directions on corrections to the same
  quantity, so it is not settled; what is settled is that neither 48.1 nor 47.5
  was ever evidence.
- ⚠️ **The sagittal arch's fore-aft authority was an artefact.** Arching moved
  the CoM 11.2 mm rearward on the belly-mounted spine and **0.7 mm forward** on
  the dorsal one: the girdles hang below the chain, so a bend swings their mounts
  forward and cancels the pull. The LIFT survives and grows, 42.1 -> 47.0 mm. The
  arch is a vertical actuator. The lateral sway is untouched.
- ⚠️ **ADR-0073's landing requirement shrank.** The leg cable used to saturate
  at spine gain 0 AND 50, clearing only at the shipped 300 -- which is what
  turned the alarm into a requirement for a stiff spine loop. Now gain 50 gives
  **64.9 N** against the 222.9 rating. The requirement is only that the spine not
  be LIMP.
- ⚠️ **The righting got slower and the gap wider.** 2.28 -> 2.59 s, and the
  precessing bend's roll rate **-52.7 -> -30.2 deg/s**: a factor of five short
  became a factor of nine. The 25/15 deg command also became illegal, driving
  the lateral joints to 17.6 deg against a +-15 ROM; 15/15 is both legal and
  faster and characterises the plant now.
- ⚠️ **The balance harness has no robust operating point, and seven tests are
  marked rather than retuned.** The undisturbed baseline goes **1.92 -> 7.32 mm
  mean, 8.25 -> 89.59 mm peak** -- an 89 mm excursion against a 30 mm
  disturbance, which is the M17 failure `test_mjsim` exists to prevent. It is not
  the stale apportionment: at the CAD-measured CoM height it still reads
  5.60 / 55.17. And it is not a knob -- stiffness and damping sweeps give
  ISLANDS (kp 130-155 passes, 120 and 160 do not; kv 8 passes, 6 and 12 do not).
  Choosing kp = 140 would make all seven pass and would be the M35 mistake.
  The balance loop is owed a design pass at the corrected CoM height. `[owed]`
- ⚠️ **The quiet baseline was always bought by a CoM 11.8 mm too low.** That is
  the uncomfortable part: `test_mjsim`'s gate, the thing every closed-loop result
  in this project rests on, held because the trunk's mass was in the wrong place.

---

## ADR-0093: the leg, and the 5 mm that was hiding under a volume threshold

- **Status:** Accepted
- **Date:** 2026-09-20 (M93)
- **⚠️ REVERSES [ADR-0092](#adr-0092)'s diagnosis of the balance harness.**

### Context

*"There are parts of the leg design that are not connected -- how can this be
called done?"* It could not. I had measured 9.5 % of every cable inside sheave
material and then written a check over four groups that left `sheave` out; it
printed 0.0 % and passed. Asked instead what each part attaches to, the leg was
**five pieces**: the paw and its pad 5.04 mm clear of the metatarsus, both via
pulleys floating with no shaft reaching them, and the return spring anchored to
nothing.

### Decision

Redesign rather than patch, and derive the routing instead of adjusting it.

A 2-groove band is 6.81 mm and 2 deg of fleet over the longest bone allows 3.3,
so **no layout where a run crosses bands can work**. With ADR-0008's continuous
cable a joint needs one groove per CABLE, which leaves a **monotone stack**: one
plane per cable, hip innermost, ankle outermost, fleet zero by construction. All
six orders were priced; hip-knee-ankle costs 0.7 g of tube against 5.86 deg.

Tubes sized to SF 2.5 at the offsets the routing actually produces: **Ø14 /
Ø12 / Ø12**, against §3.5's Ø12/Ø10/Ø8 which measured 1.83 / 1.94 / 1.75.

### Consequences

- ✅ Cable inside part material **22 % -> 0.0** (all rigid groups), fleet
  **5.86 -> 0.00 deg**, concentric pulleys sharing space **787.9 -> 0 mm3**,
  connected pieces **5 -> 1**, SF **2.62 / 2.78 / 3.03**, mass unassigned to any
  link **14.67 -> 0.00 g**.
- ⚠️ **An 80 mm3 leg/trunk overlap had passed a 1500 mm3 threshold for four
  milestones.** As a DEPTH it is the femur **5.0 mm inside the front girdle's
  flank**: the girdle's half-width is 45.0 and `TRACK_Y` was 48, set before M88
  widened the chest 83.4 -> 90.0. A volume threshold cannot see a shallow, wide
  interference. Track **96 -> 106 mm**, derived as 45.0 + 1.0 + the femur radius.
- ⚠️ Leg **167.2 -> 186.7 g**, body **4.3041 -> 4.38328 kg**; runtime
  19.53 -> **18.81 min** against NFR6's 30, and NFR5 exceeded by 8.4 %.
- ✅ **Lateral sway re-tuned 11.0 -> 12.5 deg.** The sway is tuned against the
  stance width, and at the old amplitude the wider track cost the walk
  **4.66 -> 2.04 mm** of margin. Separating the causes: track alone 2.13, heavier
  legs alone 4.58. The DYNAMIC margin picks 12.5, not the static one -- static
  keeps rising while the ZMP turns over and 14 deg is infeasible.
- ✅ **NFR15 is met from mu 0.5 at both speeds** (50.80 and 49.13 mm). M92
  recorded that guard as 22 microns from flipping.

### ⚠️ The correction to ADR-0092, and it is the important part

ADR-0092 measured the balance baseline at **7.32 mm mean / 89.59 peak**, found
stiffness and damping sweeps giving isolated islands, concluded *"the harness has
no robust operating point at the corrected CoM"*, and marked seven tests.

**Five of the seven pass now and nothing about the balance loop changed.** With
the track corrected the baseline reads **0.69 mm mean / 3.08 peak at the shipped
`kp = 80`** -- better than it has ever been, and better than the ventral-spine
baseline that was itself bought by a CoM in the wrong place.

Two things were wrong at once. I corrected one, measured the wreckage, and read
it as a property of the plant. The measurement was right; the CAUSE was assumed.

### Other reversals, recorded rather than smoothed

- ⚠️ **ADR-0073's condition is a gain WINDOW, not a floor.** At a 0.05 m held
  drop the leg cable saturates at kp 0 and again across a band from ~250 to
  ~600; kp 25-200 and 800+ are clean, and 0.30 m does not saturate where 0.05
  and 0.10 do. Same signature as M87's survival criterion: a single operating
  point is not evidence.
- ⚠️ **Correction to the above, made the same day.** I first wrote that *"the
  SHIPPED gain is 300"* and it is not. 300 is `_held_drop`'s own PD gain -- the
  test file calls it "the shipped loop" and nothing ships it. `mjsim.build` uses
  an MJCF actuator `kp = 1000`, a different quantity in different units, and
  `_spine_stand` a third law at 8.0. So the band is a property of one test's
  control law, and what it implies for a real spine controller is unestablished.
  The `[owed]` is smaller than I stated: the spine loop has no single gain to
  move, and giving it one is the actual debt.  `[owed]`
- ⚠️ **ADR-0072's assumption fails.** Compliance left the sway alone at +1 %;
  it now takes **-37 %** (4.60 -> 2.90 mm).
- ⚠️ **The compliance penalty on righting went negative**: M71's 4.7x, M87's
  3 %, and now **7 % FASTER** than rigid.
- ⚠️ The bare cable no longer inverts the robot, it falls to 48 deg; the worst
  standing tendon came **off** the 222.9 N ceiling to 192.8; M44's clipped-hip
  drift 197 -> 17.3 -> **4.5 deg**, while the ratio it is really about stayed
  at ~450x.

---

## ADR-0094: the skin, and the fibre that does not move

- **Status:** Accepted — M94, **the anchor's justification corrected the next
  day (M95)**
- **Date:** 2026-09-20 (M94)
- **⚠️ CORRECTION (M95).** ~~The flank at the spine axis is the one line that
  does not move~~ — it is the one line that does not move **in PITCH**. The
  strain table below is the sagittal case only, which this ADR said in its own
  `[owed]` and then reasoned past. Yaw turns about the vertical, so its neutral
  line is the mid-sagittal plane and the flank is the furthest thing from it:
  the flank takes **9.8 %** in yaw. There is **no fibre neutral in both**.
  The seam moves to the upper flank where the worst of the two is least —
  **6.6 %**, pitch and yaw balanced, a third better than the flank. The design
  survives the correction; the argument for it did not.

### Context

*"Finish the skeleton and then make the skin cover"* was the first instruction of
this whole arc, and the skin has waited through eight milestones for something
worth covering. It is buildable now: the dorsal line is one the process tips
actually reach (ADR-0092), four legs are on the trunk, and every rigid body is a
single piece.

### Decision

**Derive the cover from the strain, not from the silhouette.** A skin fibre at
height `z` changes length by `(z - spine_axis) * theta` at every joint, so over
the 75 deg of total pitch ROM:

| skin line | z mm | r from axis | dL mm | strain |
|---|---|---|---|---|
| dorsal, over the tips | 84.8 | +35.0 | 45.8 | **12.6 %** |
| **FLANK, at the spine axis** | **49.8** | **0.0** | **0.0** | **0 %** |
| belly at the waist | -16.4 | -66.2 | 86.6 | 23.9 % |
| belly at the chest | -46.5 | -96.3 | 125.9 | **34.7 %** |

✅ There is a neutral fibre **in pitch**, exact rather than approximate, on
the flank at the spine axis. ⚠️ M95 found there is none in yaw there — see the
correction above — so the seam sits a little above it. So:

- the cover is **seamed along the upper flank**, where the worst of pitch and
  yaw is least (6.6 %), and the seam is elastic rather than bonded because
  nothing on the cover is stationary in both DOFs;
- the dorsal panel is a **knit** -- 12.6 % is inside what one does;
- the belly panel is **not a stretch panel at all**. 34.7 % is nearly three times
  the dorsal figure, so it is **slack, gathered into a fold that pays out** as
  the spine extends. A cat has exactly that and it has a name, the primordial
  pouch, which is a good sign for a shape arrived at from a table.
- each leg leaves through an **aperture with a cuff**, r = 19 mm.

### Consequences

- ✅ One piece, 130 g in knit nylon at 1.0 mm, enclosing the skeleton with at
  least **2.0 mm** everywhere. That fits: the trunk budget leaves 666 g for what
  is not drawn.
- ⚠️ **The first run was pierced at every joint.** Hung from `Z_DORSAL` the
  cover sat at 81.8 against process tips reaching 82.8 -- **-1.0 mm**. It hangs
  from the TIPS now. The check caught it on its first execution, which is what
  the check was for.
- ⚠️ **And then the check went on reporting the defect after it was fixed**,
  because it recomputed the cover's top from `Z_DORSAL + CLEAR` instead of
  reading `outline()`. A check that recomputes what it is checking is checking
  something else. It reads the geometry now.
- ⚠️ **"Does it foul the legs" was the wrong question.** The flank clears the
  femur by **0.5 mm**, which a binary check passes and a deflecting cover does
  not survive. Measured as a margin it is obviously not clearance, and the legs
  get apertures.
- ⚠️ **It is a cover, not a structure.** Nothing here checks that it holds its
  shape under its own weight, nor what the fold does when the spine yaws rather
  than pitches -- the table above is the sagittal case only. `[owed]`

---

## ADR-0096: the tail, and what it costs to carry one

- **Status:** Accepted
- **Date:** 2026-09-21 (M96)
- **⚠️ Completes [ADR-0007](#adr-0007)'s motor count, five milestones late.**

### Context

Continuing the modelling turned up a part that could not count itself. ADR
decision F bought **19 motors** -- 12 leg, 3 spine pitch, 3 spine yaw and **1
tail** -- and `params.py` still budgets a "7-motor spine+tail bank". M88
redistributed the motors along the trunk and placed **18**. The tail's fell out
and no check counted it, because the trunk's own motor line read `%d of 18`.

The tail itself had no length, no mass and no section anywhere in the model.
And `SPINE_TAIL_SPEC` mounts it at `x = 0`, which was the pelvic girdle's rear
face when the trunk was two boxes; M88's rear girdle runs from -112 to -7 with
the hip at its FRONT, so the spec now grows a tail **112 mm forward through its
own body**.

### Decision

Draw it, mount it where the body ends, and **account for it** rather than
justify it.

- Base at the girdle's rear face on the **spine axis**, because the caudal chain
  continues the vertebral one.
- 225 mm, 0.62 of the trunk. `[assumed]` -- ANATOMY.md gives a vertebra count
  and no proportion.
- The 19th motor needs **no new row**: body 0 is 104.6 mm and two rows already
  use 88.2, but a third POSITION fits. On the centreline it must clear the pair
  by 2R, so |z| >= 28.0, and 28.5 reaches 45.8 against a 56.7 half-height.

### Consequences

- ✅ **19 of 19 placed**, and the roles separate honestly: 6 hind, 6 fore,
  6 spine, 1 tail.
- ⚠️ **The drive-train check counted by ROW and a row can hold two roles.** It
  read `tri3` as seven legs' worth on body 0 for a leg that needs six. Roles are
  per position now.
- ⚠️ **THE result, and it is an account rather than a capability:**

  | | share of the body's pitch inertia |
  |---|---|
  | what the tail ADDS | **+11.7 %** |
  | what its curl MODULATES | **+0.8 %** |
  | cost per unit of use | **14x** |

  Density does not change the ratio -- the tail is all lever, so cost and
  authority scale together. So it is **not an inertial device**, which agrees
  with G6 being withdrawn ([ADR-0071](#adr-0071)) and with ADR-0007 calling it
  coarse. The only design choice left is how light: silicone 56 g / 11.7 %,
  hollow TPU 17 g / 3.6 %, EVA foam 10 g / **2.0 %**.
- ⚠️ The righting reflex is already a factor of nine short (ADR-0093). A tail
  at silicone density makes the body 11.7 % harder to rotate for 0.8 % of
  authority, so **the material choice is a righting decision**, not a finish
  one. Foam unless something else argues.  `[owed]`
- ⚠️ The motor budget moved: 18 x 131.7 -> **19 x 131.7 = 2502 g**, so the
  trunk's structure allowance falls 1024 -> **893 g** and what is left for the
  un-drawn battery, electronics and cable falls to **522 g**. `params.py` still
  says "7-motor spine+tail bank" in the middle segment, which M92 already owed.

---

## ADR-0097: the head, and the third of the pitch inertia nobody was carrying

- **Status:** Accepted — M97, **its shape re-derived from a photograph the same
  day (M98)**
- **Date:** 2026-09-21 (M97)
- **✅ SUPERSEDED IN METHOD (M99).** The cranium now comes from **two
  orthogonal public-domain skull plates** -- Figs. 39 and 40 of Reighard &
  Jennings (1901), which `ANATOMY.md` already named as authoritative -- traced
  and intersected, so each section takes its height from one view and its width
  from the other. They ship in `reference/plates/`, so the measurement re-runs
  from the repo, which the wildcat photograph never could. Traced, the skull is
  **0.71 as wide as it is long and 0.48 as tall**; the 0.72 previously assumed
  for width happened to be right. What the photograph still supplies is the
  POSTURE, because a skull plate cannot say how high the head is carried.
- **⚠️ CORRECTION (M101). The neck was the skull's outline running out.**
  `_stations()` started the neck tube at the LAST traced skull section, and at
  the occiput a traced outline is the nuchal crest closing to a point --
  half-width **8.2 mm**, depth 11.7. So the neck left the head as a **13 mm rod
  carrying 240 g**, 0.15 of the chest where a cat is about 0.6. The first
  whole-robot render showed a stalk, and `test_head.py` had already written
  down the mechanism -- *"neck a slim tube, so more of the fixed 240 g rides in
  the head"* -- while treating it as the result rather than the cause. A skull
  is not a head: the part of a cat's neck that has any size is muscle round the
  braincase, and the plates do not draw muscle. The neck now leaves the head at
  the braincase (`NECK_AT = 0.86`, still 20 mm half-width there) with a named
  `NECK_FLESH`; the crest behind it sits inside the neck, as it does in the
  animal. Corrected, **0.15 -> 0.57 of the chest**, and the cost falls
  **40 % -> 36 %** because fattening the neck moves mass back toward the body
  (lever 267 -> **255 mm**). The FOURTH move of this number, and the first
  downward.
- **⚠️ CORRECTION (M98).** The form below was built from spheres on a rising
  axis and the proportions were invented. Traced off a side-view cat they are
  wrong in three ways: the head was carried **+35 mm** above the back where a
  cat carries it **+108** (a head-length); its depth-to-length ratio was 0.6
  where a cat's is about 0.78; and a first re-trace shipped the scan's **crop
  line** as the neck's underside. Corrected, the cost rises **24 % -> 29 %** --
  getting the shape right made the account worse.

### Context

`SpineParams.front_girdle_mass` absorbs **240 g of head and neck** and lumps it
at the girdle mount. `params.py` carried the warning in its own comment --
*"the head is the weak point and it is not in the box... putting it forward
would raise the pitch inertia"* -- and nobody had measured by how much.

### Decision

Give the lump a place and a shape, and **size it by the body's MASS**.

⚠️ The first attempt scaled the head off the trunk: 363 mm against a cat's
~250 gives 1.45x. That is the wrong ruler twice over. This robot is **4.38 kg**,
which is a cat, so a cat's head is the right head -- and the trunk is long FOR
that mass because the rear girdle reaches 112 mm behind the hip, a standing
`[owed]`. Scaling by it copies that defect into the head:

| sizing | head mm | nose x | adds Iyy | % of body |
|---|---|---|---|---|
| **by mass, a cat** | **95** | 407 | **1.02e-2** | **24 %** |
| by trunk length | 138 | 470 | 1.39e-2 | 33 % |

The neck is one rigid lump with the head: ANATOMY.md puts the 7 cervical
vertebrae explicitly out of scope, which is what the mass model always assumed.

### Consequences

- ⚠️ **Placing it costs 24 % of the body's pitch inertia** (+1.02e-2 on 4.20e-2)
  and carries the CoM **7.1 mm forward**. The lever from the body CoM goes
  **81 -> 219 mm**. ⚠️ Superseded: with the traced shape, the cat's posture
  and a neck that is a neck, it is **36 %** (+1.496e-2), the CoM moves
  **+8.4 mm**, and the lever goes **81 -> 255 mm**.
- ⚠️ **With ADR-0096's tail at 11.7 %, the plant has been missing about a third
  of its own pitch inertia** -- **48 %** at the corrected head, so nearly half
  -- and the righting reflex is already a factor of nine short
  ([ADR-0093](#adr-0093)). Neither part was invented here; both were
  in the budget with no position.
- ✅ Checks pass: the head sits 115 mm up against a dorsal line of 80, which
  is how a cat carries it, and it does not touch the fore leg.
- ⚠️ **Nothing is propagated yet.** `front_girdle_com` and
  `front_girdle_inertia` still lump the 240 g in the housing, so the plant does
  not yet know. Moving it re-baselines the dynamics a fourth time and wants the
  tail's material decided first -- the two land in the same account. `[owed]`

---

## ADR-0098: the whole robot, and a picture assembled by its viewer

- **Status:** Accepted
- **Date:** 2026-09-21 (M101)
- **⚠️ Corrects the first render of the complete machine, and the check
  written to catch it.**

### Context

M96 drew the tail, M97-M100 the head, M94 the skin. Each was checked against
the trunk inside its own module. **Nothing had ever drawn the whole machine**,
so the obvious next step was a picture of it -- and the picture was built by a
throwaway script outside the repo that called `tomcat_leg_detail.build()`
directly.

`tomcat_assembly` exists precisely to stop that. It drops the leg module's
`motor` group (`DROP`) because the trunk owns those motors, drops the leg's
duplicate hip tongue (`HIP_HARDWARE`), and hands each leg the trunk's REAL
spool centres, because the leg's default `SPOOL_OFFSET` misses them by 20 to
44 mm. The script did none of it: it put **12 reference motors** on the robot
and pointed every cable at empty space.

Those rules were comments. `tomcat_assembly.py` had **no pytest at all**, so
nothing enforced them on a caller who did not read the source -- and the
caller who did not read the source was me.

⚠️ And `assembly()`, the function named *the whole robot*, returned four
trunk bodies and four legs. It knew about the head, the tail and the skin not
at all.

### Decision

**The assembly draws its own picture**, from the same build its checks run on.

- `assembly()` returns `(bodies, legs, soft)`; `soft_parts()` places the head,
  the tail and the skin.
- `render_views()` lives in `tomcat_assembly.py` and writes `tomcat_whole.png`.
  The mesh is tessellated ONCE and shared across the three views, because a
  collection per subplot pays the CAD mesh three times.
- `tests/test_assembly.py` asserts the ownership rules, in both directions: a
  leg in the assembly carries no motor, **and** the leg module still draws
  three -- otherwise the first test would pass for the wrong reason forever.

### Consequences

- ✅ **What it measures that nothing measured before:**

  | | |
  |---|---|
  | skin / leg overlap, four legs | **0.0 mm3** |
  | cable end off its spool | 0.000 mm |
  | four feet on one plane | 0.00 mm spread |
  | envelope, L x W x H | **776 x 154 x 348 mm** |
  | drawn volume | 934.5 cm3 |

  The skin figure answers a question nobody had asked. `APERTURE_R` is sized
  from the femur tube plus ROM plus a cuff, and the femur is the **slimmest**
  thing at the hip -- the groove-plane stack, its bearings and the via shaft
  reach 23.9 mm outboard of the bone plane. It clears, but it cleared by
  accident, and now it is checked.

- ⚠️ **THE CORRECTION, and it is to the new check.** The motor-duplicate
  test matched a can by VOLUME, within 2 % of 33,747 mm3. It flagged **trunk
  body 2**, which is 34,202 -- a whole rigid body reported as a duplicated
  motor, and the assembly failed for a reason that was not true. A volume is
  not an identity. A can is Ø34.5 x 36.1 and its BOX is as particular as
  its volume; `is_motor_can` now requires both, and a test asserts that no
  trunk body matches.

- ⚠️ **The 154 mm width is not the motors.** Measured on the standalone
  leg, the `motor` group reaches y = 79.2 and it was tempting to call it the
  cause. With the bank dropped the leg bbox is still `y[+42, +77]`: the width
  is the monotone groove-plane stack that ADR-0093 chose to get zero fleet
  angle, 23.9 mm of pulleys bolted to the outer face of every bone. The
  phantom motors were clutter INSIDE the envelope, not the envelope.

- ⚠️ **The fore legs have no clearance at all.** `leg/trunk overlap` is
  73.2 mm3 on LF and RF against 0.0 on the hind pair, and the depth check reads
  *limb plane 53.0 mm ... needs 53.0*. M93 set `TRACK_Y` to exactly the
  requirement, so the front girdle's widest section touches the femur. It
  passes and it has nowhere to go; the next millimetre the chest gains is an
  interference.  `[owed]`

- ⚠️ **The picture found a defect the numbers had already described.** The
  neck came out a 13 mm stalk, because `_stations()` began it at the last
  traced skull section -- and `test_head.py` had written down that exact
  mechanism as the reason the head's cost rose to 40 %, treating it as a
  result. Corrected in [ADR-0097](#adr-0097): the neck leaves the head at the
  braincase, **0.15 -> 0.57 of the chest**, and the cost falls **40 % -> 36 %**.
  Rendering the whole machine is how it was seen. That is the point of the
  picture, and the reason it now belongs to the assembly.
- ⚠️ **`soft_parts()` shipped without the face.** `tomcat_head` draws
  `whole()` and `features()`; the first repo-side render had the cranium and no
  ears, no eyes, no nose. **Fourth time** a list that enumerates parts has
  missed a new one, after `link_inertia.RHO` dropped 14.67 g and the hip filter
  missed `shaft`. The test now names every solid each module draws and compares
  the volume that reached the assembly.
- ⚠️ Unchanged and still owed: the head and the tail are **not** in
  `params.py`. `front_girdle_com` and `front_girdle_inertia` lump 240 g of head
  in the housing and there is no tail at all, so the plant is missing about a
  third of its own pitch inertia ([ADR-0096](#adr-0096),
  [ADR-0097](#adr-0097)). This milestone made the picture honest, not the
  model.

---

## ADR-0099: the head and the tail reach the plant, and the walk has to be re-tuned

- **Status:** Accepted
- **Date:** 2026-09-21 (M102)
- **✅ Pays the `[owed]` that [ADR-0096](#adr-0096) and [ADR-0097](#adr-0097)
  both ended on**, and corrects three claims that were measured on a plant with
  no head in it.

### Context

`SpineParams.front_girdle_mass` absorbed **240 g of head and neck** and
`front_girdle_inertia` spread it uniformly through the girdle housing.
`params.py` said so in its own comment -- *"the head is the weak point and it
is not in the box... putting it forward would raise the pitch inertia"* -- and
left it there for nine milestones. The tail and the 19th motor that drives it
were not in the model at all.

⚠️ And the tag on those numbers read `[derived: cad/tomcat_packaging.py]`.
That module packs motors and sizes housings; **it never prints a mass, a CoM or
an inertia tensor**. The girdle figures came from a script that did not survive.

### Decision

Settle the tail's material, derive the girdles reproducibly, and put both
appendages where they are drawn.

- **EVA foam for the tail.** ADR-0096 priced it -- silicone 11.7 % of the
  body's pitch inertia for 0.8 % of authority, foam 2.0 % -- and wrote *"foam
  unless something else argues"*. Nothing argued, and the righting reflex is
  still a factor of nine short ([ADR-0093](#adr-0093)), so the cheap tail wins.
  **9.8 g** at 0.20 g/cm3.
- **`cad/girdle_inertia.py`** is the missing derivation. It re-derives the
  PUBLISHED numbers before replacing them: strip the head-as-housing-lump out
  of the old front girdle and what is left must be the rear girdle plus 20 g of
  structure. It is -- **0.5 % on Ixz**, exact on Iyz.
- The 19th motor and the tail move **141.8 g** from `segment_mass[1]` to the
  rear girdle. The budget already paid for both out of the structure
  allowance, so the body total does not move: **3.635 kg**, unchanged.

### Consequences

- ⚠️ **The head's placement is worth 7.2x the front girdle's pitch inertia**
  and flips the sign of its `Ixz`:

  | | was | now |
  |---|---|---|
  | front girdle CoM | (-4.6, +17.4) mm | **(+27.4, +37.8)** |
  | front girdle Iyy | 1.1412e-03 | **8.2412e-03** |
  | front girdle Ixz | +1.4855e-04 | **-3.2340e-03** |
  | rear girdle mass | 0.902 kg | **1.0438** |
  | rear girdle Iyy | 7.8852e-04 | **2.0030e-03** |
  | whole-body CoM height | 14.9 mm | **20.1 mm** |

- ⚠️ **THE consequence: the default walk stopped being feasible.** The ZMP
  margin went **+5.30 mm -> -0.14**, with a foot pulling 57 mN. And it is not
  the head's REACH that does it, it is its HEIGHT -- moving the feet forward
  changes nothing (`nominal_foot` x 0.050 -> 0.070 moves the margin -0.141 ->
  -0.145 mm). A CoM 5.2 mm higher throws the ZMP further sideways for the same
  sway, so the whole ZMP-vs-amplitude curve dropped about 5 mm and took its
  optimum with it.
- ✅ **Re-tuned by the method [ADR-0092](#adr-0092) used when the track moved:
  `lateral_amplitude` 12.5 -> 11.0 deg**, which is where it was before M93
  raised it. The corrected plant is not worse -- **5.60 mm** at the new optimum
  beats the 4.86 M93 could reach.
- ⚠️ **Three results were measured on a headless robot and change:**
  - arching the spine **moves the CoM rearward again**. ADR-0092 concluded the
    arch is a vertical actuator and not a fore-aft one; with the head placed it
    is `dx -4.52 mm, dz +50.39` against `+0.73 / +46.99`. Still mostly vertical
    (11:1) and no longer zero.
  - the base spine joint in quiet stand: **0.354 -> 0.498 N.m**, the one number
    the head made worse and the one it should -- 240 g cantilevered 149 mm
    ahead of the mount is what *"the head is the weak point"* meant. Under the
    tuned model's 0.57, with less room than it had.
  - the 1-D viable reduction under-claims by **9.2 %**, up from 4.0. It has
    widened at every step that made the body more real, always in the
    conservative direction: a projection loses more as the true set stops being
    axis-aligned.
- ✅ Two got BETTER: the exact viable worst case **33.77 -> 34.85 mm**, and the
  spine's NFR15 authority **65.2 -> 66.4 mm** against a requirement of 48.
- ⚠️ **A derivation that reads its own output is not a derivation.** The
  first `girdle_inertia.py` took its baseline from the live `params.py`. Run
  once it was right; run again, after `params.py` had been updated, it placed
  the head twice and sent the front girdle's Iyy to 1.2e-02. The
  reconstruction check caught it -- `Ixz` off by **713 %** -- and the fix is
  not to notice better: the M101 baseline is frozen in the file.
- ⚠️ **The repo carries two masses for the same motor**: `params.py` builds
  both girdles from 0.132 kg, while ADR-0096, `mass_closure` and `thermal` use
  131.7 g. 0.3 g is nothing to the physics and everything to whether the
  accounting closes. Not reconciled here.  `[owed]`
- ⚠️ **Still the two-girdle architecture M88 replaced.** The model is rear
  girdle + 3 segments + front girdle; the CAD is four rigid bodies with motors
  spread 6/4/2/6. Mapping body 3 -> front girdle and body 0 -> rear girdle is
  faithful for the leg motors and for these two appendages, and it is not the
  re-apportionment owed since M92. What this milestone did remove is the
  phrase *"the 7-motor spine+tail bank"*: the seventh was the tail's, M88 put
  it on body 0, and it is on the rear girdle now.  `[owed]`
- ⚠️ The tail is a rigid lump on the rear girdle. **Its joint is not
  modelled**, so the 19th motor drives nothing in the plant -- which matches
  ADR-0096's finding that the tail is not an inertial device, and is still a
  DOF the CAD has and the simulator does not.  `[owed]`

### What the SIMULATOR said, which is not what the analytic model said

The MuJoCo harness moved four numbers, and three of them moved the good way.

- ✅ **The spine's stiffness cliff became a pure cliff.** The lateral chain
  carries the forequarters, and the head is now 240 g of pitch inertia sitting
  on them:

  | `spine_kp` | 30 | 60 | 100 | 150 | 250 | 1000 |
  |---|---|---|---|---|---|---|
  | fell in 20 steps | yes | **yes** | no | no | no | no |
  | mean abs dcm | 50.4 | 33.2 | **0.9** | 0.9 | 0.9 | 1.0 mm |

  M86 read 150 -> 12.4 mm and 1000 -> 2.0, and the finding's closing line was
  that *"the cliff is not the whole finding"* -- there was a gradient above it.
  There is none now: 100 and 1000 are the same number, and the settled wobble
  fell **2 to 13x**. The mass that made the walk harder makes the spine quieter.

- ✅ **[ADR-0046](#adr-0046)'s degenerate survival envelope is no longer
  degenerate**, and one of its four xfails is lifted. M41 measured **37.17 mm
  at BOTH 120 and 300 deg**, above the 29.15 mm exact viable bound -- a number
  no controller can reach. Placing the head moved both ends toward each other:
  the bound GREW to **34.85 mm** and the 120 deg envelope FELL to **31.11**
  (89 %), with 300 deg at 34.39 (99 %). ⚠️ And the test could not have seen
  that degeneracy -- it measured 120 deg alone, so *"equal at both angles"* was
  invisible to it. It measures both now.

- ✅ **M30's paired design has its own evidence again.** The phase-to-phase
  spread the corrected mass model had flattened below 1.0 mm is back at
  **1.8 mm**, and the test had asked in its own message to be told if it
  returned.

- ⚠️ **And one test was measuring the wrong thing.** The baseline's wind-up
  guard read *last 10 steps < 2x first 10*, and the first ten steps are the
  quietest the run will ever be, because the harness resets to rest. Over 60
  steps the baseline is not drifting at all -- it converges:

      steps  0- 9   0.709 mm   <- the reset transient, not a baseline
      steps 20-29   1.650
      steps 40-49   1.751
      steps 50-59   1.762      <- asymptote, worst excursion 3.47 mm

  The old form passed only while the plant settled inside twice its own
  start-up, and the head pushed it to 2.3x. Wind-up is a property of the TAIL,
  so the guard compares the last two windows to each other -- and the run had
  to grow **30 -> 50 steps**, because at 30 the baseline is still 22 % from
  its asymptote. The gate that exists to establish the harness's noise floor
  **had never once observed its own steady state.**

- ✅⚠️ **[ADR-0073](#adr-0073)'s landing alarm resolved and got worse, in
  two different cables.** Dropping the held quadruped and re-sweeping the spine
  gain at 0.05 m:

  | `spine_kp` | 0 | 100 | 300 | 600 | 1000 |
  |---|---|---|---|---|---|
  | leg cable, M93 | **222.9 SAT** | 70.2 | **222.9 SAT** | **222.9 SAT** | 78.2 |
  | leg cable, M102 | **81.4** | 67.6 | **46.8** | 49.2 | 48.2 |
  | spine's own, M102 | 5743 | 5337 | **3492** | 3940 | 5178 |

  ✅ **The saturation BAND is gone at every gain**, and the leg cable never
  reaches its rating -- 46-81 N against 222.9, where the LIMP case used to
  saturate. The mechanism is the one ADR-0073 named and then mis-attributed to
  the articulation: the leg PD saturates when it has to chase a hip that moves,
  and a trunk carrying 250 g at its two ends moves less. So that margin stops
  being conditional on a spine gain nobody has room to tune.

  ⚠️ **And the spine's OWN cables got worse where the drop is SHALLOW, and
  not at all where it is deep.** At the same gain:

  | drop | leg cable | spine's own | M93's spine |
  |---|---|---|---|
  | 0.05 m | 46.8 N | **3492 N, 15.7x** | 1096 N |
  | 0.10 | 50.7 | 2960, 13.3x | 1457 |
  | 0.30 | 74.5 | 1798, **8.1x** | 1720 |

  3.2x at 0.05 m and +5 % at 0.30 -- the mass that protects the leg cable is
  mass the spine must arrest, and a shallow drop gives it the least time. The
  spine's own cables remain what ADR-0073 called *"the larger number"*, at
  8-16x their rating, and nothing here addresses them.  `[owed]`

  ⚠️ **A first draft of this entry said 15.7-25.8x**, by quoting the LIMP
  case's 5.7 kN as if it were the held one and by reading a single drop height
  as the range. The numbers above are at the same gain across all three
  heights.

- ✅ **The righting got faster, and that is the biggest single number here.**
  Closed-loop righting from fully inverted: **2.81 -> 1.35 s**. Three
  independent measurements agree and none of them depends on the gait re-tune:

  | | M93 | M102 |
  |---|---|---|
  | spine only, open loop | 74.1 deg/s | **109.8** |
  | + anti-phase leg tuck | 95.6 | **115.2** |
  | closed loop | 64 | **133** |
  | equivalent fall | 38.7 m | **8.9** |

  ⚠️ **And the leg tuck's contribution collapsed, +29 % -> +5 %.** That is
  the finding, not a side note: a righting manoeuvre is the spine reacting
  against the body's own ends, and until now the model had no ends. Placing the
  head and the tail gives the spine alone nearly everything the tuck used to
  supply -- which is what a falling cat does, and why ADR-0088's *"how much the
  legs still had to give"* kept shrinking. [ADR-0093](#adr-0093)'s factor of
  nine is not closed; about a third of it is.

- ✅ **The compliance penalty on the righting came back, and a two-sided bound
  is what caught it.** M71 measured compliance costing the righting 4.7x, M87
  had it at 3 %, and M93 measured it **NEGATIVE** -- 7 % faster -- and left a
  comment saying *"the bound is two-sided so a return to a real penalty fails
  here."* It returned: **1.49x** (2.015 s against 1.35 rigid). Placing the head
  made the rigid plant much faster and the compliant one only somewhat, so the
  springs give back a smaller share of a bigger manoeuvre. Between 4.7x, 3 %,
  -7 % and 1.49x, what that number measures is the plant it was measured on.

- The trot's copper loss and the runtime that follows moved with the plant:
  **7.4126 -> 7.4702 W** per motor, **18.81 -> 18.69 min**, total
  **133.9987 -> 134.8254 W**. A quiet stand is untouched -- it carries the same
  weight either way; it is the trot that pays.

## ADR-0100: the motor is a proxy, and the mechanism emits its specification
- **Status:** Accepted
- **Context:** M106 turned capstan friction on and everything downstream moved.

  ADR-0003 specified the capstan model. `tendon.py` implemented it, at length,
  with the sign convention written out. `TendonParams.wrap_angle` was **0.0**,
  and its own comment called it *"inert pending a per-joint-wrap extension"*.
  [ADR-0083](#adr-0083) (M78) **was** that extension: it solved every wrap in
  the leg and recorded them in `pair_wrap`. Nothing consumed `pair_wrap`.

  So the capstan factor has been exactly 1.0 for the whole project, and every
  actuator, power and thermal number was computed frictionless.

  ⚠️ **Two things were wrong at once, and the second hid the first.** The
  recorded wraps were pre-M91 — `leg_tendons.route()` falls back to
  `SPOOL_OFFSET`, a diagonal that suited the pre-M88 upright girdle, while
  `tomcat_assembly` overrides it because the defaults miss the real motors by
  20–44 mm. `route(spools=)` and `tomcat_trunk.leg_spools` both arrived in M91;
  ADR-0083 is M78. The table was solved thirteen milestones before the cables
  were connected.

  ⚠️ **And the guard could not see it.** `test_the_EXTENSOR_SIDE_was_never_solved`
  re-derives `pair_wrap` from the router — with the same missing `spools=`. A
  check that reproduces its subject's mistake agrees with it forever. Its
  docstring said the values *"cannot drift from the CAD"*.

  Re-solved on the spools the trunk actually has:

  | pair | flexor, was → now | extensor, was → now |
  |---|---|---|
  | hip | 122.1 → **135.6°** | 7.9 → **239.4°** |
  | knee | 158.6 → 141.0 | 124.7 → 150.0 |
  | ankle | 107.6 → 152.8 | 392.9 → 239.2 |

  ⚠️ ADR-0083's headline inverts. The worst cable is not the ankle extensor at
  1.985×; it is the hip and ankle extensors tied at ~1.52 — and the hip
  extensor is the one ADR-0083 recorded as essentially frictionless at 1.014×.

- **Options:**
  1. Leave friction off. Rejected: the data existed and the mechanism existed;
     only the wiring was missing.
  2. Turn it on and shrink the design until the GIM3505-9 fits again.
  3. Turn it on and treat the motor as a placeholder whose specification is an
     **output** of the mechanism, to be sourced or designed against later.

- **Decision:** **Option 3.** The linkage is the primary design. The motor
  supplies an envelope and a mass to package and to weigh; it no longer supplies
  a torque ceiling the mechanism must fit under. `MOTOR_PEAK` left
  `mass_closure.gate()` and became `PROXY_PEAK`; `motor_requirement()` publishes
  what the mechanism demands. `wrap_angle` now defaults to `None`, meaning *use
  the routed wraps*, and an explicit number — including `0.0` — overrides it, so
  the frictionless plant is one parameter away.

- **Consequences:**

  ⚠️ **The specification the mechanism emits, at the trot ADR-0008 fixes as the
  actuator sizing case:**

  ⚠️ **The peak below is the HIND leg's, and [ADR-0102](#adr-0102) found the
  budget has only ever seen one leg.** The fore leg needs **2.42 N·m**, 24 %
  over, set by its KNEE. Read the row as a floor.

  | | required | proxy | short by |
  |---|---|---|---|
  | peak torque | **2.20 N·m** (hind; fore 2.42) | 1.95 | 13 % (fore 24 %) |
  | continuous torque | **0.77 N·m** | 0.71 | 9 % |
  | peak current | **4.34 A** | 4.19 | 4 % |

  A slightly larger part, not a different class. The hip is the sizing joint,
  and its tendon passes no joint at all — none of its 239° of wrap is coupling
  it has to carry, so it is avoidable routing rather than a fact about the leg.

  ⚠️ **Nothing in the catalogue meets it.** The GIM3505-8 already failed; the -9
  now fails too. What is left is the GIM4305-10 at 3.00 N·m — and it is Ø53
  against Ø34.5, where `tomcat_packaging` sized the girdle around nineteen Ø34.5
  cans and `tomcat_trunk` placed every one. The escape hatch is real and it is
  not free.  `[owed]`

  ⚠️ **`[owed]` — the proxy still supplies the envelope and the mass.** Ø34.5 ×
  36.1 mm and 131.7 g, nineteen of which are half the body. Whether a part
  meeting the spec keeps those is exactly the question deferred; if it does not,
  the mass budget moves again.

  ⚠️ **NFR6 falls below its own re-stated range, at both Kt corners.** Runtime
  **18.81 → 12.71 min** optimistic and **13.58 → 8.78** pessimistic, against the
  14–20 min ADR-0044 re-stated it to; range **565 → 381 m** against 420–600.
  Friction is a tension multiplier and copper loss goes as current squared, so
  it lands on the battery harder than anywhere else.

  ⚠️ **Corrected by [ADR-0101](#adr-0101) from 12.13 / 8.50 / 364 m.** The first
  wiring multiplied the motor torque by the capstan once, before splitting it,
  so the mechanical term carried the factor too — friction booked as useful
  work. Copper and current were right; total power, runtime, range and
  efficiency were not.

  ⚠️ **ADR-0044's unresolved Kt stops being an accounting question.** RMS current
  is **1.380 A** at Kt 0.44 and **1.735 A** at 0.35, and the continuous rating is
  1.60 — the two readings now fall either side of it. Which Kt is true decides
  whether the motor runs cool, not merely how long it runs.

  ⚠️ **`power.py` was measuring the two gaits differently and the error was in
  the ratio, not the numbers.** `gait_power` built motor torque straight from
  `tau / arm * spool` and never went through `TendonMap`, while `standing_power`
  goes through `torque_budget.evaluate` and does. With friction live the stand
  picked it up and the trot did not, and standing appeared to cost **1.49×** a
  trot — which would have been M16's finding inverted and was an artefact.
  Applied to both, the fraction is **0.924** and M16 stands: friction costs the
  trot 71 % and the stand 64 %. A ratio is only as good as its worse-computed
  half.

  - Drive efficiency **29.6 % → 16.5 %**: friction lands entirely on the copper
    and not at all on the useful work. Per motor, trot **7.4702 → 12.7551 W**,
    stand **9.0405 → 14.8428 W**, total **134.8 → 198.2 W**. (⚠️ published as
    20.6 % and 207.7 W; corrected by [ADR-0101](#adr-0101).)

  - ✅ [ADR-0050](#adr-0050)'s ankle pair does not close at the 14 mm arm the leg
    has — 138 % of the proxy's peak at the current spool, 117 % at the best one
    M105 found. The minimum for a sound pair is ~19–20 mm. That the CAD builds a
    spring return instead may be less of an oversight than M103 called it.
    Raising the arm to 25 mm takes the ankle to 68 % and its coupling fraction
    from 62.5 % to 35 %, matching the knee, for 8.44 g of sheave on a 33 g
    metatarsus. Not adopted here; it is a design change and this entry is about
    the plant.  `[owed]`

  - ✅ Ground clearance, spool travel and link interference were all checked as
    candidate limits on that arm and none of them binds: the ankle sits 57.3 mm
    above the stance foot, 25 mm of arm is 2.51 turns on a 9 mm spool, and the
    sheaves are laterally offset from the bone plane.

  - The guard now derives from `tomcat_trunk.leg_spools`, the same source the
    assembly uses, and `test_tendon.py` gained a check that fails if the shipped
    default ever goes frictionless again.

## ADR-0101: the hip's wrap is routing, and one spool position pays for the motor
- **Status:** Accepted
- **Context:** [ADR-0100](#adr-0100) left the mechanism asking for **2.20 N·m**
  against a 1.95 N·m proxy and named the hip as the joint responsible — its
  tendon passes no joint, so none of its 239.4° of wrap is coupling it has to
  carry. That claim was worth testing rather than asserting, because the hip's
  wrap lands **entirely on its own sheave** (stations 0 and 2 are zero), and a
  sheave wrap can be the ROM the sheave must span rather than a routing choice.
  239.4° against a 240° hip ROM is close enough to be suspicious.

- **Options:** measure it. Hold the pose and the other two spools fixed, move
  the hip spool, and see whether the wrap moves with it.

- **Decision:** ⚠️ **It is routing.** The hip extensor goes **239.4° → 10.7°**
  on a spool move alone. Sweeping the hip spool in z at its own x:

  | z (mm) | flexor | extensor | trot peak | vs the 1.95 proxy |
  |---|---|---|---|---|
  | **+37.0** (today) | 135.6° | 239.4° | 2.201 | over 13 % |
  | −3.4 (its row's lower slot) | 93.9 | 28.2 | 2.046 | over 5 % |
  | **−20.0** | 60.9 | 3.5 | **1.932** | **inside** |
  | −30.0 | 44.7 | 77.3 | 1.878 | inside |

  ✅ **So the motor shortfall is a routing defect, not a physical one.** At
  z = −20 the trot peak is 1.932 N·m and standing 0.677 against the 0.71
  continuous — both back inside the proxy, with no change of part.

- **Consequences:**

  ⚠️ **The cheap version does not work, and the reason is worth keeping.** The
  hip sits in a `pairs4` row at x = −31.2 whose two slots are z = +37.0 and
  −3.4, holding the hip and the **ankle**. Swapping them costs nothing — no new
  row, no envelope change — and it makes the leg *worse*: the hip improves to
  2.046 but the ankle's wrap goes 152.8/239.2° → 297.9/384.3° and its motor
  2.282, so the peak rises 2.201 → **2.282**. Both cables want the lower slot
  and there is one.

  ⚠️ **What the routing actually asks for is 16.6 mm below that lower slot.**
  The hip spool wants z ≈ −20 where the row offers −3.4. That is a girdle
  question, not a routing one, and `spool_search` says so in its own docstring:
  it knows where the routing wants the motor and nothing about packing.  `[owed]`

  ⚠️ **The placement rule that put the hip on top is a drawing rule.**
  `leg_spools` says the hip *"needs the longest cable and the largest arm, so it
  takes the topmost spool and the ankle the lowest"*. Nothing in that follows
  from the routing, and the routing wants the opposite. This is the same shape
  as [ADR-0099](#adr-0099)'s finding about the common-mode rule.

  ⚠️ **AND THE STUDY CAUGHT A DEFECT IN ADR-0100'S OWN WIRING.** Moving the hip
  spool lowered the copper loss and the efficiency fell with it, which cannot
  happen: `p_cu` goes as the square of the capstan factor and the mechanical
  term as its first power, so inflating both moves the ratio the wrong way.
  `gait_power` had multiplied `mot_tau` by the factor **once, before splitting
  it** into the current path and the work path, so friction was booked as useful
  output. A joint's useful output is `tau · qd` whatever the routing costs to
  deliver it. Corrected:

  | | ADR-0100 published | corrected |
  |---|---|---|
  | copper, currents, RMS | — | **unchanged** |
  | total | 207.7 W | **198.2 W** |
  | trot runtime | 12.13 min | **12.71** |
  | range | 364 m | **381** |
  | efficiency | 20.6 % | **16.5 %** |
  | standing / trot | 0.924 | **0.972** |

  Both of ADR-0100's conclusions survive: NFR6 still fails at both corners, and
  M16's *"standing costs most of what moving costs"* still holds. Copper is now
  **5.07×** the mechanical work.

  ⚠️ **`[owed]` — `pair_wrap` is one tuple and the wraps are pose-dependent.**
  The budget sweeps a workspace and applies a capstan measured at the stance
  pose. That approximation was free while the factor was 1.0; it costs
  something now, and nothing here has priced it.

## ADR-0102: the girdle can take most of it for free, and the FORE leg was never budgeted
- **Status:** Accepted
- **Context:** [ADR-0101](#adr-0101) left one question: the hip spool wants
  z ≈ −20 and its row offers −3.4, so can the girdle house it 16.6 mm lower?

- **Decision / findings:**

  ⚠️ **Nothing checks whether a motor is inside the shell.** `tomcat_trunk.report`
  checks that a motor lies in its body's x range and that no two interpenetrate.
  That the can is inside the lofted **ellipse** is checked nowhere, and it is
  exactly what decides how low a spool can go. `shell_fit()` now measures it.

  ⚠️ **At the hip's own row it does not fit, and the reason is the ellipse.**
  The section at x = −31.2 spans z −44.5…78.1, so there looks to be room; but
  the motors sit at y = ±20.2 and an ellipse narrows as it falls. At y = 20.2
  the limit is **z ≈ −10**; z = −20 fits only on the **centreline**, and a
  centreline slot serves one motor where two hind legs need two.

  ⚠️ **And the row's two slots are taken by cables that both want the low one.**
  Hip and ankle share `hind_a`; ADR-0101 showed swapping them makes the leg
  worse. Dropping the hip to −20 would push the ankle to ≈ +48, worse still.

  ✅ **But the ASSIGNMENT is free, and it is worth two thirds of the overrun.**
  Which joint drives from which of the three positions is set by `leg_spools`'
  own rule — *"the hip needs the longest cable and the largest arm, so it takes
  the topmost spool and the ankle the lowest"* — which is a drawing rule. All
  six ways, hind leg:

  | hip / knee / ankle | trot peak | vs the 1.95 proxy |
  |---|---|---|
  | **`hind_b` / `hind_a` hi / `hind_a` lo** | **2.026** | over 4 % |
  | `hind_a` lo / `hind_a` hi / `hind_b` | 2.046 | over 5 % |
  | `hind_a` hi / `hind_b` / `hind_a` lo (today) | 2.200 | over 13 % |
  | `hind_a` lo / `hind_b` / `hind_a` hi | 2.282 | over 17 % |

  Moving the hip to the far row and the knee to the near row's top slot costs
  nothing and takes **13 % → 4 %**.

  ⚠️ **The last 3 % is where the two curves cross.** With that assignment, the
  hip can fall to z ≈ 10 inside the existing section (2.002, 3 % over); below
  that the routing keeps improving and the shell stops containing it. Closing
  it entirely needs the hip at z ≈ −15, which wants **hw 41.7 → 46.0, +10 % of
  section perimeter** on that body.  `[owed]`

  ⚠️ **THE ACTUATOR BUDGET HAS ONLY EVER SEEN THE HIND LEG, AND THE FORE LEG IS
  THE BINDING ONE.** `mass_closure.budget_at` takes `leg_params=DEFAULT_HINDLEG`
  by default, `tomcat_leg_detail.live_loads` builds from `DEFAULT_LEG`, and
  `params.py` sets `DEFAULT_LEG = LegParams()` which **equals** `DEFAULT_HINDLEG`.
  `close()` weighs both legs and budgets one.

  | | frictionless | with its own wraps | vs the proxy |
  |---|---|---|---|
  | hind | 1.737 | 2.201 | over 13 % |
  | **fore** | **1.886** | **2.421** | **over 24 %** |

  So ADR-0100's published requirement of **2.20 N·m is understated; it is 2.42**,
  and it is set by the fore **knee**, not by a hip.

  ⚠️ **And the two overruns have different causes, so they need different
  remedies.** Frictionless, the fore knee was already at 1.886 against the hind
  knee's 1.193 — it carries more before any capstan is counted. Feeding the fore
  leg the *hind's* wraps still gives 2.412 against 2.421 on its own, and its
  best assignment is 2.382. **The hind's overrun is routing and routes away;
  the fore's is load and does not.**

- **Consequences:**
  - `assignment_trade()` and `shell_fit()` ship in `structure_matrix.py`.
  - Nothing is adopted here. The re-assignment is free and measured, but it
    changes which motor drives which joint across the CAD, the MJCF and the
    assembly, and the fore leg — the binding one — does not benefit from it.
  - `[owed]` — making the requirement the max over both legs needs `pair_wrap`
    in a per-leg form, which it does not have. Today it holds the hind's wraps
    and the fore leg is routed with them wherever the plant reads it.
  - `[owed]` — `DEFAULT_TENDON.joint_moment_arm` is shared by both legs. The
    fore leg is a different length with a different load; whether it wants the
    same 28/25/14 has never been asked.

## ADR-0103: the fore leg's overrun is load, and the moment arm is the lever that reaches it
- **Status:** Accepted — adopted in M111, see [ADR-0104](#adr-0104)
- **Context:** [ADR-0102](#adr-0102) found the actuator budget had only ever
  seen the hind leg, and that the **fore** leg binds: its knee needs 2.421 N·m
  against the 1.95 proxy, 24 % over. Neither re-routing nor re-assignment moves
  it (best 2.382), because the overrun is load and not routing — frictionless,
  the fore knee was already at 1.886 against the hind knee's 1.193. ADR-0102
  also noted that `joint_moment_arm` is **shared** by both legs and nobody had
  asked whether the fore leg wants 28/25/14.

  ⚠️ **The continuous requirement had the same blind spot.** Standing, the fore
  leg needs **0.845 N·m** against the proxy's 0.71 continuous — 19 % over —
  where ADR-0100 published the hind's 0.77.

- **Options:** motor torque goes as `1/r` at fixed joint torque, so the arm is
  the one lever that reaches load. Sweeping each fore joint's arm alone:

  | fore joint | today | enters the proxy at |
  |---|---|---|
  | hip | 28 mm, 2.214 | ~32–34 mm |
  | knee | 25 mm, **2.421** | **32 mm** |
  | ankle | 14 mm, 2.383 | 20 mm |

  Because the arms are shared, each candidate was then evaluated on **both**
  legs, and the mass spiral closed: four legs carry the sheave delta and the
  load rises with it.

  | arms | body | trot peak | stand peak |
  |---|---|---|---|
  | 28/25/14 (today) | 4.383 kg | 2.421 — over 24 % | 0.845 — over 19 % |
  | 34/32/20 | +70 g | 1.935 — margin **0.8 %** | margin 3.9 % |
  | 34/32/22 | +77 g | 1.938 — margin 0.6 % | margin 3.7 % |
  | **36/34/22** | **+95 g** | **1.827 — margin 6.3 %** | **margin 9.0 %** |

- **Decision (proposed):** **36/34/22.** The smallest set that clears the proxy
  does so by 2 % before the spiral and 0.8 % after it, which is not a margin.
  36/34/22 puts every joint of both legs inside peak and continuous, with the
  spiral closed, for **23.7 g of sheave per leg** — hip +8.7, knee +9.2, ankle
  +5.8.

- **Consequences:**
  - ✅ **No motor change, no girdle change, no re-assignment.** One shared set
    closes both legs, so `joint_moment_arm` does not need a per-leg form.
    ADR-0102's free hind re-assignment becomes unnecessary: at 36/34/22 the hind
    hip is 1.762 on today's spools.
  - ✅ The ankle at 22 mm clears the ~19–20 mm floor M106 found for
    [ADR-0050](#adr-0050)'s antagonistic pair to close at all.
  - Joint speed at 380 rpm falls to 554 / 587 / 907 °/s, still well above the
    gait's need. Worst spool travel rises to ~126 mm, well inside the spool.
  - ⚠️ **The cost is distal mass**, in a design whose premise is centralised
    actuators: +9.2 g at each knee and +5.8 g at each ankle ride on moving links.
    The swing-inertia cost is not priced here.  `[owed]`
  - ⚠️ **`leg_tendons` works in millimetres and `TendonParams` in metres.** The
    first sweep set `LT.ARMS` in metres, drew every sheave a thousand times too
    small and produced a plausible table that was wrong everywhere; the tell was
    the baseline row not reproducing ADR-0102. `arm_trade()` sets and restores
    it in millimetres.
  - Not adopted. Changing `joint_moment_arm` moves every sheave in the CAD, the
    MJCF plant, the tendon map and most of the budget tests.

## ADR-0104: the arms are adopted, the motor spec gains a speed axis, and the trot never fitted it
- **Status:** Accepted
- **Context:** [ADR-0103](#adr-0103) proposed shared moment arms **36/34/22 mm**
  (from 28/25/14) to bring the fore leg's actuator demand inside the motor proxy.
  It was adopted. The recommendation had been made on torque alone; the speed
  axis was measured only after the decision, and it is recorded here with the
  rest of what the adoption moved.

- **Decision:** keep 36/34/22, fold the heavier leg into the plant, and add
  **speed** to the motor specification the mechanism emits (ADR-0100). Under
  "mechanism first, motor is a proxy" the arms choose where the motor sits on its
  torque-speed curve; the spec states what that point demands.

- **Consequences:**

  ✅ **The specification, both legs, spiral closed** (`mass_closure.motor_requirement`):

  | | required | proxy |
  |---|---|---|
  | peak torque (fore knee, trot) | **1.825 N·m** | 1.95 — inside, 6.4 % |
  | continuous (stand) | **0.646 N·m** | 0.71 — inside, 9 % |
  | no-load speed (fore knee, trot) | **~665 rpm** | 380 — **1.75x short** |

  ✅ **NFR6 is met at both Kt corners**, for the first time since it was
  re-stated: **21.7 / 16.0 min, 650 m** against 14–20 min, 420–600 m. The arms
  are a reduction; copper goes as current squared and halves. Efficiency
  16.5 % → **30.7 %**, RMS current 1.38 → **0.94 A**, peak 4.34 → **2.79 A** —
  every electrical figure M107 put outside the proxy is back inside it.

  ⚠️ **THE TROT HAS NEVER FITTED THE PROXY ON SPEED, AT ANY ARMS THIS PROJECT HAS
  HAD.** `motor_spec_review.speed_check` sums three joints' foot speeds as if
  they aligned (~6 m/s) and compares that with a 0.5 m/s body. The constraint
  is each joint's **swing** speed. Checked against the motor's torque-speed line
  along the walked trajectory (`power.gait_envelope`), a 1.95 N·m motor needs
  **487 / 492 / 3563 rpm** at 28/25/14 and **624 / 665 / 474 rpm** at 36/34/22.
  Both exceed 380. The arms trade torque for speed; the proxy has too little of
  their product at any ratio. ⚠️ And the trajectory torque is far below the
  workspace-worst figure ADR-0008 sizes on (hip 0.74 against 2.20 N·m at
  28 mm), so the two sizing criteria pull the arms in opposite directions —
  static wants them large, the walked trot wants the hip and knee near 17 mm.
  This decision keeps the static case binding.

  - **Body 4.383 → 4.468 kg**, leg 187 → 208 g, both legs re-measured into
    `link_mass`, `link_com`, `link_inertia`. NFR5 exceeded by 10.5 %.
  - **Swing inertia about the hip +11.4 %** (1.3816e-3 → 1.5398e-3) — the price
    ADR-0103 left `[owed]`, now paid in the plant.
  - Cable land tension 650 → 516 N, cable SF 4.94 → 6.13, bearing C0 needed
    1277 → 1033 N. The worst coupled cable moves from the knee to the hip, so
    ADR-0042's free wrap-sense lever still cuts the knee 40 % but no longer
    moves the system's margin.
  - The MJCF drivetrain stiffens as `r^2`: shipped lowest mode 32 → 43 Hz,
    usable joint gain 100 → **200** (M86's halving undone). The hind ankle,
    the one pair over its continuous rating since M56, goes **1.19 → 0.79x** —
    `test_the_VARIABLE_RADIUS_PROFILE_CANNOT_BUY_what_ADR0002_wanted` had named
    this exact fix, a constant arm of 16.6 mm. NFR12's latency edge moves
    **15–20 → 30–40 ms**.

  ✅ **Resolved by [ADR-0105](#adr-0105): legs 125 kN/m, spine unchanged.**
  ⚠️ **G3's series spring should move 175 → ~100 kN/m, and this decision does
  NOT move it.** A spring in line with a cable appears at the joint as `k r^2`,
  so ADR-0026's 80–150 N·m/rad window is now met at **80–125 kN/m**; the shipped
  1.5e5 reads 171/154 N·m/rad. A first attempt re-scaled every spring in the
  project by 0.57 to hold joint compliance — and broke the shipped spool plant's
  servo-in-series tests, which had passed at 1.5e5 on the new arms, while not
  helping the plant it was aimed at. It was reverted. The shipped plant stands at
  1.5e5; re-speccing G3 is a mechanical decision with its own re-baseline.  `[owed]`

  ⚠️ **The LEGACY plant no longer stands on foot-force allocation.** It
  collapses 176 → 38 mm. The cause is M48's documented defect, not the arms: its
  fore-leg routing was never mirrored, so the fore hip pair does not oppose
  (+6.1 / +30.2 mm/rad at 28 mm, +9.4 / +36.4 at 36). At 28 mm the allocator
  worked round it; at 36 it cannot. The **shipped** transmission, whose fore map
  equals the hind's, stands — the 18-DOF spine quadruped holds under 0.05° of
  tilt. Three legacy tests are marked `XFAIL_M111_LEGACY` rather than retuned.

  ⚠️ **Two more checks were comparing the wrong scope.** `speed_check`, above.
  And `test_the_envelope_is_horizon_limited_and_must_be_converged` compared
  ONE direction's survival envelope with the viable set's minimum over ALL
  directions; it held only while the measured directions sat under a third
  direction's bound. Measured in the viable set's own tightest direction (240°),
  the simulated robot survives **147 %** of the bound at 28 mm and **190 %** at
  36 — ADR-0040's point that survival detaches from recovery, found where
  nobody had looked. Asserted as a defect.  `[owed]`

  - Thermal (`thermal/`): the anodised girdle's continuous equilibrium falls
    **119 → 102.8 °C** — still 23 K over the 80 °C line, so anodising alone is
    still not enough — and a gentle forced draught (h = 15) recovers it again,
    77.7 °C, which M41 had ruled out.
  - `[owed]`: the legs-plus-spine compliant righting slowed 2.02 → 2.62 s
    (penalty 1.49 → 1.94x) while rigid and legs-only did not move; not isolated.
  - `[owed]`: `standing_power` prices the workspace-worst pose with pretension
    and `gait_power` the walked trajectory without, so M16's stand/trot ratio
    (now 1.04) compares two different bases.

## ADR-0105: G3 is re-specified per cable group — legs 125 kN/m, spine unchanged
- **Status:** Accepted
- **Context:** G3 — the series-elastic element in every cable — is physically a
  torsional spring between the motor rotor and the spool, `k_tors = k * R_spool²`
  ([ADR-0051](#adr-0051)). Its requirement is not a spring rate but a **joint**
  stiffness: [ADR-0026](#adr-0026)'s 80–150 N·m/rad for balance compliance, which
  [ADR-0049](#adr-0049) confirmed from force control. A cable spring appears at a
  joint as `k r²`, so when [ADR-0103](#adr-0103) grew the leg arms 28/25/14 →
  36/34/22 mm, the shipped 150 kN/m moved the hip and knee to **171 / 154
  N·m/rad**, above the window. [ADR-0104](#adr-0104) left the re-spec `[owed]`.

- **Options, measured on the shipped spooled plant:**

  | leg G3 | hip / knee N·m/rad | cascade hind-ankle tracking | env joint-angle reconstruction |
  |---|---|---|---|
  | 150 kN/m (was) | 171 / 154 — above | passes | passes |
  | **125 kN/m** | **145 / 132 — inside** | **passes** | **passes** |
  | 100 kN/m (window centre) | 118 / 109 — inside | **6.1° off** at the gait's low end | **3.4°** off |

  100 kN/m is where the window is centred and it was rejected: the extra spring
  deflection is error the controller cannot see.

- **Decision:** **legs 125 kN/m, spine 150 kN/m — G3 is specified per cable
  group, from the joint it serves.** Both now live in `params.py`
  (`TendonParams.series_k`, `SpineParams.series_k`), and `env.py`, the MJCF
  builders, the tools and the tests read them from there; before this there were
  six literal copies.

- **Consequences:**

  ⚠️ **A single project-wide spring rate was a latent coupling, and it is what
  M111's first attempt tripped on.** `quadruped_rig` passed one value to the leg
  AND spine spools, so softening the legs softened the spine. The spine's arm
  never moved, and the spooled righting — which the spine performs — went
  2.62 s at 150 kN/m → **4.72 s** at 125 → **did not right in 11 s** at 100.
  With the spine held at its own 150 kN/m it rights in **2.34 s** — faster than
  M111's 2.62, a 1.71x penalty against the rigid plant instead of 1.94x. Driven
  correctly, the softer legs help the righting rather than cost it.

  ⚠️ **And the test harnesses carried the same coupling, in two places.** Four
  of them COMMANDED the spine rotors with the leg's `K_TORS`, and one READ spine
  spring deflection back into tension with it. With the plant split and the
  harnesses not, the righting read **4.30 s** and the spine "delivered" 16.3 N of
  19.6 — both artefacts of a controller that believed the wrong spring, which is
  the error `env._roll`'s docstring warns about in the other direction.

  The leg G3 part, at 125 kN/m on the 8.75 mm spool:

  | | |
  |---|---|
  | torsional rate | **9.57 N·m/rad** |
  | working deflection at the 1.95 N·m motor peak | **±11.7°** |
  | delivered through the 3000 rad/s rotor servo | **118.7 kN/m (95 %)** |
  | back-driven by the 516 N landing cable (4.5 N·m) | **27° — needs a hard stop** |
  | series-elastic mode, `√(k/I_rotor)` | 692 rad/s (110 Hz), was 121 Hz |

  - Legacy drivetrain lowest mode 75.8 → 69.5 Hz; shipped 43.3 → 39.8 Hz. The
    usable joint gain M111 recovered (kp 200) holds.
  - The ankle angle, unknown without a load cell, grows 1.41 → **1.69°** at the
    continuous rating and 3.87 → **4.64°** at the peak: the compliance ADR-0026
    asks for is paid for in state the encoders cannot see.
  - `wbc.rotor_command` now takes `k_tors` per tendon.
  - `[owed]`: the hard stop's angle and the spring's form (flexure, spiral,
    coil) are drawing decisions this does not make.

## ADR-0106: the arm cannot size the motor — it only chooses the axis, and the cable sets its floor
- **Status:** Accepted — **option 1: ADR-0008's static criterion stays, and the arms stay at 36/34/22.** The leg keeps the ability to hold trot load at any reachable pose; a cat that jumps, falls and rights itself leaves the nominal trot often, and the price lands on a motor this project finds or designs later (ADR-0100): **~1.9 N·m / ~665 rpm**. The ankle's ~26 mm optimum is not taken — under option 1 the knee, not the ankle, sets the speed.
- **Context:** [ADR-0104](#adr-0104) found the walked trot needs ~665 rpm from a
  1.95 N·m motor at the 36/34/22 arms, against the proxy's 380, and that
  ADR-0008's static criterion and the walked trajectory want the arms in opposite
  directions. This asks what the arm can do about it.

- **Findings** (`structure_matrix.arm_speed_trade`):

  ✅ **The motor's size does not depend on the arm.** Seen from a joint a motor
  offers torque ∝ `r` and speed ∝ `1/r`, so the product that sizes a motor —
  `T_pk · ω_nl` — is fixed by the loads, and the arm only splits it. Swept 12–40
  mm, the smallest product meeting every criterion moves by 2 %: hip 115–118,
  knee 125–128, ankle exactly 90.6 (N·m·rad/s). The proxy has **77.6**.

  ⚠️ **What sets the size is the criterion, and ADR-0008's static one dominates
  the hip and knee:**

  | joint | static + walked | walked trot only | proxy |
  |---|---|---|---|
  | hip | 117 | **50.7** | 77.6 |
  | knee | 128 | **69.4** | 77.6 |
  | ankle | 90.6 | 90.6 | 77.6 |

  The worst REACHABLE pose asks 2.3x (hip) and 1.8x (knee) the motor the
  walked trot does. Against the walked trot alone the proxy is enough at both;
  only the ankle is short, by 17 %, and there the trot itself is the reason.

  ⚠️ **But the arm has a third floor, and it is the cable's.** The landing
  transient sizes cable and bearing (ADR-0008 keeps it out of the actuator
  envelope for exactly that): cable SF ≥ 4 and bearing C0 ≥ 2T both land at
  **~25 mm** for the hip and knee — at 25 mm the hip reads SF 4.05 and C0
  1482/1500 N. Below it the motor is irrelevant.

  So, with a 1.95 N·m motor, the speed the walked trot needs:

  | arms | cable SF | rpm needed (hip/knee/ankle) | worst |
  |---|---|---|---|
  | 36/34/22 (ADR-0103, static kept) | 5.8 / 6.0 / 8.0 | 624 / 665 / 474 | **665** |
  | 34/32/26 (static floor) | 5.5 / 5.3 / 9.4 | 590 / 627 / 445 | **627** |
  | 28/28/26 | 4.5 / 4.7 / 9.4 | 487 / 550 / 445 | **550** |
  | 25/25/26 (cable floor) | 4.05 / 4.16 / 9.4 | 435 / 493 / 445 | **493** |

  ⚠️ **No arm brings the proxy's 380 rpm inside.** At the cable floor the walked
  trot still needs ~490 rpm; the ankle alone needs 445 at its own optimum
  (~26 mm), below which a 1.95 N·m motor runs out of torque rather than speed.
  The arm decides whether the speed shortfall is 1.3x or 1.75x; it cannot make
  it 1.0x. That takes a faster motor or a gentler gait.

- **Options** — this is ADR-0008's to decide, not the arm's:
  1. **Keep ADR-0008's static criterion** (the leg can hold trot load at any
     reachable pose). Arms stay near 36/34/22; motor spec ~1.9 N·m / ~665 rpm.
  2. **Size the actuator to the walked gait, keep the landing cable.** Arms
     ~28/28/26 for margin on cable and bearing; spec 1.95 N·m / ~550 rpm; the
     leg can no longer hold a full trot load at its worst reachable pose.
  3. The same at the cable floor (25/25/26): ~490 rpm, with cable and bearing at
     their limits.

- **Consequences (whichever is chosen):** the ankle's arm has an interior
  optimum near 26 mm on the walked trot, independent of ADR-0008; the current 22
  costs it 30 rpm it does not need. `[owed]` — standing (`standing_power`) is
  also priced at the worst reachable pose, so relaxing ADR-0008 for the trot
  without doing the same for the stand would leave the two inconsistent.

## ADR-0107: G3 is a bidirectional planar flexure at the spool, with a hard stop just past the motor's peak
- **Status:** Accepted (form, material class and stop angles); drawn in M117; **the drawing, its stop and its mass superseded by [ADR-0111](#adr-0111) (M120)**
- **Context:** [ADR-0105](#adr-0105) fixed G3's rates — 125 kN/m on the leg
  cables, 150 kN/m on the spine — and left the part's form and hard stop open.

- **Method:** by energy (`mechanical/cad/g3_spring.py`). A spring must store
  `F²/2k` at its design load, and each form's material holds a bounded energy
  per cubic millimetre before it yields or fatigues; the volume is the part.
  Load cases from the plant: fatigue 0 → **140 N** every step (the walked
  trot's worst cable, the hind ankle), working **222.9 N** (the motor's peak),
  landing **516 N** (the hind hip's transient).

- **Findings and decision:**

  ✅ **The hard stop is what makes the spring buildable.** Without one the
  spring absorbs the landing, **1.065 J** at 125 kN/m; with a stop at 1.15× the
  motor's peak it holds **0.263 J** and the housing takes the landing. **Stop at
  ±13.4° on the leg springs and ±11.2° on the spine** (working ±11.7° / ±9.7°).

  ⚠️ **G3 cannot live in the cable line.** 125 N/mm over 2 mm of travel is too
  stiff to wind: three or more active coils forces a wire over 4.6 mm, and no
  helical compression spring fits even at 40 mm outside diameter. Rotated to the
  spool the same energy is ±13° of twist, which is where ADR-0051's model already
  put it — rotor, spring, spool.

  ⚠️ **And it has to be BIDIRECTIONAL.** One spool drives a joint's pair
  (ADR-0008's motor count), so the torque behind it reverses sense with the
  loaded cable and the fatigue is fully reversed. A helical torsion spring — the
  lightest form on a one-way load, 3.5 g, d 2.7 × D 11 × 1.6 turns — is strong
  winding and weak unwinding. A planar torsional flexure is symmetric by
  construction.

  | flexure material | stress at stop, bound | leg G3 active / part | spine G3 |
  |---|---|---|---|
  | 17-4PH H1025 | 600 MPa, yield | 6.7 / ~13.5 g | 5.6 / ~11 g |
  | **maraging C300** | 894 MPa, fatigue | **3.0 / ~6.0 g** | 2.5 / ~5.0 g |
  | **Ti-6Al-4V** | 528 MPa, yield | **2.9 / ~5.7 g** | 2.4 / ~4.8 g |

  **Decision: a planar torsional flexure in Ti-6Al-4V or maraging C300**, ~6 g
  per part, between rotor and spool, with angular stop lugs at the angles above.

- **Consequences:**
  - **Mass `[owed]`: ~156 g as drawn** (M117, below) — twelve leg parts at
    8.26 g and six spine parts at 9.52 g, against ~100 g estimated here — not
    yet folded into `params.py`. It will move the body 4.468 → ~4.62 kg and the
    spiral with it.
  - Axial space: a ~4 mm disc plus lugs, ~5 mm per actuator. The trunk rows
    have 16.4 mm spare per two-row body and 6.9 mm in the one-row spine body 2 —
    it fits, tightly there.
  - The stop carries the landing's excess, ~2.3 N·m at the leg spool. Its
    impact is `[owed]`.
  - `test_g3_spring.py` asserts the four findings above.

- **Drawn (M117, `mechanical/cad/g3_flexure.py`, `.step`/`.stl`):** one
  Ti-6Al-4V piece — hub (bolted to the rotor), three Archimedean arms of 0.75
  turn, a thin rim with three bolt ears (bolted to the spool), and the hard stop
  in its own layer above the arms.

  | | leg, 125 kN/m | spine, 150 kN/m |
  |---|---|---|
  | rate at the spool | 9.57 N·m/rad | 11.5 N·m/rad |
  | envelope | Ø30 disc, 34 mm over the ears | same |
  | arm, in-plane × axial | 1.60 × 3.74 mm, 45.6 mm long | 1.60 × 4.48 mm |
  | stress at the stop / bound | 469 / 528 MPa | 391 / 528 MPa |
  | stress, walked trot / bound | 257 / 357 MPa | 214 / 357 MPa |
  | hard stop | ±13.4° (hub lugs 123°, rim lugs 30°) | ±11.2° |
  | axial length | 5.24 mm | 5.98 mm |
  | mass | **8.26 g** | **9.52 g** |

  ⚠️ **The stop cannot share the arms' plane**: three arms of 0.75 turn sweep
  every angle between hub and rim, so no hub feature can meet a rim feature
  there. It sits 0.3 mm above the arms. The wide lugs are on the hub, where the
  same free play costs a third of the material it would at the rim.

  ⚠️ **The part is ~45 % over the energy estimate** (8.3 vs ~5.7 g): the arms
  are the active 2.9 g, and the hub, rim, ears and stop layer are the rest.

  ⚠️ **The rate is beam theory** (`k = N E T b³ / 12ℓ`, `σ = E b θ / 2ℓ`).
  Clamped-clamped arms stiffen the real part; the rate is `[owed]` to an FEA
  or a bench coupon before it is cut. The spine body 2 row (6.9 mm spare) keeps
  0.9 mm after the 5.98 mm spine part.

  `test_g3_flexure.py` asserts both stresses, the stop's free play, the spine
  variant, and a valid single solid inside the 34.5 mm can at ~8.3 g.

## ADR-0108: standing is priced at the stance actually held, and the thermal worst case is the trot
- **Status:** Accepted
- **Context:** `power.standing_power` took `torque_budget`'s standing case — the
  worst pose over the whole reachable workspace, pretension included — and
  charged it to every joint of every leg at once, while `gait_power` averaged
  the trajectory actually walked. M16's stand/trot ratio, ADR-0021's brake
  argument and ADR-0023's "standing is the worst thermal case" all compared those
  two bases. M111 saw the ratio cross 1.0 for no physical reason and left it
  `[owed]`. [ADR-0106](#adr-0106) kept the worst-pose criterion for the actuator's
  PEAK; temperature is a different question — the load actually held, as M39
  already said of the trot.

- **Decision:** price standing at the stance. The weight is split fore/hind by
  the CoM's lever over the feet (**32.5 %** fore, against **30.2 %** measured on
  the MJCF quadruped), `J^T f` at the stance pose, and the same
  `|tau| / r * R * capstan` the trot uses. The old basis survives as
  `standing_power_worst_pose`. The motor spec's **continuous** figure becomes the
  larger of the stance hold and the walked trot's RMS.

- **Consequences:**

  | | worst reachable pose (was) | stance actually held |
  |---|---|---|
  | leg copper, standing | 105.9 W | **31.7 W** |
  | per motor | 8.82 W | **2.64 W** (trot 5.85) |
  | stand / trot | 1.04 | **0.31** |
  | standing runtime | 20.8 min | **54.0 min** |
  | brake's multiplier on it | ~4x | **3.1x** |
  | continuous motor spec | 0.646 N·m | **0.624** (set by the trot's fore ankle RMS; the stance hold is 0.495) |

  ⚠️ **Three standing claims were properties of the basis, not of the robot.**
  M16's "standing costs most of what moving costs" is a third. ADR-0023's
  "standing is the worst thermal case" inverts — **the trot is**, because four
  feet share what two carry in a trot. And NFR17's brake, "required, not
  optional" on the 76 % figure, is still worth 3.1x of standing endurance and
  no longer rests on that argument. A cable still only pulls; posture still
  costs current for zero work; it costs less than walking does.

  ✅ The continuous spec barely moves (0.646 → 0.624) but its reason does: it is
  the trot, not the stand. The thermal crate's tests follow — its own worst-case
  test is renamed `trotting_not_standing_is_the_worst_thermal_case`.

## ADR-0109: the legacy wrapped plant is removed
- **Status:** Accepted
- **Context:** `mjcf_tendon` still built the plant M42–M53 measured: spatial
  tendons RESTING on the sheaves (`clamped=False`), one pull-only motor per
  cable, optionally with cable stretch (`*_rig_elastic`). [ADR-0055](#adr-0055)
  clamped the cables and M54 made the pulley transmission the default; since
  then the old plant survived only as a comparison baseline. It carried M48's
  unmirrored fore-leg routing, and at [ADR-0103](#adr-0103)'s 36 mm arms that
  defect stopped it standing at all (`XFAIL_M111_LEGACY`). Options were to fix
  its fore routing (an M111-sized re-baseline of a plant nothing ships), close it
  as won't-fix, or remove it.

- **Decision:** **remove it.** The `clamped` and `elastic` switches, the
  wrapped-routing branch of `leg_tendon_xml`, `single_leg_rig_elastic` and
  `quadruped_rig_elastic` are gone from `mjcf_tendon.py`. Every cable is clamped;
  the one-motor-per-cable variant (`pulley=False`) stays, because it does not
  carry the defect.

- **Consequences:**
  - `test_mjcf_tendon.py` 113 → **66 tests.** 47 depended on the legacy plant,
    found by a scope-aware call-graph pass rather than by name. Their findings
    are not lost: each is recorded in the ADR that made it (ADR-0042–0058), and
    the tests are in git history before this commit.
  - **Four mixed tests kept their shipped half** — the default plant is the
    pulley transmission; the drivetrain exists behind the pulley with G3; the
    whole-body model carries no G3; the shipped lowest mode (39.8 Hz) and its
    kp headroom. Only the legacy comparison line in each was cut. ⚠️ So the
    claim "one spool per pair HALVES the lowest mode" no longer has its
    denominator in a test; ADR-0060 and M111/M112 record it (63–76 Hz legacy).
  - ✅ **G3's sizing was the one live conclusion resting on the legacy plant** —
    its joint-stiffness band was measured on the elastic wrapped leg, the only
    model with cable stretch. It is re-derived analytically,
    `r² / (1/k + 1/(EA/L))` on the router's run lengths
    (`test_G3_puts_the_HIP_and_KNEE_inside_ADR0026s_window`): shipped 125 kN/m
    gives hip 138 / knee 118 N·m/rad, inside; 150 kN/m puts the hip at 160.
  - ⚠️ The shipped plant does not stretch its cables. That was already true of
    the clamped transmission; it is now the only plant, so cable stretch exists
    only in `_cable_k`'s analytic price.
  - The fore-routing defect (M48) is closed by removal rather than by a fix.

## ADR-0110: the speed is bought by the winding — a rewound 3505 at 9:1, not a lower ratio
- **Status:** Accepted (the route); the pack's cell count and the part are `[owed]`
- **Context:** [ADR-0104](#adr-0104) added speed to the motor spec: the walked
  trot needs **~665 rpm** no-load at the spool against the proxy's 380, and
  [ADR-0106](#adr-0106) showed no moment arm closes it. M119 asks what motor
  delivers it and keeps the proxy's envelope, mass and thermal budget.

- **Findings** (`tools/motor_spec_review.speed_routes`, `power.gait_envelope`):

  ✅ **The speed is the SWING, and peak torque does not buy it.** The walked
  trot's largest motor torque is the ankle's 1.23 N·m; its fastest joint is the
  knee swinging at 653 rpm and carrying almost nothing. The no-load speed needed
  is 665 rpm at a 1.95 N·m peak and still 660 at 3.5. Torque and speed are two
  independent axes of the spec — a stronger motor is not a faster one.

  ⚠️ **The stock -9 cannot reach it on any bus.** Its sheet gives
  **15.83 rpm/V at the output** (× 24 V = the 380 rpm quoted) and a **12–40 V**
  range: **633 rpm at the ceiling itself**, and at a LiPo's loaded floor
  (3.5 V/cell `[assumed]`) 332 / 443 / 499 rpm on 6S / 8S / 9S. ⚠️ `power.NO_LOAD_RPM`
  is quoted at 24 V, above a 6S floor, so every speed check against it is
  optimistic on top of being short.

  ❌ **A lower reduction reaches the speed and loses the continuous rating.**
  Speed goes as `9/N`; output Kt and the rated torque go as `N/9`:

  | pack | N for 665 rpm | rated / peak | copper at a given torque |
  |---|---|---|---|
  | 6S | 4.5 | 0.36 / 0.98 N·m | ×4.0 |
  | 8S | 6.0 | 0.47 / 1.30 N·m | ×2.3 |
  | 9S | 6.7 | 0.53 / 1.46 N·m | ×1.8 |

  against **0.624 continuous / 1.825 peak** required. It fails both, everywhere.

  ✅ **A rewind reaches it and costs only peak current.** Same frame, same 9:1,
  fewer turns of thicker wire: at equal copper fill `Kt/√R` is unchanged, so the
  continuous rating (0.71), the trot's copper loss, NFR6's runtime and NFR18's
  thermal all stand. Only the current per N·m rises:

  | pack | Kv at the output | × stock | peak current (at 1.95 N·m) |
  |---|---|---|---|
  | 6S | 31.7 rpm/V | 2.00 | 8.4 A |
  | **8S** | **23.8 rpm/V** | **1.50** | **6.3 A** |
  | 9S | 21.1 rpm/V | 1.33 | 5.6 A |

- **Decision:** the actuator spec is **a 3505-class frame at 9:1, wound for
  `Kv_out ≥ 665 rpm / V_floor`** — Ø34.5 × 36.1 mm, ≤ 132 g, peak ≥ 1.95 N·m,
  continuous ≥ 0.71 N·m (0.624 needed). Recommended bus **8S**: it charges to
  33.6 V inside the 12–40 V range with margin, and needs a 1.5× rewind at
  6.3 A peak where 6S doubles the driver's current. The cell count is an
  electronics decision and is left to it; the rule holds for any.

- **Consequences:**
  - The part is a custom winding of a stock frame, which SteadyWin and its
    peers sell as a variant (`[owed]`: a quote and a measured Kv). A custom
    motor must meet the same four numbers; nothing in the mechanism changes.
  - Driver and battery carry the 1.5× current: 6.3 A peak per phase, and the
    pack's peak current rises with it. Driver conduction losses rise as its
    square and are outside `ELECTRONICS_W`'s flat 15 W `[owed]`.
  - Equal fill is an ideal; a rewind with fewer, thicker turns usually loses
    a few percent of `Km`, which NFR6 and NFR18 would pay as copper `[owed]`.
  - `power.NO_LOAD_RPM` and `PEAK_NM` stay the PROXY's (the trot still does
    not fit them — `test_the_trot_has_never_fitted_the_proxy_on_SPEED`); the
    spec lives in `speed_routes` and this ADR until a part is chosen.
  - `test_motor_spec.py` asserts the swing finding and all three routes.


## ADR-0111: G3 goes in the trunk — thinner, with its stop in the spool, and the body carries it
- **Status:** Accepted
- **Context:** M117 drew G3 (ADR-0107) as a part and left it out of the robot:
  no row was longer than motor + spool, and its ~156 g was `[owed]` to the
  mass budget. ADR-0107 had said the trunk had room — "16.4 mm spare per
  two-row body and 6.9 mm in the one-row spine body 2".

- **Findings:**

  ⚠️ **It did not fit.** ADR-0107's spare left out the bulkhead pads (2.1 mm
  each end of a row). With them a girdle body has **8.0 mm** for two rows and
  spine body 2 **2.7 mm** for one. The M117 part needed 5.24 mm (leg, ×2 =
  10.5) and 5.98 mm (spine): neither fitted.

  ✅ **The arm WIDTH was a free choice, left with stress to spare.** Rate goes
  as `T b³` and stop stress as `b`, so at a fixed stress the ACTIVE volume is
  the energy — 714 mm³ leg, 595 spine — and the shape only trades width for
  thickness. Widened to 95 % of the stop-stress bound (1.60 → 1.71 mm leg),
  the leg part goes 3.74 → 3.06 mm; the spine, with two arms of a full turn
  (`T ∝ 1/l²` at a fixed stress), 4.48 → 1.83 mm.

  ✅ **The stop leaves the part.** It was a 1.5 mm layer of its own (three arms
  sweep every angle, so it cannot share their plane). Now two hardened Ø2
  dowels through the hub, at 90° to its two bolts, ride arc slots in the
  spool's face — inside the spool's existing 8 mm. The landing's whole spool
  torque on them: **602 N a pin, 192 MPa shear**; bearing 98 / 165 MPa in the
  leg / spine hub.

  | | leg | spine |
  |---|---|---|
  | arms × turns | 3 × 0.75 | 2 × 1.0 |
  | arm, in-plane × axial | 1.71 × 3.06 mm | 2.71 × 1.83 mm |
  | stop / trot stress (bounds 528 / 357) | 502 / 275 MPa | 502 / 275 MPa |
  | radial gap between arms | 1.89 mm | 0.84 mm |
  | in the row (± 0.3 mm clearance) | **3.66 mm** of 4.0 | **2.43 mm** of 2.7 |
  | mass | **5.20 g** | **3.86 g** |

  ⚠️ **Fusing it broke a body.** Body 1's longer row put its end bulkhead
  0.36 mm into a process post, and the post's fuse destroyed the body
  (`_fuse` caught it). The bulkheads now go in last with the cable clearance
  already cut from them — 1.5 mm off the post, and the clearance kept.

- **Decision:** G3 is in every leg and spine row (`tomcat_trunk.G3_STACK`,
  `row_len`); the bodies are NOT lengthened. The mass is folded in:
  **85.6 g** (12 × 5.20 + 6 × 3.86), leg parts on the girdles at their motors
  (`girdle_inertia.g3_parts`), spine parts on `segment_mass[1]`.
  **Body 4.4684 → 4.5540 kg.**

- **Consequences:**
  - Motor peak 1.825 → **1.859 N·m** (4.7 % under the proxy), continuous
    0.624 → 0.635, speed unchanged at 665 rpm; runtime 21.66 / 16.04 →
    **21.14 / 15.60 min**, range 650 → 634 m; hip landing
    18.41 → 18.76 N·m at 526 N; the femur's SF 2.59 → **2.54**, 0.04 above
    the 2.5 line — the thinnest margin in the leg.
  - The longer rows moved every spool centre ~1.8 mm, so `pair_wrap` is
    re-solved on the trunk's spools: hip 148.3 → 145.7°, less wrap on five of
    six cables.
  - The spine's saturated peaks FELL under the added mass — righting
    7100 → 6022 N (27× the rating), the 0.30 m drop 1798 → 1612 N (7.2×): a
    transient's peak moving 15 % on 2 % of mass. The multiple is the
    finding; the newtons are not robust.
  - `[owed]` — the spool: its arc slots, and its own bearing on the rotor
    axis (the flexure carries torque only); which end of the row it is on
    (`g3_parts` places G3 at row centre).
  - `[owed]` — the spine part's 0.84 mm arm gap and the rate are beam theory:
    FEA or a bench coupon before cutting.
  - ⚠️ **Found on the way, and not G3's:** the balance sim's survival is not
    monotonic in the push — at 120° it survives 25.8 mm and falls at 15.3 —
    so every bisected envelope is an upper bound of unknown slack.
    `test_mjsim` asserted "within 30 % of optimal" off a bisection; it passed
    at 31.1 mm by the path the bisection took and read 16.3 after this
    change on the same holed boundary. Withdrawn, and the holes asserted as a
    defect. Re-defining the envelope is `[owed]`.


## ADR-0112: the leg drive in 3-D — friction where the cable slides, spine joint 0 moved forward, and the hip crossing still owed
- **Status:** Accepted, with a **known defect**: the knee and ankle conduits cut
  the hip sheave. The hip region's redesign (a hollow hip) is M123.
- **Context:** the leg's tendon WIRING was questioned, and it had never been
  checked in three dimensions. M122 looked, with the leg CAD at its joint limits,
  the trunk's real motors, and the leg's sweep over its full range.

- **Findings — defects, all fixed:**
  - ⚠️ **The drive was a 2-D projection.** M88 laid the motors fore-aft: each
    spool is inside the trunk at y = ±20.2 on an axis along x and winds across
    the body. Every router, drawing and friction figure used it as a spool in the
    leg's plane (y 65–75) on an axis along y — 45–55 mm and 90° away. The
    assembly's "cable ends on its spool" check compared the router's first point
    with the coordinates it had been given, so it could not fail.
  - ⚠️ **The fore leg's cables were solved on the HIND leg's links**
    (`tendon_drive` called the router without `leg=`): 12–26 % of their length
    ran through parts, at every pose.
  - ⚠️ **The CAD drew an ankle return spring** (ADR-0002 option B) on a leg whose
    ankle has been an antagonistic pair since ADR-0055, and no ankle extensor.
  - ⚠️ **The anchor pin was a hind-leg rule of thumb** (`0.55 r` along the link,
    `r + 5` beside it); on the fore leg it wound the ankle extensor 346°. Each
    cable now ENDS on its joint sheave, anchored to keep 15° wrapped at the joint
    limit that unwinds it (`leg_tendons.ANCHOR_MARGIN`); the wrap moves
    one-for-one with the joint, so the arms stay exactly 36/34/22.
  - ⚠️ The trunk's end bulkheads sat 0.3 mm inside their rows since G3 filled
    them (M120); flush with the body's end now.

- **Findings — the hip crossing.** Over the full joint range the knee and ankle
  sheaves and the vias sweep nearly all of each cable's plane around the hip.
  Three ways across were tried:
  1. an exit idler in each cable's plane — the ankle cable had one window, whose
     spools sat inside spine joint 0's yoke; and an idler that turns a lateral
     lead stands 8.75 mm out of the plane, into the femur's sweep;
  2. along the hip axis (hollow shaft, turning pulleys on the femur) — each turn
     off the axis needs 2R = 17.5 mm inboard of its plane: through the hip
     sheave, and through the other three turns;
  3. Bowden conduits (chosen) — whose femur ends still cross the hip sheave's
     plane inside its rim.

- **Decision:**
  - **Spine joint 0 moves 30 mm forward of the hind hips**
    (`SpineParams.rear_hip_x = -0.030`; `hip_offset` on the hind mounts, both
    MJCF builders). The trunk grows 363 → **393 mm**, the hips 195 → 225 mm
    apart; the hind hip sits in full section (its boss cantilever 27 → 12 mm)
    and no spool sits in spine hardware any more. This was `tomcat_trunk`'s
    `[owed]` sacrum.
  - **The drive is Bowden conduits** (`mechanical/cad/tendon_exit.py`,
    `DRIVE`): each cable leaves its spool tangentially in the spool's x-plane,
    enters a conduit at a ferrule on the trunk wall, and leaves it at a trunk
    bracket above the hip (hip pair) or a ferrule on the femur (knee and ankle
    pairs) — which decouples the hip from those cables (the hip via's
    8.75 mm/rad is gone, and so is the via). Motor assignment and spool ends are
    solved, and moved on both legs. Conduit Ø3, bend radius ≥ 15 mm `[assumed]`.
  - **Friction is counted where the cable slides** (`TendonParams.friction_model
    = "drive"`): `exp(mu_c · conduit bend)` at mu_c 0.07 `[assumed]`, times one
    running pulley's efficiency 0.97 `[assumed]` (the knee via, ankle pair);
    anchored sheaves and the spool cost nothing. M107–M121 charged a capstan on
    every degree of wrap — on sheaves the cable is anchored to and pulleys that
    turn on bearings; that was 71 % of the trot's copper in M107. Factors now
    ×1.12–1.22 against ×1.29–1.56.

- **Known defect** (`test_tendon_exit`): **every knee and ankle conduit cuts the
  hip sheave, 16–39 mm into it.** Moving the femur ferrule out, and clipping
  the conduit to the femur, found no path either: between the trunk wall and the
  femur the hip boss (r 12, y 41–65), the femur's root and the hip sheave
  (r 38, y 62–68) leave a 15 mm-radius conduit no way through, at the stance
  alone. It is the HIP's packaging, not the routing; M123 redesigns it hollow.

- **Consequences** (re-measured, not scaled):

  | | M120/M121 | M122 |
  |---|---|---|
  | body | 4.554 kg | **4.501 kg** (legs −13 g each: no hip via, spring, pins) |
  | motor peak (binding) | 1.859 N·m, fore knee | **1.623 N·m**, fore hip ≈ knee |
  | continuous | 0.624 N·m | **0.575 N·m** |
  | no-load speed needed | 665 rpm | **629 rpm** (8S rewind ×1.42, 22.5 rpm/V) |
  | runtime (Kt 0.44 / 0.35) | 21.1 / 15.6 min | **24.2 / 18.2 min**, ~725 m |
  | anodised girdle, continuous | 102.8 °C | **91.6 °C**; forced air h 15 → 69.4 °C |
  | trot foothold x | 0.00214 m | **0.00722 m** (re-tuned: roll drift +0.214 rad/s/cycle) |
  | fore share of the stance | 0.302 | **0.371** |
  | viable set, worst / with spine | 34.9 / 66.4 mm | **32.9 / 65.3 mm** |
  | friction → ROM crossover | mu 0.8725 | **mu ~0.895** |
  | soft-spine cliff | spine_kp 60 | **45** |
  | diagonals' angle | 57.1° | **50.5°** |

  The arms: 0.85× now fits the motor (93 %); the motor pins them only at ~0.8×.
  - ⚠️ **One finding reversed: the leg tuck no longer helps the righting**
    (118 → 85 °/s with it). M102 had it down to a 2 % margin once the head and
    tail gave the spine its ends; with the hind hips behind spine joint 0,
    folding the legs costs more than it gives. The rigid-spine righting is
    faster, 1.35 → 1.08 s.
  - The stance's weight moved toward the fore legs, 0.302 → 0.379 in the MJCF:
    the standing cable force rose 24.2 → 32.3 N, the fore pairs' continuous
    ratings to 0.34 / 0.55 / 0.49, the hind ankle's fell to 0.70. Nothing is
    over its rating.
  - Also re-measured: the survival holes (still there: 0.14 m/s falls, 0.16
    survives at 120°), the envelope's direction span 2.9×, the soft-spine drop
    ratio 4.1 → 2.4×, skin strains down with the longer cover (belly 34.7 →
    32.1 %), the tail's share of pitch inertia 0.117 → 0.154.
  - `[owed]` — the hollow hip (M123); then the conduits' real shape and
    clearance (a Bezier here), the shell's conduit ports, the skin's aperture,
    and the ferrule brackets (a conduit's compression equals its cable's tension:
    the femur ferrules carry it into the femur).
  - `[owed]` — mu_c and the pulley efficiency are assumed; a bench measurement
    settles them.
  - `[owed]` — found on the way and not fixed here: the FORE leg's bones were
    never checked; its radius tube (Ø12) is SF 1.77 at landing (the fore knee
    carries 18.3 N·m to the hind's 11.5). The paw pad parts from the leg at two
    ROM extremes.

---

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
