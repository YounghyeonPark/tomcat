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
- **Consequences:** Sets motor/driver-channel count (~2 channels per antagonistic
  DOF, ~1 for spring-return). The tendon map must accept a per-joint `T_bias` and
  implement the AIC split (M1 task K1). Stiffness becomes commandable rather than
  fixed by hardware.

## ADR-0003: Actuator technology  ❓ Proposed
- **Status:** Proposed
- **Context:** Need backdrivable, controllable rotary actuators.
- **Options:** BLDC + FOC (best backdrivability & control, most complex);
  geared DC (simpler, more friction/backlash); integrated servo modules
  (fastest to build, least tunable).
- **Decision:** *Undecided, split by subsystem.* Tendon-drive is committed for
  the **spine and for centralizing limb mass** (P1). For the **legs**, a blanket
  "tendon everywhere" stance is not justified: the MIT Cheetah's backdrivable
  proprioceptive direct-drive achieves an Impact Mitigation Factor comparable to
  series-spring designs ([LITERATURE_REVIEW.md](LITERATURE_REVIEW.md) Q5).
  **Add a required leg-actuator trade study — tendon-drive vs. compact
  backdrivable direct-drive — scored on IMF, torque density, reflected inertia,
  and control complexity, using the M1 leg torque budget as input, before
  finalizing leg actuators.** Revisit BLDC+FOC vs. geared DC after that study.
  **Not fully sensorless:** a static high-torque stance hold cannot be run
  reliably encoderless — evaluate a **non-backdrivable reduction or a
  brake/latch** to offload the electrical DC hold, and treat that hold as a
  thermally-derated continuous-torque operating point ([sensorless-FOC note](notes/sensorless-foc-stance-hold.md)).

## ADR-0004: Tension & position sensing method
- **Status:** Accepted (rotor sensor); tension method still Proposed
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
  - **Tension sensing method still undecided** (load cell vs. current estimate
    vs. series-elastic), tied to FR2 accuracy needs — see the Q1b options in
    [LITERATURE_REVIEW.md](LITERATURE_REVIEW.md) (Kengoro load cell; compact
    single-pulley + 3D-Hall module).

## ADR-0005: Compute topology  ❓ Proposed
- **Status:** Proposed
- **Options:** single MCU (simple, must hit ≥1 kHz loops + planning); RT-MCU for
  motor loops + SBC for planning (clean separation, adds a bus + comms).
- **Decision:** *Undecided.* Depends on final DOF count and loop budget.

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

## ADR-0007: Mid-air righting via an inertial tail (+ spine twist)
- **Status:** Accepted
- **Context:** Mid-air righting (landing feet-first) is now an in-scope goal
  (G6). Reorientation conserves angular momentum via shape change; the question
  is which appendage provides it.
- **Options:** (a) **spine axial twist** only — most cat-like, reuses the spine,
  but limited authority; (b) **leg/limb shape-change** — no new hardware, proven
  in sim for roll+pitch; (c) **dedicated inertial tail** — highest reorientation
  authority.
- **Decision:** Primary mechanism is a **dedicated inertial tail**, ideally
  **morphable (telescoping)** to maximize authority during flight then retract
  before touchdown; the spine's **axial-twist DOF is retained as a complementary
  contributor**. Rationale: the "Inertial Reorientation template" analysis finds
  **tails outperform limbs and spine/body-bending** for aerial reorientation, and
  a 3-DoF morphable tail has self-righted a real quadruped in the flight phase
  ([LITERATURE_REVIEW.md](LITERATURE_REVIEW.md) Q4). Rotary-actuator-only 180°
  righting is proven, so no reaction wheels/thrusters are needed.
- **Consequences:** Adds a **tail subsystem** (actuator(s), possibly a
  length-change mechanism) to the architecture and BOM, a flight-phase
  reorientation control law, and a later righting milestone. The spine must keep
  its axial-twist DOF (ties back to ADR-0006's ~3-DOF-per-segment target).

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
