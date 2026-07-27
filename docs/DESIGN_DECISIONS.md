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

## ADR-0005: Compute & motor-drive topology
- **Status:** Accepted
- **Context:** ~30 tendon motors now (24 legs + 6 spine), ~33+ with the tail —
  each needing FOC, closed-loop tension, a rotor sensor, and a tension signal;
  plus ≥1 kHz motor loops (NFR3) and ≥100 Hz planning/righting (NFR4).
- **Options:** single MCU (rejected — cannot host ~30 FOC loops *and* planning);
  centralized multi-axis controller; **distributed smart drivers on a real-time
  bus** + a compute split.
- **Decision:** **Distributed FOC smart drivers — one per tendon motor —
  clustered in the shoulder & pelvic girdles and a tail node (falls out of P1),
  on a CAN-FD real-time bus (~5 Mbit/s, split across ~6–8 segments), under a
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
  sensor + CAN-FD + ADR-0004 tension front-end), two girdle backplanes + tail
  stub, a multi-CAN-FD bridge, and the hardware e-stop; firmware owns driver FOC
  + tension + Tier-A safety, RT-tier aggregation + Tier-C watchdog, and SBC ROS 2
  nodes. ⚠️ The ~6–8 CAN-FD segment count is an extrapolation from the mjbots
  12-axis result — **bench-verify ≥1 kHz per segment** at final axis count.

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
