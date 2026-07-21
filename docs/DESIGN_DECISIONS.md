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

## ADR-0002: Antagonistic actuation vs. return spring  ❓ Proposed
- **Status:** Proposed
- **Context:** A cable can only pull, not push. Each DOF needs a way to move in
  both directions.
- **Options:**
  - **A. Two antagonistic tendons** (two motors per DOF): full active control of
    both position and stiffness; ~2× motors, wiring, and mass.
  - **B. One tendon + passive return spring:** fewer motors; stiffness fixed by
    the spring, and the return direction is not actively driven.
- **Decision:** *Undecided.* Likely per-joint (e.g. antagonistic at hip/knee,
  spring-return at ankle). Needs mechanical review.
- **Consequences:** Drives motor count (NFR2), driver-channel count, and the
  tendon-map math in the mid-level controller.

## ADR-0003: Actuator technology  ❓ Proposed
- **Status:** Proposed
- **Context:** Need backdrivable, controllable rotary actuators.
- **Options:** BLDC + FOC (best backdrivability & control, most complex);
  geared DC (simpler, more friction/backlash); integrated servo modules
  (fastest to build, least tunable).
- **Decision:** *Undecided.* Leaning BLDC+FOC for backdrivability; revisit after
  torque/speed budget from kinematics.

## ADR-0004: Tension sensing method  ❓ Proposed
- **Status:** Proposed
- **Options:** in-line load cell per tendon (accurate, adds parts/space); motor
  current estimate (cheap, no extra parts, but friction-corrupted); series
  elastic element + displacement sensor (robust, adds compliance & size).
- **Decision:** *Undecided.* Tied to FR2 accuracy needs.

## ADR-0005: Compute topology  ❓ Proposed
- **Status:** Proposed
- **Options:** single MCU (simple, must hit ≥1 kHz loops + planning); RT-MCU for
  motor loops + SBC for planning (clean separation, adds a bus + comms).
- **Decision:** *Undecided.* Depends on final DOF count and loop budget.

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
