# TomCat — Requirements

Status: **DRAFT** — targets are placeholders to be confirmed. Open questions are
tracked inline as `❓`.

Grounded in the two [design principles](PRINCIPLES.md): (P1) tendon-driven,
centralized multi-motor actuation, and (P2) a feline form whose whole body may
curve.

## 1. Goals

- G1. Quadruped locomotion with cat-like agility (walk, trot, and eventually a
  controlled leap/land).
- G2. Tendon-driven joints with motors centralized in the body girdles to
  minimize limb inertia (P1).
- G3. Passive compliance / shock absorption at each joint.
- G4. Energy-efficient movement compared with a direct-drive baseline.
- G5. An articulated, tendon-driven spine so the body can arch, bend laterally,
  and twist like a real cat (P2).
- G6. Mid-air righting: reorient during a fall to land feet-first, via spine
  axial-twist + legs, with a coarse single-tendon tail assist
  (see [ADR-0007](DESIGN_DECISIONS.md)).

## 2. Functional requirements

| ID   | Requirement                                                                 | Priority |
|------|-----------------------------------------------------------------------------|----------|
| FR1  | Drive N tendons via rotary motors with closed-loop position control.        | Must     |
| FR2  | Measure and closed-loop control cable **tension** per driven tendon.        | Must     |
| FR3  | Sense joint angle (directly or inferred from cable displacement).           | Must     |
| FR4  | Execute a parameterized gait to produce forward walking.                    | Must     |
| FR9  | Actuate the spine to bend (dorsoventral + lateral) via tendons.             | Must     |
| FR10 | Coordinate spine curvature with leg motion (whole-body posture).            | Should   |
| FR11 | Detect a fall and reorient (tail + spine twist) to land feet-first.        | Should   |
| FR5  | Detect and recover from a foot slip / unexpected ground contact.            | Should   |
| FR6  | Report telemetry (per-motor current, tension, angle) over a host link.      | Should   |
| FR7  | Support a calibration routine for zeroing tendon tension and joint range.   | Must     |
| FR8  | Enter a safe, limp state on fault (over-current, over-tension, e-stop).     | Must     |

## 3. Non-functional / performance targets  ❓ *confirm with mechanical design*

| ID    | Target                                          | Value (placeholder) |
|-------|-------------------------------------------------|---------------------|
| NFR1  | Degrees of freedom per leg                       | 3 (hip, knee, ankle)|
| NFR2  | Spine segments (serial, tendon-driven)           | ❓ TBD (e.g. 3–5)    |
| NFR2b | DOF per spine segment                            | ❓ TBD (2: pitch+yaw)|
| NFR2c | Total actuated DOF (12 legs + 6 spine + 1 tail)  | **19** (= 19 motors, ADR-0008 + **ADR-0009** lateral) |
| NFR2d | Tail actuation (coarse assist, no accuracy)      | 1 tendon + passive return |
| NFR3  | Control loop rate (tension/position)             | ≥ 1 kHz             |
| NFR4  | Gait / trajectory update rate                    | ≥ 100 Hz            |
| NFR5  | Mass (total)                                     | **3.0 kg** (ADR-0008 closes the budget at this) |
| NFR6  | Runtime on one battery charge                    | ❓ TBD               |
| NFR7  | Max cable tension per tendon                     | ❓ TBD (N)           |

## 4. Constraints & assumptions

- Antagonistic tendon pairs are needed because a cable can only pull. Settled by
  [ADR-0008](DESIGN_DECISIONS.md): **one motor per DOF**, driving both sides of
  the pair through a **variable-radius pulley** — the mass budget does not permit
  two motors per DOF.
- Cables are inextensible enough that motor rotation maps predictably to joint
  angle, but tendon stretch and friction must be modeled/compensated.
- Motors, drivers, battery, and main compute live in the torso.

## 5. Open questions

The major architecture questions are **resolved** in the [ADR log](DESIGN_DECISIONS.md):
- Actuator choice → tendon-drive, BLDC + FOC (ADR-0003).
- Tension sensing → hybrid: motor-current estimate everywhere + joint-end load
  cells on stiffness-critical joints (ADR-0004).
- Compute split → distributed CAN-FD drivers + RT controller + SBC (ADR-0005).
- Tendons per DOF → antagonistic pairs; spring-return for distal joints (ADR-0002).

Remaining **calibration / measurement** items (not decisions):
- ❓ Routed cable friction μ and per-tendon wrap — blocks the tension error budget
  and the current-vs-load-cell placement split (ADR-0004 follow-up).
- ❓ Specific motor **part** — the *class* is now specified (~1.1 N·m peak, ≤80 g,
  24 V; `Kt` ≈ 0.44 N·m/A) by the [down-select](notes/motor-downselect.md), but a
  real part must be surveyed and its torque density confirmed — ADR-0008's mass
  closure rides on it.
- ❓ Runtime (NFR6) and per-tendon force target (NFR7).

## 6. Out of scope (for now)

- Autonomous navigation / SLAM.
- Manipulation (the cat does not need to pick things up).
- Outdoor / all-terrain operation.
