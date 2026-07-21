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

## 2. Functional requirements

| ID   | Requirement                                                                 | Priority |
|------|-----------------------------------------------------------------------------|----------|
| FR1  | Drive N tendons via rotary motors with closed-loop position control.        | Must     |
| FR2  | Measure and closed-loop control cable **tension** per driven tendon.        | Must     |
| FR3  | Sense joint angle (directly or inferred from cable displacement).           | Must     |
| FR4  | Execute a parameterized gait to produce forward walking.                    | Must     |
| FR9  | Actuate the spine to bend (dorsoventral + lateral) via tendons.             | Must     |
| FR10 | Coordinate spine curvature with leg motion (whole-body posture).            | Should   |
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
| NFR2c | Total actuated DOF (12 legs + spine)             | ❓ TBD               |
| NFR3  | Control loop rate (tension/position)             | ≥ 1 kHz             |
| NFR4  | Gait / trajectory update rate                    | ≥ 100 Hz            |
| NFR5  | Mass (total)                                     | ❓ TBD               |
| NFR6  | Runtime on one battery charge                    | ❓ TBD               |
| NFR7  | Max cable tension per tendon                     | ❓ TBD (N)           |

## 4. Constraints & assumptions

- Antagonistic tendon pairs (or tendon + return spring) are needed because a
  cable can only pull. ❓ Decide per joint: **2 motors (antagonistic)** vs.
  **1 motor + passive return spring**.
- Cables are inextensible enough that motor rotation maps predictably to joint
  angle, but tendon stretch and friction must be modeled/compensated.
- Motors, drivers, battery, and main compute live in the torso.

## 5. Open questions

- ❓ Actuator choice: BLDC + FOC, geared DC, or servo modules?
- ❓ Tension sensing method: in-line load cell, motor-current estimate, or
  series-elastic element with displacement sensing?
- ❓ Compute split: single MCU vs. real-time MCU + higher-level SBC?
- ❓ Number of tendons per DOF (1+spring vs. 2 antagonistic).

## 6. Out of scope (for now)

- Autonomous navigation / SLAM.
- Manipulation (the cat does not need to pick things up).
- Outdoor / all-terrain operation.
