# TomCat — System Architecture

Status: **DRAFT**

## 1. Overview

TomCat is organized as a layered control stack running on hardware split
between the torso (motors, drivers, compute, power) and the limbs (tendons,
joints, foot contact sensors).

```
                          ┌─────────────────────────────┐
                          │        Host / Operator       │
                          │   (config, telemetry, e-stop)│
                          └───────────────┬─────────────┘
                                          │  USB / Wi-Fi
                          ┌───────────────▼─────────────┐
        High-level        │   Gait & Trajectory Planner  │  ≥100 Hz
        (SBC or MCU)      │  body pose → per-joint angles │
                          └───────────────┬─────────────┘
                                          │ desired joint angles / stiffness
                          ┌───────────────▼─────────────┐
        Mid-level         │   Kinematics / Tendon Map    │
                          │ joint angle ↔ cable length   │
                          │ + antagonistic coordination  │
                          └───────────────┬─────────────┘
                                          │ per-motor position + tension setpoints
                          ┌───────────────▼─────────────┐
        Low-level (RT)    │  Motor Control Loops (≥1 kHz)│
                          │  position + tension PID/FOC   │
                          └───────────────┬─────────────┘
                                          │ PWM / phase currents
              ┌───────────────────────────┼───────────────────────────┐
              ▼                            ▼                            ▼
        ┌──────────┐               ┌──────────┐                 ┌──────────┐
        │ Motor +  │  tendon ~~~▶  │  Joint   │   contact ▲     │  Foot    │
        │ driver   │               │ (pulley) │                 │  sensor  │
        └──────────┘               └──────────┘                 └──────────┘
              ▲                          │
              └── tension sensor ◀───────┘  (in-line load cell or current est.)
```

## 2. Control layers

### 2.1 Low-level: motor control (real-time, ≥1 kHz)
- Runs closed-loop **position** and **tension** control per motor.
- Enforces safety limits: over-current, over-tension, thermal, e-stop → limp.
- Candidate: BLDC with field-oriented control (FOC), or current-controlled DC.

### 2.2 Mid-level: kinematics & tendon coordination
- Converts desired **joint angles** and **joint stiffness** into per-motor
  position and tension setpoints.
- Handles the many-to-one/antagonistic mapping: a cable only pulls, so each DOF
  needs either two opposing tendons or one tendon plus a passive return spring.
- Compensates tendon routing (moment arms via pulley radii), friction, and
  finite cable stiffness.

### 2.3 High-level: gait & trajectory planning (≥100 Hz)
- Generates body/foot trajectories for a chosen gait (walk, trot, …).
- Uses inverse kinematics to turn foot targets into joint-angle references.
- Consumes foot-contact events for phase timing and slip recovery.

## 3. Hardware blocks

| Block            | Role                                                          |
|------------------|---------------------------------------------------------------|
| Main compute     | High/mid-level control, planning, host comms                  |
| Real-time MCU(s) | Low-level motor loops, safety, sensor sampling                |
| Motor drivers    | Power stage per motor (BLDC/DC), current sensing              |
| Motors           | Rotary actuators pulling tendons (centralized in torso)       |
| Tension sensors  | Per-tendon force feedback (load cell or current estimate)     |
| Joint sensors    | Absolute/relative joint angle (or inferred from cable travel) |
| Foot sensors     | Ground contact / force detection                             |
| Power            | Battery, regulation, e-stop, power distribution               |
| IMU              | Body orientation for balance and gait feedback                |

❓ Compute split (single MCU vs. RT-MCU + SBC) is an open decision — see
[DESIGN_DECISIONS.md](DESIGN_DECISIONS.md).

## 4. Data flow summary

1. Planner emits foot/body targets → IK → desired joint angles + stiffness.
2. Tendon map converts to per-motor position + tension setpoints.
3. Motor loops track setpoints and stream back current/tension/angle.
4. Telemetry and faults propagate up; e-stop propagates down to limp state.

## 5. Interfaces (to be specified)

- Host ↔ compute: ❓ protocol (USB CDC, custom serial, ROS 2, …).
- Compute ↔ RT-MCU: ❓ bus (SPI, CAN, UART) and message schema.
- Driver ↔ MCU: PWM + current/encoder feedback.

See [firmware/](../firmware) and [kinematics/](../kinematics) for module stubs.
