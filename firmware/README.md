# firmware/

Embedded control firmware for TomCat.

Responsibilities (see [../docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md)):
- **Low-level (≥1 kHz):** per-motor position + tension control loops, safety
  limits (over-current, over-tension, thermal, e-stop → limp).
- **Sensor sampling:** motor current/encoder, tendon tension, joint angle, IMU,
  foot contact.
- **Comms:** telemetry up to the planner/host; setpoints down to motor loops.

> ⚠️ **"joint angle" is not a channel on the board.** `electronics/BOARD_OUTLINE.md` carries a **rotor** absolute encoder, current sense, a tension front-end (spine+hip/knee only) and an IMU — there is no joint encoder. Joint angle is *reconstructed* through the drivetrain (`wbc.joint_from_encoders`), and it needs the tension: without the ankle load cell the ankle angle is out by **1.09°**, rising to **6.08°** at the peak rating ([ADR-0077](../docs/DESIGN_DECISIONS.md), M72).

## Layout
- `src/` — implementation
- `include/` — public headers / interfaces

## Status
Stub. Target MCU, RTOS/bare-metal choice, and build system are open — see
ADR-0003 and ADR-0005 in [../docs/DESIGN_DECISIONS.md](../docs/DESIGN_DECISIONS.md).
