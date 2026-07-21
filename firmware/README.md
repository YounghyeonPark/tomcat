# firmware/

Embedded control firmware for TomCat.

Responsibilities (see [../docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md)):
- **Low-level (≥1 kHz):** per-motor position + tension control loops, safety
  limits (over-current, over-tension, thermal, e-stop → limp).
- **Sensor sampling:** motor current/encoder, tendon tension, joint angle, IMU,
  foot contact.
- **Comms:** telemetry up to the planner/host; setpoints down to motor loops.

## Layout
- `src/` — implementation
- `include/` — public headers / interfaces

## Status
Stub. Target MCU, RTOS/bare-metal choice, and build system are open — see
ADR-0003 and ADR-0005 in [../docs/DESIGN_DECISIONS.md](../docs/DESIGN_DECISIONS.md).
