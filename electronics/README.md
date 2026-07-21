# electronics/

KiCad schematics and PCB layout for the TomCat control electronics.

Planned boards (see [../docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md)):
- **Control board:** main compute / RT-MCU, IMU, host link, power regulation,
  e-stop.
- **Motor driver stage(s):** power stage per motor with current sensing;
  count depends on total DOF and antagonistic choice (ADR-0002).
- **Sensor conditioning:** tendon tension (load-cell amp) and joint-angle
  front-ends, per ADR-0004.

## Tooling
This repo has a `kicad-cli` skill available for ERC/DRC checks and BOM/Gerber
export once schematics exist. Read the skill's "Project gotchas" section before
trusting CLI ERC/netlist results.

## Status
Stub — no schematics yet. Blocked on actuator and sensing decisions
(ADR-0003, ADR-0004).
