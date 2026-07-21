# mechanical/

Mechanical design for TomCat: leg geometry, joint/pulley design, tendon routing,
and bill of materials.

Scope:
- **Skeleton:** an articulated tendon-driven spine + 4 legs, 3 DOF each (hip,
  knee, ankle). The torso is *not* rigid — see [ADR-0006](../docs/DESIGN_DECISIONS.md)
  and NFR1/NFR2 in [../docs/REQUIREMENTS.md](../docs/REQUIREMENTS.md).
- **Spine:** serial segments allowing dorsoventral + lateral bend (and, ideally,
  axial twist for the righting reflex), with tendons routed to girdle motors.
- **Tendon routing:** cable paths from torso-mounted motors over pulleys to each
  joint; sheaths, anchor points, and moment arms.
- **Compliance:** return springs and/or series-elastic elements per ADR-0002.
- **BOM:** motors, cables, pulleys, bearings, springs, fasteners.

## Status
Stub — CAD files and routing diagrams to be added. Mechanical review is the
gate for ADR-0002 (antagonistic vs. return spring) and torque budget feeding
ADR-0003 (actuator choice).
