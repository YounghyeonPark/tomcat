# mechanical/

Mechanical design for TomCat: leg geometry, joint/pulley design, tendon routing,
and bill of materials.

Scope:
- **Skeleton:** an articulated tendon-driven spine + 4 legs, 3 DOF each (hip,
  knee, ankle). The torso is *not* rigid — see [ADR-0006](../docs/DESIGN_DECISIONS.md)
  and NFR1/NFR2 in [../docs/REQUIREMENTS.md](../docs/REQUIREMENTS.md).
- **Spine:** serial segments allowing dorsoventral + lateral bend (and, ideally,
  axial twist for the righting reflex), with tendons routed to girdle motors.
- **Tail:** an inertial, ideally **morphable (telescoping)** appendage for
  mid-air righting (see [ADR-0007](../docs/DESIGN_DECISIONS.md)) — extends for
  reorientation authority in flight, retracts before touchdown.
- **Tendon routing:** cable paths from torso-mounted motors over pulleys to each
  joint; sheaths, anchor points, and moment arms.
- **Compliance:** return springs and/or series-elastic elements per ADR-0002.
- **BOM:** motors, cables, pulleys, bearings, springs, fasteners.

## Specs
- [LEG_TENDON_SPEC.md](LEG_TENDON_SPEC.md) — leg tendon-drive spec for the
  pure-tendon legs (ADR-0003): moment-arm sizing that halves the ~1 kN cable
  tension to a ~500 N land transient, UHMWPE cable spec, pulley/bearing/routing
  and wrap angles (capstan friction), a static-hold brake, and the parameter
  table for `LegParams` / `TendonParams`.
- [SPINE_TAIL_SPEC.md](SPINE_TAIL_SPEC.md) — first-pass geometry & tendon-routing
  proposal for the 3-segment articulated spine (ADR-0006) and the inertial
  morphable tail (ADR-0007): segment lengths, girdle placement, routing paths,
  moment arms, per-axis ROM, and the parameter table for `SpineParams` +
  proposed `TailParams`, plus BOM/motor-count implications.

## Status
Stub — CAD files and routing diagrams to be added. Mechanical review is the
gate for ADR-0002 (antagonistic vs. return spring) and torque budget feeding
ADR-0003 (actuator choice).
