# mechanical/

Mechanical design for TomCat: leg geometry, joint/pulley design, tendon routing,
and bill of materials.

Scope:
- **Skeleton:** an articulated tendon-driven spine + 4 legs, 3 DOF each (hip,
  knee, ankle). The torso is *not* rigid — see [ADR-0006](../docs/DESIGN_DECISIONS.md)
  and NFR1/NFR2 in [../docs/REQUIREMENTS.md](../docs/REQUIREMENTS.md).
- **Spine:** serial segments allowing dorsoventral + lateral bend (and, ideally,
  axial twist for the righting reflex), with tendons routed to girdle motors.
- **Tail:** a **single-tendon** appendage — tension to curl/raise, loosen to
  relax (passive return); no precision (see [ADR-0007](../docs/DESIGN_DECISIONS.md)).
  Coarse inertial assist only — righting authority is in the spine + legs.
- **Tendon routing:** cable paths from torso-mounted motors over pulleys to each
  joint; sheaths, anchor points, and moment arms.
- **Compliance:** return springs and/or series-elastic elements per ADR-0002.
- **BOM:** motors, cables, pulleys, bearings, springs, fasteners.

## Reference
- [reference/ANATOMY.md](reference/ANATOMY.md) — feline anatomy basis for the
  biomimetic geometry, citing **public-domain** sources (Reighard & Jennings,
  *Anatomy of the Cat*, 1901) — vertebral formula, digitigrade limbs, fore/hind
  asymmetry, thorax vs lumbar.

## CAD
- [cad/](cad/) — first **3D geometry**: a parametric build123d skeleton model
  driven by the `tomcat_kin` dimensions, exported to STEP (`cad/tomcat_skeleton.step`)
  with a rendered preview. A massing model, not yet a manufacturing model.

## Specs
- [LEG_TENDON_SPEC.md](LEG_TENDON_SPEC.md) — leg tendon-drive spec for the
  pure-tendon legs (ADR-0003): moment-arm sizing that halves the ~1 kN cable
  tension to a ~500 N land transient, UHMWPE cable spec, pulley/bearing/routing
  and wrap angles (capstan friction), a static-hold brake, and the parameter
  table for `LegParams` / `TendonParams`.
- [SPINE_TAIL_SPEC.md](SPINE_TAIL_SPEC.md) — first-pass geometry & tendon-routing
  proposal for the 3-segment articulated spine (ADR-0006) and the single-tendon
  tail (ADR-0007): segment lengths, girdle placement, routing paths, moment arms,
  per-axis ROM, and the parameter table for `SpineParams` + `TailParams`, plus
  BOM/motor-count implications.

## Status
Stub — CAD files and routing diagrams to be added. Mechanical review is the
gate for ADR-0002 (antagonistic vs. return spring) and torque budget feeding
ADR-0003 (actuator choice).
