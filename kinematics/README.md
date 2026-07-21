# kinematics/

Kinematics models, tendon mapping, and gait planning for TomCat.

Responsibilities (see [../docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md)):
- **Leg kinematics:** forward/inverse kinematics (foot position ↔ joint angles).
- **Tendon map:** joint angle & stiffness ↔ per-motor cable length + tension,
  including moment arms (pulley radii), friction, and finite cable stiffness.
- **Antagonistic coordination:** split desired joint torque/stiffness across
  opposing tendons (or tendon + return spring) — see ADR-0002.
- **Gait planning:** foot/body trajectories for walk, trot, etc.

## Layout
- `src/` — models and planners (language TBD; Python for prototyping is likely)

## Status
Stub. A pure-model prototype (no hardware) is a good first milestone: verify the
tendon map and IK against a simple single-leg case.
