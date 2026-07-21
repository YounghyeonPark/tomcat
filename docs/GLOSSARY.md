# TomCat — Glossary

- **Tendon / cable** — synthetic line that transmits pulling force from a
  torso-mounted motor to a joint. Can only pull, never push.
- **Antagonistic pair** — two tendons acting in opposite directions across one
  joint, so the joint can be actively driven both ways and its stiffness set by
  co-contraction.
- **Return spring** — passive element that pulls a single-tendon joint back when
  its tendon slackens; simpler than an antagonistic pair but with fixed
  stiffness.
- **Co-contraction** — pulling both tendons of an antagonistic pair at once to
  stiffen a joint without moving it (biomimetic muscle behavior).
- **Backdrivability** — the ability to move a joint by pushing on the output;
  good backdrivability gives natural compliance and shock absorption.
- **Moment arm** — the effective pulley/lever radius that converts cable tension
  into joint torque.
- **Tendon map** — the model converting joint angles/stiffness ↔ per-motor cable
  length and tension.
- **FOC (Field-Oriented Control)** — control scheme for smooth, efficient BLDC
  torque control.
- **Series elastic actuator (SEA)** — an actuator with a deliberate spring in
  series, enabling force control via spring deflection.
- **DOF (Degree of Freedom)** — an independently actuated joint axis.
- **Gait** — the timed pattern of leg movements (walk, trot, etc.).
- **Stance / swing phase** — leg on the ground (stance) vs. moving through the
  air (swing).
- **IK / FK** — inverse / forward kinematics: foot position ↔ joint angles.
- **Limp state** — safe fault mode where motors are de-energized (or tension
  released) so the robot goes compliant.
