# T.O.M.C.A.T. — Design Principles

These are the non-negotiable commitments that define the project. Everything in
[REQUIREMENTS.md](REQUIREMENTS.md), [ARCHITECTURE.md](ARCHITECTURE.md), and the
[ADR log](DESIGN_DECISIONS.md) must serve them.

## P1 — Tendon-driven, centralized multi-motor actuation

Every actuated joint — in the **limbs and the spine alike** — is moved by
synthetic tendons pulled by rotary motors. No motor sits at a joint. Motors are
grouped in the body (girdles / pelvis) and drive jointed actuators through
cables, the way muscles drive bones through tendons.

- A cable only *pulls*, so each degree of freedom uses either an antagonistic
  tendon pair or a single tendon plus a passive return element (see
  [ADR-0002](DESIGN_DECISIONS.md)).
- Multiple motors cooperate on a single articulated actuator (e.g. an
  antagonistic pair per joint, and several tendons routed along a multi-joint
  spine). Control must coordinate these coupled tendons.

## P2 — Feline form: the whole body may curve

The robot's shape mimics a real cat, and **all curvature of the body is
allowed**. The torso is *not* a rigid chassis: it is an articulated,
tendon-driven spine that can bend and arch, so the body deforms continuously the
way a cat's does.

Motions the design must physically permit (subject to safe limits):

- **Dorsoventral flexion/extension** — arching the back ("Halloween cat") and
  the crouch/extend of a gallop.
- **Lateral flexion** — side-to-side bending toward the reaching limb.
- **Axial rotation** — the twist used in the mid-air righting reflex.

Consequences:

- The spine is modelled as a serial chain of tendon-driven joints (a segmented
  column), not a single link. See [ADR-0006](DESIGN_DECISIONS.md).
- Total DOF is limbs **plus** spine; the kinematic model must eventually treat
  the body as a moving, curving base for the legs rather than a fixed frame.
- Compliance is a feature, not a defect: the same tendon routing that allows
  curvature also gives passive shock absorption.

## How the principles constrain the design

| Principle | Drives |
|-----------|--------|
| P1 | Motors centralized in girdles; tendon routing; antagonistic control; coupled-cable coordination in the mid-level controller. |
| P2 | Articulated spine (multi-joint); higher DOF count; whole-body kinematics; balance/righting behaviours in the planner. |
