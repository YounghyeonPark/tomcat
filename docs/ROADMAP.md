# T.O.M.C.A.T. — Roadmap

Status: **ACTIVE**. This document defines the current development milestone,
grounded in the two [design principles](PRINCIPLES.md) (P1 tendon-drive, P2
whole-body curvature), the [ADR log](DESIGN_DECISIONS.md), and the verified
findings in the [Literature Review](LITERATURE_REVIEW.md).

The single source of truth for *decisions* remains the ADR log; this roadmap
sequences the work that implements them.

---

## Milestone M1 — Whole-Body Static Kinematics (Articulated Spine + Legs)

**Goal.** Extend the working single-leg planar prototype into a whole-body
static kinematic model: an articulated, tendon-driven spine that acts as a
*moving, curving base* for the four legs, with a **combined tendon/torque
budget** across spine and legs. This is the model that turns Principle P2 and
the (Accepted) [ADR-0006](DESIGN_DECISIONS.md) from prose into numbers, and
closes the open spine-DOF questions (NFR2/NFR2b/NFR2c).

**Why this milestone (evaluation of the obvious candidate).** The candidate
"whole-body kinematics: spine + legs + combined budget" is confirmed. It is the
lowest-risk, highest-leverage next step because it:
- is **pure software** (no hardware dependency) and reuses the existing planar
  kinematic framework — the serial-spine choice was picked in the lit review
  precisely because it "reuses our kinematic framework and is easier to control";
- directly discharges P2 and ADR-0006, and produces the **motor count and
  per-tendon tension band** that unblock ADR-0002 (antagonistic), ADR-0003
  (actuator sizing) and the electronics driver-channel count;
- lets us fold the two verified control-model corrections from the lit review
  into `tendon.py` now, before they propagate into firmware.

**Scope discipline (what M1 is NOT).** M1 stays in the **sagittal plane** to
match the existing 3R legs. Full 3D (frontal-plane leg abduction, spine lateral
bend + axial twist) is *parameterized but not exercised* — the model must be
shaped to accept those DOF later without rework, but M1 only validates the
dorsoventral (arch/extend) spine DOF and the existing sagittal leg. Gait
trajectory generation, dynamics (inertias/velocities), and a tail are
explicitly **out of scope** (later milestones).

### Definition of Done

M1 is done when all of the following hold:

1. `tomcat_kin` contains a **spine model**: a serial chain of `N` tendon-driven
   segments (seed `N=3`, per amended ADR-0006) with FK producing the pose of the
   shoulder and pelvic girdle frames as a function of spine joint angles.
2. A **whole-body FK** composes spine + four legs so each leg's hip frame rides
   on the curving spine; foot poses are reported in a common body frame.
3. The **tendon map covers spine tendons** (long cables routed from girdle
   motors along the column, with per-segment moment arms), including the
   many-joint coupling where one cable spans multiple segments.
4. `tendon.py` exposes **commandable co-contraction bias** (`T_bias`) as a
   first-class input and implements an **AIC-style** agonist/antagonist rule
   (per amended ADR-0002); the fixed-`pretension` behaviour becomes the
   `T_bias`-default special case.
5. A **combined static tendon/torque budget** sweeps whole-body load cases
   (stand, arch/extend, single-leg land) and reports per-tendon peak tension,
   per-motor peak torque, and **total motor count** for legs + spine.
6. All of the above are covered by `pytest` tests (FK/IK round-trips, spine
   chain, whole-body composition, tendon coupling, budget monotonicity), and
   `python kinematics/demo.py` runs the whole-body budget end-to-end.
7. `params.py` carries the seed spine parameters (segment count, moment arms,
   per-axis ROM, per-axis rotational stiffness) with each value labelled
   verified / converted / placeholder and cited to the lit review where it came
   from a source.

All numeric outputs remain **illustrative** until mechanical design lands; M1
proves the *machinery*, not the final geometry.

---

## Task breakdown

Dependencies flow top-to-bottom. Phase 0 tasks run in parallel; Phase 1 is the
kinematics build; Phase 2 (interfaces) runs in parallel after the budget exists.

### Phase 0 — seed the model (parallel)

**tomcat-research** — *R1: stiffness unit conversion & seed numbers.*
- Close lit-review gap #1: convert the cat whole-spine **axial 53.62 ± 4.68
  N/mm** + the directional-compliance **rank** (axial-rot < extension < lateral)
  into per-joint **rotational** stiffness seeds (N·m/rad) per axis, documenting
  the geometry-based method and its assumptions.
- Confirm the AIC seed parameters (`T_bias`, `k`) and the antagonistic tension
  sanity band (~20–70 N; RoboCat) for the budget's sanity checks.
- Deliver as a short table into `LITERATURE_REVIEW.md`'s seed-parameters
  section, labelled ◐/⚠️ as appropriate.

**tomcat-mechanical** — *M1: spine geometry & routing.*
- Define the spine segment geometry: segment count (seed 3), segment lengths,
  shoulder/pelvic girdle placement, tendon routing paths along the column,
  per-segment **moment arms** (pulley radii), and anchor points.
- Set per-axis **range-of-motion limits** (dorsoventral / lateral / axial),
  ordered by the biomechanics compliance rank from R1.
- Consume R1's per-joint rotational stiffness to set spine spring/compliance
  seeds. Note the BOM/motor-count implication of the chosen tendon routing.

*M1 depends on R1 for stiffness; geometry (lengths, moment arms, routing) can
start immediately in parallel.*

### Phase 1 — build the model

**tomcat-kinematics** — *K1: spine model + whole-body kinematics + combined
budget + tendon-map upgrade.* (the bulk of M1)
- Add a `spine.py` serial-chain model (sagittal DOF exercised; lateral/axial
  parameterized). FK → girdle frames.
- Add whole-body FK composing spine + four `LegModel`s on moving girdle frames.
- Extend `TendonMap` for spine tendons (per-segment moment arms; multi-segment
  cable coupling) and generalize the leg map to share the code path.
- Rework `_resolve_antagonistic` to take a per-joint `T_bias` (co-contraction
  bias) and apply an AIC rule (agonist gain `k`, antagonist held at `T_bias`);
  keep existing tests green by defaulting `T_bias = pretension`.
- Extend the budget to a whole-body sweep with stand / arch / land load cases;
  report per-tendon tension, per-motor torque, and total motor count.
- Tests + demo update.

*K1 consumes params from R1 + M1. Interfaces to firmware/electronics are the
setpoint schema and the motor count it produces.*

### Phase 2 — lock the interfaces (parallel, after K1 budget)

**tomcat-firmware** — *F1: setpoint schema (interface only, no RT code).*
- Extend/define the mid-level → low-level **setpoint schema** so spine motors
  are first-class and each motor setpoint carries `{position, tension,
  T_bias}`. Keep it a schema/interface doc + stub; no control loops this
  milestone. Must match K1's tendon-map output field-for-field.

**tomcat-electronics** — *E1: driver-channel count note (interface only).*
- Record the **motor-driver-channel count** implied by K1's total motor count,
  noting the ADR-0002 factor (antagonistic ≈ 2 channels/DOF; spring-return ≈ 1).
  No schematic work this milestone — this is the requirement that will size the
  board later.

---

## Cross-subsystem interfaces (make these explicit)

| # | Interface | Producer → Consumer | Contract |
|---|-----------|---------------------|----------|
| I1 | **Spine parameterization** | ADR-0006 + M1 + R1 → `params.py` | `{segment count, DOF/segment, segment lengths, moment arms, per-axis ROM, per-axis rotational stiffness}` |
| I2 | **Setpoint schema** | K1 tendon map → F1 firmware | per-motor `{position, tension, T_bias}`, covering leg **and** spine motors |
| I3 | **Motor / driver-channel count** | K1 combined budget → E1 electronics / ADR-0002 | total motor count; antagonistic doubles channels/DOF |
| I4 | **Moving-base frame convention** | M1 girdle placement → K1 whole-body FK | spine provides girdle base frames that leg FK/IK hang off (documented in `spine.py`) |

---

## ADR changes this milestone drives

M1 depends on three ADR amendments (proposed to the ADR log separately; see the
milestone-planning report). In summary:
- **ADR-0006** — record the concrete baseline: serial tendon-driven chain, seed
  **2–3 segments × ~3 DOF** (pitch/yaw + roll), tensegrity kept as research
  alternative. Unblocks I1 and the spine model.
- **ADR-0002** — move toward **antagonistic with commandable co-contraction
  bias (`T_bias`) + AIC** (Kengoro), spring-return reserved for distal low-DOF
  joints. Drives the `tendon.py` rework.
- **ADR-0003** — add a required **tendon-vs-backdrivable-direct-drive leg trade
  study using the IMF metric** (MIT Cheetah) before finalizing leg actuators.
  Consumes M1's leg torque budget as an input; not blocking for M1.

---

## After M1 (candidate M2+, not committed)

- 3D extension: frontal-plane leg abduction + spine lateral bend & axial twist.
- Gait trajectory generation on the curving-base model (spine↔gait coupling;
  tunable spine stiffness vs. speed, per lit-review Q2b).
- Dynamics: leg mass in flight (~0.454 kg knee) driving trunk bending.
- Leg actuator IMF trade study (ADR-0003) fed by the M1 budget.
- **Righting milestone (in scope, [ADR-0007](DESIGN_DECISIONS.md)):** design the
  inertial (morphable) tail, model flight-phase reorientation (tail + spine
  twist), and a fall-detect → reorient control law (lit-review Q4).
