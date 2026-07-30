# T.O.M.C.A.T. — Roadmap

Status: **ACTIVE**. This document defines the current development milestone,
grounded in the two [design principles](PRINCIPLES.md) (P1 tendon-drive, P2
whole-body curvature), the [ADR log](DESIGN_DECISIONS.md), and the verified
findings in the [Literature Review](LITERATURE_REVIEW.md).

The single source of truth for *decisions* remains the ADR log; this roadmap
sequences the work that implements them.

> **Progress:** the decision phase is complete — all seven ADRs Accepted (0004's
> tension-*method* sub-point aside). **M1 done** (Phases 0–2): whole-body static
> kinematics + tendon map, combined budget, first firmware/electronics/mechanical
> specs. **M2 done**: parameterized walk gait generator. **M3 done**: whole-body
> foot placement — spine↔gait loop closed, stance feet held in the world frame
> while the legs compensate for spine motion. Plus a biomimetic pass: digitigrade
> 4-link legs, fore/hind asymmetry (model *and* CAD), thoracic ribcage, and 3D
> CAD/packaging studies (168 passing tests).
> **M4 done:** real mass — CoM & static stability (fore-aft).
> **M5 done:** the **lateral spine DOF** ([ADR-0009](DESIGN_DECISIONS.md)) — a true
> 3D support polygon, commanded lateral sway, and with it the first walk that is
> genuinely statically stable in 3D (**+10.1 mm**, up from **−21.6 mm**) *and*
> dynamically realisable inside the paw friction cone. 244 tests.

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

## Milestone M2 — Gait generation on the whole-body model  ✅ DONE

**Goal.** Turn the static M1 model into *motion*: a parameterized, periodic
walk gait producing per-leg joint-angle trajectories over a cycle via the
existing IK, spine neutral first then an optional spine oscillation coupled to
the gait (spine↔gait coupling, lit-review Q2b). Sagittal-plane, quasi-static.

**Delivered:** `gait.py` (`GaitParams`, `foot_target` cycloid swing, `GaitController`,
`GaitState`/`LegState`); a statically-stable lateral-sequence walk (offsets
LF 0 / RF 0.25 / RR 0.5 / LR 0.75, duty 0.75 → exactly 3 feet down at all
phases); optional dorsoventral spine oscillation (off by default). 67 tests
(133 total). Default: period 1.2 s, stride 60 mm, step 30 mm, body speed
67 mm/s, ~11° joint-limit margin across the cycle.

**Deferred to M3:** the spine oscillation moves feet in the world but doesn't
feed back into per-leg IK — true whole-body foot placement (feet in a
world/ground frame through the spine) is the natural next step.

## Milestone M3 — Whole-body foot placement  ✅ DONE

Closed the spine↔gait loop M2 deferred: feet are placed in a **world/ground
frame** and per-leg IK is solved *through* the moving girdle
(`world ← girdle(spine_q) ← hip ← foot`). Delivered `WholeBody.inverse` /
`inverse_pose` and a `WholeBodyGaitController` that **holds stance feet planted
in the world while the legs absorb spine motion** (verified: foot-target span
0.000 mm under a ±2°/seg spine oscillation, leg angles differ up to ~8°, FK
error ~0). 25 new tests (158 total).

## Milestone M4 — Real mass: CoM & static stability (DONE)

**Goal.** Put actual **mass** into the model. Today everything is massless and
`whole_body_budget` lumps *equal* masses at the vertebrae — its own assumption
A2 concedes a real cat is ~**60 % front-heavy**. M4 fixes that and adds the
stability check the gait has never had (it currently validates stance *count*
only, with no support-polygon margin).

**Scope — quasi-static WITH real mass** (velocities/accelerations stay out;
full Newton–Euler is M5):
1. Per-link masses + CoM fractions on both leg variants, spine segments, and
   girdles, apportioned to the ~3 kg body with the ~60/40 fore/hind split.
2. Whole-body **centre of mass** as a function of spine + leg posture.
3. **Static stability margin** — CoM ground projection vs. the fore-aft support
   interval, reported per gait phase.
4. Spine gravity loads driven by the **real distributed mass**, replacing the
   equal-lumped placeholder in the whole-body budget.

**Honest bound:** a 2D sagittal "support polygon" is really a fore-aft interval —
true lateral/roll stability needs the 3D extension. *That bound is what M5 went
on to close, and it turned out to be a hardware decision, not a modelling one.*

## Milestone M5 — Lateral spine DOF: 3D static stability (DONE)

**Goal.** Close M4's honest bound. Evaluating the *true* ground-plane support
polygon showed the walk was **laterally unstable at −27.4 mm** while its fore-aft
margin read a healthy +27.7 mm — the 2D check had been hiding a real failure.
[ADR-0009](DESIGN_DECISIONS.md) resolved it by adding a lateral bend DOF per spine
segment (16 → 19 motors); M5 is that model.

**Delivered.**
1. `SpineParams.lateral_q_min/max` (±15°/segment) and
   `SpineModel.lateral_vertebra_xy` / `lateral_segment_com_y` — a planar chain in
   the x–y plane alongside the existing sagittal one.
2. `WholeBody.center_of_mass_y` — lateral CoM including the fore-leg mass that
   rides with the front girdle. Reproduces ADR-0009's predicted authority exactly
   (15.0 / 29.4 / **42.8 mm** at 5 / 10 / 15°).
3. `GaitController.lateral_q` — the sway law, plus `support_side` and
   `lateral_slew_rate`. `support_polygon` now takes its CoM y from this **real
   actuated DOF** instead of the old `lateral_shift` what-if parameter.
4. `GaitController.crossover_accel` / `.friction_accel_limit` /
   `.crossover_is_feasible` — the dynamic reality check on the sway (see below).
5. Default gait retuned: **duty 0.75 → 0.90**, **period 1.2 → 1.4 s**, sway **12.5°**.

**Results, and the four things building it taught us:**
- The default walk is now statically stable in 3D: **+10.1 mm** (was −21.6 mm).
- Sway authority is **adequate, not ample** — margin *peaks* at 12.5° and falls
  beyond, since over-swaying tips the CoM over the far edge. ~2.5° of ROM spare.
- **Duty had to rise above 0.75.** At exactly 0.75 the swing windows tile, so the
  support side flips instantaneously; any finite ramp collapses the margin back
  to the no-sway value. Duty > 0.75 opens the four-foot crossover windows.
- **A sinusoidal sway is worse than none** — it is near zero exactly at the
  crossovers, which is where the margin is decided. The law is a ramped square.
- ⚠️ **The real constraint was FRICTION, and it sets the walk speed.** Reversing
  the sway costs `a = 4d/w²` — inverse-square in the window — and a paw delivers
  only `μg`. The first tuned gait (1.2 s, duty 0.80) demanded **9.1 g** and was
  not realisable despite a healthy quasi-static margin. Retuned to 1.4 s / duty
  0.90: **6.87 vs 7.85 m/s²**, closing with only **14 %** margin (needs μ ≥ 0.70).

**Honest bounds.** (a) +10.1 mm is a *static* margin; full inertia is still
unmodelled. (b) The friction check is a **hand calculation outside the model**.
(c) Static stability caps this walk at a **~4 cm/s crawl** — inherent to stopping
and shifting weight between steps. Cat-like speed must come from a **dynamic**
gait, which ADR-0008 already sized the motors for. That is the next milestone.

### M5 follow-up — sizing what ADR-0009 added, and a mass-model correction

ADR-0009 added three motors; M5 proved they were *needed*. This pass checked they
are *sufficient*, and in doing so found the mass model was wrong.

1. **The lateral drive was never sized.** `WholeBody.lateral_spine_loads` now does
   it. The load is **inertial, not gravitational** — the lateral bend axis is
   vertical, so gravity exerts no moment about it and *holding* a sway is nearly
   free; *reversing* it is what costs. Base joint: **2.21 N·m → 110 N cable →
   0.88 N·m motor = 0.80×** the trot sizing point, so the three extra motors are
   the **same class as the leg motors** and ADR-0009's mass arithmetic holds.
2. ⚠️ **…but only after raising the lateral moment arm 15 → 20 mm.** At the bare
   transverse-process width the base joint needs **1.18 N·m — over motor peak**.
   20 mm needs a **milled lateral pulley post** per vertebra (the trick already
   used for the 30 mm dorsoventral arm). That post is load-bearing, not detail.
3. ⚠️ **The mass model was ~347 g light on actuation.** It charged **31 channels**
   (a pre-variable-radius-pulley count) at a **31 g** motor, while the build is
   **19 motors at the down-selected 72 g** — two errors of opposite sign that hid
   each other. It also parked the spine/tail bank in the rear girdle when the CAD
   packs it **mid-body**. Corrected in `params.py`.
   - Net effect is **favourable**: CoM +100 → +108 mm, fore-aft margin +32.7 →
     **+40.2 mm**, polygon +8.4 → **+10.1 mm**, friction margin 11 → **14 %**.
   - But review finding **F2 is partly walked back** — "quiet stand barely loads
     the base joint" goes 0.13 → **0.29 N·m**. Its direction holds; its magnitude
     was optimistic because it under-weighed the actuators.
   - Full accounting: [notes/mass-budget-recheck.md](notes/mass-budget-recheck.md).
4. **CAD and electronics caught up with the decision.** Packaging now places all
   **19** motors (was still drawing 16) with the lateral tendons and posts, and
   the cluster topology is corrected to **three** nodes — shoulder 6 / pelvis 6 /
   **mid-body 7** — in ADR-0005 and the board outline. The pelvis is no longer
   the dense node; the mid-body bay is.

**ADR-0009's structure-budget ⚠️ is downgraded, not closed:** the first estimate
from real CAD geometry gives **~296 g of printed structure against a 587 g
allowance**, but that is a massing model with solid bones — directional only.

## Later milestones (candidate M6+, not committed)

- **Full dynamics — now the critical path, not an optional extra.** M5 ended by
  hitting a limit no quasi-static model can cross: the statically stable walk is
  capped at ~4 cm/s, and its friction margin is 14 %. Both need real inertia.
- **Full dynamics:** velocities/accelerations, Newton–Euler; leg mass in flight
  driving trunk bending (the lit-review MMS result — note its ~0.454 kg knee is
  from a far larger robot and must be rescaled);
  velocities/inertia beyond the current quasi-static model.
- 3D extension, remaining parts: frontal-plane leg **abduction** and spine
  **axial twist** (the lateral bend is done, M5). Yaw and twist are what the
  ADR-0007 righting work needs.
- **Righting milestone ([ADR-0007](DESIGN_DECISIONS.md)):** model flight-phase
  reorientation via **spine axial-twist + leg shape-change** (rotary-only 180°),
  with the single-tendon tail as a coarse bias term, and a fall-detect → reorient
  control law (lit-review Q4).
- Firmware/electronics build-out from the M1 interface specs (CAN-FD driver
  firmware; KiCad smart-driver schematic/PCB).

## Open reconciliation items (lead)

- **Motor count 30 vs. ~25:** the whole-body budget assumes all-antagonistic
  (24 legs + 6 spine); electronics notes ~25 if the **ankle is spring-return**
  (already sanctioned by ADR-0002) and the **spine uses a variable-radius pulley**
  (1 motor/antagonistic pair). Confirm before committing backplane channel count.
- **ADR-0004 tension method** (load-cell vs. current estimate) still open — parks
  tension scaling in both the firmware schema (`CFG_TENSION_SCALE`) and the
  driver board (DNP load-cell path).
- Motor `Kt` + bus voltage: blocked on a specific motor selection; blocks
  detailed electrical sizing.
