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
> genuinely statically stable in 3D (**+10.1 mm**, up from **−21.6 mm**).
> **M6 done:** whole-body **dynamics** — and it overturned M5's headline.
> **M7 done:** the **trot** ([ADR-0011](DESIGN_DECISIONS.md)) — the first dynamic
> gait, **67 cm/s** (60× the crawl) inside the actuator envelope.
> **M8 done:** **closed-loop balance** ([ADR-0013](DESIGN_DECISIONS.md)) — DCM foot
> placement.
> **M9 done:** latency, retiming — and a 2.3× correction to M8's envelope, then
> the **lateral spine** recovers it ([ADR-0014](DESIGN_DECISIONS.md)).
> **M10 done:** the spine is ROM- not rate-limited; abduction and a faster drive
> both close as *not needed* ([ADR-0015](DESIGN_DECISIONS.md)).
> **M11 done:** the latency budget — solved as a fixed point, **the electronics is
> not the bottleneck** ([ADR-0016](DESIGN_DECISIONS.md)).
> **M12 done:** the actuation ramp modelled (**57 mm**), abduction costed, and the
> missing **disturbance requirement** finally stated ([ADR-0017](DESIGN_DECISIONS.md)).
> **M13 done:** the `dH/dt = 0` caveat **closed with a number** — large at trot, but
> mostly pitch, which the contacts resist ([ADR-0018](DESIGN_DECISIONS.md)).
> **M14 done:** the spine assist is **not free** — it costs ground friction, which
> reinstates a withdrawn μ requirement ([ADR-0019](DESIGN_DECISIONS.md)).
> **M15 done:** its **yaw couple** doubles that cost, so the trot slows to
> **50 cm/s** ([ADR-0020](DESIGN_DECISIONS.md)).
> **M16 done:** power & runtime — **NFR6 closed** at ~30 min / 900 m, and standing
> costs 76 % of moving for zero work ([ADR-0021](DESIGN_DECISIONS.md)).
> **M17 done:** an independent physics engine says LIPM is **conservative by ~2 %**,
> and finds a blind spot it cannot see ([ADR-0022](DESIGN_DECISIONS.md)).
> **M18 done:** the **battery is the thermal protection** — and that is a coincidence
> ([ADR-0023](DESIGN_DECISIONS.md)); on dualis 0.2 the energy books are **audited**
> and the runtime is **emergent**.
> **M19 done:** the winding runs **7.7 K above the skin** — M18's caveat answered,
> not restated ([ADR-0024](DESIGN_DECISIONS.md)).
> **M20 done:** the sway was **4 % optimistic**; the friction cost needs a balance
> controller to measure at all ([ADR-0025](DESIGN_DECISIONS.md)).
> **M21 done:** the envelope is **direction-dependent** and balance needs
> **compliant legs** ([ADR-0026](DESIGN_DECISIONS.md)).
> **M22 done:** the spine assist is **not a free offset**, and **NFR15 is not
> demonstrated** ([ADR-0027](DESIGN_DECISIONS.md)).
> **M23 done:** M21/M22 measured before the limit cycle — corrected, and the gap
> localises to the **spine term** ([ADR-0028](DESIGN_DECISIONS.md)).
> **M24 done:** the spine assist has **unity loop gain** — it is harmful and the
> "+14 %" is withdrawn ([ADR-0029](DESIGN_DECISIONS.md)).
> **M25 done:** planned deployment fixes the stability — and the spine **still
> buys nothing** ([ADR-0030](DESIGN_DECISIONS.md)).
> **M26 done:** the spine credit is authority in the **wrong axis** — the binding
> mode is **along-line** ([ADR-0031](DESIGN_DECISIONS.md)).
> **M27 done:** three actuators, three failures — the **architecture** is the limit,
> so **do not buy abduction yet** ([ADR-0032](DESIGN_DECISIONS.md)).
> **M28 done:** the **viable set**, computed exactly — **NFR15 is achievable** and the
> model was never the problem ([ADR-0033](DESIGN_DECISIONS.md)).
> **M29 done:** R2’s critical table was **stale** — NFR15 is met from **μ 0.6**, and it
> no longer justifies the 50 cm/s trot ([ADR-0034](DESIGN_DECISIONS.md)).
> **M30 done:** the spine’s friction cost is **real but ~14 % not ~100 %** — after
> four failed measurement designs ([ADR-0035](DESIGN_DECISIONS.md)). 357 Python + 17 Rust.

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

## Milestone M6 — Whole-body dynamics: the quasi-static answers were wrong (DONE)

**Goal.** Stop asking *"does the CoM project inside the feet?"* — a question about
a body standing still — and start asking *"can the contacts produce the forces the
motion requires?"* Delivered as `kinematics/src/tomcat_kin/dynamics.py`: the CoM
path differentiated in time, Newton–Euler balance, a per-foot ground-reaction
solve with an active-set unilateral constraint, friction cones, and the ZMP.

**It overturned two published conclusions and found a defect.**

1. ⚠️ **The binding constraint is TIPPING, not friction.** M5 concluded friction
   set the walk speed and demanded μ ≥ 0.70. The resolved per-foot forces show the
   body-level demand at M5's own gait is only **μ ≈ 0.35** — it would never have
   slipped. What fails is the **ZMP**: accelerating the CoM sideways to make the
   sway shifts the pressure point by `(h/g)·a` the other way, **~128 mm** against a
   96 mm track. **Slipping never binds at all** — swept over 0.6–6.0 s the aggregate demand never reaches μ = 0.8, while tipping only clears at **3.8 s**. NFR2g is withdrawn.
2. ⚠️ **M5's sway law was not physically realisable.** Its crossover ramp was
   *linear in position*, so velocity **stepped** — an impulse in acceleration,
   i.e. infinite force. A static check can never see this because it never
   differentiates the trajectory. Fixed with a **raised-cosine (C¹)** ramp: 23 %
   more peak acceleration, but finite. A grid-convergence test guards it.
3. **The walk is a 1.1 cm/s crawl**, not 4 cm/s — retuned to period **5.0 s**,
   sway **11°**, giving **+6.5 mm static / +6.4 mm ZMP**.

**Also closed here (the two remaining ❓ blockers):**

- **The motor.** ADR-0008's mass closure rode on a ~72 g part. A market survey
  ([reality check](notes/motor-reality-check.md)) finds the lightest purchasable
  part meeting the torque is **~120 g (131.7 g with driver)**. Torque *density*
  was fine — motors just come in discrete sizes, and the smallest one clearing the
  bar is ~2× the capability needed. **NFR5 rose 3.0 → 4.05 kg** (ADR-0010), which
  is if anything more biomimetic: a domestic cat is 4–5 kg.
- **Paw friction.** Published PU-on-concrete is 0.8–1.2 dry *and* wet; the
  resolved demand is **μ ≈ 0.055**. Never a real constraint.

**Cascades that had to be followed through:** the heavier body pushed the hip land
transient 465 → **600 N**, dropping the 1.5 mm cable below its own SF ≥ 4 target →
upsized to **1.75 mm** (SF ≈ 5.0, spool 8.0 → 8.75 mm). The real motor is 39 %
longer than the placeholder, so girdle height went **88 → 108 mm** (width still
clears the track).

**Honest bounds.** `dH/dt = 0` (classical ZMP form), *quantified* rather than
declared: the swing leg's neglected spin is worth **~1.0 mm** against the +6.4 mm
margin — a ~6× ratio, so it holds at this crawl speed, but it scales with leg
acceleration and would **not** hold for a fast gait. No contact compliance, no
touchdown impact, no motor dynamics. The force distribution is a heuristic, not
the optimum a real controller would solve. ⚠️ Sustained trot is a **2.1× overload**
on the motor's continuous rating — thermally limited, not torque limited.

**The consequence for the project.** A 1.1 cm/s statically stable crawl is not
cat-like by any measure. Static stability is now clearly a *demonstration mode*;
the operating mode has to be **dynamic**. That is the next milestone, and it is no
longer optional.

## Milestone M7 — The trot: a dynamic gait at cat-like speed (DONE)

**Goal.** M6 left the statically stable crawl capped at 1.1 cm/s and concluded a
dynamic gait was no longer optional. This is that gait.

**Delivered.** `gait.trot_params()` — diagonal pairs at duty 0.50 — plus the
physics a line support needs, in `dynamics.py`: `line_balance` (inverted-pendulum
offset, capture point/DCM, unbalanceable moment), `trot_sweep`,
`swing_joint_torque`, and `TROT_PHASE_OFFSETS`. **Default ~67 cm/s**, feasible and
thermally sustainable to **~96 cm/s**.

**Three things it took, and what each revealed:**

1. **Foot placement is a balance condition.** Two contacts cannot produce a
   moment about the line joining them, so the CoM's perpendicular offset from
   that line is an *unbalanceable* topple moment. The crawl's placement (feet
   50 mm ahead of the hips) puts the diagonal ~42 mm forward of the CoM — a
   one-signed moment, and roll rate accumulates **−4.1 rad/s per cycle: a fall
   inside one stride.** At `nominal_foot x = 0.005` the CoM rocks symmetrically
   ±11 mm *through* the line, the moment integrates to zero, and the roll is a
   **bounded ±0.4°**. The rocking is the gait, not a defect.
2. ⚠️ **A second C¹ defect — this time in the M2 foot trajectory.** The cycloidal
   swing begins and ends at zero hip-frame velocity while stance sweeps at
   `-stride/(duty·period)`, so foot velocity **steps** at liftoff and touchdown.
   Swing-leg torque was therefore impulsive — it *doubled with every grid
   refinement*, so it was never a number — and the paw landed scuffing forward at
   full stance speed. Replaced with a velocity-**matched** quintic. Same class of
   error as the M5 sway law, found the same way: by differentiating something only
   static checks had ever examined. It survived five milestones because at crawl
   speed it is invisible.
3. **P1 gets its first hard number.** Swing-leg torque is what caps trot speed,
   and it is only **0.11 N·m** at 67 cm/s — because tendon drive keeps the legs at
   95–110 g. Motors at the joints would pay this many times over.

**It also corrected M6.** ADR-0010 called sustained trot a *2.1× thermal
overload*; that compared a quasi-static **peak** against a **continuous** rating.
The right measure is RMS over the cycle: peak 1.26 N·m (0.65× the part's peak),
**RMS 0.40 N·m = 0.56× continuous**. Sustained trot is thermally fine.

**Honest bounds.** Touchdown **impact** is still unmodelled — the matched profile
removes the tangential scuff but not the vertical impulse. No contact compliance.
`swing_joint_torque` treats links as point masses (ignores spin inertia), so it
under-estimates. Flight phases (duty < 0.5) generate but are not analysed. And
this is an **open-loop trajectory check**: it shows the prescribed motion is
dynamically consistent, not that a controller can stabilise it against
disturbances.

## Milestone M8 — Closed-loop balance: step-to-step foot placement (DONE)

**Goal.** M7 ended on an honest caveat: the trot was an *open-loop* consistency
check. It showed the error *starts* at zero, not that it stays there — and the
trot is an inverted pendulum whose deviations grow **3.2× per step, 339× over
five**. This closes that loop.

**Delivered.** `control.py`: the step-to-step DCM plant (`StepPlant`, built from
the full model's `omega` and stance), the placement law, and the measurements
that say what it is worth — `one_step_envelope`, `rejection_envelope`.

**The law.** Working in the DCM (`xi = c + c_dot/omega`) makes the plant
first-order, so balance reduces to one placement per touchdown:
`p = nominal + (growth-beta)/(growth-1) * (xi - nominal)`.

⚠️ **The coefficient exceeds one (1.45): the foot goes *beyond* the DCM.**
Placing it *at* the DCM arrests the topple but leaves the body permanently
displaced — the robot looks stable and walks away sideways. That error was made
in the first draft here; only simulating it caught the difference, which is the
argument for the module existing rather than the law being asserted on paper.

**What binds: reach, not gain.**

| | |
|---|---|
| One-step envelope | **51 mm** |
| **Guaranteed rejection envelope** | **74 mm** |
| Binding direction | **rearward** (+153 mm forward vs −74 mm back) |

The envelope is **independent of the gain** — gain sets how fast recovery is,
reach sets whether it happens. The leg's asymmetric workspace is now a *balance*
parameter, not merely a workspace one.

**It puts a sharp number on ADR-0012.** A steady bias in the estimated DCM does
not average out: it becomes a **permanent lateral offset amplified 3.2×**, so
5 mm of estimation error is 16 mm of drift. "Detect contact" was never a
sufficient spec; the estimator needs a few millimetres of DCM accuracy.

**Honest bounds.** Reduced order — one perpendicular coordinate, constant CoM
height, point feet, instantaneous support transfer. A controller-design tool, not
a simulation of the robot: no swing dynamics, no actuator limits in the loop, no
double support. Only a static sensing bias is modelled, not **latency**. The
controller places feet but does not **retime** steps, and a real disturbance
perturbs the full 3D state rather than one scalar.

## Milestone M9 — Latency, retiming, and the spine as a balance actuator (DONE)

**Goal.** Close the two gaps M8 named: sensing **latency** and step **retiming**.
Chasing them found a mistake in M8's headline number, and then a better actuator
than the one being tuned.

⚠️ **The correction first.** The DCM is measured *perpendicular* to the diagonal
support line, and that direction is ~**90 % lateral**. M8 used the leg's raw
*fore-aft* reach as its placement authority — but the legs are **sagittal-only**
(abduction rejected in ADR-0009, track fixed), so a fore-aft shift buys
perpendicular authority only through its **0.44 projection**. M8's envelope was
overstated **2.3×**: **33 mm**, not 74 mm.

**The recovery: use the lateral spine.** It moves the CoM rather than the support
line, so it *adds* to foot placement — and it pushes almost exactly along the
perpendicular, precisely where the sagittal legs are weakest.

| actuator | perpendicular authority |
|---|---|
| Foot placement | 33 mm |
| **Lateral spine** (rate-limited to ±8.9° in a 150 ms stance) | 24 mm |
| **Combined** | **68 mm** — **+108 %** |

The spine was bought for the **crawl's static stability** (ADR-0009) and the trot
preset had it switched *off*. It turns out to be the dominant **dynamic** balance
actuator — a real dividend from a decision taken for an unrelated reason.

**Latency: linear cost, no cliff.** Prediction handles it; what remains is error
amplification `e^(omega*tau)` and less time under the corrective placement.
20 ms costs 18 % of the envelope — that is the budget.

**Retiming: speeds recovery, does NOT extend the envelope.** With the placement
saturated at reach `R`, `xi_end = R + (e-R)e^(omega*T)`: for `e < R` a longer
stance amplifies the correction; for `e > R` it grows for any `T`. **Timing buys
speed, never range.** A useful negative result — it takes retiming off the
critical path.

**Consequences.** **Leg abduction is worth revisiting**: ADR-0009 rejected it on
mass grounds when the question was *static* stability, but the dynamic case is
different — abduction points straight down the perpendicular. And the spine
figure is limited by **NFR2f (119°/s)**, sized for the righting reflex rather
than balance; a faster spine drive buys envelope directly (~39 mm if the full
±15° were reachable within a stance).

**Honest bounds.** All of ADR-0013's reduced-order caveats still stand, plus: the
spine is modelled as an instantaneous bounded CoM offset, ignoring its own
dynamics and the reaction torque it puts into the trunk.

## Milestone M10 — Both M9 follow-ups close: no change needed (DONE)

**Goal.** M9 left two questions, both expected to cost motors: revisit leg
**abduction** (ADR-0009 rejected it against a static requirement that no longer
applies), and consider a **faster spine drive** (NFR2f was sized for righting).
Neither turned out to be needed.

⚠️ **A correction first.** M9's spine figure clamped the authority with NFR2f's
**119°/s** — a *requirement floor* for the righting reflex, not a capability. The
real joint rate is `380 rpm × (8 mm spool / 20 mm arm)` = **~912°/s**, while a
full ±15° traverse inside a 150 ms stance needs only **200°/s**. The spine is
**ROM-limited, not rate-limited**, and M9 under-counted it by ~40 %:

| | M9 | corrected |
|---|---|---|
| Spine authority | 24 mm | **39 mm** |
| **Combined envelope** | 68 mm | **90 mm** |

**A faster spine drive: not needed.** Capability is already **8×** the
requirement and 4.6× what a full traverse needs.

**Leg abduction: not needed.** 90 mm of DCM envelope corresponds to rejecting a
**0.70 m/s lateral shove**. Abduction would add ~40 mm of authority for
**+4 motors = 528 g (13 % of the budget)** — buying capability the robot does not
need. (ADR-0009's original "puts mass in the limbs" objection was actually weak
for a tendon drive, where the motor sits in the girdle; the conclusion survives on
the stronger ground that the requirement is already met.)

**Widening the spine ROM: available, deliberately not taken.** It scales well
(±25° → 119 mm) and costs no motors, but **lateral is the stiffest spine axis** —
SPINE_TAIL_SPEC ranks compliance *axial > dorsoventral > lateral*, so ±15° is
already the narrowest by design. Widening fights the biomechanics the geometry
came from. Recorded so the lever is known rather than rediscovered.

**Net: motor count stays 19, mass stays 4.05 kg, no spec changes.** Two
milestones of balance work land on "the hardware you already have is enough",
which is the most useful answer available.

**Honest bounds.** The 0.70 m/s figure inherits every reduced-order caveat from
M8/M9, and assumes the spine's full ROM is free for balance — true in the trot
(`lateral_amplitude = 0`), but a future gait that uses lateral sway for something
else would spend that authority and close this differently.

## Milestone M11 — The latency budget: the electronics is not the bottleneck (DONE)

**Goal.** M9/M10 produced two requirements that no subsystem owned: NFR12
(≤20 ms balance latency) and NFR11 (≤3 mm DCM accuracy). Allocating the latency
turned the calculation inside out.

**The structural finding: latency is not an independent parameter.** A bigger
disturbance needs a bigger foothold correction; a bigger correction takes the leg
longer to execute; and that time *is* the staleness of the information the
controller committed on. It has to be solved as a **fixed point**
(`control.self_consistent_envelope`).

| term | value |
|---|---|
| Pipeline (contact + estimation + transport + compute) | **7.5 ms** allocated |
| **Actuation** (the leg repositioning its foothold) | **37 ms** |
| Whole loop | **~45 ms** |

⚠️ **NFR10 corrected 90 → 59 mm.** The 90 mm assumed zero latency. Still a
**0.46 m/s** lateral shove, but a third smaller than published — the third
correction to this figure, each time from a term assumed rather than measured.

⚠️ **NFR12 re-cast** from a whole-loop ≤20 ms to a **pipeline ≤7.5 ms**, since
37 of the 45 ms is the leg, not the electronics.

**The useful surprise: the envelope is nearly insensitive to the pipeline** —
2.5 → 20 ms costs only ~16 %. So:

- **Electronics and firmware have a comfortable budget.** Chasing microseconds on
  the CAN-FD bus would be effort in the wrong place; ADR-0005's architecture is not
  the constraint. A useful negative result for two subsystems.
- **Foot speed is the lever** — doubling the leg's spare speed buys more than
  deleting the entire electronics pipeline. Ceiling 5.93 m/s vs a 1.83 m/s nominal
  swing peak, so the headroom exists.
- **Or attack the 0.44 projection**, which inflates every correction 2.3×. That
  projection has now cost the design something three times, and it is the strongest
  remaining case for **revisiting leg abduction** — ADR-0015 closed that on
  *authority* grounds, which says nothing about *actuation time*.

**Honest bounds.** The 37 ms actuation term assumes a constant-velocity correction,
ignoring the accelerate/decelerate ramp and torque limits during it. It is now the
dominant term in the entire budget, which makes it the single most valuable thing
left to measure on real hardware.

## Milestone M12 — The ramp, abduction costed, and the requirement that was missing (DONE)

**Goal.** Close M11's two follow-ups: bench the actuation ramp, and re-examine leg
abduction on *actuation-time* grounds. Both are done — and underneath them sat a
gap neither had named.

**The ramp: modelled, and it barely matters.** A trapezoidal
accelerate/cruise/decelerate profile moves the envelope only **59.2 → 57.0 mm**
(−4 %). M11's caveat was over-cautious. Another P1 dividend: at 95 g the leg has
~**107 g** of foot acceleration available, so the move is **speed**-limited, not
acceleration-limited.

⚠️ A trap worth recording: computing that limit by driving *every* joint at peak
torque reports ~665 g — an artefact of the distal joint's near-singular inertia,
not a usable acceleration. The operational-space form gives the defensible ~107 g.

**Abduction, finally costed.** It points along the perpendicular (0.897) rather
than obliquely (0.442), so the same correction needs 2.3× less travel — actuation
drops **41 → 24 ms**, and the envelope rises **+42 % to +58 %** for **+4 motors,
528 g**.

⚠️ **But the decision was unanswerable as posed**, and that is the milestone's real
finding. ADR-0015 rejected abduction because "the requirement is already met" —
except **NFR13 was recording a *capability*, not a *requirement***. That number has
been corrected three times (74 → 33 → 90 → 59 → 57 mm), so the rejection rested on
a moving figure with nothing behind it.

**So the requirement is now stated (NFR15):** a 15 N / 0.1 s push (48 mm), an
unexpected 40 mm step, and a 10° lateral slope — all met by the shipped 57 mm with
~19 % margin. A 30 N hard shove (96 mm) is explicitly **out of scope**.

**Abduction therefore stays rejected, now on solid ground**, and remains a costed
option should a future requirement need rough terrain or hard shoves.

**The pattern, named.** Three corrections in a row shared one cause: **a capability
was being used where a requirement belonged.** NFR13 is re-labelled as measured
capability; NFR15 carries the requirement.

**Honest bounds.** NFR15's cases are `[assumed]` engineering scenarios — a *stated*
basis, which is the improvement, not a *validated* one. A real disturbance test
would supersede them.

## Milestone M13 — Closing the dH/dt caveat: large, but on a resistible axis (DONE)

**Goal.** Every dynamics milestone since M6 carried the same warning — the moment
balance assumes **`dH/dt = 0`**, and M6 said outright it "would not hold for a fast
gait". The trot is that gait. Convert the warning into a number.

⚠️ **A silent-zero bug, found on the way.** `angular_momentum_caveat` required
*exactly one* swing leg. A trot moves diagonal **pairs**, so two legs are always in
flight — every phase was skipped and it returned a reassuring **0.00 mm** having
evaluated nothing. A zero meaning "not measured" is worse than no number; fixed,
and a test now asserts the trot figure is non-zero.

**The magnitude: badly violated at trot.**

| gait | swing-leg ZMP-equivalent shift |
|---|---|
| Crawl | **1.0 mm** — negligible, as assumed |
| **Trot** | **42.5 mm** — 41× larger, comparable to the entire 57 mm envelope |

**The resolution: it lands mostly where the contacts can take it.** Two point
contacts resist every moment except the one about the line joining them. The swing
legs move mostly fore-aft, so their reaction is mostly **pitch** — and only ~**21 %**
reaches the destabilising diagonal. **M7's bounded roll survives** (0.39 → 0.31°
peak-to-peak, drift still small).

| term about the diagonal | vs gravity |
|---|---|
| Swing orbital (`m r × a`) | 22 % |
| Swing **spin** (`I α`, new in M13) | **3 %** |

**A third P1 dividend.** Slender-rod inertia goes as `m L²/12`, and tendon drive
keeps the legs at 95 g *and short* — so spin is negligible. With motors at the
joints it would be first-order. P1 has now paid off measurably three times: swing
torque (M7), foot acceleration (M12), link spin here.

**Honest bounds.** Only the **legs** are resolved — **trunk and spine angular
momentum is still unmodelled**, and the spine is 40 % of body mass and moves
laterally during balance. That is the natural remaining gap. Links are slender rods
about their own CoM: no products of inertia, adequate for a planar leg but not for
the righting reflex.

## Milestone M14 — The spine assist is not free (DONE)

**Goal.** Model the spine's reaction, flagged since ADR-0014 as "an instantaneous
bounded CoM offset, ignoring its own dynamics". The first thing checked turned out
to be more basic than the reaction torque, and more consequential.

**The physics that was missed.** Bending the spine is **internal motion**, and
internal motion cannot move the whole-body CoM. Moving it *relative to the planted
feet* requires a horizontal **ground reaction** — friction. `control.py` had been
adding the spine assist straight to the DCM, free of charge.

> `mu_spine >= 4d/(t² g)`, on top of what the gait already spends.

For the shipped trot: full 39.4 mm of spine authority needs **μ = 0.71**, plus the
gait's own **0.145** → **μ = 0.86 total**, against a floor supplying 0.8–1.2.

**So spine authority has a THIRD constraint.** The history of this one number is
worth recording: M9 clamped it by **rate** (wrongly — that was a requirement floor,
not a capability), M10 corrected it to **ROM**, and friction was never checked.

| floor μ | spine authority | envelope | binds on |
|---|---|---|---|
| 0.6 | 25.1 mm | 43.7 mm | friction |
| **0.7** | **30.6 mm** | **48.2 mm** | friction |
| 0.8 | 36.1 mm | 53.7 mm | friction |
| ≥ 1.0 | 39.4 mm | **57.0 mm** | ROM |

⚠️ **This reinstates a requirement ADR-0010 withdrew — at almost the same number,
for a different mechanism.** ADR-0010 struck out μ ≥ 0.70 because friction was not
the binding constraint for the *crawl's sway crossover*. Correct, for that
mechanism. But the *trot's spine balance action* is a different use of the same
actuator, and needs **μ ≥ 0.70** for NFR15 to hold. Not a coincidence: both are
"shift the CoM laterally by tens of mm inside one stance".

**Consequences.** New **NFR16 (μ ≥ 0.70)**, and the **paw-pad handoff to mechanical
is back on** — TPU ~80A must again be shown to deliver it. Published PU-on-concrete
is 0.8–1.2 so it should pass, but it is load-bearing now, not incidental.

**Honest bounds.** This covers the *translational* cost of the assist. The spine's
own **reaction torque on the trunk** — the yaw couple its lateral swing produces —
is still unmodelled, and remains the open item.

## Milestone M15 — The spine's yaw couple, and the speed it costs (DONE)

**Goal.** M14 costed only the *translational* half of the spine's friction bill and
left its **reaction torque on the trunk** open. It is not a footnote.

**The second cost.** The spine's swing is **asymmetric** — its tip travels ~**91 mm**
while its base stays put, about twice the 44 mm CoM shift — so it dumps angular
momentum about the **vertical** axis into the trunk. Two contacts resist that with a
friction **couple**, and a couple loads each foot with the **full** force, not half.

| cost at full ROM | μ |
|---|---|
| Translation (M14) | 0.98 |
| **Yaw couple** (M15) | **0.27** |
| Gait's own | 0.145 |
| **Total** | **~1.4** — more than any ordinary floor |

⚠️ **Profile shaping does not help — measured, not assumed.** The three lateral
spine joints can be commanded differentially, and an S-bend is up to **7.5×** more
yaw-efficient per degree. But it loses more CoM shift than it saves in friction:
swept over all profiles, the **uniform** command is already optimal for a given
budget. A tempting lever that is a dead end.

**So the trot slows: 0.30 → 0.40 s, 67 → 50 cm/s.** Both costs scale as
`1/stance²`, so a longer stance buys robustness fast — and the shipped default has
to meet its own stated requirement on a realistic μ = 0.8 floor. Same call M6 made
when dynamics showed the 1.4 s crawl infeasible.

| period | speed | usable spine @ μ 0.8 | envelope | NFR15? |
|---|---|---|---|---|
| 0.30 s | 66.7 cm/s | 20.5 mm | 40.2 mm | **NO** |
| **0.40 s** | **50.0 cm/s** | **36.5 mm** | **51.8 mm** | **yes** |

**The governing relationship, finally visible:** *speed and disturbance robustness
trade against each other through friction, at a steep `1/t²` rate.* That was
invisible while the spine assist was modelled as free.

⚠️ **A side effect worth watching:** a longer stance means more time to topple, so
per-step growth rises **3.21 → 4.73**. A DCM estimation bias that was a standing
offset at 0.3 s now runs the loop away past **6.9 mm**. NFR11 asks ≤ 3 mm, leaving
~2.3× margin — smaller than before, and it moved for a reason unrelated to sensing.

## Milestone M16 — Power and runtime: NFR6 closed after fifteen milestones (DONE)

**Goal.** Fifteen milestones established what the robot can *do*; none asked how
long it could do it for. **NFR6 read `TBD` since M1**, and the 300 g battery had
never been checked against a load.

| | power | endurance |
|---|---|---|
| **Trotting** (50 cm/s) | **83.6 W** | **30 min, ~900 m** |
| Standing | 67.2 W | 37 min |
| Standing **with the ADR-0003 brake** | ~15 W | **168 min** |

⚠️ **Standing costs 76 % of what moving costs, and does no work.** A cable can
only pull, so a tendon-driven joint holds posture with **motor current**, which
burns `I²R` whether or not anything moves. ADR-0003 called the power-off brake
"essential" on qualitative grounds — this is what it is worth: **4.5×** standing
endurance. For a pet robot, idling is most of the duty cycle.

⚠️ **The drive is only 39 % efficient** — copper loss (42 W) exceeds useful
mechanical work (27 W). That is the transmission, not the gait.

**A lever this exposes.** Motor torque is `τ_joint · r_spool / r_joint`, so copper
loss goes as the **square** of that ratio — the joint moment arm sets **efficiency**
as well as cable tension, a second role LEG_TENDON_SPEC never costed:

| moment arms | total | endurance | hip sheave |
|---|---|---|---|
| 1.00× (shipped) | 83.6 W | 30 min | 56 mm |
| **1.25×** | 68.4 W | **37 min (+23 %)** | 70 mm |
| 2.00× | 52.0 W | 48 min (+60 %) | 112 mm — clearly out |

**1.25× is recorded as a costed option, not adopted** — it ripples through mass,
packaging and swing inertia, and that is a mechanical decision, not a modelling one.

**Honest bounds.** Deliberately pessimistic and flagged: **no regeneration**, `I²R`
on the phase-to-phase resistance, and **no iron, switching or gearbox losses** — so
real draw will be *higher*. Battery density (175 Wh/kg) and usable fraction (80 %)
are `[assumed]` and could move the answer ±25 % on their own.

> **Before more milestones:** [OPEN_RISKS.md](OPEN_RISKS.md) ranks what is still
> assumed by what it would break. The modelling thread has reached the point where
> its headline numbers are gated by **procurement, not analysis**.

## Milestone M17 — Checking the reduced-order model with an independent engine (DONE)

**Goal.** Every balance number since M8 rests on a **Linear Inverted Pendulum
Model** — point mass, constant height, `dH/dt = 0`. OPEN_RISKS §6 still listed the
trunk angular-momentum terms as *expected* small. A MuJoCo model, generated from the
live parameters, now checks that from outside.

**The gate first.** A drifted physics model produces confident wrong numbers, so
agreement is asserted before anything else — mass **0.00000 g**, all four paw tips
**0.000000 mm**, CoM **0.0000 mm**.

| what LIPM assumes | what the rigid-body model measures |
|---|---|
| diverges at `ω = 7.71` rad/s | **7.55 rad/s — 2 % SLOWER** |
| constant CoM height | drops **0.5–4.5 mm** of 162 mm |
| one topple axis | ⚠️ **two, 52.4° apart** |

**The reassuring result.** Real divergence is *slower* than predicted — distributed
inertia resists the topple a point mass cannot. **This closes the §6 `dH/dt` item**,
not by computing each term but by measuring their aggregate effect on the only
quantity they could change. The model errs in the safe direction.

⚠️ **The new one.** `StepPlant` collapses balance onto a single axis with a fixed
`projection = 0.4417`. Both diagonals match that *magnitude* exactly — which is why
the 1-D reduction works — but their perpendiculars are **52.4° apart**, so
consecutive steps correct along different directions and a disturbance handled on
one diagonal keeps a 0.61 component on the next. Recorded as a **known structural
limit**, not yet costed.

**An old error, independently reproduced.** M8 caught by reasoning that placing the
foot *at* the DCM arrests but does not recover. In MuJoCo the capture law fell at
**every** disturbance including 6.5 mm, while the recover law survived.

⚠️ **What this does not settle.** The closed-loop harness recovers to ~13 mm against
a predicted 30.34 mm, but its *undisturbed* baseline drifts up to 25 mm — the same
order as the signal. **That shortfall is not reportable**: a harness whose noise
floor matches its signal cannot adjudicate, and claiming otherwise would repeat this
project's own recurring mistake.

⚠️ **And it validates the model, not the inputs.** No engine knows what a motor
weighs or how grippy a pad is. **OPEN_RISKS R1 and R2 are untouched.**

## Milestone M18 — Thermal duty: the battery is what saves it (DONE)

**Goal.** ADR-0021 checked the motor **electrically** (2.79 A peak vs 4.19 A rated)
and called it comfortable. That is not the thermal question — the thermal question is
whether the heat can *leave*. OPEN_RISKS R5 parked it as "gated on having the motor".
**It was not.** A lumped-capacitance model answers it (`thermal/`, on the
`dualis-thermal` crate).

**The boundary that decides it** is the girdle, not the motor: P1 centralises six
motors, so they do not each have free air.

| front girdle, 6 motors, 21 W | continuous | one battery |
|---|---|---|
| trot, polished | **113.7 °C** | 67.1 °C |
| trot, anodised | 74.9 °C | 59.7 °C |
| stand w/o brake, polished | **134.1 °C** | 85.7 °C |

⚠️ **The battery is the thermal protection, by coincidence.** A bare girdle takes
**~47 min** to get most of the way up; the pack feeds it for **30 min**. The robot
runs out of energy before it overheats — and nothing in the design put that margin
there. **Tether it and it is gone.**

**Decision: anodise the girdles, worth ~39 K.** Radiation is the same order as
still-air convection here, so ε 0.09 → 0.90 costs no mass, volume or power.
**Surface finish is a thermal parameter, not a cosmetic one.**

⚠️ **The first place P1 charges rather than pays.** Centralising six motors costs
**38 % of the rejection area** (486 → 302 cm²). P1 has paid off four times — swing
torque, foot acceleration, link spin, light legs. This is the bill.

**And standing without the brake is the worst case**, not trotting: 4.35 W/motor vs
3.50, because a cable can only pull. ADR-0021 reached the brake from *runtime*; this
reaches it from *heat*.

**R1 decoupled:** motor mass 132 → 200 g moves the time constant 17.4 → 26.5 min and
leaves the equilibrium **exactly unchanged**. A heavier motor buys time, never a
lower final temperature.

**Audited (dualis 0.2).** The first pass compared two numbers by hand. The pack is now
a domain on the same bus, so the kernel checks conservation every step, the runtime is
**emergent** (30.17 min vs `power.py`'s 30.16 — an independent cross-check), and a
deliberately leaky pack is **refused** — an audit that cannot fail is decoration.

**And that surfaced what the hand-comparison missed:** anodising shrinks the
*dependence on the coincidence*, not just the temperature.

| | at the flat pack | continuous | gap |
|---|---|---|---|
| polished | 67.1 °C | 113.7 °C | **47 K** |
| **anodised** | 59.6 °C | 74.9 °C | **15 K** |

A bare girdle survives only because the pack dies first (47 % of its settled rise); an
anodised one sits at 69 %, so **tethering it is no longer a cliff**.

⚠️ **A self-caught error, of this project's oldest kind.** The 53 min first published
for M18 was `LumpedMass::time_constant` — `C/(hA)`, **convection only**, identical for
every emissivity. Measured from the transient it is **46.6 min polished, 25.6 min
anodised**. Correcting it sharpens the conclusion:

| | effective τ | vs 30 min | why it is safe |
|---|---|---|---|
| polished | 46.6 min | outlasts | **only because the pack dies first** |
| **anodised** | 25.6 min | shorter | **on its own merits** (≈75 °C equilibrium) |

A **nominal figure standing where a measured one belonged** — the fifth time, and the
first arriving through a dependency rather than our own model. Reported upstream.

⚠️ These are **assembly-skin** temperatures — windings run hotter, and copper loss is
the only source modelled. Reality is worse.

## Milestone M19 — The winding temperature: a caveat discharged (DONE)

**Goal.** Every M18 number carried the same warning — *"a lumped mass has ONE
temperature; the real winding is hotter, and the winding is what fails."* That was a
limit of the tool, not a judgement: two `LumpedMass` bodies could not be joined by a
conductance. Reported upstream ([dualis#2](https://github.com/YounghyeonPark/dualis/issues/2)),
`ThermalNetwork` shipped in 0.3, and **winding → stator → housing → girdle → air**
became expressible.

| | winding | stator | housing | skin (M18's figure) |
|---|---|---|---|---|
| polished, continuous | **121.4 °C** | 117.5 | 116.1 | 113.7 |
| anodised, continuous | **82.6 °C** | 78.7 | 77.2 | 74.9 |
| anodised, one battery | **62.0 °C** | — | — | 55.4 |

**+7.7 K, and identical for both finishes.** The skin finish sets where the stack
sits; the joints set how far it spreads. **Two independent levers, and only the
second is uncertain** — which is why the sweep is the deliverable:

| joints | above skin |
|---|---|
| 0.25× | **30.7 K** |
| 1.00× | 7.7 K |
| 4.00× | 1.9 K |

**The verdict holds, which is the useful part.** Anodised, the winding is at 82.6 °C
— inside class F — and the stator at 78.7 °C, which matters because the rotor magnets
sit against it. Polished, the stator hits **117.5 °C**, past ordinary NdFeB grades.
**NFR18 was a 39 K saving; it is also what keeps the magnets in range.**

⚠️ **And a trap named.** M18 leaned on a reassuring Biot number of 5e-4 for the whole
assembly — which is the Biot number of a *solid block*, not of a structure with motors
and air gaps in it. 0.3 returns `None` for an interior node rather than a comforting
figure.

## Milestone M20 — The spine in simulation: a bug, and a wall (DONE)

**Goal.** M17 ran a rigid trunk, so it tested only the feet-only envelope. The spine
supplies ~23 mm of the ~53 mm headline — **44 % of what NFR15 is checked against was
outside the simulation.** Adding the three lateral joints found a bug going in and a
wall coming out.

**The bug: the fore legs are not at the spine tip.** `center_of_mass_y` argued that
symmetric left/right track offsets cancel. They do — but the **fore-aft** offset of a
leg's CoM does not: the yaw rotates it into y and **both** fore legs push the same
way. In a trot stance that CoM sits ~52 mm behind the hip.

| | full-ROM sway |
|---|---|
| as published | 43.97 mm |
| **corrected** | **42.22 mm** |
| MuJoCo | 42.219 mm |

**4.0 % optimistic.** Envelope **53.90 → 52.72 mm** — NFR15 still passes, with
4.72 mm of margin instead of 5.90.

**The static premise checks out exactly.** Holding a full sway needs **μ 0.006**,
three orders under NFR16. `lateral_spine_loads` said holding costs nothing; it does.

⚠️ **The wall: ADR-0019/0020's dynamic friction costs cannot be measured without a
balance controller.** Three designs, all rejected — a free root topples inside one
stance (contact lost 13 % of the time even at quarter amplitude over 3× the
duration); locking roll *breaks the mechanism*, because the legs are planar and a
sway over planted feet needs slip or roll; and reading μ during a sweep just measures
a fall. **The 0.98 + 0.27 that slowed the trot from 67 to 50 cm/s is untested, not
refuted.**

**What that sharpens.** M17 blamed harness drift and suggested regulating the
along-line component. Too small a diagnosis: **both** the envelope magnitude and the
friction costs are gated on the same missing piece — a closed-loop balance controller
in the simulation. That is now one blocking item instead of two vague ones.

## Milestone M21 — Closed-loop balance: the envelope, measured (DONE)

**Goal.** M20 named one blocking item — a balance controller in simulation. Building
it corrected my own diagnosis twice before it produced a number.

⚠️ **My swing profile was the real bug, not the along-line DCM.** M17 blamed its
drift on the unregulated along-line component, and that component *did* run
+1.7 → +22 → +43 → +90 mm. But upstream of it, **my foot landed at 0.31 m/s**: a
`sin(πu)` arc has non-zero slope at touchdown. Swapping to `(1-cos(2πu))/2` took the
run from 14 steps to 40 with **no along-line regulation at all**. This is the same
C⁰ defect **M5 and M6 already fixed** in the shipped gait, reintroduced by hand.

**Balance needs compliant legs — the enabling result.**

| leg `kp` | steps | mean \|DCM\| first 10 → last 10 |
|---|---|---|
| **80** | **40, never fell** | 1.99 → **1.52 mm** |
| 250 | 23 | 6.50 → 45.8 (diverging) |
| 500 | 24 | 8.36 → 28.1 (diverging) |

Stiff servos make load transfer bang-bang — ±1 mm of differential leg extension
swings the CoP across the whole ±109 mm foot separation. The mechanical design
already specifies passive compliance for impact tolerance; **the balance loop does
not close without it**, which is a reason it was never chosen for.

**The result: the envelope is direction-dependent.** `StepPlant` quotes one number,
30.34 mm, for every direction. Measured:

| 60° | 180° | 0° | 240° | 120° | 300° |
|---|---|---|---|---|---|
| **65.7** | 44.6 | 42.8 | 37.4 | 22.3 | **19.3 mm** |

**3.4× spread, and the worst direction is 64 % of the prediction.** M17 found the
52.4° axis split but could not cost it — this is the cost, and the reduction
**over-promises in the direction that matters**.

⚠️ **Not a requirement verdict.** Peak baseline noise is ~11 mm against a 19.3 mm
worst case — only 1.75×, so that figure is uncertain. And the trunk is rigid, so the
spine's ~22 mm is not in the loop; the spine acts most strongly in the lateral
directions where the feet are weakest. **Whether NFR15's 48 mm survives is the next
question, not this one's answer.**

## Milestone M22 — The spine in the balance loop (DONE)

**Goal.** M21 measured with a **rigid trunk**, so the spine's ~22 mm was outside the
loop and NFR15 stayed open. This puts it in.

**The legs and the spine want opposite gains.** M21 found balance needs *compliant*
legs. The lateral spine is the reverse — it carries the whole forequarters, and at
the leg's gain it wobbles enough to fell a clean baseline in **10 steps**. At
`kp = 1000` it is quiet again. **A single "servo gain" would have hidden this.**

⚠️ **The spine's authority is not a static offset.** `control.py` books 36.6 mm as
free and always available. In dynamics the usable window is narrow: a gentle assist
(gain 0.2) helps, and by gain 0.4 the robot falls at the *smallest* disturbance
tested. The sway swings the whole forequarters and the reaction destabilises.

## Milestone M23 — Correcting my own measurement, and localising the gap (DONE)

⚠️ **M21 and M22 disturbed the robot at `t = 0`** — one settle after being placed,
before it had entered its limit cycle. That is not a trotting robot. Disturbing after
2, 4 or 6 undisturbed steps gives **systematically larger** envelopes; the +0 column
was the lowest in every direction.

Found while trying to *explain* the direction dependence, not while re-checking it.
The tell: opposite directions on the same axis disagreed wildly (0° = 41.8 mm,
180° = 19.3 mm), which no property of the robot explains and an unsettled start does.

| | published | **corrected** |
|---|---|---|
| worst direction, feet only | 19.3 mm | **25.3 mm** |
| worst direction, spine at 0.2 | 22.5 mm | **28.9 mm** |
| direction spread | 3.4× | **2.6×** |
| NFR15 shortfall | 2.3× | **1.66×** |

**Every qualitative conclusion survives** — direction-dependent, worst direction short
of prediction, spine helps modestly (+14 %), narrow gain window, NFR15 not
demonstrated.

**And the correction sharpens it.** Split by term:

| | predicted | measured worst | achieved |
|---|---|---|---|
| **feet only** | 30.3 mm | 25.3 mm | **84 %** |
| **with spine** | 52.7 mm | 28.9 mm | **55 %** |

⚠️ **The foot-placement model is nearly right; the spine credit is what does not
materialise.** `control.py` books 36.6 mm of static spine authority; measured, the
spine buys **3.6 mm** of worst-case envelope. The gap is one term, not the model.

⚠️ Second time in this project that the **harness**, not the model, produced the
wrong number — after M17's drift. A harness is an experiment and needs its own controls.

## Milestone M24 — Why the spine credit is not being spent (DONE)

M23 localised the envelope gap to the spine term and asked whether better control
could extract more than 3.6 mm from `control.py`'s 36.6 mm credit. Before building a
controller, I measured why the existing assist did so little. **It does worse than
little.**

**The law has unity loop gain by construction.** `q = -gain · e / 0.169`, and a sway
of `q` moves the CoM by `0.169 · q = -gain · e`. The actuator sits directly in the
position feedback path with no attenuation, so with any lag it is marginal near 1.
On the **undisturbed** baseline:

| spine gain | mean \|DCM\| over 20 steps | |
|---|---|---|
| **0.0** | **2.15 mm** | clean |
| 0.2 | **11.43 mm** | **5× worse, with no disturbance at all** |
| 0.5 | — | falls at step 6 |
| 1.0 | — | falls at step 4 |

⚠️ **So M22/M23's "+14 % from the spine" is withdrawn** — measured inside the noise
the assist itself created. On a settled cycle the worst direction is **28.9 mm with
the assist and 28.9 mm without**.

**Two things this does not show.** The **motor is not the limit** (open-loop ramps to
full ROM survive 300°/s, so ADR-0019's "ROM-limited, not rate-limited" stands for the
drive), and **slew-limiting does not rescue it** (a 3 rad/s limit left gain 0.5 still
collapsing). It is a loop-gain problem, not a bandwidth one.

⚠️ **And a non-result, recorded.** I tried to show the 36.6 mm is physically
realisable by holding a full sway while trotting. **Two runs disagreed — 44.0 mm and
16.5 mm** — because the offset is not steady: it oscillates through zero and drifts.
Averaging its magnitude reads a drift as a bias. **How much the spine can hold
against planted feet is unmeasured**, and my first answer was an artefact of the
statistic.

## Milestone M25 — Planned spine deployment: the structure was the fault, the credit still is (DONE)

M24 showed the reactive assist has unity loop gain and is harmful. That left the good
version of the question open: unreachable credit, or unreachable *by that structure*?

**The fix is structural.** The spine target is decided **once per stance**, at the
same instant the foot placement is committed, then executed **open-loop** as a C¹ ramp.
The loop closes at the step rate — exactly like the foot placement, the one structure
in this harness that works.

| spine gain | reactive baseline | **planned baseline** |
|---|---|---|
| 0.2 | 11.43 mm | **1.64 mm** |
| 0.5 | falls at step 6 | **1.38 mm** |
| 1.0 | falls at step 4 | **1.55 mm** |

**Stable to gain 1.0, and it slightly improves the baseline** — the cleanest possible
confirmation that ADR-0029's instability was the structure, not the actuator.

⚠️ **And it still buys nothing.** At **0.23 mm** resolution (earlier sweeps ran at
~3.6 mm and were quantising the answer):

| direction | gain 0.0 | gain 1.0 | best gain |
|---|---|---|---|
| **120° (worst)** | 29.62 mm | 28.26 | **+0.23 mm** |
| 0° | 54.95 | 59.02 | +4.07 |
| 180° | 65.35 | **53.14** | 0.00 — gain 1.0 *costs* 12 mm |

**A stable implementation of the model's own mechanism, at full authority, adds
0.23 mm to the worst case against a credited 36.6 mm.** M23 and M24 could not
attribute the gap; this one can. **The credit is unsupported, not merely unreached.**

⚠️ Still not an optimal controller — but the burden has moved: `plant.spine` is a
modelling claim with no supporting measurement, sitting in the middle of NFR15's margin.

## Milestone M26 — Why the spine credit does not materialise (DONE)

M25 left `plant.spine = 36.6 mm` to be justified or withdrawn, with two candidate
explanations. **Both are wrong.**

**Not double-spent.** Instrumented, `simulate` invokes the assist on **1 of 400
steps** of a recovery — the deadbeat placement nulls the error immediately after, so
it is never asked for again. Cumulative demand is 1.0× full ROM. I was wrong to
suspect the arithmetic.

**The real reason is the failure mode.** Logging a failing recovery at the worst
direction:

| step | perp | **para** | foot dx | contacts |
|---|---|---|---|---|
| 0 | -19.0 mm | **+71.4 mm** | 35.6 mm | 1 |
| 1 | -63.2 | **-115.2** | **saturated** | 1 |
| 2 | -134.2 | **+114.3** | **saturated** | 1 |

⚠️ **The along-line component is 2–4× the perpendicular one, and nothing controls
it.** Feet move fore-aft (acting on the line's position); the spine acts laterally.
Neither addresses motion *along* the support line. The credit is real lateral
authority booked against a failure mode that is not lateral.

⚠️ **And the support is barely a line** — `ncon = 1` through most of a recovery. The
robot is on **one foot**, so the geometry the reduced-order model rests on does not
hold while it is recovering.

**This corrects M21.** ADR-0026 concluded along-line regulation was unnecessary once
the C¹ swing was fixed. True for the **undisturbed baseline**; false under a
**disturbance**, where that axis is exactly what runs away. I conflated "the baseline
is quiet" with "the axis is controlled".

**Decision: `plant.spine` is re-scoped, not withdrawn.** The number is a correct
statement about lateral authority. What is unsupported is adding it to a
**single-axis** envelope as though the binding constraint were perpendicular.

**The whole M17→M26 arc resolves into one statement:** the trot's balance problem is
two-dimensional with two differently-controlled axes, and the reduced-order model
collapses it to one.

## Milestone M27 — The free along-line actuator exists, and it changes the verdict (DONE)

M26 pointed at ADR-0017's rejected **leg abduction** (+4 motors, **528 g**) as the
only costed option supplying along-line authority. Before recommending that, the free
option had to be exhausted.

**It exists, and M21 wrote it off wrongly.** Differential stance-leg extension shifts
the CoP *along* the support line — exactly the missing axis. M21 measured it at
`kp = 500`, found it bang-bang, then discovered compliance is what makes balance work
at all and **never came back**. Re-measured at the shipped `kp = 80`:

| | stiff (`kp` 500) | **compliant (`kp` 80)** |
|---|---|---|
| points keeping 2 contacts | 1 of 7 | **5 of 7** |
| normal force | collapses to 1.5–11 N | **39.67 N throughout** |
| CoP response | saturated at a foot | **linear, −39.3 mm/mm** |

⚠️ **And it buys no envelope either** — best case **+1.8 mm** (two bisection steps)
across both signs and four gains, while degrading the baseline from 1.38 to 5.53 mm.

**The pattern is the finding:**

| actuator | credited | delivered | how it failed |
|---|---|---|---|
| Lateral spine | 36.6 mm | **+0.23 mm** | wrong axis (M26) |
| CoP / weight shift | ±109 mm of CoP | **+1.8 mm** | degrades the baseline |
| Foot placement | 30.3 mm | 25.3 mm (84 %) | the one that mostly works |

**Only the actuator the controller was designed around delivers.** That is not three
coincidences about three actuators; it is one fact about the controller.

**Decision: do NOT reopen leg abduction on these grounds.** Adding 528 g of authority
to a controller that cannot exploit what it already has is mass spent on a problem it
does not solve. ADR-0017's rejection stands — on new reasoning, since its original
basis ("NFR15 already met") no longer holds.

## Milestone M28 — The viable set: what ANY controller could do (DONE)

Every envelope so far has been the achievement of *some controller*, which left the
recurring question unanswerable: model optimistic, or controller poor? This removes
the controller from the question — and it is **exact**, not optimised.

Over one stance with the CoP free inside the support set `S`,
`ξ(T) = g·ξ(0) − (g−1)·u` where `u` is the exponentially-weighted mean of the CoP.
`S` is convex, so `u` ranges over exactly `S`. The recoverable set follows in closed
form as a Minkowski recursion `Rₖ₊₁ = (Rₖ + (g−1)Sₖ)/g`. Converges by 6 steps;
the 1-step case matches the closed form to **9×10⁻¹⁴**.

| | worst direction |
|---|---|
| **Viable, feet only (exact, ANY controller)** | **29.8 mm** |
| `control.py` feet-only, 1-D | 30.3 mm |
| MuJoCo harness measured | 28.9 mm |
| **Viable, + spine (exact)** | **62.7 mm** |
| `control.py` + spine, 1-D | 52.7 mm |
| **NFR15 requires** | **48.0 mm** |

⚠️ **Three conclusions reverse.**

**1. The reduced-order model was never optimistic.** Feet-only is within **2 %** of
the exact limit; with-spine is **conservative** (52.7 vs 62.7). M23's "the spine
credit does not materialise" is **wrong in sign**.

**2. The foot-placement controller is near-optimal** — 28.9 against a true 29.8 mm is
**97 %**, not the 84 % "shortfall" M23 read. M27's indictment of the architecture
holds for the spine, **not** the feet.

**3. NFR15 is ACHIEVABLE** — 62.7 mm viable against 48 mm required. The authority
exists, so M24–M27's failure to spend it is a control problem **with a proven
target**. Building the whole-body controller is now justified work, not a hope.

**And a geometric correction to M26:** along its own axis the spine credit adds
exactly its length (36.6 mm, to the millimetre) — but the viable set is **slanted**,
because the trot's support is diagonal, so the fore-aft gain is **larger still
(63 mm)**. M26's mechanism stands; "it only helps laterally" does not follow.

**Leg abduction stays rejected, now on solid ground:** not "the controller cannot use
what it has", but **the existing authority is sufficient for the requirement**.

## Milestone M29 — Re-deriving the critical risk, and unblocking a shipped decision (DONE)

M28 made the viable set exact and instant. The first thing worth re-deriving with it
is **R2 (paw friction)** — one of only two CRITICAL risks, and the table that drove
both the NFR16 friction floor and ADR-0020's trot slowdown.

⚠️ **The published R2 table cannot be reproduced.** It quotes 40.2 / 48.1 / 53.9 mm
at μ 0.5 / 0.7 / 0.9. Neither current function produces those — and
`self_consistent_envelope` **takes no `floor_mu` at all**, so it cannot produce a
μ-dependent column. The table predates M20's sway correction and has been stale in a
CRITICAL section since. **Stale numbers in a risk register are worse than missing
ones: they are load-bearing and they look checked.**

**Re-derived exactly** (worst over 24 directions):

| stance | speed | μ 0.4 | 0.5 | **0.6** | 0.7 | 0.8 |
|---|---|---|---|---|---|---|
| 0.40 s | 50 cm/s | 42.6 | 47.6 | **52.6** | 57.7 | 62.7 |
| **0.30 s** | **67 cm/s** | 42.5 | 45.3 | **48.1** | 50.9 | 53.7 |

**NFR15's 48 mm is met from μ ≥ 0.6 at both speeds** — where R2 implied μ 0.70 with
*no margin*. At the NFR16 floor of 0.70 the margin is **20 %**, not zero.

⚠️ **And NFR15 no longer justifies the 50 cm/s trot.** ADR-0020 slowed the gait
67 → 50 cm/s partly because the envelope fell short. It does not. The fast gait is
*better* on the other axis ADR-0020 flagged too — per-step growth **3.21 at 0.30 s vs
4.73 at 0.40 s**, which widens the sensing margin.

**But the slowdown is NOT reversed.** ADR-0020 rested on two things and only one is
answered; its **friction accounting is still un-cross-checked** (ADR-0025 could not
measure it, and the viable set inherits rather than tests it). Reinstating 67 cm/s on
half an argument would repeat the error this milestone is correcting. **The speed
decision is now blocked on one specific measurement**, which is a better place than
blocked on a model.

**R2 drops from CRITICAL to SIGNIFICANT** — the drag test still matters, but the
threshold moved from "μ 0.70, no margin" to "μ 0.6", which a typical dry floor clears.

## Milestone M30 — The friction cost, measured at last (DONE)

M29 left the trot-speed decision blocked on one thing: **is ADR-0019/0020's friction
accounting right?** M20 could not measure it because the robot fell; the M21 harness
holds a settled trot.

⚠️ **Four designs failed first, and the failures are the useful part:**

| design | read | why it is wrong |
|---|---|---|
| per-contact \|fₜ\|/fₙ | pinned at the cone limit | a foot with 1.5 N at touchdown saturates any ratio |
| aggregate \|Σfₜ\|/Σfₙ | **3.238** | tangential at 3× normal is impossible — impact transients |
| foot slip | 0.4–2.5 mm, no trend | the spine's share sits inside a ~1 mm noise floor |
| CoM shift, unpaired | **sd 10–15 mm** | the effect is a few mm; averaging 5 trials showed nothing |

**Force is the wrong observable for a legged robot in contact.** What works is a
**paired** design — the simulator is deterministic, so the same deployment phase at
two frictions differs *only* by the friction:

| floor μ | CoM shift lost vs μ 5.0 | t (n = 11) |
|---|---|---|
| 0.20 | **−9.64 mm** | −2.23 |
| 0.40 | −8.01 mm | −1.83 |
| 0.70 | −5.71 mm | −1.32 |
| 1.20 | −1.11 mm | −0.22 |

**The mechanism is confirmed** — monotone across five conditions, exactly the sign
ADR-0019 predicts. ⚠️ **But the cost is far smaller than claimed**: at μ 0.70 the loss
is **5.7 mm of a 42.2 mm sway (14 %)**, where ADR-0020's accounting implies near-total
loss below μ 0.71. Even μ 0.20 costs only ~23 %.

**This corroborates M29 independently** — that found NFR15 met from μ 0.6 from the
*envelope* side; this finds the penalty overstated from the *mechanism* side.

⚠️ **The decision does not move.** Only μ 0.20 reaches |t| > 2.1, and n is capped at
11 because low-friction runs fall before later phases can be sampled. **50 cm/s
stands** — reinstating 67 on a marginal statistic would be M29's error wearing a
t-value.

## Later milestones (candidate M31+, not committed)

- **The whole-body controller** — now blocking two things: the 62.7 mm envelope
  target, and enough low-friction survival to give M30's measurement statistical power.


- **ADR-0019/0020's friction cost** — now the single blocking measurement, gating the
  trot speed. Reachable with the M21 harness.
- **The whole-body controller** — 62.7 mm target against 28.9 mm achieved.


- **The whole-body controller** — now with a proven 62.7 mm target to close against
  the 28.9 mm achieved.
- **ADR-0019/0020's friction costs**, still unmeasured.


⚠️ **Prerequisite for everything else here: a whole-body controller** (QP or
step-MPC) allocating across placement, CoP and spine simultaneously. Until then both
"the reduced-order model is optimistic" and "the robot needs abduction" are
unsupported, and **the modelling has reached the end of what this control structure
can settle.**

- **ADR-0019/0020's friction costs**, still unmeasured.


⚠️ **The next honest step is NOT a better controller.** It is deciding whether the
robot needs an actuator with **along-line authority** — which is what ADR-0017's
rejected **leg abduction** would have supplied (+4 motors, 528 g). That rejection was
taken on the basis that NFR15 was already met.

- **ADR-0019/0020's friction costs**, still unmeasured.


- **Justify or withdraw `plant.spine = 36.6 mm`** — it now sits unsupported.
- **ADR-0019/0020's friction costs**, still unmeasured.


- **Planned/feedforward spine deployment** — reactive proportional control is
  structurally wrong for an actuator sitting in its own feedback path.
- **A steady measurement of the spine's realisable offset**, which the drifting
  `perp` signal defeated.
- **ADR-0019/0020's friction costs**, still unmeasured.


- **Whole-body QP / step-MPC** — now sharply aimed: can better control extract more
  than 3.6 mm from a 36.6 mm spine credit?
- **ADR-0019/0020's friction costs**, still unmeasured but reachable.

- **The spine in the balance loop** — the direct NFR15 question, and the one thing
  that could close the direction dependence.
- **ADR-0019/0020's friction costs**, now reachable: the harness holds the robot up
  long enough to read contact forces during a recovery.


- **A closed-loop balance controller in MuJoCo** — unblocks the envelope magnitude
  AND the ADR-0019/0020 friction costs. The single highest-value modelling item left.





- **Cell selection** — the runtime now rests on two assumed battery numbers.
- **Regeneration** — currently credited at zero; a backdrivable drive could recover
  some of the 27 W of mechanical work.

















- **Full rigid-body dynamics** — M6 models the CoM and the contacts, but not
  per-link inertia tensors or angular momentum (`dH/dt = 0`). Needed before any
  fast gait, and needed to capture the lit-review Mass-Mass-Spring result (leg
  mass in flight bending a compliant trunk; note its ~0.454 kg knee is from a far
  larger robot and must be rescaled).
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
