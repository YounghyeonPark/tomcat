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
> four failed measurement designs ([ADR-0035](DESIGN_DECISIONS.md)).
> **M31 done:** my envelopes were **horizon-limited**; the 2-D optimal law helps some
> directions and hurts the worst ([ADR-0036](DESIGN_DECISIONS.md)).
> **M32 done:** four DOFs, four failures — the controller is at **86 % of optimal**
> and the gap is the **spine**, not the feet ([ADR-0037](DESIGN_DECISIONS.md)).
> **M33 done:** torque control makes contact force a **decision**, and names why a
> diagonal stance cannot be held ([ADR-0038](DESIGN_DECISIONS.md)).
> **M34 done:** step timing is the **fifth** DOF to fail — and the first where the
> **harness** is what fails ([ADR-0039](DESIGN_DECISIONS.md)).
> **M35 done:** the harness measures **survival, not recovery** — and the bound it
> was checked against measures recovery ([ADR-0040](DESIGN_DECISIONS.md)).
> **M36 done:** drawn as real parts the leg **does not close** — and the spec has
> contradicted itself since ADR-0010 ([ADR-0041](DESIGN_DECISIONS.md)).
> **M37 done:** the tendon drive **routed** — and the tendon map is **coupled**
> where the model says it is diagonal ([ADR-0042](DESIGN_DECISIONS.md)).
> **M38 done:** the mass spiral closes at **4.30 kg** — NFR5 breaks, nothing else
> does, and the tendon drive gives back **62 %** of its own inertia argument
> ([ADR-0043](DESIGN_DECISIONS.md)).
> **M39 done:** the motor holds **on spec** — NFR6's runtime does not, and the
> vendor's own numbers disagree by **27 %** ([ADR-0044](DESIGN_DECISIONS.md)).
> **M40 done:** the copper-loss formula was **1.5× low** — and correcting it
> overturns ADR-0023's headline ([ADR-0045](DESIGN_DECISIONS.md)).
> **M41 done:** the fold-in — **4.30 kg is now the model**, and it cost five
> findings ([ADR-0046](DESIGN_DECISIONS.md)).
> **M42 done:** built as a **tendon drive** in simulation — the cable is 5× too
> stiff and **G3 finally has a number** ([ADR-0047](DESIGN_DECISIONS.md)).
> **M43 done:** the whole body **leans rather than collapses** — and a sign in the
> via-routing had been inflating every M42 number
> ([ADR-0048](DESIGN_DECISIONS.md)).
> **M44 done:** it **STANDS** on foot-force allocation — and a lone tendon's
> moment arm **reverses inside its own ROM** ([ADR-0049](DESIGN_DECISIONS.md)).
> **M45 done:** the ankle takes an **antagonistic pair** — ADR-0002 settled on
> **kinematic reach**, the one cost nobody had priced
> ([ADR-0050](DESIGN_DECISIONS.md)).
> **M46 done:** a **spool behind every cable** — the drivetrain is exact, and
> adding it exposed a controller this project does not have
> ([ADR-0051](DESIGN_DECISIONS.md)).
> **M47 done:** the drivetrain **cascade**, derived not tuned — and a firmware
> gain sets how much of a mechanical spring you get
> ([ADR-0052](DESIGN_DECISIONS.md)).
> **M48 done:** the anchor migration is measured and **deferred** — because the
> **fore leg's routing was never mirrored** ([ADR-0053](DESIGN_DECISIONS.md)).
> **M49 done:** the routing audit — **only the ANKLES were ever validated across
> the gait**, and the hind hip has no fix at any anchor
> ([ADR-0054](DESIGN_DECISIONS.md)).
> **M50 done:** a resting wrap has a **working window**, a clamped capstan does
> not — and that explains every routing finding since M42
> ([ADR-0055](DESIGN_DECISIONS.md)).
> **M51 done:** housekeeping — the suite is 3× faster, and **the simulation
> never implemented ADR-0008's variable-radius pulley**
> ([ADR-0056](DESIGN_DECISIONS.md)).
> **M52 done:** the **clamped transmission** is built and exact — and ADR-0002
> and ADR-0008 turn out to want **different robots**
> ([ADR-0057](DESIGN_DECISIONS.md)).
> **M53 done:** **ADR-0008 wins** — one motor per pair, and it cures the standing
> tension saturation as a side effect ([ADR-0058](DESIGN_DECISIONS.md)).
> **M54 done:** the shipped transmission is the **default**, the re-derivation is
> done — and **eight tests had quietly stopped being able to fail**
> ([ADR-0059](DESIGN_DECISIONS.md)).
> **M55 done:** the cascade transfers **unchanged** — what the pulley cost was
> **headroom**, and M47's ankle test had been measuring saturation
> ([ADR-0060](DESIGN_DECISIONS.md)).
> **M56 done:** the variable-radius profile is **priced and declined** — the one
> joint that needs help has **no shape to exploit**
> ([ADR-0061](DESIGN_DECISIONS.md)).
> **M57 done:** the **articulated spine** ships — 18 of NFR2c's 19 DOF, and the
> body stands, but only with the stance reaction a chain carries
> ([ADR-0062](DESIGN_DECISIONS.md)).
> **M58 done:** the sway ADR-0009 bought **does not come out** on the actuated
> plant — and ADR-0009 and ADR-0017 want different robots
> ([ADR-0063](DESIGN_DECISIONS.md)).
> **M59 done:** the **foot was not a foot** — a millimetre of contact geometry
> moved five published results ([ADR-0064](DESIGN_DECISIONS.md)).
> **M60 done:** the lateral arm buys **cost, not sway** — and M59 named the
> wrong constraint ([ADR-0065](DESIGN_DECISIONS.md)).
> 487 passed + 5 xfailed Python, 17 Rust.

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

## Milestone M31 — The optimal law, and a correction to how I measured (DONE)

M28's derivation hands over the optimal LIPM policy for free: the deadbeat target is
`g/(g−1)·ξ`, so when unreachable the best choice is its **projection onto the
reachable set**. Every controller from M8 to M30 projected onto a single **axis**
instead. Implementing the real thing found something more useful first.

⚠️ **The measured envelope is HORIZON-LIMITED.** The viable set asks *can it
recover*; a simulation asks *does it survive N steps*:

| survival horizon | measured envelope |
|---|---|
| 4, 6, 8 steps | 39.2 mm |
| 12 | 34.7 mm |
| **16, 24** | **28.6 mm** (converged) |

**M21–M30 all used 8 steps.** `control.py`'s docstring records making this exact
mistake with `steps=12` — *"horizon-limited, not reach-limited, which is a different
and misleading statement"* — and I repeated it in simulation.

Converged, the shipped controller's worst direction is **25.6 mm = 86 %** of the
viable bound, against the **97 %** M28 claimed from an 8-step run. Still near-optimal;
the number moves. Every converged figure sits **below** the bound, as it must.

**The 2-D law: better in some directions, worse where it counts.**

| controller (16 steps) | 120° | 300° | worst, vs viable |
|---|---|---|---|
| axis (M8–M30) | 28.6 mm | 25.6 mm | **86 %** |
| projected 2-D | 22.6 mm | **36.2 mm** | 76 % |

**Not adopted** — a requirement is judged on the worst case. And the reason is
specific: the projection assumes **both** freedoms of the support parallelogram
(placement `dx` *and* the load position `λ` along the line), but **only `dx` is
actuated**. Solving 2 DOF and realising 1 mis-allocates.

## Milestone M32 — The last degree of freedom, and where this arc ends (DONE)

M31 named the load split `λ` as *the* missing degree of freedom. Realising it —
planned once per stance, open-loop, the structure that fixed the spine — **makes the
controller much worse**: the 300° worst case falls from **25.6 mm to 0.8 mm**.

⚠️ And it took two horizon-limited readings to see. At 4 mm of differential the worst
direction collapsed to 6.0 mm; at 1 mm it read **33.2 mm and looked like a win**,
until the horizon converged and it fell to 24.1 (120°) and 0.8 (300°). **M31's lesson,
applied to M31's own successor.**

**The pattern, stated plainly:**

| DOF | static authority | effect on the loop |
|---|---|---|
| Spine, reactive | 36.6 mm | baseline 5× worse; falls at gain 0.5 |
| Spine, planned | 36.6 mm | stable, **+0.23 mm** |
| CoP, reactive | ±109 mm | **+1.8 mm**, baseline 4× worse |
| Load split `λ`, planned | ±109 mm | **25.6 → 0.8 mm** |
| **Foot placement** | 30.3 mm | **works: 86 % of the bound** |

Four independent attempts, four mechanisms, one common factor: **only the actuator
the controller was designed around delivers.**

**And the useful reframing: 86 % is a good controller.** The feet reach 25.6 mm
against a 29.8 mm feet-only bound — the remaining 4.2 mm is not where NFR15's gap
lives. **The gap is entirely the spine credit** (62.7 mm viable *with* spine vs
25.6 achieved), and four attempts say a hand-designed per-step controller cannot
reach it.

**Decision: stop adding degrees of freedom to this controller.** Next is either a
genuine simultaneous optimisation (whole-body MPC with contact and friction
constraints) or accepting the feet-only capability and revisiting NFR15.

⚠️ **The modelling arc M17–M32 is complete for this architecture.** What it
established: the reduced-order model is **sound**, the feet-only controller is
**near-optimal**, NFR15 is **achievable but needs the spine**, and the spine is
**not reachable by this class of controller**.

## Milestone M33 — Whole-body force allocation: the first half, built and gated (DONE)

M32 ended four attempts to give a per-step **position** controller extra degrees of
freedom. The common cause is structural: **position servos do not command force.**
You command where the foot goes and the reaction is whatever the contact gives you.
Every load-allocation scheme in M21–M32 commanded a *proxy* and hoped.

`wbc.py` makes the force a decision variable — DCM law asks for a CoP, `allocate`
finds foot forces inside the friction cones, `stance_torque` maps back with
`τ = −Jᵀf`. Six variables, regularised least-squares plus a closed-form cone
projection; no solver call, because it runs every timestep.

**Gate passed on the static case:** forces sum to **39.6795 N** against a 39.681 N
weight, net moment under 0.01 N·m, CoP lands at (0.1030, 0.0002) against a CoM at
(0.1031, 0). Torque control then holds the stance with **sub-millimetre CoP error**
for about a second.

⚠️ **Two things had to be added.** Height must be regulated — commanding exactly
`mg` balances the weight and regulates *nothing*, and the first run drifted
0.165 → 0.185 m in 0.6 s undisturbed. And `p = c` is a **neutral** command, not a
balance law: with the CoP under the CoM, `ξ̇ = ċ` and the DCM simply runs.

⚠️ **The finding: a diagonal stance is not holdable, and now that is measurable.**
Two point contacts confine the CoP to the **segment between them** — a trot has no
support polygon, only a support **line**. Commanding a free 2-D point asks for
something no allocation can deliver:

| t | CoP demanded off the segment |
|---|---|
| 0.25 s | 0.5 mm |
| 1.00 s | **104.6 mm** |
| 1.25 s | **591 mm** |

**That residual is a "step now" measure** — the quantity M20 and M30 were missing
when they tried to hold a stance open-loop. The robot does not fall because the
allocation is poor; it falls because it is asked for a centre of pressure that does
not exist.

**Next is integration, not more allocation:** drive the existing gait from that
residual so a step is taken when the demand leaves the segment.

⚠️ Nothing here moves the bound. The honest test remains whether it beats the
**25.6 mm** the shipped position controller already achieves.

## Milestone M34 — Step timing, and the harness that cannot measure it (DONE)

ADR-0038 asked for one thing: drive a step from the CoP infeasibility residual. Built
— `cop_residual` reads it from live contact geometry, `plan_stance_time` inverts it in
closed form (`T* = ln(tol/((1+k)|e0|))/omega`), `swing_time_floor` keeps the answer
inside what the leg can swing, and `run(until=...)` measures on **seconds** because
sixteen re-timed stances are not sixteen nominal ones.

**It reads like the first success in five attempts, and it is not one.**

| controller, equal 3.2 s | worst direction |
|---|---|
| axis, fixed timing (shipped) | **25.6 mm** |
| + residual timing | 31.7 mm *(+24 %)* |
| **fixed 0.140 s stance, no trigger at all** | **37.7 mm** |

⚠️ **The trigger is not what produces the gain.** It saturates in the first stance
(`T_mean` 0.117 s against a 0.100 floor), the tolerance barely matters across a 4×
sweep, and doing nothing clever beats it by 6 mm. What actually moves is per-step
growth, `e^(omega T)`: **4.73 → 2.48**. Every controller gets that for free.

⚠️ **And the re-timed numbers are not trustworthy.** Worst direction across stance
reads **25.6 → 37.7 → 19.6 → 37.7 mm** — not monotone in a parameter whose mechanism
is. The undisturbed drift explains it: **4.99 mm at 0.200 s, 9.3 mm below 0.117 s.**
M21's own gate says a harness whose baseline is the order of the signal cannot
adjudicate, and this fails it.

⚠️ **The cost settles it anyway.** `spine_friction_cost` scales as 1/stance², so the
demand goes **μ 0.71 → 1.44 → 2.07**. μ 2.07 is not a floor. Foot speed — the
constraint one would expect — never binds: 1.20 m/s against 4.10 spare.

**Five DOFs, five failures — but the fifth failed differently.** The first four were
measured cleanly and were worse. This one **cannot be measured** by the harness as
built, which makes "the architecture is the limit" (ADR-0032) unsafe to keep
repeating. **The next honest step is the harness, not the controller.**

## Milestone M35 — The instrument, and what it was actually measuring (DONE)

M34 set this up as a maintenance job: flatten the noise floor, then re-evaluate
re-timed gaits. The flattening worked. What it exposed does not fit in a maintenance
job.

**The floor is loop gain, not plant.** Disabling the placement correction says the
plant is fine; sweeping the gain says the loop is not. A deadbeat law has no phase
margin to spare — it asks for the whole correction in one step — and the lag it does
not model (7.5 ms pipeline + the kp 80 servo) is *fixed* while the stance is not. A
constant `placement_gain` of **0.5** flattens 0.100–0.200 s and improves the shipped
stance too, **4.99 → 3.53 mm**. Goal met.

⚠️ **And then it certified an impossibility.** Detuned at 0.117 s the harness reports
**42.2 mm against a 39.5 mm exact viable bound** — 6.8 % over, far outside the
~1.5 mm bisection resolution. No controller beats the viable set, so the measurement
is wrong.

⚠️ **The cause is the success criterion, and it has been there since M21.** `run`
passes a trial when the CoM never drops below 0.11 m: *did not fall*. `viable.py`
computes what the robot can *recover* from. **This project has been comparing those
two numbers to each other for fourteen milestones**, and it held only because at the
shipped configuration they happen not to cross.

| | survival | **recovery** | bound |
|---|---|---|---|
| **shipped** | **25.6 mm** | **1.5 mm** | 29.8 |
| 0.117 s, detuned | 42.2 mm | 42.2 mm | 39.5 |

The shipped controller ends its certified 25.6 mm trial **26.2 mm off its support**
against a 3.9 mm floor. It did not recover; it did not fall. The mechanism is
steady-state error — the placement law arrests a topple but has no term that removes
a persistent offset, so it settles into a biased limit cycle. That is the same
*"stable, and walking away sideways"* failure ADR-0013 documented for at-DCM
placement, present in the shipped law too and small enough to have gone unnoticed.

⚠️ **What this does to the headline.** ADR-0037's *"86 % of optimal"* and ADR-0033's
*"97 %"* compare a **survival** measurement to a **recovery** bound. Like-for-like,
the controller is nowhere near the bound, and NFR15's "not demonstrated" is more
emphatic than recorded.

⚠️ **And one contradiction is left standing on purpose** — see M36.

## Milestone M36 — The leg, drawn as parts (DONE)

Thirty-five milestones had not produced a component anyone could make.
`cad/tomcat_leg_detail.py` is one hind leg as **manufacturable geometry** — bonded
inserts with a modelled glue line, clevis/tongue joints with H7 bores and h6 shafts,
turned sheaves whose groove pitch line *is* the tendon moment arm, the §0.1 root
idler, the return spring, the tactile pad — exported to STEP/STL. It also **checks
its own dimensions**, and that is where the value turned out to be.

⚠️ **LEG_TENDON_SPEC §1.1 has been stale since ADR-0010**, and it is the table that
sizes the bones:

| | §1.1 as written | live budget at 4.045 kg |
|---|---|---|
| hip land torque | 12.36 N·m | **16.67 N·m** |
| hip cable tension | ~447 N | **600 N** |

The ratio is exactly the mass increase **4.045/3.0 = 1.35**. §2 *was* re-run and says
"~600 N", so the document has disagreed with itself for ten milestones — and §1.3,
§1.3a, §3.5 and ASSEMBLY_SPEC §0.1 all derive from §1.1.

⚠️ **So the link sizing is not what it claims.** §3.5's Ø12/Ø10/Ø8 gives
**SF 1.97 / 2.08 / 1.84**, not 2.84/3.10/2.87 — the femur and metatarsus below the
SF 2 floor §0.1 relied on. **The remedy is nearly free:** one step up in stock each
(Ø12→Ø14, Ø10→Ø12, Ø8→Ø10) restores **2.78/3.16/3.11** for **under 4 g**, because
strength goes as the cube of diameter and mass only as the first power.

⚠️ **The moment-arm trade closes shut.** The three sheaves are **41 g of a 110 g
leg** — §1.3a priced only the ankle's *inertia*, never the set's mass. And they
cannot shrink: at the shipped arms the **trot** case is already **81 %** of motor
peak (agreeing with §2's independent "0.82×"), and 0.85× the arms exceeds **100 %**.
The arms are pinned by the actuator.

⚠️ **Therefore the leg does not close, and NFR5 is at risk.** Drawn as parts it is
**~160 g against a 110 g allowance (146 %)**; bearings, sheaves and clevises are
82 % of it and none is negotiable. **+50 g × 4 legs = +200 g on 4.045 kg → NFR5
breaks by ~5 %**, which re-triggers the ADR-0010 spiral.

**What passed:** the full **ROM sweep is clean** (+15.4 mm worst non-adjacent
clearance, so no hard stop is needed for self-interference), bond gaps land in
spec by construction, and the sheave radii *are* the moment arms — the CAD cannot
drift from the torque budget.

⚠️ Two of this pass's findings were **its own errors**, corrected rather than
shipped: bearings first seated inside the clevis gap instead of the arm bores, and
the trade table first compared the *land* transient to the motor peak (162 %) —
exactly the conflation ADR-0008 exists to prevent.

## Milestone M37 — The tendon drive, routed (DONE)

M36 drew the joint hardware and **did not draw a tendon.** No cable, no spool, no
anchor, no antagonistic pairing — P1 is the premise of this robot and it was the one
thing the manufacturing model omitted. What M36 produced was a linkage with pulleys
bolted to it.

`tendon_route.py` solves a tendon as a **belt problem** — signed-radius common
tangents and arcs, verified against closed forms — and `leg_tendons.py` routes this
leg's five runs from three girdle motors.

⚠️ **The tendon map is COUPLED, and `TendonMap.cable_lengths` is diagonal.** A
distal tendon must get past the proximal joints; a via-pulley concentric with the
proximal axis kills the *tangent* term but not the *arc* term, because the wrap
changes with the proximal angle. Measured `d(cable)/d(joint)`, mm/rad:

| | hip | knee | ankle |
|---|---|---|---|
| **hip tendon** | **28.00** | 0 | 0 |
| **knee tendon** | **8.75** | **25.00** | 0 |
| **ankle tendon** | **−8.75** | **−8.75** | **14.00** |

The diagonal is exact — the sheaves deliver their moment arms. The off-diagonals are
exactly the via-pulley radius: **35 %** of the knee's own arm, **62.5 % twice** for
the ankle. The model puts them at zero.

⚠️ **And it cannot be designed away.** 8.75 mm *is* the cable's minimum bend radius
(10 × Ø1.75, §2) — the same rule that forced the spool to 8.75. Coupling is a
property of routing a tendon past a joint at all.

**Three corrections follow.** `tau = −Jᵀ T` means knee and ankle tension produce
**hip** torque — 7.7 N·m against the hip's 16.67, a **46 % perturbation** the model
puts at zero. §1.4's spool travels (117/65/44 mm) become **117/102/104** — one class,
not 2.7× apart. And §3.4 *over*-estimates the capstan: the ankle is **1.21×**, not
the assumed 1.87×.

⚠️ M36 also shipped `idler()` at **r = 5.0 mm**, 43 % under the cable's own minimum
bend, at the one station that sees full tension every step. Fixed.

**What it makes concrete about P1:** the three leg motors are **395 g in the
girdle**; the tendon carrying their 600 N into the limb is **3.3 g of UHMWPE**. That
ratio is the whole argument for tendon drive, now measured rather than asserted.

## Milestone M38 — The mass spiral, closed (DONE)

M36 measured 167 g of leg against a 110 g allowance; M37 measured a tendon map that
is coupled where the model is diagonal. `total_mass = trunk_mass + Σ(leg masses)`,
so the first goes straight into body mass and body mass drives every force in the
project. ADR-0010 warned this spiral converges **only because the motor has
headroom**. Re-run with measured inputs:

| | measured | params | ratio |
|---|---|---|---|
| hind leg | **167.2 g** | 110.0 g | 1.52× |
| fore leg | **167.4 g** | 95.0 g | 1.76× |
| **BODY** | **4.304 kg** | 4.045 kg | **1.064×** |

⚠️ **NFR5's 4.05 kg is exceeded by 6.3 %.** ✅ **And every design gate still
passes** — trot at **80 %** of motor peak, cable **SF 4.70**, bearing C0
**1277/1500 N**. The overrun costs margin, not viability, which is exactly the
headroom ADR-0010 said the spiral depends on, being spent.

⚠️ **The finding: the joint hardware gives back 62 % of the P1 inertia saving.**
Swing inertia about the hip **+61.7 %**, because the hardware sits *along* the limb:

| mass share, proximal → distal | femur | tibia | meta | paw |
|---|---|---|---|---|
| params (assumed) | 47.3 | 30.0 | 15.5 | 7.3 |
| **measured** | **39.5** | **35.3** | **20.7** | 4.5 |

`link_mass` justifies "proximal-heavy" by saying the tendon drive pushes mass toward
the body. **It pushes the motors there. It does not push the pulleys there.** The
metatarsus more than doubles, and ADR-0003 accepted the whole cable-tension burden
to buy exactly this.

**It does not cascade, for a reason already in the record.** The balance envelope
moves only **52.7 → 51.9 mm (−1.6 %)** — the swing is *speed*-limited, not
acceleration-limited, which M12's `test_the_ramp_barely_moves_the_envelope`
established. NFR15 still clears.

**Two more.** The fore/hind leg asymmetry **disappears** (95/110 g assumed → 167/167
measured — the joint hardware dominates and is identical), which F2's weight split
rested on. And M37's coupling raises the **knee** tendon **+39.5 %** (435 → 607 N),
SF 4.94, still clearing 4.

✅ **And a free lever:** the off-diagonal signs come from the wrap senses, which
`route()` picks for minimum *wrap*. Picking them for minimum *load* moves the worst
tension **607 → 562 N**, cable **SF 4.94 → 5.34**. Eight per cent of margin for a
routing choice that costs nothing.

## Milestone M39 — The motor, reviewed on spec (DONE)

R1 ("buy one and weigh it") is still the cheapest high-leverage action in the
project and still open. This is what can be settled **without buying anything**:
every actuator number re-derived at ADR-0043's 4.304 kg, and the vendor's published
figures checked against each other. Working in
[motor-spec-review](notes/motor-spec-review.md), gated by `tests/test_motor_spec.py`.

**Verdict: the motor holds. The runtime requirement does not.**

⚠️ **The vendor publishes three inconsistent numbers.** Kt from the rated pair is
**0.444**, from the peak pair **0.465**, and quoted as **0.350**. The down-select
took 0.44 and dismissed 0.35 as "a different reference point" — defensible, since
the pairs agree with each other, but it is the **optimistic** branch and nothing had
swept it. Copper loss goes as `1/Kt²`, so the spread is worth **1.61×** of
dissipation, and both the runtime and the thermal duty ride on it.

✅ **Thermal duty passes on both branches** — RMS current **0.64×** and **0.81×** of
the 1.60 A continuous rating. That closes `motor-reality-check`'s sharpest open
item. ⚠️ It also corrects how I first read it: a **workspace peak is not a duty
cycle**, and comparing the 2.40×-rated worst pose against a continuous rating was
wrong.

⚠️ **NFR6 is what breaks:** ~30 min published → **25.2 min** at 4.304 kg with the
8.75 mm spool §2 requires (−17 %, and that part is a *correction*, not uncertainty)
→ **18.9 min** on the vendor's Kt (−37 %).

⚠️ **The robot is 58.1 % motor by mass** — 2.502 kg of 4.304. ADR-0008 quotes
45.6 %, which is 19 × 72 g of a 3.0 kg body: both halves superseded.

⚠️ **The down-select re-run loses a candidate.** At 1.71 N·m rather than 1.10, the
**GIM3505-8 is over its peak** where `motor-reality-check` listed it as meeting the
requirement. GIM3505-9 stays the pick at 88 % of peak (was 77 %). GIM4305-10 is the
escape hatch at 59 % for +158 g — but Ø53 against Ø34.5, so a girdle repackage
rather than a part swap.

**Speed is not a constraint anywhere:** 7.8–8.5 m/s foot ceiling against NFR14's
4.1 m/s of spare.

## Milestone M40 — The copper-loss formula, corrected (DONE)

M39 went looking for why the vendor's three motor numbers disagree by 27 %. Rotor-side
was ruled out; a six-step-vs-sinusoidal convention fits to 0.4 %. **The useful find
was next to it:** `power.py` computed copper loss as `I²R_pp`, and balanced
three-phase is `3I²R_ph`, which with a wye winding's `R_pp = 2R_ph` is **1.5× that**.

The module's own docstring had flagged the shorthand since M16 and justified it as
*"matching the convention in the motor down-select note so the two agree"*. **They
agreed on a figure 1.5× low.** Unlike the Kt question this needs no vendor and no
purchase — it is arithmetic.

| | was | now |
|---|---|---|
| copper loss | 42.0 W | **63.1 W** |
| trot draw | 83.6 W | **104.6 W** |
| drive efficiency | 38.7 % | **29.6 %** |
| trot runtime | 30.2 min | **24.1 min** |
| standing / moving | 0.76 | **0.87** |

✅ **ADR-0021's arguments get stronger.** Copper is now 2.4× the useful work rather
than 1.6×, sharpening its point that the inefficiency is the transmission and not
the gait; and standing at 87 % of moving strengthens the brake case.

⚠️ **ADR-0023's headline is overturned.** It concluded *"anodised, it is safe because
its own equilibrium is ~75 °C"*. That equilibrium is **96.1 °C**. Anodising is worth
**more** than before (59 K, not 39 — radiation goes as T⁴ and the operating point
rose) and is **no longer sufficient**. The battery-limited anodised trot lands at
**70.2 °C**, on the wrong side of a line it used to clear by 10 K. ✅ Forced air
recovers it (72.7 °C at h = 15), so it stops being optional. The winding gradient
scales too: **+11.5 K**, not +7.7.

⚠️ **A third stale copy of the same constant, outside the guard.**
`test_thermal_constants.py` exists because *"a copied number goes stale silently"*.
It guarded four constants; `TOTAL_W` was a bare `const` in **three** Rust functions
and therefore outside it. It went stale exactly as predicted, and only the
emergent-runtime cross-check caught it. Guarded now.

⚠️ **And the correction immediately double-counted itself** — `motor_spec_review`'s
hypothetical 1.5× applied on top of the now-corrected model gave 14.7 min where the
answer is 19.6. **M39's own defect-asserting tests caught it.**

## Milestone M41 — The fold-in (DONE)

M36–M40 established what the numbers should be and deliberately did not change them,
because *"every mass-derived published figure moves with it"*. This is that move: six
parameters, and then a week of consequences.

| | was | now |
|---|---|---|
| hind / fore `link_mass` | 0.110 / 0.095 kg | **0.1672 / 0.1674 kg** |
| leg + spine `motor_spool_radius` | 0.008 m | **0.00875 m** |
| `LoadCase.body_mass_kg` | 4.045 kg | **4.3041 kg** |
| trot `nominal_foot` x | 0.005 m | **0.00214 m** |
| trot draw / runtime | 83.6 W / 30.2 min | **134.2 W / 18.78 min** |
| drive efficiency | 38.7 % | **25.8 %** |

⚠️ **The trot foothold had to be re-tuned, and that is a real design change.** The
balance point is a property of where the CoM sits relative to the diagonal, so it
moved with the leg masses: at the old 0.005 the roll drift is **−0.180 rad/s per
cycle** — divergent, the robot falls inside a stride. Re-bisected the way M7 found
the original: **0.00214 m**.

⚠️ **The thermal conclusions escalated past what forced air at h = 15 can fix.**
Anodised continuous is **119.0 °C** (ADR-0045 had it at 96.1), battery-limited
**78.6 °C**, and **h = 15 now gives 90.1 °C** — only h = 25 brings it under 80. The
M18 asymmetry also **inverted**: both finishes now outlast the pack, and that is not
reassurance, because reaching 57 % of the settled rise still lands at 78.6 °C. **The
protection still exists and no longer protects.**

⚠️ **Two mechanism claims moved, both intact.** ADR-0025's sway correction is
**7.1 %, not 4.0 %** (its size is set by where the leg mass sits). And ADR-0019's
friction limit **no longer binds at μ 0.8** — the ROM-limited sway fell 42.2 → 37.0
mm, so friction binds *below* μ ~0.8 and ROM above. NFR16's 0.70 floor now sits just
inside the friction-limited region.

⚠️ **And FIVE earlier findings are SUSPENDED, not retuned.** The survival
measurement went degenerate — **37.17 mm at both 120° and 300°, above the 29.15 mm
exact bound** — which is ADR-0040's finding arriving. Four tests reading it are
`xfail(strict=True)`, plus ADR-0029's proportional-assist finding, whose *direction*
inverted (the 0.2 assist now slightly helps). **Fitting new thresholds to an
instrument this milestone just showed to be broken would be the M35 mistake.**

**What survived, which is worth saying:** every design gate still passes, the
reduced-order model's 2 % agreement with the exact viable set survived the mass
change, compliant legs still beat stiff ones, and the paw sensor's marginal cost
*fell* because the leg it loads is 52 % heavier.

## Milestone M42 — Built as a tendon drive, in simulation (DONE)

The plan is to build the robot in simulation before hardware. The first thing that
needed establishing is that **the simulation is not the robot**: `mjcf.py` puts a
`<position>` servo on every joint, so the plant under every balance result since M17
has been a **direct-drive** machine.

⚠️ **A position servo can push.** *"A cable can only pull"* is a premise of ADR-0002
(why antagonistic pairs exist), ADR-0021 (why standing costs 76–87 % of moving for
zero work) and ADR-0023 (why standing is the worst thermal case). **The simulation
never had that constraint.**

`mjcf_tendon.py` is the other thing: five spatial tendons per leg over cylinder
sheaves and concentric via-pulleys, `<motor tendon=...>` with `ctrlrange="0 T"` so a
commanded −500 N applies **+0.00 N**.

✅ **ADR-0042's coupling is EMERGENT and matches to three decimals.** It was derived
by hand as ±8.75 mm/rad — exactly the pulley radius — and ADR-0042 said the sim could
not show it. It shows it from the geometry alone, and MuJoCo even stores the Jacobian
sparsely with **1, 1, 2, 2, 3** nonzeros: the lower-triangular structure, visible in
the memory layout.

⚠️ **The headline: the cable is far stiffer than balance can tolerate.** ADR-0026
measured that balance needs compliant legs — `kp` 80–150 N·m/rad — and that
**kp ≥ 250 winds up and falls**. `k = EA/L` at the routed run lengths gives the hip
~~1269~~ **1304 N·m/rad** (**5.2×** the value that fell) and the knee
~~560~~ **638**. **ADR-0026's
requirement was on hardware that was never built.** `kp = 80` was standing in for a
compliance the machine does not have.

✅ **So G3 finally has a number.** *"Passive compliance / shock absorption at each
joint"* has been a goal since M1 with nothing attached. A series-elastic element
combining as `1/k = 1/k_cable + 1/k_series` sizes to **~175 kN/m**, putting hip
(~~128~~ **136**) and knee (~~91~~ **107**) inside ADR-0026's window — a real spring
to hand to mechanical, and M43 widened it to a **125–175 kN/m band**.

⚠️ **And the ankle fails the other way — a note on ADR-0002 Option B.** A cable
always pulls the same direction, so a joint driven by *one* tendon has **no
restoring stiffness from it at all**: ~~39.7~~ **53.9** N·m/rad measured, of which
the Option-B
return spring is 0.3. Option B buys a motor per leg; the joint's stiffness is a cost
it never counted.

⚠️ **A pair's stiffness is direction-dependent** — hip flexor run ~~0.121~~
**0.073** m against the extensor's ~~0.073~~ **0.121**, so `EA/L` makes the
**flexor** 1.7× stiffer (~~1.72×~~ **1.60×** at the hip, ~~2.21×~~ **1.77×** at
the knee). ⚠️ M43's routing repair swapped which member takes the short route.

⚠️ **Gravity feedforward cannot hold a pose; an outer position loop can** (0.00° at
the hip and knee). Which is why FR1 specifies *closed-loop* control — and a firmware
note: solving the non-negative allocation properly holds to 0.00°, **clipping** an
unconstrained solve leaves **1.22°**.

⚠️ **Three of this milestone's own errors, corrected rather than shipped:** two sign
errors that produced a tidy and false *"constant arms mean co-contraction adds no
stiffness"*; an ankle anchor in a **dead spot** where the cable already clears its
sheave (moment arm 2.6 mm instead of 14 — ⚠️ **retracted by M43**, see below);
and a single `stiffness` constant that NaN'd, when the value has to be per-tendon
and the spring needs a deadband to be a cable at all.

> ⚠️ **M43 corrected four numbers in this section.** This leg was built with hinge
> axis `(0, 1, 0)`, which folds it **upward**. Cable stiffness reads **1304/638**
> N·m/rad, not 1269/560; the pair asymmetry **1.60×/1.77×**, not 1.72/2.21; the
> lone-tendon ankle **53.9**, not 39.7; and the ankle dead spot is **withdrawn**.
> The two conclusions above — the emergent ×8.75 mm/rad coupling, and G3 at
> ~175 kN/m — both survived. [ADR-0048](DESIGN_DECISIONS.md).

## Milestone M43 — The whole body leans rather than collapses (DONE)

Four tendon-driven legs on a floating trunk over a floor: **18 DOF, 20 spatial
tendons, 20 pull-only actuators, 4.3081 kg** against `params`' 4.3041.

⚠️ **But first: the M42 leg was built on the wrong hinge axis.**
`LegModel.forward` requires `axis="0 -1 0"` (`mjcf.py` documents it);
`mjcf_tendon.py` used `(0, 1, 0)`, so **the whole leg pointed up** — feet at
z = +0.346 above a trunk at 0.176. Correcting it left the cable on the **wrong side
of every via-pulley**, and nothing complained: the knee flexor's moment arm read
**1.17 mm/rad instead of 25.10**, and the couplings 11.73/36.40/14.00/41.54 instead
of 8.75. A wrap that does not happen is not an error condition.

⚠️ **The test harness is what let it hide.** `_hold` had the tendon Jacobian
written down as a literal, copied from M42. After the axis fix the hip pair's signs
swapped, the frozen matrix commanded the wrong antagonist, and the leg collapsed
**102° while the routing was fine**. Everything measures its own map now.

✅ **What the repair could not move: the coupling column.** **-8.750 before and
after**, while the diagonal signs flipped, the knee flexor and extensor swapped
rows, and every cable length changed. That is a stronger confirmation of ADR-0042
than M42's agreement was, because it is an **invariance** rather than a coincidence.

✅ **G3 survived and gained a band.** 175 kN/m now gives 136/107 N·m/rad, still
inside ADR-0026's 80-150 window, and **125-175 kN/m keeps both joints inside it**.

⚠️ **`TENSION_MAX` is the MOTOR's limit, not the cable's — and getting that wrong
launched the robot.** The first pass took 700 N from ADR-0046's 638 N land
transient, which is a **structural** number: what the hardware must survive when
the *ground* hits the foot. Twenty tendons × 700 N is 14 kN on a 42 N robot, and a
0.1 s contact transient **threw it off the floor** (z 0.176 → 0.834). The real
ceiling is `tau_motor / r_spool`: **223 N peak, 81 N continuous**.

✅ **Welded to the world, every leg holds** — hind to **0.37°**, fore to
**1.8-2.3°**. So neither the routing nor the pull-only allocation is what fails.

⚠️ **Floating, it settles into a diagonal lean rather than collapsing:** **85 % of
target height, tilted 14.5°, on two feet of four** — while every leg holds its
commanded angles, the hind pair to 0.42°. One gain setting (kp 50, 5 N) flips it
right over. **Which is exactly the diagonal-stance problem M33 named:** a per-leg
joint controller has no term for trunk attitude. Commanding joint **angles** cannot
express *"put 10 N more through the left front foot"*; commanding foot **forces**
can, and `wbc.py` (ADR-0038) already does that allocation.

✅ **A new finding: co-contraction buys back the clipped allocator.** ADR-0047
found that clipping an unconstrained allocation at zero costs ~1° of joint error.
It does — at a 5 N floor. At ADR-0021's standing tension of **19.6 N the same
clipped allocator holds to 0.00°**, because the base tension keeps the solution
interior so nothing clips. **Clipping is only wrong when it is reached.** A second,
previously unpriced reason to pay for co-contraction.

## Milestone M44 — It stands, on foot-force allocation (DONE)

M43 left the gate open with a diagnosis: nothing in a per-leg loop has an opinion
about the trunk, so the controller has to command foot **forces**. `wbc.py`
(ADR-0038) does that allocation, was built in M33, and had never been driven against
anything but a position-servo plant.

✅ **It stands.** Trunk height **0.17600 → 0.17579 m over 3 s (0.21 mm)**, tilt
**0.006°**, four feet down throughout, allocation residual ~0.02 N·m.

That took one missing link and three fixes, and the fixes are the findings.

**The missing link — joint torque to non-negative tendon tension.** ADR-0038's
chain ends at `stance_torque`, which is where a direct-drive robot stops. `wbc.nnls`
is Lawson-Hanson, written out rather than imported because there is no scipy here and
**firmware will not have one either**. ⚠️ **Clipping escalated from ADR-0047's
"about a degree" to "loses the leg":** 197° of hip drift against 0.00°.

⚠️ **Fix 1 — `realisable_cop` had never been exercised on a support polygon.**
M33 only ran a diagonal two-foot trot, where the CoP really is a line. The 3+-contact
branch had **no inside test** (a feasible CoP in the middle of a four-foot polygon
pushed **48 mm out to the rail** — a *command to lean*) and **assumed the caller's
order was hull order** (`LF, RF, LR, RR` is a **bowtie**; two of its four "edges" were
diagonals, which masked the first bug at 16.6 mm). ⚠️ **And fixing it did not make
the robot stand** — 14.5° before, 14.5° after.

⚠️ **Fix 2 — the bookkeeping omitted `qfrc_passive`.** The actuator term is
`qfrc_bias - qfrc_passive + stance_torque`; the ADR-0002 return spring alone is
**0.508 N·m**, **54 %** of that joint's demand, asked of the tendon twice.

⚠️ **Fix 3 — a LONE TENDON's moment arm REVERSES SIGN inside its own ROM.** This
is the structural finding, and it is sharper than ADR-0047's *"no restoring
stiffness"*: **the one direction it can pull is not a fixed direction in joint
space.** Swept 12 anchor angles × the full -30–+150° range, **all 12 reverse
between 45° and 120°**, and they must — as the metatarsus sweeps 180° the
anchor sweeps 180° around the sheave, so the incoming line crosses the centre once.
The **hind hock holds +97.1°** in stance and M42/M43's anchor put the reversal at
~85°, so the hind ankle could not supply standing torque **at any tension**
(non-negative residual **0.714 N·m** — infeasibility, not a solver miss). Anchor
300° pushes the reversal past 105° and all four legs become feasible at
**0.000000**.

⚠️ **And so ADR-0002 Option B cannot serve both stance and swing.** Loaded, the
ankle needs plantarflexion; unloaded, the return spring plantarflexes too, so the
tendon must dorsiflex. **The spring and the stance load pull the same way:**

| anchor | unloaded ankle | quadruped |
|---|---|---|
| 45° (M42/M43) | **0.00°** | ⚠️ **inverts** |
| 300° (M44) | -14.6° | ✅ **stands** |

M44 ships 300° and references the spring at each leg's own stance angle (which also
drops the worst tendon from the 222.9 N ceiling to 207.4 N). **⚠️ Option A — an
antagonistic pair at the ankle — costs four more motors and removes the conflict.
It is now a live decision with numbers on both sides.**

✅ **G3 confirmed a second time, from force control rather than balance
compliance.** At 175 kN/m it stands (0.006°); with the bare cable, 5× stiffer, it
**inverts** (180°); with no elasticity it leans 14.6°. Two independent arguments,
two different failure modes, the same part. Band corrected to **150–200 kN/m**.

⚠️ **But not indefinitely: the hind hip extensor runs ~205 N mean to stand still**,
against a motor rated **81 N continuous**. That is **2.5×** the rating on one tendon
of twenty, and the hind knee flexor is at 1.6×. ADR-0023 made standing the worst
thermal case at the *nominal* 19.6 N; the thermal model has never seen this.

## Milestone M45 — The ankle takes an antagonistic pair (DONE)

ADR-0049 left ADR-0002's ankle question as a live decision with numbers on both
sides. M45 set out to settle it by measurement, and the answer came from an angle
nobody had checked.

⚠️ **First: "add an antagonist" is not sufficient.** The hip and knee build their
pairs by **mirroring** — same sheave, opposite sidesite, anchor reflected across z.
At the ankle that fails two different ways:

| antagonist anchor | spans both directions? | worst arm error |
|---|---|---|
| 120° (naive 180°-away mirror) | ⚠️ **no** — a 60° dead band with the stance hock inside it | |
| 60° (reflected across z, as hip and knee do) | yes | ⚠️ **13.83 mm** on a 14 mm arm |
| **300° — the SAME anchor as the primary** | ✅ **yes** | ✅ **1.43 mm** |

✅ **Option A must be a CAPSTAN**: both cables to one anchor, wrapping opposite
sides — physically one cable round a pin with a motor at each end. The two wraps
are then exact mirrors, so they **reverse together and stay opposite**, which turns
ADR-0049's moment-arm reversal from fatal into harmless. **A construction that works
at one joint is not a construction that works at every joint.**

✅ **Then most of the evidence favoured the CHEAP option instead.** `params` puts
the return spring at 0.3 N·m/rad; at **8 N·m/rad** it holds the unloaded ankle to
**-0.58°** at 13.3 N — Option A's performance **with no extra motors**. Its
apparent cost, travel, turned out not to bind: the trot only demands **8.6° (hind)
and 13.4° (fore)** in the tendon's own pull direction, leaving 2.9× margin. Call
it **Option B'**, and on every measurement to that point it was the better buy.

⚠️ **And then the gait was asked, and Option B cannot do it.** Over one trot cycle,
relative to the stance hock:

| leg | swing, below | swing, **ABOVE** | stance, above |
|---|---|---|---|
| fore | 13.4° | **+62.4°** | +54.8° |
| hind | 8.6° | **+25.2°** | +14.1° |

**Under Option B nothing drives the ankle above its reference.** The lone tendon
pulls it *down* — the sign ADR-0049 had to choose to make standing possible — and
the spring only pulls it *toward* the reference. The above-reference excursion in
**stance** is plausibly the ground dorsiflexing a loaded foot, which the tendon
resists; ⚠️ **in SWING the foot is unloaded and there is nothing left to do it.**
Nor is the spring reference free: moving it up to the swing extreme reachable gives
back ADR-0049's stance saving (222.9 → 207.4 N on the worst tendon).

✅ **Decision: Option A, capstan.** It costs four motors, **+528 g, a projected
4.83 kg (+12.3 %)** and 19 → 23 actuators — and it is the only option measured
that can command the gait this project already publishes.

⚠️ **Two limits, stated rather than buried.** The pair's **travel** cannot be
measured here — this plant has **no spool DOF**, so a slack antagonist acts as a
spring (2.13 mm of stretch, **273.8 N**, more than the 222.9 N driving it). And the
standing-tension comparison is **confounded** by a missing posture task: the worst
tendon climbs 207 → 223 N under *both* options because nothing controls the leg's
one internal DOF once the foot is pinned. Neither touches the argument above, which
is kinematic.

## Milestone M46 — A spool behind every cable (DONE)

ADR-0050 could not measure an antagonistic pair's **travel**, which is the thing
Option A is meant to buy: a tendon's length was a pure function of the joint angles,
so a motor could not **pay cable out**, and a slack antagonist acted as a spring
(273.8 N of resistance against the 222.9 N driving it, ankle stalled at 8.9°).

Every cable now has a **series-elastic drivetrain**, with each piece where it
physically is: *motor torque → rotor → spring → spool → cable*. The
actuator moves from the tendon to the **rotor joint**, so it is commanded in
**N·m**, and ADR-0047's G3 element becomes `k_tors = k_series * r_spool^2`
— specified in exact units, sitting between motor and cable rather than as a
deadband on the tendon.

✅ **The statics are exact.** Pin every leg joint and the spring carries the whole
motor torque: deflection matches `-tau/k_tors` to **five decimal places**, and the
tension lands on **222.9 N at the 1.95 N·m peak** — ADR-0048's ceiling, now
*arrived at* rather than asserted. ✅ **And pay-out works**: a slack antagonist
unwinds by exactly `r*theta` (**51.5 mm at −336°**), and the driven tendon takes
the ankle to its **−30° end stop, 127° of travel**.

⚠️ **Four traps, all quiet.** MuJoCo's spatial tendons are **memoryless about
winding** — a cylinder is rotationally symmetric, so wrapping and turning one
changes the path by nothing, and a site on the rotor merely orbits; the wound length
must be carried analytically. An **equality is itself a series spring** (0.717× the
specified stiffness loose, **0.998×** tight). A stiff equality then **overpowers a
default joint limit** and does not overshoot but **corrupts the answer** (215°
against a 150° limit, wrong direction). And a tendon equality is referenced at
**`qpos0`**, not at the state the caller sets — without the offset every cable
starts **24–52 mm** out, the solver snaps the leg 97° → 39° in 5 ms, and the
residuals then sit **constant**, which reads like a satisfied constraint until you
notice what they are constant at.

⚠️ **What it exposed, and why M46 stops here: adding the missing degree of freedom
exposed a missing CONTROLLER.** With the actuator on the rotor, commanding a tension
is no longer instantaneous — it arrives through a series-elastic mode at
**758 rad/s (~120 Hz)**. **Every controller this project has commands tension
directly** and none can drive this plant: the outer position loop that holds the old
plant to 0.00° leaves **5–10°** here. ⚠️ **And it is not numerical** — refining
the timestep **20×** changes it by under 2 %. A motor at zero torque on a
near-inertialess spool really does spin at hundreds of rad/s, so **open-loop torque
on one motor with the rest at zero is not an experiment a tendon robot can perform**.
That is the honest reason no travel figure beyond the end-stop case is published.

⚠️ A false lead, recorded so it is not re-checked: the ringing looked like the
**capstan** closing a kinematic loop. It is not — the **mirrored hip pair rings
harder**, 3862 rad/s against 294. Any antagonistic pair with an unactuated
antagonist has an undamped co-contraction mode.

## Milestone M47 — The drivetrain cascade (DONE)

M46 built a spool behind every cable and then could not drive it: commanding a
tension is no longer instantaneous but arrives through a **120 Hz** series-elastic
mode, and every controller in the project commands tension **directly**. M46 stopped
rather than keep guessing gains.

✅ **The cascade is three pieces and each has a closed form.** The inner loop is a
rotor **position** servo, because a real motor brings an encoder and a current loop:
`kp = I*wn^2`, `kv = 2*I*wn`, so **kp 180 N·m/rad** at 3000 rad/s. The command law
falls out of the winding constraint —
`theta_r_des = (theta_r + theta_s) + T*r/k_tors` — and the zero-tension rotor angle
reads **straight off the state**, so no reference offset is needed at all. Droop
compensation is `(kp + k_tors)/kp`, exact. Outside it, the joint PD and non-negative
allocation the project already had. **It holds the stance pose to 0.00°** where
M46's hand-tuned attempt left 5–10°.

⚠️ **A firmware gain sets how much of a mechanical spring you get.** The servo's own
stiffness sits in **series** with G3:

| rotor bandwidth | kp | raw error | delivered |
|---|---|---|---|
| 1000 rad/s | 20.0 | **−36.5 %** | **95 285 N/m** — ⚠️ outside ADR-0050's band |
| 3000 | 180.0 | −6.0 % | 141 004 |
| 6000 | 720.0 | −1.6 % | 147 645 |

The droop compensates to **−0.1 % at every gain**, but the *delivered* stiffness
cannot be compensated away. **G3 cannot be specified without the servo bandwidth
beside it** — a new coupling between the mechanical and firmware sides.

✅ **ADR-0050's open question is answered, qualified.** With the cascade the
antagonistic pair *does* reach the trot's ankle range — but only once ADR-0049's
**moment-arm reversal** is moved out of the way. On the shipped 300° anchor the
hind reversal sits at **~112°, inside** the 88.5–122.3° gait range and the
cascade cannot cross it (commanded 122.3°, reaches 85.6; with a frozen `G`, −30°).
At **270° the whole range tracks**, worst error **3.48°**.

⚠️ So ADR-0049/0050's criterion was **too weak**: reversal outside the *stance pose*
is necessary and not sufficient; it must be outside the whole *gait range*, per leg.
And no single angle serves both — their hocks stand **81° apart**, so **hind 270°,
fore 300°**. The fore leg being "inherited rather than mirrored", flagged since
M43, is finally forced.

⚠️ **M47 measures that migration and does not ship it**: 270° for the hind breaks
**14 tests across M44, M45 and M46**, because every ankle measurement in three
milestones was taken on 300°. Both numbers are recorded in `_ankle_anchor_deg` so
the migration starts from a measurement rather than a re-sweep.

## Milestone M48 — The fore leg's routing was never mirrored (DONE)

M48 set out to land ADR-0052's ankle-anchor migration (hind 270°, fore 300°).
Applying it and measuring the result **on both legs** — which no milestone had done,
because every Jacobian since M42 was taken on the **hind** leg — turned up something
larger.

⚠️ **The fore hip pair does not oppose.**

| tendon | hind hip | fore hip | spec |
|---|---|---|---|
| hip flexor | **+28.000** | **+11.636** | ±28 |
| hip extensor | **-28.000** | **+35.885** | ±28 |

Both fore arms are positive, so the two cables pull the joint the same way and there
is no antagonist at all. The other rows are as wrong: `knee_ext` puts **+44.067** on
the hip where the via-pulley says —8.75, and `ankle` **—39.909** on the knee. The
hind leg is exact.

⚠️ **And it has been true since M42.** The hip's arms cannot depend on the ankle
anchor, and the same numbers come out of the committed M47 tree. It is what M43
flagged and nothing acted on: `DEFAULT_FORELEG` folds the **opposite** way and the
fore leg's sidesites and anchors were **inherited rather than mirrored**. M43 read
the symptom as a tuning difference. It is a broken routing, and every fore-leg figure
in M43—M47 rests on it. ✅ ADR-0050's ankle decision does not — its argument came
from `gait.py`'s joint trajectories, which never touch the routing.

**What the migration measured before being deferred**, on the hind leg:

| | shipped 300° | migrated 270° |
|---|---|---|
| reversal vs the 88.5—122.3° gait range | ⚠️ inside (~112°) | ✅ outside |
| cascade tracks 122.3° | ⚠️ reaches 85.6 | ✅ **121.6** |
| quadruped, lone ankle | stands, 0.01° | ⚠️ **inverts, 179.75°** |
| quadruped, ankle **pair** | stands | ✅ **stands, 0.01°** |

✅ So the migration **forces** ADR-0050's Option A rather than preferring it: at the
migrated anchor a lone ankle tendon reaches the gait range and cannot stand at all.
✅ And the ankle finally clears ADR-0026's compliance floor — **86.3 N·m/rad**
with the pair against 55.1 with one tendon, the first time since M20.

⚠️ **Three control findings retract on the migrated plant**, all three consequences
of an under-actuated ankle rather than of a control law:

| finding | as published | on Option A, migrated |
|---|---|---|
| M42: gravity feedforward cannot hold a pose | diverges | **0.0001°** |
| M44: clipping loses the leg | 197° | **0.0002°** |
| M44: the bare cable inverts the robot, so G3 is what makes it stand | 180° | **stands, 0.07°** |

⚠️ The third matters most: M44 used it as an **independent confirmation of G3**, and
that confirmation does not survive. G3's balance-compliance argument (M42, from
ADR-0026's `kp` window) is untouched and remains the reason to specify it — but it
is **one argument again, not two**. Withdrawn now rather than later, because it is a
claim about a design target and does not wait on the migration.

⚠️ And a caution the other way: on Option B at the migrated anchor **clipping beats
NNLS** (0.0001° against 46.6), because with a lone tendon whose sign is wrong for
the load the honest minimum-residual solution drives the ankle away while clipping
happens not to. **A controller comparison on an infeasible plant measures the
plant.**

**Nothing ships.** The migration re-derives **17 tests across M42—M47** and the fore
leg those tests measure is broken. The order is: **mirror the fore leg, then migrate,
then re-derive once.** Both anchor angles stay in `_ankle_anchor_deg` and the
measurements stay in ADR-0053, so neither step starts from a re-sweep.

⚠️ **A process finding:** five milestones measured one leg and generalised. The
Jacobian check that found this is three lines and had never been run on the fore leg.
It is a test now.

## Milestone M49 — Only the ankles were ever validated (DONE)

M49 set out to mirror the fore leg (M48's finding) and began by applying ADR-0052's
criterion — the pair must stay **opposing**, with both arms near specification,
**everywhere the gait commands**, not merely at the stance pose — to all six
routings. Nobody had done that for the hip or the knee.

| leg | joint | gait range | same-sign points | worst arm error |
|---|---|---|---|---|
| hind | hip | —110.0..—35.1° | ⚠️ **6 of 13** | 8.38 mm |
| hind | knee | —114.7..—54.7° | none | ⚠️ **9.91 mm** on 25 |
| hind | ankle | 88.5..122.3° | none | ✅ **0.32 mm** |
| fore | hip | —168.8..—141.2° | ⚠️ **5 of 13** | ⚠️ **26.06 mm** on 28 |
| fore | knee | 33.4..103.8° | none | ⚠️ **23.81 mm** |
| fore | ankle | 3.0..78.8° | none | ✅ **0.28 mm** |

✅ **Only the ankles pass** — and the ankles are the only joints anyone ever swept
against a range criterion (M42, M44, M45). The hip and knee anchors came from M42's
2-D heuristic and were checked **at the stance pose only**.

⚠️ **So M48's diagnosis was too narrow.** The fore leg is worse, but the statement
is *"the hip and knee were never validated across the gait, on either leg"*. The fore
leg merely has the bad luck that its hip stance pose (—147.8°) sits **inside** its
own failure band while the hind's (—49.2°) sits outside. **That is why five
milestones of stance-pose checks saw nothing**, and it is the reusable lesson: a
check at one operating point is not a check.

✅ **Three of four have a measured fix**, using ADR-0050's capstan construction:
**hind knee 75°** (0.28 mm), **fore hip 135°** (0.00 mm), **fore knee 285°**
(0.27 mm).

⚠️ **The hind hip has none.** No capstan angle works; no mirrored angle works
(every one leaves 3—12 same-sign points of 13); no girdle-spool offset swept helps
(±40 mm in x, ±20 in z — the best left 1 of 13 at 12.26 mm). Its gait range is
**74.9° wide, 2.7× the fore hip's 27.6**, and the sheave construction's working
window is narrower than that. It needs a different **construction**, not a different
number: a larger sheave, a via-pulley at the girdle so the incoming direction turns
with the leg, or a narrower hip excursion from the gait.

**Nothing ships.** The re-derivation touches 17 tests across M42—M47 and must happen
**once**, after the geometry is settled — and the hind hip is not. All five measured
angles (three here, two ankle ones from M48) stay recorded so the milestone that
lands them starts from a measurement.

## Milestone M50 — A clamped capstan has no working window (DONE)

M49 found the hind hip has **no anchor angle that works** and called for a different
construction. This is it, and it explains everything since M42.

A cable that merely **rests** on a sheave gives the sheave's radius only while it
actually wraps. Outside that the arm is whatever the straight line gives:

| q1 | resting wrap | clamped capstan |
|---|---|---|
| —120° (past the gait range) | 12.956 mm | 28.000 |
| **—110°** (its far edge) | **21.034** | 28.000 |
| —90° | 31.979 | 28.000 |
| **—60°** | **36.332** | 28.000 |
| —10° and above | 28.000 | 28.000 |

Across the hind hip's —110..—35.1° gait range the resting arm swings **21.03 to
36.33 mm — 1.73× on a 28 mm specification**. ✅ **The clamped one is exactly
28.000 at every angle**, because the cable is *fixed to the sheave* rather than
resting on it. ✅ And it costs **21 % of one turn** of wrapped length.

✅ **What it explains:** M49's audit (the windows are narrower than the gait
ranges, and the ankles pass only because M42—M45 swept them *into* their windows);
M44's moment-arm reversal (a clamped cable cannot reverse); and the whole
anchor-sweep programme, which was a search for the corner of a window.

⚠️ **And it qualifies M42's headline.** *"The moment arm is emergent from the
geometry"* is true **where the cable wraps** — for the hip, `q1 >= —10°`, a
fraction of its range. As a validation that a wrap reproduces the sheave radius it
stands. As the model for a wide-ROM joint it does not, and five milestones of
stance-pose checks never noticed **because the stance pose sits inside the window**.

**The two alternatives, priced and rejected.** A **larger sheave**: only r = 50 mm
is clean, and it would also cut the standing tension 205 → 110 N — but that is a
±100 mm sheave on a **90 mm femur, 1.11× the segment it sits on**. A **narrower
hip excursion**: the window ends near —95° and the trot commands —110, so the gait
gives up ~15° of extension — **35 mm of a 100 mm stride, 35 %**. Clamping costs
neither; what it costs is a **cable fixed to each sheave**, which `mechanical/` does
not draw.

**Nothing ships.** M48's rule stands — the re-derivation touches 17 tests across
M42—M47 and happens **once**. What M50 adds is a construction to re-derive *to*.

## Milestone M51 — Housekeeping, and two of three items were blocked (DONE)

Three items had accumulated that were independent of the routing work: the suite had
grown to 12 minutes, ADR-0050's 4.83 kg was never folded in, and ADR-0049's 205 N
standing tendon had no thermal case. One closed; **two turned out to be blocked, and
the blockers are the findings.**

⚠️ **The simulation never implemented ADR-0008's variable-radius pulley.** ADR-0008
is *the mass-closure decision*: at the counts the architecture called for, the motors
alone exceeded the whole body (24 = 105 % of 3 kg) and **the design did not close**.
Its answer was **one motor per antagonistic pair** — 12 leg motors, and that is what
`params.py`'s `trunk_mass` carries. `mjcf_tendon.py` emits one motor per **tendon**:

| | leg motors | unbudgeted | body |
|---|---|---|---|
| ADR-0008 budget, as `params` carries it | **12** | — | 4.304 kg |
| the simulation, lone ankle | **20** | **1.054 kg** | **5.358 kg** |
| the simulation, ankle pair | **24** | **1.580 kg** | **5.885 kg** |

So **M42—M50 were built on the architecture ADR-0008 rejected**, on mass grounds
specifically, and 5.36—5.89 kg is outside the 4–5 kg band NFR5 has been anchored to
since ADR-0010. ⚠️ It also makes ADR-0050's costing wrong twice: *"+4 motors,
+528 g"* assumed the baseline was the simulation's 20 and that `params` carried them.
✅ The ankle *decision* is unaffected — it was made on kinematic reach — but the
number attached to it is not the cost. **That is why 4.83 kg was not folded in:** the
fold-in is a reconciliation of two architectures, not an addition.

⚠️ **The standing tension never converges — it saturates.** ADR-0049's *"205 N
mean, 2.5× continuous"* is not a steady state:

| t | peak tension | trunk z | tilt |
|---|---|---|---|
| 1 s | 125.5 N | 0.17589 | 0.005° |
| 4 s | 164.8 | 0.17586 | 0.005° |
| **9 s** | **222.9** — the motor ceiling | 0.17578 | 0.007° |
| 12 s | 222.9 | 0.17573 | 0.006° |

It ramps to the actuator's peak and pins there; 205 N was where the ramp had reached
in a 1–3 s window. ✅ The cause is the one M47 named — the tension tracks an
uncontrolled joint drift (2.9°/125 N at 1 s, 4.3°/217 at 7.5 s) — and the trunk
holds to 0.007° throughout, so it is a **wind-up, not a fall**. **The thermal item
is blocked behind the posture task.**

✅ **The suite is three times faster and nothing was loosened.** The dominant cost
was **re-measuring each leg's tendon Jacobian every 25 steps**, not the horizon. At
250 the standing gate is 2× faster for an answer that moves in the fifth decimal.
⚠️ Not a free constant: a test that *moves* the pose must keep 25, because M47
measured a frozen Jacobian driving a joint to the wrong end stop. So there are two
constants with the reason attached. `test_mjcf_tendon.py` **306 s → 95 s**, the
suite **12:21 → 4:41**.

## Milestone M52 — The clamped transmission, and a conflict between two ADRs (DONE)

✅ **The clamped capstan is built and it is exact.** A clamped cable's length is
exactly `sum r·q` over the joints it crosses — a `<fixed>` tendon, not a routed
path — so the moment arms are the specification at every angle, on both legs:

| | hip pair | knee pair | ankle pair | couplings |
|---|---|---|---|---|
| hind, stance | ±28.000 | ±25.000 | ±14.000 | —8.750 |
| hind, q1 = —110° | ±28.000 | ±25.000 | ±14.000 | —8.750 |
| fore, stance | ±28.000 | ±25.000 | ±14.000 | —8.750 |
| fore, q1 = —168° | ±28.000 | ±25.000 | ±14.000 | —8.750 |

Against the wrapped construction M49 audited — hind hip same-sign at 6 of 13 gait
samples, fore knee **23.81 mm out on a 25 mm specification**. ✅ **M49's audit would
pass outright**, so the routing programme that ran from M47 to M50 is finished.
✅ And the leg holds on **gravity feedforward alone**, 0.00° at `kp = 0`.

⚠️ **The arm is no longer emergent, and that is the price.** M42's headline was that
the sheave radius comes out of the routing on its own. Clamped, the map **is** the
analytic one `TendonMap` has carried since M4. M50 made that trade knowingly, but it
should be said plainly.

⚠️ **A silent failure closed on the way**: `single_leg_rig_elastic(clamped=True)`
used to return a plant with **stiffness 0.0 on every tendon** while the caller
believed it had asked for 175 kN/m. It raises now — with a clamped cable the G3
element belongs in the drivetrain (`spools=`), where M46 put it.

⚠️ **And ADR-0002 and ADR-0008 want different robots.** M51 called this *"the
simulation never implemented the variable-radius pulley"*; it is worse than an
omission:

- **ADR-0002**: antagonistic pairs *"for joints whose **stiffness must vary**, with
  co-contraction bias `T_bias` exposed as a **first-class control input**"*.
- **ADR-0008**: *"one motor per antagonistic pair via the variable-radius pulley"*,
  and *"**full articulation is retained** — a change of transmission, not of DOF"*.

That last sentence is true of the joint angles and **false of the stiffness**.
Co-contraction is the **redundant coordinate**: measured on the clamped paired leg,
`G` is 3×6 with a **three-dimensional null space**, and the same joint torque is
reachable at a 5 N floor and at a 40 N floor. One motor per pair leaves **none** — a
variable radius *schedules* co-contraction against angle, it does not command it.
ADR-0008 never mentions stiffness at all.

So the choice is **keep ADR-0002** (co-contraction commandable, 20 leg motors,
**5.36 kg**, or 5.89 with the ankle pair) or **keep ADR-0008** (budget closes at
**4.30 kg**, and the project gives up what M41 priced standing on, what ADR-0002's
AIC rule is built around, and what M43 measured as keeping the clipped allocator
exact). **M52 does not decide it** — it is a requirements-level trade with an
accepted ADR on each side.

## Milestone M53 — ADR-0008 wins, and the tension saturation goes with it (DONE)

M52 left two accepted decisions wanting different robots. **The choice is ADR-0008**:
one motor per antagonistic pair, co-contraction scheduled by pulley geometry rather
than commanded.

**Why.** ⚠️ Mass is this project's hardest constraint and it is already marginal:
independent pairs put the robot at **5.36 kg / 66.4 %** motor by mass, or **5.89 kg /
69.4 %** with the ankle pair — and that variant lands on **31 motors, the exact
count ADR-0008 was written to escape**. NFR6 is already 14–20 min and NFR18 already
says continuous trotting is out of spec in still air at any finish. ✅ And this
project has **never measured** a need for commandable co-contraction: ADR-0026's
compliance was met by **G3, a spring**; M43's "co-contraction buys back the clipped
allocator" was **retracted by M48**; M41's standing cost is a *cost* of
co-contraction, not a benefit of commanding it. ⚠️ What is given up is ADR-0002's
*"stiffness becomes commandable"*. Kengoro's AIC is itself a schedule, so the
peak-tension benefit may be recoverable in the radius profile — a design task, not
done here.

⚠️ **The prerequisite nobody had noticed.** A pulley transmits the **difference** of
its pair's cables; anything **common** to both is absorbed by stretch, which it
cannot relieve. M42 routed both cables of a pair over the **same side** of each
via-pulley:

| routing | differential (the motor) | common (stretch) |
|---|---|---|
| same side, as M42 built | `[0, r_knee]` | `[—v, 0]` |
| **opposite sides** | `[—v, r_knee]` | **`[0, 0]`** |

Same-side that is **11.44 mm** at the knee and **20.60 mm** at the ankle across the
hind gait range — **1716 N and 3090 N** against a **638 N** cable rating. The cables
break. It is also *falsely decoupling*: the differential loses the hip term entirely.
✅ Opposite sides leave the common mode at **exactly zero**. **ADR-0008 is only
buildable with the pair split across each via-pulley**, and nothing had said so.

✅ **Built: 12 leg motors, and the map is unchanged.** Three `<fixed>` tendons and
three **bidirectional** motors per leg — bidirectional because pull-only is a
property of a *cable* and a pair covers both directions. On both legs the map is
ADR-0042's exactly (hip **28.000**, knee **—8.750 / 25.000**, ankle **—8.750 /
—8.750 / 14.000**), and `params.trunk_mass` needs no change: the body stays
**4.3041 kg**. The unloaded leg holds to **0.00° at 1.7 N** of cable force.

✅ **And it cures M51's tension saturation, which was not expected.**

| | independent pairs | pulley |
|---|---|---|
| peak force at 1 s | 125.5 N | 68.2 N |
| at 9 s | **222.9 N** (motor ceiling) | **68.2 N** |
| at 12 s | 222.9 N | **68.2 N** |
| trunk sag | 0.2 mm | 3.1 mm |

**The wind-up was co-contraction growing with the uncontrolled joint drift** —
remove co-contraction as a state and there is nothing to wind up. 68.2 N is inside
the motor's **81 N continuous** rating, so the thermal item is unblocked by the
*transmission* rather than by the posture task. ⚠️ The cost line beside it: the
trunk sags **3.1 mm against 0.2**, the preload the co-contraction floor supplied.

## Milestone M54 — The re-derivation, and eight disarmed tests (DONE)

✅ **Asking for a leg now gets you the robot being built.** `single_leg_rig()`
with no keywords is three bidirectional motors; `quadruped_rig()` is **twelve**. For
eleven milestones the default was the plant ADR-0008 **rejected**. The legacy plant
is still reachable and still measured — 39 call sites moved behind seven named
wrappers, so **which machine a test measures is now part of its text**. The pinning
was behaviour-preserving: no measured number moved.

### ⚠️ The part that mattered

This project writes tests that **assert a defect**, so they fail when the defect is
fixed. **Eight were live.** Pinning them to the legacy plant — the obvious,
apparently conservative move — would have left all eight asserting defects on a
machine nobody builds, where they would have passed **forever**. Each was rewritten
to keep its legacy measurement *and* name where the shipped plant's guarantee lives.

### ✅ One measurement retires four routing defects

On the shipped leg the tendon Jacobian is a **constant** matrix. Sampled at five
poses spanning each joint's **entire ROM**, on **both** legs, the spread is
**exactly zero**:

| | hip | knee | ankle |
|---|---|---|---|
| q1 | 28.000 | —8.750 | —8.750 |
| q2 | 0 | 25.000 | —8.750 |
| q3 | 0 | 0 | 14.000 |

That closes: the **ankle sign reversal** (the arm is +14.000 everywhere), the **fore
leg was never mirrored** (the fore map is *identical*, not mirrored), the **anchor
migration** (a fix for a wrap that no longer exists), and **"only the ankles were
validated across the gait"** (every joint, whole range, which is more than the audit
asked).

### ⚠️ And a capability that did not exist: G3 had nowhere to live

ADR-0051 put the series-elastic element **in the drivetrain**. The drivetrain named
its spools per *cable* (`L_hip_flex`); the shipped plant's tendons are the three
*pairs* (`L_hip`). It did not build — it died on `unknown element 'L_hip_flex'`,
which names the symptom and not the cause. So M46's statics, M47's cascade and the
rotor servo all stood on a transmission ADR-0058 had already replaced, and **design
goal G3 had no home on the robot being built**.

✅ **Fixed: one spool per pair**, because with a variable-radius pulley the spool
*is* the pulley. Half the bodies (12 vs 18), half the constraints (3 vs 6), and
M46's two-pass `a0` still lands the winding equality to **4e-10 m**.

### ⚠️ The retraction: the pull-only allocator drops whole joints

`wbc.tendon_tension` enforces `T >= t_min` by NNLS. On the shipped plant that is
**not physical** — the pair's motor drives either way — and the cost is not a
rounding error:

| | needed | NNLS delivers | |
|---|---|---|---|
| hind ankle | 0.0114 N·m | **0** | the whole joint |
| fore knee | 0.0492 N·m | 0.0076 | **15 %** |

It clamps to zero a command the motor could deliver, and the shortfall appears only
in a residual nobody was checking. ✅ **`wbc.pair_command`** is the square, signed
solve — unique, **zero residual**. ⚠️ And `t_min` has nowhere left to act: the
floor was a coordinate of the *redundant* plant, and ADR-0058 spent it.

### ⚠️ What M54 did not do

- **The cascade and rotor servo were not moved.** They *can* be now, but the gains
  must be **re-derived, not re-pointed**: one spool per pair changes the reflected
  inertia and so the bandwidth separation. That is a measurement, and this project
  has learned to do a re-derivation once.
- **M52's six clamped tests still sit on the half-step** — clamped capstans with a
  motor per cable, which does not ship. Kept because the findings they carry are the
  evidence ADR-0058 was decided on.

## Milestone M55 — The cascade transfers unchanged, and two corrections (DONE)

✅ **M47's `kp = 50, kd = 1.0` holds the stance pose to 0.00° on the pulley
drivetrain**, at a peak commanded force of **3.2 N**, never asking past the motor.
The inner loop needed no thought at all: `kp = I·wn^2` is a property of the
**rotor**, and every motor still has one rotor.

⚠️ **Correction 1: M54 predicted the opposite.** ADR-0059 said the gains would have
to be *re-derived rather than re-pointed*. The inertia argument was right and the
mode really did move — but M47 had put its outer loop far below **both** plants'
modes, so it never got close enough to care. The re-derivation **confirms** the
gains rather than replacing them.

### ⚠️ What the pulley did cost: headroom, and nobody had measured it

| | lowest mode | usable outer `kp` |
|---|---|---|
| legacy, one spool per cable | 54.9 Hz | holds to **600** |
| **shipped, one spool per pair** | **27.4 Hz** | holds to **200** |

The lowest drivetrain mode **halves**, and M47's chosen gain goes from 12× below
the edge to **4×** below it. ✅ Still headroom, not a wall. ⚠️ But anything that
wants a stiffer joint loop — landing, disturbance rejection — has a third of the
room, and **ADR-0058 was decided on mass without this being measured**.

### ✅ The allocator was the real change

Same plant, same gains, same reference; only the allocator differs:

| allocator | hold error | peak force |
|---|---|---|
| `pair_command`, signed | **0.00 / 0.00 / 0.00°** | 3.2 N |
| `tendon_tension`, pull-only | **—70.8 / —47.2 / —127.1°** | 32.5 N |

M54 measured the pull-only allocator dropping whole joints and called it a residual.
Dynamically it is a **fall**. Same finding.

### ⚠️ Correction 2: M47's ankle test was measuring saturation

M47 reported *"the cascade tracks the hind ankle but the reversal still bounds it"*
from a **step** command. Logging the force *before* the clamp, that step demands
**3700–4300 N** against a 222.9 N motor and saturates **98–100 %** of every
timestep — on both plants. It was measuring where a saturated bang-bang controller
comes to rest. **Both of its numbers are withdrawn.**

✅ **Ramp the reference instead** — which is what a gait commands:

| commanded | final error | peak force | saturation |
|---|---|---|---|
| +20° | **0.00°** | 3.1 N | none |
| +40° | **0.00°** | 3.1 N | none |
| +52° | **0.00°** | 3.1 N | none |
| +55° | —2.06° | 130 N | none |

+55 misses by **exactly** its overshoot of the 150° end stop (the stance ankle is
at 97.06°, so headroom is **52.94°**). ✅ **The bound is the joint limit now,
not the moment arm.** And the lag is first-order: the worst error during the ramp
falls **0.81 → 0.41 → 0.20 → 0.10°** as the ramp is stretched
0.5 → 4 s. ⚠️ The legacy plant cannot do it at all — the same ramp to +20°
leaves it **36.3°** out.

✅ **M42–M47 is now entirely re-derived.** Nothing in the control story still
rests on a plant that is not being built.

## Milestone M56 — The variable-radius profile, priced and declined (DONE)

An item open since ADR-0002, closed by measurement rather than by design.

### ⚠️ Two mechanisms were being conflated

**Kengoro's AIC is a control rule** — hold the antagonist at `T_bias` while the
agonist works — and its 43 → 28 kgf is measured **against a fixed high
co-contraction**. The shipped pulley cannot co-contract at all, so it is already at
that optimum. **A variable radius is a transmission**: a gear ratio varying with
joint angle, trading speed for force where the demand peaks. That mechanism is real
— it is just not the one ADR-0002 cited, and it is the one M56 measured.

### ⚠️ The load split had to be settled first, and it names the other leg

A trot puts one fore and one hind foot down, each carrying its girdle's share.
Measured at the stance pose: **30.2 % fore, 69.8 % hind** — 12.75 N on the fore
foot, 29.47 N on the hind. ✅ `params.py`'s review finding **F2 had already said
so**: the original budget *tuned girdle masses to hit a 60/40 front-heavy split*,
and F2 retracted it, because the split is an **output** of where the hardware sits
and ADR-0005 puts more motors on the pelvis.

⚠️ Assume 50/50 and the survey names the **fore knee**; assume the discredited
60/40 fore-bias and it names the fore knee and ankle. Both point at the wrong leg.

### What the trot demands, quasi-statically

| | peak | RMS | RMS / 81.1 N | peak / 222.9 N |
|---|---|---|---|---|
| hind hip | 106.1 | 43.0 | 0.53 | 0.48 |
| hind knee | 32.6 | 15.5 | 0.19 | 0.15 |
| **hind ankle** | 136.3 | **96.4** | **1.19** | 0.61 |
| fore hip | 74.4 | 35.4 | 0.44 | 0.33 |
| fore knee | 89.0 | 49.9 | 0.62 | 0.40 |
| fore ankle | 67.1 | 47.5 | 0.59 | 0.30 |

✅ **Structurally there is no case at all** — the worst peak is **61 %** of the
motor's peak rating. ⚠️ **Thermally there is exactly one**: the hind ankle at
**1.19×** continuous, and RMS is what heats a motor.

### ⚠️ And the profile's yield on that pair is exactly zero

Through the whole stance phase the gait holds `q1 + q2 + q3` at **—55.0000°**,
span **8.5e—14** — the paw's *absolute* orientation, held fixed so the foot stays
flat while the body passes over it. So the ankle's demand is a **constant**:
136.338 N at every sample, std **4e—12 N**, peak/mean **1.0000**. **There is no
shape to remove.**

The pairs that *are* peaked have margin already — the hind hip is the most peaked
at 1.99 and runs at 0.53× its rating. And RMS is dominated by the **mean**, which a
profile does not change: flattening the most peaked pair moves its RMS by under 2 %.

✅ **Verdict: do not design it.** ⚠️ The hind ankle's 1.19× goes to
`mechanical/` instead, where the fix is a constant: **14.0 → 16.6 mm**. Left
unspent here because the arms are shared between legs and the hind leg's other pairs
have 5× margin, so the right change is a **per-leg** arm — a decision that needs
a sheave that fits, not a simulation.

## Milestone M57 — The articulated spine: 18 of 19 DOF, and it stands (DONE)

### ⚠️ What the census found

NFR2c has claimed **19 actuated DOF** since ADR-0009. Measured:

| model | actuated | leg | spine | tail |
|---|---|---|---|---|
| `mjcf`, rigid trunk | 12 | 12 | 0 | 0 |
| `mjcf`, `spine_dof=True` | 15 | 12 | **3, lateral only** | 0 |
| `mjcf_tendon`, the shipped plant | 12 | 12 | 0 | 0 |

⚠️ **The sagittal axis was in no MuJoCo model at all** — the axis ADR-0006 is
actually about, and the only spine axis that does work against gravity. What
existed was ADR-0009's lateral sway, in the rigid position-servo model.
**The requirement was being checked against a plant that could not meet it.**

### ✅ What was built

Three segments, each with a sagittal **and** a lateral joint: **six pairs, six
bidirectional motors**, ADR-0058's transmission applied unchanged. **18 actuated
DOF**, and the mass cost is **exactly zero** — 4.3081 kg either way, because the
girdles and segments sum to precisely the `trunk_mass` the box carried in one lump.

⚠️ It also settles two numbers nobody had reconciled: the rigid trunk put the
girdles **210 mm** apart, ADR-0006's segments sum to **195 mm**. The chain is the
sourced number, so the wheelbase shortens 15 mm and the fore load share moves
**30.2 % → 33.0 %**.

### ✅ It stands — and one term is the difference between standing and falling

| | sag | tilt | leg peak | spine peak |
|---|---|---|---|---|
| gravity compensation only | 89.7 mm | **77.0°** | 222.9 N | 222.9 N |
| **+ `wbc.chain_reaction`** | **2.82 mm** | **0.006°** | 65.8 N | **39.3 N** |
| rigid box, for reference | 3.08 mm | 0.006° | 68.2 N | — |

The missing term is what a foot force does to **every joint between that foot and
the root**. On a rigid box there were none; on a chain it is the whole spine.
✅ It is ADR-0044's omission one level up — the legs needed the stance term in
`actuator_torque` for the same reason. Derived, not tuned.

✅ **And the articulated body is slightly better than the box**: 2.82 mm of sag
against 3.08, 65.8 N against 68.2, while holding the spine to **0.001°**.

### ✅ The difficulty is the six DOF, not the geometry

Weld the chain's joints — same geometry, same distributed mass, zero spine DOF
— and it stands **better** than the box: **2.95 mm** sag, **65.4 N**. The shorter
wheelbase and the redistributed mass are free. ⚠️ What is not free is the six
degrees of freedom, and what they need is a **control term**, not a stiffer body
— exactly what M43 suspected when it made the trunk rigid so the gate had *"one
thing to get wrong"*.

## Milestone M58 — The sway does not come out, and a conflict to name (DONE)

M57 built ADR-0009's three lateral spine motors. **It never asked them to do the
job they were bought for.** ADR-0009's whole case — and M5's careful sway law,
raised-cosine inside the four-foot windows — is **analytic geometry and a
quasi-static margin**. This milestone runs it on the plant that has to produce it.

### ⚠️ It does not come out

Shipped walk (period 5 s, duty 0.90, ±11°/segment), at the only gain that keeps
the six spine motors inside their rating:

| | analytic | measured |
|---|---|---|
| CoM sway | **66.7 mm** p-p | **4.1 mm** |
| foot slip | — (the model has no ground) | **7.3–15.8 mm, all four feet** |
| spine force | — | **76.8 N** of 81.1 continuous |

⚠️ **The slip is larger than the sway.** The body gets about **6 %** of the
designed shift and pays by sliding every paw further than the CoM moves — while
already using **95 %** of the spine's continuous rating to do it.

### ⚠️ More gain skates, it does not sway

| `kp` | track err | CoM "sway" | spine force | foot slip |
|---|---|---|---|---|
| 8 | 12.95° | 4.1 mm | 76.8 N | 25.7 mm |
| 30 | 9.07° | 12.1 mm | **222.9 N, saturated** | 122 mm |
| 100 | 5.69° | 98.9 mm | saturated | **484 mm** |
| 300 | 3.80° | 22.8 mm | saturated | 280 mm |

The 98.9 mm at `kp = 100` is **not sway** — it is the robot sliding across the
floor, which is what the 484 mm of scrub beside it says.

### ✅ The mechanism was already written down

`mjcf.py` carries the warning: the legs are planar because **ADR-0017 rejected
abduction**, so *"a sway over planted feet needs foot slip or body roll"*. Written
of the rigid model; it holds on the actuated free-root body too. Held at
±11°/segment with the root pinned, the CoM moves **31.7 mm** while the fore feet
are carried **82.7 and 98.1 mm**.

⚠️ That fixed-root figure **overstates the fore share** — with a free root the
body counter-rotates and the hind feet move too (**2.1×** fore-to-hind, not 3×
with the hind at zero). What does not change is that **all four paws slide**.

### The conflict, not decided

- **ADR-0009** buys three lateral spine motors so the CoM can sway.
- **ADR-0017** rejects leg abduction, so no leg joint can move a foot sideways.

A requirements-level trade with an accepted ADR on each side — the same shape as
ADR-0057's. ⚠️ **M58 does not decide it**, and it suspends ADR-0009's *"+10.1 mm
polygon margin"* as an actuated result: that margin is computed from a sway the
plant does not produce.

## Milestone M59 — The foot was not a foot (DONE)

A design suggestion — *put something low-friction on the paw so it can follow the
sway* — needed a friction study. Setting the friction changed nothing, and chasing
why exposed the contact model.

### ⚠️ The defect

The distal limb carries three geoms within two millimetres: the **pad** at the toe
(the contact the model intends), the **paw capsule** 2 mm above, the fore
**metatarsal** 1 mm above. Standing sagged the trunk **3.06 mm** — more than
either clearance — so all three took load. Measured in a controlled stand: pad
200/200, hind paw capsule 196, **fore metatarsal 91**.

⚠️ **So the fore legs' effective contact sat 8.3 mm BEHIND the `_foot` site** every
controller and every support-polygon calculation uses. On a 210 mm wheelbase,
against the ±16–30 mm margins ADR-0009 argues over, that is half the quantity in
dispute — and it was silent.

### ✅ The fix

Bones do not collide. Not a convenience: **the model now matches its own stated
abstraction**, a point foot at `_foot`.

| | before | after |
|---|---|---|
| contact-vs-site offset, fore | 8.3 mm | **0.0 mm** |
| geoms touching | pad + capsule + metatarsal | **pad only** |
| standing sag | +3.06 mm | **—0.57 mm** |

### ⚠️ Five published results moved

- **ADR-0049's standing survey inverts.** The binding tendon is the **fore knee
  flexor at 100.5 N (1.24×)**; the **hind hip extensor** the headline named is at
  **63.4 N, inside its rating**. Its *"the fore legs are comfortable"* was the fore
  legs resting on their metatarsals.
- **ADR-0058's standing force**: not 68.2 N converged but **60.0 N decaying to
  33.6**, sag **0.18 mm** not 3.1. Conclusion stronger, numbers all wrong.
- **ADR-0063's gain sweep**: not skating but **divergence** — 65.7° of tracking
  error at `kp = 30`, raw demand **31 750 N**.

⚠️ Only **3 of 484** tests failed. That is because most of the suite is
quasi-static or kinematic, not because the defect was small.

### ✅ And the question that started it

| foot | CoM sway | max slip | spine raw |
|---|---|---|---|
| as built | 2.60 mm | 17.0 mm | 76.8 N |
| isotropic 0.10 | 9.17 mm | **115.9 mm** | 76.8 N |
| **anisotropic 0.03 / 1.0** | **8.26 mm** | **13.4 mm** | 136.5 N |

✅ **The anisotropic foot works as intended** — roughly **triples** the sway
while *reducing* slip. ⚠️ Isotropic does not: it reaches a similar figure only by
letting the robot skate **116 mm**. ⚠️ **But it does not close the gap**: 8.26 mm
is **12 %** of the designed 66.7, and it costs **1.68×** the spine's continuous
rating. **The binding constraint moves from friction to the spine's own torque
capacity.** Necessary, not sufficient.

## Milestone M60 — The arm buys cost, not sway (DONE)

M59 measured a directional pad tripling the sway and concluded the binding
constraint had *"moved to the spine's own torque capacity"* through the 20 mm
lateral moment arm. The obvious follow-up is to lengthen it. This does.

### ⚠️ The arm does not change the sway. At all.

| foot | lateral arm | CoM sway | max slip |
|---|---|---|---|
| as built | 20 mm | 2.60 mm | 17.0 mm |
| as built | 60 mm | **2.60 mm** | 17.0 mm |
| anisotropic 0.03 | 20 mm | 8.26 mm | 13.4 mm |
| anisotropic 0.03 | 60 mm | **8.26 mm** | 13.4 mm |

Identical to three significant figures across a **3×** change. ✅ The reason is
elementary once stated: the controller commands a **torque**, `kp·e`, and the arm
only sets what that torque costs in cable force, `f = tau/r`. Same gain, same
error, same torque, same motion. **The arm is a price list, not a capability.**

### ⚠️ So M59 named the wrong constraint

Torque capacity was never binding — nothing was clipped, and 76.8 N and 136.5 N
are both inside the 222.9 N peak. **What limits the sway is the control law**:
`kp = 8` asks for what it asks for, and every higher gain **diverges** (65.7° of
tracking error at `kp = 30`, a raw demand of 31 750 N). ⚠️ Neither the foot nor
the arm moves that wall.

### ✅ What the arm does buy is thermal

The **lateral** demand falls in exact proportion: **76.8 N at 20 mm, 25.6 N at
60** — precisely 20/60, from **95 %** of the continuous rating to **32 %**. Worth
having; simply not what ADR-0009 wanted.

### ⚠️ And the pad's bill goes where the arm cannot pay it

Peak pre-clamp demand per pair; `p` sagittal (30 mm, unchanged), `y` lateral:

| foot | arm | p1 | y1 | p2 | y2 | p3 | y3 |
|---|---|---|---|---|---|---|---|
| as built | 20 | 36.5 | **76.8** | 17.6 | **76.8** | 22.8 | **76.8** |
| as built | 60 | 36.5 | **25.6** | 17.6 | **25.6** | 22.8 | **25.6** |
| anisotropic | 20 | **130.6** | 76.8 | **136.5** | 76.8 | **127.8** | 76.8 |

⚠️ **With the pad the binding pairs are SAGITTAL**, at ~136 N and 1.68×
continuous, up from ~36. Letting the feet slide laterally lets the body move more,
and holding it up is the sagittal spine's job: **the pad buys lateral motion and
charges it to the sagittal pairs.** So "lengthen the lateral arm to pay for the
pad" reduces a demand that is no longer the binding one.

## Later milestones (candidate M61+, not committed)

> This list is **curated, not append-only**. When a milestone closes an item it is
> deleted here and the reasoning kept in the [ADR log](DESIGN_DECISIONS.md). Earlier
> revisions of this section had accumulated ten superseded copies of itself, which
> made it impossible to read off what was actually next.

### Next — fold it in, then re-publish

- **The TAIL, and it cannot be built from what exists.** ⚠️ ADR-0062: it has a
  motor in the mass budget (the 7-motor spine+tail bank) and **no `TailParams`, no
  joint, no body**. Modelling it means sourcing its geometry first — length,
  mass, ROM, moment arm — which is a `mechanical/` task, not a simulation one.
  Until then NFR2c is **18 + 1 owed**.
- **RE-CHECK [ADR-0061](DESIGN_DECISIONS.md)'s tension survey on the spine body.**
  ⚠️ It named the hind ankle using the **30.2 %** fore share; the chain gives
  **33.0 %**. The verdict may well survive — the profile's yield was *zero*, not
  marginal — but the survey itself was run on the rigid box.
- **The SWAY GAP IS A CONTROL PROBLEM.** ⚠️ ADR-0065: neither the foot nor the
  moment arm moves it. The sway sits at **12 %** of ADR-0009's design because
  `kp = 8` is the only stable gain and it asks for no more. Anything proposed has
  to beat that, and a different control structure is what it would take — none is
  proposed yet.
- **DECIDE ADR-0009 vs ADR-0017.** ⚠️ ADR-0063: the sway is only realisable if
  the paws can translate laterally, and ADR-0017 removed the joint that could do
  it. Options to price: **accept paw scrub** (size it against ADR-0009's own
  friction limit), **add abduction** (+4 motors against a budget ADR-0058 only just
  closed), **lengthen the lateral moment arm** (mechanical, trades against
  tension), or **drop static stability** for dynamic walking — ADR-0009's option
  E, rejected then because the dynamics milestone did not exist. It exists now.
- **A WALKING spine is still untested.** ⚠️ M58 is sway *in place*, legs planted.
  Swing legs and contact transitions are not in it.
- **`mechanical/` owes a PER-LEG ANKLE ARM.** ⚠️ ADR-0061: the hind ankle runs at
  **1.19×** its continuous rating over a trot and the fix is a constant,
  **14.0 → 16.6 mm**. The arms are shared between legs today and the hind leg's
  other pairs have 5× margin, so this wants a per-leg number and a sheave that
  fits.
- **`mechanical/` owes TWO routing features**: the cable **clamped** at each sheave
  (ADR-0055) and each pair **split across its via-pulleys** (ADR-0058). Neither is
  drawn.
- **`mechanical/` owes a CABLE TERMINATION at every sheave.** ⚠️ ADR-0043's leg was
  drawn with cables resting in grooves. Clamping needs a fitting, a groove profile
  and an assembly step per joint.
- **Then the ANKLE ANCHOR MIGRATION: hind 270°, fore whatever the mirrored leg
  wants.** ⚠️ It re-derives 17 tests across M42—M47, so it must come after the fore
  leg and be done **once**. Every number for it is in ADR-0053.
- **Then re-derive the fore-leg figures in M43—M47.** ⚠️ The welded/floating drift,
  ADR-0049's "the binding tendon is the hind hip extensor", ADR-0052's fore tracking
  error. The hind-leg numbers, which is most of what those ADRs argue from, stand.
- **RECONCILE THE MOTOR COUNT.** ⚠️ ADR-0056: the simulation runs 20 leg motors
  where ADR-0008's budget carries 12. Either implement the **variable-radius
  pulley** — which ADR-0008 decided and M46's spool machinery can express (one
  rotor, two spools) — or re-open ADR-0008 at **5.36 kg**. It is a transmission, so
  it belongs in the same rebuild as M50's clamped capstans, and it is now the second
  thing on the critical path.
- **PER-LEG outer gains.** ⚠️ On the same gains the fore leg tracks to 11.8°
  against the hind's 3.5 — different link lengths and a stance hock 81° away make
  it a different plant.
- **Attach a SERVO BANDWIDTH to G3's specification.** ⚠️ 175 kN/m of spring
  delivers 141 behind a 3000 rad/s rotor loop and **95 behind a 1000** one, which is
  outside the band ADR-0050 handed to mechanical.
- **`ROTOR_ARMATURE` is `[assumed]` at 2e-5 kg·m^2** and now matters — it sets
  that 120 Hz mode. ⚠️ It joins the vendor's Kt reference point as an actuator
  number the project owes itself.
- **Spool the QUADRUPED.** M46 gated one leg, the way M42 did before M43.
- **Add the null-space POSTURE task.** ⚠️ With the foot pinned by contact a
  3-joint leg has one internal DOF and ADR-0049's driver does not control it, so the
  ankle sags ~5° and the required tension climbs 207 → 223 N. A first attempt cut
  the sag to -1.7° but destabilised above kp 10.
- **Fold in ADR-0050's 4.83 kg**, the way ADR-0046 folded in 4.30: the CAD, the mass
  closure and `params` all still say 4.3041.
- **Four more motor DRIVERS.** ⚠️ Nothing in `electronics/` has seen Option A.
- **The THERMAL case for the standing tendon** — ⚠️ **blocked by ADR-0056**: the
  tension does not converge, it ramps to the 222.9 N motor ceiling by t ~ 9 s and
  pins, tracking an uncontrolled joint drift. There is no steady value to hand the
  thermal model until the **posture task** lands. ADR-0023 made standing the worst
  thermal case at the nominal 19.6 N; the real answer is worse than 2.5× and is not
  a number yet.
- **Fold the ATTITUDE term into `wbc.desired_wrench`.** ⚠️ It returns a zero desired
  moment, which is right for M33's in-place trot and wrong for standing; M44 adds an
  angular PD outside the function.
- **Re-place the girdle SPOOLS.** ⚠️ ADR-0049 found they degrade the hip moment arms
  from ×28.000 mm to 25.875 / -27.173 and make the pair asymmetric — they were
  placed by the packaging study, not by routing.
- **Decide `spring_rest_angle[2]`.** ⚠️ `params` says 0.0, which is **97° from the
  hind stance hock**, and the fore and hind stance angles are **81° apart**. The
  rigs now derive it per leg; mechanical still owes the number.
- **Add the articulated spine (6 DOF) and the tail** to reach the full 19 DOF. M43
  is 18: 6 free + 12 leg, with the trunk a single rigid box on purpose.
- **Re-run the anchor sweep whenever routing changes.** ⚠️ M43 found the dead band
  **moves between joints** under a change of hinge convention, which makes the sweep
  a build step rather than a one-off.
- **Then re-measure the balance arc on the RECOVERY criterion** and re-derive
  ADR-0029. M41 left **five findings suspended** as `xfail(strict=True)` because the
  survival measurement they read went degenerate. ⚠️ M42 changes the argument for
  doing it: the right plant to re-measure on is the **tendon** one, not the servo
  one. `strict=True` means those marks fail loudly if any starts passing on its own.
- **Size and specify G3's series-elastic element** at ~175 kN/m (ADR-0047) — it is
  now a mechanical work item with a target, and ADR-0026's compliance finding
  depends on it existing.
- **Re-examine ADR-0002 Option B** for the ankle, which has no restoring stiffness.
- **Fold in the COUPLED tendon map** (ADR-0042). ⚠️ M42 weakens the case for doing
  it analytically: the tendon plant produces the coupling for free, so the question
  is whether `TendonMap` needs to model what the simulation now measures.
- **Re-run LEG_TENDON_SPEC §1.1 and §2 again** — both are stale at 17.73 N·m /
  638 N. Third time for that document.
- **⚠️ Email the vendor about the 0.35 N·m/A reference point — now the single
  highest-value open item in the actuator story.** If the vendor's Kt is the right
  output-side constant, every M40 temperature rises a further **1.61×** and *no
  finish or airflow in the sweep saves a continuous trot*. It costs an email.
- **`power.KT` should carry BOTH Kt branches**, or become an explicit sensitivity
  the way the battery numbers already are (ADR-0044).
- **Re-target the routing objective to LOAD, not wrap.** Free 8 % of cable margin
  (ADR-0043 finding 7).
- **Re-check design review F2's fore/hind split** — it rested on a 1.16× leg
  asymmetry that measures 1.00×.
- **The BOM with real part numbers.** Now the only thing between M36's geometry and
  a leg that can be quoted. Bearings and cable are `[owed: BOM pass]`; the motor is
  already a real surveyed part.
- **Adopt the section increase** (Ø12→Ø14, Ø10→Ø12, Ø8→Ø10) in `params.py` and
  fold M36's corrected torques into LEG_TENDON_SPEC's body text, not just its
  banners. `tests/test_leg_detail.py` asserts the *defects*, so those tests failing
  is the signal that this landed.
- **A Ø60 hip sheave against the girdle.** M36 deliberately excludes the sheaves
  from its interference sweep — they cannot foul a link, but they may well foul the
  girdle, and the packaging study owns that question.

### Still open from the modelling arc

- **Resolve the 42.2 vs 39.5 mm contradiction** (was M36's slot). At a 0.117 s stance with
  `placement_gain = 0.5` the harness **genuinely recovers** (1.4× its own floor) from
  a disturbance **above** the exact feet-only viable bound. Under the recovery
  criterion those are the same quantity, so one of them is wrong. Candidates: the
  bound reuses the nominal plant's `reach` at a stance where the real reach differs;
  or the LIPM basis fails there. ADR-0022 put LIPM/MuJoCo agreement at ~2 %; this is
  6.8 %. **Whichever way it resolves, something load-bearing is wrong** — either the
  viable set that ADR-0033 used to declare NFR15 achievable, or the simulation that
  every envelope since M21 rests on.
- **Re-measure the arc on the recovery criterion.** M21–M34's figures are survival
  figures. They are internally consistent and stay reproducible via
  `measure_envelope(recover=False)`, but **every comparison to `viable.py` needs
  redoing**, starting with ADR-0033's 97 % and ADR-0037's 86 %.
- **Give the placement law integral action.** The steady-state offset is a missing
  term, not a missing actuator — and it is the first concrete, diagnosed control
  defect this arc has produced. `test_the_envelope_measures_SURVIVAL_not_recovery` is
  written to fail when this lands.

### Gated on M36

- **Whole-body MPC**, or a decision to accept the feet-only capability and revisit
  NFR15. ADR-0037 closed the "add another DOF to a per-step position controller"
  branch; these are the two that remain.
- **ADR-0019/0020's friction cost, to significance.** ADR-0035 confirmed the mechanism
  and doubted the magnitude (**~14 % at μ 0.70**, not ~100 %), but `n` caps at 11
  because low-friction runs fall before the later phases can be sampled. More power
  needs a controller that survives longer down there — the same wall as ADR-0032.
- **Reinstating the 67 cm/s trot.** Blocked on the item above, deliberately: ADR-0034
  and ADR-0035 both declined to move it on a marginal statistic.

### Gated on nothing

- **Electronics and firmware** — the largest unbuilt piece, and it has carried the
  note "gated on nothing" since M1. `electronics/` holds no schematic; `firmware/src/`
  is empty. The M1 interface specs and `firmware/include/tomcat_can_schema.h` are the
  starting point. Blocked only on the two reconciliation items below.
- **Cell selection** — the runtime rests on two `[assumed]` battery numbers
  (175 Wh/kg, 80 % usable).
- **Regeneration** — credited at zero; a backdrivable drive could recover some of the
  27 W of mechanical work.

### Open, no owner

- **Spine axial twist**, and with it the **righting milestone**
  ([ADR-0007](DESIGN_DECISIONS.md)): flight-phase reorientation via twist + leg
  shape-change, the single-tendon tail as a coarse bias term, and a fall-detect →
  reorient law (lit-review Q4). The lateral bend is done (M5); **twist is not
  started**, and it is the last unmodelled piece of design principle P2.

### Closed since this list was last curated

Kept as a short table so the deletions above are auditable rather than silent.

| item that used to sit here | closed by |
|---|---|
| Full rigid-body terms, the `dH/dt = 0` caveat | M13 — [ADR-0018](DESIGN_DECISIONS.md) |
| A closed-loop balance controller in MuJoCo | M21 — [ADR-0026](DESIGN_DECISIONS.md) |
| Sustained trot thermal duty (OPEN_RISKS R5) | M18–M19 — [ADR-0023](DESIGN_DECISIONS.md)/[ADR-0024](DESIGN_DECISIONS.md) |
| "Justify or withdraw `plant.spine = 36.6 mm`" | M28 — [ADR-0033](DESIGN_DECISIONS.md): exact along the spine's own axis |
| Leg abduction — buy it or not | M28 — stays rejected, the existing authority is sufficient |
| Realise the load split `λ` | M32 — [ADR-0037](DESIGN_DECISIONS.md): realised, and it makes the loop worse |
| "Does the robot need along-line authority?" | M27/M32 — it exists and is reachable; the controller was the limit |
| Integrate the allocation with the gait | M34 — [ADR-0039](DESIGN_DECISIONS.md): built, measured, **not adopted** |
| Flatten the harness noise floor across stance | M35 — [ADR-0040](DESIGN_DECISIONS.md): done (`placement_gain`), and it exposed the criterion |
| Shop drawings / manufacturable geometry (ASSEMBLY_SPEC §6) | M36 — [ADR-0041](DESIGN_DECISIONS.md): one leg drawn as parts; BOM still owed |
| The tendon drive actually drawn (P1) | M37 — [ADR-0042](DESIGN_DECISIONS.md): five runs routed; the map turns out coupled |
| Re-run the whole-body budget with real hardware | M38 — [ADR-0043](DESIGN_DECISIONS.md): closes at 4.30 kg; NFR5 must move |
| Thermal test at the trot duty (`motor-reality-check §5`) | M39 — [ADR-0044](DESIGN_DECISIONS.md): passes **on spec** at 0.64–0.81× the rating |
| `power.py`'s `I²R_pp` shorthand (flagged since M16) | M40 — [ADR-0045](DESIGN_DECISIONS.md): corrected to `1.5·I²R_pp`; overturns ADR-0023 |
| Fold the measured mass + spool into `params.py` | M41 — [ADR-0046](DESIGN_DECISIONS.md): done; five findings suspended |
| Build it as a real tendon drive in simulation | M42 — [ADR-0047](DESIGN_DECISIONS.md): one leg gated; G3 sized at last |
| Scale the tendon plant to the whole body | M43 — [ADR-0048](DESIGN_DECISIONS.md): 18 DOF built; it leans, and four M42 numbers were wrong |
| Make the pull-only quadruped STAND | M44 — [ADR-0049](DESIGN_DECISIONS.md): it stands at 0.006 deg; a lone tendon's moment arm reverses mid-ROM |
| Settle ADR-0002 Option A vs B at the ankle | M45 — [ADR-0050](DESIGN_DECISIONS.md): Option A, capstan; decided on kinematic reach, +528 g |
| Make cable PAY-OUT representable | M46 — [ADR-0051](DESIGN_DECISIONS.md): spool drivetrain, statics exact; it exposed a missing controller |
| Drive the spooled plant | M47 — [ADR-0052](DESIGN_DECISIONS.md): cascade derived in closed form; G3 now needs a servo bandwidth beside it |
| Migrate the ankle anchor | M48 — [ADR-0053](DESIGN_DECISIONS.md): deferred; the fore leg's routing was never mirrored, and three control findings retract |
| Mirror the fore leg | M49 — [ADR-0054](DESIGN_DECISIONS.md): narrowed — only the ankles were ever validated; 3 of 4 fixes measured, the hind hip has none |
| Settle the hind hip's construction | M50 — [ADR-0055](DESIGN_DECISIONS.md): clamp the cable to the sheave; a resting wrap has a window, a clamped capstan does not |
| Build the clamped transmission | M52 — [ADR-0057](DESIGN_DECISIONS.md): exact on both legs at every angle; and ADR-0002 vs ADR-0008 is a real conflict |
| Decide ADR-0002 vs ADR-0008 | M53 — [ADR-0058](DESIGN_DECISIONS.md): ADR-0008 wins; 12 motors, 4.30 kg, and the tension saturation goes with it |
| Re-derive onto the shipped plant | M54 — [ADR-0059](DESIGN_DECISIONS.md): it is the default now; four routing defects retire on one measurement, and G3 had no home |
| Re-derive the cascade | M55 — [ADR-0060](DESIGN_DECISIONS.md): the gains transfer unchanged; the cost was headroom, and a step command is not a test |
| Price the variable-radius profile | M56 — [ADR-0061](DESIGN_DECISIONS.md): declined; the binding joint has no shape, and the load split named the other leg |
| Build the articulated spine | M57 — [ADR-0062](DESIGN_DECISIONS.md): 18 of 19 DOF and it stands; the sagittal axis had never been in any model |
| Run the sway on the actuated plant | M58 — [ADR-0063](DESIGN_DECISIONS.md): 6 % of the designed sway, and every paw slides; ADR-0009 vs ADR-0017 |
| Make the foot a foot | M59 — [ADR-0064](DESIGN_DECISIONS.md): the contact sat 8.3 mm behind the site, and five published results moved |
| Price the pad and the arm together | M60 — [ADR-0065](DESIGN_DECISIONS.md): the arm buys cost not sway; the remaining gap is control |

## Open reconciliation items (lead)

- ~~**Motor count 30 vs. ~25**~~ — **SETTLED at 19.** [ADR-0008](DESIGN_DECISIONS.md)
  adopted the variable-radius pulley (one motor per antagonistic pair) for 16, and
  [ADR-0009](DESIGN_DECISIONS.md) added the lateral spine DOF for **19 = 12 leg + 6
  spine + 1 tail** (NFR2c). The 30-vs-25 wording predated both and survived ~25
  milestones here; **the backplane channel count is 19.**
- **ADR-0004 tension method** (load-cell vs. current estimate) still open — parks
  tension scaling in both the firmware schema (`CFG_TENSION_SCALE`) and the
  driver board (DNP load-cell path).
- Motor `Kt` + bus voltage: blocked on a specific motor selection; blocks
  detailed electrical sizing.
- ⚠️ **`BalanceHarness` mixes two values of `omega`** (found in M34). `reset()`
  re-measures `omega` from the settled CoM height (**7.7732**) and the DCM uses it;
  `self.deadbeat` is still the constructor's, from the analytical plant
  (**7.7652**). The gap is 0.1 % in omega and **0.043 % in the deadbeat
  coefficient**, and closing it is a one-line change — but it moves every
  simulation-measured envelope, so it needs its own milestone with a re-measurement,
  not a quiet fix. M34 deliberately reproduces the shipped mixture for unadapted
  stances so its own comparison is clean.
