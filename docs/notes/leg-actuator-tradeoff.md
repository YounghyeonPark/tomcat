# Leg-actuator trade study — tendon-drive vs. compact backdrivable direct-drive

**Requested by:** ADR-0003 (Proposed) — "Add a required leg-actuator trade study
— tendon-drive vs. compact backdrivable direct-drive — scored on IMF, torque
density, reflected inertia, and control complexity, using the M1 leg torque
budget as input, before finalizing leg actuators."

**Scope:** the **legs only.** Tendon-drive for the **spine** and for
**centralizing limb mass** is already committed (ADR-0001, ADR-0006, and the
ADR-0003 preamble) and is *not* reopened here. The question is narrowly: what
actuates the leg joints — long tendons from torso/girdle motors, a compact
**quasi-direct-drive (QDD)** motor at each joint, or a hybrid split?

**Confidence legend** (same as LITERATURE_REVIEW.md): ✅ adversarially verified ·
◐ primary / single-read · ⚠️ caveat (paywall / secondary / scale-mismatch /
single-design).

---

## 1. Quantitative input (M1 leg budget)

From `kinematics/src/tomcat_kin` (`params.py`, `torque_budget.py`,
`whole_body_budget.py`), placeholder geometry, worst single-leg **land** case:

| Quantity | Value | Where |
|---|---|---|
| Robot mass | 3.0 kg (placeholder) | `LoadCase.body_mass_kg` |
| Land case | 1 stance leg, dynamic factor ×2.5 | `DEFAULT_LOADS` "land (1-leg)" |
| Foot support force | **73.6 N** (= 3.0 × 9.81 × 2.5 / 1) | `foot_support_force_N` |
| Worst-case joint torque, hip / knee | **~12–13 N·m** | `torque_budget` sweep |
| Leg moment arms (hip, knee, ankle) | **15, 12, 10 mm** | `TendonParams.joint_moment_arm` |
| Resulting peak cable tension | **~870 N (hip), ~1050 N (knee)** | `resolve()` = 5 N + \|τ\|/r |

Cross-check of the tension figure (AIC split, `_resolve_antagonistic`):
`T = T_bias + |τ|/r`. Knee: `5 + 12.6/0.012 ≈ 1050 N`. Hip: `5 + 13/0.015 ≈ 872 N`.
This is the ~850–1050 N band ADR-0003 flagged. **Everything below is placeholder
geometry** — the *ratios and mechanisms* are the finding, not the absolute mm/N.

**Scale note (load-bearing):** all MIT Cheetah numbers below come from a **~33 kg
robot with ~100 N·m target joints**. T.O.M.C.A.T. is a **~3 kg / ~13 N·m** machine
— roughly **1/8 the torque**. The QDD *design philosophy* (low gear ratio, low
reflected inertia, current-based force control) scales down cleanly; the specific
torque-density / inertia numbers **do not transfer directly** and must be
re-derived at cat scale. ⚠️

---

## 2. Axis-by-axis comparison

### Axis 1 — IMF / backdrivability
The **Impact Mitigation Factor (IMF)** is Wensing et al.'s dimensionless metric
for a floating-body robot's backdrivability at impact (normalized inertial
impedance; higher = better shock rejection). Its drivers, per the paper, are
**minimal reflected actuator inertia, minimal leg mass, and minimal actuator
compliance** — and reflected inertia scales with the **square of the gear ratio**.

- MIT Cheetah uses a **single-stage 5.8:1 planetary** — "the largest ratio that
  can be obtained at a single stage in the given space." ◐
- Its whole point: the low-impedance QDD design **mitigates impacts comparably to
  quadrupeds using series-elastic actuators (SEAs)** while keeping high-bandwidth
  open-loop force control. ◐ (LITERATURE_REVIEW.md Q5, verified against the paper
  abstract + MIT open-access copy this run.)
- The comparison in the paper: a highly-geared model (HUBO, DC motor rotor
  inertia **3.33×10⁻⁶ kg·m²** behind a **160:1** reduction → reflected inertia
  **0.0852 kg·m²**) has IMF only **~52%** of a hypothetical SEA version — i.e. a
  high gear ratio *destroys* backdrivability. ◐/⚠️ (this 0.0852 / 160:1 figure is
  the **HUBO comparison case, not the Cheetah's own actuator** — do not
  misattribute.)
- Gear-ratio penalty, order of magnitude: reflected inertia ∝ N², so 5.8:1 vs
  160:1 is a **(160/5.8)² ≈ 760×** reflected-inertia difference. This is *why* low
  gearing wins on IMF.

**Tendon-drive** achieves low reflected inertia differently — the motor+gearbox
sit in the torso, so essentially **zero rotor inertia is reflected to the leg**.
But the cable path adds **friction and series stretch**, and RoboCat's shock
absorption was demonstrated only in **simulation with frictionless guides**
(LITERATURE_REVIEW.md Q6). No source gives a measured leg-tendon IMF.

- QDD (direct-drive): ✅ measured, SEA-comparable IMF; mechanism well understood.
- Tendon: ◐ low reflected rotor inertia in principle; real friction/stretch IMF
  **unquantified** — a gap.

### Axis 2 — Torque density & meeting ~12–13 N·m at cat scale
- MIT Cheetah target peak torque **~100 N·m** (from max ground-reaction forces);
  Cheetah 2 QDD transmission cited at **~58 N·m/kg** torque density. ⚠️ secondary
  (web summary of the paper, not read from primary).
- T.O.M.C.A.T. needs only **~13 N·m**. A **mini-Cheetah-class** QDD actuator
  (≈17 N·m peak, ~6:1) already exceeds this, so a **smaller/lighter** cat-scale
  QDD comfortably meets the requirement with margin. ◐/⚠️
- Tendon-drive: a torso motor + reduction meets any of these torques trivially;
  torque delivery is limited by **cable tension** (Axis 5), not motor torque.

Both architectures **can** meet 12–13 N·m. QDD does it with a small motor at the
joint; tendon does it with a torso motor but pays in cable tension.

### Axis 3 — Reflected / limb inertia (the P1 lever)
This is tendon-drive's genuine, unique win and the reason P1 exists.

- **Tendon:** motors live in the torso/girdle; only cable, pulley, and bearing
  mass sits on the limb → **lowest limb swing inertia** → best agility and lowest
  distal mass to decelerate at footfall. ✅ (mechanism), the core P1 argument.
- **QDD:** the motor mass sits **at the joint**. Limb inertia about the hip is
  `Σ mᵢ rᵢ²`, so where the motor sits matters enormously:
  - a motor **at the hip** (r ≈ 0, the body pivot) adds **negligible** swing
    inertia — nearly "free";
  - a motor **at the knee/ankle** (large r) adds inertia **∝ r²** — the worst
    place for it.
- Sci. Reports 2022 (LITERATURE_REVIEW.md Q2b) shows leg mass is **not**
  negligible: a 0.454 kg knee mass is what bends the compliant trunk — i.e. distal
  mass has real dynamic consequences. Adding a QDD motor distally works against
  both P1 and the spine energy-exchange story.

Ranking: **tendon > QDD-at-hip > QDD-at-knee/ankle** for limb inertia. This axis
is the strongest case *for* keeping motors off the distal limb.

### Axis 4 — Control complexity
- **QDD:** joint torque ≈ motor current × Kt × ratio. **Current-based, open-loop
  force control** — no cable model, no tension sensor, no coupling map. This is
  the mature, well-trodden path (MIT Cheetah, Mini-Cheetah, Doggo). ✅ simplest.
- **Tendon:** must model/compensate **friction, cable stretch, and inter-tendon
  coupling**, and needs **tension sensing** (ADR-0004, still Proposed) to close a
  force loop. ADR-0001 explicitly lists "higher control complexity" as the cost.
  The sensorless-FOC note adds that a **static high-torque stance hold** is
  already a hard, thermally-limited operating point regardless of transmission —
  compounded on the tendon side by slack/stretch. ⚠️ hardest.

### Axis 5 — Tension amplification (the specific ADR-0003 ask)
`T = |τ| / r`. Small leg moment arms turn modest joint torque into large cable
tension. At the placeholder arms the knee hits **~1050 N** — **15–50×** the
RoboCat sanity band (pretension ~50 N; antagonistic ~20–70 N,
LITERATURE_REVIEW.md Q6/Seed B). High tension drives **cable, pulley, bearing,
and frame sizing, and friction losses** (which then feed back into Axis 4).

How the design levers move it (knee, τ ≈ 12.6 N·m):

| Knee moment arm r | Peak tension `12.6/r` | Note |
|---|---|---|
| 12 mm (placeholder) | **1050 N** | ~15–50× RoboCat band |
| 20 mm | 630 N | still high |
| 25 mm | 504 N | |
| 30 mm | 420 N | matches the arm the spine was raised to |
| 40 mm | 315 N | pulley now bulky/heavy, distally |

Two structural observations:
1. **Moment-arm sizing helps but cannot fully close the gap** at leg torque
   scale: even a 30 mm pulley (2.5× the placeholder) leaves ~420 N, and a big
   distal pulley **adds distal mass/size**, partially eroding the Axis-3 inertia
   win that motivated tendon in the first place.
2. **The tension problem is worst exactly at the high-torque joints (hip, knee)**
   — which are also the legs' most demanding joints. So tendon-drive is *least*
   comfortable precisely where the legs need the most. A **QDD motor sidesteps
   tension amplification entirely** (no cable). This is the sharpest single
   argument against pure-tendon legs.

---

## 3. Scorecard

Scores 1–5 (5 best). Weights reflect P1 (agility/limb-inertia) emphasis while
keeping practicality (control, tension) material.

| Axis | Weight | Pure tendon legs | Compact QDD legs | Basis |
|---|---|---|---|---|
| 1. IMF / backdrivability | 20% | 4 ◐ | 5 ✅ | QDD SEA-comparable, measured; tendon friction/stretch IMF unquantified |
| 2. Torque density (meet 13 N·m) | 15% | 4 | 4 | both meet it at cat scale |
| 3. Reflected / limb inertia (P1) | 25% | 5 ✅ | 3 ◐ | tendon motors in torso; QDD adds joint mass (hip≈free, distal costly) |
| 4. Control complexity | 20% | 2 ⚠️ | 5 ✅ | tendon friction/stretch/coupling + tension sensing vs. current→force |
| 5. Tension amplification | 20% | 2 ⚠️ | 5 ✅ | ~1050 N at placeholder arms; QDD has no cable |
| **Weighted total** | | **3.45** | **4.35** | |

**QDD wins the legs on this weighting (4.35 vs 3.45).** The result is robust:
tendon only overtakes QDD if the limb-inertia axis is weighted to dominate almost
everything else, and even then QDD's control + tension advantages keep it close.
Tendon's win is concentrated in one axis (inertia); QDD wins or ties four of five.

---

## 4. Recommendation

**For the legs, baseline compact backdrivable QDD (direct-drive) — not
tendon-drive.** The two properties tendon-drive was meant to buy at the leg —
**shock tolerance (IMF)** and **force control** — are delivered by a low-gear-ratio
QDD *without* cables, per the MIT Cheetah result, while the costs that make tendon
hard (friction/stretch/coupling, tension sensing, and ~1000 N cable tension from
small moment arms) all land **hardest at the legs**. At ~3 kg / ~13 N·m, a
cat-scale QDD is small enough that the required torque is easily met.

**Keep tendon-drive where it uniquely wins** (already committed, not reopened):
the **spine** and **mass-centralization** — many DOF driven from the girdles,
where per-tendon tensions are far lower and the limb-inertia payoff is real.

**P1 hedge — the inertia-optimal hybrid, if distal QDD mass proves prohibitive:**
if a bench check shows a cat-scale QDD motor is too heavy to sit on the distal
limb, fall back to **QDD at the hip (motor at the body pivot ≈ inertia-free) +
tendon-driven knee (from a hip/torso motor) + spring-return ankle** (ankle already
Option-B in ADR-0002). This is **biologically faithful** (proximal muscle mass,
light distal segments via tendons) and is **exactly RoboCat's physical topology**
— direct-drive hips/shoulders + elastic-cable knees/elbows (LITERATURE_REVIEW.md
Q6). If the knee is tendon-driven, size its moment arm to **≥ ~25 mm** to cap peak
tension near ~500 N. **Do not tendon-drive the legs everywhere.**

Order of preference for the lead: **(1) full-leg QDD → (2) QDD-hip + tendon-knee +
spring-ankle hybrid → (3) pure tendon legs (not recommended).**

This also cleanly re-frames the deferred BLDC+FOC-vs-geared-DC question in
ADR-0003: a QDD leg **is** BLDC+FOC with a low single-stage reduction; geared DC
is ruled out for the leg by the IMF/backdrivability requirement (high gearing kills
IMF — the 160:1 → 52% figure).

---

## 5. Open gaps / hand-offs

1. **Cat-scale QDD motor mass vs. limb-inertia budget** — the number that decides
   full-QDD vs. hybrid. **No source gives it; the M1 model treats links as
   massless.** → **tomcat-mechanical:** mass of a candidate ~13 N·m QDD actuator;
   **tomcat-kinematics:** add motor point-masses to the leg model and compute limb
   swing inertia for QDD-at-hip vs. QDD-at-knee vs. tendon. (Gap #2 in
   LITERATURE_REVIEW.md is now closed qualitatively; this is its quantitative
   remainder.)
2. **Measured tendon-leg IMF / friction** — RoboCat's shock result was
   frictionless simulation; no measured cable-path IMF exists. Tendon's Axis-1
   score (4 ◐) is optimistic until a real cable path is characterized.
3. **Torque-density / inertia at cat scale** — the 58 N·m/kg and 0.0852 kg·m²
   figures are ⚠️ secondary and at 33 kg scale; re-derive for a ~13 N·m motor.
4. **Static-hold thermal derating** — from the sensorless-FOC note, a DC
   high-torque stance hold is thermally limited for QDD too; a non-backdrivable
   detent/brake or a small holding reduction should be weighed for stance. →
   tomcat-electronics / tomcat-mechanical.
5. **Peak-torque primary read** — the ~100 N·m Cheetah target and 5.8:1 ratio
   should get a primary-source confirmation (the paper PDF did not render this
   run); currently ◐ from the paper abstract + reputable web summaries.

---

## 6. Proposed ADR-0003 decision text (for the lead — do NOT self-apply)

> **ADR-0003: Actuator technology**
> **Status:** Accepted (supersedes the Proposed trade-study placeholder)
> **Decision:** Split by subsystem.
> - **Legs — compact backdrivable quasi-direct-drive (QDD): BLDC + FOC with a
>   low single-stage reduction (target ≲ ~8:1).** The MIT Cheetah result shows a
>   low-gear-ratio QDD achieves an Impact Mitigation Factor comparable to
>   series-elastic designs while giving high-bandwidth current-based force control
>   — delivering the shock tolerance and force control we wanted from tendons, at
>   the leg, *without* cables. Decisive at the leg: the M1 budget shows small leg
>   moment arms (10–15 mm) amplify the ~13 N·m worst-case joint torque into
>   **~850–1050 N** cable tension — 15–50× the RoboCat ~20–70 N band — and
>   moment-arm sizing cannot fully close that gap without bulky distal pulleys.
>   Tendon-drive's costs (friction/stretch/coupling, tension sensing, tension
>   amplification) land hardest exactly at the legs; its one advantage (limb
>   inertia) is a single axis. Geared DC is ruled out for the leg because high
>   gearing destroys backdrivability/IMF.
> - **Fallback hybrid (if a cat-scale QDD motor is too heavy for the distal
>   limb):** QDD at the hip (motor at the body pivot ≈ inertia-free) + tendon-driven
>   knee from a proximal motor (moment arm ≥ ~25 mm to cap tension near ~500 N) +
>   spring-return ankle (ADR-0002). This is RoboCat's proven physical topology and
>   biologically faithful.
> - **Spine & mass-centralization — tendon-drive (unchanged; ADR-0001/0006).**
> **Consequences:** Requires a cat-scale QDD actuator selection + limb-inertia
> check (gap for tomcat-mechanical / tomcat-kinematics) to confirm full-QDD over
> the hybrid. Leg force control becomes current-based, simplifying ADR-0004 for
> the legs (tension sensing stays a spine concern). A static high-torque stance
> hold remains a thermally-limited operating point for QDD (see
> sensorless-foc-stance-hold.md) — evaluate a detent/brake for stance.

---

### Sources
- Wensing, Wang, Kim et al., *Proprioceptive Actuator Design in the MIT Cheetah*,
  IEEE T-RO 2017 — [ResearchGate](https://www.researchgate.net/publication/312558722_Proprioceptive_Actuator_Design_in_the_MIT_Cheetah_Impact_Mitigation_and_High-Bandwidth_Physical_Interaction_for_Dynamic_Legged_Robots)
  · [MIT open access](https://dspace.mit.edu/server/api/core/bitstreams/53fde66c-bd98-4dd7-a95b-3c9a5d11cf69/content)
  · [Wensing copy](http://www.mit.edu/~pwensing/Papers/Wensing_et_al-2017-TRO.pdf)
  (5.8:1; reflected inertia ∝ N²; HUBO 160:1 → IMF ≈52% of SEA; SEA-comparable IMF)
- Kim et al. / Seok et al., *Actuator Design for High Force Proprioceptive Control
  in Fast Legged Locomotion* — [ResearchGate](https://www.researchgate.net/publication/260820531_Actuator_Design_for_High_Force_Proprioceptive_Control_in_Fast_Legged_Locomotion)
- Kau et al., *Stanford Doggo: An Open-Source, Quasi-Direct-Drive Quadruped* —
  [arXiv:1905.04254](https://arxiv.org/pdf/1905.04254) (QDD philosophy, low-ratio)
- RoboCat (Carpenter et al., ASME IMECE2011-63805) — hybrid direct-drive hips +
  elastic-cable knees; pretension ~50 N; tensions ~20–70 N —
  [ResearchGate](https://www.researchgate.net/publication/267593853_A_Biomimetic_Elastic_Cable_Driven_Quadruped_Robot_The_RoboCat)
- Internal: `docs/LITERATURE_REVIEW.md` (Q1, Q5, Q6, Seed B),
  `docs/notes/sensorless-foc-stance-hold.md`, `kinematics/src/tomcat_kin/`
  (`params.py`, `torque_budget.py`, `whole_body_budget.py`).
