# TomCat — Leg Tendon-Drive Mechanical Spec (first pass)

Owner: **tomcat-mechanical** · Milestone: **M1** · Ties to
[ADR-0002](../docs/DESIGN_DECISIONS.md), [ADR-0003](../docs/DESIGN_DECISIONS.md),
[ADR-0004](../docs/DESIGN_DECISIONS.md).
Status: **first-pass** — every value is a placeholder unless labelled `[sourced]`.

ADR-0003 committed the project to **pure tendon-driven legs** (honoring P1),
*knowingly* accepting a high cable-tension burden that the leg-actuator trade
study ([leg-actuator-tradeoff.md](../docs/notes/leg-actuator-tradeoff.md)) scored
against. This document is the mechanical answer to that burden: it engineers the
tension down to a buildable number and specs the cable, pulleys, bearings,
routing, and static-hold hardware that follow. It presents numbers as tables for
**tomcat-kinematics** / the lead to fold into `LegParams` / `TendonParams` later.
It does **not** edit `params.py`, `kinematics/`, or `docs/`.

Conventions match the kinematics model and
[SPINE_TAIL_SPEC.md](SPINE_TAIL_SPEC.md): **SI units**; sagittal-plane
**digitigrade 4-link leg** (hip → femur → stifle → tibia → hock → metatarsus →
*passive* paw) with **3 actuated joints**; joint torque `τ = T·r`, so cable
tension `T = τ / r` (+ a pretension / co-contraction floor).

Label legend: `[sourced]` traceable to lit-review / ADR / M1 budget;
`[assumed]` first-pass engineering guess; `[owed]` must come from another owner.

---

## 0. The tension problem, stated

`T = τ / r`. A **small moment arm hugely amplifies required cable tension** —
this is the entire burden ADR-0003 accepted. The worst case (single-leg
**land**, 73.6 N foot force × 2.5 impact) drives **~12.4 N·m at the hip**. At
the original placeholder arms (hip 15 mm, knee 12 mm) this yielded **~870 N
(hip) / ~1050 N (knee)** — **15–50× the RoboCat ~20–70 N sanity band**
`[sourced: M1 budget; lit Q6]`.

Two things must be said honestly up front:

1. **Geometry alone roughly halves the peak, no more.** The largest pulley a
   cat-scale leg joint can package is ~25–28 mm; that cuts the land peak from
   ~1050 N to ~450 N (§1). It **cannot** reach the RoboCat 20–70 N band, because
   that band is a lighter, non-impact demonstrator — our peak is a deliberate
   ×2.5 single-leg *landing transient*, not a continuous load.
2. **The residual is a transient, not a duty.** Under **stand and trot** the same
   arms give 50–150 N cable tension — near the RoboCat band. The irreducible
   ~450 N lives **only** in the rare land impact, and it sets **structural**
   sizing (cable break strength, pulley/bearing static rating), not continuous
   thermal/fatigue sizing. That distinction is what makes the design buildable.

---

## 1. Moment-arm sizing per leg joint

### 1.1 Joint torques by load case

Torque scales linearly with foot support force. All three columns are now taken
directly from the **post-M4 whole-body budget** in the digitigrade posture
`[sourced: whole_body_budget]` (trot scaled from the same sweep at 22.08 N).

| Joint | Stand (4-leg, ×1.0) | Trot (2-leg, ×1.5) | **Land (1-leg, ×2.5)** | Source |
|---|---|---|---|---|
| **Hip** | 1.24 N·m | 3.71 N·m | **12.36 N·m** ← worst | `[sourced: budget]` |
| Stifle (knee) | 0.75 N·m | 2.25 N·m | **7.49 N·m** | `[sourced: budget]` |
| Hock (ankle) | 0.48 N·m | 1.44 N·m | **4.79 N·m** | `[sourced: budget]` |

The digitigrade fold **redistributed** these versus the old flat-footed posture:
the stifle dropped sharply (12.6 → 7.49 N·m) while the hock rose (3.68 → 4.79),
because the folded metatarsus now carries more of the landing moment. ADR-0002
still reserves the hock as spring-return (Option B), so its tendon *count* is
lightest — but not its *force*.

### 1.2 Packageable arm at each joint

The lever is bounded by where the pulley physically fits, and by the P1 cost of
distal mass: a big pulley at the **knee/ankle** adds swing inertia and erodes the
very limb-inertia win that motivated tendon-drive (trade study Axis 3). So the
arm is pushed largest where it is cheapest (hip, at the body pivot) and held to
the smallest defensible value distally.

| Joint | Placeholder r | **Recommended r** | Upper bound set by | P1 cost | Label |
|---|---|---|---|---|---|
| Hip   | 0.015 m | **0.028 m** | girdle/hip housing has room; motor at pivot | ~none (r≈0 from swing axis) | `[assumed]` |
| Knee  | 0.012 m | **0.025 m** | mid-limb pulley vs. shank clearance & ROM | moderate (adds distal mass) | `[assumed]` |
| Ankle | 0.010 m | **0.014 m** | distal, tight; spring-return joint | high (worst place for mass) | `[assumed]` |

Trade-study guidance was "knee arm **≥ ~25 mm** to cap tension near ~500 N"
`[sourced: leg-actuator-tradeoff §4]` — the 25 mm recommendation meets that
exactly.

### 1.3 Resulting cable tension (T = τ/r + pretension)

Pretension floor 5 N (matches current `TendonParams`); the AIC co-contraction
bias `T_bias` (Kengoro ≈ 19.6 N, applied at runtime) would add a few % more.

> ⚠️ **Recomputed after the M4 posture fix.** The figures below are from the
> current model (digitigrade negative-knee fold, stance `(0.05, −0.19)`, real
> distributed mass). The *earlier* revision of this table was computed under the
> old positive-knee posture with feet ~0.2 m ahead of the hips — a posture since
> shown to be statically unstable. **The worst joint changed: it is now the HIP,
> not the knee, and the ankle load rose sharply.**

| Joint | r | Stand | **Land (peak)** | Old-posture land peak | Change |
|---|---|---|---|---|---|
| **Hip**   | 0.028 m | 49 N | **~447 N** ← worst | ~470 N | −5 % |
| Knee (stifle) | 0.025 m | 35 N | **~305 N** | ~510 N | **−40 %** |
| **Ankle (hock)** | 0.014 m | 39 N | **~347 N** | ~270 N | **+29 %** ⚠ |

Driving joint torques (single-leg land, ×2.5): hip **12.36 N·m**, stifle
7.49 N·m, hock 4.79 N·m.

**What geometry solved / what remains:**
- **Solved:** continuous operation (stand) now sits at **35–49 N** — inside the
  RoboCat ~20–70 N band, an ordinary cable-drive regime.
- **Irreducibly remains:** a **~450 N hip land transient.** No packageable arm
  removes it. This residual is the *structural design load* for
  cable/pulley/bearing (§2, §3) — a rare peak, not a duty.
- **Hock arm — reviewed, KEEP 14 mm.** See §1.3a below.

**Recommended moment-arm set → `TendonParams.joint_moment_arm = (0.028, 0.025,
0.014) m`** *(hock confirmed by §1.3a)*. Structural design load is now
**~465 N per tendon** (hip land + `T_bias`), down from ~525 N under the old
posture. **Hardware sizing in §2–§3 is deliberately RETAINED at the 525 N
basis** — an 11 % load drop is not a reason to downsize a structural margin,
and the placeholder inputs could still move. Components remain sized to
**~1 kN**.

### 1.3a Hock moment-arm review — **verdict: keep 14 mm**

M4's posture change raised the hock land peak 29 % (270 → 347 N) on the leg's
smallest arm, so the arm was reviewed. Because M4 also added real mass
properties, the P1 inertia cost could be *priced* rather than asserted. Leg
swing inertia is taken about the hip in the stance pose (per-link point masses at
their CoMs); the hock sits **134 mm** out on a 280 mm leg, so mass added there is
expensive. Pulley+bracket mass is estimated at 6 g at 14 mm scaling as `r²`
`[assumed]`.

| r (mm) | Land T (N) | Stand T (N) | added mass (g) | leg swing-inertia penalty |
|---|---|---|---|---|
| **14 (current)** | **347** | **39** | 6.0 | **5.1 %** |
| 18 | 271 | 32 | 9.9 | 8.4 % |
| 20 | 245 | 29 | 12.2 | 10.3 % |
| 25 | 197 | 24 | 19.1 | 16.1 % |

**Keep 14 mm.** Growing the arm buys nothing the design needs, and costs the one
thing tendon-drive exists to protect:

1. **Continuous load is already in band.** Stand tension is **39 N** at 14 mm —
   inside the RoboCat 20–70 N band. The 347 N is a *rare landing transient*.
2. **The transient is already covered.** With `T_bias`, the hock peaks at 367 N →
   **SF 6.0** on the specified 2.2 kN cable, comfortably past the SF ≥ 4 target,
   and only **37 %** of the ~1 kN hardware rating. It is *not* the governing
   tendon — the **hip** is, at 467 N / SF 4.7.
3. **It would attack P1.** Going to 25 mm adds **+11 percentage points** of leg
   swing inertia. Low limb inertia is the entire reason ADR-0003 accepted the
   tendon-drive tension burden ([trade study](../docs/notes/leg-actuator-tradeoff.md)
   Axis 3) — spending it to relieve a transient already carrying SF 6 is a bad trade.

**Revisit if** the impact factor rises above ×2.5, the body mass grows, or a
measured pulley mass comes in far below the `r²` estimate.

### 1.4 Cable-travel / spool sanity check

At `motor_spool_radius = 0.008 m`, using the **post-M4 negative-fold limits**
(hip ±120°, stifle −150…0°, hock −30…+150°):

| Joint | ROM | travel = r·ROM | motor sweep |
|---|---|---|---|
| Hip | 240° = 4.19 rad | `0.028 × 4.19 =` **117 mm** | 14.6 rad ≈ **2.3 rev** |
| Stifle | 150° = 2.62 rad | `0.025 × 2.62 =` **65 mm** | 8.2 rad ≈ 1.3 rev |
| Hock | 180° = 3.14 rad | `0.014 × 3.14 =` **44 mm** | 5.5 rad ≈ 0.9 rev |

⚠ The widened hip range pushes hip cable travel to **117 mm / ~2.3 rev** (was
88 mm at the old ±90°). Still fine for a multi-wrap spool, but it is now the
sizing case — check spool width and wrap stacking against it.

---

## 2. Cable spec

Structural design tension **~525 N** (retained basis — the post-M4 recomputed
peak is ~465 N at the hip, but the margin is kept, see §1.3); target **safety
factor ≥ 4** on peak (covers knot/splice strength loss ~30–50 %, abrasion, and
fatigue) → **breaking strength ≥ ~2.2 kN.**

**Recommendation: 1.5 mm braided UHMWPE (Dyneema SK78 / SK99, 12-strand).**

| Property | Value | Note / source |
|---|---|---|
| Material | UHMWPE (Dyneema SK78/SK99) | `[sourced]` high-modulus, creep-optimized grades |
| Diameter | **1.5 mm** | `[assumed]` sized to break strength + min bend radius |
| Breaking strength | **~2.2–2.5 kN** | `[sourced]` typical 1.5 mm 12-strand; SF ≈ 4.3–4.8 on 525 N |
| Tensile modulus (fibre) | ~100–120 GPa (SK99) | `[sourced]` — settled/spliced effective ~50–90 GPa |
| Elastic stretch at break | ~3–4 % | `[sourced]` low elastic stretch → good position fidelity |
| **Creep** | **non-negligible under sustained load** | `[sourced]` — the key drawback (see below) |
| Min sheave dia | ≥ ~10× cable dia → ≥ 15 mm | `[sourced]` UHMWPE bends tight; all joint pulleys (Ø50–56 mm) pass easily |
| Density | ~0.97 g/cm³ (floats) | `[sourced]` very light → protects P1 limb-inertia budget |

**Why not steel:** 1.0–1.2 mm 7×19 stainless wire rope reaches ~1.0–1.3 kN break
(marginal SF), has **near-zero creep** and high stiffness — but it needs a
**much larger min bend radius** (sheave dia ≥ ~15–20× rope dia → ≥ ~20 mm even
for idlers), suffers **bending fatigue** over small distal pulleys, and is
~8× denser (hurts P1 distal inertia). For compact multi-pulley routing to distal
joints, **UHMWPE wins**; steel is the fallback only if creep drift proves
unmanageable and larger pulleys are accepted.

**Creep / stretch handling (position-accuracy consequence):** UHMWPE elastic
stretch is low, but it **creeps** slowly under sustained tension — exactly the
stance-hold regime. This introduces slow position drift that a pure motor-side
encoder cannot see. Mitigations, in order:
1. **Static-hold brake (§4)** removes sustained tension from the cable during
   quiet stance — the single biggest creep reducer.
2. **Heat-set / pre-stretched** cable at build (settles construction stretch and
   much of primary creep).
3. **Cable/joint-state sensor** (ADR-0004, kept distinct from the rotor encoder)
   for periodic re-tension / recalibration — creep is slow, so low-rate
   correction suffices.

**Axial cable stiffness** (for a tendon-stretch term in the tendon map):
`k = E·A / L`. For 1.5 mm (A = 1.77 mm²), settled E ≈ 60 GPa, run L ≈ 0.30 m
(girdle → distal joint): `k ≈ 60e9 × 1.77e-6 / 0.30 ≈ 3.5×10⁵ N/m`. Stiffness is
**per-tendon and run-length dependent** — kinematics should compute it from the
per-tendon path length, not a single constant. Placeholder: `cable_stiffness ≈
3.5e5 N/m` `[assumed]`.

---

## 3. Pulley / bearing / routing (design load ~1 kN)

### 3.1 Joint pulleys

- **Sheave diameters** follow §1.2 arms: hip Ø56 mm, knee Ø50 mm, ankle Ø28 mm
  (radius = moment arm; the cable pitch line sits at `r`). All comfortably exceed
  the ≥15 mm UHMWPE min-bend.
- **Groove:** round-bottom, radius ~0.55× cable dia (≈0.85 mm) to seat the 1.5 mm
  cable without pinch; anodized aluminum or hard-anodized with a smooth groove to
  keep μ low.
- **Structural:** aluminum sheave for the ~1 kN transient; a steel bushing/insert
  at high-load hip/knee if wear shows.

### 3.2 Bearings — split the sizing by load type

The resultant radial load on a pulley bearing is the vector sum of the two cable
segments, up to **~2·T** for a 180° wrap → **~1.0 kN** at the knee land peak.

| Rating | Sized to | Value | Rationale |
|---|---|---|---|
| **Static C₀** | land transient resultant | **≥ ~1.5 kN** (SF 1.5 on ~1 kN) | rare peak; brinelling is the failure mode |
| **Dynamic C** | continuous trot resultant | **≥ ~0.3 kN** (2 × ~150 N) | fatigue life is set by continuous gait, not the rare land |

This split is the key to keeping bearings small: sizing dynamic (fatigue) rating
to the **land** peak would force needlessly large bearings, but the land is a
transient. Candidate: miniature deep-groove ball bearings (e.g. 3–5 mm bore, 623/
MR series at distal joints; a larger 6 mm-bore 626-class at hip/knee where C₀ is
easiest to hit). Exact PNs `[owed: BOM pass]`.

### 3.3 Routing paths (girdle motors → joints)

Centralized-motor / P1: all leg motors live in the shoulder & pelvic **girdles**.
Each joint tendon runs from its girdle spool, over redirect/idler pulleys, to the
joint sheave, then to its anchor. Antagonistic pairs (ADR-0002) for hip and knee;
ankle is single-tendon + return spring (Option B).

| Tendon | Path | Approx run length | Routing | Label |
|---|---|---|---|---|
| Hip (agonist/antag.) | girdle spool → hip sheave → anchor | ~0.10 m | **open pulleys** | `[assumed]` |
| Knee (agonist/antag.) | girdle spool → hip idler (via-point) → knee sheave → anchor | ~0.22 m | **open pulleys** | `[assumed]` |
| Ankle (single + spring) | girdle spool → hip idler → knee idler → ankle sheave → anchor | ~0.30 m | open pulleys; short **sheath** only where it crosses the shank | `[assumed]` |

**Open pulleys vs. sheath (Bowden):** prefer **open low-friction idler pulleys**
throughout — sheaths add large distributed capstan friction (μ high over long
wrap). Use a short PTFE-lined sheath **only** where a cable must bend around
structure with no room for a pulley (e.g. crossing the moving shank). This mirrors
Kengoro's low-friction guide approach `[sourced: lit Q6]`.

### 3.4 Wrap angles (for capstan-friction modeling)

Capstan: `T_out = T_in · e^(μ·θ)`, θ = total wrap in radians. Kinematics needs
per-pulley wrap to model the tension the **motor** must supply vs. what the
**joint** receives.

| Pulley station | Wrap angle θ | Note | Label |
|---|---|---|---|
| Motor spool | ~180° (π) | 0.5–1 wrap on the spool | `[assumed]` |
| Hip sheave (hip tendon) | up to ~240° | = hip ROM (±120°) + anchor seat | `[assumed]` |
| Hip idler (knee/ankle pass-through) | ~30–45° | redirect only | `[assumed]` |
| Stifle sheave (knee tendon) | up to ~150° | stifle ROM (−150…0°) | `[assumed]` |
| Knee idler (ankle pass-through) | ~30–45° | redirect only | `[assumed]` |
| Ankle sheave | ~60–90° | `[assumed]` |

**Friction coefficient:** UHMWPE over anodized-aluminum pulley, lightly
lubricated: **μ ≈ 0.08–0.12**; over a PTFE-lined sheath μ ≈ 0.05–0.10 but with
much larger effective wrap. Placeholder `friction_coeff = 0.10` `[assumed]`.

**Capstan penalty example (ankle tendon, the worst path):** summed wrap ≈ 180 +
45 + 45 + 90 = 360° = 6.28 rad; at μ = 0.10, `e^(0.10×6.28) = 1.87` → the motor
must pull **~87 % more** tension than the joint receives, and on release the
joint sees correspondingly less. This quantifies why routing must **minimize
wrap and use open pulleys**: friction directly inflates the motor-side tension
(and the cable break-strength margin) it takes to deliver a given joint torque.

---

### 3.5 Link (bone) structural sizing — closes review F3

The specs sized the cable, pulleys and bearings but never **the links
themselves**. A link must transmit its joint's torque, so it carries a bending
moment equal to that torque.

**Key identity — the moment arm does NOT affect this.** It is tempting to say a
big pulley "levers" the bone harder, but `M = T·r` and `T = τ/r`, so
`M = (τ/r)·r = τ` **identically**. The link's bending moment is just the joint
torque, set by the ground reaction and leg geometry. Growing `r` cuts *cable
tension* and leaves *link stress untouched* — the two design levers are
independent, and there is no trade between them. (An earlier revision of the
design review claimed such a trade; that was an error.)

Bending stress `σ = M / Z`, tube `Z = I/r_o`. Allowable **σ ≈ 400 MPa** for a
CF tube in bending `[assumed — conservative; confirm against a real tube
datasheet, and check local wall crushing/buckling, not just material strength]`.

At the original uniform **8 × 1 mm** tube the femur reached **364 MPa → SF ≈ 1.1**
— effectively no margin on the landing transient. Adopt a **graded set, thickest
proximally**, which equalises the safety factor and keeps added mass off the
distal links (P1):

| Link | Torque | **Section** | Z (mm³) | σ (MPa) | **SF** | mass |
|---|---|---|---|---|---|---|
| Femur | 12.36 N·m | **Ø12 × 1.0** | 87.8 | 141 | **2.84** | 4.8 g |
| Tibia | 7.49 N·m | **Ø10 × 1.0** | 58.0 | 129 | **3.10** | 4.2 g |
| Metatarsus | 4.79 N·m | **Ø8 × 1.0** | 34.4 | 139 | **2.87** | 2.4 g |
| Paw (passive) | — | Ø8 × 1.0 | — | — | — | 0.9 g |

**Cost: +2.7 g per leg** (9.5 → 12.2 g of bone) and **+0.9 % leg swing inertia** —
negligible, because the added material sits proximally where the inertia lever is
short. Buckling is not the failure mode (tibia Euler `P_cr ≈ 10.5 kN`, SF ≈ 34).

> Remaining check `[owed]`: joints see combined bending **and torsion**, and the
> tube must also survive the local contact/crushing load where a pulley or clevis
> clamps it. Both need a real section and joint detail (F5).

## 4. Static-hold offload

The [sensorless-FOC note](../docs/notes/sensorless-foc-stance-hold.md) establishes
that a quasi-static high-torque hold is a **thermally-limited DC operating point**
(I²R in one/two phases, no rotation to share heat; continuous holding torque must
be derated well below peak). Holding stance electrically also keeps the creeping
cable (§2) under sustained tension. Both problems are removed by a mechanical
hold.

**Recommendation: a girdle-mounted, power-off (spring-engaged, electrically
released) friction brake on each leg motor shaft.**

- **Sizing:** holds spool torque = `T_hold × r_spool`. Quiet-stance hold tension
  is 50–160 N → `0.16 × 0.008 ≈ 1.3 N·m` at the spool worst case; a small PM
  brake covers it. (It need not hold the *land* peak — that is dynamic, when the
  brake is released and the motor is active.)
- **Location:** at the girdle motor, not the joint — keeps mass centralized (P1),
  off the distal limb.
- **Fail-safe:** spring-engaged means it holds stance **on power loss** — a
  perched cat does not collapse if power drops.

**Trade-off vs. backdrivability (the crux of ADR-0003):**

| Option | Backdrivable? | Static hold cost | Verdict |
|---|---|---|---|
| Electrical DC hold (no brake) | yes | high (thermal derate, cable creep) | baseline, thermally limited |
| **Power-off friction brake (recommended)** | **yes when released** | ~zero when engaged | engage only for true quasi-static postures |
| Non-backdrivable reduction (worm/lead-screw) | **no** | ~zero | **rejected** — kills IMF/agility, violates the ADR-0003 backdrivability intent |

The brake is strictly a **stance-only** device: **engaged** for standing/perching
(offloads thermal + creep), **released** for all locomotion and landing so the
leg keeps the full backdrivability and shock compliance ADR-0003 chose
tendon-drive to preserve. A non-backdrivable reduction would give the same hold
"for free" but permanently, defeating the purpose — hence rejected.

---

## 5. Parameter table → `LegParams` / `TendonParams`

For tomcat-kinematics / the lead to fold in. Field names match existing
dataclasses; proposed **new** fields are flagged.

### 5.1 `TendonParams` (existing fields)

| Field | Current | **Proposed** | Unit | Source label |
|---|---|---|---|---|
| `joint_moment_arm` | `(0.015, 0.012, 0.010)` | **`(0.028, 0.025, 0.014)`** | m | `[assumed]` — largest packageable; halves land peak (§1) |
| `motor_spool_radius` | `0.008` | `0.008` (unchanged) | m | `[assumed]` — travel/sweep check passes (§1.4) |
| `pretension` | `5.0` | `5.0` (keep as floor) | N | `[sourced: M1]` — AIC `T_bias` (~19.6 N) applied at runtime, not baked in |
| `spring_stiffness` | `(0.5,0.5,0.3)` | ankle only, `[owed]` | N·m/rad | ankle = Option-B return spring (ADR-0002); hip/knee antagonistic |
| `spring_rest_angle` | `(0.0,0.4,0.0)` | keep | rad | `[assumed]` |

### 5.2 Proposed **new** `TendonParams` fields

| Field (proposed) | Value | Unit | Source label |
|---|---|---|---|
| `cable_stiffness` | `3.5e5` (per-tendon; compute from run length) | N/m | `[assumed]` — `EA/L`, settled E≈60 GPa, 1.5 mm, ~0.30 m (§2) |
| `cable_diameter` | `0.0015` | m | `[assumed]` (§2) |
| `cable_break_strength` | `2200` | N | `[sourced]` 1.5 mm UHMWPE; SF≈4.3 on 525 N |
| `wrap_angle` | see §3.4 per station | rad | `[assumed]` — for capstan `T_out=T_in·e^(μθ)` |
| `friction_coeff` | `0.10` | — | `[assumed]` — UHMWPE / anodized-Al, open pulley (§3.4) |

### 5.3 `LegParams` (unchanged — geometry owned by kinematics)

| Field | Value | Unit | Note |
|---|---|---|---|
| `l1, l2, l3` | `0.120, 0.120, 0.050` | m | unchanged; moment arms above assume this scale |
| `q_min / q_max` | post-M4 negative fold: hip ±120°, stifle −150…0°, hock −30…+150° | rad | hip ROM now drives the 117 mm worst-case travel (§1.4) |

### 5.4 Derived design loads (for reference, not a param)

| Quantity | Value | Note |
|---|---|---|
| Peak cable tension (knee land + `T_bias`) | **~525 N** | structural design load per tendon |
| Component design load | **~1 kN** | margin over peak; also ≈ bearing resultant (2·T) |
| Continuous (trot) tension | 55–156 N | sets bearing fatigue (dynamic C) |

---

## Handoffs

- **→ tomcat-kinematics:** §5 tables. Headline changes: `joint_moment_arm`
  `(0.015,0.012,0.010) → (0.028,0.025,0.014)`; new `cable_stiffness`,
  `wrap_angle`, `friction_coeff`, `cable_*` fields for a tendon-stretch +
  capstan-friction term in the tendon map. `cable_stiffness` and `wrap_angle` are
  **per-tendon / per-path** — compute from each tendon's routed length and pulley
  list, not a single scalar.
- **→ tomcat-electronics:** each leg motor needs a **girdle-mounted power-off
  friction brake** (§4, ~1.3 N·m spool torque) plus its rotor encoder (ADR-0004);
  budget the brake driver channel + fail-safe wiring. Static hold is a
  thermally-derated continuous-torque point when the brake is *not* used.
- **→ tomcat-research:** the RoboCat 20–70 N band is a lighter, non-impact
  demonstrator; confirm whether a cat-scale ×2.5 single-leg land is the right
  worst case or over-conservative — it is the sole driver of the ~500 N residual.
- **→ lead:** the pure-tendon leg is buildable, but the ~500 N land transient is
  irreducible by geometry alone. If it proves to drive hardware mass/cost too
  hard, the trade-study hybrid (QDD-hip + tendon-knee ≥25 mm + spring-ankle)
  remains the documented fallback — a subsystem-level decision, not mine to make.
