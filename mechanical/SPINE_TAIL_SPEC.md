# TomCat — Spine & Tail Geometry / Routing Spec (first pass)

Owner: **tomcat-mechanical** · Milestone: **M1** (task M1: spine geometry & routing)
Status: **first-pass / all values placeholder** unless labelled `[sourced]`.

This is the mechanical geometry & tendon-routing proposal for the articulated
spine ([ADR-0006](../docs/DESIGN_DECISIONS.md)) and the single-tendon tail
([ADR-0007](../docs/DESIGN_DECISIONS.md)). It presents numbers as tables for
**tomcat-kinematics** to fold into `kinematics/src/tomcat_kin/params.py`
(`SpineParams` + a proposed `TailParams`). It does **not** edit `params.py`,
`kinematics/`, or `docs/`.

Conventions match the kinematics model (`params.py`, `spine.py`): **SI units**;
sagittal plane; **rear/pelvic girdle at the origin, +x forward, +z up,
CCW-positive**. A positive dorsoventral joint angle arches the back upward
("Halloween cat"). The spine chain is indexed rear→front: vertebra 0 = rear
(pelvic) girdle, vertebra N = front (shoulder) girdle.

Label legend: `[sourced]` traceable to lit-review/ADR; `[assumed]` first-pass
engineering guess; `[owed:R1]` must come from tomcat-research's axial→rotational
stiffness conversion.

---

## 1. Spine — 3-segment serial chain

### 1.1 Topology & girdle placement

- **3 segments, revolute chain** `[sourced: ADR-0006, lit Q2 — 2–3 segments]`.
  Full-3D target is 3 DOF/segment (dorsoventral pitch, lateral yaw, axial roll);
  **M1 exercises only the dorsoventral DOF** and parameterizes the other two.
- **Rear/pelvic girdle** = vertebra 0 at world origin `(x,z)=(0,0)`. Houses the
  spine flexor/extensor motor bank **and** the tail actuators (centralized-motor
  principle P1).
- **Front/shoulder girdle** = vertebra 3 at `x = +Σ segment_lengths ≈ +0.195 m`
  (straight spine). Houses forelimb motors; also serves as the distal anchor
  point for the through-running spine tendons.
- Girdle frames are exactly the `SpineModel.girdle_pose()` outputs
  (`REAR = poses[0]`, `FRONT = poses[-1]`), so legs hang off these moving frames
  per interface I4. Leg hip offsets stay at the `(0,0)` placeholder for now.

### 1.2 Segment lengths (rear → front)

Cat torso (T1→sacrum) scale for a ~3 kg body is ~0.20 m. Lumbar (rear) segments
are longer and are the dorsoventral-mobile region, so lengths taper front-ward.

| Segment | Region | Length (m) | Label |
|---|---|---|---|
| 1 (rear, vertebra 0→1) | lumbar | 0.075 | `[assumed]` |
| 2 (mid, vertebra 1→2)  | thoraco-lumbar | 0.065 | `[assumed]` |
| 3 (front, vertebra 2→3)| thoracic | 0.055 | `[assumed]` |
| **Total** | | **0.195** | |

(Compatible with the current `segment_lengths=(0.060,0.060,0.060)` seed; this
proposal makes it a tapered 0.195 m instead of a uniform 0.180 m.)

### 1.3 Per-axis range of motion

ROM is ordered to respect the cat directional-**compliance rank** (most→least
compliant: **axial-rotation > extension(dorsoventral) > lateral-bending**)
`[sourced: lit Q3]`: the more-compliant axis is given the larger sweep. Values
are **per segment**; whole-spine range ≈ 3× (three segments in series).

| Axis | In M1? | Per-seg limit | Whole-spine (~3×) | Rank slot | Label |
|---|---|---|---|---|---|
| **Dorsoventral** (pitch, sagittal) | **yes** | ±0.436 rad (±25°) | ≈ ±75° | middle (extension) | `[assumed]` |
| Axial rotation (roll, twist) | no (placeholder) | ±0.524 rad (±30°) | ≈ ±90° | most compliant → widest | `[assumed]` |
| Lateral bending (yaw) | no (placeholder) | ±0.262 rad (±15°) | ≈ ±45° | stiffest → narrowest | `[assumed]` |

Notes:
- The dorsoventral ±25°/seg matches the current `SpineParams` seed and is the
  only axis `spine.py` currently moves.
- The ~±90° whole-spine axial range is deliberately generous because it is the
  spine's contribution to the **righting-reflex twist** (ADR-0007) that
  complements the tail.
- The **magnitudes carry no stiffness meaning** — the compliance *rank* must be
  carried by per-axis rotational stiffness from `[owed:R1]`, not by ROM. ROM is
  merely ordered consistently with the rank as a first pass.

### 1.4 Tendon routing along the column

Tendons run as long cables from motor spools in the **pelvic girdle**, forward
along the column, over a **dorsal and a ventral pulley at each vertebra**
(setting the moment arm), to their anchor. Cable only pulls, so each
dorsoventral DOF is an **antagonistic pair**: a **dorsal extensor** (arches up,
+q) and a **ventral flexor** (curls down, −q) — ADR-0002 baseline for a
stiffness-tunable joint.

Three routing options, in increasing motor cost / control authority:

- **Option A — coupled through-cables (minimum motors).** One dorsal + one
  ventral cable span **all 3 segments**, anchored at the front girdle, sharing
  one tension across every vertebral pulley. Produces a single coupled bow
  shape. 2 tendons total. Cheapest; least shape authority.
- **Option B — per-segment independent (RECOMMENDED M1 baseline).** Nested
  cables: dorsal+ventral pair anchoring at vertebra 1, another at vertebra 2,
  another at the front girdle (vertebra 3). **6 tendons**, giving independent
  per-segment sagittal control that the `spine.py` per-segment `q` vector
  assumes. This is the routing the M1 tendon map should model.
- **Option C — full 3D.** Option B plus left/right lateral pairs and an axial
  twist pair per segment. Up to 18 tendons. Deferred with the 3D model.

**Cable-path lengths** (motor spool in pelvic girdle → anchor, straight spine,
+ ~0.05 m routing slack to the spool):

| Tendon | Anchor | Run length (m) | Label |
|---|---|---|---|
| segment-1 pair | vertebra 1 | ~0.075 + 0.05 ≈ 0.125 | `[assumed]` |
| segment-2 pair | vertebra 2 | ~0.140 + 0.05 ≈ 0.19 | `[assumed]` |
| segment-3 pair | vertebra 3 (front girdle) | ~0.195 + 0.05 ≈ 0.245 | `[assumed]` |

Because a single cable's tension is common to every pulley it crosses, **peak
tension is set by the most-loaded joint it spans** — this drives the moment-arm
sizing below.

### 1.5 Moment arms (pulley radii) and the tension-amplification justification

Joint torque `τ = T · r`; for a required torque, cable tension `T = τ / r`, so a
**small moment arm hugely amplifies required tension**. The RoboCat sanity band
is **~20–70 N** (`[sourced: lit Q6]`). We therefore want `r` as **large** as the
vertebral packaging allows, bounded by belly/back clearance and cable travel.

> ✅ **Superseded by the real budget.** This section previously used an *assumed*
> `τ ≈ 2.5 N·m`. The whole-body budget now computes spine torques from the
> **real distributed mass**, rebuilt bottom-up per review F1/F2 (3.00 kg,
> **51.2 % fore** — near-balanced, not the tuned 60/40), so the numbers below are
> measured from the model, not guessed.

**Real spine loads at the adopted `r = 0.030 m`** `[sourced: whole_body_budget]`:

| Case | seg0 (base) τ | worst-joint τ | Cable tension | Verdict |
|---|---|---|---|---|
| Quiet stand | 0.07 N·m | 0.22 N·m (seg2) | **12.2 N** | ✅ low — *below* the band |
| Arch (+20°/seg) | 0.16 N·m | 0.16 N·m (seg0) | ~13 N | ✅ low |
| Land (1 front leg, ×2.5) | **11.41 N·m** | 11.41 N·m (seg0) | **~385 N** | structural transient |

**Findings (updated after review F1/F2 rebalanced the mass model):**
1. **The 0.030 m arm is more than sufficient.** Continuous spine tension is only
   **~12 N** — comfortably *below* the RoboCat 20–70 N band. (An earlier revision
   of this section reported 23–24 N "inside the band"; that was computed with a
   tuned 60/40 front-heavy mass model. Rebalancing the body roughly halved the
   continuous spine load.) No growth in `r` is needed; if anything there is now
   headroom to trade arm size back for packaging.
2. **The base joint is the worst only under asymmetric load.** With a
   near-balanced body (51/49) quiet standing barely loads the base joint at all —
   the small residual peaks at the *front* joint. But a **single-front-leg
   landing** hangs the whole body off the base joint (11.4 N·m, >3× the front
   joint). **Size seg0 hardware for the landing case**, not for standing.

The ~385 N land figure is a rare impact transient (structural sizing), not a
duty — the same distinction drawn in [LEG_TENDON_SPEC.md](LEG_TENDON_SPEC.md) §0.

| Tendon (per segment) | Moment arm r (m) | Bounded by | Label |
|---|---|---|---|
| Dorsal extensor (arch, +q) | **0.030** | spinous-process height + pulley standoff | `[assumed]` |
| Ventral flexor (curl, −q) | **0.018** | belly clearance (smaller → higher tension, but gravity assists flexion so torque demand is lower) | `[assumed]` |
| Lateral (per side) | **0.020** | **milled lateral pulley post** (⚠️ NOT the bare 15 mm transverse-process width — see below) | `[sourced: ADR-0009 f/u]` |
| Axial twist (3D) | 0.020 | vertebral-body periphery | `[assumed]` |

> **Why the lateral arm is a designed post, not a bone feature (ADR-0009 f/u).**
> The dorsoventral tendon has the tall **spinous** process to wrap (30 mm); the
> lateral tendon has only the much shorter **transverse** process. This spec
> originally took that at face value, 15 mm. Sizing the lateral drive against the
> M5 sway-reversal inertia (`WholeBody.lateral_spine_loads`) shows that does not
> work: at 15 mm the **base** lateral joint needs **1.13 N·m** of motor torque,
> *over* the selected motor's ~1.10 N·m peak. At **20 mm** it needs 0.88 N·m
> (0.80× the sizing point) and the ADR-0009 motors fit.
>
> 20 mm is realised by a **milled lateral pulley post** on each actuated vertebra
> — the same approach [ASSEMBLY_SPEC §1](ASSEMBLY_SPEC.md) already takes for the
> dorsoventral arm, where the milled spinous post *is* the moment arm. ±20 mm
> sits well inside the ±34 mm thoracic rib cavity.
>
> Per-joint lateral loads at the shipped gait's 6.87 m/s² crossover:
>
> | joint | joint torque | cable tension | motor torque |
> |---|---|---|---|
> | **0 (base)** | **2.21 N·m** | **110 N** | **0.88 N·m** (0.80×) |
> | 1 | 1.11 N·m | 55 N | 0.44 N·m |
> | 2 | 0.40 N·m | 20 N | 0.16 N·m |
>
> Note these are **inertial**: the lateral axis is vertical, so gravity produces
> no moment about it and holding a sway is nearly free.

For the single-value `SpineParams.joint_moment_arm` field (which today models one
arm per segment) use the **dorsal extensor 0.030 m** as the representative
value, i.e. `joint_moment_arm=(0.030, 0.030, 0.030)`.

**Cable-travel check** at r=0.030, ±25° (0.436 rad): travel = 0.030×0.436 ≈
**13 mm** each way. With `motor_spool_radius=0.008`, motor sweep = 13/8 ≈
1.6 rad ≈ **94°** — modest, fine for a servo/BLDC spool.

### 1.6 Compliance / return elements

Per ADR-0002, spine joints are **antagonistic** (stiffness must be tunable to
gait speed, lit Q2b) — no passive return spring on the spine baseline. The
`spring_stiffness` / `spring_rest_angle` fields therefore stay as the
spring-return-mode fallback only. **Do not** populate `spring_stiffness` from the
53.62 N/mm axial value — that is `[owed:R1]` (axial N/mm + segment lever arms →
per-joint N·m/rad, respecting the axial>extension>lateral rank). Elastic-cable
back storage for landing energy absorption (G3) is a routing property to revisit
once R1 lands.

---

## 2. Tail — single-tendon (concept level)

Per **ADR-0007 (revised)** the tail needs **no precision** — it is a single cable
that **tensions up (curls/raises the tail) and loosens (relaxes)**. Mid-air
righting authority lives in the **spine axial-twist + legs**; the tail is only a
**coarse inertial assist**, not a controlled reorientation instrument. Concept
only; no geometry committed.

- **Actuation: 1 tendon + passive return** `[sourced: ADR-0007]` — a dorsal cable
  from a single girdle motor curls the passive multi-segment tail when tensioned;
  a light spring (or gravity) relaxes it when loosened. **No telescoping, no
  antagonist, no accuracy budget, no controlled DOF.**
- **Mounting:** at the **pelvic girdle** (vertebra 0), base at `x=0`, tail
  extending rearward (−x); the single motor lives in the pelvic girdle (P1).
- **Structure:** 3–4 short passive links so it curls smoothly; raised length
  ~0.25–0.35 m `[assumed]`, mass ~0.15–0.20 kg (~5–7% body) `[assumed]`.
- **Role check:** because the tail is coarse (essentially pull/release), it
  cannot perform the *controlled* flight-phase reorientation the literature
  attributes to a precise inertial tail (lit Q4). It contributes only a gross
  inertial bias; the righting maneuver is planned on the **spine + legs**
  (rotary-only 180°, lit Q4). This is a deliberate simplification for buildability
  and P1 purity, accepting lower righting authority.

---

## 3. Parameter table (maps onto `SpineParams` + proposed `TailParams`)

For tomcat-kinematics to adopt into `params.py`. Field names match the existing
`SpineParams` dataclass.

### 3.1 `SpineParams` (existing fields)

| Field | Proposed value | Unit | Source label |
|---|---|---|---|
| `n_segments` | `3` | — | `[sourced: ADR-0006]` |
| `segment_lengths` | `(0.075, 0.065, 0.055)` | m | `[assumed]` |
| `q_min` (dorsoventral) | `(-0.436, -0.436, -0.436)` | rad | `[assumed]` |
| `q_max` (dorsoventral) | `(0.436, 0.436, 0.436)` | rad | `[assumed]` |
| `joint_moment_arm` | `(0.030, 0.030, 0.030)` | m | `[assumed]` — dorsal extensor arm; raised from 0.020 seed for tension budget (§1.5) |
| `motor_spool_radius` | `0.008` | m | `[assumed]` |
| `pretension` | `20.0` (recommend; band 20–70) | N | `[sourced: lit Q6]` — recommend raising from the 5 N comparator toward the band |
| `spring_stiffness` | *defer* | N·m/rad | `[owed:R1]` — do NOT set from 53.62 N/mm |
| `spring_rest_angle` | `(0.0, 0.0, 0.0)` | rad | `[assumed]` |

### 3.2 Forward-looking per-axis ROM (for the 3D extension of `SpineParams`)

When `SpineParams` grows per-axis limits, seed them ordered by compliance rank:

| Axis field (proposed) | Per-seg `q_min` | Per-seg `q_max` | Unit | Source label |
|---|---|---|---|---|
| dorsoventral (M1) | −0.436 | +0.436 | rad | `[assumed]` |
| axial (roll) | −0.524 | +0.524 | rad | `[assumed]` (widest — most compliant) |
| lateral (yaw) | −0.262 | +0.262 | rad | `[assumed]` (narrowest — stiffest) |

Per-axis `joint_moment_arm` when 3D lands: dorsal 0.030 / ventral 0.018 /
lateral **0.020** / axial 0.020 m (§1.5).

### 3.3 Proposed new `TailParams` dataclass (simplified — ADR-0007)

| Field (proposed) | Proposed value | Unit | Source label |
|---|---|---|---|
| `mode` | `single_tendon` (tension/loosen) | — | `[sourced: ADR-0007]` |
| `n_motors` | `1` | — | `[sourced: ADR-0007]` |
| `n_links` (passive) | `3` | — | `[assumed]` |
| `length` (raised) | `0.30` | m | `[assumed]` |
| `mass` | `0.15–0.20` | kg | `[assumed]` (~5–7% body) |
| `mount_offset` (on pelvic girdle) | `(0.0, 0.0)` | m | `[assumed]` |
| `joint_moment_arm` | `0.015` | m | `[assumed]` |
| `motor_spool_radius` | `0.008` | m | `[assumed]` |
| `return` | passive spring / gravity | — | `[sourced: ADR-0007]` |

---

## 4. BOM / motor-count implications

Motor count follows directly from the routing choice and the ADR-0002
antagonistic factor (~2 channels per stiffness-tunable DOF; ~1 for
spring-return). The RoboCat **variable-radius pulley** trick (`[sourced: lit Q6]`)
lets **one motor drive an antagonistic 2-cable joint**, turning an *n*-DOF joint
from *2n* motors into *n* — worth adopting on the spine to cut the motor bank.

### 4.1 Spine

| Scope | Routing | Tendons | Motors (naïve 2/DOF) | Motors (variable-radius pulley, 1/DOF) |
|---|---|---|---|---|
| **M1 (dorsoventral only, 3 DOF)** | Option B (per-segment) | 6 | 6 | **3** |
| M1 minimal | Option A (coupled through-cable) | 2 | 2 | **1** |
| Full 3D (9 DOF) | Option C | up to 18 | 18 | **9** |

**Recommendation:** build M1 as Option B (6 tendons). Evaluate the
variable-radius pulley to halve the spine motor bank to **3 motors** before
committing — this is the single biggest lever on spine motor/driver-channel
count and directly feeds electronics task E1 and ADR-0002.

### 4.2 Tail

**1 motor.** A single dorsal tendon curls the passive tail (tension → curl,
loosen → relax under a passive return, ADR-0007). No antagonist, no telescope
actuator. Budget **1 tail motor** in the pelvic girdle.

### 4.3 BOM deltas this spec introduces

- Per-vertebra **dorsal + ventral pulleys** (moment-arm-setting), ×3 vertebrae
  = 6 pulley stations for M1 sagittal; low-friction routing (sheaths / single
  idler pulleys, cf. Kengoro maze-slot to cut friction) from girdle to each
  station.
- Cable-anchor hardware at vertebrae 1, 2, 3 (Option B).
- Optional **variable-radius pulleys** at the motor spools (RoboCat economy).
- Tail: passive multi-link tube, one dorsal tendon + spool, light return spring.
- Motors/drivers: **3–6 spine + 1 tail** added to the leg count, gating the
  electronics driver-channel count (E1) and the ADR-0002 antagonistic factor.

---

## Handoffs

- **→ tomcat-kinematics:** §3 tables (segment lengths, moment arms, ROM,
  pretension) for `params.py`; note the moment-arm increase 0.020→0.030 and the
  pretension 5→20 N recommendation; `spring_stiffness` remains `[owed:R1]`.
- **→ tomcat-electronics:** §4 motor counts (3–6 spine + 1 tail; variable-radius
  pulley halves the spine bank) for the driver-channel note (E1).
- **→ tomcat-research:** confirm the axial→rotational stiffness conversion
  (`[owed:R1]`) and validate the 2.5 N·m design-point torque assumption used in
  §1.5 against a real forequarter-mass estimate.
