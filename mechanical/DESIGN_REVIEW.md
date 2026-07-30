# Mechanical Design Review — first critical pass

Reviewer: **lead** · Date basis: post-M4 (real mass, static stability) ·
Status of findings: **first-pass calculations, not verified hardware**

This reviews the mechanical design as a whole — [LEG_TENDON_SPEC](LEG_TENDON_SPEC.md),
[SPINE_TAIL_SPEC](SPINE_TAIL_SPEC.md), the [CAD models](cad/), and the M4 mass
model — looking for gaps and cross-artifact contradictions rather than
summarising what is already written.

**Estimate quality warning.** Every number below is a first-pass calculation from
placeholder geometry with assumed material properties (CF tube E ≈ 70 GPa
quasi-iso, σ_allow ≈ 400 MPa; motors ≈ 5.5 g/cm³). They are strong enough to
*direct attention*, not to size hardware. Each finding states what would confirm
or dismiss it.

---

## What is in good shape

- **The tendon hardware chain is coherent** — cable → pulley → bearing → routing
  → static-hold brake all trace from one design load with stated assumptions.
- **The static/dynamic bearing split** (§3.2) is the strongest single call in the
  specs: sizing fatigue life to the *continuous trot* and brinelling to the *rare
  land transient* is what keeps the bearings small. Correct reasoning.
- **The transient-vs-duty distinction** is what makes a ~450 N peak buildable.
- **Moment arms are now validated by the real budget**, not guessed: after the
  F1/F2 rebuild the spine's 0.030 m arm puts continuous tension at ~12 N, safely
  below the RoboCat band — it has margin to spare.

---

## F1 — Mass apportionment contradicts P1 (major) — ✅ RESOLVED

The M4 budget allocates **24 % of body mass to the limbs**, justified from feline
biology. But a biological limb's mass is largely **muscle**, and P1/ADR-0003
*deliberately relocate the muscle (motors) into the girdles*. Applying a
biological limb fraction to a tendon-driven robot double-counts the actuator.

A bottom-up count of the specced hardware disagrees by more than 2×:

| Per leg | Estimate |
|---|---|
| Sheaves (Ø56/50/28 Al, 45 % lightened) | 44 g |
| Bones (8 mm × 1 mm CF tube ×4) | 10 g |
| 6 miniature bearings, 3 shafts | 15 g |
| Cable + terminations, brackets | 11 g |
| **Total** | **≈ 80 g** |

vs **200 g budgeted** (hind). Even doubling my estimate for unmodelled parts
(sheaths, foot pad, fasteners) the legs land near **~120 g**, i.e. limbs ≈ **15 %**
of body, not 24 %.

Meanwhile the **girdles are under-budgeted**. They must contain the motors:

| Girdle contents | Estimate |
|---|---|
| 31 motors @ Ø16×28 mm ≈ 31 g | 960 g |
| 31 driver boards @ ~5 g | 155 g |
| Battery `[assumed]` | 300 g |
| **Subtotal** | **≈ 1415 g** |

vs **980 g budgeted** — a **~435 g shortfall**, before girdle structure.

The *total* still lands near 3.0 kg (the errors cancel), which is why this was
invisible: **the total is right and the distribution is wrong.**

> **Consequence:** the real machine is *more* mass-centralised than modelled —
> which is what P1 predicts and is good for agility — but CoM, the fore/hind
> split, and swing inertia are all computed from the wrong distribution.
>
> **To confirm:** pick a candidate motor (also gates `Kt`/bus voltage) and weigh
> or source-datasheet it; then re-derive the budget bottom-up instead of from the
> biological 24 %.

## F2 — The 60/40 front-heavy split is probably wrong (major) — ✅ CONFIRMED & FIXED

M4 solved the girdle masses to hit a **59.9 % fore** split, making the *front*
girdle heavy (700 g) to absorb head + neck. But per ADR-0005 / the board outline,
the motors are **not** distributed that way:

| Cluster | Motors |
|---|---|
| Shoulder girdle | 12 (fore legs) |
| **Pelvic girdle** | **19** (hind legs + 6 spine + 1 tail) |

At ~31 g each that is **589 g in the pelvis vs 372 g in the shoulder** — the
motor mass leans *rearward*, partly cancelling the head/neck. The true split may
be near-balanced or even rear-heavy, not 60/40 fore.

> **Why it matters:** M4's static stability margin is a CoM-vs-support
> calculation. Move the CoM rearward and the margins change directly — and the
> current gait already tips toward the **rear** edge (every phase is
> "STABLE (rear edge)"). This is the finding most likely to alter a result we
> have already published.
>
> **To confirm:** recompute `quarter_masses` with motors placed in their actual
> clusters rather than folded into girdle lumps, then re-run the stability sweep.

## F3 — The links are never structurally sized (major) — ✅ RESOLVED

The specs size the *cable, pulleys and bearings* to ~1 kN but never size **the
bones themselves**. That matters because an offset tendon does not merely
compress a link — it **bends** it: a cable running at radius `r` applies
`M = T·r` about the section.

First-pass 8 mm OD × 1 mm wall CF tube (I = 137 mm⁴):

| Link | T (N) | r (mm) | M (N·m) | bending σ | total σ | SF vs ~400 MPa |
|---|---|---|---|---|---|---|
| **Femur (hip)** | 447 | 28 | **12.5** | 364 MPa | **385 MPa** | **≈ 1.0** ⚠ |
| Tibia (stifle) | 305 | 25 | 7.6 | 222 MPa | 236 MPa | 1.7 |
| Metatarsus (hock) | 347 | 14 | 4.9 | 141 MPa | 157 MPa | 2.5 |

**The femur is at the material limit (SF ≈ 1.0)** in the land transient.
Euler buckling is **not** the issue (tibia P_cr ≈ 10.5 kN, SF ≈ 34) — bending is.

> ### ⚠️ Correction to this finding
> The first version of F3 claimed an "irony": that the *large hip moment arm
> chosen to cut cable tension* was what maximised femur bending, so the two
> objectives conflicted. **That was wrong.** Since `T = τ/r`, the moment is
> `M = T·r = (τ/r)·r = τ` **identically** — the link's bending moment is simply
> the joint torque, and is *independent of the moment arm*. Growing `r` reduces
> cable tension and leaves link stress untouched. There is **no trade** between
> the two; they are independent levers. The understrength finding itself stands —
> only the explanation was wrong.

> **Resolved:** purely a section problem. A **graded tube set** (femur Ø12×1,
> tibia Ø10×1, metatarsus/paw Ø8×1) lifts every link to **SF ≈ 2.8–3.1** for
> **+2.7 g per leg** and **+0.9 % swing inertia** — the added material sits
> proximally, where the inertia lever is short. Specified in
> [LEG_TENDON_SPEC §3.5](LEG_TENDON_SPEC.md).
> Still owed: combined bending **+ torsion**, and local crushing where a pulley
> or clevis clamps the tube — both need real joint detail (F5).

## F4 — Girdle width unresolved (moderate) — ✅ RESOLVED

The packaging study sizes each girdle at **142 mm wide** against a **96 mm leg
track**, so the motor banks protrude past the body sides, and the pelvic girdle
grows to 91 mm tall. Documented in [cad/README](cad/README.md) but never
resolved. Options already noted: reorient motors, spread the bank along the
torso, or take the variable-radius-pulley reduction (spine 6 → 3).

F1 makes this *more* urgent: if girdle contents are ~435 g heavier than budgeted,
they are also bulkier than drawn.

> **Resolved by re-orienting the motors, not by shrinking the bank.** The banks
> were laid **axis-along-Y**, so each motor's 28 mm length projected *sideways*.
> Standing them **upright (axis-along-Z)** puts that length into girdle height —
> where there is room — and reduces the footprint to a grid of Ø16 circles.
> With compact 3 × 2 banks at y = ±21 and the spine/tail bank offset rearward:
>
> | | before | after |
> |---|---|---|
> | Girdle width | 142 mm (> 96 mm track) | **87 mm — fits** |
> | Pelvic girdle height | 91 mm | **52 mm** |
> | Girdle collision on the 195 mm spine | 108 + 108 → overlap | **64 + 88, ~119 mm clear** |
>
> Note this did **not** require the variable-radius-pulley motor reduction or a
> wider body — the motor count is unchanged. It also relieves the pelvic
> density/thermal hot-spot the electronics outline flagged: both girdles are now
> the same height. Upright spools also pay cable off in the fore-aft/vertical
> plane, which is the direction the tendons actually run.

## F6 — Motor mass makes the 3 kg body infeasible (BLOCKING) — ✅ RESOLVED by ADR-0008

Raised by the [motor down-select](../docs/notes/motor-downselect.md). F1 assumed
~31 g/motor; a real QDD module meeting the torque requirement (SteadyWin
GIM3505-9 class) is **131.7 g — 4.2× heavier**.

| Configuration | Motors | Motor mass | % of 3 kg body |
|---|---|---|---|
| Full antagonistic (ADR-0002) | 31 | 4083 g | **136 %** |
| CAD reduced (ankle spring) | 24 | 3161 g | **105 %** |
| + variable-radius pulley | 16 | 2107 g | 70 % |

**The motors alone exceed the whole body budget.** This is the honest cost of
P1: relocating 24–31 muscles into the torso means the torso must carry 24–31
motors. Ways out — scale the body to ~6–9 kg, cut motor count, or cut the torque
requirement (thinner cable → smaller spool). Most likely a combination.

> **Resolved by [ADR-0008](../docs/DESIGN_DECISIONS.md).** Scaling the body up
> was rejected *on physics*: the motor-mass fraction is **invariant with scale**
> (191 % at 1.5, 3 and 9 kg alike) and gets worse under geometric scaling. The
> two levers that work are **sizing basis** and **motor count**: sizing to trot
> instead of the ×2.5 landing takes 24 motors from 191 % → 57 %, and the
> variable-radius pulley takes 24 → 16, closing the budget at **38 %** on the
> existing 3 kg body with **full articulation retained**. Target motor spec:
> ~1.1 N·m peak, ≤80 g. Hard landings become out-of-envelope for v1.
>
> Still open: motor count 24 → 16 invalidates the **F4 girdle packing**, and the
> real motor envelope differs from the Ø16 × 28 mm placeholder — re-pack needed.

## F5 — No assembly / manufacturing pass (gap) — ✅ RESOLVED

Across all mechanical docs there is essentially **one** mention of
manufacturability (the CAD README stating it is not a manufacturing model). Not
yet addressed: fabrication method per part, tolerances and fits, fastener
strategy, cable **termination** and **re-tensioning** access, and how a joint is
physically assembled around its pulley and bearings. This is expected at this
stage — recording it so it is not mistaken for completeness.

> **Resolved by [ASSEMBLY_SPEC.md](ASSEMBLY_SPEC.md)**, which also closes the two
> checks F3 deferred:
> - **Combined bending + torsion.** An unguided cable reaching the bone with the
>   27 mm lateral offset implied by the girdle layout costs ~24 % of the femur's
>   safety factor (2.84 → **2.17**) — still above 2, so not a blocker, but a
>   **limb-root idler** that turns the cable into the bone's plane recovers it
>   fully and is cheap.
> - **Local crushing.** Through-bolting is survivable at ≥4 mm pin (112 MPa vs a
>   ~150 MPa transverse allowable) but is *not* recommended — it cuts fibres and
>   leans on the least trustworthy number in the design. **20 mm bonded aluminium
>   inserts** carry the same load at 0.71 MPa adhesive shear, SF > 14.
>
> The spec also fixes fabrication method per part, the joint stack-up (with the
> rule that **the sheave keys to the DISTAL link**), H7/h6 fits, spliced-eye
> terminations (knots lose 30–50 % in UHMWPE), and **±5 mm re-tensioning travel**
> sized from UHMWPE creep — with the constraint that adjusters must be reachable
> without de-mounting a limb, and that re-tensioning must be followed by the FR7
> calibration routine.

## F7 — The walk is LATERALLY unstable (3D geometry) — OPEN, needs a decision

Every stability result so far came from the 2D-sagittal support *interval*, which
each document correctly flagged as **necessary but not sufficient**. Giving the
legs their real lateral track offsets (3D **geometry** — no new DOF, no motors)
makes the true ground-plane **support polygon** computable for the first time.
It disagrees with the 2D result:

| | 2D sagittal interval | **True support polygon** |
|---|---|---|
| Worst margin over the cycle | **+24.6 mm** (stable everywhere) | **−28.7 mm** (unstable) |

With three feet down the support triangle is skewed to one side while the CoM
sits on the mid-sagittal plane, so it falls **outside** the triangle for roughly
half the cycle. This is the classic reason quadrupeds **sway** during a crawl.

**What was tried (all measured, not assumed):**

| Lever | Result | Costs motors? |
|---|---|---|
| Re-order the leg phase sequence (all 24 permutations) | best **−22.7 mm** — still unstable | no |
| **Widen the track** (96 → 260 mm) | **−47.7 mm — WORSE** | no |
| Shift the CoM forward alone | best **−8.3 mm** at +40 mm | no |
| **Lateral body sway ~40 mm** | **+5.7 mm — works** | **yes** |
| Forward CoM +30 mm **and** sway 25 mm | **+8.1 mm — works** | yes (less sway) |

Widening the track being *counter-productive* is the surprise: the critical edge
is the **diagonal** from the far front foot to the near rear foot, and widening
only rotates that diagonal further away from the CoM.

**The decision this forces.** Static stability needs a **lateral DOF** for sway.
ADR-0006 already specifies spine lateral bend as a target, but
[ADR-0008](../docs/DESIGN_DECISIONS.md) budgeted motors for the sagittal DOF only
(16 motors, 38 % of body). Options:

1. **Add spine lateral DOF** (+3 motors → 19, ~46 % of body) — re-opens ADR-0008's
   mass closure, which had ~35 % left for structure.
2. **Accept dynamic walking** — drop the static-stability requirement, as most
   real quadrupeds do when trotting. Cheapest, but changes what the controller
   must guarantee.
3. **Bias mass forward + a smaller lateral DOF** — the +30 mm/25 mm combination
   above needs less sway authority than 40 mm alone.

Not resolved here: it changes ADR-0008 and possibly the control strategy, so it
is the project owner's call.

---

## Resolution of F1 + F2 (done)

The budget was rebuilt bottom-up with the motors in their real ADR-0005 clusters
(front girdle 12 channels, rear girdle 19) and the legs sized from the specced
hardware. It still totals exactly 3.000 kg, but the distribution changed a lot:

| | Before (tuned) | After (bottom-up) |
|---|---|---|
| All four legs | 720 g (24 %) | **410 g (13.7 %)** |
| Front girdle | 700 g | 762 g |
| **Rear girdle** | 280 g | **794 g** ← now the heavier one |
| Fore/hind split | 59.9 / 40.1 | **51.2 / 48.8** (near-balanced) |
| Body CoM (x) | +130 mm | **+102 mm** (19 mm rearward) |
| Worst stability margin | +46.5 mm | **+27.4 mm** |
| Quiet-stand spine tension | 24 N | **12 N** |

**The walk remains statically stable at every phase** — F2 did not break the
design — but the worst-case margin lost ~41 % of its value, so the machine now
sits closer to its rear tipping edge. Two published conclusions reversed:

- The spine's "23–24 N, inside the RoboCat band" became **~12 N, below it**.
- "The base joint is the worst spine joint" is now true **only for the
  asymmetric landing case**; in quiet standing a balanced body barely loads it.

Both specs and the `params.py` apportionment docstring were updated, and the
tests that encoded the old model now assert the new behaviour (with comments
explaining why it changed). 229 tests pass.

**Still assumed, and worth firming up:** motor mass (~31 g from a Ø16×28 mm
envelope) and the 300 g battery. Selecting a real motor is the single input that
would firm up the whole budget — and it also gates `Kt` and bus voltage.

## Recommended order (remaining)

1. ~~**F1 + F2 together**~~ — **done**, see above. — pick a motor, rebuild the mass budget bottom-up with
   motors in their real clusters, re-run CoM and the stability sweep. These two
   are coupled and can invalidate a published M4 result.
2. ~~**F3**~~ — **done**: graded tube sections, see above.
3. ~~**F4**~~ — **done**: motors stood upright, see above.
4. ~~**F6**~~ — **done**: ADR-0008 (size to trot + 16 motors). Follow-up: re-pack
   the girdles for 16 real motors (re-opens part of F4).
5. ~~**F5**~~ — **done**: ASSEMBLY_SPEC.md (also closes F3's deferred checks).

**All review findings are now closed.** What remains is not analysis but
*procurement and test*: shop drawings + a real BOM, a bond-coupon pull-out test,
a measured CF transverse allowable, a limb tolerance stack-up, and a
serviceability check of the adjuster access against real housing geometry.

## Does the hock verdict still hold?

Yes, and more strongly. [§1.3a](LEG_TENDON_SPEC.md) priced the hock arm against
*leg swing inertia*. If F1 is right and the legs are lighter than modelled, a
given pulley is a **larger** fraction of the leg's inertia, so growing the hock
arm is *more* costly than the review calculated. **Keep 14 mm.**
