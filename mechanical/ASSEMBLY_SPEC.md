# TomCat — Assembly & Manufacturing Spec (first pass) — closes review F5

Owner: **tomcat-mechanical** · Closes [DESIGN_REVIEW](DESIGN_REVIEW.md) **F5**,
and the two checks **F3** deferred (combined bending+torsion, local crushing).

Every prior mechanical document specified *what the parts must withstand*. None
said **how a part is made, how a joint goes together, or how a cable is
terminated and re-tensioned**. This is that document.

Label legend: `[sourced]` traceable to another spec/ADR · `[assumed]` first-pass
engineering guess · `[owed]` needs a vendor part or a test.

---

## 0. Closing the two F3 debts

### 0.1 Combined bending + torsion

A limb bone carries bending `M = τ_joint` (LEG_TENDON_SPEC §3.5). It *also*
carries **torsion** if the tendon reaches the bone with a lateral offset `e` —
which it does, because the motor spools sit inboard (y ≈ 21 mm) while the limb
plane is outboard (y ≈ 48 mm). Thin tube: `J = 2I`, so `τ_shear = T·e / 2Z`,
combined by von Mises `σ_vm = √(σ_b² + 3τ²)`.

Femur, Ø12 × 1 CF, hip land case (M = 12.36 N·m, T = 447 N):

| lateral offset `e` | torsion | σ_vm | **SF** |
|---|---|---|---|
| 0 mm (cable in the bone's plane) | 0 | 141 MPa | **2.84** |
| 10 mm | 4.5 N·m | 147 MPa | 2.71 |
| **27 mm (cable run straight from the girdle)** | 12.1 N·m | 184 MPa | **2.17** |

**Finding:** an unguided cable costs ~24 % of the femur's safety factor, but even
the worst case stays above SF 2. **Design rule: fit an idler at the limb root
that turns the cable into the bone's sagittal plane** — it recovers the full
SF 2.84 and is cheap. If omitted, SF 2.17 is still acceptable, so this is an
optimisation, not a blocker.

### 0.2 Local crushing at a pulley / clevis clamp

Pin bearing on a thin CF wall is `σ = F / (d·t)`, against a transverse allowable
of ~150 MPa `[assumed — CF transverse strength is highly layup-dependent]`:

| pin Ø | bearing stress | verdict |
|---|---|---|
| 3 mm | 149 MPa | marginal |
| **4 mm** | **112 MPa** | OK |
| 6 mm | 74 MPa | comfortable |

**Through-bolting is survivable at ≥4 mm but is NOT the recommended joint.**
Drilling a pultruded tube cuts load-bearing fibres, and the transverse allowable
is the least trustworthy number in this document. Use **bonded aluminium end
inserts** instead: they transfer load into the tube wall as *adhesive shear over
a long area* rather than point bearing.

| insert engagement | adhesive shear at 447 N | vs ~10–20 MPa structural epoxy |
|---|---|---|
| 15 mm | 0.95 MPa | SF > 10 |
| **20 mm** | **0.71 MPa** | SF > 14 |

**Rule: 20 mm bonded insert engagement**, ≥10× margin, and no drilled laminate.

---

## 1. Fabrication method per part

| Part | Method | Material | Note |
|---|---|---|---|
| Limb bones (femur/tibia/metatarsus) | **cut to length from stock tube** | pultruded CF, Ø12/Ø10/Ø8 × 1 mm | graded sections per LEG_TENDON_SPEC §3.5 |
| Bone end fittings | **turned**, bonded into the tube | 6061-T6 | 20 mm engagement (§0.2) |
| Joint clevises / forks | **CNC-milled** | 6061-T6 | carries the bearing bores |
| Joint sheaves (Ø56/50/28) | **turned**, hard-anodised groove | 6061-T6 | groove r ≈ 0.85 mm `[sourced: §2]` |
| Vertebrae, girdle shells, scapula/pelvis blades | **3D printed** (SLS nylon or MJF) | PA12 | complex, low-stress, non-critical form |
| Spine spinous-process pulley posts | **CNC-milled** | 6061-T6 | this *is* the 30 mm dorsoventral tendon moment arm |
| Spine **lateral** pulley posts (±20 mm, per actuated vertebra) | **CNC-milled** | 6061-T6 | this *is* the 20 mm lateral moment arm (ADR-0009 f/u). Shortening it to the bare transverse process puts the base lateral joint **over motor peak** |
| Motor / driver housings | **3D printed** | PA12 | cover plates for access, §4 |
| Paw pads | **cast / moulded** | TPU ~80A | compliant contact + tactile sensor pocket. ⚠️ **μ ≥ 0.70 is now a stability requirement** (NFR2g): below it the ADR-0009 sway crossover slides. `[owed: measure on the intended floor]` |

Rationale: metal only where load or precision demands it (bearing bores, sheave
grooves, bonded joints); printed everywhere else to keep mass and cost down.
**Nothing here is drawn to a machinable level yet** — these are methods, not
shop drawings `[owed]`.

## 2. Joint assembly stack-up

Every actuated joint is a hinge about the lateral axis. Assembly order, outside
in, per side:

```
   clevis arm (distal link)                    clevis arm (distal link)
   ├─ retaining circlip                        ├─ retaining circlip
   ├─ deep-groove ball bearing  ── shaft ──    ├─ deep-groove ball bearing
   │    (bore H7, press)          (h6)         │
   └───────────  SHEAVE, keyed/clamped to the DISTAL link  ──────────┘
                          ▲
              proximal link tongue rides between the arms
```

Critical rule: **the sheave is fixed to the DISTAL link, not the shaft.** Cable
tension on the sheave must rotate the distal link about the joint — if the sheave
were fixed to the shaft or the proximal link, the tendon would do no work.

| Interface | Fit | Reason |
|---|---|---|
| Bearing OD → clevis bore | **H7** | locating, light press |
| Shaft → bearing bore | **h6** slip / k6 light press | serviceable |
| Sheave hub → distal link | keyed or split-clamp | must transmit full joint torque |
| Insert OD → tube ID | **0.05–0.15 mm bond gap** | epoxy needs a controlled glue line |

Bearings are sized in LEG_TENDON_SPEC §3.2 (static C₀ ≥ 1.5 kN, dynamic C ≥ 0.3 kN)
`[sourced]`. Exact part numbers `[owed: BOM pass]`.

## 3. Cable termination

UHMWPE (Dyneema) is slippery and **loses 30–50 % of its strength in a knot** —
knots are not acceptable at a ~465 N design load.

| Location | Method | Why |
|---|---|---|
| **Joint end** | **spliced eye (Brummel) over a thimble**, onto the anchor pin | a splice retains ~90–100 % of line strength; a knot would not |
| **Spool end** | 3–4 dead turns + **clamp screw** | the capstan effect means the anchor itself sees only a few % of tension |
| Anti-creep prep | **heat-set / pre-tension** the cable before final length setting | takes out constructional stretch so §4 only has to absorb true creep |

## 4. Re-tensioning — access and travel

UHMWPE **creep** is the reason this section exists (LEG_TENDON_SPEC §2 flags it
as the cable's main drawback). Tendons will need periodic take-up.

- **Mechanism:** a **threaded anchor (eye-bolt) at the JOINT end**, adjusted with
  a hex key. Chosen over spool-side rewind because it acts directly on the loop
  that stretched and needs no motor re-indexing.
- **Travel required:** cable runs are ~0.15–0.25 m; UHMWPE service creep of
  1–2 % gives **1.5–5 mm**. Specify **±5 mm** of adjuster travel `[assumed]`.
- **Access:** each girdle and limb-root housing gets a **removable cover plate**
  on the outboard face; the joint-end adjusters must be reachable **without
  removing a bearing or de-mounting the limb**. This is a hard constraint on the
  housing design, recorded here so it is not discovered late.
- **Calibration hook:** re-tensioning changes the tendon-to-joint zero, so it
  must be followed by the **FR7 calibration routine** (zero tendon tension and
  joint range) — a mechanical action with a firmware consequence.

## 5. Assembly sequence (whole robot)

1. Bond inserts into all bone tubes; cure; **verify pull-out on a coupon** `[owed]`.
2. Build each limb: clevis + bearings + shaft + sheave per §2, working distal → proximal.
3. Populate girdle backplanes with motor modules and driver boards
   ([BOARD_OUTLINE](../electronics/BOARD_OUTLINE.md)); fit the girdle covers last.
4. Mount limbs to scapula/pelvis; fit the limb-root idlers (§0.1).
5. Route tendons girdle → idler → joint sheave; splice the joint ends (§3).
6. Pre-tension, heat-set, set length at the adjusters (§4).
7. Fit the ribcage and the electronics/battery bay inside it.
8. Run the FR7 calibration routine; verify per-tendon tension against ADR-0004
   sensing before any load test.

## 6. What is still owed

- **Shop drawings / a real BOM with part numbers** — this document specifies
  methods and fits, not manufacturable geometry.
- **Bond coupon test** for the insert (the whole 20 mm engagement rests on an
  assumed 10–20 MPa epoxy allowable).
- **CF transverse allowable** — the 150 MPa in §0.2 is the least trustworthy
  number here; it decides whether through-bolting is ever acceptable.
- **Tolerance stack-up** across a limb — individual fits are specified, but the
  accumulated effect on foot position is not analysed.
- **Serviceability review**: §4 asserts adjusters must be reachable without
  disassembly; nothing has verified that against the actual housing geometry.
