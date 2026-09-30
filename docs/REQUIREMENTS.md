# TomCat — Requirements

Status: **DRAFT** — targets are placeholders to be confirmed. Open questions are
tracked inline as `❓`.

Grounded in the two [design principles](PRINCIPLES.md): (P1) tendon-driven,
centralized multi-motor actuation, and (P2) a feline form whose whole body may
curve.

## 1. Goals

- G1. Quadruped locomotion with cat-like agility (walk, trot, and eventually a
  controlled leap/land).
- G2. Tendon-driven joints with motors centralized in the body girdles to
  minimize limb inertia (P1).
- G3. Passive compliance / shock absorption at each joint.
  ✅ **SIZED by ADR-0047 (M42), banded by ADR-0048 (M43), and CONFIRMED FROM A
  SECOND DIRECTION by ADR-0049 (M44): a 150-200 kN/m series-elastic element at the
  hip and knee**, **175 kN/m** the point value to quote. The cable alone gives the
  hip **1295 N·m/rad** — **5.2×** the stiffness at which ADR-0026's balance
  harness wound up and fell — so ADR-0026's *"balance needs compliant legs"* is a
  **hardware** requirement, not a controller gain. At 175 kN/m the hip and knee land
  at **133 / 104 N·m/rad**, inside ADR-0026's 80-150 window.
  ✅ **And under FOOT-FORCE control the element is what makes the robot stand at
  all** (ADR-0049): at 175 kN/m it stands to 0.006° of trunk tilt, with the bare
  cable it **inverts**, with no elasticity it leans 14.6°. Two independent
  arguments — balance compliance and force control — two different failure
  modes, the same part.
  ⚠️ **Not at the ankle**, which fails two other ways: a single-tendon joint has
  **no restoring stiffness** from its cable (**41.0** N·m/rad measured, 0.3 of it
  the ADR-0002 Option-B return spring), and ADR-0049 found its **moment arm reverses
  sign inside the ROM at every anchor angle** — so the one direction it can pull is
  not a fixed direction in joint space.
  ✅ **ADR-0050 (M45) settled that: the ankle takes ADR-0002 Option A, an
  antagonistic pair in a CAPSTAN construction** (both cables to one anchor, opposite
  wraps — mirroring the way the hip and knee do it leaves a 60° dead band with the
  stance hock inside it). The deciding argument was **kinematic reach**, not
  stiffness: the trot commands the ankle **+62.4° above its reference during SWING**
  and Option B has no actuator that pulls that way. Option A doubles the ankle's
  restoring stiffness too (11.4 → 22.5 N·m/rad), though that is still far below
  ADR-0026's 80 floor — the ankle cables are the longest, so `EA/L` is lowest.
  ⚠️ Numbers published here before: M42's 1269 / 39.7 / 128-91 and M43's 1304 / 53.9
  / 136-107. Each routing repair re-cut the cable runs and moved them. The
  **conclusion and the ~175 kN/m target survived all three**.
- G4. Energy-efficient movement compared with a direct-drive baseline.
- G5. An articulated, tendon-driven spine so the body can arch and bend
  laterally like a real cat (P2). ✅ **MET** — 3 sagittal + 3 lateral joints,
  built and measured ([ADR-0062](DESIGN_DECISIONS.md)).
  ⚠️ **The "and twist" clause was withdrawn by the project owner (M66,
  [ADR-0071](DESIGN_DECISIONS.md)).** Axial roll was specified by ADR-0006 and
  deferred; the only use ever recorded for it was G6's righting, which went with
  G6. Nothing else in the project asked for it, so the spine is complete at
  **6 DOF**.
- ~~G6. Mid-air righting: reorient during a fall to land feet-first, via spine
  axial-twist + legs, with a coarse single-tendon tail assist.~~
  ⚠️ **WITHDRAWN by the project owner (M66, [ADR-0071](DESIGN_DECISIONS.md)).**
  Measured, the robot rights from fully inverted in **2.14 s** — a fall from
  **22.5 m**, against the 0.247 s a 0.3 m drop allows. ⚠️ **And 2.14 s was
  the optimistic plant**: give the spine the drivetrain G3 requires and it takes
  **10.10 s**, a fall from **500 m** (M70, [ADR-0075](DESIGN_DECISIONS.md)). Every route to closing that
  was measured and closed: trajectory design bought 1.5×, the axial DOF the goal
  names is **negative**, and feedback buys direction but not speed.

## 2. Functional requirements

| ID   | Requirement                                                                 | Priority |
|------|-----------------------------------------------------------------------------|----------|
| FR1  | Drive N tendons via rotary motors with closed-loop position control.        | Must     |
| FR2  | Measure and closed-loop control cable **tension** per driven tendon.        | Must     |
| FR3  | Sense joint angle (directly or inferred from cable displacement).           | Must     |
| FR4  | Execute a parameterized gait to produce forward walking.                    | Must     |
| FR4b | Execute a diagonal **TROT** (dynamic gait) — the primary locomotion mode.   | Must     |
| FR9  | Actuate the spine to bend (dorsoventral + lateral) via tendons.             | Must     |
| FR9b | Command lateral spine sway toward the support side, in phase with the gait. | Must     |
| FR10 | Coordinate spine curvature with leg motion (whole-body posture).            | Should   |
| ~~FR11~~ | ~~Detect a fall and reorient (tail + spine twist) to land feet-first.~~ ⚠️ **WITHDRAWN with G6** ([ADR-0071](DESIGN_DECISIONS.md)). | ~~Should~~ |
| FR5  | Detect and recover from a foot slip / unexpected ground contact.            | Should   |
| FR12 | Sense **per-foot contact and normal force** (≥1 kHz) for closed-loop balance. | Must     |
| FR6  | Report telemetry (per-motor current, tension, angle) over a host link.      | Should   |
| FR7  | Support a calibration routine for zeroing tendon tension and joint range.   | Must     |
| FR8  | Enter a safe, limp state on fault (over-current, over-tension, e-stop).     | Must     |

## 3. Non-functional / performance targets  ❓ *confirm with mechanical design*

| ID    | Target                                          | Value (placeholder) |
|-------|-------------------------------------------------|---------------------|
| NFR1  | Degrees of freedom per leg                       | 3 (hip, knee, ankle)|
| NFR2  | Spine segments (serial, tendon-driven)           | **3** (ADR-0006)     |
| NFR2b | DOF per spine segment                            | **2** — dorsoventral + lateral (ADR-0006/0009) |
| NFR2c | Total actuated DOF (12 legs + 6 spine)  | **18** — ⚠️ **was 19 with a tail motor, reduced by [ADR-0071](DESIGN_DECISIONS.md) (M66)**: the tail's only cited purpose was G6's righting assist, and G6 is withdrawn. ✅ **18 of 18 BUILT** (M57, [ADR-0062](DESIGN_DECISIONS.md)): 12 leg + 6 spine, and the 18-DOF body stands. The **tail is owed** — it has a motor in the mass budget and no parameters, joint or body anywhere. ⚠️ The earlier "confirmed against the routed drive (M37)" was a check of the *motor count*, not of actuated DOF: until M57 no MuJoCo model had more than **15**, and none had a **sagittal** spine joint at all. |
| NFR2d | Tail actuation (coarse assist, no accuracy)      | 1 tendon + passive return |
| NFR2e | Spine LATERAL bend ROM (per segment)             | **±15°** (ADR-0009; gait commands 11°, so ~4° spare) |
| NFR2f | Spine lateral **slew rate** (per segment)         | **≥ 119 °/s** — sized to a FAST reference manoeuvre (righting / future dynamic gait), **not** the 5 s crawl, which needs only ~29 °/s (ADR-0010) |
| ~~NFR2g~~ | ~~Paw–ground friction μ ≥ 0.70~~ | **WITHDRAWN (ADR-0010).** The resolved per-foot demand is **μ ≈ 0.055**; friction was never the binding constraint. Any sane pad meets it. |
| NFR2h | Statically stable walk speed                      | **~1.1 cm/s** (crawl), limited by **TIPPING** (ZMP), not friction. Faster requires a *dynamic* gait — ADR-0010 |
| NFR2i | Dynamic (ZMP) stability margin, CRAWL             | **> 0** at every phase; currently **+6.4 mm** ⚠️ small |
| NFR2j | **TROT** speed (the locomotion mode)              | **~50 cm/s** default — slowed from 67 cm/s by ADR-0020: the spine's balance assist costs ground friction as `1/stance²`, and at 0.3 s it exceeded a realistic floor. Motor-thermally capable of ~96 cm/s on a better floor. |
| NFR2k | Trot roll oscillation                             | **bounded** — roll-rate drift ≈ 0 per cycle. ⚠️ Requires `nominal_foot` **x ≈ 0.00214 m**, re-tuned from 0.005 by **ADR-0046 (M41)**: the balance point is set by where the CoM sits relative to the diagonal, so it moved when the measured leg masses landed. At the old 0.005 the drift is **−0.180 rad/s per cycle** — divergent. The crawl's 0.05 m still falls in one stride. |
| NFR8  | Paw force sensing range / survival                | **0–35 N** measured, **≥100 N** survival (×2.5 land transient), ≤0.4 N resolution, ≥1 kHz (ADR-0012) |
| NFR10 | **Disturbance rejection envelope** (trot) — *reduced-order capability* | **52.7 mm** DCM error, fixed-point with real latency AND the actuation ramp. Superseded figures: 74 → 33 → 90 → 59 → 57 (ADR-0017, 0.3 s stance) → 53.9 (ADR-0020, trot slowed to 0.4 s) → **52.72** (ADR-0025, sway CoM correction). ⚠️ This is the 1-D reduced-order figure; **measured** in closed-loop simulation it is **25.6 mm** in the worst direction (ADR-0037) — see NFR15. |
| NFR12 | **Balance PIPELINE latency** (contact → command)  | **≤ 7.5 ms** — contact 1.0 + estimation 5.0 + transport 1.0 + compute 0.5 (ADR-0016). Re-cast from a whole-loop ≤20 ms: whole-loop is ~45 ms and **37 ms of it is the leg moving**, not electronics. ✅ **MET — measured on the plant** (M73, [ADR-0078](DESIGN_DECISIONS.md)): at 1 kHz the 7.5 ms budget costs **2 N** (74 against a 72 N baseline) and 0.01° of tilt; at 133 Hz it costs 47 N and 0.91°. The failure boundary is **15–20 ms**, so the re-cast from 20 ms is what buys the margin. |
| NFR13 | Lateral shove rejected — *reduced-order capability*      | **0.41 m/s** — the physical reading of NFR10 via xi = c + c_dot/omega. ⚠️ This is what the robot ACHIEVES; **NFR15 is what it must achieve** (ADR-0017). |
| **NFR15** | **Disturbance cases the robot MUST survive**  | a **15 N / 0.1 s push** (48 mm), an **unexpected 40 mm step**, and a **10° lateral slope**. A 30 N shove (96 mm) is explicitly OUT of scope. Met with ~19 % margin **in the reduced-order model** (ADR-0017). `[assumed]` scenarios. ⚠️ **Not yet DEMONSTRATED in simulation, and by a wider margin than long recorded.** The MuJoCo harness reaches **25.6 mm** worst-direction — but ⚠️ **ADR-0040 (M35) established that is a SURVIVAL figure** (the trial passes if the robot does not fall inside the horizon), whereas the viable set is a **RECOVERY** bound. Like-for-like the recovery envelope is **1.5 mm**: the shipped controller ends its certified 25.6 mm trial 26.2 mm off its support. ⚠️ The previously quoted **"86 % of optimal" is WITHDRAWN** — it compared the two criteria. Cause diagnosed: the placement law has no term removing a *persistent* DCM offset, so it settles into a biased limit cycle (the ADR-0013 "walking away sideways" mode). ✅ **Still ACHIEVABLE** (ADR-0033): the exact viable set is **62.7 mm**, past the 48 mm required, and the reduced-order model is *conservative* on the spine term. ⚠️ **But one contradiction is open** (M36): at a 0.117 s stance the harness genuinely recovers from 42.2 mm against a 39.5 mm exact bound, so either that bound or the simulation is wrong. |
| **NFR16** | **Floor friction μ** (reinstated)             | **≥ 0.70** — the spine's balance action is INTERNAL motion, so shifting the CoM against the planted feet costs ground reaction: 0.71 for full spine authority + 0.145 for the gait. ⚠️ ~~ADR-0010 withdrew this~~ — correctly, for the *crawl crossover*; it returns for a *different mechanism*. ⚠️ **Relaxed by ADR-0034:** re-derived on the exact viable set, NFR15 is met from **μ ≥ 0.6**, so 0.70 carries ~20 % margin rather than none. |
| NFR14 | **Leg spare foot speed** (for corrections)         | **≥ 4.1 m/s** — the DOMINANT term in the balance loop. Ceiling is 5.93 m/s, nominal swing uses 1.83 (ADR-0016). |
| NFR11 | **DCM estimation accuracy**                       | **≤ 3 mm** — a steady bias becomes a PERMANENT lateral offset amplified 3.2× (ADR-0013). Sharpens NFR8/ADR-0012. |
| NFR9  | **Paw sensor mass**                               | **≤ 20 g per paw** — binding via SWING INERTIA, not mass: 20 g costs top speed 120→96 cm/s, 40 g exceeds the motor's continuous rating (ADR-0012) |
| NFR3  | Control loop rate (tension/position)             | ≥ 1 kHz             |
| NFR4  | Gait / trajectory update rate                    | ≥ 100 Hz            |
| NFR5  | Mass (total)                                     | ✅ **4.589 kg — M124 ([ADR-0115](DESIGN_DECISIONS.md))**: the hub lightened, the bearing at its catalogue 22.8 g, the fore radius Ø14 (it was SF 1.76 at landing — never checked). Motor peak 1.650 N·m. ✅ 4.605 kg — M123 ([ADR-0114](DESIGN_DECISIONS.md))**: the hollow hip, +26 g a leg (a hub on two 40 × 50 bearings); the trunk is 21 mm shorter (each girdle is two rows straddling its hip). Motor peak 1.656 N·m, the hind and fore hips tied. ✅ 4.501 kg — M122 ([ADR-0112](DESIGN_DECISIONS.md))**: the legs drop the hip via, the return spring and the anchor pins (−13 g each); the trunk is 30 mm longer (spine joint 0 moved forward). Motor peak 1.623 N·m. ✅ **4.554 kg — M120 ([ADR-0111](DESIGN_DECISIONS.md)) folds in the eighteen G3 flexures, 85.6 g**, drawn thinner (the stop moved into the spool) so every row fits the trunk without lengthening a body; motor peak 1.859 N·m. ✅ **4.3041 kg — FOLDED IN by ADR-0046 (M41)**; `params.py` now carries it. ⚠️ **4.31 kg** — raised again by **ADR-0043 (M38)**: drawn as manufacturable parts a leg is **167 g**, not the assumed 110/95 g, so the body closes at **4.304 kg**, 6.3 % past the previous 4.05 kg. ~~Every design gate still passes (trot at 80 % of motor peak, cable SF 4.70, bearing C0 1277/1500 N)~~ — ⚠️ **the motor gate is gone, by [ADR-0100](DESIGN_DECISIONS.md) (M107).** ✅ **M111 (ADR-0104): 1.825 N·m peak / 0.646 continuous, INSIDE the proxy, at the 36/34/22 arms and a 4.468 kg body** — ⚠️ but the walked trot needs **~665 rpm** no-load against the proxy's 380, and has never fitted it at any arms. ✅ **M119 ([ADR-0110](DESIGN_DECISIONS.md)): bought by the WINDING** — the same 3505 frame at 9:1, rewound to `Kv_out ≥ 665 rpm / V_floor` (23.8 rpm/V, ×1.5, on 8S), keeps envelope, mass and thermal; a lower ratio cannot hold 0.624 N·m continuous. Before: with capstan friction counted the trot needs **2.42 N·m against the proxy's 1.95** — ⚠️ **2.42, not the 2.20 ADR-0100 published: that was the HIND leg, and [ADR-0102](DESIGN_DECISIONS.md) found the budget has only ever seen one leg.** The fore knee sets it, at 24 % over where the hind is 13, and the two overruns differ in kind — the hind's is routing and routes away, the fore's is load and does not, so the mechanism is the primary design and the motor's torque became an OUTPUT rather than a limit. The gates that remain are set by parts actually chosen: cable SF 4.70 and bearing C0 1277/1500 N, both holding. ⚠️ The proxy still supplies the Ø34.5 × 36.1 mm envelope and 131.7 g this row is built on, and whether a part meeting the spec keeps them is `[owed]`. History: 3.0 kg → **4.05** (ADR-0010, real 132 g motor) → **4.31** (ADR-0043, real joint hardware). A domestic cat is 4–5 kg. ~~⚠️ Not yet folded into `params.py`~~ — ✅ **that WAS the state before M41 and this cell kept saying it after**; ADR-0046 folded 4.3041 kg into `params.py` and re-derived NFR6 and NFR18 from it. M45 found the contradiction sitting inside this one cell. ⚠️ **And a FOURTH increase is now projected: 4.83 kg**, because [ADR-0050](DESIGN_DECISIONS.md) (M45) adopts ADR-0002 **Option A** at the ankle — four more motors at 132 g, **+528 g (+12.3 %)**, actuators 19 → 23. Not folded in yet: the CAD, the mass closure and `params` all still say 4.3041, and the compiled MuJoCo plant does not show it either (the motors live in `trunk_mass`). |
| NFR6  | Runtime on one battery charge                    | ✅ **17.8–23.7 min / ~711 m — M124 ([ADR-0115](DESIGN_DECISIONS.md))**. ✅ 17.8–23.6 min / ~709 m — M123 ([ADR-0114](DESIGN_DECISIONS.md))**: the hollow hip's +104 g of body. ✅ 18.2–24.2 min / ~725 m — M122 ([ADR-0112](DESIGN_DECISIONS.md))**: friction is charged where the cable slides (its Bowden conduit), not on every degree of wrap. ✅ **15.6–21.1 min / ~634 m — still MET after M120 ([ADR-0111](DESIGN_DECISIONS.md)) carries G3's 85.6 g.** ~~16.0–21.7 min / ~650 m~~ — MET at both Kt corners, by [ADR-0104](DESIGN_DECISIONS.md) (M111).** The 36/34/22 moment arms ADR-0103 adopted for the fore leg are a reduction: the same joint torque at 1/r the cable tension, and copper goes as its square, so it halves. Efficiency 16.5 → 30.7 %. ~~⚠️ **8.8–12.7 min / ~381 m — BELOW its own re-stated range, by [ADR-0100](DESIGN_DECISIONS.md) (M107), corrected by ADR-0101.** ✅ **And routing recovers some of it:** the hip spool wants z ≈ −20 where it sits at +37, which takes the trot peak 2.20 → 1.93 N·m and puts the motor requirement back inside the proxy. Whether the girdle can house it 16.6 mm below its row's lower slot is `[owed]`. Capstan friction had never reached the plant: ADR-0003 specified the model, `tendon.py` implemented it, `wrap_angle` was 0.0, and ADR-0083 solved every wrap in M78 into a `pair_wrap` that nothing read. Counted, it costs **71 %** of the trot's copper — runtime **18.81 → 12.13 min** optimistic and **13.58 → 8.50** pessimistic, range **565 → 364 m**. Both Kt corners now fall under the 14–20 min below, where before only the pessimistic one was marginal. ⚠️ And the Kt question hardened: RMS current is 1.380 A at Kt 0.44 and **1.735 A** at 0.35 against a **1.60 A** continuous rating, so it now decides whether the motor runs cool, not just how long. ~~⚠️ **14–20 min / 420–600 m**~~ trotting at 50 cm/s — **re-stated by ADR-0044 (M39)** from the published ~30 min / ~900 m. **Three corrections and one uncertainty.** Corrections: ADR-0043's 4.304 kg body and the 8.75 mm spool §2 requires (−17 %, → 25.2 min at 100.2 W), and `power.py`'s copper-loss formula, which uses `I²R_pp` where balanced three-phase is `3I²R_ph = 1.5×` that (→ 19.6 min at 128.6 W; its own docstring flagged it and nothing had priced it). Uncertainty: the vendor's Kt disagrees with its own current ratings by 27 %, worth the rest of the spread (→ 14.1 min at 178.2 W). ⚠️ Rotor-side is **ruled out** as the explanation (7.1× off, wrong direction); a six-step-vs-sinusoidal convention fits to 0.4 %, in which case both vendor numbers are right and only the driver's current sense decides. Standing with the ADR-0003 brake scales the same way. 300 g pack, `[assumed]` 175 Wh/kg / 80 % usable. |
| NFR17 | **Power-off stance brake**                        | ⚠️ **Re-priced by [ADR-0108](DESIGN_DECISIONS.md) (M115): still worth having, no longer "required, not optional" on this argument.** At the stance actually held, standing costs **31 %** of trotting, not 76 %, and the brake multiplies standing endurance **3.1×** (54 → 168 min), not 4.5×. The 76 % came from pricing the stand at the worst reachable pose on every joint at once. ~~Required, not optional — standing costs 76 % of moving for zero work; the brake is worth 4.5× standing endurance (ADR-0021).~~ |
| NFR18 | **Girdle surface finish + duty limit**            | ⚠️ **M124: anodised continuous trot 93.4 °C.** ⚠️ **M123 ([ADR-0114](DESIGN_DECISIONS.md)): anodised continuous trot 91.6 → 93.6 °C**; forced air h ≈ 15 → 70.9 °C, one battery 67.9 °C. ⚠️ M122 ([ADR-0112](DESIGN_DECISIONS.md)): anodised continuous trot 102.8 → 91.6 °C** — still over 80 °C, so forced air is still required; at h ≈ 15 it settles at 69.4 °C. Girdles **anodised** (ε ≥ 0.9) — worth **~59 K** (⚠️ re-derived by ADR-0045; the lever grew because radiation goes as T⁴ and the operating point rose). **Continuous/tethered trotting is OUT OF SPEC in still air at ANY finish** — anodised settles at **96.1 °C** (~~74.9~~). ⚠️ **And the battery-limited case is now marginal, not comfortable: 70.2 °C** against the 70 °C line it used to clear by 10 K. ⚠️ **Forced air is REQUIRED for continuous operation, not an option**: h ≈ 15 brings it to 72.7 °C, h ≈ 25 to 58.2 °C. Winding runs **+11.5 K** above skin (~~+7.7~~), so anodised continuous winding is **107.6 °C** and polished **166.7 °C** — the magnet-range concern is sharper (ADR-0024/0045). |
| NFR7  | Max cable tension per tendon                     | ❓ TBD (N)           |

## 4. Constraints & assumptions

- Antagonistic tendon pairs are needed because a cable can only pull. Settled by
  [ADR-0008](DESIGN_DECISIONS.md): **one motor per DOF**, driving both sides of
  the pair through a **variable-radius pulley** — the mass budget does not permit
  two motors per DOF.
- Cables are inextensible enough that motor rotation maps predictably to joint
  angle, but tendon stretch and friction must be modeled/compensated.
- Motors, drivers, battery, and main compute live in the torso.

## 5. Open questions

> **Prioritised by consequence in [OPEN_RISKS.md](OPEN_RISKS.md)** — which of the
> 48 `[owed]` / 89 `[assumed]` items can actually change a decision, and what
> closes each. Two dominate: **motor mass** (21 % margin before the budget breaks)
> and **paw friction** (met with none).

The major architecture questions are **resolved** in the [ADR log](DESIGN_DECISIONS.md):
- Actuator choice → tendon-drive, BLDC + FOC (ADR-0003).
- Tension sensing → hybrid: motor-current estimate everywhere + joint-end load
  cells on stiffness-critical joints (ADR-0004).
- Compute split → distributed CAN-FD drivers + RT controller + SBC (ADR-0005).
- Tendons per DOF → antagonistic pairs; spring-return for distal joints (ADR-0002).

Remaining **calibration / measurement** items (not decisions):
- ❓ Routed cable friction μ and per-tendon wrap — blocks the tension error budget
  and the current-vs-load-cell placement split (ADR-0004 follow-up).
- ✅ **Specific motor part — CLOSED** by the
  [reality check](notes/motor-reality-check.md): **SteadyWin GIM3505-9**, 0.71 /
  1.95 N·m at the output, 9:1, **131.7 g with driver**, 24 V, Kt 0.35 N·m/A. The
  ≤80 g class target does not exist in this torque band; NFR5 rose to 4.05 kg as
  a result (ADR-0010). ⚠️ Still owed: buy and weigh one, and a **thermal test** —
  sustained trot is a 2.1× overload on the continuous rating.
- ❓ Per-tendon force target (NFR7). *(Runtime/NFR6 closed by ADR-0021.)*

## 6. Out of scope (for now)

- Autonomous navigation / SLAM.
- Manipulation (the cat does not need to pick things up).
- Outdoor / all-terrain operation.
