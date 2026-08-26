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
  not a fixed direction in joint space. The ankle needs ADR-0002 **Option A**, not a
  spring.
  ⚠️ Numbers published here before: M42's 1269 / 39.7 / 128-91 and M43's 1304 / 53.9
  / 136-107. Each routing repair re-cut the cable runs and moved them. The
  **conclusion and the ~175 kN/m target survived all three**.
- G4. Energy-efficient movement compared with a direct-drive baseline.
- G5. An articulated, tendon-driven spine so the body can arch, bend laterally,
  and twist like a real cat (P2).
- G6. Mid-air righting: reorient during a fall to land feet-first, via spine
  axial-twist + legs, with a coarse single-tendon tail assist
  (see [ADR-0007](DESIGN_DECISIONS.md)).

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
| FR11 | Detect a fall and reorient (tail + spine twist) to land feet-first.        | Should   |
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
| NFR2c | Total actuated DOF (12 legs + 6 spine + 1 tail)  | **19** (= 19 motors, ADR-0008 + **ADR-0009** lateral). ✅ Confirmed against the routed drive (M37): 3 motors per leg, each driving an antagonistic pair through the ADR-0008 variable-radius pulley, ankle single-tendon + return spring. |
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
| NFR12 | **Balance PIPELINE latency** (contact → command)  | **≤ 7.5 ms** — contact 1.0 + estimation 5.0 + transport 1.0 + compute 0.5 (ADR-0016). Re-cast from a whole-loop ≤20 ms: whole-loop is ~45 ms and **37 ms of it is the leg moving**, not electronics. |
| NFR13 | Lateral shove rejected — *reduced-order capability*      | **0.41 m/s** — the physical reading of NFR10 via xi = c + c_dot/omega. ⚠️ This is what the robot ACHIEVES; **NFR15 is what it must achieve** (ADR-0017). |
| **NFR15** | **Disturbance cases the robot MUST survive**  | a **15 N / 0.1 s push** (48 mm), an **unexpected 40 mm step**, and a **10° lateral slope**. A 30 N shove (96 mm) is explicitly OUT of scope. Met with ~19 % margin **in the reduced-order model** (ADR-0017). `[assumed]` scenarios. ⚠️ **Not yet DEMONSTRATED in simulation, and by a wider margin than long recorded.** The MuJoCo harness reaches **25.6 mm** worst-direction — but ⚠️ **ADR-0040 (M35) established that is a SURVIVAL figure** (the trial passes if the robot does not fall inside the horizon), whereas the viable set is a **RECOVERY** bound. Like-for-like the recovery envelope is **1.5 mm**: the shipped controller ends its certified 25.6 mm trial 26.2 mm off its support. ⚠️ The previously quoted **"86 % of optimal" is WITHDRAWN** — it compared the two criteria. Cause diagnosed: the placement law has no term removing a *persistent* DCM offset, so it settles into a biased limit cycle (the ADR-0013 "walking away sideways" mode). ✅ **Still ACHIEVABLE** (ADR-0033): the exact viable set is **62.7 mm**, past the 48 mm required, and the reduced-order model is *conservative* on the spine term. ⚠️ **But one contradiction is open** (M36): at a 0.117 s stance the harness genuinely recovers from 42.2 mm against a 39.5 mm exact bound, so either that bound or the simulation is wrong. |
| **NFR16** | **Floor friction μ** (reinstated)             | **≥ 0.70** — the spine's balance action is INTERNAL motion, so shifting the CoM against the planted feet costs ground reaction: 0.71 for full spine authority + 0.145 for the gait. ⚠️ ~~ADR-0010 withdrew this~~ — correctly, for the *crawl crossover*; it returns for a *different mechanism*. ⚠️ **Relaxed by ADR-0034:** re-derived on the exact viable set, NFR15 is met from **μ ≥ 0.6**, so 0.70 carries ~20 % margin rather than none. |
| NFR14 | **Leg spare foot speed** (for corrections)         | **≥ 4.1 m/s** — the DOMINANT term in the balance loop. Ceiling is 5.93 m/s, nominal swing uses 1.83 (ADR-0016). |
| NFR11 | **DCM estimation accuracy**                       | **≤ 3 mm** — a steady bias becomes a PERMANENT lateral offset amplified 3.2× (ADR-0013). Sharpens NFR8/ADR-0012. |
| NFR9  | **Paw sensor mass**                               | **≤ 20 g per paw** — binding via SWING INERTIA, not mass: 20 g costs top speed 120→96 cm/s, 40 g exceeds the motor's continuous rating (ADR-0012) |
| NFR3  | Control loop rate (tension/position)             | ≥ 1 kHz             |
| NFR4  | Gait / trajectory update rate                    | ≥ 100 Hz            |
| NFR5  | Mass (total)                                     | ✅ **4.3041 kg — FOLDED IN by ADR-0046 (M41)**; `params.py` now carries it. ⚠️ **4.31 kg** — raised again by **ADR-0043 (M38)**: drawn as manufacturable parts a leg is **167 g**, not the assumed 110/95 g, so the body closes at **4.304 kg**, 6.3 % past the previous 4.05 kg. Every design gate still passes (trot at 80 % of motor peak, cable SF 4.70, bearing C0 1277/1500 N) — the overrun costs margin, not viability. History: 3.0 kg → **4.05** (ADR-0010, real 132 g motor) → **4.31** (ADR-0043, real joint hardware). A domestic cat is 4–5 kg. ⚠️ **Not yet folded into `params.py`**, so published mass-derived figures (NFR6 runtime, NFR18 thermal) still carry 4.045 kg. |
| NFR6  | Runtime on one battery charge                    | ⚠️ **14–20 min / 420–600 m** trotting at 50 cm/s — **re-stated by ADR-0044 (M39)** from the published ~30 min / ~900 m. **Three corrections and one uncertainty.** Corrections: ADR-0043's 4.304 kg body and the 8.75 mm spool §2 requires (−17 %, → 25.2 min at 100.2 W), and `power.py`'s copper-loss formula, which uses `I²R_pp` where balanced three-phase is `3I²R_ph = 1.5×` that (→ 19.6 min at 128.6 W; its own docstring flagged it and nothing had priced it). Uncertainty: the vendor's Kt disagrees with its own current ratings by 27 %, worth the rest of the spread (→ 14.1 min at 178.2 W). ⚠️ Rotor-side is **ruled out** as the explanation (7.1× off, wrong direction); a six-step-vs-sinusoidal convention fits to 0.4 %, in which case both vendor numbers are right and only the driver's current sense decides. Standing with the ADR-0003 brake scales the same way. 300 g pack, `[assumed]` 175 Wh/kg / 80 % usable. |
| NFR17 | **Power-off stance brake**                        | **Required, not optional** — standing costs 76 % of moving for zero work; the brake is worth **4.5×** standing endurance (ADR-0021). |
| NFR18 | **Girdle surface finish + duty limit**            | Girdles **anodised** (ε ≥ 0.9) — worth **~59 K** (⚠️ re-derived by ADR-0045; the lever grew because radiation goes as T⁴ and the operating point rose). **Continuous/tethered trotting is OUT OF SPEC in still air at ANY finish** — anodised settles at **96.1 °C** (~~74.9~~). ⚠️ **And the battery-limited case is now marginal, not comfortable: 70.2 °C** against the 70 °C line it used to clear by 10 K. ⚠️ **Forced air is REQUIRED for continuous operation, not an option**: h ≈ 15 brings it to 72.7 °C, h ≈ 25 to 58.2 °C. Winding runs **+11.5 K** above skin (~~+7.7~~), so anodised continuous winding is **107.6 °C** and polished **166.7 °C** — the magnet-range concern is sharper (ADR-0024/0045). |
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
