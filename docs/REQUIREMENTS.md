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
| NFR2c | Total actuated DOF (12 legs + 6 spine + 1 tail)  | **19** (= 19 motors, ADR-0008 + **ADR-0009** lateral) |
| NFR2d | Tail actuation (coarse assist, no accuracy)      | 1 tendon + passive return |
| NFR2e | Spine LATERAL bend ROM (per segment)             | **±15°** (ADR-0009; gait commands 11°, so ~4° spare) |
| NFR2f | Spine lateral **slew rate** (per segment)         | **≥ 119 °/s** — sized to a FAST reference manoeuvre (righting / future dynamic gait), **not** the 5 s crawl, which needs only ~29 °/s (ADR-0010) |
| ~~NFR2g~~ | ~~Paw–ground friction μ ≥ 0.70~~ | **WITHDRAWN (ADR-0010).** The resolved per-foot demand is **μ ≈ 0.055**; friction was never the binding constraint. Any sane pad meets it. |
| NFR2h | Statically stable walk speed                      | **~1.1 cm/s** (crawl), limited by **TIPPING** (ZMP), not friction. Faster requires a *dynamic* gait — ADR-0010 |
| NFR2i | Dynamic (ZMP) stability margin, CRAWL             | **> 0** at every phase; currently **+6.4 mm** ⚠️ small |
| NFR2j | **TROT** speed (the locomotion mode)              | **~50 cm/s** default — slowed from 67 cm/s by ADR-0020: the spine's balance assist costs ground friction as `1/stance²`, and at 0.3 s it exceeded a realistic floor. Motor-thermally capable of ~96 cm/s on a better floor. |
| NFR2k | Trot roll oscillation                             | **bounded** — roll-rate drift ≈ 0 per cycle, ±0.4° peak. Requires `nominal_foot` x ≈ 0.005 m; the crawl's 0.05 m falls in one stride |
| NFR8  | Paw force sensing range / survival                | **0–35 N** measured, **≥100 N** survival (×2.5 land transient), ≤0.4 N resolution, ≥1 kHz (ADR-0012) |
| NFR10 | **Disturbance rejection envelope** (trot) — *measured capability* | **57 mm** DCM error, fixed-point with real latency AND the actuation ramp (ADR-0017). Superseded figures: 74 → 33 → 90 → 59 → **57**. |
| NFR12 | **Balance PIPELINE latency** (contact → command)  | **≤ 7.5 ms** — contact 1.0 + estimation 5.0 + transport 1.0 + compute 0.5 (ADR-0016). Re-cast from a whole-loop ≤20 ms: whole-loop is ~45 ms and **37 ms of it is the leg moving**, not electronics. |
| NFR13 | Lateral shove rejected — *measured capability*      | **0.44 m/s** — the physical reading of NFR10 via xi = c + c_dot/omega. ⚠️ This is what the robot ACHIEVES; **NFR15 is what it must achieve** (ADR-0017). |
| **NFR15** | **Disturbance cases the robot MUST survive**  | a **15 N / 0.1 s push** (48 mm), an **unexpected 40 mm step**, and a **10° lateral slope**. A 30 N shove (96 mm) is explicitly OUT of scope. Met with ~19 % margin (ADR-0017). `[assumed]` scenarios. |
| **NFR16** | **Floor friction μ** (reinstated)             | **≥ 0.70** — the spine's balance action is INTERNAL motion, so shifting the CoM against the planted feet costs ground reaction: 0.71 for full spine authority + 0.145 for the gait. Below 0.70, NFR15 fails (ADR-0019). ⚠️ ~~ADR-0010 withdrew this~~ — correctly, for the *crawl crossover*; it returns for a *different mechanism*. |
| NFR14 | **Leg spare foot speed** (for corrections)         | **≥ 4.1 m/s** — the DOMINANT term in the balance loop. Ceiling is 5.93 m/s, nominal swing uses 1.83 (ADR-0016). |
| NFR11 | **DCM estimation accuracy**                       | **≤ 3 mm** — a steady bias becomes a PERMANENT lateral offset amplified 3.2× (ADR-0013). Sharpens NFR8/ADR-0012. |
| NFR9  | **Paw sensor mass**                               | **≤ 20 g per paw** — binding via SWING INERTIA, not mass: 20 g costs top speed 120→96 cm/s, 40 g exceeds the motor's continuous rating (ADR-0012) |
| NFR3  | Control loop rate (tension/position)             | ≥ 1 kHz             |
| NFR4  | Gait / trajectory update rate                    | ≥ 100 Hz            |
| NFR5  | Mass (total)                                     | **4.05 kg** — raised from 3.0 kg (ADR-0010) once a real motor was sourced: the lightest purchasable part is 120 g, not the 72 g class target. A domestic cat is 4–5 kg. |
| NFR6  | Runtime on one battery charge                    | **~30 min / ~900 m** trotting at 50 cm/s (83.6 W); **~168 min standing WITH the ADR-0003 brake**, 37 min without it (ADR-0021). 300 g pack, `[assumed]` 175 Wh/kg. |
| NFR17 | **Power-off stance brake**                        | **Required, not optional** — standing costs 76 % of moving for zero work; the brake is worth **4.5×** standing endurance (ADR-0021). |
| NFR18 | **Girdle surface finish + duty limit**            | Girdles **anodised** (ε ≥ 0.9) — worth **~39 K** (ADR-0023). **Continuous/tethered trotting is OUT OF SPEC** in still air: the 30 min battery, not the design, is what keeps the girdle below 70 °C. Forced air (h ≈ 15) would reopen it. **Also what keeps the rotor magnets in range**: polished, the stator reaches 117.5 °C (ADR-0024). |
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
