# TomCat — Board-Level Electronics Outline (M1 task E1)

Owner: **tomcat-electronics** · Milestone: **M1** (task E1: first board-level outline)
Ties to [ADR-0003](../docs/DESIGN_DECISIONS.md) (BLDC + FOC), [ADR-0004](../docs/DESIGN_DECISIONS.md)
(rotor sensor + tension method), [ADR-0005](../docs/DESIGN_DECISIONS.md) (distributed
smart drivers on CAN-FD, 3 safety tiers, hardware e-stop), and the mechanical
budgets in [LEG_TENDON_SPEC.md](../mechanical/LEG_TENDON_SPEC.md) /
[SPINE_TAIL_SPEC.md](../mechanical/SPINE_TAIL_SPEC.md).

**Status: BLOCK-LEVEL ARCHITECTURE ONLY.** No KiCad schematic or PCB this pass —
this document defines the boards, their blocks, the channel count/clustering, the
electrical interface for firmware, and the gaps that must close before detailed
sizing / schematic capture.

**Label legend:** `[sourced]` traceable to an ADR / mechanical spec / lit review;
`[assumed]` first-pass engineering guess; `[owed]` must come from another owner;
`[GAP]` a blocking unknown that prevents detailed component sizing.

The CAN-FD **message schema is owned by firmware** (referred to here as the
**firmware F1 schema**). This outline references it as the driver↔RT-controller
contract; it does **not** define message layouts, IDs, or timing.

---

## 0. The three gaps that block detailed sizing (read first)

Everything downstream of "which motor" is parameterized on three still-open
inputs. They are called out here once and flagged again wherever they bite:

| # | Gap | Owner / source | Blocks |
|---|---|---|---|
| G-Kt | **Motor torque constant `Kt` (N·m/A)** — no candidate motor chosen (ADR-0003 keeps actuator open) | `[owed: lead / actuator down-select]` | Phase-current sizing → FET/gate-driver rating, shunt value, thermal path |
| G-Vbus | **Motor-bus voltage** — no battery/pack chosen | `[owed: lead / power budget]` | FET Vds class, gate-driver rating, bulk-cap voltage, regulation topology |
| G-Tens | **ADR-0004 tension-sensing method** still open: in-line load cell vs. motor-current estimate vs. series-elastic + displacement | `[owed: ADR-0004 close-out]` | Whether the driver board carries an instrumentation-amp load-cell front-end, relies on phase-current estimation only, or exposes a displacement input |

Until G-Kt and G-Vbus close, the driver stage is specified as a **parameterized
power stage** (current/voltage *classes*, not part numbers), sized from the
mechanical tension budget below. Until G-Tens closes, the driver board carries
**both** a current-sense path (always present, needed for FOC anyway) **and** a
populate-optional load-cell front-end, so either ADR-0004 outcome is buildable
without a respin.

### Sizing anchors taken from the mechanical specs

| Quantity | Value | Source |
|---|---|---|
| Leg continuous tendon tension (stand/trot) | ~30–160 N (headline ~55 N) | `[sourced: LEG_TENDON_SPEC §1.3]` |
| Leg **land transient** tendon tension (rare, structural) | ~470 N hip / ~510 N knee (~525 N design) | `[sourced: LEG_TENDON_SPEC §1.3]` |
| Motor spool radius `r_spool` | 0.008 m | `[sourced: LEG_TENDON_SPEC §5]` |
| Spine continuous tendon tension | ~20–70 N band target | `[sourced: SPINE_TAIL_SPEC §1.5]` |
| Per-leg static-hold **brake** (power-off, spring-engaged) | ~1.3 N·m spool torque, needs a driver channel + fail-safe | `[owed→me: LEG_TENDON_SPEC §4 handoff]` |

**Derived motor-shaft torque** (τ_motor = T · r_spool), the real driver-sizing input:

| Regime | Tendon T | τ_motor = T·0.008 | Note |
|---|---|---|---|
| Continuous (trot) | ~55–160 N | **~0.44–1.28 N·m** | thermal / continuous-current duty |
| Land transient (leg) | ~510 N | **~4.1 N·m** | brief peak; sets peak (pulse) current, not thermal |
| Spine continuous | ~20–70 N | ~0.16–0.56 N·m | lighter than legs |

Phase current is `I_phase ≈ τ_motor / Kt`. **Without `Kt` (G-Kt) this cannot be
turned into amps**, so the power stage is specified by *class* (see §1.2, §5) and
the FET/shunt exact values are `[owed]` pending the actuator down-select.

---

## 1. Per-motor smart-driver board

One identical board per tendon motor (ADR-0005: "one FOC smart driver per tendon
motor"). Identical BOM/layout across legs, spine, and tail keeps the fleet to a
single board design; per-cluster differences are handled at the backplane, not
the driver.

### 1.1 Block diagram

```
        MOTOR BUS (Vbus, e-stop-switched)          CAN-FD SEGMENT
                 │                                    │  (H/L pair + shield)
                 │                                    │
   ┌─────────────┼────────────────────────────────────┼─────────────────────┐
   │  SMART DRIVER BOARD                                │                     │
   │             │                          ┌───────────┴──────────┐          │
   │   ┌─────────▼─────────┐                │  CAN-FD transceiver   │          │
   │   │ bulk cap + input   │                │  (5 Mbit/s data)      │          │
   │   │ filter / reverse   │                └───────────┬──────────┘          │
   │   │ prot. / soft-start │                            │ TXD/RXD             │
   │   └─────────┬─────────┘              ┌──────────────▼───────────────┐     │
   │             │  Vbus                  │      FOC-capable MCU          │     │
   │   ┌─────────▼─────────┐   PWM x6     │  - FOC current loop (≥ tens   │     │
   │   │ 3-phase GATE DRIVER│◀────────────│    of kHz PWM, ≥1 kHz outer)  │     │
   │   │ (3x half-bridge)   │             │  - CAN-FD MAC (firmware F1)   │     │
   │   └─────────┬─────────┘   fault ────▶│  - Tier-A safety state machine│     │
   │             │ gate                   │  - on-chip ADC (i-sense, temp)│     │
   │   ┌─────────▼─────────┐              └───▲────▲────▲────────▲────────┘     │
   │   │  FET BRIDGE        │ phase A/B/C      │    │    │        │             │
   │   │  (3x half-bridge,  │──────────────▶ MOTOR   │    │        │             │
   │   │   Vds/Ipk per G-   │                  │      │    │        │            │
   │   │   Vbus/G-Kt)       │                  │      │    │        │            │
   │   └─────────┬─────────┘                   │      │    │        │            │
   │   ┌─────────▼─────────┐  phase-current    │      │    │        │            │
   │   │ CURRENT SENSE      │──────────────────┘      │    │        │            │
   │   │ (3x low-side or    │  (FOC + Tier-A OC +     │    │        │            │
   │   │  inline shunt+amp) │   optional tension est.)│    │        │            │
   │   └───────────────────┘                          │    │        │            │
   │   ┌───────────────────┐  angle (SPI/ABI/BiSS)    │    │        │            │
   │   │ ROTOR SENSOR I/F   │──────────────────────────┘    │        │            │
   │   │ absolute encoder   │                                │        │            │
   │   │ (Hall floor)       │                                │        │            │
   │   └───────────────────┘                                │        │            │
   │   ┌───────────────────┐  tension (mV)                  │        │            │
   │   │ TENSION FRONT-END  │────────────────────────────────┘        │            │
   │   │ *POPULATE-OPTIONAL*│  (load-cell instrumentation amp;         │            │
   │   │ (ADR-0004 open)    │   omit if current-estimate path chosen)  │            │
   │   └───────────────────┘                                          │            │
   │   ┌───────────────────┐  temp (NTC x2: FET + motor)              │            │
   │   │ THERMAL SENSE      │──────────────────────────────────────────┘            │
   │   └───────────────────┘                                                        │
   │   ┌───────────────────┐  release/hold                                          │
   │   │ BRAKE DRIVER (opt) │◀── leg boards only: power-off friction brake          │
   │   │ low-side + flyback │    (spring-engaged, fail-safe holds on power loss)     │
   │   └───────────────────┘                                                        │
   │   ┌───────────────────┐                                                        │
   │   │ LOCAL POWER        │  Vbus → gate-drive rail (~12 V) → logic (3.3 V)        │
   │   │ regulation         │  + isolated/derived CAN rail                           │
   │   └───────────────────┘                                                        │
   └────────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Block-by-block spec

1. **FOC-capable MCU.** Single MCU runs the FOC current loop, the CAN-FD MAC, and
   the Tier-A safety state machine. Needs: ≥1 timer-driven 3-phase PWM unit with
   dead-time + fault-input trip, a fast multi-channel ADC (phase current + bus +
   temps) with PWM-synchronous sampling, a hardware CAN-FD peripheral, and enough
   headroom for the ≥1 kHz outer loop (NFR3). Class in §5. Firmware owns the loop
   code and the F1 message handling; electronics guarantees the peripherals exist.

2. **3-phase gate driver + FET bridge (parameterized power stage).**
   - Topology: 3× half-bridge (6 FETs), driven by a 3-phase gate driver with
     built-in dead-time and a **fault/enable line into the MCU** for Tier-A trip.
   - **Voltage class `[GAP: G-Vbus]`:** placeholder motor bus **24 V** `[assumed]`
     (6S Li-ion, mid-range for cat-scale FOC drivers such as moteus/Mini-Cheetah).
     FET Vds ≥ 2× bus for switching margin → **40 V class** at 24 V bus; re-rate if
     the pack lands at 36–48 V.
   - **Current class `[GAP: G-Kt]`:** must carry the trot **continuous** duty
     (τ_motor ~0.44–1.28 N·m) and survive the **land pulse** (~4.1 N·m). With a
     representative `Kt = 0.06 N·m/A` `[assumed, illustrative only]`: continuous
     phase ~7–21 A, land-pulse phase ~68 A. So the stage targets **~15–25 A
     continuous / ~60–80 A pulse** as a placeholder envelope — i.e. the
     ODrive/moteus/VESC-mini class. **These amps are illustrative; they move
     directly with the real `Kt`.** The land peak is a *transient* (structural per
     LEG_TENDON_SPEC), so it sizes pulse rating and copper, not continuous thermal.
   - Bulk capacitance on the local bus for PWM ripple + regen; reverse-polarity and
     soft-start (inrush) protection at the board input.

3. **Rotor position sensor interface (ADR-0004).** Absolute encoder **preferred**
   (a static high-torque hold cannot run sensorless — ADR-0004). Provide a digital
   sensor interface (SPI / BiSS-C / SSI for an on-axis magnetic absolute encoder,
   plus ABI incremental) **and** a **Hall-sensor input (the floor)** so a
   Hall-only motor is still commutatable. Keep this rotor sensor **electrically
   distinct** from the tendon/joint-state sensor (ADR-0004: different loops).

4. **Per-tendon tension front-end (ADR-0004, method OPEN — G-Tens).** Two paths on
   the board, so either ADR-0004 outcome ships without a respin:
   - **Current-estimate path (always populated):** the FOC phase-current sense is
     reused to estimate tendon tension via motor torque. Cheap, no extra parts,
     but friction-corrupted (LEG_TENDON_SPEC capstan factor up to ~1.87×).
   - **Load-cell path (populate-optional):** an **instrumentation-amplifier
     front-end** (in-amp + reference + filter into the MCU ADC, or a dedicated
     bridge/load-cell AFE) for an in-line tendon load cell. Highest accuracy,
     adds parts/space. Footprints laid but DNP until ADR-0004 chooses.
   - Series-elastic + displacement (third ADR-0004 option) would instead consume
     the rotor/aux sensor input; no dedicated analog front-end needed.
   - **Firmware handoff:** current-sense scaling (shunt Ω × in-amp gain → A/count)
     and, if fitted, load-cell scaling (mV/V × in-amp gain → N/count) are exported
     to firmware once component values are set (blocked on G-Kt / G-Tens).

5. **CAN-FD transceiver.** One transceiver per board onto the local CAN-FD segment
   (~5 Mbit/s data phase, ADR-0005). Node addressing, framing, and timing follow
   the **firmware F1 schema** — not defined here. Include bus-fault protection and
   an optional termination-populate footprint (only the two physical ends of a
   segment are terminated).

6. **Tier-A safety latches (ADR-0005 tier A — per-driver, autonomous).**
   Local, hardware-fast, independent of the RT controller and SBC:
   - **Over-current:** comparator on the current-sense → gate-driver fault/enable
     trip (hardware path), with MCU-level backup. Threshold above the land pulse,
     below FET SOA.
   - **Over-tension:** MCU compares the tension estimate/load-cell reading against
     a per-tendon limit (structural ~525 N / ~1 kN component design load) → commands
     zero-torque.
   - **Thermal:** NTC on the FET bridge **and** on the motor body → MCU derate/trip
     (supports the LEG_TENDON_SPEC thermally-derated static-hold point).
   A Tier-A trip drives the bridge to the **safe (high-Z / zero-torque limp)**
   state locally; it is reported up via the F1 schema but does not *depend* on it.

7. **Brake driver (leg boards only, populate-optional).** LEG_TENDON_SPEC §4 hands
   electronics a **girdle-mounted, power-off (spring-engaged, electrically
   released) friction brake per leg motor**, ~1.3 N·m spool torque. Board provides
   a low-side switch + flyback for the release coil. **Fail-safe by construction:**
   loss of board/bus power → coil de-energized → brake engaged → a perched cat does
   not collapse. Spine/tail boards leave this DNP.

8. **Local power regulation.** Motor bus → gate-drive rail (~10–12 V) → logic
   (3.3 V) → sensor/CAN rails. Bus voltage class is `[GAP: G-Vbus]`.

### 1.3 Electrical interface exported to firmware (once gaps close)

- Current-sense scaling (A per ADC count), bus-voltage scaling.
- Rotor-sensor protocol + electrical (SPI/BiSS/ABI/Hall levels, 3.3 V logic).
- Tension scaling (load-cell path) or the current→tension estimate contract.
- Brake output polarity + fail-safe semantics.
- CAN-FD PHY parameters (bit timing envelope); **message content is firmware F1**.
- Tier-A trip thresholds and the safe-state definition (zero-torque limp).

---

## 2. Channel count & clustering

Driver count follows the DOF × antagonistic factor (ADR-0002) plus the tail.
Headline figures use the antagonistic-everywhere upper bound the lead specified;
the mechanical specs offer two documented **reductions** noted below.

### 2.1 Driver channel count

| Group | Breakdown | Drivers (headline) | Documented reduction |
|---|---|---|---|
| **Legs** | 4 legs × 3 joints × 2 (antagonistic) | **24** | ADR-0002 / LEG_TENDON_SPEC reserve the **ankle as single-tendon + return spring** → 1 motor not 2 → **~20** (hip 2 + knee 2 + ankle 1 per leg × 4) |
| **Spine** | 3 segments × dorsoventral pair (Option B, 6 tendons) | **6** | SPINE_TAIL_SPEC variable-radius pulley → 1 motor/DOF → **3** |
| **Tail** | single tendon (tension/loosen) + passive return | **1** | no antagonist / no telescope (ADR-0007) |
| **Total** | | **~31** | reductions → as low as **~24** |

**Recommendation to lead:** budget the **~31** upper bound for board fleet,
power, and CAN loading (it is the stressing case), but track the ~24 reduced case
for mass/cost. **The ankle spring-return and the spine variable-radius pulley are
the two biggest levers on channel count** and both are still open — confirm before
committing a backplane channel count. (The tail is now a single motor, ADR-0007.)

### 2.2 Physical clustering (ADR-0005: girdles + tail node)

Motors are centralized in the girdles/pelvis (P1), so drivers cluster with them:

| Cluster | Motors it hosts | Driver count (headline / reduced) |
|---|---|---|
| **Shoulder-girdle cluster** | 2 forelegs | 12 / 10 |
| **Pelvic-girdle cluster** | 2 hind legs + spine bank + tail | 12 + 6 + 1 = **19 / 14** |
| *(Tail node)* | 1 tail motor (mounted at pelvic girdle per SPINE_TAIL_SPEC) | 1 — its own CAN **node ID**, sharing the pelvic segment; physically inside the pelvic girdle |

The pelvic girdle is the density hot-spot (hind legs + all spine + tail motors
live there per SPINE_TAIL_SPEC). Thermal path and power distribution there are the
tightest — flag to mechanical for airflow/heat-sinking.

### 2.3 CAN-FD segmentation (~6–8 segments, ADR-0005)

ADR-0005 fixes **~6–8 CAN-FD segments** (extrapolated from the mjbots 12-axis
result; ADR-0005 flags **bench-verify ≥1 kHz per segment** at final axis count).
At ~31 drivers that is **~4–6 drivers per segment** — comfortably under the
12-axis-per-bus reference, giving margin for the ≥1 kHz aggregation (NFR3):

| Segment (indicative) | Drivers | Notes |
|---|---|---|
| Shoulder A | left foreleg (~5–6) | |
| Shoulder B | right foreleg (~5–6) | |
| Pelvic A | left hind leg (~5–6) | |
| Pelvic B | right hind leg (~5–6) | |
| Spine | spine bank (3–6) | |
| Tail | tail node (2–3) | |
| (spare ×1–2) | headroom / split a heavy segment | keeps ≤8 |

Grouping by limb keeps a single fault/segment loss to one limb, and keeps each
segment's node count low enough to hold the ≥1 kHz cycle. Final split is
**bench-verified**, per the ADR-0005 caveat.

---

## 3. Girdle backplane / cluster board

One backplane per girdle (shoulder, pelvic; the tail node hangs off the pelvic
backplane as its own segment). The backplane is **not** a smart board — it is
power distribution + bus fan-out + the hardware e-stop, so the driver boards stay
identical and hot-swappable.

### 3.1 Functions

1. **Power distribution.** Takes the e-stop-switched motor bus in, fans Vbus out to
   each driver slot with per-slot (or per-pair) fusing/e-fuse and local bulk
   capacitance. Star/bus-bar layout sized to the cluster current budget (§4).
2. **CAN-FD segmentation + fan-out.** Carries the segment(s) for that cluster,
   with correct termination at the two physical ends only, and a stub-length
   budget appropriate to 5 Mbit/s. Connectorized to the driver slots.
3. **Multi-segment CAN-FD bridge to the RT controller.** A **bridge node** (small
   MCU or multi-CAN-FD controller) aggregates this girdle's segments up to the
   real-time controller. ADR-0005 puts the RT controller (PREEMPT_RT core or
   dedicated MCU) as the ≥1 kHz aggregation + tendon-map/AIC + safety supervisor
   (Tier C). The bridge presents each segment to the RT tier per the **firmware F1
   schema**. (EtherCAT is the ADR-0005 documented upgrade path if CAN-FD
   determinism becomes limiting — the bridge is where that swap would land.)
4. **Tier-B hardware e-stop (see §3.2).**

### 3.2 Tier-B hardware e-stop (ADR-0005 tier B)

**Independent of the SBC and the RT controller** — this is the whole point of
Tier B. The e-stop cuts the **motor-bus power** to the driver cluster (and/or
forces the bridges to command all-limp), driving the robot to **zero-torque
limp**. It must not rely on any software running:

```
  E-STOP sources (any trips):
    - physical button (NC loop, latching)
    - RT-controller "healthy" line (Tier-C watchdog heartbeat; absence trips)
    - operator/host remote (via a hardwired relay driver, not a CAN message)
              │
              ▼
    ┌───────────────────────────┐
    │  E-STOP LATCH (hardware,   │   fail-open: de-energized = tripped
    │  NC safety loop)           │
    └──────────────┬────────────┘
                   │ enable
                   ▼
    ┌───────────────────────────┐   MOTOR BUS ONLY — logic/CAN stays powered
    │  MOTOR-BUS CONTACTOR /     │   so drivers can report the fault & the
    │  high-side power switch    │   RT tier stays observable
    └──────────────┬────────────┘
                   ▼  Vbus(switched) → per-cluster distribution (§3.1)
```

Key properties:
- **Cutting motor-bus power** forces the FET bridges into passive/high-Z → motors
  freewheel (zero-torque limp), the ADR-0005 safe state.
- **Logic + CAN rails stay powered** (separately regulated, upstream of the
  contactor) so drivers still report *why* they tripped and the RT watchdog stays
  live — the estop removes torque, not observability.
- **Fail-open NC loop:** any break (button, broken wire, dead RT heartbeat) trips.
- The per-leg **power-off brakes** (§1.2 item 7) engage on the same power loss →
  complementary mechanical fail-safe.
- The SBC is **never** in this path (ADR-0005: SBC not safety-critical).

Interaction with the three tiers: Tier A (per-driver latch) handles single-motor
faults locally; **Tier B (this circuit)** is the cluster/whole-robot torque kill;
Tier C (RT-supervisor watchdog, firmware) drives the "healthy" line into this
latch. Electronics owns A's hardware hooks and all of B; firmware owns A's logic
and C.

---

## 4. Power

### 4.1 Battery / bus voltage — `[GAP: G-Vbus]`

No pack chosen. **Placeholder: 24 V nominal motor bus (6S Li-ion)** `[assumed]`,
consistent with cat-scale FOC drivers (moteus/Mini-Cheetah run ~24 V; some to
44 V). A **separate, always-on logic/CAN rail** derived upstream of the e-stop
contactor (or from a small independent regulator) keeps observability during an
estop. The bus voltage drives FET Vds class, gate-drive rail, and bulk-cap
voltage — all `[owed]` until the pack lands.

### 4.2 Per-cluster current budget

Bus current is dominated by **copper loss at low-speed high-torque holds** (mech
power ≈ torque×speed ≈ 0 at a hold, so draw ≠ mechanical power). Without `Kt` and
winding R (G-Kt) this is an estimate, not a computed value:

| Cluster | Continuous (trot, brakes released) | Peak note |
|---|---|---|
| Shoulder (12 drivers) | ~20–40 A @ 24 V `[assumed, G-Kt]` | Not all peak together |
| Pelvic (19 drivers) | ~30–60 A @ 24 V `[assumed, G-Kt]` | densest cluster |

**Why the peak is not the sum of peaks:** the ~510 N land transient is a
**single-leg** event (LEG_TENDON_SPEC) — only that leg's hip+knee agonists
(~2–4 motors) hit pulse current at once, not the whole cluster. So distribution
copper/e-fuses size to *localized* pulse + *cluster-wide* continuous, not
Σ(peak). Stance uses the **power-off brakes** to offload holding current
entirely, cutting the true continuous draw further.

Cluster budgets and bus-bar/fuse sizing are placeholders pending G-Kt/G-Vbus and
the real duty cycle from firmware's control profile.

### 4.3 Regulation

- Motor bus: battery → e-stop contactor → per-cluster distribution + bulk caps.
- Per-driver local rails: gate-drive (~10–12 V) and logic (3.3 V) derived on each
  driver board from switched Vbus.
- Logic/CAN rail: separately regulated **upstream of the estop contactor** so it
  survives a motor-bus kill (observability during limp).
- Brake coils: fed from switched Vbus (fail-safe engage on power loss).

---

## 5. Parts-class BOM (classes / candidates — NOT part numbers)

Exact part numbers are `[owed]` until G-Kt (current) and G-Vbus (voltage) close.

| Function | Class / candidate family | Selection driver | Gap |
|---|---|---|---|
| FOC MCU | 32-bit MCU with 3-phase timer + PWM-sync ADC + hardware **CAN-FD** (e.g. STM32G4 / STM32H5 class, or equivalent motor-control MCU) | needs FOC loop + CAN-FD MAC + Tier-A logic on one die | — |
| Gate driver | 3-phase gate driver, integrated dead-time + fault/enable, Vgs for chosen FETs | FET choice, bus voltage | G-Vbus |
| Power FETs | Power MOSFET half-bridge, **Vds class ≈ 2× bus (40 V @ 24 V bus)**, RDS(on)/SOA for ~15–25 A cont / ~60–80 A pulse | land-pulse SOA + trot thermal | G-Kt, G-Vbus |
| Current sense | Low-side or inline **shunt + current-sense amp** (or integrated sense in gate driver), ×3 phases | FOC + Tier-A OC + current-estimate tension | G-Kt |
| Rotor sensor | **On-axis magnetic absolute encoder** (SPI/BiSS/SSI + ABI), e.g. AS5047/MA-class; **Hall sensors as the floor** | ADR-0004 (absolute preferred, Hall floor) | motor mounting |
| Tension (load-cell path, DNP-optional) | **Instrumentation amp / bridge AFE** (e.g. INA-class in-amp or dedicated load-cell/bridge AFE) into MCU ADC | ADR-0004 method | G-Tens |
| CAN-FD transceiver | CAN-FD PHY rated ≥5 Mbit/s with bus-fault protection | ADR-0005 bus | — |
| Thermal sense | 2× NTC (FET bridge + motor body) | thermal Tier-A + static-hold derate | — |
| Brake driver (leg only, DNP) | Low-side switch + flyback for release coil | LEG_TENDON_SPEC §4 brake | brake PN |
| Backplane bridge | Small MCU or multi-CAN-FD controller | segment → RT-tier aggregation (firmware F1) | — |
| E-stop switch | **Motor-bus contactor / high-side power switch** + NC latching safety-loop logic | Tier-B kill current = cluster bus current | G-Vbus, §4.2 |

---

## 6. Handoffs & open items

**→ tomcat-firmware** (interface, once gaps close):
- Current-sense scaling (A/count), tension scaling or current→tension estimate
  contract, rotor-sensor protocol/levels, brake polarity + fail-safe semantics,
  Tier-A trip thresholds + safe-state (zero-torque limp) definition.
- Electronics **consumes** the firmware **F1 CAN-FD schema** as the
  driver↔RT-controller contract; it is not redefined here.
- Confirm the ≥1 kHz-per-segment bench verification plan (ADR-0005 caveat) so the
  final segment split (§2.3) is validated, not just extrapolated.

**→ lead / tomcat-mechanical** (blocking gaps):
- **G-Kt:** actuator/motor down-select (ADR-0003 keeps it open) — blocks all
  current sizing.
- **G-Vbus:** battery/pack + bus voltage — blocks FET voltage class + regulation.
- **G-Tens:** close ADR-0004 tension method (load cell vs. current estimate vs.
  series-elastic) — decides whether the load-cell front-end is populated.
- **Channel-count confirmations:** ankle spring-return (24→~20 legs) and spine
  variable-radius pulley (6→3) — confirm before a backplane channel count is cut.
- Pelvic-girdle thermal/airflow: it hosts hind legs + all spine + tail drivers
  (density hot-spot).

---

## Summary (for the lead)

**Boards defined:**
1. **Per-motor smart-driver board** (one design, whole fleet): FOC MCU · 3-phase
   gate driver + FET bridge (parameterized power stage) · rotor-sensor interface
   (absolute preferred, Hall floor) · dual tension path (current-estimate always +
   load-cell in-amp DNP) · CAN-FD transceiver · Tier-A OC/OT/thermal latches ·
   optional fail-safe brake driver (leg boards).
2. **Girdle backplane** (shoulder + pelvic): power distribution, CAN-FD
   segmentation + multi-segment bridge to the RT controller, and the **Tier-B
   hardware e-stop** (motor-bus contactor, fail-open NC loop, logic/CAN stays live).

**Driver channel count / clustering:** legs 24 (→~20 if ankle spring-return) +
spine 6 (→3 with variable-radius pulley) + tail 1 = **~31** (down to ~24
reduced). Clusters: shoulder girdle ~12, pelvic girdle ~19 (hind legs + spine +
1 tail motor), tail as its own node ID on the pelvic segment. **~6–8 CAN-FD
segments**, ~4–6 drivers each, grouped by limb; segment count is
bench-verify-pending per ADR-0005.

**Key electrical gaps blocking detailed sizing:** (1) **motor `Kt`** → no
phase-current → no FET/shunt/thermal sizing; (2) **bus voltage** → no FET Vds
class / regulation; (3) **ADR-0004 tension method** → load-cell front-end
populate-or-not. All three are `[owed]` to the lead / actuator down-select.
