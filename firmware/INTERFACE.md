# TomCat — Control Interface Schema (M1 task F1)

Status: **DRAFT for review** (schema v0.1). Interface/schema deliverable only —
no real-time control logic in this pass. Owned by `firmware/`. Electronics
references this doc for the CAN-FD wire format; kinematics owns the field
semantics on the setpoint side.

This defines the two interface layers fixed by **ADR-0005** (distributed CAN-FD
smart drivers + RT/SBC two-tier split + three safety tiers). The C declarations
live in [`include/tomcat_can_schema.h`](include/tomcat_can_schema.h); this doc is
the rationale, byte budget, and safety/timing contract.

```
  Host/Operator ──ROS2──▶ SBC (ROS 2, >=100 Hz planning/righting)
                              │  LAYER 1  (setpoints down / state up)
                              ▼
                          RT controller (>=1 kHz aggregation + tendon-map/AIC
                              │           fast path + Tier-C watchdog)
                              │  LAYER 2  (CAN-FD ~5 Mbit/s, ~6-8 segments)
              ┌───────────────┼───────────────┐
              ▼               ▼               ▼
        smart driver    smart driver    smart driver   (one per tendon motor;
        (FOC + tension + Tier-A latch + rotor sensor)    ADR-0004 front-end)
```

The **hardware e-stop (Tier B)** sits outside both layers: it cuts motor-bus
power / forces zero-torque limp independent of the SBC and RT controller
(ADR-0005). The schema only *reports* Tier B; it never gates it.

---

## 0. Conventions

- SI units on the logical side; compact fixed-point on the wire (LSBs below).
- Wire byte order: **little-endian** by convention (revisit if a big-endian MCU
  is picked — ADR-0003 open). Structs are laid out naturally aligned so
  `sizeof` == documented byte count with no packing pragma.
- **Fail safe by default:** any unknown mode, invalid calibration, stale link,
  or unhandled fault drives the **limp** state (de-energize, zero torque, cables
  compliant). Limp is never bypassed for convenience.
- "Motor-side" tension = tension the motor/spool must supply (matches
  `TendonSolution.motor_tension_flexor/extensor`, i.e. joint-side amplified by
  the capstan factor). The tendon-map friction/stretch model and the joint-side
  vs motor-side conversion live in kinematics + the RT tier; **drivers only see
  motor-side quantities.**

### Fixed-point LSBs (wire units)

| Quantity   | Wire type | LSB            | Range              |
|------------|-----------|----------------|--------------------|
| angle      | i32       | 1e-4 rad       | ±2.1e5 rad         |
| tension    | u16       | 0.01 N         | 0 .. 655.35 N ⚠    |
| current    | i16       | 1 mA           | ±32.767 A          |
| temperature| i16       | 0.1 °C         | ±3276.7 °C         |
| bus voltage| u16       | 0.01 V         | 0 .. 655.35 V      |
| stiffness k| u16       | 0.1 N/m        | 0 .. 6553.5 N/m ⚠  |

⚠ tension and stiffness carry open scaling assumptions — see §6.

---

## 1. Layer 1 — SBC ↔ RT controller

Transport is **not** CAN and is left open (shared memory, SPI, or Ethernet —
ADR-0005 follow-up); this layer defines the logical message contract only. Both
messages are flat arrays indexed by **logical motor id** (§3).

### 1.1 Setpoints down — `tomcat_l1_command_t`

Per-motor (`tomcat_l1_motor_cmd_t`), aligned field-for-field with the kinematics
`TendonSolution` so the tendon map's output drops straight in:

| Field             | Units / type      | Source in TendonSolution             |
|-------------------|-------------------|--------------------------------------|
| `mode`            | enum              | control-mode selection (see §1.3)    |
| `position_target` | rotor angle, rad  | `TendonMap.motor_angles(q)`          |
| `tension_target`  | motor-side N      | `motor_tension_flexor/extensor`      |
| `t_bias`          | N                 | `TendonSolution.t_bias`              |
| `stiffness_k`     | N/m               | AIC agonist gain `k` (ADR-0002)      |
| `flags`           | bits              | bit0 enable                          |

Frame header carries a monotonic `seq` (drives the Tier-C SBC watchdog),
`schema_major/minor`, a `request` word (arm / request-limp / clear-faults), and
`n_motors`.

**On the `T_bias` / stiffness split (ADR-0002):** the *static* AIC tension split
(antagonist held at `T_bias`, agonist at `T_bias + |tau|/r`) is already realized
by the kinematics tendon map, which emits per-tendon `tension_target`. The
*dynamic* stiffness gain `k` (tension per cable-length error) is the firmware
concern noted in `tendon.py`; it is carried here as `stiffness_k` so the RT tier
can close the co-contraction stiffness loop when `mode == HYBRID`.

### 1.2 State up — `tomcat_l1_telemetry_t`

Per-motor (`tomcat_l1_motor_state_t`): `rotor_angle`, `tension_meas`, `current`,
`temperature`, `fault_flags` (§4), actual `mode`. Frame header adds `seq`,
`rt_time_us`, `rt_state` (BOOT/IDLE/ACTIVE/LIMP/ESTOP/FAULT), `limp_reason`, and
a `fault_summary` (OR of all motor faults) so the SBC can react without scanning
the whole array.

### 1.3 Control modes

`LIMP` (0) / `POSITION` (1) / `TENSION` (2) / `HYBRID` (3). HYBRID is the
antagonistic co-contraction mode: position tracking plus the `T_bias`/`k`
stiffness law. Any value the driver does not recognize → LIMP.

---

## 2. Layer 2 — RT controller ↔ smart drivers (CAN-FD)

CAN-FD at ~5 Mbit/s data phase (nominal/arbitration ~1 Mbit/s), split across
**~6–8 segments** (one CAN-FD controller per segment on the RT bridge). Node ids
are unique *within* a segment; the RT tier maps `(segment, node)` → logical
motor id (§3).

### 2.1 Addressing

11-bit identifier: `ID = (FUNC << 7) | NODE_ID`. Lower numeric ID = higher
arbitration priority, so safety/sync win the bus. `NODE_ID` 1..126, `0x7F` =
broadcast.

| FUNC | Name       | Dir           | Purpose                              |
|------|------------|---------------|--------------------------------------|
| 0x0  | SAFETY     | RT→all (bcast)| forced limp / e-stop (top priority)  |
| 0x1  | SYNC       | RT→all (bcast)| 1 kHz cycle trigger + arm/estop bits |
| 0x2  | COMMAND    | RT→driver     | per-motor setpoint                   |
| 0x3  | TELEM      | driver→RT     | per-motor state                      |
| 0x4  | CFG_WR     | RT→driver     | low-rate limit/calibration write     |
| 0x5  | CFG_ACK    | driver→RT     | config ack / info                    |

29-bit extended IDs are the documented fallback if a segment ever needs >126
nodes or a wider function space (not required at the current axis count).

### 2.2 Frames and per-frame byte budget

| Frame    | FUNC   | Payload | CAN-FD DLC | Rate       |
|----------|--------|---------|------------|------------|
| SAFETY   | 0x0    | 4 B     | 4          | event      |
| SYNC     | 0x1    | 8 B     | 8          | 1 kHz      |
| COMMAND  | 0x2    | 16 B    | 16         | 1 kHz/node |
| TELEM    | 0x3    | 20 B    | 20 (pad)   | 1 kHz/node |
| CFG_WR   | 0x4    | 8 B     | 8          | on change  |
| CFG_ACK  | 0x5    | 8 B     | 8          | on change  |

COMMAND payload (16 B): `seq`(1) `mode`(1) `flags`(1) `rsvd`(1)
`position_target`(4) `tension_target`(2) `t_bias`(2) `stiffness_k`(2) `rsvd`(2).

TELEM payload (18 B used, 20 B DLC): `seq`(1) `mode_status`(1) `fault_flags`(2)
`rotor_angle`(4) `tension_meas`(2) `current`(2) `temperature`(2) `vbus`(2)
`rsvd`(2). All comfortably inside CAN-FD's 64-byte frame — >3× headroom per
motor, so an optional **grouped** command/telemetry frame (pack ~4 motors into
one 64-B frame) is a future bus-load optimization without a schema change.

### 2.3 Bus-load / segment-count check (⚠ bench-verify — ADR-0005)

Per-cycle traffic on a segment with `n` nodes at 1 kHz = 1×SYNC + n×COMMAND +
n×TELEM. Using a first-order CAN-FD frame-time estimate (arbitration/header
~25 µs at 1 Mbit/s incl. worst-case stuffing; data+CRC at 5 Mbit/s):

- SYNC (8 B) ≈ 45 µs, COMMAND (16 B) ≈ 57 µs, TELEM (20 B) ≈ 63 µs.
- Cycle time ≈ 45 + n·(57 + 63) = **45 + 120·n µs**.
- 1 kHz budget (1000 µs) → **n ≤ ~7.9 nodes/segment** before saturation.

So ~33 motors at a comfortable **5–6 nodes/segment → 6–7 segments**, matching
ADR-0005's ~6–8. **This is a paper estimate**: real arbitration bit timing, bit
stuffing, controller/DMA latency, and margin must be **bench-verified at final
axis count** (ADR-0005 explicitly flags the segment count as an extrapolation
from the mjbots 12-axis result). Grouped frames (§2.2) raise the per-segment node
ceiling if measurement demands it; EtherCAT is the ADR-0005 upgrade path.

### 2.4 Config / calibration (CFG_WR)

Low-rate, keyed writes for Tier-A latch thresholds (tension/current/temp), the
driver rx-watchdog window, position limits, and — critically — the **open**
ADR-0004 tension transfer (`TENSION_SCALE`) and motor `Kt`. These are the two
gaps (§6) parked behind a config key so the wire format is stable before the
values are known.

---

## 3. Logical motor map

~33 motors: **24 leg** (4 legs × {hip, knee, ankle} × {flexor, extensor}) +
**6 spine** (3 sagittal segments × 2; grows with the lateral/axial DOF of
ADR-0006) + **tail** (placeholder, ADR-0007). Each smart driver drives exactly
one tendon (one side of an antagonistic pair). The RT tier holds the mapping:

```
logical_id  ──▶  (girdle/tail cluster, CAN segment, node_id)  ──▶  physical driver
```

Concrete node/segment assignment depends on the girdle backplane layout owned by
electronics; the schema fixes only the addressing scheme and the ≤ ~6 nodes/
segment budget that keeps each segment ≥1 kHz (§2.3).

---

## 4. Fault-flag bitfield (`tomcat_fault_t`, 16-bit)

Identical at both layers; drivers set it, RT aggregates it, SBC reads it.

| Bit | Flag             | Tier / meaning                                    |
|-----|------------------|---------------------------------------------------|
| 0   | OVERCURRENT      | **A** — driver-local latch                        |
| 1   | OVERTENSION      | **A** — driver-local latch                        |
| 2   | OVERTEMP         | **A** — driver-local latch                        |
| 3   | ROTOR_SENSOR     | encoder/Hall loss (ADR-0004) → cannot commutate   |
| 4   | TENSION_SENSOR   | tension front-end fault → tension loop unsafe     |
| 5   | CAN_TIMEOUT      | **C** (distributed) — driver rx-watchdog expired  |
| 6   | DRIVER_FAULT     | gate driver / stage fault                         |
| 7   | ESTOP_ACTIVE     | **B** — hardware e-stop asserted (reported only)  |
| 8   | LIMP_ACTIVE      | currently in limp                                 |
| 9   | WATCHDOG_TRIP    | **C** — RT-supervisor watchdog fired              |
| 10  | CAL_INVALID      | no valid tension/Kt cal loaded → fail safe        |
| 11  | BUS_OFF          | CAN controller bus-off                            |
| 12  | OVERVOLTAGE      | bus over/under-voltage                            |
| 13  | NOT_HOMED        | rotor position not referenced                     |
| 14  | SETPOINT_CLAMP   | command exceeded a limit and was clamped          |
| 15  | reserved         |                                                   |

Any fault bit above `LIMP_ACTIVE` that was not explicitly commanded implies the
driver has autonomously entered (or is being held in) limp.

---

## 5. Limp / e-stop semantics and the three safety tiers

**Limp state** = motor de-energized / zero commanded torque so the tendon goes
slack and the body becomes compliant. It is the single fail-safe target of every
tier and of every unknown/fault condition.

- **Tier A — per-driver latch (local, autonomous).** Each smart driver latches
  over-current / over-tension / over-temp against its CFG_WR thresholds and
  drops itself to limp within its own control cycle, setting the matching fault
  bit. Latches require an explicit `clear_faults` (Layer-1 `request` bit /
  COMMAND `flags` bit1) and are never auto-cleared. Independent of the RT loop.
- **Tier B — hardware e-stop (out of band).** Cuts motor-bus power / forces
  zero-torque limp independent of SBC and RT (ADR-0005). The schema cannot enable
  or defeat it; drivers that still have logic power report `ESTOP_ACTIVE`, and RT
  advertises `RT_ESTOP` + `LIMP_REASON_ESTOP_HW`. Recovery is operator-gated.
- **Tier C — RT-supervisor watchdog (system-level).** Two arms:
  1. *Downstream (distributed):* every SYNC/COMMAND carries a `seq`; each driver
     runs an **rx-watchdog** (`CFG_RX_TIMEOUT_US`, baseline **3 ms ≈ 3 missed
     1 kHz cycles**). On expiry the driver self-limps and sets `CAN_TIMEOUT`.
     This survives an RT crash or a cut segment.
  2. *Upstream:* the RT tier tracks each driver's TELEM `seq`; a missing driver
     heartbeat (baseline **3–5 ms**) → `WATCHDOG_TRIP`, RT broadcasts a SAFETY
     limp and reports `LIMP_REASON_DRIVER_LOST`.

The SBC is **never** in the safety-critical path. RT also guards the SBC link:
Layer-1 `seq` gaps beyond the **SBC-stale window (baseline 20 ms ≈ 2 missed
100 Hz updates)** → RT drives all motors to limp (`LIMP_REASON_SBC_STALE`).

### Heartbeat / watchdog timing summary

| Link                 | Nominal rate | Stale/limp window (baseline, TBD) |
|----------------------|--------------|-----------------------------------|
| SBC → RT (Layer 1)   | ≥100 Hz      | 20 ms → RT limps all              |
| RT → driver SYNC     | 1 kHz        | driver rx-wdog 3 ms → self-limp   |
| driver → RT TELEM    | 1 kHz        | 3–5 ms → RT SAFETY limp           |

All windows are baselines to be tuned on the bench with the verified segment
timing (§2.3).

---

## 6. Open scaling / units gaps (flagged, not resolved here)

1. **Tension LSB → Newton transfer (ADR-0004 open).** The tension method is
   still Proposed (in-line load cell vs. motor-current estimate vs. series
   elastic). The wire unit is fixed (0.01 N/LSB), but the *transfer function* is
   not: a load cell needs a per-channel calibration; a current estimate needs
   motor `Kt`, spool radius, and a friction model and is friction-corrupted near
   standstill. Parked behind `CFG_TENSION_SCALE`. Until a valid scale is loaded,
   drivers set `CAL_INVALID` and refuse tension/HYBRID modes (fail safe).
2. **Current → torque needs `Kt`.** `current` telemetry (mA) is unambiguous, but
   any torque/tension inference from it needs the motor torque constant, which
   depends on the ADR-0003 BLDC not yet selected. Parked behind `CFG_MOTOR_KT`.
3. **Stiffness `k` reference frame.** Defined as N/m against *cable
   displacement* to match the `tendon.py` AIC law. If firmware ends up closing
   the stiffness loop on rotor angle, RT must convert via the spool radius
   (rad ↔ m). Needs sign-off with kinematics.
4. **position_target frame + homing.** Defined as **motor-side rotor angle**
   (rad); the RT tier owns the tendon-map joint↔motor conversion. Requires a
   homing/offset convention; unreferenced axes report `NOT_HOMED` and reject
   POSITION mode.
5. **Joint-side vs motor-side tension.** Schema carries motor-side tension only;
   kinematics `TendonSolution` exposes both — the capstan (friction) factor stays
   in the tendon map. This division of labor needs explicit sign-off with
   tomcat-kinematics via the lead.

---

## 7. Interface risks (for the lead)

- **R1 — segment count is unverified (ADR-0005).** §2.3 supports 6–7 segments on
  paper at ~5–6 nodes each, but frame timing/stuffing/latency must be measured at
  full axis count before the girdle backplane node assignment is frozen.
- **R2 — tension units blocked on ADR-0004.** Tension setpoint/telemetry are
  numerically meaningless until the sensing method and its calibration are chosen
  (§6.1). Everything downstream of tension control (co-contraction stiffness, the
  Tier-A over-tension latch threshold) inherits this gap.
- **R3 — `Kt` unknown (ADR-0003).** Blocks current-based tension estimation and
  any torque telemetry (§6.2); a hard dependency on motor selection from
  electronics.
- **R4 — spine/tail DOF count will grow.** M1 fixes 6 spine motors (sagittal);
  ADR-0006's lateral/axial DOF and the ADR-0007 tail will push motor count past
  33, re-loading the bus budget. `TOMCAT_N_MOTORS_MAX` and the node map must be
  revisited then.
- **R5 — Layer-1 transport undecided.** The SBC↔RT physical link (shared mem /
  SPI / Ethernet) is open; the logical contract here is stable, but latency of
  the chosen transport feeds the 20 ms SBC-stale window.
- **R6 — CRC polynomial / endianness unpinned.** Placeholder `crc` fields and a
  little-endian convention are declared but not finalized (ADR-0003 MCU choice).
```
