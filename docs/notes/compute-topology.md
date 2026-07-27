# Compute + motor-drive topology — note to close ADR-0005

**Requested by:** ADR-0005 (Proposed) — "single MCU vs. RT-MCU for motor loops +
SBC for planning." Deliver a cited recommendation and proposed ADR text.

**Scope:** the *compute and motor-drive network* only — how ~30+ FOC axes are
controlled, over what bus, split across what compute, and where safety lives.
Actuator technology (ADR-0003), sensing method (ADR-0004), spine/tail DOF
(ADR-0006/0007) are inputs, not reopened here.

**Confidence legend** (same as `LITERATURE_REVIEW.md`): ✅ adversarially verified ·
◐ primary / single-read · ⚠️ caveat (vendor doc / secondary / single-design /
extrapolation).

---

## 0. The constraint envelope (from the ADRs + REQUIREMENTS)

- **~30 tendon FOC axes now**, rising toward **~33+** with the tail: 24 leg
  (antagonistic) + ~6 spine (ADR-0002/0003/0006), + tail (ADR-0007). Each axis
  needs **FOC commutation + closed-loop tension + a rotor position sensor
  (ADR-0004) + a per-tendon tension signal** (load cell or current estimate).
- **Two clocked layers** (REQUIREMENTS NFR3/NFR4): a **hard real-time motor loop
  at ≥1 kHz** across many channels, and **whole-body planning/righting at
  ≥100 Hz**, plus host comms/telemetry.
- **P1 / biomimetic:** motors are **clustered in the shoulder and pelvic
  girdles** (ARCHITECTURE §3), so the drive electronics naturally form **two
  physical clusters + a tail node**, not one monolith.

> **Robust to the ADR-0003 leg debate.** The parallel leg-actuator trade study
> ([leg-actuator-tradeoff.md](leg-actuator-tradeoff.md)) recommends QDD legs;
> ADR-0003 as written keeps tendon-drive everywhere (P1). **Either outcome leaves
> ~24 leg + ~6 spine = ~30 BLDC/FOC axes**, each needing per-axis FOC + a rotor
> sensor. The only thing that changes is the *tension-sensing* front-end: QDD legs
> get force from motor current (no load cell); tendon legs need explicit tension
> sensing. **The topology below holds under both**, so ADR-0005 can close without
> waiting on ADR-0003.

---

## 1. Centralized vs. distributed motor control

**Finding: the field-proven pattern at 12–170 axes is distributed smart drivers
(one FOC controller per motor, or at most a few per limb) on a real-time bus. A
single MCU running 30 simultaneous FOC + tension loops is not field-proven and is
not recommended.** ✅/◐

Why one big MCU does not scale here:
- FOC needs a fast per-axis inner **current loop** (typ. tens of kHz) with its own
  ADC phase-current sampling, PWM timers, and encoder capture. Open BLDC stacks
  put **1–2 axes per MCU**: **moteus = one STM32G4 per motor**
  ([mjbots quad / moteus](https://hackaday.io/project/167845-mjbots-quad),
  [moteus reference](https://github.com/mjbots/pi3hat/blob/master/docs/reference.md)) ◐;
  ODrive = two motors per board; SimpleFOC = 1–2. Fanning 30 axes' worth of PWM,
  ADC, and encoder peripherals into one MCU is a pin/DMA/interrupt bottleneck, and
  a single fault or reflash takes the whole robot down.

What real multi-axis tendon / legged systems actually do:
- **Kengoro (JSK, U-Tokyo)** — the closest tendon analogue — is built from **116
  sensor-driver-integrated muscle modules (SDIMMs)**, each packaging a
  **current-controlled BLDC motor, its motor driver, a load-cell tension unit
  (~55 kgf limit), a temperature sensor, and a 29:1 gearhead into one module**;
  108–174 DOF total ([arXiv:2409.00705](https://arxiv.org/pdf/2409.00705),
  [Science Robotics 2019](https://www.science.org/doi/10.1126/scirobotics.aaq0899),
  [Musashi arXiv:2410.22000](https://arxiv.org/pdf/2410.22000)). ✅ This is
  **per-muscle distributed control** — driver + sensing co-located with each
  actuator — exactly the "cluster in the girdles" pattern P1 implies.
- **mjbots quad / MIT-Mini-Cheetah-class quadrupeds** use **one integrated smart
  driver per joint** (moteus: STM32G4 + integrated absolute magnetic encoder +
  3-phase FOC + current sense + 5 Mbit/s CAN-FD in a ~50 mm board), 12 per robot
  ([mjbots quad](https://hackaday.io/project/167845-mjbots-quad),
  [pi3hat blog](https://hackaday.io/project/167845-mjbots-quad/log/179913-pi3hat-released)). ◐
  MIT Mini-Cheetah likewise uses a per-actuator FOC controller (Ben Katz design)
  on CAN ([overview](https://pcb.mit.edu/archive/IAP2023/lectures/lecture_x/)). ◐
- **Open ecosystems** converge on the same shape: **ODrive, moteus/mjbots, and
  MyActuator** all ship *integrated per-motor FOC drivers* with an onboard MCU,
  onboard rotor encoder, and a CAN/CAN-FD interface — i.e. the driver, not the
  central computer, closes the current loop. moteus is fully open-source firmware;
  ODrive's latest offering went closed
  ([Hackaday](https://hackaday.com/2022/07/04/moteus-open-source-bldc-controller-gets-major-upgrade/)). ◐
- **Industrial / ROS 2 + EtherCAT** legged and humanoid robots (e.g. **LAURON VI**)
  put an **EtherCAT slave at each drive** and a real-time master upstream
  ([LAURON VI arXiv:2508.07689](https://arxiv.org/html/2508.07689),
  [EtherCAT humanoid control, TUM](https://mediatum.ub.tum.de/doc/1394924/0185778354196.pdf)). ◐

> **Conclusion (Q1):** go **distributed**. One FOC smart driver **per tendon
> motor** (Kengoro/moteus pattern), physically grouped into a **shoulder-girdle
> cluster, a pelvic-girdle cluster, and a tail node** (P1). A per-limb "few-axis"
> MCU is the fallback if per-motor boards prove too bulky, but per-motor is the
> better-trodden path and localizes faults, thermals, and sensing.

---

## 2. Real-time bus for the ≥1 kHz layer

**Finding: CAN-FD (5 Mbit/s), split across several bus segments, is the
field-proven, low-cost fit at our scale (≈30 axes). EtherCAT is the higher-
determinism alternative worth adopting only if a single hard-synchronized ≥1 kHz
domain across all 30+ axes becomes necessary — at meaningfully higher per-node
cost/complexity.** ◐/⚠️

Concrete numbers:

| | Classical CAN | **CAN-FD** | EtherCAT |
|---|---|---|---|
| Payload / frame | 8 B | **64 B** | full Ethernet frame; "processing on the fly" |
| Bit rate | ≤1 Mbit/s | **1 Mbit/s arbitration, up to 8 Mbit/s data (5 Mbit/s in moteus)** | 100 Mbit/s line, µs-class cycle |
| Determinism | priority arbitration; jitter under load | same arbitration model, ~6× throughput | **hardware distributed-clock, µs multi-axis sync** |
| Per-node HW cost | transceiver only | transceiver only | **dedicated EtherCAT Slave Controller (ESC) per node** |
| Topology | multidrop | multidrop | daisy-chain ring |

Sources: CAN-FD payload/bit-rate/spec (ISO 11898-1:2015, Bosch 2012) —
[CiA](https://www.can-cia.org/can-knowledge/can-fd-the-basic-idea),
[Kvaser](https://kvaser.com/can-fd-protocol-tutorial/); "≈6× throughput vs
classical CAN at 1:8 rate ratio" — CiA. EtherCAT distributed-clock / µs sync and
"CAN for auxiliary, EtherCAT for the tight loop" split —
[embedded.com](https://www.embedded.com/selecting-the-best-network-for-robot-control/),
[GigaDevice](https://www.gigadevice.com/about/blog/best-network-protocol-for-robot-control),
[promwad](https://promwad.com/news/ethercat-vs-can-vs-isobus-field-protocol-war). ◐

**Measured CAN-FD capacity at our scale (the load-bearing data point):**
- A **single moteus on a pi3hat reaches ~2200 Hz**; rate degrades as more
  controllers share a bus ([mjbots optimization blog](https://blog.mjbots.com/2024/05/16/optimizing-moteus-command-rate/)). ◐
- The **mjbots quad commands + queries all 12 servos + reads the IMU in ~740 µs**,
  i.e. **1 kHz whole-body control on one Raspberry Pi core** when the 12 servos are
  **spread across the pi3hat's ~4 high-speed CAN-FD buses (~3 per bus)**
  ([pi3hat release](https://hackaday.io/project/167845-mjbots-quad/log/179913-pi3hat-released)). ◐

> **Scaling to ~30–33 axes (⚠️ my extrapolation, not vendor data):** 12 axes over
> ~4 CAN-FD segments at 1 kHz is proven; ~30 axes is ~2.5× the traffic. Keeping the
> proven **~3–6 axes per 5 Mbit/s segment** implies **~6–8 CAN-FD segments** (or
> fewer if the loop budget is relaxed toward the low end). That is exactly a
> **multi-bus bridge** device (pi3hat-style, 5 buses) — possibly **two bridges**,
> or a dedicated RT aggregator MCU with several CAN-FD controllers. **The exact
> segment count is the concrete sizing task handed to tomcat-electronics /
> tomcat-firmware** (§4). CAN-FD's soft arbitration means the ≥1 kHz guarantee must
> be *verified* per segment, not assumed — this is the one place EtherCAT's
> hardware sync would buy determinism if bench tests fall short.

**Why not EtherCAT as the baseline:** it wins on hard determinism and µs
multi-axis sync, but costs an **ESC chip + more complex firmware per node** on
~30+ nodes, and the open tendon/quadruped ecosystem we can lean on
(moteus/ODrive/Kengoro) is **CAN/CAN-FD-native**. Reserve EtherCAT as the
migration path if a synchronized single-domain ≥1 kHz loop across all axes is
later required (the industrial/ROS 2 humanoid pattern).

---

## 3. Compute split + where safety lives

**Finding: two tiers — a real-time tier for motor loops + safety, and a
non-real-time SBC for planning/righting/telemetry. Do not put ≥1 kHz control and
ROS 2 planning on the same non-deterministic core. Safety must be autonomous at
the lowest level.** ◐/✅

Recommended split (top to bottom):
1. **SBC (application tier, ≥100 Hz, best-effort):** ROS 2, gait/trajectory
   planning, fall-detection + righting law (ADR-0007), IK, host comms/telemetry.
   Non-safety-critical timing. This is the "Gait & Trajectory Planner" +
   "Kinematics/Tendon Map" high/mid layers of ARCHITECTURE §2.
2. **Real-time controller (RT tier, ≥1 kHz, hard):** aggregates the CAN-FD
   segments, runs the tendon map's fast path + antagonistic coordination
   (AIC, ADR-0002), fans setpoints to the drivers, collects tension/position/
   current, and **owns the safety supervisor + watchdog**. Two viable forms:
   (a) a **Linux SBC + PREEMPT_RT + a CAN-FD bridge** on a dedicated core
   (mjbots' Pi hits 1 kHz for 12 axes this way — ◐); or (b) a **dedicated RT MCU/
   MPU aggregator** with the CAN-FD controllers. For ~30+ axes, prefer a dedicated
   RT core/MCU so planning jitter can never steal the motor loop.
3. **Per-motor smart drivers (§1):** each closes its own FOC current loop and
   tension/position loop locally.

**E-stop / limp-state safety — three independent tiers (highest to lowest
autonomy):**
- **Tier A — local, per-driver (autonomous):** each smart driver enforces
  **over-current, over-tension, and thermal limits itself** and drops to a known
  safe state without asking the network. Kengoro's per-module temperature +
  ~55 kgf tension unit is precedent for co-locating these limits with the actuator
  ([arXiv:2409.00705](https://arxiv.org/pdf/2409.00705)); moteus/ODrive expose
  current/thermal fault latches. ✅/◐ Satisfies **FR8** at the fastest layer.
- **Tier B — hardware e-stop (SBC-independent):** a physical loop that **cuts
  motor-bus power** and/or asserts a hardware line driving **all drivers to
  zero-torque limp**, wired so it works even if the SBC and RT controller are
  hung. This is the true fail-safe and must not route through software.
- **Tier C — RT supervisor watchdog:** the RT controller commands **limp on
  comms-loss / fault**; if the SBC stops publishing, the RT tier holds a safe
  posture or limps. The SBC is *never* in the safety-critical path.

> **Answer to "single MCU vs. RT-MCU + SBC":** **RT tier + SBC.** A single MCU
> can neither host ROS 2/planning nor run 30 FOC loops; and even a single powerful
> SBC should not co-schedule hard ≥1 kHz control with best-effort planning on the
> same cores. Safety lives **primarily in the drivers (Tier A) and a hardware
> e-stop (Tier B)** — not in the SBC.

---

## 4. Recommended architecture

```
        ┌──────────────────────────────────────────────────────────┐
        │  HOST / OPERATOR   (laptop/tablet: config, telemetry, e-stop button)
        └───────────────▲──────────────────────────────────────────┘
                        │ USB / Wi-Fi  (non-RT)
        ┌───────────────┴──────────────────────────────────────────┐
        │  SBC — APPLICATION TIER  (ROS 2, ≥100 Hz)                  │
        │  gait/trajectory planning · righting law (ADR-0007) · IK · │
        │  telemetry/host comms · NON-safety-critical               │
        └───────────────▲──────────────────────────────────────────┘
                        │ shared-memory / low-latency link (setpoints ↓, state ↑)
        ┌───────────────┴──────────────────────────────────────────┐
        │  RT CONTROLLER — REAL-TIME TIER  (≥1 kHz, hard)           │
        │  CAN-FD bridge · tendon map fast path + AIC · setpoint fan-out │
        │  SAFETY SUPERVISOR + WATCHDOG  (Tier C)                    │
        └───┬───────────────────┬───────────────────────┬──────────┘
     CAN-FD │ (5 Mbit/s,        │                       │
    segments│  ~6–8 total,      │                       │
            ▼  ~3–6 axes each)  ▼                       ▼
   ┌───────────────────┐  ┌───────────────────┐  ┌───────────────┐
   │ SHOULDER-GIRDLE   │  │ PELVIC-GIRDLE     │  │ TAIL NODE     │
   │ DRIVER CLUSTER    │  │ DRIVER CLUSTER    │  │ (ADR-0007)    │
   │ per-motor FOC     │  │ per-motor FOC     │  │ per-motor FOC │
   │ smart drivers:    │  │ smart drivers:    │  │ smart driver  │
   │  fore-leg + fore- │  │  hind-leg + hind- │  │  + tension/   │
   │  spine tendons    │  │  spine tendons    │  │  current      │
   │  (Tier A limits)  │  │  (Tier A limits)  │  │  (Tier A)     │
   └───────────────────┘  └───────────────────┘  └───────────────┘
            ▲                       ▲                      ▲
            └─────────── HARDWARE E-STOP LOOP (Tier B) ────┘
              cuts motor-bus power / forces zero-torque limp,
              independent of SBC and RT controller
```

Each smart driver = **BLDC power stage + FOC + rotor sensor (ADR-0004) + per-tendon
tension front-end** (load-cell ADC for tendon joints; motor-current estimate where
a load cell is omitted / for any QDD legs) + CAN-FD transceiver + local Tier-A
safety.

**Reasoning recap:** distributed per-motor FOC is the only field-proven way to run
~30 axes (Kengoro's 116 SDIMMs; moteus/Mini-Cheetah quadrupeds) (§1); CAN-FD at
5 Mbit/s over several segments is the proven, low-cost RT bus at this scale
(mjbots: 12 axes @ 1 kHz over ~4 segments), with EtherCAT reserved as the harder-
determinism upgrade (§2); the two-tier compute split keeps the ≥1 kHz loop off the
non-deterministic planner, and safety is autonomous in the drivers + a hardware
e-stop (§3); the two girdle clusters + tail node fall straight out of P1's
mass-centralization.

### Implications for the boards (tomcat-electronics)
- Design a **per-motor FOC smart-driver board** (moteus-class reference: STM32G4-
  class MCU + 3-phase bridge + phase current sense + onboard rotor encoder + CAN-FD
  transceiver). Add a **tension front-end**: load-cell instrumentation amp + ADC
  for tendon joints (ADR-0004 load-cell path), or rely on current sense where a
  load cell is dropped. ⚠️ This is the one ADR-0004-dependent board feature.
- **Two girdle backplanes + a tail stub**: power distribution + CAN-FD segment
  wiring, grouping ~3–6 drivers per 5 Mbit/s segment. Decide **segment count
  (~6–8)** from the bus-loading bench test (§2).
- **Hardware e-stop circuit (Tier B):** a power-cut contactor / gate on the motor
  bus and/or a shared "safe-torque-off" line to all drivers, not gated by software.
- **RT-bridge choice:** a pi3hat-style multi-CAN-FD bridge (up to 5 buses) or a
  dedicated aggregator MCU — likely **two bridges or one bridge + expansion** for
  30+ axes.

### Implications for firmware (tomcat-firmware)
- **Driver firmware:** FOC current loop + closed-loop tension/position loop +
  Tier-A over-current/over-tension/thermal latches per motor; a **CAN-FD register/
  command schema** (moteus-style) for setpoints + telemetry.
- **RT-tier firmware:** ≥1 kHz aggregation loop, tendon-map fast path + AIC split
  (ADR-0002), setpoint fan-out + state collection across CAN-FD segments, safety
  supervisor + watchdog (Tier C), limp-on-comms-loss. Run on PREEMPT_RT core or a
  dedicated MCU.
- **SBC side:** ROS 2 planning/righting nodes at ≥100 Hz + host telemetry; must
  tolerate being pre-empted without endangering the motor loop.
- **Bench-verify the ≥1 kHz guarantee per CAN-FD segment** before committing the
  segment count — CAN-FD arbitration is soft, so NFR3 must be measured, not assumed.

---

## Proposed ADR-0005 decision text (for the lead — do NOT self-apply)

> **ADR-0005: Compute topology**
> **Status:** Accepted (supersedes the Proposed "undecided" placeholder)
> **Context:** ~30 tendon FOC axes now (24 legs + ~6 spine), rising toward ~33+
> with the tail (ADR-0007); each needs FOC, closed-loop tension, a rotor sensor
> (ADR-0004), and a tension signal. Two clocked layers are required: a hard
> ≥1 kHz motor loop across many channels (NFR3) and ≥100 Hz planning/righting
> (NFR4). P1 clusters motors in the girdles.
> **Decision:** **Distributed smart drivers on a CAN-FD real-time bus, under a
> two-tier compute split, with autonomous low-level safety.**
> - **Motor control: one FOC smart driver per tendon motor** (Kengoro's 116
>   sensor-driver-integrated muscle modules; moteus/MIT-Mini-Cheetah quadrupeds),
>   grouped physically into a **shoulder-girdle cluster, a pelvic-girdle cluster,
>   and a tail node** (P1). A single central MCU running 30 FOC loops is rejected
>   as not field-proven. Per-limb few-axis MCUs are the fallback if per-motor
>   boards are too bulky.
> - **Bus: CAN-FD at 5 Mbit/s, split across ~6–8 segments** (~3–6 axes each),
>   matching the mjbots-quad pattern (12 axes @ 1 kHz over ~4 CAN-FD segments).
>   EtherCAT is the documented upgrade path if a single hardware-synchronized
>   ≥1 kHz domain across all axes is later required; its per-node ESC cost is not
>   justified at this scale today.
> - **Compute: RT tier + SBC.** A real-time controller (PREEMPT_RT core or
>   dedicated MCU) owns the ≥1 kHz aggregation, tendon-map fast path/AIC, and the
>   safety supervisor; a separate SBC (ROS 2) runs ≥100 Hz planning/righting and
>   host comms. Hard control is never co-scheduled with best-effort planning on the
>   same cores. Single-MCU is rejected.
> - **Safety (FR8):** three independent tiers — (A) per-driver over-current/
>   over-tension/thermal latches (autonomous), (B) a **hardware e-stop** that cuts
>   motor-bus power / forces zero-torque limp independent of the SBC and RT
>   controller, and (C) an RT-supervisor watchdog that limps on comms-loss. The SBC
>   is never in the safety-critical path.
> **Consequences:** tomcat-electronics designs a per-motor FOC smart-driver board
> (rotor sensor + CAN-FD + a load-cell/current tension front-end per ADR-0004), two
> girdle backplanes + a tail stub, a multi-CAN-FD bridge (or dedicated aggregator),
> and a hardware e-stop circuit. tomcat-firmware splits into driver firmware (FOC +
> tension + Tier-A safety + CAN-FD schema), RT-tier firmware (≥1 kHz aggregation +
> tendon map + Tier-C watchdog), and SBC ROS 2 nodes. **Open sizing task:**
> bench-verify the ≥1 kHz loop per CAN-FD segment to fix the segment count (≥1 kHz
> is soft-arbitrated on CAN-FD and must be measured, not assumed). Robust to the
> ADR-0003 leg outcome: QDD or tendon legs are still ~24 FOC axes; only the
> tension-sensing front-end differs.

---

## Sources

- Kawaharazuka et al., *Antagonist Inhibition Control … (Kengoro)* —
  [arXiv:2409.00705](https://arxiv.org/pdf/2409.00705) (116 sensor-driver-
  integrated muscle modules; per-module BLDC + driver + load cell ~55 kgf + temp
  sensor + 29:1 gearhead)
- Asano et al., *Design principles of a human mimetic humanoid (Kengoro)*, Science
  Robotics 2019 — [science.org](https://www.science.org/doi/10.1126/scirobotics.aaq0899)
- *Musashi* modularized musculoskeletal platform —
  [arXiv:2410.22000](https://arxiv.org/pdf/2410.22000)
- mjbots quad / moteus (per-motor STM32G4 + FOC + magnetic encoder + 5 Mbit/s
  CAN-FD) — [Hackaday.io project](https://hackaday.io/project/167845-mjbots-quad),
  [pi3hat release (5 CAN-FD buses; 12 servos + IMU in ~740 µs → 1 kHz)](https://hackaday.io/project/167845-mjbots-quad/log/179913-pi3hat-released),
  [moteus/pi3hat reference](https://github.com/mjbots/pi3hat/blob/master/docs/reference.md),
  [command-rate optimization (single moteus ~2200 Hz on pi3hat; degrades per bus)](https://blog.mjbots.com/2024/05/16/optimizing-moteus-command-rate/),
  [moteus open-source upgrade / ODrive closed](https://hackaday.com/2022/07/04/moteus-open-source-bldc-controller-gets-major-upgrade/)
- MIT Mini-Cheetah per-actuator FOC (Ben Katz) —
  [MIT PCB lecture overview](https://pcb.mit.edu/archive/IAP2023/lectures/lecture_x/)
- LAURON VI six-legged robot on EtherCAT —
  [arXiv:2508.07689](https://arxiv.org/html/2508.07689)
- EtherCAT-based real-time humanoid control architecture (TUM) —
  [mediatum PDF](https://mediatum.ub.tum.de/doc/1394924/0185778354196.pdf)
- CAN-FD spec / bit rates / payload (ISO 11898-1:2015; ≈6× throughput) —
  [CiA](https://www.can-cia.org/can-knowledge/can-fd-the-basic-idea),
  [Kvaser tutorial](https://kvaser.com/can-fd-protocol-tutorial/)
- CAN vs CAN-FD vs EtherCAT for robot control (EtherCAT = tight loop, CAN =
  low-cost auxiliary messaging; sub-1 ms needs EtherCAT/PROFINET IRT) —
  [embedded.com](https://www.embedded.com/selecting-the-best-network-for-robot-control/),
  [GigaDevice](https://www.gigadevice.com/about/blog/best-network-protocol-for-robot-control),
  [promwad](https://promwad.com/news/ethercat-vs-can-vs-isobus-field-protocol-war)
- Internal: `docs/DESIGN_DECISIONS.md` (ADR-0002/0003/0004/0006/0007),
  `docs/ARCHITECTURE.md` (3-layer stack), `docs/REQUIREMENTS.md` (NFR3/NFR4, FR8),
  `docs/LITERATURE_REVIEW.md` (Q1b Kengoro; Q5 MIT Cheetah),
  `docs/notes/leg-actuator-tradeoff.md`, `docs/notes/sensorless-foc-stance-hold.md`
</content>
</invoke>
