# Balance-loop latency budget — and why the electronics is not the bottleneck

Owner: **tomcat-lead** · Allocates **NFR12**, and corrects **NFR10**.

[ADR-0014](../DESIGN_DECISIONS.md) measured the disturbance-rejection envelope
against an *assumed* latency and set NFR12 at ≤20 ms — a figure that was never
allocated to a subsystem, so no one owned it. Allocating it turns the calculation
inside out.

Label legend: `[derived]` computed from the model · `[assumed]` engineering guess
· `[owed]` needs a bench measurement.

---

## 1. Latency is not an independent parameter

A bigger disturbance needs a bigger foothold correction. A bigger correction takes
the leg **longer to execute**. And that time *is* the staleness of the information
the controller committed on. So:

```
        disturbance  ->  correction size  ->  actuation time
             ^                                      |
             |                                      v
        envelope    <-   latency   <----------------+
```

It has to be solved as a **fixed point**, which
`control.self_consistent_envelope()` does.

## 2. The two terms

| Term | What it is | Value |
|---|---|---|
| **Pipeline** | contact detection → state estimation → bus transport → control computation. Everything *except* the leg moving. | allocated in §3 |
| **Actuation** | the leg physically repositioning its foothold | **37 ms** at the fixed point `[derived]` |

Actuation dominates, and it comes from a simple ratio: the correction must travel
along the support-line **perpendicular**, but the sagittal legs deliver it only
through their 0.44 projection, so the *fore-aft* foot travel is 2.3× the
correction. Divided by the spare foot speed:

| | value | source |
|---|---|---|
| Actuator foot-speed ceiling | **5.93 m/s** | motor 380 rpm through the tendon ratios `[derived]` |
| Nominal peak in swing | **1.83 m/s** (31 % of ceiling) | the shipped trot `[derived]` |
| **Spare for corrections** | **4.10 m/s** | the difference |

⚠️ The actuation figure is **optimistic**: it assumes a constant-velocity
correction, ignoring the accelerate/decelerate ramp and any torque limit during
it. A real leg needs longer, so 37 ms is a lower bound `[owed: measure on a leg]`.

## 3. Pipeline allocation

The pipeline is what electronics and firmware own. Allocated to **7.5 ms**:

| # | Stage | Budget | Basis |
|---|---|---|---|
| 1 | **Contact detection** (paw barometer → contact flag) | **1.0 ms** | NFR8 specifies ≥1 kHz; the flag is a comparator on the raw signal, so no filter delay `[sourced: ADR-0012]` |
| 2 | **State estimation** (contact + IMU + kinematics → DCM) | **5.0 ms** | ⚠️ the loosest number here, and the real design constraint on firmware — it caps the estimator's group delay, i.e. roughly a ≥30 Hz filter corner `[assumed]` |
| 3 | **Bus transport** (paw → RT, RT → driver) | **1.0 ms** | ~60 µs per CAN-FD frame at 1 Mbit/s arbitration + 5 Mbit/s data; 3–6 nodes per segment at 1 kHz is 18–36 % utilisation, worst-case queueing ~0.3 ms each way `[derived from ADR-0005]` |
| 4 | **Control computation** | **0.5 ms** | the placement law is one multiply-add; NFR3's ≥1 kHz loop covers it `[derived]` |
| | **Total pipeline** | **7.5 ms** | |

## 4. The result, and the two corrections it forces

| pipeline | self-consistent envelope | total latency |
|---|---|---|
| 2.5 ms | 62.1 mm | 39.8 ms |
| **7.5 ms** (allocated) | **59.2 mm** | **44.8 ms** |
| 12 ms | 56.7 mm | 49.3 ms |
| 20 ms | 52.4 mm | 57.3 mm |

⚠️ **NFR10's 90 mm assumed zero latency.** With the real actuator in the loop the
envelope is **~59 mm** — still a **0.46 m/s** lateral shove, but a third smaller
than published. Corrected.

⚠️ **NFR12's ≤20 ms was the wrong shape of requirement.** The total loop latency
at the fixed point is **~45 ms**, and most of it is the leg, not the electronics.
Re-cast as a *pipeline* budget of ≤7.5 ms, which is the part anyone can design to.

**And the useful surprise: the envelope is nearly insensitive to the pipeline.**
Going 2.5 → 20 ms costs 62 → 52 mm, about 16 %. So:

- **Electronics and firmware have a comfortable budget.** Even a sloppy 20 ms
  pipeline costs little. Chasing microseconds on the CAN-FD bus would be effort
  spent in the wrong place — ADR-0005's architecture is not the constraint.
- **Foot speed is the lever.** Halving the actuation term would buy more envelope
  than eliminating the entire electronics pipeline. That points at the leg's
  spare speed, i.e. motor free speed and the tendon ratios — not the wiring.
- **Or reduce the 2.3× projection penalty.** The correction is inflated because
  sagittal legs push at 0.44 to the perpendicular. This is the third time that
  projection has cost the design something ([ADR-0014](../DESIGN_DECISIONS.md)),
  and it is the strongest remaining argument for revisiting leg abduction — which
  [ADR-0015](../DESIGN_DECISIONS.md) closed on *authority* grounds, not on
  actuation-time grounds.

## 5. What is owed

- `[owed]` **Bench the actuation ramp.** The 37 ms assumes constant velocity; the
  accel/decel ramp and torque limits will make it worse, and this is now the
  dominant term in the whole budget.
- `[owed]` **Estimator design against the 5 ms allocation** — the number is
  `[assumed]` and it is the one firmware must actually meet.
- `[owed]` **Measure CAN-FD queueing under full load** — ADR-0005 already flags
  bench-verifying ≥1 kHz per segment; §3's 0.3 ms figure rides on that.
- `[owed]` Revisit whether the leg's spare foot speed can be raised cheaply, since
  §4 shows it is worth more than anything on the electronics side.
