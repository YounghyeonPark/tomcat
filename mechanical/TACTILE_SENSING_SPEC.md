# TomCat — Tactile sensing & the paw structure

Owner: **tomcat-mechanical** / **tomcat-electronics** · Prerequisite for the
closed-loop balance milestone (M8).

Closed-loop balance is only as good as what it can sense, and for a legged robot
the decisive measurement is **at the contact**. This specifies what the paw must
measure, why the sensing the project already owns is not enough, and how the
sensor integrates into the paw structure without violating **P1**.

Every requirement below is **derived from the robot's own dynamics**
(`kinematics/src/tomcat_kin/dynamics.py`), not taken from a catalogue.

Label legend: `[derived]` computed from the model · `[sourced]` from literature
· `[assumed]` engineering guess · `[owed]` needs a part or a test.

---

## 1. What closed-loop balance actually needs

From the M6/M7 results, in priority order:

| # | Quantity | Why it is needed | Where from |
|---|---|---|---|
| 1 | **Contact ON/OFF per foot** | The gait's phase must sync to *reality*. M7's trot balance depends on the diagonal switching at the right instant; on real ground, touchdown time varies. | **paw — nothing else can do this** |
| 2 | **Normal force per foot** | Load sharing, and the contact-force distribution the ZMP/line-balance analysis assumes. | paw + joint torque |
| 3 | **Tangential/normal ratio** | Slip margin (**FR5**). M7 trot runs at μ ≈ 0.145 against a floor of 0.8–1.2, so margin is large — but slip must still be *detected*. | joint torque + paw |
| 4 | **Body attitude & rate** | The inverted-pendulum state (`line_balance`): the DCM needs CoM position *and* velocity. | IMU (ADR-0005) |

Items 1–3 are the paw's job. Item 4 is already covered.

## 2. Derived requirements

All from the shipped crawl (5.0 s) and trot (0.3 s) `[derived]`:

| Parameter | Value | Basis |
|---|---|---|
| Normal force, **measurement** range | **0–35 N** | peak per-foot normal is 30.9 N (crawl) / 30.3 N (trot) = 0.78 × body weight |
| Normal force, **survival** | **≥ 100 N** | the ×2.5 single-leg **land** transient (2.5 × 39.7 N body weight) — must survive, need not be accurate |
| Tangential range | **0–20 N** | 2.9 N at nominal trot; sized for μ→0.5 at full normal load |
| Resolution | **≤ 0.4 N** (~1 % FS) | to see unloading before liftoff, and slip onset |
| Bandwidth | **≥ 1 kHz** | trot stance is **150 ms**; 1 % timing resolution = 1.5 ms. Matches NFR3. |
| **Mass** | **≤ 20 g per paw** | see §4 — this is the binding constraint |
| Ingress | sealed | the pad is the ground interface |

⚠️ **Touchdown impact is not modelled** (ADR-0011). The swing profile lands the
foot at zero velocity relative to the ground in *both* axes, so the trajectory
itself implies no impact — but terrain error and tracking error will produce one.
The 100 N survival figure is the land transient, and is the best available proxy
`[owed: instrument a drop test]`.

## 3. Why the sensing we already own is not sufficient

The project is not starting from zero. [ADR-0004](../docs/DESIGN_DECISIONS.md)
already puts **load cells at the JOINT end** of the hip and stifle tendons
(joint-end specifically, so the capstan friction path does not corrupt them). A
**point contact exerts no moment**, so the foot force is only two unknowns and
`tau = J^T F` is solvable from two clean torque measurements.

So why add anything? Two reasons, one quantitative and one categorical.

**(a) The inversion is badly conditioned on the hind legs.**
`dynamics.grf_observability` reports the error amplification `[derived]`:

| Gait | Fore legs (median / worst) | **Hind legs (median / worst)** |
|---|---|---|
| Crawl | 3.4 / 3.5 | 5.2 / 6.1 |
| **Trot** | 3.2 / 3.4 | **7.5 / 36.3** |

The hind-leg estimate degrades ~10× **just before liftoff** — precisely when load
is transferring to the other diagonal and accurate load sharing matters most. A
1 % tension error becomes a 36 % force error there.

**(b) Joint torque cannot detect contact at all.** A torque signal cannot
distinguish "the foot is pressing on the ground" from "the limb is accelerating"
without a full dynamics model running in the loop. For **requirement 1** — the
one closed-loop balance most depends on — the joint route is not merely
imprecise, it is *categorically* unable. Only a sensor at the contact can say
*"we are touching now."*

## 4. The P1 constraint: distal mass is the real budget

A paw sensor sits at the **most distal** point of the limb, which is exactly what
tendon drive exists to keep light. M7 showed swing-leg torque is what caps trot
speed, so this trade can be measured rather than argued `[derived]`:

| Added per paw | Swing motor torque | Worst-motor RMS @ 96 cm/s | @ 120 cm/s | Top sustainable speed |
|---|---|---|---|---|
| 0 g | 0.112 N·m | 0.645 (0.91×) | 0.709 (1.00×) | **120 cm/s** |
| 5 g | 0.124 (+10 %) | 0.651 (0.92×) | 0.732 (1.03×) | 96 cm/s |
| 10 g | 0.135 (+20 %) | 0.661 (0.93×) | 0.755 (1.06×) | 96 cm/s |
| **20 g** | 0.158 (+41 %) | 0.690 (0.97×) | 0.803 (1.13×) | **96 cm/s** |
| 40 g | 0.204 (+81 %) | **0.749 (1.06×)** | 0.903 (1.27×) | **below 96 cm/s** ❌ |

**Rule: ≤ 20 g per paw.** Up to 20 g costs only the top 20 % of speed
(120 → 96 cm/s). At 40 g the RMS motor torque exceeds the continuous rating even
at 96 cm/s, so the sensor starts *taking away usable gait*. Note the mass budget
(4 × 20 g = 80 g of 4045 g, ~2 %) is **not** the binding constraint — swing
inertia is. This is P1 quantified: on this robot, a gram at the paw is worth far
more than a gram in the girdle.

## 5. Specified paw structure

```
        metatarsus (CF tube)
              │
        ┌─────┴─────┐   aluminium paw carrier, bonded insert
        │  [ PCB ]  │   MEMS barometer + local ADC, potted
        │   ╱───╲   │
        │  ╱ air ╲  │   moulded TPU dome with an engineered air cavity
        └─╱───────╲─┘
         ╲_________╱    TPU ~80A contact surface  (mu 0.8-1.2 on hard floor)
```

| Item | Choice | Note |
|---|---|---|
| Principle | **MEMS barometer under a sealed elastomer dome** | `[sourced]` proven on quadruped feet (Unitree Go1 end-effector, ≥40 N repeated impact); sensitivity and range tuned by the dome geometry and cavity volume, not by the part |
| Why not a load cell | 3-axis load cells in this range are **tens of grams** and rigid | violates §4 and removes the compliance the pad exists to provide |
| Why not strain gauges on the bone | measures limb load, **not contact** | same categorical failure as §3(b); also needs the tube instrumented and temperature-compensated |
| Mass target | **≤ 8 g** `[assumed]` incl. pad, carrier and PCB | leaves margin under the 20 g rule |
| Output | normal force + **contact flag**, ≥1 kHz | the flag is a comparator on the raw signal, so it is available with no filtering delay |
| Interface | joins the limb's existing CAN-FD node | no new bus (ADR-0005) |
| Tangential force | **from the joint-end load cells**, fused with the paw normal | the paw gives the well-conditioned normal; the joints give shear. Together they beat either alone. |

**The pocket already exists.** [ASSEMBLY_SPEC §1](ASSEMBLY_SPEC.md) already
specifies the paw pad as *"cast/moulded TPU ~80A, compliant contact + tactile
sensor pocket"* — this spec fills that pocket.

## 6. Beyond the paw

The user's framing was *tactile sensing **body structure***, so for completeness,
in decreasing order of value to balance:

| Location | Value | Verdict |
|---|---|---|
| **Paws (×4)** | requirement 1–3 above | **specified here** |
| **Shin / metatarsus** | obstacle strike detection while swinging | ⏸ deferred — no balance role; revisit for obstacle traversal |
| **Body shell** | landing detection for the ADR-0007 righting reflex | ⏸ deferred to the righting milestone; a fall-detect IMU covers the trigger, contact covers the *end* |
| **Whiskers** | biomimetic, genuinely useful for a cat near obstacles | ⏸ out of scope — no balance role |

Only the paws are on the M8 critical path. The others are recorded so the
structure can carry provision (routing, pockets) rather than being retrofitted.

## 7. What is owed

- `[owed]` **Dome geometry design** — the force range is set by cavity volume and
  wall thickness, and needs FEA or a moulding trial. §2's range is the target, not
  a solved geometry.
- `[owed]` **Drop test** to characterise the real touchdown impulse; §2's 100 N
  is the land transient standing in for an unmodelled impact.
- `[owed]` **Barometer part selection** and its overpressure rating vs the 100 N
  survival case.
- `[owed]` **Creep/hysteresis** of TPU under sustained load — the crawl holds a
  foot loaded for 4.5 s, which is a long time for an elastomer.
- `[owed]` A **fusion estimator** combining the paw normal with the joint-end
  shear; §5 asserts the combination is better but nothing has been designed.
