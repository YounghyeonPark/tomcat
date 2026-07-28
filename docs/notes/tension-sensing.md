# Per-tendon tension sensing for T.O.M.C.A.T. (closes the open sub-point of ADR-0004)

**Question.** ADR-0004 already accepted a **rotor position sensor on every motor**
(absolute encoder preferred, Halls as the floor — [sensorless-FOC
note](sensorless-foc-stance-hold.md)). The **tension-sensing method** is still
open. Pick one (or a hybrid) of: **(1) in-line load cell per tendon**,
**(2) motor-current estimate**, **(3) series-elastic element + displacement
sensor (SEA)**, for OUR context:

- Motors centralized in the shoulder/pelvic girdles (ADR-0001); cables routed
  over **pulleys/idlers/short sheaths with friction** to the joints (ADR-0003,
  [LEG_TENDON_SPEC.md](../../mechanical/LEG_TENDON_SPEC.md)).
- Tensions up to **~500 N single-leg land transient**, **~20–160 N continuous**
  stand/trot (LEG_TENDON_SPEC §1.3).
- We need tension **to regulate the antagonistic co-contraction bias `T_bias`
  and the AIC stiffness law** (ADR-0002) — i.e. we need to know the tension the
  **joint** actually feels, and know it on **both** antagonists to a few newtons.

Confidence legend: ✅ adversarially verified / textbook physics · ◐ primary,
single-read peer-reviewed · ⚠️ depends on our placeholder μ / wrap, or
single-design.

---

## 0. The crux: friction sits BETWEEN the motor and the joint

This decision is dominated by one fact, not by sensor datasheets. Our tendons run
motor → spool → idlers → joint sheave → anchor. Coulomb (capstan / Eytelwein)
friction acts over the **total wrap angle** along that path, so the tension at the
two ends differs multiplicatively (our `tendon.py` model, and LEG_TENDON_SPEC §3.4):

```
T_motor = T_joint · exp(+μ·θ_wrap)    (motor pulling in / developing / holding)
T_motor = T_joint · exp(−μ·θ_wrap)    (motor paying out / releasing)
```

`T_joint` is what produces joint torque (`τ = r·(T_flex − T_ext)`) and sets joint
stiffness; `T_motor` is what the girdle spool — and anything sensing there —
sees. **Any sensor on the motor/girdle side of the pulley train reads `T_motor`,
not `T_joint`.**

**Our own numbers (⚠️ placeholder μ = 0.10, assumed wrap angles — LEG_TENDON_SPEC
§3.4):**

| Tendon path | Σ wrap θ | develop `e^{+μθ}` | release `e^{−μθ}` | direction-dependent band `e^{2μθ}` |
|---|---|---|---|---|
| Knee (spool 180° + hip idler 45° + knee sheave 145°) | ~6.46 rad | **×1.91** | ×0.52 | **~3.6×** |
| Ankle (worst, 360°) | ~6.28 rad | ×1.87 | ×0.54 | ~3.5× |

Read this carefully: for **one** motor-side tension reading, the tension the joint
actually feels can be anywhere in a **~3.6× band** (knee) — near `0.52·T_motor`
just after a develop stroke, near `1.91·T_motor` just after a release — and
**which end depends on the direction of the last motion (hysteresis)** and on μ,
which drifts with wear, lubrication, cable dressing, and temperature. At the μ =
0.08–0.12 range LEG_TENDON_SPEC allows, the develop factor alone spans ×1.68–×2.17.

**Independent hardware corroboration.** A cable-driven surgical robot instrumented
both ends of the same cable during a palpation event: the **proximal (motor-end)
strain gauge recorded a 0.98 N change while the distal (joint-end) FSR recorded
0.54 N** for the identical event — friction inflated the motor-end reading ~1.8×,
almost exactly our computed ~1.9× knee factor. The authors conclude *"long-distance
cable tension sensing is greatly affected by friction due to large bending angles
and long force transmission distances"* and therefore place their tension sensors
**distally, at the joint end** ([Sensors 24(10):3156, 2024,
PMC11125287](https://pmc.ncbi.nlm.nih.gov/articles/PMC11125287/), ◐ peer-reviewed).

> **Placement rule that falls out of this, and governs the whole decision:** to
> know the tension the **joint** feels, the sensor must be **downstream of the
> friction — at the joint/output end** — or the wrap between it and the joint must
> be driven near zero. A sensor in the girdle reads a friction- and
> direction-corrupted proxy.

---

## 1. In-line load cell per tendon (measures JOINT-side tension directly) ✅/◐

**What it is.** A tension unit in series with the tendon; force read directly
(strain gauge / FSR / a tension-sensing pulley that reads the cable's lateral
reaction).

- **Precedent (strong):** Kengoro packs **116 sensor-driver-integrated muscle
  modules, each with a load-cell tension unit (~55 kgf ≈ 540 N range)** plus a
  temperature sensor and current-controlled BLDC — this is the field-proven way to
  do closed-loop tendon tension at scale, and it is exactly what makes AIC/`T_bias`
  regulation possible ([Kawaharazuka et al., arXiv:2409.00705](https://arxiv.org/pdf/2409.00705),
  ✅ verified in [LIT Q1/Q1b](../LITERATURE_REVIEW.md)). Our peak per-tendon
  design load (~525 N, LEG_TENDON_SPEC §5.4) sits right at Kengoro's ~540 N cell
  class — a good sizing match.
- **Accuracy achievable:** a dedicated cable tension-sensor array resolves force to
  **0.173 N average / 0.213 N RMS error over a 0–4 N range** ([Sensors 2024,
  PMC11125287](https://pmc.ncbi.nlm.nih.gov/articles/PMC11125287/), ◐) — i.e.
  sub-newton, more than enough to regulate a ~20 N `T_bias`.
- **Compact modern variant:** a single-pulley + maze-slot fixation tension module
  that estimates tendon length with a **3D Hall-effect sensor** instead of a bulky
  encoder — the space-saving way to get a load-cell-equivalent reading
  ([MDPI Actuators 14(6):278, 2025](https://www.mdpi.com/2076-0825/14/6/278), ⚠️
  author-claimed, not independently benchmarked).
- **Cost:** direct joint-side tension, on **both** antagonists → the only option
  that measures the quantity `T_bias`/AIC actually needs (ADR-0002). Adds parts,
  wiring, calibration, and — critically — **must be placed at the joint/output end
  to escape the §0 friction band**, which puts a small mass and a wire run at the
  distal joint (a mild P1 cost). Kengoro mitigates instead by keeping routing
  low-friction and reading tension at the module output; that only works if wrap
  between cell and joint is small.

**Verdict:** the **only** option that directly delivers `T_joint` — the number the
control law needs. Cost is parts/space/wiring and the placement constraint.

---

## 2. Motor-current estimate (reuses FOC phase sensing) — free, but friction-blind ✅

**What it is.** FOC already measures phase currents to commutate; `T_motor ≈
(k_t·I_q)/r_spool` gives a per-tendon tension proxy for **zero extra parts**.

- **Precedent people cite:** the **MIT Cheetah** proprioceptive actuator does
  high-bandwidth force control from motor current alone, with IMF comparable to
  series-spring designs, >450 N contact forces, 85 ms contacts ([Wensing et al.,
  IEEE T-RO 2017](https://www.researchgate.net/publication/312558722), ◐ LIT Q5).
  **But this precedent does not transfer to us:** MIT Cheetah is **direct-drive
  (low-gear-ratio, no cable, no pulley routing)** — there is *no intervening
  friction path* between motor and joint, so motor torque maps cleanly to joint
  torque. Current-based force works there **because** the §0 friction term is
  absent. Our tendon-over-pulleys architecture is precisely the case it isn't.
- **The core problem, quantified for us:** current tells you `T_motor`. To get
  `T_joint` you must divide by `exp(±μ·θ_wrap)` — but you do not reliably know the
  **sign** (depends on instantaneous motion direction / stiction state) or the
  **value** of μ (drifts). So the current estimate maps to a **~3.6× (knee),
  direction-dependent, hard-to-calibrate band** of possible joint tensions (§0).
  The tendon-sheath literature confirms this is a real, nonlinear hysteresis
  problem requiring dedicated compensation models, not a scalar correction
  ([Non-linear hysteresis compensation of a tendon-sheath manipulator using motor
  current, IEEE 2021](https://www.researchgate.net/publication/349016843), ⚠️
  paywalled abstract).
- **Fatal for our specific need:** we must regulate `T_bias ≈ 20 N` (Kengoro) and
  read **both** antagonists to set stiffness. A ±(half of 3.6×) friction band is
  tens of newtons — it **swamps** the 20 N bias. You cannot control co-contraction
  stiffness from motor current through this routing.

**What current-estimate IS good for (keep it — it's free):** it is the natural
**always-on, kHz, motor-side** channel for exactly the jobs that only need
`T_motor`: **Tier-A over-tension / over-current latching** (ADR-0005 safety),
**cable-slack detection, backdrive/collision detection, and a coarse feedforward**.
It protects the motor and cable (which see `T_motor`, the larger value) perfectly
well. It just cannot *know the joint-side tension through friction*.

**Verdict:** viable as the cheap safety/coarse channel; **not** viable as the
primary joint-tension sensor for AIC/`T_bias` in a friction-routed tendon.

---

## 3. Series-elastic element + displacement sensor (SEA) ◐

**What it is.** Put a calibrated spring in series with the tendon; measure its
deflection; force = `k·x` (Hooke). Turns force sensing into position sensing.

- **Precedent:** the foundational SEA work (Pratt & Williamson, *Series Elastic
  Actuators*, 1995) trades bandwidth for **stable, low-noise force control, low
  impedance, and shock tolerance** ([MIT AIM-1524](https://apps.dtic.mil/sti/pdfs/ADA299658.pdf),
  ◐). Modern SEAs reach high force fidelity but note force accuracy is limited by
  **transmission nonlinearity and deflection-sensor noise**, and by the inherent
  **bandwidth-vs-compliance tradeoff** ([Towards Accurate Force Control of SEAs,
  arXiv:1902.05338](https://arxiv.org/pdf/1902.05338), ◐).
- **The appeal for T.O.M.C.A.T.:** the project *wants* compliance anyway — shock
  absorption (G3) and tunable stiffness — and SEA delivers robust force from a
  cheap position sensor rather than a load cell.
- **But it interacts badly with our chosen scheme:**
  1. **We already get commandable stiffness from antagonism + `T_bias`**
     (ADR-0002). Adding a *mechanical* series spring layers a second,
     **passive/fixed** compliance under the active variable-stiffness we spent the
     antagonistic motor count to buy — partially duplicating it and making the net
     joint stiffness a coupled function of both, harder to model in `tendon.py`.
  2. **Placement doesn't escape §0.** An SEA spring reads the tension *at the
     spring's location*. Put it in the girdle → it reads `T_motor` (friction-blind,
     same as option 2). To read `T_joint` it must sit at the joint end — same
     constraint as the load cell, but bulkier.
  3. **It adds series length/travel** to already-tension-critical distal joints and
     **lowers control bandwidth** — costly where we have the least room.
  4. Our shock/energy path is already partly covered: **UHMWPE cable elasticity**
     (LEG_TENDON_SPEC §2) + low limb inertia give some series compliance for free.

**Verdict:** a legitimate compliance *actuation* choice for specific joints, but as
a **tension-sensing baseline** it is redundant with ADR-0002's active stiffness and
carries the worst size/bandwidth cost. Not the default; revisit only if a joint
turns out to need dedicated mechanical shock compliance that cable elasticity can't
provide.

---

## 4. Recommendation — hybrid, matching the electronics dual front-end

**Adopt a two-channel (hybrid) scheme, aligned with the per-motor "ADR-0004
tension front-end" already named in ADR-0005:**

1. **Motor-current estimate — always-on, every tendon (free).** Reuses FOC phase
   sensing at kHz. Role: **Tier-A over-tension/over-current safety latch**
   (ADR-0005), slack/backdrive/collision detection, and coarse feedforward. It
   senses `T_motor` — which is the correct quantity for **protecting the motor and
   cable** (they carry the amplified tension). It is explicitly **not** trusted for
   joint-side tension through friction.

2. **In-line load cell (or compact tension-sensing pulley) at the JOINT/output
   end — on the joints that must regulate stiffness.** These are the
   **stiffness-critical antagonistic joints**: the **spine joints** (whose joint
   *angle* can't be directly instrumented, so the [tension-based joint-space
   controller, Humanoids 2016](https://dl.acm.org/doi/10.1109/HUMANOIDS.2016.7803367)
   is the intended control path — LIT Q1b) and the **proximal leg joints (hip,
   knee)** where AIC/`T_bias` matters. Kengoro-class ~540 N cells match our ~525 N
   design load. On both antagonists. Skip the cell on **spring-return distal
   joints** (ankle, ADR-0002 Option B), which have no antagonist stiffness to
   regulate — current-estimate + rotor encoder suffice there.

**This mirrors the intended dual front-end:** a cheap ubiquitous current channel +
a precision load-cell channel where joint-side accuracy is load-bearing —
minimizing parts/mass while giving the AIC loop the true `T_joint` it needs.

**Placement is the non-negotiable design rule (the §0 finding):** the load-cell
channel must sit **at the joint/output end, downstream of the routing friction**,
or the wrap between it and the joint must be minimized (open low-friction pulleys,
per LEG_TENDON_SPEC §3.3). A load cell buried in the girdle reads the same
friction-corrupted `T_motor` a current estimate does — **spend the part only where
you also put it in the right place.**

**SEA is not the sensing baseline** — its stiffness role is redundant with
ADR-0002's active co-contraction, and it shares the load cell's placement
constraint at higher size/bandwidth cost. Reserve it as a per-joint *actuation*
option, not the tension sensor.

### Handoffs
- **→ tomcat-electronics:** implement the per-motor **dual tension front-end** —
  (a) `I_q`-derived `T_motor` estimate feeding the Tier-A latch (ADR-0005), and
  (b) a load-cell/tension-pulley analog front-end wired to the **joint-end** cell
  on stiffness-critical antagonistic joints. Budget the extra wire run to the
  distal cell and its calibration.
- **→ tomcat-mechanical:** package the joint-end tension cell (Kengoro ~540 N class
  ≈ our ~525 N design load) at the **output/joint end**; keep wrap between cell and
  joint minimal. The compact single-pulley + 3D-Hall module is the space-saving
  candidate. This adds a small distal mass — weigh against P1.
- **→ tomcat-kinematics:** `tendon.py` already reports both `T_joint` (from
  `resolve()`) and the capstan-amplified `T_motor`; the load-cell channel calibrates
  the *joint-side* branch, the current channel the *motor-side* branch. The
  capstan factor `exp(μ·θ_wrap)` is the model bridging the two — its **μ and per-
  path wrap are still placeholders** (μ=0.10; LEG_TENDON_SPEC §3.4) and should be
  **bench-identified**, because the whole current-vs-loadcell error budget rides on
  them.

### Open questions / gaps
- **μ and per-tendon wrap are unmeasured** (placeholders). The ~3.6× friction band
  is computed from μ=0.10 and assumed wrap; a bench measurement of the routed μ and
  its drift is needed to size the real current-estimate error and confirm which
  joints genuinely need a cell. ⚠️
- No captured figure for **long-term load-cell drift / temperature sensitivity** at
  our cell class — Kengoro pairs each cell with a temperature sensor, suggesting it
  matters.
- Whether the **spine** joints can be adequately served by tension-only joint-space
  estimation (LIT Q1b) vs. also needing a joint-angle sensor is a control-design
  question for tomcat-kinematics.

---

## Proposed ADR-0004 decision text (for the lead to fold in — not editing the ADR here)

> **Tension-sensing method: hybrid dual front-end.**
> **(a) A motor-current (`I_q`) tension estimate on every tendon** as the always-on,
> kHz, motor-side channel — used for Tier-A over-tension/over-current latching
> (ADR-0005), slack/backdrive detection, and coarse feedforward. It senses the
> motor-side tension `T_motor`, which is the correct quantity for protecting the
> motor and cable, and reuses FOC phase sensing for **zero added parts**.
> **(b) An in-line load cell (or compact tension-sensing pulley) placed at the
> joint/output end** on the **stiffness-critical antagonistic joints** — the spine
> joints (angle not directly instrumentable; tension-based joint-space control per
> LIT Q1b) and the proximal leg joints (hip, knee) — on both antagonists, sized to
> the ~525 N per-tendon design load (Kengoro ~540 N cell class). Spring-return
> distal joints (ankle, ADR-0002 Option B) use the current estimate + rotor encoder
> only.
> **Rationale — friction/placement:** capstan friction over the routed wrap makes
> motor-side and joint-side tension differ by `exp(±μ·θ_wrap)` — for our routing
> (μ≈0.10, θ≈6.5 rad) a **~3.6× direction-dependent band** — so a motor-current
> (or girdle-mounted) estimate **cannot know the tension the joint feels**, and in
> particular cannot regulate the ~20 N co-contraction bias `T_bias`/AIC stiffness
> (ADR-0002). Only a sensor **downstream of the friction, at the joint end**, reads
> `T_joint` directly (confirmed on hardware: motor-end 0.98 N vs joint-end 0.54 N
> for the same event, Sensors 2024). MIT Cheetah's current-based force control does
> **not** transfer because it is direct-drive with no intervening friction path.
> **Series-elastic sensing (SEA) is rejected as the baseline** — its mechanical
> compliance is redundant with ADR-0002's active variable stiffness, it shares the
> load cell's joint-end placement constraint at higher size/bandwidth cost, and
> UHMWPE cable elasticity already supplies some series compliance; SEA remains a
> per-joint *actuation* option, not the tension sensor.
> **Consequences:** electronics owns a per-motor dual tension front-end (current
> estimate + joint-end load-cell analog front-end where specified); mechanical
> packages the cell at the output end with minimal wrap to it; kinematics
> calibrates the `tendon.py` joint-side branch against the cell and the motor-side
> branch against current, with μ and per-path wrap to be **bench-identified**
> (currently placeholders).

---

## Sources
- Kengoro — Kawaharazuka et al., *musculoskeletal humanoid*, arXiv:2409.00705 (✅) — https://arxiv.org/pdf/2409.00705 — 116 muscle modules, load-cell tension unit ~55 kgf, current-controlled BLDC.
- Cable-driven surgical tension sensor array — *Sensors* 24(10):3156, 2024, PMC11125287 (◐ peer-reviewed) — https://pmc.ncbi.nlm.nih.gov/articles/PMC11125287/ — distal placement due to friction; motor-end 0.98 N vs joint-end 0.54 N; 0.173 N avg / 0.213 N RMS accuracy.
- Compact single-pulley + 3D-Hall tension module — *MDPI Actuators* 14(6):278, 2025 (⚠️ author-claimed) — https://www.mdpi.com/2076-0825/14/6/278.
- Tension-based joint-space control for spherical/spine joints — *Humanoids* 2016, DOI 10.1109/HUMANOIDS.2016.7803367 (⚠️ paywalled) — https://dl.acm.org/doi/10.1109/HUMANOIDS.2016.7803367.
- MIT Cheetah proprioceptive actuator — Wensing et al., IEEE T-RO 2017 (◐) — https://www.researchgate.net/publication/312558722 — current-based force control; direct-drive, no cable friction path.
- Non-linear hysteresis compensation of a tendon-sheath manipulator using motor current — IEEE 2021 (⚠️ paywalled abstract) — https://www.researchgate.net/publication/349016843.
- Series Elastic Actuators — Pratt & Williamson, MIT AIM-1524, 1995 (◐) — https://apps.dtic.mil/sti/pdfs/ADA299658.pdf.
- Towards Accurate Force Control of SEAs (transmission force observer) — arXiv:1902.05338 (◐) — https://arxiv.org/pdf/1902.05338.
- Capstan / friction model & our wrap numbers — [LEG_TENDON_SPEC.md §3.4](../../mechanical/LEG_TENDON_SPEC.md), [tendon.py docstring](../../kinematics/src/tomcat_kin/tendon.py).
