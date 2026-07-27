# Sensorless FOC for quasi-static high-torque stance holding

**Question for T.O.M.C.A.T.:** A tendon-driven leg motor (BLDC/PMSM) must hold a
near-static stance under high torque — near-zero speed, high load, for extended
periods. Can it run **encoderless** (sensorless FOC, no rotor position sensor)?
How badly does sensorless FOC degrade at low/zero speed under load?

Informs **ADR-0003** (actuator technology) and **ADR-0004** (sensing).

Confidence legend: ✅ adversarially verified across sources · ◐ primary /
single-read or reputable vendor app note · ⚠️ vendor claim, rule-of-thumb, or
simulation-only.

---

## 1. Why back-EMF sensorless FOC fails near zero speed

The physics is unambiguous and agreed across every source: model-based
(back-EMF / flux observer, EKF, sliding-mode observer) sensorless schemes
estimate rotor position from the **back-EMF, whose amplitude is proportional to
speed**. As speed → 0, the back-EMF → 0, its signal-to-noise ratio collapses,
and position can no longer be observed.

- ◐ imperix (motor-control vendor app note), *I-f startup method*:
  > "the amplitude of the back-EMF is proportional to the speed of the motor,
  > which makes it difficult to estimate the back-EMF accurately at low speed.
  > In particular, **at standstill there is no back-EMF at all**."
  https://imperix.com/doc/implementation/i-f-startup-method

**Minimum reliable speed threshold** (commonly cited):
- ◐ imperix: "the **minimum operating speed should be in the 10–20 % range of
  the nominal speed**." i.e. back-EMF sensorless FOC is not reliable below
  roughly one-tenth of rated speed.
  https://imperix.com/doc/implementation/i-f-startup-method
- ⚠️ Some literature/patents claim back-EMF methods "perform satisfactorily
  above about **2 % of rated speed**" — a best-case figure, design-dependent,
  and still strictly **nonzero**. Treat as optimistic bound, not a design point.
  (US Patent 11,165,375, background art)
  https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11165375

**Position-estimation error at low speed:** sources describe it qualitatively —
error grows without bound as speed drops because the observable signal shrinks
into the noise/inverter-nonlinearity floor. At true standstill the back-EMF
model does not merely degrade, it **fails outright** (no signal to lock to).
This is why every practical drive uses an **open-loop I/f (or V/f) ramp** to
bootstrap the motor up to the ~10 % threshold before switching to closed-loop
sensorless FOC. For our use case — *holding* near zero speed indefinitely, not
ramping through it — this bootstrap does not help; there is no speed to reach.
✅ (consistent across imperix, the I/f-startup literature, and the patent art).

---

## 2. Zero/low-speed alternatives: HF injection & saliency observers

The only sensorless family that works at **standstill** does **not** use
back-EMF. It injects a **high-frequency (HF) voltage signal** (typically
~500 Hz–1 kHz+) and measures the current response, which is modulated by the
motor's **magnetic saliency** (the difference between d- and q-axis
inductances, Ld ≠ Lq). Demodulating that response yields rotor position even at
zero speed.

**Do they give position AND full holding torque at standstill?** Yes, in
principle, for a *salient* machine. HFI runs the current loop normally, so it
can command full torque while tracking position:
- ◐ "A sensorless algorithm can run an IPMSM assuring **constant torque
  production in the whole speed range from standstill to high speeds**" via a
  hybrid HFI-at-low-speed + observer-at-high-speed scheme.
  (Wide-Speed-Range Sensorless Control of IPMSM)
  https://www.academia.edu/93086580/Wide_Speed_Range_Sensorless_Control_of_IPMSM

**Hard requirement — saliency (i.e. an IPMSM):** this is the decisive
constraint for actuator selection.
- ◐ "Surface PMSMs (SPMSMs), which are designed **without inductive saliency,
  are not suitable for HFI** inductive based self-sensing."
  https://www.mdpi.com/2079-9292/13/6/1131
- Interior-PM (IPMSM) and reluctance-salient machines have Ld ≠ Lq and work
  well with HFI. Plain **surface-magnet BLDC (SPMSM)** — the default cheap
  outrunner — has little/no saliency, so HFI is unreliable without exotic
  fixes (saturation-based HF pulse injection, or asymmetric windings that
  deliberately create saliency). ✅

**Costs of HF injection:**
- **Audible noise & EMI** — the injected carrier is in/near the audible band;
  noise/EMI mitigation is an explicit research topic, i.e. it is a real
  drawback, not hypothetical. ◐
  https://link.springer.com/article/10.1007/s43236-024-00971-6
- **Torque ripple** — the HF current component adds ripple; minimizing it is an
  active area. ◐
- **Estimation error, degrees:** realistic reported HFI errors are
  **< ~15 electrical degrees** across operating points. ◐
  https://www.sciencedirect.com/science/article/abs/pii/S037847542030063X
  - ⚠️ One vehicle-IPMSM study reports **~0.002–0.0025°** rotor-angle error at
    zero/low speed, but gives no torque-ripple/noise data and reads as a
    **simulation** result — treat as best-case, not representative hardware.
    https://pmc.ncbi.nlm.nih.gov/articles/PMC9481327/ (f_inj = 1 kHz, 25 V)
- **Load dependence (important for us):** saliency is created partly by
  magnetic **saturation**, which shifts with load current. Under **high load**
  the saturation-induced saliency and cross-coupling move the apparent d/q
  axes, injecting a **load-dependent position-estimation error** — precisely
  the regime (high holding torque) T.O.M.C.A.T.'s stance requires. ◐/✅
  (this is the qualitatively consistent picture across the saliency-tracking
  literature).

---

## 3. Can a PMSM hold rated/high torque at TRUE zero speed sensorlessly?

- **Back-EMF sensorless: no.** Zero speed = zero signal; it cannot hold
  position closed-loop. Failure mode is **loss of synchronization** — the
  estimated angle drifts, the field decouples from the rotor, torque collapses
  or the rotor slips a pole. ✅
- **HFI on an IPMSM: yes, but conditionally.** It can track position and hold
  torque at standstill *if* the machine is genuinely salient and the
  demodulation is compensated for **load-dependent saturation/cross-coupling**.
  Under the high, sustained load in our stance case, uncompensated error can
  approach the point of **losing lock**. This is a tuning- and
  motor-characterization-heavy regime, not plug-and-play. ◐
- **SPMSM (surface BLDC): effectively no** — insufficient saliency for reliable
  HFI. ◐

**Non-observer failure modes at a DC standstill hold** (apply regardless of
sensing method):
- **Thermal / I²R at DC hold** — holding torque at zero speed means (near-)DC
  current concentrated in one or two phases; there is no rotation to share
  heating across windings. Stall/hold current ≈ V ÷ winding resistance, and the
  motor is "prone to overheating and possible damage" at sustained stall; the
  continuous holding torque must be **thermally derated** well below peak.
  Copper loss scales with I², and phase resistance rises ~**0.39 %/°C**, a
  self-reinforcing effect. ◐
  https://www.portescap.com/en/newsroom/whitepapers/2021/12/physical-parameters-affecting-stall-torque-of-a-brushless-dc-motor
- **Cogging / holding at unpowered detents** is small for typical BLDC and
  cannot be relied on for high holding torque — active current is required,
  which reopens the thermal problem above.

> Handoff to **tomcat-mechanical / tomcat-electronics:** a DC standstill hold is
> a thermal-limited operating point. The relevant spec is *continuous* holding
> torque (thermally derated), not peak/stall torque, plus a duty-cycle and
> cooling budget. A tendon drive with a **non-backdrivable reduction or a
> mechanical brake/latch** would sidestep the electrical hold entirely — worth
> weighing in ADR-0003.

---

## 4. Bottom line for T.O.M.C.A.T. — is fully encoderless advisable?

**No — a fully encoderless leg actuator is not advisable for quasi-static,
high-torque stance holding.** Reasoning:

1. Back-EMF sensorless is out by physics at the hold point (needs ≥~10–20 % of
   rated speed; standstill = no signal). ✅
2. The only standstill-capable sensorless method (HF injection) **requires a
   salient IPMSM**, and even then degrades exactly where we need it most —
   **under high load** (saturation-shifted saliency → load-dependent error, risk
   of losing lock) — while adding **audible noise and torque ripple**. ◐
3. A plain surface-magnet BLDC — the likely low-cost default — has too little
   saliency for reliable HFI at all. ◐

A **minimal rotor position sensor is effectively required** for a robust
zero-speed high-torque hold: at minimum **Hall sensors** (commutation +
coarse position), preferably an **absolute encoder** (smooth FOC, exact hold,
stiff position loop). This eliminates the standstill observability problem
entirely and removes the injection noise/ripple penalty.

**Tendon-drive wrinkle (reinforces the same conclusion):** we independently need
**cable displacement / rough joint state** for tendon coordination and slack
detection, so the actuator already carries sensing — adding/collocating a rotor
encoder is a small marginal cost, and the motor-side encoder plus a cable/joint
sensor (string-pot or joint encoder) together give both commutation and joint
state. Going encoderless would save one sensor while making the hardest
operating point (static high-torque hold) the least reliable — a bad trade.

> **Recommendation for ADR-0003 / ADR-0004:** Do **not** specify a fully
> sensorless (encoderless) leg actuator for stance holding. Baseline a **rotor
> position sensor on every leg motor** — absolute encoder preferred, Hall
> sensors as the floor — and treat the DC standstill hold as a
> **thermally-derated continuous-torque** operating point, with a
> non-backdrivable reduction or brake/latch evaluated as a way to offload the
> electrical hold. Keep the separately-required **cable/joint-state sensor**
> (ADR-0004) distinct from the rotor sensor; they serve different loops.
> Sensorless FOC may still be acceptable as a *secondary* mode for dynamic,
> higher-speed gaits, but not as the primary means of static load holding.

---

### Open questions / gaps (not found in this pass)
- No source gave a **quantified holding-torque derating curve** or duty-cycle
  limit for a cat-scale (~tens of W) leg motor at DC standstill — needs a
  thermal calc from the chosen motor's Rth/resistance (tomcat-mechanical).
- No numeric **load-dependent HFI error vs. load current** curve captured here;
  the < 15 elec.° figure is a general operating-range bound, not measured at our
  target torque.
- Unit note for **tomcat-kinematics:** any rotor-angle error (electrical
  degrees) converts to **mechanical** degrees via pole-pair count, then to
  tendon/joint error via the spool radius and moment arm — hand off the
  rotational→linear conversion; do not conflate elec.° with joint °.

### Sources
- imperix, *I-f startup method* — https://imperix.com/doc/implementation/i-f-startup-method (◐ vendor)
- US Patent 11,165,375 (background on back-EMF low-speed limits) — https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11165375 (⚠️)
- *Sensorless Control of Surface-Mounted PMSM in a Wide-Speed Range*, Electronics (MDPI) — https://www.mdpi.com/2079-9292/13/6/1131 (◐)
- *Wide-Speed-Range Sensorless Control of IPMSM* — https://www.academia.edu/93086580/Wide_Speed_Range_Sensorless_Control_of_IPMSM (◐)
- *HF injection-based sensorless position estimation in PMSM*, ScienceDirect — https://www.sciencedirect.com/science/article/abs/pii/S037847542030063X (◐, <15 elec.°)
- *IPMSM rotor position estimation by pulsating HF square-wave injection*, J. Power Electronics — https://link.springer.com/article/10.1007/s43236-024-00971-6 (◐, noise/EMI)
- *Hybrid Pulse HF Voltage Injection Sensorless IPMSM for Vehicles*, PMC — https://pmc.ncbi.nlm.nih.gov/articles/PMC9481327/ (⚠️ sim, 0.002° / 1 kHz / 25 V)
- Portescap, *Physical Parameters Affecting Stall Torque of a BLDC* — https://www.portescap.com/en/newsroom/whitepapers/2021/12/physical-parameters-affecting-stall-torque-of-a-brushless-dc-motor (◐ thermal / 0.39 %/°C)
