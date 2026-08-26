# Motor spec review — the GIM3505-9, re-checked at 4.30 kg

Owner: **tomcat-lead** · Milestone **M39** · Ties to
[ADR-0008](../DESIGN_DECISIONS.md), [ADR-0010](../DESIGN_DECISIONS.md),
[ADR-0021](../DESIGN_DECISIONS.md), [ADR-0043](../DESIGN_DECISIONS.md).
Supersedes parts of [motor-downselect](motor-downselect.md) and
[motor-reality-check](motor-reality-check.md).

OPEN_RISKS **R1** — *buy one motor and weigh it* — is still the cheapest action
with the largest blast radius, and it is still open. This note is what can be
settled **without buying anything**: every actuator number re-derived from the
vendor's own published figures at the body mass [ADR-0043](../DESIGN_DECISIONS.md)
closed on, plus a check of the published figures against each other.

Everything here is computed by `tools/motor_spec_review.py` and gated by
`tests/test_motor_spec.py`, so it cannot drift from the model.

Label legend: `[sourced]` vendor/reseller · `[derived]` computed here ·
`[assumed]` engineering guess.

---

## 0. Verdict

**The motor holds. The runtime requirement does not.**

| question | answer |
|---|---|
| Torque, trot | **88 % of peak** — inside, with less headroom than recorded |
| Torque, continuous | 2.40× the *rated* figure — but see §3, this is not the duty |
| Thermal duty (RMS current) | **0.64–0.81× the rating** — comfortable ✅ |
| Speed | 7.8–8.5 m/s foot ceiling vs 4.1 m/s needed ✅ |
| **Runtime (NFR6)** | **14–20 min** against a published ~30 ❌ |
| Mass fraction | **58.1 % of the robot is motor** |
| Down-select | GIM3505-9 stays the pick; the **GIM3505-8 now fails** |

---

## 1. ⚠️ The vendor's three published numbers disagree by 27 %

The same motor, three readings of its torque constant:

| reading | Kt (N·m/A) |
|---|---|
| rated pair, 0.71 N·m / 1.60 A | **0.444** `[derived]` |
| peak pair, 1.95 N·m / 4.19 A | **0.465** `[derived]` |
| vendor quoted | **0.350** `[sourced]` |
| `power.KT`, in use | **0.440** |

[motor-downselect §4](motor-downselect.md) took 0.44 from the current pairs and
dismissed the quoted 0.35 as *"a different reference point — use the rated/peak
current pairs for sizing"*. The two pairs **do** agree with each other to 5 %, so
that is a defensible reading.

⚠️ **But it is the optimistic branch, and nothing has ever swept it.** Current is
`tau/Kt` and copper loss is `I²R`, so the 27 % spread is worth

    (0.444 / 0.350)² = **1.61× of dissipation**

and [ADR-0021](../DESIGN_DECISIONS.md)'s runtime *and*
[ADR-0023](../DESIGN_DECISIONS.md)/[0024](../DESIGN_DECISIONS.md)'s thermal duty
both ride on it. §3 and §4 give both branches.

### 1a. It is not a rotor-side figure — that was tested and fails

The obvious first hypothesis: 0.35 N·m/A is quoted at the **rotor**, before the
9:1 planetary. Then the output constant would be `0.35 × 9 = 3.15 N·m/A`, and:

| | implied | vendor publishes |
|---|---|---|
| current for rated 0.71 N·m | **0.225 A** | 1.60 A |
| current for peak 1.95 N·m | 0.619 A | 4.19 A |

**7.1× off, and in the wrong direction** — rotor-side makes the discrepancy seven
times worse rather than explaining it. Ruled out.

### 1b. What fits is a drive/current convention, and it means both numbers are right

Same shaft, two conventions. The ratio to explain is `0.444 / 0.350 = 1.2679`:

| candidate | factor | implied Kt | error |
|---|---|---|---|
| **4/π** — square-wave fundamental (six-step vs sinusoidal) | 1.2732 | 0.4456 | **+0.4 %** |
| π/√6 | 1.2825 | 0.4489 | +1.2 % |
| √(3/2) — RMS vs peak phase | 1.2247 | 0.4287 | −3.4 % |
| √3 — line vs phase | 1.7321 | 0.6062 | +36.6 % |

⚠️ **Fitting one ratio against a list of constants is weak evidence**, and 4/π
landing inside half a percent could be coincidence. What it buys is a *sharper
question*: not *"which of your numbers is wrong"* but

> **are the 1.60 A / 4.19 A ratings six-step or sinusoidal, and is the 0.35 N·m/A
> peak-phase or RMS?**

Under a convention difference **both published numbers are correct**, and the only
thing that decides which Kt to compute with is what the integrated driver's
current sense actually reports. `[owed]` — still an email, now a better one.

### 1c. ⚠️ And a firmer factor, found while digging: the three-phase convention

Copper loss is `Σ I_ph,rms² R_ph`. Balanced three-phase, and with a wye winding's
terminal phase-to-phase resistance being `2 R_ph`, that is

    P = 3 I_rms² R_ph = **1.5 × I_rms² R_pp**

`power.py` computes `I² R_pp` — **1.5× low** for whatever current it is handed. Its
own docstring flags the simplification (*"a rigorous three-phase treatment would
use 1.5 · I_phase² · R_phase"*) and nothing had ever priced it.

⚠️ **The two factors are entangled, not independent.** Whether the current
`power.py` computes *is* the RMS phase current depends on the same convention
ambiguity as §1b, so they bracket the answer rather than multiplying cleanly. The
1.5× is firm as a *formula* correction; which Kt to feed it is open.

---

## 2. Torque — inside peak, with less headroom than the record says

At ADR-0043's **4.304 kg** and the **8.75 mm** spool
[LEG_TENDON_SPEC §2](../../mechanical/LEG_TENDON_SPEC.md) actually requires:

| case | motor torque | vs rated 0.71 | vs peak 1.95 |
|---|---|---|---|
| stand | 0.598 N·m | 0.84× | 31 % |
| **trot** | **1.706 N·m** | **2.40×** | **88 %** |
| land (×2.5, 1 leg) | 5.585 N·m | — | 286 % |

Two things moved since the record was written:

- `motor-reality-check §3` quotes **"1.3× peak headroom (1.48 vs 1.95)"** — i.e.
  77 % of peak. That was at 4.045 kg with the 8.0 mm spool. **It is now 88 %.**
- **The spool change costs 8 points of that margin.** `tau_motor = T · r_spool`,
  so 8.0 → 8.75 mm is a straight 9.4 % on motor torque. It buys the same 9.4 % of
  foot speed (§5), so it is a trade rather than a loss — but it is a trade nobody
  had priced.

The **land** case at 286 % of peak is not a finding: ADR-0008 explicitly scoped
the ×2.5 single-leg landing *outside* the actuator envelope, where it sizes cable,
pulley and bearing only.

---

## 3. Thermal duty — comfortable, and a correction to how I first read it

⚠️ **A workspace peak is not a duty cycle.** `torque_budget.evaluate` sweeps the
whole reachable foot workspace and returns the worst pose. That is the right basis
for sizing a *structure* and the wrong one for *temperature*. A first pass of this
review compared §2's "2.40× rated" against the continuous rating and called it a
thermal violation. It is not one.

What sets temperature is the RMS over the trajectory actually walked, which
`power.gait_power` integrates:

| Kt | RMS current | vs 1.60 A rating | peak | vs 4.19 A | copper |
|---|---|---|---|---|---|
| 0.44 (in use) | 1.03 A | **0.64×** | 3.25 A | 0.78× | 56.9 W |
| 0.35 (vendor) | 1.30 A | **0.81×** | 4.09 A | 0.98× | 90.0 W |

**Both branches sit inside the continuous rating.** Even believing the vendor's
Kt, the motor is thermally adequate at the trot duty — which is the single sharpest
`[owed]` item `motor-reality-check §5` left open, now answered on spec.

⚠️ The pessimistic branch puts *peak* current at **0.98×** the 4.19 A rating,
i.e. no margin at all. That is a driver-sizing note as much as a motor one.

---

## 4. ⚠️ Runtime — this is what breaks

| basis | copper | total | runtime |
|---|---|---|---|
| published (4.045 kg, 8.0 mm, Kt 0.44) | 42.0 W | 83.6 W | **30.2 min** |
| at 4.304 kg + the 8.75 mm spool | 56.9 W | 100.2 W | 25.2 min (−17 %) |
| **+ the ×1.5 three-phase correction** | 85.4 W | 128.6 W | **19.6 min** |
| on the vendor's Kt, as modelled | 90.0 W | 133.2 W | 18.9 min |
| **on the vendor's Kt, ×1.5** | 135.0 W | 178.2 W | **14.1 min** |

**NFR6 publishes "~30 min / ~900 m". The honest bracket is 14–20 min** — the two
rows carrying the §1c correction, since that applies under either Kt reading.

⚠️ **My first pass quoted 19–25 min** by leaving the copper-loss formula
uncorrected. The mass and spool corrections (−17 %) are *settled*; the ×1.5 is
*firm as a formula*; only the spread between 14 and 20 is hostage to §1b.

---

## 5. Speed — comfortable

380 rpm at the output, through the tendon ratios:

| spool | cable speed | joint rates (hip/knee/ankle) | foot ceiling |
|---|---|---|---|
| 8.00 mm | 0.318 m/s | 11.4 / 12.7 / 22.7 rad/s | 7.76 m/s |
| **8.75 mm** | 0.348 m/s | 12.4 / 13.9 / 24.9 rad/s | **8.49 m/s** |

`control.py` quotes a **5.93 m/s** ceiling, so it is conservative against this
sum-of-joints figure — a different and safer convention. **NFR14** needs 4.1 m/s
of *spare* foot speed and there is room. Speed is not a constraint anywhere in
this design.

---

## 6. ⚠️ Mass — the robot is 58 % motor

| basis | motors | fraction | left for everything else |
|---|---|---|---|
| params, 4.045 kg | 2.502 kg | 61.9 % | 1.543 kg |
| **M38, 4.304 kg** | **2.502 kg** | **58.1 %** | **1.802 kg** |

**ADR-0008's amendment quotes 45.6 %.** That is 19 × 72 g of a 3.0 kg body — the
class target that does not exist, at a body mass that was superseded. Both halves
of the figure are stale and it should be re-stated.

The 1.802 kg remainder has to cover spine, both girdles, the ribcage, the 300 g
`[assumed]` battery, 19 driver boards + controller + SBC, head/neck (240 g
`[assumed]`) and the tail. It is not obviously infeasible, but it is the tightest
it has ever been and nothing has checked it bottom-up.

---

## 7. The down-select, re-run — and the small part now fails

The original down-select sized to a **1.10 N·m** trot at a 3.0 kg body. The
requirement is now **1.71 N·m at 4.30 kg**, so it is worth redoing rather than
inheriting. Each candidate's own mass feeds back into the body it has to lift:

| part | rated | peak | mass | body it makes | torque needed | verdict |
|---|---|---|---|---|---|---|
| GIM3505-8 | 0.65 | 1.27 | 120 g | 4.082 kg | 1.618 N·m | ❌ **OVER PEAK** |
| **GIM3505-9** | 0.71 | 1.95 | 131.7 g | 4.304 kg | 1.706 N·m | ✅ 88 % of peak |
| GIM4305-10 | 1.00 | 3.00 | 140 g | 4.462 kg | 1.769 N·m | ✅ 59 % of peak |

- ⚠️ **The GIM3505-8 was listed in `motor-reality-check §2` as *"meets 1.10 N·m"*.
  At the current requirement it is over its peak.** The mass growth removed a
  candidate.
- **GIM4305-10 is the escape hatch** if 88 % of peak is judged too tight: it drops
  the trot to 59 % of peak and 1.77× rated for **+158 g** of motor. But it is
  **Ø53 against Ø34.5 — 54 % wider** (though shorter, 26 vs 36.1 mm), and the
  packaging study sized the girdles around Ø34.5×36.1. That is a girdle repackage,
  not a part swap. `[sourced: reseller listing, not a manufacturer sheet]`

**Recommendation: keep the GIM3505-9.** 88 % of peak on a case that is a
workspace worst-pose, with the thermal duty comfortable, is an acceptable place to
be. Price the GIM4305-10 properly only if the girdle has to be repackaged anyway.

---

## 8. What is still owed

- `[owed]` **Buy one and weigh it.** R1 is unchanged by any of this. Every number
  above is vendor data, and §1 shows the vendor data is not self-consistent.
- `[owed]` **Ask the vendor the §1b question** — *are the current ratings six-step
  or sinusoidal, is Kt peak-phase or RMS* — which is the cheapest way to close the
  14-versus-20 spread, and costs an email. Rotor-side is already ruled out (§1a).
- `[owed]` **Fix `power.py`'s copper-loss formula** to the rigorous three-phase
  form (§1c). This one needs no vendor and no purchase — it is arithmetic — and it
  moves ADR-0021's runtime *and* ADR-0023/0024's thermal duty.
- `[owed]` **Confirm the driver is integrated** in the 131.7 g figure for the
  variant actually ordered; an external driver adds ~12 g × 19 = 228 g.
- `[owed]` **Bottom-up check of the 1.802 kg non-motor remainder** (§6).
- `[owed]` **Cell selection** — the 300 g / 175 Wh/kg / 80 %-usable pack is three
  assumptions stacked, and §4's runtime rests on all of them.
