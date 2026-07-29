# Motor down-select — candidate, and a scale problem it exposes

Closes the long-standing `G-Kt` / `G-Vbus` gaps in
[electronics/BOARD_OUTLINE.md](../../electronics/BOARD_OUTLINE.md) §0 and tests
the ~31 g/motor assumption behind review finding
[F1](../../mechanical/DESIGN_REVIEW.md).

**Headline:** a suitable motor exists and is recommended below — but plugging its
real mass into the design shows that **a 3 kg body cannot carry 24 tendon
motors**. The blocker is no longer "which motor", it is **how many, on what size
machine**.

---

## 1. The requirement

Motor-shaft torque is `τ_motor = T · r_spool`. The cable's minimum bend diameter
(≥15 mm for the specced 1.5 mm UHMWPE) puts a **floor** under the spool at
`r_spool ≈ 7.5 mm`, so torque cannot be traded away by shrinking the spool.
At the specified `r_spool = 8 mm`, using the worst (hip) tendon:

| Load case | joint τ | tendon T | **τ_motor** |
|---|---|---|---|
| Stand (4-leg, ×1.0) | 1.24 N·m | 49 N | **0.39 N·m** |
| Trot (2-leg, ×1.5) | 3.71 N·m | 138 N | **1.10 N·m** |
| Land (1-leg, ×2.5) | 12.36 N·m | 446 N | **3.57 N·m** |

## 2. Recommended candidate — SteadyWin **GIM3505-9** (or class equivalent)

A quasi-direct-drive joint module: BLDC + **9:1** planetary + integrated driver
and dual encoder. The low ratio is the right family — it matches ADR-0003's
backdrivability intent (MIT Cheetah is 5.8:1), unlike a Kengoro-style 29:1.

| Spec | Value |
|---|---|
| Rated / peak torque | **0.71 / 1.95 N·m** |
| Rated / peak current | 1.60 / 4.19 A → **≈0.44 N·m/A** output |
| Rated voltage (range) | **24 V** (12–40 V) |
| Phase-phase resistance | 4.466 Ω |
| Gear ratio / backlash | 9:1 / 15 arcmin |
| Encoder | 14-bit, dual |
| **Mass (with driver)** | **131.7 g** |

Source: [AIFITLAB GIM3505-9](https://aifitlab.com/products/steadywin-gim3505-9-motor) ·
[SteadyWin](https://steadywin-motor.com/collections/planetary-reduction-series)

### Fit against the requirement

- **Stand 0.39 N·m** — comfortable (1.8× margin on *rated*).
- **Trot 1.10 N·m** — within rated-to-peak. ✅
- **Land 3.57 N·m** — **1.8× OVER peak.** ❌

> The landing case is the *only* one that fails, and it is a deliberately
> aggressive placeholder (single leg, ×2.5). **Recommendation: scope v1 to
> walk/trot** and treat hard single-leg landings as outside the actuator
> envelope, rather than up-sizing every one of 24 motors for a manoeuvre the
> robot is not yet asked to perform. Revisit if ADR-0007 righting/landing becomes
> a near-term goal.

## 3. ⚠️ The blocking finding: motor mass vs. body mass

Review F1 assumed **~31 g/motor**. The real figure is **131.7 g — 4.2× heavier.**

| Configuration | Motors | Motor mass | % of 3 kg body |
|---|---|---|---|
| Full antagonistic (ADR-0002) | 31 | 4083 g | **136 %** ❌ |
| CAD reduced (ankle spring-return) | 24 | 3161 g | **105 %** ❌ |
| + variable-radius pulley | 16 | 2107 g | 70 % ❌ |
| 1 motor/DOF, aggressive | 12 | 1580 g | 53 % ⚠ |

**The motors alone exceed the entire body budget** at the counts the architecture
calls for. Even the most aggressive reduction leaves no room for structure,
battery, electronics and legs.

This is the direct, quantitative consequence of P1: relocating 24–31 "muscles"
into the torso means the torso must actually *carry* 24–31 motors. The tendons
buy light limbs, but they do not make the actuators disappear.

### The three ways out (a real decision, not a calculation)

1. **Scale the machine up.** Holding motors ≤35 % of body mass:
   16 motors → **≥6 kg body**; 24 motors → **≥9 kg body**. A 9 kg
   T.O.M.C.A.T. is a large-Maine-Coon-sized robot — perfectly reasonable, but 3×
   the current NFR target.
2. **Cut the motor count hard.** The variable-radius pulley (RoboCat trick, one
   motor per antagonistic pair) takes 24 → 16; more spring-return joints take it
   lower. Costs independent stiffness control on those joints (ADR-0002).
3. **Cut the torque requirement**, allowing a smaller motor: thinner cable → a
   smaller spool → less motor torque. A 1.0 mm cable (~1.1 kN break) would allow
   `r_spool ≈ 5 mm`, cutting τ_motor by ~37 %, at SF ≈ 2.4 instead of ≥4.

Most likely a combination of 1 and 2. **This should become an ADR** — it changes
NFR5 (mass) and possibly ADR-0002.

## 4. What this unblocks now

- **`G-Kt` → ≈0.44 N·m/A** (output side, from the rated pair; the vendor's
  quoted 0.35 N·m/A appears to be a different reference point — use the
  rated/peak current pairs for sizing).
- **`G-Vbus` → 24 V nominal**, 12–40 V range.
- Per-channel currents: **0.9 A** stand, **2.5 A** trot, 4.19 A peak rating.
  Sizes the FET/shunt class the board outline left parameterized.

### A power finding that falls out

Holding a stance is pure `I²R` loss — no work is done:

| Case | current | I²R per motor |
|---|---|---|
| Stand | 0.88 A | 3.4 W |
| Trot | 2.48 A | 27.4 W |

**Standing still costs ~41 W** across ~12 loaded tendons, dissipated as heat.
That independently confirms the sensorless-FOC note and
[LEG_TENDON_SPEC §4](../../mechanical/LEG_TENDON_SPEC.md): the **power-off
brake is not an optimisation, it is essential** — without it the robot cannot
stand for long on any sane battery.

## 5. Caveats

- One vendor, one datasheet, spec'd from a reseller page — **not** a survey.
  Treat GIM3505-9 as a *representative class*, not a final part. A dedicated
  survey (mjbots, MyActuator, Eaglepower, custom) should precede purchase.
- The "94.5 × 32.3 mm" dimension on the source page is ambiguous and does **not**
  match the Ø16 × 28 mm envelope used in the CAD packaging study — so **F4's
  girdle fit must be re-checked** against real motor geometry.
- Masses assume one integrated module per tendon. The GIM's *built-in* driver
  partly duplicates ADR-0005's separate smart-driver board; using the integrated
  driver could remove ~5 g/channel and a board, at the cost of the custom CAN-FD
  firmware that ADR-0005 specifies.
