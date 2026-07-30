# Motor reality check — the 72 g class target does not exist

Owner: **tomcat-lead** · Closes the ❓ that has sat in
[REQUIREMENTS §5](../REQUIREMENTS.md) since M1: *"the specific motor part — the
class is now specified (~1.1 N·m peak, ≤80 g) but a real part must be surveyed
and its torque density confirmed — ADR-0008's mass closure rides on it."*

It does not close well. **The class target was optimistic by ~1.7× in mass, and
the 3.0 kg body does not fit around real hardware.**

Label legend: `[sourced]` from a vendor/reseller datasheet · `[derived]`
calculated here · `[assumed]` engineering guess.

---

## 1. What was assumed

| | Value | Where from |
|---|---|---|
| Motor mass | **72 g** | ADR-0008 class target |
| Driver board | **5 g** | `[assumed]` |
| Peak torque needed | **1.10 N·m** at the shaft | trot sizing, [motor-downselect](motor-downselect.md) |
| Implied torque density | **15.3 N·m/kg** | `[derived]` |

## 2. What is actually purchasable

Surveyed the SteadyWin GIM planetary/QDD family — the class the earlier
down-select had already identified as the right architecture (a low-ratio
planetary is *quasi*-direct-drive, so it keeps the backdrivability ADR-0003
requires, unlike a high-ratio gearhead).

| Part | Output torque (rated / peak) | Ratio | Mass | Note |
|---|---|---|---|---|
| **GIM3505-8** | 0.65 / **1.27** N·m | 8:1 | ~120 g | meets 1.10 N·m `[sourced]` |
| **GIM3505-9** | 0.71 / **1.95** N·m | 9:1 | **120 g bare, 131.7 g with driver** | Ø34.5×36.1 mm, 24 V, 380 rpm, Kt 0.35 N·m/A `[sourced]` |
| GIM4305-10 | 1.0 / 3.0 N·m | 10:1 | 140 g | Ø53×26 mm `[sourced]` |

**Nothing in this class comes near 80 g at ~1.1 N·m.** The floor is ~120 g.

The torque *density* assumption was not the problem — GIM3505-9 achieves
1.95 N·m / 0.120 kg = **16.3 N·m/kg**, comfortably above the 15.3 the target
implied. The problem is that **motors come in discrete sizes**, and the smallest
one that clears the torque bar is roughly twice as capable (and twice as heavy)
as strictly required.

⚠️ Note the quoted torques are at the **output shaft, after the gearbox**. The
rotor itself produces ~1/9 of that. Any future attempt to drop the gearbox for a
frameless/direct-drive gimbal must re-derive from ~0.2 N·m, not 1.95.

## 3. The consequence: 3.0 kg does not close

| Item | Mass |
|---|---|
| 19 × 131.7 g (motor + integrated driver) | **2502 g** |
| 4 legs (bottom-up, unchanged) | 410 g |
| Battery `[assumed]` | 300 g |
| Head/neck `[assumed]` | 240 g |
| Structure (girdles + spine) | 587 g |
| **Total** | **≈ 4.04 kg** |

**NFR5 was therefore raised 3.0 → 4.05 kg.** Two things make that tolerable:

1. **It is more biomimetic, not less.** A domestic cat is 4–5 kg. The 3.0 kg
   figure was a placeholder, never derived from the animal.
2. **It converges — it does not spiral.** The classic actuator-mass trap is that
   heavier motors need more torque, which needs heavier motors. Iterating the
   fixed point here settles at **2.67 kg** *if* a motor could be bought at exactly
   the requirement (≈60 g). The design lands at 4.04 kg only because of the
   discrete-size floor, and the chosen part still has **1.3× peak headroom** at
   the heavier mass (1.48 N·m needed vs 1.95 available `[derived]`).

⚠️ **But sustained trot exceeds the continuous rating.** At 4.05 kg the trot hip
needs 1.48 N·m against a **0.71 N·m rated** figure — a 2.1× overload. That is
survivable as a *transient* but not as a duty cycle. ADR-0008 sized to trot using
**peak**, which the [down-select](motor-downselect.md) did state ("within
rated-to-peak"), so this is a known consequence rather than a new error — but it
means **sustained trotting is thermally limited**, and the stance brake
(ADR-0003) matters more than previously credited.

## 4. Levers if 4.05 kg is later judged unacceptable

| Lever | Effect | Cost |
|---|---|---|
| **Larger joint moment arms** | `τ_motor = τ_joint · r_spool/r_joint` — doubling `r_joint` halves motor torque | sheaves get large; LEG_TENDON_SPEC already notes geometry alone cannot fix the land case |
| **Per-joint motor sizing** | hip 1.10 / stifle 0.76 / hock 0.86 N·m `[derived]` — the spread is only ~30 % | small saving, more part numbers |
| **Fewer motors** | directly proportional | ADR-0009 established 19 is the minimum for static stability |
| **Higher gear ratio** | smaller motor for the same output | costs backdrivability, the thing ADR-0003 is built on |

The moment-arm lever is the only one with real headroom, and it is a mechanical
redesign, not a parameter change.

## 5. What is still owed

- `[owed]` **Buy and weigh one.** Every number above is vendor/reseller data.
- `[owed]` **Thermal test at the trot duty** — §3's 2.1× continuous overload is
  the sharpest open risk in the whole actuator story.
- `[owed]` **Confirm the driver is integrated** in the 131.7 g figure for the
  variant actually chosen; if an external driver is needed, add ~12 g × 19.
- `[owed]` Battery selection (300 g `[assumed]`, 7 % of the new budget).

## Sources

- [SteadyWin GIM3505-9 (AIFITLAB)](https://aifitlab.com/products/steadywin-gim3505-9-motor)
- [SteadyWin GIM3505-8 (steadywin-motor.com)](https://steadywin-motor.com/products/built-in-planetary-reduction-motor-aloha-accessories-quadruped-robot-joint-module)
- [GIM4305-10 (Amazon listing)](https://www.amazon.com/GIM4305-Brushless-Gear-Motor-53mm/dp/B0F59NQZHK)
- [SteadyWin product series overview](https://steadywin-motor.com/products/introduction-ofproduct)
