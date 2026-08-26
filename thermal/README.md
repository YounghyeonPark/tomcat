# Thermal duty — closing OPEN_RISKS R5

```
cargo test --release      # 17 assertions on the conclusions below
cargo run  --release      # the report
cargo run  --release --example winding   # the winding gradient
```

Built on **dualis 0.3**. The pack is a domain on the same bus as the girdle, so
`Simulation::advance` runs the kernel's conservation audit every step and the runtime
is **emergent** (42 Wh at 83.6 W -> 30.17 min, against `power.py`'s 30.16). One test
feeds it a deliberately leaky pack and asserts the audit **refuses** it — an audit
that cannot fail is decoration.

## Why this is Rust, in a Python repo

It uses [`dualis-thermal`](https://crates.io/crates/dualis-thermal) — lumped masses,
conduction, radiative and convective loss. Nothing else here needs Rust, and nothing
here depends on this crate: it is a leaf. The 332 Python tests do not require a
toolchain, and `tests/test_thermal_constants.py` is the only link, guarding the
handful of numbers copied across.

## What it answers

[ADR-0021](../docs/DESIGN_DECISIONS.md) checked the motor **electrically** — 2.79 A
peak against a 4.19 A rating, 0.89 A RMS against 1.60 A — and called it comfortable.
That is not the thermal question. The thermal question is whether the heat can
*leave*, and R5 sat as "gated on hardware" because of it. It is not gated on
hardware.

> ⚠️ **Every number below rose in M40** ([ADR-0045](../docs/DESIGN_DECISIONS.md)).
> `power.py` computed copper loss as `I²R_pp` where balanced three-phase is
> `3I²R_ph = 1.5×` that. Per-motor dissipation went **3.50 → 5.25 W** and the whole
> chain moved with it. Superseded figures are struck through.

| front girdle, 6 motors | continuous | one battery |
|---|---|---|
| trot, polished | **155.2 °C** ~~113.7~~ | 78.3 °C ~~67.1~~ |
| trot, anodised | **96.1 °C** ~~74.9~~ | **70.2 °C** ~~59.7~~ |
| stand w/o brake, polished | **183.9 °C** ~~134.1~~ | 97.2 °C ~~85.7~~ |
| stand w/o brake, anodised | **110.2 °C** ~~85.4~~ | 84.5 °C ~~72.5~~ |

⚠️ **The headline conclusion is overturned.** This file used to say *"anodised, it is
safe because its own equilibrium is ~75 °C"*. It is **96.1 °C**, and the
battery-limited anodised trot lands at **70.2 °C** — on the wrong side of the 70 °C
line it was comfortably inside before.

**The battery is still the thermal protection, and it is now doing more of the work,
not less.** A bare girdle's effective time constant (~49 min) exceeds the runtime it
can be fed for (24 min), so the robot still runs out of energy before it overheats.
Tether it, or hot-swap the pack, and the protection is gone — and now the anodised
case needs it too.

⚠️ **Forced air moves from "would reopen it" to "required for continuous
operation".** At `h = 15` the anodised girdle settles at **72.7 °C**; at `h = 25`,
58.2 °C. Still air cannot hold a continuous trot at any finish.

⚠️ **Corrected.** The 53 min first published came from
`LumpedMass::time_constant`, which is `C/(hA)` — **convection only**, so it returns the
same number for a polished and an anodised body. Measured from the transient:
**46.6 min polished, 25.6 min anodised.** The temperatures were always computed with
radiation and did not move; the *mechanism* was misstated. Anodised, the girdle does
not outlast the pack at all — it is safe because its own equilibrium is ~75 °C.

Three things follow that were not visible from the electrical check:

- **Anodising is worth ~59 K** (~~39 K~~ — the lever *grew*, because radiation goes
  as the fourth power of temperature and M40 pushed the operating point up).
  Radiation is the same order as still-air convection here, so emissivity
  0.09 → 0.90 remains the cheapest lever available. ⚠️ **It is no longer
  sufficient on its own** — see the overturned conclusion above. Surface finish is a
  thermal parameter here, not a cosmetic one.
- **Centralising the motors costs 38 % of the rejection area** (486 → 302 cm²).
  This is the first measured place where design principle **P1 charges** rather
  than pays.
- **Standing without the brake is the worst case** — worse than trotting, because a
  cable can only pull. ADR-0021 reached the same conclusion about the brake from
  runtime; this reaches it from heat.

## The winding

`winding.rs` answers what this file used to warn about. dualis 0.3's `ThermalNetwork`
(from [upstream #2](https://github.com/YounghyeonPark/dualis/issues/2)) makes
**winding → stator → housing → girdle → air** expressible:

| | winding | stator | skin |
|---|---|---|---|
| polished, continuous | **166.7 °C** ~~121.4~~ | 160.9 ~~117.5~~ | 155.2 ~~113.7~~ |
| anodised, continuous | **107.6 °C** ~~82.6~~ | 101.8 ~~78.7~~ | 96.1 ~~74.9~~ |

**+11.5 K** (~~+7.7 K~~), the same for both finishes — the finish sets where the
stack sits, the joints set the spread. The gradient scales with dissipation, so M40
moved it too. The joints are `[assumed]`, so the sweep is the result rather than any
single number.

## ⚠️ Limits

The joint conductances set the entire gradient and are guesses. Copper loss is the only
source (inherited from ADR-0021), so the real dissipation is higher. The girdle
envelope, `h`, and both emissivities are `[assumed]` — every result is swept rather
than stated once.

**This does not close [R1 or R2](../docs/OPEN_RISKS.md).** Simulation checks the
model; only a measurement checks the inputs.
