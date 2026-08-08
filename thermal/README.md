# Thermal duty — closing OPEN_RISKS R5

```
cargo test --release      # 7 assertions on the conclusions below
cargo run  --release      # the report
```

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

| front girdle, 6 motors | continuous | one battery |
|---|---|---|
| trot, polished | **113.7 °C** | 67.1 °C |
| trot, anodised | 74.9 °C | 59.7 °C |
| stand w/o brake, polished | **134.1 °C** | 85.7 °C |
| stand w/o brake, anodised | 85.4 °C | 72.5 °C |

**The battery is the thermal protection, and that is a coincidence.** The girdle's
time constant (53 min) is longer than the runtime it can be fed for (30 min), so the
robot runs out of energy before it overheats. Tether it, or hot-swap the pack, and
the protection is gone.

Three things follow that were not visible from the electrical check:

- **Anodising is worth ~39 K.** Radiation is the same order as still-air convection
  at these temperatures, so emissivity 0.09 → 0.90 is the cheapest lever available.
  Surface finish is a thermal parameter here, not a cosmetic one.
- **Centralising the motors costs 38 % of the rejection area** (486 → 302 cm²).
  This is the first measured place where design principle **P1 charges** rather
  than pays.
- **Standing without the brake is the worst case** — worse than trotting, because a
  cable can only pull. ADR-0021 reached the same conclusion about the brake from
  runtime; this reaches it from heat.

## ⚠️ Limits

These are **assembly-skin** temperatures. A lumped mass has one temperature; the
real winding is hotter, and the winding is what fails. Copper loss is the only
source (inherited from ADR-0021), so the real dissipation is higher. The girdle
envelope, `h`, and both emissivities are `[assumed]` — every result is swept rather
than stated once.

**This does not close [R1 or R2](../docs/OPEN_RISKS.md).** Simulation checks the
model; only a measurement checks the inputs.
