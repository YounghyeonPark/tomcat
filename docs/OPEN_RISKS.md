# Open risks and how to close them

The model carries **48 `[owed]` items and 89 `[assumed]` values**. Most of them
cannot change a decision, and treating them as equal work would waste effort on
the wrong ones. This ranks them by **what breaks if the assumption is wrong**, and
says concretely how each is closed.

The short version:

> **Two cheap measurements de-risk almost the whole design: buy and weigh a motor,
> and measure paw friction.** Neither needs the robot to exist. Everything else is
> either low-consequence, or a decision rather than an unknown.

---

## 1. Critical — these can break the design

### R1. Motor mass — 132 g `[assumed from one reseller page]`

**Why it is critical.** The whole mass budget was already re-derived once when the
72 g class target turned out not to exist (ADR-0010). It now rides on a *single
vendor page* for the SteadyWin GIM3505-9, which the down-select note itself calls
"a representative *class*, not a final part".

| real motor mass | body mass | verdict |
|---|---|---|
| **132 g** (assumed) | 4.04 kg | NFR5 holds |
| 160 g | 4.58 kg | **budget breaks** |
| 200 g | 5.34 kg | **budget breaks badly** |

There is only **21 % margin**. And a heavier body raises every torque, which
raises the motor needed — the spiral ADR-0010 showed converges *only* because the
chosen part has headroom.

**How to fix:** buy **one** motor and weigh it. ~£60 and a week's shipping.
Also weigh the driver separately to settle whether the 131.7 g figure includes it.

**Do this first.** It is the cheapest action with the largest blast radius.

### R2. Paw friction μ — assumed ≥ 0.70 `[from literature, never measured]`

**Why it is critical.** μ ≥ 0.70 (NFR16) is met with **no margin at all**:

| floor μ | envelope | NFR15 (48 mm)? |
|---|---|---|
| 0.5 | 40.2 mm | **fails** |
| **0.70** | **48.1 mm** | **exactly meets** |
| 0.9 | 53.9 mm | meets |

This number has had a strange history — withdrawn by ADR-0010 as a non-constraint,
then reinstated by ADR-0019 for a *different mechanism* (the spine's balance action
needs ground reaction). It is now load-bearing and sitting on a literature value
for "PU on concrete", not on this pad, this compound, or the intended floor.

**How to fix:** a **drag test**. Mould two or three TPU pads, load one with a
known weight, pull it across the intended floor with a luggage scale or a
force gauge, and read the ratio. Half a day, no special equipment. Test wet, dusty
and polished cases too — the failure mode is a *bad* floor, not a typical one.

---

## 2. Significant — these change a number, not viability

### R3. Battery energy density — 175 Wh/kg, 80 % usable `[assumed]`

Runtime spans **18–48 min** across plausible cell choices, against the 30 min in
NFR6. Nothing breaks; the specification moves.

**How to fix:** pick a real cell. This is procurement, not analysis.

### R4. The actuation ramp — 37 ms `[modelled, never benched]`

It is the **dominant term** in the balance latency budget (ADR-0016), and it
assumes a trapezoidal move with no torque droop. M12 showed the ramp itself
costs little because the legs are light, but that rests on the operational-space
inertia model.

**How to fix:** one leg on a bench, commanded to step its foot 74 mm mid-swing;
measure the time. Needs a leg to exist — so it is gated on the build.

### R5. Sustained trot thermal duty

Peak 2.79 A against a 4.19 A rating and RMS 0.89 A against 1.60 A rated look
comfortable (ADR-0021), but that is copper loss only — no iron, switching or
gearbox losses, and no thermal model of the girdle.

**How to fix:** run one motor at the trot duty cycle in a representative enclosure
and log case temperature. Gated on having the motor (R1).

---

## 3. Low risk — flagged, but they cannot change a decision

| item | why it is low risk |
|---|---|
| **Structure mass** (296 g modelled vs 587 g allowed) | ~2× headroom; it would have to be twice wrong to matter |
| **Tendon friction μ / wrap** | decides *which* joints need a load cell (ADR-0004), not whether the robot works |
| **Driver board mass** (5 g) | 19 × a few grams, inside the structure margin |
| **Girdle structure allowances** (90 / 110 g) | loosest numbers in the mass model, but small |
| **NFR15 disturbance cases** | `[assumed]` scenarios — but a *stated* basis is the improvement over the capability figure they replaced (ADR-0017) |

These should stay flagged and stay unresolved until something cheap resolves them.

---

## 4. Not unknowns — decisions waiting for an owner

These are **costed options**, not risks. Each has a number attached and needs a
judgement, not an experiment.

| decision | the trade | status |
|---|---|---|
| **Joint moment arms 1.25×** | +23 % runtime (30 → 37 min) for a 70 mm hip sheave, plus mass/packaging/inertia ripple | ADR-0021, **not adopted** |
| **Leg abduction** | +42–58 % disturbance envelope for +4 motors, 528 g (13 % of budget) | ADR-0017, **rejected** — requirement already met |
| **Trot speed 50 cm/s** | slowed from 67 by the spine's friction cost; faster is available on a grippier floor | ADR-0020, **floor-dependent** |
| **Spine lateral ROM ±15°** | ±25° would give 119 mm envelope, no extra motors — but fights the biomechanics (lateral is the stiffest spine axis) | ADR-0015, **held** |

---

## 5. Genuinely unbuilt

**Electronics and firmware have specifications and no implementation** — 2 and 3
markdown files respectively, with one C header between them. That is not a *risk*
in the sense above (nothing is wrong), it is simply the largest piece of remaining
work.

ADR-0016 established these have a **comfortable** budget: the balance loop's
electronics pipeline has ~16 % influence on the envelope, so ordinary engineering
suffices and no exotic timing is needed. The specs to build against exist
([BOARD_OUTLINE](../electronics/BOARD_OUTLINE.md),
[INTERFACE](../firmware/INTERFACE.md), ADR-0005).

---

## 6. What the modelling can still settle on its own

Small, and honestly at the point of diminishing returns:

- **Regeneration** — currently credited at zero, so 27 W of mechanical work is
  written off. A backdrivable drive recovers some of it, which would improve NFR6.
- **Trunk / dorsoventral `dH/dt`** — the last unmodelled angular-momentum terms.
  Expected small by the same argument that made link spin 3 % (ADR-0018), but that
  is an expectation, and this project has been wrong about exactly that kind of
  expectation four times.

---

## Recommended order

1. **Buy and weigh a motor** (R1) — largest blast radius, ~£60.
2. **Drag-test a TPU pad** (R2) — half a day, no equipment.
3. **Pick a cell** (R3) — closes NFR6 properly.
4. Then **build electronics/firmware** (§5), which is gated on none of the above.
5. Bench R4/R5 once a leg exists.

Steps 1–3 are perhaps a week of elapsed time and a hundred pounds, and they
convert the three load-bearing assumptions in this project into measurements.
