# Mass-budget re-check — closing ADR-0009's ⚠️

Owner: **tomcat-lead** · Closes the ⚠️ recorded in
[ADR-0009](../DESIGN_DECISIONS.md): *"the structure budget should be re-checked
against real printed-part masses before this is treated as settled."*

The re-check found something bigger than the structure line: **the actuation
itself was mis-modelled by ~347 g**, and the spine motors were in the wrong
place. Correcting both changes the fore/hind split, the body CoM, and several
published conclusions.

Label legend: `[sourced]` traceable to a spec/ADR · `[assumed]` first-pass guess
· `[owed]` needs a vendor part or a test.

---

## 1. The drift

`params.py` apportioned girdle mass as **31 channels × 36 g**, where 36 g = a
31 g motor + a 5 g driver board. Both halves of that were stale:

| Error | Direction | Size |
|---|---|---|
| **31 channels** — a count from before [ADR-0008](../DESIGN_DECISIONS.md)'s variable-radius pulley cut the build to one motor per DOF | over-counts | **−432 g** |
| **31 g motor** — an assumed Ø16×28 mm envelope, superseded by the [down-select](motor-downselect.md)'s ~72 g Ø36 pancake class | under-weighs | **+779 g** |
| **net** | | **+347 g light** |

The two errors had opposite signs, which is exactly why neither was caught: the
girdle totals looked plausible while both inputs were wrong.

There was a **placement** error too. The apportionment put the spine and tail
motors in the **rear girdle**; the CAD packaging puts them in a **mid-body bay**
between the girdles, ~100 mm forward. That mislocated ~0.5 kg.

## 2. The correction

Bottom-up, with 19 motors (12 leg + 6 spine + 1 tail) at 72 g + 5 g driver:

| Item | Mass | Where |
|---|---|---|
| Front girdle | **792 g** | 6 leg motors (462) + head/neck 240 `[assumed]` + structure 90 `[assumed]` |
| Rear girdle | **572 g** | 6 leg motors (462) + structure 110 `[assumed]` |
| Spine segments | **1226 g** | 130 / **969** / 127 rear→front |
| ↳ of which mid segment | | battery 300 `[assumed]` + spine/tail bank 539 + structure 130 |
| Four legs | **410 g** | unchanged, bottom-up from specced hardware `[sourced]` |
| **Total** | **3000 g** | NFR5 held exactly |

The pelvis is now the **lighter** girdle — the opposite of the pre-correction
model — because the spine bank left it.

## 3. What moved

| | Before | After |
|---|---|---|
| Fore/hind split | 51.2 / 48.8 | **55.0 / 45.0** |
| Body CoM (x) | +100.1 mm | **+107.5 mm** |
| Worst fore-aft margin | +32.7 mm | **+40.2 mm** ✅ better |
| Worst polygon margin | +8.4 mm | **+10.1 mm** ✅ better |
| Optimal lateral sway | 13.5° | **12.5°** |
| Crossover friction margin | 11 % | **14 %** (μ ≥ 0.70) |
| Base spine joint, quiet stand | 0.13 N·m | **0.29 N·m** ⚠️ worse |
| Quiet-stand spine tension | 12 N | 14.6 N (still below the 20–70 N band) |

**The M5 stability conclusions survive and mildly improve.** Moving mass forward
and off the rear girdle centres the body better fore-aft, and the slightly
larger sway authority means less commanded bend for the same effect — which in
turn lowers the crossover acceleration and buys friction margin.

**One published conclusion is partly walked back.** Review finding F2 concluded a
near-balanced body "barely loads the base joint in quiet stand" (0.13 N·m). At
55/45 that becomes **0.29 N·m**, roughly double — still half the 0.57 N·m of the
originally tuned 60/40 model, so F2's *direction* holds but its *magnitude* was
optimistic because it under-weighed the actuators.

## 4. The structure line — ADR-0009's actual question

ADR-0009 worried that structure margin had fallen to 19.6 % (587 g). With the
corrected actuation the residual available for structure is **387 g** inside the
spine chain, plus the 90 g / 110 g girdle allowances — **587 g total**, which
happens to match ADR-0009's figure by a different route.

Against that, the first estimate from **real CAD geometry** (`tomcat_skeleton`,
solid volumes × PA12 1.01 g/cm³, excluding the electronics-bay envelope which is
a volume placeholder rather than a part):

| Group | Volume | Solid PA12 |
|---|---|---|
| bones (vertebrae, ribs, scapula/pelvis, skull, tail, limb bones) | 266.2 cm³ | 268.9 g |
| flat plates | 20.8 cm³ | 21.0 g |
| joint hardware | 6.4 cm³ | 6.4 g |
| **total** | **293.4 cm³** | **296 g** |

**≈296 g modelled against a 587 g allowance — it closes with ~2× headroom.**

⚠️ Do not over-read that. The skeleton is a **massing model**: its bones are
solid cylinders. Real limb bones are hollow CF tube (lighter), while real
brackets carry walls, bosses and fastener features (heavier), and SLS parts are
not solid. The honest conclusion is **directional**: the structure line is not
the binding constraint, and ADR-0009's ⚠️ can be downgraded from *"may not
close"* to *"closes on a massing estimate"*. It is **not** closed outright.

## 5. What is still owed

- `[owed]` **Real part designs** with walls/bosses, and limb bones counted at CF
  tube density, before the 296 g is trusted as anything but an order of magnitude.
- `[owed]` **Battery selection** — 300 g is still `[assumed]` and is 10 % of the
  whole budget.
- `[owed]` **Driver board mass** — 5 g `[assumed]`; the real figure comes from the
  [board outline](../../electronics/BOARD_OUTLINE.md).
- `[owed]` **Girdle structure allowances** (90 g / 110 g) are the loosest numbers
  in §2 and were never derived from geometry.
- `[owed]` **Motor mass confirmation** — 72 g is the down-select's *class* target;
  the surveyed GIM3505-9 is 132 g. If the real part lands nearer that, the
  correction above under-states the problem badly and the budget does not close.
