# Bench plan: what to measure before the next design decision

M132–M150 moved most of the remaining uncertainty out of the model and into
four physical numbers nobody has measured. The simulator randomises over each
of them (ADR-0084, ADR-0137), but the ranges themselves are guesses. Training
more against a guessed range sharpens the policy around the guess. This plan
measures them, in the order that unblocks the most.

Each measurement below says what decision it settles, how to take it, and
which constants and tests move when the number comes in.

| # | what | settles | blocked on |
|---|---|---|---|
| B1 | one stock motor: mass, Kt, R, Kv, heating | stock part or rewind; thermal margin; NFR5 | buying one motor |
| B2 | conduit friction μ | NFR6 runtime margin; the plant's friction | a conduit and cable sample |
| B3 | paw-pad friction μ | NFR15; the floor range trained on | a pad sample |
| B4 | G3 flexure rate | the drivetrain's stiffness range | one flexure |
| B5 | battery cell | NFR6's other half | choosing a cell |

The leg-level items, the actuation ramp (R4) and sustained thermal duty (R5),
need a built leg and are out of scope here.

---

## B1. One stock motor (SteadyWin GIM3505-9)

**Why first.** It is the cheapest measurement with the largest blast radius
(OPEN_RISKS R1). Since M140 it decides two more things:

- **The thermal margin, through Kt** (ADR-0136). The learned walk's ankle
  needs 0.60–0.72 N·m RMS (ADR-0140).
  - At the surveyed Kt 0.44 that is ~1.51 A, inside the 1.60 A rating.
  - At the vendor's 0.35 it is ~1.89 A, 18 % over.
  - A rewind does not change this: heat is `I²R` at a given torque.
- **Whether the stock part is enough, through Kv.**
  - A learned trot drives its rotors under ~380 rpm (ADR-0135).
  - The stock part gives 15.83 rpm/V at the output: 443 rpm at an 8S
    pack's loaded floor.

**Measure:**

1. **Mass**, with and without the integrated driver. Assumed: 131.7–132 g.
2. **Phase-to-phase resistance** `R_pp`, 4-wire, at room temperature.
   Assumed: 4.466 Ω.
3. **Torque constant Kt, two independent ways:**
   - **Static:** lock the output with a lever arm onto a scale or load cell.
     Command phase current in steps (0.5, 1.0, 1.6, 2.5, 4.0 A) and record
     the output torque. Kt is the slope. Note the driver's convention for
     "current": peak or RMS phase, or q-axis.
   - **Back-EMF:** spin the output with a drill at a known speed and measure
     the line-to-line voltage. Kt follows from Ke. If the two disagree, that
     is ADR-0044's six-step-versus-sinusoidal question, and the driver's
     current sense decides it.
4. **Kv:** no-load output speed at 24 V and at 28 V (8S at 3.5 V/cell).
   Assumed: 15.83 rpm/V, 380 rpm at 24 V.
5. **Peak:** the output torque at the driver's peak current. Assumed 1.95
   N·m; 4.19 A × Kt is 1.47–1.84.
6. **Heating:**
   - Hold 0.66 N·m (the ankle's RMS) at stall and log winding temperature
     (a thermocouple on the stator end-turns) until it is flat or reaches
     80 °C.
   - Repeat in still air and in a fan's draught.
   - This tests NFR18 / ADR-0023 directly and calibrates `thermal/`'s `h`.

**When it lands:**

- `kinematics/src/tomcat_kin/power.py`: `KT`, `R_PHASE_PHASE`.
- `tools/motor_spec_review.py`: `SPEC`.
- `kinematics/src/tomcat_kin/mjcf_tendon.py`: `MOTOR_PEAK_NM` (the
  current-limited peak) and `MOTOR_NOLOAD_RAD_S` (Kv × pack floor).
- `TomcatEnv.MOTOR_RANGES`: narrow to the measured part ± tolerance.
- `thermal/src/lib.rs`: its constants (`test_thermal_constants` keeps them
  in step).
- Mass: `params.py`'s trunk mass carries the 19 motors.
- Then re-run `tools/walk_loads.py` on the current policy at the measured
  motor.

**Decision rule:**

| measured Kt | ankle RMS current | verdict |
|---|---|---|
| ≥ 0.44 | ≤ 1.6 A | stock part passes; buy 19 |
| 0.38–0.44 | 1.6–1.9 A | marginal; needs forced air (h ≥ 25) and a measured heating curve |
| ≤ 0.38 | ≥ 1.9 A | over rating; a stronger frame, or larger moment arms at the ankle |

---

## B2. Conduit friction μ

**Why.**

- `params.conduit_mu = 0.07` is `[assumed]` (UHMWPE in a PTFE-lined
  conduit). Every conduit loss in `power`, and since M146 in the plant
  itself, is `exp(μ θ)`.
- On the analytic trot the vendor-Kt runtime held NFR6's 14 min only up to
  μ ≈ 0.076 (ADR-0128). On the learned walk it is 16.9 min at 0.07
  (ADR-0140).
- The ankle conduit bends up to ~310°, so a small error in μ moves the
  ankle's peak torque by tens of percent.

**Rig:**

- Route the actual conduit with the actual cable around printed formers at
  a radius ≥ 15 mm (`CONDUIT_RMIN`). Use total bends of 0°, 90°, 180°, 270°
  and 360°.
- Hang a known mass on the output (5, 10, 20 N; the walk's cables run
  20–60 N RMS).
- Pull the input through a load cell at a slow constant speed (~10 mm/s),
  in both directions, and log the tension.

**Reduce:**

- `μ = ln(T_in / T_out) / θ` for each bend; fit across bends.
- The 0° case gives the ferrules' and ends' loss, `pulley_efficiency`'s
  analogue.
- Repeat after ~10⁴ cycles: liners wear and μ drifts.

**When it lands:**

- `params.py`: `conduit_mu`, `pulley_efficiency`.
- `TendonParams.crossing_bend` is unchanged (geometry), but its friction
  moves.
- `power` runtime, `mass_closure` motor spec and `thermal/` move with it.
- The plant picks it up through `drive_capstan` with no further change.
  Consider randomising μ (± the measured spread) as a new range in
  `TomcatEnv`.

**Decision rule:**

- μ ≤ 0.07 leaves NFR6's margin as computed.
- μ 0.07–0.09 needs a re-run of `walk_loads` and possibly a lower-friction
  liner.
- μ > 0.09: re-route the ankle conduit (its knee crossing is the largest
  bend).

---

## B3. Paw-pad friction μ (OPEN_RISKS R2)

**Why.**

- NFR15 (disturbance rejection) holds from μ ≥ 0.6 (ADR-0034).
- Training draws `floor_mu` from 0.5–1.1 (ADR-0084).
- M139–M150's walks still slip 0.03–0.08 m/s on loaded feet in simulation
  (ADR-0133). How much of that a real pad would allow is exactly this
  number.

**Rig:**

- A pad sample (the TPU formulation chosen for the paw) under the walk's
  per-foot load: ~12 N standing (47 N / 4), up to ~25 N in the trot's
  stance.
- Drag it with a force gauge on the floors the robot will see: tile, sealed
  wood, low-pile carpet.
- Record static and kinetic μ, dry, and dusty if relevant.

**When it lands:**

- `TomcatEnv.RANGES["floor_mu"]` becomes the measured range across floors.
- `env._friction_geoms` and the pad `friction` in `mjcf_tendon.py` (0.8).
- If kinetic μ < 0.6 on a target floor, NFR15 needs revisiting, and the
  slip penalty (ADR-0133) matters more.

---

## B4. G3 flexure rate

**Why.**

- The legs' series spring is specified at 125 kN/m and the spine's at
  350 kN/m (ADR-0107, ADR-0117).
- Training randomises the rate ×0.7–1.4.
- The rate sets the drivetrain's lowest mode, and with it how stiff a
  controller can be (ADR-0060). A titanium flexure's real rate depends on
  the cut.

**Rig:**

- One leg flexure in a vise.
- Load it in steps to its working range (~100 N at the cable) and read the
  deflection with a dial indicator. Load and unload to see hysteresis.

**When it lands:**

- `params.py`: `series_k` (legs) and `SpineParams.series_k`.
- `TomcatEnv.RANGES["series_k_scale"]`, narrowed to the measured spread.
- `test_mjcf_tendon`'s drivetrain-mode tests will move with it.

---

## B5. Battery cell (OPEN_RISKS R3)

**Why.**

- Runtime assumes 175 Wh/kg and 80 % usable in a 300 g pack.
- With an 8S bus (ADR-0110/0136) the cell count is fixed. The candidate
  cells' Wh/kg at the trot's draw is the open number:
  - ~100–130 W total, ~4–5 A at 28 V;
  - its peaks are higher.

**Measure:** discharge one candidate pack, or a single cell scaled, at the
walk's power profile (`tools/walk_loads.py` gives it) to the 3.5 V/cell
floor. Record the delivered Wh and the voltage sag at peak.

**When it lands:**

- `power.py`: `BATTERY_KG`, `BATTERY_WH_PER_KG`, `BATTERY_USABLE`.
- `thermal/src/lib.rs`'s runtime constants follow.
- The 3.5 V/cell floor also sets `MOTOR_NOLOAD_RAD_S` (Kv × floor).

---

## Acceptance criteria (M153, ADR-0145)

Swept in simulation against the current walking policy, one number at a
time:

| number | accept | why |
|---|---|---|
| conduit μ (B2) | ≤ 0.09 | at 0.11 runtime, peak torque and heating fail together |
| pad μ (B3) | ≥ 0.6 for NFR15 | the walk itself holds from 0.3 |
| G3 rate (B4) | within ±30 % of 125 kN/m | softer AND stiffer both fall outside it |
| latency, sensor → motor | < 10 ms | 12 ms already falls 3 in 10; 16 ms always (firmware, not a bench part) |
| mass | up to 1.3× holds the walk | runtime and torque still pay for it |

## Shopping list (first round)

| item | for | note |
|---|---|---|
| 1× SteadyWin GIM3505-9 with its driver | B1 | the cable/connector kit the driver needs |
| bench PSU, 0–36 V, ≥ 5 A, current readout | B1, B5 | |
| load cell (5–20 kg) + HX711, or a calibrated scale | B1, B2, B4 | one serves all three |
| 4-wire milliohm meter (or a 4-wire lead set) | B1 | for `R_pp` |
| K-type thermocouple + reader | B1 | stator end-turn |
| optical tachometer, or the driver's encoder | B1 | Kv |
| conduit + cable sample (the specified PTFE-lined conduit, UHMWPE cable) | B2 | ~1 m each |
| printed bend formers (r ≥ 15 mm; 90–360°) and slotted weights (5–20 N) | B2 | |
| paw-pad sample (TPU) and floor samples | B3 | |
| force gauge (0–50 N) | B3 | |
| one machined G3 leg flexure, dial indicator | B4 | |
| candidate battery pack (8S) | B5 | or one cell |

---

## After the numbers come in

Log each measurement as a CSV and reduce it with `tools/bench_reduce.py` (M152).
`python tools/bench_reduce.py template <kind>` prints the columns each one
wants: `kt` (+ `--backemf`), `kv`, `heat`, `capstan`, `flexure`, `pad`. Each
command prints:

- the fitted values;
- the constants they replace;
- this plan's verdict for them.

The Kt reducer gives both the stall slope and the back-EMF figure, per peak
and per RMS phase amp. Which one the driver's "current" means is the question
ADR-0044 left open. `tests/test_bench_reduce.py` checks every reducer against
synthetic data with a known answer.

1. Update the constants listed under each item, and let the suite say what
   moved. The tests pin today's assumed values, so each changed number
   fails exactly the assertions that rest on it.
2. Re-run `tools/walk_loads.py` on the current policy at the measured motor
   and friction. That is the runtime and thermal verdict on the real parts.
3. Narrow the training ranges to the measured spread and retrain. A policy
   trained on a measured range is what goes to hardware.
