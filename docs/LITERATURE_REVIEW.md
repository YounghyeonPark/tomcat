# T.O.M.C.A.T. — Literature Review

A deep, cited synthesis of prior art for a tendon-driven, flexible-bodied robot
cat, organized to answer the project's open design questions and feed the
[ADR log](DESIGN_DECISIONS.md). Supersedes the reading-list-level
[REFERENCES.md](REFERENCES.md) (which remains the raw source index).

## Method & provenance

Produced with the `deep-research` harness: the question was decomposed into 6
search angles; 26 sources were fetched and read in full; 90 falsifiable claims
were extracted; the top 25 went through **3-vote adversarial verification**
(needing 2/3 to refute a claim to kill it). All 25 survived 3-0. Additional
primary-source extractions were recovered from the run journal for the three
questions the automated synthesis dropped on budget (righting reflex, actuator
trade-off, RoboCat).

**Confidence legend**
- ✅ **Verified** — passed 3-0 adversarial verification.
- ◐ **Primary, unverified** — extracted from a primary source (with a direct
  quote) but *not* put through the adversarial vote; treat as reliable-but-single-reader.
- ⚠️ **Caveat** — paywall, preprint, single-design, or unit-conversion risk.

---

## Q1 — Tendon / antagonistic actuation: torque **and** stiffness ✅

Antagonistic cable pairs set net joint torque/position **and** joint stiffness
*independently* via differential tension. Kengoro (Kawaharazuka et al.,
[arXiv:2409.00705](https://arxiv.org/pdf/2409.00705)) uses a muscle-stiffness law

```
T_target = T_bias + max{0, K_stiffness · (l − l_target)}
```

with **Antagonist Inhibition Control (AIC)**: the agonist gets gain `K = k`, the
antagonist `K = 0` and holds at `T_bias`, so the pair produces motion without
fighting itself. With experimental parameters `T_bias = 2 kgf, k = 10, C = 0`,
AIC measurably cut peak tendon tension (shoulder abduction 43→28 kgf; whole-arm
raise 55→45 kgf; scapula 44→37 kgf). A series-elastic antagonistic cable joint
with variable-stiffness mechanisms independently confirms angle and stiffness
can be controlled separately ([Mech. & Machine Theory 2021](https://www.sciencedirect.com/science/article/abs/pii/S0094114X21004432), ⚠️ paywalled).

> **Implication for our `tendon.py`:** our current model sets the slack side to a
> fixed `pretension` and puts all torque on the active side. The literature says
> we should instead expose **co-contraction bias** (`T_bias`) as a first-class
> control input so stiffness is commandable — and adopt an AIC-style rule to keep
> peak tensions (hence motor/cable sizing) down.

## Q1b — Tension sensing & proprioception ✅

Per-tendon proprioceptive sensing is mature and can be made compact:

- **Kengoro:** 116 sensor-driver-integrated muscle modules, each with a load-cell
  tension unit (~55 kgf limit), temperature sensor, current-controlled BLDC, and
  29:1 gearhead ([arXiv:2409.00705](https://arxiv.org/pdf/2409.00705)).
- **Compact module:** a newer design senses tension with a **single-pulley +
  maze-slot fixation** (to cut friction) and estimates tendon length via a
  **3D Hall-effect sensor** instead of a bulky encoder
  ([MDPI Actuators 14(6):278, 2025](https://www.mdpi.com/2076-0825/14/6/278), ⚠️ author-claimed, not independently benchmarked).
- **Control:** a joint-space controller driven by redundant muscle **tension**
  (not length) improves joint-torque estimation and, with integrated joint-angle
  estimation, works for **spherical joints and spine structures whose angle can't
  be measured directly** ([Humanoids 2016, DOI 10.1109/HUMANOIDS.2016.7803367](https://dl.acm.org/doi/10.1109/HUMANOIDS.2016.7803367), ⚠️ paywalled).

> **Answers ADR-0004:** in-line load cells are the proven high-fidelity route;
> current-based estimation is the cheap route; a compact Hall-effect + single-pulley
> module is the modern space-saving compromise. The tension-based joint-space
> controller is directly relevant to our spine, whose joint angles are hard to
> instrument.

## Q2 — Spine topology: serial vs. tensegrity ✅

Two validated architecture families exist:

| | Tensegrity backbone | Serial compliant chain |
|---|---|---|
| Exemplar | **Laika** ([arXiv:1804.06527](https://arxiv.org/abs/1804.06527), UC Berkeley 2018) | **SPARC** ([arXiv:2510.01984](https://arxiv.org/pdf/2510.01984), 2025, ⚠️ preprint) |
| DOF | ~2 joints × 3 DOF = 6 DOF (roll/pitch/yaw) ([MDPI Robotics 15(6):103](https://doi.org/10.3390/robotics15060103)) | 3-DoF sagittal (revolute bend + prismatic axial) |
| Actuation | Cable-length change bends; central rotating vertebra twists | Serial joints; combined revolute + prismatic compliance |
| Compliance | Active **and** passive simultaneously | Compliant, 1.26 kg compact package |
| Maturity | First working tensegrity-spine quadruped prototype (legs not yet attached; foot-lift demonstrated as gait precursor) | Compact module, preprint |

⚠️ **No source establishes tensegrity is quantitatively *superior* to a serial
chain** — they are alternatives with different trade-offs. Tensegrity is favored
for weight/force distribution and inherent compliance; serial chains are simpler
to model and control (and match our existing planar-3R kinematics approach).

> **For ADR-0006:** a **serial tendon-driven chain** is the lower-risk starting
> point — it reuses our kinematic framework and is easier to control — while
> tensegrity is the higher-ceiling research option. Concrete seed: **2–3 spine
> joints, ~3 DOF each** (pitch/yaw ± roll) matches both Laika (6 DOF) and the
> bio-inspired tensegrity spine.

## Q2b — Why a flexible spine helps, and spine↔gait coupling ✅

Efficiency benefit comes from **two mechanisms**: (1) augmenting stride length,
and (2) acting as a **mechanical low-pass filter** that attenuates high-frequency
torque fluctuations (i.e. passive impact filtering). SPARC quantifies a **21%
power reduction at 0.9 m/s** vs a rigid-spine baseline *for an optimally tuned
stiffness* ([arXiv:2510.01984](https://arxiv.org/pdf/2510.01984)). A planar
quadruped study confirms both spine motion and spinal flexibility raise running
efficiency, and that **for each speed there is an optimal spinal stiffness**
(rising with speed and mass) ([J. Bionic Eng. 2017](https://link.springer.com/article/10.1016/S1672-6529(16)60436-5)).

⚠️ Benefits are **speed-dependent** — near-zero at low speed, concentrated at
higher speed — and the 21% figure is single-design and from a preprint.

Reduced-order models show *how* to couple spine and gait:
- Leg **inertia matters**: a Mass-Mass-Spring leg model (0.454 kg at the knee)
  produces realistic trunk bending where a massless SLIP model gives a null
  bending moment; leg mass during flight is what bends the compliant trunk to
  store/release elastic energy ([Sci. Reports 2022, PMC9418320](https://pmc.ncbi.nlm.nih.gov/articles/PMC9418320/)).
- Common models: two rigid bodies (fore/hindquarters) + a torsion spring
  (e.g. **2000 N/m, 0.7 m**), or a **9-DOF planar** system (1 spine joint + four
  2-DOF legs). In a gallop the spine **extends via spring release at take-off but
  locks at a fixed angle during flight**, decoupling spine from legs mid-air
  ([BQR3, PMC10968056, 2024](https://pmc.ncbi.nlm.nih.gov/articles/PMC10968056/)).

> **Design consequence:** spine stiffness should be a **tunable parameter matched
> to gait speed**, not fixed — which is a direct argument *for* antagonistic
> co-contraction (or a variable-stiffness element) on the spine, tying Q2 back to
> Q1.

## Q3 — Feline biomechanics as design targets ✅

- **Whole-spine compressive stiffness: 53.62 ± 4.68 N/mm** (5 cat specimens, FEA +
  experiment) ([J. Bionic Eng. 2024, Lu et al.](https://link.springer.com/article/10.1007/s42235-024-00594-4)).
- **Directional compliance rank:** most compliant in **axial rotation**, then
  **extension**, then **lateral bending** (stiffest). Use this rank to set
  *per-axis* spine stiffness/limits.
- **Landing:** cats actively modulate back bending to store kinetic energy briefly
  as **elastic strain energy in the flexible back**, reducing limb dissipation
  ([J. Bionic Eng. 2015, Zhang et al.](https://link.springer.com/article/10.1016/S1672-6529(14)60063-9)) — the biological basis for our passive-shock-absorption goal (G3).

> ⚠️ **Unit caveat:** 53.62 N/mm is whole-spine **axial compressive** stiffness
> (force/length), **not** the per-joint **rotational** stiffness (N·m/rad) an
> articulated tendon spine needs. Use it for axial cushioning, and use the
> *directional rank* (not the absolute value) to seed per-axis rotational
> compliance. A geometry-based conversion is required before parameterizing.

## Q4 — Cat righting reflex / mid-air reorientation ◐

*(Primary sources, recovered from journal; not adversarially verified.)*

- **Rotary actuators suffice:** a free-falling articulated robot with **rotary
  actuators only** (no reaction wheels/thrusters) can perform a **180° longitudinal
  reorientation** via closed paths in joint space; net rotation scales with joint
  sweep amplitude; two fixed-axis manoeuvres can be alternated to reach *any* 3-D
  orientation. Validated on hardware with VICON
  ([Garant & Gosselin, IEEE/ASME TMech 2020](https://ieeexplore.ieee.org/document/9246721/)).
- **Legs alone can do it:** two maneuvers on a **9-DOF quadruped** produce roll
  *and* pitch reorientation in free fall via leg shape-change — no tail
  (simulation only) ([DOI 10.1115/1.4053897, 2022](https://doi.org/10.1115/1.4053897)).
- **But a tail is the highest-performance option:** the "Inertial Reorientation
  template" analysis concludes **dedicated tails outperform limbs and
  spine/body-bending** for aerial reorientation (validated by retrofitting a tail
  to RHex) ([Libby et al., IEEE T-RO 2016 / arXiv:1511.05958](https://arxiv.org/abs/1511.05958)).
- A **3-DoF morphable (telescoping) tail** self-rights a Unitree A1 in the flight
  phase, extending for authority then retracting to ~1/4 length before touchdown
  ([arXiv:2209.15337, 2022](https://arxiv.org/pdf/2209.15337)).

> **Tension worth flagging:** spine twist *can* achieve righting, but the
> reorientation literature says an **inertial tail is more effective**. If
> righting is a real goal, budget for a tail (which is also very cat-like) rather
> than relying on spine twist alone.

## Q5 — Actuator architecture: tendon-drive vs. proprioceptive direct-drive ◐

*(Primary source, recovered from journal; not adversarially verified.)*

The **MIT Cheetah** proprioceptive actuator (low-gear-ratio, backdrivable
direct-drive) delivers **high torque density + high-bandwidth force control +
impact mitigation via backdrivability** simultaneously — the very properties we
sought from tendon-drive + series elasticity. Crucially, its **Impact Mitigation
Factor (IMF)** — a transferable metric the paper defines — is **comparable to
quadrupeds that use dedicated series springs**. It controls contact forces in
bounding with **85 ms contact times and >450 N peak forces**
([Wensing et al., IEEE T-RO 2017](https://www.researchgate.net/publication/312558722_Proprioceptive_Actuator_Design_in_the_MIT_Cheetah_Impact_Mitigation_and_High-Bandwidth_Physical_Interaction_for_Dynamic_Legged_Robots)).

> **This challenges a blanket "tendon-drive everywhere" stance (P1/ADR-0003).**
> Backdrivable direct-drive can match series-spring impact handling *at the leg*
> without cables. The strongest honest position: tendon-drive earns its keep
> where it uniquely wins — **centralizing mass off the limbs and driving the
> many-DOF spine from the girdles** — while the *legs* deserve a real trade study
> (tendon vs. compact backdrivable direct-drive) using IMF as the yardstick.

## Q6 — RoboCat: the closest analogue, and how we differ ◐

*(Primary source, ASME IMECE 2011; not adversarially verified.)*

RoboCat (Ohio University; Carpenter, Yu, Altun, Graham, Zhu, Starzyk) is a
cat-sized, biomimetic **elastic cable-driven quadruped**
([ASME IMECE2011-63805](https://www.researchgate.net/publication/267593853_A_Biomimetic_Elastic_Cable_Driven_Quadruped_Robot_The_RoboCat)):

- **It has a RIGID trunk.** *"The trunk segment does not contain internal
  flexibility, while a biological cat has significant flexibility in the trunk."*
  It recovers turning only via extra per-leg revolute joints. **This is exactly
  the gap T.O.M.C.A.T.'s articulated curving spine targets — our core
  differentiator.**
- **Clever actuation:** elastic (stretchable) cables + a **variable-radius pulley**
  let a **single motor drive an antagonistic 2-cable joint**, so an n-DOF joint
  needs only **n** motors instead of **n+1**. Worth adopting.
- Elastic cables absorb ground-contact shock and store/return energy across the
  gait cycle (supports G3/G4).
- **Reality check:** the physical RoboCat used **hybrid** actuation (direct-drive
  hips/shoulders + elastic-cable knees/elbows) and ran **open-loop R/C-servo
  position control** with limited, oscillatory performance — closed-loop control
  was simulation-only. Pretension was **50 N**; simulated antagonistic tensions
  ~**20–70 N**; the model assumed **frictionless** guides and used **no tension
  sensing**.

> **How T.O.M.C.A.T. differentiates:** (1) an **articulated, actively curving
> spine** (RoboCat's explicit omission); (2) **closed-loop tension control with
> real sensing** (RoboCat assumed frictionless, sensed nothing); (3) **whole-body
> coordination** of spine + legs. Adopt RoboCat's elastic-cable + variable-radius-
> pulley motor economy.

---

## Recommendations for the open decisions

| Decision | Recommendation (evidence) | Confidence |
|----------|---------------------------|------------|
| **ADR-0002** antagonistic vs. spring | Antagonistic with **commandable co-contraction bias** + AIC-style control on joints where stiffness must vary (legs, spine); it's proven (Kengoro) and the efficiency data demand *tunable* stiffness. Reserve spring-return for low-DOF distal joints to save motors. | ✅ High |
| **ADR-0003** actuator tech | Tendon-drive for the **spine and to centralize limb mass**; run an explicit **tendon vs. backdrivable direct-drive trade study for the legs** using the **IMF** metric — do not assume tendon wins there. | ◐ Medium |
| **ADR-0004** tension sensing | **In-line load cells** for fidelity (Kengoro precedent); evaluate the **compact single-pulley + 3D-Hall** module to save space; use a **tension-based joint-space controller** for the spine (angles hard to sense). | ✅ High |
| **ADR-0006** spine topology | Start with a **serial tendon-driven chain** (reuses our kinematics; easier control); keep **tensegrity** as a research alternative. | ✅/⚠️ Med-High |
| **Spine DOF / segments** | **2–3 segments, ~3 DOF each** (pitch/yaw, + roll for the righting twist), per Laika (6 DOF) and the tensegrity-spine prototype. | ◐ Medium |
| **Righting (if in scope)** | Spine twist alone is feasible (rotary-only 180° proven), but an **inertial (ideally morphable) tail** is the higher-performance, still-cat-like choice. | ◐ Medium |

## Seed parameters for the model (all to be refined)

| Parameter | Seed value | Source | Note |
|-----------|-----------|--------|------|
| Spine segments × DOF | 2–3 × 3 | Laika / MDPI tensegrity | ADR-0006 |
| Spine per-axis stiffness rank | axial-rot < extension < lateral (compliant→stiff) | Cat FEA 2024 | Use rank, not absolute |
| Whole-spine axial stiffness | 53.62 ± 4.68 N/mm | Cat FEA 2024 | ⚠️ needs N·m/rad conversion |
| Backbone torsion spring (model) | ~2000 N/m over 0.7 m | Sci. Reports 2022 | Reduced-order modeling |
| Leg mass (knee) for dynamics | ~0.454 kg | Sci. Reports 2022 | Don't model legs as massless |
| Cable pretension | ~50 N (can be lower) | RoboCat 2011 | vs our current placeholder 5 N |
| Antagonistic tension range | ~20–70 N | RoboCat 2011 | sanity band |
| Efficiency gain (tuned spine) | ~21% @ 0.9 m/s | SPARC 2025 | ⚠️ single-design, speed-dependent |

## Gaps & follow-ups

1. **Unit conversion:** turn the cat's whole-spine axial 53.62 N/mm + directional
   rank into per-joint **rotational** stiffness (N·m/rad) and joint-angle limits
   per axis (dorsoventral / lateral / axial).
2. **Leg actuator trade study:** quantitative tendon-vs-direct-drive comparison at
   the leg using IMF, torque density, reflected inertia, and control complexity.
3. **RoboCat mechanics:** its actuation/spine numbers come from a design paper;
   the vision paper adds nothing mechanical — find companion RoboCat papers for
   more tension/stiffness detail if deeper benchmarking is needed.
4. Several key sources (ScienceDirect antagonistic joint; Springer cat-spine;
   ACM Humanoids 2016) are **paywalled** — abstract-level facts verified, full
   numeric detail (VSM torque-deflection curves, per-axis stiffness) not read.

## Verification summary

25/25 verified claims survived 3-0 adversarial voting; **0 refuted, 0
unverified**. Q4–Q6 rest on primary-source single-reader extractions (marked ◐),
not the adversarial vote — treat their specific numbers as reliable-but-unaudited.
