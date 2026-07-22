# T.O.M.C.A.T. — Related Work & References

> **See [LITERATURE_REVIEW.md](LITERATURE_REVIEW.md)** for the deep, fully-read
> and adversarially-verified synthesis with actionable recommendations. This file
> is the raw source index behind it.

Curated prior art for a tendon-driven, flexible-bodied robot cat. Grouped by
topic and cross-referenced to our [design principles](PRINCIPLES.md) and
[ADRs](DESIGN_DECISIONS.md). Each entry notes *why it matters here*.

> Sources gathered via web search (July 2026). Links are to the publisher or a
> public copy; some require institutional access. Not yet read in full — this is
> a reading list / prior-art map, not a validated literature review.

---

## 1. Closest prior art — cable-driven robot cats
Relevant to: **P1**, **P2**, whole project positioning.

- **RoboCat: A Biomimetic Elastic Cable-Driven Quadruped Robot** — a cat-sized
  quadruped using *stretchable elastic cable-driven joints* inspired by
  biological quadrupeds; motors mounted near the torso to cut leg inertia. The
  single closest analogue to this project.
  https://www.researchgate.net/publication/267593853_A_Biomimetic_Elastic_Cable_Driven_Quadruped_Robot_The_RoboCat

- **Bionic Multi-Legged Robots with Flexible Bodies: Design, Motion, and
  Control** (PMC, review) — survey tying flexible-body design to motion and
  control; good map of the field.
  https://pmc.ncbi.nlm.nih.gov/articles/PMC11506302/

## 2. Tendon / cable-driven & antagonistic actuation
Relevant to: **P1**, **ADR-0002** (antagonistic vs. spring), **ADR-0004** (tension sensing).

- **Antagonistic Cable Actuation in Robotics** (overview) — differential
  tensioning of an agonist/antagonist cable pair sets both net torque *and*
  stiffness; the exact scheme our `tendon.py` models.
  https://www.emergentmind.com/topics/antagonistic-cable-actuation

- **A novel cable-driven antagonistic joint with variable stiffness
  mechanisms** (Mech. & Machine Theory) — concrete variable-stiffness antagonist
  joint design.
  https://www.sciencedirect.com/science/article/abs/pii/S0094114X21004432

- **Tensegrity-based Robot Leg Design with Variable Stiffness** (arXiv 2504.19685)
  — cable-tensioned leg with tunable compliance.
  https://arxiv.org/abs/2504.19685

## 3. Flexible / articulated spine for quadrupeds
Relevant to: **P2**, **ADR-0006** (articulated spine), spine DOF (NFR2).

- **Development of a flexible coupled spine mechanism for a small quadruped
  robot** (IEEE) — couples spine flexion/extension with leg motion for faster
  run / longer jump; directly informs FR10 (whole-body coordination).
  https://ieeexplore.ieee.org/document/7866300/

- **Effect of Flexible Spine Motion on Energy Efficiency in Quadruped Running**
  (J. Bionic Eng.) — quantifies the efficiency case for a moving spine (our G4).
  https://link.springer.com/article/10.1016/S1672-6529(16)60436-5

- **Dynamic Modeling & Flying-Gait Characteristics of Quadruped Robots with
  Flexible Spines** (PMC) — flight-phase dynamics for a flexible-spine quadruped.
  https://www.ncbi.nlm.nih.gov/pmc/articles/PMC10968056/

- **Lateral flexion of a compliant spine improves motor performance in a
  bioinspired mouse robot** (Science Robotics) — small-robot evidence that
  lateral spine bend boosts speed and agility.
  https://www.science.org/doi/10.1126/scirobotics.adg7165

- **A Robust Quadruped Robot with Twisting Waist for Flexible Motions**
  (arXiv 2410.05884) — an actuated waist/spine DOF on a real quadruped.
  https://arxiv.org/abs/2410.05884

- **A Bio-Inspired Tensegrity Spine with Adjustable Stiffness for Quadruped
  Robots** (Robotics, MDPI) — tensegrity is an alternative spine topology to a
  serial joint chain; worth weighing against ADR-0006.
  https://doi.org/10.3390/robotics15060103

- **The Ultra Spine: A Tensegrity Robot for Flexible Quadruped Backbones**
  (UC Berkeley, MEng project) — tensegrity backbone case study.
  https://best.berkeley.edu/wp-content/uploads/2015/09/MENGPROJECT-37-AGOGINO-THE-ULTRA-SPINE_A-TENSEGRITY-ROBOT-FOR-FLEXIBLE-QUADRUPED-BACKBONES.pdf

## 4. Feline biomechanics (the biological target)
Relevant to: **P2**, shock absorption (G3), landing.

- **Comprehensive Biomechanical Characterization of the Flexible Cat Spine**
  (J. Bionic Eng., 2024) — FEA + experiment on cat-spine bending/twisting and
  impact mitigation; a source of realistic geometry and range-of-motion targets.
  https://link.springer.com/article/10.1007/s42235-024-00594-4

- **Effect of Flexible Back on Energy Absorption during Landing in Cats**
  (ScienceDirect) — how the spine absorbs landing energy; informs the "land"
  load case in our torque budget.
  https://www.sciencedirect.com/science/article/abs/pii/S1672652914600639

## 5. Cat righting reflex / mid-air reorientation
Relevant to: **P2** (axial twist), future righting behaviour.

- **Cat righting reflex** (Wikipedia) & **Falling cat problem** (Wikipedia) —
  the shape-change mechanism: a non-rigid body reorients at zero net angular
  momentum by twisting a flexible spine.
  https://en.wikipedia.org/wiki/Cat_righting_reflex ·
  https://en.wikipedia.org/wiki/Falling_cat_problem

- **Design & Experimental Validation of Reorientation Manoeuvres for a Free
  Falling Robot Inspired From the Cat Righting Reflex** — shows rotary-actuator
  robots can achieve a 180° longitudinal reorientation.
  https://www.researchgate.net/publication/345629414

- **Towards Safe Landing of Falling Quadruped Robots Using a 3-DoF Morphable
  Inertial Tail** (arXiv 2209.15337) — tail-based reorientation; complements
  spine twist.
  https://arxiv.org/pdf/2209.15337

## 6. Musculoskeletal tendon-driven robots (control state of the art)
Relevant to: **P1**, mid-level tendon coordination, **ADR-0004**.

- **Kenshiro / Kengoro — human-mimetic musculoskeletal humanoids** (JSK, U-Tokyo)
  — full-body tendon-driven robots (Kenshiro ≈160 "muscles"); motor winds a
  spool to set tendon length, with tension control — the actuation pattern our
  `TendonMap.motor_angles` assumes.
  https://ieeexplore.ieee.org/abstract/document/6913891

- **Characteristics, Management, and Utilization of Muscles in Musculoskeletal
  Humanoids (Kengoro, Musashi)** (arXiv) — practical lessons on managing many
  coupled tendons; directly relevant to our coupled-cable control problem.
  https://arxiv.org/pdf/2602.08518

- **The Design and Control of a Proprioceptive Modular Actuator for
  Tendon-Driven Robots** (Actuators, MDPI) — a modular tendon actuator with
  built-in sensing; a candidate building block (ADR-0003/0004).
  https://www.mdpi.com/2076-0825/14/6/278

## 7. Actuator design & compliance for dynamic legged robots
Relevant to: **ADR-0003** (actuator tech), **ADR-0004** (sensing), G3 (compliance).

- **Proprioceptive Actuator Design in the MIT Cheetah** (IEEE T-RO) — the
  *alternative* to tendon drive: backdrivable high-torque motors with
  proprioceptive (current-based) force control and an "impact mitigation factor."
  Important baseline to argue tendon-drive against.
  https://ieeexplore.ieee.org/document/7827048/

- **MIT Cheetah 3: Design and Control of a Robust, Dynamic Quadruped**
  (IROS / MIT DSpace) — full quadruped design + control reference.
  https://dspace.mit.edu/bitstream/handle/1721.1/126619/IROS.pdf

- **Series/Variable Stiffness Actuators** — background for the spring-return and
  co-contraction stiffness ideas in ADR-0002:
  - Stiffness Control of Variable SEAs (Actuators, MDPI):
    https://doi.org/10.3390/act6040028
  - Configurable Compliance for SEAs (RoMeLa):
    https://www.romela.org/wp-content/uploads/2015/05/2013_configurable_compliance_for_series_elastic_actuators.pdf

## 8. Existing open-source robot cats (community reference)
Relevant to: the project's "community-driven" positioning. NOTE: these are
**servo-per-joint** designs (direct drive), *not* tendon-driven — useful for
gait/IK code, tooling, and community model, but they are explicitly the
architecture T.O.M.C.A.T. departs from.

- **Petoi OpenCat / Nybble (robot cat) & Bittle (robot dog)** — open-source
  quadruped framework (Arduino/RPi), gaits + inverse kinematics, large community.
  https://github.com/PetoiCamp/OpenCat ·
  https://www.petoi.com/pages/opencat-open-source-robot-pet-framework

---

## How this maps to our open decisions

| Our decision | Sources to consult |
|--------------|--------------------|
| ADR-0002 antagonistic vs. spring | §2 (antagonistic joints), §7 (SEA/variable stiffness) |
| ADR-0003 actuator technology     | §6 (tendon modular actuator), §7 (MIT Cheetah proprioceptive) |
| ADR-0004 tension sensing         | §6 (Kenshiro tension-controlled muscle, modular actuator) |
| ADR-0006 articulated spine       | §3 (serial vs. tensegrity spine), §4 (cat-spine ROM/geometry) |
| Spine DOF / segment count (NFR2) | §4 (cat biomechanics), §3 (twisting-waist, coupled spine) |
| Righting/landing behaviour       | §5 (righting reflex, inertial tail), §4 (landing energy) |
