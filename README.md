# Project T.O.M.C.A.T.

**Tendon-Operated Mechanism for a Compliant Actuated Tomcat** — an open,
cat-sized quadruped whose joints are pulled by cables from motors in the body,
not driven by a motor at each joint.

[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

![the cat in its skin](docs/figures/cat_suit_m130.png)

*In its skin: a charcoal-grey plush knit on a soft foam shell, and a cat's
head made of foam round a skull that has no jaw, its face held in shape by a
firmer foam ([ADR-0119](docs/DESIGN_DECISIONS.md)–[0122](docs/DESIGN_DECISIONS.md)).
The skin's 244 g is in the simulated plant ([ADR-0124](docs/DESIGN_DECISIONS.md)).
Under it, the robot as designed:*

![the whole robot](docs/figures/whole_robot_m130.png)

## How a leg is driven

![a hind leg's drive, in 3-D](docs/figures/leg_drive_m132.webp)

Each joint is an antagonistic cable pair wound on one spool, so one motor
drives one joint both ways (red / blue / green = hip / knee / ankle; thick =
the cable pulling).

- **Motors in the body.** Each girdle holds its leg motors in two rows that
  straddle the hip, spools facing across it ([ADR-0114](docs/DESIGN_DECISIONS.md)).
- **Knee and ankle cables run along the hip axis**, through a hollow hip: the
  trunk carries a stub, the femur turns on it in two bearings, and the cables
  never cross the femur's path.
- **Bowden conduits** carry each cable from the body to the leg; friction is
  charged where the cable slides ([ADR-0112](docs/DESIGN_DECISIONS.md)). The
  ankle's conduits run on across the knee to a ferrule on the shin, so no
  pulley couples the knee and ankle ([ADR-0126](docs/DESIGN_DECISIONS.md)).
- **Each spool is as wide as its joint's cable travel** (hip 12 mm, knee and
  ankle 10, spine 8), and the conduit's first ferrule floats in a slot along
  the drum so the cable leaves every groove square
  ([ADR-0123](docs/DESIGN_DECISIONS.md)). A pair's two cables pass every
  pulley on opposite sides, so their total length never changes
  ([ADR-0118](docs/DESIGN_DECISIONS.md)).
- **A series spring (G3)** sits between each motor and its spool: a titanium
  planar flexure, 125 kN/m on the legs and 350 kN/m on the spine
  ([ADR-0107](docs/DESIGN_DECISIONS.md), [ADR-0117](docs/DESIGN_DECISIONS.md)).
- **An articulated, cable-driven spine** (3 segments, pitch and yaw) does the
  balancing sway and the mid-air righting.

| One leg, as parts | The trunk: 19 motors in four rigid bodies |
|---|---|
| ![leg detail](docs/figures/leg_parts_m132.png) | ![trunk](docs/figures/trunk_m131.png) |

## Key numbers

| | |
|---|---|
| Mass | **4.81 kg** with its skin (19 motors: 12 leg, 6 spine, 1 tail) |
| Trunk / hips apart | 384 mm / 276 mm |
| Joint moment arms (hip / knee / ankle) | 36 / 34 / 22 mm |
| Motor demand (peak / continuous) | 1.82 / 0.67 N·m per motor |
| Trot | 50 cm/s |
| Runtime, trotting | 14–19 min (~585 m) on a 300 g pack |
| Righting from upside down | ~1.0 s (simulated) |

Every number is computed from the live model, not typed in: the CAD, the
MuJoCo plant and the budgets all read `kinematics/src/tomcat_kin/params.py`.

## Status

**Modelling and design — no hardware built.** What is still uncertain, and
what closes it, is in [OPEN_RISKS.md](docs/OPEN_RISKS.md). The largest open
items: the motor (a GIM3505-9 is a proxy for size and mass; the spec it must
meet is in ADR-0100/0110, and the dressed robot asks 93 % of its peak), the
runtime (14.3 min at the vendor's Kt, 0.3 min over the 14 min floor), the
mass (4.81 kg, 19.0 % over the 4.05 kg target), bench values for conduit
friction and the G3 rates, a mock-up of the cable bundle that twists inside
the hollow hip, and retraining the policy on the dressed robot.

## Repository

| Path | Contents |
|---|---|
| `kinematics/` | `tomcat_kin`: kinematics, tendon maps, mass, gait, balance, MuJoCo plants, RL env |
| `mechanical/cad/` | build123d CAD: trunk, legs, hollow hip, drive, G3 flexure, head, tail, skin |
| `thermal/` | Rust crate: motor and girdle thermal duty |
| `tests/` | pytest suite (569 tests) |
| `docs/` | requirements, architecture, the ADR log, the progress log |
| `electronics/`, `firmware/` | not started |

```bash
pip install numpy pytest mujoco gymnasium build123d vtk matplotlib pillow
pytest tests --ignore=tests/test_mjcf_tendon.py   # conftest.py puts kinematics/src on the path
cd thermal && cargo test
python mechanical/cad/tomcat_assembly.py --render   # the whole-robot picture
python mechanical/cad/leg_drive_3d.py --hq          # the drive animation (1600x900, 30 fps)
python mechanical/cad/cat_suit.py                   # the skin: checks, STL and picture (~1 GB)
python mechanical/cad/skin_mass.py                  # deal the skin's mass to the plant's bodies
```

> ⚠️ `tests/test_mjcf_tendon.py` needs ~19 GB as one process — run it in
> slices. The whole-robot render needs ~12 GB as one process.

## Documents

- [Design decisions (ADR log)](docs/DESIGN_DECISIONS.md) — every choice, and why
- [Progress log](docs/PROGRESS.md) — the milestone-by-milestone record
- [Requirements](docs/REQUIREMENTS.md) · [Architecture](docs/ARCHITECTURE.md) ·
  [Principles](docs/PRINCIPLES.md) · [Open risks](docs/OPEN_RISKS.md)
- [Literature review](docs/LITERATURE_REVIEW.md) · [References](docs/REFERENCES.md) ·
  [Glossary](docs/GLOSSARY.md)

## License

[Apache License 2.0](LICENSE). The feline skeletal geometry derives from
Reighard & Jennings, *Anatomy of the Cat* (1901), public domain; third-party
material is listed in [NOTICE](NOTICE).
