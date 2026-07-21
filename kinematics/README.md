# kinematics/

Kinematics models, tendon mapping, and gait planning for TomCat.

Responsibilities (see [../docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md)):
- **Leg kinematics:** forward/inverse kinematics (foot position ↔ joint angles).
- **Tendon map:** joint angle & stiffness ↔ per-motor cable length + tension,
  including moment arms (pulley radii), friction, and finite cable stiffness.
- **Antagonistic coordination:** split desired joint torque/stiffness across
  opposing tendons (or tendon + return spring) — see ADR-0002.
- **Gait planning:** foot/body trajectories for walk, trot, etc.

## Layout
- `src/tomcat_kin/` — the prototype package:
  - `params.py` — placeholder geometry / mass / tendon parameters (all TBD).
  - `leg.py` — planar 3R leg: FK, IK (both knee branches), Jacobian.
  - `tendon.py` — tendon map: angle↔cable, torque↔tension; antagonistic and
    spring-return modes (ADR-0002).
  - `torque_budget.py` — static worst-case joint/tendon/motor torque sweep.
- `demo.py` — runnable end-to-end example.
- Tests live in [../tests/](../tests) (`test_leg.py`, `test_tendon.py`).

## Run it
```
python kinematics/demo.py     # FK/IK round-trip, tendon resolve, torque budget
python -m pytest -q           # 17 tests (run from repo root)
```

## Status
Working single-leg prototype. **All parameters in `params.py` are placeholders**
— the torque-budget magnitudes are illustrative only until mechanical design
lands. The workspace sweep is also limited by the placeholder joint limits
(foot pitch is held at 0), so only part of the reachable area is currently
in-limit; revisit once real joint ranges exist.

Next steps: 3D (frontal-plane abduction) leg, gait trajectory generation, and
validating the tendon map against measured cable friction/stretch.
