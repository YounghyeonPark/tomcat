"""TomCat kinematics prototype.

A pure-software, single-leg model used to validate the tendon-driven design
before committing to hardware. See docs/ARCHITECTURE.md (mid-level: kinematics /
tendon map) and docs/DESIGN_DECISIONS.md (ADR-0002).

Modules
-------
params        Placeholder geometry / mass parameters (all values are TBD).
leg           Planar 3R leg: forward/inverse kinematics and Jacobian.
tendon        Joint-angle <-> cable-length and joint-torque <-> tendon-tension.
torque_budget Static worst-case joint-torque / motor-torque estimation.
"""

from .params import LegParams, TendonParams, LoadCase, DEFAULT_LEG, DEFAULT_TENDON
from .leg import LegModel, KneeConfig, UnreachableError
from .tendon import TendonMap, ActuationMode, TendonSolution

__all__ = [
    "LegParams",
    "TendonParams",
    "LoadCase",
    "DEFAULT_LEG",
    "DEFAULT_TENDON",
    "LegModel",
    "KneeConfig",
    "UnreachableError",
    "TendonMap",
    "ActuationMode",
    "TendonSolution",
]
