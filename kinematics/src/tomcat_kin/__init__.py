"""TomCat kinematics prototype.

A pure-software, single-leg model used to validate the tendon-driven design
before committing to hardware. See docs/ARCHITECTURE.md (mid-level: kinematics /
tendon map) and docs/DESIGN_DECISIONS.md (ADR-0002).

Modules
-------
params            Placeholder geometry / mass parameters (all values are TBD).
leg               Planar 3R leg: forward/inverse kinematics and Jacobian.
spine             Serial tendon-driven spine + whole-body (spine + 4 legs) kinematics.
tendon            Joint-angle <-> cable-length and joint-torque <-> tendon-tension,
                  with commandable co-contraction bias (T_bias / AIC, ADR-0002).
torque_budget     Static worst-case per-leg joint-torque / motor-torque estimation.
whole_body_budget Combined spine+legs static tendon/torque + motor-count budget.
"""

from .params import (
    LegParams,
    TendonParams,
    SpineParams,
    LoadCase,
    WholeBodyLoadCase,
    DEFAULT_LEG,
    DEFAULT_TENDON,
    DEFAULT_SPINE,
    DEFAULT_LOADS,
    DEFAULT_WHOLE_BODY_LOADS,
)
from .leg import LegModel, KneeConfig, UnreachableError
from .spine import SpineModel, WholeBody, Girdle, LegMount, DEFAULT_MOUNTS
from .tendon import TendonMap, ActuationMode, TendonSolution
from . import torque_budget, whole_body_budget
from .whole_body_budget import WholeBodyBudgetResult, spine_joint_torques

__all__ = [
    "LegParams",
    "TendonParams",
    "SpineParams",
    "LoadCase",
    "WholeBodyLoadCase",
    "DEFAULT_LEG",
    "DEFAULT_TENDON",
    "DEFAULT_SPINE",
    "DEFAULT_LOADS",
    "DEFAULT_WHOLE_BODY_LOADS",
    "LegModel",
    "KneeConfig",
    "UnreachableError",
    "SpineModel",
    "WholeBody",
    "Girdle",
    "LegMount",
    "DEFAULT_MOUNTS",
    "TendonMap",
    "ActuationMode",
    "TendonSolution",
    "torque_budget",
    "whole_body_budget",
    "WholeBodyBudgetResult",
    "spine_joint_torques",
]
