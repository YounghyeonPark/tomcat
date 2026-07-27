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
sensitivity       Moment-arm vs. joint/motor cable-tension trade sweep (ADR-0003).
gait              Parameterized periodic WALK gait: foot trajectories -> per-leg IK
                  -> joint-angle sequences (sagittal, quasi-static; M2).
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
from . import torque_budget, whole_body_budget, sensitivity
from .whole_body_budget import WholeBodyBudgetResult, spine_joint_torques
from .sensitivity import moment_arm_sweep, MomentArmSweepResult, ROBOCAT_BAND_N
from .gait import (
    GaitParams,
    GaitController,
    GaitState,
    LegState,
    foot_target,
    swing_height,
    DEFAULT_PHASE_OFFSETS,
)

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
    "sensitivity",
    "WholeBodyBudgetResult",
    "spine_joint_torques",
    "moment_arm_sweep",
    "MomentArmSweepResult",
    "ROBOCAT_BAND_N",
    "GaitParams",
    "GaitController",
    "GaitState",
    "LegState",
    "foot_target",
    "swing_height",
    "DEFAULT_PHASE_OFFSETS",
]
