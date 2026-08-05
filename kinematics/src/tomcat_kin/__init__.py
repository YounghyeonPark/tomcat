# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""TomCat kinematics prototype.

A pure-software, single-leg model used to validate the tendon-driven design
before committing to hardware. See docs/ARCHITECTURE.md (mid-level: kinematics /
tendon map) and docs/DESIGN_DECISIONS.md (ADR-0002).

Modules
-------
params            Placeholder geometry / mass parameters (all values are TBD),
                  incl. the M4 distributed mass budget (~3 kg, ~60/40 fore/hind).
leg               Digitigrade 4-link leg (3 actuated joints + passive paw):
                  paw-tip forward/inverse kinematics and Jacobian.
mass              Rigid-body mass properties: per-link CoMs, sub-assembly and
                  whole-body centre of mass, fore/hind weight split (M4).
stability         Quasi-static stability: CoM ground projection vs. the sagittal
                  fore-aft support interval, signed tipping margin (M4).
spine             Serial tendon-driven spine + whole-body (spine + 4 legs) kinematics,
                  incl. whole-body INVERSE kinematics (world foot pose -> leg angles
                  through the moving girdle; M3) and the whole-body CoM (M4).
tendon            Joint-angle <-> cable-length and joint-torque <-> tendon-tension,
                  with commandable co-contraction bias (T_bias / AIC, ADR-0002).
torque_budget     Static worst-case per-leg joint-torque / motor-torque estimation.
whole_body_budget Combined spine+legs static tendon/torque + motor-count budget.
sensitivity       Moment-arm vs. joint/motor cable-tension trade sweep (ADR-0003).
gait              Parameterized periodic WALK gait: foot trajectories -> per-leg IK
                  -> joint-angle sequences (sagittal, quasi-static; M2). Adds the
                  world-frame WholeBodyGaitController that holds stance feet planted
                  in the world while the spine moves (closed spine<->foot loop; M3).
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
    DEFAULT_FORELEG,
    DEFAULT_HINDLEG,
    DEFAULT_BODY_MASS_KG,
)
from .leg import LegModel, KneeConfig, UnreachableError
from .mass import (
    ComResult,
    QuarterMasses,
    combine,
    leg_com,
    leg_link_coms,
    point_masses_com,
    quarter_masses,
    spine_chain_com,
    spine_segment_coms,
)
from .stability import (
    StabilityMargin,
    SupportInterval,
    sagittal_stability_margin,
    centering_shift,
    SupportPolygon,
    polygon_stability_margin,
)
from . import dynamics                                    # noqa: F401  (M6)
from . import control                                     # noqa: F401  (M8)
from .control import StepPlant, placement, rejection_envelope
from .dynamics import (
    ContactSolution,
    ZMPResult,
    CycleData,
    contact_forces,
    zero_moment_point,
)
from .spine import (
    SpineModel,
    WholeBody,
    BodyCoM,
    Girdle,
    LegMount,
    LegIKSolution,
    DEFAULT_MOUNTS,
)
from .tendon import TendonMap, ActuationMode, TendonSolution
from . import torque_budget, whole_body_budget, sensitivity, mass, stability
from .whole_body_budget import (
    WholeBodyBudgetResult,
    spine_joint_torques,
    gravity_loads,
)
from .sensitivity import moment_arm_sweep, MomentArmSweepResult, ROBOCAT_BAND_N
from .gait import (
    GaitParams,
    GaitController,
    GaitState,
    LegState,
    WholeBodyGaitController,
    WholeBodyGaitState,
    WholeBodyLegState,
    foot_target,
    swing_height,
    DEFAULT_PHASE_OFFSETS,
)

__all__ = [
    "control", "StepPlant", "placement", "rejection_envelope",
    "dynamics", "ContactSolution", "ZMPResult", "CycleData",
    "contact_forces", "zero_moment_point",
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
    "DEFAULT_FORELEG",
    "DEFAULT_HINDLEG",
    "DEFAULT_BODY_MASS_KG",
    "LegModel",
    "KneeConfig",
    "UnreachableError",
    "mass",
    "ComResult",
    "QuarterMasses",
    "combine",
    "leg_com",
    "leg_link_coms",
    "point_masses_com",
    "quarter_masses",
    "spine_chain_com",
    "spine_segment_coms",
    "stability",
    "StabilityMargin",
    "SupportPolygon",
    "polygon_stability_margin",
    "SupportInterval",
    "sagittal_stability_margin",
    "centering_shift",
    "SpineModel",
    "WholeBody",
    "BodyCoM",
    "Girdle",
    "LegMount",
    "LegIKSolution",
    "DEFAULT_MOUNTS",
    "TendonMap",
    "ActuationMode",
    "TendonSolution",
    "torque_budget",
    "whole_body_budget",
    "sensitivity",
    "WholeBodyBudgetResult",
    "spine_joint_torques",
    "gravity_loads",
    "moment_arm_sweep",
    "MomentArmSweepResult",
    "ROBOCAT_BAND_N",
    "GaitParams",
    "GaitController",
    "GaitState",
    "LegState",
    "WholeBodyGaitController",
    "WholeBodyGaitState",
    "WholeBodyLegState",
    "foot_target",
    "swing_height",
    "DEFAULT_PHASE_OFFSETS",
]
