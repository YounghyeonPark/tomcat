"""Runnable demo of the TomCat single-leg kinematics + tendon map.

    python kinematics/demo.py

Prints an FK/IK round-trip check, the tendon resolution for a sample torque,
and the static torque budget across the default load cases. All numbers use the
PLACEHOLDER parameters in tomcat_kin.params.
"""

from __future__ import annotations

import os
import sys

import numpy as np

# Make the package importable when run directly from the repo.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from tomcat_kin import (  # noqa: E402
    LegModel,
    TendonMap,
    TendonParams,
    ActuationMode,
    SpineModel,
    WholeBody,
    Girdle,
)
from tomcat_kin.params import (  # noqa: E402
    DEFAULT_LOADS,
    DEFAULT_SPINE,
    DEFAULT_WHOLE_BODY_LOADS,
)
from tomcat_kin import torque_budget, whole_body_budget  # noqa: E402
from tomcat_kin.sensitivity import moment_arm_sweep  # noqa: E402
from tomcat_kin.params import DEFAULT_TENDON  # noqa: E402


def main() -> None:
    np.set_printoptions(precision=4, suppress=True)
    leg = LegModel()

    print("=== FK / IK round-trip ===")
    q_true = np.deg2rad([30.0, -60.0, 15.0])
    pose = leg.forward(q_true)
    q_ik = leg.inverse(pose)
    print(f"q (deg)        : {np.rad2deg(q_true)}")
    print(f"foot pose x,z,phi: {pose}")
    print(f"IK recovered q : {np.rad2deg(q_ik)}")
    print(f"max angle error: {np.max(np.abs(q_true - q_ik)):.2e} rad")

    print("\n=== Tendon resolution (antagonistic) for tau = [0.4, -0.6, 0.1] ===")
    tmap = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    sol = tmap.resolve([0.4, -0.6, 0.1])
    print(f"flexor tension  N : {sol.tension_flexor}")
    print(f"extensor tension N: {sol.tension_extensor}")
    print(f"motor torque  N·m : {sol.motor_torque}")
    print(f"realized tau  N·m : {sol.joint_torque}")
    print(f"motors required   : {sol.n_motors}")

    print("\n=== Commandable co-contraction bias (T_bias / AIC, ADR-0002) ===")
    tau = [0.4, -0.6, 0.1]
    for tb in (None, 15.0):
        s = tmap.resolve(tau, t_bias=tb)
        label = "default (=pretension)" if tb is None else f"T_bias={tb} N"
        print(f"[{label}]")
        print(f"  flexor / extensor N: {s.tension_flexor} / {s.tension_extensor}")
        print(f"  peak motor torque N·m: {np.abs(s.motor_torque)}")
        print(f"  realized tau N·m    : {s.joint_torque}")
    print("  (higher T_bias stiffens the joint: both tensions rise, tau unchanged)")

    print("\n=== Static torque budget (worst case over workspace) ===")
    for load in DEFAULT_LOADS:
        result = torque_budget.evaluate(leg, tmap, load, grid=25)
        print(result.report())
        print()

    print("=== Spine FK + whole-body (straight vs. arched back) ===")
    spine = SpineModel()
    body = WholeBody(spine=spine)

    # A common standing leg pose (hip points down, knee/ankle flexed).
    stand = np.deg2rad([-80.0, 60.0, 10.0])
    leg_q = {name: stand for name in body.leg_names}

    # Uniform positive bend = dorsiflexion (arched / "Halloween cat") posture.
    q_straight = np.zeros(DEFAULT_SPINE.n_segments)
    q_arch = np.full(DEFAULT_SPINE.n_segments, np.deg2rad(20.0))

    for label, q_spine in (("straight", q_straight), ("arched", q_arch)):
        rear = spine.girdle_pose(q_spine, Girdle.REAR)
        front = spine.girdle_pose(q_spine, Girdle.FRONT)
        feet = body.foot_positions(q_spine, leg_q)
        print(f"[{label}] spine q (deg): {np.rad2deg(q_spine)}")
        print(f"  rear girdle  (x,z,theta): {rear}")
        print(f"  front girdle (x,z,theta): {front}")
        print(f"  front foot LF (x,z): {feet['LF']}")
        print(f"  rear foot  LR (x,z): {feet['LR']}")

    # Show a spine joint getting the same tendon treatment as a leg joint.
    spine_tendons = TendonMap.from_spine(DEFAULT_SPINE, mode=ActuationMode.ANTAGONISTIC)
    spine_tau = np.full(DEFAULT_SPINE.n_segments, 0.2)  # N·m per segment
    ssol = spine_tendons.resolve(spine_tau)
    print(f"\nspine tendon tensions (flexor)  N: {ssol.tension_flexor}")
    print(f"spine tendon tensions (extensor) N: {ssol.tension_extensor}")
    print(f"spine realized joint torque  N·m : {ssol.joint_torque}")

    print("\n=== Tendon friction (capstan) + stretch (series compliance) ===")
    # An illustrative routing: mu=0.3 over ~180 deg of total wrap, and a
    # moderately stiff synthetic cable.  All still PLACEHOLDER (params default to
    # the frictionless / inextensible case; these are passed here to show effect).
    fric = TendonMap(
        params=TendonParams(friction_coeff=0.3, wrap_angle=np.pi, k_cable=5.0e4),
        mode=ActuationMode.ANTAGONISTIC,
    )
    fsol = fric.resolve([0.4, -0.6, 0.1])
    print(f"capstan factor exp(mu*theta): {fric.capstan_factor():.3f} "
          f"(pay-out {fric.capstan_factor(paying_out=True):.3f})")
    print(f"joint-side tension (flexor) N: {fsol.tension_flexor}")
    print(f"motor-side tension (flexor) N: {fsol.motor_tension_flexor}")
    print(f"  -> motor must supply ~{fric.capstan_factor():.2f}x the joint tension")
    active = np.maximum(fsol.tension_flexor, fsol.tension_extensor)
    print(f"cable stretch at active tension mm : {fric.cable_stretch(active) * 1e3}")
    print(f"extra motor angle to compensate deg: "
          f"{np.rad2deg(fric.extra_motor_angle(active))}")
    print(f"joint-angle error if uncompensated deg: "
          f"{np.rad2deg(fric.joint_angle_error(active))}")

    print("\n=== Combined whole-body static budget (spine + 4 legs) ===")
    wb = WholeBody(spine=SpineModel())
    leg_tendons = TendonMap(mode=ActuationMode.ANTAGONISTIC)
    land_result = None
    for wload in DEFAULT_WHOLE_BODY_LOADS:
        result = whole_body_budget.evaluate(
            wb, leg_tendons, spine_tendons, wload, grid=25
        )
        print(result.report())
        print()
        if "land" in wload.name:
            land_result = result

    print("=== Moment-arm sensitivity for the worst leg joint (land case) ===")
    # Worst-case leg joint torque from the whole-body land sweep drives the trade.
    worst_tau = float(np.max(land_result.leg.peak_joint_torque))
    worst_joint = ("hip", "knee", "ankle")[
        int(np.argmax(land_result.leg.peak_joint_torque))
    ]
    arms = [0.010, 0.015, 0.020, 0.030, 0.050, 0.100, 0.150, 0.200]
    print(f"driving torque: worst leg joint = {worst_joint}, |tau| = {worst_tau:.2f} N·m\n")
    print("[frictionless]")
    print(moment_arm_sweep(worst_tau, arms, t_bias=DEFAULT_TENDON.pretension).report())
    print("\n[with routing friction mu=0.3, theta_wrap=180 deg]")
    print(
        moment_arm_sweep(
            worst_tau, arms,
            t_bias=DEFAULT_TENDON.pretension,
            friction_coeff=0.3, wrap_angle=np.pi,
        ).report()
    )
    print()


if __name__ == "__main__":
    main()
