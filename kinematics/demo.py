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
    ActuationMode,
    SpineModel,
    WholeBody,
    Girdle,
)
from tomcat_kin.params import DEFAULT_LOADS, DEFAULT_SPINE  # noqa: E402
from tomcat_kin import torque_budget  # noqa: E402


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


if __name__ == "__main__":
    main()
