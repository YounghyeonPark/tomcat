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

from tomcat_kin import LegModel, TendonMap, ActuationMode  # noqa: E402
from tomcat_kin.params import DEFAULT_LOADS  # noqa: E402
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


if __name__ == "__main__":
    main()
