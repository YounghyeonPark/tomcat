# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""How slim can a leg get? Joint sheave size against where the reduction lives (M159).

    python tools/sheave_trade.py

A joint's sheave radius is its moment arm, and with the spool it is the drive's
last reduction: joint torque = T r_sheave, motor torque = T r_spool, so the
pair carries the ratio r_sheave / r_spool. ADR-0103 grew the sheaves to
36/34/22 mm because the motor ran short of torque, and the knee and ankle are
now 68 and 44 mm across.

To shrink a sheave and keep the joint's torque, the reduction has to come from
somewhere else: a gear stage at the motor (or a motor with more torque, which
scales the same way), factor `g` (ratio x efficiency). The overall ratio, the
motor's torque and speed and the joint's stiffness are kept. Then:

- the cable carries g times the torque at the same spool, so it must be
  thicker: strength goes as area, so d ~ sqrt(T);
- the spool cannot bend it tighter than 10 d across (`motor_spool_radius`'s
  rule), so the spool grows with d;
- solving the two, the spool grows as g^(1/3), the tension as g^(2/3) and the
  sheave SHRINKS as g^(-2/3);
- a sheave cannot be smaller than the spool either (the same bend rule), which
  floors each joint at g = its own ratio r_sheave / r_spool.

Nothing here is a decision; it prices the options.
"""

from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "kinematics", "src"))

JOINTS = ("hip", "knee", "ankle")


def scenario(g, arms_m, r_spool_m, cable_d_mm, t_max_n):
    """The drive at reduction factor `g` moved to the motor side."""
    r_s = r_spool_m * g ** (1.0 / 3.0)
    d = cable_d_mm * g ** (1.0 / 3.0)
    t = t_max_n * g ** (2.0 / 3.0)
    r_sh = np.maximum(np.asarray(arms_m) * g ** (-2.0 / 3.0), r_s)
    # a joint held at its floor keeps the bigger overall ratio: more torque,
    # and its motor must turn that much faster for the same joint speed
    over = g * r_sh / r_s / (np.asarray(arms_m) / r_spool_m)
    return {"g": g, "spool_r_mm": 1e3 * r_s, "cable_d_mm": d, "tension_max_n": t,
            "sheave_r_mm": 1e3 * r_sh, "ratio_factor": over}


def envelope_mm(r_sheave_mm, cable_d_mm):
    """Across the joint: the sheave's flanged OD, and the suit over it
    (`cat_suit.SLEEVE`: + OFFSET + PAD each side)."""
    flange = 0.55 * cable_d_mm + 1.2          # tomcat_leg_detail GROOVE_R + FLANGE_H
    bare = 2.0 * (r_sheave_mm + flange)
    suit = 2.0 * (r_sheave_mm + 3.25 + 6.0)
    return bare, suit


def main():
    from tomcat_kin import mjcf_tendon as MT
    from tomcat_kin.params import DEFAULT_TENDON as TP

    arms = np.asarray(TP.joint_moment_arm)
    r_s0 = TP.motor_spool_radius
    d0 = 1.75
    ratio = arms / r_s0
    print("today: sheave/spool ratio hip %.2f, knee %.2f, ankle %.2f; T_max %.0f N"
          % (*ratio, MT.TENSION_MAX))
    print("each joint's floor (sheave = spool): g = its ratio\n")
    print("%5s | %-17s | %6s %6s %7s | %-26s | %-15s | %s"
          % ("g", "sheave r hip/kn/ank", "spool", "cable", "T_max",
             "knee / ankle across: bare", "suit", "ankle ratio x"))
    rows = []
    for g in (1.0, 1.5, 2.0, ratio[2], 3.0, ratio[1]):
        s = scenario(g, arms, r_s0, d0, MT.TENSION_MAX)
        bk, sk = envelope_mm(s["sheave_r_mm"][1], s["cable_d_mm"])
        ba, sa = envelope_mm(s["sheave_r_mm"][2], s["cable_d_mm"])
        rows.append((s, bk, sk, ba, sa))
        print("%5.2f | %5.1f %5.1f %5.1f | %5.2f %6.2f %7.0f | %5.1f / %5.1f mm | %5.1f / %5.1f mm | %.2f"
              % (g, *s["sheave_r_mm"], s["spool_r_mm"], s["cable_d_mm"], s["tension_max_n"],
                 bk, ba, sk, sa, s["ratio_factor"][2]))
    return rows


if __name__ == "__main__":
    main()
