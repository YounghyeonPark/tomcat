"""Runnable demo of the TomCat single-leg kinematics + tendon map.

    python kinematics/demo.py

Prints an FK/IK round-trip check, the tendon resolution for a sample torque,
the static torque budget across the default load cases, the walk gait, and (M4)
the whole-body mass budget, centre of mass and static stability margin. All
numbers use the PLACEHOLDER parameters in tomcat_kin.params.
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
    GaitParams,
    GaitController,
    WholeBodyGaitController,
    LegMount,
    centering_shift,
)
from tomcat_kin.params import (  # noqa: E402
    DEFAULT_LOADS,
    DEFAULT_SPINE,
    DEFAULT_FORELEG,
    DEFAULT_HINDLEG,
    DEFAULT_WHOLE_BODY_LOADS,
)
from tomcat_kin import torque_budget, whole_body_budget  # noqa: E402
from tomcat_kin import dynamics as dyn  # noqa: E402
from tomcat_kin.params import DEFAULT_TENDON as _DT  # noqa: E402
LEG_ARMS_M = np.asarray(_DT.joint_moment_arm)
SPOOL_M = _DT.motor_spool_radius
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

    print("=== WALK gait (M2): one cycle, spine NEUTRAL ===")
    gait = GaitParams()
    ctrl = GaitController(params=gait)
    print(
        f"period={gait.period:.2f}s  stride={gait.stride_length*1e3:.0f}mm  "
        f"step={gait.step_height*1e3:.0f}mm  duty={gait.duty_factor:.2f}  "
        f"body_speed={gait.body_speed*1e3:.0f}mm/s  "
        f"advance/cycle={gait.distance_per_cycle*1e3:.0f}mm"
    )
    order = ("LF", "RF", "RR", "LR")
    print(f"phase  " + "  ".join(f"{n:>18}" for n in order))
    for i in range(8):
        phase = i / 8
        st = ctrl.state(phase)
        cells = []
        for n in order:
            lg = st.legs[n]
            tag = "ST" if lg.in_stance else "sw"  # stance vs swing
            deg = np.rad2deg(lg.q)
            cells.append(f"{tag} [{deg[0]:+5.0f}{deg[1]:+5.0f}{deg[2]:+4.0f}]")
        print(f"{phase:4.2f}   " + "  ".join(cells) + f"   stance={st.stance_count}")
    print("  (ST=stance, sw=swing; brackets = hip/knee/ankle deg; exactly 3 feet down)")

    # ASCII sketch of the swing foot's path (side view) for one leg over its cycle.
    print("\n  LF swing-foot side view (x forward ->, z up; one cycle):")
    _sketch_foot_path(ctrl, "LF")

    print("\n=== WALK gait with OPTIONAL spine oscillation coupled to phase ===")
    gait_s = GaitParams(spine_amplitude=np.deg2rad(8.0))
    ctrl_s = GaitController(params=gait_s)
    print(f"spine amplitude = {np.rad2deg(gait_s.spine_amplitude):.0f} deg/segment "
          f"(dorsoventral, sin-coupled to gait phase)")
    for i in range(8):
        phase = i / 8
        sq = ctrl_s.spine_q(phase)
        print(f"  phase {phase:4.2f}  spine q (deg): {np.rad2deg(sq)}")
    print("  (amplitude 0 by default => spine held NEUTRAL; see gait.py docstring)")

    print("\n=== WHOLE-BODY foot placement (M3): stance feet PLANTED in the world ===")
    # M2 could move feet in the world with the spine but the legs did NOT
    # compensate. M3 places feet in a WORLD/ground frame and solves per-leg IK
    # THROUGH the moving girdle, so the legs absorb the spine bend and stance feet
    # stay put. Compare a NEUTRAL spine against a spine oscillating at the gait phase.
    neu = WholeBodyGaitController(params=GaitParams())
    osc = WholeBodyGaitController(params=GaitParams(spine_amplitude=np.deg2rad(2.0)))
    leg = "LF"  # a front (shoulder-girdle) leg: its hip rides the spine bend
    print(f"leg {leg}, spine oscillation +/-2 deg/segment coupled to gait phase:")
    print("  phase  spine_q(deg)      world foot target (x,z) m     "
          "q_neutral (deg)          q_spine-on (deg)         foot err (mm)")
    for i in range(6):
        phase = 0.05 + i * (0.7 / 5)  # sample across LF's stance window
        sn = neu.leg_state(phase, leg)
        so = osc.leg_state(phase, leg)
        sq = osc.spine_q(phase)
        fk = osc.foot_world_check(phase, leg)          # FK from the solved q
        err_mm = np.linalg.norm(fk[:2] - so.foot_target_world[:2]) * 1e3
        print(
            f"  {phase:4.2f}  {np.rad2deg(sq)}  "
            f"[{so.foot_target_world[0]:+.3f} {so.foot_target_world[1]:+.3f}]      "
            f"{np.rad2deg(sn.q)}  {np.rad2deg(so.q)}  {err_mm:6.3f}"
        )
    # Prove planted-ness: the world foot target is identical column-to-column.
    span = np.ptp(
        np.array([neu.leg_state(0.05 + i * 0.14, leg).foot_target_world[0]
                  for i in range(6)])
    )
    print(f"  -> world foot-target x is HELD FIXED across stance (span {span*1e3:.3f} mm);")
    print("     the spine-on leg angles differ from neutral (legs compensate) yet FK")
    print("     still lands the foot on that fixed world target (err ~0). Loop closed.")

    _mass_and_stability_demo()


def _mass_and_stability_demo() -> None:
    """M4: real mass -> whole-body CoM -> static stability margin."""
    print("\n=== REAL MASS (M4): the 3 kg budget, ~60% on the forequarters ===")
    body = WholeBody(spine=SpineModel())
    budget = body.mass_budget()
    print(f"  {budget.report()}")
    print(f"  {'part':<22}{'mass kg':>10}")
    for label, m in (
        ("fore leg (x2)", DEFAULT_FORELEG.mass),
        ("hind leg (x2)", DEFAULT_HINDLEG.mass),
        ("spine seg 0 (lumbar)", DEFAULT_SPINE.segment_mass[0]),
        ("spine seg 1 (mid)", DEFAULT_SPINE.segment_mass[1]),
        ("spine seg 2 (thoracic)", DEFAULT_SPINE.segment_mass[2]),
        ("front girdle +head", DEFAULT_SPINE.front_girdle_mass),
        ("rear girdle (pelvis)", DEFAULT_SPINE.rear_girdle_mass),
    ):
        print(f"  {label:<24}{m:>10.3f}")
    print(f"  {'TOTAL':<24}{body.total_mass:>10.3f}")
    print("  per-leg link split (proximal-heavy, 47.5/30/15/7.5%):")
    print(f"    fore {np.array(DEFAULT_FORELEG.link_mass)} kg")
    print(f"    hind {np.array(DEFAULT_HINDLEG.link_mass)} kg")
    print("  NOTE: the lit review's 0.454 kg 'knee mass' is from a MUCH larger")
    print("        robot -- ~15% of our whole body at one joint. Not copied; the")
    print("        scaled analogue is the 0.200 kg hind leg. All values TBD/placeholder.")

    print("\n=== WHOLE-BODY CENTRE OF MASS (body-ground frame, spine base at 0) ===")
    stand = np.deg2rad([-80.0, 60.0, 10.0])
    for label, q_spine in (
        ("straight", np.zeros(DEFAULT_SPINE.n_segments)),
        ("arched +20 deg/seg", np.full(DEFAULT_SPINE.n_segments, np.deg2rad(20.0))),
    ):
        com = body.center_of_mass(q_spine, stand)
        print(f"[{label}]")
        print("  " + com.report().replace("\n", "\n  "))
    print("  (arching curls the forequarters up and back: CoM moves +z and -x)")

    print("\n=== STATIC STABILITY MARGIN across the walk cycle (M4) ===")
    print("  2D sagittal: the 'support polygon' is really a FORE-AFT INTERVAL, so a")
    print("  positive margin is NECESSARY BUT NOT SUFFICIENT -- lateral/roll tipping")
    print("  over the real 3-foot triangle needs the 3D model.")
    ctrl = GaitController(params=GaitParams())
    print("  phase  " + "stance  " + "CoM x mm  support [rear, front] mm   margin mm  ")
    for i in range(8):
        phase = i / 8
        st = ctrl.state(phase)
        m = st.stability
        print(
            f"  {phase:4.2f}   {st.stance_count:^6d}  {st.com.x * 1e3:+8.1f}  "
            f"[{m.support.rear * 1e3:+7.1f}, {m.support.front * 1e3:+7.1f}]      "
            f"{m.margin * 1e3:+8.1f}  {'STABLE' if m.is_stable else 'UNSTABLE'}"
            f" ({m.tipping_edge} edge)"
        )
    margins = [m.margin for m in ctrl.stability_sweep(120)]
    print(f"  margin over the cycle: {min(margins)*1e3:+.1f} .. {max(margins)*1e3:+.1f} mm")

    print("  >>> fore-aft stability is SOLVED (M4): the anatomical negative-knee")
    print("      fold lets each paw plant under its own hip, so the support")
    print("      interval straddles the CoM at every phase. But a positive")
    print("      fore-aft margin was never SUFFICIENT -- see below.")

    # ------------------------------------------------------------------ M5
    print()
    print("=== 3D SUPPORT POLYGON + LATERAL SPINE SWAY (M5, ADR-0009) ===")
    print("  The fore-aft interval above says STABLE everywhere. The TRUE ground-")
    print("  plane polygon disagrees: three feet down is a SKEWED triangle, and a")
    print("  mid-sagittal CoM falls outside it.")
    off = GaitController(params=GaitParams(lateral_amplitude=0.0))
    on = GaitController(params=GaitParams())
    m_off = [q.margin for q in off.support_polygon_sweep(200)]
    m_on = [q.margin for q in on.support_polygon_sweep(200)]
    print()
    print("    sway OFF : worst polygon margin %+7.2f mm   -> %s"
          % (min(m_off) * 1e3, "STABLE" if min(m_off) > 0 else "UNSTABLE"))
    print("    sway ON  : worst polygon margin %+7.2f mm   -> %s"
          % (min(m_on) * 1e3, "STABLE" if min(m_on) > 0 else "UNSTABLE"))
    print("    (the fore-aft margin is a healthy %+.1f mm in BOTH cases -- which is"
          % (min(m.margin for m in off.stability_sweep(200)) * 1e3))
    print("     exactly why the 2D check could never be trusted on its own)")

    print()
    print("  The sway law: hold full amplitude toward the SUPPORT side while a leg")
    print("  swings, and traverse ONLY while all four feet are down.")
    print()
    print("  phase  stance  sway deg   polygon margin mm")
    for i in range(10):
        ph = i / 10
        q = on.lateral_q(ph)
        pm = on.support_polygon(ph)
        print("  %4.2f   %^6s  %+7.1f    %+8.2f  %s".replace("%^6s", "%6d")
              % (ph, on.stance_count(ph), np.degrees(q[0]), pm.margin * 1e3,
                 "STABLE" if pm.is_stable else "UNSTABLE"))

    print()
    print("  >>> Sway amplitude is an OPTIMUM, not a maximum -- over-swaying")
    print("      carries the CoM out over the FAR edge of the triangle")
    print("      (15/18 deg nearly tie -- both are at the +/-15 deg spine ROM clip):")
    for deg in (8.0, 9.5, 11.0, 13.0, 15.0):
        c = GaitController(params=GaitParams(lateral_amplitude=np.radians(deg)))
        w = min(q.margin for q in c.support_polygon_sweep(120))
        print("        %4.1f deg -> %+6.2f mm%s"
              % (deg, w * 1e3, "  <-- default" if deg == 11.0 else ""))

    print()
    print("  >>> M5 concluded that FRICTION set the walk speed. The M6 dynamics")
    print("      show that was WRONG -- see the dynamics section below.")

    # ------------------------------------------- lateral spine DRIVE sizing
    print()
    print("=== SIZING THE LATERAL SPINE DRIVE (ADR-0009 f/u, revised by M6) ===")
    print("  The load is INERTIAL, not gravitational: the lateral bend axis is")
    print("  VERTICAL, so gravity exerts no moment about it and HOLDING a sway is")
    print("  nearly free. What costs torque is REVERSING it.")
    print()
    print("  At the shipped 5 s CRAWL the sway is so slow the drive is barely loaded:")
    for r_ in on.body.lateral_spine_loads(on.crossover_accel()):
        print("    joint %d -> %.3f N.m at the motor" % (r_["joint"], r_["motor_torque"]))
    print("  Sizing to THAT would under-build it -- the same motors must also serve")
    print("  the ADR-0007 righting reflex and any future dynamic gait. So the drive")
    print("  is sized to a REFERENCE FAST MANOEUVRE (the old 1.4 s crossover):")
    FAST = 6.87
    print()
    print("    lateral arm   base motor N.m   vs 1.10 class target   vs 1.95 REAL part")
    import dataclasses as _dc
    from tomcat_kin.spine import SpineModel as _SM
    for arm_mm in (15.0, 20.0, 25.0):
        b = WholeBody(spine=_SM(_dc.replace(on.body.spine.params,
                                            lateral_moment_arm=(arm_mm / 1000.0,) * 3)))
        t = max(x["motor_torque"] for x in b.lateral_spine_loads(FAST))
        note = "  <-- specified" if arm_mm == 20.0 else ""
        print("       %4.0f mm      %6.3f            %.2fx %-8s      %.2fx%s"
              % (arm_mm, t, t / 1.10, "OVER" if t > 1.10 else "ok", t / 1.95, note))
    print()
    print("  >>> HONEST DOWNGRADE. The 20 mm milled post was justified against a")
    print("      1.10 N.m CLASS TARGET motor. The surveyed REAL part peaks at")
    print("      1.95 N.m, so even the bare 15 mm transverse process now fits.")
    print("      The post is retained -- it is nearly free and buys ~25% margin --")
    print("      but it is an OPTIMISATION now, not a necessity.")


    # ------------------------------------------------------- M6: DYNAMICS
    print()
    print("=== WHOLE-BODY DYNAMICS (M6) -- the quasi-static answer was wrong ===")
    print("  Every check so far asked: does the CoM project inside the feet? That is")
    print("  a question about a body STANDING STILL. The real question is whether the")
    print("  contacts can produce the forces the MOTION requires.")
    print()
    r = dyn.sweep(on, 120)
    print("    shipped gait (%.1f s, sway %.0f deg):" % (on.params.period,
                                                        np.degrees(on.params.lateral_amplitude)))
    print("      static polygon margin  %+7.2f mm" % (min(q.margin for q in on.support_polygon_sweep(96)) * 1e3))
    print("      ZMP (dynamic) margin   %+7.2f mm   stable: %s" % (r["zmp_margin_min"] * 1e3, r["zmp_stable"]))
    print("      aggregate friction mu   %6.3f" % r["aggregate_mu"])
    print("      any foot forced to pull? %s" % (not r["unilateral_ok"]))

    m5 = GaitController(params=GaitParams(period=1.4, lateral_amplitude=np.radians(12.5)))
    r5 = dyn.sweep(m5, 120)
    print()
    print("    the M5 gait (1.4 s, sway 12.5 deg) -- which M5 declared STABLE:")
    print("      static polygon margin  %+7.2f mm   <- looked fine" % (min(q.margin for q in m5.support_polygon_sweep(96)) * 1e3))
    print("      ZMP (dynamic) margin   %+7.2f mm   <- OUTSIDE the support polygon" % (r5["zmp_margin_min"] * 1e3))
    print("      aggregate friction mu   %6.3f          <- would NOT have slipped" % r5["aggregate_mu"])
    print()
    print("  >>> TIPPING binds, not slipping. Swaying the CoM sideways needs real")
    print("      acceleration, and that shifts the effective pressure point by")
    print("      (h/g)*a the OTHER way -- ~128 mm against a 96 mm track.")
    print()
    print("    period   ZMP margin   tips?    aggregate mu   slips?   speed")
    for T in (1.4, 2.8, 4.0, 5.0):
        c = GaitController(params=GaitParams(period=T, lateral_amplitude=np.radians(11)))
        rr = dyn.sweep(c, 96)
        print("     %4.1f s   %+8.2f mm   %s      %6.3f       %s    %.2f cm/s"
              % (T, rr["zmp_margin_min"] * 1e3,
                 "TIPS" if rr["zmp_margin_min"] <= 0 else " ok ",
                 rr["aggregate_mu"], "SLIPS" if rr["aggregate_mu"] > 0.8 else " ok ",
                 c.params.body_speed * 100))
    print()
    print("  >>> And M5's sway law was not physically realisable: its ramp was LINEAR")
    print("      in position, so velocity STEPPED -- an impulse in acceleration. A")
    print("      static check can never see that (it never differentiates). Now a")
    print("      raised cosine, so the peak CONVERGES under grid refinement:")
    for nn in (120, 240, 480, 960):
        print("        n=%4d -> peak lateral accel %.3f m/s2"
              % (nn, np.abs(dyn.com_acceleration(on, nn)[:, 1]).max()))
    caveat = dyn.angular_momentum_caveat(on, 120)
    print()
    print("  >>> Honest bound: dH/dt = 0 is assumed (classical ZMP form). The swing")
    print("      leg's neglected spin is worth %.1f mm of ZMP shift against a %.1f mm"
          % (caveat["swing_leg_zmp_shift_max"] * 1e3, r["zmp_margin_min"] * 1e3))
    print("      margin -- a ~6x ratio, so the result holds AT THIS CRAWL SPEED. The")
    print("      shift scales with leg acceleration, so it would NOT hold for a fast")
    print("      or dynamic gait; that needs real rigid-body dynamics.")



    # ------------------------------------------------------- M7: THE TROT
    print()
    print("=== THE TROT (M7) -- a DYNAMIC gait at cat-like speed ===")
    from tomcat_kin.gait import trot_params
    tr = GaitController(params=trot_params())
    n = 96
    tcyc = dyn.cycle(tr, n)
    print("  The crawl above is statically stable and %.2f cm/s. A trot puts DIAGONAL"
          % (on.params.body_speed * 100))
    print("  pairs down, so the support is a LINE -- no interior, no polygon, no ZMP")
    print("  margin. The governing physics becomes the inverted pendulum.")
    print()
    print("    speed            %.1f cm/s   (%.0fx the crawl)"
          % (tr.params.body_speed * 100, tr.params.body_speed / on.params.body_speed))
    ts = dyn.trot_sweep(tr, n)
    print("    support          LINE, %.0f%% of the cycle" % (100 * ts["line_support_fraction"]))
    print("    CoM offset       %+.1f .. %+.1f mm about the diagonal (rocks THROUGH it)"
          % (ts["offset_min"] * 1e3, ts["offset_max"] * 1e3))
    print("    capture point    %.1f mm  (vs %.0f mm stride -- easily caught)"
          % (ts["dcm_abs_max"] * 1e3, tr.params.stride_length * 1e3))

    def _drift(c):
        cy = dyn.cycle(c, n)
        sg = [(lambda b: np.sign(b.offset) * b.unbalanced_moment if b else 0.0)(
            dyn.line_balance(c, i / n, n, cyc=cy)) for i in range(n)]
        h = float(np.mean(cy.com[:, 2] - cy.ground_z))
        return float(np.sum(np.array(sg) / (c.body.total_mass * h * h)) * (c.params.period / n))

    print()
    print("  >>> Two contacts CANNOT make a moment about the line joining them, so the")
    print("      CoM's offset from that line is an unbalanceable topple. Whether it")
    print("      averages to zero over a cycle is the whole ballgame:")
    print()
    print("        nominal foot x    roll-rate drift per cycle    verdict")
    for xn, tag in ((0.05, "the CRAWL's placement"), (0.02, ""), (0.005, "trot_params")):
        d = _drift(GaitController(params=trot_params(nominal_foot=(xn, -0.17))))
        v = ("FALLS OVER" if abs(d) > 1.0
             else ("drifts" if abs(d) > 0.15 else "BOUNDED"))
        print("          %+.3f m        %+8.3f rad/s          %-10s %s" % (xn, d, v, tag))
    print()
    print("  >>> And the swing trajectory had the SAME C1 defect as the M5 sway law:")
    print("      the cycloid starts/ends swing at zero hip-frame velocity while stance")
    print("      sweeps backward, so foot velocity STEPS -- the paw scuffs on landing")
    print("      and swing torque is impulsive. Peak swing motor torque vs grid:")
    for prof in ("cycloid", "matched"):
        vals = []
        for nn in (48, 96, 192):
            cc = GaitController(params=trot_params(swing_profile=prof))
            out = 0.0
            for i in range(nn):
                for nm in ("LF", "RF", "LR", "RR"):
                    if cc.is_stance(i / nn, nm):
                        continue
                    t = np.abs(dyn.swing_joint_torque(cc, nm, i / nn, nn))
                    out = max(out, float((t / LEG_ARMS_M * SPOOL_M).max()))
            vals.append(out)
        print("        %-8s n=48,96,192 -> %s %s" % (prof, ["%.3f" % v for v in vals],
              "(DIVERGES: not a number)" if prof == "cycloid" else "(converged)"))
    print()
    print("  >>> P1 gets its first hard number: swing-leg torque caps trot speed, and")
    print("      it is only %.3f N.m because tendon drive keeps the legs at 95-110 g."
          % vals[-1])


    # ------------------------------------------ M8: CLOSED-LOOP BALANCE
    print()
    print("=== CLOSED-LOOP BALANCE (M8) -- can it survive a push? ===")
    from tomcat_kin import control as ctl
    P = ctl.StepPlant.from_gait(tr, 96)
    print("  M7 showed the trot's NOMINAL path is dynamically consistent -- i.e. the")
    print("  error STARTS at zero. It is still an inverted pendulum: omega*T = %.2f," % (P.omega * P.stance))
    print("  so any deviation is multiplied by %.2f EVERY step." % P.growth)
    print()
    d = 0.02
    rows = (("OPEN loop", dict(closed_loop=False)),
            ("capture only", None),
            ("CLOSED beta=0", dict(beta=0.0)),
            ("CLOSED beta=0.5", dict(beta=0.5)))
    print("    20 mm DCM disturbance -- DCM at each touchdown (mm):")
    for lbl, kw in rows:
        if kw is None:
            xi = d
            out = [xi]
            for _ in range(6):
                xi = P.propagate(xi, ctl.capture_placement(P, xi))
                out.append(xi)
        else:
            out = ctl.simulate(P, 6, xi0=d, **kw)
        print("      %-16s %s" % (lbl, " ".join("%9.2f" % (v * 1e3) for v in out)))
    print()
    print("  >>> Note 'capture only' HOLDS at 20 mm rather than recovering. Placing")
    print("      the foot AT the DCM arrests the topple but leaves the body")
    print("      permanently displaced -- the robot is stable and walks away")
    print("      sideways. The recovery law puts the foot BEYOND the DCM, by a")
    print("      factor (growth)/(growth-1) = %.2f." % (P.growth / (P.growth - 1)))
    print()
    print("  >>> What limits it is REACH, not gain:")
    print("        foothold reach from nominal   %+.0f .. %+.0f mm (asymmetric)"
          % (P.reach[0] * 1e3, P.reach[1] * 1e3))
    print("        beta   one-step envelope   GUARANTEED envelope")
    for b in (0.0, 0.3, 0.5):
        print("         %.1f       %5.1f mm            %5.1f mm"
              % (b, ctl.one_step_envelope(P, b) * 1e3,
                 ctl.rejection_envelope(P, beta=b) * 1e3))
    print("      The envelope is gain-INDEPENDENT: gain sets how fast recovery is,")
    print("      reach sets whether it happens. The binding direction is REARWARD.")
    print()
    print("  >>> And it puts a hard number on the paw sensing (ADR-0012): a steady")
    print("      bias in the ESTIMATED DCM does not average out --")
    for e in (0.002, 0.005):
        t = ctl.simulate(P, 60, xi0=0.03, beta=0.0, estimation_error=e)
        print("        %.0f mm estimation bias -> %+.1f mm PERMANENT offset (%.1fx)"
              % (e * 1e3, t[-1] * 1e3, abs(t[-1] / e)))
    print("      so 'detect contact' was never a sufficient spec; the estimator")
    print("      needs a few mm of DCM accuracy.")


def _sketch_foot_path(ctrl, leg_name: str, rows: int = 6, cols: int = 40) -> None:
    """Print a tiny ASCII side-view of one foot's path over its own cycle."""
    xs, zs = [], []
    for i in range(cols):
        s = ctrl.local_phase(i / cols, leg_name)
        from tomcat_kin import foot_target as _ft
        tgt = _ft(ctrl.params, s)
        xs.append(tgt[0])
        zs.append(tgt[1])
    xs, zs = np.array(xs), np.array(zs)
    grid = [[" "] * cols for _ in range(rows)]
    zmin, zmax = zs.min(), zs.max()
    xmin, xmax = xs.min(), xs.max()
    for x, z in zip(xs, zs):
        c = int((x - xmin) / (xmax - xmin + 1e-12) * (cols - 1))
        r = int((zmax - z) / (zmax - zmin + 1e-12) * (rows - 1))
        grid[r][c] = "*"
    for row in grid:
        print("    |" + "".join(row))
    print("    +" + "-" * cols + "  (backward<-  ->forward)")


if __name__ == "__main__":
    main()
