# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""M42 stage 1 — the robot BUILT in simulation as a tendon drive. Gated.

`mjcf.py` puts a `<position>` servo on every joint, so the plant under every
balance result since M17 has been a **direct-drive** robot. `mjcf_tendon.py` is the
other thing, and these tests are the gate it had to pass before anything is
measured on it.

⚠️ Two of the gate's own failures are recorded here as tests, because they are
design findings rather than bugs: an open-loop tension allocation cannot hold a
pose, and an anchor placed where the cable already clears its sheave gets no
moment arm at all.
"""

from __future__ import annotations

import math
import re
import sys

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco", reason="mujoco is an optional dependency")

from tomcat_kin import LegModel  # noqa: E402
from tomcat_kin import mjcf_tendon as MT  # noqa: E402
from tomcat_kin import wbc  # noqa: E402
from tomcat_kin.params import DEFAULT_HINDLEG, DEFAULT_TENDON  # noqa: E402
from tomcat_kin.params import DEFAULT_FORELEG  # noqa: E402

TEN = ["L_hip_flex", "L_hip_ext", "L_knee_flex", "L_knee_ext", "L_ankle"]
JNT = ["L_q1", "L_q2", "L_q3"]


@pytest.fixture(scope="module")
def rig():
    m = mujoco.MjModel.from_xml_string(MT.single_leg_rig())
    tid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, n) for n in TEN}
    dof = {n: m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in JNT}
    q = LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0))
    return m, tid, dof, np.asarray(q, float)


def _at(m, dof, q):
    d = mujoco.MjData(m)
    for k, n in enumerate(JNT):
        d.qpos[dof[n]] = q[k]
    mujoco.mj_forward(m, d)
    return d


def _tendon_jacobian(m, tid, dof, q, h=0.002):
    """d(tendon length)/d(joint), m/rad, by central differences.

    ⚠️ `d.ten_J` is stored SPARSE (9 nonzeros for 5 tendons x 3 dofs), and
    `jacobian="dense"` does not change that for tendons in MuJoCo 3.10. The
    sparsity pattern — 1, 1, 2, 2, 3 — is itself the ADR-0042 result.
    """
    J = np.zeros((len(TEN), 3))
    for k, jn in enumerate(JNT):
        Ls = []
        for sgn in (+1, -1):
            qq = q.copy()
            qq[k] += sgn * h
            d = _at(m, dof, qq)
            Ls.append(np.array([d.ten_length[tid[n]] for n in TEN]))
        J[:, k] = (Ls[0] - Ls[1]) / (2.0 * h)
    return J


def test_the_model_is_actually_TENDON_driven(rig):
    """Five cable runs, five pull-only actuators, no joint servo anywhere."""
    m, _, _, _ = rig
    assert m.ntendon == 5, "hip pair + knee pair + single ankle"
    assert m.nu == 5
    for i in range(m.nu):
        assert m.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_TENDON, (
            "every actuator must drive a TENDON, not a joint"
        )


def test_the_MOMENT_ARMS_are_emergent_from_the_geometry(rig):
    """The point of building it: `TendonParams.joint_moment_arm` was a parameter
    fed to the analytical map, and the sim never saw a pulley. Here the arm is
    `d(length)/d(angle)` over a cylinder, and it comes out as the cylinder radius.

    Antagonists must come out with OPPOSITE sign and the same magnitude — that is
    what makes them a pair.
    """
    m, tid, dof, q = rig
    J = _tendon_jacobian(m, tid, dof, q) * 1e3
    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm) * 1e3

    # ⚠️ The signs here are set by the HINGE-AXIS convention, and they flipped when
    # it was corrected to `axis="0 -1 0"` in M43. What must hold regardless is that
    # each pair is opposite and equal; the absolute sign is a bookkeeping choice, so
    # the pair relation is asserted too rather than trusting the rows alone.
    assert J[0, 0] == pytest.approx(+arms[0], abs=0.05)     # hip flexor
    assert J[1, 0] == pytest.approx(-arms[0], abs=0.05)     # hip extensor
    assert J[2, 1] == pytest.approx(+arms[1], abs=0.30)     # knee flexor
    assert J[3, 1] == pytest.approx(-arms[1], abs=0.30)     # knee extensor
    # ⚠️ M44 moved the ankle ANCHOR from 45 to 300 deg, which reverses this sign.
    # A lone tendon can only pull, so the sign is not bookkeeping here -- it decides
    # which way the joint can be driven at all. See the reversal test below.
    assert J[4, 2] == pytest.approx(+arms[2], abs=0.40)     # ankle, single
    assert J[0, 0] == pytest.approx(-J[1, 0], abs=1e-9), "the hip pair opposes"
    assert J[2, 1] * J[3, 1] < 0, "the knee pair opposes"


def test_the_ADR0042_COUPLING_appears_BY_ITSELF(rig):
    """⚠️ THE result of building it. ADR-0042 derived the via-pulley coupling
    analytically as **±8.75 mm/rad** — exactly the pulley radius — and said the
    simulation could not show it because a joint servo has no pulley.

    It shows it now, from the geometry alone, to three decimal places:

        tendon           hip      knee     ankle
        hip_flex     +28.000     0.000     0.000
        hip_ext      -28.000     0.000     0.000
        knee_flex     -8.750   +25.097     0.000
        knee_ext      -8.750   -24.845     0.000
        ankle         -8.738    -8.750   +13.861

    An independent physics engine, from the routing, agreeing with a hand
    derivation. `TendonMap.cable_lengths` is still diagonal.

    ✅ **The coupling column survived a routing repair that changed everything
    else.** M43's hinge-axis correction flipped the diagonal signs, swapped which
    row the knee flexor and extensor occupy, and re-cut every cable length — and
    the coupling column stayed at **-8.750** to three decimals. That is what it
    means for a number to come from the pulley radius rather than from a routing
    accident: it is the one column the repair could not move.
    """
    m, tid, dof, q = rig
    J = _tendon_jacobian(m, tid, dof, q) * 1e3
    via = MT.VIA_R * 1e3

    # every distal tendon picks up the proximal joints at exactly the via radius
    for row, col in ((2, 0), (3, 0), (4, 0), (4, 1)):
        assert abs(J[row, col]) == pytest.approx(via, abs=0.05), (
            f"J[{row},{col}] = {J[row, col]:.3f}, expected +/-{via:.2f}"
        )
    # and the proximal-most tendons pick up nothing distal
    assert J[0, 1] == pytest.approx(0.0, abs=1e-6)
    assert J[0, 2] == pytest.approx(0.0, abs=1e-6)
    assert J[2, 2] == pytest.approx(0.0, abs=1e-6)


def test_a_cable_can_only_PULL_and_now_the_sim_knows_it(rig):
    """⚠️ "A cable can only pull" is a premise of ADR-0002 (why antagonistic pairs
    exist), ADR-0021 (why standing costs 76-87 % of moving for zero work) and
    ADR-0023 (why standing is the worst thermal case). **A `<position>` servo can
    push, so the simulation never had it.**

    `ctrlrange="0 T"` makes it physical: commanding -500 N applies +0.00 N.
    """
    m, _, dof, q = rig
    d = mujoco.MjData(m)
    for k, n in enumerate(JNT):
        d.qpos[dof[n]] = q[k]
    d.ctrl[:] = -500.0
    mujoco.mj_step(m, d)
    assert np.allclose(d.actuator_force, 0.0, atol=1e-9), (
        f"a pushed cable must apply nothing, got {d.actuator_force}"
    )
    # and a positive command does pull
    d.ctrl[:] = 100.0
    mujoco.mj_step(m, d)
    assert np.all(d.actuator_force > 0.0)


def test_the_tension_to_torque_map_is_minus_J_transpose(rig):
    """Measured rather than assumed, because the sign is what a first pass of this
    gate got wrong — twice, and it produced a false "co-contraction cancels"
    conclusion before it was caught."""
    m, tid, dof, q = rig
    J = _tendon_jacobian(m, tid, dof, q)
    for i in range(len(TEN)):
        d = mujoco.MjData(m)
        for k, n in enumerate(JNT):
            d.qpos[dof[n]] = q[k]
        d.ctrl[:] = 0.0
        d.ctrl[i] = 100.0
        mujoco.mj_forward(m, d)
        tau = np.array([d.qfrc_actuator[dof[n]] for n in JNT]) / 100.0
        assert tau == pytest.approx(-J[i], abs=2e-5), f"tendon {TEN[i]}"


# ===================================================================
# what the gate FAILED on, kept as findings
# ===================================================================

def _hold(m, dof, q, tb: float, kp: float, kd: float, seconds: float = 0.5,
          method: str = "nnls"):
    """Run the leg with a pull-only allocation and report the drift, in degrees.

    Two things this harness got wrong before M44, both now taken from `wbc`:

    - ⚠️ **the torque bookkeeping omitted `qfrc_passive`.** MuJoCo's own equation
      of motion makes the actuator term `qfrc_bias - qfrc_passive + stance`, and the
      ankle return spring alone is **0.508 N.m** of `qfrc_passive` at the hind
      stance pose -- 54 % of that joint's whole demand, asked of the tendon twice.
      `wbc.actuator_torque` does it now.
    - ⚠️ **it CLIPPED an unconstrained solve.** ADR-0047 priced that at about a
      degree; at M44's loads it **loses the leg entirely** (197° of hip drift
      against 0.00°). `wbc.tendon_tension` solves the non-negative problem, and
      `method="clip"` is kept only to keep measuring the gap.

    ⚠️ `G` is **MEASURED** from the model, not written down. It was hard-coded
    once, and that is exactly what let the hinge-axis error hide: when `axis` was
    corrected to `(0, -1, 0)` the hip pair's signs swapped, the frozen matrix kept
    commanding the wrong antagonist, and the leg collapsed **102°** while the
    routing itself was fine. A controller that measures its own plant survives a
    change to the plant; one that quotes a number from a previous milestone does not.
    """
    G = -_tendon_jacobian(
        m, {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, n)
            for n in TEN}, dof, q)
    d = mujoco.MjData(m)
    for k, n in enumerate(JNT):
        d.qpos[dof[n]] = q[k]
    idx = [dof[n] for n in JNT]
    for _ in range(int(seconds / m.opt.timestep)):
        e = np.array([q[k] - d.qpos[dof[n]] for k, n in enumerate(JNT)])
        ev = np.array([-d.qvel[dof[n]] for n in JNT])
        mujoco.mj_forward(m, d)
        tau_des = wbc.actuator_torque(d, idx, kp * e + kd * ev)
        if method == "nnls":
            d.ctrl[:] = wbc.tendon_tension(G.T, tau_des, t_min=tb,
                                           t_max=MT.TENSION_MAX)
        else:
            base = np.full(len(TEN), tb)
            dT = np.linalg.lstsq(G.T, tau_des - G.T @ base, rcond=None)[0]
            d.ctrl[:] = np.clip(base + dT, 0.0, MT.TENSION_MAX)
        mujoco.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)):
            return np.full(3, np.inf)
    return np.degrees(np.array([d.qpos[dof[n]] for n in JNT]) - q)


def test_gravity_FEEDFORWARD_alone_cannot_hold_the_pose(rig):
    """⚠️ Finding, not a bug. Allocating tension to cancel the *measured* gravity
    term every timestep is feedforward with no error feedback, and an inverted
    multi-link leg is an unstable equilibrium — so it diverges at any
    co-contraction level (0, 5 and 19.6 N all fall).

    That is why FR1 specifies *closed-loop* position control. The servo-based sim
    could not show it, because a position servo IS the loop.

    ⚠️ A first pass of this gate read the same failure as *"constant moment arms
    mean co-contraction adds no stiffness"* — a tidy explanation that was an
    artefact of two sign errors in my own allocation (`qfrc_actuator` must supply
    **+**`qfrc_bias`). The failure is real; that reading of it was not.
    """
    m, _, dof, q = rig
    for tb in (0.0, 5.0, 19.6):
        drift = _hold(m, dof, q, tb=tb, kp=0.0, kd=0.0)
        assert np.max(np.abs(drift)) > 5.0, (
            f"feedforward held at T_bias {tb} N (drift {drift.round(2)}) -- if that "
            "is now true, the drive gained stiffness and this should be re-derived"
        )


def test_an_outer_POSITION_LOOP_holds_it(rig):
    """✅ The gate passes here. `kp` 10 N.m/rad with `kd` 0.2, allocated onto
    pull-only tendons over a 5 N co-contraction floor, holds the stance.

    The ankle settles a little off because it has one tendon and a return spring
    rather than an antagonistic pair (ADR-0002 Option B) — the spring sets where
    "zero tension" sits, so a small offset is the design, not an error.

    ⚠️ **The ALLOCATOR matters, and CO-CONTRACTION buys it back — which is the
    note for the firmware.** This test clips an unconstrained least-squares solution
    at zero rather than solving the non-negative problem properly, to avoid a scipy
    dependency the project does not have. At a 5 N co-contraction floor that
    clipping costs **1.2-1.4°** on the hip and knee.

    Raise the floor to ADR-0021's standing tension of **19.6 N and the same clipped
    allocator holds both to 0.00°** — the base tension keeps the solution interior,
    so nothing clips at all. Clipping is only wrong when it is *reached*, and
    co-contraction is what keeps it out of reach. That is a second, previously
    unpriced reason to pay for co-contraction, alongside ADR-0002's.
    """
    m, _, dof, q = rig
    # ⚠️ 2 s, not the 0.5 s the other checks use: at 0.5 s the leg is still
    # settling. Asserting a settled number on an unsettled window is how M20/M30
    # got caught.
    proper = _hold(m, dof, q, tb=5.0, kp=10.0, kd=0.2, seconds=2.0,
                   method="nnls")
    assert np.abs(proper[0]) < 0.01, f"hip drift {proper[0]:.4f} deg"
    assert np.abs(proper[1]) < 0.01, f"knee drift {proper[1]:.4f} deg"

    # and clipping does not merely cost a degree here -- it loses the leg
    clipped = _hold(m, dof, q, tb=5.0, kp=10.0, kd=0.2, seconds=2.0,
                    method="clip")
    assert np.abs(clipped[0]) > 50.0, (
        f"clipped hip drift {clipped[0]:.1f} deg -- M44 measured 197"
    )

    # ⚠️ the ankle is the one joint that does NOT hold, and that is a design
    # finding, not a controller one: see the moment-arm reversal test.
    assert 5.0 < np.abs(proper[2]) < 25.0, (
        f"ankle drift {proper[2]:.2f} deg -- expected the ~15 deg ADR-0049 records"
    )


def test_an_anchor_that_does_not_force_a_WRAP_gets_no_moment_arm(rig):
    """⚠️ A real effect, but —**M43 RETRACTED the example M42 published for it.**

    M42 reported the ANKLE anchor landing in a dead spot at ~292° around its
    sheave, moment arm 2.6 mm instead of 14. That was measured on a leg pointing
    the wrong way: the hinge axis was `(0, 1, 0)`, so the leg folded UPWARD, and the
    dead spot was an artefact of the mirrored fold. With the axis corrected the
    ankle has **no dead spot at any anchor angle** — swept at 10° steps it reads
    13.69-13.94 mm all the way round, and the 2-D heuristic point that M42 called
    dead reads **13.86 mm**. The specific claim is withdrawn.

    ✅ **The general lesson survives, and the knee shows it far more sharply.**
    Swept around the knee sheave the flexor's moment arm collapses to **2.02 mm at
    270° — 8% of the specified 25 mm** — against 25.10 mm where it actually ships.
    A sheave the cable does not touch does no work, and nothing in the model
    complains: the tendon still routes, still pulls, still reports a length. Only
    differentiating it finds out.

    ⚠️ **And this is why the anchor sweep is a build step, not a one-off.** The
    dead band moved from one joint to another under a change of hinge convention.
    Any routing change has to re-run it.
    """
    m0, tid, dof, q = rig
    r_ank = float(DEFAULT_TENDON.joint_moment_arm[2])
    r_knee = float(DEFAULT_TENDON.joint_moment_arm[1])

    # the shipped anchors all wrap
    J = _tendon_jacobian(m0, tid, dof, q) * 1e3
    assert abs(J[4, 2]) > 0.9 * r_ank * 1e3, "the shipped ankle anchor must wrap"
    assert abs(J[2, 1]) > 0.9 * r_knee * 1e3, "the shipped knee anchor must wrap"

    # the retraction: M42's "dead" ankle placement in fact wraps fine
    revived = MT.single_leg_rig().replace(
        'name="L_ankle_anchor" pos="%.5f 0.012 %.5f"'
        % (1.15 * r_ank * math.cos(math.radians(45.0)),
           1.15 * r_ank * math.sin(math.radians(45.0))),
        'name="L_ankle_anchor" pos="%.5f 0.012 %.5f"'
        % (0.55 * r_ank, -(r_ank + 0.005)))
    mr = mujoco.MjModel.from_xml_string(revived)
    tr = {n: mujoco.mj_name2id(mr, mujoco.mjtObj.mjOBJ_TENDON, n) for n in TEN}
    dr = {n: mr.jnt_dofadr[mujoco.mj_name2id(mr, mujoco.mjtObj.mjOBJ_JOINT, n)]
          for n in JNT}
    revived_arm = _tendon_jacobian(mr, tr, dr, q)[4, 2] * 1e3
    assert abs(revived_arm) > 0.9 * r_ank * 1e3, (
        f"M42 called this placement dead; corrected it reads {revived_arm:.2f} mm"
    )

    # and the general lesson, on the knee, where a dead spot really is there
    a = math.radians(270.0)
    dead = MT.single_leg_rig().replace(
        'name="L_knee_anchor" pos="%.5f 0.012 %.5f"'
        % (0.55 * r_knee, -(r_knee + 0.005)),
        'name="L_knee_anchor" pos="%.5f 0.012 %.5f"'
        % (1.15 * r_knee * math.cos(a), 1.15 * r_knee * math.sin(a)))
    assert dead != MT.single_leg_rig(), "the knee anchor substitution must bite"
    md = mujoco.MjModel.from_xml_string(dead)
    td = {n: mujoco.mj_name2id(md, mujoco.mjtObj.mjOBJ_TENDON, n) for n in TEN}
    dd = {n: md.jnt_dofadr[mujoco.mj_name2id(md, mujoco.mjtObj.mjOBJ_JOINT, n)]
          for n in JNT}
    bad = _tendon_jacobian(md, td, dd, q)[2, 1] * 1e3
    assert abs(bad) < 0.2 * r_knee * 1e3, (
        f"the knee dead spot should lose its arm, got {bad:.2f} mm"
    )


# ===================================================================
# M42 stage 2 — cable elasticity, and G3 sized for the first time
# ===================================================================

def _joint_stiffness(series_k=None, h=1e-4, q=None):
    """Restoring joint stiffness from the tendon springs, N.m/rad.

    ⚠️ Central difference, deliberately. A cable's force does not reverse sign —
    it always pulls — so the ONE-SIDED magnitude is not a restoring stiffness.
    For an antagonistic pair the two cables pull opposite ways and the central
    difference is the real thing; for a lone tendon it correctly comes out near
    zero, which is the finding below.
    """
    if q is None:
        q = LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0))
    m = mujoco.MjModel.from_xml_string(
        MT.single_leg_rig_elastic(q_ref=q, series_k=series_k))
    dof = {n: m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in JNT}
    K = np.zeros(3)
    for k, jn in enumerate(JNT):
        taus = []
        for sgn in (+1, -1):
            d = mujoco.MjData(m)
            for kk, nn in enumerate(JNT):
                d.qpos[dof[nn]] = q[kk]
            d.qpos[dof[jn]] += sgn * h
            mujoco.mj_forward(m, d)
            taus.append(d.qfrc_passive[dof[jn]])
        K[k] = -(taus[0] - taus[1]) / (2.0 * h)
    return K


def test_the_cable_stiffness_is_PER_TENDON_not_one_constant():
    """LEG_TENDON_SPEC §2 says so explicitly — *"kinematics should compute it from
    the per-tendon path length, not a single constant"* — and §5.2's proposed
    `cable_stiffness = 3.5e5` is a single constant anyway (and was never folded in).

    At the routed lengths `EA/L` spans **4.9e5 to 2.0e6 N/m**, a factor of four.
    """
    q = LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0))
    m = mujoco.MjModel.from_xml_string(MT.single_leg_rig_elastic(q_ref=q))
    ks = np.array([m.tendon_stiffness[i] for i in range(m.ntendon)])
    assert ks.min() > 4e5 and ks.max() < 2.5e6
    assert ks.max() / ks.min() > 3.0, "a single constant cannot cover this"
    # and each is EA/L for its own run
    for i in range(m.ntendon):
        L = m.tendon_lengthspring[i][1]
        # rel 1e-5, not 1e-6: the XML writes stiffness at %.1f and MuJoCo stores
        # `tendon_lengthspring` in single precision, so the round-trip loses a few
        # parts per million. That is the serialisation, not the formula.
        assert m.tendon_stiffness[i] == pytest.approx(MT._cable_k(L), rel=1e-5)


def test_the_CABLE_IS_FAR_STIFFER_than_balance_can_tolerate():
    """⚠️ **THE M42 stage-2 finding.** ADR-0026 measured that balance needs
    *compliant* legs — servo `kp` 80-150 N.m/rad — and that **kp >= 250 winds up
    and falls**. In the servo sim that compliance was a gain. In a tendon drive it
    has to come from the cable, and the cable does not supply it:

    | joint | restoring stiffness | vs the kp = 250 that FELL |
    |---|---|---|
    | hip | **1304 N.m/rad** | **5.2x** |
    | knee | **638** | 2.6x |

    (M42 published 1269 and 560 here. M43's hinge-axis repair re-cut every routed
    run length, so `EA/L` moved with it; the conclusion did not.)

    So ADR-0026's *"balance needs compliant legs"* was a requirement on hardware
    that has never been turned into hardware. `kp = 80` was standing in for a
    compliance the machine does not have.
    """
    K = _joint_stiffness()
    assert K[0] > 4 * 250.0, f"hip stiffness {K[0]:.0f} N.m/rad"
    assert K[1] > 2 * 250.0, f"knee stiffness {K[1]:.0f} N.m/rad"
    assert K[0] > K[1], "the hip is stiffer -- its cable run is shorter"


def test_a_LONE_tendon_gives_the_joint_no_restoring_stiffness():
    """⚠️ And the ankle fails the other way, which is a note on ADR-0002 Option B.

    A cable always pulls the same direction, so a joint driven by ONE tendon has no
    restoring stiffness from it at all — perturb either way and the pull does not
    reverse. Measured: **53.9 N.m/rad** (M42 read 39.7 before the M43 routing
    repair), against the 0.3 N.m/rad the Option-B return spring contributes and the
    80 floor ADR-0026 wants — and against **1304** at the hip, which has a pair.

    Option B buys a motor per leg. What it costs is the joint's stiffness, and that
    had not been priced.
    """
    K = _joint_stiffness()
    assert K[2] < 80.0, f"ankle stiffness {K[2]:.1f} N.m/rad"
    assert K[2] < 0.1 * K[0], "an order below the antagonistic joints"
    assert float(DEFAULT_TENDON.spring_stiffness[2]) < 1.0, (
        "the Option-B return spring is 0.3 N.m/rad -- not a stiffness source"
    )


def test_the_antagonistic_pair_stiffness_is_DIRECTION_DEPENDENT():
    """`k = EA/L`, and a pair's two runs are not the same length: the hip flexor
    routes **0.073 m** and its extensor **0.121 m**, so the flexor is 1.7× stiffer.

    One-sided, the hip reads **1604 against 1003 N.m/rad** depending on which way it
    is pushed — a **1.60×** asymmetry, and **1.77×** at the knee (814 / 461).
    Equalising the run lengths is a routing choice nobody has had to make yet.

    (M42 published 1.72× and 2.21×, with the two hip runs the other way round. The
    hinge-axis repair swapped which member of each pair takes the short route, so
    the asymmetry moved; that it exists at all is the finding, and it is unchanged.)
    """
    q = LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0))
    m = mujoco.MjModel.from_xml_string(MT.single_leg_rig_elastic(q_ref=q))
    dof = {n: m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in JNT}
    h = 1e-4
    for jn, lo in (("L_q1", 1.4), ("L_q2", 1.6)):
        one = []
        for sgn in (+1, -1):
            d = mujoco.MjData(m)
            for kk, nn in enumerate(JNT):
                d.qpos[dof[nn]] = q[kk]
            d.qpos[dof[jn]] += sgn * h
            mujoco.mj_forward(m, d)
            one.append(abs(d.qfrc_passive[dof[jn]]) / h)
        assert max(one) / min(one) > lo, (
            f"{jn} asymmetry {max(one) / min(one):.2f}x"
        )


def test_G3s_series_spring_SIZES_at_about_175_kN_per_m():
    """✅ Design goal **G3** — *"passive compliance / shock absorption at each
    joint"* — has been a goal since M1 with no number attached. This is the number.

    A series-elastic element in line with each cable combines as
    `1/k = 1/k_cable + 1/k_series`. Swept:

    | series k | hip | knee | both in 80-150? |
    |---|---|---|---|
    | cable only | 1295 | 629 | |
    | 3.0e5 | 211 | 159 | |
    | 2.5e5 | 181 | 139 | |
    | 2.0e5 | 150 | 116 | yes |
    | **1.75e5** | **133** | **104** | yes |
    | 1.5e5 | 116 | 92 | yes |
    | 1.25e5 | 98 | 79 | |
    | 1.0e5 | 80 | 65 | |

    (M44's ankle re-routing changed the ankle tendon's run length, and every
    stiffness moved a per cent or two with it. The band is now **150-200 kN/m**;
    175 is still the point value and still near its centre.)

    **~175 kN/m puts both the hip and the knee inside ADR-0026's 80-150 window.**
    That is a real spring to hand to mechanical, and it is the first time G3 has
    had a target.

    ✅ **Re-measured after M43's routing repair, and it held.** M42 sized this at
    175 kN/m off a leg whose hinge axis was wrong; every stiffness in the table
    moved, and 175 kN/m still lands in the window (128/91 then, 136/107 now). What
    the repair did add is the **range**: 1.0e5-2.5e5 kN/m all keep at least one
    joint in the window, and **1.25e5-1.75e5 keeps both**. A range is more useful to
    hand to mechanical than a point value, and it is what this test now asserts.

    ⚠️ It does nothing for the ankle (16.2 N.m/rad), which needs the opposite
    treatment — see the lone-tendon test above.
    """
    K = _joint_stiffness(series_k=1.75e5)
    assert 80.0 <= K[0] <= 150.0, f"hip {K[0]:.1f} N.m/rad"
    assert 80.0 <= K[1] <= 150.0, f"knee {K[1]:.1f} N.m/rad"
    # the whole usable band, which is what mechanical actually needs
    for sk in (1.5e5, 1.75e5, 2.0e5):
        Ks = _joint_stiffness(series_k=sk)
        assert 80.0 <= Ks[0] <= 150.0 and 80.0 <= Ks[1] <= 150.0, (
            f"{sk:.3g} N/m gives {Ks[0]:.0f}/{Ks[1]:.0f}"
        )
    # and it is bracketed on both sides, so the band is not an artefact
    assert _joint_stiffness(series_k=1.25e5)[1] < 80.0, "too soft below 1.5e5"
    assert _joint_stiffness(series_k=3e5)[0] > 150.0, "too stiff above 2e5"


# ===================================================================
# M43 — the WHOLE-BODY tendon plant, and where its stand gate stops
# ===================================================================

def test_the_VIA_SITE_Z_SIGN_is_set_by_the_hinge_convention():
    """⚠️ **The M43 finding, and it invalidated four published numbers.**

    M42 built the via-pulley sites at **-z**. That was chosen against a leg whose
    hinge axis was `(0, 1, 0)` — which made the whole leg fold **upward**, feet at
    z = +0.346 above a trunk at 0.176. Correcting the axis to `(0, -1, 0)` (the
    convention `mjcf.py` documents, and which `LegModel.forward` requires) put the
    feet on the floor and left the cable running past every via-pulley on the
    **wrong side**.

    Nothing raised an error. The tendons still routed, still pulled, still reported
    lengths. What they lost was their geometry:

    | | knee flexor arm | couplings |
    |---|---|---|
    | via sites at -z | **1.17 mm/rad** | 11.73, 36.40, 14.00, 41.54 |
    | via sites at +z | **25.10** | **8.75, 8.75, 8.74, 8.75** |

    ⚠️ **What this cost.** The repair re-cut every routed run length, so it moved
    ADR-0047's cable stiffnesses (1269/560 -> 1304/638 N.m/rad), its pair asymmetry
    (1.72/2.21× -> 1.60/1.77×), its lone-tendon ankle figure (39.7 -> 53.9), and it
    **retracted the dead-spot example entirely** (see the anchor test above). It
    also cut the whole-body drift from 98° to a 14.5° lean, which changed what
    M43 concludes. G3's ~175 kN/m survived.

    ✅ **What this did not cost: the coupling column.** It read -8.750 before the
    repair and -8.750 after. A number that comes from the pulley radius does not
    care which way the leg folds; a number that comes from a routing accident does.
    That asymmetry is the most useful thing this failure produced.
    """
    xml = MT.single_leg_rig()
    for site in ("L_hip_via_side", "L_femur_mid", "L_knee_via_side",
                 "L_tibia_mid"):
        line = [l for l in xml.split(chr(10)) if 'name="%s"' % site in l]
        assert len(line) == 1, site
        z = float(line[0].split('pos="')[1].split('"')[0].split()[2])
        assert z > 0.0, f"{site} must sit at +z, got {z:+.5f}"

    # and the -z build really does lose the geometry, so this is not a preference
    broken = xml
    for a, b in ((" 0.024 %.5f" % (MT.VIA_R + 0.02),
                  " 0.024 %.5f" % -(MT.VIA_R + 0.02)),
                 (" 0.024 %.5f" % MT.VIA_R, " 0.024 %.5f" % -MT.VIA_R)):
        broken = broken.replace(a, b)
    assert broken != xml, "the sign substitution must bite"

    q = np.asarray(LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0)), float)
    mb = mujoco.MjModel.from_xml_string(broken)
    tb = {n: mujoco.mj_name2id(mb, mujoco.mjtObj.mjOBJ_TENDON, n) for n in TEN}
    db = {n: mb.jnt_dofadr[mujoco.mj_name2id(mb, mujoco.mjtObj.mjOBJ_JOINT, n)]
          for n in JNT}
    Jb = _tendon_jacobian(mb, tb, db, q) * 1e3
    assert abs(Jb[2, 1]) < 5.0, (
        f"at -z the knee flexor should lose its arm, got {Jb[2, 1]:.2f} mm/rad"
    )
    assert abs(abs(Jb[4, 1]) - MT.VIA_R * 1e3) > 1.0, (
        "and at -z the couplings should stop being the pulley radius"
    )


QLEGS = ("LF", "RF", "LR", "RR")
TEN_PER_LEG = ("hip_flex", "hip_ext", "knee_flex", "knee_ext", "ankle")


def _quad_poses(foot_x=0.04, foot_z=-0.17):
    from tomcat_kin.params import DEFAULT_FORELEG
    lp = {"LF": DEFAULT_FORELEG, "RF": DEFAULT_FORELEG,
          "LR": DEFAULT_HINDLEG, "RR": DEFAULT_HINDLEG}
    return {nm: np.asarray(LegModel(lp[nm]).inverse((foot_x, foot_z, 0.0)), float)
            for nm in QLEGS}


def _qadr(m, nm):
    return [m.jnt_qposadr[mujoco.mj_name2id(
        m, mujoco.mjtObj.mjOBJ_JOINT, f"{nm}_q{i}")] for i in (1, 2, 3)]


def _dofs(m, nm):
    return [m.jnt_dofadr[mujoco.mj_name2id(
        m, mujoco.mjtObj.mjOBJ_JOINT, f"{nm}_q{i}")] for i in (1, 2, 3)]


def _acts(m, nm):
    return [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"m_{nm}_{t}")
            for t in TEN_PER_LEG]


@pytest.fixture(scope="module")
def quad():
    q = _quad_poses()
    m = mujoco.MjModel.from_xml_string(
        MT.quadruped_rig_elastic(q_ref=q, hip_height=0.176, series_k=1.75e5))
    return m, q


def test_the_whole_body_plant_is_twenty_pull_only_tendons(quad):
    """Four tendon-driven legs on a floating trunk: 18 DOF (6 free + 12 leg),
    **20 tendons, 20 actuators**, every one of them pull-only."""
    m, _ = quad
    assert m.nv == 18 and m.ntendon == 20 and m.nu == 20
    for i in range(m.nu):
        assert m.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_TENDON
        assert m.actuator_ctrlrange[i][0] == 0.0, "pull-only"


def test_the_compiled_mass_closes_against_params(quad):
    """⚠️ It did not at first: **4.532 kg against params' 4.3041**, and the 0.224 kg
    gap was exactly 4x the per-leg pulley geom masses. ADR-0041's manufacturing
    model already apportions every sheave and bearing into `link_mass`, so giving
    the geoms their own mass double-counts. The pulleys are massless now.

    The residual 4 g is the four paw-pad spheres, which `link_mass` does not carry.
    """
    from tomcat_kin.params import DEFAULT_BODY_MASS_KG

    m, _ = quad
    assert float(sum(m.body_mass)) == pytest.approx(DEFAULT_BODY_MASS_KG, abs=0.006)


def test_the_hinge_axis_convention_puts_the_feet_BELOW_the_trunk(quad):
    """⚠️ A convention `mjcf.py` documents and this module had to learn again.

    `LegModel.forward` builds the tip from cumulative angles with
    `x = l cos a, z = l sin a`, so a POSITIVE joint angle must rotate +x toward
    **+z**. MuJoCo's right-hand rule about +y does the opposite, so the axis has to
    be `(0, -1, 0)`.

    With `(0, 1, 0)` the whole leg pointed **upward** — the feet came out at
    z = +0.346 m, above a trunk at 0.176 — and the quadruped "stood" by sinking to
    the floor with its joints dutifully held. Printing the foot positions is what
    caught it.
    """
    m, q = quad
    d = mujoco.MjData(m)
    for nm in QLEGS:
        for k, a in enumerate(_qadr(m, nm)):
            d.qpos[a] = q[nm][k]
    mujoco.mj_forward(m, d)

    trunk_z = float(d.qpos[2])
    for nm in QLEGS:
        sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"{nm}_foot")
        fz = float(d.site_xpos[sid][2])
        assert fz < trunk_z, f"{nm} foot at z={fz:.4f} above a trunk at {trunk_z:.4f}"
        assert fz == pytest.approx(0.006, abs=0.002), "and it should touch the floor"


def test_the_tension_CEILING_is_the_MOTORS_not_the_CABLES():
    """⚠️ **The M43 finding, and it launched a robot.**

    The first pass set `TENSION_MAX = 700 N` from ADR-0046's 638 N land transient.
    That is a **structural** number — what the cable, pulley and bearing must
    survive when the *ground* hits the foot. It is not what the motor can pull.

    Given 700 N of authority on twenty tendons, a 0.1 s contact transient saturated
    every one of them and **threw the 4.3 kg quadruped off the floor**: z went
    0.176 -> 0.834 m with `ncon = 0`. Twenty times 700 N is 14 kN on a 42 N robot.

    The real ceiling is `tau_motor / r_spool`: **223 N** peak, **81 N** continuous.
    The structure carries 2.9x more than the actuator can ever apply, which is
    correct — the land transient arrives from the ground, not from the motor.
    """
    from tomcat_kin.params import DEFAULT_TENDON

    r = float(DEFAULT_TENDON.motor_spool_radius)
    assert MT.TENSION_MAX == pytest.approx(MT.MOTOR_PEAK_NM / r)
    assert MT.TENSION_MAX == pytest.approx(222.9, abs=1.0)
    assert MT.TENSION_CONTINUOUS == pytest.approx(81.1, abs=1.0)
    assert MT.TENSION_MAX < 0.4 * 638.0, (
        "the motor can apply well under half the structural design load"
    )


def _quad_hold(m, q, kp, kd, tb, seconds):
    """Per-leg joint PD, allocated onto pull-only tendons. Returns worst drift."""
    d = mujoco.MjData(m)
    for nm in QLEGS:
        for k, a in enumerate(_qadr(m, nm)):
            d.qpos[a] = q[nm][k]
    mujoco.mj_forward(m, d)

    G = {}
    for nm in QLEGS:
        tid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, f"{nm}_{t}")
               for t in TEN_PER_LEG]
        J = np.zeros((5, 3))
        for k in range(3):
            Ls = []
            for sgn in (+1, -1):
                dd = mujoco.MjData(m)
                dd.qpos[:] = d.qpos
                dd.qpos[_qadr(m, nm)[k]] = q[nm][k] + sgn * 0.002
                mujoco.mj_forward(m, dd)
                Ls.append(np.array([dd.ten_length[t] for t in tid]))
            J[:, k] = (Ls[0] - Ls[1]) / 0.004
        G[nm] = -J

    for _ in range(int(seconds / m.opt.timestep)):
        for nm in QLEGS:
            dof = _dofs(m, nm)
            e = np.array([q[nm][k] - d.qpos[_qadr(m, nm)[k]] for k in range(3)])
            ev = np.array([-d.qvel[dof[k]] for k in range(3)])
            b = np.array([d.qfrc_bias[dof[k]] for k in range(3)])
            base = np.full(5, tb)
            dT = np.linalg.lstsq(G[nm].T, b + kp * e + kd * ev - G[nm].T @ base,
                                 rcond=None)[0]
            T = np.clip(base + dT, 0.0, MT.TENSION_MAX)
            for i, a in enumerate(_acts(m, nm)):
                d.ctrl[a] = T[i]
        mujoco.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)):
            return {nm: float("inf") for nm in QLEGS}
    return {nm: float(np.max(np.abs(np.degrees(
        np.array([d.qpos[_qadr(m, nm)[k]] for k in range(3)]) - q[nm]))))
        for nm in QLEGS}


def test_the_legs_DO_hold_their_poses_on_a_welded_trunk(quad):
    """✅ The decisive experiment, and it isolates the failure below.

    Weld the trunk to the world and the same per-leg controller holds every leg:
    the **hind** legs to **0.37°** and the fore legs to **1.8-2.3°**. So neither the
    tendon routing nor the pull-only allocation is what fails on a floating base.

    (M42's routing defect made this read 3.4° hind and 14.5-19° fore. Both improved
    by roughly an order when the via-pulley sites were repaired, which is how much
    of the original fore/hind story was really a routing bug.)

    ⚠️ A fore/hind gap does survive, at ~5×: `DEFAULT_FORELEG` folds the OPPOSITE
    way (knee range 0...+150° against the hind's -150...0°), so the sidesites and
    anchor angles that force the right wrap on a hind leg are still not mirrored for
    a fore one. At 1.8° it is no longer what stops the robot standing.
    """
    m0, q = quad
    welded = MT.quadruped_rig_elastic(
        q_ref=q, hip_height=0.176, series_k=1.75e5
    ).replace('<freejoint name="root"/>', '')
    m = mujoco.MjModel.from_xml_string(welded)
    assert m.nv == 12, "the trunk is welded, so only the 12 leg DOF remain"

    drift = _quad_hold(m, q, kp=50.0, kd=1.0, tb=5.0, seconds=1.5)
    assert drift["LR"] < 1.0, f"hind-left drift {drift['LR']:.2f} deg"
    assert drift["RR"] < 1.0, f"hind-right drift {drift['RR']:.2f} deg"
    assert max(drift.values()) < 5.0, f"worst {max(drift.values()):.2f} deg"
    # and the fore legs really are still the worse pair
    assert max(drift["LF"], drift["RF"]) > 2 * max(drift["LR"], drift["RR"])


def _quad_stand(m, q, kp, kd, tb, seconds=2.0):
    """Run the floating quadruped and report what STANDING actually means:
    trunk height, trunk tilt, and how many feet stayed on the floor.

    ⚠️ Joint drift is not the gate. Four legs can each hold their own angles to a
    fraction of a degree while the robot leans over and lifts two feet, because
    nothing in a per-leg loop has an opinion about the trunk.
    """
    d = mujoco.MjData(m)
    for nm in QLEGS:
        for k, a in enumerate(_qadr(m, nm)):
            d.qpos[a] = q[nm][k]
    mujoco.mj_forward(m, d)

    G = {}
    for nm in QLEGS:
        tid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, f"{nm}_{t}")
               for t in TEN_PER_LEG]
        J = np.zeros((5, 3))
        for k in range(3):
            Ls = []
            for sgn in (+1, -1):
                dd = mujoco.MjData(m)
                dd.qpos[:] = d.qpos
                dd.qpos[_qadr(m, nm)[k]] = q[nm][k] + sgn * 0.002
                mujoco.mj_forward(m, dd)
                Ls.append(np.array([dd.ten_length[t] for t in tid]))
            J[:, k] = (Ls[0] - Ls[1]) / 0.004
        G[nm] = -J

    z0 = float(d.qpos[2])
    ncon = []
    for _ in range(int(seconds / m.opt.timestep)):
        for nm in QLEGS:
            dof = _dofs(m, nm)
            e = np.array([q[nm][k] - d.qpos[_qadr(m, nm)[k]] for k in range(3)])
            ev = np.array([-d.qvel[dof[k]] for k in range(3)])
            b = np.array([d.qfrc_bias[dof[k]] for k in range(3)])
            base = np.full(5, tb)
            dT = np.linalg.lstsq(G[nm].T, b + kp * e + kd * ev - G[nm].T @ base,
                                 rcond=None)[0]
            T = np.clip(base + dT, 0.0, MT.TENSION_MAX)
            for i, a in enumerate(_acts(m, nm)):
                d.ctrl[a] = T[i]
        mujoco.mj_step(m, d)
        ncon.append(int(d.ncon))
        if not np.all(np.isfinite(d.qpos)):
            return z0, 0.0, 180.0, 0
    tilt = float(np.degrees(np.arccos(np.clip(
        1.0 - 2.0 * (d.qpos[4] ** 2 + d.qpos[5] ** 2), -1.0, 1.0))))
    return z0, float(d.qpos[2]), tilt, min(ncon)


def test_a_per_leg_JOINT_controller_gets_CLOSE_but_not_LEVEL(quad):
    """⚠️ **M43 said this controller "cannot make it stand". M44 WITHDRAWS that.**

    M43 measured a 14.5° diagonal lean and concluded a per-leg joint controller
    was structurally incapable of standing. Two routing defects were inflating it,
    and both are M44 findings (see the tests below): the ankle anchor sat past its
    moment-arm **sign reversal**, and the return spring was referenced 97° from the
    stance hock. Corrected, the same controller gets to **2.4-2.8°**:

    | kp | T_bias | trunk z | tilt | min contacts |
    |---|---|---|---|---|
    | 25 | 5 | 0.184 | 9.1° | 1 |
    | 50 | 5 | 0.179 | 3.8° | 2 |
    | 100 | 19.6 | 0.178 | 2.8° | 2 |
    | 200 | 19.6 | 0.178 | **2.4°** | 2 |
    | 400 | 19.6 | 0.029 | **180°** | 0 |

    So the honest comparison is no longer "one falls and one does not". It is
    **2.4° against 0.006°** for the foot-force controller below — a 400×
    attitude improvement — and the joint controller still **inverts** at kp 400,
    which the foot-force one never does at any gain tried. That is a better-posed
    result than M43's, and it took retracting M43's to get it.
    """
    m, q = quad
    z0, z, tilt, ncon = _quad_stand(m, q, kp=200.0, kd=4.0, tb=19.6)
    assert z > 0.9 * z0, f"trunk sank to {z:.4f} from {z0:.4f}"
    assert 1.0 < tilt < 6.0, f"trunk tilt {tilt:.2f} deg -- M44 measured 2.4"

    # and it still inverts at a gain the foot-force controller tolerates
    _, _, tilt_bad, _ = _quad_stand(m, q, kp=400.0, kd=8.0, tb=19.6)
    assert tilt_bad > 90.0, (
        f"kp=400 should invert the joint controller, got {tilt_bad:.1f} deg"
    )


# ===================================================================
# M44 - driving the tendon plant with wbc.py's FOOT-FORCE allocation
# ===================================================================

def test_realisable_cop_had_no_INSIDE_test_for_a_support_POLYGON():
    """⚠️ **A defect in ADR-0038's own module, unexercised since M33.**

    `wbc.realisable_cop` clamps a commanded centre of pressure onto what the
    contacts can make. M33 only ever ran a **diagonal two-foot trot**, where the CoP
    really is confined to a line and the two-contact branch is exact. The
    three-or-more branch was written and never run, and it was wrong twice:

    1. **No inside test.** It walked the boundary and returned the nearest point on
       an edge, so a feasible CoP in the middle of a four-foot polygon was pushed
       **48 mm out to the rail**. For a standing robot that is not a clamp, it is a
       command to lean.
    2. **It assumed the caller's order was hull order.** `("LF","RF","LR","RR")`
       traverses a rectangle as a **bowtie**, so two of the four "edges" it measured
       against were diagonals. That partly masked the first bug — it moved the
       interior point 16.6 mm instead of 48.

    ⚠️ Fixing it did **not** make the robot stand: the lean was 14.5° before and
    14.5° after. Two more findings were needed. A real bug that was not the cause
    is still worth fixing, and saying so is the point of recording it this way.
    """
    feet = np.array([[0.145, 0.048], [0.145, -0.048],
                     [-0.065, 0.048], [-0.065, -0.048]])

    for inside in ([0.0, 0.0], [-0.0016, 0.0], [0.04, 0.0], [0.0, 0.02]):
        got = wbc.realisable_cop(feet, inside)
        assert np.allclose(got, inside, atol=1e-12), (
            f"{inside} is inside the polygon and must come back untouched, "
            f"got {got}"
        )

    # outside still clamps, and onto the real hull rather than a bowtie diagonal
    for outside, want in (([0.30, 0.0], [0.145, 0.0]),
                          ([0.0, 0.20], [0.0, 0.048]),
                          ([-0.20, -0.20], [-0.065, -0.048])):
        got = wbc.realisable_cop(feet, outside)
        assert np.allclose(got, want, atol=1e-9), f"{outside} -> {got}"

    # the hull is computed, not assumed: shuffled input gives the same answer
    for perm in ([2, 0, 3, 1], [3, 2, 1, 0], [1, 3, 0, 2]):
        assert np.allclose(wbc.realisable_cop(feet[perm], [0.0, 0.0]),
                           [0.0, 0.0], atol=1e-12)

    # collinear contacts degenerate to the segment, not to a zero-area polygon
    line = np.array([[0.0, 0.0], [0.1, 0.0], [0.2, 0.0]])
    assert np.allclose(wbc.realisable_cop(line, [0.05, 0.3]), [0.05, 0.0],
                       atol=1e-9)


def test_the_ANKLE_moment_arm_REVERSES_SIGN_inside_the_ROM():
    """⚠️ **The structural finding of M44, and a much sharper statement of what
    [ADR-0002](../docs/DESIGN_DECISIONS.md) Option B costs than ADR-0047's.**

    ADR-0047 found a lone-tendon joint has no restoring **stiffness**. M44 finds
    something stronger: **the one direction it can pull is not a fixed direction in
    joint space.** The ankle's moment arm changes sign partway through the ROM.

    Swept 12 anchor angles × the full -30...+150° ankle range, **every one of the
    12 reverses somewhere between 45° and 120°.** It has to: as the metatarsus
    sweeps 180° the anchor sweeps 180° around the sheave, so the incoming cable
    line must cross the sheave centre exactly once. No anchor angle avoids it.

    ⚠️ **And the hind stance pose was on the wrong side of it.** The hind hock holds
    **+97.1°** in stance; at the M42/M43 anchor of 45° the reversal sat at ~85°.
    So the hind ankle could not supply standing torque **at any tension** — the
    non-negative allocation left a **0.714 N·m residual**, which is infeasibility,
    not a solver miss. Moving the anchor to 300° pushes the reversal past 105°
    and makes all four legs feasible at residual **0.000000**.

    (The fore hock holds +16.4°, comfortably inside. The two legs behaved
    completely differently under load for this reason and no other.)
    """
    q = np.asarray(LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0)), float)
    r = float(DEFAULT_TENDON.joint_moment_arm[2])
    base = MT.single_leg_rig()

    def arm_at(anchor_deg, q3_deg):
        a = math.radians(anchor_deg)
        xml = re.sub(
            r'(name="L_ankle_anchor" pos=")[-0-9.]+( 0.012 )[-0-9.]+(")',
            lambda mo: "%s%.5f%s%.5f%s" % (mo.group(1), 1.15 * r * math.cos(a),
                                           mo.group(2), 1.15 * r * math.sin(a),
                                           mo.group(3)),
            base)
        mm = mujoco.MjModel.from_xml_string(xml)
        tt = {n: mujoco.mj_name2id(mm, mujoco.mjtObj.mjOBJ_TENDON, n)
              for n in TEN}
        dd = {n: mm.jnt_dofadr[mujoco.mj_name2id(mm, mujoco.mjtObj.mjOBJ_JOINT,
                                                 n)] for n in JNT}
        qq = q.copy()
        qq[2] = math.radians(q3_deg)
        return _tendon_jacobian(mm, tt, dd, qq)[4, 2]

    rom = range(-30, 151, 15)
    for anchor in range(0, 360, 30):
        signs = {int(np.sign(arm_at(anchor, d))) for d in rom}
        assert len(signs) == 2, (
            f"anchor {anchor} deg should reverse somewhere in the ROM, "
            f"signs seen {signs}"
        )

    # and 300 deg is chosen because it puts the STANCE pose on the usable side
    stance = math.degrees(q[2])
    assert stance == pytest.approx(97.1, abs=0.5)
    assert arm_at(300.0, stance) > 0.0, "300 deg must plantarflex at the stance"
    assert arm_at(45.0, stance) < 0.0, "45 deg (M42/M43) dorsiflexes there"
    # both still wrap: this is about sign, not about losing the moment arm
    for anchor in (45.0, 300.0):
        assert abs(arm_at(anchor, stance)) > 0.9 * r, "the sheave must still wrap"


def test_a_LONE_tendon_cannot_serve_BOTH_stance_and_swing():
    """⚠️ **The consequence, and it is a decision for mechanical, not a bug.**

    The ankle needs **opposite** torques loaded and unloaded:

    - **loaded** (stance): the ground pushes the toe up, so the joint needs
      **plantarflexion**, -0.68 N·m hind and -0.79 fore, one sign over the whole
      stance sweep;
    - **unloaded** (swing): the ADR-0002 return spring is the only thing acting, and
      referenced at 0 it pulls the +97° hock **plantarflexing too** (-0.508 N·m),
      so the tendon must **dorsiflex** to hold the pose.

    The spring and the stance load pull the same way. A lone tendon has one
    direction, so it can serve one regime or the other:

    | anchor | unloaded ankle | quadruped |
    |---|---|---|
    | 45° (M42/M43) | **0.00°** | ⚠️ **inverts** |
    | 300° (M44) | -14.6° | ✅ **stands** |

    M44 ships 300°, because closing M43's stand gate is the milestone, and
    references the spring at each leg's own stance angle, which drops the worst
    tendon from the 222.9 N ceiling to 207.4 N. **The -14.6° unloaded ankle is the
    price, and ADR-0049 hands the choice back:** an antagonistic pair at the ankle
    (ADR-0002 **Option A**) costs four more motors and removes the conflict entirely.
    Option B was chosen on motor count; this is the second cost it never counted.
    """
    q = np.asarray(LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0)), float)
    m = mujoco.MjModel.from_xml_string(MT.single_leg_rig())
    dof = {n: m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in JNT}

    # the shipped build stands (see the gate below) and pays at the unloaded ankle
    drift = _hold(m, dof, q, tb=5.0, kp=10.0, kd=0.2, seconds=2.0)
    assert np.abs(drift[0]) < 0.01 and np.abs(drift[1]) < 0.01, (
        "hip and knee have antagonists, so they hold exactly"
    )
    assert np.abs(drift[2]) > 5.0, (
        f"the lone ankle does not, and that is the finding: {drift[2]:.2f} deg"
    )

    # the spring is referenced at each leg's own stance angle, and they differ
    fore = MT._stance_ankle(DEFAULT_FORELEG)
    hind = MT._stance_ankle(DEFAULT_HINDLEG)
    assert math.degrees(hind) == pytest.approx(97.1, abs=0.5)
    assert math.degrees(fore) == pytest.approx(16.4, abs=0.5)
    assert abs(math.degrees(hind - fore)) > 70.0, (
        "one springref cannot serve both -- 81 deg apart"
    )
    # and params still says 0, which is the number ADR-0049 hands to mechanical
    assert float(DEFAULT_TENDON.spring_rest_angle[2]) == 0.0
    assert MT.ANKLE_SPRINGREF == 0.0


def test_the_allocation_must_SOLVE_the_non_negative_problem():
    """⚠️ **Clipping escalated from "a degree" to "loses the leg".**

    ADR-0047 priced clipping an unconstrained allocation at ~1° of joint error at
    a co-contraction floor. At standing loads it is not a tolerance any more:

    | | single leg, unloaded | quadruped |
    |---|---|---|
    | clipped lstsq | **197° hip drift** | 20.7° lean, 1.14 N·m residual |
    | `wbc.tendon_tension` | **0.00°** | 0.006° lean, ~0 residual |

    So `wbc.nnls` exists: Lawson-Hanson, written out because the project has no
    scipy and firmware will not have one either.
    """
    # the textbook check: where the unconstrained answer is negative, nnls differs
    a = np.array([[1.0, 1.0], [1.0, -1.0]])
    b = np.array([1.0, 3.0])
    assert np.allclose(np.linalg.lstsq(a, b, rcond=None)[0], [2.0, -1.0])
    assert np.allclose(wbc.nnls(a, b), [2.0, 0.0])
    assert np.all(wbc.nnls(a, b) >= 0.0)

    # an antagonistic pair: the floor is respected and the torque is exact
    G = np.array([[0.028, -0.028]])
    T = wbc.tendon_tension(G, [0.28], t_min=5.0)
    assert np.all(T >= 5.0 - 1e-12)
    assert float((G @ T)[0]) == pytest.approx(0.28)
    assert T.min() == pytest.approx(5.0), "the slack antagonist sits at the floor"

    # and a torque of the wrong sign for a LONE tendon is reported, not faked
    lone = np.array([[0.014]])
    T = wbc.tendon_tension(lone, [-0.5], t_min=0.0)
    assert T[0] == pytest.approx(0.0)
    assert abs(float((lone @ T)[0]) - (-0.5)) == pytest.approx(0.5, abs=1e-9), (
        "the residual IS the finding -- see the moment-arm reversal test"
    )


def _wbc_stand(m, q, *, tb=19.6, mu=0.8, seconds=3.0, refresh=25,
               attitude=(40.0, 4.0), damp=6.0):
    """Drive the tendon quadruped with ADR-0038's chain, plus M44's missing link.

        desired_wrench -> allocate -> stance_torque -> tendon_tension -> ctrl

    ⚠️ The attitude term is M44's addition: `wbc.desired_wrench` returns **zero
    desired moment**, because M33's in-place trot never needed to regulate trunk
    attitude. Standing does, and without it the trunk keeps ~1° of residual lean.
    """
    d = mujoco.MjData(m)
    for nm in QLEGS:
        for k, a in enumerate(_qadr(m, nm)):
            d.qpos[a] = q[nm][k]
    mujoco.mj_forward(m, d)

    sid = {nm: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"{nm}_foot")
           for nm in QLEGS}
    dof = {nm: _dofs(m, nm) for nm in QLEGS}
    acts = {nm: _acts(m, nm) for nm in QLEGS}
    mass = float(sum(m.body_mass))
    h0 = float(d.subtree_com[0][2])
    omega = float(np.sqrt(9.81 / h0))

    def maps():
        out = {}
        for nm in QLEGS:
            tid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON,
                                     f"{nm}_{t}") for t in TEN_PER_LEG]
            J = np.zeros((5, 3))
            qa = _qadr(m, nm)
            base = [float(d.qpos[a]) for a in qa]
            for k in range(3):
                Ls = []
                for sgn in (+1, -1):
                    dd = mujoco.MjData(m)
                    dd.qpos[:] = d.qpos
                    dd.qpos[qa[k]] = base[k] + sgn * 0.002
                    mujoco.mj_forward(m, dd)
                    Ls.append(np.array([dd.ten_length[t] for t in tid]))
                J[:, k] = (Ls[0] - Ls[1]) / 0.004
            out[nm] = (-J).T
        return out

    G = maps()
    z0 = float(d.qpos[2])
    ncon, peak, resid = [], 0.0, 0.0
    tens = {nm: [] for nm in QLEGS}
    for it in range(int(seconds / m.opt.timestep)):
        if it and it % refresh == 0:
            G = maps()
        mujoco.mj_subtreeVel(m, d)
        com = np.array(d.subtree_com[0])
        vel = np.array(d.subtree_linvel[0])
        feet = np.array([d.site_xpos[sid[nm]] for nm in QLEGS])
        cop = wbc.realisable_cop(feet, com[:2])
        w = wbc.desired_wrench(mass, com, vel, cop, omega, damp=damp,
                               height=h0)
        if attitude is not None:
            kp_r, kd_r = attitude
            sign = 1.0 if float(d.qpos[3]) >= 0.0 else -1.0
            rot = 2.0 * sign * np.array([float(v) for v in d.qpos[4:7]])
            w[3:6] = -kp_r * rot - kd_r * np.array(d.qvel[3:6])
        f = wbc.allocate(feet, com, w, mu)
        st = wbc.stance_torque(mujoco, m, d, sid,
                              {nm: f[i] for i, nm in enumerate(QLEGS)}, dof)
        for nm in QLEGS:
            tau = wbc.actuator_torque(d, dof[nm], st[nm])
            T = wbc.tendon_tension(G[nm], tau, t_min=tb,
                                   t_max=MT.TENSION_MAX)
            resid = max(resid, float(np.linalg.norm(G[nm] @ T - tau)))
            peak = max(peak, float(T.max()))
            if it > int(0.5 / m.opt.timestep):
                tens[nm].append(T)
            for i, a in enumerate(acts[nm]):
                d.ctrl[a] = T[i]
        mujoco.mj_step(m, d)
        ncon.append(int(d.ncon))
        if not np.all(np.isfinite(d.qpos)):
            return dict(z0=z0, z=0.0, tilt=180.0, ncon=0, peak=peak,
                        resid=resid, tens=tens, diverged=True)
    tilt = float(np.degrees(np.arccos(np.clip(
        1.0 - 2.0 * (d.qpos[4] ** 2 + d.qpos[5] ** 2), -1.0, 1.0))))
    # ⚠️ ncon[0] is 0-2 before the solver has found the contacts; skip the first ms
    return dict(z0=z0, z=float(d.qpos[2]), tilt=tilt, ncon=min(ncon[10:]),
                peak=peak, resid=resid, tens=tens, diverged=False)


@pytest.fixture(scope="module")
def stood(quad):
    m, q = quad
    return _wbc_stand(m, q)


def test_the_tendon_quadruped_STANDS_on_FOOT_FORCE_allocation(stood):
    """✅ **M43's gate, closed. The pull-only tendon quadruped stands.**

    Twenty cables that can only pull, a floating trunk, and `wbc.py`'s foot-force
    allocation from [ADR-0038](../docs/DESIGN_DECISIONS.md) driving it:

    - trunk height **0.17600 → 0.17579 m** over 3 s — **0.21 mm**;
    - trunk tilt **0.006°**;
    - all four feet down throughout;
    - allocation residual ~0.02 N·m, i.e. the tensions really do produce the
      torques asked of them.

    ADR-0038 was built in M33 and had never been driven against anything but a
    position-servo plant. Its chain needed **one more link** — joint torque to
    non-negative tendon tension, `wbc.tendon_tension` — and **three fixes** in
    already-published code: the CoP polygon, the `qfrc_passive` bookkeeping, and the
    ankle's moment-arm reversal. Each has its own test above.
    """
    assert not stood["diverged"]
    assert stood["z"] > stood["z0"] - 0.001, (
        f"height held to {1e3 * (stood['z0'] - stood['z']):.2f} mm"
    )
    assert stood["tilt"] < 0.5, f"trunk tilt {stood['tilt']:.3f} deg"
    assert stood["ncon"] >= 4, f"kept only {stood['ncon']} contacts"
    assert stood["resid"] < 0.1, f"allocation residual {stood['resid']:.4f} N.m"


def test_G3s_series_spring_is_what_MAKES_it_stand(quad):
    """✅ **An independent confirmation of design goal G3, from a different
    direction entirely.**

    [ADR-0047](../docs/DESIGN_DECISIONS.md) sized G3's series-elastic element at
    ~175 kN/m from a **balance-compliance** argument: ADR-0026 measured that a
    balance controller falls at `kp >= 250` N·m/rad and the bare cable is 5× that.
    M44 arrives at the same element from **force control**, and the result is not
    subtle:

    | cable | outcome |
    |---|---|
    | **series-elastic, 175 kN/m** | ✅ **stands**, tilt 0.006° |
    | bare cable (5× stiffer) | ⚠️ **inverts**, tilt 180° |
    | no cable elasticity at all | leans 14.6° |

    Two independent arguments, two different failure modes, the same part. That is
    the strongest form this project has for a component nobody has bought yet.
    """
    _, q = quad
    stiff = mujoco.MjModel.from_xml_string(
        MT.quadruped_rig_elastic(q_ref=q, hip_height=0.176, series_k=None))
    r = _wbc_stand(stiff, q, seconds=2.0)
    assert r["diverged"] or r["tilt"] > 90.0, (
        f"the bare cable should invert the robot, got tilt {r['tilt']:.1f} deg"
    )


def test_standing_runs_the_hind_hip_extensor_OVER_its_continuous_rating(stood):
    """⚠️ **It stands, but not indefinitely — and ADR-0023's thermal case does not
    cover this.**

    Per-tendon tension while standing, against a motor rated **81 N continuous** and
    **223 N peak** (ADR-0048):

    | leg | worst tendon | mean | peak |
    |---|---|---|---|
    | fore | knee flexor | 74 N | 87 N |
    | **hind** | **hip extensor** | **~205 N** | **207 N** |
    | hind | knee flexor | 129 N | 138 N |

    The hind hip extensor runs **~2.5× the continuous rating** just to stand still,
    and the hind knee flexor 1.6×. ⚠️ [ADR-0023](../docs/DESIGN_DECISIONS.md) made
    standing the worst thermal case at the **nominal 19.6 N** co-contraction
    tension; this is an order above that on one tendon of twenty, and the thermal
    model has never been run on it.

    The fore legs are comfortable, which is the CoM sitting behind the middle: the
    hind feet carry 17.3 N against the fore pair's 10.4 and 3.8.
    """
    from tomcat_kin.params import DEFAULT_TENDON

    worst, worst_name = 0.0, ""
    for nm, rows in stood["tens"].items():
        a = np.array(rows)
        for i, t in enumerate(TEN_PER_LEG):
            rms = float(np.sqrt((a[:, i] ** 2).mean()))
            if rms > worst:
                worst, worst_name = rms, f"{nm}_{t}"
    assert worst > 1.5 * MT.TENSION_CONTINUOUS, (
        f"worst tendon {worst_name} at {worst:.1f} N against a "
        f"{MT.TENSION_CONTINUOUS:.1f} N continuous rating"
    )
    assert worst_name.endswith("hip_ext") and worst_name.startswith(("LR", "RR")), (
        f"expected a hind hip extensor to be the binding tendon, got {worst_name}"
    )
    # and it stays under the peak rating, so this is thermal and not a stall
    assert stood["peak"] <= MT.TENSION_MAX + 1e-9
    # the spring reference is what keeps it off the ceiling
    assert stood["peak"] < MT.TENSION_MAX - 5.0, (
        "referencing the ankle spring at the stance angle drops the worst tendon "
        "from the 222.9 N ceiling to 207.4 N"
    )
