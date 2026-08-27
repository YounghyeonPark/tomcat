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


# ===================================================================
# M45 - ADR-0002 Option A vs B at the ankle, measured rather than argued
# ===================================================================

def _ankle_hold(pair=False, k3=None, tb=5.0, kp=10.0, kd=0.2, seconds=2.0):
    """Hold the UNLOADED hind leg and report the drift and the worst tension.

    Unloaded is the regime that separates the options: ADR-0049 showed the spring
    and the stance load pull the same way, so a lone tendon can serve one or the
    other. This measures the swing side.
    """
    q = np.asarray(LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0)), float)
    m = mujoco.MjModel.from_xml_string(MT.single_leg_rig_elastic(
        q_ref=q, series_k=1.75e5, ankle_pair=pair, ankle_spring=k3))
    names = ["L_hip_flex", "L_hip_ext", "L_knee_flex", "L_knee_ext", "L_ankle"]
    if pair:
        names.append("L_ankle_ext")
    tid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, n) for n in names]
    dof = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in JNT]
    J = np.zeros((len(names), 3))
    for k in range(3):
        Ls = []
        for sgn in (+1, -1):
            d = mujoco.MjData(m)
            for i, a in enumerate(dof):
                d.qpos[a] = q[i]
            d.qpos[dof[k]] += sgn * 0.002
            mujoco.mj_forward(m, d)
            Ls.append(np.array([d.ten_length[t] for t in tid]))
        J[:, k] = (Ls[0] - Ls[1]) / 0.004
    G = (-J).T

    d = mujoco.MjData(m)
    for i, a in enumerate(dof):
        d.qpos[a] = q[i]
    peak = 0.0
    for _ in range(int(seconds / m.opt.timestep)):
        mujoco.mj_forward(m, d)
        e = np.array([q[i] - d.qpos[a] for i, a in enumerate(dof)])
        ev = np.array([-d.qvel[a] for a in dof])
        T = wbc.tendon_tension(G, wbc.actuator_torque(d, dof, kp * e + kd * ev),
                               t_min=tb, t_max=MT.TENSION_MAX)
        peak = max(peak, float(T.max()))
        d.ctrl[:] = T
        mujoco.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)):
            return None, peak
    return np.degrees(np.array([d.qpos[a] for a in dof]) - q), peak


def test_option_A_needs_a_CAPSTAN_not_a_mirrored_pair():
    """⚠️ **"Add an antagonist" is not sufficient, and M45 nearly shipped the
    version that does not work.**

    The hip and knee build their pairs by MIRRORING. Swept across the ankle's whole
    -30...+150° range, the three candidate antagonist anchors behave completely
    differently:

    | antagonist anchor | spans both directions? | worst arm error |
    |---|---|---|
    | 120° (the naive 180°-away mirror) | ⚠️ **no** — a 60° dead band, and the
    stance hock at 97.1° is inside it | |
    | 60° (reflected across z, as the hip and knee do it) | yes | ⚠️ **13.83 mm**
    on a 14 mm arm — one member all but vanishes |
    | **300°, i.e. the SAME anchor as the primary** | ✅ **yes** | ✅ **1.43 mm** |

    At 120° the two arms reverse at **different** angles (~105° and ~45°), which is
    what leaves the same-sign band. Anchoring both cables at the same point — a
    **capstan**, physically one cable round a pin with a motor on each end — makes
    the two wraps exact mirrors of each other, so they **reverse together and stay
    opposite** all the way through the ROM.

    The hip and knee get away with mirroring because their cable arrives from a
    distant spool, so the geometry really is symmetric about z. The ankle's arrives
    from a via-pulley on the tibia and is not.
    """
    q = np.asarray(LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0)), float)
    r = float(DEFAULT_TENDON.joint_moment_arm[2])
    base = MT.single_leg_rig(ankle_pair=True)

    def arms(anchor_x_deg, q3_deg):
        a = math.radians(anchor_x_deg)
        xml = re.sub(
            r'(name="L_ankle_anchor_x"\s+pos=")[-0-9.]+( 0.012 )[-0-9.]+(")',
            lambda mo: "%s%.5f%s%.5f%s" % (mo.group(1), 1.15 * r * math.cos(a),
                                           mo.group(2), 1.15 * r * math.sin(a),
                                           mo.group(3)),
            base, flags=re.S)
        mm = mujoco.MjModel.from_xml_string(xml)
        tt = {n: mujoco.mj_name2id(mm, mujoco.mjtObj.mjOBJ_TENDON, n)
              for n in ("L_ankle", "L_ankle_ext")}
        dd = [mm.jnt_dofadr[mujoco.mj_name2id(mm, mujoco.mjtObj.mjOBJ_JOINT, n)]
              for n in JNT]
        out = {}
        for n, t in tt.items():
            Ls = []
            for sgn in (+1, -1):
                d = mujoco.MjData(mm)
                for i, a2 in enumerate(dd):
                    d.qpos[a2] = q[i]
                d.qpos[dd[2]] = math.radians(q3_deg) + sgn * 0.002
                mujoco.mj_forward(mm, d)
                Ls.append(float(d.ten_length[t]))
            out[n] = (Ls[0] - Ls[1]) / 0.004
        return out

    rom = range(-30, 151, 10)

    def worst(anchor_x):
        band, err = [], 0.0
        for dq in rom:
            a = arms(anchor_x, dq)
            if a["L_ankle"] * a["L_ankle_ext"] > 0:
                band.append(dq)
            err = max(err, max(abs(abs(v) - r) for v in a.values()))
        return band, err

    # the naive 180-deg-away mirror leaves a dead band, and the stance is in it
    band, _ = worst(120.0)
    assert band, "anchor_x 120 deg was expected to have a same-sign band"
    assert 90 in band and 100 in band, (
        f"and it should contain the stance hock at 97.1 deg; band {band}"
    )

    # the z-mirror spans the ROM, but one member of the pair all but vanishes
    band, err = worst(60.0)
    assert not band, f"anchor_x 60 deg should span the ROM; band {band}"
    assert err > 0.010, (
        f"but its worst arm error should be large, got {1e3 * err:.2f} mm"
    )

    # the shipped capstan spans it AND keeps both arms near the spec
    band, err = worst(300.0)
    assert not band, f"the capstan must span the whole ROM; band {band}"
    assert err < 0.0015, f"worst capstan arm error {1e3 * err:.2f} mm"
    assert err < 0.15 * 0.010, "an order better than the z-mirror"


def test_option_B_sags_and_option_A_holds_the_UNLOADED_leg():
    """⚠️ **The regime that separates them, from ADR-0049's finding.**

    | | unloaded ankle | worst tension |
    |---|---|---|
    | Option B, `params`' 0.3 N·m/rad spring | **-14.6°** | 58.6 N |
    | Option A, capstan pair | **0.00°** | **12.2 N** |

    Option A holds it exactly and needs **4.8× less tension to do it**, because the
    antagonist opposes directly instead of the tendon fighting a spring.

    ✅ Both options hold the hip and knee to 0.00° either way — those joints
    have had antagonists all along, which is the point.
    """
    b_drift, b_peak = _ankle_hold(pair=False)
    a_drift, a_peak = _ankle_hold(pair=True)
    assert b_drift is not None and a_drift is not None

    for d in (b_drift, a_drift):
        assert abs(d[0]) < 0.01 and abs(d[1]) < 0.01, "hip and knee hold either way"

    assert abs(b_drift[2]) > 10.0, f"Option B ankle sag {b_drift[2]:.2f} deg"
    assert abs(a_drift[2]) < 0.5, f"Option A ankle drift {a_drift[2]:.2f} deg"
    assert a_peak < 0.3 * b_peak, (
        f"Option A should need far less tension: {a_peak:.1f} vs {b_peak:.1f} N"
    )


def test_a_STIFFER_SPRING_buys_the_same_holding_with_NO_extra_motors():
    """✅ **The cheap alternative, and it is why M45 does not simply recommend
    Option A.**

    `params` specifies the Option-B return spring at **0.3 N·m/rad**. Stiffened,
    and referenced at the stance hock (which ADR-0049 fixed), it holds:

    | k3 (N·m/rad) | 0.3 | 1.0 | 2.0 | 4.0 | **8.0** | 16.0 |
    |---|---|---|---|---|---|---|
    | ankle drift | -14.62° | -4.55 | -2.29 | -1.15 | **-0.58** | -0.29 |
    | peak tension | 58.6 N | 38.7 | 27.3 | 17.7 | **13.3** | 11.3 |

    **~8 N·m/rad reaches Option A's holding performance without a single extra
    motor** — a different spring, not four motors, four spools, four drivers and
    **+528 g on a 4.30 kg robot**.

    ⚠️ **Its cost is TRAVEL.** See the next test.
    """
    _, base_peak = _ankle_hold(k3=float(DEFAULT_TENDON.spring_stiffness[2]))
    drift, peak = _ankle_hold(k3=MT.ANKLE_SPRING_TO_HOLD)
    assert drift is not None
    assert abs(drift[2]) < 1.0, f"at 8 N.m/rad the ankle holds: {drift[2]:.2f} deg"
    assert peak < 0.3 * base_peak, (
        f"and needs far less tension: {peak:.1f} vs {base_peak:.1f} N"
    )
    # monotone in the spring, so 8 is a knee and not a lucky point
    prev = None
    for k3 in (1.0, 2.0, 4.0, 8.0, 16.0):
        d, _ = _ankle_hold(k3=k3)
        assert d is not None
        if prev is not None:
            assert abs(d[2]) < abs(prev) + 1e-9, "stiffer must not be worse"
        prev = d[2]


def test_the_stiffer_spring_EATS_THE_RANGE_OF_MOTION():
    """⚠️ **What the cheap option costs, and it is severe.**

    Driving the lone ankle tendon at the 223 N motor peak, how far the joint travels
    from its stance pose:

    | k3 (N·m/rad) | 0.3 | 1.0 | 2.0 | 4.0 | **8.0** | 16.0 |
    |---|---|---|---|---|---|---|
    | travel | -196° | -166 | -103 | -55 | **-25** | -12 |

    So the spring that holds the unloaded ankle also **takes the ankle's range of
    motion with it**: at 8 N·m/rad the tendon can reach 25° of the specified
    180°. That is the trade ADR-0050 hands to mechanical, and it is why this is a
    decision rather than a fix.
    """
    q = np.asarray(LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0)), float)

    def travel(k3, seconds=1.0):
        m = mujoco.MjModel.from_xml_string(MT.single_leg_rig_elastic(
            q_ref=q, series_k=1.75e5, ankle_spring=k3))
        dof = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
               for n in JNT]
        aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "m_L_ankle")
        d = mujoco.MjData(m)
        for i, a in enumerate(dof):
            d.qpos[a] = q[i]
        for _ in range(int(seconds / m.opt.timestep)):
            d.ctrl[:] = 0.0
            d.ctrl[aid] = MT.TENSION_MAX
            d.qpos[dof[0]] = q[0]
            d.qpos[dof[1]] = q[1]
            d.qvel[dof[0]] = 0.0
            d.qvel[dof[1]] = 0.0
            mujoco.mj_step(m, d)
        return abs(math.degrees(float(d.qpos[dof[2]]) - q[2]))

    loose = travel(float(DEFAULT_TENDON.spring_stiffness[2]))
    holding = travel(MT.ANKLE_SPRING_TO_HOLD)
    assert loose > 150.0, f"the specified spring barely restricts travel: {loose:.0f}"
    assert holding < 40.0, f"the holding spring restricts it hard: {holding:.0f}"
    assert holding < 0.25 * loose, "and the loss is most of the range"


def test_option_As_TRAVEL_cannot_be_measured_on_this_PLANT():
    """⚠️ **The limitation that stops M45 from settling the decision, and it is a
    property of the model, not of Option A.**

    `mjcf_tendon.py` has **no spool degree of freedom**: a tendon's length is purely
    a function of the joint angles, so a motor cannot **pay cable out**. A slack
    antagonist therefore acts as a spring.

    Driving one ankle tendon at 223 N with its antagonist commanded to **zero**, the
    antagonist stretched **2.13 mm and developed 273.8 N** — more than the 222.9 N
    driving it — and the joint stopped at 8.9°. A real motor would have released
    cable.

    ✅ **Nothing measured on this plant so far is affected**, because moment arms,
    joint stiffness and pose-holding are all small perturbations about a pose where
    both cables are taut. ⚠️ But **the travel of an antagonistic pair cannot be
    measured here, and travel is exactly what Option A is meant to buy over a
    stiffer spring.** Adding spool DOFs is the prerequisite for settling ADR-0002.
    """
    q = np.asarray(LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0)), float)
    m = mujoco.MjModel.from_xml_string(MT.single_leg_rig_elastic(
        q_ref=q, series_k=1.75e5, ankle_pair=True))
    dof = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in JNT]
    tex = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, "L_ankle_ext")
    aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "m_L_ankle")

    d = mujoco.MjData(m)
    for i, a in enumerate(dof):
        d.qpos[a] = q[i]
    mujoco.mj_forward(m, d)
    L0 = float(d.ten_length[tex])
    for _ in range(int(1.0 / m.opt.timestep)):
        d.ctrl[:] = 0.0
        d.ctrl[aid] = MT.TENSION_MAX
        d.qpos[dof[0]] = q[0]
        d.qpos[dof[1]] = q[1]
        d.qvel[dof[0]] = 0.0
        d.qvel[dof[1]] = 0.0
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)

    stretch = float(d.ten_length[tex]) - L0
    resist = stretch * float(m.tendon_stiffness[tex])
    travel = abs(math.degrees(float(d.qpos[dof[2]]) - q[2]))

    assert MT.NO_SPOOL_DOF, "the limitation is declared in the module"
    assert stretch > 1e-3, f"the slack antagonist stretched {1e3 * stretch:.2f} mm"
    assert resist > MT.TENSION_MAX, (
        f"and resisted with {resist:.0f} N against the {MT.TENSION_MAX:.0f} N "
        "driving it -- which is why the travel figure means nothing here"
    )
    assert travel < 15.0, f"so the joint stalls at {travel:.1f} deg"


def test_option_As_MASS_cost_is_four_more_MOTORS():
    """⚠️ **The price, against a mass budget that has already moved three times.**

    Option A is one more motor per leg: **4 × 132 g = +528 g** on ADR-0046's
    4.3041 kg, so **4.83 kg (+12.3 %)**, before spools, cables and drivers. NFR5's
    history is 3.0 → 4.05 (ADR-0010, the real motor) → 4.31 (ADR-0043, real joint
    hardware) → **4.83**. A domestic cat is 4-5 kg, so it is inside the band, at
    the top of it. Actuator count goes **19 → 23**.

    ⚠️ The model's own mass does not move, because the motors live in `trunk_mass`
    and the sheaves are massless by ADR-0048 — so this cost is **not** visible in
    the compiled plant and has to be carried in the budget by hand. That is worth
    asserting, so nobody reads the unchanged 4.3081 kg as Option A being free.
    """
    from tomcat_kin.params import DEFAULT_BODY_MASS_KG

    q = _quad_poses()
    a = mujoco.MjModel.from_xml_string(MT.quadruped_rig(hip_height=0.176,
                                                        ankle_pair=True))
    b = mujoco.MjModel.from_xml_string(MT.quadruped_rig(hip_height=0.176))
    assert a.nu == b.nu + 4 == 24, "one more actuator per leg"
    assert float(sum(a.body_mass)) == pytest.approx(float(sum(b.body_mass))), (
        "and the plant's mass does NOT show it -- the budget must, by hand"
    )

    motor_g = 132.0
    with_option_a = DEFAULT_BODY_MASS_KG + 4 * motor_g * 1e-3
    assert with_option_a == pytest.approx(4.832, abs=0.001)
    assert with_option_a / DEFAULT_BODY_MASS_KG == pytest.approx(1.123, abs=0.002)
    assert with_option_a < 5.0, "still inside the 4-5 kg band a real cat occupies"
    assert q is not None


def _trot_ankle_demand():
    """Ankle demand over one trot cycle, split by phase and by direction.

    Returns {leg: {"ref", "swing_below", "swing_above", "stance_above"}}, degrees.
    """
    from tomcat_kin import gait
    from tomcat_kin.params import DEFAULT_FORELEG as FL

    lp = {"LF": FL, "RF": FL, "LR": DEFAULT_HINDLEG, "RR": DEFAULT_HINDLEG}
    c = gait.GaitController(gait.trot_params())
    acc = {}
    for st in c.sample_cycle(400):
        for nm, ls in st.legs.items():
            if ls.q is None:
                continue
            acc.setdefault(nm, []).append(
                (math.degrees(ls.q[2]), bool(ls.in_stance)))
    out = {}
    for nm, v in acc.items():
        ref = math.degrees(LegModel(lp[nm]).inverse((0.04, -0.17, 0.0))[2])
        sw = np.array([q for q, ins in v if not ins])
        sta = np.array([q for q, ins in v if ins])
        out[nm] = {"ref": ref,
                   "swing_below": ref - float(sw.min()),
                   "swing_above": float(sw.max()) - ref,
                   "stance_above": float(sta.max()) - ref}
    return out


def test_the_TROT_SETTLES_ADR0002_in_favour_of_OPTION_A():
    """⚠️ **The measurement that settles [ADR-0002](../docs/DESIGN_DECISIONS.md),
    and it is not the one M45 expected to find.**

    ADR-0049 left Option A (antagonistic ankle) against Option B (one tendon plus a
    return spring) as a live decision. Most of M45's evidence favoured **B'**, a
    stiffer spring: at 8 N·m/rad it holds the unloaded ankle to **-0.58°** with
    **no extra motors**, and the tendon still has **2.9×** the travel it needs in
    its own pull direction (24.9° available against 8.6° demanded).

    ⚠️ **Then the gait was asked what it wants, and Option B cannot do it.** Over one
    trot cycle the ankle is commanded, relative to the stance hock:

    | leg | swing, below | swing, ABOVE | stance, above |
    |---|---|---|---|
    | fore | 13.4° | **+62.4°** | +54.8° |
    | hind | 8.6° | **+25.2°** | +14.1° |

    Under Option B **nothing drives the ankle above its reference**: the lone tendon
    pulls it *down* (that is the sign ADR-0049 had to choose to make standing
    possible at all) and the spring only pulls it *toward* the reference. The
    above-reference excursion during **stance** is plausibly the ground dorsiflexing
    a loaded foot, which the tendon merely resists — but **in SWING the foot is
    unloaded and there is nothing left to do it.**

    Moving the spring reference up to the swing extreme makes the trajectory
    reachable and gives back ADR-0049's stance saving (it is what dropped the worst
    tendon from 222.9 N to 207.4). So the reference is not a free parameter either.

    ✅ **Recommendation: Option A at the ankle, capstan construction.** The
    antagonist drives the ankle up directly. It costs four motors, **+528 g, 4.83 kg
    (+12.3 %)** and 19 → 23 actuators — and it is the only option measured here
    that can command the gait the project already publishes.

    ⚠️ Two caveats kept in view: the pair's **travel** cannot be measured on this
    plant (no spool DOF), and the standing-tension comparison is confounded by the
    missing posture task. Neither touches this argument, which is kinematic.
    """
    demand = _trot_ankle_demand()

    for nm, d in demand.items():
        # the tendon's own direction is comfortably served, even by a stiff spring
        assert d["swing_below"] < 20.0, (
            f"{nm} needs only {d['swing_below']:.1f} deg in the tendon's direction"
        )
        # but the other direction is where Option B has nothing at all
        assert d["swing_above"] > 20.0, (
            f"{nm} needs {d['swing_above']:.1f} deg ABOVE its reference in SWING, "
            "and Option B has no actuator that pulls that way"
        )
        assert d["swing_above"] > 2.0 * d["swing_below"], (
            f"{nm}: the unreachable direction is the larger one "
            f"({d['swing_above']:.1f} vs {d['swing_below']:.1f} deg)"
        )

    fore = demand["LF"]["swing_above"]
    hind = demand["LR"]["swing_above"]
    assert fore == pytest.approx(62.4, abs=1.0)
    assert hind == pytest.approx(25.2, abs=1.0)

    # and the stiffest spring that still clears the tendon-direction demand does
    # not help here at all, which is what makes this decisive rather than a tuning
    q = np.asarray(LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0)), float)
    m = mujoco.MjModel.from_xml_string(MT.single_leg_rig_elastic(
        q_ref=q, series_k=1.75e5, ankle_spring=MT.ANKLE_SPRING_TO_HOLD))
    tid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, "L_ankle")
    dof = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in JNT]
    Ls = []
    for sgn in (+1, -1):
        d = mujoco.MjData(m)
        for i, a in enumerate(dof):
            d.qpos[a] = q[i]
        d.qpos[dof[2]] += sgn * 0.002
        mujoco.mj_forward(m, d)
        Ls.append(float(d.ten_length[tid]))
    arm = (Ls[0] - Ls[1]) / 0.004
    assert arm > 0.0, (
        "the shipped ankle tendon shortens as q3 falls, i.e. it can only pull the "
        "joint DOWN -- so no spring stiffness makes the +62 deg excursion reachable"
    )

    # Option A's antagonist is what pulls the other way
    ma = mujoco.MjModel.from_xml_string(MT.single_leg_rig_elastic(
        q_ref=q, series_k=1.75e5, ankle_pair=True))
    ta = {n: mujoco.mj_name2id(ma, mujoco.mjtObj.mjOBJ_TENDON, n)
          for n in ("L_ankle", "L_ankle_ext")}
    da = [ma.jnt_dofadr[mujoco.mj_name2id(ma, mujoco.mjtObj.mjOBJ_JOINT, n)]
          for n in JNT]
    got = {}
    for n, t in ta.items():
        Ls = []
        for sgn in (+1, -1):
            d = mujoco.MjData(ma)
            for i, a in enumerate(da):
                d.qpos[a] = q[i]
            d.qpos[da[2]] += sgn * 0.002
            mujoco.mj_forward(ma, d)
            Ls.append(float(d.ten_length[t]))
        got[n] = (Ls[0] - Ls[1]) / 0.004
    assert got["L_ankle"] * got["L_ankle_ext"] < 0, (
        "and Option A's pair spans both directions, which is the whole case for it"
    )


# ===================================================================
# M46 - a SPOOL behind every cable, so a motor can pay out
# ===================================================================

SERIES_K = 1.5e5            # N/m, inside ADR-0050's 150-200 kN/m band
K_TORS = SERIES_K * MT.SPOOL_R ** 2


def _spooled(pin=(), q=None, ankle_pair=True):
    """The spooled hind leg, optionally with some leg joints pinned at `q`.

    ⚠️ Pinning by writing `qpos` every step is what M46 did first, and on a plant
    with a stiff constraint network that INJECTS ENERGY rather than holding a pose.
    Pinning by collapsing the joint's `range` is a real constraint, and the joints
    then hold to 0.0001°.
    """
    if q is None:
        q = np.asarray(LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0)),
                       float)
    xml = MT.single_leg_rig_spooled(q_ref=q, series_k=SERIES_K,
                                    ankle_pair=ankle_pair)
    for i, jn in enumerate(("L_q1", "L_q2", "L_q3")):
        if jn not in pin:
            continue
        xml = re.sub(
            r'(name="%s"[^>]*range=")[-0-9. ]+(")' % jn,
            lambda mo, v=q[i]: "%s%.6f %.6f%s" % (mo.group(1), v - 1e-6,
                                                  v + 1e-6, mo.group(2)),
            xml)
    return mujoco.MjModel.from_xml_string(xml), q


def _adr(m, kind, name):
    return mujoco.mj_name2id(m, kind, name)


def test_the_spool_plant_has_a_DRIVETRAIN_and_the_old_one_does_not():
    """✅ **What M46 built, and it is opt-in on purpose.**

    Every cable gains a rotor, a torsional series spring and a winding constraint:

        motor torque → rotor → spring → spool → cable

    The actuator moves from the tendon to the **rotor joint**, so it is commanded in
    **N·m** rather than newtons, and the plant grows 2 DOF per cable. The old
    plant is still the default, because every M42-M45 measurement was taken on it.
    """
    old = mujoco.MjModel.from_xml_string(MT.single_leg_rig())
    new, _ = _spooled()

    assert old.nv == 3 and old.neq == 0
    assert new.nv == 3 + 2 * 6, "a rotor and a spool DOF per cable"
    assert new.neq == 6, "one winding constraint per cable"
    assert new.ntendon == 12, "six routed paths, six wound lengths"

    assert old.actuator_trntype[0] == mujoco.mjtTrn.mjTRN_TENDON
    assert new.actuator_trntype[0] == mujoco.mjtTrn.mjTRN_JOINT
    assert new.actuator_ctrlrange[0][1] == pytest.approx(MT.MOTOR_PEAK_NM)

    # and it costs no mass: the motors are already in the girdle budget
    assert float(sum(new.body_mass)) == pytest.approx(float(sum(old.body_mass)))


def test_the_drivetrain_STATICS_are_exact():
    """✅ **The check that says the model is right: pin the leg and the series
    spring must carry exactly the motor's torque.**

    With every leg joint held, the spool cannot turn, so at equilibrium the spring
    deflection is `-tau / k_tors` and the cable tension is `tau / r_spool`. Measured
    across three cables and two torques it matches to five decimal places, and the
    tension lands on **222.9 N at the 1.95 N·m motor peak** -- the same ceiling
    ADR-0048 derived, now arrived at through a drivetrain instead of asserted.
    """
    m, q = _spooled(pin=("L_q1", "L_q2", "L_q3"))
    qa = [m.jnt_qposadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
          for n in ("L_q1", "L_q2", "L_q3")]

    for tname in ("L_ankle", "L_ankle_ext", "L_hip_flex"):
        aid = _adr(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "m_" + tname)
        js = m.jnt_qposadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, "js_" + tname)]
        for tau in (0.5, MT.MOTOR_PEAK_NM):
            d = mujoco.MjData(m)
            for i, a in enumerate(qa):
                d.qpos[a] = q[i]
            mujoco.mj_forward(m, d)
            for _ in range(60000):
                d.ctrl[:] = 0.0
                d.ctrl[aid] = tau
                mujoco.mj_step(m, d)
            mujoco.mj_forward(m, d)
            deflection = float(d.qpos[js])
            assert deflection == pytest.approx(-tau / K_TORS, abs=1e-5), tname
            tension = K_TORS * abs(deflection) / MT.SPOOL_R
            assert tension == pytest.approx(tau / MT.SPOOL_R, rel=1e-4)
    assert MT.MOTOR_PEAK_NM / MT.SPOOL_R == pytest.approx(222.9, abs=0.5)


def test_a_slack_antagonist_now_PAYS_OUT_instead_of_resisting():
    """✅ **The capability M46 exists for, and ADR-0050 could not have.**

    ADR-0050 could not measure an antagonistic pair's travel: with no spool DOF a
    tendon's length is a pure function of the joint angles, so a motor cannot
    release cable and a slack antagonist acts as a **spring** -- it stretched 2.13 mm
    and developed **273.8 N against the 222.9 N driving it**, stalling the ankle at
    **8.9°**.

    With a spool, the same antagonist **unwinds**: its rotor turns and the cable
    lengthens by exactly `r * theta`. The driven tendon then takes the ankle all the
    way to its **-30° end stop, 127° of travel**.
    """
    m, q = _spooled(pin=("L_q1", "L_q2"))
    qa = [m.jnt_qposadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
          for n in ("L_q1", "L_q2", "L_q3")]
    aid = _adr(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "m_L_ankle")
    jr = m.jnt_qposadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, "jr_L_ankle_ext")]
    tex = _adr(m, mujoco.mjtObj.mjOBJ_TENDON, "L_ankle_ext")
    j3 = _adr(m, mujoco.mjtObj.mjOBJ_JOINT, "L_q3")

    d = mujoco.MjData(m)
    for i, a in enumerate(qa):
        d.qpos[a] = q[i]
    mujoco.mj_forward(m, d)
    L0 = float(d.ten_length[tex])
    for _ in range(40000):
        d.ctrl[:] = 0.0
        d.ctrl[aid] = MT.MOTOR_PEAK_NM
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)

    paid = float(d.ten_length[tex]) - L0
    turns = float(d.qpos[jr])
    assert paid > 0.02, f"the antagonist must lengthen, got {1e3 * paid:.1f} mm"
    assert paid == pytest.approx(-turns * MT.SPOOL_R, abs=2e-3), (
        "and the length it gives up must be exactly what the rotor unwound"
    )
    # and the driven side reaches the end stop, which the old plant never could
    assert float(d.qpos[qa[2]]) == pytest.approx(m.jnt_range[j3][0], abs=1e-3)


def test_the_equality_REFERENCE_is_qpos0_and_that_is_a_trap():
    """⚠️ **The quiet one. MuJoCo references a tendon equality at `qpos0`, not at
    whatever state the caller sets.**

    Build the rig with the joints at zero, start it at the stance pose, and every
    winding constraint begins **24-52 mm out**. The solver then snaps the leg from
    97° to 39° in **5 ms** and holds the wrong configuration perfectly, with the
    residuals sitting **constant** -- which reads exactly like a satisfied constraint
    until you notice what they are constant *at*.

    `single_leg_rig_spooled` measures the offset in a first pass and puts it in
    `polycoef`'s `a0`. Then the residual at the stance pose is **1e-10**.
    """
    m, q = _spooled()
    qa = [m.jnt_qposadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
          for n in ("L_q1", "L_q2", "L_q3")]
    d = mujoco.MjData(m)
    for i, a in enumerate(qa):
        d.qpos[a] = q[i]
    mujoco.mj_forward(m, d)
    assert d.nefc >= 6, "the six winding constraints are active"
    assert float(np.max(np.abs(d.efc_pos[:6]))) < 1e-8, (
        f"residual {float(np.max(np.abs(d.efc_pos[:6]))):.2e} -- a0 is wrong"
    )

    # and without the offset it is centimetres out, not micrometres
    naive = mujoco.MjModel.from_xml_string(
        MT.single_leg_rig(spools=SERIES_K, ankle_pair=True))
    dn = mujoco.MjData(naive)
    for i, a in enumerate([naive.jnt_qposadr[_adr(naive,
                                                  mujoco.mjtObj.mjOBJ_JOINT, n)]
                           for n in ("L_q1", "L_q2", "L_q3")]):
        dn.qpos[a] = q[i]
    mujoco.mj_forward(naive, dn)
    assert float(np.max(np.abs(dn.efc_pos[:6]))) > 0.01, (
        "without a0 the constraints must start centimetres out"
    )


def test_a_stiff_equality_OVERPOWERS_a_default_joint_limit():
    """⚠️ **And it does not overshoot -- it corrupts the answer.**

    The winding equality has to be solved stiffly, or it becomes a spring in series
    with the drivetrain and silently softens it (measured: **0.717×** the specified
    stiffness at the loose setting, **0.998×** at the tight one). But solved stiffly
    it beats a default-stiffness joint limit: driving the ankle tendon at the motor
    peak sent `q3` to **215° against a 150° limit** and settled there -- 65°
    outside its own range and in the **wrong direction**. With the limits solved as
    stiffly as the equality it stops exactly on the -30° end stop.
    """
    m, q = _spooled(pin=("L_q1", "L_q2"))
    j3 = _adr(m, mujoco.mjtObj.mjOBJ_JOINT, "L_q3")
    # the shipped rig carries the matched limit solver in its <default>
    xml = MT.single_leg_rig_spooled(q_ref=q, series_k=SERIES_K, ankle_pair=True)
    assert 'solreflimit="%s"' % MT.EQ_SOLREF in xml
    assert 'solimplimit="%s"' % MT.EQ_SOLIMP in xml
    # and the plain rig does not need it, because it has no equality to lose to
    assert "solreflimit" not in MT.single_leg_rig()

    qa = [m.jnt_qposadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
          for n in ("L_q1", "L_q2", "L_q3")]
    aid = _adr(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "m_L_ankle")
    d = mujoco.MjData(m)
    for i, a in enumerate(qa):
        d.qpos[a] = q[i]
    mujoco.mj_forward(m, d)
    for _ in range(40000):
        d.ctrl[:] = 0.0
        d.ctrl[aid] = MT.MOTOR_PEAK_NM
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    assert float(d.qpos[qa[2]]) >= m.jnt_range[j3][0] - 1e-3, "inside its range"
    assert float(d.qpos[qa[2]]) <= m.jnt_range[j3][1] + 1e-3


def test_the_spool_plant_EXPOSES_a_controller_this_project_does_not_have():
    """⚠️ **What M46 actually found, and it is why the milestone stops here.**

    Adding the missing degree of freedom exposed a **missing controller**. With the
    actuator on the rotor instead of the tendon, commanding a tension is no longer
    instantaneous: it arrives through a series-elastic mode at
    `sqrt(k_tors / I_rotor)` = **758 rad/s, about 120 Hz**. Every controller this
    project has -- M42's position loop, M44's whole-body allocation, M45's
    comparisons -- commands tension **directly**, and none of them can drive this
    plant.

    On the old plant an outer position loop holds the hip and knee to **0.00°**.
    Here the same loop with a hand-tuned tension inner loop leaves **5-10°**.

    ⚠️ **And it is not numerical.** Refining the timestep 20× (1e-4 → 5e-6)
    changes the answer by under 2 %%, and the `implicit` integrator by less. A motor
    commanded to zero torque, on a spool with almost no inertia, being dragged by a
    cable, really does spin at hundreds of rad/s. **Open-loop torque on one motor
    with the rest at zero is not an experiment a tendon robot can perform** -- which
    is the honest reason M46's travel figures beyond the end-stop case are not
    published.

    The next milestone is the cascade the plant now requires: a tension loop inside,
    a joint loop outside, separated by that 120 Hz mode.

    ⚠️ Asserts the defect: fails when the drivetrain controller lands.
    """
    freq = math.sqrt(K_TORS / MT.ROTOR_ARMATURE)
    assert freq == pytest.approx(758.0, rel=0.05), (
        f"the series-elastic mode sits at {freq:.0f} rad/s"
    )

    m, q = _spooled()
    qa = [m.jnt_qposadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
          for n in ("L_q1", "L_q2", "L_q3")]
    dof = [m.jnt_dofadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in ("L_q1", "L_q2", "L_q3")]
    names = ["hip_flex", "hip_ext", "knee_flex", "knee_ext", "ankle",
             "ankle_ext"]
    A = [_adr(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "m_L_" + t) for t in names]
    JS = [m.jnt_qposadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, "js_L_" + t)]
          for t in names]
    tid = [_adr(m, mujoco.mjtObj.mjOBJ_TENDON, "L_" + t) for t in names]

    J = np.zeros((6, 3))
    for k in range(3):
        Ls = []
        for sgn in (+1, -1):
            dd = mujoco.MjData(m)
            for i, a in enumerate(qa):
                dd.qpos[a] = q[i]
            dd.qpos[qa[k]] += sgn * 0.002
            mujoco.mj_forward(m, dd)
            Ls.append(np.array([dd.ten_length[t] for t in tid]))
        J[:, k] = (Ls[0] - Ls[1]) / 0.004
    G = (-J).T

    d = mujoco.MjData(m)
    for i, a in enumerate(qa):
        d.qpos[a] = q[i]
    mujoco.mj_forward(m, d)
    for _ in range(30000):
        e = np.array([q[i] - d.qpos[a] for i, a in enumerate(qa)])
        ev = np.array([-d.qvel[a] for a in dof])
        tau = wbc.actuator_torque(d, dof, 10.0 * e + 0.2 * ev)
        T = wbc.tendon_tension(G, tau, t_min=19.6, t_max=MT.TENSION_MAX)
        for i in range(6):
            meas = K_TORS * (-float(d.qpos[JS[i]])) / MT.SPOOL_R
            cmd = MT.SPOOL_R * T[i] + 1.0 * MT.SPOOL_R * (T[i] - meas)
            d.ctrl[A[i]] = float(np.clip(cmd, 0.0, MT.MOTOR_PEAK_NM))
        mujoco.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)):
            break

    drift = np.degrees(np.array([float(d.qpos[a]) for a in qa]) - q)
    assert float(np.max(np.abs(drift))) > 2.0, (
        f"drift {np.round(drift, 2)} -- if this now holds, the drivetrain "
        "controller landed and M46's gate should be re-run"
    )


# ===================================================================
# M47 - the DRIVETRAIN CASCADE the spool plant asked for
# ===================================================================

SERVO_KP = MT.ROTOR_ARMATURE * MT.ROTOR_BANDWIDTH ** 2
SERVO_KV = 2.0 * MT.ROTOR_ARMATURE * MT.ROTOR_BANDWIDTH
SPOOL_TN = ("hip_flex", "hip_ext", "knee_flex", "knee_ext", "ankle",
            "ankle_ext")


def _servo_rig(leg_p=None, pin=()):
    """The spooled leg with a POSITION servo on each rotor."""
    from tomcat_kin.params import DEFAULT_HINDLEG as HL

    leg_p = HL if leg_p is None else leg_p
    q = np.asarray(LegModel(leg_p).inverse((0.04, -0.17, 0.0)), float)
    xml = MT.single_leg_rig_spooled(leg_p=leg_p, q_ref=q, series_k=SERIES_K,
                                    ankle_pair=True, spool_servo=True)
    for i, jn in enumerate(("L_q1", "L_q2", "L_q3")):
        if jn not in pin:
            continue
        xml = re.sub(
            r'(name="%s"[^>]*range=")[-0-9. ]+(")' % jn,
            lambda mo, v=q[i]: "%s%.6f %.6f%s" % (mo.group(1), v - 1e-6,
                                                  v + 1e-6, mo.group(2)),
            xml)
    return mujoco.MjModel.from_xml_string(xml), q


def _idx(m):
    A = [_adr(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "m_L_" + t) for t in SPOOL_TN]
    JR = [m.jnt_qposadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, "jr_L_" + t)]
          for t in SPOOL_TN]
    JS = [m.jnt_qposadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, "js_L_" + t)]
          for t in SPOOL_TN]
    TP = [_adr(m, mujoco.mjtObj.mjOBJ_TENDON, "L_" + t) for t in SPOOL_TN]
    qa = [m.jnt_qposadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
          for n in ("L_q1", "L_q2", "L_q3")]
    dof = [m.jnt_dofadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in ("L_q1", "L_q2", "L_q3")]
    return A, JR, JS, TP, qa, dof


def test_the_rotor_servo_gains_are_DERIVED_not_tuned():
    """✅ **M46 stopped rather than guess gains. These are not guessed.**

    A real motor brings an encoder and a current loop, so the inner loop is a
    **position servo on the rotor**, and for a critically damped rotor of inertia
    `I` at bandwidth `wn` the gains follow: `kp = I*wn^2`, `kv = 2*I*wn`. At the
    default 3000 rad/s that is **kp 180 N·m/rad, kv 0.12**.

    ⚠️ And the force range is **signed**. M46 corrected this: a motor turns either
    way, and *"a cable can only pull"* is a property of the CABLE. On the old plant
    the actuator WAS the tendon, so a one-sided `ctrlrange` said it correctly; with
    a spool in between, clamping the motor to one sign also removes its ability to
    damp its own drivetrain.
    """
    m, _ = _servo_rig()
    assert m.nu == 6
    for i in range(m.nu):
        assert m.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT
        assert m.actuator_gainprm[i][0] == pytest.approx(SERVO_KP, rel=1e-6)
        assert m.actuator_forcerange[i][0] == pytest.approx(-MT.MOTOR_PEAK_NM)
        assert m.actuator_forcerange[i][1] == pytest.approx(+MT.MOTOR_PEAK_NM)
    assert SERVO_KP == pytest.approx(180.0, rel=1e-6)
    assert SERVO_KV == pytest.approx(0.12, rel=1e-6)


def test_the_SERVOS_OWN_COMPLIANCE_lands_in_series_with_G3():
    """⚠️ **A firmware gain sets how much of a mechanical spring you get.**

    The rotor servo has finite stiffness `kp`, and it sits in **series** with the G3
    spring `k_tors`. Command a rotor angle for a target tension and the tension
    comes out low by exactly `kp / (kp + k_tors)`:

    | rotor bandwidth | kp (N·m/rad) | raw error | delivered (N/m) |
    |---|---|---|---|
    | 1000 rad/s | 20.0 | **-36.5 %** | **95 285** — ⚠️ outside ADR-0050's band |
    | 2000 | 80.0 | -12.6 % | 131 170 |
    | 3000 | 180.0 | -6.0 % | 141 004 |
    | 6000 | 720.0 | -1.6 % | 147 645 |

    ✅ `wbc.rotor_command`'s `servo_kp` removes the droop exactly — **-0.1 % at
    every gain, including the one that was 36 % out.** But the *delivered* series
    stiffness is still `kp*k_tors/(kp+k_tors)`, and at a 1000 rad/s rotor loop that
    is **95 kN/m against a specification of 150** -- outside the band ADR-0050 handed
    to mechanical. G3 cannot be specified without the servo bandwidth beside it.
    """
    m, q = _servo_rig(pin=("L_q1", "L_q2", "L_q3"))
    A, JR, JS, _, qa, _ = _idx(m)

    for target in (19.6, 100.0, 222.9):
        d = mujoco.MjData(m)
        for i, a in enumerate(qa):
            d.qpos[a] = q[i]
        mujoco.mj_forward(m, d)
        cmd = wbc.rotor_command(np.zeros(6), np.zeros(6),
                                np.full(6, target), K_TORS, MT.SPOOL_R,
                                servo_kp=SERVO_KP)
        for _ in range(30000):
            for i, a in enumerate(A):
                d.ctrl[a] = float(cmd[i])
            mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        got = np.mean([K_TORS * (-float(d.qpos[j])) / MT.SPOOL_R for j in JS])
        assert got == pytest.approx(target, rel=0.01), (
            f"compensated command should hit {target} N, got {got:.1f}"
        )

    # and uncompensated it is low by exactly the stiffness ratio
    ratio = SERVO_KP / (SERVO_KP + K_TORS)
    assert ratio == pytest.approx(0.940, abs=0.002)
    delivered = (SERVO_KP * K_TORS / (SERVO_KP + K_TORS)) / MT.SPOOL_R ** 2
    assert delivered == pytest.approx(141004.0, rel=0.01)
    assert delivered < SERIES_K, "the servo always costs some of the spring"
    # a 1000 rad/s loop falls out of ADR-0050's 150-200 kN/m band
    kp_slow = MT.ROTOR_ARMATURE * 1000.0 ** 2
    slow = (kp_slow * K_TORS / (kp_slow + K_TORS)) / MT.SPOOL_R ** 2
    assert slow < 1.0e5, f"a slow rotor loop delivers only {slow:.0f} N/m"


def _cascade_hold(m, q, target_q3_deg=None, kp=50.0, kd=1.0, tb=19.6,
                  seconds=2.0, refresh=25):
    """The M47 cascade: rotor servo inside, joint PD and tension allocation out.

    ⚠️ `G` is refreshed as the pose moves, which is not optional here: the ankle's
    moment arm **reverses** inside the ROM (ADR-0049), and a `G` frozen at the target
    pose drives the joint to the opposite end stop when the path crosses it.
    """
    A, JR, JS, TP, qa, dof = _idx(m)
    qd = q.copy()
    if target_q3_deg is not None:
        qd[2] = math.radians(target_q3_deg)

    d = mujoco.MjData(m)
    for i, a in enumerate(qa):
        d.qpos[a] = q[i]
    mujoco.mj_forward(m, d)

    def jac():
        J = np.zeros((6, 3))
        base = [float(d.qpos[a]) for a in qa]
        for k in range(3):
            Ls = []
            for sgn in (+1, -1):
                dd = mujoco.MjData(m)
                dd.qpos[:] = d.qpos
                dd.qpos[qa[k]] = base[k] + sgn * 0.002
                mujoco.mj_forward(m, dd)
                Ls.append(np.array([dd.ten_length[t] for t in TP]))
            J[:, k] = (Ls[0] - Ls[1]) / 0.004
        return (-J).T

    G = jac()
    for it in range(int(seconds / m.opt.timestep)):
        if it and it % refresh == 0:
            G = jac()
        e = np.array([qd[i] - d.qpos[a] for i, a in enumerate(qa)])
        ev = np.array([-d.qvel[a] for a in dof])
        T = wbc.tendon_tension(G, wbc.actuator_torque(d, dof, kp * e + kd * ev),
                               t_min=tb, t_max=MT.TENSION_MAX)
        cmd = wbc.rotor_command([d.qpos[j] for j in JR],
                                [d.qpos[j] for j in JS], T, K_TORS,
                                MT.SPOOL_R, servo_kp=SERVO_KP)
        for i, a in enumerate(A):
            d.ctrl[a] = float(cmd[i])
        mujoco.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)):
            return None
    return np.degrees(np.array([float(d.qpos[a]) for a in qa]) - qd)


def test_the_CASCADE_holds_the_pose_that_M46_could_not():
    """✅ **M46's gap, closed, and closed by derivation rather than tuning.**

    M46 left the spooled plant undriveable: every controller in the project commands
    tension **directly**, and with the actuator on the rotor a tension arrives
    through a 120 Hz series-elastic mode. A hand-tuned attempt left **5-10°**.

    The cascade is three pieces, each with a closed form:

    1. **inner** — a rotor position servo, gains `I*wn^2` and `2*I*wn`;
    2. **the command** — `theta_r_des = (theta_r + theta_s) + T*r/k_tors`, where the
       zero-tension rotor angle reads **straight off the state** as
       `theta_r + theta_s`, so no reference offset is needed at all;
    3. **droop compensation** — `(kp + k_tors)/kp`, exact.

    Outside it, the joint PD and non-negative tension allocation the project already
    had. It holds the stance pose to **0.00°** at every joint gain from 10 to 50.
    """
    m, q = _servo_rig()
    for kp, kd in ((10.0, 0.2), (25.0, 0.5), (50.0, 1.0)):
        drift = _cascade_hold(m, q, kp=kp, kd=kd)
        assert drift is not None, f"kp={kp} diverged"
        assert float(np.max(np.abs(drift))) < 0.01, (
            f"kp={kp}: drift {np.round(drift, 4)}"
        )


def test_the_cascade_TRACKS_the_hind_ankle_but_the_REVERSAL_still_bounds_it():
    """⚠️ **The answer to ADR-0050's open question, and it is a qualified yes.**

    ADR-0050 could not confirm dynamically that an antagonistic pair reaches the
    ankle range the trot commands. With the cascade it can be asked, and the answer
    splits on ADR-0049's **moment-arm reversal**.

    On the shipped 300° anchor the hind reversal sits at **~112°, inside** the
    hind ankle's 88.5-122.3° gait range, and the cascade cannot cross it:

    | commanded | reached |
    |---|---|
    | 88.5° | 88.53 — ✅ |
    | 97.1° (stance) | 97.10 — ✅ |
    | 110° | 97.44 — ⚠️ stuck |
    | 122.3° | 85.62 — ⚠️ stuck |

    ✅ **Move the anchor to 270° and it tracks the whole range** (worst error
    **3.48°**, including the 122.3° that a frozen `G` had sent to -30°). So the
    pair *can* do it; the reversal has to be moved out of the gait range first.

    ⚠️ **M47 measures that and does not ship it.** 270° for the hind breaks **14
    tests across M44, M45 and M46** -- every ankle measurement in three milestones
    was taken on 300° -- and no single angle serves both legs, because their hocks
    stand **81° apart**. That migration is its own work; see
    `MT._ankle_anchor_deg`.

    ⚠️ Asserts the defect: fails when the anchor migration lands.
    """
    m, q = _servo_rig()
    stance = math.degrees(q[2])
    assert stance == pytest.approx(97.1, abs=0.5)
    assert MT._ankle_anchor_deg(DEFAULT_HINDLEG) == 300.0, "still the M44 anchor"

    near = _cascade_hold(m, q, target_q3_deg=88.5)
    assert near is not None and abs(near[2]) < 5.0, (
        f"the low end of the gait range is reachable: {near[2]:.2f} deg off"
    )

    far = _cascade_hold(m, q, target_q3_deg=122.3)
    assert far is not None
    assert abs(far[2]) > 20.0, (
        f"122.3 deg is past the reversal and must NOT be reachable on the 300 deg "
        f"anchor; got {far[2]:.2f} deg of error -- if this now tracks, the anchor "
        "migration landed and M47's gate should be re-run"
    )


# ===================================================================
# M48 - the ankle anchor migration, and what it exposed instead
# ===================================================================

def _leg_jacobian(leg_p, ankle_pair=False):
    """Every tendon's moment arm on this leg, mm/rad, at its stance pose."""
    names = ["L_hip_flex", "L_hip_ext", "L_knee_flex", "L_knee_ext", "L_ankle"]
    if ankle_pair:
        names.append("L_ankle_ext")
    q = np.asarray(LegModel(leg_p).inverse((0.04, -0.17, 0.0)), float)
    m = mujoco.MjModel.from_xml_string(
        MT.single_leg_rig(leg_p=leg_p, ankle_pair=ankle_pair))
    tid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, n) for n in names]
    dof = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in JNT]
    J = np.zeros((len(names), 3))
    for k in range(3):
        Ls = []
        for sgn in (+1, -1):
            d = mujoco.MjData(m)
            for i, a in enumerate(dof):
                d.qpos[a] = q[i]
            d.qpos[dof[k]] += sgn * 0.002
            mujoco.mj_forward(m, d)
            Ls.append(np.array([d.ten_length[t] for t in tid]))
        J[:, k] = (Ls[0] - Ls[1]) / 0.004
    return names, J * 1e3


def test_the_FORE_LEGS_ROUTING_WAS_NEVER_MIRRORED():
    """⚠️ **The finding of M48, and it is not what M48 set out to do.**

    M48 set out to migrate the ankle anchor (see the test below). Measuring the
    result on both legs -- which no previous milestone had done, because every
    Jacobian since M42 was taken on the **hind** leg -- turned up something larger.

    The hind leg is exact. The fore leg is not, and it is not the ankle:

    | tendon | hind hip | fore hip | spec |
    |---|---|---|---|
    | hip flexor | **+28.000** | **+11.636** | ±28 |
    | hip extensor | **-28.000** | **+35.885** | ±28 |

    ⚠️ **The fore hip pair does not oppose.** Both arms are positive, so the two
    cables pull the joint the same way and there is no antagonist at all. The knee
    and ankle rows are as wrong: `knee_ext` puts **+44.067** on the hip where the
    via-pulley says -8.75, and `ankle` puts **-39.909** on the knee where it says
    -8.75.

    ⚠️ **And this has been true since M42.** The hip's moment arms cannot depend on
    the ankle anchor, and the same numbers come out of the committed M47 tree. What
    it is, is the thing flagged in ADR-0048 and never acted on: `DEFAULT_FORELEG`
    folds the **opposite** way, and the fore leg's sidesites and anchor angles were
    **inherited from the hind leg rather than mirrored**. ADR-0048 measured the
    symptom as a fore/hind drift gap and read it as a tuning difference. It is not.

    **What rests on it:** every fore-leg figure in M43-M47 -- the welded and floating
    drift, ADR-0049's "the binding tendon is the hind hip extensor", ADR-0052's
    "the fore leg tracks to 11.8° against the hind's 3.5". ✅ ADR-0050's ankle
    decision does **not**: its deciding argument came from `gait.py`'s joint
    trajectories, which never touch the MJCF routing.

    ⚠️ Asserts the defect: **fails when the fore leg is mirrored properly.**
    """
    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm) * 1e3

    names, Jh = _leg_jacobian(DEFAULT_HINDLEG)
    assert Jh[0, 0] == pytest.approx(+arms[0], abs=0.05)
    assert Jh[1, 0] == pytest.approx(-arms[0], abs=0.05)
    assert Jh[0, 0] * Jh[1, 0] < 0, "the hind hip pair opposes, as it always has"

    names, Jf = _leg_jacobian(DEFAULT_FORELEG)
    assert Jf[0, 0] * Jf[1, 0] > 0, (
        f"the fore hip pair should NOT oppose yet: "
        f"{Jf[0, 0]:.3f} and {Jf[1, 0]:.3f} -- if it now does, the fore leg was "
        "mirrored and every fore-leg figure in M43-M47 needs re-deriving"
    )
    assert abs(abs(Jf[0, 0]) - arms[0]) > 10.0, (
        f"and neither arm is near the {arms[0]:.0f} mm specification: "
        f"{Jf[0, 0]:.3f}"
    )
    # the couplings are wrong too, so this is the whole routing and not one site
    via = MT.VIA_R * 1e3
    assert abs(abs(Jf[3, 0]) - via) > 10.0, (
        f"fore knee_ext puts {Jf[3, 0]:.3f} on the hip, not {via:.2f}"
    )


def test_the_ANKLE_ANCHOR_MIGRATION_is_measured_but_not_shipped():
    """⚠️ **M48 measured the migration and did not land it, because the fore-leg
    defect above changes the order of the work.**

    ADR-0052 established the criterion -- the ankle's moment arm must not reverse
    anywhere inside a leg's **gait** range -- and measured the angles that satisfy
    it: **hind 270°, fore 300°** against the shipped 300° for both. Applying it,
    on the hind leg alone, does what it should:

    | | shipped 300° | migrated 270° |
    |---|---|---|
    | reversal vs the 88.5-122.3° gait range | ⚠️ inside (~112°) | ✅ outside |
    | cascade tracks 122.3° | ⚠️ reaches 85.6 | ✅ 121.6 |
    | lone ankle tendon at stance | plantarflexes | ⚠️ **dorsiflexes** |
    | quadruped, lone ankle | stands, 0.01° | ⚠️ **inverts, 179.75°** |
    | quadruped, ankle PAIR | stands | ✅ stands, 0.01° |

    ✅ **So the migration forces ADR-0050's Option A rather than merely preferring
    it**: at the migrated anchor a lone ankle tendon reaches the gait range and
    **cannot stand at all**. And the ankle finally clears ADR-0026's compliance
    floor -- **86.3 N·m/rad** with the pair against 55.1 with one tendon, the first
    time this joint has met the 80 the balance work has wanted since M20.

    ⚠️ **Three control findings retract on the migrated plant**, and all three were
    consequences of an under-actuated ankle rather than of a control law:

    | finding | as published | on Option A, migrated |
    |---|---|---|
    | ADR-0047: gravity feedforward cannot hold a pose | diverges | **0.0001°** |
    | ADR-0049: clipping loses the leg | 197° | **0.0002°** |
    | ADR-0049: the bare cable inverts the robot (G3) | tilt 180° | **stands, 0.07°** |

    ⚠️ **And a caution the other way:** on Option B at the migrated anchor,
    **clipping beats NNLS** (0.0001° against 46.6°), because with a lone tendon
    whose sign is wrong for the load the honest minimum-residual solution drives the
    ankle away while clipping happens not to. A controller comparison on an
    infeasible plant measures the plant.

    ⚠️ **Why it is not shipped:** the migration re-derives 17 tests across M42-M47,
    and the fore leg those tests measure has never been mirrored. Re-deriving on a
    known-broken leg would bake the wrong numbers in. The order is: mirror the fore
    leg, then migrate, then re-derive **once**.

    ⚠️ Asserts the defect: fails when the migration lands.
    """
    assert MT._ankle_anchor_deg(DEFAULT_HINDLEG) == 300.0, (
        "still the ADR-0050 anchor for both legs -- if this is now 270, the "
        "migration landed and M48's numbers above should be re-run"
    )
    assert MT._ankle_anchor_deg(DEFAULT_FORELEG) == 300.0

    # and the reversal really is inside the hind gait range at 300 deg
    demand = _trot_ankle_demand()
    hind = demand["LR"]
    lo = hind["ref"] - hind["swing_below"]
    hi = hind["ref"] + hind["swing_above"]
    assert lo == pytest.approx(88.5, abs=1.0)
    assert hi == pytest.approx(122.3, abs=1.0)

    q = np.asarray(LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0)), float)
    base = MT.single_leg_rig(ankle_pair=True)

    def ankle_arm(q3_deg):
        m = mujoco.MjModel.from_xml_string(base)
        t = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, "L_ankle")
        dof = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
               for n in JNT]
        Ls = []
        for sgn in (+1, -1):
            d = mujoco.MjData(m)
            for i, a in enumerate(dof):
                d.qpos[a] = q[i]
            d.qpos[dof[2]] = math.radians(q3_deg) + sgn * 0.002
            mujoco.mj_forward(m, d)
            Ls.append(float(d.ten_length[t]))
        return (Ls[0] - Ls[1]) / 0.004

    signs = {int(np.sign(ankle_arm(dq))) for dq in (90, 105, 120)}
    assert len(signs) == 2, (
        "the reversal must still sit inside the hind gait range on the shipped "
        f"anchor; signs across 90-120 deg were {signs}"
    )
