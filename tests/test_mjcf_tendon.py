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


# --------------------------------------------------------------------------
# ⚠️ Which plant a test is measured on -- M54 made the SHIPPED one the default.
#
# `mjcf_tendon` now defaults to what ADR-0055 and ADR-0058 decided: **clamped
# capstans, one bidirectional motor per antagonistic pair, the pair split across
# opposite sides of each via-pulley**. Twelve leg motors, `sum r·q` exactly.
#
# ⚠️ Everything M42-M51 measured was measured on a **different machine**: a cable
# ROUTED around each sheave (moment arm emergent, and wrong outside a window), with
# an INDEPENDENT PULL-ONLY MOTOR PER TENDON (20 or 24 of them). Those findings are
# still true *of that build*, and several of them are the evidence that it was
# replaced -- so they are pinned here rather than re-pointed, and the pin is the
# statement that they describe history.
#
# The rule this module follows: a test measures the LEGACY plant only when the
# finding is **about that construction**. Anything about control, allocation, or
# what the robot can do is re-derived on the shipped plant.
# --------------------------------------------------------------------------

def _legacy_leg(**kw):
    """M42-M51's one-leg rig: wrapped routing, one pull-only motor per tendon."""
    kw.setdefault("ankle_pair", False)
    return MT.single_leg_rig(clamped=False, pulley=False, **kw)


def _legacy_leg_elastic(**kw):
    """`_legacy_leg` with per-cable elasticity -- only a routed cable can have it."""
    kw.setdefault("ankle_pair", False)
    return MT.single_leg_rig_elastic(clamped=False, pulley=False, **kw)


def _legacy_spooled(**kw):
    """M46-M47's drivetrain rig, on the legacy routing it was developed against."""
    kw.setdefault("ankle_pair", False)
    return MT.single_leg_rig_spooled(clamped=False, pulley=False, **kw)


def _legacy_quad(**kw):
    """M43-M51's quadruped: wrapped routing, 20 (or 24) pull-only motors."""
    kw.setdefault("ankle_pair", False)
    return MT.quadruped_rig(clamped=False, pulley=False, **kw)


def _legacy_quad_elastic(**kw):
    """`_legacy_quad` with per-cable elasticity."""
    kw.setdefault("ankle_pair", False)
    return MT.quadruped_rig_elastic(clamped=False, pulley=False, **kw)


def _clamped_leg(**kw):
    """M52's plant: CLAMPED capstans, but still one motor per tendon.

    ⚠️ This is a half-step and it does not ship either -- ADR-0058 put one motor on
    each PAIR. It is kept because M52's findings (the map is exact at every angle,
    the leg holds on feedforward alone, co-contraction is a real redundant
    coordinate) are what made that decision, and each is a statement about a plant
    with independent motors.
    """
    kw.setdefault("ankle_pair", False)
    return MT.single_leg_rig(clamped=True, pulley=False, **kw)


def _clamped_quad(**kw):
    """M52's quadruped: clamped capstans, one motor per tendon."""
    kw.setdefault("ankle_pair", False)
    return MT.quadruped_rig(clamped=True, pulley=False, **kw)


@pytest.fixture(scope="module")
def rig():
    m = mujoco.MjModel.from_xml_string(_legacy_leg())
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
    revived = _legacy_leg().replace(
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
    dead = _legacy_leg().replace(
        'name="L_knee_anchor" pos="%.5f 0.012 %.5f"'
        % (0.55 * r_knee, -(r_knee + 0.005)),
        'name="L_knee_anchor" pos="%.5f 0.012 %.5f"'
        % (1.15 * r_knee * math.cos(a), 1.15 * r_knee * math.sin(a)))
    assert dead != _legacy_leg(), "the knee anchor substitution must bite"
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
        _legacy_leg_elastic(q_ref=q, series_k=series_k))
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
    m = mujoco.MjModel.from_xml_string(_legacy_leg_elastic(q_ref=q))
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
    m = mujoco.MjModel.from_xml_string(_legacy_leg_elastic(q_ref=q))
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
    xml = _legacy_leg()
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
        _legacy_quad_elastic(q_ref=q, hip_height=0.176, series_k=1.75e5))
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
    welded = _legacy_quad_elastic(
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
    base = _legacy_leg()

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
    m = mujoco.MjModel.from_xml_string(_legacy_leg())
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


#: How often the whole-body drivers re-measure each leg's tendon Jacobian, in
#: steps. ⚠️ **It is half the cost of these tests, and the right value depends on
#: how far the pose moves.** Measured on the standing gate, where it barely moves:
#:
#:     refresh   wall   z_end     tilt    residual
#:        25    36.3 s  0.17579  0.006 deg  0.0170
#:        50    25.4    0.17582  0.005      0.0167
#:       250    18.4    0.17587  0.005      0.0175
#:       500    17.9    0.17588  0.005      0.0145
#:
#: 250 is 2x faster for an answer that moves in the fifth decimal. ⚠️ **A test that
#: MOVES the pose must keep 25**: ADR-0052 measured a Jacobian frozen across the
#: ankle's moment-arm reversal driving the joint to the opposite end stop.
REFRESH_STATIC = 250
REFRESH_MOVING = 25


def _wbc_stand(m, q, *, tb=19.6, mu=0.8, seconds=3.0, refresh=REFRESH_STATIC,
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
        _legacy_quad_elastic(q_ref=q, hip_height=0.176, series_k=None))
    r = _wbc_stand(stiff, q, seconds=2.0, refresh=REFRESH_STATIC)
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
    m = mujoco.MjModel.from_xml_string(_legacy_leg_elastic(
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
    base = _legacy_leg(ankle_pair=True)

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
        m = mujoco.MjModel.from_xml_string(_legacy_leg_elastic(
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
    m = mujoco.MjModel.from_xml_string(_legacy_leg_elastic(
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
    a = mujoco.MjModel.from_xml_string(_legacy_quad(hip_height=0.176,
                                                        ankle_pair=True))
    b = mujoco.MjModel.from_xml_string(_legacy_quad(hip_height=0.176))
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
    m = mujoco.MjModel.from_xml_string(_legacy_leg_elastic(
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
    ma = mujoco.MjModel.from_xml_string(_legacy_leg_elastic(
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
    xml = _legacy_spooled(q_ref=q, series_k=SERIES_K,
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
    old = mujoco.MjModel.from_xml_string(_legacy_leg())
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
        _legacy_leg(spools=SERIES_K, ankle_pair=True))
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
    xml = _legacy_spooled(q_ref=q, series_k=SERIES_K, ankle_pair=True)
    assert 'solreflimit="%s"' % MT.EQ_SOLREF in xml
    assert 'solimplimit="%s"' % MT.EQ_SOLIMP in xml
    # and the plain rig does not need it, because it has no equality to lose to
    assert "solreflimit" not in _legacy_leg()

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

    ⚠️ **This measures the LEGACY plant** -- the wrapped routing with one
    pull-only motor per cable, which M54 stopped building by default. The
    defect below is real *there*, and it is part of why that plant was
    replaced.

    ✅ **Superseded as an open item by M47's cascade**, which is the controller
    this test says is missing. It stays because the *plant* property it measures --
    a spool with no loop around it does not hold a pose -- is still true, and is
    still true of the shipped drivetrain: zero command drifts the leg 20.6° at the
    hip in half a second. What changed is that the drivetrain now exists behind the
    pulley at all; see
    `test_the_DRIVETRAIN_now_exists_BEHIND_THE_PULLEY_and_so_does_G3`.
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
    xml = _legacy_spooled(leg_p=leg_p, q_ref=q, series_k=SERIES_K,
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
                  seconds=2.0, refresh=REFRESH_MOVING):
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

    ⚠️ **This measures the LEGACY plant** -- the wrapped routing with one
    pull-only motor per cable, which M54 stopped building by default. The
    defect below is real *there*, and it is part of why that plant was
    replaced.

    ✅ **The anchor migration is no longer needed, and this bound no longer
    exists on the shipped plant.** The reversal is a property of a *wrapped* moment
    arm; clamped, the ankle arm is +14.000 mm across the whole ROM, so there is
    nothing to migrate away from. Measured in
    `test_the_SHIPPED_map_is_CONSTANT_over_the_WHOLE_ROM_on_BOTH_legs`.
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
        _legacy_leg(leg_p=leg_p, ankle_pair=ankle_pair))
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

    ⚠️ **This measures the LEGACY plant** -- the wrapped routing with one
    pull-only motor per cable, which M54 stopped building by default. The
    defect below is real *there*, and it is part of why that plant was
    replaced.

    ✅ **Closed by construction rather than by mirroring.** On the shipped
    transmission the fore leg's map is *identical* to the hind's -- not mirrored,
    the same matrix -- so the asymmetry this test measures cannot arise. Measured in
    `test_the_SHIPPED_map_is_CONSTANT_over_the_WHOLE_ROM_on_BOTH_legs`.
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

    ⚠️ **This measures the LEGACY plant** -- the wrapped routing with one
    pull-only motor per cable, which M54 stopped building by default. The
    defect below is real *there*, and it is part of why that plant was
    replaced.

    ✅ **The migration was overtaken.** It is a fix for a wrap, and the shipped
    build clamps instead, so the anchor angle stops being a free parameter. The
    numbers here stay recorded for any joint that ever goes back to a resting wrap.
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
    base = _legacy_leg(ankle_pair=True)

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


# ===================================================================
# M49 - the routing audit: only the ANKLES were ever validated
# ===================================================================

def _gait_ranges():
    """Per-leg joint ranges the trot commands, degrees."""
    from tomcat_kin import gait

    lp = {"LF": DEFAULT_FORELEG, "LR": DEFAULT_HINDLEG}
    c = gait.GaitController(gait.trot_params())
    acc = {}
    for st in c.sample_cycle(400):
        for nm, ls in st.legs.items():
            if nm in lp and ls.q is not None:
                acc.setdefault(nm, []).append(np.degrees(ls.q))
    return {nm: np.array(v) for nm, v in acc.items()}


def _pair_over_range(leg_p, joint, tendons, lo, hi, n=13):
    """Worst arm error and how many sample points have a SAME-SIGN pair."""
    q = np.asarray(LegModel(leg_p).inverse((0.04, -0.17, 0.0)), float)
    m = mujoco.MjModel.from_xml_string(
        _legacy_leg(leg_p=leg_p, ankle_pair=True))
    dof = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
           for j in JNT]
    tid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, t)
           for t in tendons]
    spec = float(DEFAULT_TENDON.joint_moment_arm[joint])
    same, worst = 0, 0.0
    for dq in np.linspace(lo, hi, n):
        vals = []
        for t in tid:
            Ls = []
            for sgn in (+1, -1):
                d = mujoco.MjData(m)
                for i, a in enumerate(dof):
                    d.qpos[a] = q[i]
                d.qpos[dof[joint]] = math.radians(dq) + sgn * 0.002
                mujoco.mj_forward(m, d)
                Ls.append(float(d.ten_length[t]))
            vals.append((Ls[0] - Ls[1]) / 0.004)
        if vals[0] * vals[1] > 0:
            same += 1
        worst = max(worst, max(abs(abs(v) - spec) for v in vals))
    return same, worst


def test_ONLY_THE_ANKLES_were_ever_validated_across_the_GAIT():
    """⚠️ **The M49 audit, and it is larger than ADR-0053 thought.**

    ADR-0052 set the criterion for the ankle: the pair must stay **opposing**, with
    both arms near specification, **everywhere in the range the gait commands** --
    not merely at the stance pose. Nobody had applied it to the hip or the knee.
    Applied to all six routings:

    | leg | joint | gait range | same-sign points | worst arm error |
    |---|---|---|---|---|
    | hind | hip | -110.0..-35.1° | ⚠️ **6 of 13** | 8.38 mm |
    | hind | knee | -114.7..-54.7° | none | ⚠️ **9.91 mm** on 25 |
    | hind | ankle | 88.5..122.3° | none | ✅ **0.32 mm** |
    | fore | hip | -168.8..-141.2° | ⚠️ **5 of 13** | ⚠️ **26.06 mm** on 28 |
    | fore | knee | 33.4..103.8° | none | ⚠️ **23.81 mm** |
    | fore | ankle | 3.0..78.8° | none | ✅ **0.28 mm** |

    ✅ **Only the ankles pass** -- and the ankles are the only joints anyone ever
    swept against a range criterion, in M42, M44 and M45. The hip and knee anchors
    were placed by M42's 2-D heuristic and checked **at the stance pose only**.

    ⚠️ **So ADR-0053's framing was too narrow.** It read this as "the fore leg's
    routing was never mirrored". The fore leg is worse, but the real statement is
    **"the hip and knee were never validated across the gait, on either leg"**. The
    fore leg merely has the bad luck that its stance pose (-147.8° at the hip)
    sits *inside* its own failure band, while the hind's (-49.2°) sits outside --
    which is exactly why five milestones of stance-pose checks saw nothing.

    ⚠️ **This measures the LEGACY plant** -- the wrapped routing with one
    pull-only motor per cable, which M54 stopped building by default. The
    defect below is real *there*, and it is part of why that plant was
    replaced.

    ✅ **Re-derived in M54, and the audit is now moot rather than passed**: the
    shipped map is the same constant matrix at every pose in every joint's full ROM
    on both legs, so there is no gait sub-range left in which it could be wrong.
    Measured in `test_the_SHIPPED_map_is_CONSTANT_over_the_WHOLE_ROM_on_BOTH_legs`.
    """
    ranges = _gait_ranges()
    cases = [
        ("LR", DEFAULT_HINDLEG, "hip", 0, ("L_hip_flex", "L_hip_ext")),
        ("LR", DEFAULT_HINDLEG, "knee", 1, ("L_knee_flex", "L_knee_ext")),
        ("LR", DEFAULT_HINDLEG, "ankle", 2, ("L_ankle", "L_ankle_ext")),
        ("LF", DEFAULT_FORELEG, "hip", 0, ("L_hip_flex", "L_hip_ext")),
        ("LF", DEFAULT_FORELEG, "knee", 1, ("L_knee_flex", "L_knee_ext")),
        ("LF", DEFAULT_FORELEG, "ankle", 2, ("L_ankle", "L_ankle_ext")),
    ]
    result = {}
    for nm, lp, name, k, tendons in cases:
        a = ranges[nm]
        result[(nm, name)] = _pair_over_range(lp, k, tendons,
                                              float(a[:, k].min()),
                                              float(a[:, k].max()))

    # the ankles pass, on both legs -- they are the ones that were swept
    for nm in ("LR", "LF"):
        same, worst = result[(nm, "ankle")]
        assert same == 0, f"{nm} ankle pair goes same-sign at {same} points"
        assert worst < 1e-3, f"{nm} ankle worst arm error {1e3 * worst:.2f} mm"

    # and nothing else does
    for nm in ("LR", "LF"):
        same, worst = result[(nm, "hip")]
        assert same > 0, (
            f"{nm} hip should still have same-sign points -- if it does not, the "
            "routing was re-derived and M49's table needs re-running"
        )
        same, worst = result[(nm, "knee")]
        assert worst > 5e-3, (
            f"{nm} knee arm error {1e3 * worst:.2f} mm -- expected it still off "
            "specification across the gait"
        )

    # the fore hip is the worst of them, and by a lot
    assert result[("LF", "hip")][1] > 3 * result[("LR", "hip")][1]


def test_three_of_the_four_BAD_ROUTINGS_have_a_measured_fix():
    """✅ **M49 swept them, and three of four come out clean.**

    Applying ADR-0050's **capstan** construction (both cables to one anchor,
    opposite wraps) and sweeping the anchor angle against the gait-range criterion:

    | leg | joint | anchor | worst arm error |
    |---|---|---|---|
    | hind | knee | **75°** | 0.28 mm |
    | fore | hip | **135°** | **0.00 mm** |
    | fore | knee | **285°** | 0.27 mm |
    | hind | hip | ⚠️ **none exists** | -- |

    ⚠️ **The hind hip has no solution at any anchor angle**, capstan or mirrored,
    and none at any girdle-spool offset swept (±40 mm in x, ±20 in z; the best
    left 1 of 13 points same-sign at 12.26 mm). Its gait range is **74.9° wide,
    2.7× the fore hip's 27.6°**, and the sheave construction's working window is
    narrower than that. It needs a different construction -- a larger sheave, a
    via-pulley at the girdle so the incoming direction turns with the leg, or a
    narrower hip excursion from the gait.

    ⚠️ **Nothing ships**, for the reason ADR-0053 gave: the re-derivation must
    happen **once**, after the geometry is settled, and the hind hip is not.
    """
    ranges = _gait_ranges()
    a = ranges["LR"]
    lo, hi = float(a[:, 0].min()), float(a[:, 0].max())
    assert hi - lo == pytest.approx(74.9, abs=1.0), (
        "the hind hip's gait range is what makes it hard"
    )
    f = ranges["LF"]
    assert (float(f[:, 0].max()) - float(f[:, 0].min())) == pytest.approx(
        27.6, abs=1.0)

    # still shipped un-fixed: the audit test above is the one that flips
    assert MT._ankle_anchor_deg(DEFAULT_HINDLEG) == 300.0


# ===================================================================
# M50 - the hip needs a CLAMPED capstan, not a resting wrap
# ===================================================================

_CLAMP_RIG = """
<mujoco>
  <compiler angle="radian"/>
  <option timestep="1e-4" gravity="0 0 0"/>
  <worldbody>
    <site name="s_spool" pos="-0.10 0 0.034" size="0.001"/>
    <body name="femur">
      <joint name="q1" type="hinge" axis="0 -1 0" range="-2.1 2.1"/>
      <geom name="sheave" type="cylinder" size="%(r).5f 0.004"
            quat="0.70711 0.70711 0 0" mass="1e-9" contype="0" conaffinity="0"/>
      <geom type="capsule" fromto="0 0 0 0.09 0 0" size="0.004" mass="0.066"/>
      <site name="s_side" pos="0 0 %(side).5f" size="0.001"/>
      <site name="s_anchor" pos="%(ax).5f 0 %(az).5f" size="0.001"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="wrapped" width="0.001">
      <site site="s_spool"/>
      <geom geom="sheave" sidesite="s_side"/>
      <site site="s_anchor"/>
    </spatial>
    <fixed name="clamped"><joint joint="q1" coef="%(r).5f"/></fixed>
  </tendon>
</mujoco>
"""


def _clamp_arms(q1_deg, r=None):
    r = float(DEFAULT_TENDON.joint_moment_arm[0]) if r is None else r
    xml = _CLAMP_RIG % {"r": r, "side": -(r + 0.02), "ax": 0.55 * r,
                        "az": -(r + 0.005)}
    m = mujoco.MjModel.from_xml_string(xml)
    out = []
    for name in ("wrapped", "clamped"):
        t = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, name)
        Ls = []
        for sgn in (+1, -1):
            d = mujoco.MjData(m)
            d.qpos[0] = math.radians(q1_deg) + sgn * 0.002
            mujoco.mj_forward(m, d)
            Ls.append(float(d.ten_length[t]))
        out.append((Ls[0] - Ls[1]) / 0.004 * 1e3)
    return out


def test_a_RESTING_WRAP_has_a_working_window_and_a_CLAMPED_one_does_not():
    """✅ **M50's answer, and it explains every routing finding since M42.**

    ADR-0054 found the hip and knee fail across the gait and the hind hip has no
    anchor angle that works. This is why. A cable that merely **rests** on a sheave
    gives the sheave's radius only while it actually wraps; outside that the arm is
    whatever the straight line happens to give:

    | q1 | resting wrap | clamped capstan |
    |---|---|---|
    | -120° (past the gait range) | 12.956 mm | 28.000 |
    | **-110°** (its far edge) | **21.034** | 28.000 |
    | -90° | 31.979 | 28.000 |
    | **-60°** | **36.332** | 28.000 |
    | -10° and above | 28.000 | 28.000 |

    Across the hind hip's **-110..-35.1°** gait range the resting arm swings
    **21.03 to 36.33 mm -- 1.73×, on a 28 mm specification** (0.75× spec at one
    end, 1.30× at the other), and it is exact only for `q1 >= -10°`. The clamped one is
    **exactly 28.000 at every angle**, because the cable is *fixed to the sheave*
    rather than resting on it, so the arm is a property of the construction and not
    of contact.

    ⚠️ **And this re-frames ADR-0047's headline.** *"The moment arm is emergent from
    the geometry"* is true **where the cable wraps** -- which for the hip is
    `q1 >= -10°`, a fraction of its range. As a *validation* that a wrap produces
    the sheave radius, M42's result stands. As the *model* for a wide-ROM joint it
    does not, and five milestones of stance-pose checks never noticed because the
    stance pose is inside the window.

    ✅ Clamping costs wrapped length, and there is plenty: `r * dq` over the hind
    hip's range is **36.7 mm** against a **176 mm** circumference -- **21 %** of one
    turn.
    """
    r = float(DEFAULT_TENDON.joint_moment_arm[0])
    spec = r * 1e3

    # inside the window both constructions agree
    for deg in (-10, 0, 15, 30):
        wrapped, clamped = _clamp_arms(deg)
        assert wrapped == pytest.approx(spec, abs=0.05), deg
        assert clamped == pytest.approx(spec, abs=1e-6), deg

    # across the hind hip's gait range the resting wrap does not
    vals = [_clamp_arms(deg)[0] for deg in range(-110, -34, 10)]
    assert min(vals) < 0.8 * spec, f"resting arm bottoms at {min(vals):.2f} mm"
    assert max(vals) > 1.25 * spec, f"and peaks at {max(vals):.2f} mm"
    assert max(vals) / min(vals) > 1.7, "a 1.73x swing on a 28 mm specification"

    # the clamped one is exact everywhere, which is the whole point
    for deg in range(-120, 31, 10):
        assert _clamp_arms(deg)[1] == pytest.approx(spec, abs=1e-6)

    # and the wrapped length it needs is a fifth of a turn
    sweep = math.radians(74.9)
    assert r * sweep / (2 * math.pi * r) == pytest.approx(0.208, abs=0.005)


def test_the_two_ALTERNATIVES_to_clamping_are_priced_and_rejected():
    """⚠️ **Both were measured before clamping was reached, and both are too
    expensive. Recorded so they are not re-derived.**

    **A larger hip sheave.** Swept against the real gait range with the anchor swept
    at 5°: only **r = 50 mm** comes out clean (0 same-sign points, 0.00 mm error).
    ✅ It would also cut the ADR-0052 standing tension from **205 N to 110 N**,
    which is a real secondary benefit. ⚠️ But r = 50 mm is a **±100 mm diameter
    sheave on a 90 mm femur -- 1.11× the segment it sits on.** The largest
    manufacturable size swept (r = 36 mm, ±72 mm) still leaves 1 of 17 sample
    points same-sign.

    **A narrower hip excursion.** The routing's working window ends near **-95°**
    and the trot commands **-110°**, so the gait would have to give up ~15° of hip
    extension. At a 90 mm femur that is **35 mm of stride** against a
    `stride_length` of **100 mm** -- ⚠️ **35 %**, and at fixed cadence 35 % of the
    speed.

    Clamping costs neither: it needs a cable **fixed to the sheave** rather than
    resting on it, which is a groove and a ferrule.
    """
    from tomcat_kin import gait

    # the working window really does end near -95 deg
    r = float(DEFAULT_TENDON.joint_moment_arm[0]) * 1e3
    assert _clamp_arms(-95)[0] == pytest.approx(29.9, abs=1.5), (
        "the resting arm is already off spec at the window edge"
    )
    assert _clamp_arms(-110)[0] == pytest.approx(21.0, abs=1.0)

    # the sheave that works is bigger than the segment it sits on
    assert 2 * 50.0 / (1e3 * DEFAULT_HINDLEG.l1) == pytest.approx(1.11, abs=0.02)

    # and the stride the gait would have to give up
    L = LegModel(DEFAULT_HINDLEG)
    q = np.asarray(L.inverse((0.04, -0.17, 0.0)), float)

    def foot_x(q1_deg):
        qq = q.copy()
        qq[0] = math.radians(q1_deg)
        return float(L.joint_positions(qq)[-1][0])

    lost = abs(foot_x(-95.0) - foot_x(-110.0))
    assert lost == pytest.approx(0.035, abs=0.003)
    assert lost / float(gait.trot_params().stride_length) > 0.3, (
        "over a third of the stride"
    )


# ===================================================================
# M51 - housekeeping, and two of the three items were blocked
# ===================================================================

def test_the_SIMULATION_never_implemented_ADR0008s_VARIABLE_RADIUS_PULLEY():
    """⚠️ **The largest of M51's findings, and it invalidates a mass figure this
    project has quoted since M38.**

    [ADR-0008](../docs/DESIGN_DECISIONS.md) is the mass-closure decision. At the
    counts the architecture called for, **the motors alone exceeded the whole
    body** (24 motors = 105 % of 3 kg, 31 = 136 %), and the design did not close.
    Its decision was **one motor per antagonistic pair, via a variable-radius
    pulley** -- 16 motors, amended to **19** by ADR-0009. `params.py` builds
    `trunk_mass` on exactly that: *"6 leg motors x 132 g"* per girdle, **12 leg
    motors**, one per DOF.

    ⚠️ **`mjcf_tendon.py` has never implemented it.** It emits an independent
    `<motor>` for every tendon:

    | | leg motors | unbudgeted | body |
    |---|---|---|---|
    | ADR-0008 budget, as `params` carries it | **12** | -- | 4.304 kg |
    | the simulation, lone ankle | **20** | **1.054 kg** | **5.358 kg** |
    | the simulation, ankle pair (ADR-0050) | **24** | **1.580 kg** | **5.885 kg** |

    So **eight milestones of tendon simulation (M42-M50) have been built on the
    architecture ADR-0008 explicitly rejected**, and on mass grounds specifically.
    A domestic cat is 4-5 kg, which NFR5 has been anchored to since ADR-0010; 5.36
    and 5.89 kg are outside it.

    ⚠️ **And it makes ADR-0050's costing wrong twice.** It priced Option A as
    *"+4 motors, +528 g, 4.83 kg"*. That assumed the baseline was the simulation's
    20 and that `params` already carried them. `params` carries 12, so the delta
    from the budget is **+1.58 kg**, not +0.53.

    The resolution is not to fold 4.83 kg in. It is to decide whether the
    variable-radius pulley gets implemented -- ADR-0008 says it does -- or whether
    ADR-0008 is re-opened at 5.36 kg. That is the next milestone.

    ⚠️ **This measures the LEGACY plant** -- the wrapped routing with one
    pull-only motor per cable, which M54 stopped building by default. The
    defect below is real *there*, and it is part of why that plant was
    replaced.

    ✅ **RETRACTED AS AN OPEN ITEM -- the pulley landed in M53 and became the
    default in M54.** The shipped quadruped has **12** leg motors, so the overrun
    this test prices is **zero** and `params.trunk_mass` needed no change. The 5.36
    and 5.89 kg figures were never paid. They stay measured here because they are
    the reason ADR-0008 was chosen over ADR-0002 rather than re-opened, and the
    numbers had to be right for that choice to mean anything.
    """
    from tomcat_kin.params import DEFAULT_BODY_MASS_KG

    motor_kg = 0.1317
    lone = mujoco.MjModel.from_xml_string(
        _legacy_quad(hip_height=0.176, ankle_pair=False))
    paired = mujoco.MjModel.from_xml_string(
        _legacy_quad(hip_height=0.176, ankle_pair=True))

    assert lone.nu == 20, "one motor per tendon, not per DOF"
    assert paired.nu == 24
    # ADR-0008's count is one per DOF: three per leg
    budgeted = 12
    assert lone.nu - budgeted == 8
    assert paired.nu - budgeted == 12

    over_lone = (lone.nu - budgeted) * motor_kg
    over_pair = (paired.nu - budgeted) * motor_kg
    assert over_lone == pytest.approx(1.054, abs=0.005)
    assert over_pair == pytest.approx(1.580, abs=0.005)
    assert DEFAULT_BODY_MASS_KG + over_lone == pytest.approx(5.358, abs=0.01)
    assert DEFAULT_BODY_MASS_KG + over_pair == pytest.approx(5.885, abs=0.01)
    # both are outside the 4-5 kg band NFR5 is anchored to
    assert DEFAULT_BODY_MASS_KG + over_lone > 5.0


def test_the_STANDING_TENSION_never_converges_it_SATURATES():
    """⚠️ **M51's second blocked item, and it corrects a published number.**

    ADR-0049 reported the hind hip extensor at *"~205 N mean, 2.5x the motor's
    continuous rating"* while standing, and left the thermal case as an open item.
    The thermal case cannot be computed from that number, because **it is not a
    steady-state value**. Run the same gate longer:

    | t | peak tension | trunk z | tilt |
    |---|---|---|---|
    | 1 s | 125.5 N | 0.17589 | 0.005° |
    | 4 s | 164.8 | 0.17586 | 0.005° |
    | 6 s | 194.3 | 0.17582 | 0.006° |
    | **9 s** | **222.9** — the motor ceiling | 0.17578 | 0.007° |
    | 12 s | 222.9 | 0.17573 | 0.006° |

    **It ramps monotonically to the actuator's peak rating and pins there.**
    ADR-0049's 205 N was simply where the ramp had reached in its 1-3 s window.

    ✅ **And the cause is measurable**: the tension tracks an uncontrolled joint
    drift, which is exactly the missing posture task [ADR-0052](../docs/DESIGN_DECISIONS.md)
    named. Hind-right drift against peak tension: 2.9°/125 N at 1 s, 3.8°/179 at
    5 s, 4.3°/217 at 7.5 s, then both stop when the motor saturates. The robot
    keeps standing throughout -- the trunk holds to 0.007° -- so this is not a fall,
    it is a wind-up.

    ⚠️ **So the thermal item is blocked behind the posture task**, and the honest
    statement is not *"2.5x the continuous rating"* but *"the tension does not
    settle; it saturates the actuator"*.

    ⚠️ **This measures the LEGACY plant** -- the wrapped routing with one
    pull-only motor per cable, which M54 stopped building by default. The
    defect below is real *there*, and it is part of why that plant was
    replaced.

    ✅ **Cured in M53, and not by the posture task this test expected.** The
    wind-up was co-contraction growing with the uncontrolled drift; the shipped
    transmission has no co-contraction to wind up, and the same gate converges at
    **68.2 N** and holds it -- inside the 81 N continuous rating. Measured in
    `test_the_pulley_also_CURES_the_standing_TENSION_SATURATION`. ⚠️ The ramp is
    still real on the legacy plant, which is what this test keeps guarding.
    """
    q = _quad_poses()
    m = mujoco.MjModel.from_xml_string(_legacy_quad_elastic(
        q_ref=q, hip_height=0.176, series_k=1.75e5))

    short = _wbc_stand(m, q, seconds=1.0)
    long = _wbc_stand(m, q, seconds=3.0)
    assert short["peak"] < long["peak"] - 5.0, (
        f"the tension must still be climbing: {short['peak']:.1f} N at 1 s, "
        f"{long['peak']:.1f} at 3 s -- if these now agree, the posture task "
        "landed and the thermal case can finally be computed"
    )
    # and it is a wind-up, not a fall: the trunk is holding the whole time
    for r in (short, long):
        assert r["z"] > 0.995 * r["z0"]
        assert r["tilt"] < 0.05


def test_the_SUITE_got_three_times_faster_by_measuring_what_cost_it():
    """✅ **M51's one clean item.**

    The suite had grown to **12 minutes**, and `tests/test_mjcf_tendon.py` alone to
    five. The dominant cost was not the simulated horizon: it was **re-measuring
    each leg's tendon Jacobian every 25 steps**, which is 24 forward passes per
    refresh per leg.

    Measured on the standing gate, where the pose barely moves:

    | refresh | wall | z_end | tilt | residual |
    |---|---|---|---|---|
    | 25 | 36.3 s | 0.17579 | 0.006° | 0.0170 |
    | 250 | **18.4 s** | 0.17587 | 0.005° | 0.0175 |
    | 500 | 17.9 s | 0.17588 | 0.005° | 0.0145 |

    ⚠️ **It is not a free constant.** A test that MOVES the pose must keep 25:
    ADR-0052 measured a Jacobian frozen across the ankle's moment-arm reversal
    driving the joint to the opposite end stop. So there are two constants with the
    reason attached, not one number tuned down.

    Result: `test_mjcf_tendon.py` **306 s → 95 s**, the whole suite **12:21 →
    4:41**, with no test shortened and no assertion loosened.
    """
    assert REFRESH_STATIC == 250
    assert REFRESH_MOVING == 25
    # the moving gate really does still refresh often
    import inspect
    src = inspect.getsource(_cascade_hold)
    assert "REFRESH_MOVING" in src, (
        "the cascade tracking test crosses the ankle reversal and must refresh"
    )


# ===================================================================
# M52 - the clamped transmission, built
# ===================================================================

def test_the_CLAMPED_transmission_is_EXACT_on_both_legs_everywhere():
    """✅ **ADR-0055's construction, built, and it passes ADR-0054's audit
    outright.**

    Clamped, a cable's length is exactly `sum r_j * q_j` over the joints it crosses,
    so the moment arms are the specification at every angle, on both legs:

    | | hip pair | knee pair | ankle pair | couplings |
    |---|---|---|---|---|
    | hind, stance | ±28.000 | ±25.000 | ±14.000 | -8.750 |
    | hind, q1 = -110° | ±28.000 | ±25.000 | ±14.000 | -8.750 |
    | fore, stance | ±28.000 | ±25.000 | ±14.000 | -8.750 |
    | fore, q1 = -168° | ±28.000 | ±25.000 | ±14.000 | -8.750 |

    Compare the wrapped construction the same audit measured: the hind hip
    same-sign at 6 of 13 gait samples, the fore hip at 5, the fore knee **23.81 mm
    out on a 25 mm specification**.

    ⚠️ **And the arm is no longer emergent.** ADR-0047's headline was that the
    sheave radius comes out of the routing on its own; clamped, the map **is** the
    analytic one `TendonMap` has carried since M4, and the simulation no longer
    derives it independently. That is the trade ADR-0055 made deliberately: the
    emergent version is only correct inside a window the gait leaves.
    """
    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm) * 1e3
    via = MT.VIA_R * 1e3
    names = ["L_hip_flex", "L_hip_ext", "L_knee_flex", "L_knee_ext",
             "L_ankle", "L_ankle_ext"]

    for leg_p, extreme in ((DEFAULT_HINDLEG, -110.0), (DEFAULT_FORELEG, -168.0)):
        q = np.asarray(LegModel(leg_p).inverse((0.04, -0.17, 0.0)), float)
        m = mujoco.MjModel.from_xml_string(
            _clamped_leg(leg_p=leg_p, ankle_pair=True))
        tid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, n)
               for n in names]
        dof = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
               for n in JNT]
        for q1 in (None, extreme):
            J = np.zeros((6, 3))
            for k in range(3):
                Ls = []
                for sgn in (+1, -1):
                    d = mujoco.MjData(m)
                    for i, a in enumerate(dof):
                        d.qpos[a] = q[i]
                    if q1 is not None:
                        d.qpos[dof[0]] = math.radians(q1)
                    d.qpos[dof[k]] += sgn * 0.002
                    mujoco.mj_forward(m, d)
                    Ls.append(np.array([d.ten_length[t] for t in tid]))
                J[:, k] = (Ls[0] - Ls[1]) / 0.004
            J = J * 1e3
            assert J[0, 0] == pytest.approx(+arms[0], abs=1e-6)
            assert J[1, 0] == pytest.approx(-arms[0], abs=1e-6)
            assert J[2, 1] == pytest.approx(+arms[1], abs=1e-6)
            assert J[3, 1] == pytest.approx(-arms[1], abs=1e-6)
            assert J[4, 2] == pytest.approx(-arms[2], abs=1e-6)
            assert J[5, 2] == pytest.approx(+arms[2], abs=1e-6)
            for row in (2, 3, 4, 5):
                assert J[row, 0] == pytest.approx(-via, abs=1e-6)


def test_the_clamped_leg_HOLDS_on_gravity_feedforward_alone():
    """✅ **And it is well conditioned enough that the control findings which
    M48 retracted stay retracted, for the same reason.**

    With every joint's pair exact at every angle, the non-negative allocation is
    never tight, and the leg holds to **0.00°** on gravity feedforward alone
    (`kp = 0`), on a co-contraction floor, and at every joint gain tried.

    ⚠️ ADR-0047 reported that feedforward *cannot* hold a pose. ADR-0053 already
    retracted that as a consequence of an under-actuated ankle rather than of the
    control law; this shows the same on a plant whose arms are exact rather than
    merely better.
    """
    q = np.asarray(LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0)), float)
    names = ["L_hip_flex", "L_hip_ext", "L_knee_flex", "L_knee_ext",
             "L_ankle", "L_ankle_ext"]
    m = mujoco.MjModel.from_xml_string(
        _clamped_leg(ankle_pair=True))
    tid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, n) for n in names]
    dof = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in JNT]
    A = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "m_" + n)
         for n in names]
    J = np.zeros((6, 3))
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

    for kp, kd, tb in ((0.0, 0.0, 19.6), (10.0, 0.2, 5.0), (50.0, 1.0, 19.6)):
        d = mujoco.MjData(m)
        for i, a in enumerate(dof):
            d.qpos[a] = q[i]
        for _ in range(20000):
            mujoco.mj_forward(m, d)
            e = np.array([q[i] - d.qpos[a] for i, a in enumerate(dof)])
            ev = np.array([-d.qvel[a] for a in dof])
            T = wbc.tendon_tension(G, wbc.actuator_torque(d, dof,
                                                          kp * e + kd * ev),
                                   t_min=tb, t_max=MT.TENSION_MAX)
            for i, a in enumerate(A):
                d.ctrl[a] = float(T[i])
            mujoco.mj_step(m, d)
            assert np.all(np.isfinite(d.qpos)), f"kp={kp} diverged"
        drift = np.degrees(np.array([d.qpos[a] for a in dof]) - q)
        assert float(np.max(np.abs(drift))) < 0.01, f"kp={kp}: {drift}"


def test_asking_for_ELASTICITY_on_a_clamped_tendon_now_RAISES():
    """⚠️ **It used to return a silently inelastic plant, which is the failure
    mode this project keeps paying for.**

    A `<fixed>` tendon's length is the commanded `sum r*q`, not a physical distance,
    so a `stiffness` on it is a passive **joint** spring pulling toward
    `springlength` -- not a stretching cable. `clamped_tendons` therefore emits no
    springs at all, and `single_leg_rig_elastic(clamped=True)` used to hand back a
    plant with **stiffness 0.0 on every tendon** while the caller believed it had
    asked for 175 kN/m.

    ✅ With the cable clamped, ADR-0047's **G3 series element belongs in the
    drivetrain**: `spools=<series_k>` (ADR-0051) puts it where it physically is, as
    an exact torsional spring between rotor and spool.
    """
    q = np.asarray(LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0)), float)
    with pytest.raises(ValueError, match="CLAMPED"):
        MT.single_leg_rig_elastic(pulley=False, q_ref=q, series_k=1.75e5, clamped=True)
    # and the inelastic clamped plant is still buildable, which is what M52 uses
    m = mujoco.MjModel.from_xml_string(
        _clamped_leg(ankle_pair=True))
    assert m.ntendon == 6 and m.nu == 6


def test_ADR0002_and_ADR0008_want_DIFFERENT_TRANSMISSIONS():
    """⚠️ **M51 called this "the simulation never implemented the pulley". It is
    worse than an omission: the two accepted decisions cannot both be had.**

    [ADR-0002](../docs/DESIGN_DECISIONS.md) decides antagonistic pairs *"for joints
    whose **stiffness must vary**, with co-contraction bias `T_bias` exposed as a
    **first-class control input**"*, and closes: *"Stiffness becomes commandable."*

    [ADR-0008](../docs/DESIGN_DECISIONS.md) decides *"one motor per antagonistic
    pair via the variable-radius pulley"* and says *"**Full articulation is
    retained** -- this is a change of transmission, not of DOF."*

    ⚠️ **That last sentence is true of the joint angles and false of the
    stiffness.** Co-contraction is not a position DOF, it is the **redundant
    coordinate**: with two motors on one joint the tension allocation has one degree
    of freedom to spend on `T_bias`, and with one motor it has none. A variable
    radius lets co-contraction be **scheduled** as a function of joint angle, baked
    into the pulley; it cannot be commanded at runtime. ADR-0008 never mentions
    stiffness or co-contraction at all.

    So the two are not "budget vs simulation" -- they are **two architectures**, and
    the project has been running ADR-0002's in simulation and ADR-0008's in the mass
    budget. Choosing ADR-0008 gives back **1.05 kg** and costs the co-contraction
    that ADR-0021 priced standing on, ADR-0043 built the AIC rule around, and M43
    measured as what keeps the clipped allocator exact.

    ⚠️ **This measures the LEGACY plant** -- the wrapped routing with one
    pull-only motor per cable, which M54 stopped building by default. The
    defect below is real *there*, and it is part of why that plant was
    replaced.

    ✅ **RESOLVED: ADR-0058 withdrew ADR-0002's commandable-stiffness clause.**
    The conflict this test measures was real and it decided the transmission, so the
    measurement stays -- a three-dimensional co-contraction null space is exactly
    what one motor per pair spends. What the shipped plant has instead is measured
    in `test_the_pulley_transmission_is_ADR0008s_MOTOR_COUNT_and_ADR0042s_MAP`:
    null space **zero**.
    """
    # the simulation implements ADR-0002: an independent actuator per tendon
    m = mujoco.MjModel.from_xml_string(
        _clamped_quad(hip_height=0.176, ankle_pair=False))
    assert m.nu == 20, "one motor per tendon -- ADR-0002's architecture"

    # and the allocation really does have the redundancy ADR-0008 would remove
    q = np.asarray(LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0)), float)
    # the PAIRED leg: every joint has an antagonist, so raising the floor can leave
    # the joint torques alone. With a lone ankle it cannot -- its torque follows its
    # tension, which is ADR-0050's finding and not this one.
    leg = mujoco.MjModel.from_xml_string(
        _clamped_leg(ankle_pair=True))
    names = ["L_hip_flex", "L_hip_ext", "L_knee_flex", "L_knee_ext", "L_ankle",
             "L_ankle_ext"]
    tid = [mujoco.mj_name2id(leg, mujoco.mjtObj.mjOBJ_TENDON, n) for n in names]
    dof = [leg.jnt_dofadr[mujoco.mj_name2id(leg, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in JNT]
    J = np.zeros((6, 3))
    for k in range(3):
        Ls = []
        for sgn in (+1, -1):
            d = mujoco.MjData(leg)
            for i, a in enumerate(dof):
                d.qpos[a] = q[i]
            d.qpos[dof[k]] += sgn * 0.002
            mujoco.mj_forward(leg, d)
            Ls.append(np.array([d.ten_length[t] for t in tid]))
        J[:, k] = (Ls[0] - Ls[1]) / 0.004
    G = (-J).T
    assert G.shape == (3, 6)
    # six tendons, three joints: THREE dimensions of co-contraction to command,
    # one per antagonistic pair -- exactly what one motor per pair would remove
    assert G.shape[1] - np.linalg.matrix_rank(G) == 3, (
        "ADR-0002's architecture leaves one co-contraction direction per pair; "
        "ADR-0008's one-motor-per-pair leaves none"
    )
    # a different T_bias really does change the tensions at the same torque
    tau = np.array([0.3, -0.2, -0.1])
    low = wbc.tendon_tension(G, tau, t_min=5.0, t_max=MT.TENSION_MAX)
    high = wbc.tendon_tension(G, tau, t_min=40.0, t_max=MT.TENSION_MAX)
    assert float(np.min(high)) > float(np.min(low)) + 20.0
    assert np.allclose(G @ low, G @ high, atol=1e-9), (
        "same joint torque, different co-contraction -- the freedom ADR-0008 spends"
    )


# ===================================================================
# M53 - ADR-0008's variable-radius pulley, chosen and built
# ===================================================================

PULLEY_PAIRS = ("hip", "knee", "ankle")


def _pulley_leg(leg_p=None):
    leg_p = DEFAULT_HINDLEG if leg_p is None else leg_p
    q = np.asarray(LegModel(leg_p).inverse((0.04, -0.17, 0.0)), float)
    m = mujoco.MjModel.from_xml_string(MT.single_leg_rig(
        leg_p=leg_p, clamped=True, pulley=True, ankle_pair=True))
    return m, q


def _pulley_G(m, q):
    dof = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in JNT]
    tid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, "L_" + p)
           for p in PULLEY_PAIRS]
    J = np.zeros((3, 3))
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
    return (-J).T, dof


def test_the_PULLEY_pair_must_route_on_OPPOSITE_SIDES_of_each_via():
    """⚠️ **The prerequisite nobody had noticed, and it decides whether ADR-0008 is
    buildable at all.**

    A variable-radius pulley takes up one cable of a pair while paying out the
    other, so it transmits their **difference**. Anything common to both has to be
    absorbed by cable stretch instead -- which is co-contraction, and the pulley has
    no way to relieve it.

    M42 routed both cables of a pair over the **same side** of each via-pulley, so
    the ADR-0042 coupling `-v*q1` lands on both equally:

    | routing | differential (what the motor sees) | common (what stretch absorbs) |
    |---|---|---|
    | same side (M42) | `[0, r_knee]` | `[-v, 0]` |
    | **opposite sides** | `[-v, r_knee]` | **`[0, 0]`** |

    ⚠️ Same-side, the common mode is **11.44 mm** at the knee across the hind gait
    range and **20.60 mm** at the ankle -- **1716 N and 3090 N** of co-contraction
    swing against a 638 N cable rating. The cables break.
    ⚠️ It is also *falsely decoupling*: the differential loses the hip term
    entirely, so the pulley would read the knee as independent of the hip while the
    coupling showed up as tension.

    ✅ Routed on opposite sides the common mode is **exactly zero** and ADR-0042's
    coupling stays in the differential, where the motor can act on it. That is what
    `pair_rows` emits.
    """
    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm, float)
    v = MT.VIA_R
    rows = MT.pair_rows(arms, ankle_pair=True)

    # the shipped pair rows carry the coupling in the DIFFERENTIAL
    assert dict(rows["knee"])["q1"] == pytest.approx(-v)
    assert dict(rows["ankle"])["q1"] == pytest.approx(-v)
    assert dict(rows["ankle"])["q2"] == pytest.approx(-v)
    assert dict(rows["hip"])["q1"] == pytest.approx(arms[0])

    # and same-side routing would put it in the common mode instead
    flex = np.array([-v, arms[1]])
    same_ext = np.array([-v, -arms[1]])
    opp_ext = np.array([+v, -arms[1]])
    assert np.allclose((flex + same_ext) / 2, [-v, 0.0])
    assert np.allclose((flex + opp_ext) / 2, [0.0, 0.0])
    assert np.allclose((flex - same_ext) / 2, [0.0, arms[1]]), (
        "same-side routing loses the hip coupling from the differential"
    )

    # the common mode same-side routing would leave is past the cable's rating
    q1 = math.radians(74.9)
    assert v * q1 * 1.5e5 > 638.0, "the 638 N structural rating, ADR-0046"


def test_the_pulley_transmission_is_ADR0008s_MOTOR_COUNT_and_ADR0042s_MAP():
    """✅ **ADR-0008 chosen and built, and it costs nothing in the map.**

    One `<fixed>` tendon per antagonistic pair, one **bidirectional** motor each --
    bidirectional because pull-only is a property of a *cable* and a pair covers
    both directions. Three per leg, **12 on the quadruped**, which is exactly the
    count `params.trunk_mass` has always carried, so the body stays **4.3041 kg**
    where the independent-pair architecture put it at 5.36 (ADR-0056).

    ✅ And the map is ADR-0042's, unchanged and on **both** legs:
    hip **28.000**, knee **-8.750 / 25.000**, ankle **-8.750 / -8.750 / 14.000**.
    """
    from tomcat_kin.params import DEFAULT_BODY_MASS_KG

    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm) * 1e3
    via = MT.VIA_R * 1e3
    for leg_p in (DEFAULT_HINDLEG, DEFAULT_FORELEG):
        m, q = _pulley_leg(leg_p)
        assert m.ntendon == 3 and m.nu == 3
        for i in range(m.nu):
            assert m.actuator_ctrlrange[i][0] < 0.0, "bidirectional"
        G, _ = _pulley_G(m, q)
        J = -G.T * 1e3
        assert J[0, 0] == pytest.approx(arms[0], abs=1e-6)
        assert J[1, 0] == pytest.approx(-via, abs=1e-6)
        assert J[1, 1] == pytest.approx(arms[1], abs=1e-6)
        assert J[2, 0] == pytest.approx(-via, abs=1e-6)
        assert J[2, 1] == pytest.approx(-via, abs=1e-6)
        assert J[2, 2] == pytest.approx(arms[2], abs=1e-6)

    quad = mujoco.MjModel.from_xml_string(MT.quadruped_rig(
        hip_height=0.176, clamped=True, pulley=True, ankle_pair=True))
    assert quad.nu == 12, "ADR-0008's twelve leg motors"
    assert DEFAULT_BODY_MASS_KG == pytest.approx(4.3041, abs=1e-4), (
        "and params' body mass needs no change, which was the point"
    )

    # ⚠️ the price: no co-contraction freedom left
    m, q = _pulley_leg()
    G, _ = _pulley_G(m, q)
    assert G.shape == (3, 3)
    assert G.shape[1] - np.linalg.matrix_rank(G) == 0, (
        "three joints, three motors -- ADR-0002's T_bias has nowhere to live"
    )


def test_the_pulley_plant_holds_at_ONE_AND_A_HALF_NEWTONS():
    """✅ **And it is dramatically cheaper in tension, which nobody predicted.**

    With no co-contraction floor the allocation is a square solve that asks for
    exactly the torque needed. The unloaded leg holds to **0.00°** at every gain
    with a peak cable force of **1.7 N**, against the independent-pair plant's
    **205 N** standing figure (ADR-0049).
    """
    m, q = _pulley_leg()
    G, dof = _pulley_G(m, q)
    A = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "m_L_" + p)
         for p in PULLEY_PAIRS]
    for kp, kd in ((0.0, 0.0), (10.0, 0.2), (50.0, 1.0)):
        d = mujoco.MjData(m)
        for i, a in enumerate(dof):
            d.qpos[a] = q[i]
        peak = 0.0
        for _ in range(20000):
            mujoco.mj_forward(m, d)
            e = np.array([q[i] - d.qpos[a] for i, a in enumerate(dof)])
            ev = np.array([-d.qvel[a] for a in dof])
            tau = wbc.actuator_torque(d, dof, kp * e + kd * ev)
            F = np.clip(np.linalg.solve(G, tau), -MT.TENSION_MAX,
                        MT.TENSION_MAX)
            peak = max(peak, float(np.abs(F).max()))
            for i, a in enumerate(A):
                d.ctrl[a] = float(F[i])
            mujoco.mj_step(m, d)
        drift = np.degrees(np.array([d.qpos[a] for a in dof]) - q)
        assert float(np.max(np.abs(drift))) < 0.01, f"kp={kp}: {drift}"
        assert peak < 10.0, f"kp={kp}: peak {peak:.1f} N"


def test_the_pulley_also_CURES_the_standing_TENSION_SATURATION():
    """✅ **The finding of M53, and it was a side effect.**

    ADR-0056 measured the standing tension ramping to the **222.9 N motor ceiling
    by t = 9 s** and pinning there, tracking an uncontrolled joint drift, and left
    the thermal case blocked behind a posture task.

    On the pulley transmission the same gate runs 12 s with the force **converged at
    68.2 N** -- no ramp, no saturation, and **inside the motor's 81 N continuous
    rating**:

    | | independent pairs | pulley |
    |---|---|---|
    | peak force at 1 s | 125.5 N | 68.2 N |
    | at 9 s | **222.9 N** (ceiling) | **68.2 N** |
    | at 12 s | 222.9 N | **68.2 N** |
    | trunk sag | 0.2 mm | 3.1 mm |
    | tilt | 0.006° | 0.007° |

    ✅ **The wind-up was co-contraction growing with the drift.** Remove
    co-contraction as a state and there is nothing to wind up: the square solve asks
    for the torque and no more. So ADR-0056's blocked thermal item is unblocked by
    the *transmission*, not by the posture task.

    ⚠️ The trunk does sag **3.1 mm against 0.2**, which is the preload the
    co-contraction floor used to provide. That is a real difference and it is the
    honest cost line beside the tension saving.
    """
    q = _quad_poses()
    m = mujoco.MjModel.from_xml_string(MT.quadruped_rig(
        hip_height=0.176, clamped=True, pulley=True, ankle_pair=True))
    d = mujoco.MjData(m)
    for nm in QLEGS:
        for k, a in enumerate(_qadr(m, nm)):
            d.qpos[a] = q[nm][k]
    mujoco.mj_forward(m, d)
    sid = {nm: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"{nm}_foot")
           for nm in QLEGS}
    dof = {nm: _dofs(m, nm) for nm in QLEGS}
    act = {nm: [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                  f"m_{nm}_{p}") for p in PULLEY_PAIRS]
           for nm in QLEGS}
    mass = float(sum(m.body_mass))
    h0 = float(d.subtree_com[0][2])
    om = float(np.sqrt(9.81 / h0))

    def maps():
        out = {}
        for nm in QLEGS:
            tid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON,
                                     f"{nm}_{p}") for p in PULLEY_PAIRS]
            J = np.zeros((3, 3))
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
    early, late = 0.0, 0.0
    for it in range(40000):
        if it and it % REFRESH_STATIC == 0:
            G = maps()
        mujoco.mj_subtreeVel(m, d)
        com = np.array(d.subtree_com[0])
        feet = np.array([d.site_xpos[sid[nm]] for nm in QLEGS])
        w = wbc.desired_wrench(mass, com, np.array(d.subtree_linvel[0]),
                               wbc.realisable_cop(feet, com[:2]), om,
                               damp=6.0, height=h0)
        sg = 1.0 if float(d.qpos[3]) >= 0 else -1.0
        w[3:6] = (-40.0 * 2.0 * sg * np.array([float(v) for v in d.qpos[4:7]])
                  - 4.0 * np.array(d.qvel[3:6]))
        f = wbc.allocate(feet, com, w, 0.8)
        st = wbc.stance_torque(mujoco, m, d, sid,
                               {nm: f[i] for i, nm in enumerate(QLEGS)}, dof)
        step_peak = 0.0
        for nm in QLEGS:
            tau = wbc.actuator_torque(d, dof[nm], st[nm])
            F = np.clip(np.linalg.solve(G[nm], tau), -MT.TENSION_MAX,
                        MT.TENSION_MAX)
            step_peak = max(step_peak, float(np.abs(F).max()))
            for i, a in enumerate(act[nm]):
                d.ctrl[a] = float(F[i])
        if it == 10000:
            early = step_peak
        if it == 39999:
            late = step_peak
        mujoco.mj_step(m, d)
        assert np.all(np.isfinite(d.qpos))

    tilt = float(np.degrees(np.arccos(np.clip(
        1.0 - 2.0 * (d.qpos[4] ** 2 + d.qpos[5] ** 2), -1.0, 1.0))))
    assert float(d.qpos[2]) > 0.17, f"it must still be standing: {d.qpos[2]:.5f}"
    assert tilt < 0.05, f"tilt {tilt:.3f} deg"
    # the force has CONVERGED, which is the whole finding
    assert abs(late - early) < 2.0, (
        f"force must not ramp: {early:.1f} N at 1 s, {late:.1f} at 4 s"
    )
    assert late < MT.TENSION_CONTINUOUS, (
        f"and it must sit inside the {MT.TENSION_CONTINUOUS:.0f} N continuous "
        f"rating, not the 222.9 N ceiling ADR-0056 measured: {late:.1f} N"
    )


# ==========================================================================
# M54 -- the SHIPPED transmission is the default, and it is measured as itself
# ==========================================================================


def _shipped_leg(leg_p=DEFAULT_HINDLEG):
    """The default plant: no keywords, because M54 made the shipped one default."""
    m = mujoco.MjModel.from_xml_string(MT.single_leg_rig(leg_p=leg_p))
    q = np.asarray(LegModel(leg_p).inverse((0.04, -0.17, 0.0)), float)
    return m, q


def _shipped_G(m, q):
    """`-J_tendon^T` for the three pair tendons, measured, not assumed."""
    dof = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in JNT]
    tid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, "L_" + p)
           for p in PULLEY_PAIRS]
    J = np.zeros((3, 3))
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
    return (-J).T, dof


def test_the_DEFAULT_plant_is_now_the_SHIPPED_transmission():
    """✅ **M54's first job: asking for a leg gets you the robot being built.**

    For eleven milestones `single_leg_rig()` handed back the plant ADR-0008
    **rejected** -- a cable routed around each sheave, one pull-only motor per
    tendon. ADR-0055 settled the construction and ADR-0058 settled the actuation,
    and until this milestone neither was what you got by default.

    ⚠️ This test exists because a default is exactly the kind of thing that drifts
    back quietly. It pins the shape, not a number:

    | | legacy | **shipped** |
    |---|---|---|
    | motors per leg | 5 or 6 | **3** |
    | motors, quadruped | 20 or 24 | **12** |
    | direction | pull-only `[0, 223]` | **bidirectional `[-223, 223]`** |
    """
    leg = mujoco.MjModel.from_xml_string(MT.single_leg_rig())
    assert leg.nu == 3 and leg.ntendon == 3, "one motor per antagonistic pair"
    assert [mujoco.mj_id2name(leg, mujoco.mjtObj.mjOBJ_TENDON, i)
            for i in range(leg.ntendon)] == ["L_" + p for p in PULLEY_PAIRS]

    quad = mujoco.MjModel.from_xml_string(MT.quadruped_rig(hip_height=0.176))
    assert quad.nu == 12, "ADR-0008's twelve leg motors, by default"
    for i in range(quad.nu):
        lo, hi = quad.actuator_ctrlrange[i]
        assert lo == pytest.approx(-MT.TENSION_MAX, abs=0.5), "bidirectional"
        assert hi == pytest.approx(+MT.TENSION_MAX, abs=0.5)

    # and the legacy plant is still reachable -- the M42-M51 findings need it
    old = mujoco.MjModel.from_xml_string(_legacy_quad(hip_height=0.176))
    assert old.nu == 20 and old.actuator_ctrlrange[0][0] == 0.0


def test_the_SHIPPED_map_is_CONSTANT_over_the_WHOLE_ROM_on_BOTH_legs():
    """✅ **One measurement retires four separate routing defects.**

    A clamped cable's length is `sum r·q` and a pulley transmits the pair's
    difference, so the shipped leg's tendon Jacobian is **the constant matrix
    `pair_rows` emits** -- not approximately, and not inside a window. Measured at
    five poses spanning each joint's **full ROM**, on both legs, the spread is
    **exactly zero**:

    | | hip | knee | ankle |
    |---|---|---|---|
    | q1 | 28.000 | -8.750 | -8.750 |
    | q2 | 0 | 25.000 | -8.750 |
    | q3 | 0 | 0 | 14.000 |

    Four findings were open against the wrapped build, and this is what closes each:

    - ✅ **the ankle's sign reversal inside the ROM** -- the arm is +14.000
      everywhere, so there is no reversal left to bound anything.
    - ✅ **the fore leg was never mirrored** -- the fore map is *identical* to the
      hind, so there is nothing to mirror.
    - ✅ **the ankle anchor migration** -- unnecessary; it was a fix for a wrap
      that no longer exists.
    - ✅ **"only the ankles were ever validated across the gait"** -- every joint
      is now validated across its whole range, which is more than the audit asked.

    ⚠️ The four tests that measured those defects are pinned to `_legacy_*` and
    still assert them, because they are true of the plant they name. **This test is
    the one that fails if the shipped map ever regresses.**
    """
    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm, float) * 1e3
    via = MT.VIA_R * 1e3
    want = np.array([[arms[0], 0.0, 0.0],
                     [-via, arms[1], 0.0],
                     [-via, -via, arms[2]]])

    for leg_p in (DEFAULT_HINDLEG, DEFAULT_FORELEG):
        m = mujoco.MjModel.from_xml_string(MT.single_leg_rig(leg_p=leg_p))
        lo = np.array([float(leg_p.q_min[i]) for i in range(3)])
        hi = np.array([float(leg_p.q_max[i]) for i in range(3)])
        seen = []
        for f in (0.0, 0.25, 0.5, 0.75, 1.0):
            G, _ = _shipped_G(m, lo + f * (hi - lo))
            seen.append(-G.T * 1e3)
        seen = np.array(seen)
        assert np.allclose(seen[0], want, atol=1e-6), f"map {seen[0]}"
        spread = seen.max(axis=0) - seen.min(axis=0)
        assert np.max(spread) < 1e-9, (
            f"the map must not move across the ROM; spread {spread}"
        )
        # the ankle arm never reverses -- that defect is gone
        assert np.all(seen[:, 2, 2] > 0.0)

    # both legs give the SAME map -- the mirroring defect is moot
    Gh, _ = _shipped_G(*_shipped_leg(DEFAULT_HINDLEG))
    Gf, _ = _shipped_G(*_shipped_leg(DEFAULT_FORELEG))
    assert np.allclose(Gh, Gf, atol=1e-9)


def test_the_DRIVETRAIN_now_exists_BEHIND_THE_PULLEY_and_so_does_G3():
    """✅ **M54's real discovery: G3 had no home on the robot that ships.**

    ADR-0051 put the series-elastic element **in the drivetrain**, between rotor and
    spool -- that is where the compliance physically sits. But the drivetrain named
    its spools per *cable* (`L_hip_flex`), and the shipped plant's tendons are the
    three *pairs* (`L_hip`). Asking for one on the other failed with MuJoCo's
    `unknown element 'L_hip_flex'`, which names the symptom and not the cause.

    So every drivetrain result -- M46's exact statics, M47's derived cascade, the
    rotor servo -- stood on a transmission ADR-0058 had already replaced, and
    **design goal G3 had nowhere to live on the shipped robot**. That is not a
    number that needed re-deriving; it is a capability that did not exist.

    ✅ **It exists now.** One spool per pair, because with a variable-radius pulley
    the spool *is* the pulley: it takes up one cable while paying out the other, so
    what winds on it is their difference -- exactly the `<fixed>` tendon. Half the
    bodies, half the constraints, and M46's two-pass `a0` still lands the winding
    equality on its reference pose to **4e-10 m**.

    | | legacy | **shipped** |
    |---|---|---|
    | motors | 6 | **3** |
    | tendons (pair + wind) | 12 | **6** |
    | winding equalities | 6 | **3** |
    | bodies | 18 | **12** |
    """
    q = list(LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0)))
    m = mujoco.MjModel.from_xml_string(
        MT.single_leg_rig_spooled(q_ref=q, series_k=SERIES_K))

    assert m.nu == 3, "one motor per pair, behind a real drivetrain"
    assert m.ntendon == 6, "three pair tendons and three winding tendons"
    assert m.neq == 3, "one winding equality per pair"

    old = mujoco.MjModel.from_xml_string(
        _legacy_spooled(q_ref=q, series_k=SERIES_K, ankle_pair=True))
    assert (old.nu, old.ntendon, old.neq, old.nbody) == (6, 12, 6, 18)
    assert m.nbody == 12, "six fewer bodies: the spools that are no longer separate"

    # ✅ G3 is present, and it is the torsional spring ADR-0051 sized
    k_tors = [float(m.jnt_stiffness[i]) for i in range(m.njnt)
              if m.jnt_stiffness[i] > 0.0]
    assert len(k_tors) == 3, "one series-elastic element per pair"
    assert k_tors[0] == pytest.approx(SERIES_K * MT.SPOOL_R ** 2, rel=1e-9), (
        "G3 is k_series * r_spool^2 at the rotor"
    )

    # ⚠️ M46's trap: the equality is referenced at qpos0, so the offset must land
    d = mujoco.MjData(m)
    for i, jn in enumerate(JNT):
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
        d.qpos[m.jnt_qposadr[j]] = q[i]
    mujoco.mj_forward(m, d)
    assert float(np.max(np.abs(d.efc_pos[:d.nefc]))) < 1e-8, (
        "the two-pass a0 must satisfy the winding equality at the reference pose"
    )


def test_the_PULL_ONLY_ALLOCATOR_silently_DROPS_JOINTS_on_the_shipped_plant():
    """⚠️ **The retraction M54 owes: every co-contraction-floor result is void.**

    `wbc.tendon_tension` solves for pull-only cables -- it substitutes
    `T = t_min + u, u >= 0` and runs NNLS, so no component can go negative. On the
    shipped plant that constraint is **not physical**: ADR-0058 put one
    *bidirectional* motor on each pair, and it drives either way.

    The cost is not a rounding error. At the stance pose, given the gravity torque
    the leg actually needs:

    | | needed | NNLS delivers | |
    |---|---|---|---|
    | hind ankle | 0.0114 N·m | **0** | the whole joint |
    | fore knee | 0.0492 N·m | 0.0076 | **15 %** |

    It clamps to zero a command the motor could deliver, and the shortfall shows up
    only in a residual nobody was checking. ✅ The signed solve is exact -- square,
    unique, zero residual -- which is `wbc.pair_command`.

    ⚠️ **And so `t_min` has nowhere left to act.** The co-contraction floor was a
    coordinate of the *redundant* per-cable plant, and ADR-0058 spent that
    redundancy. Every M42-M47 result that rested on choosing a floor describes a
    machine this project is no longer building.
    """
    from tomcat_kin import wbc

    for leg_p, joint, idx in ((DEFAULT_HINDLEG, "ankle", 2),
                              (DEFAULT_FORELEG, "knee", 1)):
        m, q = _shipped_leg(leg_p)
        G, dof = _shipped_G(m, q)
        d = mujoco.MjData(m)
        for i, a in enumerate(dof):
            d.qpos[a] = q[i]
        mujoco.mj_forward(m, d)
        tau = np.array([d.qfrc_bias[a] - d.qfrc_passive[a] for a in dof])

        signed = wbc.pair_command(G, tau, MT.TENSION_MAX)
        assert np.linalg.norm(G @ signed - tau) < 1e-12, (
            "square and signed: there is one answer and it is exact"
        )
        assert np.any(signed < 0.0), (
            "and it needs a negative command, which is why pull-only cannot serve"
        )

        pull_only = wbc.tendon_tension(G, tau, 0.0, MT.TENSION_MAX)
        assert np.all(pull_only >= 0.0)
        short = (G @ pull_only - tau)[idx]
        assert abs(short) > 0.2 * abs(tau[idx]), (
            f"the pull-only allocator must visibly fail the {joint}: "
            f"short {short:.4f} of {tau[idx]:.4f} N*m"
        )

    # ⚠️ a floor cannot even be expressed: the solve is square
    m, q = _shipped_leg()
    G, _ = _shipped_G(m, q)
    assert G.shape == (3, 3) and np.linalg.matrix_rank(G) == 3
    tau = np.array([0.05, -0.04, 0.01])
    assert np.allclose(wbc.pair_command(G, tau, MT.TENSION_MAX),
                       wbc.pair_command(G, tau, MT.TENSION_MAX)), (
        "no free coordinate means no choice to make"
    )


# ==========================================================================
# M55 -- the cascade, re-derived on the drivetrain M54 made buildable
# ==========================================================================

SHIP_PAIRS = ("hip", "knee", "ankle")


def _ship_spooled(servo=True):
    """The shipped drivetrain: clamped capstans, one spool per pair, rotor servo."""
    q = np.asarray(LegModel(DEFAULT_HINDLEG).inverse((0.04, -0.17, 0.0)), float)
    return mujoco.MjModel.from_xml_string(MT.single_leg_rig_spooled(
        q_ref=q, series_k=SERIES_K, spool_servo=servo)), q


def _ship_idx(m):
    A = [_adr(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "m_L_" + p) for p in SHIP_PAIRS]
    JR = [m.jnt_qposadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, "jr_L_" + p)]
          for p in SHIP_PAIRS]
    JS = [m.jnt_qposadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, "js_L_" + p)]
          for p in SHIP_PAIRS]
    TP = [_adr(m, mujoco.mjtObj.mjOBJ_TENDON, "L_" + p) for p in SHIP_PAIRS]
    qa = [m.jnt_qposadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JNT]
    dof = [m.jnt_dofadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JNT]
    return A, JR, JS, TP, qa, dof


def _ship_run(m, q, kp=50.0, kd=1.0, dq3_deg=0.0, ramp_s=None, hold_s=0.5,
              pull_only=False, refresh=REFRESH_MOVING):
    """Drive the shipped cascade. `ramp_s=None` means a step, which is M47's test.

    Returns `(final_error_deg, peak_raw_force, saturated_fraction, worst_error)`
    or `None` if it diverged. `peak_raw` is the force BEFORE clipping, so a demand
    past the motor is visible rather than hidden by the clamp.
    """
    A, JR, JS, TP, qa, dof = _ship_idx(m)
    d = mujoco.MjData(m)
    for i, a in enumerate(qa):
        d.qpos[a] = q[i]
    mujoco.mj_forward(m, d)

    def jac():
        J = np.zeros((3, 3))
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
    dt = m.opt.timestep
    total = (ramp_s + hold_s) if ramp_s else 2.0
    n = int(total / dt)
    peak_raw, sat, worst = 0.0, 0, 0.0
    for it in range(n):
        if it and it % refresh == 0:
            G = jac()
        t = it * dt
        f = min(1.0, t / ramp_s) if ramp_s else 1.0
        qd = q.copy()
        qd[2] = q[2] + math.radians(dq3_deg) * f
        e = np.array([qd[i] - d.qpos[a] for i, a in enumerate(qa)])
        ev = np.array([-d.qvel[a] for a in dof])
        tau = wbc.actuator_torque(d, dof, kp * e + kd * ev)
        if pull_only:
            T = raw = wbc.tendon_tension(G, tau, 19.6, MT.TENSION_MAX)
        else:
            raw = wbc.pair_command(G, tau, np.inf)
            T = np.clip(raw, -MT.TENSION_MAX, MT.TENSION_MAX)
        if np.max(np.abs(raw)) > MT.TENSION_MAX - 1e-9:
            sat += 1
        peak_raw = max(peak_raw, float(np.max(np.abs(raw))))
        if not ramp_s or t > ramp_s * 0.2:
            worst = max(worst, float(np.max(np.abs(np.degrees(e)))))
        cmd = wbc.rotor_command([d.qpos[j] for j in JR], [d.qpos[j] for j in JS],
                                T, K_TORS, MT.SPOOL_R, servo_kp=SERVO_KP)
        for i, a in enumerate(A):
            d.ctrl[a] = float(cmd[i])
        mujoco.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)):
            return None
    qd = q.copy()
    qd[2] = q[2] + math.radians(dq3_deg)
    err = np.degrees(np.array([float(d.qpos[a]) for a in qa]) - qd)
    return err, peak_raw, sat / n, worst


def _lowest_mode_hz(m, q):
    """The lowest oscillatory mode of the linearised drivetrain, in Hz."""
    d = mujoco.MjData(m)
    for i, n in enumerate(JNT):
        d.qpos[m.jnt_qposadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, n)]] = q[i]
    mujoco.mj_forward(m, d)
    A = np.zeros((2 * m.nv, 2 * m.nv))
    B = np.zeros((2 * m.nv, m.nu))
    mujoco.mjd_transitionFD(m, d, 1e-6, 1, A, B, None, None)
    with np.errstate(divide="ignore", invalid="ignore"):
        s = np.log(np.linalg.eigvals(A).astype(complex)) / m.opt.timestep
    f = np.abs(s.imag) / (2.0 * math.pi)
    return float(np.min(f[f > 1.0]))


def test_the_CASCADE_TRANSFERS_to_the_pulley_drivetrain_UNCHANGED():
    """✅ **M47's cascade holds on the shipped plant with its gains untouched.**

    `kp = 50, kd = 1.0` -- derived in M47, not tuned -- holds the stance pose to
    **0.00°** on the pulley drivetrain, at a peak commanded force of **3.2 N**.

    ⚠️ **This corrects a prediction M54 published.** ADR-0059 said the gains would
    have to be *re-derived rather than re-pointed*, because one spool per pair
    changes the reflected inertia. The inertia claim was right -- see
    `test_ONE_SPOOL_PER_PAIR_HALVES_the_lowest_drivetrain_MODE` -- but the
    conclusion was wrong: M47 chose its outer loop far below both plants' modes, so
    it was never close enough to the moved one to care. The re-derivation
    **confirms** the gains rather than replacing them.

    ✅ The inner loop needed no thought at all: `kp = I*wn^2` is a property of the
    **rotor**, and every motor still has exactly one rotor.
    """
    m, q = _ship_spooled()
    assert m.nu == 3 and m.neq == 3

    out = _ship_run(m, q, kp=50.0, kd=1.0)
    assert out is not None, "the cascade must not diverge on the shipped plant"
    err, peak, sat, _ = out
    assert np.max(np.abs(err)) < 0.01, f"holds to {err} deg"
    assert sat == 0.0, "and it does it without ever asking past the motor"
    assert peak < 10.0, f"peak commanded force {peak:.1f} N"

    # the inner loop is rotor-local, so it cannot have moved
    assert SERVO_KP == pytest.approx(MT.ROTOR_ARMATURE * MT.ROTOR_BANDWIDTH ** 2)
    assert SERVO_KP == pytest.approx(180.0, rel=1e-6)


def test_ONE_SPOOL_PER_PAIR_HALVES_the_lowest_drivetrain_MODE():
    """⚠️ **What the pulley actually cost, and nobody had measured it.**

    ADR-0058 was decided on mass and ADR-0059 built the drivetrain behind it. The
    dynamics were never checked. Linearising both plants about the stance pose:

    | | lowest mode | usable outer `kp` |
    |---|---|---|
    | legacy, one spool per cable | 54.9 Hz | holds to **600** |
    | **shipped, one spool per pair** | **27.4 Hz** | holds to **200** |

    The lowest drivetrain mode **halves**, and the outer loop's usable gain range
    falls with it -- from 16× M47's chosen `kp` down to **6×**. That is real
    headroom spent, and it is the price of the transmission that closed the mass
    budget.

    ✅ It is still headroom, not a wall: M47's `kp = 50` sits well inside, which is
    why `test_the_CASCADE_TRANSFERS_to_the_pulley_drivetrain_UNCHANGED` passes.
    ⚠️ But anything that wants a stiffer joint loop -- a landing, a disturbance
    rejection task -- now has a third of the room it used to.
    """
    ship, q = _ship_spooled()
    legacy = mujoco.MjModel.from_xml_string(
        _legacy_spooled(q_ref=q, series_k=SERIES_K, ankle_pair=True,
                        spool_servo=True))

    f_ship = _lowest_mode_hz(ship, q)
    f_legacy = _lowest_mode_hz(legacy, q)
    assert f_legacy == pytest.approx(54.9, abs=1.5)
    assert f_ship == pytest.approx(27.4, abs=1.5)
    assert f_ship < 0.6 * f_legacy, (
        f"the lowest mode must have roughly halved: {f_legacy:.1f} -> {f_ship:.1f}"
    )

    # and the outer loop's edge moved with it
    ok = _ship_run(ship, q, kp=200.0, kd=2.83)
    assert ok is not None and np.max(np.abs(ok[0])) < 0.1, "kp=200 still holds"
    bad = _ship_run(ship, q, kp=300.0, kd=3.46)
    assert bad is None or np.max(np.abs(bad[0])) > 0.5, (
        "kp=300 must not hold -- that is the headroom the pulley spent"
    )


def test_the_ALLOCATOR_not_the_gains_was_what_had_to_CHANGE():
    """⚠️ **M54's static retraction, shown dynamically: the leg collapses.**

    M54 measured `wbc.tendon_tension` dropping whole joints on the shipped plant --
    zero of the hind ankle's torque. Run the cascade with it and the consequence is
    not a residual, it is a **fall**: same plant, same gains, same reference.

    | allocator | hold error | peak force |
    |---|---|---|
    | `pair_command`, signed | **0.00 / 0.00 / 0.00°** | 3.2 N |
    | `tendon_tension`, pull-only | **-70.8 / -47.2 / -127.1°** | 32.5 N |

    ✅ So the answer to *"what had to be re-derived for the cascade?"* is: **not
    the gains -- the allocator.** Pull-only is a property of a cable, and there is
    no longer one motor per cable to be pulled.
    """
    m, q = _ship_spooled()

    good = _ship_run(m, q, kp=50.0, kd=1.0, pull_only=False)
    assert good is not None and np.max(np.abs(good[0])) < 0.01

    bad = _ship_run(m, q, kp=50.0, kd=1.0, pull_only=True)
    assert bad is not None, "it does not blow up -- it quietly falls"
    assert np.max(np.abs(bad[0])) > 40.0, (
        f"the pull-only allocator must visibly lose the leg: {bad[0]} deg"
    )
    assert abs(bad[0][2]) > 100.0, "and the ankle is where it goes first"


def test_the_ankle_TRACKS_EXACTLY_and_the_bound_is_now_the_JOINT_LIMIT():
    """✅ **M47's ankle finding, re-derived -- and its test was measuring the
    wrong thing.**

    M47 reported *"the cascade tracks the hind ankle but the reversal still bounds
    it"*, from a **step** command. ⚠️ Re-run with the raw force logged, that step
    demands **3700-4300 N** against a 222.9 N motor and saturates **98-100 %** of
    every timestep -- on both plants. It was not measuring tracking; it was
    measuring where a saturated bang-bang controller comes to rest. Both its
    numbers are artefacts.

    ✅ **Ramp the reference instead** -- which is what a gait does -- and the
    shipped cascade tracks the ankle *exactly*:

    | ° commanded | final error | peak force | saturation |
    |---|---|---|---|
    | +20 | **0.00°** | 3.1 N | none |
    | +40 | **0.00°** | 3.1 N | none |
    | +52 | **0.00°** | 3.1 N | none |
    | +55 | -2.06° | 130 N | none |

    +55 misses by **exactly** its overshoot of the 150° end stop (the stance ankle
    sits at 97.06°, so the headroom is 52.94°). **The bound is the joint limit
    now, not the moment arm** -- ADR-0049's reversal is gone, and what replaces it
    is a number from `params`.

    ✅ And the lag is first-order with no saturation anywhere: the worst error
    during the ramp falls **0.81 → 0.41 → 0.20 → 0.10°** as the ramp
    is stretched 0.5 → 1 → 2 → 4 s. Halve the speed, halve the error.

    ⚠️ The legacy plant cannot do this at all: the same 2 s ramp to +20° leaves it
    **36.3°** out, saturated 44 % of the time.
    """
    m, q = _ship_spooled()
    q3_deg = math.degrees(q[2])
    headroom = 150.0 - q3_deg
    assert headroom == pytest.approx(52.94, abs=0.05)

    for dq in (20.0, 40.0, 52.0):
        err, peak, sat, _ = _ship_run(m, q, dq3_deg=dq, ramp_s=2.0)
        assert np.max(np.abs(err)) < 0.01, f"+{dq} deg -> {err}"
        assert sat == 0.0 and peak < 10.0, f"+{dq} deg asked {peak:.1f} N"

    # past the end stop it misses by exactly the overshoot, not by more
    err, _, _, _ = _ship_run(m, q, dq3_deg=55.0, ramp_s=2.0)
    assert err[2] == pytest.approx(-(55.0 - headroom), abs=0.05), (
        f"the bound is the 150 deg joint limit, not the routing: {err}"
    )

    # the lag is first-order: stretching the ramp divides the error
    _, _, _, fast = _ship_run(m, q, dq3_deg=40.0, ramp_s=1.0)
    _, _, _, slow = _ship_run(m, q, dq3_deg=40.0, ramp_s=4.0)
    assert fast / slow == pytest.approx(4.0, rel=0.25), (
        f"worst error must scale with ramp rate: {fast:.3f} vs {slow:.3f}"
    )

    # ⚠️ and M47's step was saturated, which is why its numbers meant nothing
    _, step_peak, step_sat, _ = _ship_run(m, q, dq3_deg=20.0, ramp_s=None)
    assert step_peak > 1000.0, f"the step demanded {step_peak:.0f} N"
    assert step_sat > 0.9, f"saturated {100 * step_sat:.0f} % of the horizon"


# ==========================================================================
# M56 -- the variable-radius profile: what it could buy, measured
# ==========================================================================

#: Trot samples per leg for the demand survey. 60 is enough to resolve the shape;
#: the full 200 changes no figure below by more than 0.1 N.
TROT_N = 60


def _trot_traj(n=TROT_N):
    """One trot cycle of joint angles per leg, with the stance flag."""
    from tomcat_kin import gait

    c = gait.GaitController(gait.trot_params())
    out = {}
    for st in c.sample_cycle(n):
        for nm, ls in st.legs.items():
            if ls.q is not None:
                out.setdefault(nm, []).append(
                    (np.asarray(ls.q, float), bool(ls.in_stance)))
    return out


def _leg_rig(leg_p):
    m = mujoco.MjModel.from_xml_string(MT.single_leg_rig(leg_p=leg_p))
    dof = [m.jnt_dofadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JNT]
    qa = [m.jnt_qposadr[_adr(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JNT]
    tid = [_adr(m, mujoco.mjtObj.mjOBJ_TENDON, "L_" + p) for p in PULLEY_PAIRS]
    sid = _adr(m, mujoco.mjtObj.mjOBJ_SITE, "L_foot")
    return m, dof, qa, tid, sid


def _pair_force(rig, q, stance, foot_load):
    """Motor force per pair, quasi-static, with a vertical foot load in stance."""
    m, dof, qa, tid, sid = rig
    d = mujoco.MjData(m)
    for i, a in enumerate(qa):
        d.qpos[a] = q[i]
    mujoco.mj_forward(m, d)
    tau = np.array([d.qfrc_bias[a] - d.qfrc_passive[a] for a in dof])
    if stance:
        jacp = np.zeros((3, m.nv))
        mujoco.mj_jacSite(m, d, jacp, None, sid)
        tau = tau - jacp[2, dof] * foot_load
    J = np.zeros((3, 3))
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
    return wbc.pair_command((-J).T, tau, np.inf)


def _load_split():
    """Fraction of body weight on the FORE pair at the stance pose, measured."""
    from tomcat_kin.params import DEFAULT_FORELEG

    lp = {"LF": DEFAULT_FORELEG, "RF": DEFAULT_FORELEG,
          "LR": DEFAULT_HINDLEG, "RR": DEFAULT_HINDLEG}
    m = mujoco.MjModel.from_xml_string(MT.quadruped_rig(hip_height=0.176))
    d = mujoco.MjData(m)
    for nm, p in lp.items():
        q = LegModel(p).inverse((0.04, -0.17, 0.0))
        for i in range(3):
            j = _adr(m, mujoco.mjtObj.mjOBJ_JOINT, f"{nm}_q{i + 1}")
            d.qpos[m.jnt_qposadr[j]] = q[i]
    mujoco.mj_forward(m, d)
    feet = {}
    for i in range(m.nsite):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SITE, i)
        if n and n.endswith("_foot"):
            feet[n] = d.site_xpos[i].copy()
    fx = np.mean([p[0] for n, p in feet.items() if n[1] == "F"])
    rx = np.mean([p[0] for n, p in feet.items() if n[1] == "R"])
    return float((d.subtree_com[0][0] - rx) / (fx - rx)), float(
        m.body_subtreemass[0])


def test_the_TROT_LOAD_SPLIT_is_REAR_biased_and_params_SAID_SO():
    """⚠️ **Get this wrong and the whole tension survey names the wrong joint.**

    A trot puts one fore and one hind foot down, each carrying its girdle's share,
    so the split decides which leg is worked hardest. Measured on the shipped
    quadruped at the stance pose: **30.2 % fore, 69.8 % hind** -- 12.75 N on the
    fore foot, 29.47 N on the hind.

    ✅ **`params.py` predicted exactly this, and had already retracted the
    opposite.** Review finding F2 records that the original budget *tuned girdle
    masses to hit a 60/40 front-heavy split*, that the split is properly an
    **output** of where the hardware sits, and that the motors do not sit forward
    -- ADR-0005 puts more of them on the pelvis.

    ⚠️ Assume 50/50 and the survey reports the **fore knee** as the pair over its
    rating; assume the discredited 60/40 fore-bias and it reports the fore knee and
    ankle. Both are wrong, and both point at the wrong leg.
    """
    frac_fore, mass = _load_split()
    assert frac_fore == pytest.approx(0.302, abs=0.01), (
        f"fore pair carries {100 * frac_fore:.1f} %"
    )
    assert frac_fore < 0.5, "rear-biased, as params F2 says it must be"

    W = mass * 9.81
    assert W * frac_fore == pytest.approx(12.75, abs=0.3)
    assert W * (1.0 - frac_fore) == pytest.approx(29.47, abs=0.5)


def test_the_GAIT_holds_the_PAW_FLAT_so_the_ANKLE_DEMAND_HAS_NO_SHAPE():
    """✅ **The fact that decides M56, and it is exact rather than approximate.**

    Through the whole stance phase the gait holds `q1 + q2 + q3` at **-55.0000°**
    -- span **8.5e-14°**. That is the paw's *absolute* orientation, and holding it
    fixed is what keeps the foot flat on the ground while the body passes over it.

    ⚠️ **The consequence for a variable-radius pulley is fatal.** A profile can only
    exploit a demand that *varies with joint angle*. With the paw held flat, the
    ankle's torque is a **constant** through stance -- measured, the hind ankle's
    motor force is 136.338 N at every sample, standard deviation **4e-12 N**, so
    peak/mean is **1.0000**. There is no shape for a profile to remove.
    """
    traj = _trot_traj()
    stance = [q for q, ins in traj["LR"] if ins]
    total = np.degrees([q.sum() for q in stance])
    assert total.max() - total.min() < 1e-9, (
        f"the paw must stay flat: span {total.max() - total.min():.2e} deg"
    )
    assert total.mean() == pytest.approx(-55.0, abs=1e-6)

    rig = _leg_rig(DEFAULT_HINDLEG)
    frac_fore, mass = _load_split()
    load = (1.0 - frac_fore) * mass * 9.81
    f = np.abs([_pair_force(rig, q, True, load)[2] for q in stance])
    assert f.std() < 1e-9, f"the ankle demand must be constant: std {f.std():.2e}"
    assert f.max() / f.mean() == pytest.approx(1.0, abs=1e-9)


def test_the_ONE_PAIR_over_its_CONTINUOUS_rating_is_the_HIND_ANKLE():
    """⚠️ **One pair of six is over, thermally. Nothing is close structurally.**

    Quasi-static motor force over one trot cycle, at the measured 30/70 split:

    | | peak | RMS | RMS / 81.1 N | peak / 222.9 N |
    |---|---|---|---|---|
    | hind hip | 106.1 | 43.0 | 0.53 | 0.48 |
    | hind knee | 32.6 | 15.5 | 0.19 | 0.15 |
    | **hind ankle** | 136.3 | **96.4** | **1.19** | 0.61 |
    | fore hip | 74.4 | 35.4 | 0.44 | 0.33 |
    | fore knee | 89.0 | 49.9 | 0.62 | 0.40 |
    | fore ankle | 67.1 | 47.5 | 0.59 | 0.30 |

    ✅ **Structurally there is no case at all**: the worst peak is 61 % of the
    motor's peak rating. ⚠️ **Thermally there is exactly one**: the hind ankle at
    **1.19×** the continuous rating -- and RMS, not peak, is what heats a motor.

    ⚠️ This is a quasi-static survey: gravity plus a vertical foot load, no
    inertial or horizontal terms. It bounds the *shape* of the demand, which is what
    M56 needs; it is not a duty-cycle model.
    """
    from tomcat_kin.params import DEFAULT_FORELEG

    traj = _trot_traj()
    frac_fore, mass = _load_split()
    W = mass * 9.81
    want = {("LR", 0): 0.53, ("LR", 1): 0.19, ("LR", 2): 1.19,
            ("LF", 0): 0.44, ("LF", 1): 0.62, ("LF", 2): 0.59}

    over, worst_pk = [], 0.0
    for nm, leg_p, load in (("LR", DEFAULT_HINDLEG, (1 - frac_fore) * W),
                            ("LF", DEFAULT_FORELEG, frac_fore * W)):
        rig = _leg_rig(leg_p)
        F = np.array([_pair_force(rig, q, ins, load) for q, ins in traj[nm]])
        for j in range(3):
            f = np.abs(F[:, j])
            rms = float(np.sqrt((f ** 2).mean()))
            worst_pk = max(worst_pk, f.max() / MT.TENSION_MAX)
            assert rms / MT.TENSION_CONTINUOUS == pytest.approx(
                want[(nm, j)], abs=0.06), (
                f"{nm} {PULLEY_PAIRS[j]}: rms {rms:.1f} N"
            )
            if rms > MT.TENSION_CONTINUOUS:
                over.append((nm, PULLEY_PAIRS[j], rms))

    assert worst_pk < 0.7, f"structurally clear: worst peak {worst_pk:.2f} of max"
    assert len(over) == 1 and over[0][:2] == ("LR", "ankle"), (
        f"exactly one pair over its continuous rating: {over}"
    )


def test_the_VARIABLE_RADIUS_PROFILE_CANNOT_BUY_what_ADR0002_wanted():
    """⚠️ **M56's verdict: do not design the profile. Measured yield is zero.**

    [ADR-0002](../docs/DESIGN_DECISIONS.md) justified commandable co-contraction on
    *"Kengoro AIC, which cut peak tendon tension 43→28 kgf"*, and
    [ADR-0058](../docs/DESIGN_DECISIONS.md) speculated the benefit *"may be
    recoverable in the radius profile"*. Both are wrong, for different reasons.

    ⚠️ **The cited mechanism does not apply.** AIC is a *control rule* -- hold the
    antagonist at `T_bias` while the agonist works -- and its 43→28 kgf is
    measured against a **fixed high co-contraction**. The shipped pulley cannot
    co-contract at all, so it is already at that optimum. There is nothing left for
    an AIC-like schedule to remove.

    ⚠️ **And the mechanism that does apply pays nothing here.** A variable radius is
    a *gear ratio that varies with joint angle*: it helps where the demand is
    peaked, by trading speed for force at the peak. Measured:

    | | peak/mean in stance | RMS vs rating | what a profile buys |
    |---|---|---|---|
    | **hind ankle** (the one over) | **1.0000** | **1.19×** | **0 %** |
    | hind hip | 1.99 | 0.53× | nothing needed |
    | fore knee | 1.29 | 0.62× | nothing needed |

    The one pair that needs help has **no shape at all**, because the gait holds the
    paw flat. The pairs with shape have 1.6→5× of margin already. ⚠️ And RMS
    is dominated by the *mean*, which a profile does not change -- flattening the
    most-peaked pair in the survey moves its RMS by under 2 %.

    ✅ **What does fix the hind ankle is a constant: 14.0 → 16.6 mm**, a
    parameter and a sheave, not a profile. ⚠️ It is left unspent here because the
    arms are shared between legs and the hind leg's other pairs have 5× margin,
    so the right change is a **per-leg** arm -- a mechanical decision with a
    drawing attached, not a simulation one.
    """
    traj = _trot_traj()
    frac_fore, mass = _load_split()
    load = (1.0 - frac_fore) * mass * 9.81
    rig = _leg_rig(DEFAULT_HINDLEG)

    F = np.array([_pair_force(rig, q, ins, load) for q, ins in traj["LR"]])
    stance = np.array([ins for _, ins in traj["LR"]])
    f = np.abs(F[:, 2])
    rms = float(np.sqrt((f ** 2).mean()))
    assert rms / MT.TENSION_CONTINUOUS == pytest.approx(1.19, abs=0.06)

    # a PERFECT profile flattens the stance demand to its mean. Here that is a
    # no-op, because the demand already IS its mean.
    flat = f.copy()
    flat[stance] = f[stance].mean()
    rms_flat = float(np.sqrt((flat ** 2).mean()))
    assert rms_flat == pytest.approx(rms, rel=1e-9), (
        "the profile's yield on the binding pair must be exactly zero"
    )

    # where shape exists, there is already margin -- so nothing to spend it on
    hip = np.abs(F[:, 0])
    assert hip[stance].max() / hip[stance].mean() > 1.8, "the hip IS peaked"
    assert float(np.sqrt((hip ** 2).mean())) < 0.6 * MT.TENSION_CONTINUOUS, (
        "but it is nowhere near its rating, so flattening it buys nothing"
    )

    # and the fix that does work is a constant arm
    r_ankle = float(DEFAULT_TENDON.joint_moment_arm[2]) * 1e3
    assert r_ankle == pytest.approx(14.0, abs=1e-6)
    assert r_ankle * rms / MT.TENSION_CONTINUOUS == pytest.approx(16.6, abs=0.6)
