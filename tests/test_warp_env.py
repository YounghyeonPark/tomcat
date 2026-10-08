"""M136 / ADR-0130: the standing task batched on the GPU (MuJoCo-Warp).

Skipped where there is no `mujoco_warp` or no CUDA device."""

from __future__ import annotations

import numpy as np
import pytest

wp = pytest.importorskip("warp")
pytest.importorskip("mujoco_warp")
wp.init()
if not wp.get_cuda_devices():
    pytest.skip("no CUDA device", allow_module_level=True)

from tomcat_kin.env import TomcatEnv  # noqa: E402
from tomcat_kin.stand_baseline import StandBaseline  # noqa: E402
from tomcat_kin.warp_env import WarpStand  # noqa: E402

DR = {**TomcatEnv.RANGES, **TomcatEnv.SENSOR_RANGES}


def test_the_GPU_env_IS_the_CPU_env_step_for_step():
    """✅ Without randomisation, every world of `WarpStand` follows the CPU
    `TomcatEnv` under the same baseline: measured over a whole 4 s episode,
    tilt within 0.005 deg and reward to ~1e-3 (float32). The servo, the delay ring, the
    sensor model, the joint estimate and the baseline are all in that."""
    W = WarpStand(4, residual_n=30.0)
    cpu = TomcatEnv(rng=np.random.default_rng(0))
    o = cpu.reset()
    b = StandBaseline(cpu).reset(o)
    for i in range(150):
        _obs, r, te, _tr, info = W.step(np.zeros((4, 18)))
        o, rc, dc, ic = cpu.step(b(o))
        assert not dc and not np.any(te)
        assert np.allclose(info["tilt_deg"], ic["tilt_deg"], atol=0.02), i
        assert np.allclose(r, rc, atol=5e-3), i


def test_each_world_draws_its_OWN_plant_and_it_BITES():
    """Per-world randomisation reaches the physics: worlds that drew different
    masses and springs part ways under the same command."""
    W = WarpStand(8, randomize=DR, residual_n=30.0, seed=3)
    assert np.ptp(W.draw["mass"]) > 0.05 and np.ptp(W.draw["k"]) > 0.1
    assert len(set(W.draw["delay"].tolist())) > 1
    tilts = []
    for _ in range(40):
        _o, _r, _te, _tr, info = W.step(np.zeros((8, 18)))
        tilts.append(info["tilt_deg"])
    assert np.ptp(np.array(tilts)[-1]) > 0.1, "the worlds must not move as one"


def test_a_FINISHED_world_restarts_and_the_others_carry_on():
    W = WarpStand(4, residual_n=30.0)
    # throw world 2 over with its residual pinned; the others hold still
    a = np.zeros((4, 18))
    a[2] = 1.0
    for _ in range(300):
        _o, _r, te, tr, info = W.step(a)
        if te[2]:
            break
    assert te[2], "a saturated residual should fell world 2"
    assert W.steps[2] == 0 and np.all(W.steps[[0, 1, 3]] > 0)
    assert "terminal_obs" in info and list(info["done_idx"]) == [2]


def test_the_WALK_task_sees_a_clock_and_a_command_and_scores_true_speed():
    """M137 (ADR-0131): 67 channels -- the standing 64, the gait clock and the
    commanded speed -- and a reward that peaks where the true forward speed
    meets the command."""
    from tomcat_kin.warp_env import WarpWalk
    W = WarpWalk(4, v_range=(0.2, 0.4), seed=1)
    o = W.reset()
    assert o.shape == (4, 67)
    assert np.all((W.vcmd >= 0.2) & (W.vcmd <= 0.4))
    assert np.allclose(o[:, -1], W.vcmd)
    assert np.allclose(o[:, -3] ** 2 + o[:, -2] ** 2, 1.0, atol=1e-5)
    _o, r, _te, _tr, _i = W.step(np.zeros((4, 18)))
    assert np.all(np.isfinite(r)) and np.all(np.abs(W.vx) < 0.2), "standing still"


def test_the_TROT_term_rewards_the_diagonals_in_phase_with_the_clock():
    """M138 (ADR-0132): four feet down scores 0.5 at any phase; the clock's
    diagonal down and the other up scores 1; the wrong diagonal scores 0."""
    from tomcat_kin.warp_env import WarpWalk
    W = WarpWalk(2, gait_w=1.0)
    W.steps[:] = 0                                    # phase 0: LF + RR stance
    o = {"contact": np.array([[5.0, 5.0, 5.0, 5.0], [5.0, 0.0, 0.0, 5.0]])}
    assert np.allclose(W.trot_match(o), [0.5, 1.0])
    o = {"contact": np.array([[0.0, 5.0, 5.0, 0.0], [5.0, 0.0, 0.0, 5.0]])}
    assert np.allclose(W.trot_match(o), [0.0, 1.0])
    W.steps[:] = int(round(0.6 * W.period * W.control_hz))   # second half
    assert np.allclose(W.trot_match(o), [1.0, 0.0])


def test_the_FEET_terms_see_slip_and_lift_and_a_reset_is_not_a_slip():
    """M139 (ADR-0133): standing still, the loaded feet barely move and the
    swing feet are on the floor (clearance ~0); right after a reset the feet
    have jumped, and that is not charged as slip."""
    from tomcat_kin.warp_env import WarpWalk
    W = WarpWalk(4, slip_w=2.0, clear_w=0.5)
    for _ in range(20):
        W.step(np.zeros((4, 18)))
    slip, clear = W.last_feet
    assert np.all(slip < 0.02), slip
    assert np.all(clear < 0.1), clear
    W._reset_worlds(np.array([True, False, False, False]))
    W.step(np.zeros((4, 18)))
    assert W.last_feet[0][0] < 0.02, "the reset's jump was charged as slip"


def test_each_world_draws_its_own_MOTOR():
    """M143 (ADR-0137): with `MOTOR_RANGES` drawn, each world gets its own
    peak and no-load speed, inside the ranges, and the servo kernel reads
    them per world."""
    import math
    R = {**DR, **TomcatEnv.MOTOR_RANGES}
    W = WarpStand(16, randomize=R, residual_n=30.0, seed=5)
    lo, hi = TomcatEnv.MOTOR_RANGES["motor_peak_nm"]
    assert np.all((W.draw["peak"] >= lo) & (W.draw["peak"] <= hi))
    assert np.ptp(W.draw["peak"]) > 0.1
    rpm = W.draw["wnl"] * 60 / (2 * math.pi)
    assert np.all((rpm >= 380 - 1e-6) & (rpm <= 665 + 1e-6))
    assert np.allclose(W.peak_w.numpy(), W.draw["peak"], atol=1e-5)
    for _ in range(5):
        W.step(np.zeros((16, 18)))


def test_the_last_ACTION_is_observed_and_cleared_at_a_reset():
    """M144 (ADR-0138): with `act_obs`, the observation ends in the action
    just taken (85 channels), and a world that restarts sees zeros there."""
    from tomcat_kin.warp_env import WarpWalk
    W = WarpWalk(3, act_obs=True)
    o = W.reset()
    assert o.shape == (3, 85) and np.allclose(o[:, -18:], 0.0)
    a = np.full((3, 18), 0.25)
    o, *_ = W.step(a)
    assert np.allclose(o[:, -18:], 0.25)
    W._reset_worlds(np.array([False, True, False]))
    o = W.flatten(W._obs)
    assert np.allclose(o[1, -18:], 0.0) and np.allclose(o[0, -18:], 0.25)
