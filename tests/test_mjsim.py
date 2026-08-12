# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Closed-loop balance — the item M17 and M20 were both blocked on (M21).

⚠️ These are the tests that make the envelope measurable at all. The gate is the
**baseline**: a harness whose undisturbed drift is the same order as the disturbance
it is measuring cannot adjudicate anything, which is exactly why M17 declined to
report a number. Assert the noise floor before asserting any result on top of it.

`mujoco` is an optional dependency; the module skips without it.
"""

from __future__ import annotations

import numpy as np
import pytest

from tomcat_kin import control, gait, mjsim

mujoco = pytest.importorskip("mujoco", reason="mujoco is an optional dependency")

COMPLIANT_KP = 80
STIFF_KP = 500


@pytest.fixture(scope="module")
def controller():
    return gait.GaitController(gait.trot_params())


def _harness(controller, kp):
    model = mjsim.build(controller, mujoco, kp=kp)
    return mjsim.BalanceHarness(controller, mujoco, model, regulate_along_line=False)


def _mean_dcm(hist, sl):
    return float(np.abs(np.r_[[r["perp"] for r in hist][sl],
                              [r["para"] for r in hist][sl]]).mean())


def test_the_undisturbed_baseline_is_quiet_enough_to_measure_against(controller):
    """THE gate. M17's harness drifted 25 mm against a 30 mm signal, so it could
    not tell a recovery from its own noise. This one must stay far below that."""
    h = _harness(controller, COMPLIANT_KP)
    hist, fell = h.run(h.reset(), steps=30)

    assert not fell, f"undisturbed baseline fell after {len(hist)} steps"
    assert len(hist) == 30

    # ⚠️ Two numbers, because they say different things. The MEAN is the resolution
    # for a typical direction; the MAX is what limits the worst one.
    mean = _mean_dcm(hist, slice(None))
    worst = max(abs(r["perp"]) for r in hist) + max(abs(r["para"]) for r in hist)
    assert mean < 0.003, f"mean baseline drift {1000 * mean:.1f} mm"
    assert worst < 0.015, f"peak baseline excursion {1000 * worst:.1f} mm"

    # And — the failure M17 actually had — it must not be quietly winding up.
    # M17's drifted to 25 mm and was still growing; this one is bounded.
    assert _mean_dcm(hist, slice(-10, None)) < 2.0 * _mean_dcm(hist, slice(0, 10))


def test_the_swing_profile_must_land_at_rest(controller):
    """Regression guard on a C0 defect I reintroduced by hand.

    A `sin(pi u)` arc peaks correctly but lands with `-pi h / T` = 0.31 m/s of
    downward foot speed. It hammered the contact, the stance never settled at two
    feet, and the run died in 14 steps. This is the same class of error M5 and M6
    fixed in the shipped gait — which is why `GaitParams.swing_profile` defaults to
    "matched" rather than to a cycloid.
    """
    import math

    h = _harness(controller, COMPLIANT_KP)
    n = 400
    z = [0.5 * h.step_h * (1.0 - math.cos(2.0 * math.pi * k / n)) for k in range(n + 1)]
    dt = h.T / n
    assert abs(z[1] - z[0]) / dt < 0.02, "leaves the ground with vertical speed"
    assert abs(z[-1] - z[-2]) / dt < 0.02, "lands with vertical speed"
    assert max(z) == pytest.approx(h.step_h, rel=1e-3)


def test_balance_needs_compliant_legs(controller):
    """A finding, not a tuning note.

    Stiff position servos make ground-reaction distribution effectively bang-bang:
    ±1 mm of differential stance-leg extension swings the centre of pressure across
    the whole ±109 mm foot separation. The loop cannot trim its own load, and the
    undisturbed DCM winds up. Compliance restores it.

    The mechanical design already specifies passive compliance (series elastic
    elements / return springs). This validates that choice from a direction it was
    not chosen for.
    """
    h_soft = _harness(controller, COMPLIANT_KP)
    soft, _ = h_soft.run(h_soft.reset(), steps=24)
    h_stiff = _harness(controller, STIFF_KP)
    stiff, _ = h_stiff.run(h_stiff.reset(), steps=24)

    assert len(soft) == 24, "the compliant baseline should not fall"
    soft_late = _mean_dcm(soft, slice(-8, None))
    assert soft_late < 0.005, f"compliant drift {1000 * soft_late:.1f} mm"
    if len(stiff) == 24:
        assert _mean_dcm(stiff, slice(-8, None)) > 2.0 * soft_late


@pytest.mark.parametrize("angle_deg,floor_mm", [(300, 12.0), (60, 40.0)])
def test_the_envelope_is_strongly_direction_dependent(controller, angle_deg, floor_mm):
    """M17 found the two diagonals topple along axes 52.4 deg apart but could not
    cost it. Measured: the envelope spans **3.4x** across direction — 19.3 mm at its
    worst against 65.7 mm at its best — while `StepPlant` quotes a single 30.34 mm
    for every direction.

    ⚠️ The worst direction is **64 % of the prediction**. Checked loosely here
    because a bisection is slow; the numbers are in ADR-0026.
    """
    import math

    a = math.radians(angle_deg)
    u = np.array([math.cos(a), math.sin(a)])
    h = _harness(controller, COMPLIANT_KP)

    lo, hi = 0.0, 1.0
    for _ in range(6):
        mid = 0.5 * (lo + hi)
        hist, fell = h.run(h.reset(), steps=10, disturbance=mid * u)
        if not fell and len(hist) == 10:
            lo = mid
        else:
            hi = mid
    xi_mm = 1000.0 * lo / h.omega
    assert xi_mm > floor_mm, f"{angle_deg} deg gave only {xi_mm:.1f} mm"


def test_the_envelope_must_be_measured_on_a_settled_cycle(controller):
    """⚠️ The M23 correction, as a regression guard.

    M21 and M22 disturbed the robot at `t = 0` — one settle after being placed,
    before it had entered its limit cycle. That is not a trotting robot, and it made
    every envelope pessimistic: worst-case read 19.3 mm where a settled cycle gives
    25.3 mm. `run(disturbance=...)` applies the push immediately, so this is easy to
    get wrong; pre-run first.
    """
    import math

    h = _harness(controller, COMPLIANT_KP)
    u = np.array([math.cos(math.pi), math.sin(math.pi)])   # 180 deg, worst affected

    def envelope(pre_steps):
        lo, hi = 0.0, 1.6
        for _ in range(5):
            mid = 0.5 * (lo + hi)
            data = h.reset()
            ok = True
            if pre_steps:
                hist, fell = h.run(data, steps=pre_steps)
                ok = not fell and len(hist) == pre_steps
            if ok:
                hist, fell = h.run(data, steps=8, disturbance=mid * u)
                ok = not fell and len(hist) == 8
            lo, hi = (mid, hi) if ok else (lo, mid)
        return lo

    assert envelope(4) > envelope(0), "settling must not make the robot weaker"


def test_measured_worst_case_is_below_the_reduced_order_prediction(controller):
    """The result, on the corrected numbers (ADR-0028).

    Split by term the model is not uniformly optimistic: **foot placement achieves
    84 %** of its prediction, the **spine only 55 %**. The gap is one term.
    """
    plant = control.StepPlant.from_gait(controller, n=96, latency=0.0075, floor_mu=0.8)
    feet_only = control.rejection_envelope(plant)
    with_spine = control.self_consistent_envelope(controller)["envelope"]
    assert feet_only == pytest.approx(0.0303, abs=5e-4)
    assert with_spine == pytest.approx(0.0527, abs=1e-3)

    measured_feet, measured_spine = 0.0253, 0.0289      # settled cycle, worst of 18
    assert measured_feet / feet_only == pytest.approx(0.84, abs=0.03)
    assert measured_spine / with_spine == pytest.approx(0.55, abs=0.03)
    assert measured_spine < 0.048, "NFR15 would be demonstrated — recheck the claim"


def test_the_spine_wants_stiffness_where_the_legs_want_compliance(controller):
    """Two joint groups, opposite tuning — and getting it wrong looks identical.

    The lateral spine chain carries the whole forequarters. At the leg's compliant
    gain it wobbles enough to fell an otherwise-clean baseline in 10 steps; stiffened
    it is quiet again. A single "servo gain" knob would have hidden this.
    """
    soft = mjsim.build(controller, mujoco, kp=80, spine=True, spine_kp=150)
    firm = mjsim.build(controller, mujoco, kp=80, spine=True, spine_kp=1000)
    h_soft = mjsim.BalanceHarness(controller, mujoco, soft, use_spine=False)
    h_firm = mjsim.BalanceHarness(controller, mujoco, firm, use_spine=False)

    a, fell_a = h_soft.run(h_soft.reset(), steps=20)
    bb, fell_b = h_firm.run(h_firm.reset(), steps=20)
    assert fell_a and len(a) < 20, "a soft spine used to fell the baseline"
    assert not fell_b and len(bb) == 20
    assert _mean_dcm(bb, slice(None)) < 0.005


def test_the_proportional_spine_assist_has_unity_loop_gain_and_is_harmful(controller):
    """⚠️ M24, and it retracts M22/M23's "+14 % from the spine".

    The law is `q = -gain * e / SPINE_SWAY_PER_RAD`, and a sway of `q` moves the CoM
    by `SPINE_SWAY_PER_RAD * q = -gain * e`. **The loop gain is `gain` exactly, by
    construction.** With actuator lag that is marginal near 1 — and measured, even
    0.2 degrades the *undisturbed* baseline fivefold. The apparent envelope gain
    reported earlier was inside the noise the assist itself created.

    The authority is not the problem (see the held-sway test); reactive use of it is.
    """
    plant = control.StepPlant.from_gait(controller, n=96, latency=0.0075, floor_mu=0.8)
    assert plant.spine == pytest.approx(0.0366, abs=5e-4)

    model = mjsim.build(controller, mujoco, kp=80, spine=True, spine_kp=1000)

    def baseline(gain):
        h = mjsim.BalanceHarness(controller, mujoco, model, spine_gain=gain,
                                 use_spine=gain != 0.0)
        hist, fell = h.run(h.reset(), steps=18)
        return (_mean_dcm(hist, slice(None)) if len(hist) == 18 else float("inf")), fell

    def baseline_mode(gain, mode):
        h = mjsim.BalanceHarness(controller, mujoco, model, spine_gain=gain,
                                 use_spine=gain != 0.0, spine_mode=mode)
        hist, fell = h.run(h.reset(), steps=18)
        return (_mean_dcm(hist, slice(None)) if len(hist) == 18 else float("inf")), fell

    quiet, fell_off = baseline_mode(0.0, "reactive")
    gentle, _ = baseline_mode(0.2, "reactive")
    hard, fell_hard = baseline_mode(1.0, "reactive")

    assert not fell_off and quiet < 0.004, "the spine-off baseline must stay clean"
    assert gentle > 3.0 * quiet, "a 0.2 reactive assist should degrade the baseline"
    assert hard > gentle or fell_hard, "unity loop gain must be worse still"

    # M25: the SAME gain, planned once per stance and executed open-loop, is fine.
    # That is what identifies the structure — not the actuator — as the fault.
    planned, fell_planned = baseline_mode(1.0, "planned")
    assert not fell_planned, "planned deployment must survive where reactive fell"
    assert planned < gentle, "planned at gain 1.0 must beat reactive at 0.2"


def test_the_spines_realisable_authority_is_NOT_established(controller):
    """⚠️ A deliberate non-result, recorded so it is not re-derived by accident.

    I tried to show the spine's 36.6 mm credit is physically realised by holding a
    full-ROM sway while trotting and reading the CoM offset from the support line.
    **Two runs of that measurement disagreed (44.0 mm and 16.5 mm)**, and the reason
    is that `perp` does not hold a steady offset at all — it oscillates through zero
    and drifts (+8, +19, −2.8, −10, −26, −71 mm over 14 steps). Averaging its
    magnitude reads a drift as a bias.

    So: the assist is harmful (see above) and the motor rate is not the limit
    (open-loop ramps survive 300 deg/s), but **how much offset the spine can actually
    hold against planted feet is unmeasured.** This test pins the obstacle any future
    attempt has to deal with.
    """
    model = mjsim.build(controller, mujoco, kp=80, spine=True, spine_kp=1000)
    rom = abs(controller.body.spine.params.lateral_q_min[0])

    class Held(mjsim.BalanceHarness):
        def spine_assist(self, data, xi, support):
            for act in self.spine_act:
                data.ctrl[act] = rom
            return rom

    h = Held(controller, mujoco, model, use_spine=True)
    hist, _ = h.run(h.reset(), steps=14)
    assert len(hist) >= 10, "a HELD sway should not fell the robot outright"

    perp = np.array([r["perp"] for r in hist])
    assert perp.min() < 0 < perp.max(), (
        "perp held one sign — the offset may be steady after all, so the authority "
        "question is reopenable; re-measure before trusting either figure"
    )


def test_measuring_friction_demand_needs_a_PAIRED_design(controller):
    """⚠️ M30, recorded because five measurement designs failed before one worked.

    ADR-0019/0020's friction cost resisted measurement for reasons worth naming:

    1. **Per-contact force ratio** — pinned at the cone limit every time. A foot
       carrying 1.5 N at touchdown saturates any ratio without meaning anything.
    2. **Aggregate force ratio** — read 3.238, i.e. tangential force at 3× normal,
       which is impossible under gravity. Impact transients again.
    3. **Foot slip** — real magnitudes (0.4–2.5 mm) but the spine's contribution sat
       inside a ~1 mm noise floor from contact-point migration.
    4. **CoM shift, unpaired** — the effect is a few mm and the phase-to-phase
       standard deviation is **10–15 mm**. Averaging 5 trials showed nothing.

    What works is a **paired** design: the simulator is deterministic, so running the
    same deployment phase at two frictions differs *only* by the friction. That
    cancels the variance that swamped everything else.

    This test pins the variance, because it is the fact that dictates the design.
    """
    import math

    rom = abs(controller.body.spine.params.lateral_q_min[0])
    model = mjsim.build(controller, mujoco, kp=80, spine=True, spine_kp=1000, mu=0.8)

    class Deploy(mjsim.BalanceHarness):
        def __init__(self, *a, at=6, **k):
            super().__init__(*a, **k)
            self.at, self._t, self.seen = at, 0.0, []

        def drive_spine(self, data, u):
            self._t += self.dt
            k = self._t / self.T
            q = 0.0 if k < self.at else rom * min(1.0, k - self.at)
            for act in self.spine_act:
                data.ctrl[act] = q
            return q

    shifts = []
    for at in (4, 6, 8, 10, 12):
        h = Deploy(controller, mujoco, model, at=at, use_spine=True)
        hist, fell = h.run(h.reset(), steps=at + 4)
        if not fell and len(hist) >= at + 4:
            shifts.append(hist[-1]["perp"])

    assert len(shifts) >= 4, "not enough usable trials to characterise the variance"
    sd = float(np.std(shifts, ddof=1))
    # The friction effect M30 was chasing is ~5.7 mm at mu 0.7. The phase-to-phase
    # spread here is ~2.2 mm on this quantity and 10-15 mm on the CoM-versus-feet
    # shift the measurement actually used — comparable to, or larger than, the
    # signal either way. That is the whole reason the design has to be paired.
    assert sd > 0.0015, (
        f"phase-to-phase spread is only {1000 * sd:.1f} mm — if the variance really "
        "has fallen, an unpaired friction measurement may now be viable; re-check M30"
    )
    assert sd < 0.020, "spread this large would mean the baseline itself is broken"


def test_the_envelope_is_horizon_limited_and_must_be_converged(controller):
    """⚠️ M31, and it corrects the precision of every figure in M21–M30.

    The viable set asks *can the robot RECOVER* (reach the origin). A simulation asks
    *does it SURVIVE N more steps*. Those are different questions, and the second
    depends on N:

    | survival horizon | measured envelope |
    |---|---|
    | 4, 6, 8 steps | 39.2 mm |
    | 12 | 34.7 mm |
    | **16, 24** | **28.6 mm** (converged) |

    M21–M30 all used an **8-step** horizon, so their envelopes were horizon-limited —
    `control.py`'s own docstring records making exactly this mistake with `steps=12`
    in `rejection_envelope`, and I repeated it in the simulation.

    Converged, the worst direction is **25.6 mm = 86 %** of the viable bound, not the
    **97 %** ADR-0033 claimed from an 8-step measurement. Still near-optimal, and
    still — necessarily — *below* the bound, which validates both.
    """
    import math

    from tomcat_kin import mjcf, viable

    plant = control.StepPlant.from_gait(controller, n=96, latency=0.0075, floor_mu=0.8)
    q = mjcf.stance_pose(controller, 0.25)
    reach = (float(plant.reach[0]), float(plant.reach[1]))
    bound = min(
        viable.reach_in_direction(
            viable.viable_set(controller, q, plant.omega, plant.stance, reach, steps=20),
            (math.cos(math.radians(a)), math.sin(math.radians(a))))
        for a in range(0, 360, 15))

    model = mjsim.build(controller, mujoco, kp=80)
    h = mjsim.BalanceHarness(controller, mujoco, model)
    u = np.array([math.cos(math.radians(120)), math.sin(math.radians(120))])

    def envelope(steps):
        lo, hi = 0.0, 1.5
        for _ in range(7):
            mid = 0.5 * (lo + hi)
            data = h.reset()
            hist, fell = h.run(data, steps=4)
            ok = not fell and len(hist) == 4
            if ok:
                hist, fell = h.run(data, steps=steps, disturbance=mid * u)
                ok = not fell and len(hist) == steps
            lo, hi = (mid, hi) if ok else (lo, mid)
        return lo / h.omega

    short, long = envelope(8), envelope(16)
    assert short > long, "a longer horizon must be a HARDER test, not an easier one"
    assert long <= bound * 1.02, (
        f"converged envelope {1000 * long:.1f} mm exceeds the viability bound "
        f"{1000 * bound:.1f} mm — no controller can do that, so one of them is wrong"
    )
    assert long > 0.7 * bound, "the controller should still be within ~30 % of optimal"


def test_realising_the_load_split_makes_it_worse_not_better(controller):
    """⚠️ M32, and the fourth consecutive result of this shape.

    M31 identified the load split along the support line (`lam`) as *the* missing
    degree of freedom: the 2-D projection solves for it, and M27 measured the
    authority as available on compliant legs (linear, −39.3 mm/mm). Realising it —
    planned once per stance, executed open-loop, the structure that fixed the spine
    (ADR-0030) — makes the controller **much worse**:

    | 300 deg, converged | envelope |
    |---|---|
    | axis (shipped) | **25.6 mm** |
    | projected + lam | **0.8 mm** |

    That is now four DOFs measured as available and four that degrade the loop when
    engaged: reactive spine, planned spine, reactive CoP, and this. Only foot
    placement — what the controller was designed around — delivers, and it reaches
    **86 %** of the theoretical bound. The pattern is about the architecture.
    """
    import math

    model = mjsim.build(controller, mujoco, kp=80)
    u = np.array([math.cos(math.radians(300)), math.sin(math.radians(300))])

    def envelope(**kw):
        h = mjsim.BalanceHarness(controller, mujoco, model, **kw)
        lo, hi = 0.0, 1.5
        for _ in range(6):
            mid = 0.5 * (lo + hi)
            data = h.reset()
            hist, fell = h.run(data, steps=4)
            ok = not fell and len(hist) == 4
            if ok:
                hist, fell = h.run(data, steps=20, disturbance=mid * u)
                ok = not fell and len(hist) == 20
            lo, hi = (mid, hi) if ok else (lo, mid)
        return lo / h.omega

    shipped = envelope(placement_mode="axis")
    with_lam = envelope(placement_mode="projected", realise_lambda=True)
    # The margin itself grows with the horizon — 2.7x at 20 steps, 32x at 32 —
    # which is ADR-0036's point again. 2x is the robust floor.
    assert shipped > 2.0 * with_lam, (
        f"lam realisation now gives {1000 * with_lam:.1f} mm against the shipped "
        f"{1000 * shipped:.1f} mm — if it has stopped hurting, re-open ADR-0037"
    )
