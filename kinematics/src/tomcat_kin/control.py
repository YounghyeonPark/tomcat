# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Closed-loop balance for the trot — step-to-step foot placement (M8).

M7 showed the trot's *nominal* trajectory is dynamically consistent. That is a
weaker statement than it sounds: it says the error starts at zero, not that it
stays there. The trot is an inverted pendulum about its diagonal support line,
and any deviation grows as ``e^(omega*t)`` — on the shipped gait ``omega*T =
1.17`` per stance, so an error is multiplied by **3.2 every step** and by 339
over five. Open loop there is nothing to arrest it.

This module closes that loop the way legged robots actually do: not by tracking a
trajectory harder, but by **choosing where to put the next foot**.

The reduced model
-----------------
Take the perpendicular offset of the CoM from the support line as a single
coordinate ``c``, with the line at ``p``. The linear inverted pendulum gives::

    c_ddot = omega^2 (c - p),        omega = sqrt(g / h)

The useful change of variable is the **DCM** (divergent component of motion, a.k.a.
capture point), ``xi = c + c_dot / omega``, because it obeys a *first-order*
equation::

    xi_dot = omega (xi - p)     =>     xi(t) = p + (xi_0 - p) e^(omega t)

All the instability lives in ``xi``; the remaining mode is stable and can be
ignored for balance. So the whole control problem collapses to: **at each
touchdown, place the new support line relative to the current DCM.**

    p = nominal + (growth - beta)/(growth - 1) * (xi - nominal)

⚠️ Note the coefficient is GREATER THAN ONE (1.45 on the shipped trot). The foot
goes **beyond** the DCM, not under it. Placing it exactly at the DCM arrests the
topple but leaves the body permanently displaced — it captures without
recovering, and the robot then walks away sideways looking perfectly stable.
``capture_placement`` is kept separate so the distinction is explicit.

``beta`` is the residual error per step: 0 is one-step deadbeat. Reach is what
makes the envelope finite; ``rejection_envelope`` quantifies it.

Honest scope
------------
- ⚠️ **Reduced order.** One perpendicular coordinate, constant CoM height,
  point feet, instantaneous support transfer. It is a *controller-design* model,
  not a replacement for ``dynamics.py``. Its two parameters (``omega`` and the
  stance duration) are taken FROM the full model — see ``from_gait``.
- ⚠️ **The nominal orbit here is not identical to M7's.** In M7 the offset changes
  mostly because the support line translates under a nearly-stationary commanded
  CoM; here the CoM genuinely falls. Both are real effects; this module models the
  one that destabilises.
- ⚠️ **No sensing model.** Placement is computed from the true state. Latency,
  contact-detection jitter and DCM estimation error all degrade the envelope, and
  are exactly what ADR-0012's paw sensing exists to bound. Modelled crudely by
  ``simulate(..., estimation_error=...)``.
- No double support, no swing-leg dynamics, no actuator limits in the loop.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

GRAVITY = 9.81


@dataclass(frozen=True)
class StepPlant:
    """The step-to-step inverted-pendulum plant for one gait.

    Attributes
    ----------
    omega : float
        ``sqrt(g/h)`` (1/s) — how fast the body topples.
    stance : float
        Stance duration of one step (s).
    reach : tuple[float, float]
        (min, max) support-line placement relative to nominal, **measured along
        the perpendicular** (m) — already projected, see ``projection``. This is
        what makes the problem finite: a controller can only ask for a foot where
        the leg can actually put it.
    projection : float
        Fraction of a fore-aft foothold shift that appears along the support-line
        perpendicular (~0.44 for the diagonal trot). The perpendicular is ~90 %
        LATERAL and the legs are sagittal-only, so most of the leg's generous
        fore-aft range is simply not pointed the right way.
    spine : float
        Perpendicular CoM shift available from the ADR-0009 LATERAL SPINE bend
        within one stance (m). ROM-limited in practice, not rate-limited. This is a second balance
        actuator the project already owns -- bought for the crawl's static
        stability -- and it pushes almost exactly along the perpendicular, i.e.
        precisely where foot placement is weakest. 0 disables it.
    latency : float
        Delay (s) between measuring the DCM and the foot actually landing where
        commanded. Handled by PREDICTING forward, which works but amplifies any
        estimation error by ``e^(omega*latency)``.
    """

    omega: float
    stance: float
    reach: tuple = (-0.033, 0.068)
    projection: float = 0.442
    spine: float = 0.0
    latency: float = 0.0

    @property
    def growth(self) -> float:
        """DCM amplification over one stance, ``e^(omega*stance)``."""
        return float(math.exp(self.omega * self.stance))

    def propagate(self, xi: float, p: float) -> float:
        """DCM at the end of a stance that began at ``xi`` with the line at ``p``."""
        return p + (xi - p) * self.growth

    @classmethod
    def from_gait(cls, controller, n: int = 96, latency: float = 0.0,
                  floor_mu: "float | None" = None,
                  gait_mu: float = 0.145) -> "StepPlant":
        """Build the plant from a real gait: omega and stance from the full model.

        ``floor_mu`` clamps the spine authority to what ground friction can supply
        (see the note in the body). ``None`` leaves it ROM-limited, which is the
        pre-M14 behaviour and is correct for a floor of mu >= ~1.0.
        """
        from . import dynamics as dyn

        cyc = dyn.cycle(controller, n)
        h = float(np.mean(cyc.com[:, 2] - cyc.ground_z))
        stance = controller.params.duty_factor * controller.params.period
        nom = controller.params.nominal_foot[0]
        lo, hi = -0.069, 0.158           # measured leg foothold range at stance height

        # ⚠️ PROJECT the fore-aft reach onto the support-line PERPENDICULAR.
        # The DCM lives perpendicular to the diagonal, and that direction is ~90 %
        # LATERAL. The legs are sagittal-only -- there is no abduction (rejected in
        # ADR-0009) and the track is fixed at +/-48 mm -- so the only placement
        # freedom is fore-aft, and it buys perpendicular authority only through its
        # ~0.44 projection. Using the raw fore-aft range OVERSTATES the disturbance
        # envelope by ~2.3x; an earlier revision of this module did exactly that.
        line = dyn.support_line(cyc, 0) or dyn.support_line(cyc, n // 4)
        if line is None:
            proj = 1.0
        else:
            _, d = line
            proj = abs(-d[1])          # x-component of the unit perpendicular (-dy, dx)
        # Lateral-spine authority. ROM-limited, NOT rate-limited: the drive can do
        # ~912 deg/s at the joint (380 rpm motor x the 8/20 mm spool-to-arm ratio),
        # while traversing the full +/-15 deg ROM inside a 150 ms stance needs only
        # 200 deg/s. An earlier revision clamped this with NFR2f's 119 deg/s -- but
        # that is a REQUIREMENT floor sized for the ADR-0007 righting reflex, not a
        # capability, and using it under-counted the spine by ~40 %.
        sp = controller.body.spine.params
        rom = float(min(abs(sp.lateral_q_min[0]), abs(sp.lateral_q_max[0])))
        joint_rate = (2.0 * math.pi * 380.0 / 60.0) * (
            sp.motor_spool_radius / sp.lateral_moment_arm[0])
        usable = min(rom, joint_rate * stance)
        lateral = abs(controller.body.center_of_mass_y(np.full(sp.n_segments, usable)))
        perp = math.sqrt(max(0.0, 1.0 - proj * proj))   # y-component of the perpendicular
        spine_auth = lateral * perp

        # ⚠️ THIRD constraint on the spine, and the one that actually binds on a
        # poor floor: FRICTION. Bending the spine is INTERNAL motion, and internal
        # motion cannot move the whole-body CoM -- moving it relative to the
        # planted feet requires a horizontal GROUND reaction. Shifting by d within
        # the stance needs a ~ 4d/t^2, hence mu >= 4d/(t^2 g) ON TOP of whatever
        # the gait already spends. Earlier revisions treated the spine assist as a
        # free DCM offset; it is not.
        if floor_mu is not None:
            budget = max(floor_mu - gait_mu, 0.0)
            spine_auth = min(spine_auth, budget * GRAVITY * stance * stance / 4.0)

        return cls(omega=math.sqrt(GRAVITY / h), stance=stance,
                   reach=((lo - nom) * proj, (hi - nom) * proj), projection=proj,
                   spine=spine_auth, latency=latency)


def capture_placement(plant: StepPlant, xi_end: float) -> float:
    """Placement that merely ARRESTS the topple: put the line under the DCM.

    Then the next stance begins with ``xi = p`` and nothing grows. But note what
    this does *not* do: the DCM is held wherever the disturbance left it, so the
    robot stops falling and then walks on permanently displaced. It captures; it
    does not recover. Use ``placement`` for that.
    """
    return xi_end


def placement(plant: StepPlant, xi_end: float, nominal: float = 0.0,
              beta: float = 0.0, clamp: bool = True) -> float:
    """Foot placement that drives the DCM back to ``nominal``.

    Solving ``xi_next = nominal + beta*(xi_end - nominal)`` for the placement::

        p = nominal + (growth - beta)/(growth - 1) * (xi_end - nominal)

    Note the coefficient exceeds 1 (it is 1.45 on the shipped trot): the foot must
    be placed **beyond** the DCM, not under it. Placing it *at* the DCM only
    captures (see ``capture_placement``) — the body then never returns to the
    nominal path. Getting this wrong is an easy and quiet error: the robot looks
    stable and slowly walks away sideways.

    ``beta`` is the residual error fraction per step: 0 is one-step deadbeat
    (fastest, most foothold-hungry), 0.5 halves the error each step for a smaller
    excursion. Result is clamped to the leg's real reach, which is what makes the
    rejection envelope finite.
    """
    g = plant.growth
    p = nominal + (g - beta) / (g - 1.0) * (xi_end - nominal)
    if clamp:
        p = min(max(p, nominal + plant.reach[0]), nominal + plant.reach[1])
    return p


def spine_assist(plant: StepPlant, xi: float, nominal: float = 0.0) -> float:
    """Perpendicular CoM shift the lateral spine contributes, bounded by ``plant.spine``.

    The spine moves the CoM *relative to the body*, so it offsets the DCM directly
    rather than moving the support line. It therefore ADDS to foot placement
    instead of competing with it -- and because the support-line perpendicular is
    ~90 % lateral, the spine points almost exactly the right way while the
    sagittal-only legs only manage a 0.44 projection.

    Sign is opposite the error: lean away from the direction of the topple.
    """
    e = xi - nominal
    return -math.copysign(min(abs(e), plant.spine), e) if plant.spine else 0.0


def simulate(plant: StepPlant, steps: int = 12, xi0: float = 0.0,
             closed_loop: bool = True, beta: float = 0.0,
             estimation_error: float = 0.0, nominal: float = 0.0,
             use_spine: bool = False):
    """Run ``steps`` steps and return the DCM at each touchdown.

    ``xi0`` is the initial DCM error — the disturbance. ``estimation_error`` adds
    a fixed bias to the DCM the controller *sees*, standing in for sensing error
    (ADR-0012); the plant still propagates the true value.

    With ``closed_loop=False`` the line is left at ``nominal`` every step, which
    reproduces the open-loop divergence M7 could not rule out.
    """
    xi = float(xi0)
    out = [xi]
    for _ in range(steps):
        if not closed_loop:
            xi = plant.propagate(xi, nominal)
            out.append(xi)
            continue
        measured = xi + estimation_error
        tau = plant.latency
        if tau <= 0.0:
            s_assist = spine_assist(plant, measured, nominal) if use_spine else 0.0
            p = placement(plant, measured + s_assist, nominal=nominal, beta=beta)
            xi = plant.propagate(xi + s_assist, p)
            out.append(xi)
            continue

        # LATENCY, modelled honestly. The command lands tau late, so the OLD line
        # (left at nominal) acts for tau, and only then does the new placement act,
        # for the REMAINING (stance - tau). The controller knows this and predicts
        # xi(tau) exactly -- prediction is not the problem. Two real costs remain:
        #   (a) any estimation error is amplified by e^(omega*tau) in that
        #       prediction, and
        #   (b) there is less time under the corrective placement, so a LARGER
        #       placement is needed and the reach saturates sooner.
        gtau = math.exp(plant.omega * tau)
        grem = math.exp(plant.omega * max(plant.stance - tau, 0.0))
        pred = nominal + (measured - nominal) * gtau        # controller's estimate
        s_assist = spine_assist(plant, pred, nominal) if use_spine else 0.0
        # Solve xi_end = nominal + beta*(pred - nominal) over the REMAINING time.
        e = pred + s_assist - nominal
        p = nominal + (grem - beta) / (grem - 1.0) * e if grem > 1.0 + 1e-12 else nominal
        p = min(max(p, nominal + plant.reach[0]), nominal + plant.reach[1])
        # True propagation: tau under the old (nominal) line, then the remainder.
        xi_tau = nominal + (xi - nominal) * gtau + s_assist
        xi = p + (xi_tau - p) * grem
        out.append(xi)
    return np.array(out)


def one_step_envelope(plant: StepPlant, beta: float = 0.0) -> float:
    """Largest DCM error correctable in a SINGLE step, i.e. before the reach clamps.

    The law asks for ``(growth-beta)/(growth-1)`` times the error, so this is
    simply the reach divided by that coefficient. On the shipped trot the
    coefficient is 1.45 and the binding reach is the REARWARD one, giving ~51 mm.
    """
    coeff = (plant.growth - beta) / (plant.growth - 1.0)
    return float(min(abs(plant.reach[0]), abs(plant.reach[1])) / coeff)


def rejection_envelope(plant: StepPlant, beta: float = 0.0,
                       tol: float = 1e-4, steps: int = 400,
                       use_spine: bool = False) -> float:
    """Largest initial DCM error (m) the controller can eventually recover from.

    Larger than ``one_step_envelope``: once the placement saturates the controller
    cannot null the error in one step, but a clamped placement still *reduces* it,
    so recovery continues over several steps until it falls inside the one-step
    envelope. Beyond some error even a saturated foot cannot keep up and the DCM
    runs away.

    ``steps`` is deliberately generous. An earlier version used 12, which made a
    slow gain (``beta = 0.7``) look like a SMALLER envelope than a fast one — the
    result was horizon-limited, not reach-limited, which is a different (and
    misleading) statement. Reach is the physical limit; convergence rate only sets
    how long it takes.
    """
    # The leg reaches much further FORWARD than backward (+153 vs -74 mm on the
    # trot), so the two disturbance directions have different envelopes. Test both
    # and return the smaller: that is the one the robot is actually guaranteed.
    def recovers(d: float) -> bool:
        for sign in (+1.0, -1.0):
            traj = simulate(plant, steps=steps, xi0=sign * d,
                            closed_loop=True, beta=beta, use_spine=use_spine)
            if not (np.all(np.isfinite(traj)) and abs(traj[-1]) < tol):
                return False
        return True

    lo, hi = 0.0, 0.001
    while recovers(hi) and hi < 100.0:
        lo, hi = hi, hi * 2.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if recovers(mid):
            lo = mid
        else:
            hi = mid
    return lo


# ===================================================================
# M11 — the latency budget: where the delay actually comes from
# ===================================================================
#
# ADR-0014 measured the envelope against an ASSUMED latency and left the 20 ms
# figure (NFR12) unallocated. Allocating it turns the calculation inside out,
# because latency is not an independent parameter: a bigger disturbance needs a
# bigger foothold correction, a bigger correction takes the leg LONGER to
# execute, and that time IS the staleness of the information the controller
# committed on. Correction size therefore sets latency, which sets the envelope,
# which bounds the correction. It has to be solved as a fixed point.

# Unused foot speed available for a correction, m/s. The swing's nominal peak is
# ~1.83 m/s against an actuator ceiling of ~5.93 m/s (motor 380 rpm through the
# tendon ratios), so ~4.1 m/s is spare. [derived]
SPARE_FOOT_SPEED = 4.10


# Worst-direction foot acceleration the leg can produce, m/s^2. Computed in
# operational space -- tau = J^T Lambda a with Lambda = (J M^-1 J^T)^-1, minimised
# over foot-acceleration directions and over the swing -- from the motor's 1.95 N.m
# peak through the tendon ratios and the point-mass link inertias. ~107 g. It is
# this high because tendon drive keeps the leg at 95 g. [derived]
#
# ⚠️ Do NOT compute this by driving every joint at peak torque at once: the distal
# joint's inertia is tiny, so that route reports ~665 g, which is an artefact of a
# near-singular direction rather than a usable acceleration.
FOOT_ACCEL_LIMIT = 1051.0


def actuation_time(plant: StepPlant, disturbance: float,
                   spare_speed: float = SPARE_FOOT_SPEED,
                   beta: float = 0.0,
                   accel_limit: float = FOOT_ACCEL_LIMIT) -> float:
    """Time (s) the leg needs to execute the correction a disturbance demands.

    The placement law asks for ``(growth-beta)/(growth-1)`` times the error along
    the support-line PERPENDICULAR; the leg delivers that only through its
    ``projection``, so the fore-aft foot travel is larger by ``1/projection``.

    The move is then run as a **trapezoidal** profile — accelerate at
    ``accel_limit``, cruise at ``spare_speed``, decelerate — or triangular if the
    distance is too short to reach cruise::

        t = d/v + v/a          if d >= v^2/a   (trapezoidal)
        t = 2*sqrt(d/a)        otherwise       (triangular)

    Setting ``accel_limit=inf`` recovers the earlier constant-velocity model, which
    understated the time by 10 % on a full-scale correction and ~50 % on a small
    one — the ramp matters most when the move is short.
    """
    coeff = (plant.growth - beta) / (plant.growth - 1.0)
    perp = min(coeff * abs(disturbance), abs(plant.reach[1]))
    fore_aft = perp / plant.projection if plant.projection > 0 else perp
    if not math.isfinite(accel_limit) or accel_limit <= 0.0:
        return fore_aft / spare_speed
    if fore_aft >= spare_speed * spare_speed / accel_limit:
        return fore_aft / spare_speed + spare_speed / accel_limit
    return 2.0 * math.sqrt(fore_aft / accel_limit)


def self_consistent_envelope(controller, pipeline: float = 0.0075,
                             spare_speed: float = SPARE_FOOT_SPEED,
                             beta: float = 0.0, n: int = 96,
                             accel_limit: float = FOOT_ACCEL_LIMIT) -> dict:
    """Envelope solved as a FIXED POINT, latency included rather than assumed.

    ``pipeline`` is the electronics/firmware delay — contact detection, state
    estimation, bus transport, control computation — everything except the leg
    physically moving. The actuation term is computed per disturbance by
    ``actuation_time`` and added.

    Returns ``{"envelope", "latency", "pipeline", "actuation"}``. On the shipped
    trot this gives **~59 mm at a 7.5 ms pipeline**, against the ~90 mm that
    ADR-0014/0015 quoted for zero latency.

    The useful surprise: the result is nearly INSENSITIVE to ``pipeline``. Going
    2.5 → 20 ms costs only 62 → 52 mm, because the actuation term dominates. The
    electronics is not the bottleneck — foot speed is.
    """
    import dataclasses

    # Build the plant ONCE. Only `latency` varies over the bisection, and
    # from_gait() re-runs a full inverse-kinematics cycle sweep every call -- doing
    # that inside the loop cost ~50x more work than the whole rest of the solve.
    base = StepPlant.from_gait(controller, n)

    def envelope_for(d: float) -> tuple:
        tau = pipeline + actuation_time(base, d, spare_speed, beta, accel_limit)
        plant = dataclasses.replace(base, latency=tau)
        return rejection_envelope(plant, beta=beta, use_spine=True), tau

    lo, hi = 0.002, 0.15
    for _ in range(34):          # ~8 nm resolution on a 150 mm bracket
        mid = 0.5 * (lo + hi)
        e, _ = envelope_for(mid)
        if mid <= e:
            lo = mid
        else:
            hi = mid
    env, tau = envelope_for(lo)
    return {"envelope": lo, "latency": tau, "pipeline": pipeline,
            "actuation": tau - pipeline}
