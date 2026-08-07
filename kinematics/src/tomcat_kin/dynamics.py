# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Whole-body DYNAMICS for the walk — the first non-quasi-static layer (M6).

Everything before this milestone was a sequence of static postures. The gait was
checked against a fore-aft support interval (M4) and then a true ground-plane
support polygon (M5), both of which ask *only* "does the CoM project inside the
feet?" That question is the right one for a body that is not accelerating. This
module asks the question that is actually true of a walking robot: **can the
contacts produce the forces the motion requires?**

That matters because M5 ended on a number it could not properly defend. Reversing
the lateral spine sway needs a real sideways acceleration of the body, and a
hand calculation (`GaitController.crossover_accel`) put it near the paw's
friction limit. That calculation lumped the whole robot into one point mass and
one friction cone. Here the force is resolved **per foot**, which is where
slipping actually happens.

What IS modelled
----------------
- The whole-body CoM path in 3D over a cycle, **including the ADR-0009 lateral
  sway**, differentiated twice in time to get its acceleration.
- Newton balance: the stance feet must supply `M*(a + g)`.
- Euler/moment balance about the CoM, so the force distribution is statically
  admissible rather than arbitrary.
- A **per-foot** distribution, its friction requirement, and the unilateral
  (feet can push, never pull) check.
- The **ZMP**, and its margin inside the support polygon — the dynamic
  counterpart of the M5 static margin.

What is NOT modelled (flagged, per project convention)
------------------------------------------------------
- **Angular momentum rate is taken as zero** (`dH/dt = 0`) in the POLYGON/ZMP path.
  The moment balance there is the classical ZMP form. M13 quantified what that
  costs rather than leaving it as a warning:
  * at the **crawl** it is worth ~**1 mm** of ZMP shift -- negligible;
  * at the **trot** it is worth ~**42 mm** -- badly violated in magnitude;
  * BUT two point contacts can resist every moment except the one about the line
    joining them, and the swing-leg reaction is mostly **pitch**, which they can.
    Only ~**21 %** reaches the destabilising axis, and M7's bounded roll survives.
  ``angular_momentum_caveat`` reports the shift; ``swing_leg_moment`` resolves the
  component that actually matters.
- **Swing-leg reaction is fully modelled in the LINE-support path**
  (``swing_leg_moment``): both the **orbital** term (``m r x a``) and the **spin**
  term (``I alpha``, slender rods). Spin turns out to be only ~3 % of gravity,
  because tendon drive keeps the legs light *and* short.
- No contact compliance, no impact at touchdown, no tendon stretch/friction, no
  motor dynamics. Rigid links, point contacts.
- The force distribution is a **heuristic** (§ ``contact_forces``), not the
  optimum a real controller would solve for. It is deliberately biased toward
  vertical forces, which UNDER-states the friction requirement of a naive
  controller and OVER-states how easy the problem is for a bad one. Read the
  reported μ as "achievable by a reasonable controller", not "guaranteed".

Frames
------
Body-ground frame throughout: x forward, y left, z up, spine base at the origin,
ground at the stance feet's z. The body's forward advance is linear in time, so
it contributes nothing to acceleration and can be ignored here (see
``com_path``).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

GRAVITY = 9.81


# --------------------------------------------------------------------- results
@dataclass(frozen=True)
class ContactSolution:
    """Per-foot contact forces and what they demand of the ground at one phase.

    Attributes
    ----------
    phase : float
        Gait phase in [0, 1).
    forces : dict[str, np.ndarray]
        Ground-reaction force (N) on each STANCE foot, (fx, fy, fz) in the
        body-ground frame. fz > 0 pushes the robot up.
    com : np.ndarray
        CoM position (x, y, z) in metres.
    accel : np.ndarray
        CoM acceleration (ax, ay, az) in m/s².
    mu_required : dict[str, float]
        Per-foot ratio |tangential| / normal — the friction coefficient that foot
        needs. ``inf`` if its normal force is ~0.
    residual : float
        Norm of the unsatisfied balance equations (should be ~0).
    """

    phase: float
    forces: dict[str, np.ndarray]
    com: np.ndarray
    accel: np.ndarray
    mu_required: dict[str, float]
    residual: float

    @property
    def peak_mu(self) -> float:
        """Worst per-foot friction requirement, over MEANINGFULLY LOADED feet.

        ``mu = |F_tangential| / F_normal`` blows up as the normal force goes to
        zero, so a foot carrying 0.3 N can report mu = 3 while contributing
        nothing. Physically that foot just slides a little and its load
        redistributes. Feet carrying under 5 % of the total normal load are
        therefore excluded here; use ``mu_required`` for the raw per-foot values
        and ``aggregate_mu`` for the body-level requirement.
        """
        return self.peak_mu_loaded(0.05)

    def peak_mu_loaded(self, frac: float = 0.05) -> float:
        """Worst friction requirement among feet carrying >= ``frac`` of the load."""
        total_n = sum(max(f[2], 0.0) for f in self.forces.values())
        if total_n <= 1e-9:
            return float("inf")
        vals = [self.mu_required[nm] for nm, f in self.forces.items()
                if f[2] >= frac * total_n]
        return max(vals) if vals else 0.0

    @property
    def aggregate_mu(self) -> float:
        """Body-level friction requirement: |sum F_tangential| / sum F_normal.

        This is the number that decides whether the ROBOT slides, as opposed to
        whether one lightly-loaded paw scuffs. It is what the M5 hand
        calculation was estimating.
        """
        ft = np.zeros(2)
        fn = 0.0
        for f in self.forces.values():
            ft += f[:2]
            fn += max(f[2], 0.0)
        return float(np.linalg.norm(ft) / fn) if fn > 1e-9 else float("inf")

    @property
    def min_normal(self) -> float:
        """Smallest normal force (N). NEGATIVE means a foot would have to PULL."""
        return min((f[2] for f in self.forces.values()), default=0.0)

    @property
    def unilateral_ok(self) -> bool:
        """True if no foot is required to pull the robot down."""
        return self.min_normal >= -1e-9

    def feasible(self, mu: float = 0.8) -> bool:
        """True if the body stays inside the friction cone and no foot pulls.

        Judged on ``aggregate_mu`` — the body-level requirement — not on the
        worst lightly-loaded paw.
        """
        return self.unilateral_ok and self.aggregate_mu <= mu


@dataclass(frozen=True)
class ZMPResult:
    """Zero-moment point and its margin inside the support polygon."""

    phase: float
    zmp: np.ndarray          # (x, y) on the ground plane
    com_ground: np.ndarray   # (x, y) CoM ground projection, for comparison
    margin: float            # signed distance inside the support polygon (m)

    @property
    def is_stable(self) -> bool:
        return self.margin > 0.0

    @property
    def excursion(self) -> float:
        """How far the ZMP sits from the static CoM projection (m).

        This is the whole point of the milestone: it is the distance the
        quasi-static check was blind to.
        """
        return float(np.linalg.norm(self.zmp - self.com_ground))


@dataclass(frozen=True)
class CycleData:
    """Everything a dynamics sweep needs, evaluated ONCE over one cycle.

    Without this every per-phase query would re-solve the whole cycle to get its
    acceleration, making a sweep O(n^2) in inverse-kinematics calls. Build it
    once with ``cycle()`` and hand it to the per-phase functions.
    """

    n: int
    period: float
    mass: float
    com: np.ndarray               # (n, 3)
    accel: np.ndarray             # (n, 3)
    feet: list                    # per phase: dict name -> (3,) stance foot position
    ground_z: float

    def index(self, phase: float) -> int:
        return int(round((float(phase) % 1.0) * self.n)) % self.n


def cycle(controller, n: int = 240) -> CycleData:
    """Evaluate the whole gait cycle once: CoM path, acceleration, stance feet."""
    com = np.empty((n, 3))
    feet: list = []
    ground_z = None
    for i in range(n):
        p = i / n
        st = controller.state(p)
        leg_q = {nm: l.q for nm, l in st.legs.items()}
        y = controller.body.center_of_mass_y(controller.lateral_q(p))
        com[i] = (st.com.x, y, st.com.z)
        if ground_z is None:
            nm0 = st.stance_legs[0]
            ground_z = float(controller.body.foot_world_position(
                st.spine_q, nm0, leg_q[nm0])[1])
        xy = controller.body.foot_ground_xy(st.spine_q, leg_q)
        feet.append({nm: np.array([xy[nm][0], xy[nm][1], ground_z])
                     for nm in st.stance_legs})
    dt = controller.params.period / n
    accel = (np.roll(com, -1, axis=0) - 2.0 * com + np.roll(com, 1, axis=0)) / (dt * dt)
    return CycleData(n=n, period=controller.params.period,
                     mass=controller.body.total_mass, com=com, accel=accel,
                     feet=feet, ground_z=float(ground_z))


# ------------------------------------------------------------------ CoM motion
def com_path(controller, n: int = 240) -> np.ndarray:
    """CoM positions (n, 3) over one cycle, in the body-ground frame.

    Includes the lateral (y) offset produced by the commanded ADR-0009 spine
    sway, which is the component that makes this interesting — the sagittal CoM
    barely moves, the lateral one swings ~80 mm twice a cycle.

    The body's steady forward advance is deliberately EXCLUDED: it is linear in
    time, so it adds nothing to the acceleration this module needs, and leaving
    it out keeps every quantity in one frame.
    """
    out = np.empty((n, 3))
    for i in range(n):
        p = i / n
        st = controller.state(p)
        y = controller.body.center_of_mass_y(controller.lateral_q(p))
        out[i] = (st.com.x, y, st.com.z)
    return out


def com_acceleration(controller, n: int = 240) -> np.ndarray:
    """CoM acceleration (n, 3) in m/s², by periodic central differences.

    The gait is defined kinematically, so acceleration must come from
    differentiating the commanded path. The signal is exactly periodic, so
    ``np.roll`` gives a clean wrap with no end effects.

    ⚠️ The sway law is a ramped square wave: its velocity is piecewise constant,
    so the true acceleration is a pair of impulses at the ends of each crossover
    ramp. Central differencing on a finite grid SMEARS those impulses over one
    grid step, which under-states the peak. ``crossover_accel`` on the controller
    models the same event as a bang-bang profile instead and is the conservative
    number; the two are compared in ``compare_with_hand_calc``.
    """
    r = com_path(controller, n)
    dt = controller.params.period / n
    return (np.roll(r, -1, axis=0) - 2.0 * r + np.roll(r, 1, axis=0)) / (dt * dt)


# ------------------------------------------------------------- contact solving
def _balance_system(feet: dict[str, np.ndarray], com: np.ndarray,
                    mass: float, accel: np.ndarray):
    """Build the 6 x 3n Newton-Euler balance system for the stance feet.

    Rows 0-2  Newton :  sum F_i               = m (a + g z_hat)
    Rows 3-5  Euler  :  sum (r_i - c) x F_i   = 0        [dH/dt = 0, see module doc]
    """
    names = sorted(feet)
    n = len(names)
    A = np.zeros((6, 3 * n))
    for k, nm in enumerate(names):
        A[0:3, 3 * k:3 * k + 3] = np.eye(3)
        d = feet[nm] - com
        # cross-product matrix of d, so that A @ F gives d x F
        A[3:6, 3 * k:3 * k + 3] = np.array([
            [0.0, -d[2], d[1]],
            [d[2], 0.0, -d[0]],
            [-d[1], d[0], 0.0],
        ])
    b = np.zeros(6)
    b[0:3] = mass * (accel + np.array([0.0, 0.0, GRAVITY]))
    return names, A, b


def contact_forces(controller, phase: float, n: int = 240,
                   tangential_penalty: float = 25.0,
                   cyc: "CycleData | None" = None) -> ContactSolution:
    """Solve for per-foot ground-reaction forces at ``phase``.

    With 3 or 4 point contacts the problem is statically INDETERMINATE — 6
    balance equations, 9 or 12 unknowns — so a choice of distribution is
    required. We take the weighted minimum-norm solution that penalises
    TANGENTIAL force components ``tangential_penalty`` times more than vertical
    ones. That biases the answer toward vertical support, which is both what a
    sane controller wants and what minimises the friction demand.

    ⚠️ This is a heuristic, not the friction-optimal distribution, and it does
    not enforce ``fz >= 0`` as a constraint — violations are REPORTED
    (``unilateral_ok``) rather than silently projected away, because a foot that
    wants to pull is a real finding about the gait, not a solver artefact.
    """
    if cyc is None:
        cyc = cycle(controller, n)
    i = cyc.index(phase)
    feet, com, accel, mass = cyc.feet[i], cyc.com[i], cyc.accel[i], cyc.mass

    # ACTIVE-SET solve. The plain weighted least-norm can hand a foot a NEGATIVE
    # normal force even when a valid non-negative distribution exists -- with 4
    # contacts the vertical split is indeterminate and least-norm has no reason
    # to respect unilaterality. So: solve, and if a foot comes out pulling, drop
    # it entirely (a foot with no normal load carries no friction either) and
    # re-solve on the rest. Converges in at most n_feet passes.
    active = sorted(feet)
    forces, mu = {}, {}
    residual = 0.0
    while True:
        sub = {nm: feet[nm] for nm in active}
        names, A, b = _balance_system(sub, com, mass, accel)
        w = np.tile(np.array([tangential_penalty, tangential_penalty, 1.0]), len(names))
        Minv = np.diag(1.0 / (w * w))
        F = Minv @ A.T @ np.linalg.pinv(A @ Minv @ A.T) @ b
        fz = {nm: F[3 * k + 2] for k, nm in enumerate(names)}
        worst = min(fz, key=fz.get)
        if fz[worst] >= -1e-9 or len(active) <= 3:
            residual = float(np.linalg.norm(A @ F - b))
            for k, nm in enumerate(names):
                forces[nm] = F[3 * k:3 * k + 3]
            break
        active.remove(worst)

    for nm in feet:                       # feet dropped by the active set carry nothing
        forces.setdefault(nm, np.zeros(3))
    for nm, f in forces.items():
        mu[nm] = (float(np.hypot(f[0], f[1]) / f[2]) if f[2] > 1e-9
                  else (0.0 if abs(f[2]) <= 1e-9 and np.hypot(f[0], f[1]) <= 1e-9
                        else float("inf")))
    return ContactSolution(
        phase=float(phase) % 1.0, forces=forces, com=com, accel=accel,
        mu_required=mu, residual=residual,
    )


def zero_moment_point(controller, phase: float, n: int = 240,
                      cyc: "CycleData | None" = None) -> ZMPResult:
    """ZMP at ``phase`` and its signed margin inside the support polygon.

    For a flat ground plane and ``dH/dt = 0`` the ZMP reduces to the CoM ground
    projection shifted by the horizontal acceleration::

        zmp = com_xy - (com_z - ground_z) / (az + g) * a_xy

    i.e. the faster the body accelerates sideways, the further the effective
    pressure point slides the OTHER way. When acceleration is zero this is
    exactly the M5 static check, which is the sanity condition for this module.
    """
    from .stability import polygon_stability_margin

    if cyc is None:
        cyc = cycle(controller, n)
    i = cyc.index(phase)
    stance = {nm: (v[0], v[1]) for nm, v in cyc.feet[i].items()}
    com_xy = cyc.com[i][:2]
    height = cyc.com[i][2] - cyc.ground_z
    a = cyc.accel[i]
    denom = a[2] + GRAVITY
    zmp = com_xy - (height / denom) * a[:2] if abs(denom) > 1e-9 else com_xy

    margin = polygon_stability_margin(tuple(zmp), stance).margin
    return ZMPResult(phase=float(phase) % 1.0, zmp=zmp,
                     com_ground=com_xy, margin=margin)


# ------------------------------------------------------------------- summaries
def sweep(controller, n: int = 240, mu: float = 0.8) -> dict:
    """Run the whole cycle and summarise what the dynamics say about the gait.

    Returns a dict with the worst-case ZMP margin, the peak per-foot friction
    demand, whether any foot has to pull, and the largest ZMP excursion away
    from the static CoM projection — that last number being the size of the
    error the quasi-static milestones were carrying.
    """
    cyc = cycle(controller, n)
    zs = [zero_moment_point(controller, i / n, n, cyc=cyc) for i in range(n)]
    cs = [contact_forces(controller, i / n, n, cyc=cyc) for i in range(n)]
    return {
        "zmp_margin_min": min(z.margin for z in zs),
        "zmp_stable": all(z.is_stable for z in zs),
        "zmp_excursion_max": max(z.excursion for z in zs),
        "peak_mu": max(c.peak_mu for c in cs),
        "aggregate_mu": max(c.aggregate_mu for c in cs),
        "min_normal": min(c.min_normal for c in cs),
        "unilateral_ok": all(c.unilateral_ok for c in cs),
        "feasible": all(c.feasible(mu) for c in cs) and all(z.is_stable for z in zs),
        "max_residual": max(c.residual for c in cs),
        "peak_lateral_accel": float(np.abs(cyc.accel[:, 1]).max()),
    }


def compare_with_hand_calc(controller, n: int = 240) -> dict:
    """Compare the M5 hand calculation against the resolved dynamics.

    M5 estimated the sway-reversal cost as a single bang-bang lump,
    ``a = 4d/w²``, and compared it to ``mu*g`` for the whole robot. This returns
    both that number and what the per-foot solve actually requires, so the
    approximation can be judged instead of trusted.
    """
    s = sweep(controller, n)
    hand_a = controller.crossover_accel()
    return {
        "hand_accel": hand_a,
        "hand_mu": hand_a / GRAVITY,
        "dynamic_peak_lateral_accel": s["peak_lateral_accel"],
        "dynamic_peak_mu": s["peak_mu"],
        "dynamic_aggregate_mu": s["aggregate_mu"],
        "ratio_mu": (s["aggregate_mu"] / (hand_a / GRAVITY)) if hand_a > 0 else float("nan"),
    }


def angular_momentum_caveat(controller, n: int = 240) -> dict:
    """Size the ``dH/dt = 0`` assumption instead of just declaring it.

    Estimates the angular-momentum rate the SWING leg contributes about the body
    CoM (treating it as a point mass on its own trajectory) and expresses it as
    an equivalent ZMP shift, so the reader can see whether ignoring it matters.
    """
    dt = controller.params.period / n
    mass = controller.body.total_mass
    height = 0.16                      # nominal CoM height, m [assumed]

    shifts = []
    for i in range(n):
        p = i / n
        st = controller.state(p)
        if not st.swing_legs:
            continue
        # Sum over ALL legs in flight. An earlier version required exactly ONE
        # swing leg, which silently skipped every phase of a TROT (diagonal pairs
        # mean two legs are always in flight) and returned a reassuring 0.00 mm
        # having evaluated nothing.
        total = np.zeros(2)
        ok = True
        for nm in st.swing_legs:
            pos = []
            for k in (-1, 0, 1):
                q = controller.state((p + k * (1.0 / n)) % 1.0)
                lq = q.legs[nm].q
                if lq is None:
                    break
                fp = controller.body.foot_world_position(q.spine_q, nm, lq)
                pos.append(np.array([fp[0], fp[1]]))
            if len(pos) != 3:
                ok = False
                break
            a_leg = (pos[2] - 2 * pos[1] + pos[0]) / (dt * dt)
            total = total + controller.body.legs[nm].params.mass * a_leg
        if not ok:
            continue
        # equivalent horizontal-force -> ZMP shift = F*h / (M*g)
        shifts.append(float(np.linalg.norm(total) * height / (mass * GRAVITY)))
    return {
        "swing_leg_zmp_shift_max": max(shifts) if shifts else 0.0,
        "swing_leg_zmp_shift_mean": float(np.mean(shifts)) if shifts else 0.0,
    }


# ===================================================================
# M7 — DYNAMIC gaits: when the support is a LINE, not a polygon
# ===================================================================
#
# A trot puts DIAGONAL pairs down together, so the support "polygon" degenerates
# to a LINE. Three things follow, and none of them are bugs:
#
#  1. ``support_polygon`` / ``zero_moment_point`` REFUSE to evaluate (they raise).
#     A ZMP margin inside a polygon is a static-stability idea; a line has no
#     interior, so the concept genuinely does not apply.
#  2. Two point contacts cannot produce a moment about the line joining them.
#     ``contact_forces`` therefore returns a NON-ZERO residual, and that residual
#     is not numerical noise -- it is the physically unbalanceable moment.
#  3. That moment has to come from somewhere. In a real trot it comes from
#     ``dH/dt`` -- the swinging legs and spine. This module reports how big it is
#     so the demand on those can be judged.
#
# The right stability question for a line support is the INVERTED PENDULUM one:
# the body topples about the support line with time constant 1/omega,
# omega = sqrt(g/h), and the next diagonal has to be placed to catch it. That is
# what ``line_balance`` and the capture point (DCM) below evaluate.

# Re-exported from gait.py so there is ONE definition of the trot timing.
from .gait import TROT_PHASE_OFFSETS  # noqa: E402,F401


def lipm_omega(height: float, g: float = GRAVITY) -> float:
    """Inverted-pendulum rate ``sqrt(g/h)`` (1/s). Divergence goes as e^(omega t)."""
    return float(np.sqrt(g / height))


@dataclass(frozen=True)
class LineBalance:
    """Toppling state of the body about a 2-contact (diagonal) support line.

    Attributes
    ----------
    phase : float
    feet : tuple[str, str]
        The diagonal pair in contact.
    offset : float
        SIGNED perpendicular distance (m) from the support line to the CoM ground
        projection. Zero means the CoM is directly over the line — the (unstable)
        equilibrium a trot rocks through.
    offset_rate : float
        Rate of change of that offset (m/s).
    dcm : float
        Divergent component of motion, ``offset + offset_rate / omega`` (m). This
        is the capture point measured from the line: to arrest the topple, the
        NEXT support line must reach at least this far.
    omega : float
        ``sqrt(g/h)`` for the current CoM height.
    unbalanced_moment : float
        Moment (N·m) about the support line that the two contacts CANNOT supply,
        and which ``dH/dt`` must therefore provide.
    """

    phase: float
    feet: tuple
    offset: float
    offset_rate: float
    dcm: float
    omega: float
    unbalanced_moment: float

    @property
    def time_to_fall(self) -> float:
        """Rough time (s) for the offset to grow e-fold. ``inf`` if centred."""
        return 1.0 / self.omega if self.offset != 0.0 else float("inf")


def support_line(cyc: "CycleData", i: int):
    """(point, unit direction) of the support line, or None if not 2 contacts."""
    feet = cyc.feet[i]
    if len(feet) != 2:
        return None
    a, b = (feet[nm] for nm in sorted(feet))
    d = (b - a)[:2]
    n = np.linalg.norm(d)
    if n < 1e-12:
        return None
    return a[:2], d / n


def line_balance(controller, phase: float, n: int = 240,
                 cyc: "CycleData | None" = None) -> "LineBalance | None":
    """Inverted-pendulum balance about the diagonal support line at ``phase``.

    Returns None when the support is not exactly two contacts (i.e. when the
    polygon-based ``zero_moment_point`` is the right tool instead).
    """
    if cyc is None:
        cyc = cycle(controller, n)
    i = cyc.index(phase)
    line = support_line(cyc, i)
    if line is None:
        return None
    a, d = line

    def signed(idx: int) -> float:
        ln = support_line(cyc, idx)
        if ln is None:
            return float("nan")
        p0, dd = ln
        r = cyc.com[idx][:2] - p0
        perp = r - np.dot(r, dd) * dd
        # sign from the 2D cross product dd x r
        return float(np.sign(dd[0] * r[1] - dd[1] * r[0]) * np.linalg.norm(perp))

    off = signed(i)
    dt = cyc.period / cyc.n
    # Differentiate ONLY within one contiguous stance block. Across a support
    # switch the pair changes and the sign convention flips, so a central
    # difference there is meaningless -- it produced a spurious ~240 mm capture
    # point before this guard existed.
    here = set(cyc.feet[i])
    ip, im = (i + 1) % cyc.n, (i - 1) % cyc.n
    fwd_ok = set(cyc.feet[ip]) == here
    bwd_ok = set(cyc.feet[im]) == here
    if fwd_ok and bwd_ok:
        rate = (signed(ip) - signed(im)) / (2.0 * dt)
    elif fwd_ok:
        rate = (signed(ip) - off) / dt
    elif bwd_ok:
        rate = (off - signed(im)) / dt
    else:
        rate = 0.0

    h = cyc.com[i][2] - cyc.ground_z
    w = lipm_omega(h)

    # Moment the contacts cannot supply = component of the required moment about
    # the support line. For a flat ground plane this is m*(g+az) times the
    # perpendicular offset.
    m = cyc.mass
    moment = abs(m * (GRAVITY + cyc.accel[i][2]) * off)

    return LineBalance(phase=float(phase) % 1.0, feet=tuple(sorted(cyc.feet[i])),
                       offset=off, offset_rate=rate,
                       dcm=off + rate / w, omega=w, unbalanced_moment=moment)


def trot_sweep(controller, n: int = 240) -> dict:
    """Summarise a diagonal-support (trot) cycle.

    ``zero_moment_point`` cannot be used here — see the section banner. The
    figures returned are the inverted-pendulum ones instead.
    """
    cyc = cycle(controller, n)
    bs = [line_balance(controller, i / n, n, cyc=cyc) for i in range(n)]
    bs = [b for b in bs if b is not None]
    if not bs:
        return {"line_support_fraction": 0.0}
    offs = np.array([b.offset for b in bs])
    stance = controller.params.duty_factor * controller.params.period
    w = float(np.mean([b.omega for b in bs]))
    return {
        "line_support_fraction": len(bs) / n,
        "offset_min": float(offs.min()),
        "offset_max": float(offs.max()),
        "offset_abs_max": float(np.abs(offs).max()),
        "crosses_zero": bool(offs.min() < 0.0 < offs.max()),
        "dcm_abs_max": float(max(abs(b.dcm) for b in bs)),
        "unbalanced_moment_max": float(max(b.unbalanced_moment for b in bs)),
        "omega": w,
        "stance_time_constants": stance * w,
        "divergence": float(np.exp(stance * w)),
    }


def swing_leg_moment(controller, phase: float, n: int = 240,
                     cyc: "CycleData | None" = None,
                     include_spin: bool = True) -> dict:
    """Angular-momentum rate the SWINGING legs supply about the support line.

    This answers the question ``line_balance`` raises: two contacts cannot produce
    a moment about the line joining them, so the ``unbalanced_moment`` has to come
    from ``dH/dt``. In a trot the obvious source is the two legs in flight.

    Each leg is treated as a point mass at its own CoM::

        dH/dt = sum_i  m_i * (r_i - r_com) x (a_i - a_com)

    and the component along the support-line direction is what counts, since only
    that component can oppose the topple.

    Both angular-momentum terms are included (M13):

    - **orbital** — the leg's mass swinging about the body CoM, ``m r x a``;
    - **spin** — each link rotating about its OWN CoM, ``I_i * alpha_i``, with each
      link treated as a slender rod (``I = m L^2 / 12``). A leg link rotates about
      the LATERAL axis, so this term is almost pure pitch; only its projection onto
      the support line counts, and the diagonal is mostly fore-aft.

    Set ``include_spin=False`` to recover the point-mass-only estimate that
    milestones up to M12 used.

    Returns ``{"available", "required", "ratio", "orbital", "spin"}`` in N·m.
    """
    if cyc is None:
        cyc = cycle(controller, n)
    i = cyc.index(phase)
    line = support_line(cyc, i)
    bal = line_balance(controller, phase, n, cyc=cyc)
    if line is None or bal is None:
        return {"available": 0.0, "required": 0.0, "ratio": float("nan")}
    _, d = line
    d3 = np.array([d[0], d[1], 0.0])
    dt = cyc.period / cyc.n

    def leg_com(idx: int, name: str):
        st = controller.state(idx / cyc.n)
        q = st.legs[name].q
        if q is None:
            return None
        c = controller.body.leg_com_world(st.spine_q, name, q)
        v = np.asarray(c.com, dtype=float)
        return np.array([v[0], 0.0, v[1]]) if v.shape == (2,) else v

    st_now = controller.state(i / cyc.n)
    orbital = np.zeros(3)
    spin = np.zeros(3)
    for name in st_now.swing_legs:
        pts = [leg_com((i + k) % cyc.n, name) for k in (-1, 0, 1)]
        if any(p is None for p in pts):
            continue
        a_leg = (pts[2] - 2 * pts[1] + pts[0]) / (dt * dt)
        r = pts[1] - cyc.com[i]
        m_leg = controller.body.legs[name].params.mass
        orbital = orbital + m_leg * np.cross(r, a_leg - cyc.accel[i])

        if not include_spin:
            continue
        # SPIN term. Each link is a slender rod, I = m L^2 / 12 about its own CoM,
        # rotating about the LATERAL (y) axis at the cumulative joint angle. So
        # dH_spin/dt is a pure PITCH moment -- it reaches the support line only
        # through that line's y-component, and the diagonal is mostly fore-aft.
        lp = controller.body.leg_model_for(name).params
        lengths = np.array([lp.l1, lp.l2, lp.l3, lp.l4], dtype=float)
        masses = np.asarray(lp.link_mass, dtype=float)
        qs = []
        for k in (-1, 0, 1):
            stk = controller.leg_state(((i + k) % cyc.n) / cyc.n, name)
            if stk.q is None:
                qs = []
                break
            qs.append(np.asarray(stk.q, dtype=float))
        if len(qs) != 3:
            continue
        # absolute link angles: cumulative sums of the joint angles
        ang = [np.cumsum(np.append(q, q[-1] * 0.0 + lp.paw_angle)) for q in qs]
        alpha = (ang[2] - 2 * ang[1] + ang[0]) / (dt * dt)
        I = masses * lengths * lengths / 12.0
        spin = spin + np.array([0.0, float(np.sum(I * alpha)), 0.0])

    total = orbital + spin
    available = abs(float(np.dot(total, d3)))
    required = bal.unbalanced_moment
    return {"available": available, "required": required,
            "ratio": available / required if required > 1e-12 else float("inf"),
            "orbital": abs(float(np.dot(orbital, d3))),
            "spin": abs(float(np.dot(spin, d3)))}


def swing_joint_torque(controller, leg_name: str, phase: float, n: int = 240) -> np.ndarray:
    """Joint torques (N·m) needed to ACCELERATE one leg's own mass at ``phase``.

    The contact-force solve above covers only STANCE legs pushing on the ground.
    A swinging leg touches nothing, yet still needs torque — to accelerate its own
    links. In a crawl that is negligible; in a trot it is the dominant term, and
    it grows as 1/T², so it is what actually caps trot speed.

    Each link is treated as a point mass at its CoM (``mass.leg_link_coms``), its
    Cartesian acceleration is taken from the commanded trajectory by central
    differences, and the torques follow from the per-link Jacobians::

        tau = sum_links  J_link(q)^T * (m_link * a_link)

    ⚠️ Point-mass links: this ignores each link's spin inertia about its own CoM,
    so it UNDER-estimates. ⚠️ It also requires a C1 foot trajectory —
    ``GaitParams.swing_profile`` must be "matched"; on the legacy "cycloid" the
    accelerations are impulsive and this number is meaningless (grid-dependent).
    """
    from .mass import leg_link_coms

    leg = controller.body.leg_model_for(leg_name)
    dt = controller.params.period / n
    i = int(round((float(phase) % 1.0) * n)) % n

    qs = []
    for k in (-1, 0, 1):
        st = controller.leg_state(((i + k) % n) / n, leg_name)
        if st.q is None:
            return np.zeros(3)
        qs.append(np.asarray(st.q, dtype=float))

    coms = [np.asarray(leg_link_coms(leg, q), dtype=float) for q in qs]   # (4, 2)
    accel = (coms[2] - 2.0 * coms[0 + 1] + coms[0]) / (dt * dt)      # (n_links, 2)
    masses = np.asarray(leg.params.link_mass, dtype=float)

    q = qs[1]
    pts = leg.joint_positions(q)          # joint origins in the hip frame
    tau = np.zeros(3)
    for li, (a, m_l) in enumerate(zip(accel, masses)):
        f = m_l * a                        # required force on this link (x, z)
        for j in range(3):
            if j > li:                     # joint distal to the link: no leverage
                continue
            r = coms[1][li] - pts[j]       # lever from joint j to this link's CoM
            tau[j] += r[0] * f[1] - r[1] * f[0]     # 2D cross product
    return tau


# ===================================================================
# Contact SENSING — what closed-loop balance can actually observe
# ===================================================================

def grf_observability(controller, phase: float, leg_name: str,
                      sensed_joints: tuple = (0, 1)) -> float:
    """Error amplification when inferring ground force from JOINT torques.

    Closed-loop balance needs per-foot ground-reaction force. The project already
    buys part of that for free: ADR-0004 puts load cells at the JOINT end of the
    hip and stifle tendons, and a POINT contact exerts no moment, so the foot
    force is only two unknowns ``(fx, fz)`` related to joint torque by
    ``tau = J^T F``. Two clean torque measurements are therefore *sufficient in
    principle* — the question is how well conditioned the inversion is.

    Returns the 2x2 condition number: the factor by which a relative error in the
    measured torques is amplified in the inferred force. 1 is perfect; large
    means the estimate is worthless however good the load cell.

    Measured on the shipped gaits this is ~3.2–3.4 on the FORE legs but degrades
    to a median 7.5 and a worst 36 on the HIND legs just before liftoff — which is
    precisely when load is transferring to the other diagonal and accurate load
    sharing matters most. That is the quantitative case for a direct paw sensor;
    see mechanical/TACTILE_SENSING_SPEC.md.

    ⚠️ Conditioning is a *best case*: it assumes an ideal point contact (a
    compliant pad exerts a small moment), and ignores that a joint-torque estimate
    cannot distinguish ground contact from limb inertia at all without a dynamics
    model — so it can never provide fast touchdown DETECTION, only force.
    """
    st = controller.state(phase)
    q = st.legs[leg_name].q
    if q is None:
        return float("inf")
    J = controller.body.leg_model_for(leg_name).jacobian(q)
    rows = list(sensed_joints)
    sub = J.T[np.ix_(rows, [0, 1])]          # sensed joints x (fx, fz)
    return float(np.linalg.cond(sub))


def grf_observability_sweep(controller, n: int = 96,
                            sensed_joints: tuple = (0, 1)) -> dict:
    """Per-leg conditioning of the joint-torque ground-force estimate over a cycle."""
    out = {}
    for name in ("LF", "RF", "LR", "RR"):
        vals = [grf_observability(controller, i / n, name, sensed_joints)
                for i in range(n) if controller.is_stance(i / n, name)]
        vals = [v for v in vals if np.isfinite(v)]
        if vals:
            out[name] = {"median": float(np.median(vals)), "worst": float(max(vals))}
    return out
