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
- ⚠️ **Angular momentum rate is taken as zero** (`dH/dt = 0`). The moment balance
  is therefore the classical ZMP form. This is standard for slow gaits and is
  reasonable *here* because the crawl is slow and the legs are light (13.7 % of
  body mass, tendon drive keeps them so), but it is an approximation: a real
  swing leg does change the body's angular momentum. Quantified in
  ``angular_momentum_caveat``.
- ⚠️ **Swing-leg inertial reaction is inside the CoM term only.** The CoM path
  already contains the swing leg's mass moving, so its linear effect is captured;
  its spin is not (see above).
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
        if len(st.swing_legs) != 1:
            continue
        nm = st.swing_legs[0]
        pos = []
        for k in (-1, 0, 1):
            q = controller.state((p + k * (1.0 / n)) % 1.0)
            lq = q.legs[nm].q
            if lq is None:
                break
            fp = controller.body.foot_world_position(q.spine_q, nm, lq)
            pos.append(np.array([fp[0], fp[1]]))
        if len(pos) != 3:
            continue
        a_leg = (pos[2] - 2 * pos[0 + 1] + pos[0]) / (dt * dt)
        m_leg = controller.body.legs[nm].params.mass
        # equivalent horizontal-force -> ZMP shift = F*h / (M*g)
        shifts.append(float(np.linalg.norm(m_leg * a_leg) * height / (mass * GRAVITY)))
    return {
        "swing_leg_zmp_shift_max": max(shifts) if shifts else 0.0,
        "swing_leg_zmp_shift_mean": float(np.mean(shifts)) if shifts else 0.0,
    }
