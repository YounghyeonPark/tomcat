"""Rigid-body MASS properties: per-link CoMs, sub-assembly and whole-body CoM.

This is the M4 "real mass" layer. Everything above it (leg.py, spine.py, gait.py)
was previously MASSLESS; this module supplies the mass bookkeeping that
``WholeBody.center_of_mass``, ``stability.py`` and ``whole_body_budget`` build on.

SCOPE — QUASI-STATIC WITH REAL MASS
-----------------------------------
Masses here are used for **gravity and centre-of-mass geometry only**. There are
NO velocities, accelerations, momenta or inertia tensors: a link is a point mass
at a fixed fraction along its length, and every result is a static posture
quantity. Full Newton-Euler (link inertia tensors, Coriolis/centrifugal terms,
ground-reaction dynamics) is a LATER milestone; nothing in this module should be
read as a dynamic model.

Frames & conventions (identical to leg.py / spine.py)
-----------------------------------------------------
- Sagittal plane only: x forward, z up. Gravity acts along -z, so a "ground
  projection" of the CoM is simply its x coordinate.
- Leg CoMs are returned in that leg's own HIP frame unless a function name says
  otherwise; spine / whole-body CoMs are in the BODY-GROUND frame (spine base,
  i.e. the rear/pelvic girdle vertebra, at the origin).
- All masses are kg, all positions m.

Modelling assumptions that materially affect results (flagged per the
engineering standards):
- **Point mass per link.** Each link's mass acts at ``link_com_frac`` along it.
  Rotational inertia about that point is IGNORED, so this is exact for statics
  and wrong for dynamics.
- **Left/right collapse.** The 2D sagittal projection puts both legs of a girdle
  at the same mount, so a "leg" CoM here is one leg's mass at one sagittal
  location; the pair is counted twice at the same x. Lateral (y) CoM offset and
  therefore ROLL balance are outside this model.
- **Rigid links, no payload.** No battery/electronics/tail masses are broken out
  yet; they are folded into the girdle and segment placeholders in params.py.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

from .params import SpineParams


@dataclass(frozen=True)
class ComResult:
    """Mass and centre of mass of one body or sub-assembly.

    Attributes
    ----------
    mass : float
        Total mass (kg).
    com : np.ndarray
        (2,) centre of mass ``(x, z)`` in whatever frame the caller worked in
        (m). For a zero-mass assembly this is the origin and should be ignored.
    """

    mass: float
    com: np.ndarray

    @property
    def moment(self) -> np.ndarray:
        """First mass moment ``m * r`` (kg·m) -- the additive quantity."""
        return self.mass * self.com

    @property
    def x(self) -> float:
        """CoM x, i.e. the GROUND PROJECTION in this sagittal model (m)."""
        return float(self.com[0])

    @property
    def z(self) -> float:
        """CoM height above the body-frame origin (m)."""
        return float(self.com[1])

    def translated(self, offset) -> "ComResult":
        """Same mass, CoM shifted by ``offset`` (x, z)."""
        return ComResult(self.mass, self.com + np.asarray(offset, dtype=float))


def point_masses_com(masses, positions) -> ComResult:
    """CoM of a set of point masses.

    ``masses`` is length-n, ``positions`` is (n, 2). Returns the summed mass and
    the mass-weighted mean position; a total mass of zero yields the origin.
    """
    m = np.asarray(masses, dtype=float).reshape(-1)
    p = np.asarray(positions, dtype=float).reshape(-1, 2)
    if m.shape[0] != p.shape[0]:
        raise ValueError(
            f"masses ({m.shape[0]}) and positions ({p.shape[0]}) length mismatch"
        )
    total = float(m.sum())
    if total <= 0.0:
        return ComResult(0.0, np.zeros(2))
    return ComResult(total, (m[:, None] * p).sum(axis=0) / total)


def combine(parts: Iterable[ComResult]) -> ComResult:
    """Combine sub-assembly CoMs into one (first moments add)."""
    parts = list(parts)
    total = float(sum(p.mass for p in parts))
    if total <= 0.0:
        return ComResult(0.0, np.zeros(2))
    moment = np.zeros(2)
    for p in parts:
        moment = moment + p.moment
    return ComResult(total, moment / total)


# ------------------------------------------------------------------------ leg
def leg_link_coms(leg, q) -> np.ndarray:
    """(4, 2) CoM of each leg link in the leg's own HIP frame.

    ``leg`` is a ``LegModel`` (duck-typed: it only needs ``.params`` and
    ``.joint_positions``). Link i's CoM sits ``link_com_frac[i]`` of the way from
    its proximal joint to its distal joint, so it follows the PASSIVE paw
    correctly too (the paw rides at its rigid offset off the metatarsus).
    """
    pts = np.asarray(leg.joint_positions(q), dtype=float)  # (5, 2)
    fracs = np.asarray(leg.params.link_com_frac, dtype=float)  # (4,)
    return pts[:-1] + fracs[:, None] * (pts[1:] - pts[:-1])


def leg_com(leg, q) -> ComResult:
    """Mass + CoM of one leg in its own HIP frame (hip at the origin)."""
    return point_masses_com(leg.params.link_mass, leg_link_coms(leg, q))


# ---------------------------------------------------------------------- spine
def spine_segment_coms(vertebra_xy, com_frac) -> np.ndarray:
    """(N, 2) CoM of each spine segment given the (N+1, 2) vertebra positions.

    Segment i spans vertebra i (inboard / rear) to vertebra i+1 (outboard /
    front); its mass acts ``com_frac[i]`` of the way along it. Because the
    vertebra positions come from the spine FK, this automatically tracks the
    ARCHED geometry -- bending the spine moves the segment CoMs.
    """
    pts = np.asarray(vertebra_xy, dtype=float)
    f = np.asarray(com_frac, dtype=float)
    if pts.shape[0] != f.shape[0] + 1:
        raise ValueError(
            f"got {pts.shape[0]} vertebra positions for {f.shape[0]} segments; "
            "expected n_segments + 1"
        )
    return pts[:-1] + f[:, None] * (pts[1:] - pts[:-1])


def spine_chain_com(vertebra_xy, segment_mass, com_frac) -> ComResult:
    """Mass + CoM of the spine SEGMENTS alone (girdles excluded)."""
    return point_masses_com(
        segment_mass, spine_segment_coms(vertebra_xy, com_frac)
    )


# ------------------------------------------------------- fore / hind bookkeeping
@dataclass(frozen=True)
class QuarterMasses:
    """Fore/hind weight split of the body (the "~60% front-heavy" number).

    "Forequarters" and "hindquarters" are not disjoint rigid bodies -- the trunk
    spans both -- so the split is defined by the **lever rule**: for a STRAIGHT
    trunk simply supported at its two girdles, a distributed mass whose CoM sits
    a fraction ``s`` of the way from the rear girdle to the front girdle puts
    ``s`` of its weight on the front support and ``1 - s`` on the rear. Girdle
    masses and the legs hanging off them are attributed wholly to their own end
    (s = 1 for the front girdle, s = 0 for the rear).

    This is a POSTURE-INDEPENDENT bookkeeping quantity computed on the straight
    spine, deliberately separate from ``WholeBody.center_of_mass`` (which is the
    actual posed CoM and moves with the joints).

    Attributes
    ----------
    fore, hind : float
        Forequarter / hindquarter share of the body mass (kg). They sum to
        ``total``.
    total : float
        Whole-body mass (kg).
    """

    fore: float
    hind: float
    total: float

    @property
    def fore_fraction(self) -> float:
        return self.fore / self.total if self.total else 0.0

    @property
    def hind_fraction(self) -> float:
        return self.hind / self.total if self.total else 0.0

    def report(self) -> str:
        return (
            f"total {self.total:.3f} kg = forequarters {self.fore:.3f} kg "
            f"({self.fore_fraction * 100:.1f}%) + hindquarters {self.hind:.3f} kg "
            f"({self.hind_fraction * 100:.1f}%)"
        )


def quarter_masses(
    spine: SpineParams,
    fore_leg_masses: Sequence[float] = (),
    hind_leg_masses: Sequence[float] = (),
) -> QuarterMasses:
    """Fore/hind mass split by the lever rule (see ``QuarterMasses``).

    ``fore_leg_masses`` / ``hind_leg_masses`` are the total masses of the legs
    hanging off the FRONT and REAR girdles respectively (one entry per leg).
    """
    length = spine.total_length
    if length <= 0.0:
        raise ValueError("spine total_length must be > 0 to split fore/hind")

    # Straight-spine segment CoM distance from the rear girdle.
    edges = np.concatenate([[0.0], np.cumsum(spine.segment_lengths)])
    fracs = np.asarray(spine.segment_com_frac, dtype=float)
    s = (edges[:-1] + fracs * np.diff(edges)) / length  # fore share per segment

    seg_m = np.asarray(spine.segment_mass, dtype=float)
    fore = float((seg_m * s).sum()) + spine.front_girdle_mass + float(
        sum(fore_leg_masses)
    )
    hind = float((seg_m * (1.0 - s)).sum()) + spine.rear_girdle_mass + float(
        sum(hind_leg_masses)
    )
    return QuarterMasses(fore=fore, hind=hind, total=fore + hind)
