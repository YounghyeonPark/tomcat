"""Static (quasi-static) stability margin: CoM projection vs. the support base.

The gait generator has, until M4, validated stability only by counting stance
legs (">= 3 feet down"). A foot COUNT proves nothing on its own: the body tips
unless the centre of mass projects INSIDE the support base. This module supplies
that missing check for the sagittal model.

HONEST SCOPE — a 2D "support polygon" is really a FORE-AFT INTERVAL
--------------------------------------------------------------------
In the real 3D machine the support base is the convex POLYGON of the stance
footprints (a triangle for a 3-foot walk), and static stability requires the CoM
ground projection to lie inside it, with the tipping margin being the distance to
the nearest polygon EDGE. This model is SAGITTAL ONLY (x forward, z up); the
lateral y coordinate does not exist, so the polygon collapses to the interval
``[x_rearmost_foot, x_frontmost_foot]`` and the margin is a pure FORE-AFT
(pitch) quantity.

Consequences you must not forget when reading these numbers:
- A positive margin here is a **NECESSARY but NOT SUFFICIENT** condition for real
  static stability. It says the body will not pitch forward over the front feet
  or backward over the rear feet. It says NOTHING about ROLL.
- The lateral edges of a real 3-foot support triangle are usually the BINDING
  ones in a walk (that is exactly why cats shift their body laterally when they
  walk slowly). Capturing that requires the 3D extension (frontal-plane leg
  abduction + a y-aware spine), which is a later milestone.
- Both feet of a girdle project onto the SAME sagittal x in this model, so a
  3-foot stance and a 4-foot stance can produce an identical interval.
- Quasi-static: no inertia, no ZMP, no friction-cone or slip check. Valid only
  in the low-speed limit where accelerations are negligible.

Conventions
-----------
- Gravity is along -z, so the "ground projection" of the CoM is just its x
  coordinate; z never enters the margin.
- Margins are SIGNED distances in metres, POSITIVE = inside the support
  interval. ``margin_front`` is the distance from the CoM to the FRONT edge
  (shrinks as the body pitches nose-over), ``margin_rear`` the distance to the
  REAR edge. The scalar ``margin`` is the smaller of the two, i.e. the distance
  to the nearest tipping edge.
- Everything is frame-invariant under translation: the CoM x and the foot x must
  simply be expressed in the SAME frame (this codebase uses the body-ground
  frame, spine base at the origin).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class SupportInterval:
    """The sagittal projection of the support polygon: a fore-aft interval.

    Attributes
    ----------
    rear, front : float
        x of the rearmost / frontmost stance foot (m). NaN when no foot is down.
    feet : tuple[str, ...]
        Names of the stance feet that formed it (sorted, for reporting).
    """

    rear: float
    front: float
    feet: tuple[str, ...] = ()

    @classmethod
    def from_feet(cls, foot_x: Mapping[str, float] | Sequence[float]) -> "SupportInterval":
        """Build from stance-foot x positions (a name->x map or a bare sequence)."""
        if isinstance(foot_x, Mapping):
            names = tuple(sorted(foot_x))
            xs = np.asarray([float(foot_x[n]) for n in names], dtype=float)
        else:
            xs = np.asarray(list(foot_x), dtype=float).reshape(-1)
            names = tuple(f"foot{i}" for i in range(xs.size))
        if xs.size == 0:
            return cls(rear=float("nan"), front=float("nan"), feet=())
        return cls(rear=float(xs.min()), front=float(xs.max()), feet=names)

    @property
    def n_feet(self) -> int:
        return len(self.feet)

    @property
    def width(self) -> float:
        """Fore-aft length of the support interval (m); 0 if degenerate."""
        if self.n_feet == 0:
            return 0.0
        return float(self.front - self.rear)

    @property
    def center(self) -> float:
        """Midpoint of the interval (m); NaN if no foot is down."""
        if self.n_feet == 0:
            return float("nan")
        return 0.5 * (self.front + self.rear)


@dataclass(frozen=True)
class StabilityMargin:
    """Signed fore-aft static stability margin of one posture.

    See the module docstring for the (important) 2D caveat: this is a fore-aft
    interval, not a true support polygon, so a positive margin is necessary but
    not sufficient for real static stability.

    Attributes
    ----------
    com_x : float
        Ground projection of the whole-body CoM (m), same frame as the feet.
    support : SupportInterval
        The stance feet's fore-aft interval.
    """

    com_x: float
    support: SupportInterval

    @property
    def margin_front(self) -> float:
        """Distance from the CoM to the FRONT tipping edge (m); + = behind it."""
        if self.support.n_feet == 0:
            return float("-inf")
        return float(self.support.front - self.com_x)

    @property
    def margin_rear(self) -> float:
        """Distance from the CoM to the REAR tipping edge (m); + = ahead of it."""
        if self.support.n_feet == 0:
            return float("-inf")
        return float(self.com_x - self.support.rear)

    @property
    def margin(self) -> float:
        """Signed distance to the NEAREST tipping edge (m). Negative = outside."""
        return min(self.margin_front, self.margin_rear)

    @property
    def is_stable(self) -> bool:
        """True if the CoM projects strictly inside the support interval.

        Requires at least two DISTINCT sagittal foot positions: a single contact
        x (or all feet at one x) gives a zero-width interval, which can never be
        strictly stable in the fore-aft sense -- the body is balanced on a knife
        edge and any disturbance tips it.
        """
        return self.support.n_feet > 0 and self.margin > 0.0

    @property
    def normalized_margin(self) -> float:
        """``margin`` as a fraction of the interval HALF-width (dimensionless).

        1.0 = CoM exactly at the interval centre, 0.0 = on an edge, negative =
        outside. NaN for a degenerate (zero-width) interval.
        """
        half = 0.5 * self.support.width
        if half <= 0.0:
            return float("nan")
        return self.margin / half

    @property
    def tipping_edge(self) -> str:
        """Which edge is closest: ``"front"``, ``"rear"``, or ``"none"``."""
        if self.support.n_feet == 0:
            return "none"
        return "front" if self.margin_front <= self.margin_rear else "rear"

    def report(self) -> str:
        if self.support.n_feet == 0:
            return "no stance feet -> unsupported (no static stability)"
        return (
            f"CoM x {self.com_x * 1e3:+7.1f} mm  support "
            f"[{self.support.rear * 1e3:+7.1f}, {self.support.front * 1e3:+7.1f}] mm "
            f"({self.support.n_feet} feet)  margin {self.margin * 1e3:+7.1f} mm "
            f"(front {self.margin_front * 1e3:+.1f}, rear {self.margin_rear * 1e3:+.1f}) "
            f"-> {'STABLE' if self.is_stable else 'UNSTABLE'} "
            f"[nearest edge: {self.tipping_edge}]"
        )


def sagittal_stability_margin(
    com_x: float,
    stance_foot_x: Mapping[str, float] | Sequence[float],
) -> StabilityMargin:
    """Fore-aft static stability margin of a CoM against a set of stance feet.

    Parameters
    ----------
    com_x : float
        Whole-body CoM ground projection (m). In this sagittal model, gravity is
        along -z, so the projection is simply the CoM's x coordinate.
    stance_foot_x : mapping or sequence
        x position (m) of every foot IN STANCE, in the same frame as ``com_x``.
        Swing feet must be excluded by the caller -- a lifted foot supports
        nothing.

    Returns
    -------
    StabilityMargin
        ``.margin`` is the signed distance to the nearest tipping edge and
        ``.is_stable`` the boolean check. Quasi-static and sagittal only; see the
        module docstring.
    """
    return StabilityMargin(
        com_x=float(com_x), support=SupportInterval.from_feet(stance_foot_x)
    )


def centering_shift(margin: StabilityMargin) -> float:
    """Fore-aft shift (m) that would move the CoM to the support-interval CENTRE.

    A diagnostic, not a controller: positive means the CoM must move FORWARD
    (equivalently, the support base must move rearward by the same amount) to sit
    at the middle of the interval. Useful for reporting *how far off* a posture
    is, and for sizing a geometry correction. NaN if nothing is in stance.
    """
    return margin.support.center - margin.com_x
