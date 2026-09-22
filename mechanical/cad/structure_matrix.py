# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The leg's STRUCTURE MATRIX, swept over the ROM, and the coupling trade.

⚠️ **The leg has never been synthesised, only measured after the fact.**
[ADR-0042](../../docs/DESIGN_DECISIONS.md#adr-0042) measured the coupling at one
pose; [ADR-0049](../../docs/DESIGN_DECISIONS.md#adr-0049) found a moment arm
reversing sign inside its own ROM; [ADR-0053](../../docs/DESIGN_DECISIONS.md#adr-0053)
found the fore leg's routing was never mirrored. Each arrived separately, by
measurement. The synthesis literature (LITERATURE_REVIEW.md Q7) says the object
being designed is the matrix `B` in `tau = B f`, and that the controllability
question is whether a positive tension solution exists **throughout the
workspace** -- not at one pose.

This file computes `B` for every tendon over the whole ROM and prices the one
design choice that is still open:

- **Option A, one via per passed joint** (what is built). A via-pulley
  concentric with the proximal axis kills the *tangent* term and not the *arc*
  term, so `B` gains an off-diagonal of exactly the via radius per radian.
- **Option B, a coaxial idler PAIR** -- a second idler of the same radius on the
  *distal* link, wrapped the opposite way. A proximal rotation adds `r dtheta`
  of wrap on one and removes it from the other, so the off-diagonal is zero by
  construction. The price is the second idler's wrap, and capstan friction is
  `e^(mu beta)` in it.

⚠️ **Option B's off-diagonal is ANALYTIC here, not re-solved.** Two coaxial
pulleys of equal radius are degenerate for a planar tangent solver -- the real
pair is separated along the joint axis, which a 2-D route cannot see. What is
measured is the COST: the wrap actually carried at each via, which is what the
second idler adds. The benefit is taken from the geometry, and it is the claim
that a build has to confirm.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "kinematics", "src"))

import leg_tendons as LT                                         # noqa: E402
import tendon_route as tr                                        # noqa: E402
import tomcat_trunk as TT                                         # noqa: E402
from tomcat_kin.params import DEFAULT_HINDLEG, DEFAULT_TENDON    # noqa: E402


JOINTS = ("hip", "knee", "ankle")

#: Every strand an antagonistic leg carries. ⚠️ `tomcat_leg_detail` builds five
#: of these -- it omits `("ankle", -1)` and draws a torsion return spring
#: instead, which is ADR-0002's Option B, rejected by ADR-0050 in M45.
STRANDS = (("hip", +1), ("hip", -1), ("knee", +1), ("knee", -1),
           ("ankle", +1), ("ankle", -1))

#: Which stations of each run are VIA-pulleys -- the ones that pass a joint, and
#: so the ones an idler pair would have to double. See `leg_tendons.stations`.
VIA_IDX = {"hip": (), "knee": (1,), "ankle": (1, 2)}

#: Which joints each tendon passes on its way out, in the same order.
PASSES = {"hip": (), "knee": ("hip",), "ankle": ("hip", "knee")}


def real_spools(role: str = "hind", side: float = +1.0):
    """The spool centres the TRUNK actually has, in the leg's hip frame.

    ⚠️ **Every wrap number in this project was measured without them.**
    `leg_tendons.route()` falls back to `SPOOL_OFFSET`, a single 2-D diagonal
    that was only ever right for the pre-M88 upright girdle, and
    `tomcat_assembly` overrides it precisely because the defaults miss the real
    motors by 20 to 44 mm. `route(spools=)` and `TT.leg_spools` both arrived in
    M91; [ADR-0083](../../docs/DESIGN_DECISIONS.md#adr-0083)'s capstan table is
    M78, so it is correct for a routing that has since been reconnected and has
    never been re-measured.
    """
    hx = 0.0 if role == "hind" else 195.0
    sp3 = TT.leg_spools(role, side)
    return {t: (p[0] - hx, p[2])
            for t, p in zip(JOINTS, sp3)}


#: ✅ **THE ROUTING RULE, verified strand by strand (M105).**
#:
#:     B[strand, passed_joint] == (wrap sense at that via) * VIA_R
#:
#: The off-diagonal is not a measurement, it is the sense the cable is threaded
#: with. So the condition for an antagonistic pair to be free of COMMON MODE --
#: a length change the single bidirectional spool cannot absorb, which
#: [ADR-0058](../../docs/DESIGN_DECISIONS.md#adr-0058) priced at 1716-3090 N
#: against a 638 N cable -- is just:
#:
#:     the two strands must wrap every SHARED via in OPPOSITE senses.
#:
#: That is checkable by eye on a drawing. It is also what `route()` does not
#: know: it minimises ONE strand's total wrap, so the pair's property is an
#: accident of a per-strand objective. The knee pair happens to satisfy it; the
#: ankle pair does not.
COMMON_MODE_RULE = "opposite wrap sense at every shared via"


def pair_options(q, tendon, spool_xz, spools=None, leg=DEFAULT_HINDLEG):
    """Every admissible `(senses, total_wrap)` for both strands of one pair.

    Enumerates the free senses rather than taking `route()`'s minimum, because
    the minimum is a per-STRAND answer and the pair is the object being designed.
    """
    sp = dict(real_spools() if spools is None else spools)
    sp[tendon] = tuple(spool_xz)
    n = len(LT.stations(q, tendon, +1, leg, spools=sp))
    free = [i for i in range(n) if i != LT._SHEAVE_IDX[tendon]]
    out = {}
    for side in (+1, -1):
        got = []
        for bits in range(1 << len(free)):
            sen = [float(side)] * n
            for k, i in enumerate(free):
                sen[i] = +1.0 if (bits >> k) & 1 else -1.0
            try:
                r = LT.route(q, tendon, side=side, leg=leg, senses=sen, spools=sp)
            except tr.NoTangent:
                continue
            got.append((sen, float(r["total_wrap"])))
        out[side] = got
    return out


def pair_best(q, tendon, spool_xz, spools=None, leg=DEFAULT_HINDLEG):
    """The cheapest COMMON-MODE-FREE routing of a pair: `(worst_wrap, s+, s-)`.

    `None` if no sense combination satisfies `COMMON_MODE_RULE` at this spool.
    """
    opt = pair_options(q, tendon, spool_xz, spools=spools, leg=leg)
    best = None
    for sa, wa in opt[+1]:
        for sb, wb in opt[-1]:
            if any(sa[k] == sb[k] for k in VIA_IDX[tendon]):
                continue
            w = max(wa, wb)
            if best is None or w < best[0]:
                best = (w, sa, sb)
    return best


def spool_search(tendon, q=None, xs=range(-110, -4, 10), zs=range(-20, 61, 10),
                 leg=DEFAULT_HINDLEG):
    """Sweep spool positions for the cheapest common-mode-free pair routing.

    ⚠️ **Geometry only.** The spools are motor centres the trunk owns and packs
    in rows; this sweep does not know about packing, collision, or whether a
    position is inside the girdle at all. It says where the ROUTING wants the
    motor, which is an input to that packing and not a substitute for it.
    """
    if q is None:
        import tomcat_leg_detail as LD
        q = LD.LegModel(leg).inverse((LD.FOOT_X, LD.FOOT_Z, LD.FOOT_PITCH))
    best = None
    for x in xs:
        for z in zs:
            b = pair_best(q, tendon, (float(x), float(z)), leg=leg)
            if b is not None and (best is None or b[0] < best[0]):
                best = (b[0], (float(x), float(z)), b[1], b[2])
    return best


def _q_grid(n: int, leg=DEFAULT_HINDLEG):
    """`n` samples per joint across the ROM, as a list of joint vectors."""
    lo = np.asarray(leg.q_min, float)
    hi = np.asarray(leg.q_max, float)
    # ⚠️ A full n^3 grid is the honest sweep but it is n^3 route solves. The ROM
    # question is whether any ENTRY changes sign, and an entry is a function of
    # all three angles, so the grid is kept coarse rather than factored.
    axes = [np.linspace(a, b, n) for a, b in zip(lo, hi)]
    return [np.array(c) for c in np.stack(
        np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)]


def B_at(q, spools="real", leg=DEFAULT_HINDLEG, h: float = 0.02,
         strands=STRANDS, senses=None):
    """`d(cable length)/d(joint)` in mm/rad. Rows are strands, columns joints.

    ⚠️ **The wrap senses are solved ONCE at `q` and held fixed** through the
    finite difference, the same discipline `leg_tendons.coupling_matrix` uses:
    letting the minimum-wrap search re-run inside the difference makes it
    straddle the discontinuity where the optimiser changes its mind.
    """
    if spools == "real":
        spools = real_spools()
    B = np.full((len(strands), 3), np.nan)
    for i, (tendon, side) in enumerate(strands):
        if senses is not None:
            if (tendon, side) not in senses:
                continue
            sen = senses[(tendon, side)]
        else:
            try:
                sen = LT.route(q, tendon, side=side, leg=leg,
                               spools=spools)["senses"]
            except tr.NoTangent:
                continue
        for j in range(3):
            lo, hi = np.array(q, float), np.array(q, float)
            lo[j] -= h
            hi[j] += h
            try:
                a = LT.route(lo, tendon, side=side, leg=leg, senses=sen,
                             spools=spools)["length"]
                b = LT.route(hi, tendon, side=side, leg=leg, senses=sen,
                             spools=spools)["length"]
            except tr.NoTangent:
                continue
            B[i, j] = (b - a) / (2.0 * h)
    return B


def wraps_at(q, spools="real", leg=DEFAULT_HINDLEG, strands=STRANDS):
    """`{strand: (total_wrap_rad, capstan, [wrap at each via])}`."""
    if spools == "real":
        spools = real_spools()
    out = {}
    for tendon, side in strands:
        try:
            r = LT.route(q, tendon, side=side, leg=leg, spools=spools)
        except tr.NoTangent:
            continue
        w = np.asarray(r["wraps"], float)
        out[(tendon, side)] = (float(r["total_wrap"]), float(r["capstan"]),
                               [float(w[k]) for k in VIA_IDX[tendon]])
    return out


def sweep(n: int = 5, spools="real", leg=DEFAULT_HINDLEG, strands=STRANDS,
          q_ref=None):
    """`B` over the ROM. Returns `{strand: {...}}` with the range of every entry.

    The question the synthesis literature asks is not what `B` is at the stance
    pose; it is whether any entry **changes sign** anywhere the gait can go. A
    row that changes sign is a tendon whose pulling direction is not a fixed
    direction in joint space, which is ADR-0049's finding stated structurally.
    """
    if q_ref is None:
        import tomcat_leg_detail as LD
        q_ref = LD.LegModel(leg).inverse((LD.FOOT_X, LD.FOOT_Z, LD.FOOT_PITCH))
    if spools == "real":
        spools = real_spools()
    grid = _q_grid(n, leg)
    # ⚠️ **The senses are frozen at `q_ref` for the WHOLE sweep, and the first
    # version of this was not.** Letting `route()` re-pick them at every pose
    # reported a sign change on five of six off-diagonals; frozen, every one is
    # dead constant at ±8.75 and the flags were the optimiser changing its
    # mind. On a real leg the cable is threaded once, so frozen is also the
    # physical case. `leg_tendons.coupling_matrix` already warned about exactly
    # this and the warning had to be re-learned.
    ref = {}
    for t, sd in strands:
        try:
            ref[(t, sd)] = LT.route(q_ref, t, side=sd, leg=leg,
                                    spools=spools)["senses"]
        except tr.NoTangent:
            pass
    acc = {s: [] for s in strands}
    fails = {s: 0 for s in strands}
    for q in grid:
        B = B_at(q, spools=spools, leg=leg, strands=strands, senses=ref)
        for i, s in enumerate(strands):
            if np.isnan(B[i]).any():
                fails[s] += 1
            else:
                acc[s].append(B[i])
    out = {}
    for s in strands:
        if not acc[s]:
            out[s] = None
            continue
        A = np.array(acc[s])
        out[s] = {"min": A.min(axis=0), "max": A.max(axis=0),
                  "mean": A.mean(axis=0),
                  "sign_change": [(A[:, j].min() < -1e-9 < 1e-9 < A[:, j].max())
                                  for j in range(3)],
                  "n": len(A), "failed": fails[s], "of": len(grid)}
    return out


def trade(q=None, spools="real", leg=DEFAULT_HINDLEG):
    """Price Option A against Option B on THIS geometry.

    Returns a dict per tendon with the coupling it carries, the torque error
    that coupling produces at the land case, and the capstan penalty an idler
    pair would cost to remove it.
    """
    if q is None:
        import tomcat_leg_detail as LD
        q = LD.LegModel(leg).inverse((LD.FOOT_X, LD.FOOT_Z, LD.FOOT_PITCH))
    if spools == "real":
        spools = real_spools()
    B = B_at(q, spools=spools, leg=leg)
    W = wraps_at(q, spools=spools, leg=leg)
    arms = np.asarray(DEFAULT_TENDON.joint_moment_arm, float) * 1000.0

    out = {}
    for i, (tendon, side) in enumerate(STRANDS):
        if (tendon, side) not in W:
            continue
        jj = JOINTS.index(tendon)
        own = B[i, jj]
        off = [B[i, JOINTS.index(p)] for p in PASSES[tendon]]
        total_wrap, capstan, via_wraps = W[(tendon, side)]
        added = sum(via_wraps)                      # the second idler's arc
        out[(tendon, side)] = {
            "own_arm": own,
            "off": off,
            "coupling_frac": [abs(o) / abs(own) if own else np.nan for o in off],
            "wrap_deg": math.degrees(total_wrap),
            "capstan": capstan,
            "via_wrap_deg": [math.degrees(w) for w in via_wraps],
            "added_wrap_deg": math.degrees(added),
            "capstan_B": math.exp(tr.MU_PULLEY * (total_wrap + added)),
            "penalty": math.exp(tr.MU_PULLEY * added),
            "nominal_arm": arms[jj],
        }
    return out


def report(n: int = 4):
    import tomcat_leg_detail as LD
    leg = DEFAULT_HINDLEG
    q = LD.LegModel(leg).inverse((LD.FOOT_X, LD.FOOT_Z, LD.FOOT_PITCH))

    print("STRUCTURE MATRIX -- the leg, measured as the matrix it is\n")
    print("  stance pose, d(cable)/d(joint) in mm/rad   (rows: strand)")
    print("  %-10s %8s %8s %8s" % ("", "hip", "knee", "ankle"))
    B = B_at(q, leg=leg)
    for i, (t, s) in enumerate(STRANDS):
        row = "  %-10s" % ("%s%+d" % (t, s))
        for j in range(3):
            row += "   %6.2f" % B[i, j] if np.isfinite(B[i, j]) else "      --"
        built = "" if (t, s) != ("ankle", -1) else "   <- NOT BUILT (ADR-0050)"
        print(row + built)

    print("\n  the trade, at the stance pose")
    print("  %-10s %7s %9s %9s %8s %9s" %
          ("strand", "arm", "coupling", "wrap deg", "capstan", "+idler"))
    tr_ = trade(q, leg=leg)
    for (t, s), d in tr_.items():
        cf = ", ".join("%.0f%%" % (100 * c) for c in d["coupling_frac"]) or "-"
        print("  %-10s %7.2f %9s %9.0f %8.3f %9.3f"
              % ("%s%+d" % (t, s), d["own_arm"], cf, d["wrap_deg"],
                 d["capstan"], d["capstan_B"]))

    print("\n  Option B cost, per strand: the SECOND idler's arc")
    for (t, s), d in tr_.items():
        if not d["via_wrap_deg"]:
            continue
        print("  %-10s vias %s deg -> +%.0f deg, capstan x%.3f -> x%.3f (%+.1f %%)"
              % ("%s%+d" % (t, s),
                 "/".join("%.0f" % w for w in d["via_wrap_deg"]),
                 d["added_wrap_deg"], d["capstan"], d["capstan_B"],
                 100 * (d["capstan_B"] / d["capstan"] - 1.0)))

    print("\n  over the ROM (%d^3 poses)" % n)
    sw = sweep(n, leg=leg)
    for s, d in sw.items():
        nm = "%s%+d" % s
        if d is None:
            print("  %-10s  *** no admissible routing anywhere in the ROM" % nm)
            continue
        flag = ""
        if d["failed"]:
            flag += "   *** %d/%d poses have NO ROUTE" % (d["failed"], d["of"])
        if any(d["sign_change"]):
            which = [JOINTS[j] for j, c in enumerate(d["sign_change"]) if c]
            flag += "   *** sign change vs %s" % ", ".join(which)
        j = JOINTS.index(s[0])
        off = ", ".join(
            "%s %.2f..%.2f" % (p, d["min"][JOINTS.index(p)],
                               d["max"][JOINTS.index(p)])
            for p in PASSES[s[0]]) or "-"
        print("  %-10s own %6.2f..%6.2f   off: %-28s%s"
              % (nm, d["min"][j], d["max"][j], off, flag))
    # ✅ **THE result of the sweep, and it changes the trade.** Every
    # off-diagonal is DEAD CONSTANT at ±8.75 across the ROM -- the via radius,
    # unchanged by pose. A constant lower-triangular `B` is not a disturbance,
    # it is a known matrix, and a known matrix inverts exactly in software. So
    # the coupling does not have to be bought out with an idler pair at +29 %
    # cable tension; it has to be WRITTEN DOWN. See `TendonMap.cable_lengths`,
    # which is diagonal, and `resolve()`, which puts the off-diagonal at zero.
    return sw


if __name__ == "__main__":
    report()
