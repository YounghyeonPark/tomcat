# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Where the cat's skin weighs: its knit and foam, dealt to the bodies of the
plant (M130, ADR-0124).

`cat_suit` builds the skin and prices it -- ~245 g, a quarter of the body's
pitch inertia -- and until M130 none of it was in `params.py`. This deals every
bit of it to the body it rides on:

- the HEAD and NECK (forward of the trunk) to the front girdle, the TAIL
  (behind it) to the rear girdle;
- the TRUNK by x: the rear girdle behind spine joint 0, the three spine
  segments between the joints, the front girdle ahead of the last;
- each LEG sleeve to the link it is nearest, judged against the trunk.

A sleeve is fabric over a joint, not a part of one link, and nearest-link is
the plain approximation of a skin that slides. The masses are point masses at
the knit (its areal mass) and at the middle of the foam under it.

⚠️ The suit is minutes and ~1 GB to build, so this writes `skin_mass.json`,
and `girdle_inertia` and `link_inertia` read the skin from it. Re-run this
whenever the suit or the skeleton changes: `python mechanical/cad/skin_mass.py`.
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "kinematics", "src"))

import tomcat_trunk as TT                                        # noqa: E402
import tomcat_skin as SK                                         # noqa: E402
import tomcat_head as HD                                         # noqa: E402

PATH = os.path.join(HERE, "skin_mass.json")
LINKS = ("femur", "tibia", "meta", "paw")
#: The trunk's bodies by x, mm: the rear girdle behind spine joint 0, the
#: segments between the joints, the front girdle ahead of the last station.
TRUNK = (("rear", -1e9, TT.JOINT_X[0]), ("seg0", TT.JOINT_X[0], TT.JOINT_X[1]),
         ("seg1", TT.JOINT_X[1], TT.JOINT_X[2]), ("seg2", TT.JOINT_X[2], 195.0),
         ("front", 195.0, 1e9))


def point_masses(pv):
    """`(Q mm (n,3), m kg (n,))` for the knit and the foam, from `cat_suit`'s
    per-vertex skin: the knit at the surface, the foam at its own middle --
    a `PAD` shell on the body, stuffed in the head, firm in the face's outer
    `FIRM_DEPTH`."""
    import cat_suit as CS
    P, N, area, thick = pv["P"], pv["N"], pv["area"], pv["thick"]
    head, face = pv["head"], pv["face"]
    knit = area * 1e-6 * CS.AREAL_G_M2 * 1e-3
    t = np.where(head, thick, np.minimum(thick, CS.PAD))
    firm = np.where(face, np.minimum(t, CS.FIRM_DEPTH), 0.0)
    soft = t - firm
    foam = area * 1e-9 * (firm * CS.FOAM_FIRM[1] + soft * CS.FOAM_RHO["PU foam"])
    mid = P - 0.5 * t[:, None] * N
    return np.vstack([P, mid]), np.concatenate([knit, foam])


def _seg_dist(Q, a, b):
    d = b - a
    t = np.clip(((Q - a) @ d) / float(d @ d), 0.0, 1.0)
    return np.linalg.norm(Q - (a + t[:, None] * d), axis=1), t


def deal(Q, m):
    """`{body: indices}` -- each point mass to the body it rides on."""
    import cat_suit as CS
    x, y, z = Q[:, 0], Q[:, 1], Q[:, 2]
    body = np.empty(len(Q), dtype=object)
    head = x > HD.BASE_X
    tail = x < TT.BODIES[0][0]
    # the trunk's own surface: its structural envelope (`tomcat_skin.outline`)
    xs = np.clip(x, TT.BODIES[0][0], TT.BODIES[3][1])
    u = np.unique(np.round(xs, 0))
    hw, top, bot = (np.array(v) for v in zip(*[SK.outline(float(v)) for v in u]))
    a = np.interp(xs, u, hw)
    zc = np.interp(xs, u, 0.5 * (top + bot))
    hh = np.interp(xs, u, 0.5 * (top - bot))
    d_trunk = (np.sqrt((y / a) ** 2 + ((z - zc) / hh) ** 2) - 1.0) * np.minimum(a, hh)
    # the nearest leg link, sleeve surface to sleeve surface
    d_leg = np.full(len(Q), np.inf)
    link = np.empty(len(Q), dtype=object)
    for _nm, fore, side in CS.LEGS:
        role = "fore" if fore else "hind"
        hx = CS.HIP_X[fore]
        for (p0, p1, r0, r1, yc, _hy), lk in zip(CS.sleeves(fore), LINKS):
            A = np.array([hx + p0[0], side * yc, p0[1]])
            B = np.array([hx + p1[0], side * yc, p1[1]])
            d, t = _seg_dist(Q, A, B)
            d = d - (r0 + (r1 - r0) * t)
            closer = d < d_leg
            d_leg[closer] = d[closer]
            link[closer] = role + "." + lk
    on_leg = (d_leg < d_trunk) & ~head & ~tail
    for nm, lo, hi in TRUNK:
        body[(x >= lo) & (x < hi)] = nm
    body[head] = "front"
    body[tail] = "rear"
    body[on_leg] = link[on_leg]
    out = {}
    for b in sorted(set(body)):
        out[b] = np.nonzero(body == b)[0]
    return out


def lump(Q, m, idx):
    """`(kg, com m (3), I about com, kg m^2)` of a set of point masses; the
    legs' right side mirrored onto the left (the plant's legs are one model)."""
    q = Q[idx] * 1e-3
    mm = m[idx]
    M = float(mm.sum())
    c = (mm[:, None] * q).sum(0) / M
    r = q - c
    I = np.einsum("i,ij,ik->jk", mm, r, r)
    I = np.trace(I) * np.eye(3) - I
    return M, c, I


def build():
    """Build the suit and deal its mass; `{body: {mass, com, inertia}}` with
    the trunk's bodies in the TRUNK frame (m) and each leg link in its leg's
    LEFT hip frame (m, y about the limb plane as `link_inertia` draws it),
    per leg (the two of a pair averaged)."""
    import cat_suit as CS
    pts = CS.structure_points(os.path.join(HERE, "cat_suit_points.npz"))
    st = {}
    pd = CS.suit_mesh(pts, out=st)
    CS.report(pd=pd, points=pts, st=st)
    Q, m = point_masses(st["per_vertex"])
    out = {}
    for b, idx in deal(Q, m).items():
        if "." in b:
            role, _lk = b.split(".")
            fore = role == "fore"
            hx = CS.HIP_X[fore]
            q = Q[idx].copy()
            q[:, 1] = np.abs(q[:, 1])             # the right leg onto the left
            q[:, 0] -= hx                         # the hip frame
            Mh, c, I = lump(q, np.full(len(idx), 1.0) * m[idx] / 2.0,
                            np.arange(len(idx)))  # per leg: half of the pair
        else:
            Mh, c, I = lump(Q, m, idx)
        out[b] = {"mass": Mh, "com": c.tolist(), "inertia": I.tolist()}
    return out


def load():
    with open(PATH, encoding="utf-8") as fh:
        return json.load(fh)


def part(body, origin=(0.0, 0.0, 0.0)):
    """`(kg, com m, I)` of the skin on one body, its com relative to `origin`
    (m) -- ready for `girdle_inertia.combine`; zero mass if none."""
    d = load().get(body)
    if d is None:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    return d["mass"], np.asarray(d["com"]) - np.asarray(origin), np.asarray(d["inertia"])


def leg_link_masses(role):
    """The skin on each of one leg's links, kg, femur to paw."""
    data = load()
    return [data.get("%s.%s" % (role, lk), {"mass": 0.0})["mass"] for lk in LINKS]


def segment_parts():
    """`[(kg, com m (3, trunk frame), I)]` for spine segments 0-2."""
    return [part("seg%d" % i) for i in range(3)]


def report(data=None):
    data = data or load()
    tot = sum(v["mass"] for k, v in data.items() if "." not in k) \
        + 2 * sum(v["mass"] for k, v in data.items() if "." in k)
    print("SKIN MASS -- the suit dealt to the plant's bodies\n")
    for b, v in sorted(data.items()):
        c = np.asarray(v["com"]) * 1e3
        print("  %-12s %6.1f g  com (%6.1f, %5.1f, %6.1f) mm%s"
              % (b, v["mass"] * 1e3, *c, "  (each of two)" if "." in b else ""))
    print("  total        %6.1f g" % (tot * 1e3))
    return tot


if __name__ == "__main__":
    data = build()
    with open(PATH, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=1)
    print("wrote", PATH)
    report(data)
