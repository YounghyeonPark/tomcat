# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Split the skeleton CAD into the RIGID BODIES the MJCF model has.

⚠️ `tomcat_skeleton.build()` returns four render groups — `bone`, `flat`,
`joint`, `bay` — because it was written to draw a picture. A simulator needs
something else entirely: one solid per body that moves independently, so each
can carry its own mass, centre of mass and inertia tensor.

The good news, found by reading rather than assuming: the CAD already builds
those pieces and only *loses* them at the end. `limb()` assembles four bones in
a list and then `Compound`s them; `spine_detail()` builds every vertebra
separately and knows where the three actuated joints sit. So this is a
regrouping, not a re-modelling.

⚠️ **What is asserted rather than derived** is which CAD part belongs to which
body — the ribcage and electronics to the mid-spine, scapulae to the front
girdle, pelvis and tail to the rear. Those follow the anatomy the CAD documents,
but the CAD does not label them, so they are this file's judgement.
"""

from __future__ import annotations

import sys

import numpy as np
from build123d import Compound, Pos, Sphere

sys.path.insert(0, __file__.rsplit("\\", 1)[0] if "\\" in __file__ else ".")

import tomcat_skeleton as SK          # noqa: E402
from tomcat_skeleton import (MM, CONDYLE_R, FOOT_PITCH, FOOT_Z, TRACK,  # noqa: E402
                             bone, hinge, long_bone)

#: MJCF body names, in the order `mjcf_tendon` emits them.
LEG_LINKS = ("femur", "tibia", "meta", "paw")
LEGS = ("LF", "RF", "LR", "RR")


def limb_links(mount, foot_x, leg_model, foot_z=None):
    """`limb()`, but returning the four bones separately instead of Compounded.

    Mirrors `tomcat_skeleton.limb` exactly — same solve, same geometry — and
    stops before the `Compound`. The joint-axis markers are dropped: they are
    drawing aids, not structure, and giving them mass would be inventing it.
    """
    if foot_z is None:
        foot_z = FOOT_Z
    q = leg_model.inverse((foot_x, foot_z, FOOT_PITCH))
    pts = leg_model.joint_positions(q) * MM
    mx, my, mz = mount
    p3 = [(mx + x, my, mz + zz) for (x, zz) in pts]
    return {
        "femur": long_bone(p3[0], p3[1], r_prox=CONDYLE_R * 1.15),
        "tibia": long_bone(p3[1], p3[2]),
        "meta": long_bone(p3[2], p3[3], r_dist=CONDYLE_R * 0.7),
        "paw": bone(p3[3], p3[4], 3.4),
    }, p3


def spine_segments(z):
    """The vertebral column, cut at the three ACTUATED joints (ADR-0006).

    `spine_detail` places ~20 vertebrae along the column and knows the actuated
    joint positions in `edges`. A vertebra belongs to the segment whose span
    contains it, which is the same rule the MJCF chain uses.
    """
    seg_len = np.asarray(SK.DEFAULT_SPINE.segment_lengths) * MM
    edges = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = float(edges[-1])
    n_vert = SK.N_THORACIC + SK.N_LUMBAR
    xs = np.linspace(0.0, total, n_vert)

    buckets = {i: [] for i in range(len(seg_len))}
    for x in xs:
        lumbar = x < total * (SK.N_LUMBAR / n_vert)
        sweep = 34.0 if lumbar else -40.0
        h = SK.SPINOUS_H * (0.75 if lumbar else 1.0)
        i = int(np.clip(np.searchsorted(edges, x, side="right") - 1,
                        0, len(seg_len) - 1))
        buckets[i].append(SK.vertebra(float(x), z, SK.VERT_R, h, sweep))
    for a, b in zip(xs[:-1], xs[1:]):
        mid = 0.5 * (a + b)
        i = int(np.clip(np.searchsorted(edges, mid, side="right") - 1,
                        0, len(seg_len) - 1))
        buckets[i].append(SK.seg((float(a), 0, z), (float(b), 0, z),
                                 SK.VERT_R * 0.55))
    return {i: Compound(v) for i, v in buckets.items()}, total, edges


def build_bodies():
    """Every MJCF body as `{name: solid}`, plus the frames they hang off.

    Returns `(bodies, meta)`. `meta` carries the joint positions so the MJCF
    emitter can place each body's frame without re-deriving the kinematics.
    """
    H = -FOOT_Z * MM
    segs, total, edges = spine_segments(H)
    fore = SK.LegModel(SK.DEFAULT_FORELEG)
    hind = SK.LegModel()

    bodies, joints = {}, {}
    for i in range(len(segs)):
        bodies[f"spine{i + 1}"] = segs[i]

    # ⚠️ ASSERTED, not derived: the ribcage and its electronics bay ride the
    # middle spine segment. The CAD's own note says the thoracic basket houses
    # the battery, and the thorax spans that segment -- but nothing labels it.
    thorax0, thorax1 = total * 0.34, total * 0.92
    mid = min(range(len(segs)), key=lambda i: abs(
        0.5 * (edges[i] + edges[i + 1]) - 0.5 * (thorax0 + thorax1)))
    bodies[f"spine{mid + 1}"] = Compound([
        bodies[f"spine{mid + 1}"], SK.ribcage(H, thorax0, thorax1),
        SK.electronics_bay(0.5 * (thorax0 + thorax1), H)])

    front, rear = [], []
    for side in (+1, -1):
        sc, glenoid = SK.scapula(total, H, side)
        front.append(sc)
        rear.append(SK.pelvis(0.0, H, side))
        nm = "LF" if side > 0 else "RF"
        links, p3 = limb_links((glenoid[0], side * TRACK / 2, glenoid[2]),
                               SK.FRONT_FOOT_X, fore,
                               foot_z=-glenoid[2] / MM)
        for k, v in links.items():
            bodies[f"{nm}_{k}"] = v
        joints[nm] = p3
        nm = "LR" if side > 0 else "RR"
        links, p3 = limb_links((0.0, side * TRACK / 2, H), SK.REAR_FOOT_X, hind)
        for k, v in links.items():
            bodies[f"{nm}_{k}"] = v
        joints[nm] = p3

    # ⚠️ ASSERTED: skull and neck ride the front girdle; the tail rides the rear.
    bodies["front_girdle"] = Compound(front + [SK.neck_and_skull(total, H)])
    bodies["trunk"] = Compound(rear + [SK.tail(H)])
    return bodies, {"joints": joints, "spine_edges": edges, "total": total,
                    "ground_z": H}
