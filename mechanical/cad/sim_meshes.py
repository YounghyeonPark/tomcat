# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The CAD, as meshes in the simulator's body frames (M155).

    python mechanical/cad/sim_meshes.py            # writes mechanical/cad/out/sim/*.stl
    python mechanical/cad/sim_meshes.py --only hind

The simulator moves capsules. Its bodies are the CAD's rigid bodies, with the
CAD's lengths and the CAD's inertia (`link_inertia`), but nothing draws the
CAD on them. This module tessellates each rigid body ONCE and re-expresses it
in the MJCF body frame it rides on, so a replay can draw the designed parts
moving the way the plant moved them. Visual only: the physics never sees these.

Frames:

- **Leg links.** `link_inertia.link_frames` gives each link's MJCF frame in the
  leg's CAD coordinates (+x along the bone, +y the CAD's +y). Each solid goes to
  the link `link_inertia.assign` charges its mass to, so the picture and the
  inertia split agree. A right leg is the left one reflected in y, as in
  `tomcat_assembly.one_leg`.
- **Trunk.** The CAD trunk's origin is spine joint 0 at hip height, which is the
  MJCF `trunk` body's origin. CAD body 0 is the rear girdle (`trunk`), 1 and 2
  are `spine1`/`spine2`, and 3 runs from joint 2 through the front girdle, which
  is rigid on `spine3`. The spine bodies sit `spine_axis_z` up.

The drive harness (cables, conduits, trunk ferrules) crosses joints, so it has
no single body to ride and is left out; the MJCF's own tendons are drawn instead.
"""

from __future__ import annotations

import argparse
import os
import struct

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "sim")

#: Tessellation tolerance, mm. `tomcat_assembly._mesh` found 0.4 cost ~15 GB
#: for the whole robot; one body at a time at 0.6 is a few hundred MB.
TOL = 0.6

#: Leg solids by material, for colour. Anything not listed is metal.
MATERIAL = {"tube": "carbon", "pad": "pad", "cable": "cable"}


def tessellate(solids, tol=TOL):
    """`(V, F)` in mm, face by face; a face that will not mesh is skipped
    (`tomcat_assembly._mesh`), and the count returned."""
    V, F, n, bad = [], [], 0, 0
    for sd in solids:
        for f in sd.faces():
            try:
                verts, tris = f.tessellate(tol)
            except Exception:
                bad += 1
                continue
            if not tris:
                continue
            V.append(np.array([[v.X, v.Y, v.Z] for v in verts], float))
            F.append(np.asarray(tris, int) + n)
            n += len(verts)
    if not V:
        return np.zeros((0, 3)), np.zeros((0, 3), int), bad
    return np.concatenate(V), np.concatenate(F), bad


def write_stl(path, V, F):
    """Binary STL, metres."""
    tri = V[F].astype(np.float32)
    nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = (nrm / np.where(ln > 0, ln, 1.0)).astype(np.float32)
    rec = np.zeros(len(F), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = nrm, tri
    with open(path, "wb") as f:
        f.write(b"tomcat sim mesh".ljust(80, b" "))
        f.write(struct.pack("<I", len(F)))
        f.write(rec.tobytes())


def mirror_y(V, F):
    """Reflect in y; a reflection turns the faces inside out, so re-wind them."""
    return V * np.array([1.0, -1.0, 1.0]), F[:, ::-1].copy()


def leg_meshes(role):
    """`{(link, material): (V_m, F)}` for the LEFT `role` leg, each in its
    link's MJCF frame, metres."""
    import link_inertia as LI
    import tomcat_assembly as TA
    import tomcat_leg_detail as LD
    from tomcat_kin.params import DEFAULT_FORELEG, DEFAULT_HINDLEG

    leg = DEFAULT_FORELEG if role == "fore" else DEFAULT_HINDLEG
    comps, report, _ = LD.build(leg, role=role)
    joints, bones = LI.geometry(report)
    frames = LI.link_frames(report)
    groups = {}
    for name, comp in comps.items():
        if name in TA.DROP or name in TA.HARNESS:
            continue
        for sd in comp.solids():
            if name in TA.HIP_HARDWARE and TA._at_hip(sd):
                continue          # the trunk owns the hip (tomcat_assembly)
            c = sd.center()
            link = LI.assign(name, np.array([c.X, c.Y, c.Z]) * 1e-3, joints, bones)
            if link is None:
                continue
            groups.setdefault((link, MATERIAL.get(name, "metal")), []).append(sd)
    out, bad = {}, 0
    for key, solids in groups.items():
        V, F, b = tessellate(solids)
        bad += b
        p, R = frames[key[0]]
        out[key] = ((V * 1e-3 - p) @ R, F)
    return out, bad


def trunk_body_origins():
    """CAD trunk body -> (MJCF body, its origin in CAD trunk coordinates, m)."""
    from tomcat_kin.params import DEFAULT_SPINE as SP
    z = SP.spine_axis_z
    x = np.concatenate([[0.0], np.cumsum(SP.segment_lengths)])
    return {0: ("trunk", np.zeros(3)),
            1: ("spine1", np.array([x[0], 0.0, z])),
            2: ("spine2", np.array([x[1], 0.0, z])),
            3: ("spine3", np.array([x[2], 0.0, z]))}


def trunk_mesh(body):
    import tomcat_trunk as TT
    V, F, bad = tessellate(TT.rigid_body(body).solids())
    name, o = trunk_body_origins()[body]
    return name, (V * 1e-3 - o, F), bad


def mesh_names():
    """`(file stem, MJCF body, material)` of every mesh `main` writes."""
    import link_inertia as LI
    out = [("trunk%d" % b, n, "shell") for b, (n, _o) in trunk_body_origins().items()]
    for role, legs in (("fore", ("LF", "RF")), ("hind", ("LR", "RR"))):
        for lg in legs:
            for link in LI.LINKS:
                for mat in ("carbon", "metal", "pad", "cable"):
                    out.append(("%s_%s_%s" % (lg, link, mat), "%s_%s" % (lg, link), mat))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", choices=("fore", "hind", "trunk"), default=None)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    if a.only in (None, "trunk"):
        for b in sorted(trunk_body_origins()):
            name, (V, F), bad = trunk_mesh(b)
            write_stl(os.path.join(a.out, "trunk%d.stl" % b), V, F)
            print("trunk body %d -> %s: %d triangles, %d faces skipped" % (b, name, len(F), bad),
                  flush=True)
    for role, (left, right) in (("fore", ("LF", "RF")), ("hind", ("LR", "RR"))):
        if a.only not in (None, role):
            continue
        meshes, bad = leg_meshes(role)
        for (link, mat), (V, F) in meshes.items():
            write_stl(os.path.join(a.out, "%s_%s_%s.stl" % (left, link, mat)), V, F)
            write_stl(os.path.join(a.out, "%s_%s_%s.stl" % (right, link, mat)), *mirror_y(V, F))
        print("%s leg: %d meshes, %d triangles, %d faces skipped"
              % (role, len(meshes), sum(len(F) for _V, F in meshes.values()), bad), flush=True)


if __name__ == "__main__":
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "kinematics", "src"))
    main()
