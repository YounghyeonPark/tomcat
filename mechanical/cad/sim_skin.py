# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The cat suit, skinned to the simulator's bodies (M157).

    python mechanical/cad/sim_skin.py          # writes mechanical/cad/out/sim/skin.npz

`sim_meshes` hangs each rigid part on the one body it rides. The suit cannot
be hung that way: it is one surface over every joint, and it stretches where
the skeleton bends. So it is SKINNED, as a character in a film is:

- every vertex is first dealt to a body by `skin_mass.deal`, the same rule
  that charges the suit's MASS to the plant's bodies, so the picture and the
  inertia agree about which fabric rides which bone;
- the one-hot weights are then diffused along the surface (`BLEND_ITERS`
  steps of neighbour averaging), so across a joint the fabric follows both
  sides in proportion instead of tearing at the boundary;
- each frame, a vertex goes where its bodies take it: the sum, over its
  `K` heaviest bodies, of weight x (that body's motion since the bind pose)
  applied to the vertex (linear blend skinning).

The bind pose is the CAD stance, the pose the suit was built in, with the
trunk at the origin; each body's bind frame is the one `sim_meshes` draws its
parts in. The suit's own mass is already in the plant (`skin_mass`); this is
the picture of it only.
"""

from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "kinematics", "src"))

OUT = os.path.join(HERE, "out", "sim", "skin.npz")

#: Triangles kept after decimation: enough for a 420 px frame.
TRIANGLES = 120_000
#: Neighbour-averaging steps for the weights. Each spreads a boundary by about
#: one edge (~2 mm here), so 60 blends a joint over ~15-20 mm of fabric.
BLEND_ITERS = 60
#: Bodies kept per vertex.
K = 4

#: `skin_mass.deal`'s trunk labels -> the MJCF body.
TRUNK_BODY = {"rear": "trunk", "seg0": "spine1", "seg1": "spine2", "seg2": "spine3",
              "front": "front_girdle"}


def body_of(label, y):
    """MJCF body for a `skin_mass.deal` label at lateral position `y` (mm)."""
    if label in TRUNK_BODY:
        return TRUNK_BODY[label]
    role, link = label.split(".")
    side = "L" if y >= 0.0 else "R"
    return "%s%s_%s" % (side, "F" if role == "fore" else "R", link)


def bind_frames():
    """`{body: (p m (3), R (3,3))}`: each body's frame at the CAD stance, in the
    trunk frame. Trunk bodies from `sim_meshes.trunk_body_origins`; leg links
    from `link_inertia.link_frames`, moved to their hip, the right side
    reflected (the plant's right legs keep the left's axes)."""
    import link_inertia as LI
    import sim_meshes as SMH
    import tomcat_leg_detail as LD
    import tomcat_trunk as TT
    from tomcat_kin.params import DEFAULT_FORELEG, DEFAULT_HINDLEG, DEFAULT_SPINE as SP

    out = {n: (o, np.eye(3)) for n, o in SMH.trunk_body_origins().values()}
    out["front_girdle"] = (np.array([float(np.sum(SP.segment_lengths)), 0.0, 0.0]), np.eye(3))
    for role, leg, hx, pre in (("fore", DEFAULT_FORELEG, TT.FRONT_HIP_X, "F"),
                               ("hind", DEFAULT_HINDLEG, TT.REAR_HIP_X, "R")):
        _c, report, _p = LD.build(leg, role=role)
        for link, (p, R) in LI.link_frames(report).items():
            p = p + np.array([hx * 1e-3, 0.0, 0.0])
            out["L%s_%s" % (pre, link)] = (p, R)
            out["R%s_%s" % (pre, link)] = (p * np.array([1.0, -1.0, 1.0]), R)
    return out


def suit(cache=None):
    """`(V mm, F)`: the suit's outer surface, decimated to `TRIANGLES`."""
    import vtk
    import cat_suit as CS
    # coarser than `cat_suit`'s own 0.8 / 0.6: sampling the legs at those costs
    # ~8 GB, and a 420 px frame cannot tell the difference
    pts = CS.structure_points(cache, tol=1.6, spacing=1.2)
    pd = CS.suit_mesh(pts)
    dec = vtk.vtkQuadricDecimation()
    dec.SetInputData(pd)
    dec.SetTargetReduction(max(0.0, 1.0 - TRIANGLES / max(pd.GetNumberOfPolys(), 1)))
    dec.Update()
    cl = vtk.vtkCleanPolyData()
    cl.SetInputData(dec.GetOutput())
    cl.Update()
    return CS.mesh_arrays(cl.GetOutput())


def weights(V, F, labels, names):
    """`(idx (n,K), w (n,K))`: one-hot by label, diffused over the surface."""
    from scipy import sparse
    n, nb = len(V), len(names)
    col = {b: i for i, b in enumerate(names)}
    W = np.zeros((n, nb))
    W[np.arange(n), [col[b] for b in labels]] = 1.0
    e = np.vstack([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    e = np.vstack([e, e[:, ::-1]])
    A = sparse.coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(n, n)).tocsr()
    A.data[:] = 1.0
    A = sparse.diags(1.0 / np.maximum(np.asarray(A.sum(1)).ravel(), 1.0)) @ A
    for _ in range(BLEND_ITERS):
        W = 0.5 * W + 0.5 * (A @ W)
    idx = np.argsort(-W, axis=1)[:, :K]
    w = np.take_along_axis(W, idx, 1)
    w /= w.sum(1, keepdims=True)
    return idx, w


def build(out=OUT, cache=None):
    import skin_mass as SKM
    V, F = suit(cache)
    deal = SKM.deal(V, None)
    labels = np.empty(len(V), dtype=object)
    for lab, ix in deal.items():
        labels[ix] = [body_of(lab, y) for y in V[ix, 1]]
    binds = bind_frames()
    names = sorted(binds)
    idx, w = weights(V, F, labels, names)
    # each vertex in each of its bodies' bind frames, metres
    Vm = V * 1e-3
    P0 = np.array([binds[b][0] for b in names])
    R0 = np.array([binds[b][1] for b in names])
    local = np.einsum("nkji,nkj->nki", R0[idx], Vm[:, None, :] - P0[idx])
    os.makedirs(os.path.dirname(out), exist_ok=True)
    np.savez_compressed(out, V=Vm, F=F, names=np.array(names), idx=idx, w=w, local=local)
    print("skin: %d vertices, %d triangles, %d bodies -> %s" % (len(V), len(F), len(names), out))
    return out


class Skin:
    """Pose the suit from the plant's body frames, each frame."""

    def __init__(self, path=OUT):
        z = np.load(path, allow_pickle=False)
        self.V0, self.F = z["V"], z["F"]
        self.names = [str(s) for s in z["names"]]
        self.idx, self.w, self.local = z["idx"], z["w"], z["local"]

    def pose(self, model, data):
        """`(V world m, N unit normals)` at the plant's current state."""
        import mujoco
        ids = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b) for b in self.names])
        R = data.xmat[ids].reshape(-1, 3, 3)[self.idx]          # (n,K,3,3)
        p = data.xpos[ids][self.idx]                            # (n,K,3)
        V = np.einsum("nk,nki->ni", self.w,
                      np.einsum("nkij,nkj->nki", R, self.local) + p)
        T = V[self.F]
        fn = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        N = np.zeros_like(V)
        for j in range(3):
            np.add.at(N, self.F[:, j], fn)
        N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)
        return V, N


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", default=os.path.join(HERE, "out", "cat_suit_points.npz"))
    a = ap.parse_args()
    os.makedirs(os.path.dirname(a.cache), exist_ok=True)
    build(cache=a.cache)
