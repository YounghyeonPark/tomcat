# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Parametric 3D CAD model of the T.O.M.C.A.T. skeleton (build123d).

Turns the placeholder dimensions of the `tomcat_kin` simulation into an actual
3D solid, exported to STEP/STL. It imports the SAME parameters the kinematics
model uses (`LegModel`, `DEFAULT_LEG`, `DEFAULT_FORELEG`, `DEFAULT_SPINE`) so the
CAD and the simulation cannot drift apart.

DETAILED SKELETAL PASS
----------------------
This is no longer a capsule massing model. It implements bone FORM, joint
STRUCTURE and joint AXIS ORIENTATION, following the public-domain reference in
../reference/ANATOMY.md (Reighard & Jennings 1901) and the cat vertebral formula
C7 / T13 / L7 / S3 / Ca~20:

- **Long bones** are shafts with expanded ends (condyles), not plain capsules —
  the flare at each end is where a joint bearing and its pulley actually sit.
- **Joints are drawn as their AXIS**: every actuated joint in this sagittal model
  is a hinge about the lateral (y) axis, so it is rendered as a short cylinder
  lying along y. The hip/shoulder is drawn as a ball (in a real cat it is a
  3-DOF ball joint; we actuate only its sagittal DOF).
- **Vertebrae are individual bodies with spinous processes.** The 3 ACTUATED
  spine joints (ADR-0006) are marked with hinge axes; the remaining ~20
  presacral vertebrae are rendered as form so the thoracic/lumbar split is
  visible. Thoracic spines sweep BACKWARD, lumbar spines sweep FORWARD, as in
  the reference — they meet at the anticlinal vertebra.
- **Ribcage** is C-curved ribs from the thoracic vertebrae down-forward to a
  sternum, enclosing a real volume (not decorative rings).
- **Scapula** — cats have no functional clavicle, so the forelimb hangs from a
  large mobile shoulder blade. Modelled as a sagittal plate sloping down-forward
  with the shoulder joint at its ventral tip.
- **Pelvis** — ilium blade sweeping up-forward from the hip, ischium back.

WHAT THE DETAIL BUYS THE DESIGN (component placement)
-----------------------------------------------------
1. The **spinous process height IS the spine tendon moment arm.** The 0.030 m
   arm in `SpineParams` is no longer an arbitrary pulley radius — it is the
   dorsal process the tendon runs over, so the CAD and the parameter agree.
2. The **ribcage encloses the battery/electronics volume.** Previously the CAD
   put a "mid-body bay" box in the belly with no structural justification; the
   thoracic basket is the natural protected, central, low-CoM location.

Scope / honesty: still a SKELETAL model, not a manufacturing model — no
fasteners, bearings, tolerances or fabrication features, and every dimension
remains a placeholder driven by `params.py`.

Run:  python mechanical/cad/tomcat_skeleton.py
Out:  tomcat_skeleton.step, tomcat_skeleton.stl, tomcat_skeleton.png (this dir)
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
from build123d import (
    Box, Cylinder, Sphere, Compound, Vector, Plane, Pos,
    export_step, export_stl,
)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "kinematics", "src"))
from tomcat_kin import LegModel  # noqa: E402
from tomcat_kin.params import DEFAULT_LEG, DEFAULT_SPINE, DEFAULT_FORELEG  # noqa: E402

MM = 1000.0

# ---- placeholder body params the 2D model doesn't carry (❓ TBD) ----
TRACK = 96.0           # lateral spacing of left/right limbs (mm)
VERT_R = 6.5           # vertebral centrum radius (mm)
SPINOUS_H = float(DEFAULT_SPINE.joint_moment_arm[0]) * MM   # 30 mm == the tendon moment arm
N_THORACIC, N_LUMBAR = 13, 7      # cat vertebral formula (ANATOMY.md)
RIB_PAIRS = 8                     # drawn (of 13) — enough to read as a ribcage
SHAFT_R, CONDYLE_R = 4.5, 7.5     # long-bone shaft / articular end radii
HINGE_R, HINGE_W = 8.0, TRACK * 0.11   # joint-axis cylinder

FOOT_Z, FOOT_PITCH = -0.17, 0.0

# render tones: skeleton pale, flat bones slightly darker, ACTUATED joints
# highlighted, payload volume translucent
SKEL_STYLE = {
    "bone":  ("#dcb289", 1.00),
    "flat":  ("#c49a72", 1.00),
    "joint": ("#8a5a3c", 1.00),
    "bay":   ("#7f93a8", 0.22),
}
# Foot placement RELATIVE TO EACH LIMB'S OWN MOUNT. The forelimb hangs from the
# scapula's glenoid, which sits forward of and BELOW the shoulder girdle, so a
# small offset already puts the front paw under the shoulder (x~221 mm world vs
# a 195 mm body). NOTE: the kinematics model has no scapula -- it mounts the
# forelimb on the girdle frame -- so CAD and model differ slightly here by
# design; the model is the authority for stability, the CAD for form.
FRONT_FOOT_X, REAR_FOOT_X = 0.02, 0.04


# ----------------------------------------------------------------- primitives
def seg(p0, p1, r):
    """A plain cylinder between two points (cheap; for ribs/processes)."""
    p0, p1 = Vector(*p0), Vector(*p1)
    d = p1 - p0
    if d.length < 1e-6:
        return Sphere(r)
    return Plane(origin=(p0 + p1) / 2, z_dir=d.normalized()).location * Cylinder(r, d.length)


def bone(p0, p1, r):
    """A capsule (cylinder + rounded ends)."""
    p0v, p1v = Vector(*p0), Vector(*p1)
    if (p1v - p0v).length < 1e-6:
        return Sphere(r)
    return Compound([seg(p0, p1, r), Pos(*tuple(p0v)) * Sphere(r), Pos(*tuple(p1v)) * Sphere(r)])


def long_bone(p0, p1, r_shaft=SHAFT_R, r_prox=CONDYLE_R, r_dist=CONDYLE_R):
    """A shaft with EXPANDED ARTICULAR ENDS — the real form of a limb bone.

    The flares are the condyles: they are where a joint bearing seats and where
    a tendon pulley gets its moment arm, so drawing them is not decoration.
    """
    p0v, p1v = Vector(*p0), Vector(*p1)
    d = p1v - p0v
    if d.length < 1e-6:
        return Sphere(r_shaft)
    u = d.normalized()
    # inset the shaft slightly so the condyles read as distinct swellings
    a, b = p0v + u * r_prox * 0.5, p1v - u * r_dist * 0.5
    return Compound([
        seg(tuple(a), tuple(b), r_shaft),
        Pos(*tuple(p0v)) * Sphere(r_prox),
        Pos(*tuple(p1v)) * Sphere(r_dist),
    ])


def hinge(pos, r=HINGE_R, w=HINGE_W):
    """A joint drawn as its AXIS: a cylinder lying along lateral y.

    Every actuated joint in this sagittal model is a hinge about y, so its axis
    is the honest way to show joint structure and orientation.
    """
    return Plane(origin=pos, z_dir=(0, 1, 0)).location * Cylinder(r, w)


def plate(center, length, height, thickness, tilt_deg=0.0):
    """A flat bone (scapula / ilium) lying in the sagittal plane, tilted about y."""
    t = math.radians(tilt_deg)
    x_dir = (math.cos(t), 0.0, math.sin(t))
    return Plane(origin=center, x_dir=x_dir, z_dir=(0, 1, 0)).location * Box(
        length, height, thickness
    )


# ------------------------------------------------------------------- assemblies
def vertebra(x, z, r_body, spinous_h, sweep_deg):
    """One vertebra: centrum + dorsal spinous process + transverse processes.

    `sweep_deg` tilts the spinous process — BACKWARD over the thorax, FORWARD
    over the lumbar, meeting at the anticlinal vertebra as in a real cat.
    """
    s = math.radians(sweep_deg)
    tip = (x + spinous_h * math.sin(s), 0.0, z + spinous_h * math.cos(s))
    parts = [
        Pos(x, 0, z) * Sphere(r_body),                       # centrum
        seg((x, 0, z), tip, r_body * 0.30),                  # spinous process
        Pos(*tip) * Sphere(r_body * 0.34),                   # its tip (tendon bears here)
        seg((x, -r_body * 1.5, z), (x, r_body * 1.5, z), r_body * 0.22),  # transverse
    ]
    return Compound(parts)


def spine_detail(z):
    """Vertebral column: individual vertebrae + the 3 ACTUATED hinge joints.

    Form shows the real ~20 presacral vertebrae; only 3 joints are actuated
    (ADR-0006), and those are the ones drawn with an explicit hinge axis.
    """
    seg_len = np.asarray(DEFAULT_SPINE.segment_lengths) * MM
    edges = np.concatenate([[0.0], np.cumsum(seg_len)])       # actuated joint x
    total = float(edges[-1])
    n_vert = N_THORACIC + N_LUMBAR
    xs = np.linspace(0.0, total, n_vert)

    parts = []
    for i, x in enumerate(xs):
        lumbar = x < total * (N_LUMBAR / n_vert)   # rear third is lumbar
        # thoracic spines sweep BACK (-x), lumbar sweep FORWARD (+x)
        sweep = 34.0 if lumbar else -40.0
        h = SPINOUS_H * (0.75 if lumbar else 1.0)
        parts.append(vertebra(float(x), z, VERT_R, h, sweep))
    # centrum chain
    for a, b in zip(xs[:-1], xs[1:]):
        parts.append(seg((float(a), 0, z), (float(b), 0, z), VERT_R * 0.55))
    # the THREE actuated inter-vertebral joints, as explicit hinge axes
    joints = [hinge((float(x), 0, z), r=VERT_R * 1.25, w=TRACK * 0.16)
              for x in edges[1:-1]]
    return Compound(parts), Compound(joints), total, edges


def ribcage(z, x_start, x_end):
    """C-curved ribs from thoracic vertebrae down-forward to a sternum.

    Encloses the thoracic volume that now houses the battery / electronics.
    """
    ribs, tips = [], []
    for x in np.linspace(x_start, x_end, RIB_PAIRS):
        for side in (+1, -1):
            pts = []
            for f in np.linspace(0.0, 1.0, 5):
                ang = math.radians(20 + 150 * f)              # around the barrel
                y = side * 34.0 * math.sin(ang)
                zz = z - 4.0 - 40.0 * (1 - math.cos(ang)) * 0.62
                pts.append((float(x + 10.0 * f), y, zz))      # sweep back-down
            for a, b in zip(pts[:-1], pts[1:]):
                ribs.append(seg(a, b, 2.3))
            tips.append(pts[-1])
    # sternum: a bar joining the ventral rib tips
    sx = [p[0] for p in tips]; sz = [p[2] for p in tips]
    sternum = seg((min(sx), 0, float(np.mean(sz))), (max(sx), 0, float(np.mean(sz))), 4.5)
    return Compound(ribs + [sternum])


def scapula(x, z, side):
    """Shoulder blade: a sagittal plate sloping down-FORWARD.

    Cats have NO functional clavicle, so the forelimb hangs from this mobile
    blade rather than bolting to the trunk — the anatomical reason a cat has
    such a large shoulder excursion (and the ADR-0007 righting contribution).
    Returns (solid, shoulder_joint_position) — the glenoid at its ventral tip.
    """
    tilt = -52.0                                   # blade axis, down-forward
    length, height, thick = 58.0, 26.0, 3.0
    cx, cz = x - 12.0, z + 6.0
    blade = plate((cx, side * TRACK * 0.30, cz), length, height, thick, tilt_deg=tilt)
    t = math.radians(tilt)
    glenoid = (cx + (length / 2) * math.cos(t), side * TRACK * 0.30,
               cz + (length / 2) * math.sin(t))
    spine_ridge = seg((cx - length * 0.42 * math.cos(t), side * (TRACK * 0.30 + thick),
                       cz - length * 0.42 * math.sin(t)),
                      (cx + length * 0.42 * math.cos(t), side * (TRACK * 0.30 + thick),
                       cz + length * 0.42 * math.sin(t)), 3.0)
    return Compound([blade, spine_ridge]), glenoid


def pelvis(x, z, side):
    """Ilium blade sweeping up-FORWARD from the hip, plus the ischium behind."""
    ilium = plate((x + 16.0, side * TRACK * 0.26, z + 11.0), 48.0, 22.0, 3.0, tilt_deg=26.0)
    ischium = plate((x - 20.0, side * TRACK * 0.24, z - 3.0), 30.0, 15.0, 3.0, tilt_deg=-14.0)
    return Compound([ilium, ischium])


def neck_and_skull(x, z):
    """Cervical curve (7 vertebrae) rising forward to a small skull."""
    parts = []
    pts = [(x + 8 * i, 0.0, z + 3.0 * i + 1.2 * i * i / 2) for i in range(6)]
    for i, p in enumerate(pts):
        parts.append(Pos(*p) * Sphere(VERT_R * (0.82 - 0.03 * i)))
    for a, b in zip(pts[:-1], pts[1:]):
        parts.append(seg(a, b, VERT_R * 0.5))
    sk = pts[-1]
    parts.append(Pos(sk[0] + 22, 0, sk[2] + 6) * Sphere(19.0))       # cranium
    parts.append(seg((sk[0] + 22, 0, sk[2] + 6), (sk[0] + 46, 0, sk[2] - 1), 9.0))  # muzzle
    return Compound(parts)


def tail(z):
    """Caudal vertebrae: a tapering chain curling up and back (ADR-0007)."""
    pts = [(0, 0, z), (-40, 0, z + 8), (-78, 0, z + 26), (-112, 0, z + 52),
           (-140, 0, z + 84)]
    parts = []
    for i, (a, b) in enumerate(zip(pts[:-1], pts[1:])):
        r = 6.5 - 1.1 * i
        parts.append(bone(a, b, max(r, 2.2)))
        for f in (0.33, 0.66):                                  # caudal vertebrae
            p = tuple(np.array(a) + (np.array(b) - np.array(a)) * f)
            parts.append(Pos(*p) * Sphere(max(r * 1.15, 2.6)))
    return Compound(parts)


def limb(mount, foot_x, leg_model, side, foot_z=None):
    """One digitigrade limb: long bones with condyles + explicit joint axes.

    Each leg solves with its OWN anatomical fold (hind stifle forward, fore
    elbow back) via `LegModel.default_knee`, so both paws point forward.

    `foot_z` is the drop from THIS limb's mount to the ground. It must be given
    per-limb because the forelimb hangs from the scapula's glenoid, which sits
    LOWER than the hip — using one global drop put the front paws ~17 mm below
    the ground plane.
    """
    if foot_z is None:
        foot_z = FOOT_Z
    q = leg_model.inverse((foot_x, foot_z, FOOT_PITCH))
    pts2d = leg_model.joint_positions(q) * MM       # hip, mid, low, paw-base, paw-tip
    mx, my, mz = mount
    p3 = [(mx + x, my, mz + zz) for (x, zz) in pts2d]

    parts = [
        long_bone(p3[0], p3[1], r_prox=CONDYLE_R * 1.15),   # femur / humerus
        long_bone(p3[1], p3[2]),                            # tibia / radius
        long_bone(p3[2], p3[3], r_dist=CONDYLE_R * 0.7),    # metatarsus / metacarpus
        bone(p3[3], p3[4], 3.4),                            # phalanges (passive paw)
    ]
    # joint AXES: proximal joint is a ball (3-DOF in a cat), the two distal
    # joints are hinges about lateral y.
    parts.append(Pos(*p3[0]) * Sphere(CONDYLE_R * 1.25))
    parts.append(hinge(p3[1]))
    parts.append(hinge(p3[2], r=HINGE_R * 0.85))
    return Compound(parts)


def electronics_bay(x_center, z, length=76.0, width=44.0, height=34.0):
    """Battery + electronics, now housed INSIDE the thoracic basket.

    Placement improvement enabled by the detailed ribcage: previously this sat
    in an unjustified belly box. The rib basket is the natural protected,
    central, low location — and it keeps the mass near the CoM.
    """
    return Pos(x_center, 0, z - 26.0) * Box(length, width, height)


def build():
    H = -FOOT_Z * MM
    column, spine_joints, total, edges = spine_detail(H)

    fore = LegModel(DEFAULT_FORELEG)
    hind = LegModel()

    # scapulae carry the forelimbs; the shoulder joint is the glenoid tip
    scaps, limbs = [], []
    for side in (+1, -1):
        sc, glenoid = scapula(total, H, side)
        scaps.append(sc)
        # forelimb hangs from the glenoid, so it only has to drop that far
        limbs.append(limb((glenoid[0], side * TRACK / 2, glenoid[2]),
                          FRONT_FOOT_X, fore, side, foot_z=-glenoid[2] / MM))
        limbs.append(limb((0.0, side * TRACK / 2, H), REAR_FOOT_X, hind, side))
    pelvises = [pelvis(0.0, H, side) for side in (+1, -1)]

    thorax0, thorax1 = total * 0.34, total * 0.92
    groups = {
        # slender skeletal structure
        "bone": Compound([column, ribcage(H, thorax0, thorax1),
                          Compound(limbs), neck_and_skull(total, H), tail(H)]),
        # flat bones read better in their own tone
        "flat": Compound(scaps + pelvises),
        # the ACTUATED joint axes -- the thing a reader should be able to count
        "joint": spine_joints,
        # payload volume the ribcage encloses
        "bay": electronics_bay(0.5 * (thorax0 + thorax1), H),
    }
    return groups, H, total


def render_png(groups, path, H, front_x):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(11, 6))
    ax = fig.add_subplot(111, projection="3d")
    for name, comp in groups.items():
        verts, tris = comp.tessellate(0.45)
        V = np.array([[v.X, v.Y, v.Z] for v in verts]); T = np.array(tris)
        color, alpha = SKEL_STYLE[name]
        coll = Poly3DCollection(V[T], facecolor=color, edgecolor="none", alpha=alpha)
        coll.set_zsort("average"); ax.add_collection3d(coll)
    ax.set_xlim(-200, front_x + 190); ax.set_ylim(-190, 190); ax.set_zlim(0, H + 150)
    ax.set_box_aspect((front_x + 390, 380, H + 150))
    ax.view_init(elev=16, azim=-62); ax.set_axis_off()
    fig.tight_layout(); fig.savefig(path, dpi=130, bbox_inches="tight", facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    here = os.path.dirname(__file__)
    groups, H, front_x = build()
    bb = Compound(list(groups.values())).bounding_box()
    print(f"detailed skeleton: hip height {H:.0f} mm, presacral span {front_x:.0f} mm")
    print(f"  vertebrae drawn {N_THORACIC + N_LUMBAR} (T{N_THORACIC}/L{N_LUMBAR}), "
          f"actuated spine joints {len(DEFAULT_SPINE.segment_lengths) - 1 + 1}")
    print(f"  spinous process height = spine tendon moment arm = {SPINOUS_H:.0f} mm")
    print(f"  bounding box: {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm")
    step = os.path.join(here, "tomcat_skeleton.step")
    stl = os.path.join(here, "tomcat_skeleton.stl")
    png = os.path.join(here, "tomcat_skeleton.png")
    whole = Compound(list(groups.values()))
    export_step(whole, step)
    export_stl(whole, stl, tolerance=0.25, angular_tolerance=0.35)
    render_png(groups, png, H, front_x)
    for p in (step, stl, png):
        print(f"  wrote {os.path.basename(p)} ({os.path.getsize(p)} bytes)")
