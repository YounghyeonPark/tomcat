"""3D packaging study: motors, tendons (wires) and joint pulleys in the skeleton.

Extends the skeleton massing model with the *internal* layout the design implies:
- MOTORS clustered in the shoulder & pelvic girdles (P1 centralization), one per
  driven tendon — the girdle box is SIZED to actually contain them (a real fit
  check, not a placeholder box).
- TENDONS ("wires") routed from each motor's spool along the limbs / spine to the
  joint it drives — antagonistic pairs at hip/knee & the spine, single tendon at
  the spring-return ankle and the tail (ADR-0002 / ADR-0007).
- JOINT PULLEYS (moment-arm hardware) at hip/knee/ankle and each vertebra, drawn
  at the real moment-arm radii from the mechanical spec.

Motor/pulley sizes are placeholder cat-scale values; the point is the LAYOUT and
whether it packs. Colours in the render: bone=tan, motor=slate, tendon=copper,
pulley=steel.

Run:  python mechanical/cad/tomcat_packaging.py
Out:  tomcat_packaging.step / .stl / .png (this dir)
"""

from __future__ import annotations

import os
import sys

import numpy as np
from build123d import (
    Box, Cylinder, Sphere, Compound, Vector, Plane, Pos, export_step, export_stl,
)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "kinematics", "src"))
from tomcat_kin import LegModel, KneeConfig  # noqa: E402
from tomcat_kin.params import DEFAULT_SPINE, DEFAULT_TENDON, DEFAULT_FORELEG  # noqa: E402

MM = 1000.0

# --- placeholders ---
TRACK = 96.0
BONE_R = 6.0
MOTOR_D, MOTOR_L = 36.0, 26.0        # ~74 g pancake QDD module (ADR-0008 class)
SPOOL_D, SPOOL_L = 16.0, 8.0   # variable-radius pulley (ADR-0008)
FOOT_Z, FOOT_PITCH = -0.19, 0.0     # paw flat on the ground (digitigrade)
FRONT_FOOT_X, REAR_FOOT_X = 0.06, 0.04
TENDON_R = 1.6
LEG_ARMS = np.asarray(DEFAULT_TENDON.joint_moment_arm) * MM   # hip,knee,ankle pulley radii (mm)
SPINE_ARM = float(DEFAULT_SPINE.joint_moment_arm[0]) * MM     # 30 mm


def yaxis(origin):
    """Location whose local +Z points along world +Y (for lateral-axis parts)."""
    return Plane(origin=origin, z_dir=(0, 1, 0)).location


def bone(p0, p1, r):
    p0, p1 = Vector(*p0), Vector(*p1)
    d = p1 - p0
    L = d.length
    if L < 1e-6:
        return Sphere(r)
    mid = (p0 + p1) / 2
    return Compound([
        Plane(origin=mid, z_dir=d.normalized()).location * Cylinder(r, L),
        Pos(*tuple(p0)) * Sphere(r), Pos(*tuple(p1)) * Sphere(r),
    ])


def tube(points, r=TENDON_R):
    """A tendon as a chain of thin capsules through 3D `points`."""
    return Compound([bone(a, b, r) for a, b in zip(points[:-1], points[1:])])


def motor(pos, spool_side=+1, axis="z"):
    """A motor with a spool on its output end.

    `axis="z"` (default, review F4) stands the motor UPRIGHT: its 28 mm length
    goes into girdle HEIGHT, where there is room, instead of girdle WIDTH, where
    there is not. `axis="y"` is the original lateral layout, kept for comparison.
    """
    x, y, z = pos
    if axis == "y":
        body = yaxis((x, y, z)) * Cylinder(MOTOR_D / 2, MOTOR_L)
        sy = y + spool_side * (MOTOR_L / 2 + SPOOL_L / 2)
        return Compound([body, yaxis((x, sy, z)) * Cylinder(SPOOL_D / 2, SPOOL_L)]), (x, sy, z)
    body = Pos(x, y, z) * Cylinder(MOTOR_D / 2, MOTOR_L)          # axis along +Z
    sz = z + spool_side * (MOTOR_L / 2 + SPOOL_L / 2)
    return Compound([body, Pos(x, y, sz) * Cylinder(SPOOL_D / 2, SPOOL_L)]), (x, y, sz)


def pack_cluster(center, n, spool_side, axis="z", per_layer=(2, 1)):
    """Pack n upright motors around `center`; return (motors, spool_points).

    Ø36 pancake modules (ADR-0008) only fit ~2x2 per layer inside a cat torso, so
    the bank STACKS in z once a layer is full. `per_layer` is (nx, ny).
    """
    cx, cy, cz = center
    nx, ny = per_layer
    pitch = MOTOR_D + 3
    zpitch = MOTOR_L + SPOOL_L + 2
    motors, spools = [], []
    for i in range(n):
        layer, rem = divmod(i, nx * ny)
        r, c = divmod(rem, nx)
        m, sp = motor((cx + (c - (nx - 1) / 2) * pitch,
                       cy + (r - (ny - 1) / 2) * pitch,
                       cz + layer * zpitch), spool_side, axis="z")
        motors.append(m); spools.append(sp)
    return Compound(motors), spools


def girdle_box(center, cluster_compounds):
    """A translucent housing sized to enclose its motor clusters + margin."""
    comp = Compound(cluster_compounds)
    bb = comp.bounding_box()
    m = 5.0
    L, W, Hh = bb.size.X + 2 * m, bb.size.Y + 2 * m, bb.size.Z + 2 * m
    c = bb.center()
    return Pos(c.X, c.Y, c.Z) * Box(L, W, Hh), (L, W, Hh)


def leg_points(mount, foot_x, knee, leg_model=None):
    lm = leg_model if leg_model is not None else LegModel()
    q = lm.inverse((foot_x, FOOT_Z, FOOT_PITCH), knee=knee)
    pts = lm.joint_positions(q) * MM   # (5,2): hip, stifle, hock, paw-base, paw-tip
    mx, my, mz = mount
    return [(mx + x, my, mz + z) for (x, z) in pts]


def leg_bones(p3):
    parts = [bone(a, b, BONE_R) for a, b in zip(p3[:-1], p3[1:])]
    parts.append(Pos(*p3[0]) * Sphere(BONE_R * 1.4))
    return Compound(parts)


def leg_pulleys(p3):
    # discs at the 3 ACTUATED joints hip, stifle(knee), hock(ankle) — the paw is
    # passive, no pulley. p3 = [hip, stifle, hock, paw-base, paw-tip].
    hip, knee, ankle = p3[0], p3[1], p3[2]
    return Compound([
        yaxis(hip) * Cylinder(LEG_ARMS[0], 4),
        yaxis(knee) * Cylinder(LEG_ARMS[1], 4),
        yaxis(ankle) * Cylinder(LEG_ARMS[2], 3),
    ])


def leg_tendons(p3, spools):
    """Six cables from THREE motors: per ADR-0008 one variable-radius-pulley motor
    drives both sides of each antagonistic pair (hip, stifle, hock)."""
    hip, knee, ankle = p3[0], p3[1], p3[2]
    off = np.array([0, 0, 1.0])
    t = []
    for spool, joint, via in ((spools[0], hip, []),
                              (spools[1], knee, [hip]),
                              (spools[2], ankle, [hip, knee])):
        for sgn in (+1, -1):
            t.append(tube([spool] + via + [tuple(np.array(joint) + sgn * 6 * off)]))
    return Compound(t)


def spine_chain(z):
    seg = np.asarray(DEFAULT_SPINE.segment_lengths) * MM
    xs = np.concatenate([[0.0], np.cumsum(seg)])
    radii = np.linspace(16 * 1.1, 16 * 0.85, len(xs))
    bones = [bone((x0, 0, z), (x1, 0, z), float(r))
             for x0, x1, r in zip(xs[:-1], xs[1:], radii[:-1])]
    verts = [Pos(x, 0, z) * Sphere(float(r) * 0.9) for x, r in zip(xs, radii)]
    pulleys = [yaxis((x, 0, z)) * Cylinder(SPINE_ARM, 4) for x in xs[1:-1]]
    return Compound(bones + verts), Compound(pulleys), xs


def spine_tendons(z, xs, spools):
    """Dorsal + ventral tendons running the length over each vertebra."""
    top = [(x, 0, z + SPINE_ARM) for x in xs]
    bot = [(x, 0, z - SPINE_ARM) for x in xs]
    dorsal = tube([spools[0]] + top)
    ventral = tube([spools[1]] + bot)
    return Compound([dorsal, ventral])


def tail(z, spool):
    pts = [(0, 0, z), (-70, 0, z + 20), (-125, 0, z + 55), (-160, 0, z + 100)]
    radii = [9, 7, 5, 3.5]
    bones = [bone(a, b, r) for a, b, r in zip(pts[:-1], pts[1:], radii)]
    tendon = tube([spool] + [(x, 0, zz + r) for (x, _, zz), r in zip(pts, radii)])
    return Compound(bones), tendon


def build():
    H = -FOOT_Z * MM
    kp = None   # each leg uses its own anatomical fold
    spine_body, spine_pulleys, xs = spine_chain(H)
    front_x = float(xs[-1])

    bones, motors, tendons, pulleys = [], [], [], []

    # ---- legs (4) ----
    # (mount, foot_x, motor-side, leg_model) — the fore/hind fold difference now
    # comes from each leg's own joint limits, not from mirroring.
    fore = LegModel(DEFAULT_FORELEG)
    hind = LegModel()
    leg_defs = [
        ((front_x, +TRACK / 2, H), FRONT_FOOT_X, +1, fore),    # front-left
        ((front_x, -TRACK / 2, H), FRONT_FOOT_X, -1, fore),    # front-right
        ((0.0, +TRACK / 2, H), REAR_FOOT_X, +1, hind),        # rear-left
        ((0.0, -TRACK / 2, H), REAR_FOOT_X, -1, hind),        # rear-right
    ]
    girdles = []
    front_clusters, pelvic_clusters = [], []
    for (mount, fx, side, lm) in leg_defs:
        p3 = leg_points(mount, fx, kp, leg_model=lm)
        bones.append(leg_bones(p3))
        pulleys.append(leg_pulleys(p3))
        # motor cluster for this leg, in its girdle, on its side
        gx = mount[0]
        cluster_c = (gx, side * 21.0, H)   # one bank per leg, inboard
        cl, spools = pack_cluster(cluster_c, 3, side)   # 3 DOF/leg (VRP), stacked 2/layer
        motors.append(cl)
        tendons.append(leg_tendons(p3, spools))
        (front_clusters if gx == front_x else pelvic_clusters).append(cl)

    # ---- spine + tail motor bank in the pelvic girdle (centre) ----
    # Spine + tail bank lives in the BELLY between the girdles (still
    # centralized per P1), which is otherwise empty volume.
    spine_bank, spine_spools = pack_cluster((0.5 * front_x, 0.0, H - 26.0), 4, +1)  # 3 spine + 1 tail
    motors.append(spine_bank)
    mid_clusters = [spine_bank]
    tendons.append(spine_tendons(H, xs, [spine_spools[0], spine_spools[1]]))
    tail_body, tail_tendon = tail(H, spine_spools[3])
    bones.append(tail_body)
    tendons.append(tail_tendon)

    # ---- girdle housings sized to fit their clusters ----
    fg, fg_dims = girdle_box((front_x, 0, H), front_clusters)
    pg, pg_dims = girdle_box((0, 0, H), pelvic_clusters)
    mb, mb_dims = girdle_box((0.5 * front_x, 0, H), mid_clusters)
    girdles = [fg, pg, mb]
    print(f"mid-body (spine+tail) bay:     "
          f"{mb_dims[0]:.0f} x {mb_dims[1]:.0f} x {mb_dims[2]:.0f} mm")

    groups = {
        "bone": Compound(bones + [spine_body]),
        "motor": Compound(motors),
        "tendon": Compound(tendons),
        "pulley": Compound(pulleys + [spine_pulleys]),
        "girdle": Compound(girdles),
    }
    return groups, H, front_x, fg_dims, pg_dims


def render_png(groups, path, H, front_x):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    styles = {
        "bone": ("#d7ac86", 1.0), "motor": ("#3c4a5a", 1.0),
        "tendon": ("#c17a3a", 1.0), "pulley": ("#8a9bb0", 1.0),
        "girdle": ("#9fb0c3", 0.12),
    }
    fig = plt.figure(figsize=(12, 6.5))
    ax = fig.add_subplot(111, projection="3d")
    for name, comp in groups.items():
        if comp is None or len(comp.solids()) == 0:
            continue
        verts, tris = comp.tessellate(0.4)
        V = np.array([[v.X, v.Y, v.Z] for v in verts])
        T = np.array(tris)
        color, alpha = styles[name]
        coll = Poly3DCollection(V[T], facecolor=color, edgecolor="none", alpha=alpha)
        coll.set_zsort("average")
        ax.add_collection3d(coll)
    ax.set_xlim(-220, front_x + 140); ax.set_ylim(-170, 170); ax.set_zlim(0, H + 150)
    ax.set_box_aspect((front_x + 360, 340, H + 150))
    ax.view_init(elev=20, azim=-60)
    ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(path, dpi=130, bbox_inches="tight", facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    here = os.path.dirname(__file__)
    groups, H, front_x, fg_dims, pg_dims = build()
    n_motors = len(groups["motor"].solids()) // 2   # each motor = body + spool
    print(f"motors placed: {n_motors}  (each = body + spool cylinder)")
    print(f"front (shoulder) girdle sized: "
          f"{fg_dims[0]:.0f} x {fg_dims[1]:.0f} x {fg_dims[2]:.0f} mm")
    print(f"pelvic girdle sized:           "
          f"{pg_dims[0]:.0f} x {pg_dims[1]:.0f} x {pg_dims[2]:.0f} mm")
    whole = Compound(list(groups.values()))
    bb = whole.bounding_box()
    print(f"overall bounding box: {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm")
    step = os.path.join(here, "tomcat_packaging.step")
    stl = os.path.join(here, "tomcat_packaging.stl")
    png = os.path.join(here, "tomcat_packaging.png")
    export_step(whole, step)
    export_stl(whole, stl, tolerance=0.25, angular_tolerance=0.3)
    render_png(groups, png, H, front_x)
    for p in (step, stl, png):
        print(f"  wrote {os.path.basename(p)}  ({os.path.getsize(p)} bytes)")
