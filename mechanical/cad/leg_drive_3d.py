# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""How a leg is driven, rendered in 3-D -- the hind leg's tendon drive (M122).

The CAD leg (`tomcat_leg_detail.build`) is built ONCE, at the stance, and its
solids are dealt to the four links the way `link_inertia.assign` deals their
mass (hardware along a bone to that bone, joint hardware to the distal link).
Each frame then only moves those meshes: every link turns about its proximal
joint by the change in its absolute angle. The drive is re-solved every frame
(`tendon_exit.drive`, via senses held from the stance): each cable leaves its
spool, leads to a ferrule on the trunk wall, runs in a Bowden CONDUIT (grey) --
to a trunk bracket above the hip for the hip pair, to a ferrule on the FEMUR for
the knee and ankle pairs, so those conduits flex as the hip turns -- and runs
free from there to its anchor on its sheave. A free run drawn THICK and bright
is shortening -- its spool is reeling it in, so it is the one pulling.

The rear girdle is drawn translucent so the three motors inside it, on axes
along x, and their spools can be seen; each spool carries a marker that turns
with it. The camera orbits slowly so the depth reads.

    python mechanical/cad/leg_drive_3d.py          # -> leg_drive_3d.gif

Rendering is VTK, off-screen.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "kinematics", "src"))

import leg_tendons as LT  # noqa: E402
import tendon_exit as TE  # noqa: E402

COLORS = {"hip": (0.80, 0.22, 0.17), "knee": (0.14, 0.44, 0.64),
          "ankle": (0.12, 0.52, 0.29)}
MATERIAL = {"tube": (0.23, 0.25, 0.28), "insert": (0.74, 0.77, 0.80),
            "clevis": (0.74, 0.77, 0.80), "bearing": (0.42, 0.46, 0.52),
            "shaft": (0.55, 0.58, 0.62), "pad": (0.17, 0.17, 0.17),
            "motor": (0.35, 0.37, 0.40), "spool": (0.80, 0.80, 0.82),
            "ferrule": (0.90, 0.72, 0.30), "trunk_ferrule": (0.90, 0.72, 0.30)}
LINKS = ("femur", "tibia", "meta", "paw")


def motion(n_each: int = 30):
    """Joint angles, one joint at a time and then together, inside the ROM."""
    from tomcat_kin.params import DEFAULT_HINDLEG as LEG
    q0 = TE.stance("hind")
    lo, hi = np.asarray(LEG.q_min), np.asarray(LEG.q_max)
    amp = np.radians([35.0, 35.0, 35.0])
    qs, labels = [], []
    for j, nm in enumerate(("hip", "knee", "ankle")):
        for k in range(n_each):
            q = q0.copy()
            q[j] += amp[j] * math.sin(2 * math.pi * k / n_each)
            qs.append(np.clip(q, lo + 0.05, hi - 0.05))
            labels.append("%s alone" % nm)
    for k in range(2 * n_each):
        ph = 2 * math.pi * k / (2 * n_each)
        q = q0 + amp * np.array([math.sin(ph), math.sin(ph + 1.9), math.sin(ph + 3.6)])
        qs.append(np.clip(q, lo + 0.05, hi - 0.05))
        labels.append("all three")
    return q0, qs, labels
#: Joint hardware goes to the distal link (`link_inertia`); a CLEVIS and an
#: INSERT belong to the bone they are bonded to, so they go by nearest bone.
BY_BONE = ("tube", "insert", "clevis")


def _polydata(solids, tol=0.25):
    import vtk
    pts = vtk.vtkPoints()
    cells = vtk.vtkCellArray()
    base = 0
    for sd in solids:
        verts, tris = sd.tessellate(tol)
        for v in verts:
            pts.InsertNextPoint(v.X, v.Y, v.Z)
        for t in tris:
            cells.InsertNextCell(3)
            for k in t:
                cells.InsertCellPoint(base + k)
        base += len(verts)
    pd = vtk.vtkPolyData()
    pd.SetPoints(pts)
    pd.SetPolys(cells)
    nm = vtk.vtkPolyDataNormals()
    nm.SetInputData(pd)
    nm.SetFeatureAngle(35.0)
    nm.Update()
    return nm.GetOutput()


def _actor(pd, color, opacity=1.0, specular=0.3):
    import vtk
    m = vtk.vtkPolyDataMapper()
    m.SetInputData(pd)
    a = vtk.vtkActor()
    a.SetMapper(m)
    p = a.GetProperty()
    p.SetColor(*color)
    p.SetOpacity(opacity)
    p.SetSpecular(specular)
    p.SetAmbient(0.18)
    p.SetDiffuse(0.85)
    p.SetSpecularPower(30)
    return a


def _seg_dist(p, a, b):
    d = b - a
    t = float(np.clip((p - a) @ d / (d @ d), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * d)))


def _deal(comps, pts):
    """{(link, material): [solids]} for the leg's moving parts, and
    {material: [solids]} for the trunk-side ones (motors, spools, idlers)."""
    import tomcat_leg_detail as LD
    y0 = LD.TRACK_Y
    J = {"hip": np.array([pts[0][0], y0, pts[0][1]]),
         "knee": np.array([pts[1][0], y0, pts[1][1]]),
         "ankle": np.array([pts[2][0], y0, pts[2][1]])}
    B = {b: (np.array([pts[i][0], y0, pts[i][1]]), np.array([pts[i + 1][0], y0, pts[i + 1][1]]))
         for i, b in enumerate(LINKS)}
    to_link = {"hip": "femur", "knee": "tibia", "ankle": "meta"}
    moving, fixed = {}, {}
    for g, comp in comps.items():
        if g in ("tendon", "anchor", "cable", "conduit"):
            continue                      # drawn live, every frame
        for sd in comp.solids():
            if g in ("motor", "spool", "trunk_ferrule"):
                fixed.setdefault(g, []).append(sd)
                continue
            c = sd.center()
            c = np.array([c.X, c.Y, c.Z])
            if g == "pad":
                link = "paw"
            elif g == "ferrule":
                link = "femur"            # the conduit ferrules ride the femur
            elif g in BY_BONE:
                link = min(B, key=lambda b: _seg_dist(c, *B[b]))
            else:
                link = to_link[min(J, key=lambda j: float(np.linalg.norm(c - J[j])))]
            mat = g
            if g == "sheave":
                # tint each joint's sheave with its cable's colour
                jn = min(J, key=lambda j: float(np.linalg.norm(c - J[j])))
                mat = "sheave:" + jn
            moving.setdefault((link, mat), []).append(sd)
    return moving, fixed


def _angles(P):
    """Absolute angle of each of the four links in the x-z plane."""
    return [math.atan2(P[i + 1][1] - P[i][1], P[i + 1][0] - P[i][0]) for i in range(4)]


def _tube(points, radius):
    import vtk
    p = vtk.vtkPoints()
    line = vtk.vtkPolyLine()
    line.GetPointIds().SetNumberOfIds(len(points))
    for i, q in enumerate(points):
        p.InsertNextPoint(*q)
        line.GetPointIds().SetId(i, i)
    cells = vtk.vtkCellArray()
    cells.InsertNextCell(line)
    pd = vtk.vtkPolyData()
    pd.SetPoints(p)
    pd.SetLines(cells)
    tf = vtk.vtkTubeFilter()
    tf.SetInputData(pd)
    tf.SetRadius(radius)
    tf.SetNumberOfSides(10)
    tf.CappingOn()
    tf.Update()
    return tf.GetOutput()


def main(path=None, size=(1200, 820), fps=16, n_each=30):
    import vtk
    from PIL import Image
    from vtk.util.numpy_support import vtk_to_numpy

    import tomcat_leg_detail as LD
    import tomcat_trunk as TT
    from tomcat_kin.params import DEFAULT_HINDLEG

    path = path or os.path.join(HERE, "leg_drive_3d.gif")
    print("building the leg once ...", flush=True)
    comps, report, pts0 = LD.build(DEFAULT_HINDLEG, role="hind")
    moving, fixed = _deal(comps, pts0)
    print("building the rear girdle ...", flush=True)
    girdle = TT.rigid_body(0)

    ren = vtk.vtkRenderer()
    ren.SetBackground(1.0, 1.0, 1.0)
    ren.SetBackground2(0.86, 0.89, 0.93)
    ren.GradientBackgroundOn()
    win = vtk.vtkRenderWindow()
    win.SetOffScreenRendering(1)
    win.SetSize(*size)
    win.SetMultiSamples(8)
    win.AddRenderer(ren)
    ren.SetUseDepthPeeling(1)
    ren.SetMaximumNumberOfPeels(8)

    ga = _actor(_polydata(girdle.solids(), 0.6), (0.62, 0.70, 0.80), opacity=0.18,
                specular=0.1)
    ren.AddActor(ga)
    for g, sols in fixed.items():
        ren.AddActor(_actor(_polydata(sols), MATERIAL[g]))
    link_actors = {}
    for (link, mat), sols in moving.items():
        if mat.startswith("sheave:"):
            base = np.array(COLORS[mat.split(":")[1]])
            col = tuple(0.55 * base + 0.45 * np.array([0.85, 0.85, 0.85]))
        else:
            col = MATERIAL.get(mat, (0.7, 0.7, 0.7))
        a = _actor(_polydata(sols), col)
        ren.AddActor(a)
        link_actors.setdefault(link, []).append(a)

    # ground
    plane = vtk.vtkPlaneSource()
    plane.SetOrigin(-170, -40, -178.5)
    plane.SetPoint1(170, -40, -178.5)
    plane.SetPoint2(-170, 140, -178.5)
    pm = vtk.vtkPolyDataMapper()
    pm.SetInputConnection(plane.GetOutputPort())
    pa = vtk.vtkActor()
    pa.SetMapper(pm)
    pa.GetProperty().SetColor(0.80, 0.82, 0.84)
    pa.GetProperty().SetOpacity(0.6)
    ren.AddActor(pa)

    # live actors: cables, anchors, spool markers
    q0, qs, labels = motion(n_each)
    plan = TE.drive("hind", side=+1.0, q=q0)
    senses = {k: r["senses"] for k, r in plan.items()}
    L0 = {k: r["free_length"] for k, r in plan.items()}
    planes = LD.plane_layout({"hip": 6.4, "knee": 6.4, "ankle": 4.4},
                             {j: LD.BEARING[j][2] for j in ("hip", "knee", "ankle")})["hip"]["plane"]
    cable_actors, conduit_actors = {}, {}
    for k in plan:
        a = vtk.vtkActor()
        a.SetMapper(vtk.vtkPolyDataMapper())
        ren.AddActor(a)
        cable_actors[k] = a
        c = vtk.vtkActor()
        c.SetMapper(vtk.vtkPolyDataMapper())
        c.GetProperty().SetColor(0.55, 0.57, 0.60)
        c.GetProperty().SetOpacity(0.85)
        ren.AddActor(c)
        conduit_actors[k] = c
    marker = {}
    for t in ("hip", "knee", "ankle"):
        a = vtk.vtkActor()
        a.SetMapper(vtk.vtkPolyDataMapper())
        a.GetProperty().SetColor(*COLORS[t])
        ren.AddActor(a)
        marker[t] = a

    text = vtk.vtkTextActor()
    text.GetTextProperty().SetFontSize(22)
    text.GetTextProperty().SetColor(0.15, 0.18, 0.22)
    text.SetPosition(20, size[1] - 40)
    ren.AddViewProp(text)
    legend = vtk.vtkTextActor()
    legend.SetInput("red / blue / green = hip / knee / ankle cables     THICK = shortening (pulling)\n"
                    "grey = Bowden conduits, gold = their ferrules; motors inside the translucent girdle")
    legend.GetTextProperty().SetFontSize(16)
    legend.GetTextProperty().SetColor(0.25, 0.28, 0.32)
    legend.SetPosition(20, 16)
    ren.AddViewProp(legend)

    kit = vtk.vtkLightKit()
    kit.SetKeyLightIntensity(0.95)
    kit.AddLightsToRenderer(ren)

    cam = ren.GetActiveCamera()
    focal = np.array([-30.0, 40.0, -50.0])
    cam.SetFocalPoint(*focal)
    cam.SetViewUp(0, 0, 1)
    cam.SetViewAngle(30)

    A0 = _angles(LT.joints(q0, DEFAULT_HINDLEG))
    J0 = LT.joints(q0, DEFAULT_HINDLEG)
    frames = []
    prev = None
    grab = vtk.vtkWindowToImageFilter()
    grab.SetInput(win)
    grab.ReadFrontBufferOff()
    n = len(qs)
    for i, q in enumerate(qs):
        P = LT.joints(q, DEFAULT_HINDLEG)
        Aq = _angles(P)
        for k, link in enumerate(LINKS):
            tr = vtk.vtkTransform()
            tr.PostMultiply()
            tr.Translate(-J0[k][0], 0.0, -J0[k][1])
            tr.RotateY(-math.degrees(Aq[k] - A0[k]))
            tr.Translate(P[k][0], 0.0, P[k][1])
            for a in link_actors.get(link, []):
                a.SetUserTransform(tr)
        runs = TE.drive("hind", side=+1.0, q=q, senses=senses)
        lens = {k: r["free_length"] for k, r in runs.items()}
        for (t, sd), r in runs.items():
            yp = LD.TRACK_Y + planes[t]
            T, F1 = r["lead"]
            pull = prev is not None and lens[(t, sd)] < prev[(t, sd)] - 1e-6
            a = cable_actors[(t, sd)]
            app = vtk.vtkAppendPolyData()
            app.AddInputData(_tube([tuple(T), tuple(F1)], 0.9))
            app.AddInputData(_tube([(x, yp, z) for x, z in r["points"]],
                                   1.5 if pull else 0.85))
            app.Update()
            a.GetMapper().SetInputData(app.GetOutput())
            conduit_actors[(t, sd)].GetMapper().SetInputData(
                _tube([tuple(p) for p in r["conduit"]], TE.CONDUIT_OD / 2))
            c = np.array(COLORS[t])
            a.GetProperty().SetColor(*(c if pull else 0.55 * c + 0.45))
            a.GetProperty().SetAmbient(0.35 if pull else 0.1)
        prev = lens
        for t in ("hip", "knee", "ankle"):
            r = plan[(t, +1)]
            x, y, z = r["seat"]
            xs = r["spool_x"]
            end = 1.0 if xs > x else -1.0
            ang = -(lens[(t, +1)] - L0[(t, +1)]) / LT.SPOOL_R
            face = xs + end * 4.3
            rr = LT.SPOOL_R + 1.5
            marker[t].GetMapper().SetInputData(
                _tube([(face, y, z), (face, y + rr * math.cos(ang), z + rr * math.sin(ang))], 1.1))
        # a slow orbit, so the depth reads
        # outboard (+y), in front (+x), above: azimuth from +x toward +y
        az = math.radians(55.0 + 25.0 * math.sin(2 * math.pi * i / n))
        el = math.radians(18.0)
        dist = 680.0
        cam.SetPosition(focal[0] + dist * math.cos(el) * math.cos(az),
                        focal[1] + dist * math.cos(el) * math.sin(az),
                        focal[2] + dist * math.sin(el))
        cam.SetFocalPoint(*focal)
        cam.SetViewUp(0, 0, 1)
        d = np.degrees(q - q0)
        text.SetInput("%s      hip %+5.1f   knee %+5.1f   ankle %+5.1f deg"
                      % (labels[i], d[0], d[1], d[2]))
        win.Render()
        grab.Modified()
        grab.Update()
        img = grab.GetOutput()
        w, h, _ = img.GetDimensions()
        arr = vtk_to_numpy(img.GetPointData().GetScalars()).reshape(h, w, -1)[::-1]
        frames.append(Image.fromarray(arr[:, :, :3].copy()))
        if i % 20 == 0:
            print("frame %d / %d" % (i, n), flush=True)
    pal = [f.convert("P", palette=Image.ADAPTIVE, colors=255) for f in frames]
    pal[0].save(path, save_all=True, append_images=pal[1:], duration=int(1000 / fps),
                loop=0, optimize=True)
    frames[len(frames) // 3].save(os.path.splitext(path)[0] + "_still.png")
    print("wrote %s (%d frames)" % (path, len(frames)))
    return path


if __name__ == "__main__":
    main()
