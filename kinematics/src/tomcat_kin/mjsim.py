# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""Closed-loop balance in simulation — the item M17 and M20 were both blocked on.

Two open questions share one cause. M17 could not settle the **envelope magnitude**
because its harness drifted 25 mm against a 30 mm signal. M20 could not check
**ADR-0019/0020's friction costs** because a diagonal stance topples inside one
stance time, so no open-loop test holds the robot up long enough to read the
friction demand. Neither is a separate problem: both need a controller that keeps
the robot standing while the measurement is taken.

What M17's harness actually got wrong, measured rather than assumed
-------------------------------------------------------------------
A diagonal support is a **line**, so the centre of pressure has one free direction
(along it) and one frozen (across it). They need different controllers, and M17
only built one:

===============  ==========================================  ====================
component        controlled by                               M17
===============  ==========================================  ====================
**across** line  where the next pair of feet lands           deadbeat placement
**along** line   where the CoP sits between the two feet     *nothing*
===============  ==========================================  ====================

Instrumented, the across-line DCM stayed under 5 mm for three steps while the
along-line one ran +1.7 → +22 → +43 → +90 mm and then destroyed the other.

⚠️ **But that diagnosis was wrong, and the correction is the interesting part.** The
along-line growth was a *symptom*: the swing foot was landing at 0.31 m/s, because a
`sin(pi u)` arc has non-zero slope at touchdown. It hammered the contact so the
stance never settled on two feet. With a `(1 - cos(2 pi u))/2` profile — at rest at
both ends — the run went from 14 steps to 40 with **no along-line regulation at
all**, which is why `regulate_along_line` defaults to False.

Explicit CoP regulation turns out not to be available anyway. With stiff position
servos the load transfer is bang-bang: ±1 mm of differential stance-leg extension
swings the centre of pressure across the whole ±109 mm foot separation. What the
loop actually needs is **compliance** — at leg `kp` 80–150 the undisturbed drift
stays near 2 mm over 40 steps; at 250 and above it winds up and falls.

⚠️ **Still a model of a model.** Joints are position servos, not tendons; ADR-0016's
electronics pipeline and M12's actuation ramp are not in the loop. This answers
"does the reduced-order envelope survive rigid-body dynamics", not "will the built
robot stand up".
"""

from __future__ import annotations

import math

import numpy as np

from . import LegModel
from .params import DEFAULT_FORELEG, DEFAULT_HINDLEG

DIAGONALS = {"A": ("LF", "RR"), "B": ("RF", "LR")}

#: Proportional gain on the along-line DCM. The CoP law ``p = xi + k (xi - ref)``
#: makes that component decay at ``omega * k``, so k = 1 buys one time constant per
#: 1/omega ~ 129 ms — brisk against a 200 ms stance without demanding CoP authority
#: the foot separation cannot supply.
COP_GAIN = 1.0

#: Metres of differential stance-leg extension per metre of CoP error.
#:
#: ⚠️ Measured, not chosen. The CoP is violently sensitive to extension — 4 mm of
#: differential swings it the full +/-108.7 mm to a single foot, i.e. **27 mm of CoP
#: per mm of leg**. A gain picked by eye saturates instantly and reads as a fall.
COP_TRACK = 1.0 / 27.0

#: Clamp on the differential. Past ~2 mm the light foot unloads (ncon drops to 1) and
#: the CoP is pinned at the other foot, so more command buys nothing and costs
#: contact. Measured on the same sweep.
COP_EXTENSION_LIMIT = 0.002


class BalanceHarness:
    """A trot-in-place balance controller running against a MuJoCo model.

    Trot **in place** deliberately: the question is whether the robot stays up under
    a disturbance, and propulsion only adds a second thing to get wrong.
    """

    def __init__(self, controller, mujoco, model, phase: float = 0.25,
                 cop_gain: float = COP_GAIN, regulate_along_line: bool = False,
                 latency: float | None = None):
        from . import control as ctl

        self.c = controller
        self.b = controller.body
        self.mj = mujoco
        self.m = model
        self.dt = model.opt.timestep
        self.cop_gain = cop_gain
        self.regulate = regulate_along_line

        self.plant = ctl.StepPlant.from_gait(controller, n=96, latency=0.0075,
                                             floor_mu=0.8)
        self.T = self.plant.stance
        self.latency = self.plant.latency if latency is None else latency
        self.reach = (float(self.plant.reach[0]), float(self.plant.reach[1]))
        self.growth = math.exp(self.plant.omega * self.T)
        self.deadbeat = self.growth / (self.growth - 1.0)
        # Seeded from the analytical plant so the harness is usable before `reset`;
        # `reset` replaces it with omega at the model's own settled CoM height.
        self.omega = self.plant.omega

        self.q0 = {nm: np.asarray(controller.state(phase).legs[nm].q, dtype=float)
                   for nm in self.b.leg_names}
        self.nom_x, self.nom_z = controller.params.nominal_foot
        self.step_h = controller.params.step_height

        self.leg = {nm: LegModel(DEFAULT_FORELEG
                                 if self.b.mounts[nm].girdle.value == "front"
                                 else DEFAULT_HINDLEG) for nm in self.b.leg_names}
        self.phi = {nm: self.leg[nm].forward(self.q0[nm])[2] for nm in self.b.leg_names}

        name2id, obj = mujoco.mj_name2id, mujoco.mjtObj
        self.act = {f"{nm}{i}": name2id(model, obj.mjOBJ_ACTUATOR, f"{nm}_a{i}")
                    for nm in self.b.leg_names for i in (1, 2, 3)}
        self.adr = {f"{nm}{i}": model.jnt_qposadr[
            name2id(model, obj.mjOBJ_JOINT, f"{nm}_q{i}")]
            for nm in self.b.leg_names for i in (1, 2, 3)}
        self.site = {nm: name2id(model, obj.mjOBJ_SITE, f"{nm}_site")
                     for nm in self.b.leg_names}
        self._f = np.zeros(6)

    # ---------------------------------------------------------------- geometry
    def axes(self, pair, data):
        """Unit vector along the support line, and its normal, from LIVE contacts.

        ⚠️ Taken from where the feet actually are, not from the nominal stance. The
        two diagonals' normals are 52.4 deg apart (M17) and placement moves them
        further, so a fixed axis silently corrects along the wrong direction —
        which is what M17's first harness did.
        """
        p = [data.site_xpos[self.site[nm]][:2] for nm in pair]
        d = p[1] - p[0]
        n = np.linalg.norm(d)
        if n < 1e-9:
            return np.array([1.0, 0.0]), np.array([0.0, 1.0])
        d = d / n
        return d, np.array([-d[1], d[0]])

    def ik(self, nm, x, z):
        return self.leg[nm].inverse(np.array([x, z, self.phi[nm]]))

    def command(self, data, nm, x, z):
        data.ctrl[[self.act[f"{nm}{i}"] for i in (1, 2, 3)]] = self.ik(nm, x, z)

    # ------------------------------------------------------------------- state
    def com(self, data):
        self.mj.mj_comPos(self.m, data)
        return data.subtree_com[0].copy(), data.cvel[1][3:6].copy()

    def cop(self, data, fallback):
        """Centre of pressure from the contact forces — measured, not assumed."""
        num, den = np.zeros(2), 0.0
        for i in range(data.ncon):
            self.mj.mj_contactForce(self.m, data, i, self._f)
            if self._f[0] > 1e-6:
                num += data.contact[i].pos[:2] * self._f[0]
                den += float(self._f[0])
        return (num / den, den) if den > 1e-6 else (fallback, 0.0)

    def feet_mid(self, data, pair):
        return np.mean([data.site_xpos[self.site[nm]][:2] for nm in pair], axis=0)

    def dcm(self, data, omega):
        c, v = self.com(data)
        return c[:2] + v[:2] / omega, c

    # -------------------------------------------------------------------- init
    def reset(self, settle: float = 0.06):
        d = self.mj.MjData(self.m)
        for nm in self.b.leg_names:
            q = (self.q0[nm] if nm in DIAGONALS["A"]
                 else self.ik(nm, self.nom_x, self.nom_z + self.step_h))
            for i, v in enumerate(q, start=1):
                d.qpos[self.adr[f"{nm}{i}"]] = v
                d.ctrl[self.act[f"{nm}{i}"]] = v
        self.mj.mj_forward(self.m, d)
        for _ in range(int(settle / self.dt)):
            self.mj.mj_step(self.m, d)
        c, _ = self.com(d)
        self.omega = math.sqrt(9.81 / float(c[2]))
        return d

    # -------------------------------------------------------------------- loop
    def run(self, data, steps: int = 12, disturbance=None, record: bool = False):
        """Trot in place for `steps` stances. Returns per-step diagnostics."""
        if disturbance is not None:
            data.qvel[0] += disturbance[0]
            data.qvel[1] += disturbance[1]

        cur, nxt = "A", "B"
        xoff = {nm: self.nom_x for nm in self.b.leg_names}
        out, fell = [], False

        for step in range(steps):
            lift = {nm: xoff[nm] for nm in DIAGONALS[nxt]}
            dx, frozen, ext = 0.0, False, 0.0
            n = int(self.T / self.dt)

            for k in range(n):
                left = self.T - k * self.dt
                xi, c3 = self.dcm(data, self.omega)
                p, load = self.cop(data, xi)

                # --- across the line: where the NEXT pair lands ---------------
                if not frozen:
                    dhat_n, pn = self.axes(DIAGONALS[nxt], data)
                    end = p + (xi - p) * math.exp(self.omega * left)
                    nominal = float(self.feet_mid(data, DIAGONALS[nxt]) @ pn) \
                        - (lift[DIAGONALS[nxt][0]] - self.nom_x) * pn[0]
                    err = float(end @ pn) - nominal
                    if abs(pn[0]) > 1e-6:
                        dx = float(np.clip(self.deadbeat * err / pn[0], *self.reach))
                    if left <= self.latency:
                        frozen = True

                # --- along the line: where the CoP sits -----------------------
                if self.regulate and load > 1.0:
                    dhat_c, _ = self.axes(DIAGONALS[cur], data)
                    mid = self.feet_mid(data, DIAGONALS[cur])
                    xi_par = float((xi - mid) @ dhat_c)
                    want = xi_par + self.cop_gain * xi_par      # ref = mid
                    have = float((p - mid) @ dhat_c)
                    ext = float(np.clip(COP_TRACK * (want - have),
                                        -COP_EXTENSION_LIMIT, COP_EXTENSION_LIMIT))
                    a, bb = DIAGONALS[cur]
                    self.command(data, a, xoff[a], self.nom_z - ext)
                    self.command(data, bb, xoff[bb], self.nom_z + ext)

                # --- swing ----------------------------------------------------
                # ⚠️ Vertical profile must be C1 at BOTH ends. `sin(pi u)` peaks
                # correctly but lands with -pi h / T = -0.31 m/s of downward foot
                # speed, which hammers the contact and costs the stance: ncon never
                # settled at 2. `(1 - cos(2 pi u))/2` has zero slope at u = 0 and
                # u = 1, so the foot arrives at rest. This is the same C0 defect M5
                # and M6 fixed in the shipped swing profile, reintroduced here by
                # hand — the reason `GaitParams.swing_profile` defaults to "matched".
                u = k / n
                arc = 0.5 * self.step_h * (1.0 - math.cos(2.0 * math.pi * u))
                blend = u * u * (3 - 2 * u)
                for nm in DIAGONALS[nxt]:
                    x = lift[nm] + blend * (self.nom_x + dx - lift[nm])
                    self.command(data, nm, x, self.nom_z + arc)

                self.mj.mj_step(self.m, data)
                if self.com(data)[0][2] < 0.11:
                    fell = True
                    break
            if fell:
                break

            for nm in DIAGONALS[nxt]:
                xoff[nm] = self.nom_x + dx
            for nm in DIAGONALS[cur]:
                self.command(data, nm, xoff[nm], self.nom_z + self.step_h)

            xi, c3 = self.dcm(data, self.omega)
            dhat, pn = self.axes(DIAGONALS[nxt], data)
            mid = self.feet_mid(data, DIAGONALS[nxt])
            out.append({
                "step": step, "pair": nxt,
                "perp": float((xi - mid) @ pn),
                "para": float((xi - mid) @ dhat),
                "dx": dx, "ncon": int(data.ncon), "height": float(c3[2]),
            })
            cur, nxt = nxt, cur
        return out, fell


def build(controller, mujoco, mu: float = 0.9, timestep: float = 1e-4,
          kp: int = 500, phase: float = 0.25):
    """A model wired for balance work: stiff servos, the diagonal pair on the floor."""
    from . import mjcf

    q = mjcf.stance_pose(controller, phase)
    h = mjcf.rest_height(controller, q, DIAGONALS["A"])
    xml = mjcf.build_mjcf(controller, q, height=h, mu=mu, timestep=timestep)
    xml = xml.replace('kp="120" kv="4"', f'kp="{kp}" kv="{kp // 36}"')
    return mujoco.MjModel.from_xml_string(xml)
