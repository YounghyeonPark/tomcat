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
#: ⚠️ **Re-measured on COMPLIANT legs (M27), and the sign is negative.** M21 measured
#: this at `kp = 500` and found the centre of pressure bang-bang — 1 of 7 test points
#: kept two contacts, and the light foot unloaded past 1 mm — so along-line
#: regulation was written off. That was an artefact of the stiff gain, and M21 then
#: discovered compliance is what makes balance work at all without coming back to it.
#:
#: At `kp = 80` the same sweep is **linear and usable**: 5 of 7 points keep two
#: contacts, the total normal force stays at body weight throughout (no unloading),
#: and the CoP tracks at **-39.3 mm per mm of differential extension** across most of
#: the ±109 mm foot separation. Extending the FIRST leg of the pair moves the CoP
#: toward it, i.e. *against* `dhat` — hence the sign.
#:
#: ⚠️ **The authority is real and it still buys no envelope.** Closed-loop, the best
#: any sign or magnitude managed at the worst direction was **+1.8 mm** (about two
#: bisection steps) while degrading the undisturbed baseline from 1.38 to 5.53 mm.
#: `regulate_along_line` therefore stays **off** by default. See ADR-0032: this is
#: the third independent actuator to behave this way, and the pattern indicts the
#: control architecture rather than any of them.
COP_TRACK = -1.0 / 39.3

#: Clamp on the differential. At `kp = 80` two contacts survive to at least ±4 mm,
#: which is most of the CoP range; the stiff-leg 2 mm limit no longer applies.
COP_EXTENSION_LIMIT = 0.004

#: Metres of whole-body CoM sway per radian of per-joint lateral spine angle,
#: uniform across the three joints.
#:
#: Taken from `WholeBody.center_of_mass_y` with the real stance pose, not fitted:
#: 0.169 near zero, falling to 0.161 at full ROM as the chain curls. The small-angle
#: value is used and the command is clamped to ROM, so the mild softening only ever
#: makes the assist slightly weaker than commanded.
SPINE_SWAY_PER_RAD = 0.169

#: Proportional gain on the spine assist. 1.0 would command the sway that cancels
#: the current lateral DCM offset outright.
#:
#: ⚠️ **0.0 — the proportional assist is HARMFUL and is off by default (M24).**
#:
#: The law is `q = -gain * e / SPINE_SWAY_PER_RAD`, and a sway of `q` moves the CoM
#: by `SPINE_SWAY_PER_RAD * q = -gain * e`. **The loop gain is therefore `gain`
#: exactly, by construction**, so with any actuator lag it is marginal near 1 — and
#: measured, even 0.2 degrades the UNDISTURBED baseline 5x (2.15 -> 11.43 mm mean
#: DCM). At 0.5 and 1.0 the robot falls with no disturbance at all.
#:
#: M25 fixed the structure: planned once per stance and executed open-loop, the same
#: gain is stable to 1.0 and slightly IMPROVES the baseline (2.15 -> 1.38 mm). So the
#: instability was the control structure, not the actuator.
#:
#: ⚠️ **And it still buys nothing.** At 0.23 mm measurement resolution the planned
#: assist adds **+0.23 mm** to the worst-direction envelope against a credited
#: 36.6 mm, and at gain 1.0 it *costs* 12 mm in one direction. 0.5 is kept as the
#: default only because it is the quietest baseline — not because it buys envelope.
SPINE_GAIN = 0.5

#: Rate limit on the spine command, rad/s per joint.
#:
#: ⚠️ Not a motor limit. The drive does ~912 deg/s (ADR-0019) and an open-loop ramp
#: to full ROM survives at 300 deg/s. What destabilises is the *feedback* law
#: chattering: the sway perturbs the very DCM it is reacting to. Slew-limiting the
#: command breaks that loop without touching the authority.
SPINE_SLEW = 3.0


class BalanceHarness:
    """A trot-in-place balance controller running against a MuJoCo model.

    Trot **in place** deliberately: the question is whether the robot stays up under
    a disturbance, and propulsion only adds a second thing to get wrong.
    """

    def __init__(self, controller, mujoco, model, phase: float = 0.25,
                 cop_gain: float = COP_GAIN, regulate_along_line: bool = False,
                 latency: float | None = None, spine_gain: float = SPINE_GAIN,
                 use_spine: bool = True, spine_mode: str = "planned",
                 placement_mode: str = "axis"):
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
        self._spine_cmd = 0.0
        self._spine_from = 0.0
        self._spine_to = 0.0
        self._spine_next = 0.0

        # The spine is optional: a model built without `spine_dof` simply has no
        # such actuators, and the assist switches itself off rather than erroring.
        sp = self.b.spine.params
        self.spine_rom = float(min(abs(sp.lateral_q_min[0]), abs(sp.lateral_q_max[0])))
        self.spine_act = [name2id(model, obj.mjOBJ_ACTUATOR, f"spine_a{i}")
                          for i in range(1, sp.n_segments + 1)]
        self.has_spine = all(a >= 0 for a in self.spine_act)
        self.spine_gain = spine_gain
        self.use_spine = use_spine
        self.spine_mode = spine_mode
        self.placement_mode = placement_mode
        self._xoff = {}

    # ---------------------------------------------------------------- geometry
    def reachable_set(self, data, pair):
        """The parallelogram of CoP positions the NEXT stance can reach, in world.

        The same set `viable.support_set` builds, but around where the feet actually
        are now rather than around the nominal pose. Any placement already applied is
        undone first, so the sweep is measured from the nominal footfall.
        """
        pts = []
        for nm in pair:
            base = data.site_xpos[self.site[nm]][:2].copy()
            base[0] -= self._xoff.get(nm, self.nom_x) - self.nom_x
            pts.append(base + np.array([self.reach[0], 0.0]))
            pts.append(base + np.array([self.reach[1], 0.0]))
        return np.array(pts)

    def projected_placement(self, data, xi_end, pair):
        """2-D optimal foot placement: project the deadbeat CoP onto what is reachable.

        ⚠️ **This is the M31 law, and the difference from M8–M30 is one word.** Every
        earlier controller projected onto a single **axis** — the next diagonal's
        normal — which is where ADR-0031's "1-D collapse" actually lived.
        `viable.optimal_cop` solves the real 2-D problem: the deadbeat target is
        `g/(g-1) · ξ`, and when that is unreachable the best available choice is its
        projection onto the reachable **set**. A polygon projection, not a solver.

        Returns `(dx, lam)` — the fore-aft offset, and where along the support line
        the load would have to sit for the projection to be realised exactly.
        """
        from . import viable

        quad = viable._hull(self.reachable_set(data, pair))
        mid = self.feet_mid(data, pair)
        u = viable.optimal_cop(quad - mid, np.asarray(xi_end) - mid, self.growth) + mid

        a = data.site_xpos[self.site[pair[0]]][:2].copy()
        b = data.site_xpos[self.site[pair[1]]][:2].copy()
        a[0] -= self._xoff.get(pair[0], self.nom_x) - self.nom_x
        b[0] -= self._xoff.get(pair[1], self.nom_x) - self.nom_x
        dy = a[1] - b[1]
        lam = 0.5 if abs(dy) < 1e-9 else float(np.clip((u[1] - b[1]) / dy, 0.0, 1.0))
        dx = float(np.clip(u[0] - (lam * a[0] + (1 - lam) * b[0]), *self.reach))
        return dx, lam

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

    def body_lateral_axis(self, data):
        """The trunk's own +y in world coordinates — the axis the spine sways along.

        Read from the live orientation rather than assumed, because the trunk yaws
        during a recovery and a fixed axis would point the assist sideways.
        """
        r = np.zeros(9)
        self.mj.mju_quat2Mat(r, data.qpos[3:7])
        v = r.reshape(3, 3)[:, 1][:2]
        n = np.linalg.norm(v)
        return v / n if n > 1e-9 else np.array([0.0, 1.0])

    def plan_spine(self, data, xi_end, support):
        """Decide the spine offset ONCE per stance, from the predicted end state.

        ⚠️ This is the M25 structure, and the reason it exists is ADR-0029: a
        per-timestep proportional law puts the actuator in its own position feedback
        path with **unity loop gain**, so even a 0.2 gain degraded the undisturbed
        baseline fivefold. Deciding once per step and executing open-loop closes the
        loop at the same rate the foot placement already does — which is the one
        control structure in this harness that demonstrably works.
        """
        if not (self.has_spine and self.use_spine):
            return 0.0
        yhat = self.body_lateral_axis(data)
        e = float((xi_end - support) @ yhat)
        return float(np.clip(-self.spine_gain * e / SPINE_SWAY_PER_RAD,
                             -self.spine_rom, self.spine_rom))

    def drive_spine(self, data, u: float):
        """Execute the planned offset open-loop: a C1 ramp across the stance."""
        if not (self.has_spine and self.use_spine):
            return 0.0
        blend = u * u * (3.0 - 2.0 * u)
        q = self._spine_from + (self._spine_to - self._spine_from) * blend
        for a in self.spine_act:
            data.ctrl[a] = q
        return q

    def spine_assist(self, data, xi, support):
        """Reactive per-timestep assist. ⚠️ HARMFUL — kept only for the ADR-0029 test.

        The spine cannot move the centre of pressure — that is pinned to the support
        line — so it does the other thing: it moves the **CoM toward the CoP**. An
        offset of `e` along the trunk's lateral axis is cancelled by a sway of `-e`,
        which is `-e / SPINE_SWAY_PER_RAD` at each of the three joints.

        ⚠️ This is the balance authority ADR-0009 bought and `control.py` credits as
        `plant.spine`. Until M21 it had never been exercised in a physics model at
        all: M17 and M20's harnesses ran a rigid trunk, so ~22 mm of the ~53 mm
        envelope was outside the simulation.
        """
        if not (self.has_spine and self.use_spine):
            return 0.0
        yhat = self.body_lateral_axis(data)
        e = float((xi - support) @ yhat)
        q = float(np.clip(-self.spine_gain * e / SPINE_SWAY_PER_RAD,
                          -self.spine_rom, self.spine_rom))
        # Rate-limit, so the assist cannot chase its own reaction.
        step = SPINE_SLEW * self.dt
        q = float(np.clip(q, self._spine_cmd - step, self._spine_cmd + step))
        self._spine_cmd = q
        for a in self.spine_act:
            data.ctrl[a] = q
        return q

    def feet_mid(self, data, pair):
        return np.mean([data.site_xpos[self.site[nm]][:2] for nm in pair], axis=0)

    def dcm(self, data, omega):
        c, v = self.com(data)
        return c[:2] + v[:2] / omega, c

    # -------------------------------------------------------------------- init
    def reset(self, settle: float = 0.06):
        self._spine_cmd = 0.0
        self._spine_from = 0.0
        self._spine_to = 0.0
        self._spine_next = 0.0
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
        self._xoff = xoff
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
                    end = p + (xi - p) * math.exp(self.omega * left)
                    if self.placement_mode == "projected":
                        dx, _lam = self.projected_placement(data, end, DIAGONALS[nxt])
                    else:
                        dhat_n, pn = self.axes(DIAGONALS[nxt], data)
                        nominal = float(self.feet_mid(data, DIAGONALS[nxt]) @ pn) \
                            - (lift[DIAGONALS[nxt][0]] - self.nom_x) * pn[0]
                        err = float(end @ pn) - nominal
                        if abs(pn[0]) > 1e-6:
                            dx = float(np.clip(self.deadbeat * err / pn[0], *self.reach))
                    if left <= self.latency:
                        frozen = True
                        if self.spine_mode == "planned":
                            # Same instant the foot target is committed.
                            self._spine_next = self.plan_spine(
                                data, end, self.feet_mid(data, DIAGONALS[nxt]))

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

                # --- the spine: PLANNED at the freeze point, then open-loop ---
                if self.spine_mode == "reactive":
                    self.spine_assist(data, xi, self.feet_mid(data, DIAGONALS[cur]))
                else:
                    self.drive_spine(data, k / n)

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
            self._spine_from = self._spine_to
            self._spine_to = self._spine_next
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
          kp: int = 80, phase: float = 0.25, spine: bool = False,
          spine_kp: int = 1000):
    """A model wired for balance work, with the diagonal pair on the floor.

    ``kp`` defaults to a **compliant** 80. Stiff servos do not merely track better —
    they make ground-reaction distribution bang-bang and the balance loop does not
    close at all (ADR-0026). This is a physical choice, not a solver tolerance.

    ⚠️ ``spine_kp`` is the opposite: it wants to be **stiff**. The lateral chain
    carries the whole forequarters, and at 150 it wobbles badly enough to fell an
    otherwise-clean baseline in 10 steps. At 1000 the baseline is quiet again
    (2.1 mm mean over 25 steps). The legs want compliance; the spine wants
    stiffness, and they are not the same knob.
    """
    from . import mjcf

    q = mjcf.stance_pose(controller, phase)
    h = mjcf.rest_height(controller, q, DIAGONALS["A"])
    xml = mjcf.build_mjcf(controller, q, height=h, mu=mu, timestep=timestep,
                          spine_dof=spine)
    xml = xml.replace('kp="120" kv="4"', f'kp="{kp}" kv="{max(2, kp // 36)}"')
    xml = xml.replace('kp="30" kv="1.5"', f'kp="{spine_kp}" kv="{max(2, spine_kp // 20)}"')
    return mujoco.MjModel.from_xml_string(xml)
