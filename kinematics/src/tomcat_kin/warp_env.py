# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""`TomcatEnv`, batched on the GPU with MuJoCo-Warp (M136, ADR-0130).

ADR-0079 chose MJX-Warp as the training backend; this is that backend built.
It is the SAME task as `gym_env.TomcatStand(residual_n=...)` on
`stand_baseline.StandBaseline`, for `nworld` robots at once:

- **On the GPU**, one control period (75 physics steps at 0.1 ms) is captured
  as ONE CUDA graph. Every physics step it runs the inner rotor servo
  (`wbc.rotor_command`, the motor closing on its own shaft every step), then
  `mujoco_warp.step`, then pushes `sensordata` into a ring buffer, so the
  7.5 ms sensor delay (3-12 ms randomised, per world) is read off the device.
- **On the host**, in NumPy over all worlds at once, everything the board and
  the controller do once per control step: sensor error, quantisation, the
  joint estimate, the stance height, reward, termination, the baseline.
- **Per world:** domain randomisation (mass, floor friction, G3 rate through
  batched model fields; latency through the ring; the sensor errors on the
  host), and reset when its episode ends.

⚠️ **Speed, in CONTROL steps** (one is 75 physics steps). ADR-0079's 262k
steps/s were physics steps. Captured in a graph, the plant alone does
8-9k control steps/s at 2048-16384 worlds on the 4090, against ~500 for 16 CPU
processes: 10-18x, not hundreds. The fine timestep is the drivetrain's
(`timestep-silently-breaks-the-drivetrain`), and it is what the speed costs.

⚠️ **Not identical to the CPU env in one place: the reset fixture.** The CPU env
stands each episode up with a privileged joint PD on ITS randomised plant
(`TomcatEnv._settle`). Here every world restarts from the NOMINAL plant's
settled state and its draw applies from the first step, so a heavy draw sags
at the start. The baseline and the policy both start from there.
"""


import math

import numpy as np

from . import mjcf_tendon as MT
from .env import LEGS, PAIRS, SPINE, TomcatEnv
from .gym_env import LOAD_KEYS
from .params import DEFAULT_FORELEG, DEFAULT_HINDLEG
from . import stand_baseline as SB

#: Latency is a ring of physics steps; this many cover RANGES' 12 ms.
MAX_DELAY_STEPS = 130


def _leg_arrays():
    """(4, 5) link lengths l1..l4 and paw angle, legs in `LEGS` order."""
    out = []
    for nm in LEGS:
        p = DEFAULT_FORELEG if nm[1] == "F" else DEFAULT_HINDLEG
        out.append([p.l1, p.l2, p.l3, p.l4, p.paw_angle])
    return np.array(out, float)


def _fk_jac(q, L):
    """Planar FK (x, z) and the (x, z) rows of the Jacobian, vectorised:
    `q` (..., 4, 3), `L` (4, 5). Mirrors `LegModel.forward` / `.jacobian`."""
    a1 = q[..., 0]
    a2 = a1 + q[..., 1]
    a3 = a2 + q[..., 2]
    a4 = a3 + L[:, 4]
    l1, l2, l3, l4 = L[:, 0], L[:, 1], L[:, 2], L[:, 3]
    c = [np.cos(a) for a in (a1, a2, a3, a4)]
    s = [np.sin(a) for a in (a1, a2, a3, a4)]
    x = l1 * c[0] + l2 * c[1] + l3 * c[2] + l4 * c[3]
    z = l1 * s[0] + l2 * s[1] + l3 * s[2] + l4 * s[3]
    tx = [-l1 * s[0], -l2 * s[1], -l3 * s[2], -l4 * s[3]]
    tz = [l1 * c[0], l2 * c[1], l3 * c[2], l4 * c[3]]
    Jx = np.stack([tx[0] + tx[1] + tx[2] + tx[3], tx[1] + tx[2] + tx[3], tx[2] + tx[3]], -1)
    Jz = np.stack([tz[0] + tz[1] + tz[2] + tz[3], tz[1] + tz[2] + tz[3], tz[2] + tz[3]], -1)
    return x, z, Jx, Jz


def _quantise(x, bits):
    lsb = 2.0 * np.pi / (2 ** int(bits))
    return np.round(x / lsb) * lsb


class WarpStand:
    """`nworld` standing robots on one GPU. `step(action)` takes the residual
    actions `(nworld, 18)` in [-1, 1] and returns `(obs (nworld, 64), reward,
    terminated, truncated, info)`, resetting finished worlds itself; `info`
    carries the finished worlds' last observations."""

    def __init__(self, nworld: int = 2048, *, randomize: dict | None = None,
                 residual_n: float = 30.0, episode_s: float = 4.0, seed: int = 0,
                 device: str = "cuda:0", envelope: bool = True, friction: bool = True):
        import mujoco
        import warp as wp
        import mujoco_warp as mjw

        self.wp, self.mjw, self.mj = wp, mjw, mujoco
        wp.init()
        self.N = int(nworld)
        self.envelope = bool(envelope)
        self.rng = np.random.default_rng(seed)
        self.randomize = dict(randomize or {})
        self.residual_n = float(residual_n)
        self.device = device

        # the CPU env is the single source of the plant, its names and its
        # settled start; nothing here re-derives them
        cpu = TomcatEnv(rng=np.random.default_rng(0), envelope=envelope, friction=friction)
        cpu.reset()
        self.cpu = cpu
        m = cpu.model
        self.m = m
        self.dt = float(m.opt.timestep)
        self.decim = cpu.decim
        self.control_hz = cpu.control_hz
        self.max_steps = int(round(episode_s * self.control_hz))
        self.qpos0 = np.array(cpu.data.qpos, float)
        self.qvel0 = np.array(cpu.data.qvel, float)
        self.sens0 = np.array(cpu.data.sensordata, float)
        self.a0 = np.array([cpu.a0[p] for p in cpu.pairs])
        self.C = np.asarray(cpu.pair_map, float)
        self.Cinv = np.linalg.inv(self.C)
        self.k_tors = np.array(cpu.k_tors, float)
        self.servo_kp = float(cpu.servo_kp)
        self.legL = _leg_arrays()
        self.fall_deg, self.collapse_m = cpu.fall_deg, cpu.collapse_m

        def adr(name):
            return int(m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)])
        self.i_enc = np.array([adr("enc_" + p) for p in cpu.pairs])
        self.i_encv = np.array([adr("encv_" + p) for p in cpu.pairs])
        self.load_pairs = [p for p in cpu.pairs if ("load_" + p) in cpu._sensor]
        self.i_load = np.array([adr("load_" + p) for p in self.load_pairs])
        self.i_quat = adr("imu_quat")
        self.i_gyro = adr("imu_gyro")
        self.i_acc = adr("imu_acc")
        self.i_touch = np.array([adr("touch_" + nm) for nm in LEGS])
        # where each load reading sits among the 18 pairs (0 where there is none)
        self.load_col = np.array([cpu.pairs.index(p) for p in self.load_pairs])
        self.flat_load = np.array([self.load_pairs.index(k) for k in LOAD_KEYS])

        # the baseline's constants: solved once, on the nominal plant
        self.hold = SB.nominal_hold()
        base = SB.StandBaseline(cpu, hold=self.hold)
        self.gs = base.gs
        self.si = np.array(base.si)
        self.Gp_inv = np.linalg.inv(base.Gp)

        # batched model: the four fields `TomcatEnv._roll` randomises
        self.fields = ("body_mass", "body_inertia", "geom_friction", "jnt_stiffness")
        self.mw = mjw.put_model(m, batch_sizes={f: self.N for f in self.fields})
        self.nom = {f: np.array(getattr(m, f), float) for f in self.fields}
        self.fric_geoms = list(cpu._friction_geoms)
        self.k_mask = self.nom["jnt_stiffness"] > 0.0
        self.dw = mjw.put_data(m, cpu.data, nworld=self.N)

        self._build_kernels()
        nsd = m.nsensordata
        self.L = MAX_DELAY_STEPS + 1
        self.ring = wp.zeros((self.N, self.L, nsd), dtype=float, device=device)
        self.counter = wp.zeros(1, dtype=int, device=device)
        self.delay = wp.zeros(self.N, dtype=int, device=device)
        self.out = wp.zeros((self.N, nsd), dtype=float, device=device)
        self.T = wp.zeros((self.N, 18), dtype=float, device=device)
        kfac = self.k_tors.copy()
        kfac = MT.SPOOL_R / kfac * (self.servo_kp + kfac) / self.servo_kp
        self.kfac = wp.array(kfac.astype(np.float32), dtype=float, device=device)
        self.jr = wp.array(np.array(cpu._jr, np.int32), dtype=int, device=device)
        self.js = wp.array(np.array(cpu._js, np.int32), dtype=int, device=device)
        self.act = wp.array(np.array(cpu._act, np.int32), dtype=int, device=device)
        self.jrd = wp.array(np.array(cpu._jrd, np.int32), dtype=int, device=device)
        # M143: each world's motor (peak N.m, no-load rad/s)
        self.peak_w = wp.array(np.full(self.N, MT.MOTOR_PEAK_NM, np.float32),
                               dtype=float, device=device)
        self.wnl_w = wp.array(np.full(self.N, MT.MOTOR_NOLOAD_RAD_S, np.float32),
                              dtype=float, device=device)
        # M146 (ADR-0140): each pair's conduit friction, set every control step
        self.friction = bool(cpu.friction)
        self.cap_w = wp.array(np.ones((self.N, 18), np.float32), dtype=float, device=device)
        self.leg_qadr = np.array([cpu._qadr(nm) for nm in LEGS])          # (4, 3)
        self.sens0_w = wp.array(self.sens0.astype(np.float32), dtype=float, device=device)

        # per-world state the host keeps
        self.steps = np.zeros(self.N, int)
        self.draw = {}
        self._capture()
        self.reset()

    # -- device ---------------------------------------------------------------
    def _build_kernels(self):
        wp = self.wp

        @wp.kernel
        def servo(qpos: wp.array2d(dtype=float), qvel: wp.array2d(dtype=float),
                  T: wp.array2d(dtype=float),
                  jr: wp.array(dtype=int), js: wp.array(dtype=int),
                  jrd: wp.array(dtype=int),
                  act: wp.array(dtype=int), kfac: wp.array(dtype=float),
                  kp: float, kv: float, peak_w: wp.array(dtype=float),
                  wnl_w: wp.array(dtype=float), env_on: int,
                  cap: wp.array2d(dtype=float),
                  ctrl: wp.array2d(dtype=float)):
            w, i = wp.tid()
            th = qpos[w, jr[i]]
            # M146: the joint gets the motor's tension over the conduit's friction
            cmd = th + qpos[w, js[i]] + T[w, i] / cap[w, i] * kfac[i]
            if env_on != 0:
                # `wbc.envelope_command`: the spec motor's torque-speed line
                om = qvel[w, jrd[i]]
                peak = peak_w[w] / cap[w, i]
                wnl = wnl_w[w]
                f = kp * (cmd - th) - kv * om
                hi = peak * wp.clamp(1.0 - wp.max(om, 0.0) / wnl, 0.0, 1.0)
                lo = -peak * wp.clamp(1.0 - wp.max(-om, 0.0) / wnl, 0.0, 1.0)
                f = wp.clamp(f, lo, hi)
                cmd = th + (f + kv * om) / kp
            ctrl[w, act[i]] = cmd

        @wp.kernel
        def push(sd: wp.array2d(dtype=float), ring: wp.array3d(dtype=float),
                 counter: wp.array(dtype=int), L: int):
            w, k = wp.tid()
            ring[w, counter[0] % L, k] = sd[w, k]

        @wp.kernel
        def tick(counter: wp.array(dtype=int)):
            counter[0] = counter[0] + 1

        @wp.kernel
        def gather(ring: wp.array3d(dtype=float), counter: wp.array(dtype=int),
                   delay: wp.array(dtype=int), L: int, out: wp.array2d(dtype=float)):
            w, k = wp.tid()
            # the last push went to (counter - 1); read `delay` pushes before it
            out[w, k] = ring[w, (counter[0] - 1 - delay[w] + 2 * L) % L, k]

        @wp.kernel
        def refill(mask: wp.array(dtype=int), s0: wp.array(dtype=float),
                   ring: wp.array3d(dtype=float)):
            w, j, k = wp.tid()
            if mask[w] != 0:
                ring[w, j, k] = s0[k]

        self.k_servo, self.k_push, self.k_tick = servo, push, tick
        self.k_gather, self.k_refill = gather, refill

    def _period(self):
        wp, mjw = self.wp, self.mjw
        nsd = self.m.nsensordata
        for _ in range(self.decim):
            wp.launch(self.k_servo, dim=(self.N, 18),
                      inputs=[self.dw.qpos, self.dw.qvel, self.T, self.jr, self.js,
                              self.jrd, self.act, self.kfac,
                              float(self.servo_kp), float(self.cpu.servo_kv),
                              self.peak_w, self.wnl_w,
                              int(self.envelope), self.cap_w, self.dw.ctrl])
            mjw.step(self.mw, self.dw)
            wp.launch(self.k_push, dim=(self.N, nsd),
                      inputs=[self.dw.sensordata, self.ring, self.counter, self.L])
            wp.launch(self.k_tick, dim=1, inputs=[self.counter])

    def _capture(self):
        wp = self.wp
        self._period()                     # compile everything outside the graph
        wp.synchronize()
        with wp.ScopedCapture() as cap:
            self._period()
        self.graph = cap.graph

    def _read(self):
        wp = self.wp
        wp.launch(self.k_gather, dim=(self.N, self.m.nsensordata),
                  inputs=[self.ring, self.counter, self.delay, self.L, self.out])
        return self.out.numpy().astype(float)

    # -- randomisation and reset ---------------------------------------------
    def _roll(self, idx):
        """Draw the plant and sensors for worlds `idx` (host arrays, then the
        batched model fields)."""
        r, R, n = self.rng, self.randomize, len(idx)
        d = self.draw
        if not d:
            d.update(peak=np.full(self.N, MT.MOTOR_PEAK_NM),
                     wnl=np.full(self.N, MT.MOTOR_NOLOAD_RAD_S),
                     mass=np.ones(self.N), mu=np.full(self.N, np.nan),
                     k=np.ones(self.N),
                     delay=np.full(self.N, int(round(self.cpu.latency_s / self.dt))),
                     enc_bias=np.zeros((self.N, 18)), enc_noise=np.zeros(self.N),
                     load_gain=np.ones(self.N), imu_tilt=np.zeros(self.N),
                     gyro_noise=np.zeros(self.N))
        u = lambda key: r.uniform(*R[key], n)          # noqa: E731
        if "mass_scale" in R:
            d["mass"][idx] = u("mass_scale")
        if "floor_mu" in R:
            d["mu"][idx] = u("floor_mu")
        if "series_k_scale" in R:
            d["k"][idx] = u("series_k_scale")
        if "latency_s" in R:
            d["delay"][idx] = np.round(u("latency_s") / self.dt).astype(int)
        if "motor_peak_nm" in R:
            d["peak"][idx] = u("motor_peak_nm")
        if "motor_noload_rpm" in R:
            d["wnl"][idx] = u("motor_noload_rpm") * 2.0 * math.pi / 60.0
        if "enc_offset_rad" in R:
            s = u("enc_offset_rad")
            d["enc_bias"][idx] = r.normal(0.0, 1.0, (n, 18)) * s[:, None]
        for key, name in (("enc_noise_rad", "enc_noise"), ("load_scale", "load_gain"),
                          ("imu_tilt_deg", "imu_tilt"), ("gyro_noise", "gyro_noise")):
            if key in R:
                v = u(key)
                d[name][idx] = np.radians(v) if key == "imu_tilt_deg" else v
        d["delay"] = np.clip(d["delay"], 0, MAX_DELAY_STEPS)

    def _push_model(self):
        wp, d = self.wp, self.draw
        mass = self.nom["body_mass"][None, :] * d["mass"][:, None]
        inert = self.nom["body_inertia"][None, :, :] * d["mass"][:, None, None]
        fric = np.repeat(self.nom["geom_friction"][None], self.N, 0)
        has = ~np.isnan(d["mu"])
        for g in self.fric_geoms:
            fric[has, g, 0] = d["mu"][has]
        stiff = np.repeat(self.nom["jnt_stiffness"][None], self.N, 0)
        stiff[:, self.k_mask] *= d["k"][:, None]
        getattr(self.mw, "body_mass").assign(mass.astype(np.float32))
        getattr(self.mw, "body_inertia").assign(inert.astype(np.float32))
        getattr(self.mw, "geom_friction").assign(fric.astype(np.float32))
        getattr(self.mw, "jnt_stiffness").assign(stiff.astype(np.float32))
        self.delay.assign(d["delay"].astype(np.int32))
        self.peak_w.assign(d["peak"].astype(np.float32))
        self.wnl_w.assign(d["wnl"].astype(np.float32))

    def _reset_worlds(self, mask):
        wp, mjw = self.wp, self.mjw
        idx = np.nonzero(mask)[0]
        if len(idx) == 0:
            return
        self._roll(idx)
        self._push_model()
        m32 = wp.array(mask.astype(np.int32), dtype=int, device=self.device)
        mjw.reset_data(self.mw, self.dw, reset=m32)
        qpos = self.dw.qpos.numpy()
        qvel = self.dw.qvel.numpy()
        qpos[idx] = self.qpos0
        qvel[idx] = self.qvel0
        self.dw.qpos.assign(qpos.astype(np.float32))
        self.dw.qvel.assign(qvel.astype(np.float32))
        wp.launch(self.k_refill, dim=(self.N, self.L, self.m.nsensordata),
                  inputs=[m32, self.sens0_w, self.ring])
        self.steps[idx] = 0

    def reset(self):
        self._reset_worlds(np.ones(self.N, bool))
        obs = self._observe(self._read())
        self._base_reset(obs, np.ones(self.N, bool))
        self._obs = obs
        return self.flatten(obs)

    # -- the board ------------------------------------------------------------
    def _observe(self, s):
        d, r = self.draw, self.rng
        enc = s[:, self.i_enc] + d["enc_bias"]
        if np.any(d["enc_noise"]):
            enc = enc + r.normal(0.0, 1.0, enc.shape) * d["enc_noise"][:, None]
        enc = _quantise(enc, self.cpu.encoder_bits)
        load = s[:, self.i_load] * d["load_gain"][:, None]
        q = s[:, self.i_quat:self.i_quat + 4]
        t = d["imu_tilt"]
        if np.any(t):
            c, sn = np.cos(0.5 * t), np.sin(0.5 * t)
            w, x, y, z = q.T
            q = np.stack([c * w - sn * x, c * x + sn * w,
                          c * y - sn * z, c * z + sn * y], 1)
        gyro = s[:, self.i_gyro:self.i_gyro + 3]
        if np.any(d["gyro_noise"]):
            gyro = gyro + r.normal(0.0, 1.0, gyro.shape) * d["gyro_noise"][:, None]
        return {"rotor": enc, "rotor_vel": s[:, self.i_encv], "load": load,
                "imu_quat": q, "imu_gyro": gyro,
                "imu_acc": s[:, self.i_acc:self.i_acc + 3],
                "contact": s[:, self.i_touch]}

    def flatten(self, o):
        """`gym_env.flatten`, over worlds: 18 + 18 + 14 + 4 + 3 + 3 + 4 = 64."""
        return np.concatenate([
            o["rotor"], o["rotor_vel"] * 0.01, o["load"][:, self.flat_load] * 100.0,
            o["imu_quat"], o["imu_gyro"] * 0.1, o["imu_acc"] * 0.05,
            np.clip(o["contact"], 0.0, 50.0) * 0.02], 1).astype(np.float32)

    def _spool18(self, o):
        th = np.zeros((self.N, 18))
        th[:, self.load_col] = o["load"]
        return th

    def joint_estimate(self, o):
        """`TomcatEnv.joint_estimate` over worlds: (N, 4, 3)."""
        th = o["rotor"][:, :12] + self._spool18(o)[:, :12]
        Lc = self.a0[None, :12] - MT.SPOOL_R * th
        return (Lc.reshape(self.N, 4, 3) @ self.Cinv.T)

    def spine_angle(self, o):
        th = o["rotor"][:, self.si] + self._spool18(o)[:, self.si]
        return -MT.SPOOL_R * th / self.gs

    @staticmethod
    def tilt(o):
        return 2.0 * np.degrees(np.arccos(np.minimum(1.0, np.abs(o["imu_quat"][:, 0]))))

    def stance_height(self, o, q=None):
        q = self.joint_estimate(o) if q is None else q
        x, z, _, _ = _fk_jac(q, self.legL)
        w, qx, qy, qz = o["imu_quat"].T
        r20 = 2.0 * (qx * qz - w * qy)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
        drop = -(r20[:, None] * x + r22[:, None] * z)
        return drop.min(1)

    def reward(self, o, T, q):
        upright = np.cos(np.radians(np.minimum(180.0, self.tilt(o))))
        feet = np.count_nonzero(o["contact"] > 1e-6, 1) / 4.0
        spin = np.linalg.norm(o["imu_gyro"], axis=1)
        cost = np.mean(np.abs(T), 1) / MT.TENSION_MAX
        tall = np.minimum(1.0, self.stance_height(o, q) / 0.17)
        return (upright + 0.5 * feet + tall - 0.05 * np.minimum(spin, 20.0) - 0.2 * cost)

    # -- the baseline (`StandBaseline`, over worlds) ---------------------------
    @staticmethod
    def _roll_pitch(qt):
        w, x, y, z = qt.T
        roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
        return roll, pitch

    def _base_reset(self, o, mask):
        q = self.joint_estimate(o)
        s = self.spine_angle(o)
        rp = np.stack(self._roll_pitch(o["imu_quat"]), 1)
        if not hasattr(self, "b_q0"):
            self.b_q0, self.b_qp = q.copy(), q.copy()
            self.b_s0, self.b_sp = s.copy(), s.copy()
            self.b_att0 = rp.copy()
            return
        for a, v in ((self.b_q0, q), (self.b_qp, q), (self.b_s0, s), (self.b_sp, s),
                     (self.b_att0, rp)):
            a[mask] = v[mask]

    def baseline(self, o):
        q = self.joint_estimate(o)
        s = self.spine_angle(o)
        roll, pitch = self._roll_pitch(o["imu_quat"])
        droll = roll - self.b_att0[:, 0]
        dpitch = pitch - self.b_att0[:, 1]
        left = np.array([1.0 if nm[0] == "L" else -1.0 for nm in LEGS])
        fore = np.array([1.0 if nm[1] == "F" else -1.0 for nm in LEGS])
        fz = (left[None] * (SB.K_ROLL * droll + SB.K_ROLL_RATE * o["imu_gyro"][:, 0])[:, None]
              - fore[None] * (SB.K_PITCH * dpitch)[:, None])
        _x, _z, _Jx, Jz = _fk_jac(q, self.legL)
        dt = 1.0 / self.control_hz
        tau = (SB.KP_LEG * (self.b_q0 - q) - SB.KD_LEG * (q - self.b_qp) / dt
               + Jz * fz[..., None])
        T = np.repeat(self.hold[None], self.N, 0)
        T[:, :12] += (tau @ self.Gp_inv.T).reshape(self.N, 12)
        T[:, 12:] = (SB.KP_SPINE * (self.b_s0 - s)
                     - SB.KD_SPINE * (s - self.b_sp) / dt) / (-self.gs)
        self.b_qp, self.b_sp = q, s
        if self.friction:
            # M146: as `StandBaseline` -- ask the motor for the joint's need times
            # the conduit's friction at the estimated pose
            from .tendon import drive_capstan
            T[:, :12] *= drive_capstan(q, T[:, :12].reshape(self.N, 4, 3)).reshape(self.N, 12)
        return T

    # -- the loop -------------------------------------------------------------
    def step(self, action):
        wp = self.wp
        a = np.clip(np.asarray(action, float), -1.0, 1.0)
        T = np.clip(self.baseline(self._obs) + a * self.residual_n,
                    -MT.TENSION_MAX, MT.TENSION_MAX)
        self.T.assign(T.astype(np.float32))
        if self.friction:
            from .tendon import drive_capstan
            qp = self.dw.qpos.numpy()
            cap = np.ones((self.N, 18))
            cap[:, :12] = drive_capstan(qp[:, self.leg_qadr], T[:, :12].reshape(self.N, 4, 3)
                                        ).reshape(self.N, 12)
            self.cap_w.assign(cap.astype(np.float32))
        wp.capture_launch(self.graph)
        o = self._observe(self._read())
        self.steps += 1
        q = self.joint_estimate(o)
        rew = self.reward(o, T, q)
        h = self.stance_height(o, q)
        term = (self.tilt(o) > self.fall_deg) | (h < self.collapse_m)
        trunc = (self.steps >= self.max_steps) & ~term
        done = term | trunc
        info = {"tilt_deg": self.tilt(o)}
        if np.any(done):
            info["terminal_obs"] = self.flatten(o)[done]
            info["done_idx"] = np.nonzero(done)[0]
            self._reset_worlds(done)
            fresh = self._observe(self._read())
            for k in o:
                o[k][done] = fresh[k][done]
            self._base_reset(o, done)
        self._obs = o
        return self.flatten(o), rew.astype(np.float32), term, trunc, info


def _vec(nworld, cls, nobs, **kw):
    """Any `WarpStand`-shaped env behind stable-baselines3's `VecEnv`."""
    from stable_baselines3.common.vec_env.base_vec_env import VecEnv
    from gymnasium import spaces

    class _Vec(VecEnv):
        def __init__(self):
            self.core = cls(nworld, **kw)
            super().__init__(nworld, spaces.Box(-np.inf, np.inf, (nobs,), np.float32),
                             spaces.Box(-1.0, 1.0, (18,), np.float32))
            self._a = None

        def reset(self):
            return self.core.reset()

        def step_async(self, actions):
            self._a = actions

        def step_wait(self):
            obs, rew, term, trunc, info = self.core.step(self._a)
            done = term | trunc
            infos = [{} for _ in range(self.num_envs)]
            if "done_idx" in info:
                for j, i in enumerate(info["done_idx"]):
                    infos[i]["terminal_observation"] = info["terminal_obs"][j]
                    infos[i]["TimeLimit.truncated"] = bool(trunc[i])
            return obs, rew, done, infos

        def close(self):
            pass

        def get_attr(self, name, indices=None):
            return [None] * self.num_envs

        def set_attr(self, name, value, indices=None):
            pass

        def env_method(self, name, *a, indices=None, **k):
            return [None] * self.num_envs

        def env_is_wrapped(self, wrapper_class, indices=None):
            return [False] * self.num_envs

        def seed(self, seed=None):
            return [None] * self.num_envs

    return _Vec()


def make_vec_env(nworld=2048, **kw):
    """`WarpStand` behind stable-baselines3's `VecEnv`."""
    return _vec(nworld, WarpStand, 64, **kw)


class WarpWalk(WarpStand):
    """Walk at a commanded forward speed, as a residual on the standing
    baseline (M137, ADR-0131).

    The policy sees the standing task's 64 channels plus a gait clock
    (`sin`, `cos` of a `period` phase) and the commanded speed `v_cmd`, drawn
    per episode from `v_range`. It adds +-`residual_n` N to `StandBaseline`,
    which holds the robot up and leaves the stepping to the policy.

    ⚠️ **The reward reads the TRUE forward speed** (`qvel`), which the robot
    cannot measure. ADR-0082 kept every reward term observable so a policy
    could be fine-tuned on hardware; this is a deliberate exception for
    training in simulation. The policy itself sees only the board.
    """

    def __init__(self, nworld=2048, *, v_range=(0.0, 0.5), period=0.4,
                 residual_n=80.0, gait_w: float = 0.0, slip_w: float = 0.0,
                 clear_w: float = 0.0, clear_m: float = 0.03,
                 act_obs: bool = False, cu_w: float = 0.0, **kw):
        """`gait_w` (M138, ADR-0132): weight of the trot term -- each foot
        down when the clock says its diagonal pair is in stance (LF+RR the
        first half of the period, RF+LR the second), up when it is not.

        `act_obs` (M144, ADR-0138): append the policy's own last action (18)
        to the observation, so with a stack of frames it can see what it
        asked for against what happened -- and tell which motor it is on."""
        self.act_obs = bool(act_obs)
        self.last_a = None
        # M146 (ADR-0140): a COPPER-shaped cost, (motor tension / its max)^2
        # averaged over the pairs -- what heats the motors, where the standing
        # task's 0.2 mean|T| was too weak to stop a 206 N pull
        self.cu_w = float(cu_w)
        self.v_range = tuple(v_range)
        self.period = float(period)
        self.gait_w = float(gait_w)
        # M139 (ADR-0133): M138's walk SKATED -- loaded feet sliding at a
        # median 0.04-0.13 m/s, swing feet 4 mm off the floor. Both terms read
        # the simulator's foot sites (training-only, like the speed).
        self.slip_w, self.clear_w, self.clear_m = float(slip_w), float(clear_w), float(clear_m)
        self._foot_prev = None
        self.vcmd = None
        super().__init__(nworld, residual_n=residual_n, **kw)

    def _reset_worlds(self, mask):
        super()._reset_worlds(mask)
        self._fresh = np.asarray(mask, bool).copy()
        if self.last_a is None:
            self.last_a = np.zeros((self.N, 18))
        self.last_a[np.asarray(mask, bool)] = 0.0
        if self.vcmd is None:
            self.vcmd = np.zeros(self.N)
        idx = np.nonzero(mask)[0]
        self.vcmd[idx] = self.rng.uniform(*self.v_range, len(idx))

    def clock(self):
        """Gait phase in [0, 1), per world."""
        return (self.steps / self.control_hz) / self.period % 1.0

    def trot_match(self, o):
        """Fraction of the four feet whose contact agrees with the trot the
        clock asks for, per world."""
        first = self.clock() < 0.5
        want = np.stack([first, ~first, ~first, first], 1)      # LF RF LR RR
        down = o["contact"] > 1.0
        return np.mean(down == want, 1)

    def flatten(self, o):
        ph = 2.0 * np.pi * self.clock()
        extra = np.stack([np.sin(ph), np.cos(ph), self.vcmd], 1).astype(np.float32)
        parts = [super().flatten(o), extra]
        if self.act_obs:
            parts.append(self.last_a.astype(np.float32))
        return np.concatenate(parts, 1)

    def step(self, action):
        # the action taken now is the "last action" the next observation shows
        if self.last_a is None:
            self.last_a = np.zeros((self.N, 18))
        self.last_a = np.clip(np.asarray(action, float), -1.0, 1.0)
        return super().step(action)

    def reward(self, o, T, q):
        vx = self.dw.qvel.numpy()[:, 0].astype(float)
        self.vx = vx
        track = np.exp(-((vx - self.vcmd) / 0.15) ** 2)
        upright = np.cos(np.radians(np.minimum(180.0, self.tilt(o))))
        tall = np.minimum(1.0, self.stance_height(o, q) / 0.17)
        spin = np.linalg.norm(o["imu_gyro"], axis=1)
        cost = np.mean(np.abs(T), 1) / MT.TENSION_MAX
        gait = self.trot_match(o) if self.gait_w else 0.0
        slip, clear = self.feet_terms(o)
        self.last_feet = (slip, clear)          # computed once a step; read, do not recompute
        copper = np.mean((T / MT.TENSION_MAX) ** 2, 1)
        return (2.0 * track + 0.5 * upright + 0.5 * tall + self.gait_w * gait
                - self.slip_w * slip + self.clear_w * clear
                - 0.05 * np.minimum(spin, 20.0) - 0.2 * cost - self.cu_w * copper)

    def foot_pos(self):
        """(N, 4, 3) world positions of the four foot sites."""
        if not hasattr(self, "_foot_sites"):
            mj = self.mj
            self._foot_sites = [mj.mj_name2id(self.m, mj.mjtObj.mjOBJ_SITE, nm + "_foot")
                                for nm in LEGS]
        return self.dw.site_xpos.numpy()[:, self._foot_sites, :].astype(float)

    def feet_terms(self, o):
        """(slip m/s, clearance 0..1) per world: the mean horizontal speed of
        the feet carrying load, and how high the feet the clock has in swing
        are, as a fraction of `clear_m` above the pad's rest height."""
        p = self.foot_pos()
        prev = p if self._foot_prev is None else self._foot_prev
        v = np.linalg.norm((p - prev)[..., :2], axis=-1) * self.control_hz
        fresh = getattr(self, "_fresh", None)
        if fresh is not None:
            v[fresh] = 0.0                       # a reset moved the feet, not a slip
            self._fresh = None
        self._foot_prev = p
        loaded = o["contact"] > 5.0
        slip = np.sum(v * loaded, 1) / np.maximum(loaded.sum(1), 1)
        first = self.clock() < 0.5
        swing = np.stack([~first, first, first, ~first], 1)      # LF RF LR RR
        lift = np.clip((p[..., 2] - 0.006) / self.clear_m, 0.0, 1.0)
        clear = np.sum(lift * swing, 1) / 2.0
        return slip, clear


def make_walk_vec_env(nworld=2048, **kw):
    """`WarpWalk` behind SB3's `VecEnv` (67 observations, 85 with
    `act_obs`; stack frames with SB3's `VecFrameStack` for a history)."""
    return _vec(nworld, WarpWalk, 85 if kw.get("act_obs") else 67, **kw)
