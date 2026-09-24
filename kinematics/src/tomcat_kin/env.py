# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Younghyeon Park
"""The environment an agent trains in — and it can only see what the board sees.

⚠️ **Everything in this project until M76 read `d.qpos` out of the simulator.**
`electronics/BOARD_OUTLINE.md` puts no joint encoder on this robot (ADR-0077), so
that state does not exist on hardware. This wraps the plant so it *cannot* be
read: the observation is assembled from `d.sensordata` and nothing else.

What the board gives, and therefore what an agent gets:

* 18 rotor absolute encoders, position and velocity — quantised to 14 bits
* 14 tendon load cells — spine + hip/knee only, **DNP on the ankle** (ADR-0004)
* one trunk IMU — orientation, gyro, accelerometer
* four foot contact channels (FR12)

and three things ADR-0078 measured that were never in any loop here:

* a **control rate** — 133 Hz by default, not the 10 kHz the physics runs at
* a **transport delay** — NFR12's 7.5 ms, applied to the sensors
* the **cascade** — the action is a *tension*, and the rotor servo closes on its
  own encoder every physics step, not at the policy rate
"""

from __future__ import annotations

import math

import numpy as np

from . import mjcf_tendon as MT
from . import wbc
from .params import DEFAULT_FORELEG, DEFAULT_HINDLEG, DEFAULT_SPINE, DEFAULT_TENDON

LEGS = ("LF", "RF", "LR", "RR")
PAIRS = ("hip", "knee", "ankle")
SPINE = ("spine_p1", "spine_y1", "spine_p2", "spine_y2", "spine_p3", "spine_y3")

#: ADR-0004 wants an AS5047/MA-class absolute encoder. 14 bits over a turn.
ENCODER_BITS = 14
#: NFR12's balance pipeline: contact 1.0 + estimation 5.0 + transport 1.0 +
#: compute 0.5. ADR-0078 measured it costs 2 N at 1 kHz on force allocation.
NFR12_LATENCY_S = 7.5e-3
#: NFR12's 7.5 ms pipeline implies roughly this for the balance loop.
BALANCE_HZ = 133.0
#: G3 rates, leg and spine cables -- ONE source, `params.py` (M112, ADR-0105).
SERIES_K = DEFAULT_TENDON.series_k
SPINE_SERIES_K = DEFAULT_SPINE.series_k


def stance_pose(foot_x: float = 0.04, foot_z: float = -0.17) -> dict:
    """The pose every standing figure in this project is measured at."""
    from . import LegModel
    lp = {"LF": DEFAULT_FORELEG, "RF": DEFAULT_FORELEG,
          "LR": DEFAULT_HINDLEG, "RR": DEFAULT_HINDLEG}
    return {nm: np.asarray(LegModel(lp[nm]).inverse((foot_x, foot_z, 0.0)),
                           float) for nm in LEGS}


class TomcatEnv:
    """The plant, behind the board's sensors.

    `step(tension)` takes an 18-vector of cable tensions (N), one per
    antagonistic pair, holds it for one control period, and returns what the
    sensors report — delayed, quantised, and with no joint angle in it.
    """

    def __init__(self, *, control_hz: float = BALANCE_HZ,
                 latency_s: float = NFR12_LATENCY_S,
                 encoder_bits: int = ENCODER_BITS,
                 spine: bool = True, series_k: float = SERIES_K,
                 hip_height: float = 0.176, fall_deg: float = 45.0,
                 collapse_m: float = 0.12,
                 jitter_deg: float = 0.0, rng=None,
                 randomize: dict | None = None):
        import mujoco

        self.mj = mujoco
        self.q_ref = stance_pose()
        xml = MT.quadruped_rig_spooled(
            q_ref={nm: list(v) for nm, v in self.q_ref.items()},
            series_k=series_k, hip_height=hip_height, spine=spine,
            spool_servo=True, sensors=True)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.series_k = float(series_k)
        self.servo_kp = MT.ROTOR_ARMATURE * MT.ROTOR_BANDWIDTH ** 2
        self.encoder_bits = int(encoder_bits)
        self.latency_s = float(latency_s)
        self.dt = float(self.model.opt.timestep)
        self.decim = max(1, int(round(1.0 / (float(control_hz) * self.dt))))
        self.control_hz = 1.0 / (self.decim * self.dt)
        self.pairs = [f"{nm}_{p}" for nm in LEGS for p in PAIRS] + list(SPINE)
        # ⚠️ M112: one spring rate per pair -- the spine's G3 is not the legs'.
        self.k_tors = np.array(
            [(SPINE_SERIES_K if p in SPINE else self.series_k) * MT.SPOOL_R ** 2
             for p in self.pairs], dtype=float)

        self._sensor = {}
        for i in range(self.model.nsensor):
            n = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SENSOR, i)
            self._sensor[n] = (int(self.model.sensor_adr[i]),
                               int(self.model.sensor_dim[i]))
        self._act = [self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, "m_" + p)
                     for p in self.pairs]
        self._jr = [self.model.jnt_qposadr[self._id(mujoco.mjtObj.mjOBJ_JOINT,
                                                    "jr_" + p)]
                    for p in self.pairs]
        self._js = [self.model.jnt_qposadr[self._id(mujoco.mjtObj.mjOBJ_JOINT,
                                                    "js_" + p)]
                    for p in self.pairs]
        self.randomize = dict(randomize or {})
        self.drawn = {}
        self._enc_bias = np.zeros(18)
        self._load_gain = 1.0
        self._imu_tilt = 0.0
        self.fall_deg = float(fall_deg)
        self.collapse_m = float(collapse_m)
        self.jitter_deg = float(jitter_deg)
        self.rng = np.random.default_rng() if rng is None else rng
        self.steps = 0
        self.pair_map = MT.pair_matrix(
            [float(a) for a in DEFAULT_TENDON.joint_moment_arm])
        self._lag = None
        self.reset()

    # -- plumbing ---------------------------------------------------------
    def _id(self, objtype, name):
        i = self.mj.mj_name2id(self.model, objtype, name)
        if i < 0:
            raise KeyError(name)
        return i

    def _raw(self, name):
        adr, dim = self._sensor[name]
        return np.array(self.data.sensordata[adr:adr + dim], dtype=float)

    # -- gym-ish surface --------------------------------------------------
    def _settle(self, seconds=0.30, kp=30.0, kd=1.5):
        """Put the robot ON ITS FEET before the episode starts.

        ⚠️ **M82: every episode used to begin mid-fall.** `hip_height` puts
        the paws **6 mm in the air**, so at `reset` there were 2 contacts
        carrying **0.66 N** against a 42.26 N weight -- 2 % -- and the policy had
        to survive a landing before it could try to stand. Held by a joint PD for
        0.30 s instead: **4 contacts, 44.94 N (106 %), trunk quiet at
        0.0024 m/s**.

        ✅ This is a FIXTURE, not a controller. It uses `d.qpos` freely, which
        the robot cannot ([ADR-0081](../../../docs/DESIGN_DECISIONS.md)) -- but a
        real robot is stood up by hand before a run, and nothing here survives
        into the episode. The policy still only ever sees sensors.
        """
        from . import wbc as _w
        qa = {nm: self._qadr(nm) for nm in LEGS}
        dof = {nm: self._dofadr(nm) for nm in LEGS}
        q0 = {nm: np.array([float(self.data.qpos[a]) for a in qa[nm]])
              for nm in LEGS}
        G = {nm: self._leg_gain(nm, qa[nm]) for nm in LEGS}
        for _ in range(int(seconds / self.dt)):
            for nm in LEGS:
                e = q0[nm] - np.array([float(self.data.qpos[a]) for a in qa[nm]])
                ev = -np.array([float(self.data.qvel[a]) for a in dof[nm]])
                bias = np.array([float(self.data.qfrc_bias[a]
                                       - self.data.qfrc_passive[a])
                                 for a in dof[nm]])
                T = _w.pair_command(G[nm], bias + kp * e + kd * ev,
                                    MT.TENSION_MAX)
                idx = [self.pairs.index(f"{nm}_{p}") for p in PAIRS]
                cmd = _w.rotor_command(self.data.qpos[[self._jr[i] for i in idx]],
                                       self.data.qpos[[self._js[i] for i in idx]],
                                       T, self.k_tors[idx], MT.SPOOL_R,
                                       servo_kp=self.servo_kp)
                for k, i in enumerate(idx):
                    self.data.ctrl[self._act[i]] = float(cmd[k])
            self.mj.mj_step(self.model, self.data)
        self.data.ctrl[:] = 0.0

    def _qadr(self, nm):
        return [self.model.jnt_qposadr[self._id(self.mj.mjtObj.mjOBJ_JOINT,
                                                f"{nm}_q{i}")] for i in (1, 2, 3)]

    def _dofadr(self, nm):
        return [self.model.jnt_dofadr[self._id(self.mj.mjtObj.mjOBJ_JOINT,
                                               f"{nm}_q{i}")] for i in (1, 2, 3)]

    def _leg_gain(self, nm, qa):
        tid = [self._id(self.mj.mjtObj.mjOBJ_TENDON, f"{nm}_{p}") for p in PAIRS]
        J = np.zeros((3, 3))
        base = [float(self.data.qpos[a]) for a in qa]
        for k in range(3):
            L = []
            for sg in (+1, -1):
                dd = self.mj.MjData(self.model)
                dd.qpos[:] = self.data.qpos
                dd.qpos[qa[k]] = base[k] + sg * 0.002
                self.mj.mj_forward(self.model, dd)
                L.append(np.array([dd.ten_length[t] for t in tid]))
            J[:, k] = (L[0] - L[1]) / 0.004
        return (-J).T

    def reset(self, settle: bool = True):
        """Stance pose, ON ITS FEET, cleared delay buffer, first observation."""
        self.mj.mj_resetData(self.model, self.data)
        for nm in LEGS:
            for i, jn in enumerate((f"{nm}_q1", f"{nm}_q2", f"{nm}_q3")):
                a = self.model.jnt_qposadr[
                    self._id(self.mj.mjtObj.mjOBJ_JOINT, jn)]
                jit = math.radians(self.jitter_deg) * (
                    self.rng.standard_normal() if self.jitter_deg else 0.0)
                self.data.qpos[a] = self.q_ref[nm][i] + jit
        self._nominal()
        self._roll()
        self.mj.mj_forward(self.model, self.data)
        self.steps = 0
        self.a0 = {}
        for p in self.pairs:
            t = self._id(self.mj.mjtObj.mjOBJ_TENDON, p)
            self.a0[p] = float(self.data.ten_length[t])
        if settle:
            self._settle()
        self._lag = wbc.SensorDelay(self.latency_s, self.dt,
                                    self.data.sensordata,
                                    np.zeros_like(self.data.sensordata))
        self.t = 0.0
        self.steps = 0
        return self.observe()

    def step(self, tension):
        """Hold `tension` (N, one per pair) for one control period.

        Returns `(obs, reward, terminated, info)`. ⚠️ **No constant action
        stands this robot** -- the gravity-compensating tension at the stance
        pose reaches 38.7° of tilt and the trunk collapses to 28 mm. Standing
        is an unstable equilibrium: a fixed tension is a fixed torque, and the
        torque the pose needs changes as it tips. Feedback is the task.
        """
        tension = np.clip(np.asarray(tension, float), -MT.TENSION_MAX,
                          MT.TENSION_MAX)
        for _ in range(self.decim):
            # ✅ the INNER loop: the motor closes on its own shaft encoder every
            # physics step. ADR-0078 -- decimating this with the outer loop
            # starves the servo of its own feedback and is not the robot.
            cmd = wbc.rotor_command(self.data.qpos[self._jr],
                                    self.data.qpos[self._js], tension,
                                    self.k_tors, MT.SPOOL_R,
                                    servo_kp=self.servo_kp)
            self.data.ctrl[self._act] = cmd
            self.mj.mj_step(self.model, self.data)
            self._lag.push(self.data.sensordata,
                           np.zeros_like(self.data.sensordata))
            self.t += self.dt
        if not np.all(np.isfinite(self.data.qpos)):
            raise FloatingPointError("plant diverged")
        obs = self.observe()
        self.steps += 1
        return obs, self.reward(obs, tension), self.terminated(obs), {
            "tilt_deg": self._tilt(obs), "t": self.t, "steps": self.steps}

    def observe(self) -> dict:
        """What the board reports, delayed and quantised. No joint angle."""
        s, _ = self._lag.read()
        s = np.asarray(s, float)

        def take(name):
            adr, dim = self._sensor[name]
            return s[adr:adr + dim]

        enc = np.array([take("enc_" + p)[0] for p in self.pairs])
        encv = np.array([take("encv_" + p)[0] for p in self.pairs])
        enc = enc + getattr(self, "_enc_bias", 0.0)
        n = self.drawn.get("enc_noise_rad", 0.0) if hasattr(self, "drawn") else 0.0
        if n:
            enc = enc + self.rng.normal(0.0, n, enc.shape)
        if self.encoder_bits:
            enc = wbc.quantise(enc, self.encoder_bits)
        gain = getattr(self, "_load_gain", 1.0)
        load = {p: float(take("load_" + p)[0]) * gain
                for p in self.pairs if ("load_" + p) in self._sensor}
        quat = np.array(take("imu_quat"), float)
        tilt = getattr(self, "_imu_tilt", 0.0)
        if tilt:
            # a mounting misalignment: rotate the reported frame about x
            c, sn = math.cos(0.5 * tilt), math.sin(0.5 * tilt)
            w, x, y, z = quat
            quat = np.array([c * w - sn * x, c * x + sn * w,
                             c * y - sn * z, c * z + sn * y])
        gyro = np.array(take("imu_gyro"), float)
        gn = self.drawn.get("gyro_noise", 0.0) if hasattr(self, "drawn") else 0.0
        if gn:
            gyro = gyro + self.rng.normal(0.0, gn, 3)
        return {
            "rotor": enc, "rotor_vel": encv, "load": load,
            "imu_quat": quat, "imu_gyro": gyro,
            "imu_acc": take("imu_acc"),
            "contact": np.array([take("touch_" + nm)[0] for nm in LEGS]),
            "t": self.t,
        }

    # -- what a controller has to do for itself ---------------------------
    def joint_estimate(self, obs) -> dict:
        """Reconstruct joint angle from the observation (ADR-0077).

        ⚠️ The ankle has **no load cell**, so its spring deflection is unknown
        and taken as zero. ADR-0077 priced that at 1.09° rising to 6.08° at the
        peak rating -- the error is in here, deliberately, because it is in the
        robot.
        """
        out = {}
        for k, nm in enumerate(LEGS):
            jr = np.array([obs["rotor"][3 * k + i] for i in range(3)])
            js = np.array([obs["load"].get(f"{nm}_{p}", 0.0) for p in PAIRS])
            a0 = np.array([self.a0[f"{nm}_{p}"] for p in PAIRS])
            out[nm] = wbc.joint_from_encoders(jr, js, a0, self.pair_map,
                                              MT.SPOOL_R)
        return out

    # -- domain randomisation ---------------------------------------------
    #: What may be randomised, and the range M79 measured as biting. ⚠️
    #: `mu_capstan` is deliberately ABSENT: [ADR-0083](../../../docs/DESIGN_DECISIONS.md)
    #: found the wraps it would scale come from a routing `mechanical/` still
    #: owes (198 deg on a redirect against a 30-45 deg budget), and randomising
    #: around a geometry known to be wrong buys nothing.
    RANGES = {
        "mass_scale": (0.85, 1.15),      # build tolerance + unmodelled cabling
        "floor_mu": (0.5, 1.1),          # ADR-0058 ships 0.8
        "series_k_scale": (0.7, 1.4),    # ADR-0050's 150-200 kN/m band, widened
        "latency_s": (3.0e-3, 12.0e-3),  # NFR12 budgets 7.5 ms
    }

    #: Sensor error, drawn per episode. ⚠️ These change the OBSERVATION, not
    #: the plant — with a fixed action the trajectory is untouched, and they
    #: only reach the robot through a closed loop. So "does it bite" is asked of
    #: the observation and of `joint_estimate`, not of the episode.
    SENSOR_RANGES = {
        "enc_offset_rad": (0.0, 0.004),   # absolute-encoder mounting/zeroing
        "enc_noise_rad": (0.0, 0.0008),   # ~2 LSB of a 14-bit encoder
        "load_scale": (0.90, 1.10),       # load-cell calibration, ADR-0004 owes it
        "imu_tilt_deg": (0.0, 1.5),       # IMU mounting misalignment
        "gyro_noise": (0.0, 0.02),        # rad/s
    }

    def _nominal(self):
        """Snapshot the shipped model, so randomisation is never cumulative."""
        if hasattr(self, "_nom"):
            self.model.body_mass[:] = self._nom["mass"]
            self.model.body_inertia[:] = self._nom["inertia"]
            self.model.geom_friction[:] = self._nom["friction"]
            self.model.jnt_stiffness[:] = self._nom["stiffness"]
            return
        self._friction_geoms = [
            i for i in range(self.model.ngeom)
            if (self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_GEOM, i)
                or "") in ("floor",) or (
                self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_GEOM, i)
                or "").endswith("_pad")]
        self._nom = {
            "mass": np.array(self.model.body_mass, float),
            "inertia": np.array(self.model.body_inertia, float),
            "friction": np.array(self.model.geom_friction, float),
            "stiffness": np.array(self.model.jnt_stiffness, float),
        }

    def _roll(self):
        """Draw one episode's plant. ⚠️ The CONTROLLER is not told.

        `self.k_tors` stays nominal on purpose: the rotor servo on the real robot
        does not know the spring it actually got. Updating it here would randomise
        the plant and the model together, which trains a policy against an error
        that cancels.
        """
        r, out = self.rng, {}
        if "mass_scale" in self.randomize:
            f = r.uniform(*self.randomize["mass_scale"])
            self.model.body_mass[:] = self._nom["mass"] * f
            self.model.body_inertia[:] = self._nom["inertia"] * f
            out["mass_scale"] = f
        if "floor_mu" in self.randomize:
            f = r.uniform(*self.randomize["floor_mu"])
            # ⚠️ **Both sides, or it only works upward.** MuJoCo takes the
            # elementwise MAX of the two geoms' friction, and the paw pads ship
            # at 0.8 like the floor. Setting the floor alone to 0.5 gave a
            # BYTE-IDENTICAL episode -- max(0.5, 0.8) is still 0.8 -- which is
            # M59's mistake (friction written where the contact does not read
            # it) repeated in the randomiser.
            for g in self._friction_geoms:
                self.model.geom_friction[g, 0] = f
            out["floor_mu"] = f
        if "series_k_scale" in self.randomize:
            f = r.uniform(*self.randomize["series_k_scale"])
            k = self._nom["stiffness"] > 0.0
            self.model.jnt_stiffness[k] = self._nom["stiffness"][k] * f
            out["series_k_scale"] = f
        if "latency_s" in self.randomize:
            self.latency_s = float(r.uniform(*self.randomize["latency_s"]))
            out["latency_s"] = self.latency_s
        for k in self.SENSOR_RANGES:
            if k in self.randomize:
                out[k] = float(r.uniform(*self.randomize[k]))
        # a per-episode encoder zero error is a CONSTANT, not noise: it is where
        # the magnet sits, and it does not average away over an episode
        self._enc_bias = (r.normal(0.0, out["enc_offset_rad"], 18)
                          if out.get("enc_offset_rad") else np.zeros(18))
        self._load_gain = out.get("load_scale", 1.0)
        self._imu_tilt = math.radians(out.get("imu_tilt_deg", 0.0))
        self.drawn = out
        return out

    # -- the task ---------------------------------------------------------
    @staticmethod
    def _tilt(obs) -> float:
        """Trunk tilt from the IMU quaternion — a sensor, not privileged."""
        w = float(obs["imu_quat"][0])
        return 2.0 * math.degrees(math.acos(min(1.0, abs(w))))

    @property
    def tilt_deg(self) -> float:
        return self._tilt(self.observe())

    def reward(self, obs, tension=None) -> float:
        """Stand up, cheaply, on your feet.

        Every term is built from the OBSERVATION, so the reward is computable on
        hardware. That is not a nicety: a reward that needs `d.qpos` cannot be
        used to fine-tune on the real robot, and it hides the same gap
        [ADR-0081](../../../docs/DESIGN_DECISIONS.md) found in the controller.
        """
        upright = math.cos(math.radians(min(180.0, self._tilt(obs))))
        feet = float(np.count_nonzero(np.asarray(obs["contact"]) > 1e-6)) / 4.0
        spin = float(np.linalg.norm(obs["imu_gyro"]))
        cost = 0.0 if tension is None else float(
            np.mean(np.abs(tension)) / MT.TENSION_MAX)
        tall = min(1.0, self.stance_height(obs) / 0.17)
        return (1.0 * upright + 0.5 * feet + 1.0 * tall
                - 0.05 * min(spin, 20.0) - 0.2 * cost)

    def stance_height(self, obs) -> float:
        """How far the hips sit above the feet, in metres, from OBSERVATION only.

        ⚠️ **The robot cannot know its height above the ground**
        ([ADR-0081](../../../docs/DESIGN_DECISIONS.md)) -- no sensor gives a
        world position. It *can* know its height above its own feet: reconstruct
        the joint angles from the rotor encoders and load cells, run the leg
        forward kinematics, and read the paw drop. That is observable, and it is
        what tells a collapsed robot from a standing one.

        ⚠️ **The leg frame is not the world, and M81 found that the hard
        way.** A first version returned the paw drop straight out of
        `LegModel.forward`, which is the leg's own sagittal frame. Splay the legs
        and lie down and that number *grows*: the gravity-compensating hold sank
        the trunk to **27.8 mm** while this reported **264 mm** and rising, so a
        robot flat on its belly passed as standing for a full 4 s episode. That
        is M77's own mistake -- a termination that cannot see the failure -- one
        layer further in.

        The fix keeps everything observable: rotate the hip-to-paw vector by the
        **IMU quaternion** and take the world-vertical component. A leg pointing
        sideways contributes nothing, which is what lying down looks like.

        ⚠️ The fore legs hang off the front girdle, which the spine moves;
        this uses the trunk IMU for all four, so a strongly bent spine flatters
        the fore pair. One IMU is what the board has.
        """
        from . import LegModel
        lp = {"LF": DEFAULT_FORELEG, "RF": DEFAULT_FORELEG,
              "LR": DEFAULT_HINDLEG, "RR": DEFAULT_HINDLEG}
        est = self.joint_estimate(obs)
        R = np.zeros(9)
        self.mj.mju_quat2Mat(R, np.asarray(obs["imu_quat"], float))
        R = R.reshape(3, 3)
        drops = []
        for nm in LEGS:
            x, z = LegModel(lp[nm]).forward(est[nm])[:2]
            drops.append(-float((R @ np.array([x, 0.0, z]))[2]))
        # ⚠️ **`min`, not `max`.** Taking the largest drop let ONE extended
        # leg vouch for the whole robot: under the gravity hold the hind pair
        # folded to 32 mm with the rear on the floor while the fore pair
        # stretched to 250, and `max` reported 250 and passed it. If any corner
        # is down, the robot is down.
        return min(drops)

    def terminated(self, obs) -> bool:
        """⚠️ Tilt ALONE is not enough, and that is a measured mistake.

        A first pass terminated on `tilt > 45` and let a zero-action episode run
        its full length scoring 192 -- while the trunk collapsed from 176 mm to
        **27.9 mm**. The robot had belly-flopped, level. Both terms are needed,
        and both are computed from the observation.
        """
        return (self._tilt(obs) > self.fall_deg
                or self.stance_height(obs) < self.collapse_m)
