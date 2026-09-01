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
from .params import DEFAULT_FORELEG, DEFAULT_HINDLEG, DEFAULT_TENDON

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
SERIES_K = 1.5e5


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
                 jitter_deg: float = 0.0, rng=None):
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
        self.k_tors = float(series_k) * MT.SPOOL_R ** 2
        self.servo_kp = MT.ROTOR_ARMATURE * MT.ROTOR_BANDWIDTH ** 2
        self.encoder_bits = int(encoder_bits)
        self.latency_s = float(latency_s)
        self.dt = float(self.model.opt.timestep)
        self.decim = max(1, int(round(1.0 / (float(control_hz) * self.dt))))
        self.control_hz = 1.0 / (self.decim * self.dt)
        self.pairs = [f"{nm}_{p}" for nm in LEGS for p in PAIRS] + list(SPINE)

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
    def reset(self):
        """Stance pose, cleared delay buffer, and the first observation."""
        self.mj.mj_resetData(self.model, self.data)
        for nm in LEGS:
            for i, jn in enumerate((f"{nm}_q1", f"{nm}_q2", f"{nm}_q3")):
                a = self.model.jnt_qposadr[
                    self._id(self.mj.mjtObj.mjOBJ_JOINT, jn)]
                jit = math.radians(self.jitter_deg) * (
                    self.rng.standard_normal() if self.jitter_deg else 0.0)
                self.data.qpos[a] = self.q_ref[nm][i] + jit
        self.mj.mj_forward(self.model, self.data)
        self.steps = 0
        self.a0 = {}
        for p in self.pairs:
            t = self._id(self.mj.mjtObj.mjOBJ_TENDON, p)
            self.a0[p] = float(self.data.ten_length[t])
        self._lag = wbc.SensorDelay(self.latency_s, self.dt,
                                    self.data.sensordata,
                                    np.zeros_like(self.data.sensordata))
        self.t = 0.0
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
        if self.encoder_bits:
            enc = wbc.quantise(enc, self.encoder_bits)
        load = {p: float(take("load_" + p)[0])
                for p in self.pairs if ("load_" + p) in self._sensor}
        return {
            "rotor": enc, "rotor_vel": encv, "load": load,
            "imu_quat": take("imu_quat"), "imu_gyro": take("imu_gyro"),
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

        Returns the deepest leg's drop, so a robot folded onto its belly reports
        a small number even when it is perfectly level.
        """
        from . import LegModel
        lp = {"LF": DEFAULT_FORELEG, "RF": DEFAULT_FORELEG,
              "LR": DEFAULT_HINDLEG, "RR": DEFAULT_HINDLEG}
        est = self.joint_estimate(obs)
        return max(float(-LegModel(lp[nm]).forward(est[nm])[1]) for nm in LEGS)

    def terminated(self, obs) -> bool:
        """⚠️ Tilt ALONE is not enough, and that is a measured mistake.

        A first pass terminated on `tilt > 45` and let a zero-action episode run
        its full length scoring 192 -- while the trunk collapsed from 176 mm to
        **27.9 mm**. The robot had belly-flopped, level. Both terms are needed,
        and both are computed from the observation.
        """
        return (self._tilt(obs) > self.fall_deg
                or self.stance_height(obs) < self.collapse_m)
