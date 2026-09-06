"""Python port of rbq_sdk/cpp/example/src/rbq_low_level.cpp.

Loads an RL policy (info.json + policy.onnx) and drives the leg joints over the
``rt/rbq/ref/motion/_20`` topic. Keyboard controls (single-character,
non-canonical) match the C++ version:

    z - goToMotionGround    (folding -> ground)
    x - goToMotionReady     (ground -> ready, or folding -> ready)
    c - run RL policy (TaskState::Control)
    q - quit

Usage:
    python3 rbq_low_level.py -p <policyDir> [<networkInterface>]

The policy directory must contain info.json and policy.onnx (same layout the
C++ version expects). Supported policy names: rbq10, rbq10_trot, rbq10_trot_run.
"""

import argparse
import enum
import json
import math
import os
import signal
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

try:
    import onnxruntime as ort
except ImportError:
    ort = None  # type: ignore[assignment]

from rbq_sdk_py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)

sys.path.insert(
    0,
    os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "idl")
    ),
)
from _rbq_idl import (  # noqa: E402
    Imu_,
    Joy_,
    JointOwnershipCmd_,
    LegJointInfo_,
    MotionRef_,
)


APP_NAME = "rbq_low_level"

R2D = 57.295779513
D2R = 0.0174532925

LEG_JOINT_COUNT = 12


# ---------------------------------------------------------------------------
# JointTable / JointControl / Filter — direct ports of the C++ headers
# (rbq_sdk/cpp/rbq_sdk_cpp/include/JointControl.hpp, Filter.hpp).
# ---------------------------------------------------------------------------


class JointTable:
    """Reference postures (deg) for each leg joint index 0..11."""

    def __init__(self):
        self.ready = [0.0] * 12
        self.ground = [0.0] * 12
        self.folding = [0.0] * 12
        for leg in range(4):
            off = leg * 3
            self.ready[off + 0] = 0.0
            self.ready[off + 1] = 43.0
            self.ready[off + 2] = -80.0

            self.ground[off + 0] = -34.0 if (leg % 2 == 0) else 34.0
            self.ground[off + 1] = 65.0
            self.ground[off + 2] = -160.0

            self.folding[off + 0] = 0.0
            self.folding[off + 1] = 78.0
            self.folding[off + 2] = -160.0


class MoveCommandMode(enum.IntEnum):
    Absolute = 0
    Relative = 1


class _Joint:
    """Cosine-interpolated trajectory generator for a single joint.

    The C++ Joint uses a tick counter advanced once per controlLoop iteration
    (2 ms period), with goal_time_count = duration_ms / 2. We mirror that
    exactly: setMove(angle, duration_ms, mode) → reach in duration_ms over
    duration_ms/2 update() calls.
    """

    def __init__(self, idx: int):
        self._id = idx
        self._position = 0.0
        self._initial = 0.0
        self._target = 0.0
        self._delta = 0.0
        self._goal = 0
        self._cur = 0
        self._moving = False

    def position(self) -> float:
        return self._position

    def set_position(self, pos: float):
        self._moving = False
        self._position = float(pos)

    def set_target(self, angle: float, duration_ms: float, mode: MoveCommandMode):
        self._moving = False
        if duration_ms <= 0.0:
            print(f"[Joint {self._id}] Invalid goal time.", file=sys.stderr)
            return False
        if mode == MoveCommandMode.Absolute:
            self._target = float(angle)
        elif mode == MoveCommandMode.Relative:
            self._target = self._position + float(angle)
        else:
            print(f"[Joint {self._id}] Invalid move mode.", file=sys.stderr)
            return False
        self._initial = self._position
        self._delta = self._target - self._position
        self._cur = 0
        self._goal = max(1, int(duration_ms / 2))
        self._moving = True
        return True

    def update(self):
        if not self._moving:
            return
        self._cur += 1
        if self._cur >= self._goal:
            self._position = self._target
            self._moving = False
            return
        progress = self._cur / self._goal
        self._position = self._initial + self._delta * 0.5 * (
            1.0 - math.cos(math.pi * progress)
        )


class JointControl:
    def __init__(self):
        self._joints = [_Joint(i) for i in range(12)]

    def position(self, idx: int) -> float:
        return self._joints[idx].position()

    def set_position(self, idx: int, pos: float):
        self._joints[idx].set_position(pos)

    def set_move(self, idx: int, angle: float, duration_ms: float, mode: MoveCommandMode):
        return self._joints[idx].set_target(angle, duration_ms, mode)

    def update(self, idx: Optional[int] = None):
        if idx is None:
            for j in self._joints:
                j.update()
        else:
            self._joints[idx].update()


def lpf(input_v: float, output_v: float, f_cut: float, ts: float) -> float:
    if f_cut == 0:
        return 0.0
    tau = 1.0 / (1.0 + 2 * math.pi * f_cut * ts)
    return tau * output_v + (1.0 - tau) * input_v


# ---------------------------------------------------------------------------
# PolicyParams + Policy (onnxruntime) — port of Policy.hpp
# ---------------------------------------------------------------------------


class PolicyParamsError(enum.IntEnum):
    NONE = 0
    CONFIG_NOT_FOUND = 1
    CONFIG_READ_FAIL = 2
    CONFIG_PARSE_FAIL = 3


@dataclass
class PolicyParams:
    name: str = ""
    dt: float = 0.0
    action_scale: float = 0.0
    KP: List[float] = field(default_factory=lambda: [0.0] * 12)
    KD: List[float] = field(default_factory=lambda: [0.0] * 12)
    num_observations: int = 0
    num_actions: int = 0
    command_ranges: List[List[float]] = field(
        default_factory=lambda: [[0.0, 0.0] for _ in range(3)]
    )
    default_joint_angles: List[float] = field(default_factory=list)
    obs_lin_vel_scale: float = 0.0
    obs_ang_vel_scale: float = 0.0
    obs_dof_pos_scale: float = 0.0
    obs_dof_vel_scale: float = 0.0
    clip_observations: float = 0.0
    clip_actions: float = 0.0

    _loaded: bool = False
    _error: PolicyParamsError = PolicyParamsError.NONE

    @property
    def loaded(self) -> bool:
        return self._loaded

    @property
    def error(self) -> PolicyParamsError:
        return self._error

    def load_from_path(self, path: str) -> None:
        self._loaded = False
        config = os.path.join(path, "info.json")
        if not os.path.exists(config):
            self._error = PolicyParamsError.CONFIG_NOT_FOUND
            raise RuntimeError(f"Config file path is wrong: {config}")
        try:
            with open(config, "r") as f:
                j = json.load(f)
        except OSError as e:
            self._error = PolicyParamsError.CONFIG_READ_FAIL
            raise RuntimeError(f"Config file read failed: {config}") from e

        try:
            ci = j["config_info"]
            self.name = ci["run_name"]
            self.dt = float(ci["control"]["policy_dt"])
            self.action_scale = float(ci["control"]["action_scale"])
            stiff = ci["control"]["stiffness"]
            damp = ci["control"]["damping"]
            for leg in range(4):
                self.KP[leg * 3 + 0] = float(stiff["R"])
                self.KP[leg * 3 + 1] = float(stiff["P"])
                self.KP[leg * 3 + 2] = float(stiff["K"])
                self.KD[leg * 3 + 0] = float(damp["R"])
                self.KD[leg * 3 + 1] = float(damp["P"])
                self.KD[leg * 3 + 2] = float(damp["K"])
            self.num_observations = int(ci["env"]["num_observations"])
            self.num_actions = int(ci["env"]["num_actions"])
            ranges = ci["commands"]["ranges"]
            for i in range(2):
                self.command_ranges[0][i] = float(ranges["lin_vel_x"][i])
                self.command_ranges[1][i] = float(ranges["lin_vel_y"][i])
                self.command_ranges[2][i] = float(ranges["ang_vel_yaw"][i])
            norm = ci["normalization"]
            self.obs_lin_vel_scale = float(norm["obs_scales"]["lin_vel"])
            self.obs_ang_vel_scale = float(norm["obs_scales"]["ang_vel"])
            self.obs_dof_pos_scale = float(norm["obs_scales"]["dof_pos"])
            self.obs_dof_vel_scale = float(norm["obs_scales"]["dof_vel"])
            self.clip_observations = float(norm["clip_observations"])
            self.clip_actions = float(norm["clip_actions"])
            init = ci["init_state"]["default_joint_angles"]
            keys = [
                "joint0_HRR", "joint1_HRP", "joint2_HRK",
                "joint3_HLR", "joint4_HLP", "joint5_HLK",
                "joint6_FRR", "joint7_FRP", "joint8_FRK",
                "joint9_FLR", "joint10_FLP", "joint11_FLK",
            ]
            self.default_joint_angles = [float(init[k]) for k in keys]
            self._error = PolicyParamsError.NONE
            self._loaded = True
            print(f"Config file parse complete: {config}")
        except (KeyError, TypeError, ValueError) as e:
            self._error = PolicyParamsError.CONFIG_PARSE_FAIL
            raise RuntimeError(
                f"Config file parse failed: {config}. Error: {e}"
            ) from e


class PolicyError(enum.IntEnum):
    NONE = 0
    POLICY_NOT_FOUND = 1
    POLICY_NOT_LOADED = 2
    INVALID_INPUT_SIZE = 3
    INVALID_OUTPUT_SIZE = 4
    INPUT_SIZE_MISMATCH = 5
    OUTPUT_DATA_HAS_NAN_OR_INF = 6


class Policy:
    def __init__(self):
        self._session = None
        self._input_name = None
        self._output_name = None
        self._loaded = False
        self._error = PolicyError.NONE

    @property
    def loaded(self) -> bool:
        return self._loaded

    @property
    def error(self) -> PolicyError:
        return self._error

    def reload(self, path: str) -> None:
        if ort is None:
            raise RuntimeError("onnxruntime is not installed (pip install onnxruntime)")
        self.release()
        policy_path = os.path.join(path, "policy.onnx")
        if not os.path.exists(policy_path):
            self._error = PolicyError.POLICY_NOT_LOADED
            print(f"Policy file path is wrong: {policy_path}", file=sys.stderr)
            return
        try:
            so = ort.SessionOptions()
            so.intra_op_num_threads = 1
            so.graph_optimization_level = (
                ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            )
            self._session = ort.InferenceSession(policy_path, sess_options=so)
            inputs = self._session.get_inputs()
            outputs = self._session.get_outputs()
            if not inputs:
                self._error = PolicyError.INVALID_INPUT_SIZE
                return
            if not outputs:
                self._error = PolicyError.INVALID_OUTPUT_SIZE
                return
            self._input_name = inputs[0].name
            self._output_name = outputs[0].name
            self._loaded = True
            self._error = PolicyError.NONE
            print(f"Policy file parse complete: {policy_path}")
        except Exception as e:  # onnxruntime can raise a variety of errors
            self._error = PolicyError.POLICY_NOT_LOADED
            self._loaded = False
            print(f"Policy file load failed: {policy_path} ({e})", file=sys.stderr)

    def release(self) -> None:
        self._loaded = False
        self._session = None
        self._input_name = None
        self._output_name = None
        self._error = PolicyError.NONE

    def inference(
        self, obs: List[float], input_size: int, output_size: int
    ):
        result = [0.0] * output_size
        if not self._loaded or self._session is None:
            self._error = PolicyError.POLICY_NOT_LOADED
            return result, self._error
        if len(obs) != input_size:
            self._error = PolicyError.INPUT_SIZE_MISMATCH
            print("[ERROR] Policy observation size mismatch", file=sys.stderr)
            return result, self._error
        try:
            x = np.asarray(obs, dtype=np.float32).reshape(1, input_size)
            (out,) = self._session.run([self._output_name], {self._input_name: x})
            flat = out.reshape(-1)
            if not np.all(np.isfinite(flat[:output_size])):
                self._error = PolicyError.OUTPUT_DATA_HAS_NAN_OR_INF
                print("[ERROR] NaN or Inf detected in output data", file=sys.stderr)
                return result, self._error
            for i in range(output_size):
                result[i] = float(flat[i])
            self._error = PolicyError.NONE
        except Exception as e:
            self._error = PolicyError.POLICY_NOT_LOADED
            print(f"Policy execution error: {e}", file=sys.stderr)
        return result, self._error


# ---------------------------------------------------------------------------
# Application state
# ---------------------------------------------------------------------------


class TaskState(enum.IntEnum):
    Idle = 0
    Motion = 1
    Control = 2


class SharedState:
    def __init__(self):
        self.is_working = True
        self.current_task = TaskState.Idle
        self.policy_path = ""

        # Latest received samples (read by control loop; written by DDS listener).
        # Each callback overwrites the shared reference atomically; we never
        # mutate the inner objects in-place, so simple assignment is safe.
        # Pre-initialized with default-zero instances to mirror the C++ source,
        # which uses stack-allocated POD message structs. Callers may treat
        # these fields as "always present".
        self.msg_joy: Joy_ = Joy_()
        self.msg_imu: Imu_ = Imu_()
        self.msg_leg: LegJointInfo_ = LegJointInfo_()
        self.msg_motion_ref_final: MotionRef_ = MotionRef_()
        # Tracks whether the corresponding subscriber has produced at least
        # one sample. Useful for diagnostics (Control mode warns if we are
        # running the policy on still-zero observations).
        self.has_imu = False
        self.has_leg = False
        self.has_joy = False
        self.has_motion_ref_final = False


# ---------------------------------------------------------------------------
# Control loop
# ---------------------------------------------------------------------------


def _quat_inverse_rotate(qw, qx, qy, qz, vx, vy, vz):
    """Rotate vector (vx,vy,vz) by inverse quaternion (qw,qx,qy,qz)."""
    # Inverse of a unit quaternion equals its conjugate.
    cw, cx, cy, cz = qw, -qx, -qy, -qz
    # v' = q * v * q_inv   (eq. for unit quaternions)
    tx = 2 * (cy * vz - cz * vy)
    ty = 2 * (cz * vx - cx * vz)
    tz = 2 * (cx * vy - cy * vx)
    rx = vx + cw * tx + (cy * tz - cz * ty)
    ry = vy + cw * ty + (cz * tx - cx * tz)
    rz = vz + cw * tz + (cx * ty - cy * tx)
    return rx, ry, rz


def _zeros(n):
    return [0.0] * n


def control_loop(state: SharedState):
    print("Control loop started.")
    time.sleep(0.1)

    joint_control = JointControl()

    pub_motion_ref = ChannelPublisher("rt/rbq/ref/motion/_20", MotionRef_)
    pub_motion_ref.Init()

    pub_joint_owner = ChannelPublisher(
        "rt/rbq/cmd/motion/owner/_20", JointOwnershipCmd_
    )
    pub_joint_owner.Init()

    def _on_joy(s):
        state.msg_joy = s
        state.has_joy = True

    def _on_imu(s):
        state.msg_imu = s
        state.has_imu = True

    def _on_leg(s):
        state.msg_leg = s
        state.has_leg = True

    def _on_motion_ref_final(s):
        state.msg_motion_ref_final = s
        state.has_motion_ref_final = True

    sub_joy = ChannelSubscriber("rt/rbq/joy", Joy_)
    sub_joy.Init(handler=_on_joy, queueLen=4)

    sub_imu = ChannelSubscriber("rt/rbq/imu", Imu_)
    sub_imu.Init(handler=_on_imu, queueLen=4)

    sub_leg = ChannelSubscriber("rt/rbq/leg_joint", LegJointInfo_)
    sub_leg.Init(handler=_on_leg, queueLen=4)

    sub_motion_ref_final = ChannelSubscriber(
        "rt/rbq/ref/motion/_0", MotionRef_
    )
    sub_motion_ref_final.Init(handler=_on_motion_ref_final, queueLen=4)

    # Joint control needs to be exposed to the keyboard handler for
    # goToMotionReady / goToMotionGround sequences.
    state.joint_control = joint_control
    state.pub_motion_ref = pub_motion_ref
    state.pub_joint_owner = pub_joint_owner

    last_task = TaskState.Idle
    reset = True

    # Per-policy LP-filtered command + history buffers (rbq10_trot* keeps a 7
    # frame history, rbq10 only uses the latest).
    command = [0.0, 0.0, 0.0]
    action_1 = _zeros(LEG_JOINT_COUNT)
    action_2 = _zeros(LEG_JOINT_COUNT)
    dof_pos_hist = [list(_zeros(LEG_JOINT_COUNT)) for _ in range(7)]
    dof_vel_hist = [list(_zeros(LEG_JOINT_COUNT)) for _ in range(7)]

    params = PolicyParams()
    policy = Policy()

    decimation_cnt = 0
    dt = 0.002  # 2 ms = 500 Hz target
    next_tick = time.monotonic()
    last_cold_warn = 0.0  # monotonic timestamp; throttle the cold-topic warning

    while state.is_working:
        next_tick += dt

        if last_task != state.current_task:
            last_task = state.current_task
            reset = True

        msg_final = state.msg_motion_ref_final

        if state.current_task == TaskState.Idle:
            for idx in range(LEG_JOINT_COUNT):
                joint_control.set_position(idx, msg_final.leg_joint[idx].pos)
            ref = MotionRef_()
            for idx in range(LEG_JOINT_COUNT):
                ref.leg_joint[idx].pos = float(joint_control.position(idx))
                ref.leg_joint[idx].kp = 200.0
                ref.leg_joint[idx].kd = 2.5
                ref.leg_joint[idx].torque = 0.0
            pub_motion_ref.Write(ref)

        elif state.current_task == TaskState.Motion:
            joint_control.update()
            ref = MotionRef_()
            for idx in range(LEG_JOINT_COUNT):
                ref.leg_joint[idx].pos = float(joint_control.position(idx))
                ref.leg_joint[idx].kp = 200.0
                ref.leg_joint[idx].kd = 2.5
                ref.leg_joint[idx].torque = 0.0
            pub_motion_ref.Write(ref)
            if reset:
                reset = False
                own = JointOwnershipCmd_()
                for idx in range(LEG_JOINT_COUNT):
                    own.leg_joint[idx] = True
                pub_joint_owner.Write(own)

        elif state.current_task == TaskState.Control:
            policy_error = False
            if decimation_cnt == 5:
                decimation_cnt = 0
                try:
                    if (
                        not policy.loaded
                        or not params.loaded
                        or reset
                    ):
                        print(f"Loading policy from: {state.policy_path}")
                        params.load_from_path(state.policy_path)
                        policy.release()
                        policy.reload(state.policy_path)

                    if not (policy.loaded and policy.error == PolicyError.NONE):
                        print(
                            f"Policy not loaded (loaded={policy.loaded}, "
                            f"error={int(policy.error)}).",
                            file=sys.stderr,
                        )
                        policy_error = True
                    elif not (params.loaded and params.error == PolicyParamsError.NONE):
                        print(
                            f"PolicyParams not loaded (loaded={params.loaded}, "
                            f"error={int(params.error)}).",
                            file=sys.stderr,
                        )
                        policy_error = True
                    else:
                        # We pre-initialize msg_imu / msg_leg / msg_joy with
                        # zero defaults in SharedState (matching the C++
                        # source's stack-allocated message structs), so the
                        # policy can run even before the first sample
                        # arrives. Warn once if any subscriber is still cold
                        # so the user can spot the missing topic flow.
                        if not (state.has_imu and state.has_leg and state.has_joy):
                            now_t = time.monotonic()
                            if now_t - last_cold_warn > 2.0:
                                last_cold_warn = now_t
                                missing = []
                                if not state.has_imu:
                                    missing.append("rt/rbq/imu")
                                if not state.has_leg:
                                    missing.append("rt/rbq/leg_joint")
                                if not state.has_joy:
                                    missing.append("rt/rbq/joy")
                                print(
                                    "Warning: no samples yet on: "
                                    + ", ".join(missing)
                                    + " (running policy on zero-init "
                                    "observations; throttled to 0.5 Hz)",
                                    file=sys.stderr,
                                )
                        imu = state.msg_imu
                        leg = state.msg_leg
                        joy = state.msg_joy

                        gyro = (
                            imu.angular_velocity.x,
                            imu.angular_velocity.y,
                            imu.angular_velocity.z,
                        )
                        qw, qx, qy, qz = (
                            imu.orientation.w,
                            imu.orientation.x,
                            imu.orientation.y,
                            imu.orientation.z,
                        )
                        pos = [leg.joint[i].pos for i in range(LEG_JOINT_COUNT)]
                        vel = [leg.joint[i].vel for i in range(LEG_JOINT_COUNT)]

                        # Joystick filtering (LPF f_cut=1Hz, ts=0.1s) matches C++.
                        ax = list(joy.axes) + [0.0] * max(0, 3 - len(joy.axes))
                        command[0] = lpf(ax[1] * +1, command[0], 1.0, 0.1)
                        command[1] = lpf(ax[0] * -1, command[1], 1.0, 0.1)
                        command[2] = lpf(ax[2] * -1, command[2], 1.0, 0.1)

                        if reset:
                            action_1 = _zeros(LEG_JOINT_COUNT)
                            action_2 = _zeros(LEG_JOINT_COUNT)
                            command[:] = [0.0, 0.0, 0.0]
                            for hist in dof_pos_hist:
                                for i in range(LEG_JOINT_COUNT):
                                    hist[i] = pos[i]
                            for hist in dof_vel_hist:
                                for i in range(LEG_JOINT_COUNT):
                                    hist[i] = 0.0
                            # dof_vel_0 is initialized to current vel (per C++).
                            for i in range(LEG_JOINT_COUNT):
                                dof_vel_hist[0][i] = vel[i]

                        obs = []
                        if params.name == "rbq10":
                            for i in range(3):
                                obs.append(gyro[i] * params.obs_ang_vel_scale)
                            proj = _quat_inverse_rotate(
                                qw, qx, qy, qz, 0.0, 0.0, -1.0
                            )
                            obs.extend(proj)
                            commands_scale = (
                                params.obs_lin_vel_scale,
                                params.obs_lin_vel_scale,
                                params.obs_ang_vel_scale,
                            )
                            for i in range(3):
                                obs.append(command[i] * commands_scale[i])
                            for i in range(LEG_JOINT_COUNT):
                                obs.append(
                                    (pos[i] - params.default_joint_angles[i])
                                    * params.obs_dof_pos_scale
                                )
                            for i in range(LEG_JOINT_COUNT):
                                obs.append(vel[i] * params.obs_dof_vel_scale)
                            for i in range(LEG_JOINT_COUNT):
                                obs.append(action_1[i])
                        elif params.name in ("rbq10_trot", "rbq10_trot_run"):
                            for i in range(3):
                                obs.append(gyro[i] * params.obs_ang_vel_scale)
                            proj = _quat_inverse_rotate(
                                qw, qx, qy, qz, 0.0, 0.0, -1.0
                            )
                            obs.extend(proj)
                            commands_scale = (
                                params.obs_lin_vel_scale,
                                params.obs_lin_vel_scale,
                                params.obs_ang_vel_scale,
                            )
                            for i in range(3):
                                obs.append(command[i] * commands_scale[i])

                            if not reset:
                                # Shift history (oldest dropped).
                                for slot in range(6, 0, -1):
                                    for i in range(LEG_JOINT_COUNT):
                                        dof_pos_hist[slot][i] = dof_pos_hist[slot - 1][i]
                                        dof_vel_hist[slot][i] = dof_vel_hist[slot - 1][i]
                                for i in range(LEG_JOINT_COUNT):
                                    dof_pos_hist[0][i] = pos[i]
                                    dof_vel_hist[0][i] = vel[i]
                                for i in range(LEG_JOINT_COUNT):
                                    action_2[i] = action_1[i]

                            for slot in (0, 2, 4, 6):
                                for i in range(LEG_JOINT_COUNT):
                                    obs.append(
                                        (dof_pos_hist[slot][i] - params.default_joint_angles[i])
                                        * params.obs_dof_pos_scale
                                    )
                            for slot in (0, 2, 4, 6):
                                for i in range(LEG_JOINT_COUNT):
                                    obs.append(
                                        dof_vel_hist[slot][i] * params.obs_dof_vel_scale
                                    )
                            for i in range(LEG_JOINT_COUNT):
                                obs.append(action_1[i])
                            for i in range(LEG_JOINT_COUNT):
                                obs.append(action_2[i])
                            obs.append(0.0)  # added mass
                        else:
                            print(
                                f"Unsupported policy name: {params.name}",
                                file=sys.stderr,
                            )
                            policy_error = True

                        if not policy_error:
                            actions, perr = policy.inference(
                                obs, params.num_observations, params.num_actions
                            )
                            for i in range(LEG_JOINT_COUNT):
                                a = actions[i]
                                if a > params.clip_actions:
                                    a = params.clip_actions
                                elif a < -params.clip_actions:
                                    a = -params.clip_actions
                                actions[i] = a
                                action_1[i] = a
                            if perr == PolicyError.NONE:
                                ref = MotionRef_()
                                for i in range(LEG_JOINT_COUNT):
                                    pos_ref = (
                                        actions[i] * params.action_scale
                                        + params.default_joint_angles[i]
                                    )
                                    ref.leg_joint[i].pos = float(pos_ref)
                                    ref.leg_joint[i].kp = float(params.KP[i])
                                    ref.leg_joint[i].kd = float(params.KD[i])
                                    ref.leg_joint[i].torque = 0.0
                                pub_motion_ref.Write(ref)
                                if reset:
                                    own = JointOwnershipCmd_()
                                    for idx in range(LEG_JOINT_COUNT):
                                        own.leg_joint[idx] = True
                                    pub_joint_owner.Write(own)
                            else:
                                print(
                                    f"Policy error occurred: {int(perr)}",
                                    file=sys.stderr,
                                )
                    reset = False
                except Exception as e:
                    print(f"Policy execution error: {e}", file=sys.stderr)
                    policy_error = True

                if policy_error:
                    print(
                        "Exiting control task due to policy error.",
                        file=sys.stderr,
                    )
                    state.current_task = TaskState.Idle
            decimation_cnt += 1

        # Loop pacing: 2 ms target. Python may slip slightly under load.
        delay = next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            # Catch up rather than chase a drifting clock.
            next_tick = time.monotonic()

    print("Control loop exiting.")


# ---------------------------------------------------------------------------
# Motion sequences (called from the keyboard handler)
# ---------------------------------------------------------------------------


def go_to_motion_ready(state: SharedState):
    state.current_task = TaskState.Motion
    time.sleep(0.5)
    motion_time = 1400.0
    final = state.msg_motion_ref_final
    if not state.has_motion_ref_final:
        print(
            "Warning: no rt/rbq/ref/motion/_0 samples received yet "
            "(using zero-init joint angles for grounded check).",
            file=sys.stderr,
        )
    pitch_angles = [
        final.leg_joint[1].pos,
        final.leg_joint[4].pos,
        final.leg_joint[7].pos,
        final.leg_joint[10].pos,
    ]
    is_grounded = all(a > 60 * D2R for a in pitch_angles)
    table = JointTable()
    joint_cmd = table.folding if is_grounded else table.ready
    for idx in range(LEG_JOINT_COUNT):
        state.joint_control.set_move(
            idx, joint_cmd[idx] * D2R, motion_time, MoveCommandMode.Absolute
        )
    time.sleep((motion_time + 100) / 1000.0)
    if is_grounded:
        for idx in range(LEG_JOINT_COUNT):
            state.joint_control.set_move(
                idx, table.ready[idx] * D2R, motion_time, MoveCommandMode.Absolute
            )
        time.sleep((motion_time + 100) / 1000.0)
    state.current_task = TaskState.Idle


def go_to_motion_ground(state: SharedState):
    state.current_task = TaskState.Motion
    time.sleep(0.5)
    motion_time = 2400.0
    table = JointTable()
    for idx in range(LEG_JOINT_COUNT):
        state.joint_control.set_move(
            idx, table.folding[idx] * D2R, motion_time, MoveCommandMode.Absolute
        )
    time.sleep((motion_time + 100) / 1000.0)
    for idx in range(LEG_JOINT_COUNT):
        state.joint_control.set_move(
            idx, table.ground[idx] * D2R, motion_time, MoveCommandMode.Absolute
        )
    time.sleep((motion_time + 100) / 1000.0)
    state.current_task = TaskState.Idle


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def print_help():
    print()
    print(f" {APP_NAME} Help")
    print("\n -p | --path <path> : Specify the policy path to load")
    print("\n Controls:")
    print("  'q' - Quit the application")
    print("  'z' - Motion Sit")
    print("  'x' - Motion Stand")
    print("  'c' - Motion Walk")
    print(" Velocity Control (when in control mode):")
    print("  'left joystick up/down' - Forward/Backward")
    print("  'left joystick left/right' - Left/Right")
    print("  'right joystick left/right' - Yaw Left/Right\n")


def main(argv):
    print(f"Starting {APP_NAME}...")
    print_help()

    parser = argparse.ArgumentParser(prog=APP_NAME, add_help=False)
    parser.add_argument("-p", "--path", dest="path", default="")
    parser.add_argument("-h", "--help", action="store_true")
    parser.add_argument("network", nargs="?", default=None,
                        help="DDS network interface (e.g. eth0). Optional.")
    args = parser.parse_args(argv[1:])

    if args.help:
        print_help()
        return 0

    if not args.path:
        print(
            "Error: Policy path is required. Use -p or --path to specify it.",
            file=sys.stderr,
        )
        return -1

    state = SharedState()
    state.policy_path = args.path
    print(f"Policy path set to: {state.policy_path}")

    def signal_handler(sig, frame):
        print(f"Signal received: {sig}")
        state.is_working = False
        os._exit(sig)

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGHUP, signal_handler)

    ChannelFactoryInitialize(0, args.network)

    print("Control thread starting...")
    t_ctrl = threading.Thread(
        target=control_loop, args=(state,), name="ControlLoop", daemon=True
    )
    t_ctrl.start()

    # Wait until the control loop has built JointControl and publishers so
    # the keyboard handler can safely call go_to_motion_*.
    while state.is_working and not hasattr(state, "joint_control"):
        time.sleep(0.01)

    # Termios non-canonical, no-echo. Restored on exit.
    fd = sys.stdin.fileno()
    old_tio = None
    try:
        old_tio = termios.tcgetattr(fd)
        new_tio = termios.tcgetattr(fd)
        new_tio[3] &= ~(termios.ICANON | termios.ECHO)
        termios.tcsetattr(fd, termios.TCSANOW, new_tio)
        tty.setcbreak(fd)
    except termios.error:
        old_tio = None

    try:
        while state.is_working:
            try:
                key = os.read(fd, 1)
            except OSError:
                break
            if not key:
                break
            c = key.decode("utf-8", errors="ignore")
            if c == "q":
                state.is_working = False
                break
            elif c == "z":
                print("Executing Motion Ground...")
                go_to_motion_ground(state)
                print_help()
            elif c == "x":
                print("Executing Motion Ready...")
                go_to_motion_ready(state)
                print_help()
            elif c == "c":
                print("Executing Motion Walk...")
                state.current_task = TaskState.Control
                print_help()
            elif c == "v":
                # Placeholder for the C++ TODO vision-walk hook.
                print_help()
            else:
                print_help()
            time.sleep(0.01)
    finally:
        if old_tio is not None:
            try:
                termios.tcsetattr(fd, termios.TCSANOW, old_tio)
            except termios.error:
                pass

    print("Shutting down...")
    t_ctrl.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
