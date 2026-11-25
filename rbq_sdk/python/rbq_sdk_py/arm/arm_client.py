import json
import time
from typing import Dict, Optional, Tuple

from ..rpc.client import Client
from .arm_api import *


class ArmClient(Client):
    """RBA-1 arm service — same request/response channel and error codes as SportClient.

    The motion calls are refused with 3205 while an app holds arm joints through the
    joint-level (Lv0) channel — call ReleaseJoints() or release them from that app first. The gripper
    calls are not: the gripper is not a joint-level joint, so a joint-level app can open and close it.

    EmergencyStop() damps the arm: every arm joint gets kp 0 and a damping gain, so the arm sinks slowly under gravity
    and gives way when it is pushed. The robot's E-stop does the same to the arm. It stays on until ClearEmergencyStop()
    or AutoStart(); until then the motion calls are refused with ARM_ERR_ESTOP. The gripper calls still work.
    """

    def __init__(self, enableLease: bool = False):
        super().__init__(ARM_SERVICE_NAME, enableLease)

    def Init(self):
        self._SetApiVerson(ARM_API_VERSION)
        for apiId in (ARM_API_ID_AUTOSTART, ARM_API_ID_CONTROLSTART, ARM_API_ID_MOVEJ,
                      ARM_API_ID_MOVEL, ARM_API_ID_MOVELDELTA, ARM_API_ID_ROTATELDELTA,
                      ARM_API_ID_GRIPPEROPEN, ARM_API_ID_GRIPPERCLOSE, ARM_API_ID_GRIPPERSTOP,
                      ARM_API_ID_RELEASEJOINTS, ARM_API_ID_GETSTATE, ARM_API_ID_STOP,
                      ARM_API_ID_EMERGENCYSTOP, ARM_API_ID_CLEAREMERGENCYSTOP, ARM_API_ID_SETGAINS):
            self._RegistApi(apiId, 0)

    def _CallJson(self, apiId: int, p: dict) -> int:
        code, data = self._Call(apiId, json.dumps(p))
        return code

    def AutoStart(self) -> int:
        return self._CallJson(ARM_API_ID_AUTOSTART, {})

    def ControlStart(self) -> int:
        return self._CallJson(ARM_API_ID_CONTROLSTART, {})

    def MoveJ(self, target, seconds: float = 0.0, vel: float = 0.0, acc: float = 0.0, joint: int = None) -> int:
        """target: one of the ARM_POSE_* names (the first move after AutoStart should be "Ready"), six joint angles
        J0..J5 [rad], or - with `joint` (0 = J0 .. 5 = J5) - that one joint's angle; the other joints stay. For angles,
        the joints arrive together in `seconds` (0.5 - 20 s); with seconds 0 the robot picks the shortest time under
        `vel` [rad/s] and `acc` [rad/s^2] (0 = default 0.52 / 1.05, max 1.57 / 3.14). An angle outside the joint range is
        refused (3204). ControlStart() afterwards needs MoveJ("Ready") first."""
        if isinstance(target, str):
            return self._CallJson(ARM_API_ID_MOVEJ, {"data": target})
        return self._CallJson(ARM_API_ID_MOVEJ, self._joint_params(target, seconds, vel, acc, joint, False))

    def MoveJDelta(self, target, seconds: float = 0.0, vel: float = 0.0, acc: float = 0.0, joint: int = None) -> int:
        """Like MoveJ with angles, but by `target` [rad] from where the joints are commanded now (not the measured angle,
        so a sagging arm does not add up): six deltas J0..J5, or with `joint` that one joint's delta."""
        return self._CallJson(ARM_API_ID_MOVEJ, self._joint_params(target, seconds, vel, acc, joint, True))

    @staticmethod
    def _joint_params(target, seconds, vel, acc, joint, relative) -> dict:
        p = {"joint": int(joint), "q": float(target)} if joint is not None else {"q": [float(a) for a in target]}
        if relative:
            p["relative"] = True
        p.update({"time": seconds} if seconds > 0 else {"vel": vel, "acc": acc})
        return p

    def MoveL(self, x: float, y: float, z: float, seconds: float = 0.0, rpy=None,
              vel: float = 0.0, acc: float = 0.0, ang_vel: float = 0.0, ang_acc: float = 0.0) -> int:
        """End-effector position [m] in the body frame (the frame rt/rbq/arm_ee reports), and orientation when `rpy` =
        (roll, pitch, yaw) [rad] is given - the same numbers rt/rbq/arm_ee reports (Z-Y-X); without it the orientation
        stays. Reached in `seconds` (0.5 - 10 s); with seconds 0 the robot picks the shortest time under `vel` [m/s],
        `acc` [m/s^2], `ang_vel` [rad/s] and `ang_acc` [rad/s^2] (0 = default 0.1 / 0.2 / 0.52 / 1.05, max 0.25 / 0.5 / 1.57 /
        3.14). Needs ControlStart()."""
        p = {"x": x, "y": y, "z": z}
        if rpy is not None:
            p.update({"roll": rpy[0], "pitch": rpy[1], "yaw": rpy[2]})
        p.update({"time": seconds} if seconds > 0 else {"vel": vel, "acc": acc, "ang_vel": ang_vel, "ang_acc": ang_acc})
        return self._CallJson(ARM_API_ID_MOVEL, p)

    def RotateL(self, roll: float, pitch: float, yaw: float, seconds: float = 0.0,
                ang_vel: float = 0.0, ang_acc: float = 0.0) -> int:
        """End-effector orientation [rad] in the body frame (Z-Y-X, as rt/rbq/arm_ee reports it); the position stays.
        Reached in `seconds` (0.5 - 10 s); with seconds 0 the robot picks the shortest time under `ang_vel` [rad/s] and
        `ang_acc` [rad/s^2] (0 = default 0.52 / 1.05, max 1.57 / 3.14). Needs ControlStart()."""
        p = {"roll": roll, "pitch": pitch, "yaw": yaw}
        p.update({"time": seconds} if seconds > 0 else {"ang_vel": ang_vel, "ang_acc": ang_acc})
        return self._CallJson(ARM_API_ID_MOVEL, p)

    def MoveLDelta(self, *args, seconds: float = 0.0, rpy=None, vel: float = 0.0, acc: float = 0.0,
                   ang_vel: float = 0.0, ang_acc: float = 0.0) -> int:
        """MoveLDelta(axis, meters): the end effector by `meters` (|meters| <= 0.05) along ARM_AXIS_X/Y/Z, in 0.3 s.
        MoveLDelta(dx, dy, dz, ...): by dx dy dz [m] in the body frame from where it is commanded now (not the measured
        position, so a sagging arm does not add up), and by `rpy` = (droll, dpitch, dyaw) [rad] - added to the roll pitch
        yaw rt/rbq/arm_ee reports - when given. `seconds`, `vel` ... as in MoveL. Needs ControlStart()."""
        if len(args) == 2:
            return self._CallJson(ARM_API_ID_MOVELDELTA, {"axis": args[0], "value": args[1]})
        dx, dy, dz = args
        p = {"x": dx, "y": dy, "z": dz, "relative": True}
        if rpy is not None:
            p.update({"roll": rpy[0], "pitch": rpy[1], "yaw": rpy[2]})
        p.update({"time": seconds} if seconds > 0 else {"vel": vel, "acc": acc, "ang_vel": ang_vel, "ang_acc": ang_acc})
        return self._CallJson(ARM_API_ID_MOVEL, p)

    def RotateLDelta(self, *args, seconds: float = 0.0, ang_vel: float = 0.0, ang_acc: float = 0.0) -> int:
        """RotateLDelta(axis, radians): turn the end effector by `radians` (|radians| <= 0.26, 15 deg) about roll/pitch/yaw, in 0.3 s.
        RotateLDelta(droll, dpitch, dyaw, ...): by these [rad], added to the roll pitch yaw rt/rbq/arm_ee reports; the
        position stays. `seconds`, `ang_vel`, `ang_acc` as in RotateL. Needs ControlStart()."""
        if len(args) == 2:
            return self._CallJson(ARM_API_ID_ROTATELDELTA, {"axis": args[0], "value": args[1]})
        droll, dpitch, dyaw = args
        p = {"roll": droll, "pitch": dpitch, "yaw": dyaw, "relative": True}
        p.update({"time": seconds} if seconds > 0 else {"ang_vel": ang_vel, "ang_acc": ang_acc})
        return self._CallJson(ARM_API_ID_MOVEL, p)

    def GripperOpen(self) -> int:
        return self._CallJson(ARM_API_ID_GRIPPEROPEN, {})

    def GripperClose(self) -> int:
        return self._CallJson(ARM_API_ID_GRIPPERCLOSE, {})

    def GripperStop(self) -> int:
        return self._CallJson(ARM_API_ID_GRIPPERSTOP, {})

    def ReleaseJoints(self) -> int:
        return self._CallJson(ARM_API_ID_RELEASEJOINTS, {})

    def Stop(self) -> int:
        """Stops the running MoveJ / MoveL where the arm is. Always accepted; a Wait() on that motion returns
        ARM_ERR_CANCELED."""
        return self._CallJson(ARM_API_ID_STOP, {})

    def EmergencyStop(self) -> int:
        """Damps the arm (kp 0) until ClearEmergencyStop() or AutoStart(). Always accepted, also while an app holds the joints."""
        return self._CallJson(ARM_API_ID_EMERGENCYSTOP, {})

    def ClearEmergencyStop(self) -> int:
        """Ends the emergency stop: the arm holds the pose it sank to. MoveJ "Ready" before ControlStart() again."""
        return self._CallJson(ARM_API_ID_CLEAREMERGENCYSTOP, {})

    def SetGains(self, kp, kd) -> int:
        """Joint gains the arm uses for MoveJ, MoveL and high-level control: six kp [Nm/rad] and six kd [Nm*s/rad], J0..J5.
        Accepted range per joint: kp 10 % - 150 % and kd 10 % - 200 % of the default (GetState kp_default / kd_default);
        anything outside, or not finite, refuses the whole call (RPC_ERR_SERVER_API_PARAMETER). Takes effect over 0.5 - 2 s,
        also while the arm moves, and stays until ResetGains() or AutoStart(). Folding and other soft poses keep their own
        low gains. Refused while an app holds the joints (Lv0 - set gains there with JointStream.set_gains)."""
        return self._CallJson(ARM_API_ID_SETGAINS, {"kp": [float(v) for v in kp], "kd": [float(v) for v in kd]})

    def ResetGains(self) -> int:
        """Back to the default gains."""
        return self._CallJson(ARM_API_ID_SETGAINS, {"default": True})

    def Wait(self, seconds: float = 15.0) -> int:
        """Waits until the last accepted MoveJ / MoveL / MoveLDelta / RotateLDelta / ControlStart has finished.
        Returns 0 when it finished (or nothing was running), ARM_ERR_CANCELED when Stop() ended it,
        ARM_ERR_COLLISION when the collision torque limit stopped it (GetState collision_joint names the joint),
        ARM_ERR_PATH_BLOCKED when a joint move did not start because its path would hit the robot body or legs,
        ARM_ERR_WORKSPACE when a MoveL stopped at the gripper workspace limit, ARM_ERR_ESTOP when an emergency stop ended it,
        ARM_ERR_WAIT_TIMEOUT when it is still moving after `seconds`, or the GetState error."""
        end = time.monotonic() + seconds
        while True:
            code, state = self.GetState()
            if code != 0:
                return code
            result = (state or {}).get("result")
            if result == "canceled":
                return ARM_ERR_CANCELED
            if result == "collision":
                return ARM_ERR_COLLISION
            if result == "blocked":
                return ARM_ERR_PATH_BLOCKED
            if result == "workspace":
                return ARM_ERR_WORKSPACE
            if result == "estop":
                return ARM_ERR_ESTOP
            if result != "running":
                return 0
            if time.monotonic() >= end:
                return ARM_ERR_WAIT_TIMEOUT
            time.sleep(0.05)

    def GetState(self) -> Tuple[int, Optional[Dict[str, str]]]:
        """Arm status as strings: arm_type, can_check, con_start, task_control, high_level, estop (1 in emergency stop),
        motion (moving / idle), result (none / running / done / canceled / collision / blocked / workspace / estop - of the
        last motion call), motion_time (seconds of the last MoveJ by angles / MoveL), collision_joint, pose, joint_owner,
        sdk_slot, gains (default / custom), kp / kd (SetGains values, J0..J5 comma-separated) and kp_default / kd_default."""
        code, data = self._Call(ARM_API_ID_GETSTATE, json.dumps({}))
        if code != 0:
            return code, None
        try:
            return code, json.loads(data) if data else {}
        except Exception:
            return code, None
