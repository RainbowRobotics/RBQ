import json
import warnings
from typing import Dict, Optional, Tuple

from ..rpc.client import Client
from .sport_api import *


def _deprecated(name: str):
    warnings.warn(f"SportClient.{name} is deprecated and will be removed in the next release: "
                  "the robot has no such motion (it answers 3203).", FutureWarning, stacklevel=3)


class SportClient(Client):
    def __init__(self, enableLease: bool = False):
        super().__init__(SPORT_SERVICE_NAME, enableLease)

    def Init(self):
        self._SetApiVerson(SPORT_API_VERSION)
        self._RegistApi(SPORT_API_ID_DAMP, 0)
        self._RegistApi(SPORT_API_ID_BALANCESTAND, 0)
        self._RegistApi(SPORT_API_ID_STOPMOVE, 0)
        self._RegistApi(SPORT_API_ID_STANDUP, 0)
        self._RegistApi(SPORT_API_ID_STANDDOWN, 0)
        self._RegistApi(SPORT_API_ID_RECOVERYSTAND, 0)
        self._RegistApi(SPORT_API_ID_EULER, 0)
        self._RegistApi(SPORT_API_ID_MOVE, 0)
        self._RegistApi(SPORT_API_ID_BODYPOSITION, 0)
        self._RegistApi(SPORT_API_ID_SWITCHGAIT, 0)
        self._RegistApi(SPORT_API_ID_BODYHEIGHT, 0)
        self._RegistApi(SPORT_API_ID_SPEEDLEVEL, 0)
        self._RegistApi(SPORT_API_ID_LEFTSIDEGAIT, 0)
        self._RegistApi(SPORT_API_ID_RIGHTSIDEGAIT, 0)
        self._RegistApi(SPORT_API_ID_HANDSTAND, 0)
        self._RegistApi(SPORT_API_ID_GETSTATE, 0)
        for apiId in (SPORT_API_ID_BIPEDSTAND, SPORT_API_ID_FRONTFLIP, SPORT_API_ID_BACKFLIP,
                      SPORT_API_ID_SETAUTORECOVERY):   # deprecated, see below
            self._RegistApi(apiId, 0)

    def Damp(self) -> int:
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(SPORT_API_ID_DAMP, parameter)
        return code

    def BalanceStand(self) -> int:
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(SPORT_API_ID_BALANCESTAND, parameter)
        return code

    def StopMove(self) -> int:
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(SPORT_API_ID_STOPMOVE, parameter)
        return code

    def StandUp(self) -> int:
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(SPORT_API_ID_STANDUP, parameter)
        return code

    def StandDown(self) -> int:
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(SPORT_API_ID_STANDDOWN, parameter)
        return code

    def RecoveryStand(self) -> int:
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(SPORT_API_ID_RECOVERYSTAND, parameter)
        return code

    def Euler(self, roll: float, pitch: float, yaw: float) -> int:
        p = {"x": roll, "y": pitch, "z": yaw}
        parameter = json.dumps(p)
        code, data = self._Call(SPORT_API_ID_EULER, parameter)
        return code

    # Streaming command — fire-and-forget so high-rate Move publishes don't
    # block on a per-call ack from the robot (200 ms RTT over WiFi would
    # collapse any control loop).  Server interpolates between targets.
    def Move(self, vx: float, vy: float, vyaw: float) -> int:
        p = {"x": vx, "y": vy, "z": vyaw}
        parameter = json.dumps(p)
        code = self._CallNoReply(SPORT_API_ID_MOVE, parameter)
        return code

    # The robot's answer to the latest Move() sent within the last second (0 if none yet). Move() does not
    # wait for it, so this is where a refusal shows: 3205 control mode off, 3204 bad value.
    def LastMoveCode(self) -> int:
        return self._NoReplyCode(SPORT_API_ID_MOVE)

    def BodyPosition(self, x: float, y: float, z: float, yaw: float) -> int:
        p = {"x": x, "y": y, "z": z, "yaw": yaw}
        parameter = json.dumps(p)
        code, data = self._Call(SPORT_API_ID_BODYPOSITION, parameter)
        return code

    def SwitchGait(self, gait_type: int) -> int:
        p = {"data": gait_type}
        parameter = json.dumps(p)
        code, data = self._Call(SPORT_API_ID_SWITCHGAIT, parameter)
        return code

    def BodyHeight(self, height: float) -> int:
        p = {"data": height}
        parameter = json.dumps(p)
        code, data = self._Call(SPORT_API_ID_BODYHEIGHT, parameter)
        return code

    def SpeedLevel(self, level: int) -> int:
        p = {"data": level}
        parameter = json.dumps(p)
        code, data = self._Call(SPORT_API_ID_SPEEDLEVEL, parameter)
        return code

    def LeftSideGait(self, enter: int) -> int:
        p = {"data": enter}
        parameter = json.dumps(p)
        code, data = self._Call(SPORT_API_ID_LEFTSIDEGAIT, parameter)
        return code

    def RightSideGait(self, enter: int) -> int:
        p = {"data": enter}
        parameter = json.dumps(p)
        code, data = self._Call(SPORT_API_ID_RIGHTSIDEGAIT, parameter)
        return code

    def HandStand(self, enter: int) -> int:
        p = {"data": enter}
        parameter = json.dumps(p)
        code, data = self._Call(SPORT_API_ID_HANDSTAND, parameter)
        return code

    # Deprecated (FutureWarning) — the robot answers 3203. Removed in the next release.
    def BipedStand(self, enter: int) -> int:
        _deprecated("BipedStand")
        code, data = self._Call(SPORT_API_ID_BIPEDSTAND, json.dumps({"data": enter}))
        return code

    def FrontFlip(self) -> int:
        _deprecated("FrontFlip")
        code, data = self._Call(SPORT_API_ID_FRONTFLIP, json.dumps({}))
        return code

    def BackFlip(self) -> int:
        _deprecated("BackFlip")
        code, data = self._Call(SPORT_API_ID_BACKFLIP, json.dumps({}))
        return code

    def SetAutoRecovery(self, switch_on: int) -> int:
        _deprecated("SetAutoRecovery")
        code, data = self._Call(SPORT_API_ID_SETAUTORECOVERY, json.dumps({"data": switch_on}))
        return code

    def GetState(self) -> Tuple[int, Optional[Dict[str, str]]]:
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(SPORT_API_ID_GETSTATE, parameter)
        if code != 0:
            return code, None
        try:
            return code, json.loads(data) if data else {}
        except Exception:
            return code, None
