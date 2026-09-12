"""IDL dataclasses required by rbq_high_level.py / rbq_low_level.py.

These mirror the C++ headers under rbq_sdk/cpp/rbq_sdk_cpp/idl/. The
existing rbq_sdk_py.idl package only ships unitree_go / rbq_api / std_msgs /
sensor_msgs / geometry_msgs subsets, so the rbq_msgs structs (HighLevelCommand_,
MotionRef_, JointOwnershipCmd_, LegJointInfo_, JointInfo_, JointRef_) and the
ros2 helpers (std_msgs/Bool_, sensor_msgs/Imu_, sensor_msgs/Joy_) need to be
defined here.

Only the typename strings need to match the C++ side for wire compatibility —
the on-robot daemon dispatches purely on those topic-type names.
"""

from dataclasses import dataclass, field

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

from rbq_sdk_py.idl.builtin_interfaces.msg.dds_ import Time_
from rbq_sdk_py.idl.std_msgs.msg.dds_ import Header_
from rbq_sdk_py.idl.geometry_msgs.msg.dds_ import Quaternion_, Vector3_


MAX_LEG_JOINT = 12
MAX_ARM_JOINT = 8
MAX_WHL_JOINT = 4


def _default_header() -> Header_:
    return Header_(stamp=Time_(sec=0, nanosec=0), frame_id="")


def _default_quat() -> Quaternion_:
    return Quaternion_(x=0.0, y=0.0, z=0.0, w=1.0)


def _default_vec3() -> Vector3_:
    return Vector3_(x=0.0, y=0.0, z=0.0)


# -----------------------------------------------------------------------------
# std_msgs / sensor_msgs additions
# -----------------------------------------------------------------------------


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Bool_(idl.IdlStruct, typename="std_msgs.msg.dds_.Bool_"):
    data: bool = False


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Imu_(idl.IdlStruct, typename="sensor_msgs.msg.dds_.Imu_"):
    header: Header_ = field(default_factory=_default_header)
    orientation: Quaternion_ = field(default_factory=_default_quat)
    orientation_covariance: types.array[types.float64, 9] = field(
        default_factory=lambda: [0.0] * 9
    )
    angular_velocity: Vector3_ = field(default_factory=_default_vec3)
    angular_velocity_covariance: types.array[types.float64, 9] = field(
        default_factory=lambda: [0.0] * 9
    )
    linear_acceleration: Vector3_ = field(default_factory=_default_vec3)
    linear_acceleration_covariance: types.array[types.float64, 9] = field(
        default_factory=lambda: [0.0] * 9
    )


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Joy_(idl.IdlStruct, typename="sensor_msgs.msg.dds_.Joy_"):
    header: Header_ = field(default_factory=_default_header)
    axes: types.sequence[types.float32] = field(default_factory=list)
    buttons: types.sequence[types.int32] = field(default_factory=list)


# -----------------------------------------------------------------------------
# rbq_msgs
# -----------------------------------------------------------------------------


@dataclass
@annotate.final
@annotate.autoid("sequential")
class HighLevelCommand_(
    idl.IdlStruct, typename="rbq_msgs.msg.dds_.HighLevelCommand_"
):
    header: Header_ = field(default_factory=_default_header)
    identifier: str = ""
    roll: types.float32 = 0.0
    pitch: types.float32 = 0.0
    yaw: types.float32 = 0.0
    vel_x: types.float32 = 0.0
    vel_y: types.float32 = 0.0
    omega_z: types.float32 = 0.0
    delta_body_h: types.float32 = 0.0
    delta_foot_h: types.float32 = 0.0
    gait_state: types.int8 = 0
    gait_transition: bool = False


@dataclass
@annotate.appendable
@annotate.autoid("sequential")
class JointRef_(idl.IdlStruct, typename="rbq_msgs.msg.dds_.JointRef_"):
    pos: types.float32 = 0.0
    torque: types.float32 = 0.0
    kp: types.float32 = 0.0
    kd: types.float32 = 0.0


@dataclass
@annotate.appendable
@annotate.autoid("sequential")
class JointInfo_(idl.IdlStruct, typename="rbq_msgs.msg.dds_.JointInfo_"):
    pos: types.float32 = 0.0
    vel: types.float32 = 0.0
    acc: types.float32 = 0.0
    torque: types.float32 = 0.0
    kp: types.float32 = 0.0
    kd: types.float32 = 0.0
    owner: types.int8 = 0


@dataclass
@annotate.appendable
@annotate.autoid("sequential")
class MotionRef_(idl.IdlStruct, typename="rbq_msgs.msg.dds_.MotionRef_"):
    time: types.float64 = 0.0
    leg_joint: types.array[JointRef_, MAX_LEG_JOINT] = field(
        default_factory=lambda: [JointRef_() for _ in range(MAX_LEG_JOINT)]
    )
    arm_joint: types.array[JointRef_, MAX_ARM_JOINT] = field(
        default_factory=lambda: [JointRef_() for _ in range(MAX_ARM_JOINT)]
    )
    whl_joint: types.array[JointRef_, MAX_WHL_JOINT] = field(
        default_factory=lambda: [JointRef_() for _ in range(MAX_WHL_JOINT)]
    )


@dataclass
@annotate.appendable
@annotate.autoid("sequential")
class JointOwnershipCmd_(
    idl.IdlStruct, typename="rbq_msgs.msg.dds_.JointOwnershipCmd_"
):
    leg_joint: types.array[bool, MAX_LEG_JOINT] = field(
        default_factory=lambda: [False] * MAX_LEG_JOINT
    )
    whl_joint: types.array[bool, MAX_WHL_JOINT] = field(
        default_factory=lambda: [False] * MAX_WHL_JOINT
    )
    arm_joint: types.array[bool, MAX_ARM_JOINT] = field(
        default_factory=lambda: [False] * MAX_ARM_JOINT
    )


@dataclass
@annotate.appendable
@annotate.autoid("sequential")
class LegJointInfo_(
    idl.IdlStruct, typename="rbq_msgs.msg.dds_.LegJointInfo_"
):
    joint: types.array[JointInfo_, MAX_LEG_JOINT] = field(
        default_factory=lambda: [JointInfo_() for _ in range(MAX_LEG_JOINT)]
    )


@dataclass
@annotate.final
@annotate.autoid("sequential")
class TargetGo_(idl.IdlStruct, typename="rbq_msgs.msg.dds_.TargetGo_"):
    gait: types.int32 = 0
    mode: types.int32 = 0
    x: types.float32 = 0.0
    y: types.float32 = 0.0
    theta: types.float32 = 0.0
    offset_x: types.float32 = 0.0
    offset_y: types.float32 = 0.0
    slow: bool = False
    wide: bool = False
    vision: bool = False
