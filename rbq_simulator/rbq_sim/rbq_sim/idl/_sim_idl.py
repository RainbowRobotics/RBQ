"""Python IDL dataclasses for the rbq_sim <-> Motion wire contract (CycloneDDS).

Self-contained mirror of the C++ IDL under rbq_sdk/cpp/rbq_sdk_cpp/idl/. The installed
rbq_sdk_py.idl package ships only std_msgs / sensor_msgs / geometry_msgs / builtin_interfaces
subsets (NOT the rbq_msgs structs), and rbq_sdk's example _rbq_idl.py is not an importable
package — so the types we need (MotionRef_/JointRef_/JointInfo_/Imu_ + the new
SimInfo_/TaskInfo_/ContactInfo_) are defined here, importing only primitives from rbq_sdk_py.idl.*.

Wire compatibility (verified against rbq_sdk/cpp/rbq_sdk_cpp/idl/rbq/*.idl + ros2/Imu_.idl):
  - typename strings, field order, types and @extensibility must match the C++ side exactly,
    or DDS type-hash matching fails ("incompatible QoS, INVALID" — actually IDL drift).
  - rbq_msgs structs and sensor_msgs Imu_ are all @extensibility(FINAL).
  - float -> float32, double -> float64, int8 -> int8, boolean -> bool, string<N> -> bounded_str[N].
    Arrays -> types.array.
"""

from dataclasses import dataclass, field

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

from rbq_sdk_py.idl.builtin_interfaces.msg.dds_ import Time_
from rbq_sdk_py.idl.geometry_msgs.msg.dds_ import Quaternion_, Vector3_
from rbq_sdk_py.idl.std_msgs.msg.dds_ import Header_

# rbq_msgs/RbqConstants_.idl
MAX_LEG_JOINT = 12
MAX_ARM_JOINT = 8
MAX_WHL_JOINT = 4
MAX_LEG_CONTACT = 4


def _default_header() -> Header_:
    return Header_(stamp=Time_(sec=0, nanosec=0), frame_id="")


def _default_quat() -> Quaternion_:
    return Quaternion_(x=0.0, y=0.0, z=0.0, w=1.0)


def _default_vec3() -> Vector3_:
    return Vector3_(x=0.0, y=0.0, z=0.0)


# -----------------------------------------------------------------------------
# sensor_msgs::msg::dds_::Imu_  (ros2/Imu_.idl, FINAL)
# -----------------------------------------------------------------------------


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Imu_(idl.IdlStruct, typename="sensor_msgs.msg.dds_.Imu_"):
    header: Header_ = field(default_factory=_default_header)
    orientation: Quaternion_ = field(default_factory=_default_quat)
    orientation_covariance: types.array[types.float64, 9] = field(default_factory=lambda: [0.0] * 9)
    angular_velocity: Vector3_ = field(default_factory=_default_vec3)
    angular_velocity_covariance: types.array[types.float64, 9] = field(default_factory=lambda: [0.0] * 9)
    linear_acceleration: Vector3_ = field(default_factory=_default_vec3)
    linear_acceleration_covariance: types.array[types.float64, 9] = field(default_factory=lambda: [0.0] * 9)


# -----------------------------------------------------------------------------
# rbq_msgs  (rbq/*.idl, FINAL)
# -----------------------------------------------------------------------------


@dataclass
@annotate.final
@annotate.autoid("sequential")
class JointRef_(idl.IdlStruct, typename="rbq_msgs.msg.dds_.JointRef_"):
    pos: types.float32 = 0.0
    vel: types.float32 = 0.0  # rad/s, CAN FD robots only (the sim is CAN FD)
    torque: types.float32 = 0.0
    kp: types.float32 = 0.0
    kd: types.float32 = 0.0


@dataclass
@annotate.final
@annotate.autoid("sequential")
class JointInfo_(idl.IdlStruct, typename="rbq_msgs.msg.dds_.JointInfo_"):
    pos: types.float32 = 0.0
    vel: types.float32 = 0.0
    acc: types.float32 = 0.0
    torque: types.float32 = 0.0
    kp: types.float32 = 0.0
    kd: types.float32 = 0.0
    owner: types.int8 = 0
    ref_ff_torque: types.float32 = 0.0
    ref_position: types.float32 = 0.0
    ref_vel: types.float32 = 0.0
    current: types.float32 = 0.0
    temperature_board: types.int8 = 0
    temperature_coil: types.int8 = 0
    name: types.bounded_str[3] = ""
    status: types.bounded_str[3] = ""
    error_msg: types.bounded_str[3] = ""


@dataclass
@annotate.final
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
@annotate.final
@annotate.autoid("sequential")
class TaskInfo_(idl.IdlStruct, typename="rbq_msgs.msg.dds_.TaskInfo_"):
    pos: types.array[types.float32, 3] = field(default_factory=lambda: [0.0] * 3)
    vel: types.array[types.float32, 3] = field(default_factory=lambda: [0.0] * 3)
    acc: types.array[types.float32, 3] = field(default_factory=lambda: [0.0] * 3)
    force: types.array[types.float32, 3] = field(default_factory=lambda: [0.0] * 3)
    quat: types.array[types.float32, 4] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    rot: types.array[types.float32, 9] = field(default_factory=lambda: [0.0] * 9)
    rpy: types.array[types.float32, 3] = field(default_factory=lambda: [0.0] * 3)
    omega: types.array[types.float32, 3] = field(default_factory=lambda: [0.0] * 3)
    jacobian: types.array[types.float32, 9] = field(default_factory=lambda: [0.0] * 9)
    kp: types.array[types.float32, 3] = field(default_factory=lambda: [0.0] * 3)
    kd: types.array[types.float32, 3] = field(default_factory=lambda: [0.0] * 3)


@dataclass
@annotate.final
@annotate.autoid("sequential")
class ContactInfo_(idl.IdlStruct, typename="rbq_msgs.msg.dds_.ContactInfo_"):
    force: types.array[types.float32, 3] = field(default_factory=lambda: [0.0] * 3)
    torque: types.array[types.float32, 3] = field(default_factory=lambda: [0.0] * 3)
    active: bool = False


@dataclass
@annotate.final
@annotate.autoid("sequential")
class SimInfo_(idl.IdlStruct, typename="rbq_msgs.msg.dds_.SimInfo_"):
    imu: Imu_ = field(default_factory=Imu_)
    leg_joint: types.array[JointInfo_, MAX_LEG_JOINT] = field(
        default_factory=lambda: [JointInfo_() for _ in range(MAX_LEG_JOINT)]
    )
    arm_joint: types.array[JointInfo_, MAX_ARM_JOINT] = field(
        default_factory=lambda: [JointInfo_() for _ in range(MAX_ARM_JOINT)]
    )
    whl_joint: types.array[JointInfo_, MAX_WHL_JOINT] = field(
        default_factory=lambda: [JointInfo_() for _ in range(MAX_WHL_JOINT)]
    )
    body_task: TaskInfo_ = field(default_factory=TaskInfo_)
    leg_contact: types.array[ContactInfo_, MAX_LEG_CONTACT] = field(
        default_factory=lambda: [ContactInfo_() for _ in range(MAX_LEG_CONTACT)]
    )
