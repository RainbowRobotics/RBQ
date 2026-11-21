# Mirrors rbq_sdk/cpp/rbq_sdk_cpp/include/dds/ArmClient.hpp — RBA-1 arm service

# service identity (rt/api/arm/request, rt/api/arm/response)
ARM_SERVICE_NAME = "arm"
ARM_API_VERSION  = "1.0.0.0"

# api ids
ARM_API_ID_AUTOSTART      = 7001  # power-up sequence (axis init + reference streaming)
ARM_API_ID_CONTROLSTART   = 7002  # task-space control on — needed by the MoveL* calls
ARM_API_ID_MOVEJ          = 7003  # joint-space move to a named pose
ARM_API_ID_MOVEL          = 7004  # end effector to a position, in a given time
ARM_API_ID_MOVELDELTA     = 7005  # end effector moved along one axis
ARM_API_ID_ROTATELDELTA   = 7006  # end effector turned about one axis
ARM_API_ID_GRIPPEROPEN    = 7007
ARM_API_ID_GRIPPERCLOSE   = 7008
ARM_API_ID_GRIPPERSTOP    = 7009
ARM_API_ID_RELEASEJOINTS  = 7010  # give every SDK-held arm joint back to the arm controller
ARM_API_ID_GETSTATE       = 7011
ARM_API_ID_STOP           = 7012  # stop the running MoveJ / MoveL where the arm is
ARM_API_ID_EMERGENCYSTOP      = 7013  # arm joints damped (kp 0) until cleared
ARM_API_ID_CLEAREMERGENCYSTOP = 7014  # end the emergency stop - the arm holds where it is
ARM_API_ID_SETGAINS           = 7015  # joint gains for MoveJ / MoveL / high-level control

# arm-specific reply codes, next to the SportClient ones (0 ok, 3204 bad parameter, 3205 rejected, 3104 no answer)
ARM_ERR_BUSY         = 7101  # the arm is still moving (or a command is still pending) - Wait() first
ARM_ERR_NOT_READY    = 7102  # MoveL family without task-space control - ControlStart() from Ready first
ARM_ERR_CANCELED     = 7103  # Wait(): the motion was stopped by Stop()
ARM_ERR_WAIT_TIMEOUT = 7104  # Wait(): still moving when the time ran out
ARM_ERR_COLLISION    = 7105  # Wait(): a joint move pushed past the collision torque limit and stopped
ARM_ERR_PATH_BLOCKED = 7106  # Wait(): a joint move did not start - its path would hit the robot body or legs
ARM_ERR_WORKSPACE    = 7107  # Wait(): a MoveL stopped at the gripper workspace limit
ARM_ERR_ESTOP        = 7108  # the arm is in emergency stop - ClearEmergencyStop() (or AutoStart()) first; Wait(): stopped by it
ARM_ERR_TOO_FAST     = 7109  # the given time is faster than the max vel / acc - GetState motion_time = the shortest time

# MoveJ names — "Ready" is the start pose for task-space control
ARM_POSE_STRAIGHT          = "Straight"
ARM_POSE_FOLDING           = "Folding"
ARM_POSE_READY             = "Ready"
ARM_POSE_LIFT              = "Lift"
ARM_POSE_PACKING           = "Packing"
ARM_POSE_PICKUP_READY      = "PickUpReady"
ARM_POSE_TASK_MOTION_READY = "TaskMotionReady"

# axis ids for MoveLDelta / RotateLDelta
ARM_AXIS_X = 1  # roll for RotateLDelta
ARM_AXIS_Y = 2  # pitch
ARM_AXIS_Z = 3  # yaw
