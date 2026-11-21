#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <thread>

#include <rbq_sdk/nlohmann/json.hpp>
#include <rbq_sdk/dds/Client.hpp>

namespace rbq_sdk {

// RBA-1 arm service — same request/response channel and error codes as SportClient
// (rt/api/arm/request, rt/api/arm/response).
const std::string ROBOT_ARM_SERVICE_NAME = "arm";
const std::string ROBOT_ARM_API_VERSION  = "1.0.0.0";

// api ids
constexpr int32_t ROBOT_ARM_API_ID_AUTOSTART       = 7001;  // power-up sequence (axis init + reference streaming)
constexpr int32_t ROBOT_ARM_API_ID_CONTROLSTART    = 7002;  // task-space control on — needed by the MoveL* calls
constexpr int32_t ROBOT_ARM_API_ID_MOVEJ           = 7003;  // joint-space move to a named pose or to six joint angles
constexpr int32_t ROBOT_ARM_API_ID_MOVEL           = 7004;  // end effector to a position and/or orientation, body frame
constexpr int32_t ROBOT_ARM_API_ID_MOVELDELTA      = 7005;  // end effector moved along one axis
constexpr int32_t ROBOT_ARM_API_ID_ROTATELDELTA    = 7006;  // end effector turned about one axis
constexpr int32_t ROBOT_ARM_API_ID_GRIPPEROPEN     = 7007;
constexpr int32_t ROBOT_ARM_API_ID_GRIPPERCLOSE    = 7008;
constexpr int32_t ROBOT_ARM_API_ID_GRIPPERSTOP     = 7009;
constexpr int32_t ROBOT_ARM_API_ID_RELEASEJOINTS   = 7010;  // give every SDK-held arm joint back to the arm controller
constexpr int32_t ROBOT_ARM_API_ID_GETSTATE        = 7011;
constexpr int32_t ROBOT_ARM_API_ID_STOP            = 7012;  // stop the running MoveJ / MoveL where the arm is
constexpr int32_t ROBOT_ARM_API_ID_EMERGENCYSTOP      = 7013;  // arm joints damped (kp 0) until cleared
constexpr int32_t ROBOT_ARM_API_ID_CLEAREMERGENCYSTOP = 7014;  // end the emergency stop - the arm holds where it is
constexpr int32_t ROBOT_ARM_API_ID_SETGAINS           = 7015;  // joint gains for MoveJ / MoveL / high-level control

// Arm-specific reply codes, next to the SportClient ones (0 ok, 3204 bad parameter, 3205 rejected, 3104 no answer)
constexpr int32_t ARM_ERR_BUSY         = 7101;  // the arm is still moving (or a command is still pending) - Wait() first
constexpr int32_t ARM_ERR_NOT_READY    = 7102;  // MoveL family without task-space control - ControlStart() from Ready first
constexpr int32_t ARM_ERR_CANCELED     = 7103;  // Wait(): the motion was stopped by Stop()
constexpr int32_t ARM_ERR_WAIT_TIMEOUT = 7104;  // Wait(): still moving when the time ran out
constexpr int32_t ARM_ERR_COLLISION    = 7105;  // Wait(): a joint move pushed past the collision torque limit and stopped
constexpr int32_t ARM_ERR_PATH_BLOCKED = 7106;  // Wait(): a joint move did not start - its path would hit the robot body or legs
constexpr int32_t ARM_ERR_WORKSPACE    = 7107;  // Wait(): a MoveL stopped at the gripper workspace limit
constexpr int32_t ARM_ERR_ESTOP        = 7108;  // the arm is in emergency stop - ClearEmergencyStop() (or AutoStart()) first; Wait(): stopped by it
constexpr int32_t ARM_ERR_TOO_FAST     = 7109;  // the given time would move the arm faster than the max vel / acc - GetState motion_time
                                                //   holds the shortest time this move needs

// MoveJ names — the arm's built-in joint-space poses. "Ready" is the start pose for task-space control.
const char* const ARM_POSE_STRAIGHT          = "Straight";
const char* const ARM_POSE_FOLDING           = "Folding";
const char* const ARM_POSE_READY             = "Ready";
const char* const ARM_POSE_LIFT              = "Lift";
const char* const ARM_POSE_PACKING           = "Packing";
const char* const ARM_POSE_PICKUP_READY      = "PickUpReady";
const char* const ARM_POSE_TASK_MOTION_READY = "TaskMotionReady";

// Speed of a MoveJ / MoveL given as limits instead of a time: the robot picks the shortest time that keeps the move under
// all of them (at least 0.5 s). 0 = the default. A value above the maximum is refused (3204). GetState motion_time shows
// the time it picked. A time given instead must also keep the move under the maximums (joints 1.57 rad/s, 3.14 rad/s²;
// end effector 0.25 m/s, 0.5 m/s², 1.57 rad/s, 3.14 rad/s²) - a shorter one is refused with ARM_ERR_TOO_FAST and
// GetState motion_time holds the shortest time that move needs. Provisional until the actuator specifications are final.
// Angles are in radians throughout, as in SportClient and the arm topics.
struct ArmJointSpeed {
    float vel = 0.f;      ///< fastest joint [rad/s]      default 0.52 (30°/s), max 1.57 (90°/s)
    float acc = 0.f;      ///< fastest joint [rad/s^2]    default 1.05 (60°/s²), max 3.14 (180°/s²)
};
struct ArmTaskSpeed {
    float vel = 0.f;      ///< end effector [m/s]         default 0.1, max 0.25
    float acc = 0.f;      ///< end effector [m/s^2]       default 0.2, max 0.5
    float angVel = 0.f;   ///< orientation [rad/s]        default 0.52, max 1.57
    float angAcc = 0.f;   ///< orientation [rad/s^2]      default 1.05, max 3.14
};

// Axis ids for MoveLDelta / RotateLDelta
constexpr int ARM_AXIS_X = 1;   // roll for RotateLDelta
constexpr int ARM_AXIS_Y = 2;   // pitch
constexpr int ARM_AXIS_Z = 3;   // yaw

// Default behavior matches SportClient: every call blocks until the robot answers (1 s timeout).
// SetNoReply(true) makes the void-returning calls fire-and-forget; GetState always blocks.
//
// A motion call returns once the robot accepts it, not when the arm arrives - Wait() for that, Stop() to end it.
// While the arm moves, the next motion call is refused with ARM_ERR_BUSY.
// The motion calls are refused with 3205 while an app holds arm joints through the joint-level
// (Lv0) channel — call ReleaseJoints() or release them from that app first. The gripper calls are
// not: the gripper is not a joint-level joint, so a joint-level app can open and close it.
//
// EmergencyStop() damps the arm: every arm joint gets kp 0 and a damping gain, so the arm sinks slowly under gravity and
// gives way when it is pushed. The robot's E-stop (app E-STOP, SportClient::Damp, rt/rbq/cmd/emergency) does the same to the
// arm. It stays on until ClearEmergencyStop() or AutoStart(); until then the motion calls are refused with ARM_ERR_ESTOP and
// joint-level ownership is not given. The gripper calls still work.
class ArmClient : public Client {
public:
    ArmClient() : Client(ROBOT_ARM_SERVICE_NAME, /*enable_lease=*/false) {
        Init();
    }

    void Init() {
        SetApiVersion(ROBOT_ARM_API_VERSION);
        for (int32_t id : { ROBOT_ARM_API_ID_AUTOSTART,      ROBOT_ARM_API_ID_CONTROLSTART,
                            ROBOT_ARM_API_ID_MOVEJ,          ROBOT_ARM_API_ID_MOVEL,
                            ROBOT_ARM_API_ID_MOVELDELTA,     ROBOT_ARM_API_ID_ROTATELDELTA,
                            ROBOT_ARM_API_ID_GRIPPEROPEN,    ROBOT_ARM_API_ID_GRIPPERCLOSE,
                            ROBOT_ARM_API_ID_GRIPPERSTOP,    ROBOT_ARM_API_ID_RELEASEJOINTS,
                            ROBOT_ARM_API_ID_GETSTATE,       ROBOT_ARM_API_ID_STOP,
                            ROBOT_ARM_API_ID_EMERGENCYSTOP,  ROBOT_ARM_API_ID_CLEAREMERGENCYSTOP,
                            ROBOT_ARM_API_ID_SETGAINS })
            RegistApi(id);
    }

    int32_t AutoStart()     { return Call(ROBOT_ARM_API_ID_AUTOSTART,     std::string{}); }
    int32_t ControlStart()  { return Call(ROBOT_ARM_API_ID_CONTROLSTART,  std::string{}); }
    int32_t GripperOpen()   { return Call(ROBOT_ARM_API_ID_GRIPPEROPEN,   std::string{}); }
    int32_t GripperClose()  { return Call(ROBOT_ARM_API_ID_GRIPPERCLOSE,  std::string{}); }
    int32_t GripperStop()   { return Call(ROBOT_ARM_API_ID_GRIPPERSTOP,   std::string{}); }
    int32_t ReleaseJoints() { return Call(ROBOT_ARM_API_ID_RELEASEJOINTS, std::string{}); }
    /// Stops the running MoveJ / MoveL where the arm is. Always accepted; a Wait() on that motion returns ARM_ERR_CANCELED.
    int32_t Stop()          { return Call(ROBOT_ARM_API_ID_STOP,          std::string{}); }
    /// Damps the arm (kp 0) until ClearEmergencyStop() or AutoStart(). Always accepted, also while an app holds the joints.
    int32_t EmergencyStop()      { return Call(ROBOT_ARM_API_ID_EMERGENCYSTOP,      std::string{}); }
    /// Ends the emergency stop: the arm holds the pose it sank to. MoveJ "Ready" before ControlStart() again.
    int32_t ClearEmergencyStop() { return Call(ROBOT_ARM_API_ID_CLEAREMERGENCYSTOP, std::string{}); }

    /// Waits until the last accepted MoveJ / MoveJDelta / MoveL / RotateL / MoveLDelta / RotateLDelta / ControlStart has finished.
    /// Returns 0 when it finished (or nothing was running), ARM_ERR_CANCELED when Stop() ended it,
    /// ARM_ERR_COLLISION when the collision torque limit stopped it (GetState collision_joint names the joint),
    /// ARM_ERR_PATH_BLOCKED when a joint move did not start because its path would hit the robot body or legs,
    /// ARM_ERR_WORKSPACE when a MoveL stopped at the gripper workspace limit, ARM_ERR_ESTOP when an emergency stop ended it,
    /// ARM_ERR_WAIT_TIMEOUT when it is still moving after `seconds`, or the GetState error.
    int32_t Wait(float seconds = 15.0f) {
        const auto end = std::chrono::steady_clock::now() + std::chrono::duration<float>(seconds);
        while (true) {
            std::map<std::string, std::string> state;
            const int32_t code = GetState(state);
            if (code != RPC_OK) return code;
            if (state["result"] == "canceled") return ARM_ERR_CANCELED;
            if (state["result"] == "collision") return ARM_ERR_COLLISION;
            if (state["result"] == "blocked")   return ARM_ERR_PATH_BLOCKED;
            if (state["result"] == "workspace") return ARM_ERR_WORKSPACE;
            if (state["result"] == "estop")     return ARM_ERR_ESTOP;
            if (state["result"] != "running") return RPC_OK;
            if (std::chrono::steady_clock::now() >= end) return ARM_ERR_WAIT_TIMEOUT;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    /// name: one of the ARM_POSE_* names. The first move after AutoStart should be "Ready".
    int32_t MoveJ(const std::string& name) {
        return CallJson(ROBOT_ARM_API_ID_MOVEJ, { {"data", name} });
    }

    /// Six joint angles J0..J5 [rad], every joint arriving together in `seconds` (0.5 – 20 s). An angle outside the joint
    /// range is refused (3204). ControlStart() afterwards needs MoveJ("Ready") first.
    int32_t MoveJ(const std::array<float, 6>& q_rad, float seconds) {
        return CallJson(ROBOT_ARM_API_ID_MOVEJ, { {"q", q_rad}, {"time", seconds} });
    }

    /// Six joint angles J0..J5 [rad], as fast as `speed` allows.
    int32_t MoveJ(const std::array<float, 6>& q_rad, const ArmJointSpeed& speed) {
        return CallJson(ROBOT_ARM_API_ID_MOVEJ, { {"q", q_rad}, {"vel", speed.vel}, {"acc", speed.acc} });
    }

    /// One joint (0 = J0 .. 5 = J5) to `q_rad` [rad], in `seconds` (0.5 – 20 s). The other joints stay.
    int32_t MoveJ(int joint, float q_rad, float seconds) {
        return CallJson(ROBOT_ARM_API_ID_MOVEJ, { {"joint", joint}, {"q", q_rad}, {"time", seconds} });
    }

    /// One joint to `q_rad` [rad], as fast as `speed` allows.
    int32_t MoveJ(int joint, float q_rad, const ArmJointSpeed& speed) {
        return CallJson(ROBOT_ARM_API_ID_MOVEJ, { {"joint", joint}, {"q", q_rad}, {"vel", speed.vel}, {"acc", speed.acc} });
    }

    /// Every joint by `dq_rad` [rad] from where it is commanded now (not the measured angle, so a sagging arm does not
    /// add up), in `seconds` (0.5 – 20 s). A target outside the joint range is refused (3204).
    int32_t MoveJDelta(const std::array<float, 6>& dq_rad, float seconds) {
        return CallJson(ROBOT_ARM_API_ID_MOVEJ, { {"q", dq_rad}, {"relative", true}, {"time", seconds} });
    }

    /// Every joint by `dq_rad`, as fast as `speed` allows.
    int32_t MoveJDelta(const std::array<float, 6>& dq_rad, const ArmJointSpeed& speed) {
        return CallJson(ROBOT_ARM_API_ID_MOVEJ, { {"q", dq_rad}, {"relative", true}, {"vel", speed.vel}, {"acc", speed.acc} });
    }

    /// One joint (0 = J0 .. 5 = J5) by `dq_rad` [rad] from where it is commanded now, in `seconds` (0.5 – 20 s). The other
    /// joints stay.
    int32_t MoveJDelta(int joint, float dq_rad, float seconds) {
        return CallJson(ROBOT_ARM_API_ID_MOVEJ, { {"joint", joint}, {"q", dq_rad}, {"relative", true}, {"time", seconds} });
    }

    /// One joint by `dq_rad` [rad], as fast as `speed` allows.
    int32_t MoveJDelta(int joint, float dq_rad, const ArmJointSpeed& speed) {
        return CallJson(ROBOT_ARM_API_ID_MOVEJ,
                        { {"joint", joint}, {"q", dq_rad}, {"relative", true}, {"vel", speed.vel}, {"acc", speed.acc} });
    }

    /// End-effector position [m] in the body frame (the frame rt/rbq/arm_ee reports), reached in `seconds` (0.5 – 10 s).
    /// The orientation stays. Needs ControlStart().
    int32_t MoveL(float x, float y, float z, float seconds) {
        return CallJson(ROBOT_ARM_API_ID_MOVEL, { {"x", x}, {"y", y}, {"z", z}, {"time", seconds} });
    }

    /// The same, as fast as `speed` allows.
    int32_t MoveL(float x, float y, float z, const ArmTaskSpeed& speed) {
        nlohmann::json j = { {"x", x}, {"y", y}, {"z", z} };
        j.update(SpeedJson(speed));
        return CallJson(ROBOT_ARM_API_ID_MOVEL, j);
    }

    /// End-effector position [m] and orientation roll / pitch / yaw [rad] in the body frame - the same numbers rt/rbq/arm_ee
    /// reports (Z-Y-X: R = Rz(yaw) Ry(pitch) Rx(roll)), reached in `seconds` (0.5 – 10 s). Needs ControlStart().
    int32_t MoveL(float x, float y, float z, float roll, float pitch, float yaw, float seconds) {
        return CallJson(ROBOT_ARM_API_ID_MOVEL, { {"x", x}, {"y", y}, {"z", z},
                                                  {"roll", roll}, {"pitch", pitch}, {"yaw", yaw}, {"time", seconds} });
    }

    /// The same, as fast as `speed` allows.
    int32_t MoveL(float x, float y, float z, float roll, float pitch, float yaw, const ArmTaskSpeed& speed) {
        nlohmann::json j = { {"x", x}, {"y", y}, {"z", z}, {"roll", roll}, {"pitch", pitch}, {"yaw", yaw} };
        j.update(SpeedJson(speed));
        return CallJson(ROBOT_ARM_API_ID_MOVEL, j);
    }

    /// End-effector orientation roll pitch yaw [rad] in the body frame (Z-Y-X, as rt/rbq/arm_ee reports it), reached in
    /// `seconds` (0.5 – 10 s). The position stays. Needs ControlStart().
    int32_t RotateL(float roll, float pitch, float yaw, float seconds) {
        return CallJson(ROBOT_ARM_API_ID_MOVEL, { {"roll", roll}, {"pitch", pitch}, {"yaw", yaw}, {"time", seconds} });
    }

    /// End-effector orientation, as fast as `speed.angVel` / `speed.angAcc` allow.
    int32_t RotateL(float roll, float pitch, float yaw, const ArmTaskSpeed& speed) {
        nlohmann::json j = SpeedJson(speed);
        j.update({ {"roll", roll}, {"pitch", pitch}, {"yaw", yaw} });
        return CallJson(ROBOT_ARM_API_ID_MOVEL, j);
    }

    /// Move the end effector by `meters` (|meters| <= 0.05) along ARM_AXIS_X/Y/Z, in 0.3 s. Needs ControlStart().
    int32_t MoveLDelta(int axis, float meters) {
        return CallJson(ROBOT_ARM_API_ID_MOVELDELTA, { {"axis", axis}, {"value", meters} });
    }

    /// Turn the end effector by `radians` (|radians| <= 0.26, 15°) about roll/pitch/yaw (ARM_AXIS_X/Y/Z), in 0.3 s.
    int32_t RotateLDelta(int axis, float radians) {
        return CallJson(ROBOT_ARM_API_ID_ROTATELDELTA, { {"axis", axis}, {"value", radians} });
    }

    /// End effector by dx dy dz [m] in the body frame from where it is commanded now (not the measured position, so a
    /// sagging arm does not add up), in `seconds` (0.5 – 10 s). The orientation stays. Needs ControlStart().
    int32_t MoveLDelta(float dx, float dy, float dz, float seconds) {
        return CallJson(ROBOT_ARM_API_ID_MOVEL, { {"x", dx}, {"y", dy}, {"z", dz}, {"relative", true}, {"time", seconds} });
    }

    /// End effector by dx dy dz [m], as fast as `speed` allows.
    int32_t MoveLDelta(float dx, float dy, float dz, const ArmTaskSpeed& speed) {
        nlohmann::json j = SpeedJson(speed);
        j.update({ {"x", dx}, {"y", dy}, {"z", dz}, {"relative", true} });
        return CallJson(ROBOT_ARM_API_ID_MOVEL, j);
    }

    /// End effector by dx dy dz [m] and by droll dpitch dyaw [rad] - added to the roll pitch yaw rt/rbq/arm_ee reports -
    /// in `seconds` (0.5 – 10 s).
    int32_t MoveLDelta(float dx, float dy, float dz, float droll, float dpitch, float dyaw, float seconds) {
        return CallJson(ROBOT_ARM_API_ID_MOVEL, { {"x", dx}, {"y", dy}, {"z", dz}, {"roll", droll}, {"pitch", dpitch},
                                                  {"yaw", dyaw}, {"relative", true}, {"time", seconds} });
    }

    /// End effector by a position and an orientation, as fast as `speed` allows.
    int32_t MoveLDelta(float dx, float dy, float dz, float droll, float dpitch, float dyaw, const ArmTaskSpeed& speed) {
        nlohmann::json j = SpeedJson(speed);
        j.update({ {"x", dx}, {"y", dy}, {"z", dz}, {"roll", droll}, {"pitch", dpitch}, {"yaw", dyaw}, {"relative", true} });
        return CallJson(ROBOT_ARM_API_ID_MOVEL, j);
    }

    /// End effector turned by droll dpitch dyaw [rad] - added to the roll pitch yaw rt/rbq/arm_ee reports - in `seconds`
    /// (0.5 – 10 s). The position stays. Needs ControlStart().
    int32_t RotateLDelta(float droll, float dpitch, float dyaw, float seconds) {
        return CallJson(ROBOT_ARM_API_ID_MOVEL,
                        { {"roll", droll}, {"pitch", dpitch}, {"yaw", dyaw}, {"relative", true}, {"time", seconds} });
    }

    /// End effector turned by droll dpitch dyaw [rad], as fast as `speed.angVel` / `speed.angAcc` allow.
    int32_t RotateLDelta(float droll, float dpitch, float dyaw, const ArmTaskSpeed& speed) {
        nlohmann::json j = SpeedJson(speed);
        j.update({ {"roll", droll}, {"pitch", dpitch}, {"yaw", dyaw}, {"relative", true} });
        return CallJson(ROBOT_ARM_API_ID_MOVEL, j);
    }

    /// Joint gains the arm uses for MoveJ, MoveL and high-level control: kp [Nm/rad] and kd [Nm*s/rad] for J0..J5.
    /// Accepted range per joint: kp 10 % - 150 % and kd 10 % - 200 % of the default (GetState kp_default / kd_default);
    /// anything outside, or not finite, refuses the whole call (RPC_ERR_SERVER_API_PARAMETER). Takes effect over 0.5 - 2 s,
    /// also while the arm moves, and stays until ResetGains() or AutoStart(). Folding and other soft poses keep their own
    /// low gains. Refused while an app holds the joints (Lv0 - set gains there with JointStream::SetGains).
    int32_t SetGains(const std::array<float, 6>& kp, const std::array<float, 6>& kd) {
        return CallJson(ROBOT_ARM_API_ID_SETGAINS, { {"kp", kp}, {"kd", kd} });
    }
    /// Back to the default gains.
    int32_t ResetGains() { return CallJson(ROBOT_ARM_API_ID_SETGAINS, { {"default", true} }); }

    /// Arm status as strings: arm_type, can_check, con_start, task_control, high_level, estop (1 in emergency stop),
    /// motion (moving / idle), result (none / running / done / canceled / collision / blocked / workspace / estop - of the
    /// last motion call), motion_time (seconds of the last MoveJ by angles / MoveL), collision_joint, pose, joint_owner, sdk_slot,
    /// gains (default / custom), kp / kd (SetGains values, J0..J5 comma-separated) and kp_default / kd_default.
    int32_t GetState(std::map<std::string, std::string>& state_map) {
        std::string data;
        int32_t code = Call(ROBOT_ARM_API_ID_GETSTATE, std::string{}, data);
        if (code == RPC_OK && !data.empty()) {
            try {
                auto j = nlohmann::json::parse(data);
                for (auto& el : j.items())
                    state_map[el.key()] = el.value().get<std::string>();
            } catch (...) {
                // leave state_map untouched on malformed payload
            }
        }
        return code;
    }

private:
    static nlohmann::json SpeedJson(const ArmTaskSpeed& s) {
        return { {"vel", s.vel}, {"acc", s.acc}, {"ang_vel", s.angVel}, {"ang_acc", s.angAcc} };
    }

    int32_t CallJson(int32_t api_id, const nlohmann::json& j) {
        return Call(api_id, j.dump());
    }
};

} // namespace rbq_sdk
