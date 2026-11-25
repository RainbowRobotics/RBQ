#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <rbq_sdk/dds/ArmClient.hpp>
#include <rbq_sdk/dds/ChannelFactory.hpp>

#include "arm_helper.hpp"

// 05 — task-space moves (MoveL family) of the RBA-1 arm. Starts the arm and turns task-space control on (through
// Ready, only if it is off), then moves to (abs) or by (rel) the given position, orientation or both, or runs a short demo.

static std::string Usage()
{
    return std::string(
        "Usage: 05_arm_move_l [target]                                            demo: small moves and back\n"
        "       05_arm_move_l [target] [abs|rel] pos  <x y z>                [seconds | vel= acc=]\n"
        "       05_arm_move_l [target] [abs|rel] ori  <roll pitch yaw>       [seconds | ang_vel= ang_acc=]\n"
        "       05_arm_move_l [target] [abs|rel] pose <x y z roll pitch yaw> [seconds | vel= acc= ang_vel= ang_acc=]\n"
        "\n"
        "Task-space moves (MoveL family) of the RBA-1 arm. Starts the arm if needed and turns task-space\n"
        "control on - through Ready only when it is off; when it is already on, the arm moves from where it is.\n"
        "\n") + arm_helper::kTargetUsage +
        "  (none)           demo: up 2 cm, forward 2 cm, pitch +10 deg, then back the same way\n"
        "  abs              the numbers are where the end effector goes (MoveL, RotateL) - the default\n"
        "  rel              the numbers are how far it moves from where it is now (MoveLDelta, RotateLDelta):\n"
        "                   x y z added to the commanded position, roll pitch yaw to the measured angles 01_arm_state shows\n"
        "  pos x y z        MoveL: end effector to that position [m] in the body frame - the numbers\n"
        "                   01_arm_state shows (x forward, y left, z up). The orientation stays\n"
        "  ori r p y        RotateL: to that orientation roll pitch yaw [deg], as 01_arm_state shows it.\n"
        "                   The position stays\n"
        "  pose x y z r p y MoveL: both\n"
        "                   (without the word: 3 numbers are pos, 6 are pose)\n"
        "  seconds          in this time (0.5 to 10 s, both included), if that stays under the max limits below - otherwise\n"
        "                   refused with the time the move needs\n"
        "  vel= acc=        or the robot picks the shortest time under these limits: end effector [m/s]\n"
        "  ang_vel= ang_acc=  and [m/s^2], orientation [deg/s] and [deg/s^2] - default 0.1 0.2 30 60,\n"
        "                   max 0.25 0.5 90 180. With neither: the defaults\n"
        "\n"
        "Examples:\n"
        "  05_arm_move_l sim\n"
        "  05_arm_move_l sim pos 0.60 0 0.30 3                  to that position in 3 s\n"
        "  05_arm_move_l sim ori 0 30 0 2                       pitch 30 deg, same position\n"
        "  05_arm_move_l sim pose 0.60 0 0.30 0 60 0 3          both\n"
        "  05_arm_move_l sim pos 0.60 0 0.30 vel=0.05           no faster than 5 cm/s\n"
        "  05_arm_move_l sim rel pos -0.05 0 0 2                5 cm back in 2 s\n"
        "  05_arm_move_l sim rel ori 0 0 15 2                   yaw +15 deg, same position\n";
}

int main(int argc, char *argv[])
{
    if (!arm_helper::TargetGiven(argc, argv, Usage())) return arm_helper::HelpAsked(argc, argv) ? 0 : 1;
    // [abs|rel], [pos|ori|pose], then the numbers (+ seconds), then vel= / acc= / ang_vel= / ang_acc=
    int first = 2;
    const std::string how = argc > first ? argv[first] : "";
    const bool relative = how == "rel";
    if (how == "abs" || how == "rel") first++;
    std::string mode = argc > first ? argv[first] : "";
    if (mode == "pos" || mode == "ori" || mode == "pose") first++;
    else mode.clear();
    std::vector<float> nums;
    rbq_sdk::ArmTaskSpeed speed;
    bool limits = false;
    bool linearLimits = false;   // vel= / acc= - ori has no position to move
    for (int i = first; i < argc; i++) {
        float v = 0.f;
        if (arm_helper::ParseOption(argv[i], "vel", speed.vel) || arm_helper::ParseOption(argv[i], "acc", speed.acc)) {
            limits = linearLimits = true;
            continue;
        }
        if (arm_helper::ParseOption(argv[i], "ang_vel", speed.angVel) || arm_helper::ParseOption(argv[i], "ang_acc", speed.angAcc)) {
            limits = true;
            continue;
        }
        if (limits || !arm_helper::ParseFloat(argv[i], v))
            return arm_helper::ArgError(argv, std::string("expected a number or vel= acc= ang_vel= ang_acc=, not '") + argv[i] + "'",
                                        Usage());
        nums.push_back(v);
    }
    const size_t n = nums.size();
    if (mode.empty() && n > 0) {   // without the word: 3 (+ seconds) numbers are pos, 6 (+ seconds) pose
        if (n != 3 && n != 4 && n != 6 && n != 7)
            return arm_helper::ArgError(argv, "give 3 numbers (x y z) or 6 (x y z roll pitch yaw) [+ seconds] - got "
                                        + std::to_string(n), Usage());
        mode = n >= 6 ? "pose" : "pos";
    }
    const size_t need = mode == "pose" ? 6 : 3;
    if (!mode.empty() && n != need && n != need + 1)
        return arm_helper::ArgError(argv, mode + " needs " + std::to_string(need) + " numbers [+ seconds] - got "
                                    + std::to_string(n), Usage());
    if (mode.empty() && limits) return arm_helper::ArgError(argv, "vel= / acc= need a position", Usage());
    if (mode.empty() && first > 2) return arm_helper::ArgError(argv, how + " needs pos, ori or pose and its numbers", Usage());
    if (mode == "ori" && linearLimits) return arm_helper::ArgError(argv, "ori takes ang_vel= / ang_acc=", Usage());
    const float seconds = n == need + 1 ? nums.back() : 0.f;
    if (n == need + 1 && limits) return arm_helper::ArgError(argv, "give seconds or vel= / acc=, not both", Usage());
    if (n == need + 1 && seconds < 0.5f)
        return arm_helper::ArgError(argv, "the minimum time is 0.5 s, not " + arm_helper::Num(seconds), Usage());
    if (n == need + 1 && seconds > 10.0f)
        return arm_helper::ArgError(argv, "the maximum time is 10 s, not " + arm_helper::Num(seconds), Usage());
    std::string call = "RelativeMoves";
    if (!mode.empty()) {
        static const char *kNames[] = {"x", "y", "z", "roll", "pitch", "yaw"};
        call = std::string(mode == "ori" ? "RotateL" : "MoveL") + (relative ? "Delta" : "");
        for (size_t k = 0; k < need; k++) call += std::string(" ") + kNames[mode == "ori" ? k + 3 : k] + "=" + arm_helper::Num(nums[k]);
        call += seconds > 0.f ? " seconds=" + arm_helper::Num(seconds)
                              : " vel=" + arm_helper::Num(speed.vel) + " acc=" + arm_helper::Num(speed.acc)
                                + " ang_vel=" + arm_helper::Num(speed.angVel) + " ang_acc=" + arm_helper::Num(speed.angAcc);
    }
    if (arm_helper::DryRun(argv[1], call)) return 0;
    if (!arm_helper::InitChannel(argv[1])) return 1;
    rbq_sdk::ArmClient arm;
    arm.SetTimeout(2.0f);

    // task-space control through Ready only when it is not on yet - otherwise the arm moves from where it is
    if (!arm_helper::InitializeArm(arm, /*toReady=*/false)) return 1;
    if (!arm_helper::StartTaskControl(arm)) return 1;

    if (!mode.empty()) {
        // the command line is in degrees, ArmClient in radians: roll pitch yaw and the turning limits
        const float d2r = arm_helper::kDeg2Rad;
        std::vector<float> rad(nums.begin(), nums.begin() + need);   // without the seconds at the end
        const size_t firstAngle = mode == "ori" ? 0 : 3;
        for (size_t k = firstAngle; k < need; k++) rad[k] *= d2r;
        speed.angVel *= d2r;
        speed.angAcc *= d2r;
        const float *v = rad.data();
        const bool timed = seconds > 0.f;
        int32_t code;
        if (relative) {
            if (mode == "pose")     code = timed ? arm.MoveLDelta(v[0], v[1], v[2], v[3], v[4], v[5], seconds)
                                                 : arm.MoveLDelta(v[0], v[1], v[2], v[3], v[4], v[5], speed);
            else if (mode == "ori") code = timed ? arm.RotateLDelta(v[0], v[1], v[2], seconds) : arm.RotateLDelta(v[0], v[1], v[2], speed);
            else                    code = timed ? arm.MoveLDelta(v[0], v[1], v[2], seconds) : arm.MoveLDelta(v[0], v[1], v[2], speed);
        } else {
            if (mode == "pose")     code = timed ? arm.MoveL(v[0], v[1], v[2], v[3], v[4], v[5], seconds)
                                                 : arm.MoveL(v[0], v[1], v[2], v[3], v[4], v[5], speed);
            else if (mode == "ori") code = timed ? arm.RotateL(v[0], v[1], v[2], seconds) : arm.RotateL(v[0], v[1], v[2], speed);
            else                    code = timed ? arm.MoveL(v[0], v[1], v[2], seconds) : arm.MoveL(v[0], v[1], v[2], speed);
        }
        return arm_helper::Move(arm, std::string(mode == "ori" ? "RotateL" : "MoveL") + (relative ? "Delta" : ""), code,
                                {true, seconds, "0.25 m/s, 0.5 m/s^2, turning 90 deg/s, 180 deg/s^2"}) ? 0 : 1;
    }

    // each relative move takes 0.3 s on the robot; Wait() returns when it is done
    struct Step { const char *what; bool rotate; int axis; float value; };
    const Step steps[] = {
        {"MoveLDelta z +0.02 m",     false, rbq_sdk::ARM_AXIS_Z,  0.02f},
        {"MoveLDelta x +0.02 m",     false, rbq_sdk::ARM_AXIS_X,  0.02f},
        {"RotateLDelta pitch +10",   true,  rbq_sdk::ARM_AXIS_Y,  10.0f * arm_helper::kDeg2Rad},
        {"RotateLDelta pitch -10",   true,  rbq_sdk::ARM_AXIS_Y, -10.0f * arm_helper::kDeg2Rad},
        {"MoveLDelta x -0.02 m",     false, rbq_sdk::ARM_AXIS_X, -0.02f},
        {"MoveLDelta z -0.02 m",     false, rbq_sdk::ARM_AXIS_Z, -0.02f},
    };
    for (const auto &s : steps) {
        const int32_t code = s.rotate ? arm.RotateLDelta(s.axis, s.value) : arm.MoveLDelta(s.axis, s.value);
        if (!arm_helper::Move(arm, s.what, code, {}, 5.0f)) return 1;
    }
    return 0;
}
