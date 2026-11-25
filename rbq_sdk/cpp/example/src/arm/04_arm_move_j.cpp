#include <array>
#include <cctype>
#include <iostream>
#include <string>

#include <rbq_sdk/dds/ArmClient.hpp>
#include <rbq_sdk/dds/ChannelFactory.hpp>

#include "arm_helper.hpp"

// 04 — joint-space move of the RBA-1 arm: to one of its built-in poses (MoveJ), or by joint angles - to them (MoveJ) or
// by them from where the arm is (MoveJDelta), all six joints or one.

static std::string Usage()
{
    return std::string(
        "Usage: 04_arm_move_j [target] <pose>\n"
        "       04_arm_move_j [target] [abs|rel] <J0 J1 J2 J3 J4 J5> [seconds | vel=<deg/s> acc=<deg/s^2>]\n"
        "       04_arm_move_j [target] [abs|rel] J<n> <degrees>      [seconds | vel=<deg/s> acc=<deg/s^2>]\n"
        "\n"
        "Joint-space move of the RBA-1 arm, straight from where the arm is: to one of its built-in poses,\n"
        "or by joint angles. Starts the arm first if needed and returns when the arm has arrived.\n"
        "\n") + arm_helper::kTargetUsage +
        "  pose       a number or a name (any case), e.g. 0, home, Folding:\n" + arm_helper::PoseUsage() +
        "  abs        the angles are where the joints go (MoveJ) - the default\n"
        "  rel        the angles are how far the joints turn from where they are commanded now (MoveJDelta)\n"
        "  J0 .. J5   six joint angles [deg]. The joints may reach J1 0..180, J2 -180..2, the others -360..360\n"
        "  J<n> deg   only joint n (J0 .. J5); the others stay\n"
        "  seconds    every moving joint arrives together in this time (0.5 to 20 s, both included), if that keeps the joints\n"
        "             under 90 deg/s and 180 deg/s^2 - otherwise refused with the time the move needs\n"
        "  vel acc    or the robot picks the shortest time that keeps the fastest joint under vel [deg/s]\n"
        "             and acc [deg/s^2] - default 30 and 60, max 90 and 180. With neither: the defaults\n"
        "\n"
        "Examples:\n"
        "  04_arm_move_j sim 1                          to Ready\n"
        "  04_arm_move_j sim home                       to Folding (same as 0)\n"
        "  04_arm_move_j sim 0 30 -30 0 60 0 4          to these angles in 4 s\n"
        "  04_arm_move_j sim 0 30 -30 0 60 0 vel=20     no joint faster than 20 deg/s\n"
        "  04_arm_move_j sim J1 120 2                   J1 to 120 deg in 2 s\n"
        "  04_arm_move_j sim rel J4 -15 2               J4 turns -15 deg from where it is, in 2 s\n"
        "  04_arm_move_j sim rel 0 0 10 0 -10 0         J2 +10, J4 -10 deg\n";
}

// shown with the pose list when no pose was given - the joint-angle forms are easy to miss otherwise
static const char *kAngleExamples =
    "[arm] or by joint angles [deg] (abs = to them, the default; rel = by them from where the arm is):\n"
    "    04_arm_move_j 0 30 -30 0 60 0 4          all six joints to these angles in 4 s\n"
    "    04_arm_move_j J1 120 2                   only J1, to 120 deg in 2 s\n"
    "    04_arm_move_j rel J4 -15 vel=20          only J4, -15 deg from where it is, at most 20 deg/s\n"
    "    04_arm_move_j rel 0 0 10 0 -10 0         J2 +10, J4 -10 deg\n"
    "[arm] these go to the simulator on this PC. To the robot at 192.168.0.10, put -r (or robot) first:\n"
    "    04_arm_move_j -r 1                       the robot's arm to Ready\n"
    "[arm] 04_arm_move_j -h for everything\n";

// "J0" .. "J5" (or "j0" ..) -> 0 .. 5, otherwise -1
static int JointArg(const char *s)
{
    return (s[0] == 'J' || s[0] == 'j') && s[1] >= '0' && s[1] <= '5' && s[2] == '\0' ? s[1] - '0' : -1;
}

int main(int argc, char *argv[])
{
    if (!arm_helper::TargetGiven(argc, argv, Usage())) return arm_helper::HelpAsked(argc, argv) ? 0 : 1;

    // [abs|rel], then six numbers or J<n> and a number, then an optional time or vel= / acc=. Otherwise a pose.
    int i = 2;
    const std::string mode = argc > i ? argv[i] : "";
    const bool relative = mode == "rel";
    if (mode == "abs" || mode == "rel") i++;
    std::array<float, 6> q{};
    float first = 0.f;
    const int joint = argc > i ? JointArg(argv[i]) : -1;
    const bool angles = joint < 0 && argc > i + 1 && arm_helper::ParseFloat(argv[i], first);
    const bool byAngle = angles || joint >= 0;
    float degrees = 0.f;
    float seconds = 0.f;
    rbq_sdk::ArmJointSpeed speed;
    std::string pose;
    if (joint < 0 && argc > i && (argv[i][0] == 'J' || argv[i][0] == 'j') && std::isdigit(static_cast<unsigned char>(argv[i][1])))
        return arm_helper::ArgError(argv, std::string(argv[i]) + " - the joints are J0 .. J5", Usage());
    if (i > 2 && !byAngle)
        return arm_helper::ArgError(argv, mode + " needs six joint angles or J<n> <degrees>", Usage());
    if (joint >= 0) {
        if (argc <= i + 1 || !arm_helper::ParseFloat(argv[i + 1], degrees))
            return arm_helper::ArgError(argv, std::string(argv[i]) + " needs the degrees", Usage());
        i += 2;
    } else if (angles) {
        if (argc < i + 6)
            return arm_helper::ArgError(argv, "give six joint angles J0..J5 - got " + std::to_string(argc - i), Usage());
        for (int j = 0; j < 6; j++, i++)
            if (!arm_helper::ParseFloat(argv[i], q[j]))
                return arm_helper::ArgError(argv, std::string("joint angles must be numbers, not '") + argv[i] + "'", Usage());
    } else {
        pose = (argc > 2) ? arm_helper::PoseFromArg(argv[2]) : "";
        if (pose.empty()) {
            const std::string what = argc > 2 ? std::string("unknown pose '") + argv[2] + "' - one of:" : "which pose? one of:";
            std::printf("[arm] %s\n%s%s", what.c_str(), arm_helper::PoseUsage().c_str(), kAngleExamples);
            if (argc > 2) arm_helper::PrintTargetHint(argv);
            return 1;
        }
    }
    // an absolute angle outside the joint range is refused here already; a relative one by the robot (it knows where it is)
    for (int j = 0; byAngle && !relative && j < 6; j++) {
        if (joint >= 0 && j != joint) continue;
        const float v = joint >= 0 ? degrees : q[j];
        if (v < arm_helper::kJointMinDeg[j] || v > arm_helper::kJointMaxDeg[j])
            return arm_helper::ArgError(argv, "J" + std::to_string(j) + " must be " + std::to_string(int(arm_helper::kJointMinDeg[j]))
                                        + ".." + std::to_string(int(arm_helper::kJointMaxDeg[j])) + " deg, not " + arm_helper::Num(v), Usage());
    }
    for (const int rest = i; byAngle && i < argc; i++) {
        if (arm_helper::ParseOption(argv[i], "vel", speed.vel) || arm_helper::ParseOption(argv[i], "acc", speed.acc)) continue;
        if (i == rest && arm_helper::ParseFloat(argv[i], seconds)) {
            if (seconds < 0.5f) return arm_helper::ArgError(argv, "the minimum time is 0.5 s, not " + arm_helper::Num(seconds), Usage());
            if (seconds > 20.f) return arm_helper::ArgError(argv, "the maximum time is 20 s, not " + arm_helper::Num(seconds), Usage());
            continue;
        }
        return arm_helper::ArgError(argv, std::string("after the angles: seconds (0.5..20) or vel= / acc=, not '") + argv[i] + "'",
                                    Usage());
    }
    std::string call = "MoveJ pose=" + pose;
    if (byAngle) {
        call = relative ? "MoveJDelta" : "MoveJ";
        if (joint >= 0) call += " joint=" + std::to_string(joint) + " q=" + arm_helper::Num(degrees);
        else for (int j = 0; j < 6; j++) call += (j ? "," : " q=") + arm_helper::Num(q[j]);
        call += seconds > 0.f ? " seconds=" + arm_helper::Num(seconds)
                              : " vel=" + arm_helper::Num(speed.vel) + " acc=" + arm_helper::Num(speed.acc);
    }
    if (arm_helper::DryRun(argv[1], call)) return 0;
    if (!arm_helper::InitChannel(argv[1])) return 1;
    rbq_sdk::ArmClient arm;
    arm.SetTimeout(2.0f);

    // starts the arm if needed, then goes straight to the chosen pose
    if (!arm_helper::InitializeArm(arm, /*toReady=*/false)) return 1;

    if (!byAngle) return arm_helper::Move(arm, "MoveJ " + pose, arm.MoveJ(pose)) ? 0 : 1;   // returns when the arm has arrived
    const bool timed = seconds > 0.f;
    // the command line is in degrees, ArmClient in radians
    const float d2r = arm_helper::kDeg2Rad;
    std::array<float, 6> qRad;
    for (int j = 0; j < 6; j++) qRad[j] = q[j] * d2r;
    const float rad = degrees * d2r;
    const rbq_sdk::ArmJointSpeed speedRad{speed.vel * d2r, speed.acc * d2r};
    int32_t code;
    if (joint >= 0 && relative) code = timed ? arm.MoveJDelta(joint, rad, seconds) : arm.MoveJDelta(joint, rad, speedRad);
    else if (joint >= 0)        code = timed ? arm.MoveJ(joint, rad, seconds) : arm.MoveJ(joint, rad, speedRad);
    else if (relative)          code = timed ? arm.MoveJDelta(qRad, seconds) : arm.MoveJDelta(qRad, speedRad);
    else                        code = timed ? arm.MoveJ(qRad, seconds) : arm.MoveJ(qRad, speedRad);
    return arm_helper::Move(arm, relative ? "MoveJDelta" : "MoveJ", code, {true, seconds, "joints 90 deg/s, 180 deg/s^2"}) ? 0 : 1;
}
