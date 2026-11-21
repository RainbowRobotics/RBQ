#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "arm_helper.hpp"

// 08 — torque control of the RBA-1 arm (Lv0). Instead of joint references this program sends joint torques: the
// robot's gravity torque (rt/rbq/arm_gravity) plus a virtual spring that pulls every joint back to where it started.
// With stiffness 0 only gravity is compensated - the arm floats and can be pushed by hand.
// The second form, J<n> <Nm>, puts a torque of your own on one joint for a few seconds: that joint gets gravity + your
// torque and nothing else (it accelerates), the other joints gravity + the spring.
//
// Torque mode is JointStream::SendTorque(): kp 0, a small damping kd, and your torques. Nothing else holds the arm, so
// leave the gravity torque out and it falls. If the torques stop for 0.2 s the robot sends the gravity torque with
// damping itself, and at the end (or Ctrl-C) the joints go back to position control where the arm is.

using arm_helper::ArmState;
using arm_helper::JointStream;
using arm_helper::kArmJoints;

static constexpr float kR2D = 57.295779513f;
static constexpr int   kRateHz = 500;
// J<n> <Nm>: what this example lets you put on one joint - well under the robot's clamp (+-60 / +-25 Nm), since nothing
// holds that joint. It stops early when the joint comes within kEdgeDeg of its limit or has turned kTravelDeg.
static constexpr float kJointTorqueMax[kArmJoints] = {15.f, 15.f, 8.f, 8.f, 8.f, 8.f};   // [Nm]
static constexpr float kEdgeDeg   = 10.f;
static constexpr float kTravelDeg = 90.f;

static std::atomic<bool> g_running{true};

static std::string Usage()
{
    return std::string("Usage: 08_arm_torque [target] [stiffness] [seconds]\n"
        "       08_arm_torque [target] J<n> <Nm> [seconds]\n"
        "\n"
        "Torque control (Lv0) of the RBA-1 arm: this program sends the six joint torques itself, 500 times a second -\n"
        "the gravity torque the robot computes plus a spring toward the start pose: torque = gravity + stiffness *\n"
        "(start - q). It starts the arm where it is (moving it to Ready only from the folded pose), takes the six arm\n"
        "joints (app slot 22), and hands them back in position control where the arm is.\n"
        "\n") + arm_helper::kTargetUsage +
        "  stiffness  spring toward the start pose [Nm/rad], 0..50 (default 20); 0 = gravity only, the arm floats\n"
        "  seconds    how long to run, 2..120 s                                 (default 10)\n"
        "\n"
        "J<n> <Nm>: a torque of your own on one joint. Joint n (J0..J5) gets gravity + <Nm> and no spring, so it turns\n"
        "under that torque; the other joints get gravity + the spring (20 Nm/rad) and stay. The sign is the direction.\n"
        "  J<n>       J0 shoulder yaw  J1 shoulder pitch  J2 elbow pitch  J3 elbow roll  J4 wrist pitch  J5 wrist roll\n"
        "  Nm         -15..15 for J0 and J1, -8..8 for J2..J5\n"
        "  seconds    how long the torque is on, 0.5..20 s                      (default 2)\n"
        "It ends early when the joint comes within 10 deg of its limit or has turned 90 deg.\n"
        "\n"
        "Examples:\n"
        "  08_arm_torque sim                 spring 20 Nm/rad for 10 s\n"
        "  08_arm_torque sim 0 30            gravity only for 30 s - push the arm, it stays where it is left\n"
        "  08_arm_torque sim J4 2 1          J4 (wrist pitch) +2 Nm for 1 s, the other joints held\n"
        "  08_arm_torque sim J0 -5           J0 (shoulder yaw) -5 Nm for 2 s\n"
        "\n"
        "Twice a second it prints the joint furthest from the start pose, its torque and gravity torque [Nm], and the\n"
        "end-effector position. The robot clamps each torque to +-60 Nm (J0, J1) / +-25 Nm (J2..J5). The gravity torque\n"
        "takes the body as level - on a tilted body the arm drifts a little, which the spring or your own term must hold.\n";
}

// "J0" .. "J5" (or j0 ..) -> 0 .. 5, anything else -1
static int JointArg(const char *s)
{
    return (s[0] == 'J' || s[0] == 'j') && s[1] >= '0' && s[1] <= '9' && s[2] == '\0' ? s[1] - '0' : -1;
}

// J<n> <Nm> [seconds]: gravity + <Nm> on joint n, gravity + the spring on the others, for `seconds`
static int JointTorque(char *argv[], int argc, int joint)
{
    if (joint >= kArmJoints) return arm_helper::ArgError(argv, std::string(argv[2]) + " - the joints are J0 .. J5", Usage());
    float nm = 0.f, seconds = 2.0f;
    const float cap = kJointTorqueMax[joint];
    if (argc < 4) return arm_helper::ArgError(argv, std::string(argv[2]) + " needs the torque [Nm]", Usage());
    if (!(arm_helper::ParseFloat(argv[3], nm) && std::abs(nm) <= cap))
        return arm_helper::ArgError(argv, "J" + std::to_string(joint) + " torque must be -" + arm_helper::Num(cap) + ".."
                                    + arm_helper::Num(cap) + " [Nm], not '" + argv[3] + "'", Usage());
    if (argc > 4 && !(arm_helper::ParseFloat(argv[4], seconds) && seconds >= 0.5f && seconds <= 20.0f))
        return arm_helper::ArgError(argv, std::string("seconds must be 0.5..20, not '") + argv[4] + "'", Usage());
    if (argc > 5) return arm_helper::ArgError(argv, "too many arguments", Usage());

    if (arm_helper::DryRun(argv[1], "Torque joint=" + std::to_string(joint) + " torque=" + arm_helper::Num(nm)
                                    + " seconds=" + arm_helper::Num(seconds))) return 0;
    std::signal(SIGINT, [](int) { g_running = false; });
    if (!arm_helper::InitChannel(argv[1])) return 1;

    JointStream arm;   // app slot 22
    if (!arm.Start(/*toReady=*/true, /*onlyIfFolded=*/true)) return 1;   // where the arm is, Ready only from folded
    for (int i = 0; i < 50 && !arm.State().gravityValid; i++) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (!arm.State().gravityValid) {
        std::printf("[08_arm_torque] nothing on rt/rbq/arm_gravity - the robot software is older than this example\n");
        return 1;
    }

    constexpr float kHold = 20.0f;   // [Nm/rad] the other joints' spring
    float q0[kArmJoints];
    const ArmState s0 = arm.State();
    std::copy(s0.q, s0.q + kArmJoints, q0);
    std::printf("[08_arm_torque] J%d %+.2f Nm for %s s, the other joints held\n", joint, nm, arm_helper::Num(seconds).c_str());

    const auto period = std::chrono::microseconds(1000000 / kRateHz);
    auto next = std::chrono::steady_clock::now();
    float torque[kArmJoints];
    const char *why = "time is up";
    long tick = 0;
    for (; g_running && tick < static_cast<long>(seconds * kRateHz); tick++) {
        const ArmState s = arm.State();
        const float deg = s.q[joint] * kR2D;
        if (deg < arm_helper::kJointMinDeg[joint] + kEdgeDeg || deg > arm_helper::kJointMaxDeg[joint] - kEdgeDeg) {
            why = "the joint came within 10 deg of its limit";
            break;
        }
        if (std::abs(s.q[joint] - q0[joint]) * kR2D > kTravelDeg) {
            why = "the joint turned 90 deg";
            break;
        }
        for (int i = 0; i < kArmJoints; i++) torque[i] = s.gravity[i] + (i == joint ? nm : kHold * (q0[i] - s.q[i]));
        arm.SendTorque(torque);

        if (tick % (kRateHz / 4) == 0)
            std::printf("[08_arm_torque] t %5.2f s  J%d off start %7.2f deg  vel %7.2f deg/s  torque %6.2f Nm  gravity %6.2f Nm\n",
                        static_cast<double>(tick) / kRateHz, joint, (s.q[joint] - q0[joint]) * kR2D, s.dq[joint] * kR2D,
                        torque[joint], s.gravity[joint]);
        next += period;
        std::this_thread::sleep_until(next);
    }
    const ArmState s = arm.State();
    std::printf("[08_arm_torque] J%d stopped after %.2f s (%s): %+.2f deg from the start\n", joint,
                static_cast<double>(tick) / kRateHz, g_running ? why : "Ctrl-C", (s.q[joint] - q0[joint]) * kR2D);
    arm.Stop();   // position control again, where the arm is
    return 0;
}

int main(int argc, char *argv[])
{
    if (!arm_helper::TargetGiven(argc, argv, Usage())) return arm_helper::HelpAsked(argc, argv) ? 0 : 1;
    if (argc > 2 && JointArg(argv[2]) >= 0) return JointTorque(argv, argc, JointArg(argv[2]));
    float stiffness = 20.0f, seconds = 10.0f;
    if (argc > 2 && !(arm_helper::ParseFloat(argv[2], stiffness) && stiffness >= 0.0f && stiffness <= 50.0f))
        return arm_helper::ArgError(argv, std::string("stiffness must be 0..50 [Nm/rad], not '") + argv[2] + "'", Usage());
    if (argc > 3 && !(arm_helper::ParseFloat(argv[3], seconds) && seconds >= 2.0f && seconds <= 120.0f))
        return arm_helper::ArgError(argv, std::string("seconds must be 2..120, not '") + argv[3] + "'", Usage());
    if (argc > 4) return arm_helper::ArgError(argv, "too many arguments", Usage());

    if (arm_helper::DryRun(argv[1], "Torque stiffness=" + arm_helper::Num(stiffness) + " seconds=" + arm_helper::Num(seconds)))
        return 0;
    std::signal(SIGINT, [](int) { g_running = false; });
    if (!arm_helper::InitChannel(argv[1])) return 1;

    JointStream arm;   // app slot 22
    if (!arm.Start(/*toReady=*/true, /*onlyIfFolded=*/true)) return 1;   // where the arm is, Ready only from folded
    for (int i = 0; i < 50 && !arm.State().gravityValid; i++) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (!arm.State().gravityValid) {
        std::printf("[08_arm_torque] nothing on rt/rbq/arm_gravity - the robot software is older than this example\n");
        return 1;   // ~JointStream hands the joints back
    }

    float q0[kArmJoints];
    const ArmState s0 = arm.State();
    std::copy(s0.q, s0.q + kArmJoints, q0);

    const auto period = std::chrono::microseconds(1000000 / kRateHz);
    auto next = std::chrono::steady_clock::now();
    float torque[kArmJoints];
    for (long tick = 0; g_running && tick < static_cast<long>(seconds * kRateHz); tick++) {
        const ArmState s = arm.State();
        for (int i = 0; i < kArmJoints; i++) torque[i] = s.gravity[i] + stiffness * (q0[i] - s.q[i]);
        arm.SendTorque(torque);

        if (tick % (kRateHz / 2) == 0) {
            int far = 0;
            for (int i = 1; i < kArmJoints; i++)
                if (std::abs(q0[i] - s.q[i]) > std::abs(q0[far] - s.q[far])) far = i;
            std::printf("[08_arm_torque] t %5.2f s  J%d off start %6.2f deg  torque %6.2f Nm  gravity %6.2f Nm"
                        "  EE %6.3f %6.3f %6.3f m\n",
                        static_cast<double>(tick) / kRateHz, far, (s.q[far] - q0[far]) * kR2D, torque[far], s.gravity[far],
                        s.eePos[0], s.eePos[1], s.eePos[2]);
        }
        next += period;
        std::this_thread::sleep_until(next);
    }

    arm.Stop();   // position control again, where the arm is
    return 0;
}
