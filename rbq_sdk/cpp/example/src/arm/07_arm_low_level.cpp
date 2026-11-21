#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "arm_helper.hpp"

// 07 — joint-level control of the RBA-1 arm (Lv0). For your own algorithm (FK / IK / planner) change my_arm.cpp,
// which has the same two parts.
//
// The file has two parts:
//   1. Controller - YOUR part. It reads the arm state every tick and returns the six joint references.
//      Out of the box it moves one joint out and back; replace it with your algorithm.
//   2. main       - leave it as it is. It starts the arm, takes the six joints (app slot 22) through
//      arm_helper::JointStream, calls Controller 500 times a second, and hands the joints back where they are
//      when Controller ends or on Ctrl-C. It knows nothing about the demo, so any Controller with the same
//      five members compiles.
//
// The robot moves each joint reference at most 2 rad/s toward what you send and clamps it to the joint limits, so a
// wrong reference makes the arm travel there at that speed rather than snap. The legs stay with the robot.

using arm_helper::ArmState;
using arm_helper::JointStream;
using arm_helper::kArmJoints;

static constexpr float kD2R = 0.0174532925f;
static constexpr float kR2D = 57.295779513f;
static constexpr int   kRateHz = 500;   // Update() is called this often; it must return well within 1 / kRateHz

// ===== 1. Your controller =========================================================================================
// Keep these five members; change what is inside them.
//   kArgs / Help()  the arguments after [target], for the usage line and -h
//   Configure(args) reads those arguments; returns "" when fine, or what is wrong
//   Start(s, arm)   once, right after the joints are taken - s.qRef is where the arm is now
//   Update(t, s, qRef, arm)
//                   every tick: t [s] since Start, s = the arm now (ArmState in arm_helper.hpp);
//                   write the six joint references [rad] into qRef. Return false to end.
// Also there when you need them: arm.Gripper(JointStream::Grip::Open / Close / Stop) - returns at once;
// arm.SetGains(kp, kd); arm.Send(q, vel, torque) for feed-forward velocity [rad/s] and torque [Nm].
struct Controller {
    static constexpr const char *kArgs = "[joint] [delta_deg] [seconds] [grip]";
    static std::string Help()
    {
        return
            "  joint      the joint to move                                                     (default 0)\n"
            "             0 J0 shoulder yaw   1 J1 shoulder pitch   2 J2 elbow pitch\n"
            "             3 J3 elbow roll     4 J4 wrist pitch      5 J5 wrist roll\n"
            "  delta_deg  how far it goes before coming back [deg], -30..30; the sign is the direction (default 15)\n"
            "  seconds    time for the whole out-and-back, 2..20 s                              (default 4)\n"
            "  grip       write the word grip to also close the gripper on the way out and open it on the way back\n"
            "\n"
            "The demo moves ONE joint away by delta_deg and back along a smooth cosine curve - a single out-and-back,\n"
            "not a repeating sine.\n"
            "\n"
            "Examples:\n"
            "  07_arm_low_level sim                  J0 15 deg out and back in 4 s\n"
            "  07_arm_low_level sim 4 -20 6          J4 (wrist pitch) -20 deg out and back in 6 s\n"
            "  07_arm_low_level sim 0 20 4 grip      ... and close / open the gripper on the way\n";
    }

    // the demo's settings
    int   joint   = 0;
    float delta   = 15.0f * kD2R;
    float seconds = 4.0f;
    bool  grip    = false;

    float q0[kArmJoints];
    bool  turned = false;

    std::string Configure(const std::vector<std::string> &args)
    {
        float deltaDeg = 15.0f;
        if (args.size() > 0 && !(arm_helper::ParseInt(args[0].c_str(), joint) && joint >= 0 && joint < kArmJoints))
            return "joint must be 0..5 (J0..J5), not '" + args[0] + "'";
        if (args.size() > 1 && !(arm_helper::ParseFloat(args[1].c_str(), deltaDeg) && std::abs(deltaDeg) <= 30.0f))
            return "delta_deg must be -30..30 [deg] - this example's cap, not '" + args[1] + "'";
        if (args.size() > 2 && !(arm_helper::ParseFloat(args[2].c_str(), seconds) && seconds >= 2.0f && seconds <= 20.0f))
            return "seconds must be 2..20 (the whole out-and-back), not '" + args[2] + "'";
        if (args.size() > 3 && args[3] != "grip")
            return "the last argument can only be the word grip, not '" + args[3] + "'";
        if (args.size() > 4) return "too many arguments";
        delta = deltaDeg * kD2R;
        grip  = args.size() > 3;
        return "";
    }

    void Start(const ArmState &s, JointStream &arm)
    {
        std::copy(s.qRef, s.qRef + kArmJoints, q0);   // start from the reference the arm follows, not the measured angle
        if (grip) arm.Gripper(JointStream::Grip::Close);
    }

    bool Update(double t, const ArmState &s, float qRef[kArmJoints], JointStream &arm)
    {
        (void)s;
        if (t > seconds) return false;

        // >>> YOUR ALGORITHM: read s (q, dq, torque, eePos ...), write the six joint references [rad] into qRef.
        // The demo below holds every joint at the start pose and moves one of them out and back.
        std::copy(q0, q0 + kArmJoints, qRef);
        const double shape = 0.5 * (1.0 - std::cos(2.0 * M_PI * t / seconds));   // 0 -> 1 -> 0, smooth
        qRef[joint] = q0[joint] + delta * static_cast<float>(shape);
        if (grip && !turned && t > 0.5 * seconds) {
            turned = true;
            arm.Gripper(JointStream::Grip::Open);
        }
        // <<< YOUR ALGORITHM
        return true;
    }
};

// ===== 2. main - leave as it is ===================================================================================

static std::atomic<bool> g_running{true};

static std::string Usage()
{
    return std::string("Usage: 07_arm_low_level [target] ") + Controller::kArgs + "\n"
        "\n"
        "Joint-level (Lv0) control of the RBA-1 arm: this program sends the six joint references itself, 500 times\n"
        "a second. It starts the arm and moves it to Ready, takes the six arm joints (app slot 22) with the gains the\n"
        "arm already uses, runs Controller, and hands the joints back. The legs stay with the robot.\n"
        "\n" + arm_helper::kTargetUsage + Controller::Help() +
        "\n"
        "While it runs it prints, twice a second, the joint furthest from its reference and the end-effector position\n"
        "(rt/rbq/arm_ee - the robot's forward kinematics of the measured angles, body frame). Ctrl-C stops the move where\n"
        "it is and hands the joints back there. Meanwhile ArmClient motion calls (MoveJ, MoveL ...) are refused (3205);\n"
        "the gripper calls still work. Gains are physical values (Nm/rad, Nm*s/rad); the robot converts them, clamps\n"
        "every value to the joint limits and moves each joint reference at most 2 rad/s toward what is sent.\n"
        "\n"
        "For your own algorithm (FK / IK / planner), change Controller in my_arm - the same structure, built already.\n";
}

int main(int argc, char *argv[])
{
    if (!arm_helper::TargetGiven(argc, argv, Usage())) return arm_helper::HelpAsked(argc, argv) ? 0 : 1;
    Controller ctl;
    const std::string bad = ctl.Configure(std::vector<std::string>(argv + 2, argv + argc));
    if (!bad.empty()) return arm_helper::ArgError(argv, bad, Usage());

    if (arm_helper::DryRun(argv[1], "LowLevel joint=" + std::to_string(ctl.joint) + " delta=" + arm_helper::Num(ctl.delta * kR2D)
                                    + " seconds=" + arm_helper::Num(ctl.seconds) + " grip=" + (ctl.grip ? "1" : "0"))) return 0;
    std::signal(SIGINT, [](int) { g_running = false; });
    if (!arm_helper::InitChannel(argv[1])) return 1;

    JointStream arm;   // app slot 22 (rbq_low_level uses 20 and rbq_wheel 21, so legs, wheels and arm can be three apps)
    if (!arm.Start(/*toReady=*/true)) return 1;
    ctl.Start(arm.State(), arm);

    const auto period = std::chrono::microseconds(1000000 / kRateHz);
    auto next = std::chrono::steady_clock::now();
    float qRef[kArmJoints];
    for (long tick = 0; g_running; tick++) {
        const double t = static_cast<double>(tick) / kRateHz;
        const ArmState s = arm.State();
        if (!ctl.Update(t, s, qRef, arm)) break;
        arm.Send(qRef);

        if (tick % (kRateHz / 2) == 0) {   // the joint furthest from its reference and the end effector, twice a second
            int worst = 0;
            for (int i = 1; i < kArmJoints; i++)
                if (std::abs(qRef[i] - s.q[i]) > std::abs(qRef[worst] - s.q[worst])) worst = i;
            std::printf("[07_arm_low_level] t %5.2f s  J%d ref %7.2f deg  measured %7.2f deg  owner %d"
                        "  EE %6.3f %6.3f %6.3f m\n",
                        t, worst, qRef[worst] * kR2D, s.q[worst] * kR2D, s.owner[worst],
                        s.eePos[0], s.eePos[1], s.eePos[2]);
        }
        next += period;
        std::this_thread::sleep_until(next);
    }

    // Controller ended (the demo at its start pose) or Ctrl-C: the arm stays at the last reference
    arm.Stop();
    return 0;
}
