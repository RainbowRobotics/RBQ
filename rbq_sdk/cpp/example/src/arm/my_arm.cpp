#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

#include "arm_helper.hpp"

// my_arm — your own joint-level (Lv0) program for the RBA-1 arm. Change Controller freely; main starts the arm, takes
// the six joints, reads the gamepad on this PC if you asked for one, calls Controller 500 times a second, sends what it
// returns and hands the joints back at the end or on Ctrl-C. 07_arm_low_level.cpp has the same structure.
// As shipped, Controller moves every joint on a sine - by position, or with `torque` by torque - or, with a gamepad,
// jogs the joints with the sticks.
//
//   ./bin-x86_64/my_arm sim          (or -r for the robot)
//   ./bin-x86_64/my_arm sim torque   the same sine by torque: kp = kd = 0 on the joint boards, the torque is yours
//   ./bin-x86_64/my_arm sim joy      with the first gamepad found (or /dev/input/js1 for that one)

using arm_helper::ArmState;
using arm_helper::JointStream;
using arm_helper::kArmJoints;
using arm_helper::Pad;

static constexpr int kRateHz = 500;   // Update() is called this often; it must return well within 1 / kRateHz

// One joint-level command - what each joint board computes every tick from it:
//     motor torque = kp * (q - q_measured) + kd * (dq - dq_measured) + tau
// Position control: q (and dq) with the arm's gains - what main fills in before Update, so setting q is enough.
// Torque control:   kp = kd = 0 and your torque in tau (add s.gravity, or the arm falls).
// In between (impedance): low kp / kd and a feed-forward tau. Units: rad, rad/s, Nm, Nm/rad, Nm*s/rad.
struct Command {
    float q[kArmJoints], dq[kArmJoints], tau[kArmJoints], kp[kArmJoints], kd[kArmJoints];
};

// ===== 1. Your controller =========================================================================================
//   Start(s, arm)                once, right after the joints are taken - s.qRef is where the arm is now
//   Update(t, s, pad, cmd, arm)  every tick: t [s] since Start, s = the arm now (ArmState in arm_helper.hpp),
//                                pad = the gamepad now (Pad in arm_helper.hpp; pad.connected is false without one);
//                                change cmd (it comes filled with "hold the current reference, the arm's gains").
//                                Return false to end.
// Also there: arm.Gripper(JointStream::Grip::Open / Close / Stop).
struct Controller {
    bool  torqueDemo = false;   // set by main from the `torque` argument
    float q0[kArmJoints];
    float q[kArmJoints];   // the gamepad demo's references

    void Start(const ArmState &s, JointStream &arm)
    {
        (void)arm;
        std::copy(s.qRef, s.qRef + kArmJoints, q0);   // where the arm is now
        std::copy(s.qRef, s.qRef + kArmJoints, q);
    }

    bool Update(double t, const ArmState &s, const Pad &pad, Command &cmd, JointStream &arm)
    {
        // >>> YOUR ALGORITHM
        if (pad.connected) {
            // Gamepad demo - each stick axis turns one joint, at most 20 deg/s; A closes and B opens the gripper,
            // BACK ends. Map the sticks and buttons to what your algorithm needs instead.
            if (pad.pressed[PAD_BACK]) return false;
            if (pad.pressed[PAD_A]) arm.Gripper(JointStream::Grip::Close);
            if (pad.pressed[PAD_B]) arm.Gripper(JointStream::Grip::Open);
            const float step = 20.0f * 0.0174533f / kRateHz;   // [rad] per tick at full stick
            q[0] -= step * pad.lx;   // J0 shoulder yaw   - left stick left / right
            q[1] += step * pad.ly;   // J1 shoulder pitch - left stick up / down
            q[2] += step * pad.ry;   // J2 elbow pitch    - right stick up / down
            q[4] += step * pad.rx;   // J4 wrist pitch    - right stick left / right
            std::copy(q, q + kArmJoints, cmd.q);
            return true;
        }
        // No gamepad - every joint +-10 deg around the start pose on a sine, twice (8 s), ending where it started
        const double T = 4.0;           // period [s]
        if (t > 2 * T) return false;
        const float amp = 10.0f * 0.0174533f;   // [rad]
        const float w   = static_cast<float>(2.0 * M_PI / T);
        for (int i = 0; i < kArmJoints; i++) {
            const float qDes  = q0[i] + amp * std::sin(w * static_cast<float>(t));
            const float dqDes = amp * w * std::cos(w * static_cast<float>(t));
            if (!torqueDemo) {
                cmd.q[i] = qDes;   // position: the joint boards' PD (the arm's kp / kd) follows it
                continue;
            }
            // torque: the boards' PD off (kp = kd = 0), so the torque below is all the joint gets - gravity plus a
            // spring and damper of your own toward the same sine. Your torque controller goes here.
            static constexpr float kSpring[kArmJoints] = {300.f, 300.f, 150.f, 150.f, 150.f, 150.f};   // [Nm/rad]
            static constexpr float kDamper[kArmJoints] = { 10.f,  10.f,   5.f,   5.f,   5.f,   5.f};  // [Nm*s/rad]
            cmd.kp[i]  = 0.f;
            cmd.kd[i]  = 0.f;
            cmd.q[i]   = s.q[i];   // no position gain - only keeps the reference next to the arm for the hand-back
            cmd.dq[i]  = 0.f;
            cmd.tau[i] = s.gravity[i] + kSpring[i] * (qDes - s.q[i]) + kDamper[i] * (dqDes - s.dq[i]);
        }
        // <<< YOUR ALGORITHM
        return true;
    }
};

// ===== 2. main ====================================================================================================

static std::atomic<bool> g_running{true};

int main(int argc, char *argv[])
{
    const std::string usage = std::string("Usage: my_arm [target] [joy | <gamepad device> | torque]\n\n") + arm_helper::kTargetUsage +
        "  joy        read the first gamepad found on this PC (or $JOY); or name one, e.g. /dev/input/js1\n"
        "  torque     the sine by torque: kp = kd = 0 on the joint boards, gravity + a spring and damper of its own\n"
        "\n"
        "It starts the arm and moves it to Ready first, so the start pose is Ready.\n"
        "Without a gamepad every joint moves +-10 deg on a sine for 8 s. With one the sticks jog the joints\n"
        "(left: J0 / J1, right: J4 / J2, up to 20 deg/s), A / B close / open the gripper and BACK ends.\n"
        "Joint-level control has no workspace or collision checks - jog slowly and keep clear of the body.\n";
    if (!arm_helper::TargetGiven(argc, argv, usage)) return arm_helper::HelpAsked(argc, argv) ? 0 : 1;
    if (argc > 3) return arm_helper::ArgError(argv, "too many arguments", usage);
    const bool torque = argc > 2 && std::string(argv[2]) == "torque";
    const std::string joyArg = argc > 2 && !torque ? argv[2] : "";
    if (!joyArg.empty() && joyArg != "joy" && joyArg.compare(0, 5, "/dev/") != 0)
        return arm_helper::ArgError(argv, "unexpected argument '" + joyArg + "' - joy, a gamepad device or torque", usage);
    if (arm_helper::DryRun(argv[1], "MyArm joystick=" + (joyArg.empty() ? std::string("-") : joyArg)
                                    + " torque=" + (torque ? "1" : "0"))) return 0;

    arm_helper::Gamepad gamepad;   // opened before the arm moves, so a missing pad stops here
    if (!joyArg.empty() && !gamepad.Open(joyArg == "joy" ? "" : joyArg)) return 1;

    std::signal(SIGINT, [](int) { g_running = false; });
    if (!arm_helper::InitChannel(argv[1])) return 1;

    JointStream arm;   // app slot 22
    if (!arm.Start(/*toReady=*/true)) return 1;
    for (int i = 0; i < 50 && torque && !arm.State().gravityValid; i++) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (torque && !arm.State().gravityValid) {
        std::printf("[my_arm] nothing on rt/rbq/arm_gravity - the robot software is older than this example\n");
        return 1;   // ~JointStream hands the joints back
    }
    const ArmState s0 = arm.State();   // the arm's own gains - each tick's default, and what the hand-back holds with
    Controller ctl;
    ctl.torqueDemo = torque;
    ctl.Start(s0, arm);

    const auto period = std::chrono::microseconds(1000000 / kRateHz);
    auto next = std::chrono::steady_clock::now();
    Command cmd;
    std::copy(s0.qRef, s0.qRef + kArmJoints, cmd.q);
    for (long tick = 0; g_running; tick++) {
        const ArmState s = arm.State();
        const Pad pad = gamepad.Read();
        // default: hold the last reference with the arm's gains, no feed-forward
        std::fill(cmd.dq, cmd.dq + kArmJoints, 0.f);
        std::fill(cmd.tau, cmd.tau + kArmJoints, 0.f);
        std::copy(s0.kp, s0.kp + kArmJoints, cmd.kp);
        std::copy(s0.kd, s0.kd + kArmJoints, cmd.kd);
        if (!ctl.Update(static_cast<double>(tick) / kRateHz, s, pad, cmd, arm)) break;
        arm.SetGains(cmd.kp, cmd.kd);
        arm.Send(cmd.q, cmd.dq, cmd.tau);
        if (tick % (kRateHz / 2) == 0)   // the end effector twice a second (rt/rbq/arm_ee: FK of the measured angles, body frame)
            std::printf("[my_arm] t %5.2f s  EE %6.3f %6.3f %6.3f m  rpy %6.1f %6.1f %6.1f deg\n",
                        static_cast<double>(tick) / kRateHz, s.eePos[0], s.eePos[1], s.eePos[2],
                        s.eeRpy[0] * 57.2958f, s.eeRpy[1] * 57.2958f, s.eeRpy[2] * 57.2958f);
        next += period;
        std::this_thread::sleep_until(next);
    }

    arm.SetGains(s0.kp, s0.kd);   // hand back in position control with the arm's gains - not with a torque demo's kp 0
    arm.Stop();                   // the joints go back to the robot; the arm stays at the last reference
    return 0;
}
