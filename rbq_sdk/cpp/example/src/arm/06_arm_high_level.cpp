#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <map>
#include <string>
#include <thread>

#include <rbq_sdk/Joystick.hpp>
#include <rbq_sdk/dds/ArmClient.hpp>
#include <rbq_sdk/dds/Publisher.hpp>
#include <rbq_sdk/idl/rbq/ArmHighLevelCommand_.hpp>
#include <rbq_sdk/idl/ros2/Bool_.hpp>

#include "arm_helper.hpp"

// 06 — drive the RBA-1 end effector with a gamepad on this PC (the arm counterpart of rbq_high_level).
// Starts the arm, turns task-space control on, switches the arm to external control and streams end-effector
// velocities (rt/rbq/cmd/arm/high_level) at 50 Hz. The robot clamps them to its joystick maximum, keeps its own
// safety gates, and stops the arm within 0.5 s when the stream stops. Moving the robot's own gamepad (the app)
// takes control back. The buttons are those of the app's Manual Control.

static constexpr float kLinMax = 0.10f;   // [m/s]   full stick - this example's cap
static constexpr float kAngMax = 0.50f;   // [rad/s]
static constexpr float kLinFast = 0.50f;  // with `fast`: the robot's own cap, the same as the app's manual control
static constexpr float kAngFast = 0.80f;
static constexpr float kDead   = 0.20f;   // the robot's Manual Control dead zone
static constexpr float kSlowLin = 0.4f;   // B held: 0.2 / 0.5 of the robot's Manual Control
static constexpr float kSlowAng = 0.5f;   //         0.4 / 0.8
static constexpr int   kRateHz = 50;

// Linux joystick axis numbers (xpad / Logitech F710 in X mode); rbq_sdk/Joystick.hpp names the left stick and RX
static constexpr int kAxisRightY = 4;

static std::atomic<bool> g_running{true};

static std::string Usage()
{
    return std::string(
        "Usage: 06_arm_high_level [target] [fast] [joystick]\n"
        "\n"
        "Drive the RBA-1 end effector with a gamepad on this PC. Starts the arm, turns task-space control on\n"
        "(through Ready if it is off), switches the arm to external control and sends end-effector velocities\n"
        "50 times a second. The robot stops the arm within 0.5 s when they stop coming.\n"
        "\n") + arm_helper::kTargetUsage +
        "  fast       full stick = 0.5 m/s and 0.8 rad/s, the robot's limit (default 0.1 m/s and 0.5 rad/s)\n"
        "  joystick   gamepad device, e.g. /dev/input/js0 (default: the first gamepad found, or $JOY)\n"
        "\n"
        "Gamepad (X mode) - the same as the app's Manual Control:\n"
        "                     RB released                         RB held\n"
        "  left stick  up/dn  forward / back                      up / down          up to 0.1 m/s (fast 0.5)\n"
        "  left stick  l/r    left / right                        -\n"
        "  right stick l/r    yaw                                 roll               up to 0.5 rad/s (fast 0.8)\n"
        "  right stick up/dn  pitch                               gripper open / close\n"
        "  B held             slow (0.4x position, 0.5x rotation)\n"
        "  RT + Y / RT + A    open / close the gripper while held\n"
        "  BACK or Ctrl-C     stop and quit\n"
        "\n"
        "Moving the robot's own gamepad (the app's sticks) for 0.3 s turns external control off; this program then\n"
        "quits. Directions are the arm controller's foot-centre frame (x forward, y left, z up), like the app joystick.\n";
}

int main(int argc, char *argv[])
{
    if (!arm_helper::TargetGiven(argc, argv, Usage())) return arm_helper::HelpAsked(argc, argv) ? 0 : 1;
    // after the target: the word fast and / or a gamepad device, in either order
    bool fast = false;
    std::string joyArg;
    for (int i = 2; i < argc; i++) {
        if (std::string(argv[i]) == "fast" && !fast) fast = true;
        else if (joyArg.empty()) joyArg = argv[i];
        else return arm_helper::ArgError(argv, "too many arguments", Usage());
    }
    const float linMax = fast ? kLinFast : kLinMax;
    const float angMax = fast ? kAngFast : kAngMax;

    if (arm_helper::DryRun(argv[1], std::string("HighLevel fast=") + (fast ? "1" : "0") + " joystick="
                                    + (joyArg.empty() ? "-" : joyArg))) return 0;
    char *joyArgv[] = {argv[0], argv[1], joyArg.data()};
    const std::string jpath = PickJoyPath(joyArg.empty() ? 2 : 3, joyArgv);
    if (jpath.empty()) {
        PrintNoJoystickHint();
        if (!joyArg.empty()) arm_helper::PrintTargetHint(argv);
        return 1;
    }
    Joystick joy(jpath);
    if (!joy.Open()) return 1;
    std::printf("[06_arm_high_level] gamepad: %s, full stick %.1f m/s %.1f rad/s%s\n", jpath.c_str(), linMax, angMax,
                fast ? " (fast)" : "");

    std::signal(SIGINT, [](int) { g_running = false; });
    if (!arm_helper::InitChannel(argv[1])) return 1;

    rbq_sdk::ArmClient arm;
    arm.SetTimeout(2.0f);
    if (!arm_helper::InitializeArm(arm, /*toReady=*/false)) return 1;
    if (!arm_helper::StartTaskControl(arm)) return 1;

    rbq_sdk::Publisher<rbq_msgs::msg::dds_::ArmHighLevelCommand_> pubCmd("rt/rbq/cmd/arm/high_level");
    rbq_sdk::Publisher<std_msgs::msg::dds_::Bool_> pubMode("rt/rbq/cmd/arm/switch_control_mode");
    auto setMode = [&](bool on) {
        std_msgs::msg::dds_::Bool_ m;
        m.data(on);
        pubMode.write(m);
    };
    std::this_thread::sleep_for(std::chrono::milliseconds(200));   // DDS discovery for the two publishers
    setMode(true);
    std::printf("[06_arm_high_level] external control on - drive with the gamepad, BACK or Ctrl-C to quit\n");

    const auto period = std::chrono::microseconds(1000000 / kRateHz);
    auto next = std::chrono::steady_clock::now();
    bool on = true;
    for (long tick = 0; g_running; tick++) {
        joy.Poll();
        if (joy.Btn(PAD_BACK)) break;

        // Manual Control (ManiControl setJoy_CommandRef_*, Gripper_PWM_Control): RB switches what the sticks do
        const float ly  = -Deadzone(joy.Ax(AXIS_LEFT_Y), kDead);    // + = stick up
        const float lx  = -Deadzone(joy.Ax(AXIS_LEFT_X), kDead);    // + = stick left
        const float ry  = -Deadzone(joy.Ax(kAxisRightY), kDead);    // + = stick up
        const float rx  =  Deadzone(joy.Ax(AXIS_RIGHT_X), kDead);   // + = stick right
        const bool  rb  = joy.Btn(PAD_RB);
        const bool  rt  = joy.Ax(AXIS_RT) > 0.f;                    // trigger more than half pressed
        const float lin = linMax * (joy.Btn(PAD_B) ? kSlowLin : 1.f);
        const float ang = angMax * (joy.Btn(PAD_B) ? kSlowAng : 1.f);

        rbq_msgs::msg::dds_::ArmHighLevelCommand_ cmd;
        cmd.lin_vel()[0] = rb ? 0.f : ly * lin;
        cmd.lin_vel()[1] = rb ? 0.f : lx * lin;
        cmd.lin_vel()[2] = rb ? ly * lin : 0.f;
        cmd.ang_vel()[0] = rb ? rx * ang : 0.f;
        cmd.ang_vel()[1] = rb ? 0.f : ry * ang;
        cmd.ang_vel()[2] = rb ? 0.f : -rx * ang;
        int grip = 0;
        if (rb && ry > 0.3f) grip = 1;
        else if (rb && ry < -0.3f) grip = -1;
        else if (rt && joy.Btn(PAD_Y)) grip = 1;
        else if (rt && joy.Btn(PAD_A)) grip = -1;
        cmd.gripper(grip);
        pubCmd.write(cmd);   // also when all zero - the robot treats a silent stream as a lost link
        joy.EndFrame();

        if (tick % kRateHz == 0) {   // once a second: still in external control?
            std::map<std::string, std::string> state;
            if (arm.GetState(state) == rbq_sdk::RPC_OK && state["high_level"] != "1") {
                std::printf("[06_arm_high_level] external control was turned off on the robot (its gamepad moved, or an arm emergency stop)\n"
                            "  A browser with the robot's web page or the app open reads this same gamepad and drives\n"
                            "  the robot with it - close that page, or use another gamepad there.\n");
                on = false;
                break;
            }
        }
        if (tick % (kRateHz / 2) == 0)
            std::printf("[06_arm_high_level] lin %+5.2f %+5.2f %+5.2f m/s  ang %+5.2f %+5.2f %+5.2f rad/s  grip %+d\n",
                        cmd.lin_vel()[0], cmd.lin_vel()[1], cmd.lin_vel()[2],
                        cmd.ang_vel()[0], cmd.ang_vel()[1], cmd.ang_vel()[2], int(cmd.gripper()));
        next += period;
        std::this_thread::sleep_until(next);
    }

    // stop: a last zero command, then hand control back
    pubCmd.write(rbq_msgs::msg::dds_::ArmHighLevelCommand_{});
    if (on) setMode(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::printf("[06_arm_high_level] external control off\n");
    return 0;
}
