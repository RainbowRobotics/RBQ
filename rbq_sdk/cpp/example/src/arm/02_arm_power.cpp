#include <iostream>
#include <map>
#include <string>

#include <rbq_sdk/dds/ArmClient.hpp>
#include <rbq_sdk/dds/ChannelFactory.hpp>

#include "arm_helper.hpp"

// 02 — start the RBA-1 arm, print its status, take its joints back from a joint-level app, or stop it in an emergency.

static std::string Usage()
{
    return std::string(
        "Usage: 02_arm_power [target] [on|status|release|estop|clear]\n"
        "\n"
        "Starts the RBA-1 arm, prints its status, takes its joints back from a joint-level app,\n"
        "or stops it in an emergency.\n"
        "The other examples start the arm themselves, so this is for checking and recovering.\n"
        "\n") + arm_helper::kTargetUsage +
        "  on         start the arm if it is not running (AutoStart) and wait until it is   (default)\n"
        "  status     print the arm status (GetState) only\n"
        "  release    give every arm joint a joint-level app still holds back to the arm controller -\n"
        "             for when such an app ended without letting go (sdk_slot is not -1)\n"
        "  estop      emergency stop: every arm joint is damped (kp 0) - the arm sinks slowly under gravity\n"
        "             and gives way when pushed. Stays on until clear. The robot's E-stop does the same.\n"
        "  clear      end the emergency stop - the arm holds the pose it sank to\n"
        "\n"
        "Status: con_start 1 = started, estop 1 = emergency stop, task_control 1 = task-space control on, motion moving / idle,\n"
        "result of the last motion call, pose, joint_owner per joint, sdk_slot holding the arm (-1 none).\n";
}

int main(int argc, char *argv[])
{
    if (!arm_helper::TargetGiven(argc, argv, Usage())) return arm_helper::HelpAsked(argc, argv) ? 0 : 1;
    const std::string action = (argc > 2) ? argv[2] : "on";
    if (action != "on" && action != "status" && action != "release" && action != "estop" && action != "clear")
        return arm_helper::ArgError(argv, "action must be on, status, release, estop or clear, not '" + action + "'", Usage());
    if (arm_helper::DryRun(argv[1], "Power action=" + action)) return 0;
    if (!arm_helper::InitChannel(argv[1])) return 1;
    rbq_sdk::ArmClient arm;
    arm.SetTimeout(2.0f);

    // done: one line (status: the whole state - that is what it is for); failed: the code and what it means
    std::map<std::string, std::string> state;
    if (action == "estop") {   // first - nothing to wait for
        if (!arm_helper::CheckQuiet("EmergencyStop", arm.EmergencyStop())) return 1;
        std::printf("[arm] emergency stop on - the arm is damped (kp 0) until: 02_arm_power [target] clear\n");
        return 0;
    }
    if (!arm_helper::FirstState(arm, state)) return 1;
    if (action == "on") {   // InitializeArm says "arm started" when it starts it
        if (state["con_start"] == "1" && state["estop"] != "1" && state["sdk_slot"] == "-1") {
            std::printf("[arm] the arm is already on\n");
            return 0;
        }
        return arm_helper::InitializeArm(arm, /*toReady=*/false) ? 0 : 1;
    }
    if (action == "status") {
        for (const auto &kv : state) std::cout << "  " << kv.first << ": " << kv.second << "\n";
        return 0;
    }
    if (action == "release") {
        if (!arm_helper::CheckQuiet("ReleaseJoints", arm.ReleaseJoints())) return 1;
        std::printf("[arm] arm joints handed back to the arm controller\n");
        return 0;
    }
    if (!arm_helper::CheckQuiet("ClearEmergencyStop", arm.ClearEmergencyStop())) return 1;
    std::printf("[arm] emergency stop cleared - the arm holds where it is; MoveJ Ready (04_arm_move_j 1) before task-space moves\n");
    return 0;
}
