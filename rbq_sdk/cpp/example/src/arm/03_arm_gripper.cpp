#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <rbq_sdk/dds/ArmClient.hpp>
#include <rbq_sdk/dds/ChannelFactory.hpp>

#include "arm_helper.hpp"

// 03 — open or close the RBA-1 gripper for a while, then stop it.

static std::string Usage()
{
    return std::string(
        "Usage: 03_arm_gripper [target] <action> [seconds]\n"
        "\n"
        "Opens or closes the RBA-1 gripper for a while, then stops it. Starts the arm first if needed.\n"
        "\n") + arm_helper::kTargetUsage +
        "  action     0 or open, 1 or close (any case)\n"
        "  seconds    the gripper keeps moving until it is stopped; this example stops it after\n"
        "             `seconds` (0 < seconds <= 5, default 1)\n"
        "\n"
        "Examples:\n"
        "  03_arm_gripper sim 1          close for 1 s\n"
        "  03_arm_gripper sim open 2     open for 2 s\n";
}

int main(int argc, char *argv[])
{
    if (!arm_helper::TargetGiven(argc, argv, Usage())) return arm_helper::HelpAsked(argc, argv) ? 0 : 1;
    // 0 / open, 1 / close (any case)
    const std::string arg = (argc > 2) ? arm_helper::Lower(argv[2]) : "";
    const std::string action = (arg == "0" || arg == "open") ? "open" : (arg == "1" || arg == "close") ? "close" : "";
    float seconds = 1.0f;
    if (argc < 3)
        return arm_helper::ArgError(argv, "which action? 0 or open, 1 or close", Usage());
    if (action.empty())
        return arm_helper::ArgError(argv, std::string("action must be 0 / open or 1 / close, not '") + argv[2] + "'", Usage());
    if (argc > 3 && !(arm_helper::ParseFloat(argv[3], seconds) && seconds > 0.0f && seconds <= 5.0f))
        return arm_helper::ArgError(argv, std::string("seconds must be more than 0 and at most 5, not '") + argv[3] + "'", Usage());
    if (arm_helper::DryRun(argv[1], "Gripper action=" + action + " seconds=" + arm_helper::Num(seconds))) return 0;
    if (!arm_helper::InitChannel(argv[1])) return 1;
    rbq_sdk::ArmClient arm;
    arm.SetTimeout(2.0f);

    if (!arm_helper::InitializeArm(arm, /*toReady=*/false, /*inEstopOk=*/true)) return 1;
    const bool ok = action == "open" ? arm_helper::CheckQuiet("GripperOpen", arm.GripperOpen())
                                     : arm_helper::CheckQuiet("GripperClose", arm.GripperClose());
    if (!ok) return 1;
    std::this_thread::sleep_for(std::chrono::duration<float>(seconds));
    if (!arm_helper::CheckQuiet("GripperStop", arm.GripperStop())) return 1;
    std::printf("[arm] gripper %s for %s s, then stopped\n", action == "open" ? "opened" : "closed", arm_helper::Num(seconds).c_str());
    return 0;
}
