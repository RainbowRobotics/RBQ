#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <rbq_sdk/dds/ArmClient.hpp>
#include <rbq_sdk/dds/ChannelFactory.hpp>

#include "arm_helper.hpp"

// Test tool, not one of the numbered examples: an interactive CLI for every ArmClient call (like rbq_sport_client).
// Typical session:  auto_start -> go_ready -> control_start -> ee_up / ee_forward / move_ee ... -> go_folding

struct Command
{
    std::string name;
    int id;
    std::string what;
};

static const std::vector<Command> kCommands = {
    {"get_state",      0, "print the arm state"},
    {"auto_start",     1, "power-up sequence (axis init + reference streaming)"},
    {"go_ready",       2, "joint move to Ready - the start pose for task-space control"},
    {"go_folding",     3, "joint move to Folding (walking pose)"},
    {"go_packing",     4, "joint move to Packing"},
    {"control_start",  5, "task-space control on (from Ready)"},
    {"ee_forward",     6, "end effector +2 cm x"},
    {"ee_back",        7, "end effector -2 cm x"},
    {"ee_left",        8, "end effector +2 cm y"},
    {"ee_right",       9, "end effector -2 cm y"},
    {"ee_up",         10, "end effector +2 cm z"},
    {"ee_down",       11, "end effector -2 cm z"},
    {"ee_pitch_plus", 12, "end effector +10 deg pitch"},
    {"ee_pitch_minus",13, "end effector -10 deg pitch"},
    {"ee_yaw_plus",   14, "end effector +10 deg yaw"},
    {"ee_yaw_minus",  15, "end effector -10 deg yaw"},
    {"move_ee",       16, "end effector to x y z [m] in t [s] - asks for the numbers"},
    {"gripper_open",  17, "open the gripper (until gripper_stop)"},
    {"gripper_close", 18, "close the gripper (until gripper_stop)"},
    {"gripper_stop",  19, "stop the gripper"},
    {"release_joints",20, "hand every SDK-held arm joint back to the arm controller"},
    {"stop",          21, "stop the running MoveJ / MoveL where the arm is"},
    {"wait",          22, "wait until the last motion has finished"},
    {"estop",         23, "arm emergency stop - joints damped (kp 0) until clear_estop"},
    {"clear_estop",   24, "end the emergency stop - the arm holds where it is"},
};

static int toInt(const std::string &s)
{
    try { size_t n = 0; int v = std::stoi(s, &n); return n == s.size() ? v : -1; }
    catch (...) { return -1; }
}

static void report(const std::string &name, int32_t code)
{
    std::cout << name << " -> " << code;
    if (code == rbq_sdk::RPC_OK) std::cout << " (ok)";
    else if (code == 3204) std::cout << " (bad parameter)";
    else if (code == 3205) std::cout << " (rejected: arm joints held by a joint-level app, or E-STOP)";
    else if (code == rbq_sdk::ARM_ERR_BUSY) std::cout << " (busy: the arm is still moving)";
    else if (code == rbq_sdk::ARM_ERR_NOT_READY) std::cout << " (not ready: task-space control is off - control_start from Ready)";
    else if (code == rbq_sdk::ARM_ERR_CANCELED) std::cout << " (canceled by stop)";
    else if (code == rbq_sdk::ARM_ERR_WAIT_TIMEOUT) std::cout << " (still moving when the wait ran out)";
    else if (code == rbq_sdk::ARM_ERR_ESTOP) std::cout << " (arm emergency stop - clear_estop first)";
    else if (code == rbq_sdk::ARM_ERR_TOO_FAST) std::cout << " (too fast for the speed limits)";
    else if (code == rbq_sdk::RPC_ERR_CLIENT_API_TIMEOUT) std::cout << " (no answer - Network running? right target?)";
    std::cout << std::endl;
}

static std::string Usage()
{
    return std::string(
        "Usage: arm_cli [target]\n"
        "\n"
        "Test tool, not one of the numbered examples: type a command name or number to run any ArmClient call\n"
        "(\"list\" shows them, \"q\" quits). Each prints the reply code.\n"
        "\n") + arm_helper::kTargetUsage;
}

int main(int argc, char **argv)
{
    if (!arm_helper::TargetGiven(argc, argv, Usage())) return arm_helper::HelpAsked(argc, argv) ? 0 : 1;
    if (!arm_helper::InitChannel(argv[1])) return 1;

    rbq_sdk::ArmClient arm;
    arm.SetTimeout(2.0f);

    std::cout << "Input \"list\" for the commands, a name or id to run one, \"q\" to quit." << std::endl;
    std::string line;
    while (std::cout << "> " << std::flush, std::getline(std::cin, line)) {
        if (line == "q" || line == "quit") break;
        if (line == "list") {
            for (const auto &c : kCommands) std::cout << "  " << c.id << "  " << c.name << "  - " << c.what << "\n";
            continue;
        }
        const Command *cmd = nullptr;
        for (const auto &c : kCommands)
            if (line == c.name || toInt(line) == c.id) cmd = &c;
        if (!cmd) {
            if (!line.empty()) std::cout << "unknown command - type \"list\"" << std::endl;
            continue;
        }

        int32_t code = rbq_sdk::RPC_OK;
        switch (cmd->id) {
        case 0: {
            std::map<std::string, std::string> state;
            code = arm.GetState(state);
            for (const auto &kv : state) std::cout << "  " << kv.first << ": " << kv.second << "\n";
            break;
        }
        case 1:  code = arm.AutoStart(); break;
        case 2:  code = arm.MoveJ(rbq_sdk::ARM_POSE_READY); break;
        case 3:  code = arm.MoveJ(rbq_sdk::ARM_POSE_FOLDING); break;
        case 4:  code = arm.MoveJ(rbq_sdk::ARM_POSE_PACKING); break;
        case 5:  code = arm.ControlStart(); break;
        case 6:  code = arm.MoveLDelta(rbq_sdk::ARM_AXIS_X,  0.02f); break;
        case 7:  code = arm.MoveLDelta(rbq_sdk::ARM_AXIS_X, -0.02f); break;
        case 8:  code = arm.MoveLDelta(rbq_sdk::ARM_AXIS_Y,  0.02f); break;
        case 9:  code = arm.MoveLDelta(rbq_sdk::ARM_AXIS_Y, -0.02f); break;
        case 10: code = arm.MoveLDelta(rbq_sdk::ARM_AXIS_Z,  0.02f); break;
        case 11: code = arm.MoveLDelta(rbq_sdk::ARM_AXIS_Z, -0.02f); break;
        case 12: code = arm.RotateLDelta(rbq_sdk::ARM_AXIS_Y,  10.0f * arm_helper::kDeg2Rad); break;   // 10 deg [rad]
        case 13: code = arm.RotateLDelta(rbq_sdk::ARM_AXIS_Y, -10.0f * arm_helper::kDeg2Rad); break;
        case 14: code = arm.RotateLDelta(rbq_sdk::ARM_AXIS_Z,  10.0f * arm_helper::kDeg2Rad); break;
        case 15: code = arm.RotateLDelta(rbq_sdk::ARM_AXIS_Z, -10.0f * arm_helper::kDeg2Rad); break;
        case 16: {
            std::cout << "x y z [m] and seconds: " << std::flush;
            std::string nums;
            std::getline(std::cin, nums);
            std::istringstream in(nums);
            float x, y, z, t;
            if (!(in >> x >> y >> z >> t)) { std::cout << "need four numbers, e.g. 0.45 0 0.25 3" << std::endl; continue; }
            code = arm.MoveL(x, y, z, t);
            break;
        }
        case 17: code = arm.GripperOpen(); break;
        case 18: code = arm.GripperClose(); break;
        case 19: code = arm.GripperStop(); break;
        case 20: code = arm.ReleaseJoints(); break;
        case 21: code = arm.Stop(); break;
        case 22: code = arm.Wait(); break;
        case 23: code = arm.EmergencyStop(); break;
        case 24: code = arm.ClearEmergencyStop(); break;
        }
        report(cmd->name, code);
    }
    return 0;
}
