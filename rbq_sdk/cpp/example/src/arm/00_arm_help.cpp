#include <cstdio>

#include "arm_helper.hpp"

// The version of the SDK these examples were built against. An SDK installed before it had this header prints none.
#if __has_include(<rbq_sdk/version.hpp>)
#include <rbq_sdk/version.hpp>
#else
#define RBQ_SDK_VERSION_STRING "unknown (installed before the SDK had a version - install it again)"
#endif

// 00 — lists the RBA-1 arm examples and how to run them. Sends nothing.

int main()
{
    std::printf("RBQ SDK %s\n\n", RBQ_SDK_VERSION_STRING);
    std::printf(
        "RBA-1 arm examples - try them in this order. Each does one thing and exits, and starts the arm\n"
        "itself when it is not running, so any of them can be run on its own.\n"
        "\n"
        "  01_arm_state [target]                                  joints and end effector, twice a second (read only)\n"
        "  02_arm_power [target] [on|status|release|estop|clear]  start the arm / print its status / take joints back /\n"
        "                                                         arm emergency stop (damped) / end it\n"
        "  03_arm_gripper [target] <0|1> [seconds]                0 open / 1 close the gripper, then stop\n"
        "  04_arm_move_j [target] <pose>                          MoveJ to a pose: 0 home, 1 ready, 2 straight, ...\n"
        "  04_arm_move_j [target] [abs|rel] <J0..J5 | J<n> deg>   or by joint angles - all six or one, to them or by them\n"
        "  05_arm_move_l [target] [pos|ori|pose] <numbers>        MoveL / RotateL: position, orientation or both,\n"
        "                                                         or without numbers a few relative moves\n"
        "  06_arm_high_level [target] [fast] [joystick]           drive the end effector with a gamepad on this PC\n"
        "  07_arm_low_level [target] [joint] [delta_deg] [seconds] joint-level control at 500 Hz (app slot 22)\n"
        "  08_arm_torque [target] [stiffness] [seconds]           torque control: gravity + a spring (app slot 22)\n"
        "  08_arm_torque [target] J<n> <Nm> [seconds]             or a torque of your own on one joint\n"
        "  my_arm [target] [joy|torque]                           yours: change its Controller (moves the joints on a sine)\n"
        "\n"
        "  arm_cli [target]                                       test tool: run any ArmClient call by name\n"
        "\n"
        "%s"
        "\n"
        "Example:\n"
        "  ./01_arm_state                  the simulator - keep this one open in its own terminal\n"
        "  ./04_arm_move_j home            same as 0 or folding\n"
        "  ./04_arm_move_j -r home         the same on the robot\n"
        "  ./04_arm_move_j rel J4 -15 2    J4 turns -15 deg in 2 s\n"
        "  ./05_arm_move_l ori 0 30 0 2    the end effector to pitch 30 deg, same position\n",
        arm_helper::kTargetUsage);
    return 0;
}
