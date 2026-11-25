"""00 - lists the RBA-1 arm examples and how to run them. Sends nothing.

Python port of rbq_sdk/cpp/example/src/arm/00_arm_help.cpp.
"""

import os

import rbq_sdk_py
from arm_helper import TARGET_USAGE

# The installed rbq_sdk_py: its version and the checkout it comes from (pip install -e . points at one checkout)
print(f"RBQ SDK {rbq_sdk_py.__version__} ({os.path.dirname(os.path.dirname(os.path.abspath(rbq_sdk_py.__file__)))})\n")
print(
    "RBA-1 arm examples - try them in this order. Each does one thing and exits, and starts the arm\n"
    "itself when it is not running, so any of them can be run on its own.\n"
    "\n"
    "  01_arm_state.py [target]                                  joints and end effector, twice a second (read only)\n"
    "  02_arm_power.py [target] [on|status|release|estop|clear]  start the arm / print its status / take joints back /\n"
    "                                                            arm emergency stop (damped) / end it\n"
    "  03_arm_gripper.py [target] <0|1> [seconds]                0 open / 1 close the gripper, then stop\n"
    "  04_arm_move_j.py [target] <pose>                          MoveJ to a pose: 0 home, 1 ready, 2 straight, ...\n"
    "  04_arm_move_j.py [target] [abs|rel] <J0..J5 | J<n> deg>   or by joint angles - all six or one, to them or by them\n"
    "  05_arm_move_l.py [target] [pos|ori|pose] <numbers>        MoveL / RotateL: position, orientation or both,\n"
    "                                                            or without numbers a few relative moves\n"
    "  06_arm_high_level.py [target] [fast] [joystick]           drive the end effector with a gamepad on this PC\n"
    "  07_arm_low_level.py [target] [joint] [delta_deg] [seconds] joint-level control at 500 Hz (app slot 22)\n"
    "  08_arm_torque.py [target] [stiffness] [seconds]           torque control: gravity + a spring (app slot 22)\n"
    "  08_arm_torque.py [target] J<n> <Nm> [seconds]             or a torque of your own on one joint\n"
    "  my_arm.py [target] [joy|torque]                           yours: change its Controller (moves the joints on a sine)\n"
    "\n"
    "  arm_cli.py [target]                                       test tool: run any ArmClient call by name\n"
    "\n"
    + TARGET_USAGE +
    "\n"
    "Example:\n"
    "  python3 01_arm_state.py             the simulator - keep this one open in its own terminal\n"
    "  python3 04_arm_move_j.py home       same as 0 or folding\n"
    "  python3 04_arm_move_j.py -r home    the same on the robot\n"
    "  python3 04_arm_move_j.py rel J4 -15 2    J4 turns -15 deg in 2 s\n"
    "  python3 05_arm_move_l.py ori 0 30 0 2    the end effector to pitch 30 deg, same position"
)
