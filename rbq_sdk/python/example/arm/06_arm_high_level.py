"""06 - drive the RBA-1 end effector with a gamepad on this PC (the arm counterpart of rbq_high_level.py).

Python port of rbq_sdk/cpp/example/src/arm/06_arm_high_level.cpp. Starts the arm, turns task-space control on
(through Ready if it is off), switches the arm to external control and sends end-effector velocities 50 times a
second. The robot stops the arm within 0.5 s when they stop coming.

Usage:
    python3 06_arm_high_level.py [target] [fast] [joystick]

    fast       full stick = 0.5 m/s and 0.8 rad/s, the robot's limit (default 0.1 m/s and 0.5 rad/s)
    joystick   gamepad device, e.g. /dev/input/js0 (default: the first gamepad found, or $JOY)

Gamepad (X mode) - the same as the app's Manual Control:
                      RB released                         RB held
    left stick  up/dn forward / back                      up / down          up to 0.1 m/s (fast 0.5)
    left stick  l/r   left / right                        -
    right stick l/r   yaw                                 roll               up to 0.5 rad/s (fast 0.8)
    right stick up/dn pitch                               gripper open / close
    B held            slow (0.4x position, 0.5x rotation)
    RT + Y / RT + A   open / close the gripper while held
    BACK or Ctrl-C    stop and quit

Moving the robot's own gamepad (the app's sticks) for 0.3 s turns external control off; this program then quits.
Directions are the arm controller's foot-centre frame (x forward, y left, z up), like the app joystick.
"""

import os
import sys
import time

from rbq_sdk_py.core.channel import ChannelPublisher

from arm_helper import (AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_X, AXIS_RIGHT_Y, AXIS_RT, PAD_A, PAD_B, PAD_BACK, PAD_RB,
                        PAD_Y, TARGET_USAGE, Joystick, arg_error, dry_run, help_asked, init_channel, initialize_arm,
                        joy_name, make_client, pick_joy_path, print_target_hint, start_task_control, target_given)

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "idl")))
from _rbq_idl import ArmHighLevelCommand_, Bool_  # noqa: E402

LIN_MAX = 0.10   # [m/s]   full stick - this example's cap
ANG_MAX = 0.50   # [rad/s]
LIN_FAST = 0.50  # with `fast`: the robot's own cap, the same as the app's manual control
ANG_FAST = 0.80
DEAD = 0.20      # the robot's Manual Control dead zone
SLOW_LIN = 0.4   # B held: 0.2 / 0.5 of the robot's Manual Control
SLOW_ANG = 0.5   #         0.4 / 0.8
RATE_HZ = 50

def deadzone(v: float) -> float:
    return 0.0 if abs(v) < DEAD else (1.0 if v > 0 else -1.0) * (abs(v) - DEAD) / (1.0 - DEAD)


def main():
    if not target_given(sys.argv, __doc__ + TARGET_USAGE):
        sys.exit(0 if help_asked(sys.argv) else 1)
    # after the target: the word fast and / or a gamepad device, in either order
    fast, joy_arg = False, None
    for a in sys.argv[2:]:
        if a == "fast" and not fast:
            fast = True
        elif joy_arg is None:
            joy_arg = a
        else:
            sys.exit(arg_error(sys.argv, "too many arguments", __doc__))
    lin_cap, ang_cap = (LIN_FAST, ANG_FAST) if fast else (LIN_MAX, ANG_MAX)

    if dry_run(sys.argv[1], f"HighLevel fast={1 if fast else 0} joystick={joy_arg or '-'}"):
        sys.exit(0)
    path = pick_joy_path(sys.argv[:2] + ([joy_arg] if joy_arg else []))
    if path is None:
        print("[06_arm_high_level] no gamepad on /dev/input/js* - is the dongle in and the pad on?")
        if joy_arg:
            print_target_hint(sys.argv)
        sys.exit(1)
    joy = Joystick(path)
    print(f"[06_arm_high_level] gamepad: {joy_name(path) or '?'} ({path}), full stick {lin_cap:.1f} m/s "
          f"{ang_cap:.1f} rad/s{' (fast)' if fast else ''}")

    if not init_channel(sys.argv[1]):
        sys.exit(1)
    arm = make_client()
    if not initialize_arm(arm, to_ready=False) or not start_task_control(arm):
        sys.exit(1)

    pub_cmd = ChannelPublisher("rt/rbq/cmd/arm/high_level", ArmHighLevelCommand_)
    pub_cmd.Init()
    pub_mode = ChannelPublisher("rt/rbq/cmd/arm/switch_control_mode", Bool_)
    pub_mode.Init()
    time.sleep(0.2)  # DDS discovery for the two publishers
    pub_mode.Write(Bool_(data=True))
    print("[06_arm_high_level] external control on - drive with the gamepad, BACK or Ctrl-C to quit")

    on = True
    tick = 0
    next_tick = time.monotonic()
    try:
        while True:
            joy.poll()
            if joy.btn[PAD_BACK]:
                break
            # Manual Control (ManiControl setJoy_CommandRef_*, Gripper_PWM_Control): RB switches what the sticks do
            ly = -deadzone(joy.ax[AXIS_LEFT_Y])   # + = stick up
            lx = -deadzone(joy.ax[AXIS_LEFT_X])   # + = stick left
            ry = -deadzone(joy.ax[AXIS_RIGHT_Y])  # + = stick up
            rx = deadzone(joy.ax[AXIS_RIGHT_X])   # + = stick right
            rb = joy.btn[PAD_RB]
            rt = joy.ax[AXIS_RT] > 0.0            # trigger more than half pressed
            lin_max = lin_cap * (SLOW_LIN if joy.btn[PAD_B] else 1.0)
            ang_max = ang_cap * (SLOW_ANG if joy.btn[PAD_B] else 1.0)
            if rb:
                lin = [0.0, 0.0, ly * lin_max]
                ang = [rx * ang_max, 0.0, 0.0]
            else:
                lin = [ly * lin_max, lx * lin_max, 0.0]
                ang = [0.0, ry * ang_max, -rx * ang_max]
            if rb and abs(ry) > 0.3:
                grip = 1 if ry > 0 else -1
            elif rt and joy.btn[PAD_Y]:
                grip = 1
            elif rt and joy.btn[PAD_A]:
                grip = -1
            else:
                grip = 0
            # also when all zero - the robot treats a silent stream as a lost link
            pub_cmd.Write(ArmHighLevelCommand_(lin_vel=lin, ang_vel=ang, gripper=grip))

            if tick % RATE_HZ == 0:  # once a second: still in external control?
                code, state = arm.GetState()
                if code == 0 and state and state.get("high_level") != "1":
                    print("[06_arm_high_level] external control was turned off on the robot (its gamepad moved, or an arm emergency stop)\n"
                          "  A browser with the robot's web page or the app open reads this same gamepad and drives\n"
                          "  the robot with it - close that page, or use another gamepad there.")
                    on = False
                    break
            if tick % (RATE_HZ // 2) == 0:
                print(f"[06_arm_high_level] lin {lin[0]:+5.2f} {lin[1]:+5.2f} {lin[2]:+5.2f} m/s  "
                      f"ang {ang[0]:+5.2f} {ang[1]:+5.2f} {ang[2]:+5.2f} rad/s  grip {grip:+d}")
            tick += 1
            next_tick += 1.0 / RATE_HZ
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        pass

    # stop: a last zero command, then hand control back
    pub_cmd.Write(ArmHighLevelCommand_(lin_vel=[0.0] * 3, ang_vel=[0.0] * 3, gripper=0))
    if on:
        pub_mode.Write(Bool_(data=False))
    time.sleep(0.1)
    print("[06_arm_high_level] external control off")


if __name__ == "__main__":
    main()
