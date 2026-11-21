"""05 - task-space moves (MoveL family) of the RBA-1 arm. Starts the arm and turns task-space control on (through
Ready, only if it is off), then moves to (abs) or by (rel) the given position, orientation or both, or runs a short demo.

Python port of rbq_sdk/cpp/example/src/arm/05_arm_move_l.cpp.

Usage:
    python3 05_arm_move_l.py [target]                                            demo: small moves and back
    python3 05_arm_move_l.py [target] [abs|rel] pos  <x y z>                [seconds | vel= acc=]
    python3 05_arm_move_l.py [target] [abs|rel] ori  <roll pitch yaw>       [seconds | ang_vel= ang_acc=]
    python3 05_arm_move_l.py [target] [abs|rel] pose <x y z roll pitch yaw> [seconds | vel= acc= ang_vel= ang_acc=]
        (none)            demo: up 2 cm, forward 2 cm, pitch +10 deg, then back
        abs               the numbers are where the end effector goes (MoveL, RotateL) - the default
        rel               the numbers are how far it moves from where it is now (MoveLDelta, RotateLDelta):
                          x y z added to the commanded position, roll pitch yaw to the measured angles 01_arm_state.py shows
        pos x y z         MoveL: end effector to that position [m] in the body frame - the numbers
                          01_arm_state.py shows (x forward, y left, z up). The orientation stays
        ori r p y         RotateL: to that orientation roll pitch yaw [deg], as 01_arm_state.py shows it.
                          The position stays
        pose x y z r p y  MoveL: both
                          (without the word: 3 numbers are pos, 6 are pose)
        seconds           in this time (0.5 to 10 s, both included), if that stays under the max limits below - otherwise
                          refused with the time the move needs
        vel= acc=         or the robot picks the shortest time under these limits: end effector [m/s]
        ang_vel= ang_acc= and [m/s^2], orientation [deg/s] and [deg/s^2] - default 0.1 0.2 30 60,
                          max 0.25 0.5 90 180. With neither: the defaults

Examples:
    python3 05_arm_move_l.py sim
    python3 05_arm_move_l.py sim pos 0.60 0 0.30 3         to that position in 3 s
    python3 05_arm_move_l.py sim ori 0 30 0 2              pitch 30 deg, same position
    python3 05_arm_move_l.py sim pose 0.60 0 0.30 0 60 0 3  both
    python3 05_arm_move_l.py sim pos 0.60 0 0.30 vel=0.05  no faster than 5 cm/s
    python3 05_arm_move_l.py sim rel pos -0.05 0 0 2       5 cm back in 2 s
    python3 05_arm_move_l.py sim rel ori 0 0 15 2          yaw +15 deg, same position
"""

import math
import sys

from rbq_sdk_py.arm.arm_api import ARM_AXIS_X, ARM_AXIS_Y, ARM_AXIS_Z

from arm_helper import (TARGET_USAGE, arg_error, dry_run, help_asked, init_channel, initialize_arm, make_client, move, num,
                        parse_float, parse_option, start_task_control, target_given)

LIMIT_KEYS = ("vel", "acc", "ang_vel", "ang_acc")


def main():
    if not target_given(sys.argv, __doc__ + TARGET_USAGE):
        sys.exit(0 if help_asked(sys.argv) else 1)
    # [abs|rel], [pos|ori|pose], then the numbers (+ seconds), then vel= / acc= / ang_vel= / ang_acc=
    args = sys.argv[2:]
    how = args[0] if args and args[0] in ("abs", "rel") else ""
    relative = how == "rel"
    if how:
        args = args[1:]
    mode = args[0] if args and args[0] in ("pos", "ori", "pose") else ""
    if mode:
        args = args[1:]
    nums, limits = [], {}
    for a in args:
        key = next((k for k in LIMIT_KEYS if parse_option(a, k) is not None), None)
        if key:
            limits[key] = parse_option(a, key)
            continue
        if limits or parse_float(a) is None:
            sys.exit(arg_error(sys.argv, f"expected a number or vel= acc= ang_vel= ang_acc=, not '{a}'", __doc__))
        nums.append(parse_float(a))
    n = len(nums)
    if not mode and n > 0:  # without the word: 3 (+ seconds) numbers are pos, 6 (+ seconds) pose
        if n not in (3, 4, 6, 7):
            sys.exit(arg_error(sys.argv, f"give 3 numbers (x y z) or 6 (x y z roll pitch yaw) [+ seconds] - got {n}", __doc__))
        mode = "pose" if n >= 6 else "pos"
    need = 6 if mode == "pose" else 3
    if mode and n not in (need, need + 1):
        sys.exit(arg_error(sys.argv, f"{mode} needs {need} numbers [+ seconds] - got {n}", __doc__))
    if not mode and limits:
        sys.exit(arg_error(sys.argv, "vel= / acc= need a position", __doc__))
    if not mode and how:
        sys.exit(arg_error(sys.argv, f"{how} needs pos, ori or pose and its numbers", __doc__))
    seconds = nums[-1] if n == need + 1 else 0.0
    if n == need + 1 and limits:
        sys.exit(arg_error(sys.argv, "give seconds or vel= / acc=, not both", __doc__))
    if n == need + 1 and seconds < 0.5:
        sys.exit(arg_error(sys.argv, f"the minimum time is 0.5 s, not {seconds:g}", __doc__))
    if n == need + 1 and seconds > 10.0:
        sys.exit(arg_error(sys.argv, f"the maximum time is 10 s, not {seconds:g}", __doc__))
    if mode == "ori" and limits.keys() - {"ang_vel", "ang_acc"}:
        sys.exit(arg_error(sys.argv, "ori takes ang_vel= / ang_acc=", __doc__))
    call = "RelativeMoves"
    if mode:
        names = ("roll", "pitch", "yaw") if mode == "ori" else ("x", "y", "z", "roll", "pitch", "yaw")
        call = ("RotateL" if mode == "ori" else "MoveL") + ("Delta" if relative else "") + "".join(f" {k}={num(v)}" for k, v in zip(names, nums[:need]))
        call += f" seconds={num(seconds)}" if seconds > 0 else "".join(f" {k}={num(limits.get(k, 0))}" for k in LIMIT_KEYS)
    if dry_run(sys.argv[1], call):
        sys.exit(0)
    if not init_channel(sys.argv[1]):
        sys.exit(1)
    arm = make_client()

    # task-space control through Ready only when it is not on yet - otherwise the arm moves from where it is
    if not initialize_arm(arm, to_ready=False) or not start_task_control(arm):
        sys.exit(1)

    if mode:
        # the command line is in degrees, ArmClient in radians: roll pitch yaw and the turning limits
        limits = {k: (math.radians(v) if k.startswith("ang_") else v) for k, v in limits.items()}
        rpy = [math.radians(a) for a in nums[3:6]] if mode == "pose" else None
        if mode == "ori":
            rotate = arm.RotateLDelta if relative else arm.RotateL
            code = rotate(*[math.radians(a) for a in nums[0:3]], seconds=seconds, **limits)
        elif relative:
            code = arm.MoveLDelta(nums[0], nums[1], nums[2], seconds=seconds, rpy=rpy, **limits)
        else:
            code = arm.MoveL(nums[0], nums[1], nums[2], seconds, rpy, **limits)
        what = ("RotateL" if mode == "ori" else "MoveL") + ("Delta" if relative else "")
        sys.exit(0 if move(arm, what, code, True, seconds,
                           "0.25 m/s, 0.5 m/s^2, turning 90 deg/s, 180 deg/s^2") else 1)

    # each relative move takes 0.3 s on the robot; Wait() returns when it is done
    steps = [
        ("MoveLDelta z +0.02 m",   lambda: arm.MoveLDelta(ARM_AXIS_Z, 0.02)),
        ("MoveLDelta x +0.02 m",   lambda: arm.MoveLDelta(ARM_AXIS_X, 0.02)),
        ("RotateLDelta pitch +10", lambda: arm.RotateLDelta(ARM_AXIS_Y, math.radians(10.0))),
        ("RotateLDelta pitch -10", lambda: arm.RotateLDelta(ARM_AXIS_Y, math.radians(-10.0))),
        ("MoveLDelta x -0.02 m",   lambda: arm.MoveLDelta(ARM_AXIS_X, -0.02)),
        ("MoveLDelta z -0.02 m",   lambda: arm.MoveLDelta(ARM_AXIS_Z, -0.02)),
    ]
    for what, call in steps:
        if not move(arm, what, call(), wait=5.0):
            sys.exit(1)


if __name__ == "__main__":
    main()
