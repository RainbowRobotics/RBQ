"""04 - joint-space move of the RBA-1 arm: to one of its built-in poses (MoveJ), or by joint angles - to them (MoveJ) or
by them from where the arm is (MoveJDelta), all six joints or one.

Python port of rbq_sdk/cpp/example/src/arm/04_arm_move_j.cpp.

Usage:
    python3 04_arm_move_j.py [target] <pose>
    python3 04_arm_move_j.py [target] [abs|rel] <J0 J1 J2 J3 J4 J5> [seconds | vel=<deg/s> acc=<deg/s^2>]
    python3 04_arm_move_j.py [target] [abs|rel] J<n> <degrees>      [seconds | vel=<deg/s> acc=<deg/s^2>]
        abs       the angles are where the joints go (MoveJ) - the default
        rel       the angles are how far the joints turn from where they are commanded now (MoveJDelta)
        J0 .. J5  six joint angles [deg]. The joints may reach J1 0..180, J2 -180..2, the others -360..360
        J<n> deg  only joint n (J0 .. J5); the others stay
        seconds   every moving joint arrives together in this time (0.5 to 20 s, both included), if that keeps the joints
                  under 90 deg/s and 180 deg/s^2 - otherwise refused with the time the move needs
        vel acc   or the robot picks the shortest time that keeps the fastest joint under vel [deg/s]
                  and acc [deg/s^2] - default 30 and 60, max 90 and 180. With neither: the defaults

Examples:
    python3 04_arm_move_j.py sim 1                         to Ready
    python3 04_arm_move_j.py sim 0 30 -30 0 60 0 4         to these angles in 4 s
    python3 04_arm_move_j.py sim 0 30 -30 0 60 0 vel=20    no joint faster than 20 deg/s
    python3 04_arm_move_j.py sim J1 120 2                  J1 to 120 deg in 2 s
    python3 04_arm_move_j.py sim rel J4 -15 2              J4 turns -15 deg from where it is, in 2 s
    python3 04_arm_move_j.py sim rel 0 0 10 0 -10 0        J2 +10, J4 -10 deg
"""

import math
import sys

from arm_helper import (JOINT_MAX_DEG, JOINT_MIN_DEG, TARGET_USAGE, arg_error, dry_run, help_asked, init_channel,
                        initialize_arm, make_client, move, num, parse_float, parse_option, pose_from_arg, pose_usage,
                        print_target_hint, target_given)


# shown with the pose list when no pose was given - the joint-angle forms are easy to miss otherwise
ANGLE_EXAMPLES = (
    "[arm] or by joint angles [deg] (abs = to them, the default; rel = by them from where the arm is):\n"
    "    python3 04_arm_move_j.py 0 30 -30 0 60 0 4   all six joints to these angles in 4 s\n"
    "    python3 04_arm_move_j.py J1 120 2            only J1, to 120 deg in 2 s\n"
    "    python3 04_arm_move_j.py rel J4 -15 vel=20   only J4, -15 deg from where it is, at most 20 deg/s\n"
    "    python3 04_arm_move_j.py rel 0 0 10 0 -10 0  J2 +10, J4 -10 deg\n"
    "[arm] these go to the simulator on this PC. To the robot at 192.168.0.10, put -r (or robot) first:\n"
    "    python3 04_arm_move_j.py -r 1                the robot's arm to Ready\n"
    "[arm] python3 04_arm_move_j.py -h for everything\n")


def joint_arg(s):
    """"J0" .. "J5" (or "j0" ..) -> 0 .. 5, otherwise None"""
    return int(s[1]) if len(s) == 2 and s[0] in "Jj" and s[1] in "012345" else None


def main():
    if not target_given(sys.argv, __doc__ + TARGET_USAGE):
        sys.exit(0 if help_asked(sys.argv) else 1)
    args = sys.argv[2:]

    # [abs|rel], then six numbers or J<n> and a number, then an optional time or vel= / acc=. Otherwise a pose.
    mode = args[0] if args and args[0] in ("abs", "rel") else ""
    relative = mode == "rel"
    if mode:
        args = args[1:]
    joint = joint_arg(args[0]) if args else None
    angles = joint is None and len(args) > 1 and parse_float(args[0]) is not None
    by_angle = angles or joint is not None
    q, degrees, seconds, vel, acc, pose, rest = [], 0.0, 0.0, 0.0, 0.0, "", []
    if joint is None and args and len(args[0]) > 1 and args[0][0] in "Jj" and args[0][1].isdigit():
        sys.exit(arg_error(sys.argv, f"{args[0]} - the joints are J0 .. J5", __doc__))
    if mode and not by_angle:
        sys.exit(arg_error(sys.argv, f"{mode} needs six joint angles or J<n> <degrees>", __doc__))
    if joint is not None:
        degrees = parse_float(args[1]) if len(args) > 1 else None
        if degrees is None:
            sys.exit(arg_error(sys.argv, f"{args[0]} needs the degrees", __doc__))
        rest = args[2:]
    elif angles:
        if len(args) < 6:
            sys.exit(arg_error(sys.argv, f"give six joint angles J0..J5 - got {len(args)}", __doc__))
        for a in args[:6]:
            v = parse_float(a)
            if v is None:
                sys.exit(arg_error(sys.argv, f"joint angles must be numbers, not '{a}'", __doc__))
            q.append(v)
        rest = args[6:]
    else:
        pose = pose_from_arg(args[0]) if args else ""
        if not pose:
            what = f"unknown pose '{args[0]}' - one of:" if args else "which pose? one of:"
            print(f"[arm] {what}\n{pose_usage()}{ANGLE_EXAMPLES}", end="")
            if args:
                print_target_hint(sys.argv)
            sys.exit(1)
    # an absolute angle outside the joint range is refused here already; a relative one by the robot (it knows where it is)
    if by_angle and not relative:
        for j, v in ([(joint, degrees)] if joint is not None else enumerate(q)):
            if not JOINT_MIN_DEG[j] <= v <= JOINT_MAX_DEG[j]:
                sys.exit(arg_error(sys.argv, f"J{j} must be {JOINT_MIN_DEG[j]:g}..{JOINT_MAX_DEG[j]:g} deg, not {v:g}", __doc__))
    for i, a in enumerate(rest):
        if parse_option(a, "vel") is not None:
            vel = parse_option(a, "vel")
        elif parse_option(a, "acc") is not None:
            acc = parse_option(a, "acc")
        elif i == 0 and parse_float(a) is not None:
            seconds = parse_float(a)
            if seconds < 0.5:
                sys.exit(arg_error(sys.argv, f"the minimum time is 0.5 s, not {seconds:g}", __doc__))
            if seconds > 20.0:
                sys.exit(arg_error(sys.argv, f"the maximum time is 20 s, not {seconds:g}", __doc__))
        else:
            sys.exit(arg_error(sys.argv, f"after the angles: seconds (0.5..20) or vel= / acc=, not '{a}'", __doc__))
    call = f"MoveJ pose={pose}"
    if by_angle:
        call = "MoveJDelta" if relative else "MoveJ"
        call += f" joint={joint} q={num(degrees)}" if joint is not None else " q=" + ",".join(num(v) for v in q)
        call += f" seconds={num(seconds)}" if seconds > 0 else f" vel={num(vel)} acc={num(acc)}"
    if dry_run(sys.argv[1], call):
        sys.exit(0)
    if not init_channel(sys.argv[1]):
        sys.exit(1)
    arm = make_client()

    # starts the arm if needed, then goes straight to the chosen pose
    if not initialize_arm(arm, to_ready=False):
        sys.exit(1)

    if not by_angle:
        sys.exit(0 if move(arm, f"MoveJ {pose}", arm.MoveJ(pose)) else 1)  # returns when the arm has arrived
    call = arm.MoveJDelta if relative else arm.MoveJ
    # the command line is in degrees, ArmClient in radians
    target = math.radians(degrees) if joint is not None else [math.radians(v) for v in q]
    code = call(target, seconds, math.radians(vel), math.radians(acc), joint=joint)
    sys.exit(0 if move(arm, "MoveJDelta" if relative else "MoveJ", code, True, seconds, "joints 90 deg/s, 180 deg/s^2") else 1)


if __name__ == "__main__":
    main()
