"""08 - torque control of the RBA-1 arm (Lv0). Instead of joint references this program sends joint torques: the
robot's gravity torque (rt/rbq/arm_gravity) plus a virtual spring that pulls every joint back to where it started.
With stiffness 0 only gravity is compensated - the arm floats and can be pushed by hand.
The second form, J<n> <Nm>, puts a torque of your own on one joint for a few seconds: that joint gets gravity + your
torque and nothing else (it accelerates), the other joints gravity + the spring.

Python port of rbq_sdk/cpp/example/src/arm/08_arm_torque.cpp. Torque mode is JointStream.send_torque(): kp 0, a small
damping kd, and your torques. Nothing else holds the arm, so leave the gravity torque out and it falls. If the torques
stop for 0.2 s the robot sends the gravity torque with damping itself, and at the end (or Ctrl-C) the joints go back to
position control where the arm is.
"""

import math
import sys
import time

from arm_helper import (ARM_JOINTS, JOINT_MAX_DEG, JOINT_MIN_DEG, JointStream, TARGET_USAGE, arg_error, dry_run, help_asked,
                        init_channel, num, parse_float, target_given)

RATE_HZ = 500
# J<n> <Nm>: what this example lets you put on one joint - well under the robot's clamp (+-60 / +-25 Nm), since nothing
# holds that joint. It stops early when the joint comes within EDGE_DEG of its limit or has turned TRAVEL_DEG.
JOINT_TORQUE_MAX = (15.0, 15.0, 8.0, 8.0, 8.0, 8.0)  # [Nm]
EDGE_DEG = 10.0
TRAVEL_DEG = 90.0


def usage():
    return ("Usage:\n    python3 08_arm_torque.py [target] [stiffness] [seconds]\n"
            "    python3 08_arm_torque.py [target] J<n> <Nm> [seconds]\n\n"
            "Torque control (Lv0) of the RBA-1 arm: this program sends the six joint torques itself, RATE_HZ times a\n"
            "second - the gravity torque the robot computes plus a spring toward the start pose: torque = gravity +\n"
            "stiffness * (start - q). It starts the arm where it is (moving it to Ready only from the folded pose), takes\n"
            "the six arm joints (app slot 22), and hands them back in position control where the arm is.\n\n"
            + TARGET_USAGE +
            "  stiffness  spring toward the start pose [Nm/rad], 0..50 (default 20); 0 = gravity only, the arm floats\n"
            "  seconds    how long to run, 2..120 s                                 (default 10)\n"
            "\n"
            "J<n> <Nm>: a torque of your own on one joint. Joint n (J0..J5) gets gravity + <Nm> and no spring, so it turns\n"
            "under that torque; the other joints get gravity + the spring (20 Nm/rad) and stay. The sign is the direction.\n"
            "  J<n>       J0 shoulder yaw  J1 shoulder pitch  J2 elbow pitch  J3 elbow roll  J4 wrist pitch  J5 wrist roll\n"
            "  Nm         -15..15 for J0 and J1, -8..8 for J2..J5\n"
            "  seconds    how long the torque is on, 0.5..20 s                      (default 2)\n"
            "It ends early when the joint comes within 10 deg of its limit or has turned 90 deg.\n"
            "\n"
            "Examples:\n"
            "  python3 08_arm_torque.py sim                 spring 20 Nm/rad for 10 s\n"
            "  python3 08_arm_torque.py sim 0 30            gravity only for 30 s - push the arm, it stays where it is left\n"
            "  python3 08_arm_torque.py sim J4 2 1          J4 (wrist pitch) +2 Nm for 1 s, the other joints held\n"
            "  python3 08_arm_torque.py sim J0 -5           J0 (shoulder yaw) -5 Nm for 2 s\n"
            "\n"
            "Twice a second it prints the joint furthest from the start pose, its torque and gravity torque [Nm], and the\n"
            "end-effector position. The robot clamps each torque to +-60 Nm (J0, J1) / +-25 Nm (J2..J5). The gravity torque\n"
            "takes the body as level - on a tilted body the arm drifts a little, which the spring or your own term must hold.\n")


def joint_arg(s):
    """"J0" .. "J5" (or j0 ..) -> 0 .. 5, anything else -1"""
    return int(s[1]) if len(s) == 2 and s[0] in "Jj" and s[1].isdigit() else -1


def wait_gravity(arm):
    for _ in range(50):
        if arm.state().gravity_valid:
            return True
        time.sleep(0.02)
    print("[08_arm_torque] nothing on rt/rbq/arm_gravity - the robot software is older than this example")
    return False


def joint_torque(joint):
    """J<n> <Nm> [seconds]: gravity + <Nm> on joint n, gravity + the spring on the others, for `seconds`"""
    argv = sys.argv
    if joint >= ARM_JOINTS:
        sys.exit(arg_error(argv, f"{argv[2]} - the joints are J0 .. J5", usage()))
    cap = JOINT_TORQUE_MAX[joint]
    if len(argv) < 4:
        sys.exit(arg_error(argv, f"{argv[2]} needs the torque [Nm]", usage()))
    nm = parse_float(argv[3])
    if nm is None or abs(nm) > cap:
        sys.exit(arg_error(argv, f"J{joint} torque must be -{num(cap)}..{num(cap)} [Nm], not '{argv[3]}'", usage()))
    seconds = 2.0
    if len(argv) > 4:
        seconds = parse_float(argv[4])
        if seconds is None or not 0.5 <= seconds <= 20.0:
            sys.exit(arg_error(argv, f"seconds must be 0.5..20, not '{argv[4]}'", usage()))
    if len(argv) > 5:
        sys.exit(arg_error(argv, "too many arguments", usage()))

    if dry_run(argv[1], f"Torque joint={joint} torque={num(nm)} seconds={num(seconds)}"):
        sys.exit(0)
    if not init_channel(argv[1]):
        sys.exit(1)

    hold = 20.0  # [Nm/rad] the other joints' spring
    with JointStream() as arm:  # app slot 22; leaving the block hands the joints back
        if not arm.start(to_ready=True, only_if_folded=True) or not wait_gravity(arm):
            sys.exit(1)
        q0 = list(arm.state().q)
        print(f"[08_arm_torque] J{joint} {nm:+.2f} Nm for {num(seconds)} s, the other joints held")
        why = "time is up"
        tick = 0
        next_tick = time.monotonic()
        try:
            while tick < int(seconds * RATE_HZ):
                s = arm.state()
                deg = math.degrees(s.q[joint])
                if deg < JOINT_MIN_DEG[joint] + EDGE_DEG or deg > JOINT_MAX_DEG[joint] - EDGE_DEG:
                    why = "the joint came within 10 deg of its limit"
                    break
                if math.degrees(abs(s.q[joint] - q0[joint])) > TRAVEL_DEG:
                    why = "the joint turned 90 deg"
                    break
                torque = [s.gravity[i] + (nm if i == joint else hold * (q0[i] - s.q[i])) for i in range(ARM_JOINTS)]
                arm.send_torque(torque)
                if tick % (RATE_HZ // 4) == 0:
                    print(f"[08_arm_torque] t {tick / RATE_HZ:5.2f} s  J{joint} off start {math.degrees(s.q[joint] - q0[joint]):7.2f} deg  "
                          f"vel {math.degrees(s.dq[joint]):7.2f} deg/s  torque {torque[joint]:6.2f} Nm  gravity {s.gravity[joint]:6.2f} Nm")
                tick += 1
                next_tick += 1.0 / RATE_HZ
                time.sleep(max(0.0, next_tick - time.monotonic()))
        except KeyboardInterrupt:
            why = "Ctrl-C"
        s = arm.state()
        print(f"[08_arm_torque] J{joint} stopped after {tick / RATE_HZ:.2f} s ({why}): "
              f"{math.degrees(s.q[joint] - q0[joint]):+.2f} deg from the start")
        # leaving the block: position control again, where the arm is


def main():
    if not target_given(sys.argv, usage()):
        sys.exit(0 if help_asked(sys.argv) else 1)
    if len(sys.argv) > 2 and joint_arg(sys.argv[2]) >= 0:
        joint_torque(joint_arg(sys.argv[2]))
        return
    stiffness, seconds = 20.0, 10.0
    if len(sys.argv) > 2:
        stiffness = parse_float(sys.argv[2])
        if stiffness is None or not 0.0 <= stiffness <= 50.0:
            sys.exit(arg_error(sys.argv, f"stiffness must be 0..50 [Nm/rad], not '{sys.argv[2]}'", usage()))
    if len(sys.argv) > 3:
        seconds = parse_float(sys.argv[3])
        if seconds is None or not 2.0 <= seconds <= 120.0:
            sys.exit(arg_error(sys.argv, f"seconds must be 2..120, not '{sys.argv[3]}'", usage()))
    if len(sys.argv) > 4:
        sys.exit(arg_error(sys.argv, "too many arguments", usage()))

    if dry_run(sys.argv[1], f"Torque stiffness={num(stiffness)} seconds={num(seconds)}"):
        sys.exit(0)
    if not init_channel(sys.argv[1]):
        sys.exit(1)

    with JointStream() as arm:  # app slot 22; leaving the block hands the joints back
        if not arm.start(to_ready=True, only_if_folded=True):
            sys.exit(1)
        for _ in range(50):
            if arm.state().gravity_valid:
                break
            time.sleep(0.02)
        if not arm.state().gravity_valid:
            print("[08_arm_torque] nothing on rt/rbq/arm_gravity - the robot software is older than this example")
            sys.exit(1)

        q0 = list(arm.state().q)
        next_tick = time.monotonic()
        try:
            for tick in range(int(seconds * RATE_HZ)):
                s = arm.state()
                torque = [s.gravity[i] + stiffness * (q0[i] - s.q[i]) for i in range(ARM_JOINTS)]
                arm.send_torque(torque)

                if tick % (RATE_HZ // 2) == 0:
                    far = max(range(ARM_JOINTS), key=lambda i: abs(q0[i] - s.q[i]))
                    print(f"[08_arm_torque] t {tick / RATE_HZ:5.2f} s  J{far} off start {math.degrees(s.q[far] - q0[far]):6.2f} deg  "
                          f"torque {torque[far]:6.2f} Nm  gravity {s.gravity[far]:6.2f} Nm  "
                          f"EE {s.ee_pos[0]:6.3f} {s.ee_pos[1]:6.3f} {s.ee_pos[2]:6.3f} m")
                next_tick += 1.0 / RATE_HZ
                time.sleep(max(0.0, next_tick - time.monotonic()))
        except KeyboardInterrupt:
            pass
        # leaving the block: position control again, where the arm is


if __name__ == "__main__":
    main()
