"""07 - joint-level (Lv0) control of the RBA-1 arm. For your own algorithm (FK / IK / planner) change my_arm.py,
which has the same two parts.

Python port of rbq_sdk/cpp/example/src/arm/07_arm_low_level.cpp. The file has two parts:
  1. Controller - YOUR part. It reads the arm state every tick and returns the six joint references.
     Out of the box it moves one joint out and back; replace it with your algorithm.
  2. main       - leave it as it is. It starts the arm, takes the six joints (app slot 22) through
     arm_helper.JointStream, calls Controller RATE_HZ times a second, and hands the joints back where they are when
     Controller ends, on Ctrl-C or on an exception. It knows nothing about the demo, so any Controller with the same
     members runs.

The robot moves each joint reference at most 2 rad/s toward what you send and clamps it to the joint limits, so a
wrong reference makes the arm travel there at that speed rather than snap. The legs stay with the robot.
"""

import math
import sys
import time

from arm_helper import (ARM_JOINTS, JointStream, TARGET_USAGE, arg_error, dry_run, help_asked, init_channel, num, parse_float,
                        target_given)

RATE_HZ = 500  # update() is called this often and must return well within 1 / RATE_HZ - lower it if Python cannot keep up


# ===== 1. Your controller =========================================================================================
# Keep these members; change what is inside them.
#   ARGS / HELP      the arguments after [target], for the usage line and -h
#   configure(args)  reads those arguments; returns "" when fine, or what is wrong
#   start(s, arm)    once, right after the joints are taken - s.q_ref is where the arm is now
#   update(t, s, arm)
#                    every tick: t [s] since start, s = the arm now (ArmState in arm_helper.py);
#                    returns the six joint references [rad], or None to end.
# Also there when you need them: arm.gripper("open" / "close" / "stop") - returns at once;
# arm.set_gains(kp, kd); arm.send(q, vel, torque) for feed-forward velocity [rad/s] and torque [Nm].
class Controller:
    ARGS = "[joint] [delta_deg] [seconds] [grip]"
    HELP = (
        "  joint      the joint to move                                                     (default 0)\n"
        "             0 J0 shoulder yaw   1 J1 shoulder pitch   2 J2 elbow pitch\n"
        "             3 J3 elbow roll     4 J4 wrist pitch      5 J5 wrist roll\n"
        "  delta_deg  how far it goes before coming back [deg], -30..30; the sign is the direction (default 15)\n"
        "  seconds    time for the whole out-and-back, 2..20 s                              (default 4)\n"
        "  grip       write the word grip to also close the gripper on the way out and open it on the way back\n"
        "\n"
        "The demo moves ONE joint away by delta_deg and back along a smooth cosine curve - a single out-and-back,\n"
        "not a repeating sine.\n"
        "\n"
        "Examples:\n"
        "  python3 07_arm_low_level.py sim                  J0 15 deg out and back in 4 s\n"
        "  python3 07_arm_low_level.py sim 4 -20 6          J4 (wrist pitch) -20 deg out and back in 6 s\n"
        "  python3 07_arm_low_level.py sim 0 20 4 grip      ... and close / open the gripper on the way\n"
    )

    def __init__(self):
        # the demo's settings
        self.joint, self.delta, self.seconds, self.grip = 0, math.radians(15.0), 4.0, False
        self.q0 = None
        self.turned = False

    def configure(self, args):
        if len(args) > 0:
            if not (args[0].lstrip("-").isdigit() and 0 <= int(args[0]) < ARM_JOINTS):
                return f"joint must be 0..5 (J0..J5), not '{args[0]}'"
            self.joint = int(args[0])
        if len(args) > 1:
            d = parse_float(args[1])
            if d is None or abs(d) > 30.0:
                return f"delta_deg must be -30..30 [deg] - this example's cap, not '{args[1]}'"
            self.delta = math.radians(d)
        if len(args) > 2:
            s = parse_float(args[2])
            if s is None or not 2.0 <= s <= 20.0:
                return f"seconds must be 2..20 (the whole out-and-back), not '{args[2]}'"
            self.seconds = s
        if len(args) > 3 and args[3] != "grip":
            return f"the last argument can only be the word grip, not '{args[3]}'"
        if len(args) > 4:
            return "too many arguments"
        self.grip = len(args) > 3
        return ""

    def start(self, s, arm):
        self.q0 = list(s.q_ref)  # start from the reference the arm follows, not the measured angle
        if self.grip:
            arm.gripper("close")

    def update(self, t, s, arm):
        if t > self.seconds:
            return None

        # >>> YOUR ALGORITHM: read s (q, dq, torque, ee_pos ...), return the six joint references [rad].
        # The demo below holds every joint at the start pose and moves one of them out and back.
        q_ref = list(self.q0)
        shape = 0.5 * (1.0 - math.cos(2.0 * math.pi * t / self.seconds))  # 0 -> 1 -> 0, smooth
        q_ref[self.joint] = self.q0[self.joint] + self.delta * shape
        if self.grip and not self.turned and t > 0.5 * self.seconds:
            self.turned = True
            arm.gripper("open")
        # <<< YOUR ALGORITHM
        return q_ref


# ===== 2. main - leave as it is ===================================================================================

def usage():
    return (f"Usage:\n    python3 07_arm_low_level.py [target] {Controller.ARGS}\n\n"
            "Joint-level (Lv0) control of the RBA-1 arm: this program sends the six joint references itself, RATE_HZ\n"
            "times a second. It starts the arm and moves it to Ready, takes the six arm joints (app slot 22) with the\n"
            "gains the arm already uses, runs Controller, and hands the joints back. The legs stay with the robot.\n\n"
            + TARGET_USAGE + Controller.HELP +
            "\nWhile it runs it prints, twice a second, the joint furthest from its reference and the end-effector\n"
            "position (rt/rbq/arm_ee - the robot's forward kinematics of the measured angles, body frame). Ctrl-C stops\n"
            "the move where it is and hands the joints back there. Meanwhile ArmClient motion calls (MoveJ, MoveL ...) are\n"
            "refused (3205); the gripper calls still work. Gains are physical values (Nm/rad, Nm*s/rad); the robot\n"
            "converts them, clamps every value to the joint limits and moves each joint reference at most 2 rad/s.\n\n"
            "For your own algorithm (FK / IK / planner), change Controller in my_arm - the same structure, built already.\n")


def main():
    if not target_given(sys.argv, usage()):
        sys.exit(0 if help_asked(sys.argv) else 1)
    ctl = Controller()
    bad = ctl.configure(sys.argv[2:])
    if bad:
        sys.exit(arg_error(sys.argv, bad, usage()))

    if dry_run(sys.argv[1], f"LowLevel joint={ctl.joint} delta={num(math.degrees(ctl.delta))} seconds={num(ctl.seconds)} "
                            f"grip={1 if ctl.grip else 0}"):
        sys.exit(0)
    if not init_channel(sys.argv[1]):
        sys.exit(1)

    # app slot 22 (rbq_low_level uses 20 and rbq_wheel 21, so legs, wheels and arm can be three apps)
    with JointStream() as arm:
        if not arm.start(to_ready=True):
            sys.exit(1)
        ctl.start(arm.state(), arm)

        period = 1.0 / RATE_HZ
        next_tick = time.monotonic()
        tick = 0
        late = 0
        try:
            while True:
                t = tick / RATE_HZ
                s = arm.state()
                q_ref = ctl.update(t, s, arm)
                if q_ref is None:
                    break
                arm.send(q_ref)

                if tick % (RATE_HZ // 2) == 0:  # the joint furthest from its reference and the end effector, twice a second
                    w = max(range(ARM_JOINTS), key=lambda i: abs(q_ref[i] - s.q[i]))
                    print(f"[07_arm_low_level] t {t:5.2f} s  J{w} ref {math.degrees(q_ref[w]):7.2f} deg  "
                          f"measured {math.degrees(s.q[w]):7.2f} deg  owner {s.owner[w]}  "
                          f"EE {s.ee_pos[0]:6.3f} {s.ee_pos[1]:6.3f} {s.ee_pos[2]:6.3f} m")
                tick += 1
                next_tick += period
                wait = next_tick - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
                else:
                    late += 1
        except KeyboardInterrupt:
            pass
        if late > tick // 10:
            print(f"[07_arm_low_level] {late} of {tick} ticks ran late - lower RATE_HZ or speed up update()")
        # leaving the block hands the joints back: the arm stays at the last reference


if __name__ == "__main__":
    main()
