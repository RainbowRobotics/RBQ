"""my_arm - your own joint-level (Lv0) program for the RBA-1 arm. Change Controller freely; main starts the arm, takes
the six joints, reads the gamepad on this PC if you asked for one, calls Controller RATE_HZ times a second, sends what it
returns and hands the joints back at the end, on Ctrl-C or on an exception. As shipped, Controller moves every joint on
a sine - by position, or with `torque` by torque - or, with a gamepad, jogs the joints with the sticks.

    python3 my_arm.py sim          (or -r for the robot)
    python3 my_arm.py sim torque   the same sine by torque: kp = kd = 0 on the joint boards, the torque is yours
    python3 my_arm.py sim joy      with the first gamepad found (or /dev/input/js1 for that one)
"""

import math
import sys
import time

from arm_helper import (ARM_JOINTS, PAD_A, PAD_B, PAD_BACK, TARGET_USAGE, Gamepad, JointStream, arg_error, dry_run,
                        help_asked, init_channel, target_given)

RATE_HZ = 500  # update() is called this often and must return well within 1 / RATE_HZ - lower it if Python cannot keep up


class Command:
    """One joint-level command - what each joint board computes every tick from it:
        motor torque = kp * (q - q_measured) + kd * (dq - dq_measured) + tau
    Position control: q (and dq) with the arm's gains - what main fills in before update, so setting q is enough.
    Torque control:   kp = kd = 0 and your torque in tau (add s.gravity, or the arm falls).
    In between (impedance): low kp / kd and a feed-forward tau. Units: rad, rad/s, Nm, Nm/rad, Nm*s/rad."""

    def __init__(self, q):
        self.q = list(q)
        self.dq = [0.0] * ARM_JOINTS
        self.tau = [0.0] * ARM_JOINTS
        self.kp = [0.0] * ARM_JOINTS
        self.kd = [0.0] * ARM_JOINTS


# ===== 1. Your controller =========================================================================================
#   start(s, arm)                once, right after the joints are taken - s.q_ref is where the arm is now
#   update(t, s, pad, cmd, arm)  every tick: t [s] since start, s = the arm now (ArmState in arm_helper.py),
#                                pad = the gamepad now (Pad in arm_helper.py; pad.connected is False without one);
#                                change cmd (it comes filled with "hold the current reference, the arm's gains").
#                                Return False to end.
# Also there: arm.gripper("open" / "close" / "stop").
class Controller:
    T = 4.0  # period [s]
    SPRING = (300.0, 300.0, 150.0, 150.0, 150.0, 150.0)  # [Nm/rad]   the torque demo's spring
    DAMPER = (10.0, 10.0, 5.0, 5.0, 5.0, 5.0)            # [Nm*s/rad] and damper

    def __init__(self, torque_demo=False):
        self.torque_demo = torque_demo
        self.q0 = None
        self.q = None  # the gamepad demo's references

    def start(self, s, arm):
        self.q0 = list(s.q_ref)  # where the arm is now
        self.q = list(s.q_ref)

    def update(self, t, s, pad, cmd, arm):
        # >>> YOUR ALGORITHM
        if pad.connected:
            # Gamepad demo - each stick axis turns one joint, at most 20 deg/s; A closes and B opens the gripper,
            # BACK ends. Map the sticks and buttons to what your algorithm needs instead.
            if pad.pressed[PAD_BACK]:
                return False
            if pad.pressed[PAD_A]:
                arm.gripper("close")
            if pad.pressed[PAD_B]:
                arm.gripper("open")
            step = math.radians(20.0) / RATE_HZ  # [rad] per tick at full stick
            self.q[0] -= step * pad.lx  # J0 shoulder yaw   - left stick left / right
            self.q[1] += step * pad.ly  # J1 shoulder pitch - left stick up / down
            self.q[2] += step * pad.ry  # J2 elbow pitch    - right stick up / down
            self.q[4] += step * pad.rx  # J4 wrist pitch    - right stick left / right
            cmd.q = list(self.q)
            return True
        # No gamepad - every joint +-10 deg around the start pose on a sine, twice (8 s), ending where it started
        if t > 2 * self.T:
            return False
        amp = math.radians(10.0)
        w = 2.0 * math.pi / self.T
        for i in range(ARM_JOINTS):
            q_des = self.q0[i] + amp * math.sin(w * t)
            dq_des = amp * w * math.cos(w * t)
            if not self.torque_demo:
                cmd.q[i] = q_des  # position: the joint boards' PD (the arm's kp / kd) follows it
                continue
            # torque: the boards' PD off (kp = kd = 0), so the torque below is all the joint gets - gravity plus a
            # spring and damper of your own toward the same sine. Your torque controller goes here.
            cmd.kp[i] = 0.0
            cmd.kd[i] = 0.0
            cmd.q[i] = s.q[i]  # no position gain - only keeps the reference next to the arm for the hand-back
            cmd.dq[i] = 0.0
            cmd.tau[i] = s.gravity[i] + self.SPRING[i] * (q_des - s.q[i]) + self.DAMPER[i] * (dq_des - s.dq[i])
        # <<< YOUR ALGORITHM
        return True


# ===== 2. main ====================================================================================================

def main():
    usage = ("Usage:\n    python3 my_arm.py [target] [joy | <gamepad device> | torque]\n\n" + TARGET_USAGE +
             "  joy        read the first gamepad found on this PC (or $JOY); or name one, e.g. /dev/input/js1\n"
             "  torque     the sine by torque: kp = kd = 0 on the joint boards, gravity + a spring and damper of its own\n"
             "\n"
             "It starts the arm and moves it to Ready first, so the start pose is Ready.\n"
             "Without a gamepad every joint moves +-10 deg on a sine for 8 s. With one the sticks jog the joints\n"
             "(left: J0 / J1, right: J4 / J2, up to 20 deg/s), A / B close / open the gripper and BACK ends.\n"
             "Joint-level control has no workspace or collision checks - jog slowly and keep clear of the body.\n")
    if not target_given(sys.argv, usage):
        sys.exit(0 if help_asked(sys.argv) else 1)
    if len(sys.argv) > 3:
        sys.exit(arg_error(sys.argv, "too many arguments", usage))
    torque = len(sys.argv) > 2 and sys.argv[2] == "torque"
    joy_arg = sys.argv[2] if len(sys.argv) > 2 and not torque else ""
    if joy_arg and joy_arg != "joy" and not joy_arg.startswith("/dev/"):
        sys.exit(arg_error(sys.argv, f"unexpected argument '{joy_arg}' - joy, a gamepad device or torque", usage))
    if dry_run(sys.argv[1], f"MyArm joystick={joy_arg or '-'} torque={1 if torque else 0}"):
        sys.exit(0)

    gamepad = Gamepad()  # opened before the arm moves, so a missing pad stops here
    if joy_arg and not gamepad.open("" if joy_arg == "joy" else joy_arg):
        sys.exit(1)
    if not init_channel(sys.argv[1]):
        sys.exit(1)

    with JointStream() as arm:  # app slot 22; leaving the block hands the joints back
        if not arm.start(to_ready=True):
            sys.exit(1)
        for _ in range(50):
            if not torque or arm.state().gravity_valid:
                break
            time.sleep(0.02)
        if torque and not arm.state().gravity_valid:
            print("[my_arm] nothing on rt/rbq/arm_gravity - the robot software is older than this example")
            sys.exit(1)
        s0 = arm.state()  # the arm's own gains - each tick's default, and what the hand-back holds with
        ctl = Controller(torque)
        ctl.start(s0, arm)
        cmd = Command(s0.q_ref)

        next_tick = time.monotonic()
        tick = 0
        try:
            while True:
                s = arm.state()
                # default: hold the last reference with the arm's gains, no feed-forward
                cmd.dq = [0.0] * ARM_JOINTS
                cmd.tau = [0.0] * ARM_JOINTS
                cmd.kp = list(s0.kp)
                cmd.kd = list(s0.kd)
                if not ctl.update(tick / RATE_HZ, s, gamepad.read(), cmd, arm):
                    break
                arm.set_gains(cmd.kp, cmd.kd)
                arm.send(cmd.q, cmd.dq, cmd.tau)
                if tick % (RATE_HZ // 2) == 0:  # the end effector twice a second (rt/rbq/arm_ee: FK of the measured angles)
                    print(f"[my_arm] t {tick / RATE_HZ:5.2f} s  EE {s.ee_pos[0]:6.3f} {s.ee_pos[1]:6.3f} {s.ee_pos[2]:6.3f} m  "
                          f"rpy {math.degrees(s.ee_rpy[0]):6.1f} {math.degrees(s.ee_rpy[1]):6.1f} {math.degrees(s.ee_rpy[2]):6.1f} deg")
                tick += 1
                next_tick += 1.0 / RATE_HZ
                time.sleep(max(0.0, next_tick - time.monotonic()))
        except KeyboardInterrupt:
            pass
        arm.set_gains(s0.kp, s0.kd)  # hand back in position control with the arm's gains - not with a torque demo's kp 0


if __name__ == "__main__":
    main()
