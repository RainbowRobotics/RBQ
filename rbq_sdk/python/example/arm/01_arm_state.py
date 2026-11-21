"""01 - RBA-1 arm state. Read only: prints the joints (rt/rbq/arm_joint) and the end effector
(rt/rbq/arm_ee) twice a second until Ctrl-C. Joints 0..5 are J0..J5, 6 is the gripper (GRP).

Python port of rbq_sdk/cpp/example/src/arm/01_arm_state.cpp.

Usage:
    python3 01_arm_state.py [target]
"""

import math
import os
import sys
import threading
import time

from rbq_sdk_py.core.channel import ChannelSubscriber

from arm_helper import TARGET_USAGE, arg_error, dry_run, help_asked, init_channel, target_given

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "idl")))
from _rbq_idl import ArmJointInfo_, TaskInfo_  # noqa: E402

ARM_SLOTS = 7  # J0..J5 + GRP


def main():
    if not target_given(sys.argv, __doc__ + TARGET_USAGE):
        sys.exit(0 if help_asked(sys.argv) else 1)
    if len(sys.argv) > 2:
        sys.exit(arg_error(sys.argv, f"unexpected argument '{sys.argv[2]}'", __doc__ + TARGET_USAGE))
    if dry_run(sys.argv[1], "State"):
        sys.exit(0)
    if not init_channel(sys.argv[1]):
        sys.exit(1)

    latest = {"joint": None, "ee": None}
    lock = threading.Lock()

    def on_joint(msg):
        with lock:
            latest["joint"] = msg

    def on_ee(msg):
        with lock:
            latest["ee"] = msg

    sub_joint = ChannelSubscriber("rt/rbq/arm_joint", ArmJointInfo_)
    sub_joint.Init(handler=on_joint, queueLen=4)
    sub_ee = ChannelSubscriber("rt/rbq/arm_ee", TaskInfo_)
    sub_ee.Init(handler=on_ee, queueLen=4)

    start = time.monotonic()
    try:
        while True:
            time.sleep(0.5)
            with lock:
                msg, ee = latest["joint"], latest["ee"]
            if msg is None:
                if time.monotonic() - start > 3.0:
                    print("[01_arm_state] no data on rt/rbq/arm_joint yet (Network not running, or a different interface)")
                continue
            # each reference next to its measured value: ref_pos pos, ref_vel vel, ref_torque torque
            print("\n name stat err  owner " + " ".join(f"{h:>14}" for h in (
                "ref_pos[deg]", "pos[deg]", "ref_vel[deg/s]", "vel[deg/s]", "ref_torque[Nm]", "torque[Nm]"))
                  + f" {'kp[Nm/rad]':>11} {'kd[Nms/rad]':>12} {'cur[A]':>7}")
            for j in msg.joint[:ARM_SLOTS]:
                print(f" {j.name:<4} {j.status:<4} {j.error_msg:<4} {j.owner:5d} "
                      f"{math.degrees(j.ref_position):14.2f} {math.degrees(j.pos):14.2f} "
                      f"{math.degrees(j.ref_vel):14.2f} {math.degrees(j.vel):14.2f} "
                      f"{j.ref_ff_torque:14.3f} {j.torque:14.3f} "
                      f"{j.kp:11.2f} {j.kd:12.3f} {j.current:7.2f}")
            if ee is not None:
                print(f" EE (body frame)  pos [m] {ee.pos[0]:7.3f} {ee.pos[1]:7.3f} {ee.pos[2]:7.3f}   "
                      f"rpy [deg] {math.degrees(ee.rpy[0]):7.1f} {math.degrees(ee.rpy[1]):7.1f} {math.degrees(ee.rpy[2]):7.1f}")
    except KeyboardInterrupt:
        pass
    sub_joint.Close()
    sub_ee.Close()


if __name__ == "__main__":
    main()
