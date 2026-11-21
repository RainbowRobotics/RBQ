"""Spin the four wheels of an RBQ wheel robot (RBQ-W) from an external app.

Python port of rbq_sdk/cpp/example/src/rbq/rbq_wheel.cpp. Takes ownership of
the wheel joints only (app slot 21) - the legs stay with the robot, so it also
runs next to rbq_low_level.py (slot 20).

With kp = 0 the board applies torque - kd * vel, so a free wheel settles at
vel = torque / kd [rad/s] (default 2 rad/s). On the ground the robot rolls -
give it room. The same command works on CAN FD and classic CAN robots: send
physical gains, the robot converts them for its board.

Usage:
    python3 rbq_wheel.py <networkInterface> [torque] [kd] [seconds]

Arguments:
    networkInterface  CycloneDDS interface, e.g. lo (on the robot) or eth0 (another host)
    torque            feed-forward wheel torque [Nm], |torque| <= 10   (default 1.0)
    kd                wheel damping [Nm*s/rad], 0 < kd <= 6             (default 0.5)
    seconds           how long to drive, then release the wheels      (default 3)

Example (2 rad/s for 3 s):
    python3 rbq_wheel.py eth0 1.0 0.5 3
"""

import os
import sys
import threading
import time

from rbq_sdk_py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)

sys.path.insert(
    0,
    os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "idl")
    ),
)
from _rbq_idl import JointOwnershipCmd_, MotionRef_  # noqa: E402

# wheel order in whl_joint[] (= RBQ_API joint 12..15)
WHEEL_NAMES = ["HRW", "HLW", "FRW", "FLW"]
TORQUE_LIMIT = 10.0  # [Nm] this example's cap (robot limit is 30 Nm)
DT = 0.002           # 500 Hz


def wheel_ref(torque, kd):
    ref = MotionRef_()
    for w in ref.whl_joint:
        w.pos = 0.0
        w.kp = 0.0
        w.kd = float(kd)
        w.torque = float(torque)
    return ref


def main():
    try:
        interface = sys.argv[1]
        torque  = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
        kd      = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
        seconds = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0
    except (IndexError, ValueError):
        print(__doc__)
        sys.exit(1)
    if abs(torque) > TORQUE_LIMIT or not 0.0 < kd <= 6.0 or seconds <= 0.0:
        print(__doc__)
        sys.exit(1)

    ChannelFactoryInitialize(0, interface)

    pub_ref = ChannelPublisher("rt/rbq/ref/motion/_21", MotionRef_)
    pub_ref.Init()
    pub_owner = ChannelPublisher("rt/rbq/cmd/motion/owner/_21", JointOwnershipCmd_)
    pub_owner.Init()

    # the reference the robot actually applies (all apps merged), at 500 Hz
    applied = {"ref": None}
    lock = threading.Lock()

    def on_applied(msg):
        with lock:
            applied["ref"] = msg

    sub_applied = ChannelSubscriber("rt/rbq/ref/motion/_0", MotionRef_)
    sub_applied.Init(handler=on_applied, queueLen=4)
    time.sleep(0.5)  # DDS discovery

    # publish the first reference before taking ownership so the wheels never see stale values
    pub_ref.Write(wheel_ref(0.0, kd))
    time.sleep(0.02)
    own = JointOwnershipCmd_()
    own.whl_joint = [True] * len(WHEEL_NAMES)
    pub_owner.Write(own)
    print("[rbq_wheel] wheel ownership requested (app 21)")

    ticks = int(seconds / DT)
    next_tick = time.monotonic()
    try:
        for t in range(ticks):
            ramp = min(1.0, t / 250.0)  # 0.5 s torque ramp
            pub_ref.Write(wheel_ref(torque * ramp, kd))
            if t % 250 == 0:
                with lock:
                    a = applied["ref"]
                if a is not None:
                    print("[rbq_wheel] applied: " + " ".join(
                        f"{n}(tq={a.whl_joint[i].torque:.2f} kd={a.whl_joint[i].kd:.3f})"
                        for i, n in enumerate(WHEEL_NAMES)))
            next_tick += DT
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        pass

    # release: zero torque, then damping off - the wheels coast like a freshly started robot
    for _ in range(250):
        pub_ref.Write(wheel_ref(0.0, kd))
        time.sleep(DT)
    pub_ref.Write(wheel_ref(0.0, 0.0))
    print("[rbq_wheel] done - wheels released (kp = kd = torque = 0)")

    with lock:
        if applied["ref"] is None:
            print("[rbq_wheel] nothing on rt/rbq/ref/motion/_0 (Network not running?)")
    sub_applied.Close()


if __name__ == "__main__":
    main()
