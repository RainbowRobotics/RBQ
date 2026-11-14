"""Interop check: subscribe rt/rbq/_sim and print the decoded SimInfo_.

Checks the wire contract: run it while rbq_sim (or MuJoCo / Gazebo)
publishes SimInfo_. If it decodes here with the same _sim_idl types, the type-hash / QoS /
Imu_ FINAL extensibility all match and Motion will receive the feedback too. Needs only
rbq_sdk_py + cyclonedds (no Isaac Sim), so it can run in a plain rbq-lab shell.

    python -m rbq_sim.tools.sim_echo -i lo
"""

import argparse
import time

from rbq_sdk_py.core.channel import ChannelFactoryInitialize, ChannelSubscriber

from rbq_sim.idl._sim_idl import SimInfo_

SIM_TOPIC = "rt/rbq/_sim"


def main():
    parser = argparse.ArgumentParser(description="Echo decoded SimInfo_ from rt/rbq/_sim.")
    parser.add_argument("--interface", "-i", default="lo", help="CycloneDDS interface (default lo).")
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.interface)
    sub = ChannelSubscriber(SIM_TOPIC, SimInfo_)
    sub.Init()
    print(f"[sim_echo] subscribed {SIM_TOPIC} (SimInfo_) on '{args.interface}' — waiting for samples...")

    count = 0
    while True:
        info = sub.Read(timeout=1.0)
        if info is None:
            print("[sim_echo] (no sample in last 1s)")
            continue
        count += 1
        o = info.imu.orientation
        pos = [round(info.leg_joint[i].pos, 3) for i in range(12)]
        print(
            f"[sim_echo] #{count} quat(w,x,y,z)=({o.w:.3f},{o.x:.3f},{o.y:.3f},{o.z:.3f}) "
            f"accel.z={info.imu.linear_acceleration.z:.2f} leg_pos={pos}"
        )
        time.sleep(0.1)


if __name__ == "__main__":
    main()
