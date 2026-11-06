"""Set the bottom status LEDs and print the state the robot reports back.

Publishes one LedBottom_ on rt/rbq/cmd/led_bottom (the same fields as
PUT /api/led/bottom), then waits for the 1 Hz LedBottomState_ on
rt/rbq/led_bottom and prints the last one received.
CAN FD robots only - elsewhere Network rejects the command and publishes no state.

Usage:
    python3 rbq_led_bottom.py <networkInterface> [side] [mode] [r] [g] [b] [on_ms] [off_ms] [count]

Arguments:
    networkInterface  CycloneDDS interface, e.g. lo (on the robot) or eth0 (another host)
    side              right / left / both (the other LED keeps its setting)   (default both)
    mode              off / on / blink                                       (default on)
    r g b             colour 0-255                                           (default 0 0 255)
    on_ms off_ms      blink on / off time 10-2550 ms (10 ms steps)           (default 500 500)
    count             blink repeats, 0 = forever                             (default 0)

Example (right LED blinking red, 0.5 s on / 0.5 s off, forever):
    python3 rbq_led_bottom.py eth0 right blink 255 0 0 500 500 0
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
from _rbq_idl import LedBottom_, LedBottomState_  # noqa: E402

# side / mode constants (mirror rbq_msgs/LedBottom.msg)
SIDES = {"right": 1, "left": 2, "both": 3}
MODES = {"off": 0, "on": 1, "blink": 2}
MODE_NAMES = {v: k for k, v in MODES.items()}


def side_text(name, s):
    flags = (" limited" if s.limited else "") + (" legacy" if s.legacy else "")
    return (
        f"  {name}: mode={MODE_NAMES.get(s.mode, '?')} rgb={','.join(map(str, s.rgb))}"
        f" on_ms={s.on_ms} off_ms={s.off_ms} count={s.count}"
        f" actual={','.join(map(str, s.actual))}{flags} lpf_w={s.lpf_w:.1f}"
    )


def main():
    side_name = sys.argv[2] if len(sys.argv) > 2 else "both"
    mode_name = sys.argv[3] if len(sys.argv) > 3 else "on"
    if len(sys.argv) < 2 or side_name not in SIDES or mode_name not in MODES:
        print(__doc__)
        sys.exit(1)

    interface = sys.argv[1]
    r      = int(sys.argv[4]) if len(sys.argv) > 4 else 0
    g      = int(sys.argv[5]) if len(sys.argv) > 5 else 0
    b      = int(sys.argv[6]) if len(sys.argv) > 6 else 255
    on_ms  = int(sys.argv[7]) if len(sys.argv) > 7 else 500
    off_ms = int(sys.argv[8]) if len(sys.argv) > 8 else 500
    count  = int(sys.argv[9]) if len(sys.argv) > 9 else 0

    ChannelFactoryInitialize(0, interface)

    # the robot reports the LED state once a second - keep the latest
    latest = {"state": None}
    lock = threading.Lock()

    def on_state(msg):
        with lock:
            latest["state"] = msg

    pub = ChannelPublisher("rt/rbq/cmd/led_bottom", LedBottom_)
    pub.Init()
    sub = ChannelSubscriber("rt/rbq/led_bottom", LedBottomState_)
    sub.Init(handler=on_state, queueLen=4)
    time.sleep(0.5)  # DDS discovery

    pub.Write(LedBottom_(
        side=SIDES[side_name],
        mode=MODES[mode_name],
        rgb=[r, g, b],
        on_ms=on_ms,
        off_ms=off_ms,
        count=count,
    ))
    print(f"[rbq_led_bottom] published: side={side_name} mode={mode_name} rgb={r},{g},{b}"
          f" on_ms={on_ms} off_ms={off_ms} count={count}")

    # the IF board applies the setting and reports once a second; Network relays it at 1 Hz
    time.sleep(2.5)
    with lock:
        s = latest["state"]
    sub.Close()
    if s is None:
        print("[rbq_led_bottom] no state on rt/rbq/led_bottom (Network not running, or not a CAN FD robot)")
        return
    print(f"[rbq_led_bottom] state: age_ms={s.age_ms} configured={int(s.configured)} if_fw={s.if_fw}")
    print(side_text("right", s.right))
    print(side_text("left", s.left))


if __name__ == "__main__":
    main()
