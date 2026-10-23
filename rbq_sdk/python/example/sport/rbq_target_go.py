"""Send a targetGo command to the robot.

The robot navigates from its current position to the given (x, y, theta) goal
using the selected gait.  The robot must already be standing before calling
this example.

Usage:
    python3 rbq_target_go.py <networkInterface> [x] [y] [theta_deg] [gait] [mode]

Arguments:
    networkInterface  CycloneDDS interface, e.g. lo (sim) or eth0 (real robot)
    x                 Target X in metres, relative to robot start  (default 1.0)
    y                 Target Y in metres, relative to robot start  (default 0.0)
    theta             Target heading in radians                    (default 0.0)
    gait              0=wave(default) / 1=stairs / 2=rl
    mode              0=rot-straight-rot(default) / 1=rot-then-diag /
                      2=diag-then-rot / 3=rot+diag simultaneous

Example (sim, 1 m forward with wave gait):
    python3 rbq_target_go.py lo 1.0 0.0 0.0 0 0
"""

import os
import sys
import time

from rbq_sdk_py.core.channel import ChannelFactoryInitialize, ChannelPublisher

sys.path.insert(
    0,
    os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "idl")
    ),
)
from _rbq_idl import TargetGo_  # noqa: E402

# ---------------------------------------------------------------------------
# gait constants (mirror QuadWalk_TARGET_GO handler)
# ---------------------------------------------------------------------------
GAIT_WAVE   = 0  # static wave walk
GAIT_STAIRS = 1  # stair walking
GAIT_RL     = 2  # RL-policy walk

# approach mode constants
# 0: rotate→straight→rotate
# 1: rotate to theta → diagonal walk
# 2: diagonal walk → rotate to theta
# 3: simultaneous rotate + diagonal walk
MODE_DEFAULT = 0


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    interface   = sys.argv[1]
    x           = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    y           = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    theta       = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    gait        = int(sys.argv[5])   if len(sys.argv) > 5 else GAIT_WAVE
    mode        = int(sys.argv[6])   if len(sys.argv) > 6 else MODE_DEFAULT

    ChannelFactoryInitialize(0, interface)

    pub = ChannelPublisher("rt/rbq/cmd/target_go", TargetGo_)
    pub.Init()
    time.sleep(0.5)  # DDS discovery

    msg = TargetGo_(
        gait=gait,
        mode=mode,
        x=x,
        y=y,
        theta=theta,
        offset_x=0.0,
        offset_y=0.0,
        slow=False,
        wide=False,
        vision=False,
    )

    pub.Write(msg)
    print(f"[rbq_target_go] published: gait={gait} mode={mode} x={x} y={y} theta={theta}")


if __name__ == "__main__":
    main()
