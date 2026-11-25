"""03 - open or close the RBA-1 gripper for a while, then stop it.

Python port of rbq_sdk/cpp/example/src/arm/03_arm_gripper.cpp.

Usage:
    python3 03_arm_gripper.py [target] <action> [seconds]
        action     0 or open, 1 or close
        seconds    the gripper moves until it is stopped; this example stops it after
                   `seconds` (0 < seconds <= 5, default 1)
"""

import sys
import time

from arm_helper import (TARGET_USAGE, arg_error, check_quiet, dry_run, help_asked, init_channel, initialize_arm, make_client, num,
                        parse_float, target_given)


def main():
    if not target_given(sys.argv, __doc__ + TARGET_USAGE):
        sys.exit(0 if help_asked(sys.argv) else 1)
    # each argument checked on its own, so the message says which one is wrong
    if len(sys.argv) < 3:
        sys.exit(arg_error(sys.argv, "which action? 0 or open, 1 or close", __doc__))
    arg = sys.argv[2].lower()
    action = "open" if arg in ("0", "open") else "close" if arg in ("1", "close") else ""
    if not action:
        sys.exit(arg_error(sys.argv, f"action must be 0 / open or 1 / close, not '{sys.argv[2]}'", __doc__))
    seconds = parse_float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    if seconds is None or not 0.0 < seconds <= 5.0:
        sys.exit(arg_error(sys.argv, f"seconds must be more than 0 and at most 5, not '{sys.argv[3]}'", __doc__))
    if dry_run(sys.argv[1], f"Gripper action={action} seconds={num(seconds)}"):
        sys.exit(0)
    if not init_channel(sys.argv[1]):
        sys.exit(1)
    arm = make_client()

    if not initialize_arm(arm, to_ready=False, in_estop_ok=True):
        sys.exit(1)
    ok = check_quiet("GripperOpen", arm.GripperOpen()) if action == "open" else check_quiet("GripperClose", arm.GripperClose())
    if not ok:
        sys.exit(1)
    time.sleep(seconds)
    if not check_quiet("GripperStop", arm.GripperStop()):
        sys.exit(1)
    print(f"[arm] gripper {'opened' if action == 'open' else 'closed'} for {seconds:g} s, then stopped")


if __name__ == "__main__":
    main()
