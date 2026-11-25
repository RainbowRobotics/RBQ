"""02 - start the RBA-1 arm, print its status, take its joints back from a joint-level app, or stop it in an emergency.

Python port of rbq_sdk/cpp/example/src/arm/02_arm_power.cpp.

Usage:
    python3 02_arm_power.py [target] [on|status|release|estop|clear]
        on       start the arm if it is not running (AutoStart) and wait until it is   (default)
        status   print the arm status (GetState)
        release  give every arm joint a joint-level app holds back to the arm controller
        estop    emergency stop: every arm joint is damped (kp 0) - the arm sinks slowly under gravity
                 and gives way when pushed. Stays on until clear. The robot's E-stop does the same.
        clear    end the emergency stop - the arm holds the pose it sank to
"""

import sys

from arm_helper import (TARGET_USAGE, arg_error, check_quiet, dry_run, first_state, help_asked, init_channel, initialize_arm,
                        make_client, target_given)


def main():
    if not target_given(sys.argv, __doc__ + TARGET_USAGE):
        sys.exit(0 if help_asked(sys.argv) else 1)
    action = sys.argv[2] if len(sys.argv) > 2 else "on"
    if action not in ("on", "status", "release", "estop", "clear"):
        sys.exit(arg_error(sys.argv, f"action must be on, status, release, estop or clear, not '{action}'", __doc__))
    if dry_run(sys.argv[1], f"Power action={action}"):
        sys.exit(0)
    if not init_channel(sys.argv[1]):
        sys.exit(1)
    arm = make_client()

    # done: one line (status: the whole state - that is what it is for); failed: the code and what it means
    if action == "estop":  # first - nothing to wait for
        if not check_quiet("EmergencyStop", arm.EmergencyStop()):
            sys.exit(1)
        print("[arm] emergency stop on - the arm is damped (kp 0) until: 02_arm_power.py [target] clear")
        sys.exit(0)
    ok, state = first_state(arm)
    if not ok:
        sys.exit(1)
    if action == "on":  # initialize_arm says "arm started" when it starts it
        if state.get("con_start") == "1" and state.get("estop") != "1" and state.get("sdk_slot") == "-1":
            print("[arm] the arm is already on")
            sys.exit(0)
        sys.exit(0 if initialize_arm(arm, to_ready=False) else 1)
    if action == "status":
        for k, v in sorted(state.items()):
            print(f"  {k}: {v}")
        sys.exit(0)
    if action == "release":
        if not check_quiet("ReleaseJoints", arm.ReleaseJoints()):
            sys.exit(1)
        print("[arm] arm joints handed back to the arm controller")
        sys.exit(0)
    if not check_quiet("ClearEmergencyStop", arm.ClearEmergencyStop()):
        sys.exit(1)
    print("[arm] emergency stop cleared - the arm holds where it is; MoveJ Ready (04_arm_move_j.py 1) before task-space moves")


if __name__ == "__main__":
    main()
