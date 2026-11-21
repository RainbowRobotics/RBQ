"""Test tool, not one of the numbered examples: an interactive CLI for every ArmClient call (like rbq_sport_client.py).

Typical session:  auto_start -> go_ready -> control_start -> ee_up / ee_forward / move_ee ... -> go_folding

Usage:
    python3 arm_cli.py [target]
"""

import math
import sys

from rbq_sdk_py.arm.arm_api import (ARM_AXIS_X, ARM_AXIS_Y, ARM_AXIS_Z, ARM_POSE_FOLDING,
                                    ARM_POSE_PACKING, ARM_POSE_READY)
from rbq_sdk_py.arm.arm_client import ArmClient

from arm_helper import TARGET_USAGE, help_asked, init_channel, target_given

COMMANDS = [
    ("get_state",      "print the arm state"),
    ("auto_start",     "power-up sequence (axis init + reference streaming)"),
    ("go_ready",       "joint move to Ready - the start pose for task-space control"),
    ("go_folding",     "joint move to Folding (walking pose)"),
    ("go_packing",     "joint move to Packing"),
    ("control_start",  "task-space control on (from Ready)"),
    ("ee_forward",     "end effector +2 cm x"),
    ("ee_back",        "end effector -2 cm x"),
    ("ee_left",        "end effector +2 cm y"),
    ("ee_right",       "end effector -2 cm y"),
    ("ee_up",          "end effector +2 cm z"),
    ("ee_down",        "end effector -2 cm z"),
    ("ee_pitch_plus",  "end effector +10 deg pitch"),
    ("ee_pitch_minus", "end effector -10 deg pitch"),
    ("ee_yaw_plus",    "end effector +10 deg yaw"),
    ("ee_yaw_minus",   "end effector -10 deg yaw"),
    ("move_ee",        "end effector to x y z [m] in t [s] - asks for the numbers"),
    ("gripper_open",   "open the gripper (until gripper_stop)"),
    ("gripper_close",  "close the gripper (until gripper_stop)"),
    ("gripper_stop",   "stop the gripper"),
    ("release_joints", "hand every SDK-held arm joint back to the arm controller"),
    ("stop",           "stop the running MoveJ / MoveL where the arm is"),
    ("wait",           "wait until the last motion has finished"),
    ("estop",          "arm emergency stop - joints damped (kp 0) until clear_estop"),
    ("clear_estop",    "end the emergency stop - the arm holds where it is"),
]


def report(name, code):
    note = {0: "ok", 3204: "bad parameter",
            3205: "rejected: arm joints held by a joint-level app, or E-STOP",
            7101: "busy: the arm is still moving",
            7102: "not ready: task-space control is off - control_start from Ready",
            7103: "canceled by stop",
            7104: "still moving when the wait ran out",
            7108: "arm emergency stop - clear_estop first",
            7109: "too fast for the speed limits",
            3104: "no answer - Network running? right target?"}.get(code, "")
    print(f"{name} -> {code}" + (f" ({note})" if note else ""))


def run(arm, name):
    if name == "get_state":
        code, state = arm.GetState()
        for k, v in sorted((state or {}).items()):
            print(f"  {k}: {v}")
        return code
    if name == "move_ee":
        try:
            x, y, z, t = (float(v) for v in input("x y z [m] and seconds: ").split())
        except ValueError:
            print("need four numbers, e.g. 0.45 0 0.25 3")
            return None
        return arm.MoveL(x, y, z, t)
    calls = {
        "auto_start":     arm.AutoStart,
        "go_ready":       lambda: arm.MoveJ(ARM_POSE_READY),
        "go_folding":     lambda: arm.MoveJ(ARM_POSE_FOLDING),
        "go_packing":     lambda: arm.MoveJ(ARM_POSE_PACKING),
        "control_start":  arm.ControlStart,
        "ee_forward":     lambda: arm.MoveLDelta(ARM_AXIS_X, 0.02),
        "ee_back":        lambda: arm.MoveLDelta(ARM_AXIS_X, -0.02),
        "ee_left":        lambda: arm.MoveLDelta(ARM_AXIS_Y, 0.02),
        "ee_right":       lambda: arm.MoveLDelta(ARM_AXIS_Y, -0.02),
        "ee_up":          lambda: arm.MoveLDelta(ARM_AXIS_Z, 0.02),
        "ee_down":        lambda: arm.MoveLDelta(ARM_AXIS_Z, -0.02),
        "ee_pitch_plus":  lambda: arm.RotateLDelta(ARM_AXIS_Y, math.radians(10.0)),
        "ee_pitch_minus": lambda: arm.RotateLDelta(ARM_AXIS_Y, math.radians(-10.0)),
        "ee_yaw_plus":    lambda: arm.RotateLDelta(ARM_AXIS_Z, math.radians(10.0)),
        "ee_yaw_minus":   lambda: arm.RotateLDelta(ARM_AXIS_Z, math.radians(-10.0)),
        "gripper_open":   arm.GripperOpen,
        "gripper_close":  arm.GripperClose,
        "gripper_stop":   arm.GripperStop,
        "release_joints": arm.ReleaseJoints,
        "stop":           arm.Stop,
        "wait":           arm.Wait,
        "estop":          arm.EmergencyStop,
        "clear_estop":    arm.ClearEmergencyStop,
    }
    return calls[name]()


def main():
    if not target_given(sys.argv, __doc__ + TARGET_USAGE):
        sys.exit(0 if help_asked(sys.argv) else 1)
    if len(sys.argv) < 2:
        print(__doc__ + TARGET_USAGE)
        sys.exit(1)
    if not init_channel(sys.argv[1]):
        sys.exit(1)

    arm = ArmClient()
    arm.SetTimeout(2.0)
    arm.Init()

    print('Input "list" for the commands, a name or id to run one, "q" to quit.')
    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if line in ("q", "quit"):
            break
        if line == "list":
            for i, (name, what) in enumerate(COMMANDS):
                print(f"  {i}  {name}  - {what}")
            continue
        names = [n for n, _ in COMMANDS]
        name = names[int(line)] if line.isdigit() and int(line) < len(names) else line
        if name not in names:
            if line:
                print('unknown command - type "list"')
            continue
        code = run(arm, name)
        if code is not None:
            report(name, code)


if __name__ == "__main__":
    main()
