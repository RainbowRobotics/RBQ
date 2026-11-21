"""Shared by the numbered RBA-1 arm examples (01_ ... 06_), like RB-Y1's 00_helper:
every example starts the arm itself when it is not running, so each one runs on its own.

The arm service answers when it accepts a command, not when the motion ends, so the waits below
poll ArmClient.GetState. They go away once the service reports completion.
"""

import array
import errno
import fcntl
import glob
import ipaddress
import math
import os
import socket
import struct
import sys
import threading
import time

from rbq_sdk_py.arm.arm_api import ARM_POSE_FOLDING, ARM_POSE_PACKING, ARM_POSE_PICKUP_READY, ARM_POSE_READY, ARM_POSE_STRAIGHT
from rbq_sdk_py.arm.arm_client import ArmClient
from rbq_sdk_py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "idl")))
from _rbq_idl import ArmJointInfo_, JointOwnershipCmd_, MotionRef_, TaskInfo_  # noqa: E402

ROBOT_ADDRESS = "192.168.0.10"  # every RBQ ships at this address

# Printed by every example's usage.
TARGET_USAGE = (
    "  target     sim (-s)    the simulator on this PC - the default, so it can be left out\n"
    "             robot (-r)  the robot at 192.168.0.10, through whichever network device reaches it\n"
    "             or a network device name such as enp3s0, as in the rbq examples\n"
)

CODE_TEXT = {
    0: "ok",
    3204: "bad parameter",
    3205: "rejected: arm joints held by a joint-level app, or E-STOP",
    3104: "no answer - Network running? right target?",
    7101: "busy: the arm is still moving",
    7102: "not ready: task-space control is off",
    7103: "canceled by Stop",
    7104: "still moving when the wait ran out",
    7105: "stopped: collision torque limit (GetState collision_joint)",
    7106: "not started: the path would hit the robot body or legs",
    7107: "stopped: gripper workspace limit",
    7108: "arm emergency stop - clear it: 02_arm_power.py [target] clear",
    7109: "too fast: the time given is shorter than the speed limits allow",
}

# MoveJ poses as the examples accept them: a number, a short name, or the pose name itself (any case).
# Joint angles J0..J5 [deg] of RBA-1 (the arm controller's pose table, read back in the simulator 11-18).
POSES = [
    ("0", "home",     ARM_POSE_FOLDING,      "0 0 0 0 0 0           home / walking pose"),
    ("1", "ready",    ARM_POSE_READY,        "0 140 -70 0 -70 0     start pose for task-space moves"),
    ("2", "straight", ARM_POSE_STRAIGHT,     "0 90 -180 0 0 0       arm stretched straight up"),
    ("3", "pickup",   ARM_POSE_PICKUP_READY, "0 150 -100 0 40 0     to pick up an object from the ground"),
]


def pose_from_arg(arg: str) -> str:
    """Pose name for "0" / "home" / "Folding" / "folding" ...; empty when it is none of them."""
    a = arg.lower()
    for number, short, pose, _ in POSES:
        if a in (number, short, pose.lower()):
            return pose
    return ""


def pose_usage() -> str:
    return "".join(f"    {number}  {short:<9} {what}\n" for number, short, _, what in POSES)


_SIOCGIFADDR, _SIOCGIFNETMASK = 0x8915, 0x891B


def _ifreq_ipv4(sock, name: str, request: int):
    try:
        data = fcntl.ioctl(sock.fileno(), request, struct.pack("256s", name.encode()[:15]))
        return socket.inet_ntoa(data[20:24])
    except OSError:
        return None


def interface_for(address: str) -> str:
    """Network device that reaches `address`: asks the OS which local address it would send from (no packet goes
    out), then finds the device holding that address on the same subnet. "lo" when `address` is this PC."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect((address, 9))
            local = s.getsockname()[0]
        except OSError:
            return ""
        if local == address:
            return "lo"
        for _, name in socket.if_nameindex():
            addr, mask = _ifreq_ipv4(s, name, _SIOCGIFADDR), _ifreq_ipv4(s, name, _SIOCGIFNETMASK)
            # same subnet as the robot - otherwise the route is a default gateway, not the robot's network
            if addr == local and mask and ipaddress.ip_address(address) in ipaddress.ip_network(f"{addr}/{mask}", strict=False):
                return name
    return ""


def is_target(t: str) -> bool:
    return t in ("sim", "-s", "robot", "-r") or (bool(t) and not t.startswith("-") and t in (n for _, n in socket.if_nameindex()))


def help_asked(argv) -> bool:
    return len(argv) > 1 and argv[1] in ("-h", "--help")


_target_omitted = False


def print_target_hint(argv):
    """After a rejected first argument with no target given: says it was read as an argument, not as a target. A word
    that is neither (a typo, a device this PC does not have) would otherwise look like a wrong argument only."""
    if _target_omitted:
        print(f"[arm] no target given, so '{argv[2]}' was read as an argument - a target is sim, robot or a network "
              "device of this PC (ip -br link)")


def target_given(argv, usage: str = "") -> bool:
    """First check of every example. `usage` is the example's full description: with -h / --help it is printed and
    the example should exit (0). Without a target in argv[1] it puts "sim" there (in place), so the rest of the example
    reads its arguments from argv[2] on either way - only robot / -r (or a device name) reaches the robot."""
    if help_asked(argv):
        print(usage)
        return False
    global _target_omitted
    if not (len(argv) > 1 and is_target(argv[1])):
        argv.insert(1, "sim")
        _target_omitted = True
    return True


def arg_error(argv, what: str, usage: str) -> int:
    """A wrong argument: says which one and why, then only the usage lines. Returns the exit code (1)."""
    print(f"[arm] {what}")
    # the target hint only when the first argument is the one rejected (the message quotes it) - not for 'rel 0.1'
    if len(argv) > 2 and f"'{argv[2]}'" in what:
        print_target_hint(argv)
    lines = [l.strip() for l in usage.splitlines()]
    first = next((i for i, l in enumerate(lines) if l.startswith("python3")), None)
    if first is not None:
        last = first
        while last + 1 < len(lines) and lines[last + 1].startswith("python3"):
            last += 1
        print("\nUsage: " + "\n       ".join(lines[first:last + 1]) + f"\n  (python3 {argv[0]} -h for the full description)")
    return 1


def parse_float(s: str):
    """The number in `s`, or None when it is not one."""
    try:
        return float(s)
    except ValueError:
        return None


def parse_option(arg: str, key: str):
    """The number of a `key=<number>` option such as vel=30, or None when `arg` is not one for `key`."""
    return parse_float(arg[len(key) + 1:]) if arg.startswith(key + "=") else None


# The joint range [deg] MoveJ by angles accepts - provisional until the actuator specifications are final.
JOINT_MIN_DEG = (-360.0, 0.0, -180.0, -360.0, -360.0, -360.0)
JOINT_MAX_DEG = (360.0, 180.0, 2.0, 360.0, 360.0, 360.0)


def num(v: float) -> str:
    """A number as f"{v:g}" - the dry-run line prints numbers the way the C++ examples' printf %g does."""
    return f"{v:g}"


def dry_run(target: str, call: str) -> bool:
    """With RBQ_ARM_DRY_RUN=1 in the environment (rbq_sdk/scripts/arm_examples_check.py): prints the call the example
    would make - "[dry-run] <target> <call>" - and returns True, so the example exits there without connecting. Each
    example calls it after reading its arguments; the C++ examples print the same line."""
    if os.environ.get("RBQ_ARM_DRY_RUN") != "1":
        return False
    t = "sim" if target in ("sim", "-s") else "robot" if target in ("robot", "-r") else target
    print(f"[dry-run] {t} {call}")
    return True


def init_channel(target: str) -> bool:
    """Points the SDK at `target` (sim / -s, robot / -r, or a device name). Prints where it connects; False if it cannot."""
    if target in ("sim", "-s"):
        iface = "lo"
        print("[arm] target: simulator (lo)")
    elif target in ("robot", "-r"):
        iface = interface_for(ROBOT_ADDRESS)
        if not iface:
            print(f"[arm] no network device on the robot's network ({ROBOT_ADDRESS}) - connect to the robot's Wi-Fi or cable")
            return False
        print(f"[arm] target: robot {ROBOT_ADDRESS} via {iface}")
    else:
        if not target or target.startswith("-") or target not in (n for _, n in socket.if_nameindex()):
            print(f"[arm] unknown target '{target}'\n{TARGET_USAGE}", end="")
            return False
        iface = target
        print(f"[arm] target: network device {iface}")
    ChannelFactoryInitialize(0, iface)
    return True


def print_too_fast(arm: ArmClient, code: int, seconds: float, limits: str):
    """After a move refused with ARM_ERR_TOO_FAST (7109): the time given was within the range but would move the arm faster
    than `limits` allow - says so, and the shortest time that move needs (GetState motion_time)."""
    if code != 7109:
        return
    rc, state = arm.GetState()
    if rc == 0:
        print(f"[arm] {seconds:g} s is too fast for this move - it would go past the speed limits (max {limits}).\n"
              f"      This move needs at least {float((state or {}).get('motion_time', 0)):.2f} s: give that or longer, "
              "or leave the time out and the robot picks it")


def check(what: str, code: int) -> bool:
    """Prints "<what> -> <code> (<meaning>)" and returns True when the code is 0."""
    print(f"[arm] {what} -> {code} ({CODE_TEXT.get(code, 'error')})")
    return code == 0


def check_quiet(what: str, code: int) -> bool:
    """check for a step the user did not ask for (the state read before a move): prints only when it fails."""
    return code == 0 or check(what, code)


def move(arm: ArmClient, what: str, code: int, timed: bool = False, seconds: float = 0.0, limits: str = None,
         wait: float = 65.0) -> bool:
    """A motion call and its end. Done: one line, "[arm] <what> done" (+ "in 0.91 s" when timed - the robot reports this
    move's time: MoveJ by angles, MoveL family). Refused or stopped: the code, what it means and - when the time given
    (seconds) was too short for `limits` - the time the move needs."""
    if code != 0:
        check(what, code)
        if limits:
            print_too_fast(arm, code, seconds, limits)
        return False
    done = arm.Wait(wait)
    if done != 0:
        return check(f"{what} did not finish", done)
    rc, state = arm.GetState() if timed else (1, None)
    if rc == 0:
        print(f"[arm] {what} done in {float((state or {}).get('motion_time', 0)):.2f} s")
    else:
        print(f"[arm] {what} done")
    return True


def make_client() -> ArmClient:
    arm = ArmClient()
    arm.SetTimeout(2.0)
    arm.Init()
    return arm


def wait_for(arm: ArmClient, key: str, value: str, seconds: float) -> bool:
    """Polls GetState until state[key] == value or the timeout passes."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        code, state = arm.GetState()
        if code == 0 and state and state.get(key) == value:
            return True
        time.sleep(0.2)
    print(f"[arm] gave up waiting for {key} = {value} after {seconds:.0f} s")
    return False


def first_state(arm: ArmClient, seconds: float = 3.0):
    """First GetState of a fresh client. Right after start-up DDS has not yet matched the client with the robot's
    service, so the first answers can be lost (3104); asks again for up to `seconds` before giving up."""
    end = time.monotonic() + seconds
    code, state = arm.GetState()
    while code == 3104 and time.monotonic() < end:
        time.sleep(0.2)
        code, state = arm.GetState()
    return check_quiet("GetState", code) and state is not None, state or {}


def start_task_control(arm: ArmClient) -> bool:
    """Task-space moves (MoveL family) need task-space control on, and the arm turns it on only in the Ready pose.
    Goes through Ready and ControlStart only when it is off; when it is already on, the arm moves from where it is."""
    code, state = arm.GetState()
    if not check_quiet("GetState", code) or state is None:
        return False
    if state.get("task_control") == "1":
        return True
    if state.get("pose") != ARM_POSE_READY and not move(arm, "MoveJ Ready", arm.MoveJ(ARM_POSE_READY)):
        return False
    if not check_quiet("ControlStart", arm.ControlStart()) or not wait_for(arm, "task_control", "1", 3.0):
        return False
    print("[arm] task-space control on (ControlStart)")
    return True


def initialize_arm(arm: ArmClient, to_ready: bool = True, in_estop_ok: bool = False, only_if_folded: bool = False) -> bool:
    """Starts the arm if it is not running (AutoStart), then moves it to Ready when to_ready is set. With only_if_folded
    it goes to Ready only from the folded pose (Folding / Packing, also where AutoStart leaves it) and otherwise starts
    where the arm is - for a joint-level test right after a move to the pose you want.
    In an emergency stop it never calls AutoStart (that would end the stop): it returns False with a hint, or True
    without doing anything when in_estop_ok is set (the gripper still works then)."""
    ok, state = first_state(arm)
    if not ok:
        return False
    if state.get("sdk_slot") != "-1":
        print(f"[arm] app slot {state.get('sdk_slot')} holds the arm joints - stop that app or call ReleaseJoints()")
        return False
    if state.get("estop") == "1":
        if in_estop_ok:
            return True
        print("[arm] the arm is in emergency stop - clear it first: 02_arm_power.py [target] clear")
        return False
    if state.get("con_start") != "1":
        if not check_quiet("AutoStart", arm.AutoStart()) or not wait_for(arm, "con_start", "1", 60.0):
            return False
        print("[arm] arm started (AutoStart)")
        _, state = arm.GetState()
        state = state or {}
    folded = state.get("pose") in (ARM_POSE_FOLDING, ARM_POSE_PACKING)
    if to_ready and state.get("pose") != ARM_POSE_READY and (not only_if_folded or folded):
        return move(arm, "MoveJ Ready", arm.MoveJ(ARM_POSE_READY))
    return True


# ── Joint-level (Lv0) stream, used by 07_arm_low_level ──────────────────────────────────────────────────────────

ARM_JOINTS = 6        # J0..J5 - the gripper (GRP) is not a joint-level joint, see JointStream.gripper
POS_RATE_MAX = 2.0    # [rad/s] the robot moves a joint reference no faster than this
# Damping send_torque() applies when no kd is given [Nm*s/rad] - about a quarter of the arm's default kd, enough to
# stop the arm drifting on its own and soft enough to push by hand.
TORQUE_MODE_KD = [5.0, 5.0, 2.5, 2.5, 2.5, 2.5]


class ArmState:
    """One snapshot of the arm: rt/rbq/arm_joint for the six arm joints (lists of 6) and the gripper, rt/rbq/arm_ee
    for the end effector."""

    def __init__(self, msg, ee=None, gravity=None):
        joints = msg.joint[:ARM_JOINTS]
        self.q       = [j.pos for j in joints]           # measured position [rad]
        self.dq      = [j.vel for j in joints]           # measured velocity [rad/s]
        self.torque  = [j.torque for j in joints]        # measured torque [Nm]
        self.q_ref   = [j.ref_position for j in joints]  # reference the arm follows now [rad]
        self.kp      = [j.kp for j in joints]            # gains the arm applies now [Nm/rad]
        self.kd      = [j.kd for j in joints]            #                           [Nm*s/rad]
        self.owner   = [j.owner for j in joints]         # app slot that holds the joint
        self.gripper = msg.joint[ARM_JOINTS].pos         # gripper link angle [rad]
        self.ee_valid = ee is not None                   # False until the first rt/rbq/arm_ee has arrived
        self.ee_pos  = list(ee.pos) if ee else [0.0] * 3  # measured end-effector position [m], body frame - compare your FK
        self.ee_rpy  = list(ee.rpy) if ee else [0.0] * 3  # measured end-effector roll / pitch / yaw [rad], body frame
        # torque that holds the arm against gravity at q [Nm] (rt/rbq/arm_gravity) - the robot's arm model with the body
        # taken as level; add it in send_torque()
        self.gravity_valid = gravity is not None
        self.gravity = [j.torque for j in gravity.joint[:ARM_JOINTS]] if gravity else [0.0] * ARM_JOINTS


class JointStream:
    """Takes the six arm joints as one app slot and streams joint references to them. Does the parts every joint-level
    program needs - start the arm, reference before ownership, keep the arm's gains, give the joints back where they
    are - so a program only computes joint references and calls send(). Use it in a `with` block: the joints go back
    when the block ends, also on Ctrl-C or an exception.

    The robot moves each reference at most POS_RATE_MAX toward what you send, so a jump (say 90 deg in one tick) makes
    the arm travel there at that speed instead of snapping. send() prints a warning when it sees one.
    """

    def __init__(self, slot: int = 22):
        self.slot = slot
        self._rpc = make_client()
        self._pub_ref = ChannelPublisher(f"rt/rbq/ref/motion/_{slot}", MotionRef_)
        self._pub_ref.Init()
        self._pub_owner = ChannelPublisher(f"rt/rbq/cmd/motion/owner/_{slot}", JointOwnershipCmd_)
        self._pub_owner.Init()
        self._lock = threading.Lock()
        self._msg = None
        self._sub = ChannelSubscriber("rt/rbq/arm_joint", ArmJointInfo_)
        self._sub.Init(handler=self._on_state, queueLen=4)
        self._ee = None
        self._sub_ee = ChannelSubscriber("rt/rbq/arm_ee", TaskInfo_)
        self._sub_ee.Init(handler=self._on_ee, queueLen=4)
        self._gravity = None
        self._sub_gravity = ChannelSubscriber("rt/rbq/arm_gravity", ArmJointInfo_)
        self._sub_gravity.Init(handler=self._on_gravity, queueLen=4)
        self._torque_mode = False
        self._started = False
        self._kp = [0.0] * ARM_JOINTS
        self._kd = [0.0] * ARM_JOINTS
        self._last_q = None
        self._last_send = 0.0
        self._jump_warned = [False] * ARM_JOINTS
        self._grip = None

    def _on_state(self, msg):
        with self._lock:
            self._msg = msg

    def _on_ee(self, msg):
        with self._lock:
            self._ee = msg

    def _on_gravity(self, msg):
        with self._lock:
            self._gravity = msg

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.stop()
        self._sub.Close()
        self._sub_ee.Close()
        self._sub_gravity.Close()
        return False

    def start(self, to_ready: bool = True, only_if_folded: bool = False) -> bool:
        """Starts the arm (and goes to Ready when to_ready), waits for the arm state, then takes the joints with the
        reference the arm already follows and the gains it already applies. False, with a message, when it cannot."""
        if not initialize_arm(self._rpc, to_ready, False, only_if_folded):
            return False
        for _ in range(100):  # DDS discovery and the first arm state
            with self._lock:
                if self._msg is not None:
                    break
            time.sleep(0.02)
        s = self.state()
        if s is None:
            print("[arm] nothing on rt/rbq/arm_joint (Network not running, or the wrong target?)")
            return False
        for i, k in enumerate(s.kp):
            if k <= 0.0:
                print(f"[arm] J{i} has no position gain (kp = 0): the arm is not holding a pose")
                return False
        self._kp, self._kd = list(s.kp), list(s.kd)
        # reference first, then ownership - the arm never sees a stale reference
        self.send(s.q_ref)
        time.sleep(0.02)
        self._set_ownership(True)
        self._started = True
        print(f"[arm] arm joints taken as app slot {self.slot}")
        return True

    def state(self):
        """Latest ArmState, or None before the first one arrived."""
        with self._lock:
            return ArmState(self._msg, self._ee, self._gravity) if self._msg is not None else None

    def set_gains(self, kp, kd):
        """Gains for the next send() calls [Nm/rad, Nm*s/rad] - the stiffness and damping of each joint around the
        reference. start() sets the ones the arm already applies (state().kp / kd); scale those rather than starting
        from nothing. The robot clamps kp to 0..~53 000 (J0, J1) / ~24 000 (J2..J5) Nm/rad and kd to 0..~420 / ~190 Nm*s/rad."""
        self._kp, self._kd = [float(k) for k in kp], [float(k) for k in kd]

    def send(self, q, vel=None, torque=None):
        """Joint references [rad], and optionally feed-forward velocity [rad/s] and torque [Nm] (0 when left out)."""
        ref = MotionRef_()
        for i in range(ARM_JOINTS):
            r = ref.arm_joint[i]
            r.pos = float(q[i])
            r.vel = float(vel[i]) if vel is not None else 0.0
            r.torque = float(torque[i]) if torque is not None else 0.0
            r.kp, r.kd = self._kp[i], self._kd[i]
        self._warn_on_jump(q)
        self._pub_ref.Write(ref)
        self._torque_mode = False

    def send_torque(self, torque, kd=None):
        """Torque mode: joint torques [Nm] with no position gain (kp 0) and damping kd [Nm*s/rad] (TORQUE_MODE_KD when
        left out). Nothing holds the arm but what you send - add state().gravity, or the arm falls. The robot clamps each
        torque to +-60 Nm (J0, J1) / +-25 Nm (J2..J5). If the torques stop for 0.2 s (program hung or killed) the robot
        sends the gravity torque with damping, so the arm floats where it is. send() goes back to position control."""
        s = self.state()
        ref = MotionRef_()
        for i in range(ARM_JOINTS):
            r = ref.arm_joint[i]
            r.pos = float(s.q[i])  # no position gain, so this only keeps the reference next to the arm for send()
            r.vel = 0.0
            r.torque = float(torque[i])
            r.kp = 0.0
            r.kd = float(kd[i]) if kd is not None else TORQUE_MODE_KD[i]
        self._last_q = list(s.q)
        self._pub_ref.Write(ref)
        self._torque_mode = True

    def gripper(self, cmd: str):
        """"open" / "close" / "stop" through ArmClient. Returns at once - the call runs on its own thread so the
        control loop is not held up; the result is printed. A call made while the previous one is still running is skipped."""
        calls = {"open": self._rpc.GripperOpen, "close": self._rpc.GripperClose, "stop": self._rpc.GripperStop}
        if self._grip is not None and self._grip.is_alive():
            print("[arm] gripper call skipped - the previous one has not been answered yet")
            return
        call = calls[cmd]
        self._grip = threading.Thread(target=lambda: check_quiet(f"Gripper {cmd}", call()), daemon=True)
        self._grip.start()

    def stop(self):
        """Holds the last reference for 0.2 s, then gives the joints back - the arm stays where the last reference is.
        After send_torque() that is where the arm is, held with the position gains and the gravity torque."""
        if self._grip is not None:
            self._grip.join()
        if not self._started:
            return
        self._started = False
        hold = self.state().gravity if self._torque_mode else None
        for _ in range(100):
            self.send(self._last_q, torque=hold)
            time.sleep(0.002)
        self._set_ownership(False)
        print("[arm] arm joints handed back to the arm controller")

    def _set_ownership(self, on: bool):
        own = JointOwnershipCmd_()
        own.arm_joint = [on] * ARM_JOINTS + [False] * (len(own.arm_joint) - ARM_JOINTS)
        self._pub_owner.Write(own)

    def _warn_on_jump(self, q):
        # A reference that moves faster than the robot follows is almost always a mistake (a wrong unit, an IK branch
        # flip). Says so once per joint until the references are back to a speed the arm follows.
        now = time.monotonic()
        dt = max(now - self._last_send, 0.002)
        for i in range(ARM_JOINTS):
            step = abs(q[i] - self._last_q[i]) if self._last_q is not None else 0.0
            jump = step > POS_RATE_MAX * dt + 0.01
            if jump and not self._jump_warned[i]:
                print(f"[arm] J{i} reference jumped {math.degrees(step):.1f} deg in {dt * 1000:.0f} ms - "
                      f"the robot moves it at {math.degrees(POS_RATE_MAX):.0f} deg/s")
            self._jump_warned[i] = jump
        self._last_q = [float(v) for v in q]
        self._last_send = now


# ── Gamepad on this PC (06_arm_high_level, my_arm) ───────────────────────────────────────────────────────────────
# Linux joystick numbers (xpad / Logitech F710 in X mode) - the same as rbq_sdk/Joystick.hpp in C++
AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_X, AXIS_RIGHT_Y, AXIS_RT = 0, 1, 3, 4, 5
PAD_A, PAD_B, PAD_X, PAD_Y, PAD_LB, PAD_RB, PAD_BACK, PAD_START = 0, 1, 2, 3, 4, 5, 6, 7
JS_EVENT_BUTTON, JS_EVENT_AXIS, JS_EVENT_INIT = 0x01, 0x02, 0x80
JSIOCGNAME_128 = (2 << 30) | (128 << 16) | (ord("j") << 8) | 0x13


def joy_name(path: str) -> str:
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return ""
    buf = array.array("B", [0] * 128)
    try:
        fcntl.ioctl(fd, JSIOCGNAME_128, buf)
    except OSError:
        pass
    os.close(fd)
    return buf.tobytes().split(b"\0", 1)[0].decode("utf-8", errors="replace")


def pick_joy_path(argv):
    """argv[2], then $JOY, then the first js device that looks like a gamepad, then the first one."""
    candidates = [argv[2]] if len(argv) > 2 else [os.environ.get("JOY", "")] + sorted(glob.glob("/dev/input/js*"))
    found = [p for p in candidates if p and os.path.exists(p)]
    if len(argv) > 2:
        return found[0] if found else None
    pads = [p for p in found if any(k in joy_name(p).lower() for k in ("logitech", "gamepad", "xbox", "x-box", "wireless"))]
    return (pads or found or [None])[0]


class Joystick:
    def __init__(self, path: str):
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.ax = [0.0] * 8
        self.ax[AXIS_RT] = -1.0  # released, until the driver reports it
        self.btn = [False] * 16

    def poll(self):
        while True:
            try:
                data = os.read(self.fd, 8)
            except OSError as ex:
                if ex.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    return
                raise
            if len(data) < 8:
                return
            _t, value, typ, number = struct.unpack("<IhBB", data)
            kind = typ & ~JS_EVENT_INIT
            if kind == JS_EVENT_AXIS and number < len(self.ax):
                self.ax[number] = max(-1.0, min(1.0, value / 32767.0))
            elif kind == JS_EVENT_BUTTON and number < len(self.btn):
                self.btn[number] = value != 0




class Pad:
    """One gamepad reading for a Controller, taken once per tick. With no gamepad everything is 0 / False and
    connected is False. Buttons are numbered PAD_A, PAD_B, PAD_X, PAD_Y, PAD_LB, PAD_RB, PAD_BACK, PAD_START (xpad /
    Logitech F710 in X mode - another pad may number them differently; print axis and held to see)."""

    def __init__(self):
        self.connected = False
        self.lx = self.ly = 0.0    # left stick  -1..1 after a 0.1 dead zone: + = right, + = up
        self.rx = self.ry = 0.0    # right stick
        self.axis = [0.0] * 8      # every axis as the driver reports it, -1..1 (Linux: stick up is -; a trigger is
                                   # -1 released .. 1 pressed)
        self.held = [False] * 16     # True while the button is down
        self.pressed = [False] * 16  # True on the one tick the button went down


class Gamepad:
    """Reads a Linux gamepad (/dev/input/js*) without blocking - call read() once per tick."""

    DEAD = 0.1

    def __init__(self):
        self._joy = None
        self._prev = [False] * 16

    def open(self, device: str = "") -> bool:
        """device: a path such as /dev/input/js0, or "" for $JOY or the first gamepad found. False, with a message,
        when there is none."""
        path = pick_joy_path(["", "", device] if device else ["", ""])
        if path is None:
            print("[arm] no gamepad on /dev/input/js* - is the dongle in and the pad on?")
            return False
        self._joy = Joystick(path)
        print(f"[arm] gamepad: {joy_name(path) or '?'} ({path})")
        return True

    def read(self) -> Pad:
        p = Pad()
        if self._joy is None:
            return p
        self._joy.poll()
        dz = lambda v: 0.0 if abs(v) < self.DEAD else (1.0 if v > 0 else -1.0) * (abs(v) - self.DEAD) / (1.0 - self.DEAD)
        ax = self._joy.ax
        p.connected = True
        p.axis = list(ax)
        p.lx, p.ly = dz(ax[AXIS_LEFT_X]), -dz(ax[AXIS_LEFT_Y])
        p.rx, p.ry = dz(ax[AXIS_RIGHT_X]), -dz(ax[AXIS_RIGHT_Y])
        p.held = list(self._joy.btn)
        p.pressed = [h and not was for h, was in zip(p.held, self._prev)]
        self._prev = p.held
        return p
