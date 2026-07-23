"""Python port of rbq_sdk/cpp/example/src/rbq_high_level.cpp.

Publishes HighLevelCommand at 50 Hz and switchControlMode=true at startup,
drives gait via terminal input, and forwards joystick sticks to velocity
setpoints. Matches the C++ example's behavior 1:1.

Usage:
    python3 rbq_high_level.py <networkInterface> [joystickPath]
"""

import array
import errno
import fcntl
import glob
import os
import signal
import struct
import sys
import threading
import time

from rbq_sdk_py.core.channel import ChannelFactoryInitialize, ChannelPublisher

# Locate the sibling `example/idl/` package so `_rbq_idl` resolves when the
# example is launched directly (python3 rbq_high_level.py ...).
sys.path.insert(
    0,
    os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "idl")
    ),
)
from _rbq_idl import HighLevelCommand_, Bool_  # noqa: E402


APP_NAME = "rbq_high_level"


# ---------------------------------------------------------------------------
# Gait table (mirrors GAIT_STATE in rbq_high_level.cpp)
# ---------------------------------------------------------------------------

GAIT_OPTIONS = [
    ("sitting",         0),
    ("standing",        1),
    ("aiming",          2),
    ("trotting",        3),
    ("trot_stairs",     4),
    ("waving",          5),
    ("trot_running",    6),
    ("docking",         10),
    ("rl_trot",         30),
    ("rl_front_walk",   31),
    ("rl_left_walk",    33),
    ("rl_right_walk",   34),
    ("rl_bound",        35),
    ("rl_pace",         36),
    ("rl_pronk",        37),
    ("rl_3leg_hr",      38),
    ("rl_3leg_hl",      39),
    ("rl_3leg_fl",      41),
    ("rl_trot_vision", 42),
    ("rl_trot_run",     45),
    ("rl_silent",       46),
    ("rl_stairs",       47),
    ("rl_end",          80),
]

DEADZONE = 0.12
MAX_VX = 1.2        # m/s
MAX_VY = 0.5        # m/s
# QuadWalk treats HighLevelCommand.omega_z as deg/s and multiplies by D2R
# internally; 60 deg/s ≈ 1.05 rad/s yaw rate.
MAX_VYAW = 60.0     # deg/s

DEFAULT_GAIT_STATE = 1  # STANDING


# ---------------------------------------------------------------------------
# Joystick (linux /dev/input/js* raw API)
# ---------------------------------------------------------------------------

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

JSIOCGNAME_128 = (2 << 30) | (128 << 16) | (ord("j") << 8) | 0x13

AXIS_COUNT = 8
BTN_COUNT = 16
AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_X = 0, 1, 3


def deadzone(v: float, dz: float) -> float:
    if abs(v) < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


def list_js_nodes():
    return sorted(p for p in glob.glob("/dev/input/js*") if os.path.exists(p))


def joy_name(fd: int) -> str:
    buf = array.array("B", [0] * 128)
    try:
        fcntl.ioctl(fd, JSIOCGNAME_128, buf)
    except OSError:
        return ""
    return buf.tobytes().split(b"\0", 1)[0].decode("utf-8", errors="replace")


def name_looks_like_pad(name: str) -> bool:
    n = name.lower()
    return any(k in n for k in ("logitech", "gamepad", "x-box", "xbox", "wireless"))


def try_open(path: str) -> bool:
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        os.close(fd)
        return True
    except OSError:
        return False


def pick_joy_path(argv):
    if len(argv) >= 3:
        return argv[2] if try_open(argv[2]) else None
    env = os.environ.get("JOY", "")
    if env and try_open(env):
        return env
    fallback = ""
    for p in list_js_nodes():
        try:
            fd = os.open(p, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            continue
        try:
            nm = joy_name(fd)
        finally:
            os.close(fd)
        if name_looks_like_pad(nm):
            return p
        if not fallback:
            fallback = p
    if fallback:
        return fallback
    return "/dev/input/js0" if try_open("/dev/input/js0") else None


class Joystick:
    def __init__(self, path: str):
        self.path = path
        self.fd = -1
        self.ax = [0.0] * AXIS_COUNT
        self.btn = [False] * BTN_COUNT
        self.prev = [False] * BTN_COUNT

    def open(self) -> bool:
        try:
            self.fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as ex:
            print(f"조이스틱 열기 실패: {self.path} ({ex})", file=sys.stderr)
            return False
        nm = joy_name(self.fd)
        if nm:
            print(f"조이스틱: {nm} ({self.path})")
        return True

    def close(self):
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def poll(self):
        while True:
            try:
                data = os.read(self.fd, 8)
            except OSError as ex:
                if ex.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                print("조이 read 오류", file=sys.stderr)
                break
            if len(data) < 8:
                break
            _t, value, typ, number = struct.unpack("<IhBB", data)
            k = typ & ~JS_EVENT_INIT
            if k == JS_EVENT_AXIS and number < AXIS_COUNT:
                self.ax[number] = max(-1.0, min(1.0, value / 32767.0))
            elif k == JS_EVENT_BUTTON and number < BTN_COUNT:
                self.btn[number] = value != 0

    def end_frame(self):
        for i in range(BTN_COUNT):
            self.prev[i] = self.btn[i]


# ---------------------------------------------------------------------------
# Shared state (replaces the C++ std::atomic globals)
# ---------------------------------------------------------------------------


class SharedState:
    def __init__(self):
        self.is_working = True
        self.target_gait = DEFAULT_GAIT_STATE
        self.gait_change_pending = False
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.omega_z = 0.0
        # gait_change_pending must be consumed exactly once by the publisher
        # tick (mirrors std::atomic<bool>::exchange in the C++ source).
        self._gait_lock = threading.Lock()

    def consume_gait_pending(self) -> bool:
        with self._gait_lock:
            v = self.gait_change_pending
            self.gait_change_pending = False
            return v


# ---------------------------------------------------------------------------
# Threads
# ---------------------------------------------------------------------------


def control_loop(state: SharedState):
    """Publish HighLevelCommand_ at 50 Hz and SwitchControlMode=true at startup."""
    pub_cmd = ChannelPublisher("rt/rbq/cmd/high_level", HighLevelCommand_)
    pub_cmd.Init()

    # Without EXT_JOY=true the on-robot QuadWalk ignores HighLevelCommand
    # (see src/QuadWalk/main.cpp). Flip it on at startup.
    pub_mode = ChannelPublisher("rt/rbq/cmd/switch_control_mode", Bool_)
    pub_mode.Init()
    time.sleep(0.2)
    on = Bool_(data=True)
    pub_mode.Write(on)
    print("EXT_JOY mode: ON (switchControlMode = true)")

    msg = HighLevelCommand_()
    dt = 0.02  # 50 Hz
    next_tick = time.monotonic()
    while state.is_working:
        next_tick += dt

        msg.gait_state = int(state.target_gait)
        msg.gait_transition = state.consume_gait_pending()
        msg.vel_x = float(state.vel_x)
        msg.vel_y = float(state.vel_y)
        msg.omega_z = float(state.omega_z)
        pub_cmd.Write(msg)

        delay = next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_tick = time.monotonic()

    # Final stop on exit.
    msg.gait_transition = False
    msg.vel_x = 0.0
    msg.vel_y = 0.0
    msg.omega_z = 0.0
    pub_cmd.Write(msg)


def joystick_loop(state: SharedState, joy: Joystick):
    """Poll joystick at 100 Hz and update velocity setpoints."""
    while state.is_working:
        joy.poll()
        lx = deadzone(joy.ax[AXIS_LEFT_X], DEADZONE)
        ly = deadzone(joy.ax[AXIS_LEFT_Y], DEADZONE)
        rx = deadzone(joy.ax[AXIS_RIGHT_X], DEADZONE)
        state.vel_x = -ly * MAX_VX
        state.vel_y = -lx * MAX_VY
        state.omega_z = -rx * MAX_VYAW
        joy.end_frame()
        time.sleep(0.01)


# ---------------------------------------------------------------------------
# Terminal UI
# ---------------------------------------------------------------------------


def print_help():
    print(f"\n {APP_NAME} Help")
    print(f"\n Usage: {APP_NAME} <networkInterface> [joystickPath]")
    print("\n Gait controls (terminal input):")
    print("   list           - list all gait options")
    print("   <name>         - select gait by name (e.g. standing)")
    print("   <id>           - select gait by id   (e.g. 1)")
    print("\n Motion controls (joystick):")
    print("   Left stick     - vel_x / vel_y")
    print("   Right stick X  - omega_z\n")


def convert_to_int(s: str):
    try:
        return int(s)
    except ValueError:
        return None


def terminal_handle(state: SharedState, current):
    try:
        line = input()
    except EOFError:
        state.is_working = False
        return
    line = line.strip()

    if line == "list":
        for name, gid in GAIT_OPTIONS:
            print(f"{name}, id: {gid}")
        return

    candidate_int = convert_to_int(line)
    for name, gid in GAIT_OPTIONS:
        if line == name or candidate_int == gid:
            current["name"] = name
            current["id"] = gid
            state.target_gait = gid
            state.gait_change_pending = True
            print(f"Gait: {name}, gait_state: {gid}")
            return


# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------


def main(argv):
    if len(argv) < 2:
        print(f"Usage: {argv[0]} <networkInterface> [joystickPath]")
        return -1

    state = SharedState()

    def signal_handler(sig, frame):
        print(f"Signal received: {sig}")
        state.is_working = False
        os._exit(sig)

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGHUP, signal_handler)

    jpath = pick_joy_path(argv)
    if not jpath:
        print(
            "Joystick not detected on /dev/input/js*. "
            "Check if USB dongle is connected and joystick is on.",
            file=sys.stderr,
        )
        for p in list_js_nodes():
            print(f"  {p}", file=sys.stderr)
        return -1
    if len(argv) < 3:
        print(f"Joystick path: {jpath}")

    joy = Joystick(jpath)
    if not joy.open():
        return -1

    ChannelFactoryInitialize(0, argv[1])

    current = {"name": "standing", "id": DEFAULT_GAIT_STATE}

    print_help()
    print('Input "list" to list all gait options ...')

    t_joy = threading.Thread(
        target=joystick_loop, args=(state, joy), name="JoyLoop", daemon=True
    )
    t_ctrl = threading.Thread(
        target=control_loop, args=(state,), name="ControlLoop", daemon=True
    )
    t_joy.start()
    t_ctrl.start()

    try:
        while state.is_working:
            terminal_handle(state, current)
    except KeyboardInterrupt:
        state.is_working = False

    t_joy.join()
    t_ctrl.join()
    joy.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
