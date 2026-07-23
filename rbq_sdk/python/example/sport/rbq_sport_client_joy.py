from __future__ import annotations

import array
import errno
import fcntl
import glob
import os
import struct
import sys
import time

from rbq_sdk_py.core.channel import ChannelFactoryInitialize
from rbq_sdk_py.sport.sport_client import SportClient
from rbq_sdk_py.sport.sport_api import (
    ID_DEFAULT_MODE,
    ID_RUNNING_MODE,
    ID_CLIMB_MODE,
)

# --- js API (linux/joystick.h) ---
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

JSIOCGNAME_128 = (2 << 30) | (128 << 16) | (ord("j") << 8) | 0x13

AXIS_COUNT = 8
BTN_COUNT = 16
AXIS_LX, AXIS_LY, AXIS_RX = 0, 1, 3
AXIS_LT, AXIS_RT = 2, 5  # Xbox/Logitech trigger axis layout; idle -1, full +1
PAD_A, PAD_B, PAD_X, PAD_Y = 0, 1, 2, 3
PAD_LB, PAD_RB, PAD_BACK, PAD_START = 4, 5, 6, 7

DEADZONE = 0.12
MAX_VX, MAX_VY, MAX_VYAW = 1, 0.5, 1.0
DOUBLE_CLICK_WINDOW = 0.4  # seconds


def deadzone(v: float, dz: float) -> float:
    if abs(v) < dz:
        return 0.0
    s = 1.0 if v > 0 else -1.0
    return s * (abs(v) - dz) / (1.0 - dz)


def list_js_nodes() -> list[str]:
    paths = sorted(glob.glob("/dev/input/js*"))
    return [p for p in paths if os.path.exists(p)]


def joy_name(fd: int) -> str:
    buf = array.array("B", [0] * 128)
    try:
        fcntl.ioctl(fd, JSIOCGNAME_128, buf)
    except OSError:
        return ""
    return buf.tobytes().split(b"\0", 1)[0].decode("utf-8", errors="replace")


def name_looks_like_pad(name: str) -> bool:
    n = name.lower()
    return any(
        k in n for k in ("logitech", "gamepad", "x-box", "xbox", "wireless")
    )


def try_open(path: str) -> bool:
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        os.close(fd)
        return True
    except OSError:
        return False


def pick_joy_path(argv: list[str]) -> str | None:
    if len(argv) >= 3:
        return argv[2] if try_open(argv[2]) else None
    e = os.environ.get("JOY", "")
    if e and try_open(e):
        return e
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


class Joy:
    def __init__(self, path: str) -> None:
        self.path = path
        self.fd = -1
        self.ax = [0.0] * AXIS_COUNT
        self.btn = [False] * BTN_COUNT
        self.prev = [False] * BTN_COUNT
        self.edge = [False] * BTN_COUNT

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

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def poll(self) -> None:
        assert self.fd >= 0
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
            is_init = (typ & JS_EVENT_INIT) != 0
            k = typ & ~JS_EVENT_INIT
            if k == JS_EVENT_AXIS and number < AXIS_COUNT:
                v = max(-1.0, min(1.0, value / 32767.0))
                self.ax[number] = v
            elif k == JS_EVENT_BUTTON and number < BTN_COUNT:
                on = value != 0
                self.btn[number] = on
                if on and not self.prev[number] and not is_init:
                    self.edge[number] = True

    def end_frame(self) -> None:
        for i in range(BTN_COUNT):
            self.prev[i] = self.btn[i]
            self.edge[i] = False


def main() -> int:
    # A2 Robot's bluetooth gamepad bindings (emulated):
    #   L2 + B          : Damping (e-stop)
    #   L2 + A          : Stand/Prone toggle ("press again to go prone")
    #   START           : Default gait (unlock)
    #   L2 + START      : Running mode
    #   Double-click X  : Left-side gait
    #   Double-click Y  : Right-side gait
    #   Double-click R1 : Handstand
    #   Double-click R2 : Biped (erect) stand
    #   L2 + X          : Recovery stand (stand up from fall)
    #   R1 + X          : Climb
    #   L1 + X          : Front flip
    #   L1 + Y          : Back flip
    #   BACK            : StopMove (escape; not in A2 native scheme)

    if len(sys.argv) < 2:
        print(f"사용법: python3 {sys.argv[0]} <NIC> [조이경로]", file=sys.stderr)
        return 1

    jpath = pick_joy_path(sys.argv)
    if not jpath:
        print("조이스틱 없음. ls /dev/input/js*", file=sys.stderr)
        for p in list_js_nodes():
            print(f"  {p}", file=sys.stderr)
        return 1
    if len(sys.argv) < 3:
        print(f"조이 경로: {jpath}")

    joy = Joy(jpath)
    if not joy.open():
        return 1

    nic = sys.argv[1]
    ChannelFactoryInitialize(0, nic)
    sport = SportClient()
    sport.Init()

    # Chord/double-click state.
    last_edge = {"x": -1.0, "y": -1.0, "rb": -1.0, "r2": -1.0}
    r2_prev_pressed = False
    lock_toggle_to_prone = False  # False → next L2+A goes StandUp; True → StandDown

    def consume_double(edge: bool, key: str) -> bool:
        if not edge:
            return False
        now = time.monotonic()
        if last_edge[key] >= 0.0 and (now - last_edge[key]) < DOUBLE_CLICK_WINDOW:
            last_edge[key] = -1.0  # consume; require a fresh pair next
            return True
        last_edge[key] = now
        return False

    err_streak = 0
    dt = 0.1  # 10 Hz — high-level commands; A2 controller interpolates Move targets
    try:
        while True:
            t0 = time.perf_counter()
            joy.poll()

            L1 = joy.btn[PAD_LB]
            R1 = joy.btn[PAD_RB]
            L2 = joy.ax[AXIS_LT] > 0.0
            r2_pressed = joy.ax[AXIS_RT] > 0.0
            r2_edge = r2_pressed and not r2_prev_pressed
            r2_prev_pressed = r2_pressed

            code = 0
            label = None

            # ---- chord bindings (checked before double-click) ----
            if joy.edge[PAD_B] and L2:
                code, label = sport.Damp(), "Damp"
            elif joy.edge[PAD_A] and L2:
                if lock_toggle_to_prone:
                    code, label = sport.StandDown(), "StandDown (prone)"
                else:
                    code, label = sport.StandUp(), "StandUp (lock)"
                lock_toggle_to_prone = not lock_toggle_to_prone
            elif joy.edge[PAD_START]:
                if L2:
                    code, label = sport.SwitchGait(ID_RUNNING_MODE), "RunningMode"
                else:
                    code, label = sport.SwitchGait(ID_DEFAULT_MODE), "DefaultMode"
            elif joy.edge[PAD_X] and L2:
                code, label = sport.RecoveryStand(), "RecoveryStand"
            elif joy.edge[PAD_X] and R1:
                code, label = sport.SwitchGait(ID_CLIMB_MODE), "Climb"
            elif joy.edge[PAD_X] and L1:
                code, label = sport.FrontFlip(), "FrontFlip"
            elif joy.edge[PAD_Y] and L1:
                code, label = sport.BackFlip(), "BackFlip"
            # ---- double-click bindings on X, Y, R1 (button) and R2 (trigger) ----
            elif consume_double(joy.edge[PAD_X], "x"):
                code, label = sport.LeftSideGait(1), "LeftSide"
            elif consume_double(joy.edge[PAD_Y], "y"):
                code, label = sport.RightSideGait(1), "RightSide"
            elif consume_double(joy.edge[PAD_RB], "rb"):
                code, label = sport.HandStand(1), "Handstand"
            elif consume_double(r2_edge, "r2"):
                code, label = sport.BipedStand(1), "BipedStand"
            # ---- escape ----
            elif joy.edge[PAD_BACK]:
                code, label = sport.StopMove(), "StopMove"
            # ---- streaming Move (fire-and-forget) ----
            else:
                lx = deadzone(joy.ax[AXIS_LX], DEADZONE)
                ly = deadzone(joy.ax[AXIS_LY], DEADZONE)
                rx = deadzone(joy.ax[AXIS_RX], DEADZONE)
                code = sport.Move(-ly * MAX_VX, -lx * MAX_VY, -rx * MAX_VYAW)

            if label is not None:
                print(label)
                print(f"res: {code}")

            if code != 0:
                err_streak += 1
                if err_streak % 50 == 1:
                    print(f"Sport error {code} ({err_streak})", file=sys.stderr)
            else:
                err_streak = 0

            joy.end_frame()
            elapsed = time.perf_counter() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
    finally:
        joy.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
