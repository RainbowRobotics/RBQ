"""Vision bridge: Isaac cameras -> rt/rbq/vision/sensor_N, the same DDS contract as rbq_mujoco / rbq_gazebo.

Six cameras on the trunk at the poses of resources/model/rbq/rbq.xml: BT0-BT3 (bottom, depth + IR) and
FT0 / RR0 (front / rear, depth + RGB). Per stream: `<base>/<kind>/compressed` (RGB / IR JPEG, depth 16-bit
PNG in mm), `<kind>/camera_info` and `<kind>/tf` (camera -> base, optical frame). HAL's SimSensor feeds
them into Vision_IPC. Encoding runs on a worker thread and only for streams someone subscribes to.
"""

import queue
import threading
import time

import cv2
import numpy as np

from isaaclab.sensors import CameraCfg
import isaaclab.sim as sim_utils

from rbq_sdk_py.core.channel import ChannelPublisher
from rbq_sdk_py.idl.builtin_interfaces.msg.dds_ import Time_
from rbq_sdk_py.idl.geometry_msgs.msg.dds_ import Point_, Pose_, PoseStamped_, Quaternion_
from rbq_sdk_py.idl.sensor_msgs.msg.dds_ import CompressedImage_
from rbq_sdk_py.idl.std_msgs.msg.dds_ import Header_

from rbq_sim.idl._sim_idl import CameraInfo_

WIDTH, HEIGHT = 640, 360
FPS = 15.0
FX = FY = 320.0  # square pixels, ~90 deg horizontal FOV (MuJoCo: focal 1.93 mm on a 3.9 mm sensor)
CX, CY = WIDTH / 2.0, HEIGHT / 2.0
JPEG_QUALITY = 70
PNG_LEVEL = 1  # fast; depth PNGs are lossless at any level

# name, sensor id, pos, quat (w, x, y, z) in the trunk frame (rbq.xml *_body), colour stream
CAMERAS = [
    ("BT0", 0, (0.364, 0.0, -0.024919), (0.0, -0.1736482, 0.0, 0.9848078), "ir"),
    ("BT1", 1, (0.26097, 0.0, -0.04582), (0.0, 0.1218693, 0.0, 0.9925462), "ir"),
    ("BT2", 2, (-0.19515, 0.0065, -0.0465), (0.0, 0.0, 0.0, 1.0), "ir"),
    ("BT3", 3, (-0.352082, -0.000011, -0.018938), (0.9848078, 0.0, 0.1736482, 0.0), "ir"),
    ("FT0", 4, (0.388031, -0.0375, 0.037764), (0.5, 0.5, -0.5, -0.5), "rgb"),
    ("RR0", 5, (-0.388031, 0.0375, 0.037764), (-0.5, -0.5, -0.5, -0.5), "rgb"),
]


def camera_cfgs() -> dict:
    """CameraCfg per camera, keyed by scene attribute name. MuJoCo camera frames are OpenGL (look -Z, up +Y)."""
    spawn = sim_utils.PinholeCameraCfg.from_intrinsic_matrix(
        [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0], width=WIDTH, height=HEIGHT, clipping_range=(0.05, 20.0))
    return {
        f"cam_{name.lower()}": CameraCfg(
            prim_path=f"{{ENV_REGEX_NS}}/Robot/trunk/cam_{name.lower()}",
            offset=CameraCfg.OffsetCfg(pos=pos, rot=quat, convention="opengl"),
            spawn=spawn,
            width=WIDTH,
            height=HEIGHT,
            update_period=1.0 / FPS,
            data_types=["rgb", "distance_to_image_plane"],
        )
        for name, _, pos, quat, _ in CAMERAS
    }


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def _matched(pub) -> bool:
    """Whether a reader is matched. rbq_sdk_py keeps the count private; assume yes if it cannot be read."""
    try:
        return pub._ChannelPublisher__channel._Channel__writer._Writer__publication_matched_count > 0
    except AttributeError:
        return True


def _header(frame: str) -> Header_:
    ns = time.time_ns()  # wall clock, like rbq_mujoco: HAL and Streamer age frames against now
    return Header_(stamp=Time_(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000), frame_id=frame)


class _Stream:
    def __init__(self, base: str, kind: str, pose):
        self.topic = f"{base}/{kind}"
        self.img = ChannelPublisher(self.topic + "/compressed", CompressedImage_)
        self.info = ChannelPublisher(self.topic + "/camera_info", CameraInfo_)
        self.tf = ChannelPublisher(self.topic + "/tf", PoseStamped_)
        for p in (self.img, self.info, self.tf):
            p.Init()
        self.pose = pose

    def publish(self, encode):
        if _matched(self.tf):
            (x, y, z), (qw, qx, qy, qz) = self.pose
            self.tf.Write(PoseStamped_(header=_header("base"), pose=Pose_(
                position=Point_(x=x, y=y, z=z), orientation=Quaternion_(x=qx, y=qy, z=qz, w=qw))))
        if _matched(self.img):
            fmt, data = encode()
            if data is not None:
                self.img.Write(CompressedImage_(header=_header(self.topic), format=fmt, data=data.tobytes()))
        if _matched(self.info):
            self.info.Write(CameraInfo_(
                header=_header(self.topic), height=HEIGHT, width=WIDTH, distortion_model="rational_polynomial",
                d=[0.0] * 8, k=[FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0], r=[1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0],
                p=[FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]))


class VisionBridge:
    """Reads the scene cameras at FPS (sim time) and publishes them from a worker thread."""

    def __init__(self, scene, control_dt: float):
        self.scene = scene
        self.every = max(1, round(1.0 / FPS / control_dt))
        self.frames = 0
        self._q = queue.Queue(maxsize=1)
        self._cams = []
        for name, sid, pos, quat, colour in CAMERAS:
            # OpenGL camera frame -> optical (x right, y down, z forward): rotate 180 deg about x.
            pose = (pos, _qmul(quat, (0.0, 1.0, 0.0, 0.0)))
            base = f"rt/rbq/vision/sensor_{sid}"
            self._cams.append((scene[f"cam_{name.lower()}"], colour,
                               _Stream(base, "depth", pose), _Stream(base, colour, pose)))
        threading.Thread(target=self._worker, daemon=True).start()
        print(f"[vision] {len(CAMERAS)} cameras {WIDTH}x{HEIGHT} @ {FPS:.0f} fps -> rt/rbq/vision/sensor_0..5")

    def step(self, tick: int):
        """Call once per control period; grabs a frame set every 1/FPS of sim time."""
        if tick % self.every:
            return
        shots = []
        for cam, colour, _, _ in self._cams:
            out = cam.data.output
            shots.append((out["rgb"][0, :, :, :3].cpu().numpy(), out["distance_to_image_plane"][0, :, :, 0].cpu().numpy()))
        try:
            self._q.put_nowait(shots)
        except queue.Full:  # encoder still busy with the last set: drop, like a camera under load
            pass

    def _worker(self):
        while True:
            shots = self._q.get()
            for (rgb, dist), (_, colour, depth_s, colour_s) in zip(shots, self._cams):
                depth_s.publish(lambda d=dist: ("png", _encode_depth(d)))
                if colour == "rgb":
                    colour_s.publish(lambda c=rgb: ("jpeg", _encode_jpeg(cv2.cvtColor(c, cv2.COLOR_RGB2BGR))))
                else:
                    colour_s.publish(lambda c=rgb: ("jpeg", _encode_jpeg(cv2.cvtColor(c, cv2.COLOR_RGB2GRAY))))
            self.frames += 1


def _encode_jpeg(img):
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    return buf if ok else None


def _encode_depth(dist_m):
    mm = np.nan_to_num(dist_m, nan=0.0, posinf=0.0, neginf=0.0) * 1000.0
    mm[(mm < 0) | (mm > 65535)] = 0  # out of range reads as no return, like a depth camera
    ok, buf = cv2.imencode(".png", mm.astype(np.uint16), [cv2.IMWRITE_PNG_COMPRESSION, PNG_LEVEL])
    return buf if ok else None
