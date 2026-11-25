"""LiDAR bridge: simulated LiDARs on rt/rbq/lidar/<name>, the same DDS contract as rbq_mujoco / rbq_gazebo.

--livox: the Mid-360 pair at rbq_mujoco's sites (lidar_front / lidar_rear), cast with rbq_mujoco's own
non-repetitive scan pattern (resources/model/lidar/mid360.bin, 24000 rays per frame, 50 m, 10 Hz).
--ouster: one OS1 on the rear deck at rbq_gazebo's pose (lidar_ouster), 1024 x 32 rays over +-22.5 deg,
0.1-100 m, 10 Hz, like the Gazebo gpu_lidar.

Per LiDAR, like rbq_mujoco: PointCloud2 on rt/rbq/lidar/<name> (sensor frame; x,y,z,intensity,time f32 +
timestamp f64, 28 B), Imu on <name>/imu (200 Hz), lidar->base PoseStamped on <name>/tf; for Livox also
base_link -> <name> on rt/tf (2 Hz; the Ouster frame comes from the URDF). HAL's SimSensor turns them into
LidarFrames for SLAM. Rays are cast with Warp against the world's render meshes, snapshotted at start (the
robot itself is left out; free bodies stay where they started).
"""

import math
import os
import queue
import threading

import numpy as np
import warp as wp
from pxr import Usd, UsdGeom

from rbq_sdk_py.core.channel import ChannelPublisher
from rbq_sdk_py.idl.builtin_interfaces.msg.dds_ import Time_
from rbq_sdk_py.idl.geometry_msgs.msg.dds_ import Point_, Pose_, PoseStamped_, Quaternion_, Vector3_
from rbq_sdk_py.idl.sensor_msgs.msg.dds_ import PointCloud2_, PointField_
from rbq_sdk_py.idl.std_msgs.msg.dds_ import Header_

from rbq_sim.idl._sim_idl import Imu_, TFMessage_, Transform_, TransformStamped_

MID360_PATTERN = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                              "rbq_mujoco", "resources", "model", "lidar", "mid360.bin")
CLOUD_HZ, IMU_HZ, TF_HZ = 10.0, 200.0, 2.0

# name, pos, quat (w, x, y, z) in base_link — rbq_mujoco kLivoxSpecs (main.cpp)
LIVOX = [
    ("lidar_front", (0.371894, 0.0, 0.133331), (0.212631537, -0.212630522, -0.674380950, -0.674378547)),
    ("lidar_rear", (-0.332070, 0.0, 0.213325), (0.674378510, 0.674380987, -0.212630639, 0.212631420)),
]
# rbq_gazebo resources/models/sensor/ouster.urdf: lidar_ouster joint, yaw pi
OUSTER = [("lidar_ouster", (-0.3125, 0.0, 0.105), (0.0, 0.0, 0.0, 1.0))]

_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"),
                   ("time", "<f4"), ("timestamp", "<f8")])  # LidarDds::POINT_STEP = 28
_FIELDS = [PointField_(name=n, offset=o, datatype=t, count=1) for n, o, t in
           (("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7), ("time", 16, 7), ("timestamp", 20, 8))]


@wp.kernel
def _cast(mesh: wp.uint64, origin: wp.vec3, dirs: wp.array(dtype=wp.vec3), max_t: float,
          out: wp.array(dtype=float)):
    i = wp.tid()
    q = wp.mesh_query_ray(mesh, origin, dirs[i], max_t)
    out[i] = wp.where(q.result, q.t, -1.0)


def _rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _matched(pub) -> bool:
    try:
        return pub._ChannelPublisher__channel._Channel__writer._Writer__publication_matched_count > 0
    except AttributeError:
        return True


def _split(ns: int):
    return ns // 1_000_000_000, ns % 1_000_000_000


def _world_mesh(stage, device):
    """All visible meshes / cubes / spheres outside the robot, in world coordinates, as one Warp mesh."""
    pts, tris, n0 = [], [], 0
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if path.startswith("/World/envs") or not prim.IsA(UsdGeom.Gprim):
            continue
        img = UsdGeom.Imageable(prim)
        if img.ComputeVisibility() == UsdGeom.Tokens.invisible or img.ComputePurpose() == UsdGeom.Tokens.guide:
            continue
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            p = mesh.GetPointsAttr().Get()
            counts, idx = mesh.GetFaceVertexCountsAttr().Get(), mesh.GetFaceVertexIndicesAttr().Get()
            if not p or not counts:
                continue
            p, idx, counts = np.asarray(p, dtype=np.float64), np.asarray(idx), np.asarray(counts)
            if (counts == 3).all():
                f = idx.reshape(-1, 3)
            else:  # fan-triangulate
                f, k = [], 0
                for c in counts:
                    f.extend((idx[k], idx[k + j], idx[k + j + 1]) for j in range(1, c - 1))
                    k += c
                f = np.asarray(f, dtype=np.int64).reshape(-1, 3)
        elif prim.IsA(UsdGeom.Cube):
            s = UsdGeom.Cube(prim).GetSizeAttr().Get() / 2.0
            p = np.array([[x, y, z] for x in (-s, s) for y in (-s, s) for z in (-s, s)], dtype=np.float64)
            f = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                          [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
        elif prim.IsA(UsdGeom.Sphere):
            r, u, v = UsdGeom.Sphere(prim).GetRadiusAttr().Get(), 16, 8
            p = np.array([[r * math.sin(math.pi * j / v) * math.cos(2 * math.pi * i / u),
                           r * math.sin(math.pi * j / v) * math.sin(2 * math.pi * i / u),
                           r * math.cos(math.pi * j / v)] for j in range(v + 1) for i in range(u)])
            f = np.array([[j * u + i, (j + 1) * u + i, (j + 1) * u + (i + 1) % u] for j in range(v) for i in range(u)]
                         + [[j * u + i, (j + 1) * u + (i + 1) % u, j * u + (i + 1) % u] for j in range(v) for i in range(u)])
        else:
            continue
        m = np.array(cache.GetLocalToWorldTransform(prim), dtype=np.float64)  # row-vector convention
        pw = np.c_[p, np.ones(len(p))] @ m
        pts.append(pw[:, :3])
        tris.append(f + n0)
        n0 += len(p)
    if not pts:
        return None, 0
    P, T = np.concatenate(pts).astype(np.float32), np.concatenate(tris).astype(np.int32).reshape(-1)
    mesh = wp.Mesh(points=wp.array(P, dtype=wp.vec3, device=device),
                   indices=wp.array(T, dtype=wp.int32, device=device))
    return mesh, len(T) // 3


class _Lidar:
    def __init__(self, name, pos, quat, local_dirs, samples, rmin, rmax, device, tf_topic):
        self.name, self.pos, self.quat = name, np.array(pos), quat
        self.R = _rot(quat)                      # sensor -> base
        self.dirs, self.samples, self.rmin, self.rmax = local_dirs, samples, rmin, rmax
        self.cursor = 0
        self._dirs_w = wp.empty(samples, dtype=wp.vec3, device=device)
        self._dist = wp.empty(samples, dtype=float, device=device)
        base = f"rt/rbq/lidar/{name}"
        self.cloud, self.imu, self.tf = (ChannelPublisher(base, PointCloud2_), ChannelPublisher(base + "/imu", Imu_),
                                         ChannelPublisher(base + "/tf", PoseStamped_))
        for p in (self.cloud, self.imu, self.tf):
            p.Init()
        self.rt_tf = tf_topic                    # True: also base_link -> name on rt/tf (Livox, like MuJoCo)

    def window(self):
        n = len(self.dirs)
        idx = (self.cursor + np.arange(self.samples)) % n
        self.cursor = (self.cursor + self.samples) % n
        return self.dirs[idx]


class LidarBridge:
    """Casts the LiDARs every 1/CLOUD_HZ of sim time; packs and publishes on a worker thread."""

    def __init__(self, kind: str, robot, imu, stage, control_dt: float, device: str = "cuda:0"):
        wp.init()
        self.robot, self.imu, self.device, self.dt = robot, imu, device, control_dt
        self.mesh, ntri = _world_mesh(stage, device)
        if kind == "livox":
            pat = np.fromfile(MID360_PATTERN, dtype=np.float32).reshape(-1, 2)
            th, ph = pat[:, 0], pat[:, 1]
            dirs = np.stack([np.cos(ph) * np.cos(th), np.cos(ph) * np.sin(th), np.sin(ph)], 1)
            self.lidars = [_Lidar(n, p, q, dirs, 24000, 0.0, 50.0, device, True) for n, p, q in LIVOX]
        else:
            th = np.linspace(-math.pi, math.pi, 1024, endpoint=False)
            ph = np.linspace(-0.392699, 0.392699, 32)
            T, P = np.meshgrid(th, ph)
            dirs = np.stack([np.cos(P) * np.cos(T), np.cos(P) * np.sin(T), np.sin(P)], -1).reshape(-1, 3)
            self.lidars = [_Lidar(n, p, q, dirs, len(dirs), 0.1, 100.0, device, False) for n, p, q in OUSTER]
        self.cloud_every = max(1, round(1.0 / CLOUD_HZ / control_dt))
        self.tf_every = max(1, round(1.0 / TF_HZ / control_dt))
        self._imu_next = 0.0
        self._rt_tf = ChannelPublisher("rt/tf", TFMessage_)
        self._rt_tf.Init()
        self._q = queue.Queue(maxsize=2)
        threading.Thread(target=self._worker, daemon=True).start()
        names = ", ".join(l.name for l in self.lidars)
        print(f"[lidar] {kind}: {names} -> rt/rbq/lidar/<name>; world mesh {ntri} triangles")

    def step(self, tick: int, stamp_ns: int):
        """Call once per control period. Stamps are sim time, the /clock RViz and Network's topics follow
        (rbq_mujoco stamps wall clock, but it runs in real time); HAL takes either (SimSensor wallStamp)."""
        self._stamp_ns = stamp_ns
        t = tick * self.dt
        if t >= self._imu_next:
            self._imu_next += 1.0 / IMU_HZ
            self._publish_imu()
        if tick % self.tf_every == 0:
            self._publish_rt_tf()
        if tick % self.cloud_every or self.mesh is None:
            return
        d = self.robot.data
        pos_b = d.root_link_pos_w[0].cpu().numpy().astype(np.float64)
        R_b = _rot(d.root_link_quat_w[0].cpu().numpy().astype(np.float64))
        for lid in self.lidars:
            if not _matched(lid.cloud) and not _matched(lid.tf):
                continue
            local = lid.window()
            R_s = R_b @ lid.R
            origin = pos_b + R_b @ lid.pos
            lid._dirs_w.assign(wp.array((local @ R_s.T).astype(np.float32), dtype=wp.vec3, device=self.device))
            wp.launch(_cast, dim=lid.samples, device=self.device,
                      inputs=[self.mesh.id, wp.vec3(*origin.astype(np.float32)), lid._dirs_w, float(lid.rmax), lid._dist])
            try:
                self._q.put_nowait((lid, local, lid._dist.numpy().copy(), stamp_ns))
            except queue.Full:  # publisher still busy: drop, like a LiDAR under load
                pass

    def _publish_imu(self):
        gyro = self.imu.data.ang_vel_b[0].cpu().numpy().astype(np.float64)
        acc = self.imu.data.lin_acc_b[0].cpu().numpy().astype(np.float64)
        sec, ns = _split(self._stamp_ns)
        for lid in self.lidars:
            if not _matched(lid.imu):
                continue
            g, a = lid.R.T @ gyro, lid.R.T @ acc  # base -> sensor axes (lever-arm terms left out)
            m = Imu_()
            m.header = Header_(stamp=Time_(sec=sec, nanosec=ns), frame_id=lid.name)
            m.orientation = Quaternion_(x=0.0, y=0.0, z=0.0, w=1.0)
            m.orientation_covariance[0] = -1.0  # orientation unknown (REP-145), as LidarDds::fillImu
            m.angular_velocity = Vector3_(x=float(g[0]), y=float(g[1]), z=float(g[2]))
            m.linear_acceleration = Vector3_(x=float(a[0]), y=float(a[1]), z=float(a[2]))
            lid.imu.Write(m)

    def _publish_rt_tf(self):
        sec, ns = _split(self._stamp_ns)
        tfs = [TransformStamped_(header=Header_(stamp=Time_(sec=sec, nanosec=ns), frame_id="base_link"),
                                 child_frame_id=lid.name,
                                 transform=Transform_(translation=Vector3_(x=float(lid.pos[0]), y=float(lid.pos[1]), z=float(lid.pos[2])),
                                                      rotation=Quaternion_(w=lid.quat[0], x=lid.quat[1], y=lid.quat[2], z=lid.quat[3])))
               for lid in self.lidars if lid.rt_tf]
        if tfs:
            self._rt_tf.Write(TFMessage_(transforms=tfs))

    def _worker(self):
        while True:
            lid, local, dist, stamp_ns = self._q.get()
            sec, ns = _split(stamp_ns)
            if _matched(lid.tf):
                p, q = lid.pos, lid.quat
                lid.tf.Write(PoseStamped_(header=Header_(stamp=Time_(sec=sec, nanosec=ns), frame_id="base"),
                                          pose=Pose_(position=Point_(x=float(p[0]), y=float(p[1]), z=float(p[2])),
                                                     orientation=Quaternion_(w=q[0], x=q[1], y=q[2], z=q[3]))))
            if not _matched(lid.cloud):
                continue
            hit = (dist > max(lid.rmin, 0.0)) & (dist < lid.rmax)
            i = np.nonzero(hit)[0]
            pts = np.empty(len(i), dtype=_DTYPE)
            xyz = local[i] * dist[i, None]
            pts["x"], pts["y"], pts["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
            pts["intensity"] = dist[i]  # no reflectivity; range, as rbq_mujoco
            pts["time"] = i * (1.0 / CLOUD_HZ / lid.samples)
            pts["timestamp"] = sec + ns * 1e-9 + pts["time"]
            lid.cloud.Write(PointCloud2_(header=Header_(stamp=Time_(sec=sec, nanosec=ns), frame_id=lid.name),
                                         height=1, width=len(i), fields=_FIELDS, is_bigendian=False,
                                         point_step=_DTYPE.itemsize, row_step=_DTYPE.itemsize * len(i),
                                         data=pts.tobytes(), is_dense=True))
