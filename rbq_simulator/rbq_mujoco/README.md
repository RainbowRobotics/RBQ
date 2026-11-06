# rbq_mujoco

- MuJoCo physics simulator for the RBQ quadruped Robot.

## Layout

- `src/` — simulator sources → `bin/rbq_mujoco`
- `resources/model/` — MJCF models (`default.xml`, `wheel.xml`, `env/`, `rbq/`)
- `scripts/` — build / run (`mujoco.bash`) / docker

## Robot variants

| Flag | Robot | Model |
|---|---|---|
| *(none)* | RBQ quadruped, 12 joints | `resources/model/default.xml` |
| `--wheel` | Wheeled RBQ, 12 joints + 4 driven wheels | `resources/model/wheel.xml` |

The flag picks the joint layout, spawn pose and contact bodies; `-p` overrides only which
MJCF file is loaded. Wheel state/refs ride `SimInfo_.whl_joint` / `MotionRef_.whl_joint`,
so the robot-side Motion must run with `--wheel` too.

## LiDAR

Opt-in; nothing is drawn and no LiDAR runs without a flag.

| Flag | Device | Mounting |
|---|---|---|
| *(none)* | none | — |
| `--livox` | Livox Mid-360 x2 | front (+35deg up) / rear (-35deg up) |
| `--ouster` | Ouster OS1 x1 | rear deck |

```bash
./bin/rbq_mujoco --livox
./bin/rbq_mujoco --ouster
./bin/rbq_mujoco --wheel --livox
```

The ray-origin sites (`lidar_front` / `lidar_rear` / `lidar_ouster`) and their IMU sensors are always
in the model. `--livox` draws the two Mid-360 housings *and* starts their raytracer threads;
`--ouster` only draws its housing, since that sensor is not simulated yet. Housings live in geom
group 4 (Livox) / 5 (Ouster), which MuJoCo hides by default. The STLs are in
`resources/meshes/sensor/`, each sensor split into body + optical window so the two halves can
carry different materials.

### Housing physics

The MJCF carries the housings as meshes only; the flag also attaches, at load time, a collision hull
and the sensor's mass. Without a flag the model is the bare robot, unchanged.

| Flag | Added mass | Collision hull | Tallest point above the trunk hull |
|---|---|---|---|
| `--livox` | 2 x 0.265 kg | box, 72 x 60 x 65 mm, tilted with the sensor | rear +183 mm |
| `--ouster` | 0.447 kg | cylinder, Ø87 x 74 mm | +113 mm |

The LiDAR is the tallest part of the robot and the front Mid-360 its foremost point, so without the
hull the sim walks under an overhang that would shear the sensor off. The mass is not free either:
the Mid-360 pair raises the trunk CoM ~5 mm and its pitch inertia ~13 %.

The simulator that `bash scripts/sim.bash` launches spells the same thing `--payload livox|ouster`,
reachable as `bash scripts/sim.bash -p livox`. Note `--no-sim` skips that simulator entirely, so
the option is ignored there.

### Simulated Livox Mid-360 output

`--livox` starts one raytracer thread per sensor. Each publisher is gated on having a subscriber,
so topics nobody reads cost nothing beyond the thread itself — which is why the flag gates the
threads too rather than running them always.

| Topic | Type | Rate |
|---|---|---|
| `rt/rbq/lidar/lidar_front`, `rt/rbq/lidar/lidar_rear` | `sensor_msgs/PointCloud2` | 10 Hz |
| `<cloud topic>/imu` | `sensor_msgs/Imu` | 200 Hz |
| `<cloud topic>/tf` | `geometry_msgs/PoseStamped`, lidar -> base | with each cloud |
| `rt/tf` | `tf2_msgs/TFMessage`, `base_link` -> site | 2 Hz per sensor |

Points are in the sensor frame, `point_step` 28 (`x,y,z,intensity,time` f32 + `timestamp` f64). The
builders live in `<rbq_sdk/dds/LidarDds.hpp>`, the same header the on-robot driver and the SLAM
consumer use, so producer and consumer cannot drift apart. The non-repetitive scan pattern is a
table of 800,000 `[theta, phi]` float32 pairs in `resources/model/lidar/mid360.bin`; 24,000 of them
are cast per frame with `mj_multiRay`.

Ouster is not simulated yet: `--ouster` only draws the housing.

## SLAMNAV_3D

SLAMNAV_3D reads LiDAR only as the HAL's Vision_IPC `LidarFrame`, the same as on the robot. In sim, HAL
builds it from this simulator's DDS cloud + IMU (`rt/rbq/lidar/lidar_*`, `.../imu`, `.../tf`) when the RBQ
stack gets the same LiDAR as `-p`:

```bash
bash scripts/sim.bash --slam --no-sim -p livox     # from the repo root; --slam brings up Vision (HAL)
bash scripts/mujoco.bash --livox                    # here
```

## RViz

Off by default; `--rviz` opens the same RViz view `rbq_gazebo` uses (launch, config and URDFs live in
`../rbq_gazebo/ros2/src/sensor_bridge`, run with `sim:=mujoco`). Build that package once — it needs ROS 2
Humble but not Gazebo:

```bash
source /opt/ros/humble/setup.bash && (cd ../rbq_gazebo/ros2 && colcon build --symlink-install)

bash scripts/mujoco.bash --rviz
bash scripts/mujoco.bash --livox --rviz
```

What `sim:=mujoco` changes against Gazebo:

| | Gazebo | MuJoCo |
|---|---|---|
| Clock | `/clock` (`use_sim_time`) | wall clock |
| Joint states | `/joint_states` (gz bridge) | `/rbq/joint_states` (Network) |
| Livox cloud | `/rbq/lidar/lidar_*/points` | `/rbq/lidar/lidar_*` (RViz remaps it) |
| `base_link -> lidar_*` | URDF | the sim's own `/tf`, so the Livox housings are not drawn |

Joint states and the Fixed Frame `local_world` come from the RBQ stack (Network), so run it alongside.
Host only — the Docker image carries no ROS.

## Local - Run rbq_mujoco with RBQ Full Stack

Run the full RBQ stack (Motion / Vision / GUI) with the MuJoCo simulator:

```bash
# Clone
git clone https://github.com/RainbowRobotics/RBQ.git
cd RBQ

# Install Dependency (first time only)
cd rbq_simulator/rbq_mujoco && bash scripts/debian-dep.bash 

# Build rbq_mujoco
bash scripts/build.bash

# Run the RBQ stack
cd ../../
bash scripts/sim.bash --vision --no-sim

# Run rbq_mujoco
cd rbq_simulator/rbq_mujoco
./bin/rbq_mujoco                # quadruped

./bin/rbq_mujoco --wheel        # wheeled RBQ
```

## Docker - Run rbq_mujoco with RBQ Full Stack

Run the full RBQ stack (Motion / Vision / GUI) with the MuJoCo simulator in Docker:

```bash
# Clone
git clone https://github.com/RainbowRobotics/RBQ.git
cd RBQ

# Run the RBQ stack
bash scripts/sim.bash --vision --no-sim

# Run rbq_mujoco in Docker
cd RBQ/rbq_simulator/rbq_mujoco
bash scripts/docker/run.bash                # quadruped

bash scripts/docker/run.bash --wheel        # wheeled RBQ
```

- Shares DDS domain 0 / `lo` with the host RBQ stack via `--network host`.