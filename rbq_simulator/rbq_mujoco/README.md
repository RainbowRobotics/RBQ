# rbq_mujoco

- MuJoCo physics simulator for the RBQ quadruped Robot.

## Layout

- `src/` — simulator sources → `bin/rbq_mujoco`
- `resources/model/` — MJCF models (`default.xml`, `wheel.xml`, `env/`, `rbq/`)
- `scripts/` — build / docker

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