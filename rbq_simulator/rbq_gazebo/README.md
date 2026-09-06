# rbq_gazebo

- Ignition Gazebo (Fortress) simulator for the RBQ quadruped Robot.

## Layout

- `src/` — simulator sources → `bin/rbq_gazebo`
- `resources/` — worlds, robot/sensor URDFs, meshes
- `ros2/src/sensor_bridge/` — colcon package: bridge launch, RViz config, DDS xml
- `scripts/` — build / run / docker

## Robot variants

| Flag | Robot | Model |
|---|---|---|
| *(none)* | RBQ quadruped, 12 joints | `resources/models/rbq10/` |
| `--wheel` | Wheeled RBQ, 12 joints + 4 driven wheels | `resources/models/rbq10_wheel/` |

The flag picks the spawned model, the joint table and the spawn pose; `gazebo.bash` also
passes the matching `variant:=` to the sensor bridge so RViz loads the same URDF. Wheel
state/refs ride `SimInfo_.whl_joint` / `MotionRef_.whl_joint`, so the robot-side Motion
must run with `--wheel` too.

## LiDAR

Opt-in; nothing is spawned without a flag.

| Flag | Device | Mounting |
|---|---|---|
| *(none)* | none | — |
| `--livox` | Livox Mid-360 x2 | front (+35deg up) / rear (-35deg up) |
| `--ouster` | Ouster OS1 x1 | rear deck |

```bash
bash scripts/gazebo.bash --livox
bash scripts/gazebo.bash --ouster
bash scripts/gazebo.bash --wheel --livox
```

`gazebo.bash` forwards the value to the sensor bridge as `lidar:=`, so one flag covers all three:
the spawn URDF gets that LiDAR's `<visual>` + `<sensor>` spliced in, the static
`base_link -> lidar_*` TFs are published, and RViz loads the URDF generated for the same
combination (`rbq10_livox.urdf`, ...) so the housings show there too. Those per-LiDAR URDFs are
generated in `ros2/src/sensor_bridge/CMakeLists.txt`; the housing meshes live in
`resources/meshes/sensor/`.

An empty RViz almost always means the RBQ stack is not running: the bundled config's Fixed Frame is
`local_world`, which the Network daemon publishes — not Gazebo. Run
`bash scripts/sim.bash --vision --no-sim` alongside, or switch the Fixed Frame to `world`.

## Local - Run rbq_gazebo with RBQ Full Stack

Run the full RBQ stack (Motion / Vision / GUI) with the Gazebo simulator:

```bash
# Clone
git clone https://github.com/RainbowRobotics/RBQ.git
cd RBQ

# Install Dependency (first time only)
cd rbq_simulator/rbq_gazebo && bash scripts/debian-dep.bash 

# Build rbq_gazebo
bash scripts/build.bash

# Run the RBQ stack
cd ../../
bash scripts/sim.bash --vision --no-sim

# Run rbq_gazebo
cd rbq_simulator/rbq_gazebo
bash scripts/gazebo.bash                # quadruped

bash scripts/gazebo.bash --wheel        # wheeled RBQ
```

## Docker - Run rbq_gazebo with RBQ Full Stack

Run the full RBQ stack (Motion / Vision / GUI) with the Gazebo simulator in Docker:

```bash
# Clone
git clone https://github.com/RainbowRobotics/RBQ.git
cd RBQ

# Run the RBQ stack
bash scripts/sim.bash --vision --no-sim

# Run rbq_gazebo in Docker
cd rbq_simulator/rbq_gazebo
bash scripts/docker/run.bash                # quadruped

bash scripts/docker/run.bash --wheel        # wheeled RBQ
```

- Shares DDS domain 0 / `lo` with the host RBQ stack via `--network host`.

## External Node Access (SLAM / Custom ROS 2 Nodes)

The sensor_bridge publishes on **ROS 2 domain 0** over loopback (`lo`).
Run your node from `rbq_simulator/rbq_gazebo` with these env vars (must use CycloneDDS to match the bridge):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI=file://$(pwd)/ros2/src/sensor_bridge/config/cyclonedds_gazebo.xml
```

QoS is the `ros_gz_bridge` default (override per-topic in `ros2/src/sensor_bridge/config/bridge.yaml`):

| ROS 2 Topic | Type | QoS | Note |
|---|---|---|---|
| `/rbq/imu/IMU_state` | `sensor_msgs/msg/Imu` | Reliable · KeepLast 10 | 500 Hz |
| `/rbq/lidar/lidar_front/points` | `sensor_msgs/msg/PointCloud2` | Reliable · KeepLast 10 | `--livox` |
| `/rbq/lidar/lidar_rear/points` | `sensor_msgs/msg/PointCloud2` | Reliable · KeepLast 10 | `--livox` |
| `/rbq/lidar/lidar_ouster/points` | `sensor_msgs/msg/PointCloud2` | Reliable · KeepLast 10 | `--ouster` |
| `/tf` | `tf2_msgs/msg/TFMessage` | Reliable · KeepLast 10 | world → robot (ground truth) |
| `/joint_states` | `sensor_msgs/msg/JointState` | Reliable · KeepLast 10 | — |

`/tf` carries two independent chains. `world → robot` is the Gazebo **ground truth** pose,
published by the `rbq_gazebo` bridge. `local_world → base_link` is the **state estimator**
output (IMU + leg odometry, origin at boot/reset), published by RBQ SW. Ground truth is
deliberately attached to `robot` rather than `base_link` so the two never compete for the
same parent frame.
