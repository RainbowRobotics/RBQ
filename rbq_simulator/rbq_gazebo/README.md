# rbq_gazebo

- Ignition Gazebo (Fortress) simulator for the RBQ quadruped Robot.

## Layout

- `src/` — simulator sources → `bin/rbq_gazebo`
- `resources/` — worlds, robot/sensor URDFs, meshes
- `ros2/src/sensor_bridge/` — colcon package: bridge launch, RViz config, DDS xml
- `scripts/` — build / run / docker

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
bash scripts/gazebo.bash
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
bash scripts/docker/run.bash
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
| `/tf` | `tf2_msgs/msg/TFMessage` | Reliable · KeepLast 10 | world → base_link |
| `/joint_states` | `sensor_msgs/msg/JointState` | Reliable · KeepLast 10 | — |
