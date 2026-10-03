# Migration: `ros2/rbq_driver` → in-process DDS bridge

The standalone **`rbq_driver`** ROS 2 node has been removed.
Its responsibilities are now built directly into the on-robot core software:

| Old (`ros2/rbq_driver`)                      | Network/DdsBridge, (core-sw, always running on the robot)  |
|----------------------------------------------|--------------------------------------------                |
| `Publisher` / `Subscriber` (state + commands)| `Network/DdsBridge`                                        |
| `VisionPublisher` (camera images out)        | `HAL/Sensor` (per-sensor publishers)                       |
| custom `rbq_msgs`                            | `rbq_sdk/ros2/src/rbq_msgs` (definitions unchanged)        |

This document maps the old ROS 2 interface to the new one so existing integrations can be ported.
It does **not** describe a code change you run — it records what moved where.

## Why it changed

- `rbq_driver` was a separate `rclcpp` process that reached the robot over **TCP/UDP**.
  The bridge now runs **inside the `Network` process on the robot** and talks to the core-sw through **shared memory**.
- Transport is **bare CycloneDDS** using ROS 2-compatible IDLs, so ROS 2 tooling (`ros2 topic`, `rviz2`, `rqt`)
  still interoperates over the network — you no longer build/run a bridge node yourself.
- This change will reduce the sensor/cmd/ref data transfer delays over network.

## Topic naming

The bridge publishes/subscribes on DDS **wire names** prefixed with `rt/`
(the ROS 2 topic-mangling prefix). In ROS 2 tooling you address them **without**
the prefix:

| DDS wire name (in source) | ROS 2 topic (what you type) |
|---------------------------|-----------------------------|
| `rt/rbq/imu`              | `/rbq/imu`                  |
| `rt/rbq/cmd/high_level`   | `/rbq/cmd/high_level`       |

Note the namespace also flattened: old topics lived under
`rbq/<subsystem>/...` (e.g. `rbq/motion/...`, `rbq/stateEstimation/...`); new topics live under a flat `rbq/...`.

## State topics (robot → ROS 2)

| Old topic                            | New ROS 2 topic        | Type                            | Note        |
|--------------------------------------|------------------------|---------------------------------|-------------|
| -                                    | `/rbq/joy`             | `sensor_msgs/Joy`               | Done        |
| `rbq/imu/IMU_state`                  | `/rbq/imu`             | `sensor_msgs/Imu`               | Done        |
| `rbq/joint/joint_states`             | `/rbq/joint_states`    | `sensor_msgs/JointState`        | Done        |
| `rbq/stateEstimation/odometry`       | `/rbq/odometry`        | `nav_msgs/Odometry`             | Done        |
| -                                    | `/rbq/leg_joint`       | `rbq_msgs/LegJointInfo`         | Done        |
| `rbq/stateEstimation/footStates`     | `/rbq/foot_states`     | `rbq_msgs/FootStates`           | Done        |
| `rbq/powerControl/battery_status`    | `/rbq/battery`         | `rbq_msgs/BatteryState`         | Done        |
|--------------------------------------|------------------------|---------------------------------|-------------|
| `rbq/status/robot_status`            | `/rbq/robot_status`    | `rbq_msgs/RobotStatus`          | Check       |
|--------------------------------------|------------------------|---------------------------------|-------------|
| -                                    | `/api/sport/request`   | `api/msg/Request`               | Done        |
| -                                    | `/rbq/ref/motion/_0`   | `rbq_msgs/MotionRef`            | Done        |
| -                                    | `/rbq/_user_command`   | `rbq_msgs/UserCommand`          | Done        |
|--------------------------------------|------------------------|---------------------------------|-------------|
| `rbq/joint/joint_status`             | (use `/rbq/leg_joint`) | -                               | Deprecated  |
| `rbq/status/comm_connected`          | -                      | -                               | Deprecated  |
| `rbq/stateEstimation/robotVelocity`  | (in `/rbq/odometry`)   | `geometry_msgs/VelocityStamped` | Deprecated  |

`/rbq/joy` is published only while a client (gamepad-owner) is connected and actively streaming input to `Robot`;

The per-app joint-ownership/ref and sim topics now ship matching `rbq_msgs/*.msg`
so ROS 2 tooling can decode them, but they remain internal: `_<i>` is the app
index (`i` ≥ 20) and `/rbq/_sim` is sim-only sensor injection. Not intended for
general clients.

## Command topics (ROS 2 → robot)

| Old topic                                      | New ROS 2 topic                 | Type                        | Note       |
|------------------------------------------------|---------------------------------|-----------------------------|------------|
| `rbq/motion/emergency`                         | `/rbq/cmd/emergency`            | `std_msgs/Bool`             | Done       |
| `rbq/motion/autoStart`                         | `/rbq/cmd/auto_start`           | `std_msgs/Bool`             | Done       |
| `rbq/motion/switchControlMode`                 | `/rbq/cmd/switch_control_mode`  | `std_msgs/Bool`             | Done       |
| `rbq/motion/cmd_highLevel`                     | `/rbq/cmd/high_level`           | `rbq_msgs/HighLevelCommand` | Done       |
| `rbq/motion/docking` + `docking_mode`          | `/rbq/cmd/dock`                 | `std_msgs/Int8MultiArray`   | Done       |
| `rbq/motion/switchGait`                        | `/rbq/cmd/switch_gait`          | `std_msgs/Int8`             | Done       |
| `rbq/motion/cmd_navigateTo` (`PoseStamped`)    | `/rbq/cmd/navigate_to`          | `geometry_msgs/Pose2D`      | Done       |
| `rbq/motion/additional_payload_params`         | `/rbq/ref/payload_params`       | `std_msgs/Float64MultiArray`| Done       |
| `rbq/powerControl/setPortState`                | `/rbq/cmd/switch_power`         | `std_msgs/Int8MultiArray`   | Done       |
|------------------------------------------------|---------------------------------|-----------------------------|------------|
| -                                              | `/rbq/cmd/motion/owner/_<i>`    | `rbq_msgs/JointOwnershipCmd`| Done       |
| -                                              | `/rbq/ref/motion/_<i>`          | `rbq_msgs/MotionRef`        | Done       |
| -                                              | `/rbq/_sim`                     | `rbq_msgs/SimInfo`          | Done       |
| `rbq/ptzCamera/setPanTiltZoom`                 | `/rbq/cmd/ptz`                  | `std_msgs/Float64MultiArray`| Done       |
|------------------------------------------------|---------------------------------|-----------------------------|------------|
| `rbq/motion/canCheck`                          | -                               | -                           | Deprecated |
| `rbq/motion/findHome`                          | -                               | -                           | Deprecated |
| `rbq/motion/staticLock`                        | -                               | -                           | Deprecated |
| `rbq/motion/staticReady`                       | -                               | -                           | Deprecated |
| `rbq/motion/staticGround`                      | -                               | -                           | Deprecated |
| `rbq/motion/recoveryErrorClear`                | -                               | -                           | Deprecated |
| `rbq/motion/recoveryFlex`                      | -                               | -                           | Deprecated |
| `rbq/stateEstimation/comEstimationCompensation`| -                               | -                           | Deprecated |
| `joy` (inbound command)                        | -                               | -                           | Deprecated |
| `rbq/motion/setBodyHeight`                     | -                               | -                           | Deprecated |
| `rbq/motion/setMaxSpeed`                       | -                               | -                           | Deprecated |
| `rbq/motion/setFootHeight`                     | -                               | -                           | Deprecated |
|------------------------------------------------|---------------------------------|-----------------------------|------------|

`/rbq/cmd/ptz` (`[pan(rad), tilt(rad), zoom(x)]`) is subscribed **directly by the `Ptz` vision app**, not the bridge —
it only acts when `Ptz` is running.

Unitree-A2-compatible RPC channel (`/api/sport/request` → `/api/sport/response`) handled by the bridge. rbq C++/Python
clients use the `SportClient` (bare DDS); ROS 2 clients use the `api` package (`api/msg/Request` / `api/msg/Response`),
which mangles to the bridge's `api::msg::dds_::Request_` wire type. The `binary` field is `uint8[]` (Request) /
`int8[]` (Response) to mirror Unitree's `unitree_api` so stock `rmw_cyclonedds` clients also match the channel.

## Vision topics (robot → ROS 2)

Published per sensor by `HAL/Sensor`. Sensors are now addressed by numeric id
instead of names — the `<id>` placeholder below maps as:

| Old sensor name         | New id       | Streams              |
|-------------------------|--------------|----------------------|
| `sensor_bottom_0`       | `sensor_0`   | `ir`, `depth`        |
| `sensor_bottom_1`       | `sensor_1`   | `ir`, `depth`        |
| `sensor_bottom_2`       | `sensor_2`   | `ir`, `depth`        |
| `sensor_bottom_3`       | `sensor_3`   | `ir`, `depth`        |
| `sensor_front`          | `sensor_4`   | `rgb`, `ir`, `depth` |
| `sensor_rear`           | `sensor_5`   | `rgb`, `ir`, `depth` |

| Old topic                                  | New ROS 2 topic                             | Type                          | Note       |
|--------------------------------------------|---------------------------------------------|-------------------------------|------------|
| `rbq/vision/<sensor>/rgb/compressed`       | `/rbq/vision/<id>/rgb/compressed`           | `sensor_msgs/CompressedImage` | Done       |
| `rbq/vision/<sensor>/rgb/camera_info`      | `/rbq/vision/<id>/rgb/camera_info`          | `sensor_msgs/CameraInfo`      | Done       |
| `rbq/vision/<sensor>/ir/compressed`        | `/rbq/vision/<id>/ir/compressed`            | `sensor_msgs/CompressedImage` | Done       |
| `rbq/vision/<sensor>/ir/camera_info`       | `/rbq/vision/<id>/ir/camera_info`           | `sensor_msgs/CameraInfo`      | Done       |
| `rbq/vision/<sensor>/depth/compressed`     | `/rbq/vision/<id>/depth/compressed`         | `sensor_msgs/CompressedImage` | Done       |
| `rbq/vision/<sensor>/depth/camera_info`    | `/rbq/vision/<id>/depth/camera_info`        | `sensor_msgs/CameraInfo`      | Done       |
| static TF (`/tf_static`, `tf2`)            | `/rbq/vision/<id>/rgb/tf`                   | `geometry_msgs/PoseStamped`   | Done       |
| static TF (`/tf_static`, `tf2`)            | `/rbq/vision/<id>/ir/tf`                    | `geometry_msgs/PoseStamped`   | Done       |
| static TF (`/tf_static`, `tf2`)            | `/rbq/vision/<id>/depth/tf`                 | `geometry_msgs/PoseStamped`   | Done       |
|--------------------------------------------|---------------------------------------------|-------------------------------|------------|
| `rbq/vision/<sensor>/rgb` (raw)            | -                                           | -                             | Deprecated |
| `rbq/vision/<sensor>/ir` (raw)             | -                                           | -                             | Deprecated |
| `rbq/vision/<sensor>/depth` (raw)          | -                                           | -                             | Deprecated |
| `VisionSubscriber` (ROS 2 images → IPC)    | -                                           | -                             | Deprecated |

Where `<sensor>` is the old name and `<id>` the new numeric id from the mapping above
(e.g. `rbq/vision/sensor_front/rgb/compressed` → `/rbq/vision/sensor_4/rgb/compressed`).

TF shape changed: instead of a single `tf2` `/tf_static` tree, each camera stream
now publishes its own `base → camera` extrinsics as a `geometry_msgs/PoseStamped`
on `.../tf` (with `frame_id = base`).

## Building the new workspace

```bash
cd rbq_sdk/ros2
colcon build
source install/setup.bash
```

Packages:
`rbq_msgs` (message defs),
`api` (Unitree-compatible sport RPC `Request`/`Response` msgs),
`rbq_examples`,
`rbq_description` (URDF/meshes/rviz),
`rbq_rviz_plugins`.
