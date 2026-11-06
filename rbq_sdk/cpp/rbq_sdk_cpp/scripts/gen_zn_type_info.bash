#!/usr/bin/env bash
# Regenerate include/zn/TypeInfoTable.hpp from ROS 2 Jazzy: builds rbq_msgs and api (rbq_sdk/ros2/src) in a
# ros:jazzy container and reads the type hashes rosidl computes. Run from the repository root:
#   bash rbq_sdk/cpp/rbq_sdk_cpp/scripts/gen_zn_type_info.bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../../.." && pwd)"
SDK="rbq_sdk/cpp/rbq_sdk_cpp"
IMAGE="ros:jazzy-ros-base"

docker run --rm --user 0 -v "$ROOT:/repo" "$IMAGE" bash -c "
    set -e
    apt-get update -qq >/dev/null
    apt-get install -y -qq ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-nav-msgs ros-jazzy-tf2-msgs \
        ros-jazzy-diagnostic-msgs ros-jazzy-common-interfaces >/dev/null
    source /opt/ros/jazzy/setup.bash
    mkdir -p /ws/src && cp -r /repo/rbq_sdk/ros2/src/rbq_msgs /repo/rbq_sdk/ros2/src/api /ws/src/
    cd /ws && colcon build --packages-select rbq_msgs api >/dev/null
    python3 /repo/$SDK/scripts/gen_zn_type_info.py /repo/$SDK/idl /repo/$SDK/include/zn/TypeInfoTable.hpp \
        /opt/ros/jazzy/share /ws/install/rbq_msgs/share /ws/install/api/share
    chown $(id -u):$(id -g) /repo/$SDK/include/zn/TypeInfoTable.hpp
"
