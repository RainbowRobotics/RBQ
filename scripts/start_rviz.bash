#!/bin/bash

APP_NAME="rbq_description"
ROS2_DIR="rbq_sdk/ros2"
IFACE="lo"

print_help() {
    echo "Usage: bash scripts/start_rviz.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                  Display this help message and exit."
    echo "  -i, --interface <name>  CycloneDDS network interface (forwarded to Network). DEFAULT lo"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        -i|--interface) IFACE="$2"; shift 2 ;;
        *) echo "Unknown argument: $1"; print_help; sleep 10; exit 1 ;;
    esac
done

if pgrep -x $APP_NAME > /dev/null; then
    echo "$APP_NAME is already running. Please close it before starting a new instance."
    sleep 10
    exit 1
fi

# Build the CycloneDDS config inline from the --interface arg (default lo) instead
# of a static file, so the DDS NIC follows the -i flag.
export CYCLONEDDS_URI="<CycloneDDS>
  <Domain Id=\"any\">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine=\"false\" name=\"${IFACE}\" priority=\"default\" multicast=\"default\" />
      </Interfaces>
      <AllowMulticast>default</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "[start_rviz] CycloneDDS interface: ${IFACE}"
source /opt/ros/humble/setup.bash

cd $ROS2_DIR
colcon build --symlink-install
source install/setup.bash
echo -ne "\033]0;$APP_NAME\007"

while true; do
    pid=$(pgrep -x "$APP_NAME")
    if [ -z "$pid" ]; then
        ros2 launch rbq_description description.launch.py
    fi
    sleep 2
done
