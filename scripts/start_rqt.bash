#!/bin/bash

APP_NAME="rqt"
IFACE="lo"

while [[ $# -gt 0 ]]; do
    case $1 in
        -i|--interface) IFACE="$2"; shift 2 ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

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
source /opt/ros/humble/setup.bash
source rbq_sdk/ros2/install/setup.bash

echo -ne "\033]0;$APP_NAME\007"

while true; do
    pid=$(pgrep -x "$APP_NAME")
    if [ -z "$pid" ]; then
        echo "Starting $APP_NAME..."
        $APP_NAME
    fi
    sleep 2
done
