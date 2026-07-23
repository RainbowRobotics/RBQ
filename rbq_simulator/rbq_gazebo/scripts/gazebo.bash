#!/bin/bash
# Run the gz simulator + gz->ROS sensor bridge + RViz 

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

RVIZ=true
ROS=true
LIDAR=none
GZ_ARGS=()
for a in "$@"; do
    case "$a" in
        --livox)   LIDAR=livox;  GZ_ARGS+=("$a") ;;
        --ouster)  LIDAR=ouster; GZ_ARGS+=("$a") ;;
        --no-rviz) RVIZ=false ;;
        --no-ros)  ROS=false; RVIZ=false ;;
        --help)
            echo "Usage: bash rbq_simulator/rbq_gazebo/scripts/gazebo.bash [--no-rviz] [--no-ros] [--headless] [--livox|--ouster] [gazebo args...]"
            echo "  Runs the Gazebo simulator + gz->ROS sensor bridge + RViz (default all three)."
            echo "  --livox     Livox-Mid360 LiDAR in the sim + bridge TFs (default: no LiDAR)."
            echo "  --ouster    Ouster LiDAR in the sim + bridge TFs."
            echo "  --no-rviz   Sensor bridge without RViz."
            echo "  --no-ros    Gazebo only (no bridge, no RViz)."
            echo "  Extra args (e.g. --world empty) pass through to Gazebo."
            exit 0 ;;
        *) GZ_ARGS+=("$a") ;;
    esac
done

if [ ! -x "$PKG_DIR/bin/rbq_gazebo" ]; then
    echo "[sim] bin/rbq_gazebo not found — build it first: bash $SCRIPT_DIR/build.bash"
    exit 1
fi

cd "$PKG_DIR" || exit 1
# Kill a leftover sim
pkill -x rbq_gazebo 2>/dev/null; 
pkill -f 'ign gazebo -g' 2>/dev/null; sleep 0.3
# Enable lo multicast for DDS discovery
if ! ip link show lo | grep -qw MULTICAST; then
    if [ "$(id -u)" -eq 0 ]; then 
        ip link set lo multicast on 2>/dev/null || true
    else 
        sudo ip link set lo multicast on 2>/dev/null || true; 
    fi
fi

unset CYCLONEDDS_URI

BRIDGE_PID=""
cleanup() {
    trap - INT TERM EXIT
    [ -n "$BRIDGE_PID" ] && kill -TERM "$BRIDGE_PID" 2>/dev/null
    pkill -x rbq_gazebo 2>/dev/null
    exit 0
}
trap cleanup INT TERM EXIT

# Sensor bridge (+ RViz)
if $ROS; then
    if [ -f "$PKG_DIR/ros2/install/setup.bash" ]; then
        ( source /opt/ros/humble/setup.bash
          source "$PKG_DIR/ros2/install/setup.bash"
          exec ros2 launch sensor_bridge sensor_bridge.launch.py rviz:="$RVIZ" lidar:="$LIDAR" ) &
        BRIDGE_PID=$!
    else
        echo "[sim] sensor_bridge not built (ros2/install missing) — run: bash $SCRIPT_DIR/build.bash"
    fi
fi

# Gazebo simulator
cd "$PKG_DIR/bin" && ./rbq_gazebo --partition rbq "${GZ_ARGS[@]}"
