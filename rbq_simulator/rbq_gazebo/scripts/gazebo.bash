#!/bin/bash
# Run the Gazebo simulator + ROS 2 sensor bridge (+ RViz with --rviz, + RBQ core SW with --core)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

RVIZ=false
ROS=true
CORE=false
LIDAR=none
VARIANT=rbq10
GZ_ARGS=()
CORE_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --wheel)   VARIANT=rbq10_wheel; GZ_ARGS+=("$1"); CORE_ARGS+=(-r wheel) ;;
        --livox)   [ "$LIDAR" = ouster ] && { echo "[sim] --livox and --ouster cannot be used together: one LiDAR at a time."; exit 1; }; LIDAR=livox;  GZ_ARGS+=("$1"); CORE_ARGS+=(-p livox) ;;
        --ouster)  [ "$LIDAR" = livox ] && { echo "[sim] --livox and --ouster cannot be used together: one LiDAR at a time."; exit 1; }; LIDAR=ouster; GZ_ARGS+=("$1"); CORE_ARGS+=(-p ouster) ;;
        -i|--interface) GZ_ARGS+=("$1" "$2"); CORE_ARGS+=(-i "$2"); shift ;;
        --rviz)    RVIZ=true ;;
        --no-ros)  ROS=false; RVIZ=false ;;
        --core)    CORE=true ;;
        --vision|--no-gui) CORE_ARGS+=("$1") ;;
        --help)
            echo "Usage: bash rbq_simulator/rbq_gazebo/scripts/gazebo.bash [--core [--vision] [--no-gui]] [--wheel] [--rviz] [--no-ros] [--headless] [--livox|--ouster] [gazebo args...]"
            echo "  Runs the Gazebo simulator + gz->ROS sensor bridge; RViz only with --rviz."
            echo "  --core      Also run the RBQ core SW and open its web UI."
            echo "              Robot, LiDAR and -i follow this command."
            echo "  --vision    With --core: also run the Vision stack."
            echo "  --no-gui    With --core: do not open the web UI in the browser."
            echo "  --wheel     wheeled RBQ (12 leg joints + 4 wheels); default is the quadruped."
            echo "  --livox     Livox-Mid360 LiDAR in the sim + bridge TFs (default: no LiDAR)."
            echo "  --ouster    Ouster LiDAR in the sim + bridge TFs."
            echo "  --rviz      Also open RViz (bundled sensor view, shared with rbq_mujoco)."
            echo "  --no-ros    Gazebo only (no bridge, no RViz)."
            echo "  Extra args (e.g. --world empty) pass through to Gazebo."
            exit 0 ;;
        *) GZ_ARGS+=("$1") ;;
    esac
    shift
done

if [ ! -x "$PKG_DIR/bin/rbq_gazebo" ]; then
    echo "[sim] bin/rbq_gazebo not found — build it first: bash $SCRIPT_DIR/build.bash"
    exit 1
fi

cd "$PKG_DIR" || exit 1
# Stop a leftover sim, bridge and RViz from an earlier run (matched by their own node names).
pkill -x rbq_gazebo 2>/dev/null
pkill -f 'ign gazebo -g' 2>/dev/null
pkill -f 'ros2 launch sensor_bridge' 2>/dev/null
pkill -f 'rviz2.*__node:=rbq_sim_rviz' 2>/dev/null
pkill -f 'robot_state_publisher.*__node:=rbq_sim_rsp' 2>/dev/null
pkill -f 'ros_gz_bridge/parameter_bridge' 2>/dev/null
pkill -f 'tf2_ros/static_transform_publisher.*__node:=tf_' 2>/dev/null
sleep 0.3
# DDS discovery on lo needs multicast: without it the simulator, Motion and RViz never find each other.
if ! ip link show lo | grep -qw MULTICAST; then
    if [ "$(id -u)" -eq 0 ]; then
        ip link set lo multicast on 2>/dev/null
    else
        sudo ip link set lo multicast on 2>/dev/null
    fi
    if ! ip link show lo | grep -qw MULTICAST; then
        echo "[sim] WARNING: lo has no multicast, so the simulator cannot reach Motion over DDS."
        echo "      Run: sudo ip link set lo multicast on   (then restart Motion if it is already running)"
    fi
fi

unset CYCLONEDDS_URI

BRIDGE_PID=""
CORE_PID=""
cleanup() {
    trap - INT TERM EXIT
    [ -n "$BRIDGE_PID" ] && kill -TERM "$BRIDGE_PID" 2>/dev/null
    [ -n "$CORE_PID" ] && kill -TERM -- -"$CORE_PID" 2>/dev/null
    pkill -x rbq_gazebo 2>/dev/null
    pkill -f 'ros2 launch sensor_bridge' 2>/dev/null
    pkill -f 'rviz2.*__node:=rbq_sim_rviz' 2>/dev/null
    pkill -f 'robot_state_publisher.*__node:=rbq_sim_rsp' 2>/dev/null
    pkill -f 'ros_gz_bridge/parameter_bridge' 2>/dev/null
    pkill -f 'tf2_ros/static_transform_publisher.*__node:=tf_' 2>/dev/null
    # rviz2 can hang on TERM: wait 3 s, then KILL
    for _ in $(seq 30); do pgrep -f 'rviz2.*__node:=rbq_sim_rviz' >/dev/null || break; sleep 0.1; done
    pkill -KILL -f 'rviz2.*__node:=rbq_sim_rviz' 2>/dev/null
    exit 0
}
trap cleanup INT TERM EXIT

# --core: start the RBQ core SW with scripts/sim.bash --no-sim. It runs in its own terminal tabs and keeps
# running after the simulator exits: close those tabs to stop it.
if $CORE; then
    REPO_DIR="$(cd "$PKG_DIR/../.." && pwd)"
    # The repo root is found from this script's place, so the simulator must sit inside the RBQ repo.
    if ! grep -qs -- '--no-sim' "$REPO_DIR/scripts/sim.bash"; then
        echo "[sim] --core: no RBQ scripts/sim.bash in $REPO_DIR"
        echo "      --core works only inside the RBQ repo (RBQ/rbq_simulator/...)."
        echo "      Start the core SW yourself instead: bash scripts/sim.bash --no-sim"
        exit 1
    fi
    echo "[sim] --core: scripts/sim.bash --no-sim ${CORE_ARGS[*]}"
    ( cd "$REPO_DIR" && exec setsid bash -c 'bash scripts/sim.bash --no-sim "$@" ||
        echo "[sim] --core: the RBQ core SW failed (exit $?); the simulator keeps running without it."' _ "${CORE_ARGS[@]}" ) < /dev/null &
    CORE_PID=$!
fi

# Sensor bridge (+ RViz)
if $ROS; then
    if [ -f "$PKG_DIR/ros2/install/setup.bash" ]; then
        ( source /opt/ros/humble/setup.bash
          source "$PKG_DIR/ros2/install/setup.bash"
          exec ros2 launch sensor_bridge sensor_bridge.launch.py rviz:="$RVIZ" lidar:="$LIDAR" variant:="$VARIANT" ) &
        BRIDGE_PID=$!
    else
        echo "[sim] sensor_bridge not built (ros2/install missing) — run: bash $SCRIPT_DIR/build.bash"
    fi
fi

# Gazebo simulator
cd "$PKG_DIR/bin" && ./rbq_gazebo --partition rbq "${GZ_ARGS[@]}"
