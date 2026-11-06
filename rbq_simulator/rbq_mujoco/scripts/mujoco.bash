#!/bin/bash
# Run the MuJoCo simulator (+ RViz with --rviz, the same view rbq_gazebo uses)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
# RViz launch + config + URDFs live in rbq_gazebo's colcon package (sensor_bridge); MuJoCo reuses it.
ROS_WS="$(cd "$PKG_DIR/../rbq_gazebo/ros2" && pwd)"

RVIZ=false
LIDAR=none
VARIANT=rbq10
SIM_ARGS=()
for a in "$@"; do
    case "$a" in
        --wheel)  VARIANT=rbq10_wheel; SIM_ARGS+=("$a") ;;
        --livox)  LIDAR=livox;  SIM_ARGS+=("$a") ;;
        --ouster) LIDAR=ouster; SIM_ARGS+=("$a") ;;
        --rviz)   RVIZ=true ;;
        --help)
            echo "Usage: bash rbq_simulator/rbq_mujoco/scripts/mujoco.bash [--wheel] [--rviz] [--livox|--ouster] [mujoco args...]"
            echo "  Runs the MuJoCo simulator; RViz only with --rviz."
            echo "  --wheel     wheeled RBQ (12 leg joints + 4 wheels); default is the quadruped."
            echo "  --livox     Livox-Mid360 pair (front/rear)."
            echo "  --ouster    Ouster housing (not simulated yet)."
            echo "  --rviz      Also open RViz (bundled sensor view, shared with rbq_gazebo)."
            echo "  Extra args (e.g. -p <model.xml>) pass through to rbq_mujoco."
            exit 0 ;;
        *) SIM_ARGS+=("$a") ;;
    esac
done

if [ ! -x "$PKG_DIR/bin/rbq_mujoco" ]; then
    echo "[sim] bin/rbq_mujoco not found — build it first: bash $SCRIPT_DIR/build.bash"
    exit 1
fi

if ! $RVIZ; then
    cd "$PKG_DIR/bin" && exec ./rbq_mujoco "${SIM_ARGS[@]}"
fi

if [ ! -f "$ROS_WS/install/setup.bash" ]; then
    echo "[sim] RViz package not built — run once:"
    echo "      source /opt/ros/humble/setup.bash && (cd $ROS_WS && colcon build --symlink-install)"
    exit 1
fi

# Kill a leftover RViz launch, matched by its own node names (same as rbq_gazebo/scripts/gazebo.bash).
pkill -f 'ros2 launch sensor_bridge' 2>/dev/null
pkill -f 'rviz2.*__node:=rbq_sim_rviz' 2>/dev/null
pkill -f 'robot_state_publisher.*__node:=rbq_sim_rsp' 2>/dev/null
pkill -f 'tf2_ros/static_transform_publisher.*__node:=tf_' 2>/dev/null
sleep 0.3
# Enable lo multicast for DDS discovery
if ! ip link show lo | grep -qw MULTICAST; then
    if [ "$(id -u)" -eq 0 ]; then
        ip link set lo multicast on 2>/dev/null || true
    else
        sudo ip link set lo multicast on 2>/dev/null || true
    fi
fi

LAUNCH_PID=""
cleanup() {
    trap - INT TERM EXIT
    [ -n "$LAUNCH_PID" ] && kill -TERM "$LAUNCH_PID" 2>/dev/null
    pkill -f 'ros2 launch sensor_bridge' 2>/dev/null
    pkill -f 'rviz2.*__node:=rbq_sim_rviz' 2>/dev/null
    pkill -f 'robot_state_publisher.*__node:=rbq_sim_rsp' 2>/dev/null
    pkill -f 'tf2_ros/static_transform_publisher.*__node:=tf_' 2>/dev/null
    # rviz2 can take launch's SIGINT, drop its handlers and still hang, swallowing the TERM above
    for _ in $(seq 30); do pgrep -f 'rviz2.*__node:=rbq_sim_rviz' >/dev/null || break; sleep 0.1; done
    pkill -KILL -f 'rviz2.*__node:=rbq_sim_rviz' 2>/dev/null
    exit 0
}
trap cleanup INT TERM EXIT

( source /opt/ros/humble/setup.bash
  source "$ROS_WS/install/setup.bash"
  exec ros2 launch sensor_bridge sensor_bridge.launch.py sim:=mujoco rviz:=true lidar:="$LIDAR" variant:="$VARIANT" ) &
LAUNCH_PID=$!

# MuJoCo simulator
cd "$PKG_DIR/bin" && ./rbq_mujoco "${SIM_ARGS[@]}"
