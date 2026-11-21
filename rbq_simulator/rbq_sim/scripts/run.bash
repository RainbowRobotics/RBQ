#!/bin/bash

# Run the Isaac Sim simulator (rbq_sim/app.py) (+ RViz with --rviz, + RBQ core SW with --core)

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CORE=false
RVIZ=false
LIDAR=none
APP_ARGS=()
CORE_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        -i|--interface) APP_ARGS+=("$1" "$2"); CORE_ARGS+=(-i "$2"); shift ;;
        --vision)       APP_ARGS+=("$1"); CORE_ARGS+=("$1") ;;
        --livox)        [ "$LIDAR" = ouster ] && { echo "[sim] --livox and --ouster cannot be used together: one LiDAR at a time."; exit 1; }; LIDAR=livox;  APP_ARGS+=("$1"); CORE_ARGS+=(-p livox) ;;
        --ouster)       [ "$LIDAR" = livox ] && { echo "[sim] --livox and --ouster cannot be used together: one LiDAR at a time."; exit 1; }; LIDAR=ouster; APP_ARGS+=("$1"); CORE_ARGS+=(-p ouster) ;;
        --core)         CORE=true ;;
        --rviz)         RVIZ=true ;;
        --no-gui)       CORE_ARGS+=("$1") ;;
        --help)
            echo "Usage: bash rbq_simulator/rbq_sim/scripts/run.bash [--world <name>] [--headless] [--vision] [--livox|--ouster] [-i <iface>] [--rviz] [--core [--no-gui]]"
            echo "  --world     World from rbq_sim/worlds.py (default: rbq_test)."
            echo "  --headless  Run without the Isaac Sim window."
            echo "  --vision    Simulate the robot cameras (and, with --core, run the Vision stack)."
            echo "  --livox     Livox Mid-360 pair (front/rear), as rbq_mujoco --livox (default: no LiDAR)."
            echo "  --ouster    Ouster OS1 (rear deck), as rbq_gazebo --ouster."
            echo "  -i          CycloneDDS network interface (default: lo)."
            echo "  --rviz      Also open RViz (bundled sensor view, shared with rbq_mujoco / rbq_gazebo)."
            echo "  --core      Also run the RBQ core SW and open its web UI."
            echo "  --no-gui    With --core: do not open the web UI in the browser."
            exit 0 ;;
        *) APP_ARGS+=("$1") ;;
    esac
    shift
done

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

# --core: start the RBQ core SW with scripts/sim.bash --no-sim. It runs in its own terminal tabs and keeps
# running after the simulator exits: close those tabs to stop it.
CORE_PID=""
if $CORE; then
    REPO_DIR="$(cd "$SCRIPTS_DIR/../../.." && pwd)"
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

# --rviz: the sensor view from rbq_gazebo's sensor_bridge (sim:=isaac: Network topics, sim-time /clock).
LAUNCH_PID=""
if $RVIZ; then
    if ! $CORE && ! pgrep -x Network >/dev/null; then
        echo "[sim] --rviz: the joint states and the local_world -> base_link TF come from the RBQ core SW,"
        echo "      which is not running — the robot model stays unplaced. Add --core, or start it (scripts/sim.bash --no-sim)."
    fi
    ROS_WS="$SCRIPTS_DIR/../../rbq_gazebo/ros2"
    if [ -f "$ROS_WS/install/setup.bash" ]; then
        pkill -f 'rviz2.*__node:=rbq_sim_rviz' 2>/dev/null
        pkill -f 'robot_state_publisher.*__node:=rbq_sim_rsp' 2>/dev/null
        pkill -f 'tf2_ros/static_transform_publisher.*__node:=tf_' 2>/dev/null
        # Before the conda env is activated: ROS 2 Humble needs the system Python.
        ( source /opt/ros/humble/setup.bash
          source "$ROS_WS/install/setup.bash"
          exec setsid ros2 launch sensor_bridge sensor_bridge.launch.py sim:=isaac rviz:=true lidar:="$LIDAR" ) < /dev/null &
        LAUNCH_PID=$!
    else
        echo "[sim] --rviz: RViz package not built — run: bash rbq_simulator/rbq_gazebo/scripts/build.bash"
    fi
fi

cleanup() {
    [ -n "$CORE_PID" ] && kill -TERM -- -"$CORE_PID" 2>/dev/null
    if [ -n "$LAUNCH_PID" ]; then
        kill -TERM -- -"$LAUNCH_PID" 2>/dev/null
        # rviz2 can hang on TERM: wait 3 s, then KILL
        for _ in $(seq 30); do pgrep -f 'rviz2.*__node:=rbq_sim_rviz' >/dev/null || break; sleep 0.1; done
        pkill -KILL -f 'rviz2.*__node:=rbq_sim_rviz' 2>/dev/null
    fi
}
trap cleanup EXIT

source "$SCRIPTS_DIR/activate.bash"

# Accept the Omniverse EULA so the first run does not stop at a prompt.
export OMNI_KIT_ACCEPT_EULA=YES

# Keep DDS on loopback.
export CYCLONEDDS_URI="file://$SIM_DIR/cyclonedds_sim.xml"

echo -ne "\033]0;isaacsim\007"
echo "Running rbq_sim: python rbq_sim/app.py ${APP_ARGS[*]}"
cd "$SIM_DIR"
python rbq_sim/app.py "${APP_ARGS[@]}"
