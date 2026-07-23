#!/bin/bash
set -e

MAKE_JOBS=$(nproc)
USE_CACHE=true
BUILD_BRIDGE=true
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAIN_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$MAIN_DIR/build"
BIN_DIR="$MAIN_DIR/bin"
ROS_WS="$MAIN_DIR/ros2"    # colcon workspace for the gz->ROS sensor bridge (src/sensor_bridge tracked)

print_help() {
    echo "Usage: bash rbq_simulator/rbq_gazebo/scripts/build.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                  Display this help message and exit."
    echo "  --no-cache              Clean build directory and bypass cache."
    echo "  --no-bridge             Skip the gz->ROS sensor bridge (built by default via colcon)."
    echo "  -j [number]             Specify number of CPUs for parallel build."
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --help) print_help; exit 0 ;;
        --no-cache)     USE_CACHE=false; shift ;;
        --bridge)       BUILD_BRIDGE=true;  shift ;;
        --no-bridge)    BUILD_BRIDGE=false; shift ;;
        -j) shift;      MAKE_JOBS="${1:-$(nproc)}"; shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

if ! $USE_CACHE; then
    echo "[INFO] Cache bypassed. Cleaning build dir + Gazebo artifacts only..."
    rm -rf "$BUILD_DIR"
    rm -f "$BIN_DIR/rbq_gazebo" "$BIN_DIR/librbq_gazebo_system.so"
fi

if [ -f "$BUILD_DIR/CMakeCache.txt" ] && \
   { ! grep -qF "$BUILD_DIR" "$BUILD_DIR/CMakeCache.txt" || \
     ! grep -qxF "CMAKE_INSTALL_PREFIX:PATH=$MAIN_DIR" "$BUILD_DIR/CMakeCache.txt" || \
     ! grep -q 'CMAKE_GENERATOR:INTERNAL=Unix Makefiles' "$BUILD_DIR/CMakeCache.txt" || \
     [ ! -f "$BUILD_DIR/Makefile" ]; }; then
    echo "[INFO] build/ cache is stale, incomplete, or has a different install prefix; wiping for a clean reconfigure."
    rm -rf "$BUILD_DIR"
fi

if ! $USE_CACHE || [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    GENERATOR="Unix Makefiles"

    echo "[INFO] Using generator: $GENERATOR"
    echo "[INFO] Configuring CMake..."

    CMAKE_OPTIONS=()
    CMAKE_OPTIONS+=("-D CMAKE_BUILD_TYPE=Release")
    CMAKE_OPTIONS+=("-D BUILD_SHARED_LIBS=OFF")
    CMAKE_OPTIONS+=("-D CMAKE_INSTALL_PREFIX=$MAIN_DIR")
    
    cmake -S "$MAIN_DIR" -B "$BUILD_DIR" -G "$GENERATOR" "${CMAKE_OPTIONS[@]}" || {
        echo "[ERROR] CMake configuration failed!"
        exit 1
    }
fi

echo "[INFO] Building the project..."
cmake --build "$BUILD_DIR" -j"$MAKE_JOBS" || {
    echo "[ERROR] CMake build failed!"
    exit 1
}
echo "✅ Build complete."
echo "[INFO] Installing the project..."
cmake --install "$BUILD_DIR" || {
    echo "[ERROR] CMake install failed!"
    exit 1
}
echo "✅ Install complete."

if $BUILD_BRIDGE; then
    echo "[INFO] Building the gz->ROS sensor bridge (sensor_bridge)..."
    if [ -f /opt/ros/humble/setup.bash ]; then
        set +e
        source /opt/ros/humble/setup.bash
        ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1 || \
            echo "[WARN] ros_gz_bridge not installed: sudo apt install ros-humble-ros-gz-bridge"
        BRIDGE_BUILD="$ROS_WS/build/sensor_bridge"
        if [ -f "$BRIDGE_BUILD/CMakeCache.txt" ] && ! grep -qF "$BRIDGE_BUILD" "$BRIDGE_BUILD/CMakeCache.txt"; then
            echo "[INFO] sensor_bridge build/ configured for a different path; wiping."
            rm -rf "$BRIDGE_BUILD" "$ROS_WS/install/sensor_bridge"
        fi
        # Workspace is rbq_gazebo/ros2 (src/sensor_bridge is tracked); colcon builds in place.
        ( cd "$ROS_WS" && colcon build --symlink-install --cmake-force-configure )
        rc=$?
        set -e
        if [ $rc -eq 0 ]; then
            echo "✅ Sensor bridge built (source $ROS_WS/install/setup.bash to use)."
        else
            echo "[ERROR] sensor bridge colcon build failed!"; exit 1
        fi
    else
        echo "[WARN] /opt/ros/humble not found — skipping bridge build (install ROS2 Humble)."
    fi
fi

