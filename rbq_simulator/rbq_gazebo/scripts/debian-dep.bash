#!/usr/bin/env bash
set -e

# Runtime: meta package + CLI tools + GL/Qt5/OpenCV/xcb + terminal for the GUI tab.
RUNTIME_PKGS=(
    ignition-fortress
    ignition-tools
    libqt5core5a
    libopencv-core4.5d libopencv-imgproc4.5d
    libgl1 libglx0 libglvnd0 libegl1 libgles2 libgbm1 libgl1-mesa-dri
    libxcb1 libxcb-glx0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1
    libxcb-randr0 libxcb-render0 libxcb-render-util0 libxcb-shape0 libxcb-shm0
    libxcb-sync1 libxcb-util1 libxcb-xfixes0 libxcb-xinerama0 libxcb-xkb1
    libxkbcommon0 libxkbcommon-x11-0
    iproute2 gnome-terminal dbus-x11 dconf-cli locales
    cmake make build-essential ninja-build pkg-config
)

# Dev headers: only needed to compile the Gazebo simulator binary.
DEV_PKGS=(
    libignition-gazebo6-dev
    libignition-transport11-dev
    libignition-msgs8-dev
    libignition-plugin-dev
    libignition-sensors6-dev
    libignition-rendering6-dev
    libignition-rendering6-ogre2
    libsdformat12-dev
    libeigen3-dev
    libopencv-dev
    qtbase5-dev
)

PKGS=("${RUNTIME_PKGS[@]}" "${DEV_PKGS[@]}")
$WITH_DEV && PKGS+=("${DEV_PKGS[@]}")

all_installed() {
    for p in "${PKGS[@]}"; do
        dpkg -s "$p" >/dev/null 2>&1 || return 1
    done
    return 0
}

# --- Core Gazebo deps (osrfoundation repo) ---
if all_installed; then
    echo "✅ Gazebo (Ignition Fortress) core deps already installed."
else
    # Register the osrfoundation apt repo (idempotent).
    if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
        echo "[INFO] Registering the osrfoundation apt repository..."
        apt-get update
        apt-get install -y --no-install-recommends wget gnupg lsb-release ca-certificates
        mkdir -p /usr/share/keyrings
        wget -qO - https://packages.osrfoundation.org/gazebo.gpg \
            > /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
            > /etc/apt/sources.list.d/gazebo-stable.list
    fi
    echo "[INFO] Installing Gazebo dependencies (runtime + dev)..."
    apt-get update
    apt-get install -y --no-install-recommends "${PKGS[@]}"
    echo "✅ Gazebo (Ignition Fortress) dependencies installed (runtime + dev)."
fi

# --- Register the ROS2 Humble apt repo (idempotent) ---
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    if [ "$(id -u)" -ne 0 ]; then
        echo "[WARN] ROS2 Humble apt repo not registered — re-run as root to add it: sudo bash rbq_simulator/rbq_gazebo/scripts/debian-dep.bash"
    else
        echo "[INFO] Registering the ROS2 Humble apt repository..."
        apt-get update
        apt-get install -y --no-install-recommends curl gnupg lsb-release ca-certificates software-properties-common
        add-apt-repository -y universe
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
            > /etc/apt/sources.list.d/ros2.list
        apt-get update   # refresh index so the ROS2 candidate is visible to the check below
    fi
fi

# --- ROS2 bridge / RViz deps
BRIDGE_PKGS=(
    ros-humble-ros-base
    python3-colcon-common-extensions
    ros-humble-ros-gz-bridge
    ros-humble-rviz2
    ros-humble-rmw-cyclonedds-cpp
    ros-humble-image-transport-plugins
    ros-humble-tf2-ros
    ros-humble-robot-state-publisher
    ros-humble-joint-state-publisher
)
bridge_installed() {
    for p in "${BRIDGE_PKGS[@]}"; do
        dpkg -s "$p" >/dev/null 2>&1 || return 1
    done
    return 0
}
if bridge_installed; then
    echo "✅ ROS2 bridge / RViz deps already installed."
elif ! apt-cache policy ros-humble-ros-gz-bridge 2>/dev/null | grep -q "Candidate: [0-9]"; then
    echo "[WARN] ROS2 Humble apt repo not found — skipping bridge/RViz deps (ros-gz-bridge, rviz2)."
elif [ "$(id -u)" -ne 0 ]; then
    echo "[ERROR] ROS2 bridge/RViz deps missing — re-run as root: sudo bash rbq_simulator/rbq_gazebo/scripts/debian-dep.bash"
else
    echo "[INFO] Installing ROS2 bridge / RViz deps..."
    apt-get update || true
    apt-get install -y --no-install-recommends "${BRIDGE_PKGS[@]}" \
        && echo "✅ ROS2 bridge / RViz deps installed." \
        || echo "[WARN] ROS2 bridge / RViz deps failed to install (sim core unaffected)."
fi
