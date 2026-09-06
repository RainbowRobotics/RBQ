#!/usr/bin/env bash
set -e

apt clean
rm -rf /var/lib/apt/lists/*
apt update --quiet -o Acquire::BrokenProxy=true -o Acquire::AllowInsecureRepositories=true || true
apt upgrade -y --quiet

dpkg --configure -a

DEBIAN_FRONTEND=noninteractive apt -y --quiet --no-install-recommends install  \
    gnome-terminal dbus-x11 dconf-cli libcanberra-gtk-module libcanberra-gtk3-module sudo iputils-ping \
    locales ca-certificates apt-transport-https \
    cmake build-essential pkg-config ninja-build \
    git curl wget libssl-dev libcurl4-openssl-dev \
    locales \
    mesa-common-dev \
    xvfb libgl1-mesa-dri \
    libglu1-mesa-dev libglfw3-dev libglew-dev libgl1-mesa-dev libsoup2.4-dev libglib2.0-dev \
    libpng-dev libflatbuffers-dev libtins-dev libpcap-dev libsuitesparse-dev libgtk-3-dev \
    '^libxcb.*-dev' libx11-xcb-dev libxrender-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev libxcb1-dev libx11-dev libxext-dev libxcb-* \
    libxrandr-dev libxinerama-dev libxcursor-dev libxfixes-dev \
    libeigen3-dev libasio-dev nlohmann-json3-dev \
    libopencv-dev libopencv-contrib-dev \

echo "✅ apt installed."

# system temporary directory
export TMP_DIR=$(mktemp -d)
export MUJOCO_VERSION="3.3.0"

download() {
    local url="$1"
    local out="$2"
    if command -v wget &>/dev/null; then
        wget -q --show-progress -O "$out" "$url"
    elif command -v curl &>/dev/null; then
        curl -fSL -o "$out" "$url"
    else
        echo "[ERROR] Need wget or curl to download. Install one and retry." >&2
        exit 1
    fi
}

nproc_value() {
    if command -v nproc &>/dev/null; then
        nproc
    elif [ "$(uname -s)" = "Darwin" ]; then
        sysctl -n hw.ncpu
    else
        echo 2
    fi
}

mkdir -p $TMP_DIR && cd $TMP_DIR
rm -rf $MUJOCO_VERSION.tar.gz mujoco-$MUJOCO_VERSION
echo "[INFO] Downloading mujoco $MUJOCO_VERSION..."
download "https://github.com/google-deepmind/mujoco/archive/refs/tags/$MUJOCO_VERSION.tar.gz" "$MUJOCO_VERSION.tar.gz"
tar -xf $MUJOCO_VERSION.tar.gz && cd mujoco-$MUJOCO_VERSION

mkdir -p build && cd build
cmake -S .. -B . -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -DMUJOCO_BUILD_EXAMPLES=OFF \
    -DMUJOCO_BUILD_TESTS=OFF \
    -DMUJOCO_TEST_PYTHON_UTIL=OFF \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DMUJOCO_ENABLE_AVX=ON

echo "[INFO] Building & Installing mujoco..."
cmake --build . -j "$(nproc_value)" && cmake --install .
echo "✅ mujoco version $MUJOCO_VERSION installed"

ldconfig
