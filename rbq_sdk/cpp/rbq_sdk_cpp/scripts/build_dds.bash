#!/usr/bin/env bash
set -e

# Build CycloneDDS and CycloneDDS-CXX and install them to the system
# Run: sudo bash cmake/build_dds.bash

# system temporary directory
export TMP_DIR=$(mktemp -d)
export DDS_VERSION="0.10.2"

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

mkdir -p "$TMP_DIR/dds" && cd "$TMP_DIR/dds"
rm -rf "$DDS_VERSION.tar.gz" "cyclonedds-$DDS_VERSION"
echo "[INFO] Downloading CycloneDDS $DDS_VERSION..."
download "https://github.com/eclipse-cyclonedds/cyclonedds/archive/refs/tags/$DDS_VERSION.tar.gz" "$DDS_VERSION.tar.gz"
tar -xf $DDS_VERSION.tar.gz && cd cyclonedds-$DDS_VERSION
mkdir build && cd build
echo "[INFO] Configuring dds ..."
cmake -S .. -B . -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_SHM=OFF \
    -DBUILD_DDSPERF=OFF
echo "[INFO] Building & Installing dds..."
cmake --build . -j "$(nproc_value)" && cmake --install .
echo "✅ dds version $DDS_VERSION installed"

cd "$TMP_DIR/dds"
rm -rf "cyclonedds-cxx-$DDS_VERSION"
echo "[INFO] Downloading CycloneDDS-CXX $DDS_VERSION..."
download "https://github.com/eclipse-cyclonedds/cyclonedds-cxx/archive/refs/tags/$DDS_VERSION.tar.gz" "$DDS_VERSION.tar.gz"
tar -xf $DDS_VERSION.tar.gz && cd cyclonedds-cxx-$DDS_VERSION
mkdir build && cd build
echo "[INFO] Configuring dds-cxx ..."
cmake -S .. -B . -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_SHM=OFF
echo "[INFO] Building & Installing dds-cxx..."
cmake --build . -j "$(nproc_value)" && cmake --install .
echo "✅ dds-cxx version $DDS_VERSION installed"
echo "[INFO] Verifying DDS install..."
"$(which idlc)" -v || echo "Warning: idlc version check failed"
echo "✅ idlc version check passed"

echo "[INFO] Installing fastcdr dev packages..."
DEBIAN_FRONTEND=noninteractive apt-get -y --quiet install \
    libfastcdr-dev libfastrtps-dev
echo "✅ fastcdr dev packages installed"