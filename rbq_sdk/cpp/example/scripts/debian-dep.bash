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
    git curl wget sshpass libssl-dev libcurl4-openssl-dev \
    python3 python3-dev python3-pip \
    libeigen3-dev \

echo "✅ apt installed."

: "${ONNX_VERSION:=1.28.0}"
: "${ONNX_DIR:=/usr/local}"

ORT_CONFIG="$ONNX_DIR/lib/cmake/onnxruntime/onnxruntimeConfigVersion.cmake"
ORT_LIB64="$ONNX_DIR/lib64/libonnxruntime.so.${ONNX_VERSION}"
ORT_INCLUDE_API="$ONNX_DIR/include/onnxruntime/onnxruntime_c_api.h"
if [ -f "$ORT_CONFIG" ] && grep -qF "set(PACKAGE_VERSION \"${ONNX_VERSION}\")" "$ORT_CONFIG" \
    && [ -f "$ORT_LIB64" ] && [ -f "$ORT_INCLUDE_API" ]; then
    echo "✅ onnx version $ONNX_VERSION already installed at $ONNX_DIR"
    exit 0
fi

if [ "$(id -u)" -ne 0 ] && ! sudo -n true 2>/dev/null; then
    echo "[ERROR] Installing to $ONNX_DIR requires root. Run: sudo bash ${BASH_SOURCE[0]}"
    exit 1
fi

run_priv() {
    if [ "$(id -u)" -eq 0 ]; then
        "$@"
    else
        sudo "$@"
    fi
}

TMP_DIR=$(mktemp -d) && cd "$TMP_DIR"
rm -rf onnxruntime-linux-*

case "$(uname -m)" in
    x86_64)   ORT_ARCH="x64" ;;
    aarch64)  ORT_ARCH="aarch64" ;;
    *) echo "[ERROR] Unsupported architecture: $(uname -m)" && exit 1 ;;
esac

TARBALL="onnxruntime-linux-${ORT_ARCH}-${ONNX_VERSION}.tgz"
URL="https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/${TARBALL}"

case "$ORT_ARCH" in
    x64)
        EXPECTED_SHA256="a3e1b79d7bb1bf09696ce675f49e4064e6c81f6202b8225624fff0e93f8d6407"
        ;;
    aarch64)
        EXPECTED_SHA256="e15ff8b5d85afe6c144d97c6fd432254bf76a219daaf17658087d6ecb3e8f0bb"
        ;;
esac

echo "[INFO] Downloading ONNX Runtime $ONNX_VERSION ($ORT_ARCH) ..."
wget -q "$URL" -O "$TARBALL"

echo "$EXPECTED_SHA256  $TARBALL" | sha256sum -c -

echo "[INFO] Extracting ..."
tar -xzf "$TARBALL"
EXTRACT_DIR="onnxruntime-linux-${ORT_ARCH}-${ONNX_VERSION}"

echo "[INFO] Installing ONNX Runtime to $ONNX_DIR ..."
run_priv mkdir -p "$ONNX_DIR/include" "$ONNX_DIR/lib"
run_priv cp -a "$EXTRACT_DIR/include"/* "$ONNX_DIR/include/"
# CMake INTERFACE_INCLUDE_DIRECTORIES is ${PREFIX}/include/onnxruntime; tarball
# ships headers at include/*.h — mirror them into include/onnxruntime/.
run_priv mkdir -p "$ONNX_DIR/include/onnxruntime"
run_priv cp -a "$EXTRACT_DIR/include"/* "$ONNX_DIR/include/onnxruntime/"
run_priv cp -a "$EXTRACT_DIR/lib"/* "$ONNX_DIR/lib/"

# Upstream tarballs ship libraries under lib/, but onnxruntimeTargets-*.cmake
# hard-codes IMPORTED_LOCATION to ${PREFIX}/lib64/. Mirror lib into lib64 so
# find_package(onnxruntime) resolves.
run_priv mkdir -p "$ONNX_DIR/lib64"
run_priv cp -a "$EXTRACT_DIR/lib"/* "$ONNX_DIR/lib64/"

if command -v ldconfig >/dev/null 2>&1; then
    run_priv ldconfig
fi

echo "✅ onnx version $ONNX_VERSION installed to $ONNX_DIR"
