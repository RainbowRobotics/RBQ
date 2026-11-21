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
    libeigen3-dev libyaml-cpp-dev \

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

case "$(uname -m)" in
    x86_64)   NUGET_ARCH="x64" ;;
    aarch64)  NUGET_ARCH="arm64" ;;
    *) echo "[ERROR] Unsupported architecture: $(uname -m)" && exit 1 ;;
esac

# GitHub microsoft/onnxruntime has returned 404 since 2026-11-17, so the release tarball is gone.
# The official NuGet package carries the same build (headers + Linux .so for both arches) but not the
# tarball's CMake config or versioned .so name — both are recreated below.
NUPKG="microsoft.ml.onnxruntime.${ONNX_VERSION}.nupkg"
URL="https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime/${ONNX_VERSION}/${NUPKG}"
NUGET_SHA256="769d1d3ea8ab6cd69f737c9dd4d4462aa4ad0ccfa106eaf506efc40d7bead5db"

echo "[INFO] Downloading ONNX Runtime $ONNX_VERSION (NuGet, linux-$NUGET_ARCH) ..."
wget -q "$URL" -O "$NUPKG"
echo "$NUGET_SHA256  $NUPKG" | sha256sum -c -

echo "[INFO] Extracting ..."
NATIVE="runtimes/linux-${NUGET_ARCH}/native"
python3 -c 'import sys, zipfile
z = zipfile.ZipFile(sys.argv[1])
for n in z.namelist():
    if n.startswith(tuple(sys.argv[3:])):
        z.extract(n, sys.argv[2])' "$NUPKG" nupkg build/native/include/ "$NATIVE/" LICENSE ThirdPartyNotices.txt

EXTRACT_DIR="onnxruntime-linux-${NUGET_ARCH}-${ONNX_VERSION}"
mkdir -p "$EXTRACT_DIR/include" "$EXTRACT_DIR/lib/cmake/onnxruntime" "$EXTRACT_DIR/share/onnxruntime"
cp nupkg/build/native/include/* "$EXTRACT_DIR/include/"
install -m 755 "nupkg/$NATIVE/libonnxruntime.so" "$EXTRACT_DIR/lib/libonnxruntime.so.${ONNX_VERSION}"
install -m 755 "nupkg/$NATIVE/libonnxruntime_providers_shared.so" "$EXTRACT_DIR/lib/"
ln -s "libonnxruntime.so.${ONNX_VERSION}" "$EXTRACT_DIR/lib/libonnxruntime.so.1"
ln -s libonnxruntime.so.1 "$EXTRACT_DIR/lib/libonnxruntime.so"
cp nupkg/LICENSE nupkg/ThirdPartyNotices.txt "$EXTRACT_DIR/share/onnxruntime/"

cat > "$EXTRACT_DIR/lib/cmake/onnxruntime/onnxruntimeConfigVersion.cmake" <<EOF
set(PACKAGE_VERSION "${ONNX_VERSION}")
if(PACKAGE_FIND_VERSION VERSION_GREATER PACKAGE_VERSION)
  set(PACKAGE_VERSION_COMPATIBLE FALSE)
else()
  set(PACKAGE_VERSION_COMPATIBLE TRUE)
  if(PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)
    set(PACKAGE_VERSION_EXACT TRUE)
  endif()
endif()
EOF

# The target points at ${PREFIX}/lib64 (as the upstream tarball's did), so lib is mirrored into lib64 below.
cat > "$EXTRACT_DIR/lib/cmake/onnxruntime/onnxruntimeConfig.cmake" <<EOF
get_filename_component(_ORT_PREFIX "\${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
if(NOT TARGET onnxruntime::onnxruntime)
  add_library(onnxruntime::onnxruntime SHARED IMPORTED)
  set_target_properties(onnxruntime::onnxruntime PROPERTIES
    IMPORTED_CONFIGURATIONS RELEASE
    IMPORTED_LOCATION_RELEASE "\${_ORT_PREFIX}/lib64/libonnxruntime.so.${ONNX_VERSION}"
    IMPORTED_SONAME_RELEASE "libonnxruntime.so.1"
    INTERFACE_INCLUDE_DIRECTORIES "\${_ORT_PREFIX}/include/onnxruntime")
endif()
unset(_ORT_PREFIX)
EOF

echo "[INFO] Installing ONNX Runtime to $ONNX_DIR ..."
run_priv mkdir -p "$ONNX_DIR/include" "$ONNX_DIR/lib" "$ONNX_DIR/share"
run_priv cp -a "$EXTRACT_DIR/include"/* "$ONNX_DIR/include/"
# CMake INTERFACE_INCLUDE_DIRECTORIES is ${PREFIX}/include/onnxruntime — mirror the headers there too.
run_priv mkdir -p "$ONNX_DIR/include/onnxruntime"
run_priv cp -a "$EXTRACT_DIR/include"/* "$ONNX_DIR/include/onnxruntime/"
run_priv cp -a "$EXTRACT_DIR/lib"/* "$ONNX_DIR/lib/"
run_priv mkdir -p "$ONNX_DIR/lib64"
run_priv cp -a "$EXTRACT_DIR/lib"/* "$ONNX_DIR/lib64/"
run_priv cp -a "$EXTRACT_DIR/share/onnxruntime" "$ONNX_DIR/share/"

if command -v ldconfig >/dev/null 2>&1; then
    run_priv ldconfig
fi

echo "✅ onnx version $ONNX_VERSION installed to $ONNX_DIR"
