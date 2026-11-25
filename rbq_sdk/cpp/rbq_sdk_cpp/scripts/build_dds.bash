#!/usr/bin/env bash
set -e

# Build CycloneDDS and CycloneDDS-CXX and install them to the system
# Run: sudo bash cmake/build_dds.bash

# system temporary directory
export TMP_DIR=$(mktemp -d)
export DDS_VERSION="0.10.2"
# 받은 압축 파일을 sha256 으로 확인한다 — 판을 바꾸면 이 둘도. RBQ_DL_MIRROR(사내 캐시, 사내망에서만)가 있으면 거기 먼저 묻는다
DDS_SHA256="bc84e137e0c8a055b8cd97fbeafec94e36de1b0c2e88800896a82384fd867ae5"
DDS_CXX_SHA256="ff016df46b23015f32a57a36c8e2ddc2597942bb64ebef951747c46266d15931"

sha256_of() { if command -v sha256sum &>/dev/null; then sha256sum "$1"; else shasum -a 256 "$1"; fi | cut -d' ' -f1; }

download() {   # <url> <out> <sha256>
    local url="$1" out="$2" sha="$3" u
    command -v curl &>/dev/null || { echo "[ERROR] Need curl to download. Install it and retry." >&2; exit 1; }
    local tries=()
    local mirror="${RBQ_DL_MIRROR-${RBQ_MAVEN_MIRROR:-}}"
    if [ -n "$mirror" ] && curl -fsS -m 3 -o /dev/null "${mirror%/}/health" 2>/dev/null; then
        tries+=("${mirror%/}/github/${url#https://github.com/}")
    fi
    tries+=("$url")
    for u in "${tries[@]}"; do
        echo "[fetch] $u"
        if curl -fL --retry 3 -o "$out" "$u" && [ "$(sha256_of "$out")" = "$sha" ]; then return 0; fi
        echo "[WARN] $u: download failed or sha256 differs" >&2
    done
    echo "[ERROR] $out: no source gave sha256 $sha" >&2
    exit 1
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
download "https://github.com/eclipse-cyclonedds/cyclonedds/archive/refs/tags/$DDS_VERSION.tar.gz" "$DDS_VERSION.tar.gz" "$DDS_SHA256"
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
download "https://github.com/eclipse-cyclonedds/cyclonedds-cxx/archive/refs/tags/$DDS_VERSION.tar.gz" "$DDS_VERSION.tar.gz" "$DDS_CXX_SHA256"
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