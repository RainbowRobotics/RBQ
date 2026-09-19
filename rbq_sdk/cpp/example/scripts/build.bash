#!/usr/bin/env bash
set -e

USE_NINJA=true
MAKE_JOBS=$(nproc)
USE_CACHE=true
SOURCE_DIR="$PWD"
BUILD_DIR="$PWD/build"
BIN_DIR="$PWD/bin"

print_help() {
    echo "Usage: bash scripts/build.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                  Display this help message and exit."
    echo "  --no-cache              Clean build directory and bypass cache."
    echo "  -j [number]             Specify number of CPUs for parallel build."
    echo "  --use-make              Use Make instead of Ninja."
}
while [[ $# -gt 0 ]]; do
    case "$1" in
        --help) print_help; exit 0 ;;
        --no-cache) USE_CACHE=false;    shift ;;
        -j) shift;  MAKE_JOBS="${1:-$(nproc)}"; shift ;;
        --use-make) USE_NINJA=false;    shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done
if ! $USE_CACHE; then
    echo "[INFO] Cache bypassed. Cleaning build and bin directories..."
    rm -rf "$BUILD_DIR" "$BIN_DIR"
fi
if ! $USE_CACHE || [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    GENERATOR="Ninja"
    $USE_NINJA || GENERATOR="Unix Makefiles"

    echo "[INFO] Using generator: $GENERATOR"
    echo "[INFO] Configuring CMake..."

    CMAKE_OPTIONS=()
    CMAKE_OPTIONS+=("-D CMAKE_BUILD_TYPE=Release")
    CMAKE_OPTIONS+=("-D CMAKE_INSTALL_PREFIX=$BIN_DIR/..")

    CMAKE_OPTIONS+=("-DCMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH_STRING")

    cmake -S "$SOURCE_DIR" -B "$BUILD_DIR" -G "$GENERATOR" "${CMAKE_OPTIONS[@]}" || {
        echo "[ERROR] CMake configuration failed!"
        exit 1
    }
fi
echo "[INFO] Building the project..."
cmake --build "$BUILD_DIR" -j"$MAKE_JOBS" || {
    echo "[ERROR] CMake build failed!"
    exit 1
}
echo "[INFO] Installing the project..."
cmake --install "$BUILD_DIR" || {
    echo "[ERROR] CMake install failed!"
    exit 1
}
if [ ! -d "$BIN_DIR" ]; then
    echo "[ERROR] $BIN_DIR directory not found after installation!"
    exit 1
fi
echo "✅ Build complete."
