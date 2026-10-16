#!/bin/bash
set -e

MAKE_JOBS=$(nproc)
USE_CACHE=true
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAIN_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$MAIN_DIR/build"
BIN_DIR="$MAIN_DIR/bin"

print_help() {
    echo "Usage: bash scripts/build.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                  Display this help message and exit."
    echo "  --no-cache              Clean build directory and bypass cache."
    echo "  -j [number]             Specify number of CPUs for parallel build."
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --help) print_help; exit 0 ;;
        --no-cache)     USE_CACHE=false; shift ;;
        -j) shift;      MAKE_JOBS="${1:-$(nproc)}"; shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

if ! $USE_CACHE || [ -f "$BUILD_DIR/CMakeCache.txt" ] && \
   { ! grep -qF "$BUILD_DIR" "$BUILD_DIR/CMakeCache.txt" || \
     ! grep -q 'CMAKE_GENERATOR:INTERNAL=Unix Makefiles' "$BUILD_DIR/CMakeCache.txt" || \
     [ ! -f "$BUILD_DIR/Makefile" ]; }; then
    echo "[INFO] Cache bypassed."
    rm -rf "$BUILD_DIR" "$BIN_DIR"
fi

if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    GENERATOR="Unix Makefiles"

    echo "[INFO] Configuring CMake (generator: $GENERATOR)..."

    CMAKE_OPTIONS=()
    CMAKE_OPTIONS+=("-D CMAKE_BUILD_TYPE=Release")
    CMAKE_OPTIONS+=("-D CMAKE_INSTALL_PREFIX=$BIN_DIR/..")

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
