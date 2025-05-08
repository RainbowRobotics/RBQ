#!/bin/bash

# === Default Configuration ===
USE_NINJA=true
MAKE_JOBS=$(nproc)
USE_CACHE=true
BUILD_EXAMPLES=false
BUILD_DIR="build"
COLLECT_DEPENDENCIES=false

# === Help Message ===
print_help() {
    echo "Usage: bash scripts/build.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                  Display this help message and exit."
    echo "  --no-cache              Clean build directory and bypass cache."
    echo "  -j [number]             Specify number of CPUs for parallel build."
    echo "  --use-make              Use Make instead of Ninja."
    echo "  --examples              Build examples."
}

# === Argument Parsing ===
while [[ $# -gt 0 ]]; do
    case "$1" in
        --help) print_help; exit 0 ;;
        --no-cache) USE_CACHE=false; shift ;;
        -j) shift;  MAKE_JOBS="${1:-$(nproc)}"; shift ;;
        --use-make) USE_NINJA=false; shift ;;
        --examples) BUILD_EXAMPLES=true; shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

# === Cache Handling ===
if ! $USE_CACHE; then
    echo "[INFO] Cache bypassed. Cleaning build and bin directories..."
    rm -rf "$BUILD_DIR"
fi

# === CMake Configuration ===
source scripts/configure.bash
if ! $USE_CACHE || [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    GENERATOR="Ninja"
    $USE_NINJA || GENERATOR="Unix Makefiles"

    echo "[INFO] Using generator: $GENERATOR"
    echo "[INFO] Configuring CMake..."

    CMAKE_OPTIONS=()
    CMAKE_OPTIONS+=("-DCMAKE_BUILD_TYPE=Release")
    CMAKE_OPTIONS+=("-DBUILD_SHARED_LIBS=OFF")

    CMAKE_PREFIX_PATH_STRING="$EIGEN_DIR/share/eigen3/cmake"
    CMAKE_PREFIX_PATH_STRING+=";$JSON_DIR/share/cmake"
    CMAKE_PREFIX_PATH_STRING+=";$QT_DIR/lib/cmake"

    CMAKE_OPTIONS+=("-DCUSTOM_RBQ_PATH=$RBQ_DIR")
    CMAKE_PREFIX_PATH_STRING+=";$RBQ_DIR/lib/cmake"

    CMAKE_OPTIONS+=("-DCMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH_STRING")

    $BUILD_EXAMPLES && CMAKE_OPTIONS+=("-D BUILD_EXAMPLES=ON")

    cmake -S . -B "$BUILD_DIR" -G "$GENERATOR" "${CMAKE_OPTIONS[@]}" || {
        echo "[ERROR] CMake configuration failed!"
        exit 1
    }
fi

# === Build Project ===
echo "[INFO] Building the project..."
cmake --build "$BUILD_DIR" -j"$MAKE_JOBS" || {
    echo "[ERROR] CMake build failed!"
    exit 1
}

echo "✅ Build complete."
