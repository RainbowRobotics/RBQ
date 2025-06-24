#!/bin/bash

# === Default Configuration ===
USE_NINJA=true
MAKE_JOBS=$(nproc)
USE_CACHE=true
CURRENT_DIR="$PWD"
EXAMPLES_DIR="examples"
BUILD_DIR="build"
BIN_DIR="bin"
RBQL_DIR="$PWD/bin/rbq"

# === Help Message ===
print_help() {
    echo "Usage: bash scripts/example_build.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                  Display this help message and exit."
    echo "  --no-cache              Clean build directory and bypass cache."
    echo "  -j [number]             Specify number of CPUs for parallel build."
    echo "  --use-make              Use Make instead of Ninja."
}

# === Argument Parsing ===
while [[ $# -gt 0 ]]; do
    case "$1" in
        --help) print_help; exit 0 ;;
        --no-cache) USE_CACHE=false; shift ;;
        -j) shift;  MAKE_JOBS="${1:-$(nproc)}"; shift ;;
        --use-make) USE_NINJA=false;    shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

if [ ! -d $RBQL_DIR ]; then
    echo "[ERROR] $RBQL_DIR directory not exist!"
    exit 1
fi
echo $RBQL_DIR

if [ ! -f scripts/configure.bash ]; then
    echo "[ERROR] scripts/configure.bash not exist!"
    exit 1
fi
source scripts/configure.bash

if [ ! -d $EXAMPLES_DIR ]; then
    echo "[ERROR] $EXAMPLES_DIR directory not exist!"
    exit 1
fi
cd $EXAMPLES_DIR

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
    CMAKE_OPTIONS+=("-DCMAKE_BUILD_TYPE=Release")
    CMAKE_OPTIONS+=("-DBUILD_SHARED_LIBS=OFF")
    CMAKE_OPTIONS+=("-DCMAKE_INSTALL_PREFIX=$PWD")
    CMAKE_OPTIONS+=("-DCUSTOM_RBQ_PATH=$RBQL_DIR")
    CMAKE_PREFIX_PATH_STRING="$RBQL_DIR/lib/cmake"
    CMAKE_PREFIX_PATH_STRING+=";$EIGEN_DIR/share/eigen3/cmake"
    CMAKE_PREFIX_PATH_STRING+=";$DDS_DIR/lib/cmake"
    CMAKE_PREFIX_PATH_STRING+=";$DDS_CXX_DIR/lib/cmake"
    CMAKE_PREFIX_PATH_STRING+=";$ONNX_DIR/lib/cmake"    

    CMAKE_OPTIONS+=("-DCMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH_STRING")

    cmake -S . -B "$BUILD_DIR" -G "$GENERATOR" "${CMAKE_OPTIONS[@]}" || {
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
if [ -d $BIN_DIR ]; then
    BINARIES=($(find "$BIN_DIR" -maxdepth 1 -type f -executable))
    if [ ${#BINARIES[@]} -eq 0 ]; then
        echo "[INFO] No binaries to strip."
    else
        echo "[INFO] Stripping binaries..."
        for f in "${BINARIES[@]}"; do
            strip "$f"
        done
    fi
fi
cd $CURRENT_DIR

echo "✅ Build complete."
