#!/usr/bin/env bash
set -e

# Exit if executed with sudo
if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

sudo bash scripts/apt.bash

source scripts/configure.bash

if [ ! -d $CMAKE_DIR ]; then
    mkdir -p "$TMP_DIR" && cd "$TMP_DIR"
    rm -rf cmake-$CMAKE_VERSION-linux-x86_64.tar.gz cmake-$CMAKE_VERSION-linux-x86_64
    echo "[INFO] Downloading cmake $CMAKE_VERSION..."
    wget -q https://github.com/Kitware/CMake/releases/download/v$CMAKE_VERSION/cmake-$CMAKE_VERSION-linux-x86_64.tar.gz
    echo "[INFO] Extracting cmake ..."
    tar -xzf cmake-$CMAKE_VERSION-linux-x86_64.tar.gz
    mkdir -p $CMAKE_DIR && mv cmake-$CMAKE_VERSION-linux-x86_64/* $CMAKE_DIR
    if [[ "${REMOVE_TMP,,}" == "true" ]]; then
        echo "[INFO] Cleaning up temporary files..."
	cd $LIBS_DIR && rm -rf $TMP_DIR
    fi
fi
echo "✅ cmake version $CMAKE_VERSION installed to $CMAKE_DIR"

if [ ! -d "$EIGEN_DIR" ]; then
    mkdir -p "$TMP_DIR" && cd "$TMP_DIR"
    rm -rf eigen-$EIGEN_VERSION.tar.gz eigen-$EIGEN_VERSION
    echo "[INFO] Cloning eigen version: $EIGEN_VERSION ..."
    wget https://gitlab.com/libeigen/eigen/-/archive/$EIGEN_VERSION/eigen-$EIGEN_VERSION.tar.gz
    tar -xf eigen-$EIGEN_VERSION.tar.gz && cd eigen-$EIGEN_VERSION
    mkdir build && cd build
    echo "[INFO] Configuring eigen ..."
    cmake -S .. -B . -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=$EIGEN_DIR \
        -DEIGEN_LEAVE_TEST_IN_ALL=OFF \
        -DBUILD_TESTING=OFF \
        -DEIGEN_BUILD_DOC=OFF \
        -DEIGEN_BUILD_PKGCONFIG=OFF

    echo "[INFO] Building & Installing eigen to $EIGEN_DIR..."
    cmake --build . -j $(nproc) && cmake --install .
    if [[ "${REMOVE_TMP,,}" == "true" ]]; then
        echo "[INFO] Cleaning up temporary files..."
        cd "$LIBS_DIR" && rm -rf "$TMP_DIR"
    fi
fi
echo "✅ eigen version $EIGEN_VERSION installed to $EIGEN_DIR"

if [ ! -d "$JSON_DIR" ]; then
    mkdir -p "$TMP_DIR" && cd "$TMP_DIR"
    rm -rf v$JSON_VERSION.tar.gz json-$JSON_VERSION
    echo "[INFO] Cloning json version: $JSON_VERSION ..."
    wget https://github.com/nlohmann/json/archive/refs/tags/v$JSON_VERSION.tar.gz
    tar -xf v$JSON_VERSION.tar.gz && cd json-$JSON_VERSION
    mkdir build && cd build
    echo "[INFO] Configuring json ..."
    cmake -S .. -B . -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=$JSON_DIR \
        -DBUILD_SHARED_LIBS=OFF \
        -DJSON_BuildTests=OFF \
        -DJSON_MultipleHeaders=OFF


    echo "[INFO] Building & Installing json to $JSON_DIR..."
    cmake --build . -j $(nproc) && cmake --install .
    if [[ "${REMOVE_TMP,,}" == "true" ]]; then
        echo "[INFO] Cleaning up temporary files..."
        cd "$LIBS_DIR" && rm -rf "$TMP_DIR"
    fi
fi
echo "✅ json version $JSON_VERSION installed to $JSON_DIR"

if [ ! -d "$ONNX_DIR" ]; then
    mkdir -p "$TMP_DIR" && cd "$TMP_DIR"
    rm -rf onnxruntime
    echo "[INFO] Cloning onnx version: $ONNX_VERSION ..."
    git clone --recursive https://github.com/microsoft/onnxruntime.git
    cd onnxruntime
    git checkout $ONNX_VERSION && git submodule update --init --recursive

    echo "[INFO] Building ONNX ..."
    ./build.sh \
        --config Release \
	--parallel \
        --disable_ml_ops \
        --allow_running_as_root \
	--cmake_extra_defines \
	    onnxruntime_BUILD_SHARED_LIB=ON \
	    onnxruntime_ENABLE_PYTHON=OFF \
	    onnxruntime_BUILD_UNIT_TESTS=OFF \
	    onnxruntime_ENABLE_LTO=ON \
	    onnxruntime_ENABLE_OPENMP=ON \
	    onnxruntime_ENABLE_EXCEPTIONS=ON \
	    CMAKE_POSITION_INDEPENDENT_CODE=ON \
	    CMAKE_INSTALL_PREFIX=$ONNX_DIR

    echo "[INFO] Installing ONNX to $ONNX_DIR..."
    cd build/Linux/Release && cmake --install .

    echo "[INFO] Stripping binaries..."
    find $ONNX_DIR -type f -executable -exec strip --strip-unneeded {} +

    if [[ "${REMOVE_TMP,,}" == "true" ]]; then
        echo "[INFO] Cleaning up temporary files..."
	cd $LIBS_DIR && rm -rf $TMP_DIR
    fi
fi
echo "✅ onnx version $ONNX_VERSION installed to $ONNX_DIR"

