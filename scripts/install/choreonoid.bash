#!/usr/bin/env bash
set -e
source scripts/configure.bash

export SIM_VAR_DIR="$PWD/rbq_sdk/include/SimVariable.h"
export CONTROLLER=RBQController
export CONTROLLER_DIR="$PWD/resources/$CONTROLLER"

# Check if controller directory exists
if [ ! -d "$CONTROLLER_DIR" ]; then
    echo "Controller directory not found: $CONTROLLER_DIR"
    exit 1
fi

# Check if simulation variable header exists
if [ ! -f "$SIM_VAR_DIR" ]; then
    echo "Simulation variable header not found: $SIM_VAR_DIR"
    exit 1
fi

if [ ! -d "$CHOREONOID_DIR" ]; then
    mkdir -p "$TMP_DIR" && cd "$TMP_DIR"
    rm -rf $CHOREONOID_VERSION.tar.gz choreonoid-*
    echo "[INFO] Downloading choreonoid $CHOREONOID_VERSION..."
    wget https://github.com/choreonoid/choreonoid/archive/refs/tags/$CHOREONOID_VERSION.tar.gz
    tar xf $CHOREONOID_VERSION.tar.gz && cd choreonoid-*
    sudo ./misc/script/install-requisites-ubuntu-22.04.sh
    cp -r $CONTROLLER_DIR sample/.
    var="1i #include <$SIM_VAR_DIR>"
    sed -i "$var" sample/$CONTROLLER/$CONTROLLER.cpp
    echo "add_subdirectory($CONTROLLER)" >> sample/CMakeLists.txt
    mkdir -p build && cd build
    cmake -S .. -B . -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DENABLE_SAMPLES=ON \
        -DENABLE_PYTHON=OFF \
        -DENABLE_URDF=ON \
        -DCMAKE_INSTALL_PREFIX=$CHOREONOID_DIR \
        -DBUILD_ODE_PLUGIN=ON

    echo "[INFO] Building & Installing choreonoid ..."
    cmake --build . -j $(nproc) && cmake --install .
    if [ $REMOVE_TMP ]; then
        echo "[INFO] Cleaning up temporary files..."
        cd "$LIBS_DIR" && rm -rf "$TMP_DIR"
    fi
fi
echo "✅ choreonoid version: $CHOREONOID_VERSION installed to: $CHOREONOID_DIR"
