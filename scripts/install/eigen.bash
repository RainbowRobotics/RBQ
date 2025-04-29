#!/usr/bin/env bash
set -e
source scripts/configure.bash

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
    if [ $REMOVE_TMP ]; then
        echo "[INFO] Cleaning up temporary files..."
        cd "$LIBS_DIR" && rm -rf "$TMP_DIR"
    fi
fi
echo "✅ eigen version $EIGEN_VERSION installed to $EIGEN_DIR"
