#!/usr/bin/env bash
set -e
source scripts/configure.bash

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
    if [ $REMOVE_TMP ]; then
        echo "[INFO] Cleaning up temporary files..."
        cd "$LIBS_DIR" && rm -rf "$TMP_DIR"
    fi
fi
echo "✅ json version $JSON_VERSION installed to $JSON_DIR"
