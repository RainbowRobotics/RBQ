#!/usr/bin/env bash
set -e
source scripts/configure.bash

if [ ! -d $CMAKE_DIR ]; then
    mkdir -p "$TMP_DIR" && cd "$TMP_DIR"
    rm -rf cmake-$CMAKE_VERSION-linux-x86_64.tar.gz cmake-$CMAKE_VERSION-linux-x86_64
    echo "[INFO] Downloading cmake $CMAKE_VERSION..."
    wget -q https://github.com/Kitware/CMake/releases/download/v$CMAKE_VERSION/cmake-$CMAKE_VERSION-linux-x86_64.tar.gz
    echo "[INFO] Extracting cmake ..."
    tar -xzf cmake-$CMAKE_VERSION-linux-x86_64.tar.gz
    mkdir -p $CMAKE_DIR && mv cmake-$CMAKE_VERSION-linux-x86_64/* $CMAKE_DIR
    if [ $REMOVE_TMP ]; then
        echo "[INFO] Cleaning up temporary files..."
	cd $LIBS_DIR && rm -rf $TMP_DIR
    fi
fi
echo "✅ cmake version $CMAKE_VERSION installed to $CMAKE_DIR"
