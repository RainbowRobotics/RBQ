#!/usr/bin/env bash
set -e
source scripts/configure.bash

if [ ! -f "$QT_DIR/$QT_TAR" ]; then
    echo "[INFO] Downloading Qt version $QT_VERSION to $QT_DIR..."
    mkdir -p "$QT_DIR" && cd $QT_DIR
    wget https://download.qt.io/archive/qt/5.15/$QT_VERSION/single/$QT_TAR
    if [ ! -f "$QT_DIR/$QT_TAR" ]; then
        echo "[ERROR] Qt source not found in $QT_DIR/$QT_TAR download failed."
        exit 1
    fi
fi
echo "✅ Qt source version $QT_VERSION downloaded to: $QT_DIR/$QT_TAR"
