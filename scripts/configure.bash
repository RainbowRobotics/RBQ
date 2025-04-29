#!/usr/bin/env bash
set -e

export REMOVE_TMP=true

export LIBS_DIR="$PWD/3rdparty"
export TMP_DIR="$LIBS_DIR/tmp"

# cmake
export CMAKE_VERSION="3.27.9"
export CMAKE_DIR="$LIBS_DIR/cmake"
export PATH=$CMAKE_DIR/bin:$PATH

# eigen
export EIGEN_VERSION="3.4.0"
export EIGEN_DIR="$LIBS_DIR/eigen"

# json
export JSON_VERSION="3.12.0"
export JSON_DIR="$LIBS_DIR/json"

# Qt
export QT_VERSION="5.15.16"
export QT_DIR="$LIBS_DIR/qt"
export QT_TAR="qt-everywhere-opensource-src-$QT_VERSION.tar.xz"

# choreonoid
export CHOREONOID_DIR=$LIBS_DIR/choreonoid
export CHOREONOID_VERSION="v2.2.0"
