#!/usr/bin/env bash
set -e
source scripts/configure.bash

if [ ! -f "$QT_DIR/bin/qmake" ]; then
    mkdir -p "$TMP_DIR" && cd "$TMP_DIR"
    rm -rf $QT_TAR qt-everywhere-src-$QT_VERSION
    mv $QT_DIR/$QT_TAR . && rm -rf $QT_DIR && mkdir -p $QT_DIR && cp $QT_TAR $QT_DIR/.
    tar xf $QT_TAR && cd qt-everywhere-src-$QT_VERSION

    echo "[INFO] Configuring Qt $QT_VERSION to $QT_DIR..."
    ./configure \
        -prefix "$QT_DIR" \
        -static \
        -release \
        -opensource -confirm-license \
        -silent -nomake examples -nomake tests \
        -no-feature-xml \
        -no-feature-testlib \
        -no-dbus \
        -no-icu \
        -no-feature-sql \
        -no-feature-statemachine \
        -skip qt3d \
        -skip qtdeclarative \
        -skip qtdoc \
        -skip qtgraphicaleffects \
        -skip qtlocation \
        -skip qtlottie \
        -skip qtmultimedia \
        -skip qtnetworkauth \
        -skip qtquick3d \
        -skip qtquickcontrols \
        -skip qtremoteobjects \
        -skip qtscript \
        -skip qtsensors \
        -skip qtserialbus \
        -skip qtserialport \
        -skip qtwebchannel \
        -skip qtwebglplugin \
        -skip qtwebsockets \
        -skip qtwebview \
        -skip qtwebengine \
        -skip qttools \
        -skip qtconnectivity \
        -skip qtxmlpatterns \
        -skip qtspeech \
        -qpa xcb -no-shared \
        -qt-doubleconversion -qt-pcre -qt-zlib -qt-harfbuzz \
        -qt-libpng -qt-libjpeg -qt-tiff \
        -optimize-size -strip -fontconfig \

    echo "[INFO] Building Qt..."
    make -j$(nproc) 2>&1 | tee build.log
    if grep -i "error:" build.log; then
        echo "[ERROR] Build failed due to errors! check build.log for more info"
        exit 1
    fi
    echo "[INFO] Installing Qt..."
    make install
    if [ ! -f "$QT_DIR/bin/qmake" ]; then
        echo "[ERROR] qmake not found in $QT_DIR/bin! Installation failed."
        exit 1
    fi
    if [ $REMOVE_TMP ]; then
        echo "[INFO] Cleaning up temporary files..."
        cd "$LIBS_DIR" && rm -rf "$TMP_DIR"
    fi
fi
echo "✅ Qt version $QT_VERSION installed to $QT_DIR"
