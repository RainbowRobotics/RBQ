#!/usr/bin/env bash
set -e

apt clean
rm -rf /var/lib/apt/lists/*
apt update --quiet -o Acquire::BrokenProxy=true -o Acquire::AllowInsecureRepositories=true || true
apt upgrade -y --quiet

dpkg --configure -a

DEBIAN_FRONTEND=noninteractive apt -y --quiet --no-install-recommends install  \
    cmake build-essential pkg-config ninja-build \
    git gh curl wget libssl-dev libcurl4-openssl-dev \
    libeigen3-dev \
    libasio-dev \
    locales \
    mesa-common-dev \
    libglu1-mesa-dev libglfw3-dev libglew-dev libgl1-mesa-dev libsoup2.4-dev libglib2.0-dev \
    libpng-dev libflatbuffers-dev libtins-dev libpcap-dev libsuitesparse-dev libgtk-3-dev \
    '^libxcb.*-dev' libx11-xcb-dev libxrender-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev libxcb1-dev libx11-dev libxext-dev libxcb-* \
    libxrandr-dev libxinerama-dev libxcursor-dev libxfixes-dev \
    python3 python3-dev python3-pip \
    ca-certificates \
    libsdl2-dev \
    libspdlog-dev \
    apt-transport-https \
    fuse3 \
    v4l-utils libusb-1.0-0-dev \
    lsb-release \
    meson flex bison \
    yasm \
    doxygen \
    sshpass \
    qtbase5-dev qt5-qmake qtmultimedia5-dev libqt5multimedia5-plugins libqt5gamepad5-dev libqt5opengl5-dev qtcreator
    
echo "✅ apt installed."
