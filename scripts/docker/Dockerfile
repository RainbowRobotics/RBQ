FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:99

WORKDIR /workspace

ENV SCRIPTS_DIR="/workspace/scripts"
RUN mkdir -p $SCRIPTS_DIR

# Configure
COPY scripts/configure.bash $SCRIPTS_DIR/configure.bash
RUN bash $SCRIPTS_DIR/configure.bash

# APT
COPY scripts/install/apt.bash $SCRIPTS_DIR/apt.bash
RUN bash $SCRIPTS_DIR/apt.bash
RUN locale-gen en_US.UTF-8
RUN dpkg-reconfigure locales

# cmake
COPY scripts/install/cmake.bash $SCRIPTS_DIR/cmake.bash
RUN bash $SCRIPTS_DIR/cmake.bash

# Eigen
COPY scripts/install/eigen.bash $SCRIPTS_DIR/eigen.bash
RUN bash $SCRIPTS_DIR/eigen.bash

# JSON
COPY scripts/install/json.bash $SCRIPTS_DIR/json.bash
RUN bash $SCRIPTS_DIR/json.bash

# Qt
COPY scripts/install/download_qt.bash $SCRIPTS_DIR/download_qt.bash
RUN bash $SCRIPTS_DIR/download_qt.bash
COPY scripts/install/qt.bash $SCRIPTS_DIR/qt.bash
RUN bash $SCRIPTS_DIR/qt.bash

# Project build script
COPY scripts/build.bash $SCRIPTS_DIR/__build__.bash
