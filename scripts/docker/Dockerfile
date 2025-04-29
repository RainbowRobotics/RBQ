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

# SOPHUS
COPY scripts/install/sophus.bash $SCRIPTS_DIR/sophus.bash
RUN bash $SCRIPTS_DIR/sophus.bash

# BOOST
COPY scripts/install/boost.bash $SCRIPTS_DIR/boost.bash
RUN bash $SCRIPTS_DIR/boost.bash

# TBB
COPY scripts/install/tbb.bash $SCRIPTS_DIR/tbb.bash
RUN bash $SCRIPTS_DIR/tbb.bash

# JSON
COPY scripts/install/json.bash $SCRIPTS_DIR/json.bash
RUN bash $SCRIPTS_DIR/json.bash

# ONNX
COPY scripts/install/onnx.bash $SCRIPTS_DIR/onnx.bash
RUN bash $SCRIPTS_DIR/onnx.bash

# MuJoCo
COPY scripts/install/mujoco.bash $SCRIPTS_DIR/mujoco.bash
RUN bash $SCRIPTS_DIR/mujoco.bash

# RBDL
COPY scripts/install/rbdl.bash $SCRIPTS_DIR/rbdl.bash
RUN bash $SCRIPTS_DIR/rbdl.bash

# QPSWIFT
COPY scripts/install/qp.bash $SCRIPTS_DIR/qp.bash
RUN bash $SCRIPTS_DIR/qp.bash

# Qt
COPY scripts/install/download_qt.bash $SCRIPTS_DIR/download_qt.bash
RUN bash $SCRIPTS_DIR/download_qt.bash
COPY scripts/install/qt.bash $SCRIPTS_DIR/qt.bash
RUN bash $SCRIPTS_DIR/qt.bash

# DDS
COPY scripts/install/dds.bash $SCRIPTS_DIR/dds.bash
RUN bash $SCRIPTS_DIR/dds.bash

# mediamtx
COPY scripts/install/mediamtx.bash $SCRIPTS_DIR/mediamtx.bash
RUN bash $SCRIPTS_DIR/mediamtx.bash

# DEPTHAI
COPY scripts/install/depthai.bash $SCRIPTS_DIR/depthai.bash
RUN bash $SCRIPTS_DIR/depthai.bash

# JSONCPP
COPY scripts/install/jsoncpp.bash $SCRIPTS_DIR/jsoncpp.bash
RUN bash $SCRIPTS_DIR/jsoncpp.bash

# OUSTER
COPY scripts/install/ouster.bash $SCRIPTS_DIR/ouster.bash
RUN bash $SCRIPTS_DIR/ouster.bash

# PHTREE
COPY scripts/install/phtree.bash $SCRIPTS_DIR/phtree.bash
RUN bash $SCRIPTS_DIR/phtree.bash

# OCTOMAP
COPY scripts/install/octomap.bash $SCRIPTS_DIR/octomap.bash
RUN bash $SCRIPTS_DIR/octomap.bash

# LIVOX2
COPY scripts/install/livox2.bash $SCRIPTS_DIR/livox2.bash
RUN bash $SCRIPTS_DIR/livox2.bash

# GTSAM
COPY scripts/install/gtsam.bash $SCRIPTS_DIR/gtsam.bash
RUN bash $SCRIPTS_DIR/gtsam.bash

# FFMPEG
#COPY scripts/install/ffmpeg.bash $SCRIPTS_DIR/ffmpeg.bash
#RUN bash $SCRIPTS_DIR/ffmpeg.bash

# GStreamer
#COPY scripts/install/gst.bash $SCRIPTS_DIR/gst.bash
#RUN bash $SCRIPTS_DIR/gst.bash

# OpenCV
#COPY scripts/install/opencv.bash $SCRIPTS_DIR/opencv.bash
#RUN bash $SCRIPTS_DIR/opencv.bash

# VTK
#COPY scripts/install/vtk.bash $SCRIPTS_DIR/vtk.bash
#RUN bash $SCRIPTS_DIR/vtk.bash

# FLANN
#COPY scripts/install/flann.bash $SCRIPTS_DIR/flann.bash
#RUN bash $SCRIPTS_DIR/flann.bash

# PCL
#COPY scripts/install/pcl.bash $SCRIPTS_DIR/pcl.bash
#RUN bash $SCRIPTS_DIR/pcl.bash

# Project build script
COPY scripts/build.bash $SCRIPTS_DIR/__build__.bash
