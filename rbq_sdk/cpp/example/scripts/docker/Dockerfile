FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:99
ENV NO_AT_BRIDGE=1

WORKDIR /workspace

# Build context must be the repo root (parent of examples) so rbq_sdk_cpp
# files are included. See scripts/docker/run.bash.

# APT
COPY example/scripts/debian-dep.bash scripts/debian-dep.bash
RUN bash scripts/debian-dep.bash
RUN locale-gen en_US.UTF-8
RUN dpkg-reconfigure locales

# DDS
COPY rbq_sdk_cpp/scripts/build_dds.bash rbq_sdk_cpp/scripts/build_dds.bash
RUN bash rbq_sdk_cpp/scripts/build_dds.bash

# RBQ SDK
COPY rbq_sdk_cpp/ /workspace/rbq_sdk_cpp/
RUN cd /workspace/rbq_sdk_cpp && bash scripts/build.bash --no-cache
