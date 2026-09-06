FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV NO_AT_BRIDGE=1

WORKDIR /workspace

ENV SCRIPTS_DIR="/workspace/scripts"
RUN mkdir -p $SCRIPTS_DIR

COPY scripts/debian-dep.bash $SCRIPTS_DIR/debian-dep.bash
RUN bash $SCRIPTS_DIR/debian-dep.bash
RUN locale-gen en_US.UTF-8
RUN dpkg-reconfigure locales

COPY temp/rbq_sdk_cpp/scripts/build_dds.bash $SCRIPTS_DIR/build_dds.bash
RUN bash $SCRIPTS_DIR/build_dds.bash

COPY temp/rbq_sdk_cpp/ /workspace/rbq_sdk_cpp/
RUN cd /workspace/rbq_sdk_cpp && bash scripts/build.bash --no-cache

COPY src/ /workspace/src/
COPY CMakeLists.txt /workspace/CMakeLists.txt
COPY scripts/build.bash $SCRIPTS_DIR/build.bash
RUN cd /workspace && bash $SCRIPTS_DIR/build.bash --no-cache

COPY resources/ /workspace/resources/
ENTRYPOINT ["/workspace/bin/rbq_mujoco"]