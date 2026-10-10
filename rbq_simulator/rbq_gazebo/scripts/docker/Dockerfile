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


COPY src/                /workspace/src/
COPY CMakeLists.txt      /workspace/CMakeLists.txt
COPY resources/          /workspace/resources/
COPY ros2/src/           /workspace/ros2/src/
COPY scripts/build.bash  /workspace/scripts/build.bash
COPY scripts/gazebo.bash /workspace/scripts/gazebo.bash
RUN cd /workspace && bash scripts/build.bash --no-cache

COPY resources/ /workspace/resources/
ENTRYPOINT ["bash", "/workspace/scripts/gazebo.bash"]
