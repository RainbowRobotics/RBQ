FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
WORKDIR /workspace

# colcon + rosdep + CycloneDDS RMW (matches docs/software/sdk/ros/quick-start.md).
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions python3-rosdep \
        ros-humble-rmw-cyclonedds-cpp ros-humble-cyclonedds \
    && rm -rf /var/lib/apt/lists/*

# Resolve and install the workspace's package dependencies.
COPY src/ /workspace/src/
RUN apt-get update \
    && rosdep update \
    && rosdep install --from-paths src -y --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# Build the colcon workspace; a failed build fails the image build.
RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install
