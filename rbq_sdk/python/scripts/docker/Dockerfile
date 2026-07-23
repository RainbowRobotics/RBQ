FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace

# Toolchain to build CycloneDDS + Python build/runtime.
RUN apt-get update && apt-get install -y --no-install-recommends \
        cmake ninja-build build-essential git wget ca-certificates pkg-config \
        python3 python3-pip python3-dev

# CycloneDDS C library 0.10.2 — required to install the cyclonedds python pkg.
# (apt-get update again so build_dds.bash's libfastcdr/libfastrtps install works.)
COPY temp/build_dds.bash /tmp/build_dds.bash
RUN apt-get update && bash /tmp/build_dds.bash && ldconfig
ENV CYCLONEDDS_HOME=/usr/local

# Install the Python SDK and its deps (cyclonedds==0.10.2, numpy, opencv-python).
# No .git in the build context, so hatch-vcs installs the fallback version 0.0.0
# here; release versions are pinned to the framework version (src/rcl/version.txt
# via SETUPTOOLS_SCM_PRETEND_VERSION in rbq_sdk/python/scripts/release.bash).
COPY pyproject.toml README.md /workspace/rbq_sdk_py/
COPY rbq_sdk_py/ /workspace/rbq_sdk_py/rbq_sdk_py/
RUN pip3 install --no-cache-dir /workspace/rbq_sdk_py

# Import-test: validates deps resolved and the package imports cleanly.
RUN python3 -c "import rbq_sdk_py; print('rbq_sdk_py import OK')"
