ARG UBUNTU_VERSION=22.04
FROM ubuntu:${UBUNTU_VERSION}

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace

# Plain pip, like a customer: cyclonedds 11.0.1 ships a self-contained x86_64 wheel
# for CPython 3.10-3.13, so no CycloneDDS C build. run.bash builds this on 22.04
# (3.10) and 24.04 (3.12). PIP_BREAK_SYSTEM_PACKAGES: 24.04's pip refuses a system
# install without it (PEP 668); 22.04's pip ignores it.
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN apt-get update && apt-get install -y --no-install-recommends python3 python3-pip

# No .git in the build context, so hatch-vcs installs the fallback version 0.0.0
# here; release versions are pinned via SETUPTOOLS_SCM_PRETEND_VERSION.
COPY pyproject.toml README.md /workspace/rbq_sdk_py/
COPY rbq_sdk_py/ /workspace/rbq_sdk_py/rbq_sdk_py/
RUN pip3 install --no-cache-dir /workspace/rbq_sdk_py

# Import + DDS start on lo: proves the cyclonedds wheel's bundled C library loads and
# the SDK's DDS config XML parses under this cyclonedds version.
RUN python3 -c "from rbq_sdk_py.core.channel import ChannelFactoryInitialize; ChannelFactoryInitialize(0, 'lo'); print('rbq_sdk_py import + DDS init OK')"
