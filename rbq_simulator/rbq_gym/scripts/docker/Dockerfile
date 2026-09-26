FROM nvidia/cuda:12.1.1-runtime-ubuntu22.04
# Standalone image, GPU only. See ../../README.md "Docker" for the full rationale.
# runtime, not devel: gymtorch's JIT step only ever invokes c++ (build-essential), never
# nvcc, so the CUDA toolkit/headers in -devel are dead weight (~7GB, verified by testing
# both against a real training run).

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    wget curl ca-certificates tar build-essential git \
    libx11-6 libxext6 libgl1 libglx0 libegl1 libopengl0 \
    && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV PYTHONUNBUFFERED=1

# Bake conda+IsaacGym+rsl_rl outside the project dir so run.bash's runtime bind-mount
# of rbq_gym/ can't shadow it. configure.bash honors a pre-set LIBS_DIR.
ENV LIBS_DIR=/opt/rbq-gym/3rdparty

# One level deeper than "rbq_gym" on purpose: GYM_ASSET_DIR resolves '../../resources'
# relative to this path, so it must mirror the real repo layout. See README.
WORKDIR /workspace/rbq_simulator/rbq_gym
COPY dependencies.yaml setup.py ./
COPY scripts/configure.bash scripts/setup.bash scripts/
COPY rbq_gym/__init__.py rbq_gym/__init__.py
RUN bash scripts/setup.bash && chmod -R a+rX /opt/rbq-gym
