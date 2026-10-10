FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace

RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates gnupg

COPY scripts/docker/toolchain.bash /tmp/toolchain.bash
RUN bash /tmp/toolchain.bash && rm /tmp/toolchain.bash

ENV RUSTUP_HOME=/usr/local/rustup
ENV PATH=/usr/local/cargo/bin:$PATH
ENV ANDROID_HOME=/opt/android-sdk
ENV ANDROID_SDK_ROOT=/opt/android-sdk

ENV HOME=/workspace/cache/home
ENV GRADLE_USER_HOME=/workspace/cache/gradle
ENV CARGO_HOME=/workspace/cache/cargo
ENV npm_config_cache=/workspace/cache/npm
ENV XDG_CACHE_HOME=/workspace/cache/xdg
