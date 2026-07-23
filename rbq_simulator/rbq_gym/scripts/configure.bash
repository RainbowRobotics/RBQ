#!/usr/bin/env bash
set -e

export REMOVE_TMP=true

export LIBS_DIR="$PWD/3rdparty"
export TMP_DIR="$LIBS_DIR/tmp"

export ISAACGYM_DIR="$LIBS_DIR/isaacgym"
export RSL_RL_VERSION="1.0.2"
export RSL_RL_DIR="$LIBS_DIR/rsl_rl-$RSL_RL_VERSION"

export CONDA_DIR="$LIBS_DIR/miniconda3"
export CONDA_INSTALLER="Miniconda3-latest-Linux-x86_64.sh"
export CONDA_LINK="https://repo.anaconda.com/miniconda/$CONDA_INSTALLER"
export CONDA_PROFILE="$CONDA_DIR/etc/profile.d/conda.sh"

export ENV_NAME="rbq-gym"
export ENV_FILE="dependencies.yaml"
