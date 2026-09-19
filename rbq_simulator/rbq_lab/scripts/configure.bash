#!/usr/bin/env bash
set -e

export REMOVE_TMP=true

export LIBS_DIR="$PWD/3rdparty"
export TMP_DIR="$LIBS_DIR/tmp"

export CONDA_DIR="$LIBS_DIR/miniconda3"
export CONDA_INSTALLER="Miniconda3-latest-Linux-x86_64.sh"
export CONDA_LINK="https://repo.anaconda.com/miniconda/$CONDA_INSTALLER"
export CONDA_PROFILE="$CONDA_DIR/etc/profile.d/conda.sh"

export ENV_NAME="rbq-lab"
export ENV_FILE="dependencies.yaml"
