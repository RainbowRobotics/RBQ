#!/usr/bin/env bash
set -e

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(dirname "$SCRIPTS_DIR")"            # rbq_simulator/rbq_sim
SIMULATOR_DIR="$(dirname "$SIM_DIR")"          # rbq_simulator
export SIM_DIR SIMULATOR_DIR
export LAB_DIR="$SIMULATOR_DIR/rbq_lab"

# rbq_sim REUSES rbq_lab's conda env + Isaac Sim install (IsaacLab and Isaac Sim must match).
# CONDA_DIR/ENV_NAME honor env overrides so the Docker image can point at a baked /opt prefix
# (docker/Dockerfile.isaacsim exports CONDA_DIR=/opt/rbq-isaac/miniconda3); on the host they
# default to rbq_lab's local install under rbq_lab/3rdparty.
export CONDA_DIR="${CONDA_DIR:-$LAB_DIR/3rdparty/miniconda3}"
export ENV_NAME="${ENV_NAME:-rbq-lab}"
export CONDA_PROFILE="$CONDA_DIR/etc/profile.d/conda.sh"
