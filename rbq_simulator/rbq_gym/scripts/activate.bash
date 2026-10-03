#!/bin/bash

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GYM_DIR="$(dirname "$SCRIPTS_DIR")"
CONFIG_FILE="$SCRIPTS_DIR/configure.bash"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ Configuration file not found: $CONFIG_FILE"
    exit 1
fi
source "$CONFIG_FILE"
source $CONDA_PROFILE
conda activate $ENV_NAME
export LD_LIBRARY_PATH=$CONDA_DIR/envs/$ENV_NAME/lib:$LD_LIBRARY_PATH
