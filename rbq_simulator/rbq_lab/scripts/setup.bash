#!/usr/bin/env bash
set -euo pipefail

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAB_DIR="$(dirname "$SCRIPTS_DIR")"
CONFIG_FILE="$SCRIPTS_DIR/configure.bash"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ Configuration file not found: $CONFIG_FILE"
    exit 1
fi
source "$CONFIG_FILE"

# Conda
if [ ! -d "$CONDA_DIR" ]; then
    mkdir -p $TMP_DIR && cd $TMP_DIR
    rm -rf $CONDA_INSTALLER
    echo "[INFO] Downloading conda: $CONDA_INSTALLER ..."
    wget $CONDA_LINK
    echo "[INFO] Installing conda to: $CONDA_DIR ..."
    bash $CONDA_INSTALLER -b -p $CONDA_DIR
    if [[ "${REMOVE_TMP,,}" == "true" ]]; then
        echo "[INFO] Cleaning up temporary files..."
        cd "$LIBS_DIR" && rm -rf "$TMP_DIR"
    fi
fi
echo "✅ Conda installed to $CONDA_DIR"

if [ ! -f "$CONDA_PROFILE" ]; then
    echo "❌ Conda profile not exist: $CONDA_PROFILE"
    exit 1
fi
source "$CONDA_DIR/etc/profile.d/conda.sh"
export CONDA_OVERRIDE_TOS=1

ENV_FILE="$LAB_DIR/$ENV_FILE"
if [ ! -f "$ENV_FILE" ]; then
    echo "❌ Conda environment description file not exist: $ENV_FILE"
    exit 1
fi
if conda info --envs | awk '{print $1}' | grep -Fxq "$ENV_NAME"; then
    echo "⚠️  Conda environment '$ENV_NAME' already exists. Skipping creation."
else
    echo "Creating conda environment $ENV_NAME from $ENV_FILE ..."
    conda env create -f "$ENV_FILE" --name "$ENV_NAME"
    echo "✅ Conda environment $ENV_NAME creation is complete."
fi
conda activate "$ENV_NAME"

if [ ! -d "3rdparty/IsaacLab" ]; then
    cd 3rdparty
    git clone https://github.com/isaac-sim/IsaacLab.git -b v2.3.2
    cd IsaacLab && ./isaaclab.sh --install
fi

if [ ! -f "$LAB_DIR/setup.py" ]; then
    echo "❌ Root project setup.py not exist in: $LAB_DIR"
    exit 1
fi
pip install -e "$LAB_DIR"

cd 
echo "✅ Install complete."

