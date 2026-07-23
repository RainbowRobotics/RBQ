#!/usr/bin/env bash
set -euo pipefail

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GYM_DIR="$(dirname "$SCRIPTS_DIR")"
CONFIG_FILE="$SCRIPTS_DIR/configure.bash"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ Configuration file not found: $CONFIG_FILE"
    exit 1
fi
source "$CONFIG_FILE"

# IsaacGym
if [ ! -d "$ISAACGYM_DIR" ]; then
    mkdir -p $TMP_DIR && cd $TMP_DIR
    rm -rf isaac-gym-preview-4
    echo "[INFO] Downloading IsaacGym ..."
    wget https://developer.nvidia.com/isaac-gym-preview-4
    echo "[INFO] Extracting IsaacGym to $ISAACGYM_DIR ..."
    mkdir -p $ISAACGYM_DIR
    tar --strip-components=1 -xf isaac-gym-preview-4 -C $ISAACGYM_DIR
    if [[ "${REMOVE_TMP,,}" == "true" ]]; then
        echo "[INFO] Cleaning up temporary files..."
        cd "$LIBS_DIR" && rm -rf "$TMP_DIR"
    fi
    echo "✅ IsaacGym installed to $ISAACGYM_DIR"
fi

if [ ! -d "$RSL_RL_DIR" ]; then
    mkdir -p $TMP_DIR && cd $TMP_DIR
    rm -rf "v$RSL_RL_VERSION.tar.gz"
    echo "[INFO] Downloading RSL RL library ..."
    wget "https://github.com/leggedrobotics/rsl_rl/archive/refs/tags/v$RSL_RL_VERSION.tar.gz"
    echo "[INFO] Extracting RSL RL to $RSL_RL_DIR ..."
    mkdir -p $RSL_RL_DIR
    tar --strip-components=1 -xf "v$RSL_RL_VERSION.tar.gz" -C $RSL_RL_DIR
    if [[ "${REMOVE_TMP,,}" == "true" ]]; then
        echo "[INFO] Cleaning up temporary files..."
        cd "$LIBS_DIR" && rm -rf "$TMP_DIR"
    fi
    echo "✅ RSL RL library installed to $RSL_RL_DIR"
fi

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

ENV_FILE="$GYM_DIR/$ENV_FILE"
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

echo "Installing IsaacGym $ISAACGYM_DIR ..."
if [ ! -f "$ISAACGYM_DIR/python/setup.py" ]; then
    echo "❌ IsaacGym setup.py not exist in: $ISAACGYM_DIR/python"
    exit 1
fi
pip install -e "$ISAACGYM_DIR/python/."

pip install -e "$RSL_RL_DIR"

if [ ! -f "$GYM_DIR/setup.py" ]; then
    echo "❌ Root project setup.py not exist in: $GYM_DIR"
    exit 1
fi
pip install -e "$GYM_DIR"

echo "✅ Install complete."
