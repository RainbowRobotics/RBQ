#!/bin/bash

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(dirname "$SCRIPTS_DIR")"

echo "[INFO] Cleaning Python build artifacts under: $SIM_DIR"
find "$SIM_DIR" -maxdepth 1 -name "*.egg-info" -type d -exec rm -rf {} +
echo "✔ Removed *.egg-info directories"
find "$SIM_DIR" -type d -name "__pycache__" -print -exec rm -rf {} +
echo "✔ Removed all __pycache__ directories"
