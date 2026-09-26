#!/bin/bash

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GYM_DIR="$(dirname "$SCRIPTS_DIR")"

echo "[INFO] Cleaning Python build artifacts under: $GYM_DIR"

# Remove egg-info
find "$GYM_DIR" -maxdepth 1 -name "*.egg-info" -type d -exec rm -rf {} +
echo "✔ Removed *.egg-info directories"

# Remove all __pycache__ directories recursively
echo "[INFO] Searching for __pycache__ directories..."
find "$GYM_DIR" -type d -name "__pycache__" -print -exec rm -rf {} +
echo "✔ Removed all __pycache__ directories"

# Optional removal of logs directory
LOGS_DIR="$GYM_DIR/logs"
if [ -d "$LOGS_DIR" ]; then
    read -p "Do you want to delete '$LOGS_DIR'? [y/N]: " RESP
    RESP=${RESP,,}  # lowercase
    if [[ "$RESP" == "y" || "$RESP" == "yes" ]]; then
        rm -rf "$LOGS_DIR"
        echo "✔ Removed logs directory"
    else
        echo "Skipped removing logs directory"
    fi
else
    echo "No logs directory found"
fi

clear

