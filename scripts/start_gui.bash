#!/bin/bash

APP_NAME="GUI"
APP_PATH="bin"
CMD_ARGS=()

print_help() {
    echo "Usage: bash scripts/start_gui.bash [OPTIONS]"
    echo "Options:"
    echo "  --help      Display this help message and exit."
    echo "  --sim       Run in simulator mode."
    echo "  --rb1       Run in simulator (RB1 Arm attached) mode."
    echo "  --lims_ex   Run in simulator (LIMS_EX Arm attached) mode."
    echo "  --wheel     RBQ WHEEL enabled. DEFAULT false"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --sim) CMD_ARGS+=("--sim"); shift ;;
        --rb1) CMD_ARGS+=("--arm"); shift ;;
        --lims_ex) CMD_ARGS+=("--arm"); shift ;;
        --wheel) CMD_ARGS+=("--wheel"); shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

# Exit if executed with sudo
if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

# Check if already running
if pgrep -x "$APP_NAME" > /dev/null; then
    echo "$APP_NAME is already running. Please close it before starting a new instance."
    sleep 3
    exit 1
fi

# Check if binary exists
if [ ! -f "$APP_PATH/$APP_NAME" ]; then
    echo "$APP_PATH/$APP_NAME application not found."
    echo "Compile it first with: bash scripts/docker/run.bash"
    sleep 3
    exit 1
fi

function set_terminal_title {
    echo -ne "\033]0;$1\007"
}
set_terminal_title "$APP_NAME"

cd "$APP_PATH"

# Run loop
while true; do
    pid=$(pgrep -x "$APP_NAME")
    if [ -z "$pid" ]; then
        ./"$APP_NAME" "${CMD_ARGS[@]}"
    fi
    sleep 2
done
