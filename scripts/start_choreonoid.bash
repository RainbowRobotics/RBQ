#!/bin/bash

APP_NAME="3rdparty/choreonoid/bin/choreonoid"

print_help() {
    echo "Usage: bash scripts/start_choreonoid.bash [OPTIONS]"
    echo "Options:"
    echo "  --help      Display this help message and exit."
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --sim) SIM_MODE=true; shift ;;
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
    sleep 10
    exit 1
fi

# Check if binary exists
if [ ! -f "$APP_NAME" ]; then
    echo "$APP_NAME application not found."
    echo "Compile it first with: bash scripts/setup.bash"
    sleep 10
    exit 1
fi

function set_terminal_title {
    echo -ne "\033]0;$1\007"
}
set_terminal_title "choreonoid"

# Run loop
while true; do
    pid=$(pgrep -x "$APP_NAME")
    if [ -z "$pid" ]; then
        sudo ./$APP_NAME resources/cnoid/RBQ-10_gym.cnoid
    fi
    sleep 2
done
