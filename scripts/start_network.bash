#!/bin/bash

APP_NAME="Network"
SIM_MODE=false

print_help() {
    echo "Usage: bash scripts/start_network.bash [OPTIONS]"
    echo "Options:"
    echo "  --help      Display this help message and exit."
    echo "  --sim       Run in simulator mode."
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
if [ ! -f "bin/$APP_NAME" ]; then
    echo "$APP_NAME application not found."
    echo "Compile it first with: bash scripts/docker/run.bash"
    sleep 10
    exit 1
fi

function set_terminal_title {
    echo -ne "\033]0;$1\007"
}
set_terminal_title "$APP_NAME"

cd bin

# Run loop
while true; do
    pid=$(pgrep -x "$APP_NAME")
    if [ -z "$pid" ]; then
        if [ "$SIM_MODE" = true ]; then
            sudo ./"$APP_NAME" -s
        else
            sudo ./"$APP_NAME"
        fi
    fi
    sleep 2
done
