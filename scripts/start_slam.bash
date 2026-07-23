#!/bin/bash

APP_NAME="SLAMNAV_3D"
APP_PATH="bin"
CMD_ARGS=()

echo -ne "\033]0;$APP_NAME\007"

print_help() {
    echo "Usage: bash scripts/start_sim.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                    Display this help message and exit."
    echo "  -i, --interface <name>    CycloneDDS network interface. DEFAULT lo"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        -i|--interface) CMD_ARGS+=("--interface" "$2"); shift 2 ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

if pgrep -x "$APP_NAME" > /dev/null; then
    echo "$APP_NAME is already running. Please close it before starting a new instance."
    sleep 10
    exit 1
fi

if [ ! -f "$APP_PATH/$APP_NAME" ]; then
    echo "$APP_NAME application not found."
    echo "Compile it first."
    sleep 10
    exit 1
fi

cd $APP_PATH

while true; do
    pid=$(pgrep -x "$APP_NAME")
    if [ -z "$pid" ]; then
        ./"$APP_NAME" "${CMD_ARGS[@]}"
    fi
    sleep 2
done
