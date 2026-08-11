#!/bin/bash

APP_NAME="Motion"
APP_PATH="bin"
CMD_ARGS=()

echo -ne "\033]0;$APP_NAME\007"

print_help() {
    echo "Usage: bash scripts/start_motion.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                    Display this help message and exit."
    echo "  --sim                     Run in simulator mode."
    echo "  --rb1                     RB1 Arm enabled. DEFAULT false"
    echo "  --lims_ex                 LIMS_EX enabled. DEFAULT false"
    echo "  --wheel                   RBQ WHEEL enabled. DEFAULT false"
    echo "  -i, --interface <name>    CycloneDDS network interface (forwarded to Network). DEFAULT lo"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --sim) CMD_ARGS+=("--sim"); shift ;;
        --rb1) CMD_ARGS+=("--arm"); shift ;;
        --lims_ex) CMD_ARGS+=("--arm"); shift ;;
        --wheel) CMD_ARGS+=("--wheel"); shift ;;
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
        sudo ./"$APP_NAME" "${CMD_ARGS[@]}"
    fi
    sleep 2
done
