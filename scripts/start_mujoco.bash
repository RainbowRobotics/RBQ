#!/bin/bash

APP_NAME="Mujoco"
APP_PATH="bin"
CMD_ARGS=()

echo -ne "\033]0;$APP_NAME\007"

print_help() {
    echo "Usage: bash scripts/start_mujoco.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                    Display this help message and exit."
    echo "  --rb1                     RB1 Arm enabled. DEFAULT false"
    echo "  --lims_ex                 LIMS_EX enabled. DEFAULT false"
    echo "  --wheel                   RBQ WHEEL enabled. DEFAULT false"
    echo "  --vision                  Vision enabled. DEFAULT false"
    echo "  -i, --interface <name>    CycloneDDS network interface. DEFAULT lo"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --rb1) CMD_ARGS+=("--rb1"); shift ;;
        --lims_ex) CMD_ARGS+=("--lims_ex"); shift ;;
        --wheel) CMD_ARGS+=("--wheel"); shift ;;
        --vision) CMD_ARGS+=("--vision"); shift ;;
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
