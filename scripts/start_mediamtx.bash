#!/bin/bash

APP_NAME="mediamtx"
APP_PATH="$PWD/bin"
APP_ARGS="$PWD/configs/mediamtx.yml"

print_help() {
    echo "Usage: bash scripts/start_mediamtx.bash [OPTIONS]"
    echo "Options:"
    echo "  --help      Display this help message and exit."
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        *) echo "[ERROR] Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

echo -ne "\033]0;$APP_NAME\007"
if [ "$EUID" -eq 0 ]; then
    echo "[ERROR] Do not run this script with sudo. Exiting..."
    exit 1
fi
if pgrep -x "$APP_NAME" > /dev/null; then
    echo "[ERROR] $APP_NAME is already running. Please close it before starting a new instance."
    sleep 10
    exit 1
fi
if [ ! -f "$APP_PATH/$APP_NAME" ]; then
    echo "[ERROR] $APP_NAME application not found."
    sleep 10
    exit 1
fi
while true; do
    pid=$(pgrep -x "$APP_NAME")
    if [ -z "$pid" ]; then
        cd "$APP_PATH"
        ./$APP_NAME $APP_ARGS
    fi
    sleep 2
done
