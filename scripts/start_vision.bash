#!/bin/bash

APP_NAME="Vision"
APP_PATH="bin-$(uname -m)"
# The nightly ships bin-x86_64/ and bin-aarch64/ side by side; pick this machine's.
# build.bash (host, devcontainer, or via docker/run.bash) always writes bin-<arch>/
# directly - fail loudly rather than falling through to a confusing "binary not found".
if [ ! -d "$APP_PATH" ]; then
    echo "ERROR: $APP_PATH not found - this build carries no binaries for $(uname -m)." >&2
    exit 1
fi
CMD_ARGS=()
SIM_MODE=false

echo -ne "\033]0;$APP_NAME\007"

print_help() {
    echo "Usage: bash scripts/start_vision.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                    Display this help message and exit."
    echo "  --sim                     Run in simulator mode."
    echo "  -i, --interface <name>    CycloneDDS network interface. DEFAULT lo"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --sim) CMD_ARGS+=("--sim"); SIM_MODE=true; shift ;;
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

denied=0

while true; do
    pid=$(pgrep -x "$APP_NAME")
    if [ -z "$pid" ]; then
        # Check for a real NOPASSWD tag on this path: `sudo -n -l <cmd>` only reports
        # entitlement, not whether a password is required.
        if sudo -n -l 2>/dev/null | grep -q "NOPASSWD:.*[ =,]$PWD/$APP_NAME\(,\|$\)"; then
            denied=0
            sudo -n ./"$APP_NAME" "${CMD_ARGS[@]}"
        elif [ "$SIM_MODE" = true ] && [ -t 0 ]; then
            # Simulation runs in a terminal tab, so a tty exists but no NOPASSWD rule does.
            # Fall back to interactive sudo, otherwise this loops forever without starting.
            sudo ./"$APP_NAME" "${CMD_ARGS[@]}"
        else
            denied=$((denied + 1))
            if [ "$denied" = 5 ] || [ $((denied % 30)) = 0 ]; then
                echo "$APP_NAME: no NOPASSWD sudo rule for $PWD/$APP_NAME - cannot start."
                echo "$APP_NAME: grant it once with:"
                echo "  echo \"\$USER ALL=(ALL) NOPASSWD: $PWD/$APP_NAME\" | sudo tee /etc/sudoers.d/rbq-\$USER >/dev/null && sudo chmod 0440 /etc/sudoers.d/rbq-\$USER"
            fi
        fi
    fi
    sleep 2
done
