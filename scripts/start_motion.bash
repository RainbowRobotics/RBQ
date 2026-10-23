#!/bin/bash

APP_NAME="Motion"
APP_PATH="bin-$(uname -m)"
# The nightly ships bin-x86_64/ and bin-aarch64/ side by side; pick this machine's.
# build.bash (host, devcontainer, or via docker/run.bash) always writes bin-<arch>/
# directly - fail loudly rather than falling through to a confusing "binary not found".
if [ ! -d "$APP_PATH" ]; then
    echo "ERROR: $APP_PATH not found - this build carries no binaries for $(uname -m)." >&2
    exit 1
fi
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
    echo "  --platform <name>         Robot platform profile (rbq10 ...). --sim only. DEFAULT rbq10"
    echo "  --rbh                     Humanoid (RBH): create its shared memory, Network serves port 8000. DEFAULT false"
    echo "  --payload <name>          Attach a sim payload (ptz). --sim only."
    echo "  -i, --interface <name>    CycloneDDS network interface (forwarded to Network). DEFAULT lo"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --sim) CMD_ARGS+=("--sim"); shift ;;
        --rb1) CMD_ARGS+=("--arm"); shift ;;
        --lims_ex) CMD_ARGS+=("--arm"); shift ;;
        --wheel) CMD_ARGS+=("--wheel"); shift ;;
        --platform) CMD_ARGS+=("--platform" "$2"); shift 2 ;;
        --rbh) CMD_ARGS+=("--rbh"); shift ;;
        --payload) CMD_ARGS+=("--payload" "$2"); shift 2 ;;
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

# Boot slot (rcl/BootSlot.h): with an unexpired flag in /run/rbq-boot-alt (content = expiry, unix seconds) and a second
# build in $APP_PATH-alt (scripts/deploy.bash --alt), Motion starts from there. The app flips the flag through Network and
# ends Motion, so the folder is picked on every (re)start. /run is tmpfs: after a reboot there is no flag and the main slot starts.
BOOT_FLAG=/run/rbq-boot-alt
while true; do
    pid=$(pgrep -x "$APP_NAME")
    if [ -z "$pid" ]; then
        DIR="$APP_PATH"
        # The second slot only with a real NOPASSWD rule for it (same check as start_vision.bash; `sudo -n -l <cmd>` reports entitlement,
        # not the password): without one (old Network) sudo would wait for a password with Network down and no way back.
        EXP=$(head -c 20 "$BOOT_FLAG" 2>/dev/null | tr -dc 0-9)
        if [ -n "$EXP" ] && [ "$EXP" -gt "$(date +%s)" ] && [ -x "$APP_PATH-alt/$APP_NAME" ] \
            && sudo -n -l 2>/dev/null | grep -q "NOPASSWD:.*[ =,]\($PWD/$APP_PATH-alt/$APP_NAME\|ALL\)\(,\|$\)"; then
            DIR="$APP_PATH-alt"
            echo "Starting the second build slot ($DIR)"
        fi
        (cd "$DIR" && sudo ./"$APP_NAME" "${CMD_ARGS[@]}")
    fi
    sleep 2
done
