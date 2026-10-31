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

# On a robot PC set up by the 24.04 pc_setting/1.bash (RT threads isolated on an E-core cluster),
# keep vision on the E-cores left over. Vision forks every vision app, and they inherit this.
# Measured (i14, trot_stairs + all vision apps): spread over the P-cores the package hit 98-110 C
# with ~75 throttle events/s; on E-cores 16-21 the P-cores idle and throttling nearly stops.
# No pinning anywhere else - sim, a separate vision PC, a non-hybrid CPU, and robots on the older
# recipe whose isolcpus= are P-cores (e.g. 1,2,3,4 on an i12): those keep running as before.
# Nor with fewer than 4 E-cores left: an i12 keeps 12,14,15 only (13 is isolated), and in trot_stairs
# vision there sat the package at 100 C with ~59 thermal throttles/s. Unpinned, with every other CPU
# held at its base clock by rbq-rt-cpus (pc_setting/1.bash, same condition): max 92 C, no throttling.
# Unpinned without that cap is worse - the P-cores boost and throttle (~530 per 3 min sitting).
PIN=()
ISOL="$(grep -oE 'isolcpus=[^ ]+' /proc/cmdline | cut -d= -f2)"
if [ "$SIM_MODE" = false ] && [ -n "$ISOL" ] && [ -r /sys/devices/cpu_atom/cpus ]; then
    expand() { local r; for r in ${1//,/ }; do seq "${r%-*}" "${r#*-}"; done; }
    ECORES="$(expand "$(cat /sys/devices/cpu_atom/cpus)" | sort)"
    if [ -z "$(comm -13 <(echo "$ECORES") <(expand "$ISOL" | sort))" ]; then   # every isolated CPU is an E-core
        VISION_CPUS="$(comm -23 <(echo "$ECORES") <(expand "$ISOL" | sort) | sort -n | paste -sd,)"
        if [ "$(echo "$VISION_CPUS" | tr , '\n' | grep -c .)" -ge 4 ]; then
            PIN=(taskset -c "$VISION_CPUS") && echo "$APP_NAME: pinned to E-cores $VISION_CPUS"
        else
            echo "$APP_NAME: only E-cores ${VISION_CPUS:-none} left - not pinned"
        fi
    fi
fi

denied=0

while true; do
    pid=$(pgrep -x "$APP_NAME")
    if [ -z "$pid" ]; then
        # Check for a real NOPASSWD tag on this path: `sudo -n -l <cmd>` only reports
        # entitlement, not whether a password is required.
        if sudo -n -l 2>/dev/null | grep -q "NOPASSWD:.*[ =,]$PWD/$APP_NAME\(,\|$\)"; then
            denied=0
            "${PIN[@]}" sudo -n ./"$APP_NAME" "${CMD_ARGS[@]}"
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
