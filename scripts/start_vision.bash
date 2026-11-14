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
    echo "  --payload <livox|ouster>  Simulator LiDAR: HAL turns its DDS cloud into the LidarFrame SLAM reads. --sim only."
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --sim) CMD_ARGS+=("--sim"); SIM_MODE=true; shift ;;
        -i|--interface) CMD_ARGS+=("--interface" "$2"); shift 2 ;;
        --payload) CMD_ARGS+=("--payload" "$2"); shift 2 ;;
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

# Everything this window prints - FILE_LOG, crash backtraces, the output of the apps it starts - also goes to
# log/console/<start time>-<app>.log: it was on screen only and went with the window (robot 11-04 11:08).
# `script` gives the app a terminal of its own, so the window shows what it did before. At each (re)start files
# older than 30 days go, then the oldest idle ones while the folder is over 1 GB.
# ponytail: one run's file has no size cap - add rotation if an app ever floods its console for hours.
CONSOLE_DIR="$PWD/log/console"
mkdir -p "$CONSOLE_DIR"
CONSOLE_LOG="$CONSOLE_DIR/$(date +%Y-%m-%d_%H%M%S)-$APP_NAME.log"
run_logged() {
    find "$CONSOLE_DIR" -name '*.log' -mtime +30 -delete 2>/dev/null
    while [ "$(du -sm "$CONSOLE_DIR" | cut -f1)" -gt 1024 ]; do
        old=$(find "$CONSOLE_DIR" -name '*.log' -mmin +10 -printf '%T@ %p\n' | sort -n | head -1 | cut -d' ' -f2-)
        [ -n "$old" ] || break
        rm -f "$old"
    done
    echo "=== $(date '+%F %T') start: $*" >> "$CONSOLE_LOG"
    # In the background, so a closed window still ends the app: `script` outlives its terminal's hangup, so the window's
    # SIGHUP no longer reached the app and the next start found it running (#1312). Ending `script` ends the app - its
    # terminal hangs up. `<&0` keeps the window's input (a background job would get /dev/null).
    SHELL=/bin/bash script -qfea -c "$(printf '%q ' "$@")" "$CONSOLE_LOG" <&0 &
    RUN_PID=$!
    trap 'echo "=== $(date "+%F %T") window closed (SIGHUP) - ending it" >> "$CONSOLE_LOG"; kill -TERM "$RUN_PID" 2>/dev/null; wait "$RUN_PID"; exit 129' HUP
    wait "$RUN_PID"
    # How it ended, to the journal and the console log: a crash without a handler left nothing (Tori 11-03~09, apport only)
    local rc=$? sig=
    [ "$rc" -gt 128 ] && sig=" (signal $((rc - 128)) SIG$(kill -l $((rc - 128)) 2>/dev/null))"
    echo "=== $(date '+%F %T') exit $rc$sig: $*" | tee -a "$CONSOLE_LOG"
    return "$rc"
}

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
            run_logged "${PIN[@]}" sudo -n ./"$APP_NAME" "${CMD_ARGS[@]}"
        elif [ "$SIM_MODE" = true ] && [ -t 0 ]; then
            # Simulation runs in a terminal tab, so a tty exists but no NOPASSWD rule does.
            # Fall back to interactive sudo, otherwise this loops forever without starting.
            run_logged sudo ./"$APP_NAME" "${CMD_ARGS[@]}"
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
