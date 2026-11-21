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
    echo "  --rba-1                   RBA-1 Arm enabled. DEFAULT false"
    echo "  --lims_ex                 LIMS_EX enabled. DEFAULT false"
    echo "  --wheel                   RBQ WHEEL enabled. DEFAULT false"
    echo "  --platform <name>         Robot platform profile (rbq10 ...). --sim only. DEFAULT rbq10"
    echo "  --rbh                     Humanoid (RBH): create its shared memory, Network serves port 8000. DEFAULT false"
    echo "  --payload <name>          Attach a sim payload (ptz). --sim only."
    echo "  --qc body                 Body QC (r-pc, CAN, lan2can) before assembly - no motion apps; run it from the app (Maintenance > Body QC)."
    echo "  -i, --interface <name>    CycloneDDS network interface (forwarded to Network). DEFAULT lo"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --sim) CMD_ARGS+=("--sim"); shift ;;
        --rb1) CMD_ARGS+=("--arm" "rb1"); shift ;;
        --rba-1|--rba1) CMD_ARGS+=("--arm" "rba1"); shift ;;
        --lims_ex) CMD_ARGS+=("--arm" "lims_ex"); shift ;;
        --wheel) CMD_ARGS+=("--wheel"); shift ;;
        --platform) CMD_ARGS+=("--platform" "$2"); shift 2 ;;
        --rbh) CMD_ARGS+=("--rbh"); shift ;;
        --qc) CMD_ARGS+=("--qc" "$2"); shift 2 ;;
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
        (cd "$DIR" && run_logged sudo ./"$APP_NAME" "${CMD_ARGS[@]}")
    fi
    sleep 2
done
