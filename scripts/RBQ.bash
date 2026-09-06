#!/bin/bash

# Exit if executed with sudo
if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

# Default CycloneDDS interface. Edit this value (or pass -i <name>) to change
# which NIC Motion/Network bind to — "lo" keeps the robot isolated from other
# hosts on the same LAN.
IFACE_ARGS=("--interface" "enp45s0")

print_help() {
    echo "Usage: bash RBQ.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                    Display this help message and exit."
    echo "  -i, --interface <name>    CycloneDDS network interface (forwarded to Motion and Vision). DEFAULT lo"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        -i|--interface) IFACE_ARGS=("--interface" "$2"); shift 2 ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

path="$HOME/rbq_ws"
cd "$path" || { echo "Cannot cd to $path"; exit 1; }

if [[ -z "$DISPLAY" && -z "$WAYLAND_DISPLAY" ]]; then
    TERM_EMU="setsid"
elif command -v gnome-terminal >/dev/null 2>&1; then
    TERM_EMU="gnome-terminal"
elif command -v ptyxis >/dev/null 2>&1; then
    TERM_EMU="ptyxis"
else
    TERM_EMU="setsid"
fi

open_tab() {
    local title="$1" cmd="$2"; shift 2
    case "$TERM_EMU" in
        gnome-terminal)
            gnome-terminal --tab --working-directory="$path" -- bash -i -c "$cmd" bash "$@" ;;
        ptyxis)
            ptyxis --tab --working-directory="$path" -- bash -i -c "$cmd" bash "$@" ;;
        setsid)
            setsid bash -i -c "$cmd" bash "$@" </dev/null & echo "$title started (pid $!)" ;;
    esac
}

sleep 5
open_tab "Motion" 'bash scripts/start_motion.bash "$@"' "${IFACE_ARGS[@]}"

sleep 5
open_tab "mediamtx" 'bash scripts/start_mediamtx.bash'

sleep 5
open_tab "Vision" 'bash scripts/start_vision.bash "$@"' "${IFACE_ARGS[@]}"

exit
