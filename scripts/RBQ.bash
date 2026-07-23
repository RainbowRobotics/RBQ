#!/bin/bash

# Exit if executed with sudo
if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

# Default CycloneDDS interface. Edit this value (or pass -i <name>) to change
# which NIC Motion/Network bind to — "lo" keeps the robot isolated from other
# hosts on the same LAN.
IFACE_ARGS=("--interface" "lo")

print_help() {
    echo "Usage: bash scripts/RBQ.bash [OPTIONS]"
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

sleep 5
gnome-terminal --tab --title="PTP" -- bash -i -c "cd $path && bash scripts/start_ptp_master.bash"

sleep 5
gnome-terminal --tab --title="Motion" --working-directory="$path" -- bash -i -c 'bash scripts/start_motion.bash "$@"' bash "${IFACE_ARGS[@]}"

sleep 5
gnome-terminal --tab --title="mediamtx" -- bash -i -c "cd $path && bash scripts/start_mediamtx.bash"

sleep 5
gnome-terminal --tab --title="Vision" --working-directory="$path" -- bash -i -c 'bash scripts/start_vision.bash "$@"' bash "${IFACE_ARGS[@]}"

exit
