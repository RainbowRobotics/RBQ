#!/bin/bash

# Exit if executed with sudo
if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

SIM_TYPE="choreonoid"

print_help() {
    echo "Usage: ./sim/run.bash [OPTIONS]"
    echo "Options:"
    echo "  --help         Display this help message and exit."
    echo "  --mujoco       Use MuJoCo simulator."
    echo "  --choreonoid   Use Choreonoid simulator. (DEFAULT)"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --mujoco) SIM_TYPE="mujoco"; shift ;;
        --choreonoid) SIM_TYPE="choreonoid"; shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

# Exit if executed with sudo
if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

gnome-terminal --tab --title="Motion" -- bash -i -c "bash scripts/start_motion.bash --sim"
gnome-terminal --tab --title="GUI" -- bash -i -c "bash scripts/start_gui.bash --sim"

if [ "$SIM_TYPE" = "mujoco" ]; then
    gnome-terminal --tab --title="Mujoco  " -- bash -i -c "bash scripts/start_mujoco.bash"
    gnome-terminal --tab --title="Network " -- bash -i -c "bash scripts/start_network.bash --sim"
    gnome-terminal --tab --title="Vision  " -- bash -i -c "bash scripts/start_vision.bash --sim"
else
    gnome-terminal --tab --title="Choreonoid" -- bash -i -c "bash scripts/start_choreonoid.bash"
fi

