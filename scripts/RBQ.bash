#!/bin/bash

# Exit if executed with sudo
if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

path="$HOME/rbq_ws"

sleep 5
gnome-terminal --tab --title="Motion" -- bash -i -c "cd $path && bash scripts/start_motion.bash"

sleep 5
gnome-terminal --tab --title="Network" -- bash -i -c "cd $path && bash scripts/start_network.bash"

sleep 5
gnome-terminal --tab --title="mediamtx" -- bash -i -c "cd $path && bash scripts/start_mediamtx.bash"

sleep 5
gnome-terminal --tab --title="Vision" -- bash -i -c "cd $path && bash scripts/start_vision.bash"

exit
