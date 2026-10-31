#!/bin/bash

URL="http://127.0.0.1:8080"

print_help() {
    echo "Usage: bash scripts/start_gui.bash [OPTIONS]"
    echo "Waits for Network on 127.0.0.1:8080, then opens the web UI in the default browser."
    echo "Options:"
    echo "  --help      Display this help message and exit."
    echo "Sign in with the robot's account (default rb / rainbow2026)."
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

# Motion starts Network, so the port comes up some time after the simulation does - however long
# that takes, open the page the moment it answers.
until (exec 3<>/dev/tcp/127.0.0.1/8080) 2>/dev/null; do
    sleep 0.5
done

echo "[web-ui] $URL (sign in: rb / rainbow2026)"
if ! xdg-open "$URL" >/dev/null 2>&1; then
    echo "[web-ui] Could not open a browser here - open $URL by hand."
fi
