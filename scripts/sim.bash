#!/bin/bash

IFACE_ARGS=()
ROBOT_ARGS=()
PAYLOAD_ARGS=()
SIM_ARGS=()
MOTION_ENABLED=true
SIM_ENABLED=true
VISION_ENABLED=false
SLAM_ENABLED=false
RVIZ_ENABLED=false
GUI_ENABLED=true

print_help() {
    echo "Usage: bash scripts/sim.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                  Display this help message and exit."
    echo "  -i, --interface <name>  CycloneDDS network interface. DEFAULT lo"
    echo "  -r, --robot <flag>      Robot variant flag: --rb1 | --wheel | --lims_ex (per README; passed through to start_motion)"
    echo "  -p, --payload <name>    Attach a payload (ptz|livox|ouster) --sim only."
    echo "  --no-motion             Skip the Motion."
    echo "  --no-sim                Skip the Mujoco simulator."
    echo "  --vision                Run vision modules."
    echo "  --slam                  Run SLAM."
    echo "  --rviz                  Run Rviz."
    echo "  --no-gui                Skip the GUI."
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        -i|--interface) IFACE_ARGS=("--interface" "$2"); shift 2 ;;
        -r|--robot)     ROBOT_ARGS=("--$2"); shift 2 ;;
        -p|--payload)   case "$2" in lidar|livox|ouster) SIM_ARGS+=("--payload" "$2") ;; *) PAYLOAD_ARGS+=("--payload" "$2") ;; esac; shift 2 ;;
        --no-motion)    MOTION_ENABLED=false; shift ;;
        --no-sim)       SIM_ENABLED=false; shift ;;
        --vision)       VISION_ENABLED=true; SIM_ARGS+=("--vision"); shift ;;
        --slam)         SLAM_ENABLED=true; shift ;;
        --rviz)         RVIZ_ENABLED=true; shift ;;
        --no-gui)       GUI_ENABLED=false; shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

if [ "$MOTION_ENABLED" = "true" ]; then
    gnome-terminal --tab --title="Motion"   -- bash -i -c 'bash scripts/start_motion.bash --sim "$@"' bash "${IFACE_ARGS[@]}" "${ROBOT_ARGS[@]}" "${PAYLOAD_ARGS[@]}"
fi
if [ "$SIM_ENABLED" = "true" ]; then
    gnome-terminal --tab --title="Mujoco"   -- bash -i -c 'bash scripts/start_mujoco.bash "$@"' bash "${IFACE_ARGS[@]}" "${SIM_ARGS[@]}" "${ROBOT_ARGS[@]}" "${PAYLOAD_ARGS[@]}"
fi
if [ "$VISION_ENABLED" = "true" ]; then
    gnome-terminal --tab --title="mediamtx" -- bash -i -c 'bash scripts/start_mediamtx.bash'
    gnome-terminal --tab --title="Vision"   -- bash -i -c 'bash scripts/start_vision.bash --sim "$@"' bash "${IFACE_ARGS[@]}"
fi
if [ "$SLAM_ENABLED" = "true" ]; then
    gnome-terminal --tab --title="SLAM"     -- bash -i -c 'bash scripts/start_slam.bash "$@"' bash "${IFACE_ARGS[@]}"
fi
if [ "$RVIZ_ENABLED" = "true" ]; then
    gnome-terminal --tab --title="RBQ Rviz" -- bash -i -c 'bash scripts/start_rviz.bash "$@"' bash "${IFACE_ARGS[@]}" "${ROBOT_ARGS[@]}"
fi
if [ "$GUI_ENABLED" = "true" ]; then
    gnome-terminal --tab --title="RBQ GUI"  -- bash -i -c 'bash scripts/start_gui.bash --sim "$@"' bash "${ROBOT_ARGS[@]}"
fi