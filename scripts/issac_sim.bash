#!/bin/bash

# Run the RBQ core SW and the Isaac Sim simulator in terminal tabs.

RVIZ_ARGS=()
VISION_ENABLED=false
ROBOT_ARGS=()
IFACE_ARGS=()
WORLD_ARGS=()
HEADLESS_ARGS=()
LIDAR_ARGS=()
LIDAR_SIM_ARGS=()

print_help() {
    echo "Usage: bash scripts/issac_sim.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                    Display this help message and exit."
    echo "  --vision                  Run vision modules."
    echo "  --rviz                    Also open RViz (robot model, TF, cameras, LiDAR)."
    echo "  -i, --interface <name>    CycloneDDS network interface. DEFAULT lo"
    echo "  --world <name>            rbq_test(default, the MuJoCo/Gazebo test field)/empty (flat);"
    echo "                            grid/office/hospital/simple_room/warehouse/depot/warehouse_shelves/"
    echo "                            warehouse_forklifts (Isaac built-in USD);"
    echo "                            stairs_course/two_floor_office/terrain_park (repo-local). See worlds.py."
    echo "  --headless                Run Isaac Sim without its GUI."
    echo "  --livox | --ouster        Simulate a LiDAR (Livox Mid-360 pair / Ouster OS1); Vision's HAL reads it."
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --vision) VISION_ENABLED=true; shift ;;
        --rviz) RVIZ_ARGS=("--rviz"); shift ;;
        -i|--interface) IFACE_ARGS=("--interface" "$2"); shift 2 ;;
        --world) WORLD_ARGS=("--world" "$2"); shift 2 ;;
        --headless) HEADLESS_ARGS=("--headless"); shift ;;
        --livox|--ouster) [ ${#LIDAR_SIM_ARGS[@]} -gt 0 ] && [ "${LIDAR_SIM_ARGS[0]}" != "$1" ] && { echo "[sim] --livox and --ouster cannot be used together: one LiDAR at a time."; exit 1; }; LIDAR_SIM_ARGS=("$1"); LIDAR_ARGS=("--payload" "${1#--}"); shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

ISAAC_ARGS=("${WORLD_ARGS[@]}" "${IFACE_ARGS[@]}" "${HEADLESS_ARGS[@]}" "${LIDAR_SIM_ARGS[@]}" "${RVIZ_ARGS[@]}")
[ "$VISION_ENABLED" = "true" ] && ISAAC_ARGS+=("--vision")

gnome-terminal --tab --title="Motion"   -- bash -i -c "bash scripts/start_motion.bash --sim ${ROBOT_ARGS[@]} ${IFACE_ARGS[@]}"
sleep 1
if [ "$VISION_ENABLED" = "true" ]; then
    gnome-terminal --tab --title="mediamtx" -- bash -i -c "bash scripts/start_mediamtx.bash"
    gnome-terminal --tab --title="Vision" -- bash -i -c "bash scripts/start_vision.bash --sim ${IFACE_ARGS[@]} ${LIDAR_ARGS[@]}"
    gnome-terminal --tab --title="IsaacSim" -- bash -i -c "bash rbq_simulator/rbq_sim/scripts/run.bash ${ISAAC_ARGS[@]}"
else
    gnome-terminal --tab --title="IsaacSim" -- bash -i -c "bash rbq_simulator/rbq_sim/scripts/run.bash ${ISAAC_ARGS[@]}"
fi

gnome-terminal --tab --title="GUI" -- bash -i -c "bash scripts/start_gui.bash"
