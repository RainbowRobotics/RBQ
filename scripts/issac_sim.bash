#!/bin/bash

# Isaac Sim version of scripts/sim.bash / scripts/gazebo_sim.bash. Only difference is launching
# the Isaac Sim integration app (rbq_simulator/rbq_sim) instead of MuJoCo/Gazebo; the
# Motion/Vision/mediamtx/GUI/ROS tab layout and options are identical.

ROS_ENABLED=false
VISION_ENABLED=false
ROBOT_ARGS=()
IFACE_ARGS=()
WORLD_ARGS=()
HEADLESS_ARGS=()

print_help() {
    echo "Usage: bash scripts/issac_sim.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                    Display this help message and exit."
    echo "  --vision                  Run vision modules."
    echo "  --ros                     Run ROS2 driver."
    echo "  --rb1 / --lims_ex / --wheel  (rbq10 only - accepted for compatibility, ignored)"
    echo "  -i, --interface <name>    CycloneDDS network interface. DEFAULT lo"
    echo "  --world <name>            empty(default)/warehouse/depot/grid/office/hospital/simple_room/"
    echo "                            warehouse_shelves/warehouse_forklifts (Isaac built-in USD);"
    echo "                            stairs_course/two_floor_office/terrain_park (repo-local);"
    echo "                            raceway/maze = flat ground (no Isaac default yet). See worlds.py."
    echo "  --headless                Run Isaac Sim without its GUI."
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --vision) VISION_ENABLED=true; shift ;;
        --ros) ROS_ENABLED=true; shift ;;
        --rb1) ROBOT_ARGS+=("--rb1"); shift ;;
        --lims_ex) ROBOT_ARGS+=("--lims_ex"); shift ;;
        --wheel) ROBOT_ARGS+=("--wheel"); shift ;;
        -i|--interface) IFACE_ARGS=("--interface" "$2"); shift 2 ;;
        --world) WORLD_ARGS=("--world" "$2"); shift 2 ;;
        --headless) HEADLESS_ARGS=("--headless"); shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

# Isaac tab runs rbq_sim/scripts/run.bash, which only understands --world / --interface /
# --headless (rbq10 only). ROBOT_ARGS (--rb1/--lims_ex/--wheel) go to Motion/GUI like Gazebo.
ISAAC_ARGS=("${WORLD_ARGS[@]}" "${IFACE_ARGS[@]}" "${HEADLESS_ARGS[@]}")

gnome-terminal --tab --title="Motion"   -- bash -i -c "bash scripts/start_motion.bash --sim ${ROBOT_ARGS[@]} ${IFACE_ARGS[@]}"
sleep 1
if [ "$VISION_ENABLED" = "true" ]; then
    gnome-terminal --tab --title="mediamtx" -- bash -i -c "bash scripts/start_mediamtx.bash"
    gnome-terminal --tab --title="Vision" -- bash -i -c "bash scripts/start_vision.bash --sim ${IFACE_ARGS[@]}"
    gnome-terminal --tab --title="IsaacSim" -- bash -i -c "bash rbq_simulator/rbq_sim/scripts/run.bash ${ISAAC_ARGS[@]}"
else
    gnome-terminal --tab --title="IsaacSim" -- bash -i -c "bash rbq_simulator/rbq_sim/scripts/run.bash ${ISAAC_ARGS[@]}"
fi

if [ "$ROS_ENABLED" = "true" ]; then
    gnome-terminal --tab --title="rbq_ros_driver " -- bash -i -c "sudo bash scripts/start_ros_driver.bash --sim"
    gnome-terminal --tab --title="rbq_description" -- bash -i -c "bash scripts/start_rviz.bash --sim"
fi

gnome-terminal --tab --title="GUI" -- bash -i -c "bash scripts/start_gui.bash"
