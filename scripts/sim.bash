#!/bin/bash

SIM_TYPE="mujoco"
ROS_ENABLED=false
VISION_ENABLED=false
ROBOT_SUFFIX=""

print_help() {
    echo "Usage: bash scripts/sim.bash [OPTIONS]"
    echo "Options:"
    echo "  --help          Display this help message and exit."
    echo "  --mujoco        Use MuJoCo simulator. (DEFAULT)"
    echo "  --choreonoid    Use Choreonoid simulator."
    echo "  --vision        Run vision modules."
    echo "  --ros           Run ROS2 driver."
    echo "  --rb1           RB1 Arm enabled. DEFAULT false"
    echo "  --lims_ex       LIMS_EX enabled. DEFAULT false"
    echo "  --wheel         RBQ WHEEL enabled. DEFAULT false"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --mujoco) SIM_TYPE="mujoco"; shift ;;
        --choreonoid) SIM_TYPE="choreonoid"; shift ;;
        --vision) VISION_ENABLED=true; shift ;;
        --ros) ROS_ENABLED=true; shift ;;
        --rb1) ROBOT_SUFFIX="--rb1"; shift ;;
        --lims_ex) ROBOT_SUFFIX="--lims_ex"; shift ;;
        --wheel) ROBOT_SUFFIX="--wheel"; shift ;;

        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

# Launch core components
gnome-terminal --tab --title="Motion"   -- bash -i -c "bash scripts/start_motion.bash --sim $ROBOT_SUFFIX"
gnome-terminal --tab --title="Network"  -- bash -i -c "bash scripts/start_network.bash --sim"
sleep 1
if [ "$SIM_TYPE" = "mujoco" ]; then
    if [ "$VISION_ENABLED" = "true" ]; then
        gnome-terminal --tab --title="Vision"   -- bash -i -c "bash scripts/start_vision.bash --sim"
        gnome-terminal --tab --title="Mujoco" -- bash -i -c "bash scripts/start_mujoco.bash --vision $ROBOT_SUFFIX"
    else
        gnome-terminal --tab --title="Mujoco" -- bash -i -c "bash scripts/start_mujoco.bash $ROBOT_SUFFIX"
    fi
else
    gnome-terminal --tab --title="Choreonoid" -- bash -i -c "bash scripts/start_choreonoid.bash $ROBOT_SUFFIX"
fi

if [ "$ROS_ENABLED" = "true" ]; then
    gnome-terminal --tab --title="rbq_driver"     -- bash -i -c "bash scripts/start_ros_driver.bash --sim"
    gnome-terminal --tab --title="rbq_description" -- bash -i -c "bash scripts/start_rviz.bash"
fi

gnome-terminal --tab --title="GUI"      -- bash -i -c "bash scripts/start_gui.bash --sim $ROBOT_SUFFIX"
