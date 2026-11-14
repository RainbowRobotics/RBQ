#!/bin/bash

# Launch the Isaac Sim integration app (rbq_sim/app.py). Plays the role of start_mujoco.bash /
# start_gazebo.bash. Forwards flags (--world, -i/--interface, --headless) straight to app.py.

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPTS_DIR/activate.bash"

# Auto-accept the Omniverse EULA so the kit kernel boots non-interactively (the gnome-terminal
# tab has no one to answer the first-run "Do you accept the EULA?" prompt). See kit_app.check_eula.
export OMNI_KIT_ACCEPT_EULA=YES

# Pin CycloneDDS to loopback so the sim<->Motion link stays on lo (mirrors start_gazebo.bash).
export CYCLONEDDS_URI="file://$SIM_DIR/cyclonedds_sim.xml"

echo -ne "\033]0;isaacsim\007"
echo "Running rbq_sim: python rbq_sim/app.py $*"
cd "$SIM_DIR"
eval "python rbq_sim/app.py $*"
