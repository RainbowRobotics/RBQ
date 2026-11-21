import os

# rbq_sim = Isaac Sim integration simulator (peer of rbq_mujoco / rbq_gazebo).
# Reuses rbq_lab's Isaac Sim install, conda env (rbq-lab) and robot asset/cfg.
SIM_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))   # rbq_simulator/rbq_sim
# Shared robot assets live at the repo-root resources/ (the same dir rbq_lab uses), reached two
# levels up: rbq_simulator/rbq_sim -> rbq_simulator -> repo root. No symlink needed.
SIM_ASSET_DIR = os.path.join(SIM_ROOT_DIR, "..", "..", "resources")
