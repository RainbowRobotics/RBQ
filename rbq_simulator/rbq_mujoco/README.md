# rbq_mujoco

- MuJoCo physics simulator for the RBQ quadruped Robot.

## Layout

- `src/` — simulator sources → `bin/rbq_mujoco`
- `resources/model/` — MJCF models (`default.xml`, `env/`, `rbq/`)
- `scripts/` — build / docker

## Local - Run rbq_mujoco with RBQ Full Stack

Run the full RBQ stack (Motion / Vision / GUI) with the MuJoCo simulator:

```bash
# Clone
git clone https://github.com/RainbowRobotics/RBQ.git
cd RBQ

# Install Dependency (first time only)
cd rbq_simulator/rbq_mujoco && bash scripts/debian-dep.bash 

# Build rbq_mujoco
bash scripts/build.bash

# Run the RBQ stack
cd ../../
bash scripts/sim.bash --vision --no-sim

# Run rbq_mujoco
cd rbq_simulator/rbq_mujoco
./bin/rbq_mujoco -p resources/model/default.xml
```

## Docker - Run rbq_mujoco with RBQ Full Stack

Run the full RBQ stack (Motion / Vision / GUI) with the MuJoCo simulator in Docker:

```bash
# Clone
git clone https://github.com/RainbowRobotics/RBQ.git
cd RBQ

# Run the RBQ stack
bash scripts/sim.bash --vision --no-sim

# Run rbq_mujoco in Docker
cd RBQ/rbq_simulator/rbq_mujoco
bash scripts/docker/run.bash
```

- Shares DDS domain 0 / `lo` with the host RBQ stack via `--network host`.