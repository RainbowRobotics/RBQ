# rbq_sim

- Isaac Sim simulator for the RBQ quadruped Robot.
- Talks to the RBQ stack over the same CycloneDDS topics as `rbq_mujoco` / `rbq_gazebo`:
  subscribes `MotionRef_` on `rt/rbq/ref/motion/_0`, publishes `SimInfo_` on `rt/rbq/_sim`.
- Shares `rbq_lab`'s Isaac Sim install, conda env (`rbq-lab`) and robot cfg, so the integration sim
  and the RL trainer use the same robot model.
- Needs an NVIDIA RTX GPU and ~30 GB of disk.

## Layout

- `rbq_sim/` — Python package (`app.py` entry point)
- `worlds/` — repo-local USD worlds and their textures
- `scripts/` — setup / activate / run (mirrors `rbq_lab/scripts`)
- `cyclonedds_sim.xml` — loopback DDS config

| File | Role |
|------|------|
| `rbq_sim/app.py` | Boots Isaac Sim, builds the scene, runs the control loop |
| `rbq_sim/robot.py` | Robot cfg from `rbq_lab` (sitting spawn pose, actuator PD off) and joint-order mapping |
| `rbq_sim/scene.py` | Ground or USD world, robot, foot contact sensors, body IMU |
| `rbq_sim/worlds.py` | World registry: spawn pose + optional USD scene per world |
| `rbq_sim/bridge.py` | `MotionRef_` -> PD torque -> `SimInfo_`, lock-stepped to Motion |
| `rbq_sim/sim_state.py` | Builds `SimInfo_` from the articulation and sensors |
| `rbq_sim/skin.py` | Textured robot skin in the viewport (same textures as MuJoCo) |
| `rbq_sim/idl/_sim_idl.py` | Python IDL for `SimInfo_` / `MotionRef_` (field-for-field with the C++ IDL) |
| `rbq_sim/tools/sim_echo.py` | Decodes `rt/rbq/_sim` to check the wire contract |
| `rbq_sim/tools/build_worlds.py` | Generates the repo-local worlds |

## Worlds

`--world <name>` picks one. Default `empty`.

| Kind | Names | Source |
|---|---|---|
| Flat ground | `empty`, `raceway`, `maze` | none |
| Isaac Sim samples | `grid`, `office`, `hospital`, `simple_room`, `warehouse`, `depot`, `warehouse_shelves`, `warehouse_forklifts` | `{ISAAC_NUCLEUS_DIR}/Environments/` (downloaded on first use) |
| Repo-local | `stairs_course`, `two_floor_office`, `terrain_park` | `worlds/*.usda` (offline) |

| Repo-local world | Contents |
|---|---|
| `stairs_course` | 32 x 16 m hall: stairs 0.10 / 0.15 m rise, 10/15/20 deg ramps, rough blocks, stepping stones, curbs |
| `two_floor_office` | two storeys joined by a switchback staircase (0.15 m x 20), room loops on both floors |
| `terrain_park` | 40 x 40 m outdoor: pyramid stairs, rough heightfield, 15 deg hill, raised plaza with stairs and ramp |

To load your own USD, add an entry to `WORLDS` in `rbq_sim/worlds.py` and run it by name:

```python
"my_site": {"spawn": (0.0, 0.0, 0.30, 0.0),          # x, y, z, yaw [rad]
            "usd": "/path/to/my_site.usd"},           # absolute path, or a file:// / omniverse:// URL
```

The USD replaces the flat ground, so it must carry its own collision floor. The robot spawns sitting
at the spawn pose; raise `z` by the floor height if the floor is not at 0. See `worlds/README.md`.

## Control loop

- Physics runs at 1 kHz on the CPU. One control period is 2 ms = 2 physics steps = one Motion RT tick.
- Each period the sim stamps `SimInfo_` with the sim time. Motion runs one RT tick per new stamp, and
  Network echoes the stamp back in `MotionRef_.time`. The sim steps the next period only after the echo.
- So Motion gets exactly one tick per 2 ms of sim time even when rendering slows the sim below real
  time (headless ~0.85x, with the viewport ~0.6x). Without Motion the sim falls back to real-time pacing.
- Every 5 s the console prints the real-time factor and `lockstep ON acked N/N timeouts 0`;
  `timeouts 0` means no control tick was skipped.
- Before Motion takes over, the sim holds the sitting spawn pose (same as MuJoCo / Gazebo).
- Joint torque is the `MotionRef_` PD (`kp`, `kd`, `pos`, `vel`, `torque`) clamped by the motor T-N
  envelope while motoring (hip <= 83.2 Nm, knee <= 112 Nm); regen is not clipped.
- `SimInfo_` carries the body IMU (specific force, gyro, orientation) and per-foot contacts in
  `[RR, RL, FR, FL]` order.

## Setup

Isaac Sim is installed separately from the top-level `scripts/setup.bash` (run as a normal user;
only the apt step uses sudo):

```bash
bash scripts/install/isaacsim.bash              # apt deps + conda env rbq-lab + Isaac Sim 5.1.0 + IsaacLab
SKIP_APT=1 bash scripts/install/isaacsim.bash   # skip apt if the host already has the deps
```

## Local - Run rbq_sim with RBQ Full Stack

```bash
# Clone
git clone https://github.com/RainbowRobotics/RBQ.git
cd RBQ

# Install Isaac Sim (first time only)
bash scripts/install/isaacsim.bash

# Run Motion + Isaac Sim + web UI in one go
bash scripts/issac_sim.bash
bash scripts/issac_sim.bash --world stairs_course
bash scripts/issac_sim.bash --headless

# Or run the RBQ stack and the simulator separately
bash scripts/start_motion.bash --sim
bash rbq_simulator/rbq_sim/scripts/run.bash --world stairs_course
```

| `scripts/issac_sim.bash` flag | Meaning |
|------|---------|
| `--world <name>` | World to load (default `empty`) |
| `--headless` | Run Isaac Sim without its window |
| `--vision` | Also start mediamtx and the Vision stack |
| `--ros` | Also start the ROS 2 driver and RViz |
| `-i, --interface <name>` | CycloneDDS interface (default `lo`) |

`rbq_sim/scripts/run.bash` takes `--world`, `-i` and `--headless`; other IsaacLab app flags
(e.g. `--device cuda:0`) pass through to `app.py`.

## Check the DDS link

No Motion needed:

```bash
bash rbq_simulator/rbq_sim/scripts/run.bash --headless     # shell 1
source rbq_simulator/rbq_sim/scripts/activate.bash          # shell 2
python -m rbq_sim.tools.sim_echo -i lo
```

Joint positions and the IMU quaternion printing means the wire contract matches.

`lo` has no MULTICAST flag on most hosts. `run.bash` points `CYCLONEDDS_URI` at `cyclonedds_sim.xml`;
set the same for any other DDS client on this host:

```bash
export CYCLONEDDS_URI=file://$PWD/rbq_simulator/rbq_sim/cyclonedds_sim.xml
```

Run only one simulator at a time; Isaac Sim and MuJoCo publish the same topics.
