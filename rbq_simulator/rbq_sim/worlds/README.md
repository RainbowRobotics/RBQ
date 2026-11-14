# rbq_sim world assets (USD)

USD world scenes for rbq_sim. The robot URDF and meshes are not here; they come from the repo-root
`resources/`. The world registry (names, spawn poses, which USD each world loads) is `rbq_sim/worlds.py`.

Isaac Sim worlds are **USD** scenes (`.usd`/`.usda`/`.usdc`), unlike the Gazebo sim's SDF
(`rbq_gazebo/resources/worlds/*.sdf`) or MuJoCo's MJCF.

## Default worlds — no files needed here

The built-in worlds use the **sample environments Isaac Sim ships by default** on the Nucleus
asset server, referenced as a path relative to `{ISAAC_NUCLEUS_DIR}/Environments/` and resolved at
runtime (`scene.apply_world`). They download/cache automatically the first time — nothing goes in
this folder:

| `--world` | USD (under `{ISAAC_NUCLEUS_DIR}/Environments/`) |
|-----------|--------------------------------------------------|
| `warehouse` | `Simple_Warehouse/warehouse.usd` |
| `depot` | `Simple_Warehouse/full_warehouse.usd` |
| `warehouse_shelves` | `Simple_Warehouse/warehouse_multiple_shelves.usd` |
| `warehouse_forklifts` | `Simple_Warehouse/warehouse_with_forklifts.usd` |
| `grid` | `Grid/default_environment.usd` |
| `office` | `Office/office.usd` |
| `hospital` | `Hospital/hospital.usd` |
| `simple_room` | `Simple_Room/simple_room.usd` |
| `empty`, `raceway`, `maze` | (none — flat ground via `scene.py` plane) |

(Exact sample paths can vary by Isaac Sim version; adjust in `worlds.py` if a USD fails to resolve.
Flat-ground worlds always work offline.)

## Repo-local research worlds — in this folder

Generated from primitives by `rbq_sim/tools/build_worlds.py` (textured convex meshes with static
colliders, friction 0.8, procedural textures in `textures/`; no third-party assets). Registered in
`worlds.py` with absolute paths, so they load offline:

| `--world` | USD |
|-----------|-----|
| `stairs_course` | `stairs_course.usda` |
| `two_floor_office` | `two_floor_office.usda` |
| `terrain_park` | `terrain_park.usda` |

Change a layout in its `build_*` function and rerun `python -m rbq_sim.tools.build_worlds [name]`
(inside `scripts/activate.bash`), or edit the `.usda` in Isaac Sim — rerunning the generator overwrites
such edits.

## Custom / local worlds

Add an entry to `WORLDS` in `worlds.py` whose `usd` is an **absolute path** (starting with `/`) or a
URL (`file://`, `omniverse://`, `http(s)://`); `apply_world` uses those as is and treats anything else
as relative to the Isaac Sim `Environments/` folder. The USD replaces the flat ground, so it must
carry its own collision floor; raise the spawn `z` by the floor height if the floor is not at 0.
