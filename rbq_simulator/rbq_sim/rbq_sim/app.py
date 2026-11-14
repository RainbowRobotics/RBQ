"""rbq_sim: Isaac Sim integration simulator for RBQ, the Isaac Sim peer of rbq_mujoco / rbq_gazebo.

Spawns RBQ10 from the rbq_lab URDF cfg and bridges it to the RBQ Motion stack over CycloneDDS:
MotionRef_ (rt/rbq/ref/motion/_0) in, SimInfo_ (rt/rbq/_sim) out, lock-stepped to Motion's RT tick.
"""

import argparse
import time

from isaaclab.app import AppLauncher  # AppLauncher must be created before any other Isaac import

parser = argparse.ArgumentParser(description="RBQ Isaac Sim integration simulator (rbq_sim).")
parser.add_argument(
    "--world", type=str, default="empty",
    help="World name from rbq_sim/worlds.py (default: empty).",
)
parser.add_argument(
    "--interface", "-i", type=str, default="lo",
    help="CycloneDDS network interface (default: lo).",
)
AppLauncher.add_app_launcher_args(parser)
# One articulation at 1 kHz: CPU PhysX beats the GPU pipeline, whose per-step host<->device syncs
# dominate (measured 2.5 vs 5.4 ms per 2 ms control period). --device cuda:0 still works.
parser.set_defaults(device="cpu")
args_cli = parser.parse_args()

# Launch Omniverse / Isaac Sim.
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- Everything below runs only after the app is up (AppLauncher contract). ---
import carb  # noqa: E402
import omni.usd  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.scene import InteractiveScene  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402

from rbq_sim.bridge import MotionBridge  # noqa: E402
from rbq_sim.robot import compute_joint_order  # noqa: E402
from rbq_sim.scene import RbqSimSceneCfg, apply_world  # noqa: E402
from rbq_sim.skin import apply_skin  # noqa: E402
from rbq_sim.worlds import get_world  # noqa: E402

# Physics at 1 kHz, the timebase the Motion estimator assumes (same as MuJoCo/Gazebo).
SIM_DT = 1.0 / 1000.0
# One control period = one Motion RT tick (2 ms) = 2 physics steps. The bridge lock-steps each period
# to Motion (bridge.sync), so sim time, not wall time, is the control clock.
CONTROL_DT = 0.002
PHYSICS_SUBSTEPS = round(CONTROL_DT / SIM_DT)
CONTROL_DT_NS = round(CONTROL_DT * 1e9)
# Viewport refresh in sim time. Rendering is decoupled from physics; a slow frame only slows the
# real-time factor, never the control period.
RENDER_DT = 1.0 / 60.0
STATS_PERIOD_S = 5.0


def main():
    # Solve on the calling thread: for a scene this small, PhysX worker dispatch costs more than the
    # solve itself (measured 1.03 -> 0.23 ms per control period).
    carb.settings.get_settings().set_int("/persistent/physics/numThreads", 0)
    sim = SimulationContext(sim_utils.SimulationCfg(dt=SIM_DT, device=args_cli.device))
    sim.set_camera_view(eye=(2.0, 2.0, 1.5), target=(0.0, 0.0, 0.4))

    # World config (rbq_sim/worlds.py): per-world robot spawn pose + optional Isaac-default USD
    # environment (warehouse/office/hospital/...); flat ground otherwise. Sitting joint pose stays.
    scene_cfg = RbqSimSceneCfg(num_envs=1, env_spacing=2.0)
    apply_world(scene_cfg, get_world(args_cli.world))
    scene = InteractiveScene(scene_cfg)
    # MuJoCo's textured look in the viewport; the URDF visuals are plain grey STL (rbq_sim/skin.py).
    skinned = apply_skin(omni.usd.get_context().get_stage(), "/World/envs/env_0/Robot", scene_cfg.robot.spawn.asset_path)
    print(f"[rbq_sim] skin: {skinned} links textured")

    sim.reset()
    robot = scene["robot"]
    imu = scene["imu"]
    contact = scene["contact_sensor"]
    # InteractiveScene does not push init_state into PhysX (IsaacLab envs do it in a reset event). Without
    # this the legs start straight at q=0 inside the ground, the hold PD folds them, and the kick flips the robot.
    root_state = robot.data.default_root_state.clone()
    root_state[:, :3] += scene.env_origins
    robot.write_root_pose_to_sim(root_state[:, :7])
    robot.write_root_velocity_to_sim(root_state[:, 7:])
    robot.write_joint_state_to_sim(robot.data.default_joint_pos, robot.data.default_joint_vel)
    scene.reset()
    sim2model, _model2sim = compute_joint_order(robot)
    print(f"[rbq_sim] articulation joints ({robot.num_joints}): {robot.joint_names}")
    print(f"[rbq_sim] sim2model (wire) order indices: {sim2model.tolist()}")

    # CycloneDDS bridge: MotionRef_ -> PD-in-loop torque -> SimInfo_ (holds the sitting spawn
    # pose until Motion connects). See rbq_sim/bridge.py.
    bridge = MotionBridge(robot, imu, contact, sim2model, CONTROL_DT, interface=args_cli.interface)

    render_every = max(1, round(RENDER_DT / CONTROL_DT))
    tick = 0
    stamp_ns = 0
    stats_t0 = time.perf_counter()
    stats_tick0 = 0
    physics_s = 0.0
    while simulation_app.is_running():
        t0 = time.perf_counter()
        for _ in range(PHYSICS_SUBSTEPS):
            bridge.apply_command()
            scene.write_data_to_sim()
            sim.step(render=False)
            scene.update(SIM_DT)
        tick += 1
        stamp_ns += CONTROL_DT_NS
        bridge.publish_state(stamp_ns)
        if tick % render_every == 0:
            sim.render()
        physics_s += time.perf_counter() - t0
        # Block until Motion has ticked on this stamp; its references then drive the next period.
        bridge.sync()

        now = time.perf_counter()
        if now - stats_t0 >= STATS_PERIOD_S:
            st = bridge.stats
            n = tick - stats_tick0
            wall = now - stats_t0
            print(
                f"[rbq_sim] sim {n * CONTROL_DT / wall:.2f}x real-time | control {n / wall:.0f} Hz wall "
                f"(sim {1.0 / CONTROL_DT:.0f} Hz) | lockstep {'ON' if bridge.locked else 'OFF'} "
                f"acked {st['acked']}/{st['ticks']} timeouts {st['timeouts']} | "
                f"sim work {physics_s / n * 1e3:.2f} ms, Motion wait avg "
                f"{(st['wait_s'] / st['acked'] * 1e3) if st['acked'] else 0.0:.2f} ms max {st['wait_max_s'] * 1e3:.1f} ms",
                flush=True,
            )
            bridge.stats = {"ticks": 0, "acked": 0, "timeouts": 0, "wait_s": 0.0, "wait_max_s": 0.0}
            stats_t0, stats_tick0, physics_s = now, tick, 0.0


if __name__ == "__main__":
    main()
    simulation_app.close()
