"""Scene config: ground (or a USD world) + RBQ10 + foot contact sensors + body IMU."""

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, ImuCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from rbq_sim.robot import make_robot_cfg
from rbq_sim.worlds import yaw_to_quat

# IMU mount offset on the trunk link (IMU site in resources/model/rbq/rbq.xml).
IMU_OFFSET = (0.00665, 0.0, -0.0404)

# Foot links in SimInfo_.leg_contact order [RR, RL, FR, FL] = [HR, HL, FR, FL].
FOOT_LINKS = ["HR_foot", "HL_foot", "FR_foot", "FL_foot"]


@configclass
class RbqSimSceneCfg(InteractiveSceneCfg):
    # Update sensors every physics step. The Imu's lin_acc is a one-step finite difference over the step
    # dt; with lazy updates it is only evaluated when read (once per 2-step control period), which divides
    # a 2-step velocity change by a 1-step dt and doubles the reported acceleration.
    lazy_sensor_update = False

    # Flat ground, friction 0.8 (same as MuJoCo).
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="average",
            restitution_combine_mode="average",
            static_friction=0.8,
            dynamic_friction=0.8,
        ),
        debug_vis=False,
    )

    robot = make_robot_cfg(prim_path="{ENV_REGEX_NS}/Robot")

    # Contact forces on every robot body; feet are selected by name in the bridge.
    contact_sensor = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", track_air_time=False)

    # Body IMU on the trunk. Default gravity_bias (0,0,9.81) makes lin_acc_b report specific force
    # (~+9.81 z at rest), which the Motion estimator expects.
    imu = ImuCfg(prim_path="{ENV_REGEX_NS}/Robot/trunk", offset=ImuCfg.OffsetCfg(pos=IMU_OFFSET))

    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.9, 0.9, 0.9)),
    )

    # Optional USD environment scene — set by apply_world() when --world has a USD (replaces the
    # flat ground, which is then set to None). Left None => flat-ground world.
    world_scene: AssetBaseCfg | None = None


def apply_world(scene_cfg: "RbqSimSceneCfg", world: dict):
    """Apply a worlds.py entry: robot spawn pose, and the USD scene in place of the flat ground if set."""
    sx, sy, sz, syaw = world["spawn"]
    scene_cfg.robot.init_state.pos = (sx, sy, sz)
    scene_cfg.robot.init_state.rot = yaw_to_quat(syaw)

    usd = world.get("usd")
    if usd:
        usd_path = usd if usd.startswith("/") or "://" in usd else f"{ISAAC_NUCLEUS_DIR}/Environments/{usd}"
        scene_cfg.terrain = None  # the env USD provides its own floor
        scene_cfg.world_scene = AssetBaseCfg(
            prim_path="/World/Scene",
            spawn=sim_utils.UsdFileCfg(usd_path=usd_path),
        )
