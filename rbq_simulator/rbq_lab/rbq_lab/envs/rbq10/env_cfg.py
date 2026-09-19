import math
import isaaclab.sim as sim_utils

from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from isaaclab.assets import Articulation, ArticulationCfg, AssetBaseCfg

from isaaclab.managers import CurriculumTermCfg
from isaaclab.managers import ObservationGroupCfg
from isaaclab.managers import ObservationTermCfg
from isaaclab.managers import RewardTermCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg
from isaaclab.managers import EventTermCfg

from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import FrameTransformerCfg, ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.sim.simulation_cfg import PhysxCfg, SimulationCfg, RenderCfg

from isaaclab.terrains import TerrainImporterCfg

# from isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG  # isort: skip
from rbq_lab.utils.rough import ROUGH_TERRAINS_CFG
from rbq_lab.utils.marker import *

from .rbq10 import *

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from . import env_mdp

from ..base.base_task_cfg import BaseEnvCfg
from ..base.base_task_cfg import BaseSceneCfg

# region Scene
class Environment:
    num_envs = 2048
    env_spacing = 3.0
    episode_length_s = 30.0
    mode = "train"
# endregion

# region Scene
@configclass
class Scene(BaseSceneCfg):
    """Configuration for the terrain scene with a legged robot."""
    # robots
    robot: ArticulationCfg = RBQ10_URDF_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
 
    # ground terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator", # "plane",
        terrain_generator=ROUGH_TERRAINS_CFG,
        max_init_terrain_level=5,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="average", # ["average", "min", "multiply", "max"]
            restitution_combine_mode="average",
            static_friction=0.45,
            dynamic_friction=0.40,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            project_uvw=True,
            texture_scale=(0.25, 0.25),
        ),
        debug_vis=False,
    )

    # sensors
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/trunk",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=(1.8, 1.0)),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    )

    contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*",
        # history_length=3,
        track_air_time=False,
    )
    
# endregion

# region Commands
class Commands:
    resampling_time = 8.0
    num_commands = 3
    max_curriculum = 2.5
    class ranges:
        lin_vel_x=[-1.0, 1.0]
        lin_vel_y=[-1.0, 1.0]
        ang_vel_z=[-1.3, 1.3]
# endregion

# region Actions
class Actions:
    num_actions = 12
    scale = 0.1
    raw_actions_limit = 40.0
    decimation=5
    action_delay = True
# endregion

# region Observations
class ObservationsCfg:
    clip_obs = 100
    class scales:
        lin_vel = 2.0
        ang_vel = 0.25
        dof_pos = 1.0
        dof_vel = 0.05
        height_scale = 5.0   

    class noise:
        dof_pos = 0.01
        dof_vel = 0.2
        lin_vel = 0.1
        ang_vel = 0.2
        gravity = 0.05
        height_measurements = 0.06

    class delay:
        dof_vel_delay_steps = 2
        base_quat_delay_steps = 2
        base_ang_vel_delay_steps = 2

@configclass
class Observations:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        base_ang_vel = ObservationTermCfg(
            func=env_mdp.base_ang_vel,
            noise=Unoise(n_min=-ObservationsCfg.noise.ang_vel, 
                         n_max=ObservationsCfg.noise.ang_vel),
            scale=ObservationsCfg.scales.ang_vel,
        )
        projected_gravity = ObservationTermCfg(
            func=env_mdp.projected_gravity,
            noise=Unoise(n_min=-ObservationsCfg.noise.gravity, 
                         n_max=ObservationsCfg.noise.gravity),
        )
        velocity_commands = ObservationTermCfg(
            func=env_mdp.velocity_commands,
        )
        dof_pos = ObservationTermCfg(
            func=env_mdp.dof_pos,
            noise=Unoise(n_min=-ObservationsCfg.noise.dof_pos, 
                         n_max=ObservationsCfg.noise.dof_pos),
            scale=ObservationsCfg.scales.dof_pos,
        )
        dof_vel = ObservationTermCfg(
            func=env_mdp.dof_vel,
            noise=Unoise(n_min=-ObservationsCfg.noise.dof_vel, 
                         n_max=ObservationsCfg.noise.dof_vel),
            scale=ObservationsCfg.scales.dof_vel,
        )
        actions = ObservationTermCfg(
            func=env_mdp.actions,
        )

        def __post_init__(self):
            self.concatenate_terms = True
            self.enable_corruption = True


    @configclass
    class CriticCfg(ObservationGroupCfg):
        base_lin_vel = ObservationTermCfg(
            func=env_mdp.base_lin_vel,
            scale=ObservationsCfg.scales.lin_vel,
        )
        base_ang_vel = ObservationTermCfg(
            func=env_mdp.base_ang_vel,
            scale=ObservationsCfg.scales.ang_vel,
        )
        projected_gravity = ObservationTermCfg(
            func=env_mdp.projected_gravity,
        )
        velocity_commands = ObservationTermCfg(
            func=env_mdp.velocity_commands,
        )
        dof_pos = ObservationTermCfg(
            func=env_mdp.dof_pos,
            scale=ObservationsCfg.scales.dof_pos,
        )
        dof_vel = ObservationTermCfg(
            func=env_mdp.dof_vel,
            scale=ObservationsCfg.scales.dof_vel,
        )
        actions = ObservationTermCfg(
            func=env_mdp.actions,
        )
        measured_height = ObservationTermCfg(
            func=env_mdp.height_scan,
            params={"offset": 0.47},
            scale=ObservationsCfg.scales.height_scale,
            clip=(-1.0, 1.0),
        )
        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
            
    # Observation groups:
    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()
# endregion


# region Events
@configclass
class Events:
    """Configuration for events."""
    # startup
    physics_material = EventTermCfg(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.4, 0.9),
            "dynamic_friction_range": (1.0, 1.0),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
            "make_consistent": True,
        },
    )

    joint_parameters = EventTermCfg(
        func=mdp.randomize_joint_parameters,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "friction_distribution_params": (0.5, 0.5),
            "armature_distribution_params": (0.95, 1.05),
            "operation": "scale",
        }
    )    

    actuator_gains = EventTermCfg(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stiffness_distribution_params": (0.9, 1.1),
            "damping_distribution_params": (0.9, 1.1),
            "operation": "scale",
        }
    )

    add_base_mass = EventTermCfg(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="trunk"),
            "mass_distribution_params": (-1.0, 15.0),
            "operation": "add",
        },
    )

    base_com = EventTermCfg(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="trunk"),
            "com_range": {"x": (-0.06, 0.06)},
        },
    )

    # reset
    joint_offset = EventTermCfg(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "position_range": (-0.02, 0.02),
            "velocity_range": (-0.0, 0.0),
        },
    )

    reset_base = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
        },
    )

    reset_robot_joints = EventTermCfg(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.5, 1.5),
            "velocity_range": (0.0, 0.0),
        },
    )

    # interval
    push_robot = EventTermCfg(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
    )

# endregion


# region Rewards
class RewardsCfg:
    base_height_target = 0.48 # 0.43
    tracking_sigma = 0.25 # tracking reward = exp(-error^2/sigma)
    penalize_contacts_on = [".*_calf"]
    terminate_after_contacts_on = ["trunk", ".*_thigh"]

@configclass
class Rewards:
    tracking_lin_vel = RewardTermCfg(
        func   = env_mdp.tracking_lin_vel_reward, 
        weight = 1.0,
    )
    
    tracking_ang_vel = RewardTermCfg(
        func   = env_mdp.tracking_ang_vel_reward,
        weight = 0.7,
    )

    lin_vel_z = RewardTermCfg(
        func   = env_mdp.lin_vel_z_penalty, 
        weight = -2.0,
    )
    
    ang_vel_xy = RewardTermCfg(
        func   = env_mdp.ang_vel_xy_penalty, 
        weight = -0.05,
    )

    # base_height = RewardTermCfg(
    #     func   = env_mdp.base_height_penalty,
    #     weight = -5.0,
    #     params = {
    #         "base_height_target": 0.48,
    #     },
    # )

    collision = RewardTermCfg(
        func   = env_mdp.collision_penalty,
        weight = -1.0,
    )

    # feet_slip = RewardTermCfg(
    #     func   = env_mdp.feet_slip_reward,
    #     weight = 0.5,
    # )

    hip_pos = RewardTermCfg(
        func   = env_mdp.hip_pos_penalty,
        weight = -0.5,
    )

    torques = RewardTermCfg(
        func   = env_mdp.torques_penalty,
        weight = -1e-5,
    )

    # dof_vel = RewardTermCfg(
    #     func   = env_mdp.dof_vel_penalty,
    #     weight = -1e-3,
    # )
    
    dof_acc = RewardTermCfg(
        func   = env_mdp.dof_acc_penalty,
        weight =-2.5e-7
    )

    action_rate = RewardTermCfg(
        func   = env_mdp.action_rate_penalty,
        weight = -0.01,
    )

    # smoothness = RewardTermCfg(
    #     func   = env_mdp.action_smoothness_penalty,
    #     weight = -0.01,
    # )

    all_feet_contact = RewardTermCfg(
        func   = env_mdp.all_feet_contact_reward,
        weight = 0.5,
    )
# endregion


# region Terminations
@configclass
class Terminations:
    """Termination terms for the MDP."""
    time_out = TerminationTermCfg(func=env_mdp.time_out, time_out=True)
    base_contact = TerminationTermCfg(func=env_mdp.illegal_contact)
# endregion

# region Curriculum
@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""
    terrain_levels = CurriculumTermCfg(
        func=env_mdp.update_terrain_levels,
    )
    max_commands = CurriculumTermCfg(
        func=env_mdp.update_commands,
        params={
            "reward_term_name": "tracking_lin_vel",
        },
    )
# endregion

# region CFG
@configclass
class EnvCfg(BaseEnvCfg):
    env_cfg : Environment = Environment()
    scene: Scene = Scene(num_envs=Environment.num_envs, env_spacing=Environment.env_spacing) 
    commands: Commands = Commands()
    actions: Actions = Actions()
    observations: Observations = Observations()
    observations_cfg: ObservationsCfg = ObservationsCfg()
    events: Events = Events()
    rewards: Rewards = Rewards()
    rewards_cfg: RewardsCfg = RewardsCfg()
    terminations: Terminations = Terminations()
    curriculum: CurriculumCfg = CurriculumCfg()
    sim = SimulationCfg(
        device="cuda:0",
        dt=0.002,
        gravity=(0.0, 0.0, -9.81),
        render_interval=actions.decimation,
        physics_material = scene.terrain.physics_material,
        physx = PhysxCfg(
            solver_type=1,
            max_position_iteration_count=4,
            max_velocity_iteration_count=0,
            bounce_threshold_velocity=0.5,
            gpu_max_rigid_contact_count=2**23,
        ),
    )

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.episode_length_s = self.env_cfg.episode_length_s
        self.decimation = self.actions.decimation
                
        # self.sim.dt = 0.002
        # self.sim.render_interval = self.decimation
        # self.sim.physics_material = self.scene.terrain.physics_material
        # self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15

        # update sensor update periods
        if self.scene.height_scanner is not None:
            self.scene.height_scanner.update_period = self.sim.dt

        if self.scene.contact_sensor is not None:
            self.scene.contact_sensor.update_period = self.sim.dt

        if getattr(self.curriculum, "terrain_levels", None) is not None:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = True
        else:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = False


@configclass
class EnvCfg_PLAY(EnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 5
        self.scene.env_spacing = 2.5
        self.episode_length_s = 5.0*10e+5
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator.curriculum = False
        self.scene.terrain.terrain_generator.num_rows = 5
        self.scene.terrain.terrain_generator.num_cols = 5

        self.events.physics_material.params["static_friction_range"] = (0.8, 0.8)
        self.events.push_robot = None
        self.env_cfg.mode = "play"


# endregion
