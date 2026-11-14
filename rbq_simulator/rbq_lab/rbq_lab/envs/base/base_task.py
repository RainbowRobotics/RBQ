import torch
from prettytable import PrettyTable

from isaaclab.envs.manager_based_rl_env import ManagerBasedRLEnv
from rbq_lab.utils.math import *
from .base_task_cfg import BaseEnvCfg as EnvCfg

import gymnasium as gym
import builtins
import omni.log

from isaaclab.managers import ObservationManager, RecorderManager, RewardManager, TerminationManager, CurriculumManager, EventManager
from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext
from isaaclab.sim.utils import attach_stage_to_usd_context, use_stage
from isaaclab.utils.timer import Timer
from isaaclab.ui.widgets import ManagerLiveVisualizer

from isaaclab.envs.ui import ViewportCameraController
from collections.abc import Sequence
from isaaclab.envs.utils.spaces import sample_space, spec_to_gym_space
from isaacsim.core.version import get_version


class BaseEnv(ManagerBasedRLEnv):
    cfg: EnvCfg
    def __init__(self, cfg: EnvCfg, render_mode: str | None = None, **kwargs):  
        self.common_step_counter = 0
        self.episode_length_buf = torch.zeros(cfg.scene.num_envs, device=cfg.sim.device, dtype=torch.long)

        cfg.validate()
        self.cfg = cfg

        self._is_closed = False

        if self.cfg.seed is not None:
            self.cfg.seed = self.seed(self.cfg.seed)
        else:
            omni.log.warn("Seed not set for the environment. The environment creation may not be deterministic.")

        if SimulationContext.instance() is None:
            self.sim: SimulationContext = SimulationContext(self.cfg.sim)
        else:
            if not builtins.ISAAC_LAUNCHED_FROM_TERMINAL:
                raise RuntimeError("Simulation context already exists. Cannot create a new one.")
            self.sim: SimulationContext = SimulationContext.instance()

        if "cuda" in self.device:
            torch.cuda.set_device(self.device)

        print("[INFO]: Base environment:")
        print(f"\tEnvironment device    : {self.device}")
        print(f"\tEnvironment seed      : {self.cfg.seed}")
        print(f"\tPhysics step-size     : {self.physics_dt}")
        print(f"\tRendering step-size   : {self.physics_dt * self.cfg.sim.render_interval}")
        print(f"\tEnvironment step-size : {self.step_dt}")

        if self.cfg.sim.render_interval < self.cfg.decimation:
            msg = (
                f"The render interval ({self.cfg.sim.render_interval}) is smaller than the decimation "
                f"({self.cfg.decimation}). Multiple render calls will happen for each environment step. "
                "If this is not intended, set the render interval to be equal to the decimation."
            )
            omni.log.warn(msg)

        self._sim_step_counter = 0

        self.extras = {}

        with Timer("[INFO]: Time taken for scene creation", "scene_creation"):
            with use_stage(self.sim.get_initial_stage()):
                self.scene = InteractiveScene(self.cfg.scene)
                attach_stage_to_usd_context()
        print("[INFO]: Scene manager: ", self.scene)

        if self.sim.render_mode >= self.sim.RenderMode.PARTIAL_RENDERING:
            self.viewport_camera_controller = ViewportCameraController(self, self.cfg.viewer)
        else:
            self.viewport_camera_controller = None

        self.event_manager = EventManager(self.cfg.events, self)

        if "prestartup" in self.event_manager.available_modes:
            self.event_manager.apply(mode="prestartup")

        if builtins.ISAAC_LAUNCHED_FROM_TERMINAL is False:
            print("[INFO]: Starting the simulation. This may take a few seconds. Please wait...")
            with Timer("[INFO]: Time taken for simulation start", "simulation_start"):
                with use_stage(self.sim.get_initial_stage()):
                    self.sim.reset()
                self.scene.update(dt=self.physics_dt)
            # ================== #
            self._init_buffers()  #
            # ================== #
            self.load_managers()

        # ========================== #
        self._init_manager_buffers()  #
        # ========================== #
        
        if self.sim.has_gui() and self.cfg.ui_window_class_type is not None:
            self.setup_manager_visualizers()
            self._window = self.cfg.ui_window_class_type(self, window_name="IsaacLab")
        else:
            self._window = None

        self.obs_buf = {}

        if self.cfg.export_io_descriptors:
            self.export_IO_descriptors()

        self.render_mode = render_mode
        self.metadata["render_fps"] = 1 / self.step_dt

        table = PrettyTable()
        table.field_names = ["Type", "Name", "Index"]
        for idx, name in enumerate(self.scene.articulations["robot"].joint_names):
            table.add_row(["Joint", name, idx])
        table.add_row(["-" * 5, "-" * 10, "-" * 5])
        for idx, name in enumerate(self.scene.articulations["robot"].body_names):
            table.add_row(["Link", name, idx])
        table.align["Type"] = "l"
        table.align["Name"] = "l"
        table.align["Index"] = "r"
        print(table)

    def _init_buffers(self):
        pass

    def _init_manager_buffers(self):
        pass

    def load_managers(self):

        # # -- command manager
        # self.command_manager: CommandManager = CommandManager(self.cfg.commands, self)
        # print("[INFO] Command Manager: ", self.command_manager)
        # -- event manager
        if "startup" in self.event_manager.available_modes:
            self.event_manager.apply(mode="startup")
            
        print("[INFO] Event Manager: ", self.event_manager)
        # -- recorder manager
        self.recorder_manager = RecorderManager(self.cfg.recorders, self)
        print("[INFO] Recorder Manager: ", self.recorder_manager)
        # -- observation manager
        self.observation_manager = ObservationManager(self.cfg.observations, self)
        print("[INFO] Observation Manager:", self.observation_manager)
        # -- termination manager
        self.termination_manager = TerminationManager(self.cfg.terminations, self)
        print("[INFO] Termination Manager: ", self.termination_manager)
        # -- reward manager
        self.reward_manager = RewardManager(self.cfg.rewards, self)
        print("[INFO] Reward Manager: ", self.reward_manager)
        # -- curriculum manager
        self.curriculum_manager = CurriculumManager(self.cfg.curriculum, self)
        print("[INFO] Curriculum Manager: ", self.curriculum_manager)

        # setup the action and observation spaces for Gym
        self._configure_gym_env_spaces(cfg=self.cfg)

    def setup_manager_visualizers(self):
        """Creates live visualizers for manager terms."""

        self.manager_visualizers = {
            "observation_manager": ManagerLiveVisualizer(manager=self.observation_manager),
            "termination_manager": ManagerLiveVisualizer(manager=self.termination_manager),
            "reward_manager": ManagerLiveVisualizer(manager=self.reward_manager),
            "curriculum_manager": ManagerLiveVisualizer(manager=self.curriculum_manager),
        }

    def close(self):
        if not self._is_closed:
            # destructor is order-sensitive
            del self.reward_manager
            del self.termination_manager
            del self.curriculum_manager

            # destructor is order-sensitive
            del self.viewport_camera_controller
            del self.observation_manager
            del self.event_manager
            del self.recorder_manager
            del self.scene

            # clear callbacks and instance
            if float(".".join(get_version()[2])) >= 5:
                if self.cfg.sim.create_stage_in_memory:
                    # detach physx stage
                    omni.physx.get_physx_simulation_interface().detach_stage()
                    self.sim.stop()
                    self.sim.clear()

            self.sim.clear_all_callbacks()
            self.sim.clear_instance()

            # destroy the window
            if self._window is not None:
                self._window = None
            # update closing status
            self._is_closed = True

    def _configure_gym_env_spaces(self, cfg):
        """Configure the action and observation spaces for the Gym environment."""
        # observation space (unbounded since we don't impose any limits)
        self.single_observation_space = gym.spaces.Dict()
        for group_name, group_term_names in self.observation_manager.active_terms.items():
            # extract quantities about the group
            has_concatenated_obs = self.observation_manager.group_obs_concatenate[group_name]
            group_dim = self.observation_manager.group_obs_dim[group_name]
            # check if group is concatenated or not
            # if not concatenated, then we need to add each term separately as a dictionary
            if has_concatenated_obs:
                self.single_observation_space[group_name] = gym.spaces.Box(low=-np.inf, high=np.inf, shape=group_dim)
            else:
                group_term_cfgs = self.observation_manager._group_obs_term_cfgs[group_name]
                term_dict = {}
                for term_name, term_dim, term_cfg in zip(group_term_names, group_dim, group_term_cfgs):
                    low = -np.inf if term_cfg.clip is None else term_cfg.clip[0]
                    high = np.inf if term_cfg.clip is None else term_cfg.clip[1]
                    term_dict[term_name] = gym.spaces.Box(low=low, high=high, shape=term_dim)
                self.single_observation_space[group_name] = gym.spaces.Dict(term_dict)
        # action space (unbounded since we don't impose any limits)
        # action_dim = sum(self.action_manager.action_term_dim)
        action_dim = cfg.actions.num_actions
        self.single_action_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(action_dim,))

        # batch the spaces for vectorized environments
        self.observation_space = gym.vector.utils.batch_space(self.single_observation_space, self.num_envs)
        self.action_space = gym.vector.utils.batch_space(self.single_action_space, self.num_envs)

    def _reset_idx(self, env_ids: Sequence[int]):
        self.curriculum_manager.compute(env_ids=env_ids)
        self.scene.reset(env_ids)
        if "reset" in self.event_manager.available_modes:
            env_step_count = self._sim_step_counter // self.cfg.decimation
            self.event_manager.apply(mode="reset", env_ids=env_ids, global_env_step_count=env_step_count)

        self.extras["log"] = dict()
        # -- observation manager
        info = self.observation_manager.reset(env_ids)
        self.extras["log"].update(info)
        # -- rewards manager
        info = self.reward_manager.reset(env_ids)
        self.extras["log"].update(info)
        # -- curriculum manager
        info = self.curriculum_manager.reset(env_ids)
        self.extras["log"].update(info)
        # # -- command manager
        # info = self.command_manager.reset(env_ids)
        # self.extras["log"].update(info)
        # -- event manager
        info = self.event_manager.reset(env_ids)
        self.extras["log"].update(info)
        # -- termination manager
        info = self.termination_manager.reset(env_ids)
        self.extras["log"].update(info)
        # -- recorder manager
        info = self.recorder_manager.reset(env_ids)
        self.extras["log"].update(info)


    def _check_termination(self):
        self.reset_buf = self.termination_manager.compute()
        self.reset_terminated = self.termination_manager.terminated
        self.reset_time_outs = self.termination_manager.time_outs

    def _compute_reward(self):
        self.reward_buf = self.reward_manager.compute(dt=self.step_dt)

    def _reset(self, reset_env_ids):
        if len(reset_env_ids) > 0:
            self._reset_idx(reset_env_ids)
            if self.sim.has_rtx_sensors() and self.cfg.rerender_on_reset:
                self.sim.render()
                