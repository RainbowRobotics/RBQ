import torch
import math
import numpy as np
from collections.abc import Sequence

from isaaclab.envs.common import VecEnvStepReturn
from isaaclab.sensors import ContactSensor, RayCaster
from rbq_lab.utils.math import *
import isaaclab.utils.math as math_utils


from ..base.base_task import BaseEnv
from .env_cfg import EnvCfg

class Env(BaseEnv):
    cfg: EnvCfg
    def __init__(self, cfg: EnvCfg, render_mode: str | None = None, **kwargs):  
        super().__init__(cfg=cfg, render_mode=render_mode)

    def step(self, action: torch.Tensor) -> VecEnvStepReturn:
        actions_limit = self.cfg.actions.raw_actions_limit
        self.actions = torch.clip(action, -actions_limit, actions_limit).to(self.device)

        self.delayed_actions = self.actions.clone().view(self.num_envs, 1, self.cfg.actions.num_actions).repeat(1, self.cfg.decimation, 1)
        if self.cfg.actions.action_delay:
            for i in range(self.cfg.decimation):
                self.delayed_actions[:, i] = self.last_actions + (self.actions - self.last_actions) * (i >= self.delay_steps)

        is_rendering = self.sim.has_gui() or self.sim.has_rtx_sensors()
        for i in range(self.cfg.decimation):
            self.target_pos = self.compute_target(self.delayed_actions[:, i]).view(self.dof_pos.shape)
            self._robot.set_joint_position_target(self.target_pos)
            self.scene.write_data_to_sim()

            self.sim.step(render=False)
            self._sim_step_counter += 1
            if self._sim_step_counter % self.cfg.sim.render_interval == 0 and is_rendering:
                self.sim.render()
            self.scene.update(dt=self.physics_dt)
            self.refresh_dof_state()

        self.post_physics_step()

        self.obs_buf["policy"] = self.obs_buf["policy"].clamp(-self.cfg.observations_cfg.clip_obs, self.cfg.observations_cfg.clip_obs)
        if "critic" in self.observation_manager.active_terms:
            self.obs_buf["critic"] = self.obs_buf["critic"].clamp(-self.cfg.observations_cfg.clip_obs, self.cfg.observations_cfg.clip_obs)

        # return observations, rewards, resets and extras
        return self.obs_buf, self.reward_buf, self.reset_terminated, self.reset_time_outs, self.extras


    def post_physics_step(self):
        self.refresh_root_state()
        self.refresh_contact_forces()
        self.refresh_measured_heights()
        self.refresh_rigid_body_state()

        self.episode_length_buf += 1  # step in current episode (per env)
        self.common_step_counter += 1  # total step (common for all envs)

        # _post_physics_step_callback
        resample_commands_ids = (self.episode_length_buf % int(self.cfg.commands.resampling_time / self.step_dt)==0).nonzero(as_tuple=False).flatten()
        self.resample_commands(resample_commands_ids)

        # -- step interval events
        if "interval" in self.event_manager.available_modes:
            self.event_manager.apply(mode="interval", dt=self.step_dt)

        # -- check terminations
        self._check_termination()

        # -- reward computation
        self._compute_reward()

        # -- reset envs that terminated/timed-out and log the episode information
        reset_env_ids = self.reset_buf.nonzero(as_tuple=False).squeeze(-1)
        self._reset(reset_env_ids)

        # -- compute observations
        self.obs_buf = self.observation_manager.compute(update_history=True)

        self.last_last_actions[:] = self.last_actions[:]
        self.last_actions[:] = self.actions[:]
        self.last_dof_vel[:] = self.dof_vel[:]

    def compute_target(self, actions: torch.Tensor):
        actions_scaled = actions * self.cfg.actions.scale
        target_pos = (self.default_dof_pos + actions_scaled)
        return target_pos[:, self.model2sim_order]

    def resample_commands(self, env_ids):
        self.commands[env_ids, 0] = torch_rand_float(self.cfg.commands.ranges.lin_vel_x[0], self.cfg.commands.ranges.lin_vel_x[1], (len(env_ids), 1), device=self.device).squeeze(1)
        self.commands[env_ids, 1] = torch_rand_float(self.cfg.commands.ranges.lin_vel_y[0], self.cfg.commands.ranges.lin_vel_y[1], (len(env_ids), 1), device=self.device).squeeze(1)
        self.commands[env_ids, 2] = torch_rand_float(self.cfg.commands.ranges.ang_vel_z[0], self.cfg.commands.ranges.ang_vel_z[1], (len(env_ids), 1), device=self.device).squeeze(1)
        
        commmand_deadzone = 0.2 * self.cfg.commands.ranges.lin_vel_x[1]
        self.commands[env_ids, :] = torch.where(torch.norm(self.commands[env_ids, :], dim=1, keepdim=True) < commmand_deadzone,
                                                torch.zeros_like(self.commands[env_ids, :]),
                                                self.commands[env_ids, :] * (1 - commmand_deadzone / torch.norm(self.commands[env_ids, :], dim=1, keepdim=True)))

    def _init_buffers(self):
        self._robot = self.scene.articulations["robot"]
        self.set_joint_order()

        self.num_dofs = self._robot.data.joint_pos.shape[1]
        self.num_actions = self.cfg.actions.num_actions

        self.refresh_dof_state()
        self.refresh_root_state()
        
        self.default_dof_pos = self._robot.data.default_joint_pos[:, self.sim2model_order]

        self.feet_ids = torch.as_tensor(self._robot.find_bodies([".*_foot"], preserve_order=True)[0], device=self.device, dtype=torch.long)
        self.refresh_rigid_body_state()

        # contact_body_ids order : [HRR,HRP,HRK, HLR,HLP,HLK, FRR,FRP,FRK, FLR,FLP,FLK]
        self.contact_sensors: ContactSensor = self.scene.sensors["contact_sensor"]
        self.penalised_contact_indices = torch.as_tensor(self.contact_sensors.find_bodies(self.cfg.rewards_cfg.penalize_contacts_on, preserve_order=True)[0], device=self.device, dtype=torch.long)
        self.termination_contact_indices = torch.as_tensor(self.contact_sensors.find_bodies(self.cfg.rewards_cfg.terminate_after_contacts_on, preserve_order=True)[0], device=self.device, dtype=torch.long)
        self.feet_contact_indices = torch.as_tensor(self.contact_sensors.find_bodies([".*_foot"], preserve_order=True)[0], device=self.device, dtype=torch.long)
        self.last_contact = torch.zeros(self.scene.num_envs, len(self.feet_ids), dtype=torch.bool, device=self.device, requires_grad=False)
        self.refresh_contact_forces()

        self.height_sensor: RayCaster = self.scene.sensors["height_scanner"]
        self.measured_heights = torch.zeros(self.scene.num_envs, self.height_sensor.num_rays, device=self.device, dtype=torch.float, requires_grad=False)
        self.refresh_measured_heights()

        self.gravity_vec = self._robot.data.GRAVITY_VEC_W
        self.forward_vec = self._robot.data.FORWARD_VEC_B

        self.actions = torch.zeros(self.scene.num_envs, self.num_actions, device=self.device, dtype=torch.float, requires_grad=False)
        self.last_actions = torch.zeros_like(self.actions)
        self.last_last_actions = torch.zeros_like(self.actions)

        self.torques = torch.zeros(self.scene.num_envs, self.num_actions, dtype=torch.float, device=self.device, requires_grad=False)

        self.commands = torch.zeros(self.scene.num_envs, self.cfg.commands.num_commands, dtype=torch.float, device=self.device, requires_grad=False)

        self.last_dof_vel = torch.zeros(self.scene.num_envs, self.num_dofs, dtype=torch.float, device=self.device, requires_grad=False)

        self.delay_steps = torch.randint(1, 4, (self.scene.num_envs, 1), device=self.device, dtype=torch.int, requires_grad=False)

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids=env_ids)

        # reset the episode length buffer
        self.episode_length_buf[env_ids] = 0

        self.resample_commands(env_ids)

        self.actions[env_ids] = 0.0
        self.last_actions[env_ids] = 0.0
        self.last_last_actions[env_ids] = 0.0
        self.last_dof_vel[env_ids] = 0.0
        self.torques[env_ids] = 0.0

        self.delay_steps[env_ids] = torch.randint(1, 4, (len(env_ids), 1), device=self.device, dtype=torch.int, requires_grad=False)


    def set_joint_order(self):
        from .rbq10 import RBQ10_LEG_JOINT, RBQ10_HIP_JOINT, RBQ10_THIGH_JOINT, RBQ10_KNEE_JOINT
        self.sim2model_order = torch.as_tensor(self._robot.find_joints(RBQ10_LEG_JOINT, preserve_order=True)[0], device=self.device, dtype=torch.long)
        
        self.model2sim_order = torch.empty_like(self.sim2model_order)
        self.model2sim_order[self.sim2model_order] = torch.arange(len(self.sim2model_order), device=self.device, dtype=torch.long)

    # -------------------------------------------------------------------------------------

    def refresh_dof_state(self):
        self.dof_pos = self._robot.data.joint_pos[:, self.sim2model_order]
        self.dof_vel = self._robot.data.joint_vel[:, self.sim2model_order]
        self.dof_acc = self._robot.data.joint_acc[:, self.sim2model_order]

    def refresh_root_state(self):
        self.base_pos = self._robot.data.root_link_pos_w
        self.base_quat = self._robot.data.root_link_quat_w
        self.base_lin_vel = self._robot.data.root_link_lin_vel_b
        self.base_ang_vel = self._robot.data.root_link_ang_vel_b
        self.projected_gravity = self._robot.data.projected_gravity_b

    def refresh_contact_forces(self):
        self.contact_forces = self.scene.sensors["contact_sensor"].data.net_forces_w

    def refresh_measured_heights(self):
        self.measured_heights = torch.clamp(self.height_sensor.data.ray_hits_w[..., 2], -1e6, 1e6)
        
    def refresh_rigid_body_state(self):
        self.foot_positions = self._robot.data.body_link_pos_w[:, self.feet_ids, :]
        self.foot_velocities = self._robot.data.body_link_lin_vel_w[:, self.feet_ids, :]

        