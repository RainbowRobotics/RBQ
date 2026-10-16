from __future__ import annotations

import torch
import numpy as np
from collections.abc import Sequence

from isaaclab.terrains import TerrainImporter
from isaaclab.utils.math import  quat_from_angle_axis, quat_apply_inverse


# -------------- #
# | Curriculum | #
# -------------- #
def update_commands(env, env_ids: Sequence[int], reward_term_name: str):
    commands = env.cfg.commands
    reward_term_cfg = env.reward_manager.get_term_cfg(reward_term_name)
        
    if env.common_step_counter % env.max_episode_length == 0:
        if (torch.mean(env.reward_manager._episode_sums[reward_term_name][env_ids]) / env.max_episode_length) > (0.8 * reward_term_cfg.weight * env.step_dt):
            commands.ranges.lin_vel_x[0] = np.clip(commands.ranges.lin_vel_x[0] - 0.3, -commands.max_curriculum, 0.)
            commands.ranges.lin_vel_x[1] = np.clip(commands.ranges.lin_vel_x[1] + 0.3, 0., commands.max_curriculum)

    return torch.tensor(commands.ranges.lin_vel_x[1], device=env.device)

def update_terrain_levels(env, env_ids: Sequence[int]) -> torch.Tensor:
    terrain: TerrainImporter = env.scene.terrain
    command = env.commands
    distance = torch.norm(env.base_pos[env_ids, :2] - env.scene.env_origins[env_ids, :2], dim=1)
    move_up = distance > terrain.cfg.terrain_generator.size[0] / 2
    move_down = distance < torch.norm(command[env_ids, :2], dim=1) * env.max_episode_length_s * 0.5
    move_down *= ~move_up
    terrain.update_env_origins(env_ids, move_up, move_down)
    return torch.mean(terrain.terrain_levels.float())


# ---------- #
# | Reward | #
# ---------- #
def tracking_lin_vel_reward(env, tracking_sigma: float = 0.25):
    lin_vel_command_error = torch.sum(torch.square(env.commands[:, :2] - env.base_lin_vel[:, :2]), dim=1)
    return torch.exp(-lin_vel_command_error / tracking_sigma)

def tracking_ang_vel_reward(env, tracking_sigma: float = 0.25):
    ang_vel_command_error = torch.square(env.commands[:, 2] - env.base_ang_vel[:, 2])
    return torch.exp(-ang_vel_command_error / tracking_sigma)

def lin_vel_z_penalty(env):
    return torch.square(env.base_lin_vel[:, 2])

def ang_vel_xy_penalty(env):
    return torch.sum(torch.square(env.base_ang_vel[:, :2]), dim=1)

def base_height_penalty(env, base_height_target: float = 0.5):
    base_height = torch.mean(env.base_pos[:, 2].unsqueeze(1) - env.measured_heights, dim=1)
    return torch.square(base_height - base_height_target)

def collision_penalty(env):
    return torch.sum((torch.norm(env.contact_forces[:, env.penalised_contact_indices, :], dim=-1) > 1.0), dim=1)

def feet_slip_reward(env, tracking_sigma: float = 0.25):
    contact = env.contact_forces[:, env.feet_contact_indices, 2] > 1.0
    contact_filt = torch.logical_or(contact, env.last_contact)
    env.last_contact = contact

    feet_vel = torch.square(torch.norm(env.foot_velocities[:, :, 0:2], dim=2).view(env.num_envs, -1))

    slip_error = torch.sum(contact_filt * feet_vel, dim=1)
    return torch.exp(-slip_error / tracking_sigma)

def hip_pos_penalty(env):
    return torch.sum(torch.square(env.dof_pos[:, [0, 3, 6, 9]]), dim=1)
    
def torques_penalty(env):
    return torch.sum(torch.square(env.torques), dim=1)

def dof_vel_penalty(env):
    return torch.sum(torch.square(env.dof_vel), dim=1)

def dof_acc_penalty(env):
    return torch.sum(torch.square((env.last_dof_vel - env.dof_vel) / env.step_dt), dim=1)

def action_rate_penalty(env):
    return torch.sum(torch.square(env.last_actions - env.actions), dim=1)

def action_smoothness_penalty(env):
    return torch.sum(torch.square(env.actions - env.last_actions - env.last_actions + env.last_last_actions), dim=1)

def all_feet_contact_reward(env):
    is_command_zero = torch.norm(env.commands, dim=1) < 0.05
    feet_contact = env.contact_forces[:, env.feet_contact_indices, 2] > 1.0
    return torch.all(feet_contact, dim=1).float() * is_command_zero.float()


# --------------- #
# | Observation | #
# --------------- #
def base_lin_vel(env) -> torch.Tensor:
    return env.base_lin_vel

def base_ang_vel(env) -> torch.Tensor:
    return env.base_ang_vel

def projected_gravity(env) -> torch.Tensor:
    return env.projected_gravity

def velocity_commands(env) -> torch.Tensor:
    commands_scale = torch.tensor([2.0, 2.0, 0.25], device=env.device, requires_grad=False)
    return env.commands * commands_scale

def dof_pos(env) -> torch.Tensor:
    return env.dof_pos - env.default_dof_pos

def dof_vel(env):
    return env.dof_vel

def actions(env) -> torch.Tensor:
    return env.actions

def height_scan(env, offset: float = 0.5) -> torch.Tensor:
    height = torch.clip(env.measured_heights - env.base_pos[:, 2].unsqueeze(1) + offset, -1, 1)
    return height


# --------------- #
# | Termination | #
# --------------- #
def time_out(env) -> torch.Tensor:
    return env.episode_length_buf >= env.max_episode_length

def illegal_contact(env) -> torch.Tensor:
    return torch.any(torch.norm(env.contact_forces[:, env.termination_contact_indices, :], dim=-1) > 1.0, dim=1)
