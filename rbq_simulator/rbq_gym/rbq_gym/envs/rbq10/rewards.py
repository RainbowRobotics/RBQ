from isaacgym.torch_utils import *
import torch

from rbq_gym.utils.math import *

class Rewards:
    def __init__(self, env):
        self.env = env

    def load_env(self, env):
        self.env = env

    # region LOCOMOTION ----------------------------------
    def _reward_tracking_lin_vel(self):
        # Tracking of linear velocity commands (xy axes)
        lin_vel_error = torch.sum(torch.square(self.env.commands[:, :2] - self.env.base_lin_vel[:, :2]), dim=1)
        return torch.exp(-lin_vel_error/self.env.cfg.rewards.tracking_sigma)

    def _reward_tracking_ang_vel(self):
        # Tracking of angular velocity commands (yaw) 
        ang_vel_error = torch.square(self.env.commands[:, 2] - self.env.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error/self.env.cfg.rewards.tracking_sigma)
    # endregion

    # region POSTURE -------------------------------------
    def _reward_orientation(self):
        # Penalize non flat base orientation
        return torch.sum(torch.square(self.env.projected_gravity[:, :2]), dim=1)

    def _reward_base_height(self):
        # Penalize base height away from target
        base_height = torch.mean(self.env.root_states[:, 2].unsqueeze(1) - self.env.measured_heights, dim=1)
        return torch.square(base_height - self.env.cfg.rewards.base_height_target)

    def _reward_hip_pos(self):

        target_hip_angle = torch.zeros((self.env.num_envs, 4), device=self.env.device, dtype=torch.float)  # (num_envs, 4)

        current_hip_angle = self.env.dof_pos[:, [0, 3, 6, 9]]  # (num_envs, 4)

        hip_angle_error = current_hip_angle - target_hip_angle  # (num_envs, 4)

        hip_angle_error_sum = torch.sum(torch.square(hip_angle_error), dim=1)  # (num_envs,)
        reward = torch.exp(-hip_angle_error_sum / self.env.cfg.rewards.tracking_sigma) 

        return reward
    
    def _reward_lin_vel_z(self):
        # Penalize z axis base linear velocity
        return torch.square(self.env.base_lin_vel[:, 2])
    
    def _reward_ang_vel_xy(self):
        # Penalize xy axes base angular velocity using exp(-x) form
        ang_vel_error = torch.sum(torch.square(self.env.base_ang_vel[:, :2]), dim=1)
        reward = torch.exp(-ang_vel_error / self.env.cfg.rewards.tracking_sigma)
        return reward

    def _reward_feet_slip(self):
        contact = self.env.contact_forces[:, self.env.feet_indices, 2] > 1.0
        contact_filt = torch.logical_or(contact, self.env.last_contacts)
        self.env.last_contacts = contact

        foot_velocities = torch.square(torch.norm(self.env.foot_velocities[:, :, 0:2], dim=2).view(self.env.num_envs, -1))

        slip_error = torch.sum(contact_filt * foot_velocities, dim=1)

        reward = torch.exp(-slip_error / self.env.cfg.rewards.tracking_sigma)

        return reward
    
    def _reward_dof_pos_limits(self):
        # Penalize dof positions too close to the limit
        out_of_limits = -(self.env.dof_pos - self.env.dof_pos_limits[:, 0]).clip(max=0.) # lower limit
        out_of_limits += (self.env.dof_pos - self.env.dof_pos_limits[:, 1]).clip(min=0.)
        return torch.sum(out_of_limits, dim=1)

    def _reward_dof_vel_limits(self):
        # Penalize dof velocities too close to the limit
        # clip to max error = 1 rad/s per joint to avoid huge penalties
        return torch.sum((torch.abs(self.env.dof_vel) - self.env.dof_vel_limits*self.env.cfg.rewards.soft_dof_vel_limit).clip(min=0., max=1.), dim=1)

    def _reward_torque_limits(self):
        # penalize torques too close to the limit
        return torch.sum((torch.abs(self.env.torques) - self.env.torque_limits*self.env.cfg.rewards.soft_torque_limit).clip(min=0.), dim=1)
    # endregion

    # region ENERGY --------------------------------------
    def _reward_dof_pos(self):
        # Penalize dof positions
        return torch.sum(torch.square(self.env.dof_pos - self.env.default_dof_pos), dim=1)
    
    def _reward_dof_vel(self):
        # Penalize dof velocities
        return torch.sum(torch.square(self.env.dof_vel), dim=1)
    
    def _reward_dof_acc(self):
        # Penalize dof accelerations
        return torch.sum(torch.square((self.env.last_dof_vel - self.env.dof_vel) / self.env.dt), dim=1)
    
    def _reward_action_rate(self):
        # Penalize changes in actions
        return torch.sum(torch.square(self.env.last_actions - self.env.actions), dim=1)

    def _reward_torques(self):
        # Penalize torques
        return torch.sum(torch.square(self.env.torques), dim=1)

    def _reward_delta_torques(self):
        return torch.sum(torch.square(self.env.last_torques - self.env.torques), dim=1)

    def _reward_joint_power(self):
        #Penalize high power
        return torch.sum(self.env.dof_vel * self.env.torques, dim=1)    

    def _reward_smoothness(self):
        # second order smoothness
        return torch.sum(torch.square(self.env.actions - self.env.last_actions - self.env.last_actions + self.env.last_last_actions), dim=1)

    def _reward_feet_air_time(self):
        target_air_time = 0.4 

        contact = self.env.contact_forces[:, self.env.feet_indices, 2] > 1.0
        contact_filt = torch.logical_or(contact, self.env.last_contacts)
        self.env.last_contacts = contact
        first_contact = (self.env.feet_air_time > 0.0) & contact_filt
        self.env.feet_air_time += self.env.dt
        error = self.env.feet_air_time - target_air_time

        rew_airTime = torch.sum((error ** 2) * first_contact, dim=1)
        rew_airTime *= (torch.norm(self.env.commands[:, :2], dim=1) > 0.1)

        self.env.feet_air_time *= ~contact_filt
        return rew_airTime
    # endregion

    # region PENALTIES -----------------------------------
    def _reward_collision(self):
        # Penalize collisions on selected bodies
        return torch.sum(1.*(torch.norm(self.env.contact_forces[:, self.env.penalised_contact_indices, :], dim=-1) > 0.1), dim=1)
    
    def _reward_termination(self):
        # Terminal reward / penalty
        return self.env.reset_buf * ~self.env.time_out_buf
    # endregion

    def _reward_all_feet_contact(self):
        command_norm = torch.norm(self.env.commands[:, :3], dim=1)  # (num_envs,)
        is_command_zero = command_norm < 0.05  # command가 0일 때만 리워드 계산

        contact_threshold = 20.0  
        feet_contact = self.env.contact_forces[:, self.env.feet_indices, 2] > contact_threshold  # (num_envs, num_feet)

        all_feet_contact = torch.all(feet_contact, dim=1)  # (num_envs,)

        all_contact = all_feet_contact.float() * is_command_zero.float()

        return all_contact
    