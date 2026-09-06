# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
from multiprocessing import Process, Value


class Logger:
    def __init__(self, dt):
        self.state_log = defaultdict(list)
        self.rew_log = defaultdict(list)
        self.dt = dt
        self.num_episodes = 0
        self.plot_process = None

    def log_state(self, key, value):
        self.state_log[key].append(value)

    def log_states(self, dict):
        for key, value in dict.items():
            self.log_state(key, value)

    def log_env(self, env, i, stop_state_log):
        if i < stop_state_log:
            action = {f'action_{j}': env.actions[0, j].item() * env.cfg.control.action_scale for j in range(env.num_dofs)}
            dof_pos = {f'dof_pos_{j}': env.dof_pos[0, j].item() for j in range(env.num_dofs)}
            dof_vel = {f'dof_vel_{j}': env.dof_vel[0, j].item() for j in range(env.num_dofs)}
            dof_torque = {f'dof_torque_{j}': env.torques[0, j].item() for j in range(env.num_dofs)}


            command = {f'command_{axis}': env.commands[0, i].item() for i, axis in enumerate(['x', 'y', 'yaw'])}
            base_pos = {f'base_pos_{axis}': env.base_pos[0, i].item() for i, axis in enumerate(['x', 'y', 'z'])}
            base_lin_vel = {f'base_lin_vel_{axis}': env.base_lin_vel[0, i].item() for i, axis in enumerate(['x', 'y', 'z'])}
            base_ang_vel = {f'base_ang_vel_{axis}': env.base_ang_vel[0, i].item() for i, axis in enumerate(['x', 'y', 'z'])}

            contact_forces = {f'contact_force_{j}_{axis}': env.contact_forces[0, env.feet_indices[j], axis_idx].item()/1000 for j in range(len(env.feet_indices)) for axis_idx, axis in enumerate(['x', 'y', 'z'])
            }


            swing_time = {f'swing_time_{j}': env.feet_swingTtime[0, j].item() for j in range(4)}

            self.log_states({
                **action,
                **dof_pos,
                **dof_vel,
                **dof_torque,
                **command,
                **base_pos,
                **base_lin_vel,
                **base_ang_vel,
                **contact_forces,
                **swing_time
            })

    def log_rewards(self, dict, num_episodes):
        for key, value in dict.items():
            if 'rew' in key:
                self.rew_log[key].append(value.item() * num_episodes)
        self.num_episodes += num_episodes

    def reset(self):
        self.state_log.clear()
        self.rew_log.clear()

    def plot_states(self):
        self.plot_process = Process(target=self._plot)
        self.plot_process.start()

    def _plot(self):
        nb_rows = 3
        nb_cols = 3
        fig, axs = plt.subplots(nb_rows, nb_cols)
        for key, value in self.state_log.items():
            time = np.linspace(0, len(value)*self.dt, len(value))
            break
        log= self.state_log
        # plot joint targets and measured positions
        a = axs[1, 0]
        if log["dof_pos"]: a.plot(time, log["dof_pos"], label='measured')
        if log["dof_pos_target"]: a.plot(time, log["dof_pos_target"], label='target')
        a.set(xlabel='time [s]', ylabel='Position [rad]', title='DOF Position')
        a.legend()
        # plot joint velocity
        a = axs[1, 1]
        if log["dof_vel"]: a.plot(time, log["dof_vel"], label='measured')
        if log["dof_vel_target"]: a.plot(time, log["dof_vel_target"], label='target')
        a.set(xlabel='time [s]', ylabel='Velocity [rad/s]', title='Joint Velocity')
        a.legend()
        # plot base vel x
        a = axs[0, 0]
        if log["base_vel_x"]: a.plot(time, log["base_vel_x"], label='measured')
        if log["command_x"]: a.plot(time, log["command_x"], label='commanded')
        a.set(xlabel='time [s]', ylabel='base lin vel [m/s]', title='Base velocity x')
        a.legend()
        # plot base vel y
        a = axs[0, 1]
        if log["base_vel_y"]: a.plot(time, log["base_vel_y"], label='measured')
        if log["command_y"]: a.plot(time, log["command_y"], label='commanded')
        a.set(xlabel='time [s]', ylabel='base lin vel [m/s]', title='Base velocity y')
        a.legend()
        # plot base vel yaw
        a = axs[0, 2]
        if log["base_vel_yaw"]: a.plot(time, log["base_vel_yaw"], label='measured')
        if log["command_yaw"]: a.plot(time, log["command_yaw"], label='commanded')
        a.set(xlabel='time [s]', ylabel='base ang vel [rad/s]', title='Base velocity yaw')
        a.legend()
        # plot base vel z
        a = axs[1, 2]
        if log["base_vel_z"]: a.plot(time, log["base_vel_z"], label='measured')
        a.set(xlabel='time [s]', ylabel='base lin vel [m/s]', title='Base velocity z')
        a.legend()
        # plot contact forces
        a = axs[2, 0]
        if log["contact_forces_z"]:
            forces = np.array(log["contact_forces_z"])
            for i in range(forces.shape[1]):
                a.plot(time, forces[:, i], label=f'force {i}')
        a.set(xlabel='time [s]', ylabel='Forces z [N]', title='Vertical Contact forces')
        a.legend()
        # plot torque/vel curves
        a = axs[2, 1]
        if log["dof_vel"]!=[] and log["dof_torque"]!=[]: a.plot(log["dof_vel"], log["dof_torque"], 'x', label='measured')
        a.set(xlabel='Joint vel [rad/s]', ylabel='Joint Torque [Nm]', title='Torque/velocity curves')
        a.legend()
        # plot torques
        a = axs[2, 2]
        if log["dof_torque"]!=[]: a.plot(time, log["dof_torque"], label='measured')
        a.set(xlabel='time [s]', ylabel='Joint Torque [Nm]', title='Torque')
        a.legend()
        plt.show()

    def plot_swing_time(self):
        self.plot_process = Process(target=self._plot_swing_time)
        self.plot_process.start()

    def _plot_swing_time(self):
        nb_rows = 2
        nb_cols = 1
        fig, axs = plt.subplots(nb_rows, nb_cols)
        for key, value in self.state_log.items():
            time = np.linspace(0, len(value)*self.dt, len(value))
            break
        log= self.state_log

        a = axs[0]
        if log["swing_time_0"]: a.plot(time, log['swing_time_0'], label='HR_swing_time')
        if log["contact_force_0_z"]: a.plot(time, log["contact_force_0_z"], label='HR')

        a.set(xlabel='time [s]', ylabel='time [s]', title='rear_swing_time')
        a.legend()

        a = axs[1]
        if log["swing_time_1"]: a.plot(time, log['swing_time_1'], label='HL_swing_time')
        if log["contact_force_1_z"]: a.plot(time, log["contact_force_1_z"], label='HL')

        a.set(xlabel='time [s]', ylabel='time [s]', title='rear_swing_time')
        a.legend()

        # a = axs[1]
        # if log["swing_time_2"]: a.plot(log['swing_time_2'], label='front_swing_time')
        # if log["swing_time_3"]: a.plot(log["swing_time_3"], label='front_swing_time')
        # a.set(xlabel='time [s]', ylabel='time [s]', title='front_swing_time')
        # a.legend()

        plt.show()

    def plot_base_state(self):
        self.plot_process = Process(target=self._plot_base_state)
        self.plot_process.start()

    def _plot_base_state(self):
        nb_rows = 2
        nb_cols = 3
        fig, axs = plt.subplots(nb_rows, nb_cols)
        for key, value in self.state_log.items():
            time = np.linspace(0, len(value)*self.dt, len(value))
            break
        log= self.state_log

        a = axs[0, 0]
        if log["base_pos_x"] and log["base_pos_y"]: a.plot(log['base_pos_x'], log["base_pos_y"], label='base_pos_xy')
        a.set(xlabel='X [m]', ylabel='Y [m]', title='Base Position XY')
        a.legend()

        a = axs[0, 1]
        if log["base_pos_z"]: a.plot(time, log["base_pos_z"], label='base_pos_z')
        a.set(xlabel='time [s]', ylabel='Z [m]', title='Base Position Z')
        a.legend()

        a = axs[1, 0]
        if log["base_lin_vel_x"]: a.plot(time, log["base_lin_vel_x"], label='base_lin_vel_x')
        if log["command_x"]: a.plot(time, log["command_x"], label='command_x')
        a.set(xlabel='time [s]', ylabel='Lin Vel [m/s]', title='Base Velocity X')
        a.legend()

        a = axs[1, 1]
        if log["base_lin_vel_y"]: a.plot(time, log["base_lin_vel_y"], label='base_lin_vel_y')
        if log["command_y"]: a.plot(time, log["command_y"], label='command_y')
        a.set(xlabel='time [s]', ylabel='Lin Vel [m/s]', title='Base Velocity Y')
        a.legend()

        a = axs[1, 2]
        if log["base_ang_vel_z"]: a.plot(time, log["base_ang_vel_z"], label='base_ang_vel_z')
        if log["command_yaw"]: a.plot(time, log["command_yaw"], label='command_yaw')
        a.set(xlabel='time [s]', ylabel='Ang Vel [rad/s]', title='Base Velocity Yaw')
        a.legend()


        plt.show()

    def plot_dof_vel(self):
        self.plot_process = Process(target=self._plot_dof_vel)
        self.plot_process.start()

    def _plot_dof_vel(self):
        nb_rows = 2
        nb_cols = 2
        fig, axs = plt.subplots(nb_rows, nb_cols)
        for key, value in self.state_log.items():
            time = np.linspace(0, len(value)*self.dt, len(value))
            break
        log= self.state_log

        a = axs[0, 0]
        if log["dof_vel_0"]: a.plot(time, log["dof_vel_0"], label='HR_ROLL')
        if log["dof_vel_1"]: a.plot(time, log["dof_vel_1"], label='HR_PITCH')
        if log["dof_vel_2"]: a.plot(time, log["dof_vel_2"], label='HR_KNEE')
        a.set(xlabel='time [s]', ylabel='Dof Vel [rad/s]', title='HR Dof Vel')
        a.legend()

        a = axs[0, 1]
        if log["dof_vel_3"]: a.plot(time, log["dof_vel_3"], label='HL_ROLL')
        if log["dof_vel_4"]: a.plot(time, log["dof_vel_4"], label='HL_PITCH')
        if log["dof_vel_5"]: a.plot(time, log["dof_vel_5"], label='HL_KNEE')
        a.set(xlabel='time [s]', ylabel='Dof Vel [rad/s]', title='HL Dof Vel')
        a.legend()

        a = axs[1, 0]
        if log["dof_vel_6"]: a.plot(time, log["dof_vel_6"], label='FR_ROLL')
        if log["dof_vel_7"]: a.plot(time, log["dof_vel_7"], label='FR_PITCH')
        if log["dof_vel_8"]: a.plot(time, log["dof_vel_8"], label='FR_KNEE')
        a.set(xlabel='time [s]', ylabel='Dof Vel [rad/s]', title='FR Dof Vel')
        a.legend()

        a = axs[1, 1]
        if log["dof_vel_9"]: a.plot(time, log["dof_vel_9"], label='FL_ROLL')
        if log["dof_vel_10"]: a.plot(time, log["dof_vel_10"], label='FL_PITCH')
        if log["dof_vel_11"]: a.plot(time, log["dof_vel_11"], label='FL_KNEE')
        a.set(xlabel='time [s]', ylabel='Dof Vel [rad/s]', title='FL Dof Vel')
        a.legend()

        plt.show()

    def plot_torques(self):
        self.plot_process = Process(target=self._plot_torques)
        self.plot_process.start()

    def _plot_torques(self):
        nb_rows = 2
        nb_cols = 2
        fig, axs = plt.subplots(nb_rows, nb_cols)
        for key, value in self.state_log.items():
            time = np.linspace(0, len(value)*self.dt, len(value))
            break
        log= self.state_log

        a = axs[0, 0]
        if log["dof_torque_0"]: a.plot(time, log["dof_torque_0"], label='HR_ROLL')
        if log["dof_torque_1"]: a.plot(time, log["dof_torque_1"], label='HR_PITCH')
        if log["dof_torque_2"]: a.plot(time, log["dof_torque_2"], label='HR_KNEE')
        a.set(xlabel='time [s]', ylabel='Torque [Nm]', title='HR Torques')
        a.legend()

        a = axs[0, 1]
        if log["dof_torque_3"]: a.plot(time, log["dof_torque_3"], label='HL_ROLL')
        if log["dof_torque_4"]: a.plot(time, log["dof_torque_4"], label='HL_PITCH')
        if log["dof_torque_5"]: a.plot(time, log["dof_torque_5"], label='HL_KNEE')
        a.set(xlabel='time [s]', ylabel='Torque [Nm]', title='HL Torques')
        a.legend()

        a = axs[1, 0]
        if log["dof_torque_6"]: a.plot(time, log["dof_torque_6"], label='FR_ROLL')
        if log["dof_torque_7"]: a.plot(time, log["dof_torque_7"], label='FR_PITCH')
        if log["dof_torque_8"]: a.plot(time, log["dof_torque_8"], label='FR_KNEE')
        a.set(xlabel='time [s]', ylabel='Torque [Nm]', title='FR Torques')
        a.legend()

        a = axs[1, 1]
        if log["dof_torque_9"]: a.plot(time, log["dof_torque_9"], label='FL_ROLL')
        if log["dof_torque_10"]: a.plot(time, log["dof_torque_10"], label='FL_PITCH')
        if log["dof_torque_11"]: a.plot(time, log["dof_torque_11"], label='FL_KNEE')
        a.set(xlabel='time [s]', ylabel='Torque [Nm]', title='FL Torques')
        a.legend()

        plt.show()

    def plot_contact_forces(self):
        self.plot_process = Process(target=self._plot_contact_forces)
        self.plot_process.start()

    def _plot_contact_forces(self):
        nb_rows = 1
        nb_cols = 1
        fig, axs = plt.subplots(nb_rows, nb_cols)
        for key, value in self.state_log.items():
            time = np.linspace(0, len(value)*self.dt, len(value))
            break
        log= self.state_log

        a = axs
        if log["contact_force_0_z"]: a.plot(time, log["contact_force_0_z"], label='HR')
        if log["contact_force_1_z"]: a.plot(time, log["contact_force_1_z"], label='HL')
        if log["contact_force_2_z"]: a.plot(time, log["contact_force_2_z"], label='FR')
        if log["contact_force_3_z"]: a.plot(time, log["contact_force_3_z"], label='FL')
        a.set(xlabel='time [s]', ylabel='Force [N]', title='Contact Forces')
        a.legend()
        plt.show()

    def print_rewards(self):
        print("Average rewards per second:")
        for key, values in self.rew_log.items():
            mean = np.sum(np.array(values)) / self.num_episodes
            print(f" - {key}: {mean}")
        print(f"Total number of episodes: {self.num_episodes}")

    def __del__(self):
        if self.plot_process is not None:
            self.plot_process.kill()