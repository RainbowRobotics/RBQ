# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to play a checkpoint if an RL agent from RSL-RL."""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys

from isaaclab.app import AppLauncher

# local imports
from rbq_lab.utils import cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point", help="Name of the RL agent configuration entry point."
)
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    help="Use the pre-trained checkpoint from Nucleus.",
)
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")

parser.add_argument("--no_keyboard", action="store_true", default=False, help="Disable keyboard input.")


# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import os
import time
import torch
import json

from rsl_rl.runners import DistillationRunner, OnPolicyRunner

from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.dict import print_dict
from isaaclab_rl.utils.pretrained_checkpoint import get_published_pretrained_checkpoint

from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg, RslRlVecEnvWrapper, export_policy_as_jit, export_policy_as_onnx

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

# PLACEHOLDER: Extension template (do not remove this comment)
import envs  # noqa: F401
from rbq_lab.utils.keyboard import KeyboardController

@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg, agent_cfg: RslRlBaseRunnerCfg):
    """Play with RSL-RL agent."""
    # grab task name for checkpoint path
    task_name = args_cli.task.split(":")[-1]
    train_task_name = task_name.replace("-Play", "")

    # override configurations with non-hydra CLI arguments
    agent_cfg: RslRlBaseRunnerCfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs

    # set the environment seed
    # note: certain randomizations occur in the environment initialization so we set the seed here
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # specify directory for logging experiments
    log_root_path = os.path.join("logs", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint(train_task_name)
        if not resume_path:
            print("[INFO] Unfortunately a pre-trained checkpoint is currently unavailable for this task.")
            return
    elif args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    log_dir = os.path.dirname(resume_path)

    # set the log directory for the environment (works for all environment types)
    env_cfg.log_dir = log_dir

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    print(f"[INFO]: Loading model checkpoint from: {resume_path}")
    # load previously trained model
    if agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    else:
        raise ValueError(f"Unsupported runner class: {agent_cfg.class_name}")
    runner.load(resume_path)

    # obtain the trained policy for inference
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    # extract the neural network module
    # we do this in a try-except to maintain backwards compatibility.
    try:
        # version 2.3 onwards
        policy_nn = runner.alg.policy
    except AttributeError:
        # version 2.2 and below
        policy_nn = runner.alg.actor_critic

    # extract the normalizer
    if hasattr(policy_nn, "actor_obs_normalizer"):
        normalizer = policy_nn.actor_obs_normalizer
    elif hasattr(policy_nn, "student_obs_normalizer"):
        normalizer = policy_nn.student_obs_normalizer
    else:
        normalizer = None

    dt = env.unwrapped.step_dt
    obs = env.get_observations()

    # export policy to onnx/jit
    export_model_dir = os.path.join(log_root_path, "exported")
    export_policy_as_jit(policy_nn, normalizer=normalizer, path=export_model_dir, filename="policy.jit")
    export_policy_as_onnx(policy_nn, normalizer=normalizer, path=export_model_dir, filename="policy.onnx")

    # Export config info as JSON
    config_info = {
        "run_name": agent_cfg.experiment_name,
        "control": {
            "policy_dt": env_cfg.decimation * env_cfg.sim.dt,
            "action_scale": env_cfg.actions.scale,
            "stiffness": {
                "R": env_cfg.scene.robot.actuators["legs_R"].stiffness,
                "P": env_cfg.scene.robot.actuators["legs_P"].stiffness,
                "K": env_cfg.scene.robot.actuators["legs_K"].stiffness,
            },
            "damping" : { 
                "R": env_cfg.scene.robot.actuators["legs_R"].damping,
                "P": env_cfg.scene.robot.actuators["legs_P"].damping,
                "K": env_cfg.scene.robot.actuators["legs_K"].damping,
            },
        },
        "env": {
            "num_observations": obs["policy"].shape[1],
            "num_actions": env_cfg.actions.num_actions,
        },
        "commands": {
            "ranges": {
                "lin_vel_x": env_cfg.commands.ranges.lin_vel_x,
                "lin_vel_y": env_cfg.commands.ranges.lin_vel_y,
                "ang_vel_yaw": env_cfg.commands.ranges.ang_vel_z,
            },
        },
        "init_state": {
            "default_joint_angles": {
                "joint0_HRR": env_cfg.scene.robot.init_state.joint_pos["joint0_HRR"],
                "joint1_HRP": env_cfg.scene.robot.init_state.joint_pos["joint1_HRP"],
                "joint2_HRK": env_cfg.scene.robot.init_state.joint_pos["joint2_HRK"],
                "joint3_HLR": env_cfg.scene.robot.init_state.joint_pos["joint3_HLR"],
                "joint4_HLP": env_cfg.scene.robot.init_state.joint_pos["joint4_HLP"],
                "joint5_HLK": env_cfg.scene.robot.init_state.joint_pos["joint5_HLK"],
                "joint6_FRR": env_cfg.scene.robot.init_state.joint_pos["joint6_FRR"],
                "joint7_FRP": env_cfg.scene.robot.init_state.joint_pos["joint7_FRP"],
                "joint8_FRK": env_cfg.scene.robot.init_state.joint_pos["joint8_FRK"],
                "joint9_FLR": env_cfg.scene.robot.init_state.joint_pos["joint9_FLR"],
                "joint10_FLP": env_cfg.scene.robot.init_state.joint_pos["joint10_FLP"],
                "joint11_FLK": env_cfg.scene.robot.init_state.joint_pos["joint11_FLK"],
            },
        },
        "normalization": {
            "obs_scales": {
                "lin_vel": env_cfg.observations_cfg.scales.lin_vel,
                "ang_vel": env_cfg.observations_cfg.scales.ang_vel,
                "dof_pos": env_cfg.observations_cfg.scales.dof_pos,
                "dof_vel": env_cfg.observations_cfg.scales.dof_vel,
            },
            "clip_observations": env_cfg.observations_cfg.clip_obs,
            "clip_actions": env_cfg.actions.raw_actions_limit,
        }
    }
    info = {
        'config_info': config_info
    }
    info_path = os.path.join(export_model_dir, 'info.json')
    with open(info_path, 'w') as f:
        json.dump(info, f, indent=4)
    print('Exported config info to: ', info_path)

    if not args_cli.no_keyboard:
    	keyboard_controller = KeyboardController(env_cfg.commands.ranges)
    	keyboard_controller.start_listener()

    # reset environment
    timestep = 0
    # simulate environment
    while simulation_app.is_running():
        start_time = time.time()
        # run everything in inference mode
        with torch.inference_mode():
            if not args_cli.no_keyboard:
                keyboard_controller.update_velocity()
                keyboard_controller.update_commands(env)       
            # agent stepping
            actions = policy(obs)
            # env stepping
            obs, _, _, _ = env.step(actions)
        if args_cli.video:
            timestep += 1
            # Exit the play loop after recording one video
            if timestep == args_cli.video_length:
                break

        # time delay for real-time evaluation
        sleep_time = dt - (time.time() - start_time)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
