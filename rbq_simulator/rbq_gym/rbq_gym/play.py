import os
import json

import numpy as np

import isaacgym
import torch

from rbq_gym import GYM_ROOT_DIR
from rbq_gym.envs import *
from rbq_gym.utils import  get_args, export_policy_as_jit, task_registry, Logger
from rbq_gym.utils.keyboard import KeyboardController  

def play(args):
    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)
    env_cfg.env.test = True
    # override some parameters for testing
    env_cfg.env.num_envs = min(env_cfg.env.num_envs, 5)
    env_cfg.terrain.num_rows = 5
    env_cfg.terrain.num_cols = 5
    env_cfg.terrain.curriculum = False
    env_cfg.noise.add_noise = False
    env_cfg.domain_rand.randomize_friction = True
    env_cfg.terrain.static_friction = 0.45
    env_cfg.terrain.dynamic_friction = 0.4
    env_cfg.domain_rand.friction_range = [0.8, 0.8]
    env_cfg.domain_rand.push_robots = False
    env_cfg.domain_rand.push_interval_s = 3 # sec
    env_cfg.domain_rand.max_push_vel_xy = 2.0 # m/s
    env_cfg.domain_rand.randomize_base_mass = True 
    env_cfg.env.episode_length_s = 200000

    # prepare environment
    env, _ = task_registry.make_env(name=args.task, args=args, env_cfg=env_cfg)
    obs = env.get_observations()
    # load policy
    train_cfg.runner.resume = True
    ppo_runner, train_cfg = task_registry.make_alg_runner(env=env, name=args.task, args=args, train_cfg=train_cfg)
    policy = ppo_runner.get_inference_policy(device=env.device)

    onnx_export_dir = os.path.join(GYM_ROOT_DIR, 'logs', train_cfg.runner.experiment_name, 'exported', 'policies')
    os.makedirs(onnx_export_dir, exist_ok=True)

    actor_critic = ppo_runner.alg.actor_critic.actor
    actor_critic.eval()

    obs_shape = obs.shape[1]
    dummy_input = torch.randn(1, obs_shape).to(env.device)

    onnx_path = os.path.join(onnx_export_dir, 'policy.onnx')
    torch.onnx.export(
        actor_critic,
        dummy_input,
        onnx_path,
        export_params=True,
        opset_version=11,
        do_constant_folding=True,
        input_names=['obs'],
        output_names=['actions'],
        dynamic_axes={'obs': {0: 'batch_size'}, 'actions': {0: 'batch_size'}}
    )
    print('Exported policy as onnx script to: ', onnx_path)

    # Export config info as JSON
    export_info = {
        "onnx_path": onnx_path,
        "obs_shape": obs_shape,
    }
    config_info = {
        "run_name": train_cfg.runner.experiment_name,
        "control": {
            "policy_dt": env_cfg.control.decimation * env_cfg.sim.dt,
            "action_scale": env_cfg.control.action_scale,
            "stiffness": env_cfg.control.stiffness,
            "damping": env_cfg.control.damping,
        },
        "env": {
            "num_observations": env_cfg.env.num_observations,
            "num_actions": env_cfg.env.num_actions,
        },
        "terrain": {
            "measured_obs_points_num": len(env_cfg.terrain.measured_points_x),
            "measured_obs_points_xy": env_cfg.terrain.measured_points_y,
        },
        "commands": {
            "ranges": {
                "lin_vel_x": env_cfg.commands.ranges.lin_vel_x,
                "lin_vel_y": env_cfg.commands.ranges.lin_vel_y,
                "ang_vel_yaw": env_cfg.commands.ranges.ang_vel_yaw,
            },
        },
        "init_state": {
            "default_joint_angles": env_cfg.init_state.default_joint_angles,
        },
        "normalization": {
            "obs_scales": {
                "lin_vel": env_cfg.normalization.obs_scales.lin_vel,
                "ang_vel": env_cfg.normalization.obs_scales.ang_vel,
                "dof_pos": env_cfg.normalization.obs_scales.dof_pos,
                "dof_vel": env_cfg.normalization.obs_scales.dof_vel,
                "height_measurements": env_cfg.normalization.obs_scales.height_measurements,
            },
            "clip_observations": env_cfg.normalization.clip_observations,
            "clip_actions": env_cfg.normalization.clip_actions,
        }
    }
    info = {
        'export_info': export_info,
        'config_info': config_info
    }
    info_path = os.path.join(onnx_export_dir, 'info.json')
    with open(info_path, 'w') as f:
        json.dump(info, f, indent=4)
    print('Exported config info to: ', info_path)

    logger = Logger(env.dt)
    robot_index = 0 # which robot is used for logging
    joint_index = 1 # which joint is used for logging
    stop_state_log = 1000 # number of steps before plotting states
    stop_rew_log = env.max_episode_length + 1 # number of steps before print average episode rewards
    camera_position = np.array(env_cfg.viewer.pos, dtype=np.float64)
    camera_vel = np.array([1., 1., 0.])
    camera_direction = np.array(env_cfg.viewer.lookat) - np.array(env_cfg.viewer.pos)
    img_idx = 0

    keyboard_controller = KeyboardController(env_cfg.commands.ranges)
    keyboard_controller.start_listener()
    
    for i in range(10*int(env.max_episode_length)):
        keyboard_controller.update_velocity()
        keyboard_controller.update_commands(env)       
        actions = policy(obs.detach())
        obs, _, rews, dones, infos = env.step(actions.detach())
        if RECORD_FRAMES:
            if i % 2:
                filename = os.path.join(GYM_ROOT_DIR, 'logs', train_cfg.runner.experiment_name, 'exported', 'frames', f"{img_idx}.png")
                env.gym.write_viewer_image_to_file(env.viewer, filename)
                img_idx += 1
                
        if args.tracking_camera:
            camera_position = env.root_states[0, 0:3].numpy() + np.array([-1, -1.5, 1])
            env.set_camera(camera_position, camera_position + camera_direction)

        if args.plot:
            if i < stop_state_log:
                logger.log_env(env, i, stop_state_log)
            elif i==stop_state_log:
                logger.plot_states()
                # logger.plot_swing_time()
                # logger.plot_base_state()
                # logger.plot_torques()
                # logger.plot_contact_forces()
                # logger.plot_dof_vel()

        if  0 < i < stop_rew_log:
            if infos["episode"]:
                num_episodes = torch.sum(env.reset_buf).item()
                if num_episodes>0:
                    logger.log_rewards(infos["episode"], num_episodes)
        elif i==stop_rew_log:
            logger.print_rewards()

if __name__ == '__main__':
    EXPORT_POLICY = True
    EXPORT_ONNX = True
    RECORD_FRAMES = False
    MOVE_CAMERA = False
    args = get_args()
    play(args)
