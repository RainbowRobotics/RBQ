from rbq_gym.envs.base.base_config import BaseConfig

D2R = 0.017453289

class RBQ10Cfg(BaseConfig):
    class env:
        num_envs = 4096
        num_observations = 45
        num_privileged_obs = 45 + 3 + 1 + 12 + 12 + 1 + 17*11 + 12
        num_actions = 12
        env_spacing = 3. 
        send_timeouts = True 
        episode_length_s = 30 
        test = False

    class terrain:
        mesh_type = 'trimesh' # "heightfield" # none, plane, heightfield or trimesh
        horizontal_scale = 0.1 # [m]
        vertical_scale = 0.005 # [m]
        border_size = 25 # [m]
        static_friction = 0.45
        dynamic_friction = 0.4
        restitution = 0.
        # rough terrain only:
        measure_heights = True
        priv_measure_heights = True
        measured_points_x = [-0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8] # 1mx1.6m rectangle (without center line)
        measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]

        terrain_kwargs = None # Dict of arguments for selected terrain
        max_init_terrain_level = 5 # starting curriculum state
        terrain_length = 8.
        terrain_width = 8.
        num_rows= 10 # number of terrain rows (levels)
        num_cols = 20 # number of terrain cols (types)
        # trimesh only:
        slope_treshold = 0.75
        
        selected = False 
        selected_terrain_proportions = [1.0, # noise
                                        0.,  # rough_slope
                                        ] 
        curriculum = True
        curriculum_terrain_proportions = [0.1, # smooth_slope
                                          0.1, # rough_slope
                                          0.2, # stairs_up
                                          0.2, # stairs_down
                                          0.2, # discrete
                                          0., # stepping_stone
                                          0., # gap
                                          0.,  # pit
                                          0.2,
                                          ]   # trimesh only:
        step_height = 0.20

    class commands:
        curriculum = True
        max_curriculum = 2.5
        num_commands = 3 # default: lin_vel_x, lin_vel_y, ang_vel_yaw
        resampling_time = 8. # time before command are changed[s]
        class ranges:
            lin_vel_x = [-1.0, 1.0] # min max [m/s]
            lin_vel_y = [-1.0, 1.0]   # min max [m/s]
            ang_vel_yaw = [-1.3, 1.3]    # min max [rad/s]

    class init_state:
        pos = [0.0, 0.0, 0.5] # x,y,z [m]
        rot = [0.0, 0.0, 0.0, 1.0] # x,y,z,w [quat]
        lin_vel = [0.0, 0.0, 0.0]  # x,y,z [m/s]
        ang_vel = [0.0, 0.0, 0.0]  # x,y,z [rad/s]
        default_joint_angles = { 
            'joint0_HRR': -0.,
            'joint1_HRP': 46.5*D2R,
            'joint2_HRK': -80*D2R,
            'joint3_HLR': 0.,
            'joint4_HLP': 46.5*D2R,
            'joint5_HLK': -80*D2R,
            'joint6_FRR': -0.,
            'joint7_FRP': 45.5*D2R,
            'joint8_FRK': -82*D2R,
            'joint9_FLR': 0.,
            'joint10_FLP': 45.5*D2R,
            'joint11_FLK': -82*D2R,
        }

    class control:
        control_type = 'P' # P: position, V: velocity, T: torqus
        # PD Drive parameters:
        stiffness = {'R': 123.39138, 'P': 123.39138, 'K': 127.77215}  # [N*m/rad]
        damping = {'R': 2.5, 'P': 2.5, 'K': 2.5}     # [N*m*s/rad]

        action_scale = 0.1
        decimation = 5

    class asset:
        file = '{GYM_ASSET_DIR}/urdf/rbq10.urdf'
        name = "rbq10"
        foot_name = "foot"
        penalize_contacts_on = ["calf"]
        terminate_after_contacts_on = ["trunk", "thigh"]
        disable_gravity = False
        collapse_fixed_joints = True # merge bodies connected by fixed joints. Specific fixed joints can be kept by adding " <... dont_collapse="true">
        fix_base_link = False # fixe the base of the robot
        default_dof_drive_mode = 3 # see GymDofDriveModeFlags (0 is none, 1 is pos tgt, 2 is vel tgt, 3 effort)
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter
        replace_cylinder_with_capsule = False # replace collision cylinders with capsules, leads to faster/more stable simulation
        flip_visual_attachments = False # Some .obj meshes must be flipped from y-up to z-up
        
        density = 0.001
        angular_damping = 0.
        linear_damping = 0.
        max_angular_velocity = 1000.
        max_linear_velocity = 1000.
        armature = 0.
        thickness = 0.01

    class domain_rand:
        action_delay = True

        push_robots = True
        push_interval_s = 15
        max_push_vel_xy = 1.5

        randomize_friction = True
        friction_range = [0.4, 0.9]
                
        use_armature = True
        randomize_armature = True
        armature_range = [0.95, 1.05]

        randomize_motor_strength = True
        motor_strength_range = [0.90, 1.1]
        
        randomize_motor_offset = True
        motor_offset_range = [-0.02, 0.02]

        randomize_base_mass = True
        added_mass_range = [-1., 15.]
        
        randomize_base_com = True
        com_offset_range = [-0.06, 0.06]

    class observations:
        priv_friction_coeffs = True # +1 
        priv_motor_strength = True # +12
        priv_motor_offset = True # +12
        priv_armature = False # +12
        priv_base_mass = True # +1
        priv_base_com = False # +1

    class rewards:
        only_positive_rewards = False # if true negative total rewards are clipped at zero (avoids early termination problems)
        tracking_sigma = 0.25 # tracking reward = exp(-error^2/sigma)
        soft_dof_pos_limit = 0.9
        soft_dof_vel_limit = 1.0
        soft_torque_limit = 1.0
        base_height_target = 0.48 
        max_contact_force = 400 # forces above this value are penalized
        class scales:
            tracking_lin_vel = 1.0
            tracking_ang_vel = 0.7
            lin_vel_z = -2.0
            ang_vel_xy = 0.8          
            
            base_height = -5
            collision = -1.0
            feet_slip = 0.5
            
            hip_pos = 0.2

            torques = -0.00001
            dof_vel = -1e-3
            dof_acc = -2.5e-7
            action_rate = -0.005
            smoothness = -0.01

            all_feet_contact = 1.0

    class normalization:
        class obs_scales:
            lin_vel = 2.0
            ang_vel = 0.25
            dof_pos = 1.0
            dof_vel = 0.05
            height_measurements = 5.0
        clip_observations = 100.
        clip_actions = 100.

    class noise:
        add_noise = True
        noise_level = 1.0 # scales other values
        class noise_scales:
            dof_pos = 0.01 # [rad]
            dof_vel = 0.2 # [rad/s]
            lin_vel = 0.1
            ang_vel = 0.2
            gravity = 0.05
            height_measurements = 0.1

    class viewer:
        ref_env = 0
        pos = [10, 0, 6]  # [m]
        lookat = [11., 5, 3.]  # [m]

    class sim:
        dt =  0.002
        substeps = 1
        gravity = [0., 0. ,-9.81]  # [m/s^2]
        up_axis = 1  # 0 is y, 1 is z

        class physx:
            num_threads = 10
            solver_type = 1  # 0: pgs, 1: tgs
            num_position_iterations = 4
            num_velocity_iterations = 0
            contact_offset = 0.01  # [m]
            rest_offset = 0.0   # [m]
            bounce_threshold_velocity = 0.5 #0.5 [m/s]
            max_depenetration_velocity = 1.0
            max_gpu_contact_pairs = 2**23 #2**24 -> needed for 8000 envs and more
            default_buffer_size_multiplier = 5
            contact_collection = 2 # 0: never, 1: last sub-step, 2: all sub-steps (default=2)

class RBQ10CfgPPO(BaseConfig):
    seed = 1
    runner_class_name = 'OnPolicyRunner'
    
    class policy:
        init_noise_std = 1.0
        actor_hidden_dims = [512, 256, 128]
        critic_hidden_dims = [512, 256, 128]
        activation = 'elu' # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid

    class algorithm:
        # training params
        value_loss_coef = 1.0
        use_clipped_value_loss = True
        clip_param = 0.2
        entropy_coef = 0.01
        num_learning_epochs = 5
        num_mini_batches = 4 # mini batch size = num_envs*nsteps / nminibatches
        learning_rate = 1.e-3 #5.e-4
        schedule = 'adaptive' # could be adaptive, fixed
        gamma = 0.99
        lam = 0.95
        desired_kl = 0.01
        max_grad_norm = 1.

    class runner:
        policy_class_name = 'ActorCritic'
        algorithm_class_name = 'PPO'
        num_steps_per_env = 60 
        max_iterations = 50000 

        # logging
        save_interval = 50 
        run_name = 'rbq10'
        experiment_name = run_name
        # load and resume
        resume = False
        load_run = -1 
        checkpoint = -1 
        resume_path = None