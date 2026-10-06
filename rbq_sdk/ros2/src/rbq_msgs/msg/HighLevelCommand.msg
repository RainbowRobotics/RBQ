# gait_state constants
int8 STATE_FALL                 = -2    # Fall Mode - Triggered upon unexpected loss of balance
int8 STATE_OFF                  = -1    # Control Off - All actuators disabled
int8 STATE_SIT                  = 0     # Sitting - Low posture, resting on the ground
int8 STATE_STAND                = 1     # Standing - Neutral posture, ready to walk
int8 STATE_AIM                  = 2     # Aim Mode - Aiming posture for targeting
int8 STATE_WALK                 = 3     # Walk Mode - walking Trot gait
int8 STATE_STAIRS               = 4     # Stairs Mode - Stair-adaptive gait using camera sensor
int8 STATE_WAVE                 = 5     # Wave Mode - walking Walk gait
int8 STATE_RUN                  = 6     # Run Mode - High-speed gait (if supported)
int8 STATE_RL_TROT              = 30    # RL Trot - Reinforcement Learning Trot gait
int8 STATE_RL_FRONT_WALK        = 31    # RL Front Walk - Reinforcement Learning Front Walk gait
int8 STATE_RL_LEFT_WALK         = 33    # RL Left Walk - Reinforcement Learning Left Walk gait
int8 STATE_RL_RIGHT_WALK        = 34    # RL Right Walk - Reinforcement Learning Right Walk gait
int8 STATE_RL_BOUND             = 35    # RL Bound - Reinforcement Learning Bound gait
int8 STATE_RL_PACE              = 36    # RL Pace - Reinforcement Learning Pace gait
int8 STATE_RL_PRONK             = 37    # RL Pronk - Reinforcement Learning Pronk gait
int8 STATE_RL_3LEG_HR           = 38    # RL 3Leg HR - Reinforcement Learning 3-Leg gait (Hind Right)
int8 STATE_RL_3LEG_HL           = 39    # RL 3Leg HL - Reinforcement Learning 3-Leg gait (Hind Left)
int8 STATE_RL_3LEG_FR           = 40    # RL 3Leg FR - Reinforcement Learning 3-Leg gait (Front Right)
int8 STATE_RL_3LEG_FL           = 41    # RL 3Leg FL - Reinforcement Learning 3-Leg gait (Front Left)
int8 STATE_RL_TROT_VISION       = 42    # RL Trot Vision - Reinforcement Learning Trot with Vision
int8 STATE_RL_TROT_RUN          = 45    # RL Trot Run - Reinforcement Learning Trot Run gait
int8 STATE_RL_SILENT            = 46    # RL Silent - Reinforcement Learning Silent gait
int8 STATE_RL_STAIRS            = 47    # RL Stairs - Reinforcement Learning stair gait

# Message fields
std_msgs/Header header          # ROS message header with timestamp and frame_id
string identifier               # Command identifier for tracking/logging purposes
float32 roll                    # Body roll angle in degrees (ZYX Euler angles)
float32 pitch                   # Body pitch angle in degrees (ZYX Euler angles)
float32 yaw                     # Body yaw angle in degrees (ZYX Euler angles)
float32 vel_x                   # Linear velocity in X direction (m/s)
float32 vel_y                   # Linear velocity in Y direction (m/s)
float32 omega_z                 # Angular velocity around Z axis (deg/s)
float32 delta_body_h            # Body height adjustment from default (m)
float32 delta_foot_h            # Foot height adjustment from default (m)
int8	gait_state              # Target gait state (use constants defined above)
bool	gait_transition         # Enable(true)/disable(false) gait transition during command execution
