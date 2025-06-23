# GaitState
int8 STATE_FALL = -2
int8 STATE_OFF = -1
int8 STATE_SIT = 0
int8 STATE_STAND = 1
int8 STATE_WALK = 2
int8 STATE_AIM = 3
int8 STATE_STAIRS = 4
int8 STATE_WAVE = 5
int8 STATE_RUN = 6

std_msgs/Header header
int8 gait_state
bool joints_comm_checked
bool joints_calibrated
bool joints_control_started
bool comm_connected
sensor_msgs/Imu imu_state
sensor_msgs/JointState joint_states
rbq_msgs/BatteryState battery_state
rbq_msgs/FootState[] foot_states
geometry_msgs/VelocityStamped body_velocity_rt_world
geometry_msgs/PoseStamped body_pose_rt_world
