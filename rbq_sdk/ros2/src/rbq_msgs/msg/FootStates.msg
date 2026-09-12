std_msgs/Header header

# Contact constants
uint8 CONTACT_LOST = 0
uint8 CONTACT_MADE = 1

# Arrays for 4 legs (FL, FR, RL, RR)
geometry_msgs/Point[4] foot_position_rt_body    # Foot position relative to body frame (4 elements)
geometry_msgs/Point[4] foot_velocity_rt_body    # Foot velocity relative to body frame (4 elements)
uint8[4] foot_contact_est                       # Contact state for each foot (4 elements)
float32[4] foot_force_est                       # Foot force (4 elements)
