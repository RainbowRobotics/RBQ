# Simulator sensor injection. Mirrors idl/rbq/SimInfo_.idl (FINAL extensibility).
# Field order and types MUST stay in sync with the IDL for DDS interop.
# Array sizes mirror RbqConstants_.idl: leg=12, arm=8, whl=4, leg_contact=4.
sensor_msgs/Imu imu
JointInfo[12] leg_joint
JointInfo[8] arm_joint
JointInfo[4] whl_joint
TaskInfo body_task
ContactInfo[4] leg_contact
