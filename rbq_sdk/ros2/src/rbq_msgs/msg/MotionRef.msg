# Whole-body joint reference. Mirrors idl/rbq/MotionRef_.idl (FINAL extensibility).
# Field order and types MUST stay in sync with the IDL for DDS interop.
# Array sizes mirror RbqConstants_.idl: leg=12, arm=8, whl=4.
float64 time             # seconds
JointRef[12] leg_joint
JointRef[8] arm_joint
JointRef[4] whl_joint
