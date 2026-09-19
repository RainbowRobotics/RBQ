# Per-joint ownership request. Mirrors idl/rbq/JointOwnershipCmd_.idl (FINAL).
# Field order and types MUST stay in sync with the IDL for DDS interop.
# Array sizes mirror RbqConstants_.idl: leg=12, whl=4, arm=8.
bool[12] leg_joint
bool[4] whl_joint
bool[8] arm_joint
