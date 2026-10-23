# Per-joint reference. Mirrors idl/rbq/JointRef_.idl (FINAL extensibility).
# Field order and types MUST stay in sync with the IDL for DDS interop.
float32 pos      # rad
float32 torque   # N*m
float32 kp       # Nm/rad
float32 kd       # Nm*s/rad
