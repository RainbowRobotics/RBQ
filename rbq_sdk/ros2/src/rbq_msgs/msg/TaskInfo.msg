# Cartesian task-space state. Mirrors idl/rbq/TaskInfo_.idl (FINAL extensibility).
# Field order and types MUST stay in sync with the IDL for DDS interop.
float32[3] pos        # position, m
float32[3] vel        # velocity, m/s
float32[3] acc        # acceleration, m/s^2
float32[3] force      # force, N
float32[4] quat       # orientation quaternion, w/x/y/z
float32[9] rot        # rotation matrix, row-major 3x3
float32[3] rpy        # euler roll/pitch/yaw, rad
float32[3] omega      # angular velocity, rad/s
float32[9] jacobian   # jacobian, row-major 3x3
float32[3] kp         # task-space stiffness
float32[3] kd         # task-space damping
