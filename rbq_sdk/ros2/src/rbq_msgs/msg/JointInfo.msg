# Per-joint state. Mirrors idl/rbq/JointInfo_.idl (FINAL extensibility).
# Field order and types MUST stay in sync with the IDL for DDS interop.
float32 pos              # rad
float32 vel              # rad/s
float32 acc              # rad/s^2
float32 torque           # N*m
float32 kp               # Nm/rad
float32 kd               # Nm*s/rad
int8 owner               # 0~40
float32 ref_ff_torque    # N*m
float32 ref_position     # rad
float32 ref_vel          # rad/s
float32 current          # Amperes
int8 temperature_board   # Celsius
int8 temperature_coil    # Celsius
string<=3 name           # 3-letter joint name e.g. "HRR"
string<=3 status         # 3-letter lifecycle: NON/CON/HOM/RUN
string<=3 error_msg      # 3-letter error code e.g. "JAM", "OK"
