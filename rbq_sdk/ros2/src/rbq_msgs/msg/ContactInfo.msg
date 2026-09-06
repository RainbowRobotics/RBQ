# Foot contact state. Mirrors idl/rbq/ContactInfo_.idl (FINAL extensibility).
# Field order and types MUST stay in sync with the IDL for DDS interop.
float32[3] force    # contact force, N
float32[3] torque   # contact torque, N*m
bool active         # true if contact is detected
