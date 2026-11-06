# Generic user command. Mirrors idl/rbq/UserCommand_.idl (FINAL extensibility).
# Field order and types MUST stay in sync with the IDL for DDS interop.
# Field names are lowercase (ROS 2 requires it); the IDL was renamed to match.
int32 command_target
int32 user_command
int8[] user_para_char
int32[] user_para_int
float32[] user_para_float
float64[] user_para_double
