# RBA-1 arm end-effector velocity command (/rbq/cmd/arm/high_level) — the arm counterpart of HighLevelCommand.
# Read only while task-space control (ControlStart) and the arm's external control mode are on.
float32[3] lin_vel   # m/s   — x forward, y left, z up (the arm controller's foot-centre frame, like the app joystick)
float32[3] ang_vel   # rad/s — roll, pitch, yaw about the end effector
int8 gripper         # 1 open, -1 close, 0 stop
