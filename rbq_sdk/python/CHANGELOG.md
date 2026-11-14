# Changelog

All notable changes to `rbq_sdk_py` are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Version numbers track the framework release, not an independent SemVer sequence
for the SDK.

The SDK is published to PyPI as part of the framework's public release, at the
same version as the framework. PyPI is immutable: every release needs a new version.

## [Unreleased]

### Added

- `SportClient.LastMoveCode()` — the robot's answer to the latest `Move()` sent within the last second
  (`3205` control mode off, `3204` bad value, `0` if none yet). `Move()` itself is unchanged: it does not wait
  and returns `0`, so a refusal was not visible before.
- Message types for the robot's DDS topics, under `rbq_sdk_py.idl.<package>.msg.dds_` and generated from
  the robot's IDL: `rbq_msgs` (`RobotStatus_`, `BatteryState_`, `FootStates_`, `LegJointInfo_`,
  `LedBottomState_`, `LedBottom_`, `HighLevelCommand_`, `TargetGo_`, `JointOwnershipCmd_`, `MotionRef_`),
  `sensor_msgs` (`Imu_`, `JointState_`, `Joy_`, `CameraInfo_`), `std_msgs` (`Bool_`, `Int8_`,
  `Int8MultiArray_`), `geometry_msgs.TransformStamped_`, `tf2_msgs.TFMessage_`,
  `diagnostic_msgs.DiagnosticArray_` and `nmea_msgs.Sentence_`. Reading `rt/rbq/robot_status`, `battery`,
  `joint_states`, `foot_states`, `tf` or the heartbeats — and sending `rt/rbq/cmd/switch_gait`, `dock` or
  `switch_power` — used to need hand-written types.
- `JointRef_.vel` — a joint velocity reference [rad/s] in `MotionRef_`
  (`example/idl/_rbq_idl.py`, after `pos`). CAN FD robots only: a classic CAN
  robot keeps it at 0 and logs once. The type changed, so an SDK from before
  this does not match the new robot software on `rt/rbq/ref/motion/_*`.
  `LegJointInfo_.joint[].ref_vel` (`rt/rbq/leg_joint`) now reads the velocity
  reference back (it was always `0`).
- `example/sport/rbq_wheel.py` — drive the wheels of an RBQ wheel robot (RBQ-W)
  from app slot 21, wheel joints only.
- `rbq_sdk_py.idl.sensor_msgs.msg.dds_.CompressedImage_` — the robot's camera
  frames (`rt/rbq/vision/sensor_<N>/<stream>/compressed`).
- The robot now answers `SportClient.GetState`, `Euler`, `BodyHeight`,
  `BodyPosition` (`x`, `y` must be `0`) and `SpeedLevel` (with a new robot
  software release — `Network`).

### Deprecated

- `SportClient.BipedStand`, `FrontFlip`, `BackFlip`, `SetAutoRecovery`, `VideoClient` and `RobotStateClient`
  warn (`FutureWarning`) and are removed in the next release. The robot has no such motion or service:
  the calls answer `3203` (not implemented) or time out, as before. They were first removed outright;
  they are back so that code calling them keeps running for one release. Use the camera topic
  (`rt/rbq/vision/sensor_4/rgb/compressed`) instead of `VideoClient` and `rt/rbq/robot_status`
  (`RobotStatus_`) or `GetState()` instead of `RobotStateClient`.

### Fixed

- `unitree_go.msg.dds_.LidarState_` has `dirty_percentage` (`uint8`, after
  `error_state`) like the robot's IDL. Without it the type did not match.
- `example/idl/_rbq_idl.py`: `JointRef_`, `JointInfo_`, `MotionRef_`,
  `JointOwnershipCmd_` and `LegJointInfo_` are now `final` like the C++ IDL, and
  `JointInfo_` carries all of its fields. Before, DDS type matching failed and
  `rbq_low_level.py` sent and received nothing.
- `SportClient` calls get the robot's answer. The response's `binary` field was
  `uint8` while the robot sends `int8`, so DDS dropped every response and each
  call ended in timeout `3104` even though the robot ran the command. The
  robot also lost its first answer after each start (fixed in `Network`).
- `ChannelFactoryInitialize` no longer fails with `channel factory init error`
  on a PC that also runs the robot stack with vision. The DDS config now allows
  60 participants per host, like the C++ SDK (Cyclone's default is 9).
- The SDK no longer writes a CycloneDDS trace to `/tmp/cdds.LOG`.
- Examples: the network interface is optional and defaults to `lo` (all but
  `rbq_led_bottom.py` and `rbq_wheel.py`). `rbq_sport_client.py` reports any
  non-zero code as an error, and `list` or an unknown command sends nothing
  instead of repeating the last command. `rbq_target_go.py` usage names the
  heading `theta` (radians), not `theta_deg`.

### Changed

- Installs on Python 3.10 - 3.13 (Ubuntu 22.04 and 24.04): `cyclonedds` is now
  `11.0.1` (was `0.10.2`, whose wheels stop at 3.10 — `pip install rbq_sdk`
  failed on 24.04's Python 3.12). Upgrade with `pip install -U rbq_sdk`; the
  robot needs no update. Values and the API are unchanged.
  Two things made 11 and the robot's `0.10.2` miss each other, both fixed in the
  SDK: on `lo` 11 assumed multicast and found no peer (the SDK's DDS config now
  turns multicast off on `lo`), and the robot could not read 11's type codes for
  `int8`/`uint8` members, so every RPC timed out with `3102` and topics with
  such members stayed silent (`rbq_sdk_py/core/dds_compat.py` describes them as
  `char`/`byte`, like 0.10). Linux arm64 still has no `cyclonedds` wheel.
- `example/front_camera/` subscribes to the camera topic instead of calling
  `VideoClient`.
- `SwitchGait(0)` and `SwitchGait(1)` select the walk gait `49`, `SwitchGait(2)`
  the vision walk `48` (were the deprecated `30`, `45`, `47`). Leaving the
  left-side gait or the handstand returns to `49`.
- PyPI distribution renamed `rbq_sdk_py` → `rbq_sdk` (`pip install rbq_sdk`).
  The import name is unchanged (`import rbq_sdk_py`).
- PyPI publish moved from a standalone GitHub Actions job (OIDC Trusted
  Publishing) into `rbq_sdk/python/scripts/release.bash`, invoked by
  `scripts/release.bash` Step 9 alongside the C++ PPA publish; uploads now use
  a stored PyPI API token (`PYPI_TOKEN`).

## [1.0.1] - 2026-06-26

### Added

- PEP 517/621 packaging with the `hatchling` backend.
- Version single-sourced from `rbq_sdk_py-v*` git tags via `hatch-vcs`.
- Automated release to PyPI through GitHub Actions + Trusted Publishing (OIDC).
