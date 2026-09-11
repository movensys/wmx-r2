# Robot Option Reference

`wmx_robot_option_node` (`wmx_r2_package/src/wmx_robot_option_node.cpp`) is a single
rclcpp lifecycle node that exposes the **WMX3 Robot Option**: the licensed
kinematics and robot-motion layer (`KinematicsApi.h`, `RobotMotionApi.h`,
`CoordinateApi.h`) that sits above CoreMotion. It attaches to the WMX3 device
itself and drives `wmx3Api::RobotMotion`, so the general nodes own the engine,
not the motion this node commands.

check `doc/launch_robot_option.md`

```
 set_robot_param ───────────▶┌──────────────────────────────┐
  (XML / URDF)               │ wmx_robot_option_node            │──▶ WMX3 Kinematics
                             │  (lifecycle)                 │    SetRobotParam
 start_ptp ─────────────────▶│                              │──▶ StartPTPPos
  (joint / cartesian)        │  one robotId, tool index 0   │
 start_motion ──────────────▶│                              │──▶ SetMotion + StartMotion
  (line / arc / spline)      │  RobotMotionProfile from     │
                             │  the robot parameter file    │
 stop / pause / resume ─────▶│                              │──▶ StopMotion, PauseMotion, ...
 e_stop / release_e_stop     │                              │
                             │                              │──▶ /wmx/robot/status
 calc_*_kinematics ─────────▶│  UpdateRobotStatus @ rate    │    (RobotStatus)
                             └──────────────────────────────┘
```

Where `wmx_core_motion_node` commands axes one by one, this node commands the
robot as a whole: the kinematics engine turns a tool pose into coordinated axis
motion. The two talk to the same axes, so only one of them should be driving at
a time.

---

## Launch arguments

`wmx_r2_robot_option.launch.py` starts the general nodes
(`wmx_r2_general_nodes.launch.py`) plus `wmx_robot_option_node`.

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use simulation clock |
| `config_file` | `config/wmx_r2_robot_option_config.yaml` | YAML with the general node and `wmx_robot_option_node` parameters. Unlike the other robot launches this one defaults to a shipped file, because the node needs `wmx_robot_option_node` listed in `managed_nodes` to be brought up at all |
| `wmx_param_file` | `""` | WMX3 axis parameter XML imported at engine start. Empty imports nothing |
| `robot_param_file` | `""` | Robot parameter XML or URDF registered at configure. Overrides `robot_param_file` from `config_file`; empty registers nothing and leaves the node waiting for `wmx/robot/set_robot_param` |

`wmx_param_file` and `robot_param_file` are two different files. The first
describes the **axes** (gear ratio, units, polarity) and is imported by
`wmx_engine_node`. The second describes the **robot** (model, DH parameters,
link and joint limits, motion profile) and is imported here.

---

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `robot_param_file` | `""` | Absolute path to the robot parameter file, read once at `on_configure`. A `.urdf` suffix (case-insensitive) selects `ImportParamURDF`, anything else `ImportParamXML`. Empty logs a warning and configures anyway |

An XML takes its robot id from its own `<Robot ID="...">`. A URDF has no id to
take, so the one registered at configure is hardcoded to `0`; the `robot_id`
field on `wmx/robot/set_robot_param` is the only way to register a URDF under a
different id at runtime, and XML imports ignore it.
| `robot_status_rate` | `10` | `wmx/robot/status` rate in Hz. Values `<= 0` fall back to 10 |

Parameters are read once in the constructor, so set them in the config YAML, not
with `ros2 param set`.

---

## Services and topics

All services answer `success` plus a `message` carrying the WMX3 call name, the
error number and `ErrorToString` text. Every one takes `robot_id`, and every one
except `set_robot_param` rejects an id that is not the registered one. Servo
state is not among them: it belongs to `wmx_core_motion_node`.

### Power

This node has no servo service. `wmx_core_motion_node` owns servo state, and
`wmx/axes/set_servo_on` stays reachable while the robot option holds the axes:

```bash
wros ros2 service call /wmx/axes/set_servo_on wmx_r2_message/srv/SetAxes \
  "{axis: [0,1,2,3,4,5], data: [1,1,1,1,1,1]}"
```

The axis list is the `<Axis>` values of the robot parameter file, which for
`example/cr3a_robot_option_parameters.xml` are `0` to `5`.

### Registration

| Name | Type | WMX3 call |
|---|---|---|
| `wmx/robot/set_robot_param` | `RobotSetRobotParam` | `ImportParamXML` or `ImportParamURDF`, then `SetRobotParam` |
| `wmx/robot/release_robot` | `RobotId` | `ReleaseRobot` |

```bash
wros ros2 service call /wmx/robot/set_robot_param wmx_r2_message/srv/RobotSetRobotParam \
  "{param_file: '/opt/wmx3/robot_sample/common/robotParamXML_MZ07L.xml', robot_id: 0}"
wros ros2 service call /wmx/robot/release_robot wmx_r2_message/srv/RobotId "{robot_id: 0}"
```

`set_robot_param` replaces whatever was registered before, so it re-registers a
robot after `release_robot` and re-reads a file edited at runtime. The response
returns the `robot_id` and `num_joints` the engine accepted; both are what every
later call must match. `release_robot` invalidates the id, and the status topic
goes quiet until a robot is registered again.

### Motion

| Name | Type | WMX3 call |
|---|---|---|
| `wmx/robot/start_ptp` | `RobotStartPtp` | `StartPTPPos` with `PtpPosParam` / `PtpMovParam` / `PtpJogParam` |
| `wmx/robot/start_motion` | `RobotStartMotion` | `SetMotion` then `StartMotion` |
| `wmx/robot/stop_motion` | `RobotId` | `StopMotion` |
| `wmx/robot/pause_motion` | `RobotId` | `PauseMotion` |
| `wmx/robot/resume_motion` | `RobotId` | `ResumeMotion` |
| `wmx/robot/clear_motion_error` | `RobotId` | `ClearMotionError` |
| `wmx/robot/e_stop` | `RobotId` | `EStop` |
| `wmx/robot/release_e_stop` | `RobotId` | `ReleaseEStop` |
| `wmx/robot/override_velocity` | `RobotOverrideVelocity` | `OverrideVelocityByRatio` |

**`start_ptp`** moves in joint space. `mode` picks the PTP mode, `use_cartesian`
picks what drives it.

| `mode` | Meaning | `target_joint` | `target_pose` |
|---|---|---|---|
| `0` | pos, absolute | absolute joint values | absolute tool pose |
| `1` | mov, relative | per-joint distance | pose displacement |
| `2` | vel, jog | direction per joint, `-1`, `0` or `1` | not used |

```bash
# absolute joint target
wros ros2 service call /wmx/robot/start_ptp wmx_r2_message/srv/RobotStartPtp \
  "{robot_id: 0, mode: 0, use_cartesian: false, target_joint: [0, 0, 0, 0, -90, 0]}"

# relative cartesian displacement
wros ros2 service call /wmx/robot/start_ptp wmx_r2_message/srv/RobotStartPtp \
  "{robot_id: 0, mode: 1, use_cartesian: true, target_pose: {x: 50.0, y: 20.0, z: 10.0}}"

# jog joints 0 and 1 until stop_motion
wros ros2 service call /wmx/robot/start_ptp wmx_r2_message/srv/RobotStartPtp \
  "{robot_id: 0, mode: 2, target_joint: [1, -1, 0, 0, 0, 0]}"
wros ros2 service call /wmx/robot/stop_motion wmx_r2_message/srv/RobotId "{robot_id: 0}"
```

`s`, `e` and `r` are the inverse-kinematics configuration flags, used only when
`use_cartesian` is true. They map to `RobotState::serShapeFlag`: `s` shoulder
(`1` left, `-1` right), `e` elbow (`1` above, `-1` below), `r` wrist (`1` flip,
`-1` no flip), `0` auto on all three.

**`start_motion`** runs a continuous path. It sets the trajectory and starts it in
one call, and clears the trajectory again if `StartMotion` fails, so a rejected
start does not leave a stale path armed.

| `trajectory_type` | Path | `through_pose` | `target_pose` | `arc_angle` |
|---|---|---|---|---|
| `0` | line | unused | end pose | unused |
| `1` | arc | exactly one via point | end pose | `0` runs the three point arc, non-zero runs the path arc over that many degrees |
| `2` | c-spline | the whole path, two points or more | unused | unused |
| `3` | b-spline | the whole path, two points or more | unused | unused |

`is_tool_coordinate` applies to the line only: `true` reads `target_pose` as a
displacement in the tool frame, `false` as a pose in the work frame.

```bash
# line in the work frame
wros ros2 service call /wmx/robot/start_motion wmx_r2_message/srv/RobotStartMotion \
  "{robot_id: 0, trajectory_type: 0, is_tool_coordinate: false,
    target_pose: {x: 400.0, y: 0.0, z: 300.0, u: 0.0, v: 0.0, w: 0.0}}"

# 200 mm retract along the tool z axis
wros ros2 service call /wmx/robot/start_motion wmx_r2_message/srv/RobotStartMotion \
  "{robot_id: 0, trajectory_type: 0, is_tool_coordinate: true,
    target_pose: {x: 0.0, y: 0.0, z: -200.0}}"

# three point arc
wros ros2 service call /wmx/robot/start_motion wmx_r2_message/srv/RobotStartMotion \
  "{robot_id: 0, trajectory_type: 1, arc_angle: 0.0,
    target_pose: {x: 500.0, y: 0.0, z: 300.0},
    through_pose: [{x: 450.0, y: 50.0, z: 300.0}]}"
```

**Stopping.** `stop_motion` decelerates the current motion, `pause_motion` holds
it on the path and `resume_motion` continues from there. `e_stop` is a level 1
emergency stop: motion ends immediately, the robot lands in `EStopActive` with
motion error `UserEStop`, and only `release_e_stop` brings it back to idle.
`clear_motion_error` clears the other motion errors and does **not** recover a
user EStop.

**`override_velocity`** scales the **current** motion's velocity, acceleration and
deceleration. `1.0` is 100%, and values above `1.0` push past the profile in the
robot parameter file.

```bash
wros ros2 service call /wmx/robot/override_velocity wmx_r2_message/srv/RobotOverrideVelocity \
  "{robot_id: 0, vel_ratio: 0.5, acc_ratio: 1.0, dec_ratio: 1.0}"
```

### Coordinates and kinematics

| Name | Type | WMX3 call |
|---|---|---|
| `wmx/robot/set_tool_coordinate` | `RobotSetCoordinate` | `SetToolCoordinate` |
| `wmx/robot/get_tool_coordinate` | `RobotGetCoordinate` | `GetToolCoordinate` |
| `wmx/robot/set_work_coordinate` | `RobotSetCoordinate` | `SetWorkCoordinate` |
| `wmx/robot/get_work_coordinate` | `RobotGetCoordinate` | `GetWorkCoordinate` |
| `wmx/robot/calc_forward_kinematics` | `RobotCalcForwardKinematics` | `CalcForwardKinematics` |
| `wmx/robot/calc_inverse_kinematics` | `RobotCalcInverseKinematics` | `CalcInverseKinematics` |

```bash
# 100 mm tool offset along flange z
wros ros2 service call /wmx/robot/set_tool_coordinate wmx_r2_message/srv/RobotSetCoordinate \
  "{robot_id: 0, pose: {x: 0.0, y: 0.0, z: 100.0}}"
wros ros2 service call /wmx/robot/get_tool_coordinate wmx_r2_message/srv/RobotGetCoordinate "{robot_id: 0}"

wros ros2 service call /wmx/robot/calc_forward_kinematics \
  wmx_r2_message/srv/RobotCalcForwardKinematics \
  "{robot_id: 0, joint_position: [0, 0, 0, 0, -90, 0]}"
wros ros2 service call /wmx/robot/calc_inverse_kinematics \
  wmx_r2_message/srv/RobotCalcInverseKinematics \
  "{robot_id: 0, tool_pose: {x: 400.0, y: 0.0, z: 300.0}, s: 0, e: 0, r: 0}"
```

Both kinematics services are pure computation: they move nothing, and they answer
from the registered parameters plus the current tool and work coordinates. A pose
the arm cannot reach comes back `success: false` with the engine's
`InverseKinematicsError`, which makes `calc_inverse_kinematics` a cheap
reachability check before a `start_ptp`.

### Status

| Name | Dir | Type | QoS | Rate |
|---|---|---|---|---|
| `wmx/robot/status` | pub | `wmx_r2_message/RobotStatus` | default, depth 1 | `robot_status_rate` |

```bash
wros ros2 topic echo /wmx/robot/status
```

One `UpdateRobotStatus` per tick fills the message: `robot_id`, `motion_state`,
`motion_error_code`, `motion_error_joint`, `tool_pose_cmd`, `tool_pose_fb`, and
`joint_pos_cmd`, `joint_pos_fb`, `joint_velocity_cmd`, `joint_velocity_fb`,
`joint_torque_fb` sized to `num_joints`. Nothing is published while no robot is
registered.

`motion_state` (`kinematics::constants::MotionState`):

| | | | |
|---|---|---|---|
| `0` idle | `5` setup | `10` error | `15` in resume |
| `1` no new command | `6` in CP motion | `11` pausing CP | `16` direct teaching |
| `2` stopping CP | `7` in PTP motion | `12` pausing PTP | `17` EStop stopping |
| `3` stopping PTP | `8` error stopping CP | `13` paused CP | `18` EStop active |
| `4` EStop (deprecated) | `9` error stopping PTP | `14` paused PTP | |

`motion_error_code` (`kinematics::constants::MotionErrorCode`):

| | | |
|---|---|---|
| `0` none | `6` axis vel limit | `12` axis following error |
| `1` system EStop | `7` axis acc limit | `13` axis offline |
| `2` trajectory calc error | `8` axis amp error | `14` axis interrupt mismatch |
| `3` inverse kinematics error | `9` tool range error | `15` collision EStop |
| `4` inverse dynamics error | `10` servo off | `16` axis command mode mismatch |
| `5` axis pos limit | `11` user EStop | |

---

### Interface types

Every type this node uses lives in `wmx_r2_message`. Two messages and nine
services, all added for the robot option:

```bash
wros ros2 interface list | grep wmx_r2_message

wros ros2 interface show wmx_r2_message/msg/RobotCartesianPose
wros ros2 interface show wmx_r2_message/msg/RobotStatus

wros ros2 interface show wmx_r2_message/srv/RobotSetRobotParam
wros ros2 interface show wmx_r2_message/srv/RobotId
wros ros2 interface show wmx_r2_message/srv/RobotStartPtp
wros ros2 interface show wmx_r2_message/srv/RobotStartMotion
wros ros2 interface show wmx_r2_message/srv/RobotOverrideVelocity
wros ros2 interface show wmx_r2_message/srv/RobotSetCoordinate
wros ros2 interface show wmx_r2_message/srv/RobotGetCoordinate
wros ros2 interface show wmx_r2_message/srv/RobotCalcForwardKinematics
wros ros2 interface show wmx_r2_message/srv/RobotCalcInverseKinematics
```

`RobotId` is reused by six services, `RobotSetCoordinate` and
`RobotGetCoordinate` by two each, which is why nine types cover seventeen
services.

The live graph, with the node active:

```bash
wros ros2 service list -t | grep /wmx/robot/
wros ros2 topic list -t | grep /wmx/robot/
wros ros2 param list /wmx_robot_option_node
wros ros2 lifecycle get /wmx_robot_option_node
```

---

## Units and conventions

- **Poses are mm and degrees.** `RobotCartesianPose` maps one to one onto
  `coordinate::CartesianPose`: `x`, `y`, `z` are `point` in millimetres, and `u`,
  `v`, `w` are `rotation`, Z-Y-X Euler angles (roll, pitch, yaw) in degrees. This
  is the WMX3 convention, not the ROS one, so nothing here is in metres or
  radians and nothing is a `geometry_msgs/Pose`.
- **Joint values are WMX3 user units**, degrees for revolute joints and
  millimetres for prismatic ones. The axis parameter XML owns that scaling.
- **Joint arrays must be exactly `num_joints` long.** A shorter or longer
  `target_joint` or `joint_position` is rejected before the SDK is called, with
  the expected and received lengths in the message. `calc_inverse_kinematics`
  returns exactly `num_joints` values.
- **There is no per-call velocity.** Unlike `wmx/axes/start_pos`, the profile
  comes from `RobotMotionParam.profile` in the robot parameter file: profile
  type, tool linear velocity and acceleration, tool rotation ratio, per-joint
  velocity and acceleration, the override gains and the PTP blending type.
  Speed is changed by editing that file and re-registering, or at runtime with
  `override_velocity`.
- **Motion services do not block.** They return as soon as the engine accepts the
  command; the SDK's `Wait` is never called from a callback. Completion is read
  from `motion_state` on the status topic, which is why the default 10 Hz is a
  floor rather than a maximum for a client that polls it.
- **Poses are in the work frame** as set by `set_work_coordinate`, with the tool
  offset from `set_tool_coordinate` applied. `tool_pose_cmd` and `tool_pose_fb`
  on the status topic follow the same frames.

---

## Robot parameter file

`robot_param_file` is **not** `wmx_parameters.xml`. The two files describe
different things and are read by different components:

| | `wmx_param_file` | `robot_param_file` |
|---|---|---|
| Describes | axes: gear ratio, units, polarity, encoder, limits | robot: model, joint geometry, limits, motion profile |
| Read by | `wmx_engine_node` at engine start | `wmx_robot_option_node` at configure |
| SDK call | `CoreMotion::config->ImportAndSetAll` | `RobotConfig::ImportParamXML` + `SetRobotParam` |
| Shipped | `config/wmx_parameters.xml`, `example/cr3a_wmx_parameters.xml` | `example/cr3a_robot_option_parameters.xml` |
| SDK samples | `/opt/wmx3/robot_sample/common/wmx_parameter_*.xml` | `/opt/wmx3/robot_sample/common/robotParamXML_*.xml` |

### Structure

```xml
<Robot ID="0">
  <RobotName>CR3A</RobotName>
  <RobotType>0</RobotType>
  <Joint ID="0">
    <JointType>RotateZAxis</JointType>
    <Axis>0</Axis>
    <JointOrigin x="0.0" y="0.0" z="128.3" u="0.0" v="0.0" w="0.0"/>
    <MaxAngle>3.0</MaxAngle>
    <MinAngle>-3.0</MinAngle>
    <VelocityLimit>3.0</VelocityLimit>
    <AccelerationLimit>10.0</AccelerationLimit>
  </Joint>
  ...
  <EndEffector x="0.0" y="0.0" z="0.0" u="0.0" v="0.0" w="0.0"/>
  <WorkCoordinate x="0.0" y="0.0" z="0.0" u="0.0" v="0.0" w="0.0"/>
  <ToolVelocity>100.0</ToolVelocity>
  <ToolAcceleration>10.0</ToolAcceleration>
  <ToolVelRotRatio>0.017453292519943</ToolVelRotRatio>
  <AxisVelocity0>0.2617993877991494</AxisVelocity0>
  <AxisAcceleration0>0.2617993877991494</AxisAcceleration0>
  ...
  <VelOverride>0.005</VelOverride>
  <AccOverride>0.005</AccOverride>
  <ProfileType>Trapezoida</ProfileType>
</Robot>
```

| Tag | Lands in | Note |
|---|---|---|
| `Robot ID` | `robotParam.robotId` | the id every service must pass |
| `RobotName` | `robotParam.robotName` | |
| `RobotType` | `robotParam.robotType` | `RobotModel` enum, see below |
| `Joint/JointType` | `jointParams[i].jointType` | `RotateXAxis`, `RotateYAxis`, `RotateZAxis` and the linear variants |
| `Joint/Axis` | `jointParams[i].axis` | **WMX3 axis index**, and the `axis` list to pass to `wmx/axes/set_servo_on` |
| `Joint/JointOrigin` | `jointParams[i].jointOrigin` | offset from the previous joint, mm and radians |
| `Joint/Max,MinAngle` | `jointParams[i].max,minAngle` | radians in the file, degrees in the API |
| `Joint/Velocity,AccelerationLimit` | `jointParams[i]` limits | radians, hard limits |
| `EndEffector` | `robotParam.toolCoordinate[0]` | the `set_tool_coordinate` default |
| `WorkCoordinate` | `robotParam.workCoordinate` | the `set_work_coordinate` default |
| `Tool*`, `Axis*`, `*Override`, `ProfileType` | `RobotMotionParam.profile` | what every motion command uses, since no service carries a velocity |

**Angles are radians in the file and degrees in the API.** `MaxAngle`
`2.9670597` reads back as `170.0`, and `AxisVelocity0` `0.2617993` as `15.0`
deg/s. The joint values on `start_ptp` and the status topic are the degrees, not
the radians.

`RobotType` values (`kinematics::constants::RobotModel`):

| | | |
|---|---|---|
| `0` articulated 6 axis | `3` delta 4 axis | `6` cobot 6 axis, DH |
| `1` SCARA 4 axis | `4` H-bot 4 axis | `7`, `8` dual-tool SCARA 5 / 6 axis |
| `2` cartesian 4 axis | `5` classic 6 axis, DH | `9` user defined |

Types `5` and `6` are DH-parameter based and take `<DHParam d>` / `<DHParam a>`
per joint instead of the joint geometry above. Type `0` is the joint-origin form
shown here.

### The old `robot.xml` schema does not load

The `wmx-server-development` server used the previous SDK (`KinematicsRobot.h`,
`Robot::Load`), whose `model/robot.xml` looks similar but is **not** the same
schema. Importing it with `ImportParamXML` **returns 0 and silently produces an
unusable robot**: every joint comes back `axis = -1` and `jointType` invalid, all
joint origins zero, and the angle and velocity values left unconverted in
radians.

| Old tag | New tag | |
|---|---|---|
| `Comment` | `RobotName` | |
| `LinkType` | `JointType` | same value names |
| `MotorID` | `Axis` | **ignored if left as `MotorID`** |
| `VecFrom` | `JointOrigin` | **ignored if left as `VecFrom`** |
| `MaxVelocity`, `MinVelocity` | `VelocityLimit` | one limit, take the positive one |
| `MaxAcceleration`, `MinAcceleration` | `AccelerationLimit` | one limit |
| `LinkDirection` | none | polarity lives in the axis parameter XML |
| `InitialAngle`, `CoeffToMakeRadian`, `GearRatio*`, `EncoderPulse`, `DistPerEncRevolve`, `HomeOffset` | none | the axis parameter XML owns all of this |
| `Compensation1..5`, `DecOverride` | none | dropped |
| `RobotType` | `RobotType` | **different enum**, must be remapped |

When migrating a file, `RobotType` is the trap: the old enum is gone, and old
`8` means `DualToolScara6Axis` today. A file carrying joint-origin geometry
rather than DH parameters belongs on type `0`.

`example/cr3a_robot_option_parameters.xml` is not that migration. Every value in
it is sourced:

| Field | Source |
|---|---|
| `JointOrigin`, `MaxAngle`, `MinAngle` | the CR3A description, `movensys_manipulator_description/urdf/dobot_cr3a/dobot_cr3a.xacro`, read into the WMX joint-origin form with the SDK's own `ImportParamURDF` and written back as type `0` |
| `Axis` | the `joint_axes` of `example/cr3a_manipulator_config.yaml`, so `0` to `5` |
| `VelocityLimit` | Dobot CR A Series User Guide V1.8, Appendix A Table 1: J1 and J2 180 deg/s, J3 to J6 223 deg/s |
| `ToolVelocity` | half the guide's 2 m/s rated linear speed, so 1000 mm/s |
| `AccelerationLimit`, `ToolAcceleration` | **derived, not published.** Dobot gives no acceleration figure anywhere in the guide. Taken as rated speed reached from rest in 0.25 s |
| `AxisVelocity*`, `AxisAcceleration*` | half the limits above, so 90 deg/s on J1 and J2 and 111.5 deg/s on J3 to J6 |
| `VelOverride`, `AccOverride` | `0.1`, a 10% derate on top of that for bring-up. Raise it with `override_velocity` |

Two things still need checking against the arm. The 0.25 s acceleration figure
is an assumption, not a specification. And the joint limits are the
description's, which are tighter than the guide's mechanical range (J3 is
-146 to 11.5 deg here against a rated +-155 deg), so the engine will refuse
poses the arm could physically reach. Tighter is the safe direction, but confirm
it suits the cell.

```bash
wros ros2 service call /wmx/robot/set_robot_param wmx_r2_message/srv/RobotSetRobotParam \
  "{param_file: '/opt/wmx3/robot_sample/common/robotParamXML_MZ07L.xml', robot_id: 0}"
```

---

## Lifecycle and runtime behaviour

The node starts `unconfigured` and does nothing until
`wmx_lifecycle_manager_node` drives it, automatically once `wmx_engine_node`
reports `Communicating`, or on demand:

```bash
wros ros2 lifecycle set /wmx_robot_option_node configure
wros ros2 lifecycle set /wmx_robot_option_node activate
```

**`on_configure`** attaches to the WMX3 device with
`CreateDevice(WMX3_SDK_PATH, DeviceTypeNormal, 10 s)` and names it
`wmx_robot_option_node`; a lock-busy failure reports that the engine may not be
communicating. Then, if `robot_param_file` is set, it imports the file and calls
`SetRobotParam`. A failure there closes the device again and fails the
transition, so a bad path is caught at bring-up rather than at the first motion
call. An empty `robot_param_file` only warns.

**`on_activate`** advertises the seventeen services, creates the publisher and
starts the status timer.

**`on_deactivate`** stops the timer, drops the publisher and the services, and
keeps the device attached and the robot registered.

**`on_cleanup`** closes the device and marks the robot unregistered.
`on_shutdown` deactivates first if it was active, then cleans up.

**Arbitration.** `wmx_robot_option_node` is listed in the `motion_controllers` of
`wmx_core_motion_node`, so it owns the axes the same way a manipulator
controller does. While it is `active`, `wmx/axes/start_pos`, `start_mov`,
`start_vel`, `start_jog` and `start_home` answer `success: false` and the axes
can only be driven through `wmx/robot/*`. Deactivating it
(`ros2 lifecycle set /wmx_robot_option_node deactivate`) hands the axes back.

`wmx/axes/stop` is never blocked, and neither are the servo and configuration
services, so `set_servo_on`, `clear_amp_alarm` and `stop` stay reachable while
the robot option holds the axes. That is deliberate: servo state has one owner,
and it is not this node.

The guard needs no code in this node. `wmx_core_motion_node` watches each listed
name's `/<name>/transition_event` and `/<name>/get_state`, which every lifecycle
node publishes, and it assumes a listed node is **active until proven
otherwise**: a name that never appears on the graph blocks axis motion forever.
Keep the list and `managed_nodes` in step.

The guard is one-way. Nothing stops `wmx/robot/start_ptp` while a manipulator
controller is active, so a config that runs both still needs one of them driving
at a time.

**Thread safety.** One mutex guards the registered robot parameter and every
robot SDK call, so a service call and the status timer never enter the SDK
together. `CreateDevice` and `CloseDevice` sit outside it, in the lifecycle
transitions. The services share the node's default callback group and therefore
run one at a time anyway.

**Status read errors.** `UpdateRobotStatus` returns a bitmask, and the motion
error bit is state rather than a read failure, so it is masked off and reported
in the message. Only a genuine read failure skips the tick, throttled to one
warning per second.

---

## Known limitations

- **One robot per node.** The node holds a single `RobotMotionParam`, so a second
  `robot_id` is rejected even though the engine handles up to
  `MAX_NUMBER_OF_ROBOT` (5). Run one node per robot, each with its own device
  name, or extend the node to a map of parameters.
- **Tool index 0 only.** The `Ex` overloads (`SetMotionEx`, `StartPTPPosEx`,
  `SetToolCoordinateEx`, `CalcForwardKinematicsEx`) are not exposed, so the
  dual-tool models (`DualToolScara5Axis`, `DualToolScara6Axis`) and
  `JointCoupledCPMotionParam` are out of reach.
- **Up to six joints.** `MAX_NUMBER_OF_JOINT` is 6 in this SDK, and the joint
  arrays follow it.
- **No `Wait` service.** A client that needs blocking completion polls
  `motion_state` on the status topic. Raise `robot_status_rate` if the default
  10 Hz is too coarse for the poll.
- **The licence is not surfaced.** `GetLicenseInfo` is not exposed, so a missing
  or exhausted Robot Option licence shows up as a `SetRobotParam` failure at
  configure rather than as a dedicated check.
- **Not exposed at all:** input shaping (`IstParam`), gravity compensation and
  direct teaching, collision detection, fitting data, the default motion profile
  and blending type setters, and the data buffer channel for streamed pose
  arrays. All are available on `kinematics::Kinematics` if needed.
- **No MoveIt2 contract.** This node speaks WMX3 poses over services, not
  `FollowJointTrajectory` and `/joint_states`. For the MoveIt2 path see
  [reference_manipulator.md](reference_manipulator.md).

---

## Configuration files

A deployment is one YAML plus the two parameter files:

1. **ROS parameter YAML**, `config/wmx_r2_robot_option_config.yaml`. Carries the
   `wmx_robot_option_node` block plus `wmx_engine_node`,
   `wmx_lifecycle_manager_node` and `wmx_core_motion_node`.
   `wmx_robot_option_node` must appear twice: in `managed_nodes` after the
   device-level nodes so it is brought up, and in the `motion_controllers` of
   `wmx_core_motion_node` so it owns the axes while active. None of the
   manipulator controllers are listed, since none run here. An entry of `""` is
   skipped, but a bare `[]` is rejected by rclcpp as an untyped empty list.
2. **WMX3 axis parameter XML**, passed as `wmx_param_file` and imported by
   `wmx_engine_node`. The SDK ships examples under
   `/opt/wmx3/robot_sample/common/`, for instance `wmx_parameter_MZ07L.xml`.
3. **Robot parameter XML or URDF**, passed as `robot_param_file` and imported
   here. The package ships `example/cr3a_robot_option_parameters.xml`; the SDK ships
   `robotParamXML_MZ07L.xml`, `robotParamXML_MTR.xml` and
   `robotParamURDF_CR12A.urdf` in the same folder as its axis samples. A URDF
   import always lands as `UserDefinedModel` and takes its id from the
   `robot_id` field, so an XML file is the better choice when the model is one
   the SDK knows. See [Robot parameter file](#robot-parameter-file) for the
   schema.

Paths are handed to the SDK unchanged, so use absolute paths.

---

## Build

`wmx_robot_option_node` compiles against the WMX3 SDK at the CMake cache path
`WMX3_SDK_PATH` (default `/opt/wmx3`), and links `robotmotionapi`,
`kinematicsapi`, `coordinateapi`, `coremotionapi`, `wmx3api` and `imdll`. The
first three are what separate it from the other nodes in the package.

At **runtime** the dynamic linker must find the SDK's shared libraries
(`libimdll.so` and friends): either an `ld.so.conf.d` entry for `/opt/wmx3/lib`
or `LD_LIBRARY_PATH=/opt/wmx3/lib`.

The Robot Option is a **licensed** SDK option. Without a valid licence
`SetRobotParam` fails and the node will not configure with a `robot_param_file`
set.

The launch needs **root** for real-time scheduling. `wros`
(`docker/wros.bash`) already execs into the container as root, which is how
[launch_robot_option.md](launch_robot_option.md) starts it. The general node
startup sequence this node depends on is in
[reference_general_nodes.md](reference_general_nodes.md).
