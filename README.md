# WMX R2: ROS2 robotics on industrial-grade motion

[![CI](https://github.com/movensys/wmx-r2/actions/workflows/ci.yml/badge.svg)](https://github.com/movensys/wmx-r2/actions/workflows/ci.yml)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE.txt)
[![Docs](https://img.shields.io/badge/docs-wmx--r2-brightgreen)](https://movensys.github.io/wmx-r2-doc/)

WMX R2 is a ROS 2 interface to [WMX3](https://www.movensys.com/en/products/software_motion_control/wmx_en),
Movensys' software real-time EtherCAT motion engine. It turns planner output,
such as MoveIt2 and Nav2 trajectories, into precisely timed servo commands on a
fixed cycle over EtherCAT. Perception, planning and deterministic motion all run
on **one edge device**: no external motion controller, and no TCP/IP hop between
the planner and the drives. In the See-Think-Act loop of Physical AI, this is the
*Act* layer.

**WMX R2 drives any EtherCAT machine, regardless of the brand.** It commands the
servo drives directly with CoE (CANopen over EtherCAT) and controls **axes**, not
a robot model, so a six-axis arm on an assembly line, a mobile base moving
material through a plant and a custom multi-axis machine are all driven the same
way. Moving between them changes three files: the axis parameter XML, the URDF
and the planning config.

The host is an ordinary PC: **x86-64 or arm64**, from an industrial PC to an
NVIDIA Jetson Thor, running Ubuntu with a **PREEMPT_RT** kernel. Motion is
software, so the EtherCAT master runs on a standard network port and no
motion-control card is needed.

## Why WMX R2

| | |
|---|---|
| **ROS 2 and AI on the industrial machine** | MoveIt2, MoveIt Servo and Nav2 command EtherCAT servos through standard ROS 2 actions and topics, with no external motion controller |
| **Production-grade, not bench-grade** | Deterministic cycle timing, servo-level error handling and direct drive access, for machines that must hold path accuracy and run for long periods without drift |
| **One IPC, 85% less tracking error** | Removing the external controller's TCP/IP hop and its redundant control stage cut mean absolute tracking error by 85% versus a conventional setup ([benchmark](https://movensys.github.io/wmx-r2-doc/)) |
| **Any EtherCAT machine** | Nothing is baked into the launch files. A manipulator, a mobile base or a 50-axis machine differ only by the config, URDF and WMX parameter XML you pass in |
| **Down to the register** | Axis, IO, EtherCAT master and engine control are all exposed as services, so industrial commissioning and diagnostics need no extra tooling |
| **Proven engine** | WMX3: 25+ years of development, 40+ patents, 40,000+ licences, 500+ customers in semiconductor and industrial automation |
| **Two distros, CI-tested** | ROS 2 Humble and Jazzy, amd64 and arm64 |
| **Free to develop with** | Nothing to buy to start. The motion engine runs free in **6-hour sessions**, renewed by restarting it; a commercial licence lifts the session limit for production |

Full explanation, application examples and integration scenarios:
**[movensys.github.io/wmx-r2-doc](https://movensys.github.io/wmx-r2-doc/)**

## Requirements

> This package drives real motion hardware in real time. It is beyond a simulator.

- **WMX Linux** (real-time patched) with the WMX3 SDK installed. See [WMX installation](https://movensys.github.io/wmx-r2-doc/getting_started/index.html).
- EtherCAT servo drives / IO reachable from the WMX3 master.
- ROS2 **Humble** or **Jazzy**, with `rmw_cyclonedds` as the RMW.
- Root for real-time scheduling: `sudo --preserve-env` on the host, or `wros` in the container.

## Quickstart

Set up `~/.bashrc`, clone and start the container first: see
[doc/first_setup.md](doc/first_setup.md). `wros` runs a command inside the
container as root with ROS and the workspace sourced.

```bash
# 1. Build (messages first, then the rest)
wros colcon build --packages-select wmx_r2_message
wros colcon build

# 2. Launch the low-level nodes (engine, core motion, IO, EtherCAT)
wros ros2 launch wmx_r2_package wmx_r2_general_nodes.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/config/wmx_r2_general_nodes_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/config/wmx_parameters.xml'

# 3. Bring axes online and command a move
wros ros2 service call /wmx/axes/set_servo_on wmx_r2_message/srv/SetAxes "{axis: [0,1], data: [1,1]}"
wros ros2 service call /wmx/axes/start_pos wmx_r2_message/srv/StartAxesPose \
    "{axis: [0,1], target: [8388608, 10000], velocity: [1000000, 5000], acc: [100000, 1000], dec: [100000, 1000]}"
```

Full startup sequence and the complete service/topic catalog:
[doc/reference_general_nodes.md](doc/reference_general_nodes.md).

## Architecture

### Low-level control ([wmx_r2_general_nodes.launch.py](wmx_r2_package/launch/wmx_r2_general_nodes.launch.py))

```mermaid
---
title: Low-level Control
---
flowchart LR;
    A[ROS2 Services/Topics] --> B[wmx_engine_node];
    A --> M[wmx_lifecycle_manager_node];
    A --> C["wmx_core_motion_node (lifecycle)"];
    A --> D["wmx_io_node (lifecycle)"];
    A --> E["wmx_ethercat_node (lifecycle)"];
    B -->|engine status| M;
    M -->|configure / activate| C;
    M -->|configure / activate| D;
    M -->|configure / activate| E;
    B --> F[WMX3 API];
    C --> F;
    D --> F;
    E --> F;
    F --> G[WMX Engine];
```

`wmx_engine_node` owns the WMX3 engine and nothing else.
`wmx_lifecycle_manager_node` watches it and drives every other WMX node, each a
[managed (lifecycle) node](https://design.ros2.org/articles/node_lifecycle.html)
that starts `unconfigured` and only attaches to the device at `configure`.

**The managed nodes follow the engine.** While the engine is communicating, every
node found on the graph is brought up to `active`; a node that joins late or
respawns is picked up on a later sweep. When the engine stops or its device is
closed, they are all deactivated and cleaned back to `unconfigured` (their device
handles are dead), and brought up again when the engine returns. This is not
limited to WMX nodes: any managed node in the same namespace (a lifecycle
`joint_state_publisher`, a nav2 node, your own) is driven the same way. Order it
with the manager's `managed_nodes` parameter, or drive nodes by hand:

```bash
# Lifecycle nodes the manager can see, and their states
ros2 service call /wmx/lifecycle/get_node_states wmx_r2_message/srv/GetNodeStates "{}"

# Drive one node by name
ros2 service call /wmx/lifecycle/set_node_state wmx_r2_message/srv/SetNodeState \
  "{node_name: 'wmx_io_node', transition: 'deactivate'}"

# Drive every node at once: leave node_name empty
ros2 service call /wmx/lifecycle/set_node_state wmx_r2_message/srv/SetNodeState \
  "{node_name: '', transition: 'bringdown'}"
```

`transition` is one of `configure`, `activate`, `deactivate`, `cleanup`,
`shutdown`, `bringup` (configure + activate) or `bringdown` (deactivate).
Transitions that take nodes down are applied in reverse bring-up order. The
standard `ros2 lifecycle` CLI works on the nodes directly as well.

### Manipulator control ([wmx_r2_manipulator.launch.py](wmx_r2_package/launch/wmx_r2_manipulator.launch.py))

```mermaid
---
title: Manipulator Control
---
flowchart LR;
    A["MoveIt2 move_group"] -->|FollowJointTrajectory| B[joint_trajectory_controller];
    S["MoveIt Servo"] -->|JointTrajectory| P[joint_position_controller];
    K["/wmx/set_gripper"] -->|SetBool| R[gripper_controller];
    B -->|execution_active| P;
    B --> C[WMX3 API];
    P --> C;
    R --> C;
    C --> D[WMX Engine];
    D --> E[Robot];
    E --> D;
    D --> C;
    C --> F[joint_state_broadcaster];
    F --> G["/joint_states"];
```

Four nodes, each attaching to the WMX3 device itself:

- `joint_trajectory_controller` takes planned goals from MoveIt2 as a
  `FollowJointTrajectory` action and runs them as a WMX3 time-based C-spline.
- `joint_position_controller` takes MoveIt Servo's streamed `JointTrajectory` and
  runs it as WMX3 linear interpolation, so every axis arrives at the same instant.
- `gripper_controller` drives one WMX IO output bit from a `std_srvs/SetBool`
  service. Started only with `use_gripper:=true`.
- `joint_state_broadcaster` publishes encoder feedback to `/joint_states`. It also
  clears amp alarms and switches the servos on when it activates.

The two motion controllers interlock over
`/moveit2_trajectory/execution_active`. While a planned goal runs,
`joint_position_controller` drops every streamed trajectory, so a servo command
cannot fight a running trajectory. See
[doc/reference_manipulator.md](doc/reference_manipulator.md).

### Differential-drive control ([wmx_r2_differential.launch.py](wmx_r2_package/launch/wmx_r2_differential.launch.py))

```mermaid
---
title: Differential-drive Control
---
flowchart LR;
    A["Nav2 / teleop"] -->|"/cmd_vel_safe"| B[differential_drive_controller];
    B --> C[WMX3 API];
    C --> D[WMX Engine];
    D --> E[Wheels];
    E --> D[WMX Engine];
    D --> C[WMX3 API];
    C --> B;
    B --> F["/odom_enc, /omega_enc"];
```

- `TwistStamped` on `/cmd_vel_safe` -> `differential_drive_controller` -> WMX3 API -> WMX Engine -> wheels
- Wheels -> WMX Engine -> WMX3 API -> `differential_drive_controller` -> `/odom_enc` (`nav_msgs/Odometry`), `/omega_enc`

The controller holds the kinematics and the dead-reckoning. It converts body
velocity to wheel velocity, drives both wheels with `StartVel`, and integrates
the encoder velocities back into an odometry pose. A stale command stops the
wheels within `cmd_vel_timeout`. See
[doc/reference_differential.md](doc/reference_differential.md).

## Packages

| Package | Description |
|---------|-------------|
| [wmx_r2_message](wmx_r2_message/) | Custom messages and services for axis, IO, EtherCAT, and engine control |
| [wmx_r2_package](wmx_r2_package/) | Main nodes, launch files, and robot configurations |
| [wmx_r2_control](wmx_r2_control/) | `ros2_control` hardware interface, URDF xacros, and controller configs |

This repository is the **execution layer**. It implements the interfaces the
planners drive, but ships no planner configuration of its own. The planning and
perception stacks live in companion repositories:

| Repository | Provides |
|------------|----------|
| [movensys-manipulator](https://github.com/movensys/movensys-manipulator) | MoveIt2 and Isaac cuMotion planning, Nvblox / YOLO / AprilTag perception |
| [movensys-navigation](https://github.com/movensys/movensys-navigation) | Nav2 configuration, EKF odometry and SLAM for a differential-drive base |
| [movensys-intelligence](https://github.com/movensys/movensys-intelligence) | VLM / LLM voice layer built on the manipulator stack |
| [movensys-simulation](https://github.com/movensys/movensys-simulation) | Isaac Sim scenes for the manipulator and navigation scenarios |

## Nodes

| Node | Role |
|------|------|
| `wmx_engine_node` | Owns the WMX3 engine: device creation, EtherCAT communication, parameter import, engine status |
| `wmx_lifecycle_manager_node` | Drives every lifecycle node below, following the engine's status |
| `wmx_core_motion_node` | Per-axis servo, alarm, gear-ratio, homing, point-to-point, velocity and jog services plus `wmx/axes/status` (lifecycle) |
| `wmx_io_node` | IO control for input/output bits and bytes (lifecycle) |
| `wmx_ethercat_node` | EtherCAT master operations, network scan and slave management (lifecycle) |
| `joint_trajectory_controller` | Receives `FollowJointTrajectory` goals and executes them via WMX3 C-spline (lifecycle) |
| `joint_position_controller` | Follows MoveIt Servo's streamed `JointTrajectory` via WMX3 linear interpolation, so every axis arrives at the same instant (lifecycle) |
| `differential_drive_controller` | Differential-drive command and dead-reckoned odometry loop (lifecycle) |
| `joint_state_broadcaster` | Publishes encoder feedback to `/joint_states`; clears alarms and switches the servos on when activated (lifecycle) |
| `gripper_controller` | Gripper open/close over a WMX IO output bit (lifecycle) |

## Launch files

| Launch file | Purpose | Nodes started |
|-------------|---------|---------------|
| [wmx_r2_general_nodes.launch.py](wmx_r2_package/launch/wmx_r2_general_nodes.launch.py) | Low-level axis / IO / EtherCAT control | `wmx_engine_node`, `wmx_lifecycle_manager_node`, `wmx_core_motion_node`, `wmx_io_node`, `wmx_ethercat_node` |
| [wmx_r2_manipulator.launch.py](wmx_r2_package/launch/wmx_r2_manipulator.launch.py) | Manipulator trajectory control | general nodes + `joint_state_broadcaster`, `joint_trajectory_controller`, `joint_position_controller`, and `gripper_controller` when `use_gripper:=true` |
| [wmx_r2_differential.launch.py](wmx_r2_package/launch/wmx_r2_differential.launch.py) | Differential-drive base control | general nodes + `joint_state_broadcaster`, `differential_drive_controller` |
| [wmx_r2_control_manipulator.launch.py](wmx_r2_control/launch/wmx_r2_control_manipulator.launch.py) | Manipulator through `ros2_control` | general nodes + `joint_trajectory_controller`, `joint_position_controller`, `robot_state_publisher`, `ros2_control_node` (`WmxSystemHardware`), a `joint_state_broadcaster` spawner and an Isaac Sim relay |
| [wmx_r2_control_differential.launch.py](wmx_r2_control/launch/wmx_r2_control_differential.launch.py) | Differential base through `ros2_control` | general nodes + `robot_state_publisher`, `ros2_control_node` (`WmxSystemHardware`), `joint_state_broadcaster` and `diff_drive_controller` spawners, and an Isaac Sim relay |

No robot is baked into any launch file. Each takes its paths as launch
arguments (`config_file` and `wmx_param_file`, plus `urdf_file` and
`controllers_file` on the `ros2_control` ones), so one launch file serves every
robot of that kind. The per-robot examples live in
[wmx_r2_package/example/](wmx_r2_package/example/).

## MoveIt2 integration

`joint_trajectory_controller` serves a `FollowJointTrajectory` action, and
`joint_position_controller` subscribes to the `JointTrajectory` stream MoveIt
Servo publishes. To connect with `movensys-manipulator`, set the names in the
manipulator config (example: `example/cr3a_manipulator_config.yaml`) to what
MoveIt2 is configured to call:

```yaml
joint_trajectory_action: /movensys_manipulator_arm_controller/follow_joint_trajectory
joint_trajectory_topic: /movensys_manipulator_arm_controller/joint_trajectory
```

The action name must match the controller name in the MoveIt2 controllers YAML
on the planning side. See
[doc/reference_manipulator.md](doc/reference_manipulator.md).

## Nav2 integration

`differential_drive_controller` implements the interface Nav2 drives a base
through. It takes `geometry_msgs/TwistStamped` velocity commands and publishes
`nav_msgs/Odometry`, so the Nav2 controller server and the `robot_localization`
EKF both connect without a bridge. To connect with `movensys-navigation`, set the
topic names in the differential config (example:
`example/diffbot_differential_config.yaml`):

```yaml
cmd_vel_topic: /cmd_vel_safe          # TwistStamped in, from the Nav2 controller
encoder_odometry_topic: /odom_enc     # nav_msgs/Odometry out, the EKF odom0 input
encoder_omega_topic: /omega_enc       # per-wheel velocity, JointState
```

Set `publish_tf: true` only when no EKF is running, otherwise two publishers
fight over `odom -> base_link`. The `ros2_control` path is an alternative:
`wmx_r2_control_differential.launch.py` spawns
`diff_drive_controller/DiffDriveController` on `WmxSystemHardware` instead of
this node. See [doc/reference_differential.md](doc/reference_differential.md).

Both integrations are plain parameters, so no topic remapping is needed on either
side.

## Documentation

| Doc | Description |
|-----|-------------|
| [doc/first_setup.md](doc/first_setup.md) | Environment setup, Docker setup, dependencies, build |
| [doc/launch_general_nodes.md](doc/launch_general_nodes.md) | Launch the WMX general nodes |
| [doc/launch_manipulator.md](doc/launch_manipulator.md) | Launch a manipulator |
| [doc/launch_differential.md](doc/launch_differential.md) | Launch the differential-drive base |
| [doc/reference_general_nodes.md](doc/reference_general_nodes.md) | Every service and topic of the general nodes, with the startup sequence |
| [doc/reference_manipulator.md](doc/reference_manipulator.md) | Manipulator node reference: parameters, arbitration, lifecycle |
| [doc/reference_differential.md](doc/reference_differential.md) | Differential node reference: parameters, kinematics, odometry |

Full documentation, application examples and integration scenarios:
**[movensys.github.io/wmx-r2-doc](https://movensys.github.io/wmx-r2-doc/)**

## What's included

- [x] Engine, lifecycle, axis, IO, and EtherCAT master control nodes
- [x] Lifecycle manager that follows the engine state and drives every controller
- [x] Manipulator stack: joint state feedback, trajectory action, streamed position, gripper
- [x] Differential-drive stack: velocity command, wheel feedback, dead-reckoned odometry
- [x] MoveIt2 integration: `FollowJointTrajectory` action plus a MoveIt Servo streaming path
- [x] `ros2_control` hardware interface (`wmx_system_hardware`) for both stacks
- [x] Hardware-agnostic launch files: config, WMX parameters, URDF and controllers are all arguments
- [x] Digital-twin mirror topics for Isaac Sim and Gazebo
- [x] Containerised setup for amd64 and arm64
- [x] CI on ROS2 Humble and Jazzy: lint, message build/test, launch-description checks

## Demo

### Physical AI powered by WMX ROS2 on NVIDIA Jetson Thor

[!["WMX Next" with NVIDIA Isaac](images/wmx_gtc_presentation.png)](https://www.youtube.com/watch?v=h-G9vtAGAIU)

## License

The ROS2 interface in this repository is [MIT](LICENSE.txt). It builds against
and runs on the proprietary WMX3 motion engine, which is free to develop with in
6-hour sessions renewed by restarting the engine; production use needs a
commercial licence. **"WMX R2" as a whole is therefore not MIT.** See the
[licensing boundary](https://movensys.github.io/wmx-r2-doc/licensing.html).
