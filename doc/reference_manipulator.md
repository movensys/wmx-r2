# Manipulator Reference

The manipulator stack is four standalone rclcpp lifecycle nodes
(`wmx_r2_package/src/joint_state_broadcaster.cpp`,
`joint_trajectory_controller.cpp`, `joint_position_controller.cpp`,
`gripper_controller.cpp`) that sit on top of the general WMX nodes and expose the
MoveIt2 contract: a `FollowJointTrajectory` action in, `/joint_states` out, plus a
streaming (servo) path and a gripper service. Each attaches to the WMX3 device
itself and talks to CoreMotion / AdvancedMotion / IO directly — the general nodes
own the engine, not the motion these controllers command.

```
                             ┌───────────────────────────────┐
 MoveIt2 move_group ────────▶│ joint_trajectory_controller   │──▶ WMX3 AdvancedMotion
  (FollowJointTrajectory)    │  (lifecycle) C-spline, planned│    StartCSplinePos
                             └───────────────────────────────┘
                                  │ /moveit2_trajectory/execution_active (Bool, latched)
                                  ▼
 MoveIt Servo ──────────────▶┌───────────────────────────────┐
  (JointTrajectory stream)   │ joint_position_controller     │──▶ WMX3 CoreMotion
                             │  (lifecycle) linear interp    │    StartLinearIntplPos
                             └───────────────────────────────┘
                             ┌───────────────────────────────┐
 configure / activate ──────▶│ joint_state_broadcaster       │──▶ /joint_states  (JointState)
  from                       │  (lifecycle) @ rate (100 Hz)  │──▶ isaacsim topic (JointState)
  wmx_lifecycle_manager_node │  clears alarms + servo on     │──▶ gazebo topic   (Float64MultiArray)
                             └───────────────────────────────┘
                             ┌───────────────────────────────┐
 /wmx/set_gripper ──────────▶│ gripper_controller (lifecycle)│──▶ WMX3 IO SetOutBit
  (std_srvs/SetBool)         └───────────────────────────────┘
```

| Node | Role | Launched by |
|---|---|---|
| `joint_state_broadcaster` | Encoder feedback → `/joint_states`; clears amp alarms and switches servos on at activate | CR3A, CR5A |
| `joint_trajectory_controller` | `FollowJointTrajectory` action → WMX3 time-based C-spline | CR3A, CR5A |
| `joint_position_controller` | MoveIt Servo's streamed `JointTrajectory` → WMX3 linear interpolation | CR3A, CR5A |
| `gripper_controller` | Gripper open/close over a WMX IO output bit | CR3A only |

---

## Parameters

All parameters are read **once at node construction** (`setRosParameter()`); there
is no parameter-set callback. A runtime `ros2 param set` is accepted by rclcpp but
has no effect on behaviour — restart the node to apply new values.

Unlike the differential-drive controller, the defaults here are **deliberate
non-values**: topic/action names default to `/<name>/no_param` and the axis lists
default to empty, so an unconfigured node is loud and inert rather than silently
wrong. Every deployment supplies a YAML.

### A. `joint_state_broadcaster`

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `joint_axes` | int[] | `[]` | – | WMX3 axis index per joint, in the same order as `joint_name`. An index outside `[0, maxAxes)` fails the whole status read with `ArgumentOutOfRange` and the cycle publishes nothing (throttled warn). |
| `joint_name` | string[] | `[j1..j6]` | – | Joint names published in `JointState.name`. A list shorter than `joint_axes` is not an error: the extra axes are dropped at configure with a warning, so the two always line up in the publish loop. |
| `joint_feedback_rate` | int | `0` | Hz | Feedback publish rate. The timer period is `1000 / joint_feedback_rate` truncated to whole milliseconds, so prefer rates that divide 1000 (100, 125, 200, 250, 500). A value of `0` or less falls back to 100 Hz with a warning, but set it explicitly (both shipped configs use `100`). |
| `encoder_joint_topic` | string | `/encoder_joint_topic/no_param` | – | Real-robot feedback topic; the deployment sets it to `/joint_states` (MoveIt / robot_state_publisher input). |
| `isaacsim_joint_topic` | string | `/isaacsim_joint_topic/no_param` | – | Mirror of the same message for Isaac Sim (`/isaacsim/joint_command`). Published **before** the header stamp is filled in, i.e. with a zero stamp — Isaac consumes positions by name, not by time. |
| `gazebo_joint_topic` | string | `/gazebo_joint_topic/no_param` | – | Positions only, as `Float64MultiArray`, for a Gazebo position controller. |
| `gripper_joint_name` | string[] | `[]` | – | Extra joint names appended to the feedback message so the gripper shows up in RViz/MoveIt. Empty = no gripper (CR5A). |
| `gripper_address` | int[2] | `[0, 0]` | – | `[byte, bit]` of the WMX **output** bit read back for gripper state. |
| `gripper_open_value` | double | `0.0` | m or rad | Joint value reported for every `gripper_joint_name` while the output bit is 0. |
| `gripper_close_value` | double | `0.0` | m or rad | Joint value reported while the bit is 1. CR3A uses `0.045`. |

### B. `joint_trajectory_controller`

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `joint_axes` | int[] | `[]` | – | WMX3 axis index per trajectory joint. |
| `joint_name` | string[] | `[]` | – | Joint name per entry of `joint_axes`, same order. When the goal carries `joint_names`, each goal column is matched to its axis by name and a goal that names an unknown joint, repeats one, or leaves one out is rejected. Leave empty only to keep the old positional mapping, which the node warns about at startup. A length that does not match `joint_axes` is logged as an error and the list is dropped. |
| `joint_trajectory_action` | string | `/joint_trajectory_action/no_param` | – | Name of the `FollowJointTrajectory` action server. Must equal the controller name MoveIt2 is configured to call (e.g. `/movensys_manipulator_arm_controller/follow_joint_trajectory`). |

`MAX_TRAJ_POINTS` (1000) is a compile-time constant, not a parameter: it sizes the
WMX spline buffer allocated at `configure` and caps the accepted goal length.

### C. `joint_position_controller`

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `joint_axes` | int[] | `[]` | – | WMX3 axis index per joint. |
| `joint_name` | string[] | `[j1..j6]` | – | Name→axis map used when the incoming `JointTrajectory` carries `joint_names`; pairs positionally with `joint_axes`. An unknown name drops the message (throttled warn). With empty `joint_names` the message is taken positionally. |
| `joint_trajectory_topic` | string | `/joint_trajectory_topic/no_param` | – | Streamed trajectory input from MoveIt Servo (e.g. `/movensys_manipulator_arm_controller/joint_trajectory`). |
| `default_velocity` | double | `0.1` | user-unit/s | Per-axis velocity used when the point's `time_from_start` is 0 (no time base to derive a velocity from). Deployments use `0.5`. |
| `accel_ratio` | double | `0.5` | – | Fraction of the step time spent accelerating: `acc = velocity / (accel_ratio · dt)`. Lower = harder ramp. Deployments use `0.3`. Not guarded: `0` yields a non-finite acceleration. |
| `min_step` | double | `0.1` | user-unit | Deadband. A message whose largest per-axis move from the current *command* position is below this is dropped, so servo jitter does not restart an interpolation every cycle. Deployments use `0.001` — the default is deliberately coarse. |

### D. `gripper_controller`

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `wmx_gripper_topic` | string | `/wmx_gripper_topic/no_param` | – | Name of the `std_srvs/SetBool` **service** (the parameter is named "topic" for historical reasons). CR3A uses `/wmx/set_gripper`. |
| `gripper_address` | int[2] | `[0, 0]` | – | `[byte, bit]` of the WMX output bit driven by the service. A list shorter than 2 falls back to `[0, 0]` with a warning. |

`MANIPULATOR_MODEL` is an **environment variable**, not a parameter: when it equals
`dobot_cr3a` (injected by the CR3A launch file via `additional_env`), `configure`
runs the CR3A gripper power-up sequence (`SetOutByte(28, 113)`, then a readback of
output byte 28 and input bit 0.1). Any other value skips it with an INFO log.

---

## Topics, actions and services

| Name (deployment default) | Dir | Type | QoS | Rate | Notes |
|---|---|---|---|---|---|
| `/movensys_manipulator_arm_controller/follow_joint_trajectory` | action server | `control_msgs/FollowJointTrajectory` | action default | per goal | Name from `joint_trajectory_action`. Goals are **rejected** unless the node is `active`; accepted goals run `ACCEPT_AND_EXECUTE` on a detached thread. |
| `/movensys_manipulator_arm_controller/joint_trajectory` | sub | `trajectory_msgs/JointTrajectory` | default, depth 1 | producer | Name from `joint_trajectory_topic`. Only the **last point** of each message is used — this is a streaming target, not a queued trajectory. |
| `/moveit2_trajectory/execution_active` | pub (JTC) / sub (JPC) | `std_msgs/Bool` | **transient_local**, depth 1 | on change | Arbitration latch: `true` for the duration of a planned goal. Latched so a controller that joins late sees the current state. |
| `/servo_node/delta_joint_cmds` | pub (JTC) | `control_msgs/JointJog` | default, depth 10 | on goal exit | Zero-velocity jog for the goal's joint names, published on *every* exit path (success, abort, cancel) so MoveIt Servo does not resume with a stale delta. Hardcoded, not a parameter. |
| `/joint_states` | pub | `sensor_msgs/JointState` | default, depth 1 | `joint_feedback_rate` | Name from `encoder_joint_topic`. `name` = `joint_name` + `gripper_joint_name`; `position`/`velocity` = `actualPos`/`actualVelocity`; no `effort`. Header stamped from the node clock. |
| `/isaacsim/joint_command` | pub | `sensor_msgs/JointState` | default, depth 1 | `joint_feedback_rate` | Same content, **zero header stamp** (published before stamping). |
| `/gazebo_position_controller/commands` | pub | `std_msgs/Float64MultiArray` | default, depth 1 | `joint_feedback_rate` | `data` = the position vector above (joints then gripper). |
| `/wmx/set_gripper` | service | `std_srvs/SetBool` | services QoS | on call | Name from `wmx_gripper_topic`. `data: true` = close (bit 1), `false` = open (bit 0). Returns `success: false` when the node is not `active`. |
| `wmx/axes/clear_amp_alarm`, `wmx/axes/set_servo_on` | client (broadcaster) | `wmx_r2_message/SetAxes` | services QoS | at activate | Served by `wmx_core_motion_node`. See the lifecycle section. |
| `wmx/engine/get_axis_param` | client (broadcaster) | `wmx_r2_message/GetAxisParam` | services QoS | at configure | Served by `wmx_engine_node`. The dump for `joint_axes` is logged at INFO so the axis setup is captured in the startup log; a missing engine service only warns, configuration still succeeds. |

**Namespaces.** The data-topic defaults resolve to *absolute* names in the shipped
configs, so launching in a ROS namespace does **not** namespace them — override the
topic/action parameters explicitly for multi-robot deployments. The lifecycle
services (`~/change_state`, `~/get_state`) do follow the namespace; the lifecycle
manager discovers each node under its fully-qualified name, and
`wmx/lifecycle/set_node_state` takes that same name.

---

## Units and conventions

- Joint values are passed through **unconverted**: `JointState.position` is the raw
  WMX `actualPos`, and a goal position is written straight into the spline /
  interpolation command. The WMX axis user-unit scaling (encoder counts, gear
  ratio) must be configured WMX-side — via the axis parameter XML — so that one
  axis user unit = 1 rad at the joint, matching the URDF. There is no gear-ratio
  or offset parameter in any of these nodes.
- Velocity feedback is the servo's `actualVelocity` (same user unit per second),
  not a numerical derivative of position.
- `joint_trajectory_controller` is **time-based**: it consumes `positions` and
  `time_from_start` only. `velocities` and `accelerations` in the goal are logged
  and discarded — the WMX C-spline derives the profile from the position/time
  sequence. `time_from_start` is converted to milliseconds; the first point is
  forced to `t = 0`, and a final point less than 1 ms after its predecessor is
  dropped (planners routinely emit such a duplicate endpoint).
- `joint_position_controller` is **step-based**: per axis, `velocity = |target −
  posCmd| / dt` and `acc = dec = velocity / (accel_ratio · dt)`, where `dt` is the
  point's `time_from_start`. Because every axis gets its own velocity for a shared
  `dt`, `StartLinearIntplPos` makes them all arrive at the same instant. The step
  is measured against the **commanded** position (`posCmd`), not the encoder, so
  following error does not feed back into the next command.
- Gripper state is a single IO **bit**, so feedback is two-valued: the broadcaster
  reports `gripper_close_value` or `gripper_open_value`, never anything between.

---

## Lifecycle and runtime behaviour

**Startup.** All four are managed (lifecycle) nodes. They start `unconfigured` and
do nothing until `wmx_lifecycle_manager_node` drives them — automatically once
`wmx_engine_node` reports `Communicating`, or on demand through
`wmx/lifecycle/set_node_state` / `ros2 lifecycle set`. The manipulator configs list
them in `managed_nodes` **after** the device-level nodes, so
`wmx_core_motion_node` is already active and serving `wmx/axes/*` by the time the
broadcaster activates and calls it.

`on_configure` is the same shape everywhere:

1. `CreateDevice(WMX3_SDK_PATH, DeviceTypeNormal, 10 s)` — any error fails the
   transition and leaves the node `unconfigured` (the manager logs it and retries
   on the next discovery sweep).
2. `SetDeviceName("<node name>")`.
3. Build the API wrappers (`CoreMotion`, plus `AdvancedMotion` +
   `CreateSplineBuffer(0, 1000)` for the trajectory controller, `IO` for the
   broadcaster and gripper).

The ROS interfaces are **not** created here. Publishers, subscriptions, services,
action servers and timers are created in `on_activate` and destroyed in
`on_deactivate`, so a configured-but-inactive node advertises nothing.

Per-node activation:

- **`joint_state_broadcaster`** — `on_configure` reads `wmx/engine/get_axis_param`
  for its `joint_axes` and logs the dump (informational: a failure only warns).
  `on_activate` is the only place in the stack that touches servo power. It calls `wmx/axes/clear_amp_alarm` then
  `wmx/axes/set_servo_on` for every `joint_axes` entry, with up to 5 attempts
  (10 s service wait, 15 s per call, 2 s backoff when the server answers "not
  active" / "not initialized"); a hard failure fails the transition and leaves the
  node `inactive`. Only then does the feedback timer start. `on_deactivate` stops
  the timer and switches the servos **off** directly through CoreMotion — so
  deactivating the broadcaster drops the arm's holding torque.
- **`joint_trajectory_controller`** — `on_activate` creates the action server and
  publishes `execution_active: false`. `on_deactivate` stops the axes, republishes
  `false` and destroys the action server, so no new goal can arrive; a goal already
  running on its detached thread sees the deactivation, stops the axes and aborts,
  and `on_deactivate` waits for it (warning if it outlasts the stop timeout).
- **`joint_position_controller`** — `on_activate` just arms the callback.
  `on_deactivate` issues `Stop` + `Wait` on all `joint_axes`, so the streaming path
  never leaves an interpolation running behind it.
- **`gripper_controller`** — `on_activate`/`on_deactivate` only flip the gate that
  the service checks; the output bit is left as-is.

`on_cleanup` drops the interfaces and closes the WMX device in every node;
`on_shutdown` delegates to `on_cleanup`. A failed `configure` is not terminal: the
node stays alive and `unconfigured`, and the transition is retried on the next
sweep. No node exits with an error code on init failure — supervise via logs,
lifecycle state, or topic liveness (`/joint_states` at `joint_feedback_rate`), not
exit codes.

**Manual vs. controller arbitration.** `wmx_core_motion_node` refuses its own motion
services (`start_pos`, `start_mov`, `start_vel`, `start_jog`, `start_home`) while any
node in its `motion_controllers` list is ACTIVE, so a manual jog cannot fight a running
controller. `wmx/axes/stop` is never blocked, and the servo/config services stay open —
the broadcaster needs `set_servo_on` to activate. See `reference_general_nodes.md`.

**Planned vs. streamed arbitration.** Both motion controllers can command the same
axes, so they interlock over `/moveit2_trajectory/execution_active`:

1. A `FollowJointTrajectory` goal publishes `true` on entry.
2. `joint_position_controller` latches that and drops every incoming streamed
   trajectory while it holds.
3. On exit (success, abort, or cancel) the trajectory controller publishes `false`
   and sends a zero `JointJog` to `/servo_node/delta_joint_cmds`.

The latch is one-directional: a servo motion already in flight is not preempted
when a planned goal starts. Keep MoveIt Servo paused (or accept that the first
planned goal wins the race) if both can be commanded at once.

**Goal execution** (`joint_trajectory_controller`, on one execution thread the node
owns and joins):

1. Reject a second goal while one is running, and reject any goal while the node is
   not ACTIVE.
2. Reject if `points.size() > 1000` (abort, no motion).
3. Map each goal column to its axis by `joint_name`, then fill the spline buffer from
   `positions` + `time_from_start`, normalize the first timestamp to 0, drop a
   sub-millisecond final point.
4. `StartCSplinePos(buffer 0, ...)`; a WMX error aborts the goal with
   `result.error_code` set to the raw WMX code.
5. Wait out the planned duration of the trajectory, then poll every 10 ms until
   `motionComplete` and `inPos` are both set on every `joint_axes` entry. Cancel
   issues `Stop` + `Wait` and reports `canceled`. A goal that has not finished 10 s
   past its planned duration is aborted with `GOAL_TOLERANCE_VIOLATED` after a
   `Stop`.

The planned duration comes from the last `time_from_start` in the goal, so a goal
cannot report success before the motion it asked for has had time to run. No
feedback messages are published during execution, and there is no path tolerance or
goal tolerance check beyond that deadline.

**Process model.** Each node ships as its own executable, one process per node.
`joint_state_broadcaster` runs on a `MultiThreadedExecutor` (its activate-time
service calls need it); the other three run on the default single-threaded
executor. The trajectory controller runs its goal on a separate thread that the node
joins on deactivate, cleanup and destruction, so no goal outlives the device handle.
None is built as a composable component.

---

## Known limitations

- **No engine gate in the feedback loop.** `publishJointState()` checks the
  `GetStatus` return and skips the cycle when it fails, but it does not check the
  engine state, so an engine that stops without failing the read leaves the
  broadcaster publishing the last values it saw. Watch the engine through
  `wmx/engine/get_engine_status` (the lifecycle manager already does, and takes
  these nodes down when it stops).
- **`gripper_controller` hardcodes its SDK path.** It calls `CreateDevice("/opt/lmx/", ...)`
  rather than the compiled-in `WMX3_SDK_PATH` that every other node uses; an
  installation elsewhere fails to configure this node only.
- **CR3A gripper setup is model-gated by environment.** Running
  `gripper_controller` outside its launch file (no `MANIPULATOR_MODEL`) skips the
  power-up sequence, and the service then toggles a bit on an unpowered gripper.

---

## Configuration files

A deployment is one YAML plus the launch wiring
(example: `launch/wmx_r2_cr3a_manipulator.launch.py`):

1. **ROS parameter YAML** — `config/cr3a_manipulator_config.yaml` (or
   `cr5a_...`), with one key per node (all tables above) **plus** the
   `wmx_engine_node` and `wmx_lifecycle_manager_node` keys: the manipulator launch
   passes this same file down to the included general-nodes launch as
   `config_file`, so engine core/affinity (`core`, `affinity_mask`), the WMX
   parameter XML path (`wmx_param_file_path`), and the bring-up order all live
   in it.
2. **WMX parameter XML** — `config/cr3a_wmx_parameters.xml`: axis-level
   gear/feedback/limit/`inPos` setup. This is where the "axis user unit = joint
   rad" scaling and the hardware-level motion limits live. Loaded by
   `wmx_engine_node` via `wmx_param_file_path`, or on demand through
   `wmx/engine/import_and_set_all`.
3. **Launch** — includes the general nodes, then starts the manipulator nodes as
   `LifecycleNode`s (unconfigured; the manager drives them) and injects
   `use_sim_time` and, for CR3A, `MANIPULATOR_MODEL=dobot_cr3a`.

```yaml
joint_state_broadcaster:
  ros__parameters:
    joint_feedback_rate: 100          # Hz — must be > 0
    joint_axes: [0, 1, 2, 3, 4, 5]
    joint_name: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    gripper_joint_name: ["picker_1_joint", "picker_2_joint"]   # omit for CR5A
    gripper_address: [0, 0]           # [byte, bit]
    gripper_open_value: 0.00
    gripper_close_value: 0.045
    encoder_joint_topic: /joint_states
    isaacsim_joint_topic: /isaacsim/joint_command
    gazebo_joint_topic: /gazebo_position_controller/commands

joint_trajectory_controller:
  ros__parameters:
    joint_axes: [0, 1, 2, 3, 4, 5]
    joint_name: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    joint_trajectory_action: /movensys_manipulator_arm_controller/follow_joint_trajectory

joint_position_controller:
  ros__parameters:
    joint_axes: [0, 1, 2, 3, 4, 5]
    joint_name: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    joint_trajectory_topic: /movensys_manipulator_arm_controller/joint_trajectory
    accel_ratio: 0.3
    default_velocity: 0.5
    min_step: 0.001

gripper_controller:                   # CR3A only
  ros__parameters:
    wmx_gripper_topic: /wmx/set_gripper
    gripper_address: [0, 0]

wmx_engine_node:
  ros__parameters:
    core: -1                          # RT engine CPU core (-1 = SDK default)
    affinity_mask: 0                  # CPU affinity bitmask (0 = SDK default)
    wmx_param_file_path: ""           # injected by launch

wmx_lifecycle_manager_node:
  ros__parameters:
    managed_nodes:                    # device-level nodes first
      - wmx_core_motion_node
      - wmx_io_node
      - wmx_ethercat_node
      - joint_state_broadcaster
      - joint_trajectory_controller
      - joint_position_controller
      - gripper_controller
```

The action name is also the MoveIt2 integration point: it must match the
controller name in the MoveIt controllers YAML on the planning side.

---

## Toolkit integration checklist

What the Toolkit needs to template per robot / per deployment:

- **Always per robot:** `joint_axes` and `joint_name` (identical lists across the
  three motion nodes), `joint_feedback_rate`, and the WMX parameter XML
  (`wmx_param_file_path`) that defines the joint unit scaling and limits.
- **Per gripper:** `gripper_address`, `gripper_open_value`/`gripper_close_value`,
  `gripper_joint_name`, `wmx_gripper_topic` — and drop `gripper_controller` from
  `managed_nodes` entirely on grippersless arms (CR5A).
- **Per planning stack:** `joint_trajectory_action` and `joint_trajectory_topic`,
  which must match the MoveIt2 controller configuration and the Servo output topic.
- **Usually defaults:** `accel_ratio`, `default_velocity`, `min_step`, and the
  simulator mirror topics (leave them at the shipped values unless Isaac/Gazebo is
  in the loop).
- **Not parameterized (by design):** the lifecycle gate
  (`wmx_lifecycle_manager_node` owns it), the arbitration topic
  `/moveit2_trajectory/execution_active` and servo reset topic
  `/servo_node/delta_joint_cmds`, the 1000-point spline buffer size, the
  clear-alarm/servo-on sequence (fixed to activate), and any gear-ratio scaling
  (WMX XML owns it).

## Build

`wmx_r2_package` compiles against the WMX3 SDK at the CMake cache path
`WMX3_SDK_PATH` (default `/opt/wmx3`); the same path (with a trailing `/` appended
by CMake) is compiled in and passed to `CreateDevice` at runtime — except in
`gripper_controller`, which hardcodes `/opt/lmx/`. The trajectory controller
additionally links `advancedmotionapi` for the C-spline path.

At **runtime** the dynamic linker must be able to find the SDK's shared libraries
(`libimdll.so` etc.): either an `ld.so.conf.d` entry for `/opt/wmx3/lib` (the SDK
installer's default) or `LD_LIBRARY_PATH=/opt/wmx3/lib` — relevant when running in
containers that only mount the SDK.

The manipulator launches need **root** for real-time scheduling; start them with
`sudo --preserve-env` as shown in
[launch_dobot_cr3a_manipulator.md](launch_dobot_cr3a_manipulator.md).

`joint_position_controller` has a launch test
(`test/test_joint_position_controller.py`, no hardware required) exercised by
`colcon test --packages-select wmx_r2_package`. The startup service sequence the
broadcaster depends on is documented in
[reference_general_nodes.md](reference_general_nodes.md).
