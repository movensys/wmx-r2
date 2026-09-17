# Differential Drive Controller Reference

Standalone rclcpp node (`wmx_r2_package/src/differential_drive_controller.cpp`)
that drives two Ethercat wheel axes directly via CoreMotion `StartVel` and exposes the
autonomy contract (command velocity in, odometry feedback out). The node holds
the math (kinematics, dead-reckoning, deltas) alongside the ROS wiring; the WMX3
device access is isolated in `DifferentialDriveControllerApi`.

```
/cmd_vel_safe ──────────────▶┌───────────────────────────────┐ ──▶ /odom_enc  (Odometry)
 (TwistStamped)              │ differential_drive_controller │ ──▶ /omega_enc (JointState)
configure / activate ───────▶│  (lifecycle)                  │ ──▶ /omega_cmd (JointState)
 from                        │  two loops @ rate (100 Hz)    │
 wmx_lifecycle_manager_node  │  WMX3 CoreMotion StartVel,    │ ──▶ /tf odom→base_link
                             │  one GetStatus per loop       │     (optional)
                             └───────────────────────────────┘
```

---

## Launch arguments

`wmx_r2_differential.launch.py`:

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use simulation clock |
| `config_file` | **required** | YAML with the differential node parameters, e.g. `example/diffbot_differential_config.yaml` |
| `wmx_param_file` | `""` | WMX3 parameter XML imported at engine start, e.g. `example/diffbot_wmx_parameters.xml`; empty imports nothing |

`wmx_r2_control_differential.launch.py` takes the same three, with two additions
and one tightening:

| Argument | Default | Description |
|---|---|---|
| `wmx_param_file` | **required** | Also fed to the xacro, so an empty value would blank the description's own default |
| `urdf_file` | **required** | Robot description xacro, e.g. `urdf/diffbot.wmx.urdf.xacro` |
| `controllers_file` | **required** | `ros2_control` controller manager YAML, e.g. `config/diffbot_controllers.yaml` |

The two launch files run **different controllers**.
`wmx_r2_differential.launch.py` runs this node. `wmx_r2_control_differential.launch.py`
does **not**: it starts `ros2_control_node` with `WmxSystemHardware` and spawns
`diff_drive_controller/DiffDriveController` under the same name, so everything
below describes the standalone node only.

No robot is baked into either launch file, see
[launch_differential.md](launch_differential.md).

---

## Parameters

All parameters are declared with defaults; the defaults already match the
autonomy contract, so a deployment only *needs* to override the per-robot
hardware values (group A below).

All parameters are read **once at node construction**; there is no
parameter-set callback. A runtime `ros2 param set` is accepted by rclcpp but
has no effect on behaviour — restart the node to apply new values.

### A. Per-robot hardware (must be set per machine)

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `left_axis` | int | `0` | – | WMX3 axis index of the left wheel. The range is checked on every status read: a value outside `[0, maxAxes)` fails the read with `ArgumentOutOfRange` and the cycle publishes nothing (throttled warn). Setting it equal to `right_axis` is **not** checked and gives a base that reads one wheel twice. |
| `right_axis` | int | `1` | – | WMX3 axis index of the right wheel. Same rules as `left_axis`. |
| `wheel_radius` | double | `0.095` | m | Drive-wheel radius `R`. Must be > 0 — `configure` is refused otherwise. |
| `wheel_to_wheel` | double | `0.55` | m | Wheel separation `L` (distance between the two drive wheels). Must be > 0 — `configure` is refused otherwise. |

The WMX parameter XML (axis gear/feedback/limit setup) is **not** a parameter of
this node: it is imported once by `wmx_engine_node` through its
`wmx_param_file_path` parameter, which the launch file forwards from its
`wmx_param_file` argument (example: `example/diffbot_wmx_parameters.xml`).

### B. Motion profile / loop

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `rate` | int | `100` | Hz | Rate of **both** loops: the control loop and the feedback loop each run at `rate` on their own wall timer and each issues its own `GetStatus`, so publishing can never delay a command. Must be > 0, or `configure` is refused. The timer period is `1000 / rate` truncated to whole milliseconds, so prefer rates that divide 1000 (100, 125, 200, 250, 500). |
| `acc_time` | double | `1.0` | **ms** | `StartVel` trapezoidal profile acceleration time (`profile.accTimeMilliseconds`, `ProfileType::TimeAccTrapezoidal`). Note the unit: milliseconds — the default 1.0 ms is effectively an instant ramp; the WMX-side axis limits do the real shaping. Not guarded: passed to WMX unvalidated. |
| `dec_time` | double | `1.0` | **ms** | Same as `acc_time` for deceleration. Also applies to the stale-command stop (see `cmd_vel_timeout`). |

Both timers are **wall timers**, but the odometry step `dt` comes from the ROS
clock, so `use_sim_time` affects pose integration as well as message stamps and
the `cmd_vel_timeout` stale check: a paused `/clock` freezes the pose (`dt = 0`,
no contribution) and a stretched one scales it. A paused `/clock` also disables
the stale-command stop (the last wheel target keeps being held), because both the
arrival stamp and the comparison come from the same paused clock.

### C. Behaviour / safety

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `cmd_vel_timeout` | double | `0.25` | s | Stale-command safety: if no command arrives within this window, the wheel target is forced to zero. Freshness is measured against **arrival time** on this node's clock, so a publisher whose clock runs ahead cannot hold the base alive after it dies. The stop decelerates over `dec_time`, so it is **not** an emergency stop; a true e-stop must go through the WMX hardware-level stop path. Not guarded: a negative value makes every cycle stale (permanent zero target, no warning). |
| `publish_tf` | bool | `false` | – | Publish `odom_frame → base_frame` TF from the integrated pose. Keep **false** when a localization EKF owns that TF (the EKF is launched when an IMU is configured). Enable only as the fallback for IMU-less / no-EKF configs where this node is the sole odometry source. |
| `odom_frame` | string | `odom` | – | `frame_id` for `/odom_enc` and the optional TF parent. |
| `base_frame` | string | `base_link` | – | `child_frame_id` for `/odom_enc` and the optional TF. |

### D. Topic names (defaults = autonomy contract)

| Parameter | Default | Description |
|---|---|---|
| `cmd_vel_topic` | `/cmd_vel_safe` | Command input (subscription). |
| `cmd_omega_topic` | `/omega_cmd` | Per-wheel velocity command output (what the control loop sends to `StartVel`). |
| `encoder_odometry_topic` | `/odom_enc` | Encoder odometry output (EKF `odom0` input). |
| `encoder_omega_topic` | `/omega_enc` | Per-wheel encoder velocity output. |
| `joint_name` | `["left_wheel_joint", "right_wheel_joint"]` | Wheel joint names published in `/omega_enc` and `/omega_cmd`, ordered `[left, right]` to match `left_axis`/`right_axis`. Needs at least 2 entries, or `configure` is refused; only the first two are used. |

Topic names are plain parameters (not ROS remap-only), so the Toolkit can set them
in the generated node config like any other value.

---

## Topics

| Topic (default) | Dir | Type | QoS | Rate | Notes |
|---|---|---|---|---|---|
| `/cmd_vel_safe` | sub | `geometry_msgs/TwistStamped` | default (reliable, volatile), depth 1 | producer | **TwistStamped is mandatory** by message type; the header stamp is not read, the staleness timeout runs on arrival time. Uses `twist.linear.x` [m/s], `twist.angular.z` [rad/s]. |
| `/odom_enc` | pub | `nav_msgs/Odometry` | default, depth 1 | `rate` | `header.frame_id = odom_frame`, `child_frame_id = base_frame`. **Pose** = dead-reckoned from the body twist over the loop `dt` (`Δs = v·dt`, `Δθ = ω·dt`), exact-arc via the sinc midpoint form. **Twist** = `vx`, `vy`(=0), `vyaw` from `actualVelocity` (forward kinematics). Covariance: see below. |
| `/omega_cmd` | pub | `sensor_msgs/JointState` | default, depth 1 | `rate` | `velocity = [left, right]` wheel angular velocity **command** [rad/s] — the last target sent to `StartVel` (zero while a fault gate holds), named by `joint_name`. Recording/monitoring only: written by the control loop, published by the feedback loop. `position` and `effort` are left empty. |
| `/omega_enc` | pub | `sensor_msgs/JointState` | default, depth 1 | `rate` | `velocity = [left, right]` wheel angular velocity [rad/s] (`actualVelocity` from `GetStatus`), named by `joint_name`. `header.stamp` is the loop time; `position` and `effort` are left empty. |
| `/tf` (`odom_frame → base_frame`) | pub | TF | tf2 default | `rate` | Only when `publish_tf: true`. |

**Namespaces.** The four data-topic defaults are *absolute* names, so launching
the node in a ROS namespace does **not** namespace them — override the topic
parameters explicitly for multi-robot/namespaced deployments. The lifecycle
services (`~/change_state`, `~/get_state`) do follow the namespace; the engine
discovers the controller under its fully-qualified name, and
`wmx/lifecycle/set_node_state` takes that same name.

### `/odom_enc` covariance (fixed, not parameterized)

The localization EKF fuses **only twist `vx`, `vy`, `vyaw`** from this
source (`robot_localization` `odom0_config`). Pose x/y/yaw are nevertheless kept
authoritative for the no-EKF fallback where this odometry feeds Nav2 directly.

| Block | Authoritative (variance `0.01`) | Non-authoritative (variance `99999`) |
|---|---|---|
| pose | x, y, yaw | z, roll, pitch |
| twist | vx, vy, vyaw | vz, v_roll, v_pitch |

### Known limitations (velocity-integrated odometry)

- **`dt` sensitivity:** pose is `Σ v·dt` with `dt` measured between feedback
  cycles, so it assumes constant velocity across the cycle and inherits any
  scheduling jitter in that interval. Errors accumulate without bound (no absolute
  reference) — the EKF, which fuses twist only, is unaffected.
- **Pose during engine downtime:** motion that happens while
  `engineState != Communicating` is not integrated; `dt` restarts on recovery, so
  that motion is simply lost from the pose.
- **Per-axis unit scaling:** `actualVelocity` is taken to be wheel rad/s. The
  loaded WMX param XML must scale both axes that way; verify in the XML / sim.
  There is no node-side conversion.

---

## Units and conventions

- Command: `linear.x` [m/s], `angular.z` [rad/s]; positive `angular.z` = CCW (REP-103).
- Wheel velocity (`StartVel` target and `actualVelocity` feedback) is the wheel
  angular velocity in **rad/s**. The WMX axis user-unit scaling (encoder counts,
  gear ratio) must be configured WMX-side — via the engine's
  `wmx_param_file_path` XML —
  so that one axis velocity unit = 1 rad/s at the wheel. There is no gear-ratio
  parameter in the node.
- Kinematics (`inverseKinematics` / `forwardKinematics`):
  - inverse: `ωl = (2v − ωL)/(2R)`, `ωr = (2v + ωL)/(2R)`
  - forward: `v = R(ωr + ωl)/2`, `ω = R(ωr − ωl)/L`
- **Odometry** (pose) is dead-reckoned from the body twist over each feedback
  cycle: `Δs = v·dt`, `Δθ = ω·dt`, applied with the exact-arc sinc midpoint form.
- **Twist** (`/odom_enc.twist`, `/omega_enc`) comes from the servo's
  `actualVelocity`, not `Δpos/dt`: the EKF fuses only twist, and the servo velocity
  is a cleaner signal than a numerical position derivative.

---

## Lifecycle and runtime behaviour

**Startup.** This is a managed (lifecycle) node. It starts `unconfigured` and
does nothing until `wmx_lifecycle_manager_node` drives it: automatically once
`wmx_engine_node` reports `Communicating`, or on demand through
`wmx/lifecycle/set_node_state` / `ros2 lifecycle set`. List it in the manager's
`managed_nodes` **after** the device-level nodes.

`on_configure`:

1. *Parameter check* (`parametersValid`) — `rate`, `wheel_radius`,
   `wheel_to_wheel` must be > 0 and `joint_name` must hold at least two entries.
   Anything else logs the offending value and fails the transition; there are
   **no silent fallbacks**, so a bad config never runs with substituted values.
   The check re-reads the values cached at construction, so a `ros2 param set`
   followed by a retry changes nothing. Fix the YAML and restart the node.
2. `CreateDevice(WMX3_SDK_PATH, DeviceTypeNormal, 10 s)` — any error fails the
   transition and leaves the node `unconfigured` (the lifecycle manager logs it
   and retries on the next discovery sweep).
3. `SetDeviceName("differential_drive_controller")`.

The WMX parameter XML is not imported here — `wmx_engine_node` does that once,
right after it creates the device, from its `wmx_param_file_path` parameter.

`on_activate` creates the publishers, the `cmd_vel` subscription and the control
timer, and re-baselines the odometry and command state (encoders may have moved
while inactive). `on_deactivate` drops the timer, commands both wheels to zero and
destroys those interfaces — while inactive the node publishes nothing and accepts
no commands. `on_cleanup` closes the device.

A failed `configure` is not terminal: the node stays alive and `unconfigured`,
and the transition can be retried at any time. The process never exits
with an error code on init failure — it stays alive and inert (no topics, no
timer). Supervise via logs or topic liveness (`/odom_enc` at `rate` Hz), not
exit codes.

The node does **not** start communication, clear alarms, or switch servos on —
that is owned by the engine/general nodes (see
`reference_general_nodes.md` for the service sequence).

**Manual vs. controller arbitration.** While this node is ACTIVE it owns the wheel
axes: `wmx_core_motion_node` rejects the `start_pos`, `start_mov`, `start_vel`, `start_jog`
and `start_home` for as long as it stays active (it is listed in that node's
`motion_controllers`). `wmx/axes/stop` is never blocked. See
`reference_general_nodes.md`.

Two timers, each every `1/rate` and each issuing its own `GetStatus`, so
publishing can never delay a command.

**Feedback loop** — `publishMotorFeedback`:

1. *Engine gate* — a failed `GetStatus` or `engineState != Communicating` drops the
   `dt` baseline (`haveFeedbackTime_`) and publishes nothing, so the next cycle
   after recovery contributes no pose step instead of integrating the gap. It
   stays silent; the warnings come from the control loop.
2. *Odometry* — runs even with servo off (encoder feedback stays valid): integrate
   the pose from the body twist (`v·dt`, `ω·dt`), both derived from
   `actualVelocity`; publish `/omega_cmd`, `/omega_enc`, `/odom_enc`, optional TF.
   `/omega_cmd` echoes the control loop's last wheel target for the record.

**Control loop** — `controlStep`:

1. *Engine gate* — a failed `GetStatus` or `engineState != Communicating`: warn
   (1 s throttle), command nothing, zero the wheels and drop the resend cache.
2. *Servo gate* — skipped (with 1 s-throttled warn, resend cache invalidated) while
   an amp alarm is active or either servo is off; recovery therefore always
   re-sends the current target.
3. *Stale-command check* — no fresh command within `cmd_vel_timeout` ⇒ target zero.
4. *Resend-on-change* — `StartVel` is re-sent only when the wheel target changes,
   so the velocity trapezoid is not restarted every cycle. The "last sent" cache
   commits only if **both** axes accept the command; a failed `StartVel` (e.g. a
   transient motion-state conflict) is retried next cycle — this guarantees a
   timeout→zero stop can never be swallowed by a failed send.

**Shutdown.** `deactivate` cancels both timers and commands both wheels to zero;
the destructor closes the WMX device.

**Process model.** Ships as a standalone executable on a `MultiThreadedExecutor`.
`controlStep` and the `cmd_vel` subscription share one **MutuallyExclusive**
callback group, which is what keeps the command state lock-free;
`publishMotorFeedback` runs in the node's default group, so it executes on
another thread. It is not built as a composable component — run it as its own
process, one per robot.

---

## Configuration files

A deployment consists of two files plus the launch wiring
(example: `launch/wmx_r2_differential.launch.py`):

1. **ROS parameter YAML** — the differential config (example:
   `example/diffbot_differential_config.yaml`), key
   `differential_drive_controller.ros__parameters` (all tables above), plus the
   `wmx_engine_node` key (`core`, `affinity_mask`, `wmx_param_file_path`) and
   the other general-node keys.
2. **WMX parameter XML** — the axis file (example:
   `example/diffbot_wmx_parameters.xml`): axis-level
   gear/feedback/limit/e-stop setup. This is where the
   "axis unit = wheel rad/s" scaling and the hardware-level motion limits live.
   `wmx_engine_node` imports it once, right after it creates the device.
3. **Launch** — starts the general WMX nodes (engine etc.), the
   `joint_state_broadcaster`, and this node; injects `use_sim_time` and the
   engine's `wmx_param_file_path` from the `wmx_param_file` launch argument.
   `config_file` is a required launch argument, so the same YAML reaches this
   node and the general nodes.

```yaml
differential_drive_controller:
  ros__parameters:
    left_axis: 0
    right_axis: 1
    rate: 100
    acc_time: 1.0        # ms (StartVel trapezoid)
    dec_time: 1.0        # ms
    wheel_radius: 0.095  # m
    wheel_to_wheel: 0.55 # m
    cmd_vel_timeout: 0.25     # s — stale-command stop window
    publish_tf: false    # true only for IMU-less / no-EKF configs
    odom_frame: odom
    base_frame: base_link
    joint_name: ["drivewheel_left_joint", "drivewheel_right_joint"]
    cmd_vel_topic: /cmd_vel_safe
    cmd_omega_topic: /omega_cmd
    encoder_odometry_topic: /odom_enc
    encoder_omega_topic: /omega_enc

joint_state_broadcaster:
  ros__parameters:
    joint_feedback_rate: 100
    joint_axes: [0, 1]
    joint_name: ["drivewheel_left_joint", "drivewheel_right_joint"]
    encoder_joint_topic: /joint_states
    isaacsim_joint_topic: /isaacsim/joint_command
    gazebo_velocity_joint_topic: /velocity_controller/commands
    gazebo_velocity_joint_axes: [0, 1]

wmx_engine_node:
  ros__parameters:
    core: -1                # RT engine CPU core (-1 = SDK default)
    affinity_mask: 0        # CPU affinity bitmask (0 = SDK default)
    wmx_param_file_path: "" # injected by launch

wmx_core_motion_node:
  ros__parameters:
    axes_status_rate: 100
    motion_controllers:     # this node owns the wheel axes while active
      - differential_drive_controller
    controller_resync_period: 0.2

wmx_lifecycle_manager_node:
  ros__parameters:
    managed_nodes:          # device-level nodes first
      - wmx_core_motion_node
      - wmx_io_node
      - wmx_ethercat_node
      - joint_state_broadcaster
      - differential_drive_controller
    discovery_period: 1.0
```

---

## Toolkit integration checklist

What the Toolkit needs to template per robot / per deployment:

- **Always per robot:** `left_axis`, `right_axis`, `wheel_radius`,
  `wheel_to_wheel`, and the WMX parameter XML (the engine's
  `wmx_param_file_path`).
- **Per deployment config:** `publish_tf` — must be `true` exactly when the
  localization EKF is *not* running (the EKF launches only with an IMU
  configured); otherwise two publishers would fight over `odom → base_link`.
- **Usually defaults:** topic names (already the autonomy contract), frames, `rate`,
  `cmd_vel_timeout`, `acc_time`/`dec_time`.
- **Not parameterized (by design):** the lifecycle gate (`wmx_lifecycle_manager_node` owns it),
  the `/odom_enc` covariance values, servo-on/alarm-clear handling (engine/general
  nodes own these), and any gear-ratio scaling (WMX XML owns it).

## Build

`wmx_r2_package` compiles against the WMX3 SDK at the CMake cache path
`WMX3_SDK_PATH` (default `/opt/wmx3`); the same path (with a trailing `/`
appended by CMake) is compiled in and passed to `CreateDevice` at runtime.
The node is built from `src/differential_drive_controller.cpp` with its header in
`include/`; run the package tests with
`colcon test --packages-select wmx_r2_package`.

At **runtime** the dynamic linker must be able to find the SDK's shared
libraries (`libimdll.so` etc.): either an `ld.so.conf.d` entry for
`/opt/wmx3/lib` (the SDK installer's default) or
`LD_LIBRARY_PATH=/opt/wmx3/lib` — relevant when running in containers that
only mount the SDK.

Verified on ROS 2 Humble (Ubuntu 22.04, native) and ROS 2 Jazzy
(Ubuntu 24.04, `ros:jazzy-ros-base` container): clean build of all four
packages against the real WMX3 SDK, all unit tests green, node startup smoke
test.
