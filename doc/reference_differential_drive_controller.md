# Differential Drive Controller Reference

Standalone rclcpp node (`wmx_r2_package/src/differential_drive_controller.cpp`)
that drives two WMX3 wheel axes directly via CoreMotion `StartVel` and exposes the
autonomy contract (command velocity in, odometry feedback out). The WMX/ROS-free
math (kinematics, dead-reckoning, deltas, accel EMA) lives in the unit-tested
header `differential_drive_controller.hpp`; the node is the ROS/WMX wiring around it.

```
/cmd_vel_safe ──────────▶ ┌──────────────────────────────┐ ──▶ /odom_enc    (Odometry)
                          │ differential_drive_controller │ ──▶ /odom_deltas (TwistStamped)
lifecycle configure/    ─▶│  single loop @ rate (100 Hz)  │ ──▶ /odom_accel  (AccelStamped)
activate (manager)        │  WMX3 CoreMotion StartVel /   │ ──▶ /omega_enc   (Float64MultiArray)
                          │  GetStatus (one per cycle)    │ ──▶ /tf odom→base_link (optional)
                          └──────────────────────────────┘
```

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
| `left_axis` | int | `0` | – | WMX3 axis index of the left wheel. **Not validated**: must be a valid axis index and ≠ `right_axis`; an out-of-range value causes an out-of-bounds status read (garbage odometry / undefined behaviour), not an error message. |
| `right_axis` | int | `1` | – | WMX3 axis index of the right wheel. Same caveat as `left_axis`. |
| `wheel_radius` | double | `0.095` | m | Drive-wheel radius `R`. Guarded: values ≤ 0 fall back to the default (warn). |
| `wheel_to_wheel` | double | `0.55` | m | Wheel separation `L` (distance between the two drive wheels). Guarded: ≤ 0 falls back to the default (warn). |
| `pos_unit_scale` | double | `1.0` | rad/user-unit | Converts `actualPos` (WMX user-unit) to wheel **radians** for the position-delta odometry. `1.0` when the loaded WMX param XML already scales to rad (same scaling that makes `actualVelocity` read rad/s — our config). Guarded: non-finite or `0` falls back to `1.0` (warn). |
| `wmx_param_file_path` | string | `/diff_drive/no_param` | – | WMX parameter XML imported at init via `config->ImportAndSetAll()` (axis gear/feedback/limit setup). The default is a deliberate non-path: the import fails with an ERROR log but the node keeps running with whatever parameters the engine already has. The diffbot launch file overrides it with `config/diffbot_wmx_parameters.xml` resolved at launch time. |

### B. Motion profile / loop

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `rate` | int | `100` | Hz | Control-loop rate. One loop does **one** `GetStatus` and drives both the odometry and the command path. Guarded: ≤ 0 falls back to 100. The timer period is `1000 / rate` truncated to whole milliseconds, so prefer rates that divide 1000 (100, 125, 200, 250, 500). |
| `acc_time` | double | `1.0` | **ms** | `StartVel` trapezoidal profile acceleration time (`profile.accTimeMilliseconds`, `ProfileType::TimeAccTrapezoidal`). Note the unit: milliseconds — the default 1.0 ms is effectively an instant ramp; the WMX-side axis limits do the real shaping. Not guarded: passed to WMX unvalidated. |
| `dec_time` | double | `1.0` | **ms** | Same as `acc_time` for deceleration. Also applies to the stale-command stop (see `cmd_vel_timeout`). |

The control timer is a **wall timer**. Pose and `/odom_deltas` are integrated from
encoder **position deltas** (dt-free), so a paused or stretched `/clock` does **not**
corrupt them. `use_sim_time` (ROS clock) only affects time-derived quantities:
message stamps, the `cmd_vel_timeout` stale check, the jump guard's expected step
(`actualVelocity·dt`), and the `/odom_accel` rate limit. A paused `/clock` still
disables the stale-command stop (the last wheel target keeps being held).

### C. Behaviour / safety

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `cmd_vel_timeout` | double | `0.25` | s | Stale-command safety: if no command is fresh within this window, the wheel target is forced to zero. Freshness is measured against the command's **header stamp** (rejects stale/buffered commands, not just gaps in receipt; a zero/unset stamp falls back to arrival time). The stop decelerates over `dec_time` — **not** an emergency stop; a true e-stop must go through the WMX hardware-level stop path. Not guarded: a negative value makes every cycle stale (permanent zero target, no warning). |
| `accel_publish_rate` | double | `10.0` | Hz | Rate limit for `/odom_accel` relative to the control loop. `0` = publish every control cycle. Guarded: negative values fall back to 10.0. |
| `accel_alpha` | double | `0.3` | – | EMA weight of the newest raw acceleration sample, valid range (0, 1]; higher = more responsive, lower = smoother. The estimator snaps to zero when both the current and previous velocity samples are ~0 (kills the EMA tail at standstill). Not guarded: the range is not enforced (0 pins `/odom_accel` to zero; >1 destabilizes the EMA — validate in the config layer). |
| `publish_tf` | bool | `false` | – | Publish `odom_frame → base_frame` TF from the integrated pose. Keep **false** when a localization EKF owns that TF (the EKF is launched when an IMU is configured). Enable only as the fallback for IMU-less / no-EKF configs where this node is the sole odometry source. |
| `odom_frame` | string | `odom` | – | `frame_id` for `/odom_enc`, `/odom_deltas` and the optional TF parent. |
| `base_frame` | string | `base_link` | – | `child_frame_id` for `/odom_enc`/TF and `frame_id` for `/odom_accel`. |
| `jump_guard_tol` | double | `0.5` | rad | Velocity-consistency guard for the position-delta odometry: if a per-wheel step `|Δφ − actualVelocity·dt|` exceeds this, the cycle is treated as a homing / encoder-rollover / glitch jump — odometry re-baselines (contributes nothing that cycle) and warns, instead of integrating a bogus jump. Generous by default (a real ~10 ms step's `Δφ` and `actualVelocity·dt` agree closely; only gross jumps trip). Guarded: ≤ 0 falls back to 0.5. |

### D. Topic names (defaults = autonomy contract)

| Parameter | Default | Description |
|---|---|---|
| `cmd_vel_topic` | `/cmd_vel_safe` | Command input (subscription). |
| `encoder_odometry_topic` | `/odom_enc` | Encoder odometry output (EKF `odom0` input). |
| `encoder_omega_topic` | `/omega_enc` | Per-wheel encoder velocity output. |
| `odom_deltas_topic` | `/odom_deltas` | Accumulated travel output (DistanceTraveled monitor). |
| `odom_accel_topic` | `/odom_accel` | Body acceleration output (Motion monitor). |

Topic names are plain parameters (not ROS remap-only), so the Toolkit can set them
in the generated node config like any other value.

---

## Topics

| Topic (default) | Dir | Type | QoS | Rate | Notes |
|---|---|---|---|---|---|
| `/cmd_vel_safe` | sub | `geometry_msgs/TwistStamped` | default (reliable, volatile), depth 1 | producer | **TwistStamped is mandatory** — the header stamp drives the staleness timeout (a zero/unset stamp falls back to arrival time). Uses `twist.linear.x` [m/s], `twist.angular.z` [rad/s]. |
| `/odom_enc` | pub | `nav_msgs/Odometry` | default, depth 1 | `rate` | `header.frame_id = odom_frame`, `child_frame_id = base_frame`. **Pose** = dead-reckoned from per-wheel encoder **position deltas** (`actualPos`), exact-arc via the sinc midpoint form (dt-free). **Twist** = `vx`, `vy`(=0), `vyaw` from `actualVelocity` (forward kinematics). Covariance: see below. |
| `/odom_deltas` | pub | `geometry_msgs/TwistStamped` | default, depth 1 | `rate` | Accumulated `Σ|Δs|` (in `twist.linear.x`, m) and `Σ|Δθ|` (in `twist.angular.z`, rad) from encoder **position deltas** since the previous publish (more exact than `Σ|v|·dt`); resets each publish. `frame_id = odom_frame`. |
| `/odom_accel` | pub | `geometry_msgs/AccelStamped` | default, depth 1 | `accel_publish_rate` | EMA-filtered derivative of body velocity over the actual inter-publish interval. `frame_id = base_frame`. |
| `/omega_enc` | pub | `std_msgs/Float64MultiArray` | default, depth 1 | `rate` | `data = [left, right]` wheel angular velocity [rad/s] (`actualVelocity` from `GetStatus`). |
| `/tf` (`odom_frame → base_frame`) | pub | TF | tf2 default | `rate` | Only when `publish_tf: true`. |

**Namespaces.** The five data-topic defaults are *absolute* names, so launching
the node in a ROS namespace does **not** namespace them — override the topic
parameters explicitly for multi-robot/namespaced deployments. The lifecycle
services (`~/change_state`, `~/get_state`) follow the namespace, so
`wmx_lifecycle_manager_node` must run where it can discover the controller —
in practice, the **same namespace** — or the controller is never brought up.

### `/odom_enc` covariance (fixed, not parameterized)

The localization EKF fuses **only twist `vx`, `vy`, `vyaw`** from this
source (`robot_localization` `odom0_config`). Pose x/y/yaw are nevertheless kept
authoritative for the no-EKF fallback where this odometry feeds Nav2 directly.

| Block | Authoritative (variance `0.01`) | Non-authoritative (variance `99999`) |
|---|---|---|
| pose | x, y, yaw | z, roll, pitch |
| twist | vx, vy, vyaw | vz, v_roll, v_pitch |

### Known limitations (position-delta odometry)

- **Long-uptime precision:** pose uses `actualPos` (a `double` user-unit); over very
  long uptime `actualPos` grows large and `actualPos − prev` loses low-order bits
  (catastrophic cancellation of two large near-equal doubles). The exact-integer
  alternative is `CoreMotionAxisStatus.accumulatedEncoderFeedback` (`long long`)
  differenced as integers then scaled — switch to it if this ever surfaces.
- **`/odom_deltas` during engine downtime:** motion that happens while
  `engineState != Communicating` is not accumulated (the baseline re-anchors on
  recovery). Correct — it was unobservable — but a change from the old `|v|·dt`
  accumulation that ran whenever the engine was communicating.
- **Per-axis unit scaling:** the rad assumption (`pos_unit_scale`) requires both
  axes to share identical WMX user-unit scaling; verify in the WMX param XML / sim.
- **Jump-guard trip:** when the guard trips it drops that cycle's contribution to
  **both** the `/odom` pose and `/odom_deltas` (re-baselines instead of integrating
  the jump). Intended for homing/rollover; the EKF is unaffected (twist only), but
  the DistanceTraveled monitor very slightly under-counts across such an event.

---

## Units and conventions

- Command: `linear.x` [m/s], `angular.z` [rad/s]; positive `angular.z` = CCW (REP-103).
- Wheel velocity (`StartVel` target and `actualVelocity` feedback) is the wheel
  angular velocity in **rad/s**. The WMX axis user-unit scaling (encoder counts,
  gear ratio) must be configured WMX-side — via the `wmx_param_file_path` XML —
  so that one axis velocity unit = 1 rad/s at the wheel. There is no gear-ratio
  parameter in the node.
- Kinematics (`diff_drive::DiffDriveModel`):
  - inverse: `ωl = (2v − ωL)/(2R)`, `ωr = (2v + ωL)/(2R)`
  - forward: `v = R(ωr + ωl)/2`, `ω = R(ωr − ωl)/L`
- **Odometry** (pose + `/odom_deltas`) is dead-reckoned from per-wheel encoder
  **position deltas** `Δφ = (actualPos − prev)·pos_unit_scale` (dt-free; exact-arc
  via the sinc midpoint form). This is more precise than `velocity·dt` — no
  constant-velocity-over-`dt` assumption and no `dt`-jitter sensitivity.
- **Twist** (`/odom_enc.twist`, `/odom_accel`, `/omega_enc`) comes from the servo's
  `actualVelocity`, not `Δpos/dt`: the EKF fuses only twist, and the servo velocity
  is a cleaner signal than a numerical position derivative. (The velocity-based
  `odometryPoseCalculation(vel,dt)` path remains in the header, unit-tested, but the
  node now uses the position-delta path.)

---

## Lifecycle and runtime behaviour

**Startup.** The node is a managed lifecycle node: it comes up `unconfigured` and
does nothing until `wmx_lifecycle_manager_node` transitions it (the manager waits
for the engine to report `Communicating` first). See "Node Lifecycle" in
`reference_wmx_r2_general_nodes.md`.

`on_configure`:

1. `CreateDevice(WMX3_SDK_PATH, DeviceTypeNormal, 10 s)` — any error fails the
   transition and leaves the node `unconfigured`.
2. `SetDeviceName("differential_drive_controller")`.
3. `ImportAndSetAll(wmx_param_file_path)` — failure is logged but non-fatal.
4. Create publishers/subscriber and the control timer, with the timer cancelled.

`on_activate` clears the odometry baselines and starts the control timer.
`on_deactivate` stops the timer and commands both wheels to zero velocity;
`on_cleanup` destroys the interfaces and closes the device.

A failed `configure` is not terminal: the transition returns `FAILURE`, the node
stays `unconfigured`, and the manager retries on its next discovery sweep. The
process never exits with an error code on a failed transition — it stays alive
and inert (no topics, no timer). Supervise via `wmx/lifecycle/get_node_states` or
topic liveness (`/odom_enc` at `rate` Hz), not exit codes.

The node does **not** start communication, clear alarms, or switch servos on —
that is owned by the engine/general nodes (see
`reference_wmx_r2_general_nodes.md` for the service sequence).

**Control loop** (every `1/rate`, single `GetStatus` per cycle):

1. *Engine gate* — if `engineState != Communicating`: warn (1 s throttle), publish
   nothing, command nothing, and drop the position baseline (`havePrev_`) + resend
   cache so odometry and the `StartVel` state re-baseline cleanly on recovery
   (re-anchoring to the current absolute encoder position).
2. *Odometry path* — runs even with servo off (encoder feedback stays valid):
   integrate the pose and `/odom_deltas` from per-wheel encoder **position deltas**
   (twist/accel from `actualVelocity`); the jump guard re-baselines on a
   homing/rollover jump; publish `/omega_enc`, `/odom_enc`, `/odom_deltas`,
   `/odom_accel` (rate-limited), optional TF.
3. *Command path* — skipped (with 1 s-throttled warn, resend cache invalidated)
   while an amp alarm is active or either servo is off; recovery therefore always
   re-sends the current target.
4. *Stale-command check* — no fresh command within `cmd_vel_timeout` ⇒ target zero.
5. *Resend-on-change* — `StartVel` is re-sent only when the wheel target changes,
   so the velocity trapezoid is not restarted every cycle. The "last sent" cache
   commits only if **both** axes accept the command; a failed `StartVel` (e.g. a
   transient motion-state conflict) is retried next cycle — this guarantees a
   timeout→zero stop can never be swallowed by a failed send.

**Shutdown.** Destructor cancels the timer, commands both wheels to zero, and
closes the WMX device.

**Process model.** Ships as a standalone executable on the default
single-threaded executor; the command state shared between the `cmd_vel`
callback and the control loop is unsynchronized by design (single-threaded
callback execution is a correctness assumption). It is not built as a
composable component — run it as its own process, one per robot.

---

## Configuration files

A deployment consists of two files plus the launch wiring
(example: `launch/wmx_r2_diffbot_navigation.launch.py`):

1. **ROS parameter YAML** — `config/diffbot_navigation_config.yaml`, key
   `differential_drive_controller.ros__parameters` (all tables above).
2. **WMX parameter XML** — `config/diffbot_wmx_parameters.xml`: axis-level
   gear/feedback/limit/e-stop setup imported at node init. This is where the
   "axis unit = wheel rad/s" scaling and the hardware-level motion limits live.
3. **Launch** — starts the general WMX nodes (engine etc.), the
   `joint_state_broadcaster`, and this node; injects `wmx_param_file_path`
   (resolved from the package share at launch time) and `use_sim_time`.

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
    accel_publish_rate: 10.0  # Hz — /odom_accel rate limit (0 = every cycle)
    accel_alpha: 0.3
    publish_tf: false    # true only for IMU-less / no-EKF configs
    odom_frame: odom
    base_frame: base_link
    cmd_vel_topic: /cmd_vel_safe
    encoder_odometry_topic: /odom_enc
    encoder_omega_topic: /omega_enc
    odom_deltas_topic: /odom_deltas
    odom_accel_topic: /odom_accel
    wmx_param_file_path: ""  # injected by launch
```

---

## Toolkit integration checklist

What the Toolkit needs to template per robot / per deployment:

- **Always per robot:** `left_axis`, `right_axis`, `wheel_radius`,
  `wheel_to_wheel`, and the WMX parameter XML (`wmx_param_file_path`).
- **Per deployment config:** `publish_tf` — must be `true` exactly when the
  localization EKF is *not* running (the EKF launches only with an IMU
  configured); otherwise two publishers would fight over `odom → base_link`.
- **Usually defaults:** topic names (already the autonomy contract), frames, `rate`,
  `cmd_vel_timeout`, `accel_publish_rate`, `accel_alpha`, `acc_time`/`dec_time`.
- **Not parameterized (by design):** the lifecycle gate (the manager owns it), the
  `/odom_enc` covariance values, servo-on/alarm-clear handling (engine/general
  nodes own these), and any gear-ratio scaling (WMX XML owns it).

## Build

`wmx_r2_package` compiles against the WMX3 SDK at the CMake cache path
`WMX3_SDK_PATH` (default `/opt/wmx3`); the same path (with a trailing `/`
appended by CMake) is compiled in and passed to `CreateDevice` at runtime.
The diff-drive logic is header-only (`include/differential_drive_controller.hpp`,
no WMX/ROS deps) and unit-tested via 4 gtest suites in this package —
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
