# Run the Buffered Servo Streamer on the Dobot CR3A

How to drive the **six-axis CR3A** with MoveIt Servo through the WMX3 API buffer,
i.e. `servo_stream_controller` in place of `joint_position_controller`.

- Node reference: [reference_servo_stream_controller.md](reference_servo_stream_controller.md)
- Single-axis bench procedure: [launch_single_servo_streamer.md](launch_single_servo_streamer.md)
- Normal (direct-path) CR3A launch: [launch_manipulator.md](launch_manipulator.md)

---

## ⚠ Read this before the first six-axis run

Every characterisation of the buffered path so far was done on **one axis**, and
those measurements predate the removal of the pacing block — so at the time of
writing **neither** the one-axis nor the six-axis case has been verified on
hardware against the current design. Run
[launch_single_servo_streamer.md](launch_single_servo_streamer.md) end to end
first; this guide assumes it passed.

There is also a specific reason to expect six axes to behave differently.

`StartLinearIntplPos` costs the engine roughly 1 ms of setup time per setpoint on
one axis. There is no pacing loop to absorb that any more — each setpoint is one
block, executed as soon as it lands — so the question is no longer whether the
loop has an equilibrium but simply whether the engine can consume setpoints
faster than they arrive. At 40 Hz that is a 25× margin on one axis. If the block
cost scales with axis count, six axes shrink the margin, and depth standing
above 0–1 is the symptom.

So the first six-axis run is a **measurement, not a teleop session**. Do §4
before §5, and only pick up a teleop device once depth has been seen to stay at
0–1 and the motion has been seen to be smooth.

---

## 1. Environment

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspaces/movensys_ws/install/setup.bash

export MANIPULATOR_MODEL=dobot_cr3a    # selects config/dobot_cr3a/* in the moveit config
export ROBOT=cr3a
```

`MANIPULATOR_MODEL` defaults to `dobot_cr3a`, so it is only strictly needed if
your shell has it set to something else — check it rather than assume:

```bash
echo "${MANIPULATOR_MODEL:-dobot_cr3a}"
```

Root access uses the `~/wmxrun` wrapper described in
[launch_single_servo_streamer.md](launch_single_servo_streamer.md#0-environment).

---

## 2. Bring up the arm with the buffered path — terminal 1

```bash
~/wmxrun ros2 launch wmx_r2_package wmx_r2_manipulator.launch.py \
  use_sim_time:=false use_gripper:=true use_api_buffer:=true \
  'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr3a_manipulator_config.yaml' \
  'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr3a_wmx_parameters.xml'
```

`use_api_buffer:=true` is the whole difference from the standard CR3A launch
(see [launch_manipulator.md](launch_manipulator.md)). The launch starts:

| node | note |
|---|---|
| general nodes | engine, core motion, io, EtherCAT |
| `joint_state_broadcaster` | publishes `/joint_states`; **servos the axes on** during its init |
| `joint_trajectory_controller` | MoveIt trajectory execution; also publishes the `execution_active` interlock |
| `servo_stream_controller` | **the buffered path** — replaces `joint_position_controller` |
| `gripper_controller` | |

Exactly one of `servo_stream_controller` / `joint_position_controller` runs; the
launch condition enforces it, since both subscribe to the same topic and would
otherwise double-command the arm. Do not start either by hand alongside this
launch.

Parameters come from the `servo_stream_controller:` block of
`example/cr3a_manipulator_config.yaml` — all six axes, `nominal_period_us: 25000`,
`max_joint_velocity: 3.14`.

Expect, from the controller:

```
Recorded e-stop routine into channel 1
Watch enabled on 6 axes, trigger routine on channel 1
axis 0 motion params: calcMode=AxisLimit, overrideType=FastBlending, apiWaitUntilMotionStart=false
servo_stream_controller is ready
```

`Watch enabled on 6 axes` confirms the RT-side safety net covers the whole arm.
No `Stream started` yet — the stream begins on the first setpoint.

**Check the `motion params` line before going further.** Neither value is set by
this node; both come from the axis parameter file, and both decide whether the
streaming design holds at all:

| field | wanted | if it differs |
|---|---|---|
| `overrideType` | `FastBlending` (or `Blending`) | `Smoothing` means consecutive setpoints do not override each other, so the motion will be stop-and-go no matter what this node does. Everything in §4 would be measuring the wrong thing. The node warns. |
| `calcMode` | `AxisLimit` | Anything else means the per-axis velocity and acceleration limits the node computes may not govern the profile. The node warns. |

It reports `axis[0]` only, since that is the axis whose calc mode governs the
whole interpolation.

---

## 3. Axis bring-up — mostly automatic on this launch

Unlike the single-axis bench procedure, **this launch does the bring-up for you**.
Confirm it in the log rather than repeating it by hand:

| step | done by | log line to look for |
|---|---|---|
| Load axis parameters | `wmx_engine_node`, from the `wmx_param_file` launch argument | per-axis numerator / denominator / polarity / abs encoder after `Device created` (a failure logs `Failed to load params from ...`) |
| Clear alarms | `joint_state_broadcaster`, on activation by `wmx_lifecycle_manager_node` | `wmx/axes/clear_amp_alarm succeeded: ...` |
| Servo on | `joint_state_broadcaster`, on activation by `wmx_lifecycle_manager_node` | `wmx/axes/set_servo_on succeeded: ...` |

If the parameter lines are missing, check `wmx_param_file` in the launch —
the axes will otherwise run on whatever scaling the engine already had.

### Homing — do not do this routinely ⚠

`/wmx/axes/start_home` sets `HomeType::CurrentPos` and starts homing, which
**redefines the axis' current physical position as zero**. The CR3A reports
`abs encoder: 1` on all six axes, so it already knows where it is at power-on and
does not need homing to establish a position.

Homing at an arbitrary pose on an absolute-encoder arm discards the calibrated
zero. From that point the URDF and MoveIt kinematics no longer describe the
physical arm, so planning and collision checking are computed against the wrong
pose. Only home deliberately, with the arm at its known zero pose, and treat it
as a recalibration rather than a startup step.

Nothing in the streaming path needs it: `servo_stream_controller` seeds each
segment from `posCmd`, which the engine initialises from the absolute encoder.

Verify feedback before going further:

```bash
ros2 topic hz /joint_states            # ~100 Hz
ros2 topic echo --once /joint_states   # all six joint names present
```

---

## 4. Six-axis validation — do this before any teleop

Use the synthetic source rather than Servo, so the input is a known constant and
nothing depends on IK, collision checking, or an operator's hand.

**It must publish all six joints.** `servo_stream_controller` drops any setpoint
that does not command every axis it drives, with
`Dropped trajectory: no command for axis N` — a six-axis controller fed a
one-joint stream simply never starts.

### 4a. Hold test — zero commanded motion

```bash
ros2 run wmx_r2_package servo_stream_test_source --ros-args \
  -p joint_name:="[joint1,joint2,joint3,joint4,joint5,joint6]" \
  -p amplitude_rad:=0.0
```

In another terminal:

```bash
ros2 topic echo /servo_stream_controller/queue_depth
```

| result | meaning |
|---|---|
| Depth reads **0**, no warnings, no fault | Good. On a pure hold every setpoint falls under `min_step_rad` once tracking converges, so no blocks are recorded at all. What this confirms is that the channel stays Active and error-free while idle — it does not prove motion will be fine. |
| Depth reads 0 but `Motion channel stopped` appears | Stop. The channel is faulting with nothing being commanded; read the cause in the message before going near motion. |
| Depth standing above 0–1 | Unexpected on a hold — blocks are being recorded when they should not be. Check `min_step_rad` against the arm's encoder noise. |

### 4b. Motion test — the measurement that matters

```bash
ros2 run wmx_r2_package servo_stream_test_source --ros-args \
  -p joint_name:="[joint1,joint2,joint3,joint4,joint5,joint6]" \
  -p amplitude_rad:=0.02
```

All six joints move together, ±20 mrad at 0.25 Hz, ramping in over 2 s. Watch
`queue_depth` for at least 30 s.

| result | meaning | do |
|---|---|---|
| Stays at 0–1 | The engine keeps up on six axes | Continue to §5 |
| Sits at 2–3 | Block cost scales with axis count; margin is thinner but workable | Continue to §5, and note it |
| Trips `API buffer not draining` | Blocks are arriving faster than the engine executes them | Raise `nominal_period_us` to match the publisher's real rate first; if it persists, the six-axis block cost is the limit |
| `Streaming but no blocks consumed` | Channel Active but the engine has stalled | Stop and diagnose — check axis state, servo-on, alarms |

Also watch for `Pump behind: skipped N setpoint(s)` — occasional entries are
normal on a loaded host, sustained ones mean the pump thread is being starved —
and for any `Motion channel stopped (...)`, which always means stop and diagnose.

**Judge the motion itself, not only the depth.** All six joints should travel
smoothly. Visible 40 Hz stepping is the defect this path was rebuilt to remove,
so if it is still there, go back to the `overrideType` check in §2 before
tuning anything.

Ctrl-C the test source when done. Expect `Servo stream starved (>75 ms);
stopping axes`, then `Stream ended`, and depth → 0.

---

## 5. MoveIt and Servo — terminal 2

Only once §4b passed: depth at 0–1 and the motion visibly smooth.

```bash
ros2 launch movensys_manipulator_moveit_config moveit.launch.py use_sim_time:=false
```

This starts `move_group`, RViz, the robot state publisher, the trajectory
service, and a composable container holding `servo_node`, configured from
`config/dobot_cr3a/servo.yaml`. The parts of that config that matter here:

| setting | value | why it matters |
|---|---|---|
| `publish_period` | `0.025` | 40 Hz — must match `nominal_period_us: 25000` |
| `command_out_topic` | `/movensys_manipulator_arm_controller/joint_trajectory` | must match the controller's `joint_trajectory_topic` |
| `command_out_type` | `trajectory_msgs/JointTrajectory` | |
| `publish_joint_positions` | `true` | the controller reads positions |
| `incoming_command_timeout` | `0.1` | Servo stops publishing 100 ms after teleop input stops |
| `joint` scale | `0.5` | max joint speed for joint-jog, rad/s |
| `check_collisions` | `true` | Servo-side protection, separate from the RT watch |

If you change Servo's `publish_period`, change `nominal_period_us` to match — it
is the time each segment is given to cover its distance, so it sets commanded
velocity directly. Too short over-drives and brings back stop-and-go; too long
under-drives and the arm lags the stream.

---

## 6. Teleop — terminal 3

Keyboard:

```bash
ros2 run movensys_manipulator_moveit_config keyboard_teleop
```

It publishes to `/servo_node/delta_twist_cmds`, `/servo_node/delta_joint_cmds`,
and `/servo_node/pose_target_cmds`, and switches Servo's command type through
`/servo_node/switch_command_type`.

For the Quest controller path see
[quest_servo_teleop.md](https://github.com/movensys/movensys-manipulator/blob/main/doc/quest_servo_teleop.md)
in the manipulator repo.

> These executables are built only where `moveit_msgs/srv/ServoCommandType`
> exists — Jazzy and newer. On Humble the package skips them at configure time
> with a `STATUS` message, so a missing executable means the guard tripped, not
> that the build failed.

### What normal teleop looks like

**A start/stop cycle per jog burst is expected.** Servo stops publishing 100 ms
after your input stops (`incoming_command_timeout`, plus four halt messages), and
the controller then starves 75 ms later and does a controlled stop. So each pause
in jogging produces:

```
Servo stream starved (>75 ms); stopping axes
Stream ended: stream starved (axes stopped)
```

and the next jog produces a fresh `Stream started (channel 0 Active)`. That is
the configuration working as designed, not a fault — `starvation_timeout_ms: 75`
is deliberately shorter than Servo's own timeout.

What should **not** appear on those restarts is `Motion channel stopped` or
`holding off`.

### Interlock with MoveIt trajectory execution

`joint_trajectory_controller` publishes `/moveit2_trajectory/execution_active`.
While it is true — you planned and executed from RViz — `servo_stream_controller`
clears its ring, ends the stream without stopping the axes, and discards
setpoints, because the trajectory controller owns the arm. Streaming resumes when
execution finishes.

If teleop appears dead with the stream up and no warnings, check that topic
first:

```bash
ros2 topic echo /moveit2_trajectory/execution_active
```

---

## 7. What to expect, and what is a fault

| observation | verdict |
|---|---|
| `queue_depth` at 0–1 while streaming | healthy — 0 is the expected value, not a symptom |
| `queue_depth` at 0 with the stream down | expected, and indistinguishable from healthy on this topic alone — depth is no longer a liveness signal |
| Start/stop cycles as jogging pauses | expected, see §6 |
| Occasional `Pump behind: skipped N setpoint(s)` | expected on a loaded host |
| `Setpoint needs X rad/s, over max_joint_velocity` | the arm is catching up from behind, or the ceiling is set too low for this motion |
| `API buffer not draining: N blocks in flight` | **fault** — the engine is not consuming as fast as setpoints arrive |
| `Streaming but no blocks consumed in N ms` | **fault** — channel Active but the engine has stalled mid-interpolation |
| Motion in visible surges rather than smoothly | **fault** — this is what removing the pacing block was meant to fix; check `overrideType` from §2 |
| Arm lag that *grows* over a run | **fault** — the `posCmd`-referenced segment origin is not doing its job |
| `ROS queue full ... dropping oldest` | **fault** — the pump is not keeping up |
| `Motion channel stopped (watch fired ...)` | **fault** — a watched axis went servo-off, alarmed, offline, or hit a limit |

---

## 8. Shutdown

Teleop → MoveIt launch → manipulator launch. Ctrl-C on the manipulator launch
lets `servo_stream_controller` run `endStream`, stopping the axes and freeing the
engine-side buffer channels. A `kill -9` leaves the channels allocated; the next
run reclaims them with `CreateApiBuffer failed ... reclaiming`.

```bash
ros2 service call /wmx/axes/set_servo_on wmx_r2_message/srv/SetAxes \
  "{axis: [0,1,2,3,4,5], data: [0,0,0,0,0,0]}"
```

---

## Troubleshooting

Differences from the single-axis case; the rest of the table in
[launch_single_servo_streamer.md](launch_single_servo_streamer.md#troubleshooting)
still applies.

| symptom | cause | fix |
|---|---|---|
| `Dropped trajectory: no command for axis N` | The publisher does not cover all six axes | With the test source, name all six joints. With Servo, check that `joint_name` in the config matches the URDF's joint names |
| `Watch enabled on 1 axes` | The controller loaded a single-axis config | You are running the bench params file instead of `cr3a_manipulator_config.yaml` |
| Both `joint_position_controller` and `servo_stream_controller` in `ros2 node list` | A manual node was started alongside the launch | Kill one — they double-command the arm |
| Servo runs, controller silent, no `Stream started` | Setpoints are not reaching the topic | `ros2 topic hz /movensys_manipulator_arm_controller/joint_trajectory`; expect ~40 Hz while jogging |
| Teleop dead, stream up, no warnings | `execution_active` latched true | Check for an in-progress MoveIt execution |
| `keyboard_teleop` not found | Built only where `ServoCommandType` exists (Jazzy+) | Check the configure-time `STATUS` message in the build log |
