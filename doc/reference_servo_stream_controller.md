# servo_stream_controller Reference

Buffered path for MoveIt Servo commands: setpoints are recorded into a WMX3 API
buffer and replayed by the engine under the real-time OS, instead of being
issued from a ROS callback.

- Package: `wmx_r2_package`
- Executable: `servo_stream_controller`
- Source: [`src/servo_stream_controller.cpp`](../wmx_r2_package/src/servo_stream_controller.cpp)
- Config: `servo_stream_controller:` block in `config/cr3a_manipulator_config.yaml` / `config/cr5a_manipulator_config.yaml`

---

## What it is

`joint_position_controller` (the default) calls `StartLinearIntplPos` directly
from the ROS subscription callback. Every command crosses from a non-real-time
Linux thread straight into the engine, so ROS scheduling jitter lands on motion
timing and a late thread leaves a gap in the stream.

`servo_stream_controller` puts the API buffer in between:

```
Servo topic -> ROS ring (bounded, drop-oldest) -> pump thread -> API buffer -> engine
```

The buffer absorbs ROS jitter, and its watch function adds a safety net that
lives entirely on the RT side.

**The two nodes subscribe to the same topic and must not run together.** The
launch files enforce this with the `use_api_buffer` argument.

### How a setpoint becomes motion

Each Servo setpoint is recorded as **one block**:

| block | purpose |
|---|---|
| `StartLinearIntplPos(n axes)` | coordinated move, lands as a `FastBlending` override |

There is no pacing block. The WMX3 manual states that buffered API functions
"will be executed as fast as possible" unless a `Wait` or `Sleep` is recorded
between them, and that motion APIs execute one per communication cycle. So the
channel drains at ~1 ms per setpoint against a 25 ms arrival interval, and
`remainingBlockCount` sits at **0–1** in steady state.

**The API buffer is a real-time mailbox here, not a queue.** It still moves the
hand-off onto the RT side and still carries the watch trigger, but it no longer
stores latency. Pacing comes from the ROS arrival rate.

Cost is 320 bytes per setpoint, so a 1 MB channel holds ~3200. The channel is a
ring and reclaims space as blocks execute, so buffer size does not bound how long
a stream can run.

### Why there is no pacing block

The earlier design recorded `USleep(T)` after each interpolation, which held the
channel for the whole period. Every segment therefore ran to its target,
decelerated to rest under `endVelocity = 0`, and sat there until the sleep
expired — `FastBlending` never engaged. That is what produced the surging
measured in the single-axis runs (~8× peak velocity, ~24 velocity sign changes
per second), and the pacing loop could only cover the per-block setup cost by
holding a standing depth error of ~7 setpoints, costing 225–250 ms of latency
instead of the intended 50 ms.

Dropping the sleep lets each command override its predecessor mid-flight, which
is what `FastBlending` is for.

Ride-through no longer comes from queue depth. Each block carries a full period
of programmed travel, so a late setpoint continues the previous segment rather
than leaving a gap; and if the publisher dies outright, the last interpolation
decelerates to rest on its own within one period, before `starvation_timeout_ms`
even fires.

### Segment distance is measured from `posCmd`

`nominal_period_us` is now the time each segment is given to cover its distance,
which is what sets commanded velocity:

```
velocity_i = |target_i - posCmd_i| / nominal_period_us
```

The origin is the **current commanded position**, not the previous setpoint —
the reverse of the buffered design, and it follows from the queue being gone.
With commands overriding mid-flight, `posCmd` never reaches the previous target,
so a setpoint-to-setpoint delta understates the distance still to cover and the
lag grows without bound with nothing in the loop noticing. Measuring from
`posCmd` is self-correcting: the further behind the axis is, the higher the
velocity it asks for. Depth 0–1 is what makes `posCmd` trustworthy again — it is
one block plus one cycle stale, ~1–2 ms.

Two guards sit on top of that:

- **`max_joint_velocity`** is applied as a *uniform scale* across all axes, not
  a per-axis clamp. Clamping one axis while the others run free would bend the
  coordinated path; slowing the whole move keeps it straight and only costs time.
- **`min_step_rad`** — below this the setpoint is a hold and no block is
  recorded at all. Recording it would ask for `maxVelocity = 0` on every axis,
  which the engine rejects, and with `stop_on_error` that stops the channel — so
  holding a pose would read as a fault. Servo publishes the held position
  whenever its input is centred, so this is a normal case, not an edge one.

### The pump takes the newest setpoint

The ROS ring is drained on every pump pass and only the **newest** entry is fed
in; the rest are dropped with a throttled warning. Feeding a backlog would
execute every entry within a few cycles, each overriding the last, leaving the
survivor to cover the whole backlog's distance in one period — a velocity spike
proportional to the backlog, exactly when the system is already behind.

### Safety and hand-off behaviour

| situation | behaviour |
|---|---|
| Watched axis goes servo-off, offline, amp-alarm, or hits a limit | The engine halts the motion channel and runs a pre-recorded `Stop` on every joint axis from `estop_buffer_channel`. No dependency on ROS or Linux still being alive. |
| No new setpoint for `starvation_timeout_ms` | Controlled stop, then `Halt` + `Clear`. The queue is discarded, not played out. |
| `move_group` execution starts (`/moveit2_trajectory/execution_active` true) | Ring cleared, stream ended without a `Stop` — the trajectory controller owns the axes from that point. Setpoints are discarded until execution finishes. |
| Motion channel reports `Stop` with a recorded error | Fault backoff, doubling from 500 ms to a 5 s cap, then retry. |
| A previous run died without freeing its channel | The channel is reclaimed on startup rather than failing with "Queue ID is already used". |

---

## Prerequisites

The engine must be up and the axes ready before this node can stream. Follow the
startup sequence in
[reference_wmx_r2_general_nodes.md](reference_wmx_r2_general_nodes.md): load axis
parameters, set gear ratios, clear alarms, servo on, home.

The node polls `wmx/engine/get_engine_status` once a second and initialises
when the engine reports `Communicating`, so it can be started first — it will
sit at `waiting for engine...` until the engine node comes up. It is a plain
node, not a lifecycle node, so `wmx_lifecycle_manager_node` does not manage it.

Engine access needs root. All commands below assume this wrapper, which is the
same `sudo --preserve-env` invocation used throughout `doc/`:

```bash
cat > ~/wmxrun <<'EOF'
#!/bin/bash
sudo --preserve-env=PATH --preserve-env=AMENT_PREFIX_PATH \
     --preserve-env=COLCON_PREFIX_PATH --preserve-env=PYTHONPATH \
     --preserve-env=LD_LIBRARY_PATH --preserve-env=ROS_DISTRO \
     --preserve-env=ROS_VERSION --preserve-env=ROS_PYTHON_VERSION \
     --preserve-env=ROS_DOMAIN_ID --preserve-env=RMW_IMPLEMENTATION \
  bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source $HOME/workspaces/movensys_ws/install/setup.bash && $*"
EOF
chmod +x ~/wmxrun
```

---

## How to run

### 1. With the manipulator launch (normal use)

Pass `use_api_buffer:=true` to swap `joint_position_controller` for this node:

```bash
~/wmxrun ros2 launch wmx_r2_package wmx_r2_cr5a_manipulator.launch.py \
  use_sim_time:=false use_api_buffer:=true
```

Use `wmx_r2_cr3a_manipulator.launch.py` for the CR3A. Parameters come from that
robot's `*_manipulator_config.yaml`.

### 2. Standalone (bench work and tuning)

Run the node by itself against an already-running engine, using the single-axis
bench config that ships with the package:

```
wmx_r2_package/config/bench_servo_stream_config.yaml
```

It carries a section for each node a bench session uses —
`servo_stream_controller`, `joint_state_broadcaster`, `servo_stream_test_source`,
and `api_buffer_probe` — so the same file drives all of them. Each top-level key
must match a node's name, and each node reads only its own section.

Point `ros2 run` at the **source tree** copy so it can be edited and re-run
without a rebuild:

```bash
export BENCH=$HOME/workspaces/movensys_ws/src/wmx-r2/wmx_r2_package/config/bench_servo_stream_config.yaml
ls -l "$BENCH"        # confirm it resolves — see below
```

Check that `ls` every time you open a new terminal. If `BENCH` is already
exported from a shell rc file or an earlier session, your value silently wins
over the one you just typed, and `rcl` then reports
`Error opening YAML file` — a message that does not tell you the path was wrong
rather than the YAML.

The installed copy also exists after a build, at
`install/wmx_r2_package/share/wmx_r2_package/config/bench_servo_stream_config.yaml`,
but editing that one is overwritten by the next `colcon build`.

This is a bring-up and measurement config, not a robot config — the launch files
use `cr3a_manipulator_config.yaml` / `cr5a_manipulator_config.yaml`. Keep tuning
experiments in the bench file and promote settled values into the robot config.

#### Bench session bring-up

> For the full single-axis test procedure — bring-up, hold test, motion test,
> starvation and restart checks, pass criteria — see
> [launch_single_servo_streamer.md](launch_single_servo_streamer.md). The summary
> below is the bring-up order only.

A bench session needs four processes, in this order. Nothing streams until all of
them are up, and the failure at each stage is a quiet wait rather than an error.

**1. Engine and fieldbus** (one terminal):

```bash
~/wmxrun ros2 launch wmx_r2_package wmx_r2_general_nodes.launch.py
```

This starts `wmx_engine_node`, `wmx_core_motion_node`, `wmx_io_node`, and
`wmx_ethercat_node`. It does **not** start `joint_state_broadcaster` — the
manipulator launches add that separately, so a bench session has to run it by
hand.

**2. Axis bring-up.** Load axis parameters, set gear ratios, clear alarms, servo
on, home — the sequence in
[reference_wmx_r2_general_nodes.md](reference_wmx_r2_general_nodes.md). Nothing
below will move an axis until this is done.

**3. Joint feedback** (second terminal). Needs root, since it reads axis status
straight from the engine:

```bash
~/wmxrun ros2 run wmx_r2_package joint_state_broadcaster --ros-args \
  --params-file $BENCH
```

Confirm it before going further — the test source will wait forever otherwise:

```bash
ros2 topic hz /joint_states          # expect ~100 Hz
ros2 topic echo --once /joint_states # expect joint1 in `name`
```

**4. The controller** (third terminal):

```bash
~/wmxrun ros2 run wmx_r2_package servo_stream_controller --ros-args \
  --params-file $BENCH
```

Wait for `Stream started (channel 0 Active)`. If it stays at
`waiting for engine...`, step 1 is not up.

The test source (§3 below) is the fourth process.

If the params file is missing or unreadable, `rcl` fails before the node starts
with `Couldn't parse params file ... Error opening YAML file` — check the path
first, since that message does not distinguish "absent" from "malformed".

Single overrides work too:

```bash
~/wmxrun ros2 run wmx_r2_package servo_stream_controller --ros-args \
  --params-file $BENCH -p nominal_period_us:=20000
```

The node echoes every parameter at startup — check that block first when
behaviour does not match expectations.

### 3. Driving it without MoveIt: `servo_stream_test_source`

`servo_stream_test_source` is a synthetic stand-in for MoveIt Servo. It publishes
`trajectory_msgs/JointTrajectory` on Servo's output topic at Servo's rate, so the
streaming path can be characterised without IK, collision checking, smoothing, or
an operator's hand in the loop. The input is a known analytic function, so
commanded-vs-actual measures the streaming path alone.

It needs no root, but it **does** need a live `/joint_states` carrying the joints
it is asked to publish — it centres the waveform on their current positions and
will not start streaming until it sees them. Bring up
`joint_state_broadcaster` first (bench bring-up step 3 above).

```bash
ros2 run wmx_r2_package servo_stream_test_source --ros-args \
  --params-file $BENCH
```

The bench config sets `amplitude_rad: 0.0`. Raise it with an inline override
rather than by editing the file, so the safe default is what you get next time:

```bash
ros2 run wmx_r2_package servo_stream_test_source --ros-args \
  --params-file $BENCH -p amplitude_rad:=0.02
```

| parameter | default | notes |
|---|---|---|
| `joint_name` | `["joint1"]` | Publishes only these joints |
| `joint_states_topic` | `/joint_states` | Used to centre the waveform |
| `output_topic` | `/movensys_manipulator_arm_controller/joint_trajectory` | Must match the controller's `joint_trajectory_topic` |
| `amplitude_rad` | `0.0` | Hard-capped at 0.5 rad so a typo cannot command a large excursion |
| `frequency_hz` | `0.25` | |
| `publish_rate_hz` | `40.0` | Servo's rate |
| `ramp_seconds` | `2.0` | Amplitude ramps in from zero, so starting never steps |
| `waveform` | `sine` | `sine` or `triangle` |

The waveform is centred on the joint's **current** position read from
`/joint_states`, so starting the node never produces a step command.

**Recommended bring-up order** on any new robot or config:

1. `amplitude_rad: 0.0` — a pure hold. The full streaming path runs at rate with
   zero commanded motion. This is the correct first test with the servos on,
   before commanding any movement at all. Every setpoint should fall under
   `min_step_rad` once tracking converges, so no blocks are recorded and
   `queue_depth` reads 0 — with **no** fault and no `Stream ended`.
2. Small amplitude, e.g. `amplitude_rad: 0.02`. Watch tracking and confirm
   `queue_depth` stays at 0–1.
3. Increase only once tracking is clean and no warnings are firing.

Ctrl-C on the test source stops the stream, which is the intended way to exercise
the starvation path.

### 4. `api_buffer_probe` — characterise the buffer first

```bash
~/wmxrun ros2 run wmx_r2_package api_buffer_probe --ros-args \
  --params-file $BENCH
```

Run-once node: attaches, measures, prints a report, exits. It answers bytes per
block, real `USleep` resolution, and whether the block counters can stand in for
the `GetCumulativeBlockCount` this SDK does not provide.

It is a **different node** with its own parameters — `joint_axes`,
`api_buffer_channel`, `api_buffer_size_mb`, `sample_count`,
`sleep_sample_count` — which is why the bench config carries an
`api_buffer_probe:` section of its own. Without one, the probe would start
happily and silently fall back to its defaults of all six axes and 5 MB, giving a
report for a configuration you did not ask for.

**It commands no motion** — every recorded target is the axis' current commanded
position, and the motion blocks are cleared before anything is executed. Safe to
run with the servos off, and that is the recommended way.

---

## Parameters

| parameter | default | meaning |
|---|---|---|
| `joint_axes` | `[]` | WMX3 axis indices. **Order matters**: `axis[0]` determines `LinearIntplProfileCalcMode` for the whole interpolation. |
| `joint_name` | `[]` | ROS joint names, positionally paired with `joint_axes` |
| `joint_trajectory_topic` | — | Servo's `command_out_topic` |
| `api_buffer_channel` | `0` | Motion channel |
| `estop_buffer_channel` | `1` | Watch trigger routine; must differ from the motion channel, or the trigger is disabled |
| `api_buffer_size_mb` | `5` | 1 MB ≈ 2650 setpoints ≈ 66 s of stream |
| `nominal_period_us` | `25000` | Time each segment is given to cover its distance — this is what sets commanded velocity. Set it to Servo's actual publish period |
| `ros_queue_depth` | `8` | Ring size, bounding memory only; the pump takes the newest entry and drops the rest |
| `starvation_timeout_ms` | `75` | Gap without a setpoint before a controlled stop |
| `accel_ratio` | `0.3` | Accel phase spans ~30% of each segment |
| `max_joint_velocity` | `3.0` | rad/s ceiling, applied as a uniform scale across axes. **Verify against the arm's real joint limits** |
| `min_step_rad` | `1e-6` | Below this a setpoint is a hold and no block is recorded |
| `stop_on_error` | `true` | Stop rather than skip a rejected setpoint |

`nominal_period_us` is the single most important value here. Too short
over-drives: the axis reaches target early, decelerates to rest, and the
stop-and-go this design exists to remove comes back. Too long under-drives and
the arm lags the stream.

---

## Topics

| topic | type | direction | notes |
|---|---|---|---|
| `<joint_trajectory_topic>` | `trajectory_msgs/JointTrajectory` | in | Servo setpoints; only the last point of each message is used |
| `wmx/engine/get_engine_status` | `std_srvs/Trigger` (service client) | out | Polled at 1 Hz until the engine answers `Communicating`; gates initialisation |
| `/moveit2_trajectory/execution_active` | `std_msgs/Bool` | in | Transient-local; blocks streaming while `move_group` owns the arm |
| `/servo_stream_controller/queue_depth` | `std_msgs/Int32` | out | Setpoints in flight, published at 10 Hz |

### Joint matching

- **Named messages**: matched by joint name, so a subset config
  (`joint_axes: [0]`) can consume Servo's full six-joint output, and the
  publisher's ordering does not have to match ours.
- **Unnamed messages**: positional, assumed already in `joint_axes` order.
- A setpoint that does not command **every** axis this node drives is dropped —
  a partial setpoint would hold some joints and move others, bending the path
  Servo computed.

---

## Checking that it is working

```bash
ros2 topic echo /servo_stream_controller/queue_depth
ros2 topic hz /joint_states
```

A healthy stream:

```
[INFO] Attached WMX3 device 'servo_stream_rec'
[INFO] Attached WMX3 device 'servo_stream_ctl'
[INFO] Recorded e-stop routine into channel 1
[INFO] Watch enabled on 1 axes, trigger routine on channel 1
[INFO] servo_stream_controller is ready
[INFO] Stream started (channel 0 Active)
```

- One `Stream started`, with no fault and no `Stream ended` following it.
- `queue_depth` reading **0–1**. That is the healthy value now, not a fault:
  with no pacing block the engine consumes each setpoint as soon as it lands.
  Anything standing above `3` trips `API buffer not draining`.
- The `axis N motion params:` line reporting `overrideType=FastBlending` and
  `calcMode=AxisLimit`. Both are inherited from the axis configuration rather
  than set by this node, and the node warns if either is something else.
- No repeating warnings.

**`queue_depth` is no longer a liveness signal.** It reads 0 both when the
stream is healthy and when it is down, so it cannot distinguish them. What
proves the engine is still consuming is `cumulativeBlockCount` advancing, which
the node checks itself — three consecutive status polls (300 ms) with nothing
consumed while streaming produces `Streaming but no blocks consumed`.

### Recording a run for analysis

Two scripts in `wmx_r2_package/scripts/` do the capture and the plot. Use them in
preference to a bag when the question is "how well is it tracking" — the bag is
for keeping a run, these are for measuring one.

```bash
# capture: setpoint, pos_cmd and actual_pos on one clock
python3 src/wmx-r2/wmx_r2_package/scripts/capture_stream_trace.py \
  --seconds 30 --out trace.csv

# plot one joint + print the metric table for all six
python3 src/wmx-r2/wmx_r2_package/scripts/plot_stream_trace.py trace.csv --joint 1
python3 src/wmx-r2/wmx_r2_package/scripts/plot_stream_trace.py trace.csv --all

# the whole capture instead of a single cycle
python3 src/wmx-r2/wmx_r2_package/scripts/plot_stream_trace.py trace.csv --cycles 0
```

The plot shows **one cycle** of the move by default, taken from the middle of
the capture so the start-up transient is not in frame. Across a 30 s run the
three traces overlap into a single band and the lag between them is a line
width; across one period it is visible. `--cycles N` widens the window, and the
metric table is computed over whatever window is plotted, so the numbers and the
picture always agree.

`plot_stream_trace.py` needs `matplotlib` and `numpy`; the capture script needs
neither. Both take `--help`.

**The three traces are the point.** Capturing only the setpoint and the encoder
cannot tell you where a tracking error came from; adding WMX3's own interpolator
output splits it:

| split | what it measures | if the error lives here |
|---|---|---|
| `setpoint` → `pos_cmd` | the API buffer and interpolation path | this node's problem |
| `pos_cmd` → `actual_pos` | the servo loop underneath it | tuning; no amount of work on the streaming path will move it |

That split is what known issue 3 could not previously resolve.

A bag still makes sense for keeping a run around:

```bash
ros2 bag record -o stage_test \
  /joint_states \
  /movensys_manipulator_arm_controller/joint_trajectory \
  /servo_stream_controller/queue_depth
```

`stage5_buffered/` in the repo root is an example recording, taken under the
earlier `USleep`-paced design.

---

## Troubleshooting

| message | meaning | what to do |
|---|---|---|
| `waiting for engine...` and nothing more | `wmx/engine/get_engine_status` never answered `Communicating` | Start the general nodes (`wmx_r2_general_nodes.launch.py`); check the engine and EtherCAT state |
| `servo_stream_test_source`: `Waiting for 'joint1' on /joint_states ...` forever | Either nothing publishes `/joint_states`, or it does but without that joint name | `ros2 topic info /joint_states` — `Publisher count: 0` means `joint_state_broadcaster` is not running; a nonzero count means the name does not match, so compare its `joint_name` with the test source's |
| `API buffer not draining: N blocks in flight (expected 0-1)` | The engine is not consuming setpoints as fast as they arrive | Check the publisher's rate against `nominal_period_us`; check `apiWaitUntilMotionStart` in the startup log |
| `Streaming but no blocks consumed in N ms` | Channel is Active but `cumulativeBlockCount` is not advancing | The engine has stalled mid-interpolation — check axis state, servo-on, and alarms |
| `Pump behind: skipped N setpoint(s) to take the newest` | Setpoints arrived faster than the pump drained them | Occasional entries are normal on a loaded host; sustained ones mean the pump thread is being starved |
| `Setpoint needs X rad/s, over max_joint_velocity Y` | A step needed more speed than the ceiling allows | Either the arm was far behind and is catching up, or `max_joint_velocity` is set too low for this motion |
| `CoreMotion::GetStatus failed, cannot place segment origin` | Could not read `posCmd` to anchor the segment | Engine-side problem; the setpoint is dropped rather than commanded from a stale origin |
| `Servo stream starved (>75 ms); stopping axes` | No setpoint arrived within the timeout | Check the publisher's rate and the ROS transport; expected when you Ctrl-C the source |
| `ROS queue full (8); dropping oldest setpoint` | The pump cannot keep up with arrivals | Check `nominal_period_us` against the publisher's actual rate |
| `Dropped trajectory: no command for axis N` | The setpoint did not cover every driven axis | `joint_name` does not match the publisher's joint names |
| `Dropped trajectory: N positions for M axes` | Unnamed message shorter than `joint_axes` | Fix the publisher or name the joints |
| `Motion channel stopped (watch fired on axis A, code C)` | The RT watch tripped | Servo-off, amp alarm, limit, or drive offline on axis A |
| `Motion channel stopped (block N failed, code C)` | A recorded block was rejected at execution | Usually a limit or a velocity/accel violation on that segment |
| `Motion channel stopped (no error recorded, channel never left Stop after Execute)` | The channel did not start and nothing errored | Check that the axes are servo-on and homed |
| `CreateApiBuffer failed ... reclaiming` | A previous run left the channel allocated | Informational; the node reclaims it |
| `estop_buffer_channel must differ from api_buffer_channel` | Both set to the same channel | Fix the config — the watch trigger is disabled until you do |

---

## Status of the previously measured issues

The measurements below were taken on a **single axis** (`joint_axes: [0]`) with
`servo_stream_test_source` at 0.25 Hz, ±50 mrad, 40 Hz, against the earlier
`USleep`-paced design. Removing the pacing block is expected to address the first
three; **none of it has been re-measured on hardware yet.**

**1. Queue depth settled far above target during motion.** *(expected fixed)*
`StartLinearIntplPos` cost roughly 1.05 ms of engine time to set up real motion,
which the proportional pacing loop could only cover by holding a standing depth
error of about `1050 / 150 ≈ 7` setpoints — observed depth 9–10 instead of 2,
i.e. 225–250 ms of latency instead of 50 ms. There is no pacing loop and no
target depth any more, so there is no standing error to hold. Expect depth 0–1.

**2. The correction could exceed the loop's authority on six axes.**
*(no longer applicable)* There is no correction and no clamp authority to exceed.

**3. Motion was delivered in surges, not smoothly.** *(expected fixed — verify)*
Position tracking of the commanded fundamental was good (gain 1.015, ~290 ms of
pure delay, 3.1 mrad rms residual on a 50 mrad amplitude), but the joint moved in
bursts: peak velocity ~8× the per-segment `maxVelocity` the node computes, with
roughly 24 velocity sign changes per second. Stiction was ruled out (following
error showed no correlation with direction of travel).

The cause was the pacing block itself: `USleep(T)` held the channel for the full
period, so each segment ran to its target and decelerated to rest under
`endVelocity = 0` before the next one was issued. `FastBlending` never engaged.
With the sleep gone, each command overrides its predecessor mid-flight.

**This is the primary thing to verify on hardware.** Success looks like peak
velocity falling from ~8× toward ~1× `maxVelocity`, and velocity sign changes
falling from ~24/sec toward ~0 during smooth motion.

The node now logs `linearIntplProfileCalcMode` and `linearIntplOverrideType` for
`axis[0]` at startup and warns if they are not `AxisLimit` / a blending type —
both are inherited from the axis configuration, and issue 3 could not previously
be diagnosed because neither was recorded.

Caveat on that measurement: `wmx_core_motion_node` sizes `wmx/axes/status` from
the axis count it reads out of `GetEngineStatus` at attach time, and on a CR3A
launch it has been observed logging `is ready (1 axes, 100 Hz)` — so the topic
may carry only axis 0 even on a six-axis arm. Check that line before relying on
the topic for anything but axis 0.

**4. The CR5A config is untuned.**
Its `servo_stream_controller` values were copied from the CR3A. Re-tune before
relying on them, starting with `nominal_period_us` and `max_joint_velocity`.

---

## Known issues

**1. `max_joint_velocity` defaults are placeholders.**
`3.14` rad/s in the CR3A and CR5A configs is a plausible number, not one taken
from the arms' datasheets. Verify against the real per-joint limits before
relying on the ceiling to catch anything.

**2. Jitter absorption is now the motion's job, not the buffer's.**
With depth 0–1 the buffer no longer smooths ROS arrival jitter; what carries the
arm across a late setpoint is the previous segment's remaining programmed travel.
This should degrade gracefully, but it has not been measured under a loaded host.
If the arm proves jittery on hardware, the WMX3 manual's endorsed pattern for
this exact problem is `StartLinearIntplPos` → `ApiBuffer::Wait(DecelerationStarted)`
→ next interpolation: still two blocks per setpoint, but the second is a
condition-wait rather than a fixed sleep, so the buffer self-paces off the motion
instead of a clock and the queue survives. `ApiBufferConditionType` also offers
`RemainingTime`, `DistanceToTarget`, and `MotionStartedOverrideReady` as
finer-grained variants.

**3. Concurrency defects predating this change remain.**
`endStream` runs on both the executor thread (via `onExecActive`) and the pump
thread with no mutex, and `errString_` is a single shared scratch buffer written
from three threads. Neither is affected by the pacing change, and both are still
open. See the source review notes.
