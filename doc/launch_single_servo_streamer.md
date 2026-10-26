# Launch and Test the Single-Axis Servo Streamer

Step-by-step procedure for bringing up `servo_stream_controller` on **one axis**
and verifying it, without MoveIt, IK, or a teleop device in the loop.

For what the node is and what every parameter means, see
[reference_servo_stream_controller.md](reference_servo_stream_controller.md).

**What this exercises**

- The full streaming path: command source → ROS ring → pump → API buffer → engine
- The buffer draining to depth 0–1 with no pacing block, and the engine
  consuming setpoints as fast as they land
- The starvation path (controlled stop when commands stop arriving)
- Clean restart after a starvation teardown

**Processes involved** — four, plus monitors. Nothing streams until all four are
up, and each stage fails by *waiting quietly* rather than erroring, so bring them
up in order and check each one.

| # | process | root? | provides |
|---|---|---|---|
| 1 | `wmx_r2_general_nodes.launch.py` | yes | engine, core motion, io, EtherCAT |
| 2 | `joint_state_broadcaster` | yes | `/joint_states` |
| 3 | `servo_stream_controller` | yes | the node under test |
| 4 | `servo_stream_test_source` | no | synthetic Servo command stream |

---

## Safety, before anything else

- **E-stop within reach.** Step 3 energises the axis on its own, before you
  command any motion.
- **Clear travel around the test joint**, with margin: the streaming path
  currently delivers motion in surges reaching several times the commanded
  velocity (see Known issues in the reference doc).
- **Do not run a manipulator launch or `joint_position_controller` at the same
  time.** They subscribe to the same topic as `servo_stream_controller` and would
  double-command the arm.
- Only the axis in `joint_axes` is driven. Everything else stays put.

---

## 0. Environment

Every terminal needs the workspace sourced and both variables set. Put them in
`~/.bashrc` so new terminals inherit them:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspaces/movensys_ws/install/setup.bash

export ROBOT=cr3a     # or cr5a — must match the actual arm
export BENCH=$HOME/workspaces/movensys_ws/src/wmx-r2/wmx_r2_package/config/bench_servo_stream_config.yaml
```

**Verify both resolve, in every new terminal:**

```bash
echo "$ROBOT"
ls -l "$BENCH"
ls -l "$HOME/workspaces/movensys_ws/install/wmx_r2_package/share/wmx_r2_package/config/${ROBOT}_wmx_parameters.xml"
```

A `BENCH` left over from a shell rc file or an earlier session silently wins over
one you export by hand, and the resulting failure is
`Couldn't parse params file ... Error opening YAML file` — which does not
distinguish a wrong path from malformed YAML. An unset `ROBOT` collapses the
parameter path to `_wmx_parameters.xml` and fails the load. Both are cheap to
check and expensive to debug.

Build if you have not already:

```bash
cd ~/workspaces/movensys_ws
colcon build --packages-select wmx_r2_package
source install/setup.bash
```

Root access uses the same `sudo --preserve-env` invocation as the other launch
docs. If you do not have the wrapper yet:

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

## 1. Engine and fieldbus — terminal 1

```bash
~/wmxrun ros2 launch wmx_r2_package wmx_r2_general_nodes.launch.py
```

Verify:

```bash
ros2 service call /wmx/engine/get_engine_status std_srvs/srv/Trigger "{}"
# expect: success=True, message='Communicating'
ros2 node list
# expect: wmx_engine_node, wmx_lifecycle_manager_node, wmx_core_motion_node, wmx_io_node, wmx_ethercat_node
```

This launch does **not** start `joint_state_broadcaster` — the manipulator
launches add that, so a bench session runs it by hand in step 3.

---

## 2. Axis bring-up

Plain service calls, no root needed.

```bash
ros2 service call /wmx/engine/load_wmx_params wmx_r2_message/srv/LoadWmxParams \
  "{file_path: '$HOME/workspaces/movensys_ws/install/wmx_r2_package/share/wmx_r2_package/config/${ROBOT}_wmx_parameters.xml'}"
```

**This must return `success=True` before you go on.** On failure the engine keeps
whatever parameters it already had — possibly defaults with the wrong gear ratio
and wrong soft limits, in which case a commanded 0.02 rad can scale into a much
larger physical move. The three calls below will happily succeed on an axis whose
parameters never loaded, so a `success=True` from them is not evidence of a good
state.

Both robots' parameter files exist, so loading the **wrong** one will not error.
Confirm `ROBOT` matches the arm in front of you.

```bash
ros2 service call /wmx/axes/clear_amp_alarm wmx_r2_message/srv/SetAxes "{indices: [0], data: [0]}"
ros2 service call /wmx/axes/set_servo_on    wmx_r2_message/srv/SetAxes "{indices: [0], data: [1]}"
```

If you reload parameters later, **re-run both** — they act on the axis as scaled
at the time they were called.

### Homing is not part of this procedure ⚠

`/wmx/axes/start_home` sets `HomeType::CurrentPos`, which **redefines the axis'
current physical position as zero**. On an arm with absolute encoders
(`abs encoder: 1` in the parameter-load log) the position is already known at
power-on, and re-zeroing at an arbitrary pose discards the calibration that the
URDF and MoveIt kinematics depend on.

The streaming path does not need it. `servo_stream_controller` seeds each segment
from `posCmd`, which the engine initialises from the absolute encoder, so it
already agrees with the `actualPos` that `servo_stream_test_source` centres its
waveform on.

Home only deliberately, with the axis at its known zero pose, as a
recalibration.

---

## 3. Joint feedback — terminal 2 ⚠ energises the axis

```bash
~/wmxrun ros2 run wmx_r2_package joint_state_broadcaster --ros-args --params-file $BENCH
```

This is a lifecycle node: `wmx_lifecycle_manager_node` (part of step 1)
configures and activates it once the engine reports `Communicating`. On
activation it calls `wmx/axes/clear_amp_alarm` and `wmx/axes/set_servo_on` for
its own `joint_axes`, so **starting it turns the servos on**. If those calls
fail it logs `Activation failed at ...` and the manager retries on its next
discovery sweep. Start it only after step 1 is confirmed.

Verify — the test source waits forever otherwise:

```bash
ros2 topic hz /joint_states            # expect ~100 Hz
ros2 topic echo --once /joint_states   # `name` must contain joint1
```

---

## 4. The controller — terminal 3

```bash
~/wmxrun ros2 run wmx_r2_package servo_stream_controller --ros-args --params-file $BENCH
```

Expect the parameter echo, then:

```
Attached WMX3 device 'servo_stream_rec'
Attached WMX3 device 'servo_stream_ctl'
Recorded e-stop routine into channel 1
Watch enabled on 1 axes, trigger routine on channel 1
axis 0 motion params: calcMode=AxisLimit, overrideType=FastBlending, apiWaitUntilMotionStart=false
servo_stream_controller is ready
```

**Read the `motion params` line before going on.** Neither value is set by this
node — both are inherited from the axis parameter file loaded in step 2 — and
both decide whether the streaming design holds:

| field | wanted | if it differs |
|---|---|---|
| `overrideType` | `FastBlending` (or `Blending`) | `Smoothing` means consecutive setpoints do not override, and the motion will be stop-and-go no matter what this node does. The node warns. |
| `calcMode` | `AxisLimit` | Anything else means the per-axis velocity and acceleration limits this node computes may not govern the profile at all. The node warns. |
| `apiWaitUntilMotionStart` | either | `true` costs a full communication cycle per block, which matters only if depth stops reading 0–1. |

**It stops there, and that is correct.** The stream starts on the first setpoint,
so `Stream started` does not appear until step 6. If it stays at
`waiting for engine...` instead, step 1 is not up.

---

## 5. Depth monitor — terminal 4

```bash
ros2 topic echo /servo_stream_controller/queue_depth
```

Reads `0` until the stream starts. This topic goes to 0 whenever the stream is
down, so it doubles as a liveness signal.

---

## 6. Hold test — terminal 5

The bench config ships `amplitude_rad: 0.0`, which runs the whole path at 40 Hz
with **zero commanded motion**. This is the correct first test with servos on.

```bash
ros2 run wmx_r2_package servo_stream_test_source --ros-args --params-file $BENCH
```

| check | expected |
|---|---|
| test source | `Centred on [...]; streaming.` |
| controller | `Stream started (channel 0 Active)` — once, with no fault and no `Stream ended` |
| `queue_depth` | settles at **2–3** and stays there |
| the axis | does not move |

A flicker between 0 and 1 is normal: depth is `remainingBlockCount`, so a block
in flight at sampling time flips the reported integer.

**Do not proceed until depth reads 0 with no fault here.** On a pure hold, every
setpoint should fall under `min_step_rad` once tracking converges, so no blocks
are recorded at all. The thing being confirmed is that the channel stays Active
and error-free while idle — not that a queue has formed, because there is no
queue to form.

---

## 7. Motion test

Raise amplitude with an inline override rather than by editing the file, so the
safe default survives to next time.

```bash
ros2 run wmx_r2_package servo_stream_test_source --ros-args \
  --params-file $BENCH -p amplitude_rad:=0.02
```

Amplitude ramps in over `ramp_seconds` (2 s), so there is no step at start. Step
up gradually — 0.02, then 0.05 — watching depth and the arm at each level. The
node hard-caps amplitude at 0.5 rad.

**Expected:**

- `queue_depth` flickering between **0 and 1** and never standing higher
- **No** repeating warnings of any kind
- The axis moving **smoothly** — this is the point of the test, and it is best
  judged by hand and ear before it is judged by a bag file. Under the old
  `USleep` design the axis advanced in visible 40 Hz steps with ~24 velocity
  reversals a second; it should now travel continuously.

**Real failures:**

| symptom | meaning |
|---|---|
| `API buffer not draining: N blocks in flight` | The engine is not consuming as fast as setpoints arrive. Check `nominal_period_us` against the source's real rate, and `apiWaitUntilMotionStart` from step 4 |
| `Streaming but no blocks consumed in N ms` | Channel Active but the engine has stalled mid-interpolation |
| `Pump behind: skipped N setpoint(s)` | Occasional is fine on a loaded host; sustained means the pump thread is starved |
| `Setpoint needs X rad/s, over max_joint_velocity` | Either the arm is catching up from behind, or `max_joint_velocity` is too low for this motion |
| Motion still visibly stepping | The change did not achieve its purpose — check `overrideType` from step 4 first, then see the fallback in the reference doc |
| Any `Motion channel stopped (...)` | A watch trip or a rejected block — read the cause in the message |

**Watch for lag specifically.** The `posCmd`-referenced segment origin is what
should keep the arm from falling progressively behind the waveform. A steady
phase offset is expected; an offset that *grows* over 60 s is not, and would mean
the origin change is not doing its job.

---

## 8. Starvation test

Ctrl-C the test source.

| check | expected |
|---|---|
| controller | `Servo stream starved (>75 ms); stopping axes` |
| controller | `Stream ended: stream starved (axes stopped)` |
| `queue_depth` | → **0** within ~100 ms, and holds 0 |

The buffer is `Halt`ed and `Clear`ed rather than played out, so depth drops
straight to 0 with no ramp-down. A gradual decay would mean the queue was
draining on its own, which is not what a controlled stop does.

---

## 9. Restart test

Start the test source again, at `amplitude_rad: 0.0`.

| check | expected |
|---|---|
| controller | a single `Stream started (channel 0 Active)` |
| must **not** appear | `Motion channel stopped`, `holding off`, `Stream ended` |

Repeat two or three times. The leftover `Halt` from step 8 is exactly the
condition that used to make the first status poll after `Execute` read back a
stale `Stop` and tear down the stream it had just started — and because it was a
race, one clean restart is not proof.

---

## 10. Measurement run (optional)

```bash
ros2 bag record -o bench_$(date +%H%M) \
  /joint_states \
  /movensys_manipulator_arm_controller/joint_trajectory \
  /servo_stream_controller/queue_depth
```

Record 60 s or more at a fixed amplitude. This is the run that decides whether
the change worked, since §7 only establishes that nothing is obviously broken.

Compare against the pre-change baseline in `stage5_buffered/`, which was taken at
the same 0.25 Hz, ±50 mrad, 40 Hz:

| measurement | before (`USleep`) | expected now |
|---|---|---|
| Transport latency (commanded vs measured cross-correlation) | ~290 ms | **~25–50 ms** |
| Peak velocity vs the per-segment `maxVelocity` the node computes | ~8× | **~1×** |
| Velocity sign changes per second during smooth motion | ~24 | **~0** |
| Position gain / rms residual | 1.015 / 3.1 mrad | no worse |

The velocity columns are the primary criterion — latency falling is expected and
easy, but the surging was the defect the pacing block was actually causing.
Differentiate `/joint_states` position to get velocity; do not rely on a velocity
field unless you have confirmed the broadcaster populates it.

---

## 11. Shutdown

In order: test source → controller → broadcaster → engine launch.

Ctrl-C on the controller runs `endStream`, which stops the axes and frees the
engine-side buffer channels. A `kill -9` leaves the channel allocated instead;
the next run detects and reclaims it
(`CreateApiBuffer failed ... reclaiming`), so it is recoverable, just noisy.

Servo off when you are done:

```bash
ros2 service call /wmx/axes/set_servo_on wmx_r2_message/srv/SetAxes "{indices: [0], data: [0]}"
```

---

## Pass criteria

| test | pass |
|---|---|
| Hold (§6) | Depth reads 0; axis still; no warnings and no fault |
| Motion (§7) | Axis tracks the waveform; depth stays 0–1; no faults, no dropped setpoints. Motion should be **smooth** — the surging measured under the old `USleep` design is what this change targets |
| Starvation (§8) | Starve warning, controlled stop, depth → 0 and stays |
| Restart (§9) | Single clean `Stream started`, no fault, repeatable |

---

## Troubleshooting

| symptom | cause | fix |
|---|---|---|
| `Couldn't parse params file ... Error opening YAML file` | `$BENCH` unset, stale, or pointing at a deleted file | `ls -l "$BENCH"`; the message does not distinguish a missing file from bad YAML |
| `LoadWmxParams` → `success=False, 'File operation failed'` | `$ROBOT` unset, so the path collapsed to `_wmx_parameters.xml` | `echo "$ROBOT"`, then re-run |
| Controller stuck at `waiting for engine...` | `wmx/engine/get_engine_status` never answered `Communicating` | Step 1 is not up, or the engine is not communicating |
| Test source repeats `Waiting for 'joint1' on /joint_states ...` | No publisher, or a joint-name mismatch | `ros2 topic info /joint_states` — `Publisher count: 0` means the broadcaster is not running; nonzero means compare its `joint_name` with the test source's |
| Broadcaster logs `Activation failed at clear_amp_alarm` / `at set_servo_on` | The core motion services did not answer | The lifecycle manager retries on its next sweep; check `wmx_core_motion_node` is active |
| Broadcaster crashes immediately | `joint_feedback_rate` is 0 (its default), giving `1000 / 0` in the timer period | Set it; the bench config uses 100 |
| Controller runs but nothing moves | `/moveit2_trajectory/execution_active` is latched true, so setpoints are discarded | Check that no `move_group` execution is in progress |
