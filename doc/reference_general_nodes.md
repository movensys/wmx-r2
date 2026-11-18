# WMX R2 General Nodes Reference

Every interface of the five nodes started by `wmx_r2_general_nodes.launch.py`:
`wmx_engine_node`, `wmx_lifecycle_manager_node`, `wmx_core_motion_node`,
`wmx_io_node`, `wmx_ethercat_node`.

How to launch them: [launch_general_nodes.md](launch_general_nodes.md).

Every command below runs through `wros`, which hands the whole command to the
container's shell as one string. Only quotes inside the argument survive that
second parse, so payloads are spelled `'"{...}"'`. From a shell already inside
the container, drop `wros` and use plain `"{...}"`.

## Launch arguments

`wmx_r2_general_nodes.launch.py`:

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use simulation clock |
| `config_file` | `""` | Absolute path to the YAML with the general node parameters, e.g. `$(ros2 pkg prefix --share wmx_r2_package)/config/wmx_r2_general_nodes_config.yaml`. Only `wmx_engine_node`, `wmx_lifecycle_manager_node` and `wmx_core_motion_node` are given this file; `wmx_io_node` and `wmx_ethercat_node` declare no parameters |
| `wmx_param_file` | `""` | Absolute path to the WMX3 parameter XML imported at engine start, e.g. `$(ros2 pkg prefix --share wmx_r2_package)/config/wmx_parameters.xml`. Overrides `wmx_param_file_path` from `config_file`; empty leaves that value in charge |

Nothing is baked into the launch file; the robot launches forward their own
`config_file` and `wmx_param_file` here. Wrap an argument holding `$(...)` in
single quotes so it expands inside the container, not on the host:

```bash
wros ros2 launch wmx_r2_package wmx_r2_general_nodes.launch.py \
  use_sim_time:=false \
  'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/config/wmx_r2_general_nodes_config.yaml' \
  'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/config/wmx_parameters.xml'
```

---

## Engine — `wmx_engine_node`

A plain node, not a lifecycle node: its services are up as soon as it starts. At
startup it already creates the device, imports `wmx_param_file_path` and starts
communication, so none of the calls below is needed for a normal bring-up. If the
import fails it closes the device rather than communicate with default
parameters.

**`wmx/engine/set_engine`** — `true` creates the WMX3 device, imports `wmx_param_file_path` and starts communication (only starts communication if the device already exists). `false` stops the engine and closes the device; the lifecycle manager takes the managed nodes down after three failed status sweeps, so about `3 × discovery_period`.
```bash
wros ros2 service call /wmx/engine/set_engine std_srvs/srv/SetBool '"{data: true}"'
```

**`wmx/engine/set_communication`** — Starts or stops EtherCAT communication on the open device.
```bash
wros ros2 service call /wmx/engine/set_communication std_srvs/srv/SetBool '"{data: true}"'
```

**`wmx/engine/get_engine_status`** — Returns the engine state in `message`: `Idle`, `Running`, `Communicating`, `Shutdown`, `Unknown` or `Invalid`. `success: false` means the engine could not be queried (device closed). The lifecycle manager brings nodes up only on `Communicating`.
```bash
wros ros2 service call /wmx/engine/get_engine_status std_srvs/srv/Trigger
```

**`wmx/engine/import_and_set_all`** — Imports a WMX3 parameter XML into the running engine (`ImportAndSetAll`), absolute path only. The `$(...)` resolves inside the container, so it works whatever the workspace path is.
```bash
wros ros2 service call /wmx/engine/import_and_set_all wmx_r2_message/srv/ImportAndSetAll \
  '"{path: $(ros2 pkg prefix --share wmx_r2_package)/config/wmx_parameters.xml}"'
```

**`wmx/engine/get_axis_param`** — Returns the active gear ratio, polarity and command mode of the requested axes as text lines in `axis_param`. Axis indices must be within the SDK's `maxAxes`.
```bash
wros ros2 service call /wmx/engine/get_axis_param wmx_r2_message/srv/GetAxisParam '"{axis: [0,1,2,3,4,5]}"'
```

| Parameter | Default | Meaning |
|---|---|---|
| `core` | `-1` | CPU core for the RT engine (`-1` = SDK default) |
| `affinity_mask` | `0` | CPU affinity bitmask (`0` = SDK default) |
| `wmx_param_file_path` | `""` | Absolute path of the XML imported right after the device is created; set by the `wmx_param_file` launch argument when given |

---

## Lifecycle — `wmx_lifecycle_manager_node`

A plain node. Every `discovery_period` it asks `wmx/engine/get_engine_status`; while the answer is `Communicating` it brings every discovered node up (`configure` + `activate`) once. After three failed answers in a row it brings them all down.

**`wmx/lifecycle/set_node_state`** — Drives one node (or every discovered node, with an empty `node_name`) through `configure`, `activate`, `deactivate`, `cleanup`, `shutdown`, `bringup` or `bringdown`. A node taken down this way stays down; the automatic sweep does not bring it back until it is brought up by hand. The response lists the touched nodes and their new states.
```bash
wros ros2 service call /wmx/lifecycle/set_node_state wmx_r2_message/srv/SetNodeState \
  '"{node_name: wmx_io_node, transition: deactivate}"'
wros ros2 service call /wmx/lifecycle/set_node_state wmx_r2_message/srv/SetNodeState \
  '"{node_name: wmx_io_node, transition: bringup}"'
```

**`wmx/lifecycle/get_node_states`** — Lists every discovered lifecycle node with its current state, in bring-up order.
```bash
wros ros2 service call /wmx/lifecycle/get_node_states wmx_r2_message/srv/GetNodeStates
```

The standard CLI works too and bypasses the manager. The manager brings each node up only once per engine session, so a node taken down this way also stays down until it is brought up by hand.
```bash
wros ros2 lifecycle get /wmx_io_node
wros ros2 lifecycle set /wmx_io_node activate
```

| Parameter | Default | Meaning |
|---|---|---|
| `managed_nodes` | `[]` | Nodes to manage, in bring-up order; take-down is the reverse. A non-empty list restricts the manager to exactly these nodes. Empty manages every lifecycle node on the graph, `wmx_*` nodes first, then alphabetical. The shipped config lists `wmx_core_motion_node`, `wmx_io_node`, `wmx_ethercat_node` |
| `discovery_period` | `2.0` | Seconds between engine-state sweeps. The shipped config sets `0.2` |

---

## Axes — `wmx_core_motion_node`

Every position, velocity and acceleration below is in the **axis user unit**, not
in degrees, radians or metres by itself. See
[Units and conventions](#units-and-conventions) for what the shipped files make
that unit.

**`wmx/axes/set_servo_on`** — Servo on (`1`) or off (`0`) per axis.
```bash
wros ros2 service call /wmx/axes/set_servo_on wmx_r2_message/srv/SetAxes '"{axis: [0,1], data: [1,1]}"'
```

**`wmx/axes/clear_amp_alarm`** — Clears the amplifier alarm per axis; `data` is ignored and may be left out.
```bash
wros ros2 service call /wmx/axes/clear_amp_alarm wmx_r2_message/srv/SetAxes '"{axis: [0,1]}"'
```

**`wmx/axes/set_axis_command_mode`** — Sets the command mode: `0` = Position, `1` = Velocity. Any other value (torque included) is rejected.
```bash
wros ros2 service call /wmx/axes/set_axis_command_mode wmx_r2_message/srv/SetAxes '"{axis: [0,1], data: [0,0]}"'
```

**`wmx/axes/set_axis_polarity`** — Sets the axis direction: `1` = normal, `-1` = reversed. Any other value is rejected.
```bash
wros ros2 service call /wmx/axes/set_axis_polarity wmx_r2_message/srv/SetAxes '"{axis: [0,1], data: [1,1]}"'
```

**`wmx/axes/set_gear_ratio`** — Sets `numerator` encoder counts per `denominator` user units, per axis. Normally the parameter XML sets this; the call overrides it until the next import. Example: the CR3A joint 1 ratio from `example/cr3a_wmx_parameters.xml`, which makes the user unit radians.
```bash
wros ros2 service call /wmx/axes/set_gear_ratio wmx_r2_message/srv/SetAxesGearRatio \
  '"{axis: [0], numerator: [42467328.0], denominator: [6.283185307179586]}"'
```

**`wmx/axes/start_home`** — Overwrites the axis `HomeType` with `CurrentPos`, then
homes, so the current position becomes zero. Whatever home type the parameter XML
configured (switch, Z pulse, ...) is replaced. `data` is ignored and may be left
out. Rejected while a motion controller is active.
```bash
wros ros2 service call /wmx/axes/start_home wmx_r2_message/srv/SetAxes '"{axis: [0,1]}"'
```

**`wmx/axes/stop`** — Decelerates the axes to a stop and cancels any jog on them; never blocked, even while a controller owns the axes. `data` is ignored but must have the same length as `axis`.
```bash
wros ros2 service call /wmx/axes/stop wmx_r2_message/srv/SetAxes '"{axis: [0,1], data: [0,0]}"'
```

**`wmx/axes/status`** — Topic (`wmx_r2_message/msg/AxesStatus`, every axis the engine reports, at `axes_status_rate`) with per-axis `amp_alarm`, `servo_on`, `home_done`, `home_switch`, `negative_ls`, `positive_ls`, `motion_complete`, plus `pos_cmd`, `velocity_cmd`, `actual_pos`, `actual_velocity` and `actual_torque`.
```bash
wros ros2 topic echo /wmx/axes/status
```

The four motion services below **rotate the axes**. Their examples assume the
1:1 `config/wmx_parameters.xml`, so the numbers are encoder counts and counts/s.
All of them answer `success: false` while a motion controller is active.

**`wmx/axes/start_pos`** — Moves the axes to an absolute target with a trapezoidal profile.
```bash
wros ros2 service call /wmx/axes/start_pos wmx_r2_message/srv/StartAxesPose \
  '"{axis: [0,1], target: [8388608, 10000], velocity: [1000000, 5000], acc: [100000, 1000], dec: [100000, 1000]}"'
```

**`wmx/axes/start_mov`** — Moves the axes by a relative distance.
```bash
wros ros2 service call /wmx/axes/start_mov wmx_r2_message/srv/StartAxesPose \
  '"{axis: [0,1], target: [1000000, -1000], velocity: [1000000, 5000], acc: [100000, 1000], dec: [100000, 1000]}"'
```

**`wmx/axes/start_vel`** — Runs the axes at constant velocity until `wmx/axes/stop`. Works in Position command mode; no `set_axis_command_mode` needed.
```bash
wros ros2 service call /wmx/axes/start_vel wmx_r2_message/srv/StartAxesVelocity \
  '"{axis: [0,1], velocity: [100000, -1000], acc: [100000, 1000], dec: [100000, 1000]}"'
```

**`wmx/axes/start_jog`** — Dead-man jog: each call with the same `velocity` refreshes it, and the axis stops `jog_timeout_ms` after the last refresh. Sign selects direction; `velocity: [0]` stops at once; a single jog never runs longer than `jog_run_time_ms`. One `wros ros2 service call` per refresh takes far longer than 200 ms to start, so a shell loop stutters. Let one process repeat the call with `-r` (Hz) and press Ctrl-C to release.
```bash
wros ros2 service call -r 1 /wmx/axes/start_jog wmx_r2_message/srv/StartAxesVelocity \
  '"{axis: [0], velocity: [100000], acc: [1000000], dec: [1000000]}"'
```

| Parameter | Default | Meaning |
|---|---|---|
| `axes_status_rate` | `100` | `wmx/axes/status` rate in Hz. The shipped config sets `1` |
| `jog_timeout_ms` | `200.0` | Axis stops this long after jog refreshes stop arriving |
| `jog_run_time_ms` | `2000.0` | Maximum duration of one jog, enforced engine-side |
| `jog_jerk_ratio` | `0.75` | Jerk ratio of the jog profile |
| `motion_controllers` | `joint_trajectory_controller`, `differential_drive_controller`, `joint_position_controller` | Any lifecycle node name, not just these three: `wmx_r2_robot_option_config.yaml` lists `wmx_robot_option_node` here. While one of them is active it owns the axes and `start_pos`, `start_mov`, `start_vel`, `start_jog` and `start_home` answer `success: false`. `stop` and the servo/config services stay open. A listed name that never appears on the graph is treated as active and blocks motion, so a config must list only controllers that are actually launched. The shipped `wmx_r2_general_nodes_config.yaml` sets `['']` (none), since this launch starts no controller |
| `controller_resync_period` | `0.2` | Seconds between re-queries of each controller's state |

---

## IO — `wmx_io_node`

`addr` is the IO byte address, `bit` the index within that byte (0–7), `data` values are decimal.

**`wmx/io/get_in_bit`** — Reads one input bit.
```bash
wros ros2 service call /wmx/io/get_in_bit wmx_r2_message/srv/GetIoBit '"{addr: 0, bit: 0}"'
```

**`wmx/io/get_out_bit`** — Reads one output bit.
```bash
wros ros2 service call /wmx/io/get_out_bit wmx_r2_message/srv/GetIoBit '"{addr: 0, bit: 0}"'
```

**`wmx/io/get_in_bits`** — Reads scattered input bits, one `data` entry per `addr`/`bit` pair.
```bash
wros ros2 service call /wmx/io/get_in_bits wmx_r2_message/srv/GetIoBits '"{addr: [0, 2], bit: [1, 5]}"'
```

**`wmx/io/get_out_bits`** — Reads scattered output bits, one `data` entry per pair.
```bash
wros ros2 service call /wmx/io/get_out_bits wmx_r2_message/srv/GetIoBits '"{addr: [0, 2], bit: [1, 5]}"'
```

**`wmx/io/get_in_byte`** — Reads one input byte.
```bash
wros ros2 service call /wmx/io/get_in_byte wmx_r2_message/srv/GetIoByte '"{addr: 0}"'
```

**`wmx/io/get_out_byte`** — Reads one output byte.
```bash
wros ros2 service call /wmx/io/get_out_byte wmx_r2_message/srv/GetIoByte '"{addr: 0}"'
```

**`wmx/io/get_in_bytes`** — Reads `size` consecutive input bytes from `addr`.
```bash
wros ros2 service call /wmx/io/get_in_bytes wmx_r2_message/srv/GetIoBytes '"{addr: 0, size: 4}"'
```

**`wmx/io/get_out_bytes`** — Reads `size` consecutive output bytes from `addr`.
```bash
wros ros2 service call /wmx/io/get_out_bytes wmx_r2_message/srv/GetIoBytes '"{addr: 0, size: 4}"'
```

**`wmx/io/set_out_bit`** — Writes one output bit, `0` or `1`.
```bash
wros ros2 service call /wmx/io/set_out_bit wmx_r2_message/srv/SetIoBit '"{addr: 0, bit: 0, data: 1}"'
```

**`wmx/io/set_out_bits`** — Writes scattered output bits together in one SDK call.
```bash
wros ros2 service call /wmx/io/set_out_bits wmx_r2_message/srv/SetIoBits \
  '"{addr: [0, 2], bit: [1, 5], data: [1, 0]}"'
```

**`wmx/io/set_out_byte`** — Writes one output byte (`15` = `0x0F`).
```bash
wros ros2 service call /wmx/io/set_out_byte wmx_r2_message/srv/SetIoByte '"{addr: 2, data: 15}"'
```

**`wmx/io/set_out_bytes`** — Writes consecutive output bytes from `addr`.
```bash
wros ros2 service call /wmx/io/set_out_bytes wmx_r2_message/srv/SetIoBytes '"{addr: 2, data: [15, 14]}"'
```

---

## EtherCAT — `wmx_ethercat_node`

**`wmx/ecat/get_master_info`** — Reads master and per-slave status, with `state` `None`=0, `Init`=1, `Preop`=2, `Boot`=4, `Safeop`=8, `Op`=16 and `mode` `Cyclic`=0, `PP`=1, `Monitor`=2.
```bash
wros ros2 service call /wmx/ecat/get_master_info wmx_r2_message/srv/EcatGetMasterInfo '"{master_id: 0}"'
```

**`wmx/ecat/register_read`** — Reads `len` bytes from a slave ESC register, `reg_addr` decimal in `0x000`–`0xFFF` with `reg_addr + len` ≤ `0x1000`.
```bash
wros ros2 service call /wmx/ecat/register_read wmx_r2_message/srv/EcatRegisterRead \
  '"{master_id: 0, slave_id: 0, reg_addr: 16, len: 4}"'
```

**`wmx/ecat/reset_statistics`** — Resets the ref-clock and transmit statistics, then re-scans the network.
```bash
wros ros2 service call /wmx/ecat/reset_statistics wmx_r2_message/srv/EcatResetStatistics '"{master_id: 0}"'
```

**`wmx/ecat/scan_network`** — Re-scans the network for slaves.
```bash
wros ros2 service call /wmx/ecat/scan_network wmx_r2_message/srv/EcatScanNetwork '"{master_id: 0}"'
```

**`wmx/ecat/start_hotconnect`** — Enables dynamic slave discovery; call once after the network reaches `Op`.
```bash
wros ros2 service call /wmx/ecat/start_hotconnect wmx_r2_message/srv/EcatStartHotconnect '"{master_id: 0}"'
```

---

## Typical startup sequence

The engine starts communicating on its own and the lifecycle manager activates
the managed nodes, so a bring-up only checks the state and powers the axes.

```bash
# 1. Engine should answer "Communicating"
wros ros2 service call /wmx/engine/get_engine_status std_srvs/srv/Trigger

# 2. Managed nodes should be "active"
wros ros2 service call /wmx/lifecycle/get_node_states wmx_r2_message/srv/GetNodeStates

# 3. Clear any amp alarms
wros ros2 service call /wmx/axes/clear_amp_alarm wmx_r2_message/srv/SetAxes '"{axis: [0,1]}"'

# 4. Enable servos
wros ros2 service call /wmx/axes/set_servo_on wmx_r2_message/srv/SetAxes '"{axis: [0,1], data: [1,1]}"'

# 5. Zero the axes at their current position
wros ros2 service call /wmx/axes/start_home wmx_r2_message/srv/SetAxes '"{axis: [0,1]}"'

# 6. Watch the result
wros ros2 topic echo --once /wmx/axes/status
```

---

## Units and conventions

Nothing on `wmx/axes/*` carries a fixed physical unit. Position, velocity and
acceleration are all in the **axis user unit**, which the axis parameter XML sets
per axis as `AxisGearRatioNumerator` encoder counts per
`AxisGearRatioDenominator` user units. Change that ratio and every number on
these services changes meaning.

| Field | Unit |
|---|---|
| `target` on `start_pos`, `start_mov` | user unit |
| `velocity` on `start_pos`, `start_mov`, `start_vel`, `start_jog` | user unit / s |
| `acc`, `dec` | user unit / s² |
| `pos_cmd`, `actual_pos` on `wmx/axes/status` | user unit |
| `velocity_cmd`, `actual_velocity` | user unit / s |
| `actual_torque` | **%** of rated torque |
| `axes_status_rate` | Hz |
| `jog_timeout_ms`, `jog_run_time_ms` | ms |
| `controller_resync_period` | s |
| `jog_jerk_ratio` | ratio, 0 to 1 |
| everything on `wmx/io/*` | raw bits and bytes, no unit |

---

## Notes

- Every array in an `axis` request must be the same length as `axis` (`clear_amp_alarm` and `start_home` ignore `data`).
- `wmx_core_motion_node`, `wmx_io_node` and `wmx_ethercat_node` advertise their services and topics only while `active`. `wmx_engine_node` and `wmx_lifecycle_manager_node` are plain nodes and are always up.
- Every command is a service answering `success` (true only if every axis succeeded) and one `message` string with the per-axis results joined by `; `. `wmx/axes/status` is the only topic.
- `deactivate` drops the interfaces but keeps the WMX3 device attached; `cleanup` detaches it.
- Node parameters are read once at startup — set them in the config YAML, not with `ros2 param set`.
- Inspect the graph with `wros ros2 service list -t | grep /wmx/`, `wros ros2 interface show wmx_r2_message/srv/<Type>` and `wros ros2 param get /<node> <param>`.
