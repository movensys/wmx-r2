# WMX R2 General Nodes Reference

Every interface of the five nodes started by `wmx_r2_general_nodes.launch.py`:
`wmx_engine_node`, `wmx_lifecycle_manager_node`, `wmx_core_motion_node`,
`wmx_io_node`, `wmx_ethercat_node`.

check `doc/launch_wmx_r2_general_nodes.md`

---

## Engine — `wmx_engine_node`

**`wmx/engine/set_engine`** — Creates the WMX3 device and starts communication; `false` closes the device and the managed nodes follow within one `discovery_period`.
```bash
ros2 service call /wmx/engine/set_engine std_srvs/srv/SetBool "{data: true}"
```

**`wmx/engine/set_communication`** — Starts or stops EtherCAT communication on the open device.
```bash
ros2 service call /wmx/engine/set_communication std_srvs/srv/SetBool "{data: true}"
```

**`wmx/engine/get_engine_status`** — Returns the engine state: `Idle`, `Running`, `Communicating`, `Shutdown` or `Unknown`.
```bash
ros2 service call /wmx/engine/get_engine_status std_srvs/srv/Trigger "{}"
```

**`wmx/engine/import_and_set_all`** — Imports a WMX3 parameter XML into the running engine, absolute path only.
```bash
ros2 service call /wmx/engine/import_and_set_all wmx_r2_message/srv/ImportAndSetAll \
  "{path: '$HOME/movensys_ws/install/wmx_r2_package/share/wmx_r2_package/config/cr3a_wmx_parameters.xml'}"
```

**`wmx/engine/get_axis_param`** — Dumps the active gear ratio, polarity and command mode of the requested axes.
```bash
ros2 service call /wmx/engine/get_axis_param wmx_r2_message/srv/GetAxisParam "{axis: [0,1,2,3,4,5]}"
```

| Parameter | Default | Meaning |
|---|---|---|
| `core` | `-1` | CPU core for the RT engine (`-1` = SDK default) |
| `affinity_mask` | `0` | CPU affinity bitmask (`0` = SDK default) |
| `wmx_param_file_path` | `""` | XML imported right after the device is created |

---

## Lifecycle — `wmx_lifecycle_manager_node`

**`wmx/lifecycle/set_node_state`** — Drives one node (or every node found, with an empty `node_name`) through `configure`, `activate`, `deactivate`, `cleanup`, `shutdown`, `bringup` or `bringdown`.
```bash
ros2 service call /wmx/lifecycle/set_node_state wmx_r2_message/srv/SetNodeState \
  "{node_name: 'wmx_io_node', transition: 'deactivate'}"
```

**`wmx/lifecycle/get_node_states`** — Lists every lifecycle node on the graph with its current state.
```bash
ros2 service call /wmx/lifecycle/get_node_states wmx_r2_message/srv/GetNodeStates "{}"
```

The standard CLI works too and bypasses the manager.
```bash
ros2 lifecycle get /wmx_io_node
ros2 lifecycle set /wmx_io_node activate
```

| Parameter | Default | Meaning |
|---|---|---|
| `managed_nodes` | `[]` | Bring-up order; take-down is the reverse |
| `discovery_period` | `2.0` | Seconds between engine-state sweeps |

---

## Axes — `wmx_core_motion_node`

**`wmx/axes/set_servo_on`** — Servo on (`1`) or off (`0`) per axis.
```bash
ros2 service call /wmx/axes/set_servo_on wmx_r2_message/srv/SetAxes "{axis: [0,1], data: [1,1]}"
```

**`wmx/axes/clear_amp_alarm`** — Clears the amplifier alarm per axis; `data` is ignored.
```bash
ros2 service call /wmx/axes/clear_amp_alarm wmx_r2_message/srv/SetAxes "{axis: [0,1], data: [0,0]}"
```

**`wmx/axes/set_axis_command_mode`** — Sets the command mode: `0` = Position, `1` = Velocity, `2` = Torque.
```bash
ros2 service call /wmx/axes/set_axis_command_mode wmx_r2_message/srv/SetAxes "{axis: [0,1], data: [0,0]}"
```

**`wmx/axes/set_axis_polarity`** — Sets the axis direction: `1` = normal, `-1` = reversed.
```bash
ros2 service call /wmx/axes/set_axis_polarity wmx_r2_message/srv/SetAxes "{axis: [0,1], data: [1,1]}"
```

**`wmx/axes/set_gear_ratio`** — Sets the encoder-count-to-user-unit ratio per axis.
```bash
ros2 service call /wmx/axes/set_gear_ratio wmx_r2_message/srv/SetAxesGearRatio \
  "{axis: [0,1], numerator: [8388608.0, 8388608.0], denominator: [360.0, 360.0]}"
```

**`wmx/axes/start_home`** — Homes with the `HomeType` configured in the WMX parameter
file for that axis. The shipped parameter files use `CurrentPos`, which makes the
current encoder position zero. The service does not write the home configuration,
so a switch based or Z pulse setup is left alone. `data` is ignored.
```bash
ros2 service call /wmx/axes/start_home wmx_r2_message/srv/SetAxes "{axis: [0,1], data: [0,0]}"
```

**`wmx/axes/stop`** — Decelerates the axes to a stop; never blocked, even while a controller owns the axes.
```bash
ros2 service call /wmx/axes/stop wmx_r2_message/srv/SetAxes "{axis: [0,1], data: [0,0]}"
```

**`wmx/axes/status`** — Publishes per-axis alarms, servo state, limit switches, and commanded and actual values.
```bash
ros2 topic echo /wmx/axes/status
```

**`wmx/axes/start_pos`** — Moves the axes to an absolute target (**rotates the axes**).
```bash
ros2 service call /wmx/axes/start_pos wmx_r2_message/srv/StartAxesPose \
  "{axis: [0,1], target: [45, -90], velocity: [10, 20], acc: [10, 20], dec: [10, 20]}"
```

**`wmx/axes/start_mov`** — Moves the axes by a relative distance (**rotates the axes**).
```bash
ros2 service call /wmx/axes/start_mov wmx_r2_message/srv/StartAxesPose \
  "{axis: [0,1], target: [10, -10], velocity: [10, 20], acc: [10, 10], dec: [10, 20]}"
```

**`wmx/axes/start_vel`** — Runs the axes at constant velocity until stopped (**rotates the axes**).
```bash
ros2 service call /wmx/axes/start_vel wmx_r2_message/srv/StartAxesVelocity \
  "{axis: [0,1], velocity: [10, -10], acc: [10, 20], dec: [10, 20]}"
```

**`wmx/axes/start_jog`** — Dead-man jog in Position mode: keep calling to keep moving, stop calling to release, sign selects direction.
```bash
while true; do
  ros2 service call /wmx/axes/start_jog wmx_r2_message/srv/StartAxesVelocity \
    "{axis: [0], velocity: [10000], acc: [100000], dec: [100000]}"
  sleep 0.05
done
```

| Parameter | Default | Meaning |
|---|---|---|
| `axes_status_rate` | `100` | `wmx/axes/status` rate in Hz |
| `jog_timeout_ms` | `200.0` | Axis stops this long after jog refreshes stop arriving |
| `jog_run_time_ms` | `2000.0` | Maximum duration of one jog, enforced engine-side |
| `jog_jerk_ratio` | `0.75` | Jerk ratio of the jog profile |
| `motion_controllers` | the three robot controllers | While one of them is active it owns the axes and every motion service here answers `success: false`, except `stop` |
| `controller_resync_period` | `0.2` | Seconds between re-queries of each controller's state |

---

## IO — `wmx_io_node`

`addr` is the IO byte address, `bit` the index within that byte (0–7), `data` values are decimal.

**`wmx/io/get_in_bit`** — Reads one input bit.
```bash
ros2 service call /wmx/io/get_in_bit wmx_r2_message/srv/GetIoBit "{addr: 0, bit: 0}"
```

**`wmx/io/get_out_bit`** — Reads one output bit.
```bash
ros2 service call /wmx/io/get_out_bit wmx_r2_message/srv/GetIoBit "{addr: 0, bit: 0}"
```

**`wmx/io/get_in_bits`** — Reads scattered input bits, one `data` entry per `addr`/`bit` pair.
```bash
ros2 service call /wmx/io/get_in_bits wmx_r2_message/srv/GetIoBits "{addr: [0, 2], bit: [1, 5]}"
```

**`wmx/io/get_out_bits`** — Reads scattered output bits, one `data` entry per pair.
```bash
ros2 service call /wmx/io/get_out_bits wmx_r2_message/srv/GetIoBits "{addr: [0, 2], bit: [1, 5]}"
```

**`wmx/io/get_in_byte`** — Reads one input byte.
```bash
ros2 service call /wmx/io/get_in_byte wmx_r2_message/srv/GetIoByte "{addr: 0}"
```

**`wmx/io/get_out_byte`** — Reads one output byte.
```bash
ros2 service call /wmx/io/get_out_byte wmx_r2_message/srv/GetIoByte "{addr: 0}"
```

**`wmx/io/get_in_bytes`** — Reads `size` consecutive input bytes from `addr`.
```bash
ros2 service call /wmx/io/get_in_bytes wmx_r2_message/srv/GetIoBytes "{addr: 0, size: 4}"
```

**`wmx/io/get_out_bytes`** — Reads `size` consecutive output bytes from `addr`.
```bash
ros2 service call /wmx/io/get_out_bytes wmx_r2_message/srv/GetIoBytes "{addr: 0, size: 4}"
```

**`wmx/io/set_out_bit`** — Writes one output bit, `0` or `1`.
```bash
ros2 service call /wmx/io/set_out_bit wmx_r2_message/srv/SetIoBit "{addr: 0, bit: 0, data: 1}"
```

**`wmx/io/set_out_bits`** — Writes scattered output bits together in one SDK call.
```bash
ros2 service call /wmx/io/set_out_bits wmx_r2_message/srv/SetIoBits \
  "{addr: [0, 2], bit: [1, 5], data: [1, 0]}"
```

**`wmx/io/set_out_byte`** — Writes one output byte (`15` = `0x0F`).
```bash
ros2 service call /wmx/io/set_out_byte wmx_r2_message/srv/SetIoByte "{addr: 2, data: 15}"
```

**`wmx/io/set_out_bytes`** — Writes consecutive output bytes from `addr`.
```bash
ros2 service call /wmx/io/set_out_bytes wmx_r2_message/srv/SetIoBytes "{addr: 2, data: [15, 14]}"
```

---

## EtherCAT — `wmx_ethercat_node`

**`wmx/ecat/get_master_info`** — Reads master and per-slave status, with `state` `None`=0, `Init`=1, `Preop`=2, `Boot`=4, `Safeop`=8, `Op`=16 and `mode` `Cyclic`=0, `PP`=1, `Monitor`=2.
```bash
ros2 service call /wmx/ecat/get_master_info wmx_r2_message/srv/EcatGetMasterInfo "{master_id: 0}"
```

**`wmx/ecat/register_read`** — Reads `len` bytes from a slave ESC register, `reg_addr` decimal in `0x000`–`0xFFF` with `reg_addr + len` ≤ `0x1000`.
```bash
ros2 service call /wmx/ecat/register_read wmx_r2_message/srv/EcatRegisterRead \
  "{master_id: 0, slave_id: 0, reg_addr: 16, len: 4}"
```

**`wmx/ecat/reset_statistics`** — Resets the ref-clock and transmit statistics, then re-scans the network.
```bash
ros2 service call /wmx/ecat/reset_statistics wmx_r2_message/srv/EcatResetStatistics "{master_id: 0}"
```

**`wmx/ecat/scan_network`** — Re-scans the network for slaves.
```bash
ros2 service call /wmx/ecat/scan_network wmx_r2_message/srv/EcatScanNetwork "{master_id: 0}"
```

**`wmx/ecat/start_hotconnect`** — Enables dynamic slave discovery; call once after the network reaches `Op`.
```bash
ros2 service call /wmx/ecat/start_hotconnect wmx_r2_message/srv/EcatStartHotconnect "{master_id: 0}"
```

---

## Typical startup sequence

```bash
# 1. Verify engine is communicating
ros2 service call /wmx/engine/get_engine_status std_srvs/srv/Trigger "{}"

# 2. Set gear ratio
ros2 service call /wmx/axes/set_gear_ratio wmx_r2_message/srv/SetAxesGearRatio \
  "{axis: [0, 1], numerator: [8388608.0, 8388608.0], denominator: [360.0, 360.0]}"

# 3. Clear any amp alarms
ros2 service call /wmx/axes/clear_amp_alarm wmx_r2_message/srv/SetAxes "{axis: [0,1], data: [0,0]}"

# 4. Enable servos
ros2 service call /wmx/axes/set_servo_on wmx_r2_message/srv/SetAxes "{axis: [0,1], data: [1,1]}"

# 5. Home all axes
ros2 service call /wmx/axes/start_home wmx_r2_message/srv/SetAxes "{axis: [0,1], data: [0,0]}"
```

---

## Notes

- `axis` and `data` arrays must be the same length.
- Services and topics exist only while a node is `active`, so an inactive node advertises nothing.
- Every motion command is a service and answers `success` plus a per-axis `message`; `wmx/axes/status` is the only topic.
- `deactivate` drops the interfaces but keeps the WMX3 device attached; `cleanup` detaches it.
- Node parameters are read once at startup — set them in the config YAML, not with `ros2 param set`.
- `ros2 service list`, `ros2 service type <service>` and `ros2 param list|get|set <node>` work on every node.
