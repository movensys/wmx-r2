# WMX R2 General Nodes Reference

## Typical Startup Sequence
- Note that user can reduce the axis dimension for their purpose.

```
# 1. Verify engine is communicating
ros2 service call /wmx/engine/get_engine_status std_srvs/srv/Trigger "{}"

# 2. Load axis parameters from file
## 2-1. Predefined robot case
## Dobot CR3a, CR5a, Synustech (synustech_wmx_parameters.xml), Diffbot AMR
ros2 service call /wmx/engine/load_wmx_params wmx_r2_message/srv/LoadWmxParams \
  "{file_path: '/home/$USER/workspaces/movensys_ws/install/wmx_r2_package/share/wmx_r2_package/config/cr3a_wmx_parameters.xml'}"
## 2-2. User's own robot or arbitary motors
ros2 service call /wmx/engine/load_wmx_params wmx_r2_message/srv/LoadWmxParams \
  "{file_path: '/home/$USER/workspaces/movensys_ws/install/wmx_r2_package/share/wmx_r2_package/config/default_wmx_parameters.xml'}"

## 3. Set Gear Ratio (2-1 stage can skip this stage)
## Panasonic MADLNO5BE servo driver
ros2 service call /wmx/axes/set_gear_ratio wmx_r2_message/srv/SetAxesGearRatio \
  "{indices: [0], numerators: [8388608.0], denominators: [360.0]}"

# 4. Clear any amp alarms
ros2 service call /wmx/axes/clear_amp_alarm wmx_r2_message/srv/SetAxes "{indices: [0,1,2,3,4,5], data: [0,0,0,0,0,0]}"

# 5. Enable servos
ros2 service call /wmx/axes/set_servo_on wmx_r2_message/srv/SetAxes "{indices: [0,1,2,3,4,5], data: [1,1,1,1,1,1]}"

# 6. Home all axes (sets current position as home)
ros2 service call /wmx/axes/start_home wmx_r2_message/srv/SetAxes "{indices: [0,1,2,3,4,5], data: [0,0,0,0,0,0]}"
```
---
i

## Engine Topics (Warning: These command will rotate the axes)
### Send Axis Absolute Position
```
ros2 topic pub --once /wmx/axes/start_pos wmx_r2_message/msg/AxesPose \
    "{indices: [0,1], positions: [8388608, 10000 ], velocities: [1000000, 5000], accelerations: [100000, 1000], decelerations: [100000, 1000]}" 
```

### Send Axis Relative Position
```
ros2 topic pub --once /wmx/axes/start_mov wmx_r2_message/msg/AxesPose \
    "{indices: [0, 1], positions: [8388608, 10000], velocities: [1000000, 5000], accelerations: [100000, 1000], decelerations: [100000, 1000]}"
```i


### Send Axis Velocity
```
ros2 topic pub --once /wmx/axes/start_vel wmx_r2_message/msg/AxesVelocity \
    "{indices: [0, 1], velocities: [1000000, 5000], accelerations: [100000, 1000], decelerations: [100000, 1000]}"  
```



i
### Jog (hold-to-move)
`/wmx/axes/start_jog` is a dead-man command: the publisher must keep republishing while
the operator holds the control. The axis is stopped once refreshes stop arriving
(`jog_timeout_ms`). The sign of `velocity` selects the direction.

```
# Jog axis 0 in the positive direction. Ctrl-C acts as the release.
ros2 topic pub -r 20 /wmx/axes/start_jog wmx_r2_message/msg/AxesVelocity \
    "{indices: [0], velocities: [10000], accelerations: [100000], decelerations: [100000]}"
i
# Negative direction
ros2 topic pub -r 20 /wmx/axes/start_jog wmx_r2_message/msg/AxesVelocity \
    "{indices: [0], velocities: [-10000], accelerations: [100000], decelerations: [100000]}"
```

---

## Engine Services
### Get Status
```
ros2 service call /wmx/engine/get_engine_status std_srvs/srv/Trigger "{}"
```

### Set Communication (start/stop EtherCAT comms)
```
# Start communication
ros2 service call /wmx/engine/set_comm std_srvs/srv/SetBool "{data: true}"

# Stop communication
ros2 service call /wmx/engine/set_comm std_srvs/srv/SetBool "{data: false}"
```

### Set Engine (create/close WMX3 device)
The SDK path and device name are fixed by the node, so this is a plain on/off.
```
# Create device
ros2 service call /wmx/engine/set_engine std_srvs/srv/SetBool "{data: true}"

# Close device
ros2 service call /wmx/engine/set_engine std_srvs/srv/SetBool "{data: false}"
```

---

## WMX Parameter Services
### Load Parameters from File
```
# Dobot CR3A
ros2 service call /wmx/engine/load_wmx_params wmx_r2_message/srv/LoadWmxParams \
  "{file_path: '/home/$USER/movensys_ws/install/wmx_r2_package/share/wmx_r2_package/config/cr3a_wmx_parameters.xml'}"

# Diffbot
ros2 service call /wmx/engine/load_wmx_params wmx_r2_message/srv/LoadWmxParams \
  "{file_path: '/home/$USER/movensys_ws/install/wmx_r2_package/share/wmx_r2_package/config/diffbot_wmx_parameters.xml'}"
```

### Get Parameters (inspect active axis config)
```
# Single axis
ros2 service call /wmx/engine/get_wmx_params wmx_r2_message/srv/GetWmxParams "{indices: [0]}"

# Multiple axes
ros2 service call /wmx/engine/get_wmx_params wmx_r2_message/srv/GetWmxParams "{indices: [0,1,2,3,4,5]}"
```

---

## Axis Services

### Clear Alarm
```
ros2 service call /wmx/axes/clear_amp_alarm wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [0,0]}"
```

### Set Servo On / Off
```
# Servo On
ros2 service call /wmx/axes/set_servo_on wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [1,1]}"

# Servo Off
ros2 service call /wmx/axes/set_servo_on wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [0,0]}"
```

### Set Command Mode
```
# Position mode (0)
ros2 service call /wmx/axes/set_axis_command_mode wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [0,0]}"

# Velocity mode (1)
ros2 service call /wmx/axes/set_axis_command_mode wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [1,1]}"
```

### Set Polarity
```
# Normal (1)
ros2 service call /wmx/axes/set_axis_polarity wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [1,1]}"

# Reversed (-1)
ros2 service call /wmx/axes/set_axis_polarity wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [-1,-1]}"
```

### Set Gear Ratio
```
ros2 service call /wmx/axes/set_gear_ratio wmx_r2_message/srv/SetAxesGearRatio \
  "{indices: [0,1], numerators: [1.0,1.0], denominators: [1.0,1.0]}"
```



### Homing (sets current position as home)
```
ros2 service call /wmx/axes/start_home wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [0,0]}"
```

### Stop (decelerate to a stop)
```
ros2 service call /wmx/axes/stop wmx_r2_message/srv/SetAxes "{indices: [0,1], data: [0,0]}"
```

---

## Node Lifecycle

Every node except `wmx_engine_node` is a managed
[lifecycle node](https://design.ros2.org/articles/node_lifecycle.html). It starts
`unconfigured` and only opens its WMX3 device — and only advertises its own topics,
services and actions — once it is configured and activated.

`wmx_lifecycle_manager_node` does that for you. It polls
`wmx/engine/get_engine_status`, and once the engine reports `Communicating` it
discovers every lifecycle node on the graph and brings it up (`configure` then
`activate`), device-level `wmx_*` nodes first. If the engine stops communicating it
takes them back down (`deactivate` then `cleanup`) and brings them up again when it
returns.

### List Node States
```
ros2 service call /wmx/lifecycle/get_node_states wmx_r2_message/srv/GetNodeStates "{}"
```

### Drive a Single Node
```
# Take the IO node out of service without stopping the process
ros2 service call /wmx/lifecycle/set_node_state wmx_r2_message/srv/SetNodeState \
  "{node_name: 'wmx_io_node', transition: 'deactivate'}"

# Put it back
ros2 service call /wmx/lifecycle/set_node_state wmx_r2_message/srv/SetNodeState \
  "{node_name: 'wmx_io_node', transition: 'activate'}"
```

### Drive Every Node
```
# An empty node_name applies the transition to every discovered lifecycle node
ros2 service call /wmx/lifecycle/set_node_state wmx_r2_message/srv/SetNodeState \
  "{node_name: '', transition: 'bringdown'}"

ros2 service call /wmx/lifecycle/set_node_state wmx_r2_message/srv/SetNodeState \
  "{node_name: '', transition: 'bringup'}"
```

### Transitions
| `transition` | Effect |
|--------------|--------|
| `configure` | `unconfigured` -> `inactive`: opens the WMX3 device, creates the interfaces |
| `activate` | `inactive` -> `active`: starts the timers, the node serves requests |
| `deactivate` | `active` -> `inactive`: stops the timers and the motion it owns |
| `cleanup` | `inactive` -> `unconfigured`: destroys the interfaces, closes the device |
| `shutdown` | any state -> `finalized`, whatever the current state is |
| `bringup` | `configure` and `activate` as needed to reach `active` |
| `bringdown` | `deactivate` if active, otherwise a no-op |

### Node Parameters
| Parameter | Default | Meaning |
|-----------|---------|---------|
| `managed_nodes` | `[]` | Restrict the manager to these node names. Empty manages everything it discovers |
| `engine_status_service` | `wmx/engine/get_engine_status` | Service polled for the engine state |
| `require_engine` | `true` | Set `false` to bring nodes up without waiting for the engine |
| `discovery_period` | `2.0` | Seconds between discovery sweeps |

---

## IO Services

### Read Input Bit
```
ros2 service call /wmx/io/get_input_bit wmx_r2_message/srv/GetIoBit "{byte: 0, bit: 0}"
```

### Read Output Bit
```
ros2 service call /wmx/io/get_output_bit wmx_r2_message/srv/GetIoBit "{byte: 0, bit: 0}"
```

### Read Input Bytes
```
ros2 service call /wmx/io/get_input_bytes wmx_r2_message/srv/GetIoBytes "{byte: 0, length: 4}"
```

### Read Output Bytes
```
ros2 service call /wmx/io/get_output_bytes wmx_r2_message/srv/GetIoBytes "{byte: 0, length: 4}"
```

### Set Output Bit
```
# Set bit
ros2 service call /wmx/io/set_output_bit wmx_r2_message/srv/SetIoBit "{byte: 0, bit: 0, value: 1}"

# Clear bit
ros2 service call /wmx/io/set_output_bit wmx_r2_message/srv/SetIoBit "{byte: 0, bit: 0, value: 0}"
```

### Set Output Bytes
```
# Set output byte 2 to 0x0F, byte 3 to 0x0E
ros2 service call /wmx/io/set_output_bytes wmx_r2_message/srv/SetIoBytes "{byte: 2, data: [15, 14]}"
```

---

## EtherCAT Services

### Get Network State
```
# Master 0
ros2 service call /wmx/ecat/get_network_state wmx_r2_message/srv/EcatGetNetworkState "{master_id: 0}"

# Master 1
ros2 service call /wmx/ecat/get_network_state wmx_r2_message/srv/EcatGetNetworkState "{master_id: 1}"
```

### Register Read
```
# 1 byte from slave 0, register 0x000 (type register)
ros2 service call /wmx/ecat/register_read wmx_r2_message/srv/EcatRegisterRead \
  "{master_id: 0, slave_id: 0, reg_address: 0, length: 1}"

# 4 bytes from slave 0, register 0x010 (vendor ID)
ros2 service call /wmx/ecat/register_read wmx_r2_message/srv/EcatRegisterRead \
  "{master_id: 0, slave_id: 0, reg_address: 16, length: 4}"

# 16 bytes from slave 1, register 0x100 (DL status)
ros2 service call /wmx/ecat/register_read wmx_r2_message/srv/EcatRegisterRead \
  "{master_id: 0, slave_id: 1, reg_address: 256, length: 16}"
```

### Scan Network
```
# Re-scan the EtherCAT network of master 0
ros2 service call /wmx/ecat/scan_network wmx_r2_message/srv/EcatScanNetwork "{master_id: 0}"
```

### Reset Statistics
```
# Resets ref-clock, transmit statistics, and re-scans network
ros2 service call /wmx/ecat/reset_statistics wmx_r2_message/srv/EcatResetStatistics "{master_id: 0}"
```

### Start Hotconnect
```
# Enable dynamic slave discovery (call once after network reaches Op state)
ros2 service call /wmx/ecat/start_hotconnect wmx_r2_message/srv/EcatStartHotconnect "{master_id: 0}"
```

---

## Notes

> Standard ROS2 parameter services (get, set, list, describe) are available on all nodes via
> `ros2 param list|get|set <node_name>`.

**General**
- A node's services only exist while it is configured; if `ros2 service list` does not
  show them, check `wmx/lifecycle/get_node_states` first
- Use `ros2 service list` to see all available services
- Use `ros2 service type <service_name>` to verify service types
- `index` and `data` arrays must be the same length

**Axis**
- `data` for `set_on`: `1` = servo on, `0` = servo off
- `data` for `set_mode`: `0` = Position, `1` = Velocity, `2` = Torque
- `data` for `set_polarity`: `1` = normal, `-1` = reversed
- Homing always uses `CurrentPos` type — sets the current encoder position as home (zero)
- `stop` uses `Motion::Stop` (decelerate); `data` is ignored

**Jog**
- Runs `Motion::StartJog`, which requires the axis to be in Position mode (`set_mode` `0`)
- `jog_timeout_ms` (default `200`) — axis stops this long after refreshes stop arriving
- `jog_run_time_ms` (default `2000`) — max duration of a single jog, enforced by the
  engine so the axis still stops if the publishing node dies. Once it elapses the axis
  stays stopped until the operator releases and presses again
- Republishing the same velocity only refreshes the dead-man; it does not re-issue
  `StartJog` (jog-over-jog override is undefined in WMX3)

**WMX Parameters**
- `params/load` requires an absolute path to a valid WMX3 XML file; engine must be ready first
- `params/get` returns a structured dump in `params_dump` per requested axis
- `CommandMode`: `0`=Position, `1`=Velocity, `2`=Torque
- `HomeType`: `0`=CurrentPos, `1`=ZPulse, `2`=HS, `4`=HSZPulse
- `HomeDirection`: `0`=Positive, `1`=Negative

**IO**
- `byte` is the IO byte address; `bit` is the index within that byte (0–7)
- `set_output_bit` value must be `0` or `1`
- `set_output_bytes` data values are decimal (e.g. `15` = `0x0F`)

**EtherCAT**
- Master state values: `None`=0, `Init`=1, `Preop`=2, `Boot`=4, `Safeop`=8, `Op`=16
- Master mode values: `CyclicMode`=0, `PPMode`=1, `MonitorMode`=2
- `reg_address` is a 12-bit ESC register address (decimal), valid range `0x000`–`0xFFF`
- `reg_address + length` must not exceed `0x1000` (4096 bytes)
- `reset_statistics` calls `ResetRefClockInfo` + `ResetTransmitStatisticsInfo` + `ScanNetwork` in sequence
