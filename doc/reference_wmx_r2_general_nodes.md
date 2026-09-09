# WMX R2 General Nodes Reference

## Typical Startup Sequence
- Note that user can reduce the axis dimension for their purpose.

```
# 1. Verify engine is communicating
ros2 service call /wmx/engine/get_status std_srvs/srv/Trigger "{}"

# 2. Load axis parameters from file
## 2-1. Predefined robot case
## 2-2. User's own robot or arbitary motors
ros2 service call /wmx/params/load wmx_r2_message/srv/LoadWmxParams \
  "{file_path: '/home/$USER/workspaces/movensys_ws/install/wmx_r2_package/share/wmx_r2_package/config/default_wmx_parameters.xml'}"

## 3. Set Gear Ratio (2-1 stage can skip this stage)
## Panasonic MADLNO5BE servo driver
ros2 service call /wmx/axis/set_gear_ratio wmx_r2_message/srv/SetAxisGearRatio \
  "{index: [0], numerator: [8388608.0], denominator: [360.0]}"

# 4. Clear any amp alarms
ros2 service call /wmx/axis/clear_alarm wmx_r2_message/srv/SetAxis "{index: [0,1,2,3,4,5], data: [0,0,0,0,0,0]}"

# 5. Enable servos
ros2 service call /wmx/axis/set_on wmx_r2_message/srv/SetAxis "{index: [0,1,2,3,4,5], data: [1,1,1,1,1,1]}"

# 6. Home all axes (sets current position as home)
ros2 service call /wmx/axis/homing wmx_r2_message/srv/SetAxis "{index: [0,1,2,3,4,5], data: [0,0,0,0,0,0]}"
```
---
i

## Engine Topics (Warning: These command will rotate the axes)
### Send Axis Absolute Position
```
ros2 topic pub --once /wmx/axis/position wmx_r2_message/msg/AxisPose \
    "{index: [0,1], target: [8388608, 10000 ], velocity: [1000000, 5000], acc: [100000, 1000], dec: [100000, 1000]}" 
```

### Send Axis Relative Position
```
ros2 topic pub --once /wmx/axis/position/relative wmx_r2_message/msg/AxisPose \
    "{index: [0, 1], target: [8388608, 10000], velocity: [1000000, 5000], acc: [100000, 1000], dec: [100000, 1000]}"
```i


### Send Axis Velocity
```
ros2 topic pub --once /wmx/axis/velocity wmx_r2_message/msg/AxisVelocity \
    "{index: [0, 1], velocity: [1000000, 5000], acc: [100000, 1000], dec: [100000, 1000]}"  
```



i
### Jog (hold-to-move)
`/wmx/axis/jog` is a dead-man command: the publisher must keep republishing while
the operator holds the control. The axis is stopped once refreshes stop arriving
(`jog_timeout_ms`). The sign of `velocity` selects the direction.

```
# Jog axis 0 in the positive direction. Ctrl-C acts as the release.
ros2 topic pub -r 20 /wmx/axis/jog wmx_r2_message/msg/AxisVelocity \
    "{index: [0], velocity: [10000], acc: [100000], dec: [100000]}"
i
# Negative direction
ros2 topic pub -r 20 /wmx/axis/jog wmx_r2_message/msg/AxisVelocity \
    "{index: [0], velocity: [-10000], acc: [100000], dec: [100000]}"
```

#### Jog with keyboard teleop
- Increase the sensitivity of keyboard.
- This command should be ran on keyboard connected PC. It can be different via SSH connection.
```
xset r rate 150 30
```

- Run Jog node for keyboard teleop (`a` = negative, `d` = positive, `q` = quit):
```
ros2 run wmx_r2_package jog_keyboard_node --ros-args \
    -p axis:=0 -p velocity:=1000.0 -p acc:=10000.0 -p dec:=100000.0
```

- Rollback to default sensitivity
```
xset r rate 660 25
```
---

## Engine Services
### Get Status
```
ros2 service call /wmx/engine/get_status std_srvs/srv/Trigger "{}"
```

### Set Communication (start/stop EtherCAT comms)
```
# Start communication
ros2 service call /wmx/engine/set_comm std_srvs/srv/SetBool "{data: true}"

# Stop communication
ros2 service call /wmx/engine/set_comm std_srvs/srv/SetBool "{data: false}"
```

### Set Device (create/close WMX3 device)
```
# Create device
ros2 service call /wmx/engine/set_device wmx_r2_message/srv/SetEngine \
  "{data: true, path: '/opt/wmx3/', name: 'my_device'}"

# Close device
ros2 service call /wmx/engine/set_device wmx_r2_message/srv/SetEngine \
  "{data: false, path: '', name: ''}"
```

### Scan Network
```
ros2 service call /wmx/engine/scan_network std_srvs/srv/Trigger "{}"
```

---

## WMX Parameter Services
### Load Parameters from File
```
# Diffbot
ros2 service call /wmx/params/load wmx_r2_message/srv/LoadWmxParams \
  "{file_path: '/home/$USER/movensys_ws/install/wmx_r2_package/share/wmx_r2_package/config/diffbot_wmx_parameters.xml'}"
```

### Get Parameters (inspect active axis config)
```
# Single axis
ros2 service call /wmx/params/get wmx_r2_message/srv/GetWmxParams "{index: [0]}"

# Multiple axes
ros2 service call /wmx/params/get wmx_r2_message/srv/GetWmxParams "{index: [0,1,2,3,4,5]}"
```

---

## Axis Services

### Clear Alarm
```
ros2 service call /wmx/axis/clear_alarm wmx_r2_message/srv/SetAxis "{index: [0,1], data: [0,0]}"
```

### Set Servo On / Off
```
# Servo On
ros2 service call /wmx/axis/set_on wmx_r2_message/srv/SetAxis "{index: [0,1], data: [1,1]}"

# Servo Off
ros2 service call /wmx/axis/set_on wmx_r2_message/srv/SetAxis "{index: [0,1], data: [0,0]}"
```

### Set Command Mode
```
# Position mode (0)
ros2 service call /wmx/axis/set_mode wmx_r2_message/srv/SetAxis "{index: [0,1], data: [0,0]}"

# Velocity mode (1)
ros2 service call /wmx/axis/set_mode wmx_r2_message/srv/SetAxis "{index: [0,1], data: [1,1]}"
```

### Set Polarity
```
# Normal (1)
ros2 service call /wmx/axis/set_polarity wmx_r2_message/srv/SetAxis "{index: [0,1], data: [1,1]}"

# Reversed (-1)
ros2 service call /wmx/axis/set_polarity wmx_r2_message/srv/SetAxis "{index: [0,1], data: [-1,-1]}"
```

### Set Gear Ratio
```
ros2 service call /wmx/axis/set_gear_ratio wmx_r2_message/srv/SetAxisGearRatio \
  "{index: [0,1], numerator: [1.0,1.0], denominator: [1.0,1.0]}"
```



### Homing (sets current position as home)
```
ros2 service call /wmx/axis/homing wmx_r2_message/srv/SetAxis "{index: [0,1], data: [0,0]}"
```

### Stop (decelerate to a stop)
```
ros2 service call /wmx/axis/stop wmx_r2_message/srv/SetAxis "{index: [0,1], data: [0,0]}"
```

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
