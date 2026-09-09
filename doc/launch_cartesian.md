# Movensys 4-axis cartesian robot

## Launch (root: real-time EtherCAT master)
```
sudo --preserve-env=PATH \
     --preserve-env=AMENT_PREFIX_PATH \
     --preserve-env=COLCON_PREFIX_PATH \
     --preserve-env=PYTHONPATH \
     --preserve-env=LD_LIBRARY_PATH \
     --preserve-env=ROS_DISTRO \
     --preserve-env=ROS_VERSION \
     --preserve-env=ROS_PYTHON_VERSION \
     --preserve-env=ROS_DOMAIN_ID \
     --preserve-env=RMW_IMPLEMENTATION \
     bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && source $HOME/workspaces/movensys_ws/install/setup.bash && \
     ros2 launch wmx_r2_package wmx_r2_cartesian.launch.py use_sim_time:=false"
```
`use_sim_time:=true` for HiL.

| arg | default | |
|---|---|---|
| `auto_servo_on` | `true` | clear alarms + servo on |
| `auto_home` | `false` | `HomeType CurrentPos` homing. Keep off: drives are absolute, homing overwrites the remembered position |

## Axis map

| URDF joint | WMX axis | travel (m) |
|---|---|---|
| `axis_x` | 0 | [-0.115, 0.100] |
| `axis_y` | 1 | [-0.100, 0.100] |
| `axis_z` | 2 | [ 0.012, 0.090] |
| `axis_r` | 3 | continuous (rad) |

`joint_trajectory_controller` maps columns by position: `joint_axes: [0, 1, 2]` =
`axis_x, axis_y, axis_z`. `move_xyz` (PTP only) sends that order. Do not execute
LIN/CIRC from RViz: they emit y, x, z.

## Units
WMX user unit = URDF unit (m / rad). Gear ratio in `config/cartesian_wmx_parameters.xml`:
`1000000 / 1` (1 pulse = 1 um). Replace with the real machine's export before
running on hardware.

## One-time zero (also re-zero, or after a gear ratio change)
1. Park at the model home pose (X/Y centred, Z at bottom).
2. ```
   ros2 service call /wmx/axis/homing wmx_r2_message/srv/SetAxis \
     "{index: [0,1,2,3], data: [0,0,0,0]}"
   ```
3. Export parameters from WMX Studio over `config/cartesian_wmx_parameters.xml`.

If skipped, MoveIt aborts every plan:
```
[check_start_state_bounds]: Joint 'axis_z' from the starting state is outside
bounds by: [0 ] should be in the range [0.012 ], [0.09 ].
```

## Isaac Sim (HiL)
Scene subscribes `/joint_command`, must not publish `/joint_states`.
```
ros2 topic info /joint_states     # publisher count must be 1
```
