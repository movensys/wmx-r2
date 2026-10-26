# Synustech manipulator (SIASUN DUCO GCR30-1100)

The Synustech robot is a mobile manipulator. WMX axes 0 and 1 are the drive
wheels of the base; the six arm joints are WMX axes **2 to 7**
(`joint_axes: [2, 3, 4, 5, 6, 7]`). The commands below bring up the arm only,
through the generic manipulator launch files with the Synustech example files.
The base axes are left servo-off.

`example/synustech_wmx_parameters.xml` is the vendor's WMX3 export of the whole
robot with the arm axes changed from degrees to radians. The DUCO joint drives
report output-side position, 524288 counts per joint revolution, so the gear
ratio is 524288 / 2π with no reducer term, unlike the Dobot files. Home offsets
are in pulses and unchanged from the export; the WMX zero is the DUCO
straight-up pose, which is also the URDF zero.

On the MoveIt side use `export MANIPULATOR_MODEL=synustech`. For the first
runs on hardware set `vel_scale` and `acc_scale` in
`config/synustech/moveit2_client.yaml` to 0.1.

# Simulation platform (no hardware)
When `/opt/wmx3/Module.ini` has the simu platform enabled, the virtual
encoders read zero pulses and the real file's home offsets would place the
joints at -5.68, 0.57, -6.83, -4.35, -4.37, 1.90 rad, outside the URDF
limits. Use the `_sim` parameter file, which zeroes the offsets on axes 2 to 7
and is otherwise identical:
```
wros ros2 launch wmx_r2_package wmx_r2_manipulator.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/synustech_manipulator_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/synustech_wmx_parameters_sim.xml'
```
Never load the `_sim` file on the real robot: the zero would then be wherever
the encoders happen to read, not the DUCO straight-up pose.

# Trajectory control
`wros` runs the command as root with ROS and the workspace sourced; see
[first_setup.md](first_setup.md). Add `use_api_buffer:=true` to stream MoveIt
Servo through the WMX3 API buffer instead of `joint_position_controller`.
```
wros ros2 launch wmx_r2_package wmx_r2_manipulator.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/synustech_manipulator_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/synustech_wmx_parameters.xml'
```


# ROS2 Control (joint state relay only)
```
wros ros2 launch wmx_r2_control wmx_r2_control_manipulator.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/synustech_manipulator_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/synustech_wmx_parameters.xml' \
    'urdf_file:=$(ros2 pkg prefix --share wmx_r2_control)/urdf/synustech.wmx.urdf.xacro' \
    'controllers_file:=$(ros2 pkg prefix --share wmx_r2_control)/config/synustech_controllers.yaml'
```

# Shutdown
Ctrl-C the launch, then servo off the arm axes:
```
ros2 service call /wmx/axes/set_servo_on wmx_r2_message/srv/SetAxes "{axis: [2,3,4,5,6,7], data: [0,0,0,0,0,0]}"
```
Do not call `/wmx/axes/start_home` on this arm. All six axes use absolute
encoders and homing would overwrite the zero with the current position.
