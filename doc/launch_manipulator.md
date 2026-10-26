# Launch WMX R2 Manipulator
## Example: Dobot CR3A
```
wros ros2 launch wmx_r2_package wmx_r2_manipulator.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr3a_manipulator_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr3a_wmx_parameters.xml' \
    use_gripper:=true
```

## Example: Dobot CR3A with ROS2 Control
```
wros ros2 launch wmx_r2_control wmx_r2_control_manipulator.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr3a_manipulator_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr3a_wmx_parameters.xml' \
    'urdf_file:=$(ros2 pkg prefix --share wmx_r2_control)/urdf/cr3a.wmx.urdf.xacro' \
    'controllers_file:=$(ros2 pkg prefix --share wmx_r2_control)/config/cr3a_controllers.yaml' \
    use_gripper:=true
```

## Example: Dobot CR5A
```
wros ros2 launch wmx_r2_package wmx_r2_manipulator.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr5a_manipulator_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr5a_wmx_parameters.xml'
```

## Example: Dobot CR5A with ROS2 Control
```
wros ros2 launch wmx_r2_control wmx_r2_control_manipulator.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr5a_manipulator_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr5a_wmx_parameters.xml' \
    'urdf_file:=$(ros2 pkg prefix --share wmx_r2_control)/urdf/cr5a.wmx.urdf.xacro' \
    'controllers_file:=$(ros2 pkg prefix --share wmx_r2_control)/config/cr5a_controllers.yaml'
```

## Example: Dobot CR3A on the buffered Servo path
`use_api_buffer:=true` swaps `joint_position_controller` for
`servo_stream_controller`, which records MoveIt Servo setpoints into the WMX3
API buffer. See
[launch_dobot_cr3a_manipulator_streamer.md](launch_dobot_cr3a_manipulator_streamer.md)
and [reference_servo_stream_controller.md](reference_servo_stream_controller.md).
```
wros ros2 launch wmx_r2_package wmx_r2_manipulator.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr3a_manipulator_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr3a_wmx_parameters.xml' \
    use_gripper:=true use_api_buffer:=true
```

## Example: Synustech (SIASUN DUCO GCR30-1100)
The arm sits on WMX axes 2 to 7 of a mobile base; see
[launch_synustech_manipulator.md](launch_synustech_manipulator.md) for the
parameter file to use on the WMX simulation platform.
```
wros ros2 launch wmx_r2_package wmx_r2_manipulator.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/synustech_manipulator_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/synustech_wmx_parameters.xml'
```

## Example: Synustech with ROS2 Control
```
wros ros2 launch wmx_r2_control wmx_r2_control_manipulator.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/synustech_manipulator_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/synustech_wmx_parameters.xml' \
    'urdf_file:=$(ros2 pkg prefix --share wmx_r2_control)/urdf/synustech.wmx.urdf.xacro' \
    'controllers_file:=$(ros2 pkg prefix --share wmx_r2_control)/config/synustech_controllers.yaml'
```
