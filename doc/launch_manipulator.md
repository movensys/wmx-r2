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
The ros2_control launch needs the `*_control_manipulator_config.yaml` node config (not
`*_manipulator_config.yaml`): it lists `movensys_manipulator_arm_controller` in
`motion_controllers` and leaves the native trajectory/position controllers out of `managed_nodes`.
```
wros ros2 launch wmx_r2_control wmx_r2_control_manipulator.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr3a_control_manipulator_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr3a_wmx_parameters.xml' \
    'urdf_file:=$(ros2 pkg prefix --share wmx_r2_control)/urdf/cr3a.wmx.urdf.xacro' \
    'controllers_file:=$(ros2 pkg prefix --share wmx_r2_control)/config/cr3a_controllers.yaml' \
    use_gripper:=true
```

Passthrough arm controller (one WMX CSpline per goal, like native): same command with
```
    'controllers_file:=$(ros2 pkg prefix --share wmx_r2_control)/config/cr3a_passthrough_controllers.yaml' \
```

Startup order, like the native lifecycle gate: `ros2_control_node` starts `WmxSystemHardware`
unconfigured (`hardware_components_initial_state.unconfigured`), `wmx_hardware_bringup` polls
`wmx/engine/get_engine_status` every 2 s until it answers `Communicating`, then calls
`/controller_manager/set_hardware_component_state` to `inactive` (CreateDevice) and `active`
(SetServoOn, CreateSplineBuffer) and exits; the controller spawners run after that exit.

Which ros2_control controller for which comparison:

| Comparison | `controllers_file` |
|---|---|
| trajectory_test, official JTC (streams `StartLinearIntplPos` at 100 Hz) | `cr3a_controllers.yaml` |
| trajectory_test, same WMX API as native (`StartCSplinePos`) | `cr3a_passthrough_controllers.yaml` |
| JPC-style stream input (Servo / `stream_cmd`) | `cr3a_controllers.yaml`: official JTC topic `/movensys_manipulator_arm_controller/joint_trajectory` (same topic and message as native `joint_position_controller`) |

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
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr5a_control_manipulator_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr5a_wmx_parameters.xml' \
    'urdf_file:=$(ros2 pkg prefix --share wmx_r2_control)/urdf/cr5a.wmx.urdf.xacro' \
    'controllers_file:=$(ros2 pkg prefix --share wmx_r2_control)/config/cr5a_controllers.yaml'
```
