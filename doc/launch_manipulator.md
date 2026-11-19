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

## Example: Nachi MZ07
```
wros ros2 launch wmx_r2_package wmx_r2_manipulator.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/nachi_mz07_manipulator_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/nachi_mz07_wmx_parameters.xml'
```

## Example: Nachi MZ07 with ROS2 Control
```
wros ros2 launch wmx_r2_control wmx_r2_control_manipulator.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/nachi_mz07_manipulator_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/nachi_mz07_wmx_parameters.xml' \
    'urdf_file:=$(ros2 pkg prefix --share wmx_r2_control)/urdf/nachi_mz07.wmx.urdf.xacro' \
    'controllers_file:=$(ros2 pkg prefix --share wmx_r2_control)/config/nachi_mz07_controllers.yaml'
```
