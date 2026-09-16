# Launch WMX R2 Differential
## Example: Diffbot
```
wros ros2 launch wmx_r2_package wmx_r2_differential.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/diffbot_differential_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/diffbot_wmx_parameters.xml'
```

## Example: Diffbot with ROS2 Control
```
wros ros2 launch wmx_r2_control wmx_r2_control_differential.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/diffbot_differential_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/diffbot_wmx_parameters.xml' \
    'urdf_file:=$(ros2 pkg prefix --share wmx_r2_control)/urdf/diffbot.wmx.urdf.xacro' \
    'controllers_file:=$(ros2 pkg prefix --share wmx_r2_control)/config/diffbot_controllers.yaml'
```
