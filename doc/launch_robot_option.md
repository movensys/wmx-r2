# Launch WMX R2 Robot Option
## Example: Dobot CR3A
```
wros ros2 launch wmx_r2_package wmx_r2_robot_option.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/config/wmx_r2_robot_option_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr3a_wmx_parameters.xml' \
    'robot_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr3a_robot_option_parameters.xml'
```

