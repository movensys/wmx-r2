# Launch WMX R2 Robot Option
## Example: Dobot CR3A
```
wros ros2 launch wmx_r2_package wmx_r2_robot_option.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr3a_robot_option_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr3a_wmx_parameters_deg.xml' \
    'robot_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/cr3a_robot_option_parameters.xml'
```

Note the axis file: the robot option needs **degree/mm** axes, so this stack uses
`cr3a_wmx_parameters_deg.xml` (`AxisGearRatioDenominator` 360), not the 2*pi file
the MoveIt stack runs on. `cr3a_robot_option_config.yaml` sets
`degree_axes: [0, 1, 2, 3, 4, 5]` so `/joint_states`, Isaac Sim and Gazebo still
receive radians.
