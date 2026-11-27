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
`degree_to_radian_axes: [0, 1, 2, 3, 4, 5]` so `/joint_states`, Isaac Sim and Gazebo still
receive radians.

## Example: Oriental Motor OVR6048K1-V
```
wros ros2 launch wmx_r2_package wmx_r2_robot_option.launch.py \
    use_sim_time:=false \
    'config_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/ovr6048k1-v_robot_option_config.yaml' \
    'wmx_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/ovr6048k1-v_wmx_parameters_deg.xml' \
    'robot_param_file:=$(ros2 pkg prefix --share wmx_r2_package)/example/ovr6048k1-v_robot_option_parameters.xml'
```

The kinematics in `ovr6048k1-v_robot_option_parameters.xml` are transcribed from
`movensys_manipulator_description/urdf/ovr6048k1-v/ovr6048k1-v.xacro`, with the
`robot_joint` mount (`rpy="1.5708 0 0"`) folded into joint 0 so the robot
option's base frame is Z-up. Joint velocity and acceleration limits are 0.5 rad/s
and 0.5 rad/s², the same as the URDF and MoveIt `joint_limits.yaml`; the
Cartesian tool speed is 50 mm/s. Raise them once the arm is validated.
