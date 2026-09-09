import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_share = get_package_share_directory('wmx_r2_package')
    cartesian_config = os.path.join(pkg_share, 'config', 'cartesian_config.yaml')
    wmx_param_file_path = os.path.join(pkg_share, 'config', 'cartesian_wmx_parameters.xml')

    with open(cartesian_config) as f:
        jtc_action = yaml.safe_load(f)[
            'joint_trajectory_controller']['ros__parameters']['joint_trajectory_action']

    start_wmx_r2_general_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'wmx_r2_general_nodes.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    start_joint_state_broadcaster = Node(
        package='wmx_r2_package',
        executable='joint_state_broadcaster',
        name='joint_state_broadcaster',
        parameters=[cartesian_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_joint_trajectory_controller = Node(
        package='wmx_r2_package',
        executable='joint_trajectory_controller',
        name='joint_trajectory_controller',
        parameters=[
            cartesian_config,
            {
                'use_sim_time': use_sim_time,
                'wmx_param_file_path': wmx_param_file_path,
            },
        ],
        output='screen',
    )

    axes = '[0,1,2,3]'
    bringup = rf'''
ros2 topic echo /wmx/core_motion/ready std_msgs/msg/Bool \
  --qos-durability transient_local --qos-reliability reliable --once >/dev/null

for _ in $(seq 60); do
  if ros2 action list 2>/dev/null | grep -qx "{jtc_action}"; then break; fi
  sleep 0.5
done

ros2 service call /wmx/axis/clear_alarm wmx_r2_message/srv/SetAxis \
  "{{index: {axes}, data: [0,0,0,0]}}"
ros2 service call /wmx/axis/set_on wmx_r2_message/srv/SetAxis \
  "{{index: {axes}, data: [1,1,1,1]}}"
'''

    home = rf'''
if [ "$AUTO_HOME" = "true" ]; then
  ros2 service call /wmx/axis/homing wmx_r2_message/srv/SetAxis \
    "{{index: {axes}, data: [0,0,0,0]}}"
fi
'''

    servo_on = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('auto_servo_on')),
        cmd=[
            'bash', '-c',
            ['AUTO_HOME=', LaunchConfiguration('auto_home'), '\n', bringup, home],
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_servo_on', default_value='true'),
        DeclareLaunchArgument('auto_home', default_value='false'),
        start_wmx_r2_general_nodes,
        start_joint_state_broadcaster,
        start_joint_trajectory_controller,
        servo_on,
    ])
