import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

PKG_SHARE = get_package_share_directory('wmx_r2_package')


def launch_wmx_robot_option_node(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file').perform(context)

    parameters = [config_file] if config_file else []
    parameters.append({'use_sim_time': use_sim_time})

    robot_param_file = LaunchConfiguration('robot_param_file').perform(context)
    if robot_param_file:
        parameters.append({'robot_param_file': robot_param_file})

    return [
        LifecycleNode(
            package='wmx_r2_package',
            executable='wmx_robot_option_node',
            name='wmx_robot_option_node',
            namespace='',
            parameters=parameters,
            output='screen',
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    start_wmx_r2_general_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(PKG_SHARE, 'launch', 'wmx_r2_general_nodes.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_file': LaunchConfiguration('config_file'),
            'wmx_param_file': LaunchConfiguration('wmx_param_file'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                PKG_SHARE, 'config', 'wmx_r2_robot_option_config.yaml'),
            description='YAML with the general node and wmx_robot_option_node parameters'
        ),
        DeclareLaunchArgument(
            'wmx_param_file',
            default_value='',
            description='WMX3 parameter XML imported at engine start; '
                        'empty imports nothing'
        ),
        DeclareLaunchArgument(
            'robot_param_file',
            default_value='',
            description='Robot parameter XML or URDF registered with the WMX3 robot '
                        'option on configure. Overrides robot_param_file from '
                        'config_file; empty registers nothing and leaves the node '
                        'waiting for wmx/robot/set_robot_param'
        ),

        start_wmx_r2_general_nodes,
        OpaqueFunction(function=launch_wmx_robot_option_node),
    ])
