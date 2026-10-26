import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node

EXAMPLE_CONFIG = os.path.join(
    get_package_share_directory('wmx_r2_package'),
    'config',
    'wmx_r2_general_nodes_config.yaml',
)


def launch_general_nodes(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    engine_config = [LaunchConfiguration('config_file').perform(context)]

    engine_params = [{'use_sim_time': use_sim_time}]
    wmx_param_file = LaunchConfiguration('wmx_param_file').perform(context)
    if wmx_param_file:
        engine_params.append({'wmx_param_file_path': wmx_param_file})

    start_wmx_engine_node = Node(
        package='wmx_r2_package',
        executable='wmx_engine_node',
        name='wmx_engine_node',
        parameters=engine_config + engine_params,
        output='screen',
        emulate_tty=True,
    )

    start_wmx_lifecycle_manager_node = Node(
        package='wmx_r2_package',
        executable='wmx_lifecycle_manager_node',
        name='wmx_lifecycle_manager_node',
        parameters=engine_config + [{'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )

    start_wmx_core_motion_node = LifecycleNode(
        package='wmx_r2_package',
        executable='wmx_core_motion_node',
        name='wmx_core_motion_node',
        namespace='',
        parameters=engine_config + [{'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )

    start_wmx_io_node = LifecycleNode(
        package='wmx_r2_package',
        executable='wmx_io_node',
        name='wmx_io_node',
        namespace='',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )

    start_wmx_ethercat_node = LifecycleNode(
        package='wmx_r2_package',
        executable='wmx_ethercat_node',
        name='wmx_ethercat_node',
        namespace='',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )

    return [
        start_wmx_engine_node,
        start_wmx_lifecycle_manager_node,
        start_wmx_core_motion_node,
        start_wmx_io_node,
        start_wmx_ethercat_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=EXAMPLE_CONFIG,
            description='YAML with the wmx_engine_node and '
                        'wmx_lifecycle_manager_node parameters. Defaults to '
                        'config/wmx_r2_general_nodes_config.yaml, an example '
                        'holding the node defaults'
        ),
        DeclareLaunchArgument(
            'wmx_param_file',
            default_value='',
            description='Absolute path to the WMX3 parameter XML imported at '
                        'engine start. Overrides wmx_param_file_path from '
                        'config_file. The robot launch files pass their own '
                        'config/<robot>_wmx_parameters.xml; empty imports nothing'
        ),

        OpaqueFunction(function=launch_general_nodes),
    ])
