import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_share = get_package_share_directory('wmx_r2_package')
    diffbot_config = os.path.join(pkg_share, 'config', 'diffbot_navigation_config.yaml')
    wmx_param_file = os.path.join(pkg_share, 'config', 'diffbot_wmx_parameters.xml')

    start_wmx_r2_general_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'wmx_r2_general_nodes.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': diffbot_config,
            'wmx_param_file': wmx_param_file,
        }.items(),
    )

    start_joint_state_broadcaster = LifecycleNode(
        package='wmx_r2_package',
        executable='joint_state_broadcaster',
        name='joint_state_broadcaster',
        namespace='',
        parameters=[diffbot_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_differential_drive_controller = LifecycleNode(
        package='wmx_r2_package',
        executable='differential_drive_controller',
        name='differential_drive_controller',
        namespace='',
        parameters=[diffbot_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        start_wmx_r2_general_nodes,
        start_joint_state_broadcaster,
        start_differential_drive_controller,
    ])
