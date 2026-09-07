import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

PKG_SHARE = get_package_share_directory('wmx_r2_package')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    wmx_param_file = LaunchConfiguration('wmx_param_file')

    start_wmx_r2_general_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(PKG_SHARE, 'launch', 'wmx_r2_general_nodes.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': config_file,
            'wmx_param_file': wmx_param_file,
        }.items(),
    )

    start_joint_state_broadcaster = LifecycleNode(
        package='wmx_r2_package',
        executable='joint_state_broadcaster',
        name='joint_state_broadcaster',
        namespace='',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )

    start_differential_drive_controller = LifecycleNode(
        package='wmx_r2_package',
        executable='differential_drive_controller',
        name='differential_drive_controller',
        namespace='',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'config_file',
            description='YAML with the differential node parameters; '
                        'see the example folder'
        ),
        DeclareLaunchArgument(
            'wmx_param_file',
            default_value='',
            description='WMX3 parameter XML imported at engine start; '
                        'see the example folder, empty imports nothing'
        ),

        start_wmx_r2_general_nodes,
        start_joint_state_broadcaster,
        start_differential_drive_controller,
    ])
