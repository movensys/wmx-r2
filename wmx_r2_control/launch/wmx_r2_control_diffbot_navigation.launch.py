import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CTRL_SHARE = get_package_share_directory('wmx_r2_control')
WMX_SHARE = get_package_share_directory('wmx_r2_package')

URDF_XACRO = os.path.join(CTRL_SHARE, 'urdf', 'diffbot.wmx.urdf.xacro')
CONTROLLERS = os.path.join(CTRL_SHARE, 'config', 'diffbot_controllers.yaml')
DIFFBOT_CONFIG = os.path.join(WMX_SHARE, 'config', 'diffbot_navigation_config.yaml')
DIFFBOT_WMX_PARAM_FILE = os.path.join(WMX_SHARE, 'config', 'diffbot_wmx_parameters.xml')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', URDF_XACRO, ' wmx_param_file:=', DIFFBOT_WMX_PARAM_FILE]),
            value_type=str,
        )
    }

    start_wmx_r2_general_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(WMX_SHARE, 'launch', 'wmx_r2_general_nodes.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': DIFFBOT_CONFIG,
            'wmx_param_file': DIFFBOT_WMX_PARAM_FILE,
        }.items(),
    )

    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )

    start_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, CONTROLLERS, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/differential_drive_controller/cmd_vel', '/cmd_vel_safe'),
            ('/differential_drive_controller/odom', '/odom_enc'),
        ],
        output='screen',
        emulate_tty=True,
    )

    start_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
        emulate_tty=True,
    )

    start_differential_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['differential_drive_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
        emulate_tty=True,
    )

    # Mirror the actual wheel joint states to Isaac Sim's joint-command topic so
    # the digital twin follows the real/WMX hardware.
    start_isaacsim_joint_command_relay = Node(
        package='topic_tools',
        executable='relay',
        name='isaacsim_joint_command_relay',
        arguments=['/joint_states', '/isaacsim/joint_command'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        start_wmx_r2_general_nodes,
        start_robot_state_publisher,
        start_controller_manager,
        start_joint_state_broadcaster_spawner,
        start_differential_drive_controller_spawner,
        start_isaacsim_joint_command_relay,
    ])
