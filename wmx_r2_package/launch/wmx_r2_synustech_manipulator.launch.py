import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # Selects which node consumes MoveIt Servo's joint_trajectory output.
    # Exactly one may run: both subscribe to the same topic and would otherwise
    # double-command the arm.
    use_api_buffer = LaunchConfiguration('use_api_buffer', default='false')

    pkg_share = get_package_share_directory('wmx_r2_package')
    manipulator_config = os.path.join(pkg_share, 'config', 'synustech_manipulator_config.yaml')
    wmx_param_file_path = os.path.join(pkg_share, 'config', 'synustech_wmx_parameters.xml')

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
        parameters=[manipulator_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_joint_trajectory_controller = Node(
        package='wmx_r2_package',
        executable='joint_trajectory_controller',
        name='joint_trajectory_controller',
        parameters=[
            manipulator_config,
            {
                'use_sim_time': use_sim_time,
                'wmx_param_file_path': wmx_param_file_path,
            },
        ],
        output='screen',
    )

    # Direct path: one StartLinearIntplPos per Servo message, issued from the
    # ROS callback. The default.
    start_joint_position_controller = Node(
        package='wmx_r2_package',
        executable='joint_position_controller',
        name='joint_position_controller',
        parameters=[manipulator_config, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=UnlessCondition(use_api_buffer),
    )

    # Buffered path: Servo setpoints are recorded into the WMX3 API buffer and
    # replayed under the RT OS at cycle boundaries.
    start_servo_stream_controller = Node(
        package='wmx_r2_package',
        executable='servo_stream_controller',
        name='servo_stream_controller',
        parameters=[manipulator_config, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_api_buffer),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'use_api_buffer',
            default_value='false',
            description=(
                'Stream MoveIt Servo commands through the WMX3 API buffer '
                '(servo_stream_controller) instead of calling StartLinearIntplPos '
                'directly (joint_position_controller). Exactly one of the two runs.'
            ),
        ),
        start_wmx_r2_general_nodes,
        start_joint_state_broadcaster,
        start_joint_trajectory_controller,
        start_joint_position_controller,
        start_servo_stream_controller,
    ])
