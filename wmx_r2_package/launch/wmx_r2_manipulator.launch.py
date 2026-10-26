import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node

PKG_SHARE = get_package_share_directory('wmx_r2_package')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    wmx_param_file = LaunchConfiguration('wmx_param_file')
    # Selects which node consumes MoveIt Servo's joint_trajectory output.
    # Exactly one may run: both subscribe to the same topic and would otherwise
    # double-command the arm.
    use_api_buffer = LaunchConfiguration('use_api_buffer')

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

    start_joint_trajectory_controller = LifecycleNode(
        package='wmx_r2_package',
        executable='joint_trajectory_controller',
        name='joint_trajectory_controller',
        namespace='',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )

    # Direct path: one StartLinearIntplPos per Servo message, issued from the
    # ROS callback. The default.
    start_joint_position_controller = LifecycleNode(
        package='wmx_r2_package',
        executable='joint_position_controller',
        name='joint_position_controller',
        namespace='',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_api_buffer),
        output='screen',
        emulate_tty=True,
    )

    # Buffered path: Servo setpoints are recorded into the WMX3 API buffer and
    # replayed under the RT OS at cycle boundaries. A plain node: it polls
    # wmx/engine/get_engine_status itself and is not lifecycle-managed.
    start_servo_stream_controller = Node(
        package='wmx_r2_package',
        executable='servo_stream_controller',
        name='servo_stream_controller',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_api_buffer),
        output='screen',
        emulate_tty=True,
    )

    start_gripper_controller = LifecycleNode(
        package='wmx_r2_package',
        executable='gripper_controller',
        name='gripper_controller',
        namespace='',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_gripper')),
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
            description='YAML with the manipulator node parameters; '
                        'see the example folder'
        ),
        DeclareLaunchArgument(
            'wmx_param_file',
            default_value='',
            description='WMX3 parameter XML imported at engine start; '
                        'see the example folder, empty imports nothing'
        ),
        DeclareLaunchArgument(
            'use_gripper',
            default_value='false',
            description='Start gripper_controller'
        ),
        DeclareLaunchArgument(
            'use_api_buffer',
            default_value='false',
            description='Stream MoveIt Servo commands through the WMX3 API '
                        'buffer (servo_stream_controller) instead of calling '
                        'StartLinearIntplPos directly (joint_position_controller). '
                        'Exactly one of the two runs'
        ),

        start_wmx_r2_general_nodes,
        start_joint_state_broadcaster,
        start_joint_trajectory_controller,
        start_joint_position_controller,
        start_servo_stream_controller,
        start_gripper_controller,
    ])
