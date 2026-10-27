import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    Command,
    LaunchConfiguration,
    NotSubstitution,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue

WMX_SHARE = get_package_share_directory('wmx_r2_package')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    wmx_param_file = LaunchConfiguration('wmx_param_file')
    urdf_file = LaunchConfiguration('urdf_file')
    controllers_file = LaunchConfiguration('controllers_file')
    stream_position = LaunchConfiguration('stream_position')
    activate_arm_controller = LaunchConfiguration('activate_arm_controller')
    arm_controller = LaunchConfiguration('arm_controller')

    # The stream_position / cyclic_* mappings are read by xacros that declare them
    # (cr3a.wmx.urdf.xacro); xacro ignores mappings a file does not declare.
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', urdf_file,
                ' wmx_param_file:=', wmx_param_file,
                ' stream_position:=', stream_position,
                ' cyclic_max_acc:=', LaunchConfiguration('cyclic_max_acc'),
                ' cyclic_target_periods:=', LaunchConfiguration('cyclic_target_periods'),
                ' cyclic_push_period_ms:=', LaunchConfiguration('cyclic_push_period_ms'),
                ' cyclic_subsample:=', LaunchConfiguration('cyclic_subsample'),
            ]),
            value_type=str,
        )
    }

    start_wmx_r2_general_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(WMX_SHARE, 'launch', 'wmx_r2_general_nodes.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': config_file,
            'wmx_param_file': wmx_param_file,
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
        parameters=[robot_description, controllers_file,
                    {'use_sim_time': use_sim_time}],
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

    # stream_position:=true -- the arm controller (a stock JointTrajectoryController
    # from controllers_file) writes 'position' command interfaces that
    # WmxSystemHardware streams through the WMX3 cyclic buffer.
    start_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[arm_controller, '--controller-manager', '/controller_manager'],
        condition=IfCondition(AndSubstitution(stream_position, activate_arm_controller)),
        output='screen',
        emulate_tty=True,
    )

    # '--inactive' loads and configures the arm controller without activating it,
    # so it claims no command interfaces: the hardware still opens the cyclic
    # buffer and holds position (first validation step on a real arm).
    start_arm_controller_spawner_inactive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[arm_controller, '--controller-manager', '/controller_manager', '--inactive'],
        condition=IfCondition(
            AndSubstitution(stream_position, NotSubstitution(activate_arm_controller))),
        output='screen',
        emulate_tty=True,
    )

    # Superseded by the arm controller when streaming positions: once the cyclic
    # buffer runs, the axes are in DirectControl, so StartCSplinePos /
    # StartLinearIntplPos would be rejected -- and joint_position_controller
    # subscribes to the arm controller's own joint_trajectory topic.
    start_joint_trajectory_controller = LifecycleNode(
        package='wmx_r2_package',
        executable='joint_trajectory_controller',
        name='joint_trajectory_controller',
        namespace='',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        condition=UnlessCondition(stream_position),
        output='screen',
        emulate_tty=True,
    )

    start_joint_position_controller = LifecycleNode(
        package='wmx_r2_package',
        executable='joint_position_controller',
        name='joint_position_controller',
        namespace='',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        condition=UnlessCondition(stream_position),
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
        DeclareLaunchArgument(
            'config_file',
            description='YAML with the manipulator node parameters; '
                        'see the wmx_r2_package example folder'
        ),
        DeclareLaunchArgument(
            'wmx_param_file',
            description='WMX3 parameter XML imported at engine start and read '
                        'by the xacro; see the wmx_r2_package example folder'
        ),
        DeclareLaunchArgument(
            'urdf_file',
            description='Robot description xacro'
        ),
        DeclareLaunchArgument(
            'controllers_file',
            description='ros2_control controller manager YAML'
        ),
        DeclareLaunchArgument(
            'use_gripper',
            default_value='false',
            description='Start gripper_controller'
        ),
        DeclareLaunchArgument(
            'stream_position',
            default_value='false',
            description='Stream joint positions through the WMX3 cyclic buffer, driven by '
                        'the arm controller in controllers_file, instead of the standalone '
                        'joint_trajectory_controller / joint_position_controller nodes. '
                        'Needs a xacro that declares stream_position (cr3a.wmx.urdf.xacro)'
        ),
        DeclareLaunchArgument(
            'activate_arm_controller',
            default_value='true',
            description='With stream_position:=true, activate the arm controller on start. '
                        'false loads it inactive: the hardware holds position with nothing '
                        'commanding it'
        ),
        DeclareLaunchArgument(
            'arm_controller',
            default_value='movensys_manipulator_arm_controller',
            description='Arm controller name in controllers_file (stream_position:=true)'
        ),
        DeclareLaunchArgument(
            'cyclic_subsample',
            default_value='linear',
            description="'linear': one command per write(), split linearly by the engine; "
                        "'hermite': one-cycle samples of the (position, velocity) cubic, the "
                        'arm controller must claim [position, velocity] '
                        '(config/cr3a_controllers_hermite.yaml)'
        ),
        DeclareLaunchArgument(
            'cyclic_max_acc',
            default_value='5.0',
            description='Cyclic buffer acceleration trip [rad/s^2], checked per engine cycle'
        ),
        DeclareLaunchArgument(
            'cyclic_target_periods',
            default_value='2',
            description='write() periods of runway kept queued in the cyclic buffer'
        ),
        DeclareLaunchArgument(
            'cyclic_push_period_ms',
            default_value='10.0',
            description='Seed for the write() period [ms]; keep 1000 / update_rate'
        ),

        start_wmx_r2_general_nodes,
        start_robot_state_publisher,
        start_controller_manager,
        start_joint_state_broadcaster_spawner,
        start_arm_controller_spawner,
        start_arm_controller_spawner_inactive,
        start_joint_trajectory_controller,
        start_joint_position_controller,
        start_gripper_controller,
        start_isaacsim_joint_command_relay,
    ])
