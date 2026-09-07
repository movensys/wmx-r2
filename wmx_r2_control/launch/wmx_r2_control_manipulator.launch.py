import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue

CTRL_SHARE = get_package_share_directory('wmx_r2_control')
WMX_SHARE = get_package_share_directory('wmx_r2_package')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    wmx_param_file = LaunchConfiguration('wmx_param_file')
    urdf_file = LaunchConfiguration('urdf_file')
    controllers_file = LaunchConfiguration('controllers_file')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_file, ' wmx_param_file:=', wmx_param_file]),
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

    start_joint_trajectory_controller = LifecycleNode(
        package='wmx_r2_package',
        executable='joint_trajectory_controller',
        name='joint_trajectory_controller',
        namespace='',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )

    start_joint_position_controller = LifecycleNode(
        package='wmx_r2_package',
        executable='joint_position_controller',
        name='joint_position_controller',
        namespace='',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
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
            description='YAML with the manipulator node parameters, e.g. '
                        'wmx_r2_package example/cr3a_manipulator_config.yaml'
        ),
        DeclareLaunchArgument(
            'wmx_param_file',
            description='WMX3 parameter XML imported at engine start and read '
                        'by the xacro, e.g. wmx_r2_package '
                        'example/cr3a_wmx_parameters.xml'
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=os.path.join(CTRL_SHARE, 'urdf', 'cr3a.wmx.urdf.xacro'),
            description='Robot description xacro'
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=os.path.join(CTRL_SHARE, 'config', 'cr3a_controllers.yaml'),
            description='ros2_control controller manager YAML'
        ),
        DeclareLaunchArgument(
            'use_gripper',
            default_value='false',
            description='Start gripper_controller'
        ),

        start_wmx_r2_general_nodes,
        start_robot_state_publisher,
        start_controller_manager,
        start_joint_state_broadcaster_spawner,
        start_joint_trajectory_controller,
        start_joint_position_controller,
        start_gripper_controller,
        start_isaacsim_joint_command_relay,
    ])
