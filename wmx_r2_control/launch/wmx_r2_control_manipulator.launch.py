import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WMX_SHARE = get_package_share_directory('wmx_r2_package')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    wmx_param_file = LaunchConfiguration('wmx_param_file')
    urdf_file = LaunchConfiguration('urdf_file')
    controllers_file = LaunchConfiguration('controllers_file')
    use_gripper = LaunchConfiguration('use_gripper')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_file, ' wmx_param_file:=', wmx_param_file,
                     ' use_gripper:=', use_gripper]),
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
        # The WMX3 SDK (libimdll.so) runs a helper thread with a 16 KB stack. Lazy PLT
        # resolution on that thread overflows it and kills ros2_control_node at random
        # during CreateDevice. Bind every symbol at load time instead.
        additional_env={'LD_BIND_NOW': '1'},
        output='screen',
        emulate_tty=True,
    )

    start_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )

    start_arm_controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['movensys_manipulator_arm_controller',
                   '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )

    start_arm_controller_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=start_joint_state_broadcaster_spawner,
            on_exit=[start_arm_controller_spawner_node],
        )
    )

    start_gripper_controller_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=start_arm_controller_spawner_node,
            on_exit=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=['movensys_manipulator_gripper_controller',
                           '--controller-manager', '/controller_manager'],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
                emulate_tty=True,
            )],
        ),
        condition=IfCondition(use_gripper),
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
            description='Spawn movensys_manipulator_gripper_controller and add the '
                        'picker_1_joint gripper to ros2_control'
        ),

        start_wmx_r2_general_nodes,
        start_robot_state_publisher,
        start_controller_manager,
        start_joint_state_broadcaster_spawner,
        start_arm_controller_spawner,
        start_gripper_controller_spawner,
        start_isaacsim_joint_command_relay,
    ])
