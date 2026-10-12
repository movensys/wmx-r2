import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ctrl_share = get_package_share_directory('wmx_r2_control')
    wmx_share = get_package_share_directory('wmx_r2_package')

    urdf_xacro = os.path.join(ctrl_share, 'urdf', 'synustech.wmx.urdf.xacro')
    controllers = os.path.join(ctrl_share, 'config', 'synustech_controllers.yaml')
    # See wmx_r2_synustech_manipulator.launch.py: pass the _sim variant when the
    # WMX simulation platform is active.
    wmx_param_file = LaunchConfiguration(
        'wmx_param_file',
        default=os.path.join(wmx_share, 'config', 'synustech_wmx_parameters.xml'),
    )
    manipulator_config = os.path.join(wmx_share, 'config', 'synustech_manipulator_config.yaml')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_xacro, ' wmx_param_file:=', wmx_param_file]),
            value_type=str,
        )
    }

    general_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wmx_share, 'launch', 'wmx_r2_general_nodes.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, controllers, {'use_sim_time': use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    joint_trajectory_controller = Node(
        package='wmx_r2_package',
        executable='joint_trajectory_controller',
        name='joint_trajectory_controller',
        parameters=[
            manipulator_config,
            {
                'use_sim_time': use_sim_time,
                'wmx_param_file_path': wmx_param_file,
            },
        ],
        output='screen',
    )

    joint_position_controller = Node(
        package='wmx_r2_package',
        executable='joint_position_controller',
        name='joint_position_controller',
        parameters=[manipulator_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    isaacsim_joint_command_relay = Node(
        package='topic_tools',
        executable='relay',
        name='isaacsim_joint_command_relay',
        arguments=['/joint_states', '/isaacsim/joint_command'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'wmx_param_file',
            default_value=os.path.join(wmx_share, 'config', 'synustech_wmx_parameters.xml'),
            description=(
                'WMX3 axis parameter XML. Use synustech_wmx_parameters_sim.xml '
                'with the WMX simulation platform.'
            ),
        ),
        general_nodes,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller,
        joint_position_controller,
        isaacsim_joint_command_relay,
    ])
