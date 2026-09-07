# WMX R2 bring-up for the Movensys 4-axis cartesian robot.
#
# Same shape as wmx_r2_cr3a_manipulator.launch.py: no ros2_control anywhere.
# WMX R2 owns the motion, and MoveIt talks to it through a plain
# FollowJointTrajectory action (a ROS action = a long-running request with
# feedback and the option to cancel).
#
#   wmx_r2_general_nodes      : WMX engine, core motion, IO, EtherCAT master
#   joint_state_broadcaster   : WMX encoder positions -> /joint_states
#                               (and -> /joint_command for the Isaac Sim scene)
#   joint_trajectory_controller: FollowJointTrajectory action server -> WMX spline
#   servo-on + homing         : see auto_servo_on / auto_home below
#
# Usage (needs root for the real-time EtherCAT master, see
#        wmx-r2/doc/launch_cartesian.md):
#     ros2 launch wmx_r2_package wmx_r2_cartesian.launch.py use_sim_time:=true   # HiL
#     ros2 launch wmx_r2_package wmx_r2_cartesian.launch.py                      # real
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_share = get_package_share_directory('wmx_r2_package')
    cartesian_config = os.path.join(pkg_share, 'config', 'cartesian_config.yaml')
    wmx_param_file_path = os.path.join(pkg_share, 'config', 'cartesian_wmx_parameters.xml')

    with open(cartesian_config) as f:
        jtc_action = yaml.safe_load(f)[
            'joint_trajectory_controller']['ros__parameters']['joint_trajectory_action']

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
        parameters=[cartesian_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_joint_trajectory_controller = Node(
        package='wmx_r2_package',
        executable='joint_trajectory_controller',
        name='joint_trajectory_controller',
        parameters=[
            cartesian_config,
            {
                'use_sim_time': use_sim_time,
                'wmx_param_file_path': wmx_param_file_path,
            },
        ],
        output='screen',
    )

    # WMX3 rejects any motion command while the drives are in servo-off state
    # ("StartCSplinePos Error: ... One or more axes are not in servo on state"),
    # which surfaces in MoveIt as error code -4 / CONTROL_FAILED. Nothing else
    # in this stack enables the servos, so do it here.
    #
    # wmx/core_motion/ready is a latched (transient_local) Bool published once
    # wmx_core_motion_node has attached to the device; the services exist before
    # that but reject calls with "CoreMotion not initialized", hence the wait.
    #
    # Homing is OFF by default (auto_home:=false), same as the manipulator
    # launches. The cartesian drives run with AbsoluteEncoderMode 1 in
    # cartesian_wmx_parameters.xml, so they remember their position across a
    # power cycle and nothing has to be homed at bring-up. Running
    # `wmx/axis/homing` here would *overwrite* that remembered position with the
    # axis' HomePosition (HomeType CurrentPos = "you are at HomePosition now"),
    # so only pass auto_home:=true on a machine whose drives are NOT absolute,
    # and only when it is parked at the model home pose (X/Y centred, Z at the
    # bottom of its travel). Without a valid position MoveIt aborts every plan
    # with "Joint 'axis_z' from the starting state is outside bounds", because
    # axis_z travels [0.012, 0.090] m and a raw 0.0 is outside that.
    # The one-time position offset of absolute drives (AbsoluteEncoderHomeOffset)
    # is set from WMX Studio and exported into cartesian_wmx_parameters.xml;
    # the ROS side never rewrites coordinates. The manual homing call, for a
    # deliberate re-zero at the model home pose only:
    #   ros2 service call /wmx/axis/homing wmx_r2_message/srv/SetAxis \
    #     "{index: [0,1,2,3], data: [0,0,0,0]}"
    axes = '[0,1,2,3]'
    bringup = rf'''
ros2 topic echo /wmx/core_motion/ready std_msgs/msg/Bool \
  --qos-durability transient_local --qos-reliability reliable --once >/dev/null

# joint_trajectory_controller imports cartesian_wmx_parameters.xml and only then
# creates its action server, so waiting for the action is what guarantees the
# engine already has axis 2's HomePosition when homing runs below.
for _ in $(seq 60); do
  if ros2 action list 2>/dev/null | grep -qx "{jtc_action}"; then break; fi
  sleep 0.5
done

ros2 service call /wmx/axis/clear_alarm wmx_r2_message/srv/SetAxis \
  "{{index: {axes}, data: [0,0,0,0]}}"
ros2 service call /wmx/axis/set_on wmx_r2_message/srv/SetAxis \
  "{{index: {axes}, data: [1,1,1,1]}}"
'''

    home = rf'''
if [ "$AUTO_HOME" = "true" ]; then
  ros2 service call /wmx/axis/homing wmx_r2_message/srv/SetAxis \
    "{{index: {axes}, data: [0,0,0,0]}}"
fi
'''

    servo_on = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('auto_servo_on')),
        cmd=[
            'bash', '-c',
            ['AUTO_HOME=', LaunchConfiguration('auto_home'), '\n', bringup, home],
        ],
        output='screen',
    )

    # No gripper_controller: the cartesian machine has no gripper.
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'auto_servo_on',
            default_value='true',
            description='Clear alarms and enable the servos once CoreMotion is ready',
        ),
        DeclareLaunchArgument(
            'auto_home',
            default_value='false',
            description='Home (HomeType CurrentPos, no motion) after servo-on. Off by '
                        'default: the drives are absolute (AbsoluteEncoderMode 1) and '
                        'homing would overwrite the remembered position. Only for '
                        'non-absolute drives parked at the model home pose. Requires '
                        'auto_servo_on.',
        ),
        start_wmx_r2_general_nodes,
        start_joint_state_broadcaster,
        start_joint_trajectory_controller,
        servo_on,
    ])
