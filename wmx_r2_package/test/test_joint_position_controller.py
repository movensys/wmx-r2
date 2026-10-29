"""Integration test: joint_position_controller parameters and trajectory input."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from lifecycle_msgs.srv import GetState
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
TRAJECTORY_TOPIC = '/movensys_manipulator_arm_controller/joint_trajectory'


def wait_for_subscription(node, pub, timeout_sec):
    """Spin until the controller subscribes to pub's topic, or the timeout expires."""
    end_time = node.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
    while node.get_clock().now() < end_time:
        rclpy.spin_once(node, timeout_sec=0.5)
        if pub.get_subscription_count() > 0:
            return True
    return False


def wait_until_active(node, target, timeout_sec=60.0):
    """Spin until `target` reports the active lifecycle state, or time out."""
    client = node.create_client(GetState, f'{target}/get_state')
    if not client.wait_for_service(timeout_sec=20):
        return False

    end_time = time.monotonic() + timeout_sec
    while time.monotonic() < end_time:
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        result = future.result()
        if result is not None and result.current_state.label == 'active':
            return True
        time.sleep(1.0)

    return False


@pytest.mark.launch_test
def generate_test_description():
    engine_node = launch_ros.actions.Node(
        package='wmx_r2_package',
        executable='wmx_engine_node',
        name='wmx_engine_node',
        output='screen',
    )

    manager_node = launch_ros.actions.Node(
        package='wmx_r2_package',
        executable='wmx_lifecycle_manager_node',
        name='wmx_lifecycle_manager_node',
        output='screen',
    )
    joint_position_controller = launch_ros.actions.LifecycleNode(
        package='wmx_r2_package',
        executable='joint_position_controller',
        name='joint_position_controller',
        namespace='',
        parameters=[{
            'joint_axes': [0, 1, 2, 3, 4, 5],
            'joint_name': JOINT_NAMES,
            'joint_trajectory_topic': TRAJECTORY_TOPIC,
            'accel_ratio': 0.5,
            'default_velocity': 0.1,
            'min_step': 0.1,
        }],
        output='screen',
    )

    return launch.LaunchDescription([
        engine_node,
        manager_node,
        joint_position_controller,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'engine_node': engine_node,
        'joint_position_controller': joint_position_controller,
    }


class TestJointPositionController(unittest.TestCase):
    """Test joint_position_controller parameters and trajectory subscription."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        probe = Node('test_joint_position_controller_probe')
        try:
            cls.node_active = wait_until_active(probe, 'joint_position_controller')
        finally:
            probe.destroy_node()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_joint_position_controller')

    def tearDown(self):
        self.node.destroy_node()

    def test_parameters_declared(self):
        """Node should declare its motion parameters with the launched values."""
        client = self.node.create_client(
            GetParameters, '/joint_position_controller/get_parameters')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'joint_position_controller parameter service not available',
        )

        req = GetParameters.Request()
        req.names = ['accel_ratio', 'default_velocity', 'min_step', 'joint_trajectory_topic']
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        result = future.result()
        self.assertIsNotNone(result, 'Parameter service call returned no result')
        self.assertEqual(len(result.values), 4, 'Not every parameter was declared')
        self.assertAlmostEqual(result.values[0].double_value, 0.5)
        self.assertAlmostEqual(result.values[1].double_value, 0.1)
        self.assertAlmostEqual(result.values[2].double_value, 0.1)
        self.assertEqual(result.values[3].string_value, TRAJECTORY_TOPIC)

    def test_subscribes_to_trajectory_topic(self):
        """Once the engine is up, the node should subscribe to the configured topic."""
        if not self.node_active:
            self.skipTest('joint_position_controller never became active (engine unavailable)')

        pub = self.node.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 1)

        self.assertTrue(
            wait_for_subscription(self.node, pub, 30),
            f'joint_position_controller did not subscribe to {TRAJECTORY_TOPIC}',
        )

    def test_accepts_trajectory(self, joint_position_controller, proc_info):
        """A streamed trajectory should be consumed without killing the node."""
        if not self.node_active:
            self.skipTest('joint_position_controller never became active (engine unavailable)')

        pub = self.node.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 1)
        wait_for_subscription(self.node, pub, 10)

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 33333333
        msg.points = [point]

        for _ in range(10):
            pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertNotIn(
            joint_position_controller, proc_info,
            'joint_position_controller exited while receiving trajectories',
        )

    def test_rejects_mismatched_joint_count(self, joint_position_controller, proc_info):
        """A trajectory with the wrong number of joints should be dropped, not fatal."""
        if not self.node_active:
            self.skipTest('joint_position_controller never became active (engine unavailable)')

        pub = self.node.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 1)
        wait_for_subscription(self.node, pub, 10)

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES[:3]
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0]
        point.time_from_start.nanosec = 33333333
        msg.points = [point]

        pub.publish(msg)
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertNotIn(
            joint_position_controller, proc_info,
            'joint_position_controller exited on a mismatched trajectory',
        )


@launch_testing.post_shutdown_test()
class TestJointPositionControllerShutdown(unittest.TestCase):
    """Verify joint_position_controller exits cleanly."""

    def test_exit_code(self, proc_info, joint_position_controller):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -15],
            process=joint_position_controller,
        )
