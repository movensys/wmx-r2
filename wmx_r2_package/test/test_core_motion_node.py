"""Integration test: wmx_core_motion_node services and state publishing."""

import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node

from wmx_r2_message.msg import AxisState
from wmx_r2_message.srv import GetWmxParams
from wmx_r2_message.srv import SetAxis


@pytest.mark.launch_test
def generate_test_description():
    engine_node = launch_ros.actions.Node(
        package='wmx_r2_package',
        executable='wmx_engine_node',
        name='wmx_engine_node',
        output='screen',
    )
    core_motion_node = launch_ros.actions.Node(
        package='wmx_r2_package',
        executable='wmx_core_motion_node',
        name='wmx_core_motion_node',
        output='screen',
    )

    return launch.LaunchDescription([
        engine_node,
        core_motion_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'engine_node': engine_node, 'core_motion_node': core_motion_node}


class TestCoreMotionNode(unittest.TestCase):
    """Test wmx_core_motion_node services and topic publishing."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_core_motion_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_axis_state_published(self):
        """Core motion node should publish AxisState at 100 Hz."""
        received = []

        def cb(msg):
            received.append(msg)

        self.node.create_subscription(AxisState, 'wmx/axis/state', cb, 10)

        end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=20)
        while self.node.get_clock().now() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            if len(received) >= 5:
                break

        self.assertGreaterEqual(
            len(received), 5,
            'Did not receive at least 5 AxisState messages within 20 seconds',
        )

    def test_axis_state_has_header(self):
        """Published AxisState should have a populated header."""
        received = []

        def cb(msg):
            received.append(msg)

        self.node.create_subscription(AxisState, 'wmx/axis/state', cb, 10)

        end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=20)
        while self.node.get_clock().now() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            if received:
                break

        self.assertTrue(received, 'No AxisState received')
        msg = received[0]
        self.assertNotEqual(msg.header.stamp.sec, 0, 'Header stamp should be non-zero')
        self.assertEqual(msg.header.frame_id, 'base_link')

    def test_axis_state_field_lengths_match(self):
        """All AxisState array fields should have the same length."""
        received = []

        def cb(msg):
            received.append(msg)

        self.node.create_subscription(AxisState, 'wmx/axis/state', cb, 10)

        end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=20)
        while self.node.get_clock().now() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            if received:
                break

        self.assertTrue(received, 'No AxisState received')
        msg = received[0]
        n = len(msg.amp_alarm)
        self.assertGreater(n, 0, 'AxisState should have at least 1 axis')
        for field in [
            'amp_alarm_code', 'servo_on', 'home_done', 'motion_complete',
            'negative_ls', 'positive_ls', 'home_switch',
            'pos_cmd', 'velocity_cmd', 'actual_pos',
            'actual_velocity', 'actual_torque',
        ]:
            self.assertEqual(
                len(getattr(msg, field)), n,
                f'{field} length {len(getattr(msg, field))} != {n}',
            )

    def test_set_axis_on_service_available(self):
        """set_on service should be available."""
        client = self.node.create_client(SetAxis, 'wmx/axis/set_on')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/axis/set_on service not available',
        )

    def test_clear_alarm_service_available(self):
        """clear_alarm service should be available."""
        client = self.node.create_client(SetAxis, 'wmx/axis/clear_alarm')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/axis/clear_alarm service not available',
        )

    def test_set_mode_service_available(self):
        """set_mode service should be available."""
        client = self.node.create_client(SetAxis, 'wmx/axis/set_mode')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/axis/set_mode service not available',
        )

    def test_set_polarity_service_available(self):
        """set_polarity service should be available."""
        client = self.node.create_client(SetAxis, 'wmx/axis/set_polarity')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/axis/set_polarity service not available',
        )

    def test_homing_service_available(self):
        """Homing service should be available."""
        client = self.node.create_client(SetAxis, 'wmx/axis/homing')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/axis/homing service not available',
        )

    def test_get_params_service_call(self):
        """Get params service should respond with axis parameters."""
        client = self.node.create_client(GetWmxParams, 'wmx/params/get')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/params/get service not available',
        )

        req = GetWmxParams.Request()
        req.index = [0]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        self.assertIsNotNone(future.result(), 'Service call returned no result')
        result = future.result()
        self.assertTrue(result.success)
        self.assertGreater(len(result.params_dump), 0, 'params_dump should not be empty')


@launch_testing.post_shutdown_test()
class TestCoreMotionShutdown(unittest.TestCase):
    """Verify nodes exit cleanly."""

    def test_exit_code(self, proc_info, core_motion_node):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -6, -15],
            process=core_motion_node,
        )
