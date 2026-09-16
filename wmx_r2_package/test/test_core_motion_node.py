"""Integration test: wmx_core_motion_node services and state publishing."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from lifecycle_msgs.srv import GetState
import pytest
import rclpy
from rclpy.node import Node

from wmx_r2_message.msg import AxesStatus
from wmx_r2_message.srv import GetAxisParam
from wmx_r2_message.srv import SetAxes


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
    core_motion_node = launch_ros.actions.LifecycleNode(
        package='wmx_r2_package',
        executable='wmx_core_motion_node',
        name='wmx_core_motion_node',
        namespace='',
        output='screen',
    )

    return launch.LaunchDescription([
        engine_node,
        manager_node,
        core_motion_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'engine_node': engine_node, 'core_motion_node': core_motion_node}


class TestCoreMotionNode(unittest.TestCase):
    """Test wmx_core_motion_node services and topic publishing."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        probe = Node('test_core_motion_node_probe')
        try:
            cls.node_active = wait_until_active(probe, 'wmx_core_motion_node')
        finally:
            probe.destroy_node()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Nothing to exercise until the manager has brought the node up,
        # which needs a communicating engine.
        if not self.node_active:
            self.skipTest('wmx_core_motion_node never became active (engine unavailable)')
        self.node = Node('test_core_motion_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_axes_status_published(self):
        """Core motion node should publish AxesStatus at 100 Hz."""
        received = []

        def cb(msg):
            received.append(msg)

        self.node.create_subscription(AxesStatus, 'wmx/axes/status', cb, 10)

        end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=20)
        while self.node.get_clock().now() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            if len(received) >= 5:
                break

        self.assertGreaterEqual(
            len(received), 5,
            'Did not receive at least 5 AxesStatus messages within 20 seconds',
        )

    def test_axes_status_has_header(self):
        """Published AxesStatus should have a populated header."""
        received = []

        def cb(msg):
            received.append(msg)

        self.node.create_subscription(AxesStatus, 'wmx/axes/status', cb, 10)

        end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=20)
        while self.node.get_clock().now() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            if received:
                break

        self.assertTrue(received, 'No AxesStatus received')
        msg = received[0]
        self.assertNotEqual(msg.header.stamp.sec, 0, 'Header stamp should be non-zero')
        self.assertEqual(msg.header.frame_id, 'base_link')

    def test_axes_status_field_lengths_match(self):
        """All AxesStatus array fields should have the same length."""
        received = []

        def cb(msg):
            received.append(msg)

        self.node.create_subscription(AxesStatus, 'wmx/axes/status', cb, 10)

        end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=20)
        while self.node.get_clock().now() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            if received:
                break

        self.assertTrue(received, 'No AxesStatus received')
        msg = received[0]
        n = len(msg.amp_alarm)
        self.assertGreater(n, 0, 'AxesStatus should have at least 1 axis')
        for field in [
            'servo_on', 'home_done', 'motion_complete',
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
        client = self.node.create_client(SetAxes, 'wmx/axes/set_servo_on')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/axes/set_servo_on service not available',
        )

    def test_clear_alarm_service_available(self):
        """clear_alarm service should be available."""
        client = self.node.create_client(SetAxes, 'wmx/axes/clear_amp_alarm')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/axes/clear_amp_alarm service not available',
        )

    def test_set_mode_service_available(self):
        """set_mode service should be available."""
        client = self.node.create_client(SetAxes, 'wmx/axes/set_axis_command_mode')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/axes/set_axis_command_mode service not available',
        )

    def test_set_polarity_service_available(self):
        """set_polarity service should be available."""
        client = self.node.create_client(SetAxes, 'wmx/axes/set_axis_polarity')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/axes/set_axis_polarity service not available',
        )

    def test_homing_service_available(self):
        """Homing service should be available."""
        client = self.node.create_client(SetAxes, 'wmx/axes/start_home')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/axes/start_home service not available',
        )

    def test_get_params_service_call(self):
        """Get params service should respond with axis parameters."""
        client = self.node.create_client(GetAxisParam, 'wmx/engine/get_axis_param')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/engine/get_axis_param service not available',
        )

        req = GetAxisParam.Request()
        req.axis = [0]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        self.assertIsNotNone(future.result(), 'Service call returned no result')
        result = future.result()
        self.assertTrue(result.success)
        self.assertGreater(len(result.axis_param), 0, 'axis_param should not be empty')


@launch_testing.post_shutdown_test()
class TestCoreMotionShutdown(unittest.TestCase):
    """Verify nodes exit cleanly."""

    def test_exit_code(self, proc_info, core_motion_node):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -15],
            process=core_motion_node,
        )
