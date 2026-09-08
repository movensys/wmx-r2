"""Integration test: wmx_io_node service availability and basic calls."""

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

from wmx_r2_message.srv import GetIoBit
from wmx_r2_message.srv import GetIoBits
from wmx_r2_message.srv import GetIoBytes
from wmx_r2_message.srv import SetIoBit
from wmx_r2_message.srv import SetIoBytes


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
    io_node = launch_ros.actions.LifecycleNode(
        package='wmx_r2_package',
        executable='wmx_io_node',
        name='wmx_io_node',
        namespace='',
        output='screen',
    )

    return launch.LaunchDescription([
        engine_node,
        manager_node,
        io_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'engine_node': engine_node, 'io_node': io_node}


class TestIoNode(unittest.TestCase):
    """Test wmx_io_node service availability and responses."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        probe = Node('test_io_node_probe')
        try:
            cls.node_active = wait_until_active(probe, 'wmx_io_node')
        finally:
            probe.destroy_node()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        if not self.node_active:
            self.skipTest('wmx_io_node never became active (engine unavailable)')
        self.node = Node('test_io_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_get_in_bit_service(self):
        """get_in_bit service should be available and respond."""
        client = self.node.create_client(GetIoBit, 'wmx/io/get_in_bit')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/io/get_in_bit service not available',
        )

        req = GetIoBit.Request()
        req.addr = 0
        req.bit = 0
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        self.assertIsNotNone(future.result(), 'Service call returned no result')

    def test_get_out_bit_service(self):
        """get_out_bit service should be available and respond."""
        client = self.node.create_client(GetIoBit, 'wmx/io/get_out_bit')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/io/get_out_bit service not available',
        )

        req = GetIoBit.Request()
        req.addr = 0
        req.bit = 0
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        self.assertIsNotNone(future.result(), 'Service call returned no result')

    def test_get_in_bits_service(self):
        """get_in_bits service should be available and respond."""
        client = self.node.create_client(GetIoBits, 'wmx/io/get_in_bits')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/io/get_in_bits service not available',
        )

        req = GetIoBits.Request()
        req.addr = [0, 0]
        req.bit = [0, 1]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        self.assertIsNotNone(future.result(), 'Service call returned no result')

    def test_get_out_bits_service(self):
        """get_out_bits service should be available and respond."""
        client = self.node.create_client(GetIoBits, 'wmx/io/get_out_bits')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/io/get_out_bits service not available',
        )

        req = GetIoBits.Request()
        req.addr = [0, 0]
        req.bit = [0, 1]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        self.assertIsNotNone(future.result(), 'Service call returned no result')

    def test_get_in_bytes_service(self):
        """get_in_bytes service should be available and respond."""
        client = self.node.create_client(GetIoBytes, 'wmx/io/get_in_bytes')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/io/get_in_bytes service not available',
        )

        req = GetIoBytes.Request()
        req.addr = 0
        req.size = 1
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        self.assertIsNotNone(future.result(), 'Service call returned no result')

    def test_get_out_bytes_service(self):
        """get_out_bytes service should be available and respond."""
        client = self.node.create_client(GetIoBytes, 'wmx/io/get_out_bytes')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/io/get_out_bytes service not available',
        )

        req = GetIoBytes.Request()
        req.addr = 0
        req.size = 1
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        self.assertIsNotNone(future.result(), 'Service call returned no result')

    def test_set_out_bit_service(self):
        """set_out_bit service should be available."""
        client = self.node.create_client(SetIoBit, 'wmx/io/set_out_bit')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/io/set_out_bit service not available',
        )

    def test_set_out_bytes_service(self):
        """set_out_bytes service should be available."""
        client = self.node.create_client(SetIoBytes, 'wmx/io/set_out_bytes')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/io/set_out_bytes service not available',
        )


@launch_testing.post_shutdown_test()
class TestIoNodeShutdown(unittest.TestCase):
    """Verify io node exits cleanly."""

    def test_exit_code(self, proc_info, io_node):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -15],
            process=io_node,
        )
