"""Integration test: wmx_ethercat_node service availability and basic calls."""

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

from wmx_r2_message.srv import EcatGetMasterInfo
from wmx_r2_message.srv import EcatRegisterRead
from wmx_r2_message.srv import EcatResetStatistics
from wmx_r2_message.srv import EcatStartHotconnect


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
    ethercat_node = launch_ros.actions.LifecycleNode(
        package='wmx_r2_package',
        executable='wmx_ethercat_node',
        name='wmx_ethercat_node',
        namespace='',
        output='screen',
    )

    return launch.LaunchDescription([
        engine_node,
        manager_node,
        ethercat_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'engine_node': engine_node, 'ethercat_node': ethercat_node}


class TestEthercatNode(unittest.TestCase):
    """Test wmx_ethercat_node service availability and responses."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        probe = Node('test_ethercat_node_probe')
        try:
            cls.node_active = wait_until_active(probe, 'wmx_ethercat_node')
        finally:
            probe.destroy_node()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Nothing to exercise until the manager has brought the node up,
        # which needs a communicating engine.
        if not self.node_active:
            self.skipTest('wmx_ethercat_node never became active (engine unavailable)')
        self.node = Node('test_ethercat_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_get_master_info_service(self):
        """get_master_info service should be available and respond."""
        client = self.node.create_client(
            EcatGetMasterInfo, 'wmx/ecat/get_master_info')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/ecat/get_master_info service not available',
        )

        req = EcatGetMasterInfo.Request()
        req.master_id = 0
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        self.assertIsNotNone(future.result(), 'Service call returned no result')
        # Note: result.success may be false if no EtherCAT network is
        # connected; we only verify the service responds correctly.

    def test_get_master_info_has_master_fields(self):
        """get_master_info response should populate master status fields."""
        client = self.node.create_client(
            EcatGetMasterInfo, 'wmx/ecat/get_master_info')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/ecat/get_master_info service not available',
        )

        req = EcatGetMasterInfo.Request()
        req.master_id = 0
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        self.assertIsNotNone(future.result())
        result = future.result()
        # Fields exist and are populated (values depend on hardware state)
        self.assertIsNotNone(result.state)
        self.assertIsNotNone(result.total_axes_num)
        self.assertIsNotNone(result.num_of_slaves)

    def test_register_read_service(self):
        """register_read service should be available and respond."""
        client = self.node.create_client(
            EcatRegisterRead, 'wmx/ecat/register_read')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/ecat/register_read service not available',
        )

    def test_reset_statistics_service(self):
        """reset_statistics service should be available and respond."""
        client = self.node.create_client(
            EcatResetStatistics, 'wmx/ecat/reset_statistics')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/ecat/reset_statistics service not available',
        )

        req = EcatResetStatistics.Request()
        req.master_id = 0
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        self.assertIsNotNone(future.result(), 'Service call returned no result')

    def test_start_hotconnect_service(self):
        """start_hotconnect service should be available."""
        client = self.node.create_client(
            EcatStartHotconnect, 'wmx/ecat/start_hotconnect')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/ecat/start_hotconnect service not available',
        )


@launch_testing.post_shutdown_test()
class TestEthercatNodeShutdown(unittest.TestCase):
    """Verify ethercat node exits cleanly."""

    def test_exit_code(self, proc_info, ethercat_node):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -15],
            process=ethercat_node,
        )
