"""Unit tests: verify all wmx_r2_message interfaces have correct fields."""

import unittest

from std_msgs.msg import Header

from wmx_r2_message.msg import AxisPose
from wmx_r2_message.msg import AxisState
from wmx_r2_message.msg import AxisVelocity
from wmx_r2_message.srv import EcatGetNetworkState
from wmx_r2_message.srv import EcatRegisterRead
from wmx_r2_message.srv import EcatResetStatistics
from wmx_r2_message.srv import EcatStartHotconnect
from wmx_r2_message.srv import GetIoBit
from wmx_r2_message.srv import GetIoBytes
from wmx_r2_message.srv import GetWmxParams
from wmx_r2_message.srv import LoadWmxParams
from wmx_r2_message.srv import SetAxis
from wmx_r2_message.srv import SetAxisGearRatio
from wmx_r2_message.srv import SetEngine
from wmx_r2_message.srv import SetIoBit
from wmx_r2_message.srv import SetIoBytes


class TestAxisStateMsg(unittest.TestCase):
    """Verify AxisState.msg fields and types."""

    def test_has_header(self):
        msg = AxisState()
        self.assertIsInstance(msg.header, Header)

    def test_boolean_status_fields(self):
        msg = AxisState()
        for field in [
            'amp_alarm', 'servo_on', 'home_done', 'motion_complete',
            'negative_ls', 'positive_ls', 'home_switch',
        ]:
            self.assertTrue(hasattr(msg, field), f'Missing field: {field}')
            attr = getattr(msg, field)
            self.assertIsInstance(attr, list, f'{field} should be a list')

    def test_float_fields(self):
        msg = AxisState()
        for field in [
            'pos_cmd', 'velocity_cmd', 'actual_pos',
            'actual_velocity', 'actual_torque',
        ]:
            self.assertTrue(hasattr(msg, field), f'Missing field: {field}')

    def test_amp_alarm_code_field(self):
        """The drive's fault number is published next to the amp_alarm flag."""
        msg = AxisState()
        self.assertTrue(hasattr(msg, 'amp_alarm_code'), 'Missing field: amp_alarm_code')
        msg.amp_alarm = [False, True]
        msg.amp_alarm_code = [0, 0x7380]
        self.assertEqual(list(msg.amp_alarm_code), [0, 0x7380])

    def test_populate_and_read(self):
        msg = AxisState()
        msg.amp_alarm = [True, False]
        msg.servo_on = [True, True]
        msg.motion_complete = [False, True]
        msg.actual_pos = [1.0, 2.0]
        self.assertEqual(list(msg.amp_alarm), [True, False])
        self.assertEqual(list(msg.actual_pos), [1.0, 2.0])

    def test_no_in_pos_field(self):
        """Verify in_pos was renamed to motion_complete."""
        msg = AxisState()
        self.assertFalse(hasattr(msg, 'in_pos'))
        self.assertTrue(hasattr(msg, 'motion_complete'))


class TestAxisPoseMsg(unittest.TestCase):
    """Verify AxisPose.msg fields."""

    def test_fields_exist(self):
        msg = AxisPose()
        for field in ['index', 'target', 'velocity', 'acc', 'dec']:
            self.assertTrue(hasattr(msg, field), f'Missing field: {field}')

    def test_populate(self):
        msg = AxisPose()
        msg.index = [0, 1]
        msg.target = [100.0, 200.0]
        msg.velocity = [50.0, 50.0]
        msg.acc = [10.0, 10.0]
        msg.dec = [10.0, 10.0]
        self.assertEqual(len(msg.index), 2)


class TestAxisVelocityMsg(unittest.TestCase):
    """Verify AxisVelocity.msg fields."""

    def test_fields_exist(self):
        msg = AxisVelocity()
        for field in ['index', 'velocity', 'acc', 'dec']:
            self.assertTrue(hasattr(msg, field), f'Missing field: {field}')

    def test_no_target_field(self):
        """Verify AxisVelocity has no target field (unlike AxisPose)."""
        msg = AxisVelocity()
        self.assertFalse(hasattr(msg, 'target'))


class TestSetEngineSrv(unittest.TestCase):
    """Verify SetEngine.srv fields."""

    def test_request_fields(self):
        req = SetEngine.Request()
        self.assertTrue(hasattr(req, 'data'))
        self.assertTrue(hasattr(req, 'path'))
        self.assertTrue(hasattr(req, 'name'))

    def test_response_fields(self):
        res = SetEngine.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))


class TestSetAxisSrv(unittest.TestCase):
    """Verify SetAxis.srv fields."""

    def test_request_fields(self):
        req = SetAxis.Request()
        self.assertTrue(hasattr(req, 'index'))
        self.assertTrue(hasattr(req, 'data'))

    def test_response_fields(self):
        res = SetAxis.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))

    def test_populate_multi_axis(self):
        req = SetAxis.Request()
        req.index = [0, 1, 2]
        req.data = [1, 1, 0]
        self.assertEqual(len(req.index), 3)


class TestSetAxisGearRatioSrv(unittest.TestCase):
    """Verify SetAxisGearRatio.srv fields."""

    def test_request_fields(self):
        req = SetAxisGearRatio.Request()
        self.assertTrue(hasattr(req, 'index'))
        self.assertTrue(hasattr(req, 'numerator'))
        self.assertTrue(hasattr(req, 'denominator'))

    def test_no_denumerator(self):
        """Verify typo 'denumerator' was fixed to 'denominator'."""
        req = SetAxisGearRatio.Request()
        self.assertFalse(hasattr(req, 'denumerator'))
        self.assertTrue(hasattr(req, 'denominator'))


class TestLoadWmxParamsSrv(unittest.TestCase):
    """Verify LoadWmxParams.srv fields."""

    def test_request_fields(self):
        req = LoadWmxParams.Request()
        self.assertTrue(hasattr(req, 'file_path'))

    def test_response_fields(self):
        res = LoadWmxParams.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))


class TestGetWmxParamsSrv(unittest.TestCase):
    """Verify GetWmxParams.srv fields."""

    def test_request_fields(self):
        req = GetWmxParams.Request()
        self.assertTrue(hasattr(req, 'index'))

    def test_response_fields(self):
        res = GetWmxParams.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))
        self.assertTrue(hasattr(res, 'params_dump'))


class TestIoServicesSrv(unittest.TestCase):
    """Verify IO service fields."""

    def test_get_io_bit(self):
        req = GetIoBit.Request()
        self.assertTrue(hasattr(req, 'byte'))
        self.assertTrue(hasattr(req, 'bit'))
        res = GetIoBit.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'value'))
        self.assertTrue(hasattr(res, 'message'))

    def test_get_io_bytes(self):
        req = GetIoBytes.Request()
        self.assertTrue(hasattr(req, 'byte'))
        self.assertTrue(hasattr(req, 'length'))
        res = GetIoBytes.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'data'))
        self.assertTrue(hasattr(res, 'message'))

    def test_set_io_bit(self):
        req = SetIoBit.Request()
        self.assertTrue(hasattr(req, 'byte'))
        self.assertTrue(hasattr(req, 'bit'))
        self.assertTrue(hasattr(req, 'value'))
        res = SetIoBit.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))

    def test_set_io_bytes(self):
        req = SetIoBytes.Request()
        self.assertTrue(hasattr(req, 'byte'))
        self.assertTrue(hasattr(req, 'data'))
        res = SetIoBytes.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))


class TestEcatServicesSrv(unittest.TestCase):
    """Verify EtherCAT service fields."""

    def test_ecat_get_network_state(self):
        req = EcatGetNetworkState.Request()
        self.assertTrue(hasattr(req, 'master_id'))
        res = EcatGetNetworkState.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'master_state'))
        self.assertTrue(hasattr(res, 'total_axes'))
        self.assertTrue(hasattr(res, 'num_of_slaves'))
        self.assertTrue(hasattr(res, 'slave_ids'))
        self.assertTrue(hasattr(res, 'slave_states'))
        self.assertTrue(hasattr(res, 'packet_loss'))

    def test_ecat_register_read(self):
        req = EcatRegisterRead.Request()
        self.assertTrue(hasattr(req, 'master_id'))
        self.assertTrue(hasattr(req, 'slave_id'))
        self.assertTrue(hasattr(req, 'reg_address'))
        self.assertTrue(hasattr(req, 'length'))
        res = EcatRegisterRead.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'data'))
        self.assertTrue(hasattr(res, 'message'))

    def test_ecat_reset_statistics(self):
        req = EcatResetStatistics.Request()
        self.assertTrue(hasattr(req, 'master_id'))
        res = EcatResetStatistics.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))

    def test_ecat_start_hotconnect(self):
        req = EcatStartHotconnect.Request()
        self.assertTrue(hasattr(req, 'master_id'))
        res = EcatStartHotconnect.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))


if __name__ == '__main__':
    unittest.main()
