"""Unit tests: verify all wmx_r2_message interfaces have correct fields."""

import unittest

from std_msgs.msg import Header

from wmx_r2_message.msg import AxesStatus
from wmx_r2_message.srv import EcatGetMasterInfo
from wmx_r2_message.srv import EcatRegisterRead
from wmx_r2_message.srv import EcatResetStatistics
from wmx_r2_message.srv import EcatStartHotconnect
from wmx_r2_message.srv import GetAxisParam
from wmx_r2_message.srv import GetIoBit
from wmx_r2_message.srv import GetIoBits
from wmx_r2_message.srv import GetIoBytes
from wmx_r2_message.srv import GetNodeStates
from wmx_r2_message.srv import ImportAndSetAll
from wmx_r2_message.srv import SetAxes
from wmx_r2_message.srv import SetAxesGearRatio
from wmx_r2_message.srv import SetEngine
from wmx_r2_message.srv import SetIoBit
from wmx_r2_message.srv import SetIoBytes
from wmx_r2_message.srv import SetNodeState
from wmx_r2_message.srv import StartAxesPose
from wmx_r2_message.srv import StartAxesVelocity


class TestAxesStatusMsg(unittest.TestCase):
    """Verify AxesStatus.msg fields and types."""

    def test_has_header(self):
        msg = AxesStatus()
        self.assertIsInstance(msg.header, Header)

    def test_boolean_status_fields(self):
        msg = AxesStatus()
        for field in [
            'amp_alarm', 'servo_on', 'home_done', 'motion_complete',
            'negative_ls', 'positive_ls', 'home_switch',
        ]:
            self.assertTrue(hasattr(msg, field), f'Missing field: {field}')
            attr = getattr(msg, field)
            self.assertIsInstance(attr, list, f'{field} should be a list')

    def test_float_fields(self):
        msg = AxesStatus()
        for field in [
            'pos_cmd', 'velocity_cmd', 'actual_pos',
            'actual_velocity', 'actual_torque',
        ]:
            self.assertTrue(hasattr(msg, field), f'Missing field: {field}')

    def test_populate_and_read(self):
        msg = AxesStatus()
        msg.amp_alarm = [True, False]
        msg.servo_on = [True, True]
        msg.motion_complete = [False, True]
        msg.actual_pos = [1.0, 2.0]
        self.assertEqual(list(msg.amp_alarm), [True, False])
        self.assertEqual(list(msg.actual_pos), [1.0, 2.0])

    def test_no_in_pos_field(self):
        """Verify in_pos was renamed to motion_complete."""
        msg = AxesStatus()
        self.assertFalse(hasattr(msg, 'in_pos'))
        self.assertTrue(hasattr(msg, 'motion_complete'))


class TestStartAxesPoseSrv(unittest.TestCase):
    """Verify StartAxesPose.srv fields."""

    def test_request_fields(self):
        req = StartAxesPose.Request()
        for field in ['axis', 'target', 'velocity', 'acc', 'dec']:
            self.assertTrue(hasattr(req, field), f'{field} missing')

    def test_response_fields(self):
        res = StartAxesPose.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))

    def test_assign_arrays(self):
        req = StartAxesPose.Request()
        req.axis = [0, 1]
        req.target = [100.0, 200.0]
        req.velocity = [50.0, 50.0]
        req.acc = [10.0, 10.0]
        req.dec = [10.0, 10.0]
        self.assertEqual(len(req.axis), 2)
        self.assertEqual(list(req.target), [100.0, 200.0])


class TestStartAxesVelocitySrv(unittest.TestCase):
    """Verify StartAxesVelocity.srv fields."""

    def test_request_fields(self):
        req = StartAxesVelocity.Request()
        for field in ['axis', 'velocity', 'acc', 'dec']:
            self.assertTrue(hasattr(req, field), f'{field} missing')

    def test_has_no_target(self):
        """Verify StartAxesVelocity has no target field (unlike StartAxesPose)."""
        req = StartAxesVelocity.Request()
        self.assertFalse(hasattr(req, 'target'))

    def test_response_fields(self):
        res = StartAxesVelocity.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))


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


class TestSetNodeStateSrv(unittest.TestCase):
    """Verify SetNodeState.srv fields."""

    def test_request_fields(self):
        req = SetNodeState.Request()
        self.assertTrue(hasattr(req, 'node_name'))
        self.assertTrue(hasattr(req, 'transition'))

    def test_response_fields(self):
        res = SetNodeState.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))
        self.assertTrue(hasattr(res, 'node_names'))
        self.assertTrue(hasattr(res, 'states'))

    def test_response_arrays_are_index_aligned(self):
        res = SetNodeState.Response()
        res.node_names = ['wmx_io_node', 'wmx_ethercat_node']
        res.states = ['active', 'inactive']
        self.assertEqual(len(res.node_names), len(res.states))


class TestGetNodeStatesSrv(unittest.TestCase):
    """Verify GetNodeStates.srv fields."""

    def test_request_is_empty(self):
        req = GetNodeStates.Request()
        self.assertFalse(hasattr(req, 'node_name'))

    def test_response_fields(self):
        res = GetNodeStates.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))
        self.assertTrue(hasattr(res, 'node_names'))
        self.assertTrue(hasattr(res, 'states'))

    def test_response_arrays_are_index_aligned(self):
        res = GetNodeStates.Response()
        res.node_names = ['/wmx_io_node', '/wmx_ethercat_node']
        res.states = ['active', 'inactive']
        self.assertEqual(len(res.node_names), len(res.states))


class TestSetAxesSrv(unittest.TestCase):
    """Verify SetAxes.srv fields."""

    def test_request_fields(self):
        req = SetAxes.Request()
        self.assertTrue(hasattr(req, 'axis'))
        self.assertTrue(hasattr(req, 'data'))

    def test_response_fields(self):
        res = SetAxes.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))

    def test_populate_multi_axis(self):
        req = SetAxes.Request()
        req.axis = [0, 1, 2]
        req.data = [1, 1, 0]
        self.assertEqual(len(req.axis), 3)


class TestSetAxesGearRatioSrv(unittest.TestCase):
    """Verify SetAxesGearRatio.srv fields."""

    def test_request_fields(self):
        req = SetAxesGearRatio.Request()
        self.assertTrue(hasattr(req, 'axis'))
        self.assertTrue(hasattr(req, 'numerator'))
        self.assertTrue(hasattr(req, 'denominator'))

    def test_no_denumerator(self):
        """Verify typo 'denumerator' was fixed to 'denominator'."""
        req = SetAxesGearRatio.Request()
        self.assertFalse(hasattr(req, 'denumerator'))
        self.assertTrue(hasattr(req, 'denominator'))


class TestImportAndSetAllSrv(unittest.TestCase):
    """Verify ImportAndSetAll.srv fields."""

    def test_request_fields(self):
        req = ImportAndSetAll.Request()
        self.assertTrue(hasattr(req, 'path'))

    def test_response_fields(self):
        res = ImportAndSetAll.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))


class TestGetAxisParamSrv(unittest.TestCase):
    """Verify GetAxisParam.srv fields."""

    def test_request_fields(self):
        req = GetAxisParam.Request()
        self.assertTrue(hasattr(req, 'axis'))

    def test_response_fields(self):
        res = GetAxisParam.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))
        self.assertTrue(hasattr(res, 'axis_param'))


class TestIoServicesSrv(unittest.TestCase):
    """Verify IO service fields."""

    def test_get_io_bit(self):
        req = GetIoBit.Request()
        self.assertTrue(hasattr(req, 'addr'))
        self.assertTrue(hasattr(req, 'bit'))
        res = GetIoBit.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'data'))
        self.assertTrue(hasattr(res, 'message'))

    def test_get_io_bits(self):
        req = GetIoBits.Request()
        self.assertTrue(hasattr(req, 'addr'))
        self.assertTrue(hasattr(req, 'bit'))
        res = GetIoBits.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'data'))
        self.assertTrue(hasattr(res, 'message'))

    def test_get_io_bytes(self):
        req = GetIoBytes.Request()
        self.assertTrue(hasattr(req, 'addr'))
        self.assertTrue(hasattr(req, 'size'))
        res = GetIoBytes.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'data'))
        self.assertTrue(hasattr(res, 'message'))

    def test_set_io_bit(self):
        req = SetIoBit.Request()
        self.assertTrue(hasattr(req, 'addr'))
        self.assertTrue(hasattr(req, 'bit'))
        self.assertTrue(hasattr(req, 'data'))
        res = SetIoBit.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))

    def test_set_io_bytes(self):
        req = SetIoBytes.Request()
        self.assertTrue(hasattr(req, 'addr'))
        self.assertTrue(hasattr(req, 'data'))
        res = SetIoBytes.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'message'))


class TestEcatServicesSrv(unittest.TestCase):
    """Verify EtherCAT service fields."""

    def test_ecat_get_master_info(self):
        req = EcatGetMasterInfo.Request()
        self.assertTrue(hasattr(req, 'master_id'))
        res = EcatGetMasterInfo.Response()
        self.assertTrue(hasattr(res, 'success'))
        self.assertTrue(hasattr(res, 'state'))
        self.assertTrue(hasattr(res, 'total_axes_num'))
        self.assertTrue(hasattr(res, 'num_of_slaves'))
        self.assertTrue(hasattr(res, 'slave_ids'))
        self.assertTrue(hasattr(res, 'slave_states'))
        self.assertTrue(hasattr(res, 'packet_loss'))

    def test_ecat_register_read(self):
        req = EcatRegisterRead.Request()
        self.assertTrue(hasattr(req, 'master_id'))
        self.assertTrue(hasattr(req, 'slave_id'))
        self.assertTrue(hasattr(req, 'reg_addr'))
        self.assertTrue(hasattr(req, 'len'))
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
