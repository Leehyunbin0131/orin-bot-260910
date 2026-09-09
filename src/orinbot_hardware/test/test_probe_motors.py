import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    'probe_motors', Path(__file__).resolve().parents[1] / 'scripts/probe_motors.py')
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class ReadOnlyPacket:
    def __init__(self, communication=0, error=0):
        self.communication, self.error = communication, error

    def read1ByteTxRx(self, port, motor_id, address):
        values = {64: 0, 70: 32, 128: 0xffffffff, 144: 121, 146: 35}
        return values.get(address, 0), self.communication, self.error

    read2ByteTxRx = read1ByteTxRx
    read4ByteTxRx = read1ByteTxRx

    def getTxRxResult(self, result): return f'communication={result}'
    def getRxPacketError(self, error): return f'error={error}'


class ProbeTests(unittest.TestCase):
    def read(self, packet):
        with patch.dict('sys.modules', {'dynamixel_sdk': SimpleNamespace(COMM_SUCCESS=0)}):
            return probe.read_registers(packet, object(), 1)

    def test_alert_preserves_fault_and_sensor_data_without_writes(self):
        item = self.read(ReadOnlyPacket(error=0x80))
        self.assertTrue(item['hardware_alert'])
        self.assertEqual(item['hardware_error'], 32)
        self.assertEqual(item['hardware_error_flags'], ['Overload Error'])
        self.assertEqual(item['torque_enabled'], 0)
        self.assertEqual(item['voltage_raw'], 121)
        self.assertEqual(item['temperature_c'], 35)
        self.assertEqual(item['present_velocity_raw'], -1)

    def test_instruction_errors_do_not_expose_invalid_values(self):
        for error in (0x07, 0x87):
            with self.subTest(error=error):
                item = self.read(ReadOnlyPacket(error=error))
                self.assertIsInstance(item['hardware_error'], dict)
                self.assertNotIn('hardware_error_flags', item)

    def test_communication_failure_does_not_expose_invalid_values(self):
        item = self.read(ReadOnlyPacket(communication=-3001))
        self.assertIsInstance(item['temperature_c'], dict)
        self.assertNotIn('hardware_alert', item)


if __name__ == '__main__':
    unittest.main()
