import importlib.util
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import new_auto_plotter as plotter
from discrete_pi import DiscretePI


class ExportChecks(unittest.TestCase):
    def test_firmware_rows_are_accepted_by_collector(self):
        for feedback, mode in (("encoder", "original"), ("observer", "original"),
                               ("observer", "discrete_pi")):
            with self.subTest(feedback=feedback, mode=mode):
                self.check_export(feedback, mode)

    def check_export(self, feedback, mode):
        pyb = ModuleType('pyb')
        pyb.USB_VCP = object
        spec = importlib.util.spec_from_file_location(
            'user_under_test', Path(__file__).resolve().parents[1] / 'taskuser.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'pyb': pyb}):
            spec.loader.exec_module(module)
        user = module.TaskUser.__new__(module.TaskUser)
        data = dict(time_us=[0, 20000], setpoint_rad_s=[1.5, 1.5],
                    velocity_rad_s=[0.0, 0.1], estimated_velocity_rad_s=[0.0, 0.2],
                    voltage_V=[2.0, 1.9], count=2, status='completed')
        user.left_data = user.right_data = SimpleNamespace(value=data)
        user.left_test_flag = user.right_test_flag = SimpleNamespace(value=False)
        user.current_test = 'right'
        user.pending_output = b''
        user.export_phase = 'header'
        user.export_index = 0
        user.run_setpoint = 1.5
        user.run_kp = 0.6
        user.motor_period_ms = 20
        user.battery_voltage = 8.0
        user.motor_test = dict(motors=[SimpleNamespace(
            velocity_feedback=feedback,
            pid=SimpleNamespace(ki=4.0, kd=0.0), period_ms=20,
            test_duration_us=2000000)])
        if mode == 'discrete_pi':
            user.motor_test['motors'][0].pid = DiscretePI(0.6, 0.08)
        output = bytearray()
        def write(chunk):
            chunk = chunk[:17]
            output.extend(chunk)
            return len(chunk)
        user.serial = SimpleNamespace(write=write)
        for _ in range(200):
            user.transmit_sample()
            if not user.current_test:
                break
        self.assertEqual(user.current_test, '')
        lines = iter(output.decode().splitlines())
        reader = SimpleNamespace(read=lambda deadline: next(lines))
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / 'trial.csv'
            _, config, rows = plotter.collect_dataset(reader, 'right', 1.5, 0.6, filename)
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[1], [20000.0, 1.5, 0.1, 0.2, 1.9])
            self.assertEqual(float(config['ki']), 0.08 if mode == 'discrete_pi' else 4.0)
            self.assertEqual(config['mode'], mode)
            self.assertEqual(config['feedback'], feedback)
            self.assertEqual(next(lines), 'Ready for next test')
            plotter.save_plot(Path(directory) / 'trial.png', 'right', 1.5, 0.6, config, rows)
            self.assertGreater((Path(directory) / 'trial.png').stat().st_size, 0)


if __name__ == '__main__':
    unittest.main()
