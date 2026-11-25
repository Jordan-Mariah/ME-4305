import csv
from pathlib import Path
import tempfile
import unittest
try:
    from compare_responses import CONFIGURATIONS, HEADERS, make_comparisons
except ModuleNotFoundError as error:
    if error.name != 'compare_responses':
        raise
    raise unittest.SkipTest('Optional compare_responses.py was removed')


class ComparisonChecks(unittest.TestCase):
    def write_runs(self, root):
        for speed in (1.5, 13.0):
            for mode, feedback, _ in CONFIGURATIONS:
                config = dict(motor='right', mode=mode, feedback=feedback,
                              setpoint_rad_s=speed, kp=0.6,
                              ki=0.08 if mode == 'discrete_pi' else 4.0,
                              kd=0.0, feedforward='disabled',
                              integral_convention='per_sample' if mode == 'discrete_pi' else 'time_weighted_seconds',
                              motor_period_ms=20, observer_period_ms=20,
                              battery_voltage_V=8.0, voltage_min_V=-8.0,
                              voltage_max_V=8.0, duration_ms=2000)
                path = root / ('%s_%s_%g.csv' % (mode, feedback, speed))
                with path.open('w', newline='') as stream:
                    stream.write('# Begin data: ' + ','.join('%s=%s' % item for item in config.items()) + '\n')
                    writer = csv.writer(stream)
                    writer.writerow(HEADERS)
                    writer.writerows([[0, speed, 0, 0, 2], [20000, speed, 0.1, 0.2, 2]])
                    stream.write('# End of data: completed\n')

    def test_six_runs_produce_two_figures(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_runs(root)
            outputs = make_comparisons(root, root / 'out', 'right', [1.5, 13.0])
            self.assertEqual(len(outputs), 2)
            for path in outputs:
                self.assertGreater(path.stat().st_size, 0)
                self.assertTrue(path.with_suffix('.txt').exists())

    def test_missing_run_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                make_comparisons(Path(directory), Path(directory) / 'out', 'right', [1.5, 13.0])

    def test_original_gain_change_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_runs(root)
            path = root / 'original_observer_1.5.csv'
            path.write_text(path.read_text().replace('kp=0.6', 'kp=0.7'))
            with self.assertRaises(ValueError):
                make_comparisons(root, root / 'out', 'right', [1.5, 13.0])


if __name__ == '__main__':
    unittest.main()
