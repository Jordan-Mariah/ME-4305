
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace, MethodType
import unittest
from unittest.mock import patch
from observer import Observer
from discrete_pi import DiscretePI


def load_coordinator(clock):
    time = ModuleType('time')
    time.ticks_us = lambda: clock[0]
    time.ticks_diff = lambda a, b: ((a - b + (1 << 29)) % (1 << 30)) - (1 << 29)
    time.ticks_add = lambda a, b: (a + b) % (1 << 30)
    modules = {'time': time}
    for module, name in [('driver', 'Motor'), ('encoder', 'Encoder'),
                         ('tshare', 'Share'), ('PIDcontroler', 'PID')]:
        stub = ModuleType(module)
        setattr(stub, name, object)
        modules[module] = stub
    modules['line_sensor'] = ModuleType('line_sensor')
    spec = importlib.util.spec_from_file_location(
        'motor_under_test', Path(__file__).resolve().parents[1] / 'taskmotor.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class Share:
    def __init__(self, value):
        self.value = value

    def set(self, value):
        self.value = value


class Experiment:
    def __init__(self, module, left, right):
        self.shared = dict(motors=(left, right), active=False, selected=None,
                           start=None, ready=[None, None])
        self.left = left
        for item in (left, right):
            item.experiment = self.shared
            for name in ('start_test', 'finish_test', 'run_test'):
                setattr(item, name, MethodType(getattr(module.TaskMotor, name), item))
            for name in ('REST_SPEED_RAD_S', 'REST_TIME_US', 'SETTLE_TIMEOUT_US'):
                setattr(item, name, getattr(module.TaskMotor, name))

    @property
    def active(self):
        return self.shared['active']

    def request(self, selected, speed, kp):
        return self.left.start_test(selected, speed, kp)

    def finish(self, status):
        self.left.finish_test(status)

    def tick(self, item):
        item.run_test()


def motor(name, capacity=102):
    output = SimpleNamespace(effort=0, enabled=False)
    output.set_effort = lambda value: setattr(output, 'effort', value)
    output.enable = lambda: setattr(output, 'enabled', True)
    output.disable = lambda: setattr(output, 'enabled', False)
    pid = SimpleNamespace(kp=None, resets=0)
    pid.set_kp = lambda value: setattr(pid, 'kp', value)
    pid.reset = lambda: setattr(pid, 'resets', pid.resets + 1)
    data = {key: [0] * capacity for key in
            ('time_us', 'setpoint_rad_s', 'velocity_rad_s', 'estimated_velocity_rad_s', 'voltage_V')}
    data.update(count=0, status='completed')
    item = SimpleNamespace(name=name, motor=output, pid=pid, mode=Share('stop'),
                           data=Share(data), test_flag=Share(False),
                           actual_velocity=Share(0.0), period_ms=20,
                           observer=Observer(), position_rad=4.0,
                           test_duration_us=2_000_000, S6_SETTLE=6,
                           S8_TAKE_SAMPLES=8, S0_IDLE=0, commands=[])
    def control(speed):
        item.commands.append(speed)
        output.set_effort(25)
        return 2.0
    item.control_velocity = control
    return item


class CoordinatorChecks(unittest.TestCase):
    def setUp(self):
        self.clock = [0]
        self.left, self.right = motor('left'), motor('right')
        self.module = load_coordinator(self.clock)
        self.run = Experiment(self.module, self.left, self.right)

    def tick(self, now):
        self.clock[0] = now % (1 << 30)
        for item in (self.left, self.right):
            if self.run.active:
                self.run.tick(item)

    def start(self, selected='right'):
        self.assertTrue(self.run.request(selected, 1.5, 0.6))
        for now in range(0, 340_000, 20_000):
            self.tick(now)

    def assert_stopped(self, status):
        self.assertFalse(self.run.active)
        for item in (self.left, self.right):
            self.assertEqual(item.motor.effort, 0)
            self.assertFalse(item.motor.enabled)
            self.assertFalse(item.test_flag.value)
            self.assertEqual(item.data.value['status'], status)

    def test_complete_without_pc_and_selected_logging(self):
        self.start()
        for now in range(340_000, 2_340_000, 20_000):
            self.tick(now)
        self.assert_stopped('completed')
        self.assertEqual(self.left.commands, self.right.commands)
        self.assertTrue(all(v == 1.5 for v in self.left.commands))
        self.assertEqual(self.left.data.value['count'], 0)
        self.assertEqual(self.right.data.value['count'], 100)
        self.assertEqual(self.right.data.value['time_us'][0], 0)

    def test_abort_stops_both_immediately(self):
        self.start()
        self.right.test_flag.set(False)
        self.tick(340_000)
        self.assert_stopped('aborted')

    def test_moving_wheel_blocks_start_and_times_out(self):
        self.left.actual_velocity.value = 1.0
        self.run.request('left', 1.5, 0.6)
        self.tick(0)
        self.tick(500_000)
        self.assertFalse(self.right.motor.enabled)
        self.tick(5_000_000)
        self.assert_stopped('settle_timeout')

    def test_buffer_full_stops_both(self):
        self.right.data.value['time_us'] = [0]
        self.start()
        self.tick(340_000)
        self.assert_stopped('buffer_full')

    def test_busy_and_invalid_requests(self):
        with self.assertRaises(ValueError):
            self.run.request('left', -1.0, 0.6)
        self.run.request('left', 1.5, 0.6)
        self.assertFalse(self.run.request('right', 13.0, 0.6))
        self.run.finish('aborted')
        self.left.data.value['count'] = 1
        self.assertFalse(self.run.request('left', 1.5, 0.6))

    def test_clock_rollover_during_settling(self):
        base = (1 << 30) - 100_000
        self.clock[0] = base
        self.run.request('left', 1.5, 0.6)
        for offset in range(0, 340_000, 20_000):
            self.tick(base + offset)
        self.assertTrue(self.left.commands)
        self.assertTrue(self.right.commands)
        self.run.finish('aborted')

    def test_logs_current_estimate_then_advances_both_observers(self):
        self.start()
        self.assertEqual(self.right.data.value['estimated_velocity_rad_s'][0], 0.0)
        self.assertNotEqual(self.left.observer.velocity(), 0.0)
        self.assertAlmostEqual(self.left.observer.velocity(), self.right.observer.velocity())
        current = self.right.observer.velocity()
        self.tick(340_000)
        self.assertEqual(self.right.data.value['estimated_velocity_rad_s'][1], current)
        self.run.finish('completed')
        self.right.data.value['count'] = 0
        self.start()
        self.assertEqual(self.right.data.value['estimated_velocity_rad_s'][0], 0.0)

    def test_controller_uses_encoder_and_returns_final_clipped_voltage(self):
        received = []
        item = SimpleNamespace(actual_velocity=Share(1.25),
                               velocity_feedback="encoder",
                               battery_voltage=8.0, driver_sign=-1,
                               pid=SimpleNamespace(next_output=lambda target, current:
                                                   received.append((target, current)) or 20.0),
                               motor=SimpleNamespace(set_effort=lambda effort:
                                                     max(-50.0, min(50.0, effort))))
        voltage = self.module.TaskMotor.control_velocity(item, 13.0)
        self.assertEqual(received, [(13.0, 1.25)])
        self.assertEqual(voltage, 4.0)
        item.velocity_feedback = "observer"
        item.observer = Observer()
        item.observer.x[0, 0] = 2.5
        self.module.TaskMotor.control_velocity(item, 13.0)
        self.assertEqual(received[-1], (13.0, 2.5))
        self.assertEqual(item.actual_velocity.value, 1.25)

    def test_feedback_keeps_accumulated_signed_radians(self):
        item = SimpleNamespace(encoder=SimpleNamespace(update=lambda: None,
                               get_velocity=lambda: 100.0,
                               get_position=lambda: 5000.0),
                               encoder_sign=-1, counts_per_revolution=1000,
                               wheel_radius_cm=3.5, actual_velocity=Share(0.0),
                               position_cm=Share(0.0))
        self.module.TaskMotor.feedback(item)
        self.assertAlmostEqual(item.position_rad, -10 * self.module.pi)
        self.assertAlmostEqual(item.position_cm.value, item.position_rad * 3.5)

    def test_pi_gate_uses_driver_clipping_and_observer_feedback(self):
        pi = DiscretePI(1.0, 0.1)
        item = SimpleNamespace(actual_velocity=Share(99.0), observer=Observer(),
                               velocity_feedback='observer', battery_voltage=8.0,
                               driver_sign=-1, pid=pi,
                               motor=SimpleNamespace(set_effort=lambda effort:
                                   max(-25.0, min(25.0, effort))))
        voltage = self.module.TaskMotor.control_velocity(item, 4.0)
        self.assertEqual(voltage, 2.0)
        self.assertEqual(pi.error, 4.0)
        self.assertEqual(pi.integral, 0.0)


if __name__ == '__main__':
    unittest.main()
