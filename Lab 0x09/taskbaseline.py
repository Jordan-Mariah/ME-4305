from array import array
from math import pi
from time import ticks_us, ticks_diff


class TaskBaseline:
    def __init__(self, left_motor, right_motor, left_encoder, right_encoder,
                 left_pid, right_pid, serial, battery_voltage,
                 counts_per_revolution, left_driver_sign=-1,
                 right_driver_sign=-1, left_encoder_sign=-1,
                 right_encoder_sign=-1, period_ms=30, duration_ms=2000,
                 setpoint_rad_s=1.5, selected_motor="left", disable_flags=(),
                 voltage_limit=None, surface="unreported", loading="unreported",
                 battery_condition="unreported"):
        self.motors = (left_motor, right_motor)
        self.encoders = (left_encoder, right_encoder)
        self.pids = (left_pid, right_pid)
        self.serial = serial
        self.battery_voltage = battery_voltage
        self.voltage_limit = battery_voltage if voltage_limit is None else voltage_limit
        self.counts_per_revolution = counts_per_revolution
        self.driver_signs = (left_driver_sign, right_driver_sign)
        self.encoder_signs = (left_encoder_sign, right_encoder_sign)
        self.period_ms = period_ms
        self.period_us = period_ms * 1000
        self.duration_us = duration_ms * 1000
        self.setpoint = setpoint_rad_s
        self.selected_motor = selected_motor
        self.disable_flags = disable_flags
        self.surface = surface
        self.loading = loading
        self.battery_condition = battery_condition
        capacity = duration_ms // period_ms + 3
        self.times = array("L", [0] * capacity)
        self.setpoints = array("f", [0.0] * capacity)
        self.velocities = array("f", [0.0] * capacity)
        self.voltages = array("f", [0.0] * capacity)
        self.feedback = [0.0, 0.0]
        self.count = 0
        self.state = "idle"
        self.entry = ""
        self.number = ""
        self.exit_initiated = False
        self.stop_motors()

    def stop_motors(self):
        for motor in self.motors:
            motor.set_effort(0)
            motor.disable()

    def update_feedback(self):
        for i in range(2):
            self.encoders[i].update()
            self.feedback[i] = (self.encoder_signs[i] * self.encoders[i].get_velocity()
                                * 2 * pi / self.counts_per_revolution)

    def start(self, selected_motor="left"):
        self.stop_motors()
        for pid in self.pids:
            pid.reset()
        self.selected_motor = selected_motor
        self.count = 0
        self.settle_start = ticks_us()
        self.last_update = self.settle_start
        self.state = "settling"

    def begin_step(self):
        for pid in self.pids:
            pid.reset()
        for motor in self.motors:
            motor.enable()
        self.test_start = ticks_us()
        self.last_update = self.test_start
        self.state = "running"
        self.record_step(0)

    def record_step(self, elapsed):
        voltage = [0.0, 0.0]
        for i in range(2):
            requested = self.pids[i].next_output(self.setpoint, self.feedback[i])
            voltage[i] = max(-self.voltage_limit, min(self.voltage_limit, requested))
        for i in range(2):
            duty = 100 * voltage[i] / self.battery_voltage
            self.motors[i].set_effort(duty * self.driver_signs[i])
        selected = 0 if self.selected_motor == "left" else 1
        self.times[self.count] = elapsed
        self.setpoints[self.count] = self.setpoint
        self.velocities[self.count] = self.feedback[selected]
        self.voltages[self.count] = voltage[selected]
        self.count += 1

    def finish(self, status="cancelled"):
        self.stop_motors()
        for pid in self.pids:
            pid.reset()
        self.status = status
        self.state = "exporting"

    def send_line(self, line):
        data = (line + "\r\n").encode()
        while data:
            sent = self.serial.write(data[:128])
            if sent:
                data = data[sent:]

    def export_data(self):
        selected = 0 if self.selected_motor == "left" else 1
        pid = self.pids[selected]
        metadata = {
            "motor": self.selected_motor, "mode": "P",
            "setpoint_rad_s": self.setpoint, "kp": pid.kp,
            "ki": pid.ki, "kd": pid.kd, "ff_gain": pid.ff_gain,
            "left_kp": self.pids[0].kp, "right_kp": self.pids[1].kp,
            "left_ki": self.pids[0].ki, "right_ki": self.pids[1].ki,
            "left_ff_gain": self.pids[0].ff_gain,
            "right_ff_gain": self.pids[1].ff_gain,
            "motor_period_ms": self.period_ms,
            "controller_period_ms": self.period_ms,
            "velocity_window": "encoder_update_interval",
            "nominal_velocity_window_ms": self.period_ms,
            "voltage_min_V": -self.voltage_limit,
            "voltage_max_V": self.voltage_limit,
            "battery_voltage_V": self.battery_voltage,
            "duration_ms": self.duration_us // 1000,
            "surface": self.surface, "loading": self.loading,
            "battery_condition": self.battery_condition,
            "starting_condition": "stopped_below_0.2_rad_s_after_500_ms",
            "corrections": "disabled",
            "controller_structure": "P_plus_optional_I_and_feedforward",
            "pid_integral_time_unit": "microseconds",
            "derivative_implemented": False,
        }
        self.send_line("Begin data: " + ",".join(
            "{}={}".format(key, value) for key, value in metadata.items()))
        self.send_line("time_us,setpoint_rad_s,velocity_rad_s,limited_voltage_V")
        for i in range(self.count):
            self.send_line("{},{},{},{}".format(
                self.times[i], self.setpoints[i], self.velocities[i], self.voltages[i]))
        self.send_line("End of data: " + self.status)
        self.send_line("Ready for next test")
        self.state = "idle"

    def process_command(self, char):
        if char.lower() in ("s", "e"):
            self.entry = ""
            self.finish()
            self.exit_initiated = char.lower() == "e"
        elif self.state == "idle":
            if self.entry:
                if char in "\r\n":
                    value = float(self.number)
                    if self.entry == "v":
                        self.setpoint = value
                    else:
                        for pid in self.pids:
                            pid.set_kp(value)
                    self.send_line("Value set to {}".format(value))
                    self.entry = ""
                else:
                    self.number += char
            elif char.lower() in ("v", "p"):
                self.entry = char.lower()
                self.number = ""
                self.send_line("Enter velocity setpoint [rad/s]:" if self.entry == "v"
                               else "Enter Kp [V/(rad/s)]:")
            elif char.lower() in ("l", "r", "b"):
                self.start("right" if char.lower() == "r" else "left")

    def run(self):
        while not self.exit_initiated:
            for flag in self.disable_flags:
                flag.set(False)
            now = ticks_us()
            if self.state == "running" and ticks_diff(now, self.test_start) >= self.duration_us:
                self.finish("completed")
            elif self.state in ("settling", "running") and ticks_diff(now, self.last_update) >= self.period_us:
                self.last_update = now
                self.update_feedback()
                if self.state == "settling":
                    if ticks_diff(now, self.settle_start) >= 500000 and max(abs(v) for v in self.feedback) < 0.2:
                        self.begin_step()
                else:
                    self.record_step(ticks_diff(now, self.test_start))
            if self.serial.any():
                self.process_command(self.serial.read(1).decode())
            if self.state == "exporting":
                self.export_data()
            yield self.state
