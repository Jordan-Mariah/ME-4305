from math import isfinite


class DiscretePI:
    def __init__(self, kp, ki_sample):
        if not isfinite(ki_sample) or ki_sample < 0:
            raise ValueError("Integral gain must be finite and nonnegative")
        self.set_kp(kp)
        self.ki = float(ki_sample)
        self.kd = 0.0
        self.reset()

    def set_kp(self, kp):
        if not isfinite(kp) or kp < 0:
            raise ValueError("Proportional gain must be finite and nonnegative")
        self.kp = float(kp)

    def reset(self):
        self.integral = 0.0
        self.error = 0.0
        self.requested_voltage = 0.0

    def next_output(self, target, current):
        self.error = target - current
        self.requested_voltage = self.kp * self.error + self.ki * self.integral
        return self.requested_voltage

    def update_integral(self, applied_voltage):
        residual = self.requested_voltage - applied_voltage
        if residual * self.error <= 0:
            self.integral += self.error
