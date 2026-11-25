try:
    from ulab import numpy as np
except ImportError:
    import numpy as np


# Lab 0x09 - Observer Numerical Verification
# Selected observer parameters:
# Km = 3.5
# tau = 0.047
# Ts = 0.02
# t_set = 0.30
# zeta = 0.7
# s3 = -80
#observer state
Ao = np.array([
    [0.65342212771441543, -8.8387999412756564, 0.016289159997422472],
    [0.016289159997422472, 0.023990734861125906, 0.00017440948012114373],
    [0.0, -272.24589895928926, 1.0],
])



# Observer input matrix- how voltage and measured position effect next estimate
# limitted voltage and encoder position
Bo = np.array([
    [1.2130225529995458, 8.8387999412756564],
    [0.012987940009021343, 0.97600926513887409],
    [0.0, 272.24589895928926],
])


# which estimates are reported
Co = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
])


# Direct feedthrough matrix
Do = np.array([
    [0.0, 0.0],
    [0.0, 0.0],
    [0.0, 0.0],
])

OBSERVER_PERIOD_MS = 20

# vectors test from MATLAB
# Calculate current observer output
# Calculate next observer state
# Expected MATLAB results from updated Live Script
class Observer:
    def __init__(self):

        # State order array [vel, pos, disturb]
        #initial state estimate = 0
        self.x = np.zeros((3, 1))


        # Input array w = [uk, pos_meas]
        self.w = np.zeros((2, 1))


        #array for the observer to calculate
        self.x_next = np.zeros((3, 1))

    #simply used to read the velocity
    def velocity(self):
        return float(self.x[0, 0])

    def next_estimate(self, encoder_position: float, voltage: float):

        self.w[0, 0] = voltage
        self.w[1, 0] = encoder_position
    

        for i in range(3):
            value = 0.0
            for j in range(3):
                value += Ao[i, j] * self.x[j, 0]
            for j in range(2):
                value += Bo[i, j] * self.w[j, 0]
            self.x_next[i, 0] = value


        self.x, self.x_next = self.x_next, self.x
        return self.velocity() #return velocity estimate


    #used to reset observer when motor is at rest
    def reset(self, encoder_position: float):

        # Call while the motor is at rest.
        #initial velocity estimate = 0
        #set initial position 
        #iniital disturbance estimate= 0
        self.x[0, 0] = 0.0

        self.x[1, 0] = encoder_position

        self.x[2, 0] = 0.0

        self.w[0, 0] = 0.0
        self.w[1, 0] = encoder_position

        for i in range(3):
            self.x_next[i, 0] = self.x[i, 0]


def numeric_check():
    from math import isfinite
    observer = Observer()
    for i, value in enumerate((1.0, 0.2, -0.5)):
        observer.x[i, 0] = value
    observer.w[0, 0] = 2.0
    observer.w[1, 0] = 0.21
    output = np.dot(Co, observer.x) + np.dot(Do, observer.w)
    expected_output = (1.0, 0.2, -0.5)
    observer.next_estimate(0.21, 2.0)
    expected_next = (3.15971065312755, 0.251937927926793, 2.22245898959289)
    for label, actual, expected in (("Current output", output, expected_output),
                                    ("Next state", observer.x, expected_next)):
        for i in range(3):
            value = float(actual[i, 0])
            difference = abs(value - expected[i])
            print("%s[%d]: measured=%.8f MATLAB=%.8f diff=%.8g" %
                  (label, i, value, expected[i], difference))
            if not isfinite(value) or difference > 1e-5:
                raise AssertionError(label)
        print(label + ": PASS")
    print("ALL OBSERVER CHECKS PASSED")

if __name__ == "__main__":
    numeric_check()
    
