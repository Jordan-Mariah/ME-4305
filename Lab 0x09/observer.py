
#from ulab 
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

# vectors test from MATLAB
x_check = np.array([1.0, 0.2, -0.5])
w_check = np.array([2.0, 0.21])

# Calculate current observer output
y_check = np.dot(Co, x_check) + np.dot(Do, w_check)

# Calculate next observer state
x_next_check = np.dot(Ao, x_check) + np.dot(Bo, w_check)

print("Current output y_check:")
print(y_check)

print("Next state x_next_check:")
print(x_next_check)

# Expected MATLAB results from updated Live Script
expected_y = [1.0, 0.2, -0.5]

expected_next = [
    3.15971065312755,
    0.251937927926793,
    2.22245898959289
]

class Observer:
    def __init__(self):
        # State order array [vel, pos, disturb]
        self.x = (np.zeros((3, 1))) #initial state estimate = 0

        # Input array w = [uk, pos_meas]
        self.w = np.zeros((2, 1))

        #array for the observer to calculate
        self.x_next = (np.zeros((3, 1)))

    def next_estimate(self, encoder_position: float, voltage: float):
        self. w[0] = voltage
        self.w[1] = encoder_position
    
        self.x_next = np.dot(Ao, self.x_next) + np.dot(Bo, self.w)
        estimated_velocity = self.x_next[0]
        
        return estimated_velocity

    #used to reset observer when motor is at rest
    def reset(self, encoder_position: float):

        self.x_next = (np.zeros((3, 1)))
        # Call while the motor is at rest.
        #initial velocity estimate = 0
        self.x[0] = 0.0
        #set initial position 
        self.x[1] = encoder_position
        #iniital disturbance estimate= 0
        self.x[2] = 0.0

        self.w[0] = 0.0
        self.w[1] = encoder_position