
from ulab import numpy as np

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

# Observer input matrix
# Input order: [limited voltage, encoder position]
Bo = np.array([
    [1.2130225529995458, 8.8387999412756564],
    [0.012987940009021343, 0.97600926513887409],
    [0.0, 272.24589895928926],
])

# Observer output matrix
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

# Compare results with MATLAB
def compare_vector(label, measured, expected, tolerance=0.001):
    passed = True

    for i in range(len(expected)):
        difference = abs(float(measured[i]) - expected[i])

        print(
            "%s[%d]: measured=%.8f, MATLAB=%.8f, diff=%.8g"
            % (label, i, float(measured[i]), expected[i], difference)
        )

        if difference > tolerance:
            passed = False

    print(label + ": " + ("PASS" if passed else "FAIL"))
    return passed

output_ok = compare_vector("y_check", y_check, expected_y)

state_ok = compare_vector("x_next_check", x_next_check, expected_next)

if output_ok and state_ok:
    print("OVERALL: PASS")
else:
    print("OVERALL: FAIL")
