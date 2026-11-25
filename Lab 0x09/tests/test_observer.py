import unittest
from observer import Observer, Ao, Bo, Co, Do, np

def extended_check():

    from math import isfinite

    def check(label, actual, expected, tolerance=1e-5):
        actual = float(actual)
        difference = abs(actual - expected)
        if not isfinite(actual) or difference > tolerance:
            raise AssertionError("%s: actual=%r expected=%r" %
                                 (label, actual, expected))
        print("%s: PASS (diff=%.8g)" % (label, difference))

    observer = Observer()
    first = observer.x
    second = observer.x_next
    inputs = observer.w

    observer.reset(4.0)
    for i, expected in enumerate((0.0, 4.0, 0.0)):
        check("Reset state %d" % i, observer.x[i, 0], expected)
        check("Reset workspace %d" % i, observer.x_next[i, 0], expected)
    check("Reset voltage", observer.w[0, 0], 0.0)
    check("Reset position input", observer.w[1, 0], 4.0)

    for _ in range(3):
        observer.next_estimate(4.0, 0.0)
    for i, expected in enumerate((0.0, 4.0, 0.0)):
        check("Stationary state %d" % i, observer.x[i, 0], expected)

    for i, value in enumerate((1.0, 0.2, -0.5)):
        observer.x[i, 0] = value
    observer.w[0, 0] = 2.0
    observer.w[1, 0] = 0.21
    y_check = np.dot(Co, observer.x) + np.dot(Do, observer.w)
    for i, expected in enumerate((1.0, 0.2, -0.5)):
        check("MATLAB current output %d" % i, y_check[i, 0], expected)
    check("Current velocity", observer.velocity(), 1.0)

    result = observer.next_estimate(0.21, 2.0)
    expected_next = (3.15971065312755, 0.251937927926793,
                     2.22245898959289)
    for i, expected in enumerate(expected_next):
        check("MATLAB next state %d" % i, observer.x[i, 0], expected)
    check("Returned next velocity", result, expected_next[0])


    reference = (np.dot(Ao, observer.x)
                 + np.dot(Bo, np.array([[1.5], [0.25]])))
    observer.next_estimate(0.25, 1.5)
    for i in range(3):
        check("Repeated update %d" % i, observer.x[i, 0],
              float(reference[i, 0]), tolerance=1e-4)

    observer.reset(-1.25)
    other = Observer()
    other.reset(8.0)
    other.next_estimate(8.1, 1.0)
    for i, expected in enumerate((0.0, -1.25, 0.0)):
        check("Independent reset state %d" % i,
              observer.x[i, 0], expected)
    if not ((observer.x is first and observer.x_next is second)
            or (observer.x is second and observer.x_next is first)):
        raise AssertionError("State arrays were reallocated")
    if observer.w is not inputs:
        raise AssertionError("Input array was reallocated")
    print("Array reuse: PASS")
    print("ALL OBSERVER CHECKS PASSED")


class ObserverChecks(unittest.TestCase):
    def test_extended_checks(self):
        extended_check()
