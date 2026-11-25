import unittest
from discrete_pi import DiscretePI


class PIChecks(unittest.TestCase):
    def test_current_command_precedes_unsaturated_accumulation(self):
        pi = DiscretePI(0.6, 0.08)
        command = pi.next_output(2.0, 1.0)
        self.assertAlmostEqual(command, 0.6)
        self.assertEqual(pi.integral, 0.0)
        pi.update_integral(command)
        self.assertEqual(pi.integral, 1.0)
        self.assertAlmostEqual(pi.next_output(2.0, 1.0), 0.68)

    def test_upper_and_lower_saturation_hold(self):
        for error in (20.0, -20.0):
            pi = DiscretePI(1.0, 0.1)
            request = pi.next_output(error, 0.0)
            pi.update_integral(max(-8.0, min(8.0, request)))
            self.assertEqual(pi.integral, 0.0)

    def test_error_reversal_unwinds_while_still_saturated(self):
        for sign in (1.0, -1.0):
            pi = DiscretePI(1.0, 0.1)
            pi.integral = sign * 100.0
            request = pi.next_output(-sign, 0.0)
            self.assertAlmostEqual(request, sign * 9.0)
            pi.update_integral(sign * 8.0)
            self.assertEqual(pi.integral, sign * 99.0)

    def test_exact_limit_allows_accumulation(self):
        pi = DiscretePI(1.0, 0.1)
        pi.update_integral(pi.next_output(8.0, 0.0))
        self.assertEqual(pi.integral, 8.0)

    def test_reset_and_independence(self):
        left, right = DiscretePI(0.6, 0.08), DiscretePI(0.6, 0.08)
        left.update_integral(left.next_output(1.0, 0.0))
        self.assertEqual(right.integral, 0.0)
        left.reset()
        self.assertEqual((left.integral, left.error, left.requested_voltage), (0.0, 0.0, 0.0))

    def test_invalid_gains(self):
        for kp, ki in ((-1, 0.1), (1, -0.1), (float('nan'), 0.1)):
            with self.assertRaises(ValueError):
                DiscretePI(kp, ki)


if __name__ == '__main__':
    unittest.main()
