import unittest

import numpy as np

from torque_platform.safety import SafetyConfig, SafetyMonitor


class SafetyMonitorTests(unittest.TestCase):
    def test_clips_torque_to_per_joint_limits(self):
        monitor = SafetyMonitor(SafetyConfig(torque_limit=[2.0, 1.0]))
        monitor.reset(np.zeros(2))
        result = monitor.limit_torque(0.0, [3.0, -4.0])
        np.testing.assert_allclose(result.torque, [2.0, -1.0])
        self.assertFalse(result.stop)
        self.assertTrue(result.events)

    def test_nonfinite_torque_stops(self):
        monitor = SafetyMonitor(SafetyConfig(torque_limit=2.0))
        monitor.reset(np.zeros(2))
        result = monitor.limit_torque(0.0, [np.nan, 0.0])
        self.assertTrue(result.stop)
        np.testing.assert_allclose(result.torque, np.zeros(2))

    def test_invalid_limit_fails_closed(self):
        monitor = SafetyMonitor(SafetyConfig(torque_limit=float("nan")))
        monitor.reset(np.zeros(2))
        with self.assertRaisesRegex(ValueError, "有限"):
            monitor.limit_torque(0.0, [1.0, -1.0])

    def test_position_bound_stops(self):
        monitor = SafetyMonitor(SafetyConfig(position_bound=0.2))
        monitor.reset(np.zeros(2))
        message = monitor.check_state(0.1, [0.21, 0.0], [0.0, 0.0])
        self.assertIn("[STOP]", message)
        self.assertIn("POSITION_BOUND", message)


if __name__ == "__main__":
    unittest.main()
