import unittest

import numpy as np

from torque_platform.controllers.hold import HoldController
from torque_platform.controllers.pid import PIDController


class ControllerContractTests(unittest.TestCase):
    def setUp(self):
        self.q = np.array([0.1, -0.2])
        self.dq = np.array([0.02, -0.03])
        self.xr = np.array([0.15, -0.1])
        self.dxr = np.array([0.0, 0.0])
        self.ddxr = np.array([0.0, 0.0])

    def test_hold_returns_one_torque_per_joint(self):
        controller = HoldController()
        controller.reset(self.q, self.dq)
        result = controller.compute(
            0.0, self.q, self.dq, self.xr, self.dxr, self.ddxr
        )
        self.assertEqual(result.torque.shape, self.q.shape)
        self.assertTrue(np.all(np.isfinite(result.torque)))
        self.assertIn("error", result.log)

    def test_pid_returns_finite_torque_and_resets_integral(self):
        controller = PIDController(phase_lead_s=(0.0, 0.0))
        controller.reset(self.q, self.dq)
        controller.compute(0.0, self.q, self.dq, self.xr, self.dxr, self.ddxr)
        result = controller.compute(
            0.01, self.q, self.dq, self.xr, self.dxr, self.ddxr
        )
        self.assertEqual(result.torque.shape, self.q.shape)
        self.assertTrue(np.all(np.isfinite(result.torque)))
        self.assertTrue(np.any(controller.integral != 0.0))

        controller.reset(self.q, self.dq)
        np.testing.assert_allclose(controller.integral, np.zeros_like(self.q))


if __name__ == "__main__":
    unittest.main()
