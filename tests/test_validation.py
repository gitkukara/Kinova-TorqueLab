import unittest
from types import SimpleNamespace

from torque_platform.validation import validate_experiment_args


def valid_args(**overrides):
    values = {
        "ip": "192.168.1.10",
        "username": "admin",
        "password": "admin",
        "duration": 5.0,
        "dt": 0.001,
        "torque_limit": 10.0,
        "cyclic_timeout_ms": 3,
        "log_every": 1,
        "torque_joints": [3, 5],
        "start_angles_deg": [0.0] * 7,
        "reference_center_rad": [0.0, 0.0],
        "reference_amplitude_deg": [5.0, 5.0],
        "reference_period_s": [5.0, 5.0],
        "safety_torque_limit": [8.0, 4.0],
        "torque_rate_limit": None,
        "position_bound": 0.45,
        "velocity_bound": 1.0,
        "stop_on_position_bound": True,
        "stop_on_velocity_bound": True,
        "stop_on_loop_overrun": True,
        "loop_overrun_limit_s": 0.005,
        "loop_overrun_max_consecutive": 3,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class ValidationTests(unittest.TestCase):
    def test_valid_configuration(self):
        validate_experiment_args(valid_args())

    def test_reports_multiple_actionable_errors(self):
        args = valid_args(
            ip="192.168.1.x",
            torque_joints=[3, 3],
            reference_period_s=[-1.0, 5.0],
            torque_limit=float("nan"),
        )

        with self.assertRaises(ValueError) as caught:
            validate_experiment_args(args)

        message = str(caught.exception)
        self.assertIn("placeholder", message)
        self.assertIn("duplicates", message)
        self.assertIn("reference_period_s", message)
        self.assertIn("finite", message)

    def test_rejects_wrong_per_joint_limit_length(self):
        with self.assertRaisesRegex(ValueError, "safety_torque_limit"):
            validate_experiment_args(valid_args(safety_torque_limit=[1.0, 2.0, 3.0]))

    def test_disabled_bound_may_be_none(self):
        validate_experiment_args(
            valid_args(position_bound=None, stop_on_position_bound=False)
        )


if __name__ == "__main__":
    unittest.main()
