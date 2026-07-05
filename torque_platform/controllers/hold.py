"""保持初始关节位置的低增益 PD 控制器。"""

import numpy as np

from .base import BaseController, ControlResult


class HoldController(BaseController):
    name = "hold"

    def __init__(
        self,
        kp=(8.0, 8.0),
        kd=(0.8, 0.8),
        torque_limit=10.0,
    ):
        self.kp = np.asarray(kp, dtype=float)
        self.kd = np.asarray(kd, dtype=float)
        self.torque_limit = float(torque_limit)
        self.q_hold = None

    def reset(self, q0, dq0=None):
        self.q_hold = np.asarray(q0, dtype=float).copy()

    def get_params(self):
        return {
            "kp": self.kp,
            "kd": self.kd,
            "torque_limit": self.torque_limit,
        }

    def compute(self, t, q, dq, xr, dxr, ddxr):
        error = self.q_hold - q
        torque = self.kp * error - self.kd * dq
        torque = np.clip(torque, -self.torque_limit, self.torque_limit)
        return ControlResult(
            torque=torque,
            log={
                "error": error,
                "hold_target": self.q_hold.copy(),
            },
        )
