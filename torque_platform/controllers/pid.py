"""PID 力矩控制器。

用于基础轨迹跟踪，也可作为新流程的低风险测试控制器。
"""

import numpy as np

from .base import BaseController, ControlResult


class PIDController(BaseController):
    name = "pid"

    def __init__(
        self,
        kp=(20.0, 17.0),
        ki=(0.4, 0.5),
        kd=(1.5, 1.5),
        integral_limit=1.0,
        torque_limit=50.0,
        phase_lead_s=(1.47, 1.12),
    ):
        self.kp = np.diag(kp)
        self.ki = np.diag(ki)
        self.kd = np.diag(kd)
        self.integral_limit = float(integral_limit)
        self.torque_limit = float(torque_limit)
        self.phase_lead_s = np.asarray(phase_lead_s, dtype=float)
        self.reference = None
        self.integral = np.zeros(2)
        self.last_t = None

    def set_reference(self, reference):
        self.reference = reference

    def reset(self, q0, dq0=None):
        self.integral = np.zeros_like(q0, dtype=float)
        self.last_t = None

    def compute(self, t, q, dq, xr, dxr, ddxr):
        if self.reference is not None and np.any(self.phase_lead_s):
            xr, dxr, ddxr = self.reference.sample(t, self.phase_lead_s)

        error = xr - q
        d_error = dxr - dq
        if self.last_t is not None:
            dt = max(t - self.last_t, 0.0)
            self.integral += error * dt
            self.integral = np.clip(
                self.integral, -self.integral_limit, self.integral_limit
            )
        self.last_t = t

        torque = self.kp @ error + self.ki @ self.integral + self.kd @ d_error
        torque = np.clip(torque, -self.torque_limit, self.torque_limit)
        return ControlResult(
            torque=torque,
            log={
                "error": error,
                "pid_i": self.integral.copy(),
                "pid_d": d_error,
            },
        )
