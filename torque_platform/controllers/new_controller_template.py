"""新控制器模板。

复制本文件并改名，例如 my_controller.py。只要类继承 BaseController，
并设置唯一的 name，registry.py 会自动发现它。
"""

import numpy as np

from .base import BaseController, ControlResult


class NewController(BaseController):
    """Example controller; replace this with its purpose and limitations."""

    name = "new_controller"
    supports_realtime = True
    recommended_min_dt = 0.001

    def __init__(self, dt=0.001, torque_limit=50.0):
        self.dt = float(dt)
        self.torque_limit = float(torque_limit)

    def reset(self, q0, dq0=None):
        # Initialize all per-run state here. Do not reuse state across runs.
        self.state = {}

    def compute(self, t, q, dq, xr, dxr, ddxr):
        # Inputs use SI units: rad, rad/s, rad/s^2. Output uses N*m and must
        # have the same length as q and TORQUE_JOINTS.
        error = xr - q
        torque = np.zeros_like(q)
        torque = np.clip(torque, -self.torque_limit, self.torque_limit)
        return ControlResult(torque=torque, log={"error": error})
