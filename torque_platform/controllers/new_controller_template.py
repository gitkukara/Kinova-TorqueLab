"""新控制器模板。

复制本文件并改名，例如 my_controller.py。只要类继承 BaseController，
并设置唯一的 name，registry.py 会自动发现它。
"""

import numpy as np

from .base import BaseController, ControlResult


class NewController(BaseController):
    """单文件控制器示例。"""

    name = "new_controller"
    supports_realtime = True
    recommended_min_dt = 0.001

    def __init__(self, dt=0.001, torque_limit=50.0):
        self.dt = float(dt)
        self.torque_limit = float(torque_limit)

    def reset(self, q0, dq0=None):
        # 放 Matlab 主循环前的初始化内容。
        self.state = {}

    def compute(self, t, q, dq, xr, dxr, ddxr):
        # 只迁移控制律计算；q、dq 来自实机反馈。
        error = xr - q
        torque = np.zeros_like(q)
        torque = np.clip(torque, -self.torque_limit, self.torque_limit)
        return ControlResult(torque=torque, log={"error": error})
