"""新控制器模板。

复制本文件并改名，例如 my_controller.py。只要类继承 BaseController，
并设置唯一的 name，registry.py 会自动发现它。
"""

import numpy as np

from .base import BaseController, ControlResult


class NewController(BaseController):
    """控制器示例，请在复制后改为算法的用途和局限说明。"""

    name = "new_controller"
    supports_realtime = True
    recommended_min_dt = 0.001

    def __init__(self, dt=0.001, torque_limit=50.0):
        self.dt = float(dt)
        self.torque_limit = float(torque_limit)

    def reset(self, q0, dq0=None):
        # 在此初始化每次实验使用的内部状态，不要沿用上一次实验的状态。
        self.state = {}

    def compute(self, t, q, dq, xr, dxr, ddxr):
        # 输入使用 SI 单位：rad、rad/s、rad/s^2。输出使用 N*m，
        # 且力矩向量长度必须与 q 和 TORQUE_JOINTS 的长度一致。
        error = xr - q
        torque = np.zeros_like(q)
        torque = np.clip(torque, -self.torque_limit, self.torque_limit)
        return ControlResult(torque=torque, log={"error": error})
