"""参考轨迹生成。

默认提供可用于任意受控关节数量的正弦轨迹，内部计算统一使用弧度。
如需其他轨迹，可修改本文件或新建轨迹类，并保持 ``sample()`` 的返回格式不变。
"""

import math

import numpy as np


class SineReference:
    """按关节分别设置中心、幅值和周期的正弦参考轨迹。"""

    def __init__(self, center, amplitude_deg=(15.0, 15.0), period_s=(5.0, 5.0)):
        self.center = np.asarray(center, dtype=float)
        self.amplitude = np.radians(np.asarray(amplitude_deg, dtype=float))
        self.period_s = np.asarray(period_s, dtype=float)
        self.omega = 2.0 * math.pi / self.period_s

    def sample(self, t, phase_lead_s=None):
        """返回时刻 ``t`` 的参考位置、速度和加速度，单位均采用 SI 制。"""

        if phase_lead_s is None:
            phase_lead_s = 0.0
        t_eff = t + np.asarray(phase_lead_s, dtype=float)

        xr = self.center + self.amplitude * np.sin(self.omega * t_eff)
        dxr = self.amplitude * self.omega * np.cos(self.omega * t_eff)
        ddxr = -self.amplitude * (self.omega**2) * np.sin(self.omega * t_eff)
        return xr, dxr, ddxr
