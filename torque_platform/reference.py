"""参考轨迹生成。

当前提供两关节正弦轨迹，内部计算统一使用弧度。
"""

import math

import numpy as np


class SineReference:
    """两关节正弦参考轨迹。"""

    def __init__(self, center, amplitude_deg=(15.0, 15.0), period_s=(5.0, 5.0)):
        self.center = np.asarray(center, dtype=float)
        self.amplitude = np.radians(np.asarray(amplitude_deg, dtype=float))
        self.period_s = np.asarray(period_s, dtype=float)
        self.omega = 2.0 * math.pi / self.period_s

    def sample(self, t, phase_lead_s=None):
        if phase_lead_s is None:
            phase_lead_s = 0.0
        t_eff = t + np.asarray(phase_lead_s, dtype=float)

        xr = self.center + self.amplitude * np.sin(self.omega * t_eff)
        dxr = self.amplitude * self.omega * np.cos(self.omega * t_eff)
        ddxr = -self.amplitude * (self.omega**2) * np.sin(self.omega * t_eff)
        return xr, dxr, ddxr
