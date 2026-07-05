"""控制器统一接口。

所有新控制算法都继承 BaseController，并返回 ControlResult。
"""

from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np


@dataclass
class ControlResult:
    torque: np.ndarray
    log: Dict[str, np.ndarray] = field(default_factory=dict)


class BaseController:
    """所有可替换控制算法需要实现的最小接口。"""

    name = "base"

    def reset(self, q0: np.ndarray, dq0: Optional[np.ndarray] = None) -> None:
        """实验开始前初始化控制器内部状态。"""

    def get_params(self) -> Dict[str, object]:
        return {}

    def compute(
        self,
        t: float,
        q: np.ndarray,
        dq: np.ndarray,
        xr: np.ndarray,
        dxr: np.ndarray,
        ddxr: np.ndarray,
    ) -> ControlResult | Tuple[np.ndarray, Dict[str, np.ndarray]]:
        raise NotImplementedError


def as_control_result(value) -> ControlResult:
    if isinstance(value, ControlResult):
        return value
    torque, log = value
    return ControlResult(np.asarray(torque, dtype=float), dict(log or {}))
