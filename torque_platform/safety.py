"""力矩实验的通用安全检查和输出限幅。"""

from dataclasses import dataclass, field

import numpy as np


@dataclass
class SafetyConfig:
    torque_limit: object = 50.0
    torque_rate_limit: object = None
    position_bound: object = 0.45
    velocity_bound: object = 1.0
    loop_overrun_limit_s: float = 0.005
    loop_overrun_max_consecutive: int = 3
    stop_on_position_bound: bool = True
    stop_on_velocity_bound: bool = True
    stop_on_nonfinite_feedback: bool = True
    stop_on_nonfinite_torque: bool = True
    stop_on_loop_overrun: bool = True


@dataclass
class SafetyResult:
    torque: np.ndarray
    stop: bool = False
    reason: str = ""
    events: list = field(default_factory=list)


SAFETY_HINTS = {
    "POSITION_BOUND": "关节超出设定的位置窗口，请检查轨迹幅值、控制器增益和起始姿态。",
    "VELOCITY_BOUND": "关节速度超过设定上限，请检查控制器增益、轨迹速度和通信是否卡顿。",
    "FEEDBACK_NONFINITE": "反馈中出现 NaN 或 Inf，请检查周期反馈、API 状态和网络稳定性。",
    "TORQUE_NONFINITE": "控制器输出 NaN 或 Inf，请检查除法、矩阵运算和内部状态。",
    "TORQUE_DIMENSION": "控制器输出的力矩向量长度与 TORQUE_JOINTS 不一致。",
    "TORQUE_RATE_LIMITED": "力矩变化超过 TORQUE_RATE_LIMIT，下发前已进行变化率限制。",
    "TORQUE_CLIPPED": "力矩指令超过最终安全上限，下发前已进行限幅。",
    "LOOP_OVERRUN": "控制周期耗时过长，可能由通信延迟、控制器计算较慢或系统调度延迟引起。",
    "COMMUNICATION_REFRESH": "周期刷新失败，请检查网络、Kortex 连接、机械臂故障状态和实时 UDP 通路。",
    "CLEANUP_FAILED": "清理阶段发生异常，机械臂可能未完成零力矩、模式恢复或返回起始姿态。",
    "SAFETY_EVENT": "未分类的安全事件，请结合详细消息和相邻日志检查。",
}


def _limit_array(value, size, name, allow_none=False):
    if value is None and allow_none:
        return None
    array = np.asarray(value, dtype=float)
    if array.ndim == 0:
        array = np.full(size, float(array), dtype=float)
    else:
        array = array.reshape(-1)
    if array.size != size:
        raise ValueError(f"{name} 必须是单个数值，或包含 {size} 个关节对应值")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} 只能包含有限数值，不能出现 NaN 或 Inf")
    if np.any(array <= 0.0):
        raise ValueError(f"{name} 中的数值必须全部大于 0")
    return array


class SafetyMonitor:
    def __init__(self, config):
        self.config = config
        self.q_center = None
        self.previous_torque = None
        self.loop_overrun_count = 0
        self.event_history = []

    def reset(self, q0):
        self.q_center = np.asarray(q0, dtype=float).copy()
        self.previous_torque = np.zeros_like(self.q_center, dtype=float)
        self.loop_overrun_count = 0
        self.event_history = []

    def check_state(self, t, q, dq):
        q = np.asarray(q, dtype=float)
        dq = np.asarray(dq, dtype=float)

        if self.config.stop_on_nonfinite_feedback:
            if not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq)):
                return self._stop(t, "反馈中出现非有限数值")

        if self.q_center is not None and self.config.stop_on_position_bound:
            limit = _limit_array(self.config.position_bound, q.size, "position_bound")
            offset = np.abs(q - self.q_center)
            if np.any(offset > limit):
                return self._stop(
                    t,
                    f"位置超出边界：偏移={offset}，上限={limit}",
                )

        if self.config.stop_on_velocity_bound:
            limit = _limit_array(self.config.velocity_bound, dq.size, "velocity_bound")
            speed = np.abs(dq)
            if np.any(speed > limit):
                return self._stop(
                    t,
                    f"速度超出边界：速度={speed}，上限={limit}",
                )

        return ""

    def check_loop_timing(self, t, elapsed_s):
        if not self.config.stop_on_loop_overrun:
            return ""

        limit = float(self.config.loop_overrun_limit_s)
        if limit <= 0.0:
            return ""

        elapsed_s = float(elapsed_s)
        if elapsed_s <= limit:
            self.loop_overrun_count = 0
            return ""

        self.loop_overrun_count += 1
        max_count = max(int(self.config.loop_overrun_max_consecutive), 1)
        message = (
            f"控制周期超时 {self.loop_overrun_count}/{max_count}："
            f"耗时={elapsed_s:.6f}s，上限={limit:.6f}s"
        )

        if self.loop_overrun_count >= max_count:
            return self._stop(t, message)

        self._record(t, message)
        return ""

    def limit_torque(self, t, torque):
        torque = np.asarray(torque, dtype=float).reshape(-1)
        events = []

        if self.previous_torque is not None and torque.size != self.previous_torque.size:
            reason = (
                f"力矩向量长度不一致：实际为 {torque.size}，"
                f"应为 {self.previous_torque.size}"
            )
            reason = self._stop(t, reason)
            result = SafetyResult(
                torque=np.zeros_like(self.previous_torque),
                stop=True,
                reason=reason,
                events=[reason],
            )
            return result

        if self.config.stop_on_nonfinite_torque and not np.all(np.isfinite(torque)):
            reason = "力矩指令中出现非有限数值"
            reason = self._stop(t, reason)
            result = SafetyResult(
                torque=np.zeros_like(self.previous_torque),
                stop=True,
                reason=reason,
                events=[reason],
            )
            return result

        limited = torque.copy()

        rate_limit = _limit_array(
            self.config.torque_rate_limit,
            limited.size,
            "torque_rate_limit",
            allow_none=True,
        )
        if rate_limit is not None and self.previous_torque is not None:
            delta = limited - self.previous_torque
            clipped_delta = np.clip(delta, -rate_limit, rate_limit)
            if not np.allclose(delta, clipped_delta):
                events.append(
                    f"力矩变化率已限制：原始变化量={delta}，上限={rate_limit}"
                )
            limited = self.previous_torque + clipped_delta

        torque_limit = _limit_array(self.config.torque_limit, limited.size, "torque_limit")
        clipped = np.clip(limited, -torque_limit, torque_limit)
        if not np.allclose(limited, clipped):
            events.append(f"力矩已限幅：原始值={limited}，上限={torque_limit}")
        limited = clipped

        self.previous_torque = limited.copy()
        formatted_events = []
        for event in events:
            formatted_events.append(self._record(t, event))
        return SafetyResult(torque=limited, events=formatted_events)

    def _stop(self, t, reason):
        return self._record(t, reason, level="STOP")

    def stop(self, t, reason):
        return self._stop(t, reason)

    def warning(self, t, message):
        return self._record(t, message, level="WARN")

    def _record(self, t, message, level="WARN"):
        formatted = self._format_event(level, message)
        self.event_history.append((float(t), formatted))
        return formatted

    def _format_event(self, level, message):
        message = str(message)
        if message.startswith("[SAFETY]"):
            return message
        code = self._classify(message)
        hint = SAFETY_HINTS.get(code, SAFETY_HINTS["SAFETY_EVENT"])
        return f"[SAFETY][{level}][{code}] {message} | 建议={hint}"

    def _classify(self, message):
        text = message.lower()
        if "位置超出边界" in text:
            return "POSITION_BOUND"
        if "速度超出边界" in text:
            return "VELOCITY_BOUND"
        if "反馈中出现非有限数值" in text:
            return "FEEDBACK_NONFINITE"
        if "力矩指令中出现非有限数值" in text:
            return "TORQUE_NONFINITE"
        if "力矩向量长度不一致" in text:
            return "TORQUE_DIMENSION"
        if "力矩变化率已限制" in text:
            return "TORQUE_RATE_LIMITED"
        if "力矩已限幅" in text:
            return "TORQUE_CLIPPED"
        if "控制周期超时" in text:
            return "LOOP_OVERRUN"
        if "通信刷新失败" in text:
            return "COMMUNICATION_REFRESH"
        if "清理失败" in text:
            return "CLEANUP_FAILED"
        return "SAFETY_EVENT"
