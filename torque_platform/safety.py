"""力矩实验的通用安全检查和输出限幅。"""

from dataclasses import dataclass, field

import numpy as np


@dataclass
class SafetyConfig:
    torque_limit: object = 50.0
    torque_rate_limit: object = None
    position_bound: object = 0.45
    velocity_bound: object = 1.0
    stop_on_position_bound: bool = True
    stop_on_velocity_bound: bool = True
    stop_on_nonfinite_feedback: bool = True
    stop_on_nonfinite_torque: bool = True


@dataclass
class SafetyResult:
    torque: np.ndarray
    stop: bool = False
    reason: str = ""
    events: list = field(default_factory=list)


def _limit_array(value, size, name, allow_none=False):
    if value is None and allow_none:
        return None
    array = np.asarray(value, dtype=float)
    if array.ndim == 0:
        return np.full(size, float(array), dtype=float)
    array = array.reshape(-1)
    if array.size != size:
        raise ValueError(f"{name} must be scalar or contain {size} values")
    return array


class SafetyMonitor:
    def __init__(self, config):
        self.config = config
        self.q_center = None
        self.previous_torque = None
        self.event_history = []

    def reset(self, q0):
        self.q_center = np.asarray(q0, dtype=float).copy()
        self.previous_torque = np.zeros_like(self.q_center, dtype=float)
        self.event_history = []

    def check_state(self, t, q, dq):
        q = np.asarray(q, dtype=float)
        dq = np.asarray(dq, dtype=float)

        if self.config.stop_on_nonfinite_feedback:
            if not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq)):
                return self._stop(t, "non-finite feedback")

        if self.q_center is not None and self.config.stop_on_position_bound:
            limit = _limit_array(self.config.position_bound, q.size, "position_bound")
            offset = np.abs(q - self.q_center)
            if np.any(offset > limit):
                return self._stop(
                    t,
                    f"position bound exceeded: offset={offset}, limit={limit}",
                )

        if self.config.stop_on_velocity_bound:
            limit = _limit_array(self.config.velocity_bound, dq.size, "velocity_bound")
            speed = np.abs(dq)
            if np.any(speed > limit):
                return self._stop(
                    t,
                    f"velocity bound exceeded: speed={speed}, limit={limit}",
                )

        return ""

    def limit_torque(self, t, torque):
        torque = np.asarray(torque, dtype=float).reshape(-1)
        events = []

        if self.previous_torque is not None and torque.size != self.previous_torque.size:
            reason = (
                f"torque dimension mismatch: got {torque.size}, "
                f"expected {self.previous_torque.size}"
            )
            result = SafetyResult(
                torque=np.zeros_like(self.previous_torque),
                stop=True,
                reason=reason,
                events=[reason],
            )
            self._record(t, reason)
            return result

        if self.config.stop_on_nonfinite_torque and not np.all(np.isfinite(torque)):
            reason = "non-finite torque command"
            result = SafetyResult(
                torque=np.zeros_like(self.previous_torque),
                stop=True,
                reason=reason,
                events=[reason],
            )
            self._record(t, reason)
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
                    f"torque rate limited: raw_delta={delta}, limit={rate_limit}"
                )
            limited = self.previous_torque + clipped_delta

        torque_limit = _limit_array(self.config.torque_limit, limited.size, "torque_limit")
        clipped = np.clip(limited, -torque_limit, torque_limit)
        if not np.allclose(limited, clipped):
            events.append(f"torque clipped: raw={limited}, limit={torque_limit}")
        limited = clipped

        self.previous_torque = limited.copy()
        for event in events:
            self._record(t, event)
        return SafetyResult(torque=limited, events=events)

    def _stop(self, t, reason):
        self._record(t, reason)
        return reason

    def _record(self, t, message):
        self.event_history.append((float(t), str(message)))
