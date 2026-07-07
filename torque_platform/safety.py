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
    "POSITION_BOUND": "Joint moved beyond the configured position window; check reference amplitude, controller gains, and start pose.",
    "VELOCITY_BOUND": "Joint speed exceeded the configured limit; check gains, trajectory speed, and possible communication stalls.",
    "FEEDBACK_NONFINITE": "Robot feedback contains NaN or Inf; check cyclic feedback, API state, and network stability.",
    "TORQUE_NONFINITE": "Controller produced NaN or Inf torque; check controller math, divisions, matrix operations, and internal states.",
    "TORQUE_DIMENSION": "Controller torque vector length does not match TORQUE_JOINTS.",
    "TORQUE_RATE_LIMITED": "Torque command changed faster than TORQUE_RATE_LIMIT and was smoothed before sending.",
    "TORQUE_CLIPPED": "Torque command exceeded the final safety torque limit and was clipped before sending.",
    "LOOP_OVERRUN": "Control loop took too long; likely communication latency, slow controller computation, or OS scheduling delay.",
    "COMMUNICATION_REFRESH": "Cyclic Refresh failed; check network, Kortex connection, robot fault state, and real-time UDP path.",
    "CLEANUP_FAILED": "Cleanup raised an exception; robot may not have completed zero-torque, mode switch, or return-home.",
    "SAFETY_EVENT": "Unclassified safety event; inspect the detailed message and nearby logs.",
}


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
            f"control loop overrun {self.loop_overrun_count}/{max_count}: "
            f"elapsed={elapsed_s:.6f}s, limit={limit:.6f}s"
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
                f"torque dimension mismatch: got {torque.size}, "
                f"expected {self.previous_torque.size}"
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
            reason = "non-finite torque command"
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
                    f"torque rate limited: raw_delta={delta}, limit={rate_limit}"
                )
            limited = self.previous_torque + clipped_delta

        torque_limit = _limit_array(self.config.torque_limit, limited.size, "torque_limit")
        clipped = np.clip(limited, -torque_limit, torque_limit)
        if not np.allclose(limited, clipped):
            events.append(f"torque clipped: raw={limited}, limit={torque_limit}")
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
        return f"[SAFETY][{level}][{code}] {message} | hint={hint}"

    def _classify(self, message):
        text = message.lower()
        if "position bound exceeded" in text:
            return "POSITION_BOUND"
        if "velocity bound exceeded" in text:
            return "VELOCITY_BOUND"
        if "non-finite feedback" in text:
            return "FEEDBACK_NONFINITE"
        if "non-finite torque" in text:
            return "TORQUE_NONFINITE"
        if "torque dimension mismatch" in text:
            return "TORQUE_DIMENSION"
        if "torque rate limited" in text:
            return "TORQUE_RATE_LIMITED"
        if "torque clipped" in text:
            return "TORQUE_CLIPPED"
        if "control loop overrun" in text:
            return "LOOP_OVERRUN"
        if "communication refresh failed" in text:
            return "COMMUNICATION_REFRESH"
        if "cleanup failed" in text:
            return "CLEANUP_FAILED"
        return "SAFETY_EVENT"
