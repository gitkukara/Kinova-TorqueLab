"""实验运行器。

负责 1 kHz 控制循环、调用控制器、记录数据和保存结果。
"""

import os
import time
from collections import defaultdict

import numpy as np

from controllers.base import as_control_result
from safety import SafetyConfig, SafetyMonitor


def _param_array(value):
    if value is None:
        return np.asarray([])
    return np.asarray(value)


def _add_param_arrays(arrays, prefix, params):
    for key, value in params.items():
        arrays[f"{prefix}_{key}"] = _param_array(value)


class ExperimentRunner:
    def __init__(
        self,
        robot,
        controller,
        reference,
        duration=20.0,
        dt=0.001,
        torque_limit=50.0,
        safety_config=None,
        log_every=1,
        data_dir=None,
    ):
        self.robot = robot
        self.controller = controller
        self.reference = reference
        self.duration = float(duration)
        self.dt = float(dt)
        self.torque_limit = torque_limit
        self.safety_config = safety_config or SafetyConfig(torque_limit=torque_limit)
        self.safety = SafetyMonitor(self.safety_config)
        self.log_every = max(int(log_every), 1)
        self.data_dir = data_dir
        if hasattr(self.controller, "set_reference"):
            self.controller.set_reference(reference)

    def run(self):
        q0, dq0 = self.robot.prepare()
        self.controller.reset(q0, dq0)
        self.safety.reset(q0)

        log = defaultdict(list)
        ok = False
        try:
            print("Warmup: zero torque for 200 ms")
            self.robot.warmup_zero_torque(duration=0.2, dt=self.dt)

            start_time = time.perf_counter()
            next_loop_time = start_time
            step = 0
            last_print_t = -1.0

            while True:
                now = time.perf_counter()
                t = now - start_time
                if t >= self.duration:
                    break

                feedback, (q, dq) = self.robot.refresh()
                xr, dxr, ddxr = self.reference.sample(t)
                safety_msg = self.safety.check_state(t, q, dq)
                if safety_msg:
                    print(f"SAFETY STOP: {safety_msg}")
                    if step % self.log_every == 0:
                        log["t"].append(t)
                        log["q"].append(q.copy())
                        log["dq"].append(dq.copy())
                        log["xr"].append(xr.copy())
                        log["dxr"].append(dxr.copy())
                        log["ddxr"].append(ddxr.copy())
                        log["u_raw"].append(np.zeros_like(q))
                        log["u"].append(np.zeros_like(q))
                        log["safety"].append(safety_msg)
                    break

                result = as_control_result(
                    self.controller.compute(t, q, dq, xr, dxr, ddxr)
                )
                raw_torque = np.asarray(result.torque, dtype=float).reshape(-1)
                safety_result = self.safety.limit_torque(t, result.torque)
                torque = safety_result.torque
                safety_msg = safety_result.reason or "; ".join(safety_result.events)
                if safety_result.stop:
                    print(f"SAFETY STOP: {safety_result.reason}")
                    if step % self.log_every == 0:
                        log["t"].append(t)
                        log["q"].append(q.copy())
                        log["dq"].append(dq.copy())
                        log["xr"].append(xr.copy())
                        log["dxr"].append(dxr.copy())
                        log["ddxr"].append(ddxr.copy())
                        log["u_raw"].append(raw_torque.copy())
                        log["u"].append(torque.copy())
                        log["safety"].append(safety_msg)
                        for key, value in result.log.items():
                            log[key].append(np.asarray(value).copy())
                    break

                self.robot.send_torque(feedback, torque)

                if step % self.log_every == 0:
                    log["t"].append(t)
                    log["q"].append(q.copy())
                    log["dq"].append(dq.copy())
                    log["xr"].append(xr.copy())
                    log["dxr"].append(dxr.copy())
                    log["ddxr"].append(ddxr.copy())
                    log["u_raw"].append(raw_torque.copy())
                    log["u"].append(torque.copy())
                    log["safety"].append(safety_msg)
                    for key, value in result.log.items():
                        log[key].append(np.asarray(value).copy())

                if t - last_print_t >= 0.5:
                    last_print_t = t
                    error = xr - q
                    error_text = ", ".join(f"{value:+.4f}" for value in error)
                    torque_text = ", ".join(f"{value:+6.2f}" for value in torque)
                    print(
                        f"t={t:5.1f}s | "
                        f"e=[{error_text}] rad | "
                        f"u=[{torque_text}] Nm"
                    )

                step += 1
                next_loop_time += self.dt
                wait = next_loop_time - time.perf_counter()
                if wait > 0:
                    time.sleep(wait)

            ok = True
            return ok, dict(log)
        finally:
            self.robot.cleanup(return_home=True)

    def save(self, log, controller_name):
        if len(log.get("t", [])) == 0:
            print("No data to save.")
            return None
        data_dir = self.data_dir or os.path.join(os.getcwd(), "data")
        os.makedirs(data_dir, exist_ok=True)
        filename = os.path.join(
            data_dir,
            f"{controller_name}_kinova_{time.strftime('%Y%m%d_%H%M%S')}.npz",
        )
        arrays = {key: np.asarray(value) for key, value in log.items()}
        arrays["p_duration"] = np.asarray(self.duration)
        arrays["p_dt"] = np.asarray(self.dt)
        arrays["p_torque_limit"] = _param_array(self.safety_config.torque_limit)
        arrays["p_torque_rate_limit"] = _param_array(
            self.safety_config.torque_rate_limit
        )
        arrays["p_position_bound"] = _param_array(self.safety_config.position_bound)
        arrays["p_velocity_bound"] = _param_array(self.safety_config.velocity_bound)
        arrays["p_stop_on_position_bound"] = np.asarray(
            self.safety_config.stop_on_position_bound
        )
        arrays["p_stop_on_velocity_bound"] = np.asarray(
            self.safety_config.stop_on_velocity_bound
        )
        arrays["p_stop_on_nonfinite_feedback"] = np.asarray(
            self.safety_config.stop_on_nonfinite_feedback
        )
        arrays["p_stop_on_nonfinite_torque"] = np.asarray(
            self.safety_config.stop_on_nonfinite_torque
        )
        arrays["p_controller"] = np.asarray(controller_name)

        _add_param_arrays(
            arrays,
            "p_robot",
            {
                "torque_joints": self.robot.torque_joints,
                "start_angles_deg": self.robot.start_angles_deg,
            },
        )
        _add_param_arrays(
            arrays,
            "p_reference",
            {
                "center_rad": self.reference.center,
                "amplitude_deg": np.degrees(self.reference.amplitude),
                "period_s": self.reference.period_s,
            },
        )
        _add_param_arrays(
            arrays,
            "p_controller",
            self.controller.get_params(),
        )
        np.savez(filename, **arrays)

        safety_path = os.path.splitext(filename)[0] + "_safety_events.txt"
        with open(safety_path, "w", encoding="utf-8") as f:
            for t, message in self.safety.event_history:
                f.write(f"{t:.6f}: {message}\n")
        print(f"Saved -> {filename}")
        return filename
