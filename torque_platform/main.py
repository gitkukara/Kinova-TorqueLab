"""平台入口脚本。

默认读取 config.py 中的实验参数，也可以用命令行参数临时覆盖部分配置。
"""

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
API_ROOT = os.path.dirname(HERE)
MY_CODE = os.path.join(API_ROOT, "my_code")
for path in (HERE, API_ROOT, MY_CODE):
    if path not in sys.path:
        sys.path.insert(0, path)

from controllers.registry import (
    available_controller_names,
    create_controller as create_registered_controller,
)
from reference import SineReference
from robot_interface import KinovaTorqueInterface
from runner import ExperimentRunner
from safety import SafetyConfig
from plot_results import plot_results
import utilities
import config


def parse_float_list(text, expected_len=None, name="value"):
    values = [float(item.strip()) for item in text.split(",") if item.strip()]
    if expected_len is not None and len(values) != expected_len:
        raise argparse.ArgumentTypeError(
            f"{name} must contain {expected_len} comma-separated numbers"
        )
    return values


def parse_int_list(text, expected_len=None, name="value"):
    values = [int(item.strip()) for item in text.split(",") if item.strip()]
    if expected_len is not None and len(values) != expected_len:
        raise argparse.ArgumentTypeError(
            f"{name} must contain {expected_len} comma-separated integers"
        )
    return values


def parse_optional_float_list(text, name="value"):
    if text is None:
        return None
    if str(text).strip().lower() in ("none", "off", "disabled"):
        return None
    values = parse_float_list(text, name=name)
    return values[0] if len(values) == 1 else values


def ensure_len(values, expected_len, name):
    if len(values) != expected_len:
        raise ValueError(
            f"{name} must contain {expected_len} comma-separated values "
            f"to match torque_joints"
        )
    return values


def apply_legacy_reference_overrides(args):
    center = list(args.reference_center_rad)
    amplitude = list(args.reference_amplitude_deg)
    period = list(args.reference_period_s)

    legacy_values = (
        args.amp_j4_deg,
        args.amp_j6_deg,
        args.period_j4,
        args.period_j6,
        args.center_j4,
        args.center_j6,
    )
    if any(value is not None for value in legacy_values):
        for values in (center, amplitude, period):
            if len(values) < 2:
                raise ValueError("Legacy J4/J6 reference overrides require two values")
        if args.amp_j4_deg is not None:
            amplitude[0] = args.amp_j4_deg
        if args.amp_j6_deg is not None:
            amplitude[1] = args.amp_j6_deg
        if args.period_j4 is not None:
            period[0] = args.period_j4
        if args.period_j6 is not None:
            period[1] = args.period_j6
        if args.center_j4 is not None:
            center[0] = args.center_j4
        if args.center_j6 is not None:
            center[1] = args.center_j6

    joint_count = len(args.torque_joints)
    args.reference_center_rad = ensure_len(center, joint_count, "reference-center-rad")
    args.reference_amplitude_deg = ensure_len(
        amplitude, joint_count, "reference-amplitude-deg"
    )
    args.reference_period_s = ensure_len(period, joint_count, "reference-period-s")


def build_parser():
    parser = argparse.ArgumentParser(
        description="Kinova Gen3 reusable torque-control experiment platform."
    )
    parser.add_argument("--ip", type=str, default=config.IP)
    parser.add_argument("-u", "--username", type=str, default=config.USERNAME)
    parser.add_argument("-p", "--password", type=str, default=config.PASSWORD)
    parser.add_argument(
        "--controller",
        choices=available_controller_names(),
        default=config.CONTROLLER,
        help="Control algorithm to run.",
    )
    parser.add_argument("--duration", type=float, default=config.DURATION)
    parser.add_argument("--dt", type=float, default=config.DT)
    parser.add_argument("--torque-limit", type=float, default=config.TORQUE_LIMIT)
    parser.add_argument(
        "--cyclic-timeout-ms",
        type=int,
        default=config.CYCLIC_TIMEOUT_MS,
        help=(
            "Kortex cyclic Refresh timeout in ms. Default is 3 ms; modify "
            "cautiously because larger values can hide communication latency."
        ),
    )
    parser.add_argument(
        "--safety-torque-limit",
        type=lambda s: parse_optional_float_list(s, "safety-torque-limit"),
        default=config.SAFETY_TORQUE_LIMIT,
        help="Final torque clamp before sending commands. None uses --torque-limit.",
    )
    parser.add_argument(
        "--torque-rate-limit",
        type=lambda s: parse_optional_float_list(s, "torque-rate-limit"),
        default=config.TORQUE_RATE_LIMIT,
        help="Maximum torque change per loop in N*m. Use 'none' to disable.",
    )
    parser.add_argument(
        "--position-bound",
        type=lambda s: parse_optional_float_list(s, "position-bound"),
        default=config.POSITION_BOUND,
        help="Stop if abs(q - q_start) exceeds this radian bound.",
    )
    parser.add_argument(
        "--velocity-bound",
        type=lambda s: parse_optional_float_list(s, "velocity-bound"),
        default=config.VELOCITY_BOUND,
        help="Stop if abs(dq) exceeds this rad/s bound.",
    )
    parser.add_argument(
        "--loop-overrun-limit-s",
        type=float,
        default=config.LOOP_OVERRUN_LIMIT_S,
        help="Stop if a control loop takes longer than this many seconds.",
    )
    parser.add_argument(
        "--loop-overrun-max-consecutive",
        type=int,
        default=config.LOOP_OVERRUN_MAX_CONSECUTIVE,
        help="Number of consecutive loop overruns required before stopping.",
    )
    parser.add_argument(
        "--stop-on-position-bound",
        action=argparse.BooleanOptionalAction,
        default=config.STOP_ON_POSITION_BOUND,
        help="Enable or disable position-bound safety stop.",
    )
    parser.add_argument(
        "--stop-on-velocity-bound",
        action=argparse.BooleanOptionalAction,
        default=config.STOP_ON_VELOCITY_BOUND,
        help="Enable or disable velocity-bound safety stop.",
    )
    parser.add_argument(
        "--stop-on-nonfinite-feedback",
        action=argparse.BooleanOptionalAction,
        default=config.STOP_ON_NONFINITE_FEEDBACK,
        help="Enable or disable NaN/Inf feedback safety stop.",
    )
    parser.add_argument(
        "--stop-on-nonfinite-torque",
        action=argparse.BooleanOptionalAction,
        default=config.STOP_ON_NONFINITE_TORQUE,
        help="Enable or disable NaN/Inf torque safety stop.",
    )
    parser.add_argument(
        "--stop-on-loop-overrun",
        action=argparse.BooleanOptionalAction,
        default=config.STOP_ON_LOOP_OVERRUN,
        help="Enable or disable control-loop overrun safety stop.",
    )
    parser.add_argument(
        "--torque-joints",
        type=lambda s: parse_int_list(s, name="torque-joints"),
        default=config.TORQUE_JOINTS,
        help="Actuator indexes for torque control. Defaults to 3,5 for J4/J6.",
    )
    parser.add_argument(
        "--start-angles-deg",
        type=lambda s: parse_float_list(s, 7, "start-angles-deg"),
        default=config.START_ANGLES_DEG,
        help="Seven start joint angles in degrees, e.g. 0,0,0,0,0,0,-90.",
    )
    parser.add_argument(
        "--reference-center-rad",
        type=lambda s: parse_float_list(s, name="reference-center-rad"),
        default=config.REFERENCE_CENTER_RAD,
        help="Reference centers in rad, one value per torque joint.",
    )
    parser.add_argument(
        "--reference-amplitude-deg",
        type=lambda s: parse_float_list(s, name="reference-amplitude-deg"),
        default=config.REFERENCE_AMPLITUDE_DEG,
        help="Reference sine amplitudes in deg, one value per torque joint.",
    )
    parser.add_argument(
        "--reference-period-s",
        type=lambda s: parse_float_list(s, name="reference-period-s"),
        default=config.REFERENCE_PERIOD_S,
        help="Reference sine periods in seconds, one value per torque joint.",
    )
    parser.add_argument("--amp-j4-deg", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--amp-j6-deg", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--period-j4", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--period-j6", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--center-j4", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--center-j6", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--data-dir", default=os.path.join(HERE, "data"))
    parser.add_argument("--log-every", type=int, default=config.LOG_EVERY)
    parser.add_argument(
        "--plot-after-run",
        action=argparse.BooleanOptionalAction,
        default=config.PLOT_AFTER_RUN,
        help="Generate quick-look figures after saving experiment data.",
    )
    parser.add_argument(
        "--plot-show",
        action=argparse.BooleanOptionalAction,
        default=config.PLOT_SHOW,
        help="Show figures after the experiment. Saved figures are controlled separately.",
    )
    parser.add_argument(
        "--plot-save",
        action=argparse.BooleanOptionalAction,
        default=config.PLOT_SAVE,
        help="Save quick-look figures after the experiment.",
    )
    parser.add_argument(
        "--plot-outdir",
        default=config.PLOT_OUTDIR,
        help="Directory for quick-look figures.",
    )
    parser.add_argument(
        "--plot-fmt",
        nargs="+",
        default=config.PLOT_FORMATS,
        help="Figure formats used when --plot-save is enabled.",
    )
    return parser



def main():
    args = build_parser().parse_args()
    apply_legacy_reference_overrides(args)
    safety_torque_limit = (
        args.torque_limit
        if args.safety_torque_limit is None
        else args.safety_torque_limit
    )
    safety_config = SafetyConfig(
        torque_limit=safety_torque_limit,
        torque_rate_limit=args.torque_rate_limit,
        position_bound=args.position_bound,
        velocity_bound=args.velocity_bound,
        loop_overrun_limit_s=args.loop_overrun_limit_s,
        loop_overrun_max_consecutive=args.loop_overrun_max_consecutive,
        stop_on_position_bound=args.stop_on_position_bound,
        stop_on_velocity_bound=args.stop_on_velocity_bound,
        stop_on_nonfinite_feedback=args.stop_on_nonfinite_feedback,
        stop_on_nonfinite_torque=args.stop_on_nonfinite_torque,
        stop_on_loop_overrun=args.stop_on_loop_overrun,
    )
    controller = create_registered_controller(
        args.controller,
        config,
        extra={"dt": args.dt, "torque_limit": args.torque_limit},
    )
    reference = SineReference(
        center=np.asarray(args.reference_center_rad, dtype=float),
        amplitude_deg=args.reference_amplitude_deg,
        period_s=args.reference_period_s,
    )

    with utilities.DeviceConnection.createTcpConnection(args) as router:
        with utilities.DeviceConnection.createUdpConnection(args) as router_real_time:
            robot = KinovaTorqueInterface(
                router,
                router_real_time,
                torque_joints=args.torque_joints,
                start_angles_deg=args.start_angles_deg,
                cyclic_timeout_ms=args.cyclic_timeout_ms,
            )
            runner = ExperimentRunner(
                robot=robot,
                controller=controller,
                reference=reference,
                duration=args.duration,
                dt=args.dt,
                torque_limit=args.torque_limit,
                safety_config=safety_config,
                log_every=args.log_every,
                data_dir=args.data_dir,
            )
            ok, log = runner.run()

    if ok:
        data_path = runner.save(log, controller.name)
        if data_path and args.plot_after_run:
            plot_results(
                data_path,
                save=args.plot_save,
                outdir=args.plot_outdir,
                formats=args.plot_fmt,
                show=args.plot_show,
            )


if __name__ == "__main__":
    main()
