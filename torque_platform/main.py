"""平台入口脚本。

默认读取 config.py 中的实验参数，也可以用命令行参数临时覆盖部分配置。
"""

import argparse
import inspect
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
    controller_classes,
    create_controller as create_registered_controller,
)
from reference import SineReference
from robot_interface import KinovaTorqueInterface
from runner import ExperimentRunner
from safety import SafetyConfig
from validation import validate_experiment_args
import utilities
import config


def parse_float_list(text, expected_len=None, name="value"):
    values = [float(item.strip()) for item in text.split(",") if item.strip()]
    if expected_len is not None and len(values) != expected_len:
        raise argparse.ArgumentTypeError(
            f"{name} 必须包含 {expected_len} 个用逗号分隔的数值"
        )
    return values


def parse_int_list(text, expected_len=None, name="value"):
    values = [int(item.strip()) for item in text.split(",") if item.strip()]
    if expected_len is not None and len(values) != expected_len:
        raise argparse.ArgumentTypeError(
            f"{name} 必须包含 {expected_len} 个用逗号分隔的整数"
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
            f"{name} 必须包含 {expected_len} 个用逗号分隔的值，"
            f"以便与 torque_joints 对应"
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
                raise ValueError("旧版 J4/J6 参考轨迹覆盖参数要求配置两个关节值")
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


def print_available_controllers():
    print("可用控制器：")
    for name, controller_cls in sorted(controller_classes().items()):
        own_doc = controller_cls.__dict__.get("__doc__")
        doc = inspect.cleandoc(own_doc) if own_doc else "暂无说明。"
        summary = doc.splitlines()[0]
        print(f"  {name:<16} {summary}")


def print_config_summary(args, safety_torque_limit):
    print(
        f"[RUN][CONFIG] controller={args.controller}, ip={args.ip}, "
        f"torque_joints={args.torque_joints}, duration={args.duration}s, "
        f"dt={args.dt}s"
    )
    print(
        f"[RUN][SAFETY] torque_limit={safety_torque_limit}, "
        f"torque_rate_limit={args.torque_rate_limit}, "
        f"position_bound={args.position_bound}, "
        f"velocity_bound={args.velocity_bound}"
    )
    if args.controller == "hold":
        print(
            "[RUN][提示] hold 是低增益 PD 调试控制器，不含重力补偿，"
            "机械臂在负载下可能缓慢下沉。"
        )


def build_parser():
    parser = argparse.ArgumentParser(
        description="Kinova Gen3 可复用力矩控制实验平台。"
    )
    parser.add_argument("--ip", type=str, default=config.IP, help="机械臂 IP 地址。")
    parser.add_argument(
        "-u", "--username", type=str, default=config.USERNAME, help="登录用户名。"
    )
    parser.add_argument(
        "-p", "--password", type=str, default=config.PASSWORD, help="登录密码。"
    )
    parser.add_argument(
        "--list-controllers",
        action="store_true",
        help="列出自动发现的控制器后退出，不连接机械臂。",
    )
    parser.add_argument(
        "--check-config",
        action="store_true",
        help="检查配置并尝试创建控制器，然后退出，不连接机械臂。",
    )
    parser.add_argument(
        "--controller",
        choices=available_controller_names(),
        default=config.CONTROLLER,
        help="本次实验使用的控制算法。",
    )
    parser.add_argument(
        "--duration", type=float, default=config.DURATION, help="实验时长，单位 s。"
    )
    parser.add_argument(
        "--dt", type=float, default=config.DT, help="目标控制周期，单位 s。"
    )
    parser.add_argument(
        "--torque-limit",
        type=float,
        default=config.TORQUE_LIMIT,
        help="控制器使用的力矩限幅，单位 N*m。",
    )
    parser.add_argument(
        "--cyclic-timeout-ms",
        type=int,
        default=config.CYCLIC_TIMEOUT_MS,
        help=(
            "Kortex cyclic Refresh 的超时时间，单位 ms。默认值为 3 ms；"
            "增大该值可能掩盖通信延迟，请谨慎修改。"
        ),
    )
    parser.add_argument(
        "--safety-torque-limit",
        type=lambda s: parse_optional_float_list(s, "safety-torque-limit"),
        default=config.SAFETY_TORQUE_LIMIT,
        help="指令下发前的最终力矩限幅；设为 none 时使用 --torque-limit。",
    )
    parser.add_argument(
        "--torque-rate-limit",
        type=lambda s: parse_optional_float_list(s, "torque-rate-limit"),
        default=config.TORQUE_RATE_LIMIT,
        help="每个控制周期允许的最大力矩变化，单位 N*m；使用 none 可关闭。",
    )
    parser.add_argument(
        "--position-bound",
        type=lambda s: parse_optional_float_list(s, "position-bound"),
        default=config.POSITION_BOUND,
        help="当 abs(q - q_start) 超过此边界时停机，单位 rad。",
    )
    parser.add_argument(
        "--velocity-bound",
        type=lambda s: parse_optional_float_list(s, "velocity-bound"),
        default=config.VELOCITY_BOUND,
        help="当 abs(dq) 超过此边界时停机，单位 rad/s。",
    )
    parser.add_argument(
        "--loop-overrun-limit-s",
        type=float,
        default=config.LOOP_OVERRUN_LIMIT_S,
        help="单个控制周期超过此时长时记为超时，单位 s。",
    )
    parser.add_argument(
        "--loop-overrun-max-consecutive",
        type=int,
        default=config.LOOP_OVERRUN_MAX_CONSECUTIVE,
        help="触发停机所需的连续控制周期超时次数。",
    )
    parser.add_argument(
        "--stop-on-position-bound",
        action=argparse.BooleanOptionalAction,
        default=config.STOP_ON_POSITION_BOUND,
        help="启用或关闭位置越界停机。",
    )
    parser.add_argument(
        "--stop-on-velocity-bound",
        action=argparse.BooleanOptionalAction,
        default=config.STOP_ON_VELOCITY_BOUND,
        help="启用或关闭速度越界停机。",
    )
    parser.add_argument(
        "--stop-on-nonfinite-feedback",
        action=argparse.BooleanOptionalAction,
        default=config.STOP_ON_NONFINITE_FEEDBACK,
        help="启用或关闭反馈出现 NaN/Inf 时的停机保护。",
    )
    parser.add_argument(
        "--stop-on-nonfinite-torque",
        action=argparse.BooleanOptionalAction,
        default=config.STOP_ON_NONFINITE_TORQUE,
        help="启用或关闭力矩指令出现 NaN/Inf 时的停机保护。",
    )
    parser.add_argument(
        "--stop-on-loop-overrun",
        action=argparse.BooleanOptionalAction,
        default=config.STOP_ON_LOOP_OVERRUN,
        help="启用或关闭控制周期连续超时停机。",
    )
    parser.add_argument(
        "--torque-joints",
        type=lambda s: parse_int_list(s, name="torque-joints"),
        default=config.TORQUE_JOINTS,
        help="进入力矩控制的执行器索引；默认 3,5 对应 J4/J6。",
    )
    parser.add_argument(
        "--start-angles-deg",
        type=lambda s: parse_float_list(s, 7, "start-angles-deg"),
        default=config.START_ANGLES_DEG,
        help="7 个起始关节角，单位 deg，例如 0,0,0,0,0,0,-90。",
    )
    parser.add_argument(
        "--reference-center-rad",
        type=lambda s: parse_float_list(s, name="reference-center-rad"),
        default=config.REFERENCE_CENTER_RAD,
        help="参考轨迹中心，单位 rad，每个受控关节填写一个值。",
    )
    parser.add_argument(
        "--reference-amplitude-deg",
        type=lambda s: parse_float_list(s, name="reference-amplitude-deg"),
        default=config.REFERENCE_AMPLITUDE_DEG,
        help="正弦参考轨迹幅值，单位 deg，每个受控关节填写一个值。",
    )
    parser.add_argument(
        "--reference-period-s",
        type=lambda s: parse_float_list(s, name="reference-period-s"),
        default=config.REFERENCE_PERIOD_S,
        help="正弦参考轨迹周期，单位 s，每个受控关节填写一个值。",
    )
    parser.add_argument("--amp-j4-deg", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--amp-j6-deg", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--period-j4", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--period-j6", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--center-j4", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--center-j6", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument(
        "--data-dir",
        default=os.path.join(HERE, "data"),
        help="实验数据输出目录。",
    )
    parser.add_argument(
        "--log-every",
        type=int,
        default=config.LOG_EVERY,
        help="每隔多少个控制周期记录一次数据。",
    )
    parser.add_argument(
        "--plot-after-run",
        action=argparse.BooleanOptionalAction,
        default=config.PLOT_AFTER_RUN,
        help="保存实验数据后生成快速预览图。",
    )
    parser.add_argument(
        "--plot-show",
        action=argparse.BooleanOptionalAction,
        default=config.PLOT_SHOW,
        help="实验后显示图窗；是否保存图片由 --plot-save 单独控制。",
    )
    parser.add_argument(
        "--plot-save",
        action=argparse.BooleanOptionalAction,
        default=config.PLOT_SAVE,
        help="实验后保存快速预览图。",
    )
    parser.add_argument(
        "--plot-outdir",
        default=config.PLOT_OUTDIR,
        help="快速预览图的输出目录。",
    )
    parser.add_argument(
        "--plot-fmt",
        nargs="+",
        default=config.PLOT_FORMATS,
        help="启用 --plot-save 时使用的图片格式。",
    )
    return parser



def main():
    parser = build_parser()
    args = parser.parse_args()
    if args.list_controllers:
        print_available_controllers()
        return

    try:
        apply_legacy_reference_overrides(args)
        validate_experiment_args(args)
    except (TypeError, ValueError) as exc:
        parser.error(str(exc))

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

    print_config_summary(args, safety_torque_limit)
    if args.check_config:
        print(
            f"[CHECK][通过] 配置有效，控制器 '{controller.name}' 已成功创建。"
        )
        return

    print(f"[RUN][连接] 正在连接 {args.ip} 的 TCP/UDP 会话...")
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        with utilities.DeviceConnection.createUdpConnection(args) as router_real_time:
            print("[RUN][连接] TCP/UDP 会话已就绪。")
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
            try:
                from plot_results import plot_results
            except ImportError as exc:
                if exc.name == "matplotlib":
                    print(
                        "[RUN][警告] 未安装 matplotlib，已跳过绘图。可运行 "
                        "'python -m pip install matplotlib' 安装，或将 "
                        "PLOT_AFTER_RUN 设为 False。"
                    )
                    return
                raise
            plot_results(
                data_path,
                save=args.plot_save,
                outdir=args.plot_outdir,
                formats=args.plot_fmt,
                show=args.plot_show,
            )


if __name__ == "__main__":
    main()
