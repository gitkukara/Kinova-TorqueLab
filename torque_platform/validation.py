"""实验配置检查。

这里只检查参数是否完整、类型和尺寸是否合理，不连接机械臂，也不会下发指令。
检查通过只能说明配置形式正确，不能代替上机前的安全确认。
"""

import math


def _values(value, name, errors, allow_none=False):
    """将单个数值或序列统一转换为有限浮点数列表。"""

    if value is None:
        if allow_none:
            return None
        errors.append(f"{name} 不能为 None")
        return None

    if isinstance(value, (str, bytes)):
        errors.append(f"{name} 必须是数值或数值序列")
        return None

    try:
        items = list(value)
    except TypeError:
        items = [value]

    if not items:
        errors.append(f"{name} 不能为空")
        return None

    try:
        numbers = [float(item) for item in items]
    except (TypeError, ValueError):
        errors.append(f"{name} 只能包含数值")
        return None

    if not all(math.isfinite(number) for number in numbers):
        errors.append(f"{name} 只能包含有限数值，不能出现 NaN 或 Inf")
        return None
    return numbers


def _positive_scalar(value, name, errors):
    """检查参数是否为大于 0 的单个数值。"""

    numbers = _values(value, name, errors)
    if numbers is None:
        return
    if len(numbers) != 1:
        errors.append(f"{name} 必须是单个数值")
    elif numbers[0] <= 0.0:
        errors.append(f"{name} 必须大于 0")


def _joint_limit(value, name, joint_count, errors, allow_none=False):
    """检查安全限制是单值，或与受控关节一一对应的正数序列。"""

    numbers = _values(value, name, errors, allow_none=allow_none)
    if numbers is None:
        return
    if len(numbers) not in (1, joint_count):
        errors.append(f"{name} 必须是单个数值，或包含 {joint_count} 个关节对应值")
    if any(number <= 0.0 for number in numbers):
        errors.append(f"{name} 中的数值必须全部大于 0")


def validate_experiment_args(args):
    """汇总配置中的问题，并通过 ``ValueError`` 一次性报告。"""

    errors = []

    # 连接信息必须已经替换模板中的占位值。
    ip = str(args.ip).strip()
    if not ip or ip.lower() in {"192.168.1.x", "robot-ip", "your-robot-ip"}:
        errors.append("ip 仍是占位值，请填写机械臂 IP 地址")
    if not str(args.username).strip() or str(args.username).strip() == "username":
        errors.append("username 仍是占位值，请填写登录用户名")
    if not str(args.password).strip() or str(args.password).strip() == "password":
        errors.append("password 仍是占位值，请填写登录密码")

    # 实验时长、控制周期、力矩上限和通信超时必须是正数。
    _positive_scalar(args.duration, "duration", errors)
    _positive_scalar(args.dt, "dt", errors)
    _positive_scalar(args.torque_limit, "torque_limit", errors)
    _positive_scalar(args.cyclic_timeout_ms, "cyclic_timeout_ms", errors)

    if int(args.log_every) < 1:
        errors.append("log_every 必须大于或等于 1")

    # 受控关节索引必须为不重复的整数，并落在 Gen3 的 7 个关节范围内。
    try:
        torque_joints = [int(joint) for joint in args.torque_joints]
    except (TypeError, ValueError):
        torque_joints = []
        errors.append("torque_joints 必须包含整数形式的执行器索引")

    if not torque_joints:
        errors.append("torque_joints 不能为空")
    elif len(set(torque_joints)) != len(torque_joints):
        errors.append("torque_joints 不能包含重复索引")

    start_angles = _values(args.start_angles_deg, "start_angles_deg", errors)
    if start_angles is not None:
        if len(start_angles) != 7:
            errors.append("Kinova Gen3 的 start_angles_deg 必须包含 7 个关节角")
        invalid = [joint for joint in torque_joints if joint < 0 or joint >= len(start_angles)]
        if invalid:
            errors.append(
                "torque_joints 包含超出范围的索引："
                + ", ".join(str(joint) for joint in invalid)
            )

    # 正弦参考轨迹的三组参数必须与受控关节数量一致，周期还必须大于 0。
    joint_count = len(torque_joints)
    for value, name in (
        (args.reference_center_rad, "reference_center_rad"),
        (args.reference_amplitude_deg, "reference_amplitude_deg"),
        (args.reference_period_s, "reference_period_s"),
    ):
        numbers = _values(value, name, errors)
        if numbers is not None and len(numbers) != joint_count:
            errors.append(f"{name} 必须包含 {joint_count} 个关节对应值")

    periods = _values(args.reference_period_s, "reference_period_s", [])
    if periods is not None and any(period <= 0.0 for period in periods):
        errors.append("reference_period_s 中的周期必须全部大于 0")

    # 各项安全限制可统一设置，也可为每个受控关节分别设置。
    _joint_limit(
        args.safety_torque_limit,
        "safety_torque_limit",
        joint_count,
        errors,
        allow_none=True,
    )
    _joint_limit(
        args.torque_rate_limit,
        "torque_rate_limit",
        joint_count,
        errors,
        allow_none=True,
    )
    _joint_limit(
        args.position_bound,
        "position_bound",
        joint_count,
        errors,
        allow_none=not args.stop_on_position_bound,
    )
    _joint_limit(
        args.velocity_bound,
        "velocity_bound",
        joint_count,
        errors,
        allow_none=not args.stop_on_velocity_bound,
    )

    # 只有启用控制周期超时保护时，才要求对应阈值有效。
    if args.stop_on_loop_overrun:
        _positive_scalar(args.loop_overrun_limit_s, "loop_overrun_limit_s", errors)
        if int(args.loop_overrun_max_consecutive) < 1:
            errors.append("loop_overrun_max_consecutive 必须大于或等于 1")

    if errors:
        details = "\n".join(f"  - {message}" for message in errors)
        raise ValueError(f"实验配置无效：\n{details}")
