"""Hardware-independent validation for experiment configuration."""

import math


def _values(value, name, errors, allow_none=False):
    if value is None:
        if allow_none:
            return None
        errors.append(f"{name} must not be None")
        return None

    if isinstance(value, (str, bytes)):
        errors.append(f"{name} must be numeric")
        return None

    try:
        items = list(value)
    except TypeError:
        items = [value]

    if not items:
        errors.append(f"{name} must not be empty")
        return None

    try:
        numbers = [float(item) for item in items]
    except (TypeError, ValueError):
        errors.append(f"{name} must contain only numbers")
        return None

    if not all(math.isfinite(number) for number in numbers):
        errors.append(f"{name} must contain only finite numbers")
        return None
    return numbers


def _positive_scalar(value, name, errors):
    numbers = _values(value, name, errors)
    if numbers is None:
        return
    if len(numbers) != 1:
        errors.append(f"{name} must be a single number")
    elif numbers[0] <= 0.0:
        errors.append(f"{name} must be greater than zero")


def _joint_limit(value, name, joint_count, errors, allow_none=False):
    numbers = _values(value, name, errors, allow_none=allow_none)
    if numbers is None:
        return
    if len(numbers) not in (1, joint_count):
        errors.append(f"{name} must be scalar or contain {joint_count} values")
    if any(number <= 0.0 for number in numbers):
        errors.append(f"{name} values must be greater than zero")


def validate_experiment_args(args):
    """Raise ``ValueError`` with all actionable configuration problems."""

    errors = []

    ip = str(args.ip).strip()
    if not ip or ip.lower() in {"192.168.1.x", "robot-ip", "your-robot-ip"}:
        errors.append("ip still contains a placeholder; set the robot IP")
    if not str(args.username).strip() or str(args.username).strip() == "username":
        errors.append("username still contains a placeholder")
    if not str(args.password).strip() or str(args.password).strip() == "password":
        errors.append("password still contains a placeholder")

    _positive_scalar(args.duration, "duration", errors)
    _positive_scalar(args.dt, "dt", errors)
    _positive_scalar(args.torque_limit, "torque_limit", errors)
    _positive_scalar(args.cyclic_timeout_ms, "cyclic_timeout_ms", errors)

    if int(args.log_every) < 1:
        errors.append("log_every must be at least 1")

    try:
        torque_joints = [int(joint) for joint in args.torque_joints]
    except (TypeError, ValueError):
        torque_joints = []
        errors.append("torque_joints must contain integer actuator indexes")

    if not torque_joints:
        errors.append("torque_joints must not be empty")
    elif len(set(torque_joints)) != len(torque_joints):
        errors.append("torque_joints must not contain duplicates")

    start_angles = _values(args.start_angles_deg, "start_angles_deg", errors)
    if start_angles is not None:
        if len(start_angles) != 7:
            errors.append("start_angles_deg must contain 7 values for Kinova Gen3")
        invalid = [joint for joint in torque_joints if joint < 0 or joint >= len(start_angles)]
        if invalid:
            errors.append(
                "torque_joints contains out-of-range indexes: "
                + ", ".join(str(joint) for joint in invalid)
            )

    joint_count = len(torque_joints)
    for value, name in (
        (args.reference_center_rad, "reference_center_rad"),
        (args.reference_amplitude_deg, "reference_amplitude_deg"),
        (args.reference_period_s, "reference_period_s"),
    ):
        numbers = _values(value, name, errors)
        if numbers is not None and len(numbers) != joint_count:
            errors.append(f"{name} must contain {joint_count} values")

    periods = _values(args.reference_period_s, "reference_period_s", [])
    if periods is not None and any(period <= 0.0 for period in periods):
        errors.append("reference_period_s values must be greater than zero")

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

    if args.stop_on_loop_overrun:
        _positive_scalar(args.loop_overrun_limit_s, "loop_overrun_limit_s", errors)
        if int(args.loop_overrun_max_consecutive) < 1:
            errors.append("loop_overrun_max_consecutive must be at least 1")

    if errors:
        details = "\n".join(f"  - {message}" for message in errors)
        raise ValueError(f"Invalid experiment configuration:\n{details}")
