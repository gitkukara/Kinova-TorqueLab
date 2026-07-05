"""实验主配置文件。

日常实验调参优先改这里。除变量名特别说明外，角度参数均使用度。
"""


# 机器人连接
IP = "192.168.1.x"
USERNAME = "username"
PASSWORD = "password"


# 实验设置
CONTROLLER = "pid"  # pid / hold / brl_ppc

# Hold 控制器参数。用于短时间保持进入实验时的初始关节角。
HOLD_KP = [8.0, 8.0]
HOLD_KD = [0.8, 0.8]
HOLD_TORQUE_LIMIT = 10.0
DURATION = 20.0
DT = 0.001
TORQUE_LIMIT = 50.0
LOG_EVERY = 1

# 实验结束后自动显示本次实验的速览图。
# 默认只看一眼，不保存图片；需要正式出图时再单独运行 plot_results.py。
PLOT_AFTER_RUN = True
PLOT_SHOW = True
PLOT_SAVE = False
PLOT_OUTDIR = "figures"
PLOT_FORMATS = ["png", "pdf"]


# 框架安全设置
# 最终下发前的力矩限幅。None 表示沿用 TORQUE_LIMIT。
# 可填一个数，也可填与 TORQUE_JOINTS 等长的列表。
SAFETY_TORQUE_LIMIT = None

# 每个控制周期允许的最大力矩变化量，单位 N*m/周期。
# None 表示关闭该限幅。可填一个数或每个受控关节一个数。
TORQUE_RATE_LIMIT = None

# 位置安全边界，单位 rad。若 abs(q - q_start) 超过该值则停机。
POSITION_BOUND = 0.45

# 速度安全边界，单位 rad/s。若 abs(dq) 超过该值则停机。
VELOCITY_BOUND = 1.0

# 通用安全停机开关。
STOP_ON_POSITION_BOUND = True
STOP_ON_VELOCITY_BOUND = True
STOP_ON_NONFINITE_FEEDBACK = True
STOP_ON_NONFINITE_TORQUE = True


# 机器人位姿设置
# Kortex 关节索引：默认使用 J4/J6 -> 3,5。
TORQUE_JOINTS = [3, 5]
START_ANGLES_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -90.0]


# 参考轨迹设置
# 公式：xr(t) = center + amplitude * sin(2*pi*t/period)
# center 使用弧度(rad)，amplitude 使用度(deg)，period 使用秒(s)。
REFERENCE_CENTER_RAD = [0.0, 0.0]
REFERENCE_AMPLITUDE_DEG = [15.0, 15.0]
REFERENCE_PERIOD_S = [5.0, 5.0]


# PID 控制器参数
PID_KP = [20.0, 17.0]
PID_KI = [0.4, 0.5]
PID_KD = [1.5, 1.5]
PID_INTEGRAL_LIMIT = 1.0

# PID 参考轨迹相位提前，单位 s。用于补偿原 PID 脚本中的跟踪滞后。
PID_PHASE_LEAD_S = [1.47, 1.12]


# 新增控制器参数示例
# 规则：<控制器名大写>_<参数名大写> 会自动传给控制器构造函数。
# 例如 PID_DEADBAND 会传给 PIDController(deadband=...)。
# 因此通常只需要：
# 1. 在本文件添加对应参数。
# 2. 在控制器 __init__() 中添加同名小写参数。
# 3. 在控制器 compute() 中使用该参数。
