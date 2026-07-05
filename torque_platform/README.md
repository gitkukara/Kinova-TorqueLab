# Kinova Gen3 力矩控制算法验证平台

常用实验参数统一改 [config.py](E:/Kinova_py/api_python/torque_platform/config.py)，包括控制器、初始位姿、参考轨迹、力矩限幅和 PID 参数。一般不用在命令行里写一长串参数。

## 运行

先修改 `config.py`：

```python
CONTROLLER = "pid"  # pid / hold / brl_ppc
DURATION = 20.0
TORQUE_LIMIT = 50.0

START_ANGLES_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -90.0]

REFERENCE_CENTER_RAD = [0.0, 0.0]
REFERENCE_AMPLITUDE_DEG = [15.0, 15.0]
REFERENCE_PERIOD_S = [5.0, 5.0]
```

## Current framework notes

The default experiment still controls J4/J6, so the default lists in `config.py`
remain two-dimensional:

```python
TORQUE_JOINTS = [3, 5]
REFERENCE_CENTER_RAD = [0.0, 0.0]
REFERENCE_AMPLITUDE_DEG = [15.0, 15.0]
REFERENCE_PERIOD_S = [5.0, 5.0]
```

For other controlled-joint sets, keep the reference lists the same length as
`TORQUE_JOINTS`, for example:

```bash
python torque_platform/main.py --torque-joints 2,3,5 \
  --reference-center-rad 0,0,0 \
  --reference-amplitude-deg 10,15,8 \
  --reference-period-s 5,5,7
```

New controllers are auto-discovered from single Python files in `controllers/`.
Create a class that inherits `BaseController`, give it a unique `name`, then set
`CONTROLLER` in `config.py` or pass `--controller` on the command line.

再运行：

```bash
python api_python/torque_platform/main.py
```

VS Code 可选择：

- `Torque Platform: PID`
- `Torque Platform: BRL-PPC 5s`

## 临时覆盖

偶尔临时改一次，也可以用命令行覆盖配置：

```bash
python api_python/torque_platform/main.py --controller brl_ppc --duration 5
```

## 新增控制算法

1. 复制 `controllers/new_controller_template.py`。
2. 实现 `compute()`。
3. 在 `main.py` 的 `create_controller()` 中注册。
4. 在 `config.py` 中把 `CONTROLLER` 改成新名字。

最小接口：

```python
def compute(self, t, q, dq, xr, dxr, ddxr):
    torque = ...
    return ControlResult(torque=torque, log={"error": xr - q})
```

单位：`q/xr` 为 rad，`dq/dxr` 为 rad/s，`ddxr` 为 rad/s^2，`torque` 为 N*m。
