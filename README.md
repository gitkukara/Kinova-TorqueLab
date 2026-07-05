# Kinova TorqueLab

Kinova TorqueLab 是一个面向 Kinova Gen3 的轻量级力矩控制实验框架，用于快速接入、切换和验证不同关节力矩控制算法。

框架默认面向 J4/J6 两关节实验，集成了机器人连接、参考轨迹生成、控制器注册、安全限幅、实时控制循环和数据记录。新的控制算法通常只需要新增一个控制器文件，不需要改动主循环。


## 项目结构

```text
Kinova-TorqueLab/
├─ README.md
├─ utilities.py
└─ torque_platform/
   ├─ main.py                 # 程序入口，读取配置并启动实验
   ├─ config.py               # 日常调参入口
   ├─ runner.py               # 1 kHz 实验循环、日志记录和数据保存
   ├─ robot_interface.py      # Kinova/Kortex 连接、模式切换和力矩下发
   ├─ reference.py            # 参考轨迹生成
   ├─ safety.py               # 安全检查和限幅
   └─ controllers/
      ├─ base.py              # 控制器统一接口
      ├─ registry.py          # 自动发现控制器
      ├─ hold.py              # 初始位置保持控制器
      ├─ pid.py               # PID 跟踪控制器
      ├─ brl_ppc.py           # BRL-PPC 控制器
      └─ new_controller_template.py
```


## 环境依赖

需要提前安装：

- Python 3.11 或相近版本
- `numpy`
- Kinova Kortex Python API

Kinova Kortex Python API 需要从 Kinova 官方 Artifactory 下载对应版本的 `.whl` 文件：

```text
https://artifactory.kinovaapps.com/ui/repos/tree/General/generic-public/kortex/API/2.7.0
```

下载后在终端中进入项目所在环境，使用 `pip` 安装该 wheel 文件：

```powershell
python -m pip install <whl relative fullpath name>.whl
```

例如：

```powershell
python -m pip install .\kortex_api-2.7.0.post5-py3-none-any.whl
```

如果你的 Windows 环境中 `python` 指向的不是实验用解释器，也可以使用：

```powershell
py -m pip install .\kortex_api-2.7.0.post5-py3-none-any.whl
```


## 快速运行

日常实验优先修改：

```text
torque_platform/config.py
```

然后运行：

```powershell
py torque_platform/main.py
```

也可以用命令行临时覆盖部分参数：

```powershell
py torque_platform/main.py --controller hold --duration 5
```

VS Code 中可以直接使用 `.vscode/launch.json` 里的运行配置。


## 常用配置

```python
CONTROLLER = "pid"          # pid / hold / brl_ppc
DURATION = 20.0
DT = 0.001
TORQUE_JOINTS = [3, 5]      # 默认 J4/J6
START_ANGLES_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -90.0]

REFERENCE_CENTER_RAD = [0.0, 0.0]
REFERENCE_AMPLITUDE_DEG = [15.0, 15.0]
REFERENCE_PERIOD_S = [5.0, 5.0]
```

安全相关参数也在 `config.py` 中：

```python
SAFETY_TORQUE_LIMIT = None
TORQUE_RATE_LIMIT = None
POSITION_BOUND = 0.45
VELOCITY_BOUND = 1.0
```


## 新增控制器

最简单的方式：

1. 复制 `torque_platform/controllers/new_controller_template.py`。
2. 改成新的文件名，例如 `my_controller.py`。
3. 修改类名和 `name`。
4. 实现 `reset()` 和 `compute()`。
5. 在 `config.py` 中设置：

```python
CONTROLLER = "my_controller"
```

`controllers/registry.py` 会自动发现继承 `BaseController` 且带唯一 `name` 的控制器，不需要手动注册。


## 数据输出

实验结束后，`runner.py` 会保存 `.npz` 数据文件，主要字段包括：

- `t`: 时间
- `q`, `dq`: 实际关节角和角速度
- `xr`, `dxr`, `ddxr`: 参考轨迹
- `u_raw`: 控制器原始输出
- `u`: 安全层处理后的实际下发力矩
- `safety`: 安全事件信息

同时会生成同名 `_safety_events.txt`，用于记录限幅或停机事件。


## 注意事项

- 上机前确认 Kinova 官方界面中机械臂处于 ready 状态。
- 首次实验建议使用 `hold` 或小幅值 `pid`。
- 保守设置 `SAFETY_TORQUE_LIMIT`、`TORQUE_RATE_LIMIT`、`POSITION_BOUND` 和 `VELOCITY_BOUND`。
- 控制器内部可以自带限幅，但框架安全层会在最终下发前再做一次保护。
