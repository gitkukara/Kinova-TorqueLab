# Kinova TorqueLab

Kinova TorqueLab 是一个面向 Kinova Gen3 的轻量级力矩控制实验框架，用于快速接入、切换和验证不同关节力矩控制算法。

本框架基于 [Kinova Kortex API](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L)，默认面向两关节实验，集成了机器人连接、参考轨迹生成、控制器注册、安全限幅、实时控制循环和数据记录。新的控制算法通常只需要新增一个控制器文件，不需要改动主循环。


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
      └─ new_controller_template.py
```


## 环境依赖

Windows 或 Linux 系统，需要提前安装：

- Python 3.11 或相近版本
- `numpy`
- Kinova Kortex Python API

其中 Kinova Kortex Python API 需要从 Kinova 官方 Artifactory 下载机械臂硬件对应版本的 `.whl` 文件，如 2.7.0 版本链接如下：

```text
https://artifactory.kinovaapps.com/ui/repos/tree/General/generic-public/kortex/API/2.7.0
```

下载后在终端中进入项目所在环境，使用 `pip` 安装该 wheel 文件：

Windows：

```powershell
python -m pip install .\kortex_api-2.7.0.post5-py3-none-any.whl
```

Linux：

```bash
python3 -m pip install ./kortex_api-2.7.0.post5-py3-none-any.whl
```


## 快速运行

实验前建议先看这几个文件：

- `torque_platform/config.py`：最主要的调参入口。机器人连接、实验时长、受控关节、初始位置、参考轨迹、安全限幅和控制器参数都优先在这里修改。
- `.vscode/launch.json`：VS Code 调试入口。适合直接启动默认实验，或启动短时间 `hold` 测试。
- `torque_platform/controllers/`：控制器文件夹。新增算法通常只需要在这里新增一个控制器文件。
- `torque_platform/main.py`：程序入口。一般不需要改，除非要新增命令行参数或改变整体运行流程。

当前已有控制器：

- `hold`：低增益 PD 保持进入实验时的初始位置，适合先检查力矩模式和安全流程。
- `pid`：带相位提前的 PID 轨迹跟踪控制器，适合基础跟踪实验。

日常使用时，通常只需要在 `config.py` 中选择控制器并调整参数，然后运行：

```powershell
py torque_platform/main.py
```

也可以用命令行临时覆盖部分参数：

```powershell
py torque_platform/main.py --controller hold --duration 5
```

VS Code 中可以直接使用 `.vscode/launch.json` 里的运行配置：

- `实验：按 config.py 运行`：按 `config.py` 当前设置启动完整实验。
- `实验：hold 保持 5 秒`：短时间运行 `hold` 控制器，用于检查连接、模式切换和基础安全流程。
- `绘图：显示最新数据`：自动读取最新 `.npz` 数据并显示速览图。
- `绘图：显示指定数据`：手动输入 `.npz` 数据文件路径并显示图像。
- `绘图：保存指定数据 PNG+PDF`：手动输入 `.npz` 数据文件路径，并把图像保存到 `figures/`。


## 新增控制器

1. 复制模板 `torque_platform/controllers/new_controller_template.py`。
2. 重命名，例如 `my_controller.py`。
3. 修改模板内的类名和 `name`。
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

默认情况下，实验结束并保存 `.npz` 后会自动弹出本次实验的速览图，但不会保存图片。速览图包括跟踪表现、跟踪误差、控制力矩三类，每类图会按受控关节数量自动生成对应数量的子图。

如需关闭自动绘图，可以在 `config.py` 中设置：

```python
PLOT_AFTER_RUN = False
```

也可以用离线绘图脚本查看已有实验结果：

```powershell
py torque_platform/plot_results.py
```

也可以指定数据文件并保存图片。默认会同时保存 PNG 和 PDF：

```powershell
py torque_platform/plot_results.py torque_platform/data/xxx.npz --save
```


## 注意事项

- 上机前确认 Kinova 官方界面中机械臂处于 ready 状态。
- 首次实验建议使用 `hold` 或小幅值 `pid`。
- 保守设置 `SAFETY_TORQUE_LIMIT`、`TORQUE_RATE_LIMIT`、`POSITION_BOUND` 和 `VELOCITY_BOUND`。
- 控制器内部可以自带限幅，但框架安全层会在最终下发前再做一次保护。
