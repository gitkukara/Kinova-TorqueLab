# Kinova TorqueLab

Kinova TorqueLab 是一个面向 Kinova Gen3 的轻量级关节力矩控制实验框架。它统一处理 Kortex 通信、控制模式切换、实验循环、安全保护、数据记录和控制器调用，方便用户接入并验证自己的控制算法。

框架默认面向两个受控关节。公开版本提供可用的 PID 跟踪基线，以及用于短时通路检查的 `hold` 控制器。放入 `torque_platform/controllers/` 的本地控制器会被自动发现，不需要修改主循环。

> `hold` 没有重力补偿，机械臂可能在负载下缓慢下沉。它只用于短时间检查通信、模式切换和数据通路，不应被当作可靠的位置保持控制器。

## 项目结构

```text
Kinova-TorqueLab/
├─ README.md
├─ requirements.txt
├─ torque_platform/
│  ├─ config.py              # [主要修改] 实验、安全与控制器参数
│  ├─ controllers/           # [主要修改] 接入和调整控制算法
│  │  ├─ pid.py              # PID 跟踪基线
│  │  ├─ hold.py             # 短时通路检查
│  │  └─ new_controller_template.py  # 新控制器模板
│  ├─ main.py / __main__.py  # 框架入口与组件装配
│  ├─ runner.py              # 控制循环、记录与保存
│  ├─ robot_interface.py     # Kortex 通信与模式切换
│  ├─ safety.py              # 公共安全层
│  ├─ validation.py          # 运行前配置检查
│  ├─ reference.py           # 参考轨迹
│  └─ plot_results.py        # 实验结果绘图
```

标记为 `[主要修改]` 的位置是日常使用入口。其余文件负责框架公共流程，通常不需要修改。

## 环境依赖

Windows 或 Linux 系统，需要提前安装：

- Python 3.11 或相近版本
- `numpy`
- `matplotlib`（绘图时使用）
- Kinova Kortex Python API

Kinova Kortex Python API 需要从 Kinova 官方 Artifactory 下载与机械臂硬件和固件对应的 `.whl` 文件。例如 2.7.0 版本：

```text
https://artifactory.kinovaapps.com/ui/repos/tree/General/generic-public/kortex/API/2.7.0
```

下载后进入 wheel 所在目录并安装：

Windows：

```powershell
python -m pip install .\kortex_api-2.7.0.post5-py3-none-any.whl
```

Linux：

```bash
python3 -m pip install ./kortex_api-2.7.0.post5-py3-none-any.whl
```

框架的其余依赖也可以通过以下命令安装：

```powershell
python -m pip install -r requirements.txt
```

## 关键配置

日常实验主要修改 `torque_platform/config.py`：

- 机器人连接信息。
- `CONTROLLER`：当前控制器。
- `TORQUE_JOINTS`：进入力矩模式的关节索引。
- `START_ANGLES_DEG`：实验起始姿态。
- `DURATION` 和 `DT`：实验时长与目标控制周期。
- `REFERENCE_*`：参考轨迹。
- `SAFETY_*`、位置边界和速度边界：最终安全限制。
- `<CONTROLLER>_<PARAMETER>`：对应控制器的构造参数。

`DT` 是目标周期，并不表示普通 Windows/Python 环境能够提供硬实时保证。评估实验时应以实际记录的时间为准。

## 运行

查看已发现的控制器，不连接机器人：

```powershell
python -m torque_platform --list-controllers
```

检查配置和控制器能否正确创建，不连接机器人：

```powershell
python -m torque_platform --check-config
```

按 `config.py` 运行实验：

```powershell
python -m torque_platform
```

命令行参数可以临时覆盖常用配置：

```powershell
python -m torque_platform --controller pid --duration 5
```

上机前请确认机器人状态、起始姿态、受控关节、轨迹范围和安全限制。首次运行新控制器时应缩短实验时间并使用保守参数。

## 控制器

控制器继承 `BaseController`，实现以下两个方法：

```python
class MyController(BaseController):
    name = "my_controller"

    def reset(self, q0, dq0=None):
        ...

    def compute(self, t, q, dq, xr, dxr, ddxr):
        torque = ...
        return ControlResult(torque=torque, log={"error": xr - q})
```

新增控制器时：

1. 复制 `torque_platform/controllers/new_controller_template.py`。
2. 修改文件名、类名和唯一的 `name`。
3. 在 `reset()` 中初始化状态，在 `compute()` 中实现控制律。
4. 在 `config.py` 中添加控制器参数，并运行 `--list-controllers` 和 `--check-config`。

控制器只负责根据反馈和参考轨迹计算力矩，不应直接调用 Kortex API。单位、返回值和日志约定已写在基类、控制器模板及运行器的代码注释中。

## 数据与绘图

实验完成后，运行器会保存 `.npz` 数据和对应的安全事件日志。数据结构及字段含义记录在 `runner.py` 的写入代码旁。

显示最新实验结果：

```powershell
python -m torque_platform.plot_results
```

只运行控制而不需要绘图时，可在 `config.py` 中设置：

```python
PLOT_AFTER_RUN = False
```
