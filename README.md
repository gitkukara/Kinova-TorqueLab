# Kinova TorqueLab

Kinova TorqueLab 是一个面向 Kinova Gen3 的轻量级关节力矩控制实验框架。它把机器人连接、模式切换、实验循环、安全检查、控制器调用、数据保存和绘图放在统一流程中，让使用者可以把主要精力放在控制算法本身。

框架默认面向两个受控关节，但参考轨迹、安全层和控制器接口都按向量组织。公开版本提供 PID 基线和 `hold` 调试控制器；放入 `torque_platform/controllers/` 的本地控制器会被自动发现。

> `hold` 只是低增益 PD 调试控制器，没有重力补偿。它适合短时间检查连接、模式切换和数据通路，不能保证机械臂在重力作用下保持姿态。

## 运行流程

```text
config.py / CLI
       |
       v
配置预检 -> 控制器创建 -> 机器人准备 -> 周期控制循环
                                      |
                         反馈 -> 控制器 -> 安全层 -> 力矩命令
                                      |
                                      v
                              数据与安全日志
```

- `main.py` 负责参数解析、预检和组件装配。
- `robot_interface.py` 封装 Kortex API 和控制模式切换。
- `runner.py` 负责周期循环、数据采集和实验收尾。
- `safety.py` 是所有控制器共用的最终安全层。
- `controllers/` 中的每个文件提供一个可替换控制器。

`DT` 是目标控制周期，不代表普通 Windows/Python 环境能够保证硬实时 1 kHz。评估控制器时应以保存数据中的 `t` 为准，并同时关注通信延迟和周期抖动。

## 1. 安装环境

环境不需要严格锁定。建议使用 Python 3.10 或更高版本；只要 Kinova Kortex Python API 能在当前 Python 环境中正常安装和导入，框架通常就可以运行。虚拟环境是可选的：

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

框架运行只依赖 `numpy`；`matplotlib` 用于实验后的自动绘图和离线绘图。`requirements.txt` 不锁定具体版本，由 `pip` 选择与当前 Python 兼容的版本。如果只运行控制且关闭自动绘图，可以不安装 `matplotlib`：

```python
PLOT_AFTER_RUN = False
```

最后安装与机器人固件及当前 Python 环境匹配的 Kinova Kortex Python API wheel。仓库中的文件名只是示例，不要求必须使用 2.7.0：

```powershell
python -m pip install .\kortex_api-2.7.0.post5-py3-none-any.whl
```

Kortex wheel 可从 Kinova 官方 Artifactory 下载：

```text
https://artifactory.kinovaapps.com/ui/repos/tree/General/generic-public/kortex/API/2.7.0
```

## 2. 配置机器人和实验

日常使用主要修改 `torque_platform/config.py`：

```python
IP = "192.168.1.10"
USERNAME = "admin"
PASSWORD = "admin"

CONTROLLER = "pid"
TORQUE_JOINTS = [3, 5]       # Kortex 索引 3、5，即 J4、J6
DURATION = 20.0
DT = 0.001
```

同一文件还包含：

- 初始关节角和正弦参考轨迹。
- 最终力矩、位置、速度和周期超时保护。
- PID 以及其他控制器的构造参数。
- 实验结束后的绘图选项。

命令行参数可以临时覆盖常用配置。运行 `python -m torque_platform --help` 可查看完整列表。

## 3. 连接机械臂前先预检

查看当前发现的控制器：

```powershell
python -m torque_platform --list-controllers
```

检查参数长度、范围和控制器能否创建，不连接机器人：

```powershell
python -m torque_platform --check-config
```

成功时会输出：

```text
[CHECK][OK] Configuration is valid and controller 'pid' was created successfully.
```

预检会一次列出可操作的配置错误，例如占位 IP、重复关节、参考轨迹维度不一致、非正周期或非法限幅。

## 4. 运行实验

使用 `config.py` 中的设置：

```powershell
python -m torque_platform
```

临时运行 5 秒 PID：

```powershell
python -m torque_platform --controller pid --duration 5
```

短时间检查力矩模式通路：

```powershell
python -m torque_platform --controller hold --duration 2
```

`hold` 可能在重力作用下缓慢下沉。确认通信和模式切换后，应使用经过验证的 PID 或自己的控制器进行实验。

上机前至少确认：

- 机器人处于 ready 状态，急停和操作空间可用。
- `TORQUE_JOINTS` 与控制器输出维度一致。
- 初始角、轨迹幅值、速度和各关节力矩上限合理。
- 首次试验使用较短 `DURATION`，并观察最终下发力矩 `u`。

## 新增控制器

1. 复制 `torque_platform/controllers/new_controller_template.py`。
2. 修改文件名、类名和唯一的 `name`。
3. 在 `reset()` 中初始化每次实验的内部状态。
4. 在 `compute()` 中返回 `ControlResult`。
5. 在 `config.py` 中添加以控制器名开头的参数。
6. 运行控制器列表、配置预检和无硬件测试。

最小接口如下：

```python
class MyController(BaseController):
    name = "my_controller"

    def reset(self, q0, dq0=None):
        ...

    def compute(self, t, q, dq, xr, dxr, ddxr):
        torque = ...
        return ControlResult(torque=torque, log={"error": xr - q})
```

接口约定：

- `t` 使用秒。
- `q`、`xr` 使用 rad，`dq`、`dxr` 使用 rad/s。
- 返回力矩使用 N*m，长度必须等于 `TORQUE_JOINTS`。
- `log` 中的值应为标量或固定形状的 NumPy 数组。
- 控制器可以带内部限幅，但最终命令仍会经过公共安全层。

控制器构造参数会自动从 `config.py` 读取。假设控制器名为 `my_controller`，构造参数名为 `gain`，对应配置写法是：

```python
MY_CONTROLLER_GAIN = [1.0, 1.0]
```

私有算法文件可以加入 `.gitignore` 或 `.git/info/exclude`，自动发现机制仍然有效。

## 数据和绘图

成功实验会在 `torque_platform/data/` 保存 `.npz` 和同名安全日志。主要字段包括：

- `t`：实际采样时间。
- `q`、`dq`：关节角和角速度。
- `xr`、`dxr`、`ddxr`：参考轨迹。
- `u_raw`：控制器原始输出。
- `u`：公共安全层处理后的下发力矩。
- `safety`：与样本对应的安全事件。
- `p_*`：机器人、参考轨迹、安全层和控制器参数快照。

显示最新实验：

```powershell
python -m torque_platform.plot_results
```

保存指定数据的 PNG 和 PDF：

```powershell
python -m torque_platform.plot_results torque_platform/data/example.npz `
  --save --no-show --fmt png pdf
```

## 项目结构

```text
Kinova-TorqueLab/
├─ README.md
├─ requirements.txt
├─ utilities.py
├─ tests/
└─ torque_platform/
   ├─ __main__.py             # python -m torque_platform 入口
   ├─ config.py               # 用户配置入口
   ├─ main.py                 # CLI、预检与组件装配
   ├─ validation.py           # 无硬件配置校验
   ├─ runner.py               # 周期循环、记录和保存
   ├─ robot_interface.py      # Kortex 接口与模式切换
   ├─ reference.py            # 参考轨迹
   ├─ safety.py               # 公共安全层
   ├─ plot_results.py         # 离线绘图
   └─ controllers/
      ├─ base.py
      ├─ registry.py
      ├─ hold.py
      ├─ pid.py
      └─ new_controller_template.py
```

## 无硬件测试

测试不连接机器人：

```powershell
python -m unittest discover -s tests -v
```

测试覆盖配置预检、PID/hold 接口以及公共安全层。新的控制器建议增加确定性的数值测试，并与原论文、MATLAB 或仿真实现对照。

## 常见问题

`No module named kortex_api`

安装与固件匹配的 Kortex wheel，并确认当前终端已激活正确虚拟环境。

`hold` 下机械臂缓慢下沉

这是预期的能力边界。`hold` 没有机器人动力学或重力补偿，只用于短时调试。

找不到新控制器

确认控制器文件位于 `torque_platform/controllers/`，类继承 `BaseController`，`name` 非空且不与其他控制器重复，然后运行 `--list-controllers`。

控制周期达不到 1 ms

先根据数据中的 `t` 统计真实周期，再区分 Kortex 通信、控制器计算、终端输出和操作系统调度开销。普通 Python 进程不提供硬实时保证。
