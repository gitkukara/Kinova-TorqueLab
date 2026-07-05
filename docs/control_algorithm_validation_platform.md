# Kinova Gen3 实时力矩控制算法验证平台设计

## 1. 项目目标

当前 `api_python/my_code/BRL-PPC_3.0.py` 已经实现了一个完整的 Kinova Gen3 机械臂 J4/J6 关节力矩控制实验脚本，包含机械臂连接、初始位姿配置、伺服模式切换、实时反馈读取、控制律计算、力矩指令下发、数据记录、绘图和安全恢复等功能。

后续可以将该单文件脚本重构为“实时力矩控制算法验证平台”：把机械臂实验流程中的共性部分抽离出来，把 PID、BRL-PPC、MFASFC 等控制方法作为可替换的控制模块接入。这样后续验证新控制算法时，只需要实现统一的控制器接口，而不需要重复编写机械臂通信、控制循环、数据记录和安全恢复逻辑。

## 2. 当前脚本中的共性流程

从 `BRL-PPC_3.0.py` 中可以提取出的公共实验流程包括：

- 解析机器人 IP、用户名、密码等运行参数
- 连接 Kinova Gen3 机械臂并创建 Kortex API 客户端
- 移动机械臂到实验初始位姿
- 读取初始关节角度和速度反馈
- 切换到 `LOW_LEVEL_SERVOING` 模式
- 将指定关节切换为 `TORQUE` 控制模式
- 构建 1 kHz 实时控制循环
- 在循环中读取 J4/J6 的角度和速度反馈
- 生成参考轨迹 `xr`、`dxr`、`ddxr`
- 调用控制器计算力矩指令
- 对输出力矩进行限幅和安全检查
- 下发力矩指令到机械臂
- 记录轨迹、误差、力矩和控制器内部变量
- 实验结束后清零力矩、恢复位置模式、恢复伺服模式并归位
- 保存 `.npz` 实验数据并进行离线可视化

这些内容与具体控制算法关系较弱，适合封装为统一实验平台。

## 3. 需要保留为可替换模块的部分

不同控制算法之间真正需要替换的是控制律计算部分。例如当前 BRL-PPC 脚本中的 PPC 误差变换、Backstepping 虚拟控制、Actor-Critic 神经网络更新、BLS 节点扩展、DDE Bellman 误差计算和最终力矩输出，都应封装到 `BRLPPCController` 中。

平台主循环只需要调用统一接口：

```python
torque, extra_log = controller.compute(t, q, dq, xr, dxr, ddxr)
```

其中：

- `torque` 是输出给 J4/J6 的力矩指令
- `extra_log` 是控制器希望额外记录的数据，例如 PPC 边界、神经网络节点数、Actor 输出等

## 4. 建议目录结构

```text
api_python/
└─ torque_platform/
   ├─ main.py
   ├─ robot_runner.py
   ├─ robot_interface.py
   ├─ reference.py
   ├─ logger.py
   ├─ safety.py
   └─ controllers/
      ├─ base.py
      ├─ pid.py
      ├─ brl_ppc.py
      └─ mfasfc.py
```

各文件职责建议如下：

- `main.py`：解析命令行参数，选择控制器并启动实验
- `robot_runner.py`：实现统一实验流程和 1 kHz 控制循环
- `robot_interface.py`：封装 Kinova Kortex API 连接、模式切换、反馈读取和指令下发
- `reference.py`：生成正弦轨迹或其他参考轨迹
- `logger.py`：统一记录实验数据并保存 `.npz`
- `safety.py`：封装力矩限幅、异常处理和实验结束恢复逻辑
- `controllers/base.py`：定义所有控制器必须实现的统一接口
- `controllers/pid.py`：PID 控制器
- `controllers/brl_ppc.py`：BRL-PPC 自适应控制器
- `controllers/mfasfc.py`：MFASFC 控制器

## 5. 统一控制器接口示例

```python
class BaseController:
    name = "base"

    def reset(self, q0, dq0=None):
        """在实验开始前初始化控制器内部状态。"""
        raise NotImplementedError

    def compute(self, t, q, dq, xr, dxr, ddxr):
        """根据当前状态和参考轨迹计算关节力矩。"""
        raise NotImplementedError

    def get_log(self):
        """返回控制器内部需要记录的变量。"""
        return {}
```

PID 控制器可以实现为：

```python
class PIDController(BaseController):
    name = "pid"

    def __init__(self, kp, kd, torque_limit):
        self.kp = kp
        self.kd = kd
        self.torque_limit = torque_limit

    def reset(self, q0, dq0=None):
        pass

    def compute(self, t, q, dq, xr, dxr, ddxr):
        error = xr - q
        d_error = dxr - dq
        torque = self.kp @ error + self.kd @ d_error
        torque = np.clip(torque, -self.torque_limit, self.torque_limit)
        return torque, {"error": error}
```

BRL-PPC 控制器则把当前 `BRL-PPC_3.0.py` 中的 `_init_controller`、`_compute_ppc`、`_bls_expand`、`_zeta_lag`、Actor-Critic 更新和最终控制律迁移进去。

## 6. 重构步骤

建议按以下顺序重构，风险较低：

1. 先复制当前 `BRL-PPC_3.0.py`，保留原始可运行版本。
2. 抽出 `ReferenceTrajectory` 到 `reference.py`。
3. 抽出 Kinova 连接、模式切换、反馈读取和力矩下发到 `robot_interface.py`。
4. 抽出 1 kHz 主循环、日志记录和清理流程到 `robot_runner.py`。
5. 定义 `BaseController` 接口。
6. 将当前 BRL-PPC 控制律迁移为 `BRLPPCController`。
7. 再实现一个简单 `PIDController`，用它验证平台是否真的支持算法替换。
8. 最后接入 MFASFC 或其他控制算法。

## 7. 简历表述建议

如果还没有完成上述重构，建议写：

> 基于 Python/Kortex API 开发 Kinova Gen3 机械臂低层力矩控制实验程序，完成机械臂连接、初始位姿配置、1 kHz 反馈采集与力矩指令下发、力矩限幅、异常恢复、数据记录和离线可视化，并在该流程中集成 BRL-PPC 控制策略完成 J4/J6 轨迹跟踪验证。

如果完成重构并验证 PID、BRL-PPC 等多个控制器后，可以写：

> 搭建 Kinova Gen3 机械臂实时力矩控制算法验证平台，抽象机械臂通信、伺服模式切换、1 kHz 控制循环、数据记录与安全恢复等公共流程，支持 PID、BRL-PPC、MFASFC 等控制算法通过统一接口快速接入和实机验证。

