"""Kinova 机械臂接口封装。

负责连接后的模式切换、关节反馈读取、力矩指令下发和实验结束恢复。
控制算法不需要直接调用 Kortex API。
"""

import math
import threading
import time

import numpy as np

from kortex_api.RouterClient import RouterClientSendOptions
from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import (
    ActuatorConfigClient,
)
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import ActuatorConfig_pb2, BaseCyclic_pb2, Base_pb2


def normalize_deg(deg):
    return deg - 360.0 if deg > 180.0 else deg


class KinovaTorqueInterface:
    """Kinova Gen3 力矩控制相关操作。"""

    def __init__(
        self,
        router,
        router_real_time,
        torque_joints=(3, 5),
        start_angles_deg=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -90.0),
        action_timeout=20.0,
    ):
        self.base = BaseClient(router)
        self.base_cyclic = BaseCyclicClient(router_real_time)
        self.actuator_config = ActuatorConfigClient(router)
        self.torque_joints = list(torque_joints)
        self.jid2idx = {jid: idx for idx, jid in enumerate(self.torque_joints)}
        self.start_angles_deg = list(start_angles_deg)
        self.action_timeout = float(action_timeout)

        self.command = BaseCyclic_pb2.Command()
        self.command.frame_id = 0
        self.actuator_count = None
        self.previous_servoing_mode = None

        self.send_option = RouterClientSendOptions()
        self.send_option.andForget = False
        self.send_option.delay_ms = 0
        self.send_option.timeout_ms = 3

    def prepare(self):
        if not self.move_to_angles(self.start_angles_deg, "MoveToStart"):
            raise RuntimeError("Cannot reach start position")

        feedback = self.base_cyclic.RefreshFeedback()
        self.actuator_count = len(feedback.actuators)
        for actuator in feedback.actuators:
            cmd = self.command.actuators.add()
            cmd.position = actuator.position
            cmd.velocity = 0.0
            cmd.torque_joint = 0.0
            cmd.command_id = 0

        self.previous_servoing_mode = self.base.GetServoingMode()
        svm = Base_pb2.ServoingModeInformation()
        svm.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
        self.base.SetServoingMode(svm)
        time.sleep(1.0)

        current_mode = self.base.GetServoingMode()
        if current_mode.servoing_mode != Base_pb2.LOW_LEVEL_SERVOING:
            mode_name = Base_pb2.ServoingMode.Name(current_mode.servoing_mode)
            raise RuntimeError(f"Failed to enter LOW_LEVEL_SERVOING: {mode_name}")

        for jid in self.torque_joints:
            self.set_joint_mode(jid, "TORQUE")

        return self.read_state()

    def read_state(self):
        feedback = self.base_cyclic.RefreshFeedback()
        return self.state_from_feedback(feedback)

    def refresh(self):
        feedback = self.base_cyclic.Refresh(self.command, 0, self.send_option)
        return feedback, self.state_from_feedback(feedback)

    def state_from_feedback(self, feedback):
        q = []
        dq = []
        for jid in self.torque_joints:
            actuator = feedback.actuators[jid]
            q.append(math.radians(normalize_deg(actuator.position)))
            dq.append(math.radians(actuator.velocity))
        return np.asarray(q, dtype=float), np.asarray(dq, dtype=float)

    def send_torque(self, feedback, torque):
        torque = np.asarray(torque, dtype=float)
        for i in range(self.actuator_count):
            self.command.actuators[i].position = feedback.actuators[i].position
            if i in self.jid2idx:
                self.command.actuators[i].torque_joint = float(torque[self.jid2idx[i]])
            else:
                self.command.actuators[i].torque_joint = 0.0
        self.increment_frame_id()

    def warmup_zero_torque(self, duration=0.2, dt=0.001):
        end_time = time.perf_counter() + duration
        while time.perf_counter() < end_time:
            feedback = self.base_cyclic.Refresh(self.command, 0, self.send_option)
            self.send_torque(feedback, np.zeros(len(self.torque_joints)))
            time.sleep(dt)

    def cleanup(self, return_home=True):
        print("\n--- Cleanup ---")
        self.zero_torque(duration=0.2)
        for jid in self.torque_joints:
            self.set_joint_mode(jid, "POSITION")
        if self.previous_servoing_mode is not None:
            self.base.SetServoingMode(self.previous_servoing_mode)
        time.sleep(0.5)

        try:
            self.base.ClearFaults()
            time.sleep(1.0)
        except Exception as exc:
            print(f"ClearFaults ignored: {exc}")

        if return_home:
            svm = Base_pb2.ServoingModeInformation()
            svm.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(svm)
            time.sleep(0.5)
            self.move_to_angles(self.start_angles_deg, "ReturnToStart")

    def zero_torque(self, duration=0.2):
        if self.actuator_count is None:
            return
        end_time = time.perf_counter() + duration
        while time.perf_counter() < end_time:
            feedback = self.base_cyclic.RefreshFeedback()
            self.send_torque(feedback, np.zeros(len(self.torque_joints)))
            self.base_cyclic.Refresh(self.command, 0, self.send_option)
            time.sleep(0.001)

    def move_to_angles(self, angles_deg, name):
        svm = Base_pb2.ServoingModeInformation()
        svm.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(svm)

        actuator_count = self.base.GetActuatorCount().count
        if actuator_count != len(angles_deg):
            print(f"Actuator count mismatch: {actuator_count} != {len(angles_deg)}")
            return False

        event = threading.Event()
        handle = self.base.OnNotificationActionTopic(
            self._check_for_end_or_abort(event), Base_pb2.NotificationOptions()
        )

        action = Base_pb2.Action()
        action.name = name
        action.application_data = ""
        for i, angle in enumerate(angles_deg):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = i
            joint_angle.value = float(angle)

        self.base.ExecuteAction(action)
        finished = event.wait(self.action_timeout)
        self.base.Unsubscribe(handle)
        if finished:
            time.sleep(2.0)
        return finished

    def set_joint_mode(self, joint_id, mode):
        msg = ActuatorConfig_pb2.ControlModeInformation()
        msg.control_mode = ActuatorConfig_pb2.ControlMode.Value(mode)
        self.actuator_config.SetControlMode(msg, joint_id + 1)
        print(f"Joint {joint_id + 1} -> {mode}")

    def increment_frame_id(self):
        self.command.frame_id = (self.command.frame_id + 1) % 65536
        for i in range(self.actuator_count):
            self.command.actuators[i].command_id = self.command.frame_id

    def _check_for_end_or_abort(self, event):
        def check(notification, event=event):
            name = Base_pb2.ActionEvent.Name(notification.action_event)
            print("EVENT:", name)
            if notification.action_event in (Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT):
                event.set()

        return check
