#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""官方 Mini Cheetah 平地主动站立/行走 Webots 主控制器。

主入口只在 Webots 进程内导入 ``controller``。普通 Python 环境可以直接
导入本文件并测试状态机、输入和 PD 纯模块，不因缺少 Webots 包而失败。
"""

from __future__ import annotations

import math
import os
import sys
import time
from typing import Any, List, Optional, Sequence, Tuple


# Webots 直接执行脚本时通常已把控制器目录加入 sys.path；显式补充可支持
# 测试工具用绝对路径加载本文件的情况。
MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
if MODULE_DIR not in sys.path:
    sys.path.insert(0, MODULE_DIR)

from joint_safety import (  # noqa: E402
    DEFAULT_CROUCH,
    FLAT_MOTOR_NAMES,
    FLAT_SENSOR_NAMES,
    JOINT_COUNT,
    MAX_ABS_JOINT_POSITION,
    MAX_ABS_JOINT_VELOCITY,
    MAX_TORQUE,
    TargetRateLimiter,
    MiniCheetahGait,
    clamp_torque,
    compute_pd_torques,
    standing_targets,
    validate_joint_feedback,
)
from teleop_input import (  # noqa: E402
    INPUT_TIMEOUT_SECONDS,
    KEY_A,
    KEY_D,
    KEY_E,
    KEY_ESCAPE,
    KEY_Q,
    KEY_R,
    KEY_S,
    KEY_SPACE,
    KEY_W,
    KeyboardState,
    MOTION_KEYS,
    WebotsJoystickAdapter,
    merge_commands,
    read_webots_keyboard,
)
from teleop_state import TeleopStateMachine  # noqa: E402
from teleop_types import ControlMode, TeleopCommand  # noqa: E402


DEFAULT_TIMESTEP_MS = 4
STATUS_PERIOD_SECONDS = 2.0
MOTOR_POSITION_CONTROL_DISABLED = float("inf")


def print_operation_help() -> None:
    """启动时输出清楚的键盘/手柄操作提示。"""

    print("=== 官方 Mini Cheetah 平地主动遥控 ===")
    print("默认模式：站立；所有电机使用 12 路 PD，力矩限幅 15 N·m")
    print("Space：站立/行走切换")
    print("A：站立时按下进入行走；该帧不横移，持续按时从下一帧向左横移")
    print("W/S：前进/后退；D：向右横移；Q/E：左转/右转")
    print("R：回站立并清空状态；Esc：强制站立")
    print("手柄：轴0=vy、轴1=vx、轴2=wz；A钮切换模式、B钮复位")
    print("安全规则：输入超时、设备缺失、NaN/Inf、手柄断连均立即回站立")
    print("请先点击 Webots 3D 视图使键盘获得焦点")
    print("======================================")


class FlatGroundTeleopController:
    """Webots 设备适配器 + 主动站立/简化行走控制循环。"""

    def __init__(self, robot: Any, *, clock: Optional[Any] = None) -> None:
        self.robot = robot
        try:
            self.timestep = int(robot.getBasicTimeStep()) or DEFAULT_TIMESTEP_MS
        except Exception:
            self.timestep = DEFAULT_TIMESTEP_MS
        if self.timestep <= 0:
            self.timestep = DEFAULT_TIMESTEP_MS
        self.dt = self.timestep / 1000.0

        self._clock = clock or (lambda: float(self.robot.getTime()))
        self.keyboard_state = KeyboardState(clock=self._clock)
        self.joystick_adapter = WebotsJoystickAdapter()
        self.state_machine = TeleopStateMachine(
            clock=self._clock,
            input_timeout=INPUT_TIMEOUT_SECONDS,
        )
        self.gait = MiniCheetahGait()
        self.target_limiter = TargetRateLimiter(DEFAULT_CROUCH)

        self.motors: List[Any] = []
        self.sensors: List[Any] = []
        self.keyboard: Any = None
        self.joystick: Any = None
        self.missing_devices: List[str] = []
        self.device_fault = False
        self.feedback_fault = False
        self._previous_positions: List[Optional[float]] = [None] * JOINT_COUNT
        self._last_status_at = -float("inf")
        self._warning_printed = set()
        self.mode = ControlMode.STANDING
        self.last_reason = "startup"

    def _warn_once(self, key: str, message: str) -> None:
        if key in self._warning_printed:
            return
        self._warning_printed.add(key)
        print(message)

    def initialize_devices(self) -> bool:
        """获取 12 电机、12 传感器及输入设备；缺失不抛异常。"""

        get_device = getattr(self.robot, "getDevice", None)
        if not callable(get_device):
            self.device_fault = True
            self._warn_once("no_get_device", "【安全】Robot.getDevice 不可用，回站立")
            return False

        for name in FLAT_MOTOR_NAMES:
            try:
                motor = get_device(name)
            except Exception as exc:
                motor = None
                self._warn_once(f"motor_error:{name}", f"【安全】电机 {name} 读取失败: {exc}")
            self.motors.append(motor)
            if motor is None:
                self.missing_devices.append(name)

        for name in FLAT_SENSOR_NAMES:
            try:
                sensor = get_device(name)
            except Exception as exc:
                sensor = None
                self._warn_once(f"sensor_error:{name}", f"【安全】传感器 {name} 读取失败: {exc}")
            self.sensors.append(sensor)
            if sensor is None:
                self.missing_devices.append(name)

        for motor in self.motors:
            if motor is None:
                continue
            try:
                # 关闭 Webots 位置控制，随后每个周期直接给 PD 力矩。
                motor.setPosition(MOTOR_POSITION_CONTROL_DISABLED)
            except Exception as exc:
                self.device_fault = True
                self._warn_once("motor_mode", f"【安全】电机力矩模式设置失败: {exc}")

        for sensor in self.sensors:
            if sensor is None:
                continue
            try:
                sensor.enable(self.timestep)
            except Exception as exc:
                self.device_fault = True
                self._warn_once("sensor_enable", f"【安全】传感器启用失败: {exc}")

        try:
            self.keyboard = self.robot.getKeyboard()
            self.keyboard.enable(self.timestep)
        except Exception as exc:
            self.keyboard = None
            self._warn_once("keyboard", f"【提示】键盘设备不可用，仅使用手柄: {exc}")

        try:
            self.joystick = self.robot.getJoystick()
            if self.joystick is not None:
                self.joystick.enable(self.timestep)
        except Exception as exc:
            self.joystick = None
            self._warn_once("joystick", f"【提示】Webots Joystick 不可用，仅使用键盘: {exc}")

        if self.missing_devices:
            self.device_fault = True
            self._warn_once(
                "missing_devices",
                f"【安全】设备缺失 {len(self.missing_devices)} 个，模式锁定站立: "
                + ", ".join(self.missing_devices),
            )
        return not self.device_fault

    def _read_input(self) -> TeleopCommand:
        now = float(self._clock())
        events = read_webots_keyboard(self.keyboard)
        keyboard_frame = self.keyboard_state.update(events, now)
        joystick_sample = self.joystick_adapter.update(self.joystick)

        vx_key = keyboard_frame.latest_key((KEY_W, KEY_S))
        vy_key = keyboard_frame.latest_key((KEY_A, KEY_D))
        wz_key = keyboard_frame.latest_key((KEY_Q, KEY_E))
        key_vx = {KEY_W: 0.6, KEY_S: -0.3}.get(vx_key, 0.0)
        key_vy = {KEY_A: 0.3, KEY_D: -0.3}.get(vy_key, 0.0)
        key_wz = {KEY_Q: 1.0, KEY_E: -1.0}.get(wz_key, 0.0)
        motion_active = bool(
            any(code in MOTION_KEYS for code in keyboard_frame.active_codes)
            or any(abs(value) > 1e-12 for value in joystick_sample.velocity)
        )

        keyboard_command = TeleopCommand(
            vx=key_vx,
            vy=key_vy,
            wz=key_wz,
            mode_toggle_edge=keyboard_frame.rising(KEY_SPACE),
            a_edge=keyboard_frame.rising(KEY_A),
            reset_edge=keyboard_frame.rising(KEY_R),
            force_stand=keyboard_frame.active(KEY_R) or keyboard_frame.active(KEY_ESCAPE),
            valid=True,
            reason=(
                "reset"
                if keyboard_frame.active(KEY_R)
                else "escape"
                if keyboard_frame.active(KEY_ESCAPE)
                else ""
            ),
            motion_active=motion_active,
        )
        joystick_command = TeleopCommand(
            vx=joystick_sample.vx,
            vy=joystick_sample.vy,
            wz=joystick_sample.wz,
            mode_toggle_edge=joystick_sample.mode_toggle_edge,
            reset_edge=joystick_sample.reset_edge,
            force_stand=joystick_sample.disconnected_edge,
            valid=joystick_sample.valid,
            reason=(
                "joystick_disconnected"
                if joystick_sample.disconnected_edge
                else joystick_sample.warning
            ),
            motion_active=any(abs(value) > 1e-12 for value in joystick_sample.velocity),
        )
        if joystick_sample.warning:
            self._warn_once(f"joystick_warning:{joystick_sample.warning}", f"【安全】{joystick_sample.warning}")
        return merge_commands(keyboard_command, joystick_command)

    def _read_joint_feedback(self, dt: float) -> Tuple[Tuple[float, ...], Tuple[float, ...], bool]:
        positions: List[float] = []
        velocities: List[float] = []
        valid = len(self.sensors) == JOINT_COUNT
        for index, sensor in enumerate(self.sensors):
            if sensor is None:
                positions.append(0.0)
                velocities.append(0.0)
                valid = False
                continue
            try:
                position = float(sensor.getValue())
            except Exception as exc:
                positions.append(0.0)
                velocities.append(0.0)
                valid = False
                self._warn_once(f"sensor_read:{index}", f"【安全】关节反馈读取失败: {exc}")
                continue
            if not math.isfinite(position) or abs(position) > MAX_ABS_JOINT_POSITION:
                positions.append(0.0)
                velocities.append(0.0)
                valid = False
                continue

            previous = self._previous_positions[index]
            if previous is None:
                velocity = 0.0
            else:
                velocity = (position - previous) / dt if dt > 0.0 else 0.0
            if not math.isfinite(velocity) or abs(velocity) > MAX_ABS_JOINT_VELOCITY:
                positions.append(0.0)
                velocities.append(0.0)
                valid = False
                continue
            positions.append(position)
            velocities.append(velocity)
            self._previous_positions[index] = position

        if not validate_joint_feedback(positions, velocities):
            valid = False
        self.feedback_fault = not valid
        return tuple(positions), tuple(velocities), valid

    def _apply_torques(self, torques: Sequence[float]) -> None:
        for index, motor in enumerate(self.motors):
            if motor is None:
                continue
            torque = clamp_torque(torques[index] if index < len(torques) else 0.0, MAX_TORQUE)
            try:
                motor.setTorque(torque)
            except Exception as exc:
                self.device_fault = True
                self._warn_once(f"motor_write:{index}", f"【安全】电机力矩写入失败: {exc}")

    def control_step(self) -> ControlMode:
        """执行一个非阻塞控制周期。"""

        command = self._read_input()
        if self.device_fault:
            command = TeleopCommand(
                force_stand=True,
                valid=True,
                reason="device_missing" if self.missing_devices else "device_fault",
            )

        state_update = self.state_machine.update(command, float(self._clock()))
        self.mode = state_update.mode
        self.last_reason = state_update.reason
        positions, velocities, feedback_valid = self._read_joint_feedback(self.dt)
        if not feedback_valid:
            self.mode = ControlMode.STANDING
            self.last_reason = "sensor_invalid"
            self.gait.reset()
            self.target_limiter = TargetRateLimiter(DEFAULT_CROUCH)
            self.state_machine.reset(float(self._clock()))
            self._apply_torques((0.0,) * JOINT_COUNT)
            self._print_status_if_due()
            return self.mode

        if self.mode == ControlMode.WALKING:
            targets = self.gait.step(state_update.vx, state_update.vy, state_update.wz, self.dt)
        else:
            self.gait.reset()
            targets = standing_targets()

        previous_targets = self.target_limiter.current
        limited_positions = self.target_limiter.apply(targets.positions, self.dt)
        limited_velocities = tuple(
            (current - previous) / self.dt if self.dt > 0.0 else 0.0
            for previous, current in zip(previous_targets, limited_positions)
        )
        torques = compute_pd_torques(
            limited_positions,
            limited_velocities,
            positions,
            velocities,
        )
        self._apply_torques(torques)
        self._print_status_if_due()
        return self.mode

    def _print_status_if_due(self) -> None:
        now = float(self._clock())
        if now - self._last_status_at < STATUS_PERIOD_SECONDS:
            return
        self._last_status_at = now
        mode_text = "行走" if self.mode == ControlMode.WALKING else "站立"
        print(f"[t={now:.2f}] 模式={mode_text} 安全原因={self.last_reason or '-'}")

    def run(self) -> int:
        """Webots 主循环；不做启动阻塞，单步推进由 Webots 驱动。"""

        print_operation_help()
        self.initialize_devices()

        # 先推进一个有限步长，使传感器有首个有效采样；绝不在电机动作前等待。
        try:
            if self.robot.step(self.timestep) < 0:
                return 0
        except Exception as exc:
            self._warn_once("first_step", f"【安全】Webots 首步失败，退出: {exc}")
            return 0

        while True:
            try:
                result = self.robot.step(self.timestep)
            except Exception as exc:
                self._warn_once("step", f"【安全】Webots 步进异常，保持站立并退出: {exc}")
                self._apply_torques((0.0,) * JOINT_COUNT)
                return 0
            if result < 0:
                break
            self.control_step()
        return 0


def main() -> int:
    """Webots 控制器入口；非 Webots 环境给出提示而不会 import 崩溃。"""

    try:
        from controller import Robot  # type: ignore
    except ImportError:
        print("【提示】未找到 Webots controller 包；本文件可导入纯模块进行单测。")
        return 2
    try:
        robot = Robot()
        return FlatGroundTeleopController(robot).run()
    except Exception as exc:
        print(f"【安全】控制器启动异常，不执行电机动作: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
