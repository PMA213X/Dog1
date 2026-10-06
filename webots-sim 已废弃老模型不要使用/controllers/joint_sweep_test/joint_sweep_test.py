#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""独立的 Webots 关节扫掠测试控制器。

控制器按 fr → fl → hr → hl 的腿序逐个扫掠 12 个关节，每条腿内按
abd → hip → kn 的顺序执行。单个关节的目标路径固定为
0 → min → 0 → max → 0，并以 0.20 rad/s 的速度插值；min 和 max
两个端点在实际位置到位后各保持 0.5 秒。

本文件只依赖 Webots Python 控制器接口，不加载模型、checkpoint、TCP 或
RL 依赖，可直接作为 Webots 控制器运行，也可由同目录单测导入。
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from typing import Callable, Sequence


LEG_ORDER: tuple[str, ...] = ("fr", "fl", "hr", "hl")
JOINT_ORDER: tuple[str, ...] = ("abd", "hip", "kn")
MOTOR_NAMES: tuple[str, ...] = tuple(
    f"{leg}_{joint}_motor" for leg in LEG_ORDER for joint in JOINT_ORDER
)
SENSOR_NAMES: tuple[str, ...] = tuple(
    f"{leg}_{joint}_sensor" for leg in LEG_ORDER for joint in JOINT_ORDER
)

SWEEP_SPEED_RAD_S = 0.20
ENDPOINT_HOLD_SECONDS = 0.5
POSITION_ERROR_LIMIT_RAD = 0.05
POSITION_ERROR_TIMEOUT_SECONDS = 10.0
RETURN_ZERO_TOLERANCE_RAD = 0.05

# Webots Keyboard 对 Esc 的键码；同时接受大小写 r/R。
ESCAPE_KEY = 27
STOP_KEYS: frozenset[int] = frozenset((ESCAPE_KEY, ord("r"), ord("R")))

STATUS_RUNNING = "running"
STATUS_COMPLETED = "completed"
STATUS_TIMEOUT = "timeout"
STATUS_STOP_KEY = "stop_key"
STATUS_SIMULATION_ENDED = "simulation_ended"


@dataclass(frozen=True)
class MotionPhase:
    """单关节路径中的一段匀速运动。"""

    start: float
    end: float
    duration_seconds: float
    hold_endpoint: bool


def build_joint_path(minimum: float, maximum: float) -> tuple[float, ...]:
    """返回批准的单关节路径 0 → min → 0 → max → 0。"""

    return (0.0, float(minimum), 0.0, float(maximum), 0.0)


def build_motion_phases(
    minimum: float,
    maximum: float,
    speed_rad_s: float = SWEEP_SPEED_RAD_S,
) -> tuple[MotionPhase, ...]:
    """把单关节路径拆成四段匀速运动，并标记 min/max 端点保持。"""

    if not math.isfinite(minimum) or not math.isfinite(maximum):
        raise ValueError("关节 min/max 位置必须是有限数值")
    if not minimum < 0.0 < maximum:
        raise ValueError(f"关节范围必须严格跨越零位: min={minimum}, max={maximum}")
    if not math.isfinite(speed_rad_s) or speed_rad_s <= 0.0:
        raise ValueError(f"扫掠速度必须大于 0: {speed_rad_s}")

    path = build_joint_path(minimum, maximum)
    phases: list[MotionPhase] = []
    for index, (start, end) in enumerate(zip(path, path[1:])):
        duration = abs(end - start) / speed_rad_s
        hold_endpoint = index in (0, 2)
        phases.append(
            MotionPhase(
                start=float(start),
                end=float(end),
                duration_seconds=float(duration),
                hold_endpoint=hold_endpoint,
            )
        )
    return tuple(phases)


class JointSweepController:
    """维护关节顺序、路径状态、错误超时和停止保持逻辑。"""

    def __init__(
        self,
        robot,
        keyboard=None,
        *,
        output: Callable[[str], None] = print,
        time_step_ms: int | None = None,
        speed_rad_s: float = SWEEP_SPEED_RAD_S,
        endpoint_hold_seconds: float = ENDPOINT_HOLD_SECONDS,
        position_error_limit_rad: float = POSITION_ERROR_LIMIT_RAD,
        position_error_timeout_seconds: float = POSITION_ERROR_TIMEOUT_SECONDS,
    ) -> None:
        if time_step_ms is None:
            time_step_ms = int(robot.getBasicTimeStep())
        if int(time_step_ms) <= 0:
            raise ValueError(f"Webots 基本步长必须大于 0: {time_step_ms}")
        if endpoint_hold_seconds < 0.0:
            raise ValueError("端点保持时间不能为负数")
        if position_error_limit_rad <= 0.0:
            raise ValueError("位置误差阈值必须大于 0")
        if position_error_timeout_seconds <= 0.0:
            raise ValueError("位置误差超时必须大于 0")

        self.robot = robot
        self.keyboard = keyboard
        self.output = output
        self.time_step_ms = int(time_step_ms)
        self.speed_rad_s = float(speed_rad_s)
        self.endpoint_hold_seconds = float(endpoint_hold_seconds)
        self.position_error_limit_rad = float(position_error_limit_rad)
        self.position_error_timeout_seconds = float(
            position_error_timeout_seconds
        )

        self.motors = self._get_devices(MOTOR_NAMES, "电机")
        self.sensors = self._get_devices(SENSOR_NAMES, "位置传感器")
        for sensor in self.sensors:
            enable = getattr(sensor, "enable", None)
            if callable(enable):
                enable(self.time_step_ms)

        self._limits: list[tuple[float, float]] = []
        for index, motor in enumerate(self.motors):
            minimum, maximum = self._read_limits(motor, MOTOR_NAMES[index])
            if not minimum < 0.0 < maximum:
                raise ValueError(
                    f"关节范围必须严格跨越零位: {MOTOR_NAMES[index]} "
                    f"min={minimum}, max={maximum}"
                )
            self._limits.append((minimum, maximum))

        self._phases = tuple(
            build_motion_phases(minimum, maximum, self.speed_rad_s)
            for minimum, maximum in self._limits
        )
        self._initialized = False
        self._mode = "sweep"
        self._joint_index = 0
        self._phase_index = 0
        self._phase_started_at = 0.0
        self._dwell_started_at: float | None = None
        self._arrival_logged = False
        self._error_started_at: float | None = None
        self._status = STATUS_RUNNING
        self._stop_reason = ""
        self._hold_positions: tuple[float, ...] | None = None

    @staticmethod
    def _read_limits(motor, name: str) -> tuple[float, float]:
        get_minimum = getattr(motor, "getMinPosition", None)
        get_maximum = getattr(motor, "getMaxPosition", None)
        if not callable(get_minimum) or not callable(get_maximum):
            raise RuntimeError(f"电机不支持读取位置范围: {name}")
        minimum = float(get_minimum())
        maximum = float(get_maximum())
        return minimum, maximum

    def _get_devices(self, names: Sequence[str], kind: str):
        devices = []
        for name in names:
            device = self.robot.getDevice(name)
            if device is None:
                raise RuntimeError(f"找不到{kind}: {name}")
            devices.append(device)
        return devices

    @property
    def status(self) -> str:
        """当前控制器状态。"""

        return self._status

    @property
    def stop_reason(self) -> str:
        """停止原因；运行中为空字符串。"""

        return self._stop_reason

    @property
    def hold_positions(self) -> tuple[float, ...] | None:
        """停止时冻结的 12 关节实际位置。"""

        return self._hold_positions

    @property
    def active_joint_name(self) -> str:
        """当前扫掠的关节设备名；回零阶段返回 `全部回零`。"""

        if self._mode == "return_zero":
            return "全部回零"
        return MOTOR_NAMES[self._joint_index]

    def _log(
        self,
        stage: str,
        joint_name: str,
        target: float,
        actual: float,
        now: float,
        *,
        extra: str = "",
    ) -> None:
        suffix = f" {extra}" if extra else ""
        self.output(
            f"[{stage}] 关节={joint_name} 时间={now:.6f} "
            f"目标={target:.6f} 实际={actual:.6f}{suffix}"
        )

    def _read_actual(self) -> list[float]:
        values = []
        for sensor in self.sensors:
            value = sensor.getValue()
            if value is None or not math.isfinite(float(value)):
                raise RuntimeError("关节位置传感器返回非有限数值")
            values.append(float(value))
        return values

    def _command(self, index: int, target: float) -> None:
        if not math.isfinite(target):
            raise RuntimeError(f"电机目标不是有限数值: {MOTOR_NAMES[index]}")
        self.motors[index].setPosition(target)

    def _initialize(self, now: float) -> None:
        """读取初始位置，非当前关节只按当前实际位置下一次目标。"""

        initial = self._read_actual()
        for index, position in enumerate(initial):
            self._command(index, position)

        self._initialized = True
        self._mode = "sweep"
        self._joint_index = 0
        self._phase_index = 0
        self._phase_started_at = now
        self._dwell_started_at = None
        self._arrival_logged = False
        self._error_started_at = None

        phase = self._phases[0][0]
        self._command(0, phase.start)
        actual = self._read_actual()[0]
        self._log(
            "开始",
            MOTOR_NAMES[0],
            phase.start,
            actual,
            now,
            extra=f"路径=0→min→0→max→0 速度={self.speed_rad_s:.6f}",
        )

    @staticmethod
    def _interpolate(phase: MotionPhase, elapsed: float) -> float:
        if phase.duration_seconds <= 0.0:
            return phase.end
        ratio = min(max(elapsed / phase.duration_seconds, 0.0), 1.0)
        return phase.start + (phase.end - phase.start) * ratio

    def _start_phase(self, now: float) -> float:
        phase = self._phases[self._joint_index][self._phase_index]
        self._phase_started_at = now
        self._dwell_started_at = None
        self._arrival_logged = False
        self._error_started_at = None
        target = phase.start
        self._command(self._joint_index, target)
        actual = self._read_actual()[self._joint_index]
        self._log("开始", MOTOR_NAMES[self._joint_index], target, actual, now)
        return target

    def _finish_phase(self, now: float, target: float, actual: float) -> bool:
        """记录一段路径结束；返回 `True` 表示端点保持尚未结束。"""

        joint_name = MOTOR_NAMES[self._joint_index]
        if not self._arrival_logged:
            self._arrival_logged = True
            self._log("到位", joint_name, target, actual, now)
            self._log("结束", joint_name, target, actual, now)

        phase = self._phases[self._joint_index][self._phase_index]
        if phase.hold_endpoint and self._dwell_started_at is None:
            self._dwell_started_at = now
            self._log("端点保持开始", joint_name, target, actual, now)
            return True

        if phase.hold_endpoint and self._dwell_started_at is not None:
            if now - self._dwell_started_at + 1e-12 < self.endpoint_hold_seconds:
                return True
            self._log(
                "端点保持结束",
                joint_name,
                target,
                actual,
                now,
                extra=f"保持={self.endpoint_hold_seconds:.6f}",
            )

        self._phase_index += 1
        self._dwell_started_at = None
        self._arrival_logged = False
        if self._phase_index < len(self._phases[self._joint_index]):
            self._start_phase(now)
            return False

        self._advance_joint(now)
        return False

    def _advance_joint(self, now: float) -> None:
        completed_index = self._joint_index
        self._command(completed_index, 0.0)
        self._joint_index += 1
        if self._joint_index < len(MOTOR_NAMES):
            self._phase_index = 0
            self._start_phase(now)
            return
        self._start_return_zero(now)

    def _start_return_zero(self, now: float) -> None:
        self._mode = "return_zero"
        self._joint_index = 0
        self._phase_index = 0
        self._dwell_started_at = None
        self._arrival_logged = False
        self._error_started_at = None
        for index in range(len(self.motors)):
            self._command(index, 0.0)
        actuals = self._read_actual()
        self._log(
            "开始",
            "全部回零",
            0.0,
            sum(actuals) / len(actuals),
            now,
            extra="全部12关节回零",
        )

    def _check_position_error(
        self,
        now: float,
        joint_name: str,
        target: float,
        actual: float,
    ) -> bool:
        """超时则冻结当前实际位置并返回 `True`。"""

        error = abs(target - actual)
        if error <= self.position_error_limit_rad:
            self._error_started_at = None
            return False
        if self._error_started_at is None:
            self._error_started_at = now
            return False
        elapsed = now - self._error_started_at
        if elapsed + 1e-12 < self.position_error_timeout_seconds:
            return False
        self._stop_and_hold(
            now,
            STATUS_TIMEOUT,
            "位置误差超时",
            joint_name=joint_name,
            target=target,
            actual=actual,
            extra=(
                f"误差={error:.6f} 阈值={self.position_error_limit_rad:.6f} "
                f"持续={elapsed:.6f} 超时={self.position_error_timeout_seconds:.6f}"
            ),
        )
        return True

    def _stop_and_hold(
        self,
        now: float,
        status: str,
        reason: str,
        *,
        joint_name: str,
        target: float | None,
        actual: float | None,
        extra: str = "",
    ) -> None:
        actual_positions = self._read_actual()
        for index, position in enumerate(actual_positions):
            self._command(index, position)
        self._hold_positions = tuple(actual_positions)
        self._status = status
        self._stop_reason = reason
        logged_actual = (
            actual_positions[0] if actual is None else float(actual)
        )
        logged_target = logged_actual if target is None else float(target)
        self._log(
            "停止并保持",
            joint_name,
            logged_target,
            logged_actual,
            now,
            extra=f"原因={reason}{(' ' + extra) if extra else ''}",
        )

    @staticmethod
    def _is_stop_key(key) -> bool:
        if key is None:
            return False
        try:
            return int(key) in STOP_KEYS
        except (TypeError, ValueError):
            return str(key) in ("ESC", "Esc", "esc", "r", "R")

    def _tick_sweep(self, now: float) -> str:
        phase = self._phases[self._joint_index][self._phase_index]
        elapsed = max(0.0, now - self._phase_started_at)
        target = (
            phase.end
            if self._dwell_started_at is not None
            else self._interpolate(phase, elapsed)
        )
        self._command(self._joint_index, target)
        actual = self._read_actual()[self._joint_index]
        joint_name = MOTOR_NAMES[self._joint_index]
        if self._check_position_error(now, joint_name, target, actual):
            return self._status

        planned_finished = elapsed + 1e-12 >= phase.duration_seconds
        arrived = abs(actual - phase.end) <= self.position_error_limit_rad
        if planned_finished and arrived:
            if self._finish_phase(now, phase.end, actual):
                return STATUS_RUNNING
        return STATUS_RUNNING

    def _tick_return_zero(self, now: float) -> str:
        for index in range(len(self.motors)):
            self._command(index, 0.0)
        actuals = self._read_actual()
        worst_index = max(
            range(len(actuals)),
            key=lambda index: abs(actuals[index]),
        )
        worst_actual = actuals[worst_index]
        if self._check_position_error(
            now,
            "全部回零",
            0.0,
            worst_actual,
        ):
            return self._status

        if all(
            abs(actual) <= RETURN_ZERO_TOLERANCE_RAD for actual in actuals
        ):
            self._log("到位", "全部回零", 0.0, worst_actual, now)
            self._log("结束", "全部回零", 0.0, worst_actual, now)
            self._status = STATUS_COMPLETED
            self.output("关节扫掠完成，已全部回零")
            return STATUS_COMPLETED
        return STATUS_RUNNING

    def tick(self, now: float, key=None) -> str:
        """推进一个控制周期；`now` 为 Webots 仿真时间（秒）。"""

        if self._status != STATUS_RUNNING:
            return self._status
        if not math.isfinite(float(now)):
            raise ValueError(f"仿真时间必须是有限数值: {now}")
        if not self._initialized:
            self._initialize(float(now))

        if self._is_stop_key(key):
            self._stop_and_hold(
                float(now),
                STATUS_STOP_KEY,
                "Esc/R 停止",
                joint_name=self.active_joint_name,
                target=None,
                actual=None,
                extra=f"键码={key!r}",
            )
            return self._status

        if self._mode == "return_zero":
            return self._tick_return_zero(float(now))
        return self._tick_sweep(float(now))


def _load_webots_robot():
    """延迟加载 Webots Robot，避免单测环境依赖 controller 包。"""

    try:
        from controller import Robot
    except ImportError:  # pragma: no cover - 仅在 Webots 外部直接运行时触发
        home = os.environ.get("WEBOTS_HOME", "/usr/local/webots")
        sys.path.insert(0, os.path.join(home, "lib", "controller", "python"))
        from controller import Robot
    return Robot()


def run_webots_controller() -> str:
    """Webots 主循环；完成后打印回零完成信息并返回状态。"""

    robot = _load_webots_robot()
    time_step_ms = int(robot.getBasicTimeStep())
    keyboard = robot.getKeyboard() if hasattr(robot, "getKeyboard") else None
    if keyboard is not None:
        enable = getattr(keyboard, "enable", None)
        if callable(enable):
            enable(time_step_ms)

    controller = JointSweepController(
        robot,
        keyboard,
        time_step_ms=time_step_ms,
    )
    # 先推进一个仿真步，让刚启用的位置传感器获得首个有效读数。
    if robot.step(time_step_ms) == -1:
        return STATUS_SIMULATION_ENDED

    while True:
        if robot.step(time_step_ms) == -1:
            return STATUS_SIMULATION_ENDED
        key = keyboard.getKey() if keyboard is not None else None
        status = controller.tick(float(robot.getTime()), key)
        if status != STATUS_RUNNING:
            return status


if __name__ == "__main__":  # pragma: no cover - Webots 控制器入口
    run_webots_controller()
