#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Rapid Mini Cheetah 冻结策略的观测、历史和关节安全适配。"""

from __future__ import annotations

import hashlib
import json
import math
import pickle
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, Iterable, Sequence

import numpy as np
import torch

from . import contract


RAPID_MODEL_DIR = (
    contract.PROJECT_ROOT / "external_models" / "rapid_locomotion_f5143ef"
)
SOURCE_COMMIT = "f5143ef940e934849c00284e34caf164d6ce7b6e"
SOURCE_OBS_DIM = 42
SOURCE_HISTORY_LENGTH = 15
SOURCE_HISTORY_DIM = SOURCE_OBS_DIM * SOURCE_HISTORY_LENGTH
SOURCE_LATENT_DIM = 18
SOURCE_BODY_INPUT_DIM = SOURCE_OBS_DIM + SOURCE_LATENT_DIM
SOURCE_ACTION_DIM = contract.ACTION_DIM
SOURCE_COMMAND_SCALE = np.asarray((2.0, 2.0, 0.25), dtype=np.float32)
SOURCE_JOINT_VEL_SCALE = 0.05
SOURCE_OBS_CLIP = 100.0
SOURCE_ACTION_CLIP = 100.0
CURRENT_ACTION_CLIP = 1.0

SOURCE_LEGS = ("FR", "FL", "RR", "RL")
SOURCE_JOINTS = ("hip", "thigh", "calf")
SOURCE_JOINT_NAMES = tuple(
    f"{leg}_{joint}_joint"
    for leg in SOURCE_LEGS
    for joint in SOURCE_JOINTS
)
CURRENT_JOINT_NAMES = tuple(
    f"{leg}_{joint}"
    for leg in contract.LEG_NAMES
    for joint in contract.JOINT_NAMES
)
# 源模型顺序 FR/FL/RR/RL×hip/thigh/calf 与当前顺序 fr/fl/hr/hl×abd/hip/kn
# 对齐；符号由源 URDF 和当前 world 的关节轴点积硬校验。
SOURCE_TO_CURRENT_INDICES = tuple(range(contract.ACTION_DIM))
JOINT_SIGNS = (1,) * contract.ACTION_DIM
JOINT_MAPPING = tuple(
    (
        f"{source_leg}_{source_joint}_joint",
        f"{current_leg}_{current_joint}",
        source_index,
        JOINT_SIGNS[source_index],
    )
    for source_index, (
        source_leg,
        source_joint,
        current_leg,
        current_joint,
    ) in enumerate(
        (
            (source_leg, source_joint, current_leg, current_joint)
            for source_leg, current_leg in zip(
                SOURCE_LEGS,
                contract.LEG_NAMES,
            )
            for source_joint, current_joint in zip(
                SOURCE_JOINTS,
                contract.JOINT_NAMES,
            )
        )
    )
)
SOURCE_ACTION_SCALES = tuple(
    value
    for _leg in SOURCE_LEGS
    for value in (0.125, 0.25, 0.25)
)
CURRENT_ACTION_SCALES = tuple(contract.ACTION_SCALE)
SOURCE_DEFAULT_ANGLES = tuple(
    value
    for leg_sign in (-1.0, 1.0, -1.0, 1.0)
    for value in (0.1 * leg_sign, -0.8, 1.62)
)

MANIFEST_REQUIRED_FILES = (
    "ac_weights_last.pt",
    "body_latest.jit",
    "adaptation_module_latest.jit",
    "parameters.pkl",
    "mini_cheetah.urdf",
    "LICENSE",
)


class RapidActionMapper:
    """play、训练和 Gate 共用的源动作到当前动作映射门面。"""

    def __init__(
        self,
        model_dir: Path | str = RAPID_MODEL_DIR,
        *,
        validate: bool = True,
    ) -> None:
        self.model_dir = Path(model_dir).resolve()
        if validate:
            verify_asset_manifest(self.model_dir)
            validate_joint_mapping(self.model_dir)
        self.source_to_current = np.asarray(
            SOURCE_TO_CURRENT_INDICES,
            dtype=np.int64,
        )
        self.joint_signs = np.asarray(JOINT_SIGNS, dtype=np.float32)
        self.source_scales = np.asarray(
            SOURCE_ACTION_SCALES,
            dtype=np.float32,
        )
        self.current_scales = np.asarray(
            CURRENT_ACTION_SCALES,
            dtype=np.float32,
        )

    def map_source_to_current(
        self,
        source_action: Sequence[float],
    ) -> np.ndarray:
        """按角位移等效比例把单个源动作映射到当前动作空间。"""
        raw = _finite_vector(
            source_action,
            SOURCE_ACTION_DIM,
            "源动作",
        )
        source = np.clip(raw, -SOURCE_ACTION_CLIP, SOURCE_ACTION_CLIP)
        mapped = np.empty_like(source, dtype=np.float32)
        for source_index, current_index in enumerate(
            self.source_to_current
        ):
            mapped[current_index] = (
                self.joint_signs[source_index]
                * source[source_index]
                * self.source_scales[source_index]
                / self.current_scales[current_index]
            )
        return np.clip(
            mapped,
            -CURRENT_ACTION_CLIP,
            CURRENT_ACTION_CLIP,
        ).astype(np.float32, copy=False)

    def map_batch_to_current(
        self,
        source_actions: Sequence[Sequence[float]],
    ) -> np.ndarray:
        """批量映射 `(...,12)` 源动作并保持前导维度。"""
        values = np.asarray(source_actions, dtype=np.float32)
        if values.ndim < 1 or values.shape[-1] != SOURCE_ACTION_DIM:
            raise ValueError(
                f"源动作最后一维必须为 {SOURCE_ACTION_DIM}，收到 {values.shape}"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("源动作必须全部有限")
        flat = values.reshape(-1, SOURCE_ACTION_DIM)
        mapped = np.stack(
            [self.map_source_to_current(row) for row in flat]
        ).astype(np.float32, copy=False)
        return mapped.reshape(values.shape)


def _finite_vector(
    values: Iterable[float],
    length: int,
    label: str,
) -> np.ndarray:
    """返回指定长度的有限 float32 向量，拒绝错误维度和非有限值。"""
    try:
        result = np.asarray(tuple(values), dtype=np.float32)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} 必须是 {length} 个有限数") from exc
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} 必须是 {length} 个有限数")
    return result


def projected_gravity_from_rpy(rpy: Sequence[float]) -> np.ndarray:
    """按源环境四元数约定，由 roll/pitch/yaw 计算机体系重力。"""
    roll, pitch, yaw = _finite_vector(rpy, 3, "rpy")
    # yaw 不影响世界 Z 轴重力在机体坐标系中的投影。
    del yaw
    sin_roll, cos_roll = math.sin(float(roll)), math.cos(float(roll))
    sin_pitch, cos_pitch = math.sin(float(pitch)), math.cos(float(pitch))
    return np.asarray(
        (sin_pitch, -cos_pitch * sin_roll, -cos_pitch * cos_roll),
        dtype=np.float32,
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_asset_manifest(
    model_dir: Path | str = RAPID_MODEL_DIR,
) -> Dict[str, str]:
    """校验固定提交 manifest、必备文件和全部 SHA256。"""
    root = Path(model_dir).resolve()
    manifest_path = root / "manifest.json"
    if not manifest_path.is_file():
        raise RuntimeError(f"缺少 Rapid manifest：{manifest_path}")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Rapid manifest 无效：{exc}") from exc
    if manifest.get("source_repository") != (
        "https://github.com/SellCXHarsha/rapid-locomotion-rl"
    ):
        raise RuntimeError("Rapid 来源仓库与固定提交不一致")
    if manifest.get("commit") != SOURCE_COMMIT:
        raise RuntimeError("Rapid 提交不是 f5143ef 固定提交")
    if manifest.get("license") != "MIT":
        raise RuntimeError("Rapid 许可证不是 MIT")
    files = manifest.get("files")
    if not isinstance(files, dict):
        raise RuntimeError("Rapid manifest files 字段无效")
    missing = [name for name in MANIFEST_REQUIRED_FILES if name not in files]
    if missing:
        raise RuntimeError(f"Rapid manifest 缺少文件条目：{missing}")
    verified: Dict[str, str] = {}
    for name, metadata in sorted(files.items()):
        if not isinstance(metadata, dict):
            raise RuntimeError(f"Rapid manifest 文件元数据无效：{name}")
        path = root / name
        expected = metadata.get("sha256")
        if not path.is_file() or not isinstance(expected, str):
            raise RuntimeError(f"Rapid 文件缺失或无哈希：{name}")
        actual = _sha256(path)
        if actual != expected:
            raise RuntimeError(
                f"Rapid 文件 SHA256 不匹配：{name} "
                f"expected={expected} actual={actual}"
            )
        verified[name] = actual
    if not (root / "LICENSE").is_file():
        raise RuntimeError("Rapid 缺少 MIT LICENSE")
    return verified


def _joint_axes_from_urdf(path: Path) -> Dict[str, tuple[float, float, float]]:
    """读取源 Mini Cheetah URDF 的 12 个活动关节轴。"""
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise RuntimeError(f"源 URDF 无效：{exc}") from exc
    result: Dict[str, tuple[float, float, float]] = {}
    for joint in root.findall("joint"):
        if joint.get("type") != "revolute":
            continue
        name = joint.get("name")
        axis = joint.find("axis")
        if name is None or axis is None:
            continue
        try:
            xyz = tuple(float(value) for value in axis.get("xyz", "").split())
        except ValueError as exc:
            raise RuntimeError(f"源 URDF 关节轴无效：{name}") from exc
        if len(xyz) != 3 or not all(math.isfinite(value) for value in xyz):
            raise RuntimeError(f"源 URDF 关节轴无效：{name}")
        result[name] = xyz  # type: ignore[assignment]
    return result


def _world_joint_axes(path: Path) -> Dict[str, tuple[float, float, float]]:
    """读取当前 Webots world 中 12 个 motor 所属 HingeJoint 的轴。"""
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(f"当前 world 无效：{exc}") from exc
    result: Dict[str, tuple[float, float, float]] = {}
    for current_name in CURRENT_JOINT_NAMES:
        motor_name = f"{current_name}_motor"
        motor_positions = [
            match.start()
            for match in re.finditer(
                rf'\bname\s+"{re.escape(motor_name)}"',
                text,
            )
        ]
        if not motor_positions:
            raise RuntimeError(f"当前 world 缺少 motor：{motor_name}")
        observed: set[tuple[float, float, float]] = set()
        for motor_pos in motor_positions:
            block_start = text.rfind("HingeJoint {", 0, motor_pos)
            if block_start < 0:
                raise RuntimeError(
                    f"当前 world motor 不属于 HingeJoint：{motor_name}"
                )
            block = text[block_start:motor_pos]
            axis_match = re.search(
                r"\baxis\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)",
                block,
            )
            position_match = re.search(
                r"\bposition\s+([-\d.eE+]+)",
                block,
            )
            if axis_match is None or position_match is None:
                raise RuntimeError(
                    f"当前 world 关节轴或零位无效：{motor_name}"
                )
            try:
                axis = tuple(float(value) for value in axis_match.groups())
                position = float(position_match.group(1))
            except ValueError as exc:
                raise RuntimeError(
                    f"当前 world 关节数值无效：{motor_name}"
                ) from exc
            if not all(math.isfinite(value) for value in (*axis, position)):
                raise RuntimeError(
                    f"当前 world 关节数值无效：{motor_name}"
                )
            if not math.isclose(position, 0.0, abs_tol=1e-12):
                raise RuntimeError(
                    f"当前 world motor 不是零位启动：{motor_name}"
                )
            observed.add(axis)  # type: ignore[arg-type]
        if len(observed) != 1:
            raise RuntimeError(
                f"当前 world 同名 motor 关节轴不一致：{motor_name} "
                f"axes={sorted(observed)}"
            )
        result[current_name] = observed.pop()  # type: ignore[assignment]
    return result


def _dot_sign(
    left: Sequence[float],
    right: Sequence[float],
    label: str,
) -> int:
    """返回两关节轴的符号；不共线或零轴直接拒绝加载。"""
    left_norm = math.sqrt(sum(float(value) ** 2 for value in left))
    right_norm = math.sqrt(sum(float(value) ** 2 for value in right))
    if left_norm <= 0.0 or right_norm <= 0.0:
        raise RuntimeError(f"{label} 关节轴为零向量")
    cosine = sum(float(a) * float(b) for a, b in zip(left, right))
    cosine /= left_norm * right_norm
    if cosine > 1.0 - 1e-9:
        return 1
    if cosine < -1.0 + 1e-9:
        return -1
    raise RuntimeError(f"{label} 关节轴既不平行也不同向：cos={cosine}")


def validate_joint_mapping(
    model_dir: Path | str = RAPID_MODEL_DIR,
) -> Dict[str, Any]:
    """依据源 URDF、parameters、当前 contract/world 硬校验映射和符号。"""
    root = Path(model_dir).resolve()
    if contract.LEG_NAMES != ("fr", "fl", "hr", "hl"):
        raise RuntimeError(f"当前腿顺序不可映射：{contract.LEG_NAMES}")
    if contract.JOINT_NAMES != ("abd", "hip", "kn"):
        raise RuntimeError(f"当前关节顺序不可映射：{contract.JOINT_NAMES}")
    if len(CURRENT_JOINT_NAMES) != SOURCE_ACTION_DIM:
        raise RuntimeError("当前 12 关节数量错误")
    if tuple(contract.ACTION_SCALE) != CURRENT_ACTION_SCALES:
        raise RuntimeError(f"当前动作尺度变化：{contract.ACTION_SCALE}")
    if len(contract.DEFAULT_CROUCH) != SOURCE_ACTION_DIM:
        raise RuntimeError("当前屈膝基准维度错误")
    try:
        with (root / "parameters.pkl").open("rb") as handle:
            parameters = pickle.load(handle)
    except (OSError, pickle.UnpicklingError, EOFError, AttributeError) as exc:
        raise RuntimeError(f"Rapid parameters.pkl 无效：{exc}") from exc
    if not isinstance(parameters, dict):
        raise RuntimeError("Rapid parameters.pkl 顶层不是 dict")
    cfg = parameters.get("Cfg")
    if not isinstance(cfg, dict):
        raise RuntimeError("Rapid parameters.pkl 缺少 Cfg")
    env_cfg = cfg.get("env")
    normalization = cfg.get("normalization")
    control = cfg.get("control")
    init_state = cfg.get("init_state")
    if not all(
        isinstance(item, dict)
        for item in (env_cfg, normalization, control, init_state)
    ):
        raise RuntimeError("Rapid parameters.pkl 配置字段不完整")
    expected_env = {
        "num_observations": SOURCE_OBS_DIM,
        "num_privileged_obs": SOURCE_LATENT_DIM,
        "num_observation_history": SOURCE_HISTORY_LENGTH,
        "num_actions": SOURCE_ACTION_DIM,
    }
    for key, expected in expected_env.items():
        if int(env_cfg.get(key, -1)) != expected:
            raise RuntimeError(
                f"Rapid 观测/动作配置不匹配：{key}="
                f"{env_cfg.get(key)!r} expected={expected}"
            )
    obs_scales = normalization.get("obs_scales")
    if not isinstance(obs_scales, dict):
        raise RuntimeError("Rapid obs_scales 缺失")
    if float(obs_scales.get("dof_pos", math.nan)) != 1.0:
        raise RuntimeError("Rapid dof_pos 缩放不是 1.0")
    if float(obs_scales.get("dof_vel", math.nan)) != SOURCE_JOINT_VEL_SCALE:
        raise RuntimeError("Rapid dof_vel 缩放不是 0.05")
    if float(normalization.get("clip_observations", math.nan)) != SOURCE_OBS_CLIP:
        raise RuntimeError("Rapid 观测 clip 不是 100")
    if float(control.get("action_scale", math.nan)) != 0.25:
        raise RuntimeError("Rapid 基础 action_scale 不是 0.25")
    if float(control.get("hip_scale_reduction", math.nan)) != 0.5:
        raise RuntimeError("Rapid hip action_scale reduction 不是 0.5")
    defaults = init_state.get("default_joint_angles")
    if not isinstance(defaults, dict) or set(defaults) != set(
        SOURCE_JOINT_NAMES
    ):
        raise RuntimeError("Rapid 默认关节名称与源 URDF 顺序语义不一致")
    source_defaults = _finite_vector(
        (float(defaults[name]) for name in SOURCE_JOINT_NAMES),
        SOURCE_ACTION_DIM,
        "Rapid 默认关节角",
    )
    if not np.allclose(source_defaults, SOURCE_DEFAULT_ANGLES, atol=1e-9):
        raise RuntimeError(
            f"Rapid 默认关节角变化：{tuple(float(v) for v in source_defaults)}"
        )

    source_axes = _joint_axes_from_urdf(root / "mini_cheetah.urdf")
    if set(source_axes) != set(SOURCE_JOINT_NAMES):
        raise RuntimeError("源 URDF 活动关节集合与 parameters 不一致")
    current_axes = _world_joint_axes(contract.WORLD_PATH)
    eval_axes = _world_joint_axes(contract.EVAL_WORLD_PATH)
    if current_axes != eval_axes:
        raise RuntimeError("训练与评测 world 的关节轴不一致")

    derived_signs: list[int] = []
    for source_index, source_name in enumerate(SOURCE_JOINT_NAMES):
        current_index = SOURCE_TO_CURRENT_INDICES[source_index]
        current_name = CURRENT_JOINT_NAMES[current_index]
        sign = _dot_sign(
            source_axes[source_name],
            current_axes[current_name],
            f"{source_name}->{current_name}",
        )
        derived_signs.append(sign)
        # 正负默认角语义一致，才能证明零位方向没有被翻转。
        source_default = float(source_defaults[source_index])
        current_default = float(contract.DEFAULT_CROUCH[current_index])
        if (
            source_default != 0.0
            and current_default != 0.0
            and (source_default > 0.0) != (current_default > 0.0)
        ):
            raise RuntimeError(
                f"默认角零位方向不一致：{source_name}->{current_name}"
            )
    if tuple(derived_signs) != JOINT_SIGNS:
        raise RuntimeError(
            f"Rapid 关节符号无法安全映射：{tuple(derived_signs)}"
        )
    if set(SOURCE_TO_CURRENT_INDICES) != set(range(SOURCE_ACTION_DIM)):
        raise RuntimeError("Rapid 关节置换不是双射")

    source_scales = tuple(
        float(control["action_scale"])
        * (
            float(control["hip_scale_reduction"])
            if index % len(SOURCE_JOINTS) == 0
            else 1.0
        )
        for index in range(SOURCE_ACTION_DIM)
    )
    if source_scales != SOURCE_ACTION_SCALES:
        raise RuntimeError(f"Rapid 动作尺度变化：{source_scales}")
    if any(
        sign * source <= 0.0 or current <= 0.0
        for sign, source, current in zip(
            JOINT_SIGNS,
            SOURCE_ACTION_SCALES,
            CURRENT_ACTION_SCALES,
        )
    ):
        raise RuntimeError("Rapid 动作尺度符号无法安全转换")
    return {
        "source_commit": SOURCE_COMMIT,
        "source_to_current_indices": tuple(SOURCE_TO_CURRENT_INDICES),
        "joint_signs": tuple(JOINT_SIGNS),
        "source_action_scales": SOURCE_ACTION_SCALES,
        "current_action_scales": CURRENT_ACTION_SCALES,
        "source_default_angles": tuple(float(v) for v in source_defaults),
        "current_default_angles": tuple(float(v) for v in contract.DEFAULT_CROUCH),
    }


class RapidPolicyAdapter:
    """把当前 57 维遥控观测转换为 Rapid 冻结模型的 630/60 维接口。"""

    def __init__(
        self,
        model_dir: Path | str = RAPID_MODEL_DIR,
        *,
        device: str | torch.device = "cpu",
    ) -> None:
        self.model_dir = Path(model_dir).resolve()
        self.verified_files = verify_asset_manifest(self.model_dir)
        self.mapping = validate_joint_mapping(self.model_dir)
        self.device = torch.device(device)
        try:
            self.adaptation = torch.jit.load(
                str(self.model_dir / "adaptation_module_latest.jit"),
                map_location=self.device,
            ).eval()
            self.body = torch.jit.load(
                str(self.model_dir / "body_latest.jit"),
                map_location=self.device,
            ).eval()
        except (OSError, RuntimeError) as exc:
            raise RuntimeError(f"Rapid TorchScript 模型加载失败：{exc}") from exc
        self._validate_model_shapes()
        self._source_defaults = np.asarray(
            SOURCE_DEFAULT_ANGLES,
            dtype=np.float32,
        )
        self._source_to_current = np.asarray(
            SOURCE_TO_CURRENT_INDICES,
            dtype=np.int64,
        )
        self._joint_signs = np.asarray(JOINT_SIGNS, dtype=np.float32)
        self._source_scales = np.asarray(
            SOURCE_ACTION_SCALES,
            dtype=np.float32,
        )
        self._current_scales = np.asarray(
            CURRENT_ACTION_SCALES,
            dtype=np.float32,
        )
        self._history: list[np.ndarray] = []
        self._previous_source_action = np.zeros(
            SOURCE_ACTION_DIM,
            dtype=np.float32,
        )
        self.action_mapper = RapidActionMapper(
            self.model_dir,
            validate=False,
        )

    def _validate_model_shapes(self) -> None:
        """用真实 TorchScript 资产验证 630→18 和 60→12。"""
        with torch.inference_mode():
            latent = self.adaptation(
                torch.zeros(
                    (1, SOURCE_HISTORY_DIM),
                    dtype=torch.float32,
                    device=self.device,
                )
            )
            action = self.body(
                torch.zeros(
                    (1, SOURCE_BODY_INPUT_DIM),
                    dtype=torch.float32,
                    device=self.device,
                )
            )
        if tuple(latent.shape) != (1, SOURCE_LATENT_DIM):
            raise RuntimeError(
                f"Rapid adaptation 输出维度错误：{tuple(latent.shape)}"
            )
        if tuple(action.shape) != (1, SOURCE_ACTION_DIM):
            raise RuntimeError(
                f"Rapid body 输出维度错误：{tuple(action.shape)}"
            )
        if not bool(torch.isfinite(latent).all()):
            raise RuntimeError("Rapid adaptation 零输入产生非有限 latent")
        if not bool(torch.isfinite(action).all()):
            raise RuntimeError("Rapid body 零输入产生非有限动作")

    def reset(self) -> None:
        """清空 15 帧历史和源动作状态，供 play reset 调用。"""
        self._history.clear()
        self._previous_source_action.fill(0.0)

    def _map_current_to_source(
        self,
        values: np.ndarray,
    ) -> np.ndarray:
        """把当前关节顺序、符号映射到源模型顺序。"""
        source = np.empty_like(values, dtype=np.float32)
        for source_index, current_index in enumerate(
            SOURCE_TO_CURRENT_INDICES
        ):
            source[source_index] = (
                self._joint_signs[source_index] * values[current_index]
            )
        return source

    def _map_source_to_current(
        self,
        values: np.ndarray,
    ) -> np.ndarray:
        """把源模型关节顺序、符号映射回当前顺序。"""
        current = np.empty_like(values, dtype=np.float32)
        for source_index, current_index in enumerate(
            SOURCE_TO_CURRENT_INDICES
        ):
            current[current_index] = (
                self._joint_signs[source_index] * values[source_index]
            )
        return current

    def build_source_observation(
        self,
        observation_57: Sequence[float],
        *,
        previous_source_action: Sequence[float] | None = None,
    ) -> np.ndarray:
        """构造固定 42 维 Rapid 当前帧，不改变调用方 57 维观测。"""
        observation = _finite_vector(
            observation_57,
            contract.OBS_DIM,
            "57 维观测",
        )
        gravity = projected_gravity_from_rpy(
            observation[contract.OBS_SLICES["rpy"]]
        )
        command = (
            observation[contract.OBS_SLICES["cmd"]]
            * SOURCE_COMMAND_SCALE
        )
        current_q = observation[contract.OBS_SLICES["q"]]
        current_dq = observation[contract.OBS_SLICES["dq"]]
        source_q = self._map_current_to_source(current_q)
        source_dq = self._map_current_to_source(current_dq)
        q_offset = source_q - self._source_defaults
        if previous_source_action is None:
            previous = self._previous_source_action
        else:
            previous = _finite_vector(
                previous_source_action,
                SOURCE_ACTION_DIM,
                "上一源动作",
            )
        result = np.concatenate(
            (
                gravity,
                command,
                q_offset,
                source_dq * SOURCE_JOINT_VEL_SCALE,
                previous,
            )
        ).astype(np.float32, copy=False)
        if result.shape != (SOURCE_OBS_DIM,) or not np.all(
            np.isfinite(result)
        ):
            raise RuntimeError("Rapid 42 维观测构造失败")
        return np.clip(result, -SOURCE_OBS_CLIP, SOURCE_OBS_CLIP)

    def build_history(
        self,
        current: Sequence[float],
        *,
        previous_frames: Sequence[Sequence[float]] | None = None,
    ) -> tuple[np.ndarray, bool]:
        """返回 630 维历史；不足 15 帧时用首帧补齐。"""
        frame = _finite_vector(current, SOURCE_OBS_DIM, "42 维当前帧")
        if previous_frames is None:
            frames = list(self._history)
        else:
            frames = [
                _finite_vector(item, SOURCE_OBS_DIM, "历史帧")
                for item in previous_frames
            ]
        if not frames:
            frames = [frame]
        else:
            frames.append(frame)
        if len(frames) > SOURCE_HISTORY_LENGTH:
            frames = frames[-SOURCE_HISTORY_LENGTH:]
        warmup = len(frames) < SOURCE_HISTORY_LENGTH
        first = frames[0]
        padded = [first] * (SOURCE_HISTORY_LENGTH - len(frames)) + frames
        history = np.concatenate(padded).astype(np.float32, copy=False)
        if history.shape != (SOURCE_HISTORY_DIM,) or not np.all(
            np.isfinite(history)
        ):
            raise RuntimeError("Rapid 630 维历史构造失败")
        return np.clip(history, -SOURCE_OBS_CLIP, SOURCE_OBS_CLIP), warmup

    def map_source_action_to_current(
        self,
        source_action: Sequence[float],
    ) -> np.ndarray:
        """复用统一映射门面，保持 play、训练与 Gate 语义一致。"""
        return self.action_mapper.map_source_to_current(source_action)

    def predict(
        self,
        observation_57: Sequence[float],
        deterministic: bool = True,
        *,
        jump_request: bool = False,
    ) -> tuple[np.ndarray, Dict[str, Any]]:
        """执行冻结模型，返回当前策略动作和可直接写 JSONL 的 info。"""
        if not isinstance(deterministic, (bool, np.bool_)):
            raise ValueError("deterministic 必须是 bool")
        observation = _finite_vector(
            observation_57,
            contract.OBS_DIM,
            "57 维观测",
        )
        source_observation = self.build_source_observation(observation)
        history, warmup = self.build_history(source_observation)
        history_tensor = torch.from_numpy(history).reshape(
            1,
            SOURCE_HISTORY_DIM,
        ).to(self.device)
        observation_tensor = torch.from_numpy(source_observation).reshape(
            1,
            SOURCE_OBS_DIM,
        ).to(self.device)
        with torch.inference_mode():
            latent_tensor = self.adaptation(history_tensor)
            body_input = torch.cat(
                (observation_tensor, latent_tensor),
                dim=-1,
            )
            source_tensor = self.body(body_input)
        if tuple(latent_tensor.shape) != (1, SOURCE_LATENT_DIM):
            raise RuntimeError(
                f"Rapid latent 维度错误：{tuple(latent_tensor.shape)}"
            )
        if tuple(source_tensor.shape) != (1, SOURCE_ACTION_DIM):
            raise RuntimeError(
                f"Rapid 动作维度错误：{tuple(source_tensor.shape)}"
            )
        latent = latent_tensor.detach().cpu().numpy().reshape(-1)
        raw_source = source_tensor.detach().cpu().numpy().reshape(-1)
        if not np.all(np.isfinite(latent)) or not np.all(
            np.isfinite(raw_source)
        ):
            raise RuntimeError("Rapid 模型产生非有限 latent 或动作")
        raw_source = raw_source.astype(np.float32, copy=False)
        source_history_action = np.clip(
            raw_source,
            -SOURCE_ACTION_CLIP,
            SOURCE_ACTION_CLIP,
        ).astype(np.float32, copy=False)
        mapped = self.map_source_action_to_current(source_history_action)
        if not np.all(np.isfinite(mapped)) or mapped.shape != (
            SOURCE_ACTION_DIM,
        ):
            raise RuntimeError("Rapid 关节映射产生非有限动作")

        # 推理成功后才推进历史；失败时状态保持不变，便于安全重试。
        self._history.append(source_observation)
        if len(self._history) > SOURCE_HISTORY_LENGTH:
            self._history = self._history[-SOURCE_HISTORY_LENGTH:]
        self._previous_source_action = source_history_action
        info: Dict[str, Any] = {
            "raw_source_action": [
                float(value) for value in raw_source
            ],
            "source_action_for_history": [
                float(value) for value in source_history_action
            ],
            "mapped_yobo_action": [
                float(value) for value in mapped
            ],
            "latent": [float(value) for value in latent],
            "observation_42": [
                float(value) for value in source_observation
            ],
            "history_630": [
                float(value) for value in history
            ],
            "history_warmup": bool(warmup),
            "history_frames": min(
                len(self._history),
                SOURCE_HISTORY_LENGTH,
            ),
            "deterministic": bool(deterministic),
            "jump_request_supported": False,
            "jump_request_rejected": bool(jump_request),
            "unsupported_requests": ["jump"] if jump_request else [],
            "source_commit": SOURCE_COMMIT,
        }
        return mapped, info


__all__ = [
    "CURRENT_ACTION_SCALES",
    "JOINT_MAPPING",
    "JOINT_SIGNS",
    "RapidActionMapper",
    "RAPID_MODEL_DIR",
    "SOURCE_COMMIT",
    "SOURCE_HISTORY_DIM",
    "SOURCE_HISTORY_LENGTH",
    "SOURCE_OBS_DIM",
    "SOURCE_TO_CURRENT_INDICES",
    "RapidPolicyAdapter",
    "projected_gravity_from_rpy",
    "validate_joint_mapping",
    "verify_asset_manifest",
]
