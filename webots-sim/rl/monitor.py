#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""训练监控面板 —— 让巡检和用户随时看懂 RL 训练状态。

只用 Python 标准库；若本机装了 tensorboard 则优先用它解析事件文件，
否则用内置的最小 TFRecord/protobuf 解析器，再不行就回退到 logs/train.log。

用法：
    python3 webots-sim/rl/monitor.py           # 打印状态 + 写 logs/STATUS.md
    python3 webots-sim/rl/monitor.py --watch   # 每 30 秒刷新一次（Ctrl-C 退出）

监控内容：
    当前阶段 P0-P4 / 训练进程 / 最新 checkpoint / TensorBoard 事件增长 /
    近 100 集奖励与存活步数 / GPU·CPU·内存·磁盘 / 错误计数 /
    启动至今时长与 14h 剩余 / ETA
"""

from __future__ import annotations

import argparse
import re
import struct
import subprocess
import sys
import time
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------
BUDGET_HOURS = 14          # 自主训练时间窗口（小时）
WATCH_INTERVAL = 30        # --watch 刷新间隔（秒）
RECENT_N = 100             # 「最近 N 集」奖励/存活步数的统计窗口
STATUS_MD_NAME = "STATUS.md"

# 阶段定义：P0冒烟 → P1站立 → P2行走 → P3转向 → P4台阶
# 中英文关键词用于 checkpoint 文件名 / 日志；config.py 只认显式标注（见 collect_stage）
STAGES: List[Tuple[str, str, Tuple[str, ...]]] = [
    ("P0", "冒烟", ("p0", "smoke", "冒烟")),
    ("P1", "站立", ("p1", "stand", "站立")),
    ("P2", "行走", ("p2", "walk", "行走")),
    ("P3", "转向", ("p3", "turn", "转向")),
    ("P4", "台阶", ("p4", "stair", "台阶")),
]

# 通用前缀不算阶段证据（否则 ppo_walk 会被误判成 P2 行走）
GENERIC_PREFIXES = ("ppo_walk", "ppo", "walk_env", "ckpt", "checkpoint")

# config.py 中的显式阶段标注（变量/注释/字符串）
STAGE_MARKER_RE = re.compile(
    r"(?:stage|阶段)\s*[:=]?\s*[\"']?(p[0-4]|冒烟|站立|行走|转向|台阶)"
    r"|\b(p[0-4])\b"
    r"|(冒烟|站立|行走|转向|台阶)",
    re.IGNORECASE,
)

# 训练日志中抓奖励/存活步数的正则（依次尝试）
REWARD_PATTERNS = [
    re.compile(r"reward\s*=\s*(-?\d+(?:\.\d+)?)", re.IGNORECASE),
    re.compile(r"平均奖励\s*=\s*(-?\d+(?:\.\d+)?)"),
    re.compile(r"奖励\s*=\s*(-?\d+(?:\.\d+)?)"),
]
LEN_PATTERNS = [
    re.compile(r"ep_len\s*=\s*(-?\d+(?:\.\d+)?)", re.IGNORECASE),
    re.compile(r"平均存活步数\s*=\s*(-?\d+(?:\.\d+)?)"),
    re.compile(r"存活步数\s*=\s*(-?\d+(?:\.\d+)?)"),
]

# TensorBoard 中 episode 奖励 / 存活步数的常见 tag
REWARD_TAGS = ("rollout/ep_remean", "rollout/ep_rew_mean", "rollout/ep_reward_mean")
LEN_TAGS = ("rollout/ep_len_mean", "rollout/ep_len")

# checkpoint 文件名 → 训练步数
CKPT_STEP_RE = re.compile(r"(\d+)\.zip$")
# config.py 里读总步数（兼容 `X: int = 200_000` / `X = 200000` / `X: int: 200_000` 等写法）
TOTAL_STEPS_RE = re.compile(r"DEFAULT_TOTAL_STEPS\s*(?::[^=\n]*)?=\s*([\d_]+)")


# ---------------------------------------------------------------------------
# 路径定位：优先当前工作目录，其次仓库根（monitor.py 的上上级）
# ---------------------------------------------------------------------------
def find_paths() -> Dict[str, Path]:
    """定位 checkpoints / runs / logs / config.py，返回绝对路径字典。"""
    here = Path(__file__).resolve()
    repo_root = here.parents[2] if len(here.parents) >= 3 else Path.cwd()
    candidates = [Path.cwd(), repo_root]

    def pick(name: str) -> Path:
        for base in candidates:
            p = base / name
            if p.exists():
                return p
        return repo_root / name

    return {
        "root": repo_root,
        "checkpoints": pick("checkpoints"),
        "runs": pick("runs"),
        "logs": pick("logs"),
        "config": here.parent / "config.py",
    }


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def run_cmd(cmd: List[str], timeout: float = 5.0) -> str:
    """执行外部命令，失败返回空串（监控脚本绝不能因命令缺失而崩）。"""
    try:
        out = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, check=False
        )
        return (out.stdout or "").strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def fmt_bytes(n: float) -> str:
    """字节数 → 人类可读。"""
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if abs(n) < 1024.0:
            return f"{n:.1f} {unit}"
        n /= 1024.0
    return f"{n:.1f} PB"


def fmt_dur(seconds: float) -> str:
    """秒 → 中文时长。"""
    if seconds < 0:
        seconds = 0.0
    total = int(seconds)
    d, rem = divmod(total, 86400)
    h, rem = divmod(rem, 3600)
    m, s = divmod(rem, 60)
    parts = []
    if d:
        parts.append(f"{d} 天")
    if h or d:
        parts.append(f"{h} 小时")
    parts.append(f"{m} 分")
    if not d and not h:
        parts.append(f"{s} 秒")
    return " ".join(parts)


def fmt_ts(ts: Optional[float]) -> str:
    """epoch → 本地时间字符串。"""
    if ts is None:
        return "—"
    return datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S")


def mean(xs: List[float]) -> Optional[float]:
    return sum(xs) / len(xs) if xs else None


# ---------------------------------------------------------------------------
# 1) 当前阶段 P0-P4
# ---------------------------------------------------------------------------
def _stage_hits(text: str) -> List[str]:
    """在文本里找阶段关键词，返回命中的阶段 code 列表。

    先剔除通用前缀（ppo_walk 等），再按「非字母数字」切词，
    这样 ppo_walk_20000 不会命中 P2，而 p2_walk / stand_5000 会命中。
    """
    low = text.lower()
    for gen in GENERIC_PREFIXES:
        low = low.replace(gen, " ")
    tokens = set(re.split(r"[^0-9a-z\u4e00-\u9fff]+", low))
    hits: List[str] = []
    for code, _name, kws in STAGES:
        if any(kw in tokens for kw in kws):
            hits.append(code)
    return hits


def collect_stage(paths: Dict[str, Path]) -> Dict[str, str]:
    """从 config.py / checkpoint 文件名 / 训练日志推断当前阶段。

    权重：checkpoint 文件名(5) > config.py 显式标注(3) > 训练日志(1)。
    config.py 里 ppo_walk 之类通用名不算证据，只认 STAGE/阶段/P0-P4/中文阶段词。
    """
    scores: Dict[str, int] = {code: 0 for code, _, _ in STAGES}
    evidence: Dict[str, str] = {code: "" for code, _, _ in STAGES}

    # --- config.py：只认显式阶段标注 ---
    cfg = paths["config"]
    if cfg.exists():
        try:
            text = cfg.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            text = ""
        for m in STAGE_MARKER_RE.finditer(text):
            token = next((g for g in m.groups() if g), "")
            token = token.lower()
            for code, name, kws in STAGES:
                if token in kws or token == code.lower():
                    scores[code] += 3
                    evidence[code] = evidence[code] or f"config.py 标注「{token}」"
                    break

    # --- checkpoint 文件名：权重最高 ---
    ckpt_dir = paths["checkpoints"]
    if ckpt_dir.is_dir():
        for p in sorted(ckpt_dir.glob("*.zip")):
            for code in _stage_hits(p.name):
                scores[code] += 5
                evidence[code] = evidence[code] or f"checkpoint {p.name}"

    # --- 训练日志 ---
    logs_dir = paths["logs"]
    if logs_dir.is_dir():
        for p in sorted(logs_dir.glob("*.log")):
            try:
                head = p.read_text(encoding="utf-8", errors="ignore")[:200_000]
            except OSError:
                continue
            for code in _stage_hits(head):
                scores[code] += 1
                evidence[code] = evidence[code] or f"日志 {p.name}"

    best = max(scores, key=lambda c: scores[c])
    if scores[best] <= 0:
        return {"code": "—", "name": "等待启动", "source": "未发现阶段标注"}
    for code, name, _ in STAGES:
        if code == best:
            return {"code": code, "name": name, "source": evidence[code] or "关键词匹配"}
    return {"code": "—", "name": "未知", "source": ""}


# ---------------------------------------------------------------------------
# 2) 训练进程 pid / 运行时长
# ---------------------------------------------------------------------------
def collect_process() -> Dict[str, Any]:
    """用 pgrep 找 train_ppo，再用 ps 取运行时长。"""
    out = run_cmd(["pgrep", "-af", "train_ppo"])
    pids: List[Dict[str, Any]] = []
    for line in out.splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split(None, 1)
        if not parts:
            continue
        pid = parts[0]
        cmd = parts[1] if len(parts) > 1 else ""
        # 排除 monitor 自身与 pgrep 自己
        if "monitor.py" in cmd or cmd.startswith("pgrep"):
            continue
        if not pid.isdigit():
            continue
        etime = run_cmd(["ps", "-o", "etime=", "-p", pid])
        pcpu = run_cmd(["ps", "-o", "pcpu=", "-p", pid])
        pids.append({
            "pid": int(pid),
            "cmd": cmd.strip(),
            "etime": etime.strip() or "—",
            "pcpu": pcpu.strip() or "—",
        })
    return {"alive": bool(pids), "procs": pids}


# ---------------------------------------------------------------------------
# 3) 最新 checkpoint
# ---------------------------------------------------------------------------
def collect_checkpoints(paths: Dict[str, Path]) -> Dict[str, Any]:
    """扫描 checkpoints/*.zip，返回最新一个的时间/大小与步数序列。"""
    ckpt_dir = paths["checkpoints"]
    items: List[Dict[str, Any]] = []
    if ckpt_dir.is_dir():
        for p in ckpt_dir.glob("*.zip"):
            try:
                st = p.stat()
            except OSError:
                continue
            m = CKPT_STEP_RE.search(p.name)
            step = int(m.group(1)) if m else None
            items.append({
                "path": p,
                "name": p.name,
                "mtime": st.st_mtime,
                "size": st.st_size,
                "step": step,
            })
    items.sort(key=lambda d: d["mtime"])
    latest = items[-1] if items else None
    return {"count": len(items), "latest": latest, "all": items}


# ---------------------------------------------------------------------------
# 4) TensorBoard 事件文件（大小 / 增量 / 步数与墙钟）
# ---------------------------------------------------------------------------
def collect_events(paths: Dict[str, Path], prev_sizes: Optional[Dict[str, int]] = None) -> Dict[str, Any]:
    """扫描 runs/*/events*，记录大小、mtime，并解析出 (step, wall_time) 轨迹。"""
    runs_dir = paths["runs"]
    files: List[Dict[str, Any]] = []
    traj: List[Tuple[float, int]] = []   # (wall_time, step)
    if runs_dir.is_dir():
        for p in sorted(runs_dir.rglob("events*")):
            if not p.is_file():
                continue
            try:
                st = p.stat()
            except OSError:
                continue
            key = str(p)
            delta = None
            if prev_sizes is not None and key in prev_sizes:
                delta = st.st_size - prev_sizes[key]
            files.append({
                "path": p,
                "name": str(p.relative_to(runs_dir)),
                "size": st.st_size,
                "mtime": st.st_mtime,
                "delta": delta,
            })
            traj.extend(parse_event_trajectory(p))
    traj.sort(key=lambda t: t[0])
    return {"files": files, "traj": traj, "prev_sizes": {str(f["path"]): f["size"] for f in files}}


# ---- 最小 TFRecord + protobuf 解析（纯标准库，仅取需要的字段） ----
def _read_varint(buf: bytes, i: int) -> Tuple[int, int]:
    """读一个 varint，返回 (值, 新偏移)。"""
    shift = 0
    val = 0
    while i < len(buf):
        b = buf[i]
        i += 1
        val |= (b & 0x7F) << shift
        if not (b & 0x80):
            return val, i
        shift += 7
        if shift > 63:
            break
    return val, i


def _iter_proto_fields(buf: bytes):
    """遍历 protobuf 消息的顶层字段，产出 (field_no, wire_type, payload)。"""
    i = 0
    n = len(buf)
    while i < n:
        key, i = _read_varint(buf, i)
        field_no = key >> 3
        wire = key & 0x07
        if wire == 0:          # varint
            val, i = _read_varint(buf, i)
            yield field_no, wire, val
        elif wire == 1:        # 64-bit
            if i + 8 > n:
                return
            yield field_no, wire, buf[i:i + 8]
            i += 8
        elif wire == 2:        # length-delimited
            length, i = _read_varint(buf, i)
            if i + length > n:
                return
            yield field_no, wire, buf[i:i + length]
            i += length
        elif wire == 5:        # 32-bit
            if i + 4 > n:
                return
            yield field_no, wire, buf[i:i + 4]
            i += 4
        else:
            return  # 未知 wire type，放弃本条消息剩余部分


def parse_event_trajectory(path: Path, max_records: int = 200_000) -> List[Tuple[float, int]]:
    """从 TensorBoard event 文件里抽出 (wall_time, step) 序列（跳过 CRC 校验）。"""
    points: List[Tuple[float, int]] = []
    try:
        with path.open("rb") as fh:
            for _ in range(max_records):
                header = fh.read(8)
                if len(header) < 8:
                    break
                (length,) = struct.unpack("<Q", header)
                if length <= 0 or length > 64 * 1024 * 1024:
                    break
                if len(fh.read(4)) < 4:      # length 的 CRC，跳过
                    break
                data = fh.read(length)
                if len(data) < length:
                    break
                if len(fh.read(4)) < 4:      # data 的 CRC，跳过
                    break

                wall = None
                step = None
                # Event: field1 double wall_time, field2 int64 step, field5 Summary
                for fno, wire, payload in _iter_proto_fields(data):
                    if fno == 1 and wire == 1:
                        wall = struct.unpack("<d", payload)[0]
                    elif fno == 2 and wire == 0:
                        step = int(payload)
                if wall is not None and step is not None:
                    points.append((float(wall), int(step)))
    except OSError:
        return []
    # 同一 step 可能有多个 tag 各写一条 Event，按 step 去重（保留最早墙钟）
    dedup: Dict[int, float] = {}
    for wall, step in points:
        if step not in dedup or wall < dedup[step]:
            dedup[step] = wall
    return sorted((wall, step) for step, wall in dedup.items())


def parse_event_scalars(path: Path, wanted: Tuple[str, ...], max_records: int = 200_000) -> List[float]:
    """从 event 文件抽取指定 tag 的 simple_value 序列。"""
    values: List[float] = []
    try:
        with path.open("rb") as fh:
            for _ in range(max_records):
                header = fh.read(8)
                if len(header) < 8:
                    break
                (length,) = struct.unpack("<Q", header)
                if length <= 0 or length > 64 * 1024 * 1024:
                    break
                if len(fh.read(4)) < 4:
                    break
                data = fh.read(length)
                if len(data) < length:
                    break
                if len(fh.read(4)) < 4:
                    break
                for fno, wire, payload in _iter_proto_fields(data):
                    if fno != 5 or wire != 2:   # 只看 Summary（length-delimited）
                        continue
                    for vno, vwire, vpay in _iter_proto_fields(payload):
                        if vno != 1 or vwire != 2:  # Summary.Value 重复字段
                            continue
                        tag = None
                        simple = None
                        for tno, twire, tpay in _iter_proto_fields(vpay):
                            if tno == 1 and twire == 2:
                                tag = tpay.decode("utf-8", "ignore")
                            elif tno == 2 and twire == 5:
                                simple = struct.unpack("<f", tpay)[0]
                        if tag in wanted and simple is not None:
                            values.append(float(simple))
    except OSError:
        return []
    return values


def read_scalars_via_tensorboard(paths: List[Path], wanted: Tuple[str, ...]) -> Optional[List[float]]:
    """若装了 tensorboard，用 EventAccumulator 读标量（更稳）。"""
    try:
        from tensorboard.backend.event_processing.event_accumulator import (  # type: ignore
            EventAccumulator,
        )
    except Exception:
        return None
    values: List[float] = []
    for p in paths:
        try:
            acc = EventAccumulator(str(p.parent if p.name.startswith("events") else p))
            acc.Reload()
            for tag in wanted:
                if tag in acc.Tags().get("scalars", []):
                    for ev in acc.Scalars(tag):
                        values.append(float(ev.value))
                    break
        except Exception:
            continue
    return values or None


# ---------------------------------------------------------------------------
# 5) 奖励 / 存活步数（event 优先，train.log 兜底）
# ---------------------------------------------------------------------------
def collect_train_metrics(paths: Dict[str, Path], event_files: List[Dict[str, Any]]) -> Dict[str, Any]:
    """最近 RECENT_N 集的奖励均值与存活步数均值。"""
    event_paths = [f["path"] for f in event_files]

    rewards = read_scalars_via_tensorboard(event_paths, REWARD_TAGS)
    if rewards is None:
        rewards = []
        for p in event_paths:
            rewards.extend(parse_event_scalars(p, REWARD_TAGS))
    lens = read_scalars_via_tensorboard(event_paths, LEN_TAGS)
    if lens is None:
        lens = []
        for p in event_paths:
            lens.extend(parse_event_scalars(p, LEN_TAGS))

    source = "TensorBoard 事件"
    # 回退：logs/train.log 里的 reward= / 存活步数= 行
    if not rewards or not lens:
        log_path = paths["logs"] / "train.log"
        if log_path.exists():
            try:
                text = log_path.read_text(encoding="utf-8", errors="ignore")
            except OSError:
                text = ""
            if text:
                if not rewards:
                    for pat in REWARD_PATTERNS:
                        vals = [float(m.group(1)) for m in pat.finditer(text)]
                        if vals:
                            rewards = vals
                            source = "logs/train.log"
                            break
                if not lens:
                    for pat in LEN_PATTERNS:
                        vals = [float(m.group(1)) for m in pat.finditer(text)]
                        if vals:
                            lens = vals
                            break

    rewards = rewards[-RECENT_N:]
    lens = lens[-RECENT_N:]
    return {
        "reward_mean": mean(rewards),
        "len_mean": mean(lens),
        "reward_n": len(rewards),
        "len_n": len(lens),
        "source": source if (rewards or lens) else "无数据",
    }


# ---------------------------------------------------------------------------
# 6) 系统资源：GPU / CPU / 内存 / 磁盘
# ---------------------------------------------------------------------------
def _read_cpu_sample(interval: float = 0.25) -> float:
    """采样 /proc/stat 两次算 CPU 使用率（%）。"""
    def snap() -> Optional[Tuple[int, int]]:
        try:
            with open("/proc/stat", "r", encoding="utf-8") as fh:
                parts = fh.readline().split()
            vals = [int(x) for x in parts[1:8]]  # user nice system idle iowait irq softirq
            idle = vals[3] + vals[4]
            return idle, sum(vals)
        except (OSError, ValueError, IndexError):
            return None

    a = snap()
    if a is None:
        return -1.0
    time.sleep(interval)
    b = snap()
    if b is None:
        return -1.0
    didle = b[0] - a[0]
    dtotal = b[1] - a[1]
    if dtotal <= 0:
        return -1.0
    return max(0.0, min(100.0, 100.0 * (1.0 - didle / dtotal)))


def collect_system(root: Path) -> Dict[str, str]:
    """GPU / CPU / 内存 / 磁盘一览。"""
    info: Dict[str, str] = {}

    # GPU：nvidia-smi（没有就标记不可用）
    gpu = run_cmd([
        "nvidia-smi",
        "--query-gpu=name,utilization.gpu,memory.used,memory.total,temperature.gpu",
        "--format=csv,noheader,nounits",
    ])
    if gpu:
        parts = [x.strip() for x in gpu.splitlines()[0].split(",")]
        if len(parts) >= 5:
            info["gpu"] = (
                f"{parts[0]}｜占用 {parts[1]}%｜显存 {parts[2]}/{parts[3]} MiB｜{parts[4]}°C"
            )
        else:
            info["gpu"] = gpu.replace("\n", " | ")
    else:
        info["gpu"] = "不可用（nvidia-smi 缺失或无 GPU）"

    # CPU：1/5/15 分钟负载 + 瞬时使用率
    try:
        load = Path("/proc/loadavg").read_text(encoding="utf-8").split()[:3]
        load_s = " / ".join(load)
    except OSError:
        load_s = "—"
    usage = _read_cpu_sample()
    usage_s = f"{usage:.0f}%" if usage >= 0 else "—"
    info["cpu"] = f"使用率 {usage_s}｜负载(1/5/15) {load_s}"

    # 内存：MemTotal / MemAvailable
    try:
        mem: Dict[str, int] = {}
        with open("/proc/meminfo", "r", encoding="utf-8") as fh:
            for line in fh:
                k, _, rest = line.partition(":")
                if k in ("MemTotal", "MemAvailable", "SwapTotal", "SwapFree"):
                    mem[k] = int(rest.strip().split()[0])  # kB
        total = mem.get("MemTotal", 0) / 1024 / 1024
        avail = mem.get("MemAvailable", 0) / 1024 / 1024
        used = total - avail
        pct = (used / total * 100) if total else 0.0
        swap_t = mem.get("SwapTotal", 0) / 1024 / 1024
        swap_f = mem.get("SwapFree", 0) / 1024 / 1024
        swap_u = swap_t - swap_f
        info["mem"] = (
            f"已用 {used:.1f}/{total:.1f} GiB（{pct:.0f}%）｜可用 {avail:.1f} GiB"
            f"｜Swap {swap_u:.1f}/{swap_t:.1f} GiB"
        )
    except (OSError, ValueError):
        free = run_cmd(["free", "-h"])
        info["mem"] = free.replace("\n", " | ") if free else "—"

    # 磁盘：仓库所在分区（-h 人类可读）
    df = run_cmd(["df", "-Ph", str(root)])
    lines = df.splitlines()
    if len(lines) >= 2:
        cols = lines[-1].split()
        if len(cols) >= 6:
            info["disk"] = f"{cols[4]} 已用（{cols[2]}/{cols[1]}，剩 {cols[3]}）｜挂载 {cols[5]}"
        elif len(cols) >= 5:
            info["disk"] = f"{cols[4]} 已用（{cols[2]}/{cols[1]}）"
        else:
            info["disk"] = lines[-1]
    else:
        info["disk"] = "—"

    return info


# ---------------------------------------------------------------------------
# 7) 错误计数
# ---------------------------------------------------------------------------
def collect_errors(paths: Dict[str, Path]) -> Dict[str, int]:
    """在 logs/ 下统计 ERROR / NaN / OOM 出现次数（大小写敏感，按关键词）。"""
    counts = {"ERROR": 0, "NaN": 0, "OOM": 0}
    logs_dir = paths["logs"]
    if not logs_dir.is_dir():
        return counts
    for p in logs_dir.rglob("*"):
        if not p.is_file():
            continue
        if p.suffix.lower() in (".zip", ".png", ".jpg", ".mp4", ".pyc"):
            continue
        if p.name == STATUS_MD_NAME:      # 不把面板自身算进错误
            continue
        try:
            text = p.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        for kw in counts:
            counts[kw] += text.count(kw)
    return counts


# ---------------------------------------------------------------------------
# 8) 时间轴：启动至今 / 14h 剩余 / ETA
# ---------------------------------------------------------------------------
def collect_timing(
    paths: Dict[str, Path],
    proc: Dict[str, Any],
    ckpt: Dict[str, Any],
    events: Dict[str, Any],
    total_steps: Optional[int],
) -> Dict[str, Any]:
    """推断训练起点、已运行时长、14h 窗口剩余与完成 ETA。"""
    now = time.time()
    starts: List[float] = []

    # 进程启动时间（ps lstart 换算失败则用 etime 近似）
    if proc["alive"]:
        for pr in proc["procs"]:
            raw = run_cmd(["ps", "-o", "lstart=", "-p", str(pr["pid"])])
            if raw:
                try:
                    st = datetime.strptime(raw.strip(), "%a %b %d %H:%M:%S %Y")
                    starts.append(st.timestamp())
                except ValueError:
                    pass
            if starts:
                break

    # 产物时间：最早 checkpoint / 最早 event
    for item in ckpt.get("all", []):
        starts.append(item["mtime"])
    for f in events.get("files", []):
        starts.append(f["mtime"])
    train_log = paths["logs"] / "train.log"
    if train_log.exists():
        try:
            starts.append(train_log.stat().st_mtime)
        except OSError:
            pass

    started = bool(starts)
    start_ts = min(starts) if starts else None
    elapsed = (now - start_ts) if start_ts else 0.0
    budget = BUDGET_HOURS * 3600.0
    remain = max(0.0, budget - elapsed) if started else budget

    # 当前步数与推进速率：事件轨迹 + checkpoint 文件名步数
    samples: List[Tuple[float, float]] = []  # (时刻, 步数)
    for wall, step in events.get("traj", []):
        samples.append((wall, float(step)))
    for item in ckpt.get("all", []):
        if item.get("step") is not None:
            samples.append((item["mtime"], float(item["step"])))
    samples.sort(key=lambda s: s[0])

    cur_step = int(samples[-1][1]) if samples else None
    rate = None  # steps/s
    if len(samples) >= 2:
        t0, s0 = samples[0]
        t1, s1 = samples[-1]
        dt = t1 - t0
        if dt > 1.0 and s1 > s0:
            rate = (s1 - s0) / dt

    eta_s = None
    if rate and total_steps and cur_step is not None:
        remain_steps = max(0, total_steps - cur_step)
        eta_s = remain_steps / rate

    return {
        "started": started,
        "start_ts": start_ts,
        "elapsed": elapsed,
        "remain_window": remain,
        "cur_step": cur_step,
        "total_steps": total_steps,
        "rate": rate,
        "eta_s": eta_s,
    }


def read_total_steps(paths: Dict[str, Path]) -> Optional[int]:
    """从 config.py 读 DEFAULT_TOTAL_STEPS。"""
    cfg = paths["config"]
    if not cfg.exists():
        return None
    try:
        text = cfg.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return None
    m = TOTAL_STEPS_RE.search(text)
    if not m:
        return None
    try:
        return int(m.group(1).replace("_", ""))
    except ValueError:
        return None


# ---------------------------------------------------------------------------
# 报告渲染（中文表格）
# ---------------------------------------------------------------------------
def render_report(
    paths: Dict[str, Path],
    stage: Dict[str, str],
    proc: Dict[str, Any],
    ckpt: Dict[str, Any],
    events: Dict[str, Any],
    metrics: Dict[str, Any],
    system: Dict[str, str],
    errors: Dict[str, int],
    timing: Dict[str, Any],
) -> str:
    now = datetime.now()
    lines: List[str] = []
    A = lines.append

    A("# 训练监控面板 — 状态快照")
    A("")
    A(f"> 生成时间：{now.strftime('%Y-%m-%d %H:%M:%S')}　|　"
      f"工具：`webots-sim/rl/monitor.py`　|　时间窗口：{BUDGET_HOURS} 小时")
    A("")

    # ---------- 一、总体状态 ----------
    A("## 一、总体状态")
    A("")
    A("| 项 | 值 |")
    A("|---|---|")
    A(f"| 当前阶段 | **{stage['code']} {stage['name']}**（来源：{stage['source']}） |")

    if proc["alive"]:
        first = proc["procs"][0]
        extra = f" 等 {len(proc['procs'])} 个进程" if len(proc["procs"]) > 1 else ""
        A(f"| 训练进程 | ✅ PID {first['pid']}（运行 {first['etime']}，CPU {first['pcpu']}%）{extra} |")
    else:
        A("| 训练进程 | ⏸ 未运行（`pgrep train_ppo` 无匹配） |")

    if timing["started"]:
        A(f"| 启动至今 | {fmt_dur(timing['elapsed'])}（起点 {fmt_ts(timing['start_ts'])}） |")
        A(f"| {BUDGET_HOURS}h 窗口剩余 | {fmt_dur(timing['remain_window'])} |")
    else:
        A("| 启动至今 | —— **等待启动** |")
        A(f"| {BUDGET_HOURS}h 窗口剩余 | {fmt_dur(timing['remain_window'])}（尚未起算） |")

    if timing["cur_step"] is not None and timing["total_steps"]:
        pct = timing["cur_step"] / timing["total_steps"] * 100
        A(f"| 训练步数 | {timing['cur_step']:,} / {timing['total_steps']:,}（{pct:.1f}%） |")
    elif timing["cur_step"] is not None:
        A(f"| 训练步数 | {timing['cur_step']:,}（目标步数未知） |")
    else:
        A("| 训练步数 | 0（尚无步数记录） |")

    if timing["rate"]:
        A(f"| 步速 | {timing['rate']:.2f} steps/s |")
    else:
        A("| 步速 | ——（样本不足，无法估算） |")

    if timing["eta_s"] is not None:
        eta_abs = datetime.now() + timedelta(seconds=timing["eta_s"])
        # 若 14h 窗口先到，提示可能训不完
        warn = ""
        if timing["eta_s"] > timing["remain_window"]:
            warn = " ⚠️ 超出剩余窗口，可能训不完"
        A(f"| ETA | 约 {fmt_dur(timing['eta_s'])}（预计 {eta_abs.strftime('%m-%d %H:%M')} 完成）{warn} |")
    else:
        A("| ETA | ——（无推进速率） |")

    # 状态结论
    if not timing["started"] and not proc["alive"]:
        verdict = "🟡 **等待启动**——训练未开始，先跑冒烟再放大步数"
    elif proc["alive"] and sum(errors.values()) == 0:
        verdict = "🟢 **正常训练中**"
    elif proc["alive"] and sum(errors.values()) > 0:
        verdict = f"🟠 **训练中但日志有错误**（共 {sum(errors.values())} 处，见第四节）"
    elif timing["started"] and not proc["alive"]:
        verdict = "🔴 **训练已停止**——有历史产物但进程不在，需确认是正常结束还是中断"
    else:
        verdict = "⚪ **状态未知**"
    A(f"| 状态结论 | {verdict} |")
    A("")

    # ---------- 二、训练进度 ----------
    A("## 二、训练进度")
    A("")
    A("| 项 | 值 |")
    A("|---|---|")
    lat = ckpt.get("latest")
    if lat:
        age = time.time() - lat["mtime"]
        step_s = f"，步数 {lat['step']:,}" if lat["step"] is not None else ""
        A(f"| 最新 checkpoint | `{lat['name']}`（{fmt_ts(lat['mtime'])}，{fmt_bytes(lat['size'])}，"
          f"{fmt_dur(age)}前{step_s}） |")
        A(f"| checkpoint 总数 | {ckpt['count']} 个 |")
    else:
        A("| 最新 checkpoint | ——（`checkpoints/*.zip` 为空） |")
        A("| checkpoint 总数 | 0 |")

    if events["files"]:
        # 只列最新一个，避免表格过长；--watch 时给出增量
        f = max(events["files"], key=lambda d: d["mtime"])
        delta_s = ""
        if f["delta"] is not None:
            sign = "+" if f["delta"] >= 0 else ""
            delta_s = f"，本周期 {sign}{fmt_bytes(f['delta'])}"
        A(f"| TensorBoard 事件 | `{f['name']}`（{fmt_bytes(f['size'])}，"
          f"{fmt_ts(f['mtime'])}{delta_s}） |")
        A(f"| 事件文件总数 | {len(events['files'])} 个 |")
    else:
        A("| TensorBoard 事件 | ——（`runs/*/events*` 为空） |")
        A("| 事件文件总数 | 0 |")

    if metrics["reward_mean"] is not None:
        A(f"| 近 {RECENT_N} 集奖励均值 | {metrics['reward_mean']:.3f}（{metrics['reward_n']} 个样本） |")
    else:
        A(f"| 近 {RECENT_N} 集奖励均值 | —— 无数据 |")
    if metrics["len_mean"] is not None:
        A(f"| 存活步数均值 | {metrics['len_mean']:.1f}（{metrics['len_n']} 个样本） |")
    else:
        A("| 存活步数均值 | —— 无数据 |")
    A(f"| 指标来源 | {metrics['source']} |")
    A("")

    # ---------- 三、系统资源 ----------
    A("## 三、系统资源")
    A("")
    A("| 资源 | 状态 |")
    A("|---|---|")
    A(f"| GPU | {system['gpu']} |")
    A(f"| CPU | {system['cpu']} |")
    A(f"| 内存 | {system['mem']} |")
    A(f"| 磁盘 | {system['disk']} |")
    A("")

    # ---------- 四、错误与告警 ----------
    A("## 四、错误与告警")
    A("")
    A("| 关键词 | 出现次数 |")
    A("|---|---|")
    for kw, cnt in errors.items():
        flag = "✅" if cnt == 0 else "❗"
        A(f"| {kw} | {flag} {cnt} |")
    A(f"| **合计** | **{sum(errors.values())}** |")
    A("")
    A("> 扫描范围：`logs/` 下全部文本文件（不含 `STATUS.md` 自身）。")
    A("")

    # ---------- 五、提示 ----------
    A("## 五、提示")
    A("")
    if not timing["started"]:
        A("- 训练尚未开始。就绪后启动：`python3 webots-sim/rl/train_ppo.py --total-steps 2048 --device cpu --eval-interval 0`（冒烟 P0）。")
        A("- 依赖未装齐时先确认：`python3 -c 'import torch, stable_baselines3'`，以及 `webots-sim/rl/walk_env.py` 是否存在。")
    elif proc["alive"]:
        A("- 看曲线：`tensorboard --logdir runs`（关注 `rollout/ep_remean` 上升、`rollout/ep_len_mean` 变长）。")
        A("- 阶段推进参考：P0 冒烟 → P1 站立 → P2 行走 → P3 转向 → P4 台阶。")
    else:
        A("- 进程已退出。若要续训：`python3 webots-sim/rl/train_ppo.py --resume checkpoints/<最新>.zip`。")
    if sum(errors.values()) > 0:
        A(f"- ⚠️ 日志中发现 {sum(errors.values())} 处 ERROR/NaN/OOM，建议先排查再继续。")
    A(f"- 刷新本面板：`python3 webots-sim/rl/monitor.py`；持续监控：`python3 webots-sim/rl/monitor.py --watch`。")
    A("")
    A("---")
    A("")
    A(f"*本文件由 `monitor.py` 自动生成于 {now.strftime('%Y-%m-%d %H:%M:%S')}，请勿手工编辑。*")
    A("")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def collect_all(prev_sizes: Optional[Dict[str, int]] = None) -> Tuple[str, Dict[str, Any]]:
    """收集全部指标并渲染报告，返回 (报告文本, 中间状态)。"""
    paths = find_paths()
    stage = collect_stage(paths)
    proc = collect_process()
    ckpt = collect_checkpoints(paths)
    events = collect_events(paths, prev_sizes)
    metrics = collect_train_metrics(paths, events["files"])
    system = collect_system(paths["root"])
    errors = collect_errors(paths)
    total_steps = read_total_steps(paths)
    timing = collect_timing(paths, proc, ckpt, events, total_steps)
    report = render_report(paths, stage, proc, ckpt, events, metrics, system, errors, timing)
    state = {"prev_sizes": events.get("prev_sizes", {}), "paths": paths}
    return report, state


def write_status(paths: Dict[str, Path], text: str) -> Path:
    """写 logs/STATUS.md。"""
    logs_dir = paths["logs"]
    logs_dir.mkdir(parents=True, exist_ok=True)
    out = logs_dir / STATUS_MD_NAME
    out.write_text(text, encoding="utf-8")
    return out


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="RL 训练监控面板（纯标准库，打印状态并写 logs/STATUS.md）"
    )
    parser.add_argument(
        "--watch", action="store_true",
        help=f"每 {WATCH_INTERVAL} 秒刷新一次（Ctrl-C 退出）",
    )
    args = parser.parse_args(argv)

    prev_sizes: Optional[Dict[str, int]] = None
    try:
        while True:
            report, state = collect_all(prev_sizes)
            # 清屏仅在 watch 模式，避免污染单次输出
            if args.watch:
                sys.stdout.write("\033[2J\033[H")
            print(report)
            out = write_status(state["paths"], report)
            if args.watch:
                print(f"[monitor] 已写入 {out}，{WATCH_INTERVAL} 秒后刷新（Ctrl-C 退出）…")
                sys.stdout.flush()   # 管道/重定向时避免块缓冲导致看不到输出
                prev_sizes = state.get("prev_sizes")
                time.sleep(WATCH_INTERVAL)
            else:
                print(f"[monitor] 已写入 {out}", file=sys.stderr)
                break
    except KeyboardInterrupt:
        print("\n[monitor] 已停止监控", file=sys.stderr)
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
