#!/usr/bin/env bash
# yobogo_loco_jump_v1 正式训练启动入口。
# 默认与 --dry-run 均只打印计划；只有显式 --start-training 才会写命令清单并启动。
set -euo pipefail

RL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$RL_DIR/../.." && pwd)"
RUN_ID="yobogo_loco_jump_v1"
RUN_ROOT="$PROJECT_ROOT/logs/$RUN_ID"
CHECKPOINT_DIR="$PROJECT_ROOT/checkpoints/$RUN_ID"
RUNS_DIR="$PROJECT_ROOT/runs/$RUN_ID"
COMMAND_FILE="$RUN_ROOT/train_command.json"
WATCHDOG="$RL_DIR/loco_jump_watchdog.sh"
MONITOR="$RL_DIR/loco_jump_monitor.py"
TRAIN_SCRIPT="$RL_DIR/train_loco_jump.py"
PYTHON_BIN="${PYTHON_BIN:-python3}"
TENSORBOARD_BIN="${TENSORBOARD_BIN:-tensorboard}"
TENSORBOARD_HOST="127.0.0.1"
TENSORBOARD_PORT="6006"
TENSORBOARD_PID_FILE="$RUN_ROOT/pids/tensorboard.pid"
TENSORBOARD_LOG="$RUN_ROOT/tensorboard.log"

usage() {
    cat <<'EOF'
用法：
  start_loco_jump_training.sh                 # 默认 dry-run，绝不启动正式训练
  start_loco_jump_training.sh --dry-run       # 显式 dry-run，绝不启动正式训练
  start_loco_jump_training.sh --start-training
                                               # 写阶段命令清单，启动固定
                                               # TensorBoard 127.0.0.1:6006
                                               # 和本地监控；监控再启动 S0

安全约束：
  正式训练必须显式传 --start-training；当前测试只允许 dry-run。
EOF
}

print_dry_run() {
    "$PYTHON_BIN" "$MONITOR" \
        --dry-run \
        --project-root "$PROJECT_ROOT" \
        --run-root "$RUN_ROOT" \
        --run-id "$RUN_ID"
    printf '  启动脚本模式     : dry-run\n'
    printf '  训练入口         : %s\n' "$TRAIN_SCRIPT"
    printf '  checkpoint间隔   : 50000 步\n'
    print_stage_commands
    printf '  TensorBoard 命令 : %q --host %s --port %s --logdir %q\n' \
        "$TENSORBOARD_BIN" "$TENSORBOARD_HOST" "$TENSORBOARD_PORT" "$RUNS_DIR"
    printf '  正式训练         : 未启动\n'
}

build_command_manifest() {
    "$PYTHON_BIN" - "$1" "$COMMAND_FILE" "$RUN_ROOT" "$PROJECT_ROOT" \
        "$RUN_ID" "$CHECKPOINT_DIR" "$RUNS_DIR" "$PYTHON_BIN" "$TRAIN_SCRIPT" <<'PY'
import json
import os
import sys
from pathlib import Path

output_mode = sys.argv[1]
command_file = Path(sys.argv[2])
run_root = Path(sys.argv[3])
project_root = Path(sys.argv[4])
run_id = sys.argv[5]
checkpoint_dir = Path(sys.argv[6])
runs_dir = Path(sys.argv[7])
python_bin = sys.argv[8]
train_script = Path(sys.argv[9])

# 阶段目标与赛题契约保持累计步数一致，tag 使用现有训练入口别名。
stages = [
    ("S0_smoke", "S0 冒烟", 5000, "phase0"),
    ("S1_stand", "S1 站立", 200000, "phase1"),
    ("S2_command", "S2 命令", 500000, "phase2"),
    ("S3_jump", "S3 跳跃", 800000, "phase3"),
    ("S4_mobile_terrain", "S4 移动越障", 1200000, "phase4"),
]

command_stages = []
for stage_id, full_name, target, tag in stages:
    argv = [
        python_bin,
        str(train_script),
        "--total-steps", str(target),
        "--tag", tag,
        "--ckptdir", str(checkpoint_dir),
        "--ckpt-prefix", run_id,
        "--logdir", str(runs_dir / stage_id),
        "--checkpoint-interval", "50000",
        "--eval-interval", "20000",
        "--eval-episodes", "10",
        "--device", "cuda",
    ]
    command_stages.append({
        "id": stage_id,
        "full_name": full_name,
        "cumulative_target_steps": target,
        "tag": tag,
        "log_tag": stage_id,
        "checkpoint_prefix": run_id,
        "logdir": str(runs_dir / stage_id),
        "argv": argv,
    })

data = {
    "schema_version": 1,
    "run_id": run_id,
    "project_root": str(project_root),
    "run_root": str(run_root),
    "checkpoint_dir": str(checkpoint_dir),
    "checkpoint_prefix": run_id,
    "runs_dir": str(runs_dir),
    "tensorboard_url": "http://127.0.0.1:6006/",
    "stages": command_stages,
    "completion_markers": [
        "最终模型已保存",
        "训练完成",
        "TRAINING_COMPLETED",
        "RECOVERY_COMPLETED",
    ],
    "resume_flag": "--resume",
    "resume_value": "{checkpoint}",
    "generated_at": __import__("datetime").datetime.now().astimezone().isoformat(
        timespec="seconds"
    ),
}
payload = json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
if output_mode == "-":
    print(payload, end="")
else:
    tmp = command_file.with_name(f".{command_file.name}.{os.getpid()}.tmp")
    tmp.write_text(payload, encoding="utf-8")
    os.replace(tmp, command_file)
PY
}

write_command_manifest() {
    mkdir -p "$RUN_ROOT/pids" "$CHECKPOINT_DIR" "$RUNS_DIR"
    build_command_manifest "$COMMAND_FILE"
    printf '阶段命令清单已写入：%s\n' "$COMMAND_FILE"
}

print_stage_commands() {
    printf '  完整阶段命令     :\n'
    build_command_manifest - | "$PYTHON_BIN" -c '
import json
import shlex
import sys

data = json.load(sys.stdin)
for stage in data["stages"]:
    print("    [%s] %s 累计目标=%s tag=%s log_tag=%s" % (
        stage["id"],
        stage["full_name"],
        stage["cumulative_target_steps"],
        stage["tag"],
        stage["log_tag"],
    ))
    print("      " + shlex.join(stage["argv"]))
'
}

read_pid_file() {
    local path="$1"
    [ -f "$path" ] || return 1
    local pid
    pid="$(tr -cd '0-9' < "$path" 2>/dev/null || true)"
    [ -n "$pid" ] || return 1
    printf '%s' "$pid"
}

pid_alive() {
    local pid="${1:-}"
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

port_is_open() {
    "$PYTHON_BIN" - "$TENSORBOARD_HOST" "$TENSORBOARD_PORT" <<'PY'
import socket
import sys

host, port = sys.argv[1], int(sys.argv[2])
try:
    with socket.create_connection((host, port), timeout=0.5):
        raise SystemExit(0)
except OSError:
    raise SystemExit(1)
PY
}

start_tensorboard() {
    local existing=""
    existing="$(read_pid_file "$TENSORBOARD_PID_FILE" 2>/dev/null || true)"
    if pid_alive "$existing"; then
        # 已有 PID 时只等待其绑定固定端口，不重复拉起第二个 TensorBoard。
        local wait_count=0
        while ! port_is_open && [ "$wait_count" -lt 10 ]; do
            sleep 0.5
            wait_count=$((wait_count + 1))
        done
        if ! port_is_open; then
            printf '错误：TensorBoard PID=%s 未监听 %s:%s，拒绝重复启动\n' \
                "$existing" "$TENSORBOARD_HOST" "$TENSORBOARD_PORT" >&2
            return 13
        fi
        printf 'TensorBoard 已运行 PID=%s：%s:%s\n' \
            "$existing" "$TENSORBOARD_HOST" "$TENSORBOARD_PORT"
        return 0
    fi
    if port_is_open; then
        printf '错误：%s:%s 已被其他进程占用，拒绝重复启动\n' \
            "$TENSORBOARD_HOST" "$TENSORBOARD_PORT" >&2
        return 10
    fi
    if ! command -v "$TENSORBOARD_BIN" >/dev/null 2>&1; then
        printf '错误：找不到 tensorboard 可执行文件：%s\n' "$TENSORBOARD_BIN" >&2
        return 11
    fi
    mkdir -p "$RUN_ROOT/pids"
    nohup "$TENSORBOARD_BIN" \
        --host "$TENSORBOARD_HOST" \
        --port "$TENSORBOARD_PORT" \
        --logdir "$RUNS_DIR" \
        >> "$TENSORBOARD_LOG" 2>&1 &
    local pid=$!
    printf '%s\n' "$pid" > "$TENSORBOARD_PID_FILE"
    sleep 1
    if ! pid_alive "$pid"; then
        printf '错误：TensorBoard 启动失败，请查看 %s\n' "$TENSORBOARD_LOG" >&2
        return 12
    fi
    printf 'TensorBoard 已启动 PID=%s：%s:%s\n' \
        "$pid" "$TENSORBOARD_HOST" "$TENSORBOARD_PORT"
    printf 'TensorBoard 日志：%s\n' "$TENSORBOARD_LOG"
}

start_formal_training() {
    command -v "$PYTHON_BIN" >/dev/null 2>&1 || {
        printf '错误：找不到 Python：%s\n' "$PYTHON_BIN" >&2
        return 20
    }
    [ -f "$TRAIN_SCRIPT" ] || {
        printf '错误：找不到训练入口：%s\n' "$TRAIN_SCRIPT" >&2
        return 21
    }
    [ -f "$MONITOR" ] || {
        printf '错误：找不到监控程序：%s\n' "$MONITOR" >&2
        return 22
    }

    local monitor_pid=""
    monitor_pid="$(read_pid_file "$RUN_ROOT/pids/monitor.pid" 2>/dev/null || true)"
    if pid_alive "$monitor_pid"; then
        printf '错误：监控已在运行 PID=%s，拒绝重复启动正式训练\n' "$monitor_pid" >&2
        return 30
    fi

    write_command_manifest
    local tensorboard_pid_before=""
    local tensorboard_started_here=0
    tensorboard_pid_before="$(read_pid_file "$TENSORBOARD_PID_FILE" 2>/dev/null || true)"
    if ! start_tensorboard; then
        return $?
    fi
    if ! pid_alive "$tensorboard_pid_before"; then
        tensorboard_started_here=1
    else
        tensorboard_started_here=0
    fi
    if ! "$WATCHDOG" start; then
        if [ "$tensorboard_started_here" -eq 1 ]; then
            local tb_pid=""
            tb_pid="$(read_pid_file "$TENSORBOARD_PID_FILE" 2>/dev/null || true)"
            if pid_alive "$tb_pid"; then
                kill -TERM "$tb_pid" 2>/dev/null || true
            fi
        fi
        return 31
    fi
    printf '正式训练监控已显式启动。\n'
    printf '状态：%s\n' "$RUN_ROOT/status.json"
    printf '监控日志：%s\n' "$RUN_ROOT/monitor.log"
    printf '训练日志：%s\n' "$RUN_ROOT/train.log"
    printf 'TensorBoard：http://%s:%s/\n' "$TENSORBOARD_HOST" "$TENSORBOARD_PORT"
}

case "${1:-}" in
    ""|--dry-run)
        print_dry_run
        ;;
    --start-training)
        start_formal_training
        ;;
    -h|--help)
        usage
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac
