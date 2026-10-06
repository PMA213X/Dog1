#!/usr/bin/env bash
# yobogo_loco_jump_v1 人工巡检入口。
# 默认只显示状态；只有显式 start 才允许启动监控，进而按命令清单启动训练。
set -u

RL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$RL_DIR/../.." && pwd)"
RUN_ID="${LOCO_JUMP_RUN_ID:-yobogo_loco_jump_v1}"
RUN_ROOT="${LOCO_JUMP_RUN_ROOT:-$PROJECT_ROOT/logs/$RUN_ID}"
MONITOR="$RL_DIR/loco_jump_monitor.py"
PYTHON_BIN="${PYTHON_BIN:-python3}"
MONITOR_LOG="$RUN_ROOT/monitor.log"
MONITOR_PID_FILE="$RUN_ROOT/pids/monitor.pid"
WATCHDOG_PID_FILE="$RUN_ROOT/pids/watchdog.pid"
TRAINING_PID_FILE="$RUN_ROOT/pids/training.pid"
COMMAND_FILE="$RUN_ROOT/train_command.json"
STATUS_FILE="$RUN_ROOT/status.json"

log() {
    printf '[%s] %s\n' "$(date '+%F %T')" "$*"
}

read_pid() {
    local path="$1"
    [ -f "$path" ] || return 1
    local pid
    pid="$(tr -cd '0-9' < "$path" 2>/dev/null || true)"
    [ -n "$pid" ] || return 1
    printf '%s' "$pid"
}

pid_alive() {
    local pid="$1"
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

usage() {
    cat <<'EOF'
用法：
  loco_jump_watchdog.sh                 # 默认只读状态，不启动训练
  loco_jump_watchdog.sh status          # 读取 status.json
  loco_jump_watchdog.sh once            # 单次只读巡检
  loco_jump_watchdog.sh dry-run         # 打印路径、阶段和阈值计划
  loco_jump_watchdog.sh start           # 显式启动监控（命令清单存在时才可训练）
  loco_jump_watchdog.sh stop            # 停止监控及其管理的训练进程

PID 约定：
  pids/watchdog.pid、pids/monitor.pid   # 同一后台监控进程
  pids/training.pid                     # 当前训练子进程
  pids/tensorboard.pid                  # TensorBoard 固定 127.0.0.1:6006
EOF
}

print_status() {
    if [ -f "$STATUS_FILE" ]; then
        "$PYTHON_BIN" "$MONITOR" \
            --project-root "$PROJECT_ROOT" \
            --run-root "$RUN_ROOT" \
            --run-id "$RUN_ID" \
            --status
    else
        log "尚无状态文件：$STATUS_FILE"
        return 3
    fi
}

start_monitor() {
    if [ ! -f "$COMMAND_FILE" ]; then
        log "缺少命令清单，拒绝启动：$COMMAND_FILE"
        log "请先显式执行 start_loco_jump_training.sh --start-training"
        return 4
    fi
    local monitor_pid=""
    monitor_pid="$(read_pid "$MONITOR_PID_FILE" 2>/dev/null || true)"
    if pid_alive "$monitor_pid"; then
        log "监控已在运行 PID=$monitor_pid，拒绝重复启动"
        return 5
    fi
    mkdir -p "$RUN_ROOT/pids"
    (
        exec "$PYTHON_BIN" "$MONITOR" \
            --run \
            --project-root "$PROJECT_ROOT" \
            --run-root "$RUN_ROOT" \
            --run-id "$RUN_ID" \
            >> "$MONITOR_LOG" 2>&1
    ) &
    local pid=$!
    printf '%s\n' "$pid" > "$WATCHDOG_PID_FILE"
    sleep 1
    if ! pid_alive "$pid"; then
        log "监控启动后立即退出，请查看 $MONITOR_LOG"
        return 6
    fi
    log "监控已启动 PID=$pid"
    log "监控日志：$MONITOR_LOG"
    log "状态文件：$STATUS_FILE"
}

stop_monitor() {
    local pid=""
    pid="$(read_pid "$MONITOR_PID_FILE" 2>/dev/null || true)"
    [ -n "$pid" ] || pid="$(read_pid "$WATCHDOG_PID_FILE" 2>/dev/null || true)"
    if ! pid_alive "$pid"; then
        log "监控未运行"
        return 0
    fi
    log "发送 SIGTERM 给监控 PID=$pid"
    kill -TERM "$pid" 2>/dev/null || true
    local deadline=$((SECONDS + 8))
    while pid_alive "$pid" && [ "$SECONDS" -lt "$deadline" ]; do
        sleep 0.2
    done
    if pid_alive "$pid"; then
        log "监控未在 8 秒内退出，拒绝强制清理，请人工检查"
        return 7
    fi
    rm -f "$MONITOR_PID_FILE" "$WATCHDOG_PID_FILE"
    log "监控已停止"
}

case "${1:-status}" in
    status)
        print_status
        ;;
    once)
        exec "$PYTHON_BIN" "$MONITOR" \
            --once \
            --project-root "$PROJECT_ROOT" \
            --run-root "$RUN_ROOT" \
            --run-id "$RUN_ID"
        ;;
    dry-run|--dry-run)
        exec "$PYTHON_BIN" "$MONITOR" \
            --dry-run \
            --project-root "$PROJECT_ROOT" \
            --run-root "$RUN_ROOT" \
            --run-id "$RUN_ID"
        ;;
    start)
        start_monitor
        ;;
    stop)
        stop_monitor
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac
