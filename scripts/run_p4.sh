#!/usr/bin/env bash
# =============================================================================
# run_p4.sh —— P4 台阶训练启动器（等 P3 结束后自动热启动）
#
# 流程：
#   1. 轮询等待 P3（train_ppo.py --tag phase3_turn）结束，期间不碰 P3
#   2. 找 P3 最新 checkpoint（checkpoints/p3_turn_*.zip）
#   3. 用 RL_BRIDGE_PORT=11453 热启动 P4（tag=phase4_stairs，51 维台阶环境）
#
# 用法（后台启动等待器）：
#   nohup bash scripts/run_p4.sh > logs/p4_waiter.log 2>&1 &
# =============================================================================
set -u

# 项目根目录（脚本在 scripts/ 下）
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT" || exit 1

PY=/tmp/rl_venv/bin/python
LOG_DIR=logs
CKPT_DIR=checkpoints
P3_PATTERN='tag phase3_turn'          # P3 进程命令行特征
P4_LOG="$LOG_DIR/train_p4.log"
P4_PID_FILE="$LOG_DIR/train_p4.pid"
BRIDGE_PORT=11453                     # P4 专用 TCP 桥端口（P3 用 11452）
TOTAL_STEPS=200000
CKPT_INTERVAL=20000
CKPT_PREFIX=p4_stairs
TAG=phase4_stairs

mkdir -p "$LOG_DIR" "$CKPT_DIR"

# --- 防重复启动：P4 若已在跑就直接退出 -------------------------------------
if [[ -f "$P4_PID_FILE" ]] && kill -0 "$(cat "$P4_PID_FILE")" 2>/dev/null; then
    echo "【P4】训练进程已在运行（PID $(cat "$P4_PID_FILE")），退出"
    exit 0
fi

# --- 1. 等 P3 结束（轮询 pgrep，每 60 秒查一次）-----------------------------
echo "【P4】$(date '+%F %T') 等待 P3 结束（匹配模式：$P3_PATTERN）..."
while pgrep -f "$P3_PATTERN" >/dev/null 2>&1; do
    # 每 10 分钟打一行心跳，便于观察
    if (( $(date +%s) % 600 < 60 )); then
        echo "【P4】$(date '+%F %T') P3 仍在运行，继续等待 ..."
    fi
    sleep 60
done
echo "【P4】$(date '+%F %T') P3 已结束，准备热启动"

# --- 2. 找 P3 最优 checkpoint（按修改时间取最新）----------------------------
CKPT="$(ls -t "$CKPT_DIR"/p3_turn_*.zip 2>/dev/null | head -1 || true)"
if [[ -z "$CKPT" || ! -f "$CKPT" ]]; then
    echo "【P4】错误：找不到 P3 checkpoint（$CKPT_DIR/p3_turn_*.zip），退出"
    exit 1
fi
echo "【P4】热启动 checkpoint：$CKPT"

# --- 3. 启动 P4 训练 -------------------------------------------------------
echo "【P4】$(date '+%F %T') 启动训练：port=$BRIDGE_PORT steps=$TOTAL_STEPS tag=$TAG"
RL_BRIDGE_PORT="$BRIDGE_PORT" PYTHONUNBUFFERED=1 nohup "$PY" \
    webots-sim/rl/train_ppo.py \
    --total-steps "$TOTAL_STEPS" \
    --tag "$TAG" \
    --resume "$CKPT" \
    --checkpoint-interval "$CKPT_INTERVAL" \
    --ckpt-prefix "$CKPT_PREFIX" \
    --eval-interval "$CKPT_INTERVAL" \
    --eval-episodes 3 \
    > "$P4_LOG" 2>&1 &

P4_PID=$!
echo "$P4_PID" > "$P4_PID_FILE"
echo "【P4】已启动 PID=$P4_PID，日志 $P4_LOG，等待器退出"
