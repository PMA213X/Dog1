#!/usr/bin/env bash
# P1 训练看门狗：每 5 分钟巡检，挂了从最近 checkpoint 续训；
# P1 完成（200k）且存活率达标后，自动启动 P2 行走（400k 步）。
# 用法: nohup bash webots-sim/rl/watchdog_p1.sh > logs/watchdog.log 2>&1 &
set -u
cd /run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1

TOTAL_P1=200000
TOTAL_P2=400000
TAG_P1=phase1_stand
TAG_P2=phase2_walk
CKPT_DIR=checkpoints
LOG=logs/train.log
PID_FILE=logs/train.pid
DONE_MARK=logs/.p1_done
INTERVAL=300   # 5 分钟

log() { echo "[$(date '+%F %T')] $*"; }

latest_ckpt() {
  # 取 phase1_stand_*_steps.zip 中步数最大的；没有则取 phase1_stand_final.zip
  ls -1t "$CKPT_DIR"/${TAG_P1}_[0-9]*_steps.zip 2>/dev/null | head -1
}

current_steps() {
  # 从 train.log 抓最后一次 total_timesteps
  grep -oE 'total_timesteps[[:space:]]*\|[[:space:]]*[0-9]+' "$LOG" 2>/dev/null \
    | tail -1 | grep -oE '[0-9]+$' || echo 0
}

start_p1_resume() {
  local ckpt="$1"
  # 崩溃现场留档，便于事后排查
  if [ -f "$LOG" ]; then
    cp "$LOG" "logs/train_crash_$(date +%H%M%S).log" 2>/dev/null || true
  fi
  if [ -n "$ckpt" ] && [ -f "$ckpt" ]; then
    log "从 checkpoint 续训 P1: $ckpt"
    nohup python3 webots-sim/rl/train_ppo.py \
      --total-steps "$TOTAL_P1" --tag "$TAG_P1" --resume "$ckpt" \
      > "$LOG" 2>&1 &
  else
    log "无可用 checkpoint，从头启动 P1"
    nohup python3 webots-sim/rl/train_ppo.py \
      --total-steps "$TOTAL_P1" --tag "$TAG_P1" > "$LOG" 2>&1 &
  fi
  echo $! > "$PID_FILE"
  log "P1 已启动 PID=$(cat "$PID_FILE")"
}

start_p2() {
  log "启动 P2 行走（${TOTAL_P2} 步，奖励含 3*vx）"
  nohup python3 webots-sim/rl/train_ppo.py \
    --total-steps "$TOTAL_P2" --tag "$TAG_P2" \
    > logs/train_p2.log 2>&1 &
  echo $! > logs/train_p2.pid
  log "P2 已启动 PID=$(cat logs/train_p2.pid)"
  # P2 的看门狗由后续人工/巡检接管
}

# ---------- 主循环 ----------
log "看门狗启动：监控 P1（目标 ${TOTAL_P1} 步）"
while true; do
  # 已完成 P1 则退出
  if [ -f "$DONE_MARK" ]; then
    log "P1 已完成标记存在，看门狗退出"
    exit 0
  fi

  PID=$(cat "$PID_FILE" 2>/dev/null || echo "")
  STEPS=$(current_steps)
  if [ -n "$PID" ] && ps -p "$PID" > /dev/null 2>&1; then
    log "训练存活 PID=$PID 已跑 ${STEPS}/${TOTAL_P1} 步"
  else
    log "训练进程不在！已跑约 ${STEPS} 步"
    if [ "${STEPS:-0}" -ge "$TOTAL_P1" ] 2>/dev/null; then
      log "P1 步数达标，视为完成"
      touch "$DONE_MARK"
      start_p2
      exit 0
    fi
    # 日志显示正常完成（累计步数可能略超 n_steps 整数倍）
    if grep -q '最终模型已保存' "$LOG" 2>/dev/null; then
      log "P1 训练完成并已保存最终模型"
      touch "$DONE_MARK"
      start_p2
      exit 0
    fi
    CKPT=$(latest_ckpt)
    start_p1_resume "$CKPT"
  fi

  sleep "$INTERVAL"
done
