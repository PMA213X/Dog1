#!/usr/bin/env bash
# Isaac Sim 4.5.0.0 后台安装脚本；只负责记录状态并执行 pip 安装。
set -u
OUT='/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/mini-cheetah-isaaclab/outputs/environment-install'
VENV='/home/pma213x/.venvs/minicheetah-isaaclab'
STATUS="$OUT/isaacsim-install.status"
PIDFILE="$OUT/isaacsim-install.pid"
LOG="$OUT/isaacsim-install.log"
START_AT="$(date --iso-8601=seconds)"

write_status() {
    local state="$1" exit_code="$2" end_at="$3"
    local tmp="${STATUS}.tmp.$$"
    {
        printf 'state=%s\n' "$state"
        printf 'pid=%s\n' "$$"
        printf 'started_at=%s\n' "$START_AT"
        printf 'finished_at=%s\n' "$end_at"
        printf 'exit_code=%s\n' "$exit_code"
        printf 'log=%s\n' "$LOG"
    } > "$tmp"
    mv -f "$tmp" "$STATUS"
}

cleanup_signal() {
    local signal="$1"
    printf '\n[状态] 收到信号 %s，安装任务终止。\n' "$signal" >> "$LOG"
    write_status failed 143 "$(date --iso-8601=seconds)"
    exit 143
}
trap 'cleanup_signal TERM' TERM
trap 'cleanup_signal INT' INT
trap 'cleanup_signal HUP' HUP

printf '%s\n' "$$" > "$PIDFILE"
write_status running '' ''

export HOME='/home/pma213x'
export XDG_CACHE_HOME='/home/pma213x/.cache'
export PIP_CACHE_DIR='/home/pma213x/.cache/pip'
export UV_CACHE_DIR='/home/pma213x/.cache/uv'
export TMPDIR='/home/pma213x/.cache/tmp'
mkdir -p "$PIP_CACHE_DIR" "$UV_CACHE_DIR" "$TMPDIR"

printf 'START %s\n' "$START_AT" >> "$LOG"
"$VENV/bin/python" -m pip install --progress-bar off \
    'isaacsim[all,extscache]==4.5.0.0' \
    --extra-index-url https://pypi.nvidia.com >> "$LOG" 2>&1
rc=$?
printf 'END %s\nEXIT %s\n' "$(date --iso-8601=seconds)" "$rc" >> "$LOG"
if [ "$rc" -eq 0 ]; then
    write_status success "$rc" "$(date --iso-8601=seconds)"
else
    write_status failed "$rc" "$(date --iso-8601=seconds)"
fi
exit "$rc"
