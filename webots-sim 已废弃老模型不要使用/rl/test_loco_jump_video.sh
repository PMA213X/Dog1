#!/usr/bin/env bash
# yobogo_loco_jump_v1 第三人称录像短链路测试。
# 只生成合成 mock 片段，不加载 checkpoint、不启动 Webots，结果不可用于验收。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLAY="${ROOT}/webots-sim/rl/play_third_person.py"
PYTHON="${PYTHON:-python3}"
FRAMES_DIR="${ROOT}/videos/frames/loco_jump_chain"
FRAME_COUNT="${FRAME_COUNT:-30}"

command -v ffprobe >/dev/null 2>&1 || { echo "缺少 ffprobe" >&2; exit 127; }
"${PYTHON}" -c "import cv2" >/dev/null 2>&1 || { echo "缺少 OpenCV" >&2; exit 127; }

for task in move turn jump; do
    video="${ROOT}/videos/_loco_jump_chain_${task}.mp4"
    echo "=== 测试 ${task}: ${video} ==="
    "${PYTHON}" "${PLAY}" \
        --mock-frames "${FRAME_COUNT}" \
        --mock-task "${task}" \
        --video "${video}" \
        --frames-dir "${FRAMES_DIR}" \
        --fps 30 \
        --min-duration 1.0 \
        --expected-width 640 \
        --expected-height 480 \
        --min-nonblack-ratio 0.5

    ffprobe -v error -select_streams v:0 \
        -show_entries stream=width,height,r_frame_rate \
        -show_entries format=duration \
        -of default=noprint_wrappers=1 "${video}"

    "${PYTHON}" - "${video}" <<'PY'
import sys
from pathlib import Path

import cv2
import numpy as np

path = Path(sys.argv[1])
capture = cv2.VideoCapture(str(path))
assert capture.isOpened(), f"无法打开：{path}"
decoded = 0
nonblack = 0
resolution = None
while True:
    ok, frame = capture.read()
    if not ok:
        break
    decoded += 1
    if resolution is None:
        resolution = (frame.shape[1], frame.shape[0])
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if float(gray.max()) >= 16.0 and float(gray.std()) >= 1.0:
        nonblack += 1
capture.release()
assert resolution == (640, 480), f"解码分辨率错误：{resolution}"
assert decoded == 30, f"解码帧数错误：{decoded}"
assert nonblack >= 15, f"非黑帧不足：{nonblack}/{decoded}"
print(f"OpenCV 解码通过：{resolution[0]}x{resolution[1]}，"
      f"非黑帧 {nonblack}/{decoded}")
PY
done

echo "全部第三人称录像短链路测试通过；产物仅限 mock 验证，禁止作为模型验收。"
