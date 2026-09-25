#!/usr/bin/env python3
# 分析 /tmp/ball_detect 下的 PPM 帧：内容统计、橙色像素阈值扫描、导出 PNG 供目视检查
import glob
import os
import sys

import numpy as np
from PIL import Image

SRC = "/tmp/ball_detect"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frame_preview")
os.makedirs(OUT, exist_ok=True)

frames = sorted(glob.glob(os.path.join(SRC, "frame_*.ppm")))
print(f"帧数量: {len(frames)}")
if not frames:
    sys.exit(1)

def stats(path):
    im = Image.open(path)
    a = np.asarray(im)  # H x W x 3
    h, w, _ = a.shape
    r = a[:, :, 0].astype(np.int32)
    g = a[:, :, 1].astype(np.int32)
    b = a[:, :, 2].astype(np.int32)
    # 当前阈值规则
    m = (r > 150) & (g > 40) & (g < 160) & (b < 90) & (r > g + 40) & (g > b)
    # 更宽松的"橙色"统计，用于判断阈值是否过严
    loose = (r > 120) & (r > g + 20) & (g > b) & (b < 120)
    very_loose = (r > g) & (g >= b) & (r > 80)
    return {
        "size": (w, h),
        "mean_rgb": tuple(a.reshape(-1, 3).mean(axis=0).round(1)),
        "min_rgb": tuple(a.reshape(-1, 3).min(axis=0)),
        "max_rgb": tuple(a.reshape(-1, 3).max(axis=0)),
        "n_strict": int(m.sum()),
        "n_loose": int(loose.sum()),
        "n_very_loose": int(very_loose.sum()),
        "n_unique_colors": len(np.unique(a.reshape(-1, 3), axis=0)),
    }

for idx in [0, len(frames) // 2, len(frames) - 1]:
    p = frames[idx]
    s = stats(p)
    print(f"\n== {os.path.basename(p)} ==")
    for k, v in s.items():
        print(f"  {k}: {v}")
    # 导出缩小的 PNG 供目视
    im = Image.open(p)
    im.resize((320, 240)).save(os.path.join(OUT, os.path.basename(p).replace(".ppm", ".png")))
    # 也导出全尺寸 PNG（第一帧）便于放大看
    if idx == 0:
        im.save(os.path.join(OUT, "full_" + os.path.basename(p).replace(".ppm", ".png")))

# 帧间差异：判断相机画面是否在变化（机器人是否在动）
def load_small(p):
    return np.asarray(Image.open(p).resize((64, 48)), dtype=np.int16)

a0 = load_small(frames[0])
a1 = load_small(frames[len(frames) // 2])
a2 = load_small(frames[-1])
print("\n帧间平均绝对差:")
print(f"  first vs mid:  {np.abs(a0 - a1).mean():.2f}")
print(f"  mid vs last:   {np.abs(a1 - a2).mean():.2f}")
print(f"  first vs last: {np.abs(a0 - a2).mean():.2f}")

# 找出最"橙"的像素位置（宽松阈值），看是否在画面边缘外的接近物
p = frames[-1]
a = np.asarray(Image.open(p))
r, g, b = a[:, :, 0].astype(np.int32), a[:, :, 1].astype(np.int32), a[:, :, 2].astype(np.int32)
loose = (r > 120) & (r > g + 20) & (g > b) & (b < 120)
ys, xs = np.nonzero(loose)
print(f"\n最后一帧宽松橙像素: {loose.sum()}")
if loose.sum():
    print(f"  x range {xs.min()}..{xs.max()}  y range {ys.min()}..{ys.max()}")
    print(f"  质心 ({xs.mean():.1f}, {ys.mean():.1f})")

print(f"\nPNG 预览已写入 {OUT}")
