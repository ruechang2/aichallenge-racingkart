#!/usr/bin/env python3
"""抽出したデータセットの形と値域を確認する。"""
import glob
import os

import numpy as np

for model in ("tiny_lidar_net", "pilot_net"):
    print(f"## {model}")
    for split in ("train", "val"):
        d = f"/aichallenge/ml_workspace/{model}/dataset/{split}"
        for f in sorted(glob.glob(d + "/**/*.npy", recursive=True)):
            a = np.load(f)
            print(f"  {split:5s} {os.path.basename(f):18s} "
                  f"shape={a.shape} dtype={a.dtype} "
                  f"min={a.min():.3f} max={a.max():.3f} mean={a.mean():.3f}")
