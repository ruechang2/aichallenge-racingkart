#!/usr/bin/env python3
"""記録済み rosbag の control_cmd に速度指令が入っているかを確認する。

目標速度ヘッドの教師信号に longitudinal.speed が使えるかどうかの判断に使う。
"""
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

bag_path = Path(sys.argv[1])
speeds, accels = [], []

with AnyReader([bag_path]) as reader:
    conns = [c for c in reader.connections if c.topic == "/control/command/control_cmd"]
    for conn, _timestamp, raw in reader.messages(connections=conns):
        msg = reader.deserialize(raw, conn.msgtype)
        speeds.append(msg.longitudinal.speed)
        accels.append(msg.longitudinal.acceleration)

speeds = np.array(speeds)
accels = np.array(accels)
print(f"messages: {len(speeds)}")
print(f"longitudinal.speed        min={speeds.min():.3f} max={speeds.max():.3f} "
      f"mean={speeds.mean():.3f} nonzero={int((speeds != 0).sum())}")
print(f"longitudinal.acceleration min={accels.min():.3f} max={accels.max():.3f} "
      f"mean={accels.mean():.3f}")
