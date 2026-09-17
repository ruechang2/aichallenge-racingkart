#!/usr/bin/env python3
"""学習した PyTorch モデルと、ROS ノードが使う NumPy 実装の出力が一致するか確認する。

重みを変換してデプロイしても、推論側の入力の作り方（チャネル数・並び順）が
ずれていれば車は正しく走らない。走らせる前にここで気づけるようにする。

    python3 check_numpy_parity.py --ckpt <best_model.pth> --weights <converted.npy> --n-frames 4
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, "/aichallenge/ml_workspace/tiny_lidar_net")
sys.path.insert(
    0,
    "/aichallenge/workspace/src/aichallenge_submit/tiny_lidar_net_controller/tiny_lidar_net_controller",
)

from lib.model import TinyLidarNet  # noqa: E402
from model.tinylidarnet import TinyLidarNetNp  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ckpt", type=Path, required=True)
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--input-dim", type=int, default=750)
    parser.add_argument("--output-dim", type=int, default=2)
    parser.add_argument("--n-frames", type=int, default=1)
    parser.add_argument("--samples", type=int, default=32)
    args = parser.parse_args()

    torch_model = TinyLidarNet(
        input_dim=args.input_dim, output_dim=args.output_dim, in_channels=args.n_frames
    )
    torch_model.load_state_dict(torch.load(args.ckpt, map_location="cpu"))
    torch_model.eval()

    np_model = TinyLidarNetNp(
        input_dim=args.input_dim, output_dim=args.output_dim, in_channels=args.n_frames
    )
    loaded = np.load(args.weights, allow_pickle=True).item()
    for key, value in loaded.items():
        np_model.params[key.replace(".", "_")] = value

    rng = np.random.default_rng(0)
    x = rng.random((args.samples, args.n_frames, args.input_dim), dtype=np.float32)

    with torch.no_grad():
        expected = torch_model(torch.from_numpy(x)).numpy()
    actual = np_model(x)

    max_diff = float(np.abs(expected - actual).max())
    print(f"input shape : {x.shape}")
    print(f"max abs diff: {max_diff:.3e}")
    if max_diff < 1e-4:
        print("OK: NumPy 推論は PyTorch と一致しています")
    else:
        print("NG: 出力がずれています。推論側の入力の作り方を確認してください")
        sys.exit(1)


if __name__ == "__main__":
    main()
