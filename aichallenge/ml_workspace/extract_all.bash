#!/bin/bash
# 1回の走行から TinyLidarNet 用と PilotNet 用のデータセットを両方作る。
# train/ と val/ には別々の rosbag を置いておくこと（同じものをコピーすると
# val loss が訓練誤差と同じものになり、学習の判断材料にならない）。
set -e
for m in tiny_lidar_net pilot_net; do
    echo "########## ${m} ##########"
    cd "/aichallenge/ml_workspace/${m}"
    python3 ./extract_data_from_bag.py --bags-dir /aichallenge/ml_workspace/train/ --outdir ./dataset/train/
    python3 ./extract_data_from_bag.py --bags-dir /aichallenge/ml_workspace/val/   --outdir ./dataset/val/
done
