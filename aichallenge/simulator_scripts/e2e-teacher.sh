#!/bin/bash
# E2E の教師データ収集用（NPC 2体 / imu・gnss on / タイムアウト実質なし）
#
# e2e.sh との違いは自己位置に必要なセンサを on にしていること。MPC 教師は
# map 座標で走るため imu/gnss が要るが、生徒が学習に使うのは LiDAR スキャンと
# control_cmd だけなので、収集時に on でも生徒の観測は汚れない。
# 開始位置は毎回変える（--start-random on）。決勝の相手は他チームの車なので、
# 決まった位置の停車車両を覚えるデータにしないため。

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
export ROS_DOMAIN_ID=0

exec $AWSIM_DIRECTORY/AWSIM.x86_64 \
    --venue citycircuit \
    --start-mode count \
    --start-count-seconds 0 \
    --vehicles 1 \
    --npcs 2 \
    --boosts 2 \
    --laps unlimited \
    --timeout 10000000.0 \
    --steer-source ackermann \
    --sound off \
    --collisions on \
    --handicap off \
    --wall-recovery off \
    --start-random on \
    --ranking off \
    --camera off \
    --lidar on \
    --imu on \
    --gnss on \
    --v2x off

# Cameraを使う場合 : --camera off or gpu
# LiDARを使う場合 : --lidar on or gpu
# GPUがない場合 -headlessを末尾に追加
