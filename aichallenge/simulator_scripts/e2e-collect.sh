#!/bin/bash
# E2E の練習兼提出参考用（NPC 2体 / handicap・ranking なし / タイムアウト実質なし）

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
export ROS_DOMAIN_ID=0

exec $AWSIM_DIRECTORY/AWSIM.x86_64 \
    --venue citycircuit \
    --start-mode count \
    --start-count-seconds 0 \
    --vehicles 1 \
    --npcs 0 \
    --boosts 2 \
    --laps unlimited \
    --timeout 10000000.0 \
    --steer-source ackermann \
    --sound off \
    --collisions on \
    --handicap off \
    --wall-recovery off \
    --start-random off \
    --ranking off \
    --camera gpu \
    --lidar on \
    --imu on \
    --gnss on \
    --v2x off

# Cameraを使う場合 : --camera gpu or gpu
# LiDARを使う場合 : --lidar on or gpu
# GPUがない場合 -headlessを末尾に追加
