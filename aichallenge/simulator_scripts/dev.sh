#!/bin/bash

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
export ROS_DOMAIN_ID=0

# 車両数: 第1引数（既定 1）
vehicles="${1:-1}"
# NPCカート数: 第2引数 または NPCS 環境変数（既定 0）。
# 本番は他チームのカート2台と混走するので、回避のテストには NPCS=2 を使う。
npcs="${2:-${NPCS:-0}}"

exec $AWSIM_DIRECTORY/AWSIM.x86_64 \
    --venue citycircuit \
    --start-mode count \
    --start-count-seconds 5 \
    --vehicles "${vehicles}" \
    --npcs "${npcs}" \
    --boosts 2 \
    --laps unlimited \
    --timeout 10000000.0 \
    --steer-source ackermann \
    --sound off \
    --collisions on \
    --handicap off \
    --wall-recovery off \
    --ranking off \
    --camera off \
    --lidar off

# Cameraを使う場合 : --camera cpu or gpu
# LiDARを使う場合 : --lidar cpu or gpu
# GPUがない場合 -headlessを末尾に追加
