#!/bin/bash
# E2E の学習データ用 rosbag を録る。
#
#   record_e2e_dataset.bash [出力名]
#
# 生徒（TinyLidarNet）が使うのは scan と control_cmd だけだが、あとから
# 「どこで何をしたデモか」を追えるように自己位置と車速も入れておく。
# 全 topic 録り（record_all_rosbag.bash）は画像まで入って巨大になるため使わない。
#
# 教師データを録るときは MPC を走らせていること（CONTROL_METHOD=mpc）。
# 生徒が走っている状態で録っても、ラベルは自分の出力の写しにしかならない。

# ROS の setup.bash は未定義変数を参照するので -u は使えない（build_autoware.bash と同じ）。
set -eo pipefail

name="${1:-teacher-$(date +%Y%m%d-%H%M%S)}"
out_dir="${OUTPUT_ROOT:-/output}/${name}"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash

TOPICS=(
    "/sensing/lidar/scan"                  # 生徒の入力
    "/control/command/control_cmd"         # 教師のラベル
    "/localization/kinematic_state"        # 走行位置の追跡用
    "/vehicle/status/velocity_status"      # 実速度（止まった区間の切り出しに使う）
)

echo "[record_e2e_dataset] recording to ${out_dir}"
exec ros2 bag record "${TOPICS[@]}" \
    -o "${out_dir}" -s mcap \
    --compression-format zstd --compression-mode file
