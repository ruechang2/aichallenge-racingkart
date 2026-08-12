#!/bin/bash

# 実車両3台を、AWSIM の描画コストを最小にして走らせる。
# 目的は「3台とも自分で走るレース」を 8 コア機で成立させること。実測
# (2026-08-04) では Autoware スタック3本は載らず load average 30.6 で3台とも
# 0周だった。Autoware 側も RUN_MODE=awsim-no-viz (rviz なし) と併用する。
#
# **-batchmode -nographics は使えない (2026-08-11 に確認)。** 描画を完全に切ると
# フレーム同期がなくなり AWSIM が実時間から解き放たれる。Autoware が接続する前に
# 600 秒のセッションを数秒で消化し、`state → Start` の直後に 0周で
# result-summary.json を保存して Terminate した。ここでは描画は残したまま、
# 描画は残す。320x240 / -screen-quality Fastest も試したが AWSIM が SIGSEGV
# (exit 139) で落ちたので、parallel.sh と同じ実績のある設定を使う。GPU は
# RTX 3090 で 1280x720 low の描画は安く、律速は 3 スタック分の CPU である。
#
# start-mode は count。sync は StartSyncCoordinator と awsim_state_manager の
# ハンドシェイクが要り、スタックを別コンテナで立てる構成では WaitStart のまま
# SelectMode に戻る (2026-08-11 に確認)。count なら握手不要。

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
export ROS_DOMAIN_ID=0

vehicles="${1:-3}"
handicap="${HANDICAP:-on}"
ranking="${RANKING:-on}"

exec "$AWSIM_DIRECTORY/AWSIM.x86_64" \
    --venue citycircuit \
    --start-mode count \
    --start-count-seconds 5 \
    --vehicles "${vehicles}" \
    --npcs 0 \
    --boosts 2 \
    --laps 6 \
    --timeout 600 \
    --steer-source ackermann \
    --sound off \
    --collisions on \
    --handicap "${handicap}" \
    --wall-recovery off \
    --ranking "${ranking}" \
    --camera off \
    --lidar off \
    -screen-fullscreen 1 \
    -screen-width 1280 \
    -screen-height 720 \
    -screen-quality low \
    -window-mode borderless
