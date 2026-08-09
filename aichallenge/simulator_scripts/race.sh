#!/bin/bash

# 本番相当の評価。eval.sh と同じ 6 laps / 600s / sync 開始のセッションに、
# 本番のグリッド（他チームのカート2台）とハンディキャップを足したもの。
# eval.sh を書き換えないのは、あちらが「公式の評価設定そのまま」であるべきだから。
#
# ここで確認したいのは順位やタイムではなく **5周完走できるか**。混走グリッドは
# 実際に0周で終わった原因なので（collision_guard の停止時 emergency brake）、
# タイム計測は eval.sh、完走判定はこちらで見る。

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
export ROS_DOMAIN_ID=0

# NPCカート数: 第1引数 または NPCS 環境変数（既定 2 = 本番のグリッド）
npcs="${1:-${NPCS:-2}}"
# ハンディキャップ / 順位表示: 既定 on（本番相当）。ハンディキャップは順位に応じて
# 効くので ranking と併用する。1位だと終端速度が 24km/h 付近まで落ちる。
handicap="${HANDICAP:-on}"
ranking="${RANKING:-on}"

# count（`eval.sh` は sync）。sync + NPC は AWSIM 側が壊れる: JudgeSystem は
# 3台を数えるのに "Skipping initialization ... because SelectedVehicleCount=1" で
# NPC を初期化しないため、`state → Start` の直後に 0周のまま result を保存して
# Terminate する（2026-08-08 に確認）。count なら NPC 込みで普通に走る。
exec "$AWSIM_DIRECTORY/AWSIM.x86_64" \
    --venue citycircuit \
    --start-mode count \
    --start-count-seconds 5 \
    --vehicles 1 \
    --npcs "${npcs}" \
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
    --lidar off

# Cameraを使う場合 : --camera cpu or gpu
# LiDARを使う場合 : --lidar cpu or gpu
# GPUがない場合 -headlessを末尾に追加
