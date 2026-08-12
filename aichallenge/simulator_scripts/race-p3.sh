#!/bin/bash

# race.sh と同一のセッションを、自車が **グリッド3番手** から始める形で走らせる。
# ポールから逃げるのではなく、前に2台いる状態から順位を上げられるかを見るため。
#
# 位置は --scenario で与える。official.yaml の3枠はそのままに、自車 (vehicle "1")
# だけを最後尾の枠へ移した start_p3.yaml を読ませる。official.yaml 自体は公式配布物
# なので書き換えない。
#
# ハンディキャップは順位に応じて効くので、最後尾スタートは「先頭の速度制限を受けない
# 状態から始める」ことでもある。

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
SCENARIO=/aichallenge/simulator_scripts/start_p3.yaml
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
    --scenario "${SCENARIO}" \
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
