#!/bin/bash
# 生徒（TinyLidarNet・現行 ckpt）を NPC 2 台・固定スタートで走らせ、ラップを数える（ホストで実行）。
#   run_student.bash <ログ名> <秒数> [sim script 名: e2e-npc|e2e]
set -eo pipefail
NAME="$1"; DUR="${2:-480}"; SIM="${3:-e2e-npc}"
cd "$(dirname "$0")/../../.."; mkdir -p output/_check
export DISPLAY=:99 XAUTHORITY=
pgrep -x Xvfb >/dev/null || (Xvfb :99 -screen 0 1920x1080x24 >/dev/null 2>&1 &)
make down >/dev/null 2>&1 || true
make simulator-$SIM >/dev/null 2>&1
sleep 3
make autoware-simulator >/dev/null 2>&1
sleep 2; LOG=$(ls -td output/2026*/ | head -1)
echo "[run_student] log dir: $LOG  weights: $(md5sum aichallenge/workspace/src/aichallenge_submit/tiny_lidar_net_controller/ckpt/tinylidarnet_weights.npy | cut -c1-8)"
for i in $(seq 1 60); do
  v=$(bash aichallenge/utils/e2e/in.bash 'timeout 3 ros2 topic echo --once /vehicle/status/velocity_status 2>/dev/null | grep longitudinal_velocity | awk "{print \$2}"' 2>/dev/null || true)
  if [ -n "$v" ] && awk "BEGIN{exit !($v > 1.0)}"; then echo "[run_student] moving (v=$v) after ~$((i*2))s"; break; fi
  sleep 2
done
bash aichallenge/utils/e2e/in.bash "python3 /aichallenge/utils/e2e/race_log.py $DUR /output/_check/race_$NAME.csv" 2>&1 | grep -E "speed=|wrote|Traceback" || true
echo "[run_student] recovery fired: $(grep -c -iE 'recovery|stuck' $LOG/d1/autoware.log) lines; collision: $(grep -c -i 'collision' $LOG/d1/autoware.log)"
python3 aichallenge/utils/e2e/laps.py output/_check/race_$NAME.csv
