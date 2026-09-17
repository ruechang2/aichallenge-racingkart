#!/bin/bash
# 教師（MPC・LiDAR 観測）を NPC 入りで走らせ、rosbag と走行ログを録る（ホストで実行）。
#   run_teacher.bash <bag名> <秒数> [sim script 名: e2e-teacher|e2e-teacher-eval]
set -eo pipefail
NAME="$1"; DUR="${2:-420}"; SIM="${3:-e2e-teacher}"
cd "$(dirname "$0")/../../.."; mkdir -p output/_check
export DISPLAY=:99 XAUTHORITY=
pgrep -x Xvfb >/dev/null || (Xvfb :99 -screen 0 1920x1080x24 >/dev/null 2>&1 &)
make down >/dev/null 2>&1 || true
make simulator-$SIM >/dev/null 2>&1
sleep 3
CONTROL_METHOD=mpc USE_OBSTACLE_AVOIDANCE=true USE_LIDAR_OBSTACLES=true make autoware-simulator >/dev/null 2>&1
sleep 2; LOG=$(ls -td output/2026*/ | head -1)
echo "[run_teacher] log dir: $LOG"
# 車が動き出すまで待つ（最大 120 秒）
for i in $(seq 1 60); do
  v=$(bash aichallenge/utils/e2e/in.bash 'timeout 3 ros2 topic echo --once /vehicle/status/velocity_status 2>/dev/null | grep longitudinal_velocity | awk "{print \$2}"' 2>/dev/null || true)
  if [ -n "$v" ] && awk "BEGIN{exit !($v > 1.0)}"; then echo "[run_teacher] moving (v=$v) after ~$((i*2))s"; break; fi
  sleep 2
done
grep -q "dynamic obstacle source: LiDAR" $LOG/d1/autoware.log && echo "[run_teacher] teacher = MPC with LiDAR obstacles" || echo "[run_teacher] WARNING: LiDAR source line not found"
docker exec -d -e ROS_DOMAIN_ID=1 aichallenge-e2e-autoware-1 bash -c "source /opt/ros/humble/setup.bash; /aichallenge/utils/record_e2e_dataset.bash $NAME > /output/_check/record_$NAME.log 2>&1"
bash aichallenge/utils/e2e/in.bash "python3 /aichallenge/utils/e2e/race_log.py $DUR /output/_check/race_$NAME.csv" 2>&1 | grep -E "speed=|wrote|Traceback" || true
docker exec aichallenge-e2e-autoware-1 bash -c 'pkill -INT -f "ros2 bag record"' || true
sleep 3
echo "[run_teacher] gap keeping lines: $(grep -c 'gap keeping' $LOG/d1/autoware.log)"
python3 aichallenge/utils/e2e/laps.py output/_check/race_$NAME.csv
