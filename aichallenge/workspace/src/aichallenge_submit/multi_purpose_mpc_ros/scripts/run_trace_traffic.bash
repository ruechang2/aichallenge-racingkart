#!/bin/bash
# Record ego-vs-kart relative geometry for measuring an overtake.
# shellcheck disable=SC1091
source /autoware/install/setup.bash
source /aichallenge/workspace/install/setup.bash
source "$(ros2 pkg prefix multi_purpose_mpc_ros)/.venv/bin/activate"
exec timeout "${DURATION:-240}" python3 \
  /aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/scripts/trace_traffic.py \
  --out "${OUT:-/output/traffic.csv}"
