#!/bin/bash
# Publish synthetic V2X karts for avoidance testing. Run inside the autoware
# container, on the same ROS_DOMAIN_ID as the stack under test:
#
#   CMD="KARTS='d2:18:25' bash /aichallenge/workspace/src/aichallenge_submit/\
#       multi_purpose_mpc_ros/scripts/run_fake_v2x.bash" \
#     docker compose run --rm --no-deps autoware-command
#
# KARTS is a space-separated list of id:speed_kmh:start_s[:lateral].
# shellcheck disable=SC1091
source /autoware/install/setup.bash
source /aichallenge/workspace/install/setup.bash
source "$(ros2 pkg prefix multi_purpose_mpc_ros)/.venv/bin/activate"

# shellcheck disable=SC2086
exec python3 \
  /aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/scripts/fake_v2x_publisher.py \
  --karts ${KARTS:-d2:18:25} ${EXTRA_ARGS:-}
