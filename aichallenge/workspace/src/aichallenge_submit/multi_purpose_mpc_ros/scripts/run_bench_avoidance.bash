source /autoware/install/setup.bash
source /aichallenge/workspace/install/setup.bash
source "$(ros2 pkg prefix multi_purpose_mpc_ros)/.venv/bin/activate"
python3 /aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/scripts/bench_avoidance.py "$@"
