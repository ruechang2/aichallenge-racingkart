# E2E 部門の作業ツール

教師（MPC・LiDAR 観測）で bag を録る → 抽出・学習・デプロイ → 生徒を NPC 混走で評価、を回すためのスクリプト。
ホストで実行するものと、autoware コンテナ内で実行するものがある。走行ログ・CSV は `output/_check/` に出る（git 管理外）。

| スクリプト | 実行場所 | 用途 |
|---|---|---|
| `run_teacher.bash <bag名> <秒> [e2e-teacher\|e2e-teacher-eval]` | ホスト | AWSIM＋教師 MPC を起動し、`record_e2e_dataset.bash` で録画、`race_log.py` で監視、`laps.py` で集計 |
| `run_student.bash <名> <秒> [e2e-npc]` | ホスト | 生徒（現行 ckpt）を NPC 2 台・固定スタートで走らせて集計 |
| `pipeline.bash <タグ> <train bag...> -- <val bag...>` | コンテナ | 抽出 → 学習 → NumPy 変換 → 等価性確認 → `ckpt/` へデプロイ。`CMD="bash /aichallenge/utils/e2e/pipeline.bash ..." docker compose run --rm --no-deps autoware-command` |
| `in.bash '<cmd>'` | ホスト | autoware コンテナ内で ROS 環境つきでコマンドを実行 |
| `race_log.py` / `laps.py` | コンテナ / ホスト | `/awsim/status` の自車行（残ブースト数 = 2）でラップを数える。E2E モードは自己位置が来ないため |
| `cmd_probe.py`, `status_probe.py` | コンテナ | control_cmd・車速・ギア、`/awsim/status` の生値を見る |
| `stall_point.py`, `pinch.py`, `bounds.py`, `where.py`, `corridor_*.py` | コンテナ（MPC の venv） | 停止地点のコリドー幅、車幅より狭い区間、自車がコリドー内かを見る。`source $(ros2 pkg prefix multi_purpose_mpc_ros)/.venv/bin/activate` してから |

AWSIM は `Xvfb :99` 上で動かす（`DISPLAY=:99 XAUTHORITY=`）。走行後の `aichallenge/d1-result-details.json` にペナルティの種別・時刻・長さが残る（次の走行で上書きされる）。
