#!/bin/bash
# ホストから: bash output/_check/in.bash <コンテナ内コマンド...>  — autoware コンテナで ROS 環境つきで実行
exec docker exec -e ROS_DOMAIN_ID=1 aichallenge-e2e-autoware-1 bash -c 'source /opt/ros/humble/setup.bash; source /aichallenge/workspace/install/setup.bash; '"$*"
