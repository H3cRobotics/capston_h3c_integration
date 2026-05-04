#!/bin/bash

# HOW TO USE
# 서버 IP만 지정하여 실행
# ./run_all.sh 192.168.0.16
#
# 서버 IP + GUI 실행 여부 지정
# ./run_all.sh 192.168.0.16 true
# ./run_all.sh 192.168.0.16 false

cd ~/capston_h3c_integration || exit 1

./clean_before_launch.sh
source install/setup.bash

SERVER_IP=${1:-192.168.0.221}
ENABLE_GUI=${2:-true}

taskset -c 4,5,6,7 ros2 launch capston_bringup bringup_all.launch.py \
  server_ip:=$SERVER_IP \
  enable_gui:=$ENABLE_GUI