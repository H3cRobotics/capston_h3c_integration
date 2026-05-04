# HOW TO USE
# 서버 IP를 지정하여 실행
# ./run_all.sh 192.168.0.16


cd ~/capston_h3c_integration
./clean_before_launch.sh
source install/setup.bash

SERVER_IP=${1:-192.168.0.221}

ros2 launch capston_bringup bringup_all.launch.py \
  server_ip:=$SERVER_IP