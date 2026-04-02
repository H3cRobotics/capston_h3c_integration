sudo modprobe mttcan

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up

ifconfig can0


data 확인
candump can0


--

# 실행

cd capston_h3c_integration
source install/setup.bash

ros2 launch patrol_bringup bringup.launch.py server_ip:=127.0.0.1
ros2 run patrol_bridge dummy_patrol_server_node

# waypoint 리로드
ros2 topic pub--once /reload_waypoints std_msgs/msg/Empty"{}"
ros2 topicecho /waypoints_json


# 캡처
place_id 전달
ros2 topic pub /patrol/current_place std_msgs/msg/String "{data: 'P03'}" --once

캡처 트리거
ros2 topic pub /patrol/capture_trigger std_msgs/msg/Empty "{}" --once