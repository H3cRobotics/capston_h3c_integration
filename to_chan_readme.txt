
# =============================================================================================================================================================================================================
# ================= 교시 및 캡처노드 서버와 연결하지 않고 테스트하는 법 (서버 연결시와 로직 동일하므로 이거로 테스트하면 됨)
# ================= 실행 순서 준수

# 1. 더미 서버 및 브릿지 통합 런치파일 실행 ------------------------------------------------------
<공통>
cd capston_h3c_integration

터미널 1: 통합 브릿지 노드
source install/setup.bash
ros2 launch capston_bringup bringup_all.launch.py server_ip:=127.0.0.1

터미널 2: 더미 서버 노드
source install/setup.bash
ros2 run patrol_bridge dummy_patrol_server_node
* 터미널 1에서 에러 메세지는 무시하면 됨 ( 전체 서버와 연결되지 않아 발생하는 오류인데 문제없음)

# 2. 교시 로직 -----------------------------------------------------------------------------

터미널 3 : waypoint reload 수신 토픽
source install/setup.bash
ros2 topic echo /patrol/waypoints_json --full-length

터미널 4 : waypoint reload 트리거 토픽
source install/setup.bash
ros2 topic pub --once /patrol/reload_waypoints std_msgs/msg/Empty "{}"

* 트리거 1회당 /waypoints_json에 "한번" 발행하므로 수신 토픽은 항상 켜져잇어야함


# 3. 캡처 로직 -----------------------------------------------------------------------------

# 터미널 5 : place_id를 포함한 캡처 트리거 발행
# 로봇이 도착한 place_id(예: P03)를 /patrol/capture_trigger 로 직접 보냄
source install/setup.bash
ros2 topic pub --once /patrol/capture_trigger std_msgs/msg/String "{data: 'P03'}"



cf. 필요한 경우만 사용 ==============================================================================================

# ================== CAN 통신 활성화 코드

sudo modprobe mttcan

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up

ifconfig can0


data 확인
candump can0


# ================= dummy /robot_pose
주행부와 같이 하지 않고 더미 로봇 포즈를 발생하고 싶은 경우 사용

cd capston_h3c_integration
source install/setup.bash
ros2 run patrol_bridge dummy_pose_input