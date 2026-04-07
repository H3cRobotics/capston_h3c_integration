0. 
- pip install ultralytics

- python3 export_engine.py --model yolov8n.pt --half
# 추론 이미지 사이즈 640, FP16 최적화 >> engine 파일 생성

- yaml 설정에서 model path를 engine파일의 절대경로로 변경


==================================

1. 
cd ~/ros2_ws
colcon build --packages-select person_tracker_ros
source install/setup.bash

ros2 launch person_tracker_ros person_tracker.launch.py


==============================


2. 확인 및 I/O정리

트레킹 결과 토픽 -- /person_tracking/tracks_json

ros2 topic echo /person_tracking/tracks_json


트레킹 시각화 토픽 -- /person_tracking/annotated
rqt_image_view
