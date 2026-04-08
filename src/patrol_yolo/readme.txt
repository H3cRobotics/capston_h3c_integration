**** 참고 https://rainbow-chaser.tistory.com/4


0. 
제트팩 버전 확인  ==============================================
dpkg-query --show nvidia-l4t-core
apt show nvidia-jetpack

>>jP 6.2 계열로 확인


파이썬, 넘파이 , 제트팩 버전 확인  ==============================================

python3 --version
python3 -c "import sys; import numpy; print(sys.version); print(numpy.__version__)"
uname -m
>>
JetPack Runtime 6.2.1
Python 3.10.12
aarch64

설치 ==============================================
sudo apt-get update
sudo apt-get install -y python3-pip libopenblas-dev

pip3 install numpy==1.26.4
pip3 uninstall torch torchvision ultralytics -y

wget https://raw.githubusercontent.com/pytorch/pytorch/5c6af2b583709f6176898c017424dc9981023c28/.ci/docker/common/install_cusparselt.sh
export CUDA_VERSION=12.6
bash ./install_cusparselt.sh

========= 다운받은 wget이 cuda version 적용 및 문법 오류가 있을 수 있는데 gpt 시키기
1. torch
wget https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
python3 -m pip install --no-cache ./torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl

python3 - <<EOF
import torch
print(torch.__version__)
print(torch.cuda.is_available())
EOF

pip install https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl

확인 ========================================== 
python3 - <<EOF
import torch
print(torch.__version__)
print(torch.cuda.is_available())
print(torch.version.cuda)
if torch.cuda.is_available():
    print(torch.cuda.get_device_name(0))
EOF

python3 -m pip install -U ultralytics



0.1 pt파일 텐서화 

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
