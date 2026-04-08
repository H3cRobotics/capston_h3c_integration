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
python3 -m pip uninstall -y torch torchvision
pip3 uninstall ultralytics -y

wget https://raw.githubusercontent.com/pytorch/pytorch/5c6af2b583709f6176898c017424dc9981023c28/.ci/docker/common/install_cusparselt.sh
export CUDA_VERSION=12.1
bash ./install_cusparselt.sh

wget "https://pypi.jetson-ai-lab.dev/jp6/cu126/+f/6cc/6ecfe8a5994fd/torch-2.6.0-cp310-cp310-linux_aarch64.whl#sha256=6cc6ecfe8a5994fd6d58fb6d6eb73ff2437428bb4953f3ebaa409f83a5f4db99"
wget "https://pypi.jetson-ai-lab.dev/jp6/cu126/+f/aa2/2da8dcf4c4c8d/torchvision-0.21.0-cp310-cp310-linux_aarch64.whl#sha256=aa22da8dcf4c4c8dc897e7922b1ef25cb0fe350e1a358168be87a854ad114531"

pip3 install torch-2.6.0-cp310-cp310-linux_aarch64.whl
pip3 install torchvision-0.21.0-cp310-cp310-linux_aarch64.whl

python3 -m pip install -U ultralytics

확인 ========================================== 
python3 - <<EOF
import torch
print(torch.__version__)
print(torch.cuda.is_available())
print(torch.version.cuda)
if torch.cuda.is_available():
    print(torch.cuda.get_device_name(0))
EOF

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
