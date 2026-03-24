sudo modprobe mttcan

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up

ifconfig can0


data 확인
candump can0