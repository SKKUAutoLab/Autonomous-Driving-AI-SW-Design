# Autonomous-Driving-AI-SW-Design
성균관대학교 [자율주행 AI SW 설계] 비교과 프로그램

## 초기 환경설정
```
git clone https://github.com/SKKUAutoLab/Autonomous-Driving-AI-SW-Design.git
cd ~/Autonomous-Driving-AI-SW-Design
sh install.sh
source ~/.bashrc
```

```
cd ~/Autonomous-Driving-AI-SW-Design
export AMENT_PREFIX_PATH=''
export CMAKE_PREFIX_PATH=''
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y
```

## 패키지 빌드
```
cd ~/Autonomous-Driving-AI-SW-Design
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```
## 검증 SW 사용법
초기 환경 설정과 패키지 빌드를 마친 후 사용

아두이노: ./src/control/sw_verification 경로의 sw_verification.ino 사용

GUI:
``` 
cd ~/Autonomous-Driving-AI-SW-Design
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run serial_communication_pkg sw_verification_node
```
