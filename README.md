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

