# F1TENTH Gym ROS2 PID Controller & Ackermann Steering

> F1TENTH Gym 시뮬레이션에 PID 기반 속도 제어와 Ackermann 조향을 적용하는 ROS 2 패키지

---

## 🔍 프로젝트 개요

- **목표**  
  - F1TENTH Gym 환경에서 차량 시뮬레이션을 ROS 2로 브리지  
  - PID 제어기로 목표 속도를 정확히 유지  
  - Ackermann 수식으로 자연스러운 조향 구현  
- **주요 기능**  
  - `pid_speed_control` 노드: 목표 속도 vs. 실제 속도 오차를 PID로 보정  
  - 휠베이스 기반 조향 각도 계산 (Ackermann)  
  - `teleop_twist_keyboard` 연동으로 수동·자동 운행 가능  

---

## ⚙️ 주요 요구 사항

- OS: Ubuntu 20.04  
- ROS 2 Foxy  
- Python 3.8+  

```bash
sudo apt update
sudo apt install -y \
  python3-pip python3-colcon-common-extensions \
  ros-foxy-rclpy \
  ros-foxy-ackermann-msgs \
  ros-foxy-nav-msgs \
  ros-foxy-geometry-msgs \
  ros-foxy-tf2-ros \
  ros-foxy-xacro \
  ros-foxy-joint-state-publisher \
  ros-foxy-joint-state-publisher-gui \
  ros-foxy-ackermann-msgs
pip3 install gym pygame
키보드 조작 시뮬레이션, PID제어, Ac…

📥 설치 및 빌드
1. 워크스페이스 생성 & 패키지 클론
bash
복사
편집
mkdir -p ~/f1tenth_ws/src && cd ~/f1tenth_ws/src
git clone https://github.com/f1tenth/f1tenth_gym_ros.git
2. 종속성 이슈 해결
colcon build 중 setuptools 버전 충돌 시:

bash
복사
편집
pip3 install setuptools==59.6.0
3. 빌드 & 셋업
bash
복사
편집
cd ~/f1tenth_ws
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
🐳 Docker & NVIDIA GPU (선택)
bash
복사
편집
# Docker CE 설치
sudo apt install -y ca-certificates curl gnupg lsb-release
# … (Docker & nvidia-docker2 공식 가이드 따라 설치) …

# Rocker 설치 (X11 & GPU 포워딩)
sudo apt install -y python3-rocker

# 리포 클론 & 이미지 빌드
cd ~/f1tenth_ws/src/f1tenth_gym_ros
docker build -t f1tenth_gym_ros -f Dockerfile .

# 컨테이너 실행
xhost +local:root
rocker --nvidia --x11 \
  --volume "$(pwd)":/sim_ws/src/f1tenth_gym_ros \
  f1tenth_gym_ros \
  bash
컨테이너 내부:

bash
복사
편집
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build --symlink-install
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
🏃 Native 실행
맵 경로 설정
~/f1tenth_ws/src/f1tenth_gym_ros/config/sim.yaml 에서

yaml
복사
편집
map_path: "/home/<user>/f1tenth_ws/src/f1tenth_gym_ros/maps/levine"
Static TF 퍼블리시 (RViz “map” 프레임 경고 해결용)

bash
복사
편집
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
시뮬레이션 실행

bash
복사
편집
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
키보드 조작 시뮬레이션, PID제어, Ac…

🚀 사용 방법
터미널 구성 (최소 3개)
터미널	역할	명령어 예제
1	시뮬레이션 + RViz	ros2 launch f1tenth_gym_ros gym_bridge_launch.py
(map→odom TF 필요 시 별도 터미널)	ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
2	PID 속도 제어기 실행	ros2 run pid_controller pid_speed_control -p kp:=1.0 -p ki:=0.1 -p kd:=0.01 -p wheelbase:=0.325
3	키보드 텔레옵 (수동 조작)	ros2 run teleop_twist_keyboard teleop_twist_keyboard

📝 PID & Ackermann
PID 제어

𝑢
(
𝑡
)
=
𝐾
𝑝
 
𝑒
(
𝑡
)
+
𝐾
𝑖
 ⁣
∫
𝑒
(
𝑡
)
 
𝑑
𝑡
+
𝐾
𝑑
𝑑
𝑒
(
𝑡
)
𝑑
𝑡
,
𝑒
(
𝑡
)
=
𝑣
c
m
d
−
𝑣
a
c
t
u
a
l
u(t)=K 
p
​
 e(t)+K 
i
​
 ∫e(t)dt+K 
d
​
  
dt
de(t)
​
 ,e(t)=v 
cmd
​
 −v 
actual
​
 
Ackermann 조향

𝜃
=
arctan
⁡
(
𝐿
 
𝜔
𝑣
)
,
𝐿
=
wheelbase
,
  
𝜔
=
angular.z
,
  
𝑣
=
linear.x
θ=arctan( 
v
Lω
​
 ),L=wheelbase,ω=angular.z,v=linear.x
🎮 키보드 단축키
yaml
복사
편집
   u    i    o        (전/후진 + 회전)
   j    k    l
   m    ,    .        (후진 + 회전)
k : 정지
q/z : max speed ±10%
w/x : linear speed ±10%
e/c : angular speed ±10%
🤝 기여 방법
Fork & Clone

브랜치 생성: git checkout -b feature/<your-feature>

수정 & 커밋

Pull Request 발행

📄 라이선스
MIT © 2025 icegoraebab
