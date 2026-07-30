# 스마트 쓰레기통 Raspberry Pi 5 + 서버 개발 환경 구축 가이드

> Pi 5 구매 후 순서대로 따라하면 되는 체크리스트 중심 문서

---

## Phase 1: Raspberry Pi 5 하드웨어 준비

### 1.1 개봉 및 점검
- [ ] Pi 5 4GB 본체 확인
- [ ] 공식 27W PSU 또는 임시 5V 5A PSU 준비
- [ ] microSD 64GB A2 또는 USB SSD 준비
- [ ] 키보드/마우스/모니터 또는 UART 콘솔 케이블 준비
- [ ] 방열판/쿨러 부착 (Pi 5 발열 심함)

### 1.2 Ubuntu 24.04 설치

```bash
# Windows에서 Raspberry Pi Imager 다운로드
# https://www.raspberrypi.com/software/

# 1. Imager 실행
# 2. OS 선택 → "Other general-purpose OS" → Ubuntu → Ubuntu Server 24.04 LTS (64-bit)
# 3. 저장 장치 선택 → microSD 또는 USB SSD
# 4. 설정: SSH 활성화, Wi-Fi SSID/비밀번호, 사용자명/비밀번호 입력
# 5. Write
```

부팅 후 SSH로 접속:

```bash
ssh ubuntu@raspberrypi.local
# 또는 IP로 접속
ssh ubuntu@192.168.x.x
```

### 1.3 기본 시스템 설정

```bash
# 패키지 업데이트
sudo apt update && sudo apt upgrade -y

# 한국 시간대 설정
sudo timedatectl set-timezone Asia/Seoul

# 호스트명 변경 (각 로봇별)
sudo hostnamectl set-hostname trashbot-01

# IP 고정 (선택)
# /etc/netplan/00-installer-config.yaml 편집
```

### 1.4 I2C, UART, 카메라 활성화

```bash
# Raspberry Pi Utils 설치
sudo apt install -y raspi-config rpi-eeprom

# config.txt 편집
sudo nano /boot/firmware/config.txt
```

아래 내용 추가:

```ini
# Pi 5 전류 제한 해제 (5V Buck 사용 시 필수)
PSU_MAX_CURRENT_3000=5000
usb_max_current_enable=1

# I2C 활성화
dtparam=i2c_arm=on

# UART 활성화 (Yahboom 드라이버, ESP32-S3 통신용)
enable_uart=1

# 카메라 활성화 (선택, VSLAM 서브 옵션 사용 시에만 필요)
camera_auto_detect=1

# GPU 메모리 최소화 (SLAM/센서 처리는 CPU 중심이라 GPU 메모리는 최소화)
gpu_mem=128
```

재부팅:

```bash
sudo reboot
```

확인:

```bash
# I2C 확인
sudo i2cdetect -y 1

# UART 확인
ls /dev/ttyS*
ls /dev/ttyAMA*

# 카메라 확인
rpicam-hello -t 0
```

---

## Phase 2: ROS2 Jazzy 설치 (Pi 5)

### 2.1 Locale 및 필수 도구 설치

```bash
sudo apt update && sudo apt install -y locales curl gnupg lsb-release software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2.2 ROS2 Jazzy apt 설치

```bash
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-jazzy-robot-state-publisher python3-rosdep python3-colcon-common-extensions python3-vcstool ros-jazzy-cv-bridge ros-jazzy-image-transport
```

### 2.3 환경 설정

```bash
# .bashrc에 추가
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
source ~/.bashrc

# rosdep 초기화
sudo rosdep init
rosdep update
```

### 2.4 설치 확인

```bash
ros2 --version
ros2 run demo_nodes_cpp talker
# 다른 터미널에서
ros2 run demo_nodes_py listener
```

---

## Phase 3: 개발 워크스페이스 생성

### 3.1 ROS2 워크스페이스

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 필수 패키지 클론
git clone https://github.com/rpng/open_vins.git
git clone https://github.com/ori-drs/allan_variance_ros.git

cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select ov_msckf ov_eval allan_variance_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 3.2 자체 프로젝트 패키지 생성

```bash
cd ~/catkin_ws/src
ros2 pkg create --build-type ament_python smart_trash_bringup
ros2 pkg create --build-type ament_python smart_trash_sensors
ros2 pkg create --build-type ament_python smart_trash_motion
ros2 pkg create --build-type ament_python smart_trash_voice

colcon build --packages-select smart_trash_bringup smart_trash_sensors smart_trash_motion smart_trash_voice
source install/setup.bash
```

---

## Phase 4: 센서 드라이버 설치

### 4.0 LD06 2D LiDAR (Phase 0 메인 SLAM 센서)

```bash
# 시리얼 포트 권한 (최초 1회, 재로그인 필요)
sudo usermod -aG dialout $USER

ls -la /dev/ttyUSB*

cd ~/catkin_ws/src
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
cd ~/catkin_ws
colcon build --packages-select ldlidar_stl_ros2
source install/setup.bash

# 테스트 (product_name=LDLiDAR_LD06으로 launch 파일 설정 후)
ros2 launch ldlidar_stl_ros2 ld06.launch.py
ros2 topic echo /scan
```

### 4.1 [선택] 카메라 (IMX219, VSLAM 서브 옵션)

> Phase 0은 LD06 LiDAR가 메인 SLAM 센서라 카메라 구매/설치가 필수는 아님. 추후 VSLAM 보조 검토 시에만 진행.

```bash
sudo apt install -y rpicam-apps libcamera-dev

# 테스트
rpicam-hello -t 0

# ROS2 카메라 노드
sudo apt install -y ros-jazzy-camera-ros

# 또는 picamera2 기반 노드
sudo apt install -y python3-picamera2
```

### 4.2 IMU (ICM-20948, Qwiic)

```bash
sudo apt install -y python3-smbus2

# Python으로 직접 읽기 (테스트용)
# 또는 ROS2 패키지 사용
```

### 4.3 ToF 센서 (VL53L1X / VL53L0X)

```bash
sudo pip3 install vl53l1x
sudo pip3 install vl53l0x
```

### 4.4 모터 드라이버 (Yahboom)

```bash
# Yahboom UART 통신 테스트
python3 -m pip install pyserial

# /dev/ttyAMA0 또는 /dev/ttyS0 확인
ls -la /dev/tty*
```

---

## Phase 5: ATS 서버 개발 환경 구축

### 5.1 Ubuntu 24.04 + ROS2 Jazzy

서버가 이미 Ubuntu 24.04라면 Pi와 동일하게 설치:

```bash
# ROS2 Jazzy Desktop 설치 (서버는 GUI도 OK)
sudo apt install -y ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-rviz2

# 추가 패키지
sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-robot-localization

# FastDDS Unicast 설정을 위한 xml
mkdir -p ~/.ros
cat > ~/.ros/fastdds_unicast.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_transport</transport_id>
      <type>UDPv4</type>
      <sendBufferSize>1048576</sendBufferSize>
      <receiveBufferSize>1048576</receiveBufferSize>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="unicast_profile" is_default_profile="true">
    <rtps>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator>
            <udpv4>
              <address>0.0.0.0</address>
              <port>7412</port>
            </udpv4>
          </locator>
        </metatrafficUnicastLocatorList>
        <initialPeersList>
          <locator>
            <udpv4>
              <address>PI_5_IP_ADDRESS</address>
              <port>7412</port>
            </udpv4>
          </locator>
        </initialPeersList>
      </builtin>
      <transportDescriptors>
        <transportDescriptor>udp_transport</transportDescriptor>
      </transportDescriptors>
    </rtps>
  </participant>
</profiles>
EOF

echo "export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_unicast.xml" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

### 5.2 Docker/Podman 권장 (선택)

```bash
# ROS2 개발용 Dockerfile 예시
mkdir -p ~/smart-trash-docker
cat > ~/smart-trash-docker/Dockerfile << 'EOF'
FROM ros:jazzy-ros-base-noble

RUN apt update && apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-navigation2 \
    ros-jazzy-rviz2 \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install fastapi uvicorn paho-mqtt

WORKDIR /app
CMD ["bash"]
EOF

docker build -t smart-trash-server ~/smart-trash-docker
```

### 5.3 Python 가상환경 및 FastAPI

```bash
mkdir -p ~/smart-trash-server
cd ~/smart-trash-server
python3 -m venv venv
source venv/bin/activate

pip install fastapi uvicorn websocket-client paho-mqtt faster-whisper piper-tts opencv-python
```

---

## Phase 6: GitHub 프로젝트 생성 및 연동

### 6.1 GitHub 저장소 생성

1. https://github.com/new 접속
2. 저장소명: `smart-trash-bin`
3. Public 또는 Private 선택
4. README 초기화

### 6.2 로컬 코드 푸시

```bash
# 서버에서
mkdir -p ~/smart-trash-bin
cd ~/smart-trash-bin
git init
git remote add origin https://github.com/YOUR_USERNAME/smart-trash-bin.git

# .gitignore 생성
cat > .gitignore << 'EOF'
__pycache__/
*.pyc
*.pyo
*.pyd
.Python
*.so
*.egg
*.egg-info/
dist/
build/
install/
log/
test_results/
*.bag
.vscode/
.idea/
EOF

# 기본 구조
cat > README.md << 'EOF'
# 스마트 쓰레기통

## 개요
집안 각 방의 이동형 쓰레기통이 음성/앱 명령으로 집합하고, 비우기 후 원위치 복귀하는 자율 로봇 시스템.

## 구성
- `robot/`: Raspberry Pi 5 로컬 코드 (ROS2 패키지)
- `server/`: ATS 중앙 서버 코드 (FastAPI + SLAM/Nav2)
- `app/`: Flutter 모바일 앱
- `docs/`: 문서 및 설계도
- `hardware/`: 회로도, 부품 리스트

## 시작하기
- `docs/setup_pi5.md`: Pi 5 설정 가이드
- `docs/setup_server.md`: 서버 설정 가이드
EOF

git add .
git commit -m "Initial commit: project structure and setup docs"
git push -u origin main
```

### 6.3 Pi 5에서도 동일한 저장소 클론

```bash
# Pi 5에서
cd ~
git clone https://github.com/YOUR_USERNAME/smart-trash-bin.git
cd smart-trash-bin
```

---

## Phase 7: 서버 ↔ Pi 5 연결 및 배포

### 7.1 Pi 5 고정 IP 설정

```bash
# /etc/netplan/00-installer-config.yaml 예시
sudo nano /etc/netplan/00-installer-config.yaml
```

```yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: true
  wifis:
    wlan0:
      dhcp4: no
      addresses: [192.168.1.101/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      access-points:
        "YOUR_WIFI_SSID":
          password: "YOUR_WIFI_PASSWORD"
```

```bash
sudo netplan apply
```

### 7.2 FastDDS Unicast 설정 (양쪽)

Pi 5와 서버 양쪽에 `~/.ros/fastdds_unicast.xml` 배치.

### 7.3 ROS2 토픽 테스트

```bash
# Pi 5에서os2 run demo_nodes_cpp talker --ros-args --remap __ns:=/robot_1

# 서버에서
ros2 run demo_nodes_py listener --ros-args --remap __ns:=/robot_1
```

### 7.4 코드 배포 스크립트

서버에서 작성한 코드를 Pi 5로 배포하는 간단한 스크립트:

```bash
# ~/smart-trash-bin/deploy.sh
cat > ~/smart-trash-bin/deploy.sh << 'EOF'
#!/bin/bash
# 로컬 변경사항 커밋/푸시
cd ~/smart-trash-bin
git add .
git commit -m "deploy update $(date +%Y%m%d_%H%M%S)"
git push origin main

# Pi 5에서 pull (SSH key 설정 필요)
ssh ubuntu@192.168.1.101 'cd ~/smart-trash-bin && git pull origin main && source /opt/ros/jazzy/setup.bash && colcon build'
EOF

chmod +x ~/smart-trash-bin/deploy.sh
```

---

## Phase 8: 첫 번째 동작 테스트

### 8.1 센서 데이터 확인

```bash
# Pi 5에서
cd ~/catkin_ws
source install/setup.bash
ros2 launch smart_trash_sensors bringup.launch.py

# 서버에서
ros2 topic list
ros2 topic echo /robot_1/imu/data
ros2 topic echo /robot_1/camera/image_raw
ros2 topic echo /robot_1/scan
```

### 8.2 모터 제어 테스트

```bash
# Pi 5에서
cd ~/catkin_ws
source install/setup.bash
ros2 run smart_trash_motion test_motor
```

### 8.3 SLAM Toolbox 실행 (Pi 또는 서버)

```bash
cd ~/catkin_ws
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```

> [선택] 카메라 기반 VSLAM(OpenVINS)을 보조로 검토하는 경우: `ros2 launch ov_msckf subscribe.launch.py config:=robot_config.yaml`

---

## 전체 흐름 요약

```
[Pi 5 구매]
    ↓
[Ubuntu 24.04 설치] → SSH 설정, config.txt 수정, I2C/UART/카메라 활성화
    ↓
[ROS2 Jazzy 설치] → 워크스페이스 생성, OpenVINS 빌드
    ↓
[센서/모터 드라이버 설치] → IMX219, ICM-20948, VL53L1X, Yahboom
    ↓
[GitHub 저장소 생성] → 프로젝트 구조 커밋
    ↓
[ATS 서버 개발환경] → ROS2 + FastAPI + faster-whisper + Docker
    ↓
[Pi 5 ↔ 서버 연결] → 고정 IP, FastDDS Unicast, ROS2 네임스페이스
    ↓
[코드 배포 스크립트] → git pull + colcon build
    ↓
[첫 통합 테스트] → 센서 데이터 → 서버 SLAM → 모터 제어
```

---

## 다음 Action Items

- [ ] Pi 5 발주 및 도착
- [ ] microSD/USB SSD 준비
- [ ] GitHub 저장소 생성
- [ ] Ubuntu 24.04 flash
- [ ] SSH 접속 확인
- [ ] I2C/UART/카메라 활성화
- [ ] ROS2 Jazzy 설치
- [ ] OpenVINS 클론/빌드
- [ ] 서버 Docker 이미지 빌드
- [ ] FastDDS Unicast 설정
- [ ] 첫 ROS2 talker/listener 통신 테스트
