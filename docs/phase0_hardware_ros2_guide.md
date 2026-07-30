# 스마트 쓰레기통 Phase 0 부품 리스트 및 ROS2 개발 환경 구축 가이드

> **버전**: LD06 2D LiDAR + ICM-20948 IMU (Qwiic) + 4WD 엔코더 모터 기반 2D SLAM (SLAM Toolbox / Nav2)  
> **작성일**: 2026-07-28 (업데이트)

---

## 1. Phase 0 부품 리스트 (2대 기준)

### 1.1 필수 부품 (1대당)

> **구매 현황 범례**: ✅ 배송완료(입수) / 🚚 배송대기(국내) / 🚚🌊 해외배송대기(약 2주 소요, LiDAR·모터/섀시)

| # | 부품 | 추천 모델/사양 | 수량/1대 | 역할 | 국내 구매처 예시 | 대략 단가(원) | 비고 |
|---|------|----------------|---------|------|----------------|------------|------|
| 1 | **Raspberry Pi 5** | 4GB RAM + 액티브 쿨러 | 1대 | 메인 컴퓨터, SLAM, 통신, 모터 제어 | 디바이스마트, 엘레파츠, ICBanQ | ₩186,000 | ✅ **배송완료** |
| 2 | **저장매체** | **최종용: Samsung T7 128GB microSD (A2)** / **개발용: 외장 USB SSD 128GB** | 1개 | OS 저장 | 삼성/온라인 | 실구매가 미기입 | ✅ **배송완료(둘 다 확보).** 개발 1호기는 현재 **외장 SSD로 부팅 중**(`/dev/sda2`) — 소스 빌드 시 microSD의 속도/수명 문제를 피하기 위함. **개발 완료 후 최종 microSD로 전환 예정** |
| 3 | **LD06 2D LiDAR** | 12m 거리 / 360° / **DTOF 방식**, USB 어댑터 포함 | 1개 | **2D SLAM 메인 센서, 맵핑, 장애물 인식** | 디바이스마트, AliExpress | **₩42,000** | 🚚🌊 **해외배송대기** (약 2주 후 도착 예정) |
| 4 | **IMU (9축)** | **ICM-20948 (Qwiic)** | 1개 | 자세 추정, Odom 융합 | 디바이스마트, AliExpress | ₩15,000 | ✅ **배송완료.** 내일 Pi 5와 연결 테스트 예정 |
| 5 | **Qwiic 연결 케이블** | STEMMA QT / Qwiic JST SH 4pin Cable | 1개 | IMU ↔ Pi 5 연결 | 디바이스마트, AliExpress | ₩3,000~5,000 | 🚚 **배송대기** |
| 6 | **Raspberry Pi Camera (선택)** | Camera Module 2 (IMX219) + 광각 렌즈 or Arducam 8MP IMX219 175° | 0~1개 | VSLAM 서브 옵션, 마커 인식 | 디바이스마트, Amazon/AliExpress | ₩25,000~50,000 | **Phase 0 필수 아님 — LD06으로 대체됨.** 미구매 |
| 7 | **카메라 케이블 (선택)** | Pi 5용 FFC 케이블 (22핀, 30cm 이상) | 0~1개 | 카메라 연결 | 디바이스마트 | ₩3,000~5,000 | 카메라를 구매하는 경우에만 필요, 미구매 |
| 8 | **ToF 거리센서 (장애물 보조)** | VL53L1X 모듈 (최대 4m) | 2~3개 | LiDAR 사각지대(저상 장애물 등) 보조 감지 | 디바이스마트, AliExpress | ₩3,000~6,000/개 | 미구매 |
| 9 | **DC 모터 + 4WD 섀시 (통합 키트)** | **4WD 로봇 자동차 섀시 카트 370 DC WITH ENCODER** (모터 4개 + 섀시 일체) | 1세트 | 4WD 차동 구동, odom 피드백, 본체 프레임 | [AliExpress JGA25-370 4WD encoder kit](https://ko.aliexpress.com/item/1005012268741388.html) | ₩45,000~95,000 | 🚚🌊 **해외배송대기** (약 2주 후 도착 예정). PH2.0 6핀 양단 케이블 — 도착 시 멀티미터 도통 테스트 필수(3.8절 참고) |
| 10 | **모터 드라이버** | **Yahboom 4-Channel Encoder Motor Drive Module (P016551737)** | 1개 | 4개 DC 모터 + 4채널 인코더 통합 제어 | ICBanQ P016551737 | ₩37,620 | 🚚 **배송대기.** STM32 보조 프로세서, UART 통신 |
| 11 | **TB6612FNG 듀얼 모터 드라이버 (예비/폴백)** | keyestudio TB6612FNG | 1개 | 모터 2개 제어용 폴백 | ICBanQ P008202439 | ₩5,000~10,000 | 미구매 |
| 12 | **휴지통 본체** | 기성 10~20L 휴지통 | 1개 | 쓰레기통 본체 | 다이소/마트/온라인 | ₩15,000~30,000 | 미구매 |
| 13 | **쓰레기 채움 센서** | VL53L0X 모듈 | 2개 | 쓰레기 높이 측정 | 디바이스마트, AliExpress | ₩3,000~5,000/개 | 미구매 |
| 14 | **마이크 모듈** | INMP441 MEMS I2S 마이크 | 1개 | 음성 트리거/확인 | AliExpress, Devicemart | ₩3,000~5,000 | 미구매 |
| 15 | **스피커 (옵션)** | 3W 4Ω 미니 스피커 + 앰프 (PAM8302A 등) | 1세트 | TTS 응답/알림 | AliExpress, Devicemart | ₩5,000~10,000 | 미구매 |
| 16 | **배터리팩** | **12V 3500mAh 18650 리튬이온 (DC배럴 + XT60 커넥터, 12V1A 전용 충전기 포함)** | 1개 | 모바일 전원 | 배터리 전문 쇼핑몰, AliExpress | 실구매가 미기입 | 🚚 **배송대기.** ⚠️ BMS 내장 여부 확인 필요(3S 셀 보호 필수). 12V1A 충전 시 완충까지 4시간 이상 소요 예상 |
| 17 | **DC-DC 스텝다운 컨버터 (Buck)** | **DC 9-36V 입력 → USB 5V 5A 출력** 모듈, **동기식(Synchronous)** | 2개 | Pi 5 및 센서용 안정적 5V | - | 실구매가 미기입 | 🚚 **배송대기.** ✅ 동기식 확인 완료 — LM2596/XL4015 대비 발열/전류 부족 위험 낮음 |
| 18 | **평활 콘덴서** | **E/C 16V 1000µF (85도)** | 1대당 1개 (10개 구매, 예비 포함) | Pi 5 입력단 전압 급변 완화 | 전자부품 쇼핑몰 | 실구매가 미기입 | ✅ 스펙 부합(1000µF/16V). 🚚 **배송대기** |
| 19 | **DC-DC 컨버터 (모터용)** | XL4015 또는 LM2596 | 0~1개 | 모터 전압 강하 필요 시 조정 | AliExpress, Devicemart | ₩3,000~5,000 | 12V 배터리 직결로 불필요할 가능성 높음, 미구매 |
| 20 | **퓨즈 세트** | **자동차용 퓨즈홀더(CAR) 2개 + 자동차 퓨즈 세트** (20A 모터용, 5A Pi용) | 1세트 | 과전류 보호 | 전자부품 쇼핑몰 | 실구매가 미기입 | 🚚 **배송대기** |
| 21 | **배선재 (전원/신호선)** | **16AWG 검/빨 10M(전원선), 22AWG 6색 10M(신호선)** | 1세트 | 배터리·모터 등 전원 배선 + 센서 신호선 | 전자부품 쇼핑몰 | 실구매가 미기입 | 🚚 **배송대기.** 브레드보드/점퍼선 대체 가능 |
| 22 | **USB Wi-Fi 동글 (옵션)** | 내장 Wi-Fi 불안 시 5GHz USB 동글 | 0~1개 | 무선 통신 안정화 | 온라인 마트 | ₩10,000~20,000 | 미구매 |
| 23 | **ESP32-S3 (음성 폴백용)** | DevKitC-1 또는 유사 보드 | 1개 | wake word + 명령 폴백 | AliExpress, Devicemart | ₩10,000~18,000 | 미구매 |

### 1.2 2대 기준 총 예산

| 항목 | 1대 예상 비용 | 2대 예상 비용 |
|------|--------------|--------------|
|| 필수 핵심 부품 (Pi 5 + **LD06 LiDAR** + IMU + 4WD 메탈 기어 모터 + Yahboom 드라이버 + 4WD 섀시 + 배터리) | ₩378,000~471,000 | ₩756,000~942,000 |
|| 센서/주변기기 (ToF, 마이크, 스피커, ESP32, 기타) | ₩40,000~80,000 | ₩80,000~160,000 |
|| **총계** | **₩418,000~551,000** | **₩836,000~1,102,000** |

> **목표 예산**: **약 ₩84~110만원대**. LD06(₩42,000 고정가)이 카메라 대비 저렴해 기존 추정(₩76~118만원)보다 다소 절감. 카메라/VSLAM 부품(#6~7)을 뺀 실질 지출이라 상한선도 낮아짐. 키트/할인/중고로 추가 조정 가능.  
> **1대 먼저 구매**하여 LiDAR SLAM과 이동 검증 후 2대째 복제 권장.

---

## 1.5 3S 분리형 전원부 설계 (최종 확정)

```mermaid
graph TD
    BAT["18650 3S팩 (BMS 내장)<br/>11.1V, 3000mAh"] -->|커넥터 탈착용| SW["메인 로커 스위치"]
    
    SW --> F20["퓨즈 20A"]
    SW --> F5["퓨즈 5A"]
    
    F20 -->|12V 그대로| YAH["Yahboom 드라이버<br/>(스크류 터미널 연결)"]
    YAH --> M["모터 ×4<br/>(PH2.0 6핀 엔코더 포함)"]
    
    F5 --> BUCK["벅컨버터 5V 5A<br/>(12V → 5V 변환)<br/>+ 콘덴서 1000µF"]
    BUCK --> RPI["라즈베리파이 5<br/>(GPIO 5V핀 또는 USB-C)"]
    RPI --> SENSORS["LD06 / IMU<br/>(Pi에서 5V/3.3V 공급)"]
    
    YAH <.-.->|UART| RPI
```

### 전원부 핵심 규칙

| 항목 | 설정 | 이유 |
|------|------|------|
| **배터리** | 18650 3S (11.1V~12.6V) | 모터 기동 전압 강하를 Pi 5가 보지 않도록 충분한 마진 확보 |
| **선로 분리** | 모터 전원과 Pi 전원을 BMS 직후 분리 | 모터 Inrush Current로 Pi 5 리셋 방지 |
| **Buck Converter** | **동기식 정류 5V 5A** (MP2482/TPS54560 기반) | LM2596/XL4015는 발열/전류 부족으로 Pi 5 불안정 |
| **평활 콘덴서** | 470µF~1000µF / 16V, Pi 5 5V 입력단 | 잔류 전압 변동 흡수 |
| **퓨즈** | 모터 선 20A, Pi 선 5A | 과전류 보호 |
| **JGA25-370 모터 전압 확인** | 12V용인지 6V용인지 구매 전 확인 | 12V용이면 3S 직결 가능. 6V용이면 별도 강하 필요 |

### Pi 5 전류 제한 해제

`/boot/firmware/config.txt`에 아래 추가:

```ini
PSU_MAX_CURRENT_3000=5000
usb_max_current_enable=1
```

> Pi 5는 전용 27W PSU가 아닌 일반 5V 입력 시 USB 포트 출력을 600mA로 자동 제한. 위 설정으로 센서/ESP32-S3 등에 충분한 전력 공급.

---

## 2. 구매 링크 및 쇼핑 후보 (참고)

> 가격과 재고는 변동되므로, 구매 전 최신 가격 확인 필요.

| 부품 | 추천 구매처 예시 |
|------|----------------|
| Raspberry Pi 5 | [RPiLocator](https://rpilocator.com), [디바이스마트](https://www.devicemart.co.kr), [엘레파츠](https://www.eleparts.co.kr), [ICBanQ](https://www.icbanq.com) |
| **LD06 2D LiDAR (₩42,000, 구매 완료)** | 디바이스마트, AliExpress |
| Pi Camera Module 2 (IMX219) / Arducam 광각 (선택, VSLAM 서브 옵션) | 디바이스마트, AliExpress, Amazon |
| ICM-20948 (Qwiic, 구매 완료) | 디바이스마트, AliExpress, Amazon |
| VL53L1X / VL53L0X | 디바이스마트, AliExpress |
|| DC 모터 + 엔코더 + 섀시 | [AliExpress JGA25-370 4WD encoder kit](https://ko.aliexpress.com/item/1005012268741388.html), Devicemart 4WD smart car kit |
|| 모터 드라이버 | **ICBanQ P016551737 (Yahboom 4-Channel Encoder Motor Driver)** |
|| TB6612FNG 폴백 | ICBanQ P008202439 |
| INMP441 | AliExpress, 디바이스마트 |
| 18650 배터리/BMS | 국내 배터리 전문 쇼핑몰, AliExpress |
| 휴지통 | 다이소, 온라인 마트 |

---

## 3.2-LiDAR Phase 0: LD06 + SLAM Toolbox 실전 설정 (메인 경로)

### 3.2-LiDAR.1 LD06 드라이버 설치

LD06은 USB 어댑터를 통해 시리얼로 연결되며, LDROBOT 공식 ROS2 드라이버(`ldlidar_stl_ros2`)를 사용한다.

```bash
cd ~/catkin_ws/src
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
cd ~/catkin_ws
colcon build --packages-select ldlidar_stl_ros2
source install/setup.bash

# 시리얼 포트 확인 (보통 /dev/ttyUSB0)
ls -la /dev/ttyUSB*

# 포트 권한 (dialout 그룹 추가, 최초 1회)
sudo usermod -aG dialout $USER
# 재로그인 필요
```

`ldlidar_stl_ros2`의 launch 파일에서 `product_name`을 `LDLiDAR_LD06`으로, `port_name`을 확인된 포트로 설정 후 실행:

```bash
ros2 launch ldlidar_stl_ros2 ld06.launch.py
ros2 topic echo /scan
```

### 3.2-LiDAR.2 SLAM Toolbox 설치 및 실행

```bash
sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-robot-localization

# 온라인 비동기 SLAM (실시간 맵핑)
ros2 launch slam_toolbox online_async_launch.py
```

`/scan`(LD06) + `/imu/data`(ICM-20948, `robot_localization` EKF로 odom과 융합) 토픽을 SLAM Toolbox가 구독해 `/map`을 생성한다. Nav2와 연동 시 `ros-jazzy-navigation2 ros-jazzy-nav2-bringup` 추가 설치.

> Rpi 5 자체에서 SLAM Toolbox를 돌려도 되고, 부하가 크면 LD06/IMU 데이터만 Pi에서 퍼블리시하고 SLAM Toolbox는 ATS 서버에서 원격 실행하는 방식으로 분리 가능 (overview.md 아키텍처 참고).

---

## 3.2 [선택 / VSLAM 서브 옵션] IMX219 + OpenVINS 실전 설정

> **Phase 0 메인 경로는 위 LD06 + SLAM Toolbox입니다.** 아래 카메라 기반 VSLAM 설정은 추후 보조 옵션 검토 시에만 참고하세요 (구매 필수 아님).

### 3.2.1 Rolling Shutter 보정

IMX219는 롤링 셔터 센서로, 로봇 회전/진동 시 Jello Effect와 기하 왜곡이 발생할 수 있음. OpenVINS 설정 파일(`kalibr_imx219.yaml`)에 readout time 적용.

1280×720 @ 30fps 기준:

```yaml
rolling_shutter_readout_time: 0.032
```

### 3.2.2 카메라 노출/게인 고정

OpenVINS 특징점 추적 시 밝기 변화를 위치 이동으로 오인하지 않도록 자동 노출(Auto Exposure)과 오토 화이트 밸런스(AWB)를 끄고 수동값 고정.

```bash
# libcamera 예시
rpicam-still --shutter 10000 --gain 2.0 --awbgains 1.5,1.5
```

또는 ROS2 카메라 노드에서 `AeEnable=0`, `AwbEnable=0`, `ExposureTime`, `AnalogueGain` 파라미터 설정.

### 3.2.3 IMU 주파수 매칭

| 센서 | 권장 주파수 | 이유 |
|------|------------|------|
| IMU (Accel/Gyro) | **200Hz** | 가공 부하 대비 정확도 양호 |
| 카메라 | **30fps** | IMU 대비 1:6~7 비율 유지 |

> MPU9250/ICM-20948의 실제 최대 주파수는 센서/라이브러리에 따라 1kHz까지 가능하지만, OpenVINS에서 200Hz가 안정적이고 충분한 성능을 냄.

---

## 3. ROS2 개발 환경 구축 가이드

### 3.1 환경 선택

| 대상 | OS | ROS2 버전 | 이유 |
|------|-----|----------|------|
| **Raspberry Pi 5** | Ubuntu 24.04 LTS Server/Desktop | **ROS 2 Jazzy Jalisco** | Ubuntu 24.04 공식 Tier 1 지원 |
| **ATS 서버** | Ubuntu 24.04 LTS | **ROS 2 Jazzy Jalisco** | Pi와 동일 버전으로 멀티 로봇/네트워크 통일 |

> **중요**: Raspberry Pi **Camera Module 3 (IMX708)**은 Ubuntu 24.04 + libcamera에서 ROS2 `camera_ros`와의 호환 이슈가 보고됨. 이 프로젝트에서는 **Camera Module 2 (IMX219)** 또는 Arducam IMX219 광각 모듈을 우선 권장. libcamera/rpicam-apps를 소스 빌드하면 Module 3도 가능하나, 중학생 팀의 초기 난이도를 고려하면 IMX219가 안정적.  
> **참고**: Ubuntu 22.04 + Humble도 가능하지만, Jazzy가 Pi 5와 Ubuntu 24.04에 최적화되어 있음.

---

### 3.2 Raspberry Pi 5에 Ubuntu 24.04 + ROS2 Jazzy 설치

#### Step 1: Ubuntu 24.04 이미지 준비

1. Raspberry Pi Imager 또는 [Ubuntu Raspberry Pi 이미지](https://ubuntu.com/download/raspberry-pi) 다운로드
2. microSD(64GB 이상) 또는 USB SSD에 flash
3. Pi 5에 부팅, Wi-Fi/Ethernet 설정, 사용자 생성
4. SSH 활성화

```bash
# Pi에서 네트워크 확인
ip addr
sudo apt update && sudo apt upgrade -y
```

#### Step 2: ROS2 Jazzy 설치 (apt)

```bash
# locale 설정
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 필수 도구
sudo apt install -y software-properties-common curl gnupg lsb-release

# ROS2 apt 저장소 추가
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 설치
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-base python3-rosdep python3-colcon-common-extensions python3-vcstool

# 환경 설정
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# rosdep 초기화
sudo rosdep init
rosdep update
```

#### Step 3: 설치 확인

```bash
ros2 --version
ros2 run demo_nodes_cpp talker
# 다른 터미널에서
ros2 run demo_nodes_py listener
```

---

### 3.3 ATS 서버에 ROS2 Jazzy 설치

ATS 서버가 이미 Ubuntu 24.04라면 Pi와 동일한 명령으로 설치.

```bash
# Docker/Podman이 있다면 컨테이너에서도 가능
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-base python3-rosdep python3-colcon-common-extensions

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**추가 권장**: `ros-jazzy-navigation2`, `ros-jazzy-slam-toolbox`, `ros-jazzy-rviz2`

```bash
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-rviz2
```

---

### 3.4 멀티 로봇 통신 설정

ROS2 Jazzy는 DDS 기반. 동일 Wi-Fi 네트워크에 있으면 자동 디스커버리됨.

#### 네임스페이스 분리 (필수)

각 로봇은 고유 네임스페이스를 가져야 토픽 충돌 방지.

```bash
# robot_1의 launch 예시
export ROS_NAMESPACE=robot_1
ros2 launch my_robot robot.launch.py namespace:=robot_1

# robot_2
export ROS_NAMESPACE=robot_2
ros2 launch my_robot robot.launch.py namespace:=robot_2
```

#### DOMAIN_ID 분리 (필요 시)

서버가 Gemma4 등 다른 서비스와 충돌 우려 시:

```bash
export ROS_DOMAIN_ID=42
```

#### FastDDS Unicast 설정 (Phase 0 Wi-Fi 안정화)

기본 DDS 멀티캐스트는 가정 Wi-Fi에서 패킷 손실/지연이 발생하기 쉬움. Pi 5와 ATS 서버 양쪽에 FastDDS Unicast 프로필을 적용.

**`~/.ros/fastdds_unicast.xml`**:

```xml
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
              <address>192.168.1.10</address>
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
```

> `192.168.1.10`은 ATS 서버 고정 IP로 변경. Pi에서는 서버 IP를 initial peer로 지정.

**환경변수**:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_unicast.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

**QoS 권장 설정**:

```python
# ROS2 Python 예시
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

---

### 3.5 [선택 / VSLAM 서브 옵션] VSLAM 패키지 설치

> Phase 0 메인 경로(LD06 + SLAM Toolbox)에는 필요 없음. 카메라 기반 VSLAM을 보조로 검토할 때만 참고.

#### OpenVINS (ov_msckf) — 권장 시작점

```bash
# 종속성
sudo apt install -y cmake libeigen3-dev libboost-all-dev libopencv-dev

# ROS2 workspace 생성
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/rpng/open_vins.git

# ROS2 빌드
cd ~/catkin_ws
colcon build --packages-select ov_msckf ov_eval --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

#### 카메라 + IMU ROS2 노드 작성

IMX219 + MPU9250 데이터를 `/camera/image_raw` 및 `/imu/data` 토픽으로 퍼블리시하는 노드가 필요.

```bash
# 예시: picamera2 (또는 v4l2) + smbus2 기반 ROS2 노드
# 자체 패키지: smart_trash_sensors
```

#### ORB-SLAM3 (ROS2 wrapper)

```bash
cd ~/catkin_ws/src
git clone https://github.com/zang09/ORB_SLAM3_ROS2.git
cd ..
colcon build --packages-select orbslam3 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> **참고**: ORB-SLAM3는 빌드가 무거움. Pi 5에서도 가능하나 첫 빌드에 30분~1시간 이상 소요.

### 3.8 IMU 캘리브레이션

`allan_variance_ros`로 IMU 노이즈(white noise, random walk) 측정. **LD06+SLAM Toolbox 경로에서도 `robot_localization` EKF의 IMU 노이즈 파라미터 튜닝에 필요**하므로 VSLAM 여부와 무관하게 진행 권장.

```bash
cd ~/catkin_ws/src
git clone https://github.com/ori-drs/allan_variance_ros.git
cd .. && colcon build --packages-select allan_variance_ros
```

> [선택 / VSLAM 서브 옵션] 카메라-IMU 외부 파라미터(extrinsic)까지 필요하면 **Kalibr**(Ubuntu 24.04에서 Docker 권장) 추가 사용. OpenVINS 튜토리얼: https://docs.openvins.com/gs-calibration.html

---

### 3.6 카메라 및 센서 드라이버

#### Raspberry Pi Camera Module 3 (libcamera / rpicam)

Ubuntu 24.04에서는 `rpicam-apps` 사용:

```bash
sudo apt install -y rpicam-apps libcamera-dev

# 테스트
rpicam-hello -t 0
```

ROS2용 카메라 노드:

```bash
sudo apt install -y ros-jazzy-camera-ros
# 또는 picamera2 기반 커스텀 노드 빌드
```

#### IMU (MPU9250 / ICM-20948)

```bash
sudo pip3 install smbus2
# ROS2용 imu_publisher 패키지 설치 또는 직접 작성
```

#### VL53L1X / VL53L0X

```bash
sudo pip3 install vl53l1x
# 또는 VL53L0X Python 라이브러리
```

---

### 3.7 Docker 컨테이너 구성 (ATS 서버 권장)

Gemma4와 격리하여 쓰레기통 서비스를 운영하기 위해 Docker/Podman 권장.

```dockerfile
# Dockerfile 예시: smart-trash-server
FROM ros:jazzy-ros-base-noble

RUN apt update && apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-navigation2 \
    ros-jazzy-rviz2 \
    python3-pip \
    mosquitto-clients \
    && rm -rf /var/lib/apt/lists/*

# FastAPI, MQTT broker, custom packages 추가 가능
RUN pip3 install fastapi uvicorn paho-mqtt

WORKDIR /app
COPY . /app

CMD ["bash"]
```

```bash
# 빌드 및 실행
docker build -t smart-trash-server .
docker run -it --network host --name smart-trash-server smart-trash-server
```

---

### 3.8 ⚠️ [주의] 370 메탈 엔코더 모터 PH2.0 핀배열 멀티미터 도통 테스트

PH2.0 6핀 커넥터가 모터 및 Yahboom 드라이버와 물리적으로 조립되더라도, **제조사/배치에 따라 핀 배열(M+, M-, VCC, GND, A상, B상)이 다를 수 있습니다.** 만약 12V 모터 전원 선(M+/M-)이 엔코더 VCC(5V) 선으로 들어갈 경우 엔코더 센서 IC가 소손(태움)됩니다.

**멀티미터 필수 점검 절차**:
1. **저항(Resistor) / 도통 모드 측정**:
   - 6개 핀 중 두 핀 간 저항이 몇 옴(Ω)~몇십 옴(Ω) 수준으로 일정하게 측정되는 쌍이 바로 **모터 코일 핀 (M+, M-)**입니다. (나머지 엔코더 핀들과는 절연 상태여야 함)
2. **엔코더 VCC / GND 핀 확인**:
   - 모터 섀시 데이터시트를 참조하여 VCC/GND 핀에 5V가 제대로 인가되는지, 12V 모터 전원 단자와 쇼트되어 있지 않은지 확인합니다.
3. **핀 재압착/교체**:
   - 핀 순서가 드라이버와 일치하지 않으면 핀 픽(또는 정밀 드라이버)으로 PH2.0 커넥터 리테이너를 들어올려 핀 순서를 맞춘 후 재삽입합니다.

---

## 4. 개발 워크플로우 제안

1. **Week 1**: 부품 구매 + Pi 5에 Ubuntu 24.04 + ROS2 Jazzy 설치
2. **Week 2**: LD06 LiDAR + IMU 연결, SLAM Toolbox로 1개 방 맵핑
3. **Week 3**: DC 모터 + 엔코더 + Yahboom 드라이버 연결, diff-drive ROS2 노드 작성
4. **Week 4**: VL53L1X 장애물 감지, 간단한 집합/복귀 동작
5. **Week 5**: 2대 구성, 멀티 로봇 네임스페이스, 서버 연동
6. **Week 6**: 음성 파이프라인, 모바일 앱 상태 연동
7. **Week 7~**: 퍼포먼스 동작 녹화-재생, 군무 설계

---

## 5. 다음 Action Items

- [ ] 부품 리스트 최종 검토 및 구매처/가격 확정
- [ ] Raspberry Pi 5 2대 + 주변기기 발주
- [ ] Ubuntu 24.04 microSD 또는 USB SSD 준비
- [ ] ATS 서버에 ROS2 Jazzy 설치 및 Docker 환경 구성
- [ ] LD06 LiDAR + IMU + 모터 드라이버 실물 연결 테스트
- [ ] SLAM Toolbox 빌드 및 1개 방 맵핑 시도
- [ ] IMU 캘리브레이션 (allan_variance_ros)
- [ ] diff-drive ROS2 노드 템플릿 작성
- [ ] 2대 조립 후 기본 집합/복귀 테스트

---

*참고: 본 문서는 2026년 7월 기준 가격과 사용 가능한 소프트웨어를 기준으로 작성. 구매/설치 전 최신 정보 확인 필요.*
