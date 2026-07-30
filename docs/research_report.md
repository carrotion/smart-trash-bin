# 스마트 쓰레기통 (Smart Trash Bin) — 기초 조사 및 설계 보고서

> 작성일: 2026-07-26  
> 프로젝트 단계: Phase 0 / PoC 설계  
> 대상 독자: 중학생 팀원 및 멘토  

> ⚠️ **업데이트 노트 (2026-07-28)**: 본 보고서는 카메라+IMU 기반 VSLAM을 메인으로 조사한 자료입니다. 이후 **LD06 2D LiDAR(실구매가 ₩42,000)** 가격/성능이 확인되면서 **LiDAR + SLAM Toolbox 방식으로 최종 전환**되었습니다 (상세는 [`overview.md`](overview.md) 8절, [`phase0_hardware_ros2_guide.md`](phase0_hardware_ros2_guide.md) 참고). 아래 3장의 VSLAM/OpenVINS 비교 조사는 **배경 연구 자료로는 유효**하지만, Phase 0 실제 구현 경로는 LiDAR 기반이며 카메라/VSLAM은 서브 옵션으로 격하되었습니다.

---

## 1. Executive Summary

본 보고서는 **방마다 배치된 이동형 스마트 쓰레기통이 음성 트리거로 집합 장소로 모였다가, 비우기 및 봉투 교체 후 원위치로 복귀**하는 시스템을 구현하기 위한 기초 조사 결과를 정리한 것이다. 추가로 **웰컴/군무 등 이벤트성 퍼포먼스 동작**을 단계적으로 학습·재생하는 방안도 포함한다.

**핵심 결론 (2026-07-28 업데이트: LiDAR 전환 반영)**
- **하드웨어**: Raspberry Pi 5 4GB + **LD06 2D LiDAR(₩42,000)** + ICM-20948 IMU(Qwiic) + DC 모터(엔코더) + VL53L1X/0X + ESP32-S3 폴백으로 구성. (카메라는 Phase 0 필수 아님, VSLAM 서브 옵션으로만 검토)
- **SLAM/네비게이션**: **LD06 LiDAR 기반 2D SLAM**(SLAM Toolbox) + **IMU 융합(robot_localization)** + **Nav2**. ~~LiDAR 없이 카메라+IMU 기반 VSLAM~~ → 아래 3장 비교조사 결과 및 실구매가를 반영해 LiDAR로 전환, 주행 안정성 대폭 상승.
- **중앙 처리**: 기존 ATS 서버(12코어 AMD, RTX 5060 Ti ×2)에서 ROS2 백엔드/SLAM/AI/음성을 Docker 컨테이너로 운영.
- **음성**: 서버 기반 STT(TTS)를 기본으로 하되, 통신 장애 시 ESP32-S3에서 **웨이크워드("스마트 쓰레기통")**와 **"모여"/"돌아가"** 같은 소수 명령만 오프라인 처리.
- **멀티 로봇 회피**: 실시간 위치 공유 기반 **병렬 이동**. 군무 동작을 위해 단순 순차 이동이 아닌 동시 제어가 핵심.
- **예산**: 2대 기준 **₩52~82만원** 목표.

---

## 2. 시장/학술 사례 조사

### 2.1 상용 제품

| 제품/회사 | 개요 | 특징 | 가격/상태 |
|-----------|------|------|-----------|
| **SmartCan (Rezzi)** | 쓰레기통 밑에 모터/바퀴를 부착해 자율으로 집밖 쓰레기 수거점까지 이동 | 스마트폰 앱 스케줄링, 도킹 스테이션 | 2019년 콘셉트, 베타 진행 중 |
| **AI-powered trash can (소셜미디어 사례)** | 카메라로 날아오는 쓰레기를 추적해 받아내는 로봇 휴통 | 딥러닝 기반 궤적 예측 | DIY/프로토타입 수준 |

### 2.2 학술/교육 프로젝트

| 제목 | 핵심 내용 | 참고 |
|------|-----------|------|
| "Smart Autonomous Multi-Purpose Trash Picking Robot" (AJAST) | 바퀴 달린 쓰레기통, 내비게이션/장애물 회피/제스처 인식 | Raspberry Pi 3 + OpenCV |
| "Autonomous Garbage Collection Robot" (IJRASET) | 도시 환경에서 자율로 쓰레기 수거 | 센서퓨전 기반 |
| Cornell 로봇 쓰레기통 (NYC 실험) | 사람-로봇 상호작용(HRI) 연구 | 상용화보다 사회적 수용성 연구 |
| "BALL IS LIFE" trash can | Kinect + ROS + AR 마커, 던져진 쓰레기 받기 | ROS, AR 태그 기반 위치 추정 |

**교훈**: 이동형 쓰레기통은 상용화보다 **학교/메이커 프로젝트에서 활발히 시도**되며, 핵심은 **로봇 청소기와 유사한 저가 모바일 플랫폼 + SLAM/네비게이션 + 사용자 편의 인터페이스** 조합이다.

---

## 3. VSLAM/네비게이션 기술 비교

### 3.1 LiDAR 없이 카메라+IMU로 할 수 있는가?

**가능하다.** ROS2 커뮤니티와 여러 프로젝트에서 **monocular Visual-Inertial Odometry (VIO)** 또는 **Visual SLAM (VSLAM)**로 실내 이동 로봇을 구현한 사례가 많다. 다만 **카메라 단독 모노 VSLAM은 절대 스케일(크기) 모호성**이 있어 IMU(가속도+자이로)를 함께 쓰는 **VIO**가 권장된다.

### 3.2 주요 SLAM/VIO 라이브러리 비교

| 항목 | OpenVINS (`ov_msckf`) | ORB-SLAM3 | RTAB-Map |
|------|------------------------|-----------|----------|
| **입력** | 모노/스테레오 + IMU | 모노/스테레오/RGB-D + IMU | 모노/스테레오/RGB-D/LiDAR |
| **ROS2 지원** | 공식 Jazzy 지원 | ROS2 wrapper 존재 (빌드 무거움) | ROS2 지원, Nav2 연동 우수 |
| **맵 형태** | 희소 3D 포인트 | 희소 3D 포인트/키프레임 | 점구름/메쉬/점유격자 |
| **Pi 5 적합성** | 가볍고 빠름, 권장 시작점 | 무거우나 정확, 빌드 30분~1시간 | 기능이 많으나 리소스 부담 |
| **장점** | 가벼움, 칼리브레이션 문서 풍부 | 다양한 센서 모드, 루프 클로저 | 2D/3D 지도보내기, Nav2 연동 |
| **단점** | 루프 클로저 약함 | 빌드 복잡, 무거움 | Pi 5에서 과부하 가능 |
| **권장 용도** | **Phase 0 메인** | 정확도 검증/백업 | 추후 확장 또는 서버에서 실행 |

### 3.3 카메라 선택의 중요성

- **Raspberry Pi Camera Module 3 (IMX708)**: Ubuntu 24.04 + libcamera + ROS2 `camera_ros` 조합에서 **호환성 이슈**가 다수 보고됨. 시야각이 넓고 화질은 좋으나, 초기 빌드/설정 난이도가 높음.
- **Raspberry Pi Camera Module 2 (IMX219)**: Ubuntu 24.04에서 상대적으로 안정적. **Arducam 8MP IMX219 175° 광각 모듈** 등으로 시야각 확보 가능.
- **권장**: 중학생 팀의 **Phase 0에서는 IMX219 기반 광각 모듈** 사용. libcamera/rpicam-apps를 소스 빌드하면 Module 3도 가능하나, 이는 추후 과제로.

### 3.4 카메라-IMU 칼리브레이션

VSLAM/VIO의 정확도는 **카메라 내부 파라미터 + 카메라-IMU 외부 파라미터 + IMU 노이즈 파라미터**에 크게 의존한다.

- **Kalibr**: 카메라-IMU 칼리브레이션의 표준. Ubuntu 24.04에서는 Docker 사용 권장.
- **allan_variance_ros**: IMU 노이즈(white noise, random walk) 측정용 ROS2 패키지.
- OpenVINS 공식 튜토리얼: https://docs.openvins.com/gs-calibration.html

**Phase 0 체크리스트**: IMU 30분~2시간 정적 데이터 수집 → Allan variance로 노이즈 추정 → Kalibr로 카메라-IMU 외부 파라미터 추정 → OpenVINS config에 반영.

### 3.5 장애물 회피 (LiDAR 없이)

Nav2는 기본적으로 **LaserScan** 또는 **PointCloud2** 입력을 받는다. 카메라만으로는 다음 옵션이 있다.

| 방법 | 원리 | 난이도 | 실용성 |
|------|------|--------|--------|
| **VL53L1X ToF 센서 × N개** | 직접 거리 측정 → LaserScan 발행 | 낮음 | **Phase 0 권장 폴백** |
| **Depth Anything V3 + ROS2** | 단안 카메라로 깊이 추정 → PointCloud2 | 중간 | RTX 5060 Ti 서버에서 실행 가능 |
| **ORB-SLAM3 희소 맵 기반** | 키포인트 거리 추정 | 높음 | PoC에는 무리 |

**권장**: VL53L1X 2~3개로 전방/측방 장애물 감지 + Nav2 Voxel/Obstacle Layer에 LaserScan 토픽 투입. Depth Anything V3는 서버 GPU에서 병행 테스트.

---

## 4. 멀티 로봇 군무/퍼포먼스 기술

### 4.1 멀티 로봇 시뮬레이션

- **Gazebo Harmonic + ROS2 Jazzy**: 공식 권장 시뮬레이터. TurtleBot3, Jackal 등 여러 플랫폼에서 멀티 로봇 네임스페이스 테스트 가능.
- **ROS2swarm**: ROS2 기반 스웜 행동 라이브러리. attraction, dispersion, aggregation 등 기본 패턴 제공. TurtleBot3 Waffle Pi/Burger, Jackal UGV에서 검증됨.
- **Multi-TurtleBot3 Gazebo**: GitHub `arshadlab/tb3_multi_robot` 등에서 Nav2를 포함한 멀티 로봇 Gazebo 프레임워크 제공.

### 4.2 충돌 회피 알고리즘

| 알고리즘 | 개요 | 적용성 |
|----------|------|--------|
| **Velocity Obstacles (VO)** | 상대 속도 공간에서 충돌 가능 영역 제거 | 이론적 기반 |
| **Reciprocal Velocity Obstacles (RVO)** | 양쪽 에이전트가 회피를 분담 | 실시간 멀티 에이전트 |
| **Optimal Reciprocal Collision Avoidance (ORCA)** | RVO의 선형 근사, 빠르고 안정 | 로봇/게임/영화 군무 |

**현실적 접근**: Nav2 기반 경로 계획 + 실시간 위치 공유로 **각 로봇의 목표 속도를 중앙 서버에서 조정**. ORCA 구현은 시뮬레이션에서 먼저 검증 후, 단순화된 버전(목표 지점까지 거리 기반 속도 조절 + 상대 위치 기반 회피)을 하드웨어에 적용.

### 4.3 퍼포먼스 동작 학습 방안

단계적으로 접근해야 중학생 팀원이 성취감을 느끼며 진행할 수 있다.

| 레벨 | 방법 | 출력 | 난이도 |
|------|------|------|--------|
| **Level 1: 조종 녹화-재생** | 조이스틱/앱으로 수동 조종 시 `cmd_vel` 시퀀스를 `ros2 bag`으로 녹화 → 재생 | 고정 동작 반복 | 낮음 |
| **Level 2: 키프레임 시퀀스** | 집합 장소, 특정 위치 등을 키프레임으로 저장 → 시간/트리거 기반 재생 | 웰컴/군무 동작 | 중간 |
| **Level 3: 시뮬레이션 최적화** | Gazebo에서 동작을 파라미터 최적화 → 실기 이전 | 고급 군무 | 높음 |

**군무 동작 예시**: 집합 장소에 로봇들이 일정 간격으로 들어와 정렬 → 동일한 회전/전진 패턴 수행 → 각자 홈 포지션으로 복귀. 이를 키프레임 JSON으로 정의하면 모든 로봇이 동일한 동작을 동기화된 시간에 실행.

---

## 5. 음성 트리거 아키텍처

### 5.1 전체 흐름

```
[사용자] --음성--> [마이크 INMP441]
                   ├─> [ESP32-S3 웨이크워드] "스마트 쓰레기통" 감지
                   │       └─> Wi-Fi로 Pi/서버에 wake 이벤트 전송
                   └─> [Pi 5] 오디오 버퍼 → [ATS 서버 STT]
                               └─> 명령("모여", "돌아가", "다 했어") 인식
                               └─> TTS 응답 또는 동작 실행
```

### 5.2 STT (Speech-to-Text) 옵션

| 엔진 | 특징 | 한국어 | Pi 5 적합성 | 서버 적합성 |
|------|------|--------|-------------|-------------|
| **Whisper (faster-whisper)** | OpenAI Whisper 고성능 재구현 | 기본 large-v3 권장, small도 가능 | CPU 7,200초/시간 (느림) | **RTX 5060 Ti에서 실시간 권장** |
| **Vosk** | Kaldi 기반 오프라인 STT | 한국어 모델 존재 | Raspberry Pi에서도 가능 | 무난 |
| **Whisper.cpp** | C++ 포팅, 가벼움 | 지원 | Pi 5에서 실시간 가능 | 가능 |

**권장**: 서버에서 **faster-whisper small/base** 실행 → Pi는 오디오 스트리밍만 담당. 통신 단절 시 ESP32-S3의 **microWakeWord**로 "모여"/"돌아가" 소수 명령만 처리.

### 5.3 웨이크워드/엣지 폴백

- **microWakeWord**: ESP32-S3에서 동작하는 커스텀 웨이크워드. microwakeword.com에서 "스마트 쓰레기통" 등 한국어 문구도 TTS 샘플 생성 후 학습 가능.
- **Edge Impulse**: ESP32-S3용 keyword spotting 모델 학습. klumw/wakeword GitHub 템플릿 참고.
- **Vosk on ESP32**: 보다 복잡한 명령 인식 가능하나 리소스 부담.

**Phase 0 권장**: ESP32-S3에 microWakeWord로 **"스마트 쓰레기통"** 감지 → Pi喚醒 → 서버 STT로 나머지 명령 처리.

### 5.4 TTS (Text-to-Speech)

| 엔진 | 특징 | 한국어 | 적합성 |
|------|------|--------|--------|
| **Piper TTS** | VITS 기반, CPU 실시간, 초경량 | 영어 메인, 한국어 모델 제한적 | **Pi 5/서버 모두 우수** |
| **Mimic3 (Mycroft)** | 자연스러운 음성 | 다국어 지원 | 서버 권장 |
| **gTTS/네이버 클라우드** | 클라우드 기반 | 한국어 우수 | 인터넷 필요 |

**권장**: 영어 응답은 Piper, 한국어 응답은 **네이버 클로바 TTS API** 또는 **gTTS** 폴백. 음성 응답이 필수가 아니라면 **LED/앱 알림**으로 대체 가능.

---

## 6. 백엔드/앱 아키텍처

### 6.1 시스템 구성도

```
┌─────────────────────────────────────────────────────────────┐
│                       ATS 서버 (Ubuntu 24.04)                │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │ ROS2 Jazzy   │  │ Faster-Whisper│  │ FastAPI + WebSocket│  │
│  │ (SLAM/Nav2)  │  │   (STT/TTS)   │  │   (Fleet API)    │  │
│  │  Docker      │  │    Docker     │  │     Docker       │  │
│  └──────┬───────┘  └──────┬───────┘  └────────┬─────────┘  │
└─────────┼─────────────────┼─────────────────────┼───────────┘
          │                 │                     │
          │   Wi-Fi / DDS Discovery Server     │
          │                 │                     │
┌─────────┴─────────┐  ┌────┴──────────────┐  ┌──┴─────────────┐
│   Robot #1 (Pi5)   │  │   Robot #2 (Pi5)   │  │   Mobile App    │
│  VSLAM/Nav2/센서   │  │  VSLAM/Nav2/센서   │  │  Flutter/Web   │
│  ESP32-S3 폴백     │  │  ESP32-S3 폴백     │  │  WebSocket      │
└────────────────────┘  └────────────────────┘  └────────────────┘
```

### 6.2 ROS2 멀티 로봇 통신

- **DDS Discovery Server**: Fast DDS의 Discovery Server 모드를 사용하면 Pi 5 2대와 서버가 멀티캐스트 없이 중앙에서 discovery. Wi-Fi 환경에서 안정적.
- **네임스페이스**: 각 로봇을 `/smartbin_1`, `/smartbin_2`로 격리. 공통 맵(`/map`)과 상태는 공유.
- **rosbridge**: Flutter/Web 앱이 ROS2 토픽을 JSON/WebSocket으로 직접 구독/발행할 수 있음. 다만 보안/안정성을 위해 FastAPI 게이트웨이 권장.

### 6.3 FastAPI 백엔드

| 기능 | API/WebSocket | 설명 |
|------|---------------|------|
| 로봇 상태 조회 | GET /robots | 위치, 배터리, 채움량, 동작 상태 |
| 명령 발행 | POST /command/{robot_id} | "gather", "return", "pause", "perform" |
| 실시간 상태 | WebSocket /ws/status | 모든 로봇 상태 push |
| 퍼포먼스 트리거 | POST /performance/{routine_id} | 웰컴/군무 루틴 시작 |
| 음성 처리 | POST /stt (파일), POST /tts (텍스트) | 서버 STT/TTS |

### 6.4 모바일 앱 (Flutter 권장)

| 화면 | 기능 |
|------|------|
| **홈** | 전체 로봇 위치/상태, 채움량, 배터리 |
| **수동 제어** | 조이스틱, "모두 모여", "모두 돌아가" |
| **일정/알림** | 쓰레기 비우기 시간, 채움량 임박 알림 |
| **퍼포먼스** | 웰컴/군무 루틴 선택 및 실행 |
| **설정** | 로봇 등록, 집합 장소 지정, 음성 명렘 설정 |

---

## 7. 전원/배터리 및 안전

### 7.1 전원 소비 예산 (1대 기준)

| 부품 | 대략 소비 전력 | 비고 |
|------|----------------|------|
| Raspberry Pi 5 4GB | idle ~2.7W, full load 10~12W | VSLAM 실행 시 6~8W 예상 |
| DC 모터 2개 (주행) | 2~10W (부하에 따라 변동) | 정지 시 거의 0 |
| VL53L1X ×3 + VL53L0X ×2 | 0.1~0.3W | I2C 저전력 |
| IMU, ESP32-S3, 기타 | 0.5~1W | |
| **총계 (이동 중)** | **8~18W** | |
| **총계 (대기)** | **3~5W** | |

### 7.2 배터리 구성 예시

| 구성 | 용량 | 전압 | Pi 5 단독 사용 시 | 이동 중 사용 시 |
|------|------|------|-------------------|-----------------|
| 18650 2S1P (2개 직렬) | 3000mAh | 7.4V | ~3~4시간 | ~1~1.5시간 |
| 18650 2S2P (4개) | 6000mAh | 7.4V | ~6~8시간 | ~2~3시간 |
| 18650 3S2P (6개) | 6000mAh | 11.1V | 더 긴 모터 구동 | ~3~4시간 |

**권장**: 2S2P(4개) 또는 3S2P(6개) + **BMS(과충전/과방전/과전류/단락 보호)** + **DC-DC buck**로 5V/모터 전원 분리.

### 7.3 안전 규칙

1. **정격 BMS 사용**: 셀 수(2S/3S)와 최대 전류(모터 스톨 전류의 2배 이상)에 맞는 BMS.
2. **배터리 박스 격리**: 금속 접촉 방지, 충격/습기 방지 케이스.
3. **충전 시 감시**: 불연성 표면(금속/콘크리트) 위에서 충전, 충전 중 unattended 금지.
4. **퓨즈 추가**: BMS와 로봇 메인 사이에 적절한 퓨즈.
5. **온도 모니터링**: 배터리 온도가 45°C 이상 상승 시 사용 중지.

---

## 8. Phase 0 부품 리스트 (2대 기준)

> 상세는 `phase0_hardware_ros2_guide.md` 참고. 여기서는 핵심만 요약.

| 부품 | 권장 모델 | 수량 | 역할 | 대략 단가 |
|------|-----------|------|------|-----------|
| Raspberry Pi 5 4GB | 공식 27W PSU 포함 | 2 | 메인 컴퓨터 | ₩20.4~21만/대 |
| microSD 64GB A2 | 삼성/샌디스크/공식 | 2 | OS 저장 | ₩1.5~3만/개 |
| Camera Module 2 / Arducam IMX219 광각 | IMX219 센서 기반 | 2 | VSLAM | ₩2.5~5만/개 |
| 9축 IMU | MPU9250/ICM-20948 | 2 | VIO | ₩0.5~1.5만/개 |
| VL53L1X ToF | 4m 거리 | 5~6 | 장애물 폴백 | ₩0.3~0.6만/개 |
| VL53L0X ToF | 2m 거리 | 4 | 쓰레기 채움 | ₩0.3~0.5만/개 |
| DC 모터 + 엔코더 + 2WD 섀시 | 6V~12V 키트 | 2세트 | 이동 | ₩3~5만/세트 |
| 모터 드라이버 | TB6612FNG | 2 | 모터 구동 | ₩0.5~1.5만/개 |
| ESP32-S3 DevKitC | DevKitC-1 | 2 | 음성 폴백 | ₩1~1.8만/개 |
| INMP441 I2S 마이크 | MEMS | 2 | 음성 입력 | ₩0.3~0.5만/개 |
| 배터리팩 + BMS | 18650 2S2P/3S2P | 2세트 | 전원 | ₩2.5~4.5만/세트 |
| DC-DC buck | LM2596/XL4015 | 2 | 5V 변환 | ₩0.3~0.5만/개 |
| 휴통 본체 | 10~20L | 2 | 쓰레기통 | ₩1.5~3만/개 |
| 기타(브레드보드, 점퍼선, 휴즈 등) | | | | ₩1~2만 |

**2대 총 예산**: **₩52~82만원** (목표 ₩50~80만원)

---

## 9. 개발 로드맵 (8주 기준)

| 주차 | 목표 | 산출물 |
|------|------|--------|
| **Week 1** | 부품 구매 + Pi 5 Ubuntu 24.04 + ROS2 Jazzy 설치 | 설치된 Pi 2대 |
| **Week 2** | LD06 LiDAR + IMU 연결, SLAM Toolbox로 1개 방 맵핑 | LiDAR SLAM 데모 영상 |
| **Week 3** | DC 모터 + 엔코더 + Yahboom 드라이버 연결, diff-drive ROS2 노드 | 이동 가능한 1대 |
| **Week 4** | VL53L1X 장애물 감지, 간단한 집합/복귀 동작 | 자율 이동 PoC |
| **Week 5** | 2대 구성, 멀티 로봇 네임스페이스, 서버 연동 | 2대 동시 제어 |
| **Week 6** | 음성 파이프라인, 모바일 앱 상태 연동 | 음성 호출 데모 |
| **Week 7~** | 퍼포먼스 동작 녹화-재생, 군무 설계 | 웰컴/군무 시연 |

---

## 10. 위험 요소 및 대응

| 위험 | 영향 | 대응 |
|------|------|------|
| LiDAR SLAM이 유리 벽/좁은 복도 등에서 드리프트 | 위치 정확도 저하 | IMU 캘리브레이션 철저, ArUco 마커로 홈/집합 장소 보정 |
| Pi 5 전원 부족 | 갑작스러운 재부팅 | 공식 27W PSU 사용, 배터리팩 용량 충분히 확보 |
| Wi-Fi 지연/단절 | 음성/명령 지연 | ESP32-S3 웨이크워드 + 소수 명령 폴백 |
| 멀티 로봇 충돌 | 물리적 손상 | 초기에는 순차 이동 모드 제공, 병렬 모드는 시뮬레이션 검증 후 적용 |
| 배터리 화재 | 안전 사고 | 정격 BMS/퓨즈, 충전 시 감시, 불연성 충전 공간 |
| 팀 역량/시간 부족 | 전체 구현 실패 | Phase 0에서 2대로 범위 축소, 핵심 기능(집합/복귀) 우선 |

---

## 11. 참고 자료

- OpenVINS: https://docs.openvins.com
- ORB-SLAM3 ROS2: https://github.com/zang09/ORB_SLAM3_ROS2
- ROS2 Jazzy on Raspberry Pi: https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html
- Nav2 Depth Anything V3: https://docs.nav2.org/tutorials/docs/depth_ai_integration.html
- ROS2swarm: https://github.com/ROS2swarm
- microWakeWord: https://microwakeword.com
- Faster-Whisper: https://github.com/SYSTRAN/faster-whisper
- Piper TTS: https://github.com/rhasspy/piper
- ROS2 multi-robot namespace: https://github.com/arshadlab/tb3_multi_robot

---

*보고서 끝*
