# 스마트 쓰레기통 (Smart Trash Bin)

집안 각 방의 이동형 쓰레기통이 음성/앱 명령으로 집합 장소에 모이고, 비우기·봉투 교체 완료 후 원래 방으로 복귀하는 자율 로봇 시스템. 추가로 웰컴/군무 등 이벤트성 퍼포먼스 동작을 학습·재생.

## 구성

- `robot/`: Raspberry Pi 5 ROS2 워크스페이스 (`~/catkin_ws`는 이 폴더를 가리키는 심볼릭 링크). LD06 드라이버는 [`carrotion/ldlidar_stl_ros2`](https://github.com/carrotion/ldlidar_stl_ros2) fork를 git submodule로 포함
- `server/`: 중앙 서버 코드 (FastAPI + SLAM/Nav2)
- `app/`: 모바일 앱
- `docs/`: 프로젝트 문서 (개요, 조사 보고서, 설치 가이드, 부품 리스트)
- `hardware/`: 회로도, 배선도, 부품 리스트

> ⚠️ 이 저장소를 새로 클론할 때는 서브모듈까지 같이 받아야 함:
> `git clone --recurse-submodules https://github.com/carrotion/smart-trash-bin.git`
> (이미 클론했다면 `git submodule update --init`)

## 시작하기

- [`docs/overview.md`](docs/overview.md): 프로젝트 개요 및 아키텍처
- [`docs/schedule.md`](docs/schedule.md): **주간 일정 및 팀 체크리스트 (최신)**
- [`docs/research_report.md`](docs/research_report.md): 기초 조사 및 설계 보고서
- [`docs/setup_guide_full.md`](docs/setup_guide_full.md): Pi 5 + 서버 개발 환경 구축 가이드
- [`docs/phase0_hardware_ros2_guide.md`](docs/phase0_hardware_ros2_guide.md): Phase 0 부품 리스트 및 ROS2 환경 구축 가이드

## Phase 0 하드웨어 요약

Raspberry Pi 5 4GB + LD06 2D LiDAR + ICM-20948 IMU (Qwiic) + 4WD 370 엔코더 모터/섀시 + Yahboom 4채널 드라이버 기반 2D SLAM(SLAM Toolbox + Nav2). 2대 기준 PoC로 시작.
