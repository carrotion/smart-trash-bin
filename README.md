# 스마트 쓰레기통 (Smart Trash Bin)

집안 각 방의 이동형 쓰레기통이 음성/앱 명령으로 집합 장소에 모이고, 비우기·봉투 교체 완료 후 원래 방으로 복귀하는 자율 로봇 시스템. 추가로 웰컴/군무 등 이벤트성 퍼포먼스 동작을 학습·재생.

## 구성

- `robot/`: Raspberry Pi 5 ROS2 워크스페이스 (`~/catkin_ws`는 이 폴더를 가리키는 심볼릭 링크). LD06 드라이버는 [`carrotion/ldlidar_stl_ros2`](https://github.com/carrotion/ldlidar_stl_ros2) fork를 git submodule로 포함
- `server/`: 중앙 서버 코드 (FastAPI + SLAM/Nav2)
- `app/`: 모바일 앱
- `docs/`: 프로젝트 문서 (개요, 조사 보고서, 설치 가이드, 부품 리스트)
- `hardware/`: 회로도, 배선도, 부품 리스트
- `tools/`: Pi5 개발기 진단용 웹 대시보드 4종(전원/모터/IMU/LiDAR, systemd 서비스로 상시 실행) — 아래 "진단 대시보드" 참고

## 진행상황 (2026-08-15 기준)

Phase 0 개발 1호기(Pi 5) 하드웨어가 전부 도착해 검증 단계. 상세 근거/절차는 `CLAUDE.md`에 계속 누적 기록 중이며, 여기는 요약만.

- ✅ **환경**: Ubuntu 24.04 + ROS2 Jazzy, 4개 자체 패키지 빌드 정상
- ✅ **IMU(ICM-20948)**: GPIO I2C1 연결, `/imu/data` 실측 검증 완료
- ✅ **LD06 LiDAR**: GPIO UART(`/dev/ttyAMA0`) 연결, 상판 거꾸로 마운트 방향 캘리브레이션 완료, `/scan` 10Hz 정상 발행
- ✅ **모터 드라이버(Yahboom 4채널)**: 4바퀴 전부 방향·엔코더·정지 실측 검증 완료 (감속비 21.3:1 JGA25-370 확정)
- ✅ **SLAM Toolbox 파이프라인**: LD06+SLAM Toolbox 기동까지 에러 없이 확인 — 실제 맵 생성/저장은 아직
- ⏳ **다음 작업**: `smart_trash_motion`(diff-drive + 휠 오도메트리) 노드 → 실제 맵핑 주행

### 진단 대시보드

개발 중 Pi5 상태를 브라우저로 바로 보기 위한 웹 대시보드 4종, 전부 systemd 서비스로 등록돼 재부팅해도 자동 시작됨(`tools/systemd/*.service`).

| 대시보드 | 포트 | 내용 |
|---|---|---|
| 허브 | 8764 | 전체 모듈 요약, 클릭하면 아래 상세로 이동 |
| Pi | 8765 | CPU 온도/사용률/클럭/메모리/디스크/업타임 + PMIC 레일 전압, 저전압·스로틀링 이벤트 로그 |
| 모터 | 8766 | 4바퀴 엔코더 상태 시각화 + 방향키(레버로 속도 조절) 수동 조작 |
| IMU/LiDAR | 8767 | 가속도/자이로, LiDAR 포인트 수·거리 실시간 |
| 맵 | 8768 | SLAM Toolbox가 만드는 `/map`을 실시간 렌더링 (맵핑 미실행 시 대기 상태) |

`http://<Pi IP>:8764`(허브)로 접속하면 각 모듈의 핵심 지표 3~4개를 카드로 한눈에 보고, 카드를 클릭하면 해당 상세 대시보드로 이동.

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
