# CLAUDE.md — 스마트 쓰레기통 프로젝트 환경 메모

Claude Code가 이 저장소에서 작업할 때마다 참고해야 하는 환경 특이사항과 확정 결정사항. 컨텍스트가 압축되어도 여기 내용은 다시 읽고 재발견 없이 이어서 작업할 것.

## ✅ Ubuntu 24.04 재설치 완료 (2026-08-01)

**배경**: 개발 1호기가 원래 Ubuntu 26.04(Resolute)라 ROS2 Jazzy(24.04 전용)와 안 맞아서 하루에 3번(rclpy 파이썬 버전, cv_bridge의 boost-python, nav2-map-server의 graphicsmagick) 크로스레포 충돌이 났음. 26.04 네이티브 배포판 "Lyrical Luth"는 나온 지 3개월밖에 안 돼서 slam_toolbox/navigation2가 아직 바이너리로 없어 전환 불가 판단. 그래서 사용자가 로컬 PC에서 Raspberry Pi Imager로 외장 SSD에 **Ubuntu Server 24.04 LTS(64-bit)를 새로 구워 재설치**함.

**재설치 후 진행 및 결과 (2026-08-01, 이 세션에서 확인)**:
1. ROS2 apt 레포: `noble` 레포 하나만 등록되어 있음 확인 (resolute 우회용 레포 불필요해짐)
2. `ros-jazzy-ros-base` 등 201개 패키지 이미 설치돼 있었음
3. **python3.12 venv/wrapper 불필요 확인**: 시스템 기본 `python3`가 3.12라 `import rclpy` 바로 동작. `~/ros2_venv`, `~/.local/bin/ros2` wrapper 둘 다 없음(정상) — **다시 만들지 말 것**
4. git 설정 완료: user.email `59335136+carrotion@users.noreply.github.com`, user.name `carrotion`
5. **인증 방식은 SSH 키로 변경**(이전 문서엔 `gh auth login`/https로 되어 있었으나, 이번 Pi5에선 이 세션에서 새로 `~/.ssh/id_ed25519` 생성 후 GitHub 계정에 등록해 인증함 — `gh` CLI는 설치 안 함). 저장소는 `git@github.com:carrotion/smart-trash-bin.git`로 clone
6. `git submodule update --init --recursive`로 `ldlidar_stl_ros2` 서브모듈 확보 완료
7. `~/catkin_ws` → `~/smart-trash-bin/robot` 심볼릭 링크 재생성 완료
8. rosdep: `/etc/ros/rosdep/sources.list.d/20-default.list` 이미 초기화돼 있었음. `rosdep install --from-paths src --ignore-src -r -y`로 slam_toolbox 등 의존성 정상 설치
9. **`colcon build` 최초 시도는 실패함 — 원인은 `g++`(C++ 컴파일러) 자체가 이미지에 없었기 때문** (`gcc`만 있고 `g++`는 없는 상태, `CMAKE_CXX_COMPILER` not found 에러). `sudo apt-get install -y build-essential`로 해결. **새 Pi5 이미지를 또 굽게 되면 이 패키지가 기본 설치돼 있는지 먼저 확인할 것**
10. build-essential 설치 후 `colcon build --symlink-install` **5개 패키지 전부 빌드 성공** (`ldlidar_stl_ros2`의 pthread.h 패치도 fork에 그대로 살아있어 문제 없었음)
11. **Nav2 전체 설치 확인 완료 — 예상대로 24.04에서는 충돌 없음**: `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`, `ros-jazzy-nav2-map-server`, `ros-jazzy-cv-bridge` 전부 정상 설치됨 (`dpkg -l`로 4개 패키지 존재 확인). 26.04에서 막혔던 boost-python/graphicsmagick 크로스버전 충돌 완전히 해소됨

**다음에 할 일**: 실제 하드웨어(LD06, IMU, 모터 드라이버) 연결 및 각 패키지(`smart_trash_sensors`/`smart_trash_motion`/`smart_trash_voice`) 노드 구현은 아직 시작 전. `docs/schedule.md`의 Week 진행 상황과 대조해서 다음 작업 확인할 것.

## 환경

- 개발 1호기: **Raspberry Pi 5 (4GB)**, Ubuntu **24.04.4 LTS (Noble Numbat)**, 커널 raspi. (2026-08-01 이전엔 26.04였다가 재설치함 — 위 "Ubuntu 24.04 재설치 완료" 섹션 참고)
- **ROS 2 Jazzy**는 공식적으로 Ubuntu 24.04(Noble)/Python 3.12 타겟이고, 이 시스템의 기본 `python3`가 정확히 3.12라 별도 venv/wrapper 없이 `import rclpy`가 바로 동작함. 26.04 시절 썼던 `~/ros2_venv`/`~/.local/bin/ros2` wrapper 우회는 더 이상 필요 없고 만들지 말 것.
- 빌드에 `build-essential`(특히 `g++`) 필요 — 이 Pi5 이미지엔 기본으로 없어서 최초 `colcon build` 시 `CMAKE_CXX_COMPILER` 에러로 실패했었음. `sudo apt-get install -y build-essential`로 해결. 새 이미지로 또 재설치하게 되면 이 패키지부터 확인할 것.
- 개발 중 부팅 디스크는 **외장 USB SSD 128GB**(`/dev/sda2`)를 사용 (microSD 쓰기속도/수명 문제 회피). 최종 배포용 128GB A2 microSD는 별도 확보돼 있음 — 개발 완료 후 전환 예정.
- GitHub: `carrotion/smart-trash-bin` (private). **인증은 SSH 키**(`~/.ssh/id_ed25519`, 2026-08-01 이 Pi5에서 새로 생성 후 GitHub 계정에 등록) — `gh` CLI는 설치돼 있지 않음, git 명령은 `git@github.com:...` SSH remote로 동작. git 커밋 이메일은 `59335136+carrotion@users.noreply.github.com`.
- **설치 완료 (2026-08-01, 24.04에서 전부 재확인)**: `ros-jazzy-ros-base`, `ros-jazzy-slam-toolbox`, Nav2 전체(`ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`, `ros-jazzy-nav2-map-server`), `ros-jazzy-cv-bridge`. 26.04에서 겪었던 boost-python/graphicsmagick 크로스버전 충돌은 24.04에서 재현되지 않음 — Nav2 관련 막힘 이슈는 해소된 것으로 간주.

## 워크스페이스

- ROS2 워크스페이스의 실제 위치는 **`~/smart-trash-bin/robot/`** (git 저장소 안, 버전관리됨). `~/catkin_ws`는 여기를 가리키는 **심볼릭 링크**일 뿐 (`ln -s ~/smart-trash-bin/robot ~/catkin_ws`) — 문서에 있는 `~/catkin_ws` 명령어들은 그대로 다 써도 됨. `build/`, `install/`, `log/`는 `.gitignore`에 걸려 있어 커밋 안 됨.
- 자체 패키지 4개(ament_python, `robot/src/` 밑): `smart_trash_bringup`, `smart_trash_sensors`, `smart_trash_motion`, `smart_trash_voice`. `colcon build` 정상 동작 확인됨(2026-08-01).
- **LD06 드라이버**는 `robot/src/ldlidar_stl_ros2`에 **git submodule**로 연결됨, 가리키는 저장소는 `carrotion/ldlidar_stl_ros2` (공식 `ldrobotSensorTeam/ldlidar_stl_ros2`를 fork). **fork에 패치 커밋 완료**: `ldlidar_driver/src/logger/log_module.cpp`에 `#include <pthread.h>` 추가 (누락 시 최신 GCC 15.2.0/Ubuntu 26.04에서 빌드 실패). 로컬 패치가 아니라 fork에 정식 커밋+푸시했으니 재클론해도 유지됨.
- ⚠️ **이 저장소를 새로 클론할 때는 `git clone --recurse-submodules ...` 또는 클론 후 `git submodule update --init`을 꼭 해야 `robot/src/ldlidar_stl_ros2` 내용이 채워짐.**

## 알려진 미해결 이슈

- 현재 없음. (26.04 시절의 python3.12 런타임 의존성/Nav2 크로스버전 충돌 이슈는 24.04 재설치로 해소됨 — 위 "Ubuntu 24.04 재설치 완료" 섹션 참고)

## 확정된 하드웨어/설계 결정

- **SLAM**: LD06 2D LiDAR(DTOF, 12m/360°) + ICM-20948 IMU 기반, **SLAM Toolbox + Nav2**. 카메라 기반 VSLAM은 사용 안 함(서브 옵션으로만 문서에 남겨둠).
- **전원부**: 12V 3500mAh 18650(BMS 내장) → 메인 로커스위치 → 20A(모터)/5A(Pi) 독립 퓨즈 분기 → 모터는 Yahboom 드라이버로, Pi는 **동기식** 5V5A 벅컨버터+콘덴서 경유. 전원선 AWG16, 센서 신호선 AWG22.
- 상세 부품/가격/배송 현황은 `docs/phase0_hardware_ros2_guide.md`, 예산은 `docs/overview.md` 9.1절 참고.
- **최신 주간 일정/팀 체크리스트는 `docs/schedule.md`가 항상 최신** — 다른 문서의 로드맵 섹션은 참고용으로만 남아있음.

## 작업 방식

- 문서 수정 후에는 커밋 메시지를 남기고 `git push origin main`까지 완료할 것 (사용자가 매번 명시적으로 요청하지 않아도 이 저장소에서는 문서 변경 후 커밋/푸시가 관례로 자리잡음).
