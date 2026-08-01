# CLAUDE.md — 스마트 쓰레기통 프로젝트 환경 메모

Claude Code가 이 저장소에서 작업할 때마다 참고해야 하는 환경 특이사항과 확정 결정사항. 컨텍스트가 압축되어도 여기 내용은 다시 읽고 재발견 없이 이어서 작업할 것.

## 환경

- 개발 1호기: **Raspberry Pi 5 (4GB)**, Ubuntu **26.04 LTS (Resolute Raccoon)**, 커널 raspi
- **ROS 2 Jazzy**는 공식적으로 Ubuntu 24.04(Noble)/Python 3.12 타겟. 이 시스템은 26.04라 기본 `python3`가 **3.14**를 가리켜서 rclpy 등 컴파일된 확장(.so)이 로드 안 됨 (`ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`).
- **해결 방법 (2026-08-01 확정, 이미 적용됨)**:
  1. `python3.12`, `python3.12-dev`, `python3.12-venv` apt 설치
  2. `~/ros2_venv` = `python3.12 -m venv`로 만든 전용 venv. 여기에 `numpy setuptools packaging netifaces empy lark catkin_pkg distro psutil lxml argcomplete pyyaml` pip 설치 (system python3.14용 apt 패키지들과 동일한 목록을 python3.12 venv에 맞춰 재설치한 것)
  3. `~/.local/bin/ros2` — 실행 가능한 wrapper 스크립트, 내용은 `exec ~/ros2_venv/bin/python3 /opt/ros/jazzy/bin/ros2 "$@"`
  4. `~/.bashrc`에 `export PATH="$HOME/.local/bin:$PATH"` 추가해서 이 wrapper가 `/opt/ros/jazzy/bin/ros2`보다 먼저 잡히게 함
  - **왜 alias가 아니라 wrapper 스크립트인가**: bash alias는 **non-interactive 셸(스크립트, Claude Code의 Bash 도구, launch 파일 등)에서는 확장되지 않음**. 처음엔 `.bashrc`에 `alias ros2=...`로 시도했다가 이 문제로 실패해서 실행 파일 wrapper로 교체함.
  - **`colcon`/`rosdep`은 이 문제 없음** — rclpy(컴파일된 3.12 전용 확장)를 안 써서 시스템 기본 python3(3.14)로 그대로 잘 동작. `ros2` CLI/노드 실행에만 이 wrapper가 필요.
  - 현재 세션에서 새 패키지를 python3.12 쪽에서도 써야 하면 (예: 커스텀 노드에서 새 pip 패키지 import) `~/ros2_venv/bin/pip install <pkg>`로 설치할 것.
- 개발 중 부팅 디스크는 **외장 USB SSD 128GB**(`/dev/sda2`)를 사용 (microSD 쓰기속도/수명 문제 회피). 최종 배포용 128GB A2 microSD는 별도 확보돼 있음 — 개발 완료 후 전환 예정.
- GitHub: `carrotion/smart-trash-bin` (private). `gh` CLI 인증 완료(HTTPS 프로토콜, `git_protocol=https`로 설정됨). git 커밋 이메일은 `59335136+carrotion@users.noreply.github.com`.

## 알려진 미해결 이슈

- python3.12용 numpy가 깨져 있음 (`ModuleNotFoundError: No module named 'numpy._core._multiarray_umath'`) — `ros2 topic list`, `ros2 doctor` 등 numpy 의존 서브커맨드 실패. `ros2 pkg list` 등 numpy 미사용 커맨드는 정상. 해결 중.

## 확정된 하드웨어/설계 결정

- **SLAM**: LD06 2D LiDAR(DTOF, 12m/360°) + ICM-20948 IMU 기반, **SLAM Toolbox + Nav2**. 카메라 기반 VSLAM은 사용 안 함(서브 옵션으로만 문서에 남겨둠).
- **전원부**: 12V 3500mAh 18650(BMS 내장) → 메인 로커스위치 → 20A(모터)/5A(Pi) 독립 퓨즈 분기 → 모터는 Yahboom 드라이버로, Pi는 **동기식** 5V5A 벅컨버터+콘덴서 경유. 전원선 AWG16, 센서 신호선 AWG22.
- 상세 부품/가격/배송 현황은 `docs/phase0_hardware_ros2_guide.md`, 예산은 `docs/overview.md` 9.1절 참고.
- **최신 주간 일정/팀 체크리스트는 `docs/schedule.md`가 항상 최신** — 다른 문서의 로드맵 섹션은 참고용으로만 남아있음.

## 작업 방식

- 문서 수정 후에는 커밋 메시지를 남기고 `git push origin main`까지 완료할 것 (사용자가 매번 명시적으로 요청하지 않아도 이 저장소에서는 문서 변경 후 커밋/푸시가 관례로 자리잡음).
