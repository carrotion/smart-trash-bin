# CLAUDE.md — 스마트 쓰레기통 프로젝트 환경 메모

Claude Code가 이 저장소에서 작업할 때마다 참고해야 하는 환경 특이사항과 확정 결정사항. 컨텍스트가 압축되어도 여기 내용은 다시 읽고 재발견 없이 이어서 작업할 것.

## ⚠️⚠️ 안전 경고 — 모터 드라이버 관련 (2026-08-14 사고 발생, 필독)

**절대 `$mtype:3#` 또는 `$mtype:4#`를 보내지 말 것.** 이 프로젝트 모터는 **"370" 캔모터(Hall 엔코더 내장)**이지, TT모터(3번: 속도디스크 TT, 4번: TT직류감속)가 아님. **TT 프리셋(3/4)에서는 모터가 실제로 돌아도 `$MAll`/`$MTEP` 엔코더 값이 전혀 갱신 안 되고, 정지 명령(`$pwm:0,0,0,0#`/`$spd:0,0,0,0#`)도 안 먹혀서 바퀴 4개가 안 멈추는 사고가 실제로 발생했음** (배터리 커넥터로 물리 전원차단해서 겨우 정지, Pi도 비정상종료됐으나 재부팅 후 fs 손상은 없었음). **원인/해결 확정**: 아래 "확정된 모터 스펙" 섹션 참고 — TYPE=1(520프리셋)로 바꾸니 엔코더 정상 리포트 + 정지 명령도 정상 동작 확인됨(2026-08-14).

**향후 모터 테스트 시 필수 수칙 (계속 유효)**:
1. **TT 프리셋(3, 4번) 다시 시도 금지.**
2. 정지 명령을 보내고 **모터가 확실히 멈추는지 눈으로 확인 후에만** 다음 단계 진행할 것.
3. 모터 테스트는 항상 **바퀴가 허공에 뜬 상태**로, **메인스위치/배터리 커넥터에 바로 손이 닿는 상태**로 진행할 것.
4. `$pwm:`과 `$spd:`를 같은 세션에서 섞어 쓰지 말 것.

## ✅ 확정된 모터 스펙 — JGA25-370, 감속비 21.3:1 (2026-08-14)

- **감속비 21.3:1 확정**: 기어박스 축방향 길이 실측(~18~19mm, JGA25-370-1260(12V) 스펙표의 19mm 행과 일치) + 무부하 RPM 280(표와 정확히 일치) + 정격부하 RPM ~230(표의 220과 근사) — 세 단서가 전부 감속비 21.3:1로 수렴.
- **엔코더 라인수 11 확정**: 모터 판매 페이지의 배선 설명 이미지에서 "Yellow/Green: signal feedback, 11 signals per motor revolution"로 명시됨.
- **6핀 색상↔기능 매핑, 최종 확정** (제조사 공식 배선도로 재검증 완료, 이전에 멀티미터 실측으로 만든 표와 100% 일치): 빨강=M+, 흰색=M-, 검정=엔코더 GND, 파랑=엔코더 VCC, 노랑/초록=A상/B상(H1A/H1B, 어느 쪽이 A인지는 무관 — 소프트웨어에서 부호로 처리).
- **Yahboom MOTOR_TYPE 프리셋 중 정확히 "21.3:1, 11라인"에 맞는 건 없음** — TYPE1(520, pulse_line=11·deadzone=1900·휠지름67mm)이 pulse_line은 일치하는 가장 가까운 프리셋. **`$mtype:1#` + `$deadzone:1900#` + `$wdiameter:67.00#`로 설정 후 실제 검증 완료**: M2(왼쪽앞) `$pwm:0,1000,0,0#`으로 2초 구동 → `$MAll` M2 값이 3142→-9357로 실제 변화(-12499 카운트), 나머지 채널(M1/M3/M4)은 정확히 그대로(크로스토크 없음 재확인), 정지 명령도 정상 작동(인코더 값 3초간 고정 + 육안 정지 확인).
- **주의**: TYPE1의 pulse_phase(=30, 감속비 프리셋값)가 실제 21.3과 다르므로, **드라이버가 내부 계산하는 `$MSPD`(mm/s) 값은 부정확할 것으로 예상** — 정확한 거리/속도 환산은 `smart_trash_motion` 노드에서 실측값(21.3:1, 11라인, 휠지름은 실측 필요)으로 직접 계산할 것. `$MAll`/`$MTEP`의 원시 펄스 카운트 자체는 정확함.

## 🔍 Yahboom 4채널 엔코더 모터 드라이버 — 검증된 시리얼 프로토콜 (2026-08-14)

실제 하드웨어와 주고받아 검증한 내용만 기록. **공식 문서에 없는 내용은 절대 추측해서 코드에 넣지 말 것** — 이번 세션에서 지어낸 바이너리 프로토콜/가짜 `Rosmaster_Lib` GitHub 저장소 정보에 두 번 낚일 뻔했음(둘 다 실행 전 검증으로 걸러냄).

- **포트/보드레이트**: `/dev/ttyUSB0`, **115200 8N1**. `carrotian`은 `dialout` 그룹에 있어야 함(이미 완료).
- **프레임 포맷**: ASCII 텍스트, `$COMMAND:value1,value2,...#` 형태. 응답도 동일 포맷 또는 `Set X:OK!\r\n` / `fail,Command not found!\r\n`. **잘못된 명령어 이름은 명확히 "fail"로 알려주므로, 이름 후보를 낮은 위험으로 탐색 가능** (단, 모터 속도/PWM 관련 값은 항상 0으로 테스트하고 절대 임의값으로 탐색하지 말 것).
- **확인된 명령어**: `$mtype:N#`(모터 타입 설정), `$deadzone:N#`, `$wdiameter:N#`(휠 지름), `$upload:N#`(엔코더 데이터 업로드 모드: 0=끔 1=총 누적($MAll) 2=실시간($MTEP) 3=속도($MSPD)), `$pwm:m1,m2,m3,m4#`(개루프 PWM 구동), `$spd:m1,m2,m3,m4#`(폐루프 속도 구동, TT 프리셋 전용으로 추정 — 이번 사고 이후 사용 안 함).
- **못 찾은 명령어**: pulse_line/pulse_phase 설정 명령(`$pline:`/`$pphase:` 등 여러 후보 시도했으나 전부 fail) — 정확한 이름 미확인. custom 감속비/라인수를 드라이버에 직접 넣을 방법은 아직 못 찾음 (그래서 위처럼 TYPE1 프리셋 + 우리 노드에서 자체 환산하는 방식 채택).
- **모터→드라이버 채널 매핑 (4채널 전부 확인 완료, 2026-08-14)**: M1=왼쪽뒤, M2=왼쪽앞, M3=오른쪽뒤, M4=오른쪽앞. TYPE=1로 4채널 전부 `$pwm:`로 개별 구동 검증: 전 채널 실제 엔코더 변화량 반영(약 -11700~-12500 카운트대, 채널간 일관됨), 크로스토크 없음, 정지 명령 전부 정상(정지 후 1초+ 값 고정 확인).

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
- **`smart_trash_bringup/launch/mapping.launch.py`** (2026-08-01 작성): LD06 드라이버(`ldlidar_stl_ros2`) + SLAM Toolbox(`online_async_launch.py`, 파라미터는 `config/mapper_params_online_async.yaml`)를 함께 띄워 매핑하는 launch. **`use_sim_time`을 명시적으로 `false`로 넘김** — slam_toolbox의 기본 launch는 `use_sim_time` 기본값이 `true`라 실물 로봇에서 그대로 include하면 `/clock`을 기다리며 멈춤. `odom→base_link`는 아직 Week 3 diff-drive 오도메트리 노드가 없어서 `publish_placeholder_odom`(기본 true) 인자로 identity tf를 임시 발행 — 실제 오도메트리 노드가 생기면 이 launch 인자를 `false`로 끌 것. `ros2 launch smart_trash_bringup mapping.launch.py --show-args`로 파싱 검증 완료(2026-08-01), LiDAR 실물 미연결 상태라 실행 테스트는 아직 못 함.
- **`smart_trash_sensors/imu_publisher_node`** (2026-08-01 작성): ICM-20948을 SparkFun의 `sparkfun-qwiic-icm20948` pip 패키지(`pip3 install --break-system-packages sparkfun-qwiic-icm20948`, 이미 설치됨)로 읽어 `imu/data`(`sensor_msgs/Imu`)로 퍼블리시. 기본 풀스케일(가속도 ±2g/16384 LSB·자이로 ±250dps/131 LSB)로 raw→물리단위 변환, orientation은 채우지 않고 `orientation_covariance[0] = -1`(REP-145)로 표시 — 자세 융합은 Week 4 `robot_localization` EKF가 담당. **하드웨어 없이 실행하면 `RuntimeError`로 깔끔하게 실패하는 것까지만 확인함**(`ICM-20948을 I2C 버스에서 찾을 수 없습니다`), 실제 센서 값 검증은 IMU 연결 후(Week 1~2) 필요.
- **I2C 권한**: `/dev/i2c-1`이 `dialout` 그룹 소유인데 `carrotian`이 그 그룹에 없어서 최초 실행 시 `Permission denied` 발생했음. `sudo usermod -aG dialout carrotian`으로 해결(2026-08-01) — **재로그인/재부팅해야 적용됨**, 다음 세션에서 IMU 노드 실행 전에 `groups` 명령으로 `dialout` 포함 여부 확인할 것.
- **LD06 드라이버**는 `robot/src/ldlidar_stl_ros2`에 **git submodule**로 연결됨, 가리키는 저장소는 `carrotion/ldlidar_stl_ros2` (공식 `ldrobotSensorTeam/ldlidar_stl_ros2`를 fork). **fork에 패치 커밋 완료**: `ldlidar_driver/src/logger/log_module.cpp`에 `#include <pthread.h>` 추가 (누락 시 최신 GCC 15.2.0/Ubuntu 26.04에서 빌드 실패). 로컬 패치가 아니라 fork에 정식 커밋+푸시했으니 재클론해도 유지됨.
- ⚠️ **이 저장소를 새로 클론할 때는 `git clone --recurse-submodules ...` 또는 클론 후 `git submodule update --init`을 꼭 해야 `robot/src/ldlidar_stl_ros2` 내용이 채워짐.**

## 알려진 미해결 이슈

- 현재 없음. (26.04 시절의 python3.12 런타임 의존성/Nav2 크로스버전 충돌 이슈는 24.04 재설치로 해소됨 — 위 "Ubuntu 24.04 재설치 완료" 섹션 참고)

## 목표/일정

- **방학 중간 목표**: 2026-08-01 방학 시작 기준, 방학 안에 **맵을 그리는 것(LD06 + SLAM Toolbox로 실내 맵핑 성공)까지**를 중간 목표로 삼기로 함. `docs/schedule.md`의 Week 3 "LD06 + SLAM Toolbox 맵핑 검증"이 이 목표에 해당 — 부품(LD06) 도착이 전제조건이라 일정이 빠듯하므로, 부품 없이 미리 할 수 있는 SW 작업(워크스페이스/스캐폴딩/드라이버 빌드/`mapping.launch.py` 등)은 최대한 앞당겨 진행 중.

## 확정된 하드웨어/설계 결정

- **SLAM**: LD06 2D LiDAR(DTOF, 12m/360°) + ICM-20948 IMU 기반, **SLAM Toolbox + Nav2**. 카메라 기반 VSLAM은 사용 안 함(서브 옵션으로만 문서에 남겨둠).
- **전원부**: 12V 3500mAh 18650(BMS 내장) → 메인 로커스위치 → 20A(모터)/5A(Pi) 독립 퓨즈 분기 → 모터는 Yahboom 드라이버로, Pi는 **동기식** 5V5A 벅컨버터+콘덴서 경유. 전원선 AWG16, 센서 신호선 AWG22.
- 상세 부품/가격/배송 현황은 `docs/phase0_hardware_ros2_guide.md`, 예산은 `docs/overview.md` 9.1절 참고.
- **최신 주간 일정/팀 체크리스트는 `docs/schedule.md`가 항상 최신** — 다른 문서의 로드맵 섹션은 참고용으로만 남아있음.

## 작업 방식

- 문서 수정 후에는 커밋 메시지를 남기고 `git push origin main`까지 완료할 것 (사용자가 매번 명시적으로 요청하지 않아도 이 저장소에서는 문서 변경 후 커밋/푸시가 관례로 자리잡음).
