# CLAUDE.md — 스마트 쓰레기통 프로젝트 환경 메모

Claude Code가 이 저장소에서 작업할 때마다 참고해야 하는 환경 특이사항과 확정 결정사항. 컨텍스트가 압축되어도 여기 내용은 다시 읽고 재발견 없이 이어서 작업할 것.

## ✅ 자율탐색 전 파이프라인 첫 완주 + 비상정지 실전 검증 (2026-08-16 밤)

**도크(바퀴가 뜬 정비/충전 거치대)에서 explore_lite까지 전 구간이 처음으로 동작함.**
- explore_lite가 프론티어를 찾아 `NavigateToPose` 목표 발행 → bt_navigator 주행 →
  controller_server가 `cmd_vel` 발행 → 바퀴 실제 회전. 20초간 162건(0 아닌 명령
  160건, `linear.x` 최대 0.20 / `angular.z` 0.72)까지 확인.
- **바퀴가 도는 중에 비상정지를 눌러 검증**: 대상 6개(explore 포함) 전부 종료,
  0.26초, 이후 10초간 `cmd_vel` 0건. 오늘 한 번도 못 했던 검증을 여기서 통과.
- ⚠️ 도크 위에서는 바퀴가 헛돌아 odom이 유령 이동을 적분하므로(스캔은 정지)
  **맵이 망가진다.** 실주행 전에 반드시 `mapping_cleanup.py --restart`로 새로 시작할 것.

### 🏠 도크(거치대)에 대한 환경 사실 (사용자 확인, 2026-08-16)
**도크는 바퀴가 뜬 정비/충전용 공간이고, 로봇이 스스로 내려올 수 없다 — 사람이
옮겨줘야 한다.** 그래서 도크는 "주행 시작점"이 아니라 **안전한 통합 테스트 장소**로
쓰는 게 맞다(명령이 모터까지 가는지, 비상정지가 실제로 바퀴를 멈추는지를 위험 없이
확인 가능). 자율주행 실주행은 사람이 바닥의 시작 위치에 놓아준 뒤에 시작.

### ⚠️ `smart-trash-lidar` 서비스가 되살아나 `/scan`을 오염시킴 (해결됨)
`smart-trash-sensor-dashboard`에 `Wants=smart-trash-lidar`가 걸려 있어서, **대시보드를
재시작할 때마다 라이다 서비스가 되살아났다.** 그 서비스는 `/scan`에 **필터 안 된**
스캔을 직접 발행하므로 `scan_filter_node`와 발행자가 2개가 되고, SLAM/코스트맵이
필터된 것과 안 된 것을 **섞어서** 받는다 → 자기반사 링이 코스트맵에 lethal로 박혀
로봇 셀이 inscribed(99)가 되고 **경로계획이 전부 ABORTED**. `/dev/ttyAMA0` 경합도 발생.
→ `Wants`에서 제거함(`After`는 유지). 라이다 단독 진단은 명시적으로 start할 것.
**증상 감별법**: 필터 노드는 "N% 걸러냄"이라고 정상 로그를 찍는데 `/scan`엔 여전히
0.08m 반사가 보이면 이 상황이다. `ros2 topic info /scan --verbose`로 발행자 수를 볼 것.

## 🛑 비상정지가 실제로는 작동하지 않았음 (2026-08-16 밤, 첫 실주행 직전 발견)

첫 실주행 전에 비상정지를 눌러봤는데 **응답이 아예 안 왔다.** 실주행 전에 눌러본 게
다행이었음 — 안 눌러봤으면 로봇이 도는 중에 못 세웠을 것. 원인이 3중이었다:

1. **순서가 틀렸다.** `0속도 발행 → 프로세스 kill` 순이라, 발행이 막히면 kill까지
   영영 도달을 못 했다. **진짜 정지는 명령 소스를 죽이는 것** — `diff_drive_node`가
   `cmd_vel`이 0.5초만 끊겨도 자동 정지하므로(`cmd_vel_timeout`) 소스만 죽이면
   0.5초 안에 반드시 선다. 0속도 발행은 보조라서 나중에, 타임아웃 걸고 실행.
2. **`subprocess.run(["pkill", ...], capture_output=True)`가 죽일 프로세스가
   실제로 있을 때만 안 돌아왔다**(없으면 0.4초, 있으면 무한대기). 안전 경로에서
   외부 프로세스+파이프에 의존하는 걸 없애고 `/proc` 직접 스캔 + `os.kill`로 교체.
3. **`pkill -f`의 부분문자열 매칭이 무관한 프로세스를 죽였다.** `bt_navigator`
   패턴이 진단용 셸(`pgrep -f 'nav2_bt_navigator|...'`)의 명령줄에 걸려서 그 셸이
   죽음. 실전이면 사용자 SSH 세션도 같은 이유로 죽는다.
   → **`argv[0]`의 basename 정확 일치**로 변경(`AUTONOMOUS_EXECUTABLES`).

수정 후 드릴 2회 연속 검증: 대상 5개 전부 종료, 0.26초, 무관 프로세스 피해 없음.

⚠️ **`pkill -f <패턴>`은 그걸 실행하는 셸 자신도 매칭한다** — 이 세션에서 작업 셸이
여러 번 날아갔다. 프로세스를 이름으로 죽일 땐 `tools/mapping_cleanup.py`를 쓸 것.

### `tools/mapping_cleanup.py` (신규)

매핑/자율주행 세션 뒷정리 전용. `--restart`를 주면 diff-drive/EKF도 재시작한다.
- 실행파일명 정확 일치로만 죽여서 자기 셸/무관 프로세스를 안 건드림
- 런치를 SIGKILL하면 자식(라이다/SLAM/필터)이 **고아로 남아 중복 누적**되므로
  자식까지 전부 훑고, 0이 될 때까지 반복
- `/dev/shm/fastrtps_*` 고아 파일만 삭제(`/proc/*/fd`로 사용중 판별) — 이걸 안
  하면 다음 SLAM이 `Failed init_port`로 안 뜸
- ⚠️ **정리 중에 대시보드가 떠 있으면 그 DDS 상태가 깨질 수 있다.** 실제로
  대시보드의 ROS publish가 먹통이 돼서 비상정지가 죽은 적 있음 — 대량 정리 후엔
  대시보드 서비스도 재시작할 것.

## 🎯 Nav2 경로계획 파이프라인 첫 성공 + 버그 5건 (2026-08-16 밤)

`ComputePathToPose`가 여러 목표에 대해 전부 `error_code: 0` SUCCEEDED. **아직 실제
주행(explore_lite)은 안 돌림** — 경로계획까지만 검증됨. 검증은 새로 추가한
`enable_explore:=false` 런치 인자로 했음(Nav2만 뜨고 목표를 보내는 주체가 없어
**로봇이 절대 안 움직임**). 설정 검증은 앞으로도 이 모드로 할 것.

찾아낸 버그와 원인:

1. **🔴 라이다가 로봇 자기 구조물을 봄 — 이게 진짜 원인.** 마운트/상판이 360°
   전방향 0.06~0.10m에 153포인트로 찍혀서, 로봇 자기 위치가 맵에 장애물로 박히고
   코스트맵엔 로봇을 감싼 lethal 링이 생김 → **어떤 목표도 경로가 안 나옴.**
   실측 히스토그램상 0.10~0.30m가 완전히 비어 있어 임계값 **0.15m**로 확정.
   - 해결: `smart_trash_sensors/scan_filter_node.py` 신규(드라이버 `scan_raw` →
     필터 → `scan`). 다른 노드는 전부 기존처럼 `scan`만 보면 되므로 수정 불필요.
   - ⚠️ **slam_toolbox에는 `min_laser_range` 파라미터가 아예 없다**(`max_laser_range`만
     있음). yaml에 적어도 조용히 무시됨 — 여기 기대다가 한참 헤맸음.
   - ⚠️ 필터 발행 QoS는 **RELIABLE**이어야 함. BEST_EFFORT로 뒀더니 RELIABLE로
     구독하는 `sensor_dashboard`가 조용히 끊김(SLAM/Nav2는 BEST_EFFORT라 괜찮았음).
2. **global_costmap이 `/map`을 영원히 못 받음.** 코스트맵은 `/global_costmap`
   네임스페이스 안의 별도 노드라 상대 토픽명 `"map"`이 `/global_costmap/map`으로
   해석됨. → **`map_topic: "/map"`처럼 절대경로로** 써야 함. `map_subscribe_transient_local: true`도 필요(SLAM이 TRANSIENT_LOCAL로 발행).
3. **lifecycle_manager `node_names`에 코스트맵을 넣으면 안 됨.** 코스트맵은
   planner/controller가 자기 `on_configure`에서 직접 전이시킴 — 표준 nav2_bringup의
   `lifecycle_nodes` 목록에도 코스트맵은 없음.
4. **EKF가 세션 간에 상태를 넘김.** `diff_drive`만 재시작하고 `ekf`를 안 했더니
   누적 yaw −60.6°를 그대로 들고 시작함(맵 방향이 통째로 틀어짐).
   → **매핑 세션 시작 전 `smart-trash-diff-drive`와 `smart-trash-ekf`를 같이 재시작할 것.**
5. **`kill -9` 후 DDS 공유메모리 잠금파일이 남아 SLAM이 안 뜸**
   (`Failed init_port fastrtps_portNNNN`). 좀비 프로세스가 `_el` 파일을 잡고 있는
   게 원인. `pgrep`으로 잔존 프로세스를 **0이 될 때까지** 확실히 죽인 뒤,
   `lsof`로 살아있는 프로세스가 쓰는 파일을 제외하고 고아 파일만 지울 것.

**진단 시 주의**: `ros2 param get /planner_server global_costmap.*`는 항상
"Parameter not set"이 나온다 — 코스트맵은 `/planner_server`가 아니라
**`/global_costmap/global_costmap`** 노드다. 이걸 몰라서 멀쩡한 YAML을 한참 의심했음.

## 🧭 자율주행 프론티어 탐색 첫 통합 (2026-08-16, 실주행 테스트 전)

`robot/src/m-explore-ros2`(서브모듈, `feature/slam_toolbox_compat` 브랜치 — main엔
SLAM Toolbox와 같이 쓸 때 나던 seg fault가 있어서 이걸 씀) + Nav2로 자율 프론티어
탐색 구성 완료. `ros2 launch smart_trash_bringup autonomous_mapping.launch.py`.
- map_server/amcl 없이 SLAM Toolbox의 /map을 Nav2 global_costmap이 직접 구독
  ("매핑 중 네비게이션" 표준 패턴). explore_lite도 같은 /map, /map_updates를
  직접 구독(코스트맵 안 거침).
- 로봇 실측 20x20cm(사용자 확인) 기준 robot_radius=0.18m, 속도는 보수적
  (max 0.20m/s, 0.8rad/s) — 오늘 실주행에서 0.3m/s가 맵 품질에 적절했던 것 참고.
- **진행상황(2026-08-16 밤 갱신)**: 경로계획까지는 검증 완료(위 "Nav2 경로계획
  파이프라인 첫 성공" 절 참고). **explore_lite로 실제 주행하는 건 아직 안 해봄.**
  범퍼/낙하 센서가 없어 라이다 단독 회피만 가능 — 첫 실주행은 반드시 사람이
  옆에서 지켜보고 즉시 개입 가능한 상태로 할 것(맵 대시보드 `:8768`에 비상정지
  버튼 있음).
- 다음 단계(사용자 요청): 탐색 중 `tools/wifi_signal_logger.py`도 같이 돌려서
  위치별 WiFi 신호 기록 → 맵 위에 커버리지 히트맵 시각화 → 커버리지 나쁜
  구역은 Nav2 Keepout Zones 코스트맵 필터로 자율주행 금지구역 지정 (아직 미착수).

## ⚠️ 미해결: TCP 서비스만 응답불능 (2026-08-16 오후, 원인 미상)

**증상**: 매핑 세션 진행 중 대시보드 무응답 + SSH 접속 안 됨(오래 기다려도 안 풀려서 전원버튼으로 강제 종료 후 재부팅). 공유기에서 보면 WiFi는 계속 연결 상태 유지. Pi 녹색 LED 켜짐, SSD LED 계속 깜빡임(단, 이건 `power_monitor.py`가 매초 fsync하는 정상 동작 패턴일 수 있어 증거로는 약함).

**중요 반증**: `power_trace.csv`로 정지 직전(전원버튼 3초 전, 14:43:00)까지 확인한 결과 **메모리 63.3~63.5%로 완전히 평평**(스왑 근처도 안 감), CPU 8~20%, 전압 5.20V 안정 — 스왑 스래싱 가설은 데이터로 반증됨. 커널 로그도 이 구간 전혀 에러 없음(`journalctl -b -1 -k` 확인). `map_autosave`(ROS2/DDS, UDP 기반)는 14:43:00까지 정상적으로 60초 간격 맵 저장 계속함 — **즉 ROS2/DDS 트래픽은 살아있었는데 TCP 서비스(SSH, 대시보드)만 응답불능**이었던 비대칭적 증상. 원인 특정 못함 — 그럴듯한 이야기 지어내지 말 것.

**다음 재발 시를 위한 대비**: `~/tcp_heartbeat.log`에 5초 간격으로 SSH(22)/대시보드허브(8764) 로컬 loopback 연결 성공 여부 + `ss -tan established` 연결 수를 기록하는 백그라운드 스크립트를 세션에서 임시로 띄워둠(재부팅하면 안 살아있음 — 재현 조사 재개 시 다시 띄울 것, 스크립트 원본은 이 세션의 scratchpad에만 있으므로 필요하면 다시 작성). 재발하면 이 로그로 정확히 언제부터 TCP가 막혔는지 좁힐 수 있음.

**Davi(antigravity) 감사 리포트 검토 결과 (2026-08-16, `docs/audit/2026-08-16_ssh_unresponsive_incident_report.md`)**: "WiFi silent stall"(공유기가 명시적 deauth 없이 무선 링크만 조용히 죽는 현상) 가설을 제시했으나, 리포트가 핵심 근거로 든 로그(`14:25:56 cmd_vel 타임아웃`)는 사실 Remi가 직진 트림 캘리브레이션 중 실행한 테스트 명령의 정상 종료 로그였음(무선과 무관) — 검증 후 정정함. 가설 자체는 직접 증거 없이 여전히 "가능성 있는 후보" 중 하나일 뿐, 확정 아님. 다만 근본원인 확정 여부와 무관하게 안전한 하드닝으로 판단해 다음 2가지는 적용함:
- `tools/wifi_watchdog.sh` + `smart-trash-wifi-watchdog.timer`(30초 간격): 게이트웨이 응답없으면 `wpa_cli reassociate`로 wlan0 재연결.
- SSH `ClientAliveInterval 15`/`ClientAliveCountMax 3` (`/etc/ssh/sshd_config.d/keepalive.conf`).

## ✅ robot_localization EKF 도입 + 직진 트림 캘리브레이션 (2026-08-16)

- **EKF 설계**: `diff_drive_node`는 순수 휠 기반 odom만 발행(자체 IMU 융합·TF 발행 중단, `use_imu_for_yaw`/`publish_tf` 기본값 False). 새 `smart-trash-ekf.service`(`robot_localization`의 `ekf_node`, 상시 구동)가 원본 `/imu/data` 자이로를 직접 구독해 회전각 융합을 전담, `/odometry/filtered` + `odom->base_link` TF 발행. 설정은 `robot/src/smart_trash_bringup/config/ekf.yaml`. odom 메시지에 공분산도 추가함(x,y,vx는 신뢰도 높게, yaw/vyaw는 낮게 — 이중계산 방지).
- **직진 트림 보정** (`diff_drive_node.py`의 `straight_trim` 파라미터, 기본값 **0.6**): 전진 명령에도 우측으로 쏠리는 현상(2026-08-14 기록된 M4 인코더 반응 저하와 일치하는 것으로 추정)을 실측 기반으로 보정. 0.3m/s 직진 테스트를 반복하며 cliff test로 범위 확정 — **사용 가능 범위 0.5~0.7(오차 <1%), 채택값 0.6**. 상세 실측 이력은 `diff_drive_node.py`의 `straight_trim` 선언부 주석 참고. **중요 통찰**: 총 편차는 "주행 중 편향"(trim으로 보정됨)과 "정지 시 관성으로 원래 기계 얼라인먼트로 복귀하는 편향"(trim과 무관, 항상 존재)의 합이라 trim을 올려도 개선이 완전히 선형으로 줄진 않음 — 다만 실제 조종도 버튼 On/Off를 반복하므로 이 "정지 후 최종 위치 편차" 자체가 실사용과 가까운 지표라 이 기준으로 캘리브레이션하는 게 맞음.
- 부수 발견: `motor_dashboard.py`/`sensor_dashboard.py`가 `rclpy.spin()`을 데몬 스레드로 돌리고 정리 없이 인터프리터를 종료하는 구조라, ROS 그래프가 커지자(ekf_node 추가 등) 종료 시 `terminate called without an active exception`으로 core dump 나는 문제 발견/수정함(`rclpy.shutdown()`→`spin_thread.join()`→`node.destroy_node()` 순서로 명시적 정리). 재현 테스트로 반복 검증 완료.

## ⚠️ 두 개의 별개 사고를 하나로 착각했던 기록 정정 (2026-08-16)

2026-08-16 새벽~아침에 **서로 다른 원인의 사고 두 건**이 시간상 겹쳐서 발생했음. 최초엔 하나로 묶어서 "전부 배터리 방전"이라고 잘못 기록했다가 사용자가 정정함. 반드시 구분해서 볼 것:

1. **SSD read-only 전환 + WiFi 응답불능 사고** (재매핑 테스트 중, 부팅 `dea491...` 07:47:57~08:25:13) — 아래 "🔌 SSD read-only 전환" 섹션 참고. **배터리 문제 아님.** 원인/해결 진행 중.
2. **급속 재부팅 루프** (그 이후, 08:46~10:01 사이 8분→18초→1분→1분으로 생존시간이 계속 짧아지며 반복 재부팅) — 이건 배터리 저전압(사용자 실측 10V 미만, 3S 정격 11.1V 대비 심각한 방전)이 맞았음. **배터리 교체로 해결 확인됨.** DC 충전기가 연결돼 있어도 고부하(모터+LiDAR+SLAM) 시 소모 전류가 충전 전류보다 크면 순유출될 수 있음 — "충전기 연결됨" ≠ "순충전 중", 고부하 테스트 중엔 전압을 직접 실측해서 판단할 것.
- 당일 미리 해둔 하드웨어 워치독/journald 5초 동기화/스왑 4GB/softlockup→panic 전환 조치는 두 사고 중 어느 쪽이 원인이었든 일반적인 안전망으로 유효하므로 그대로 유지.
- 파일시스템 무결성은 두 사고 이후 확인 완료(2026-08-16): `tune2fs -l`상 `Filesystem state: clean`, 저널 자동복구 정상, git 저장소/맵 파일 손상 없음.

## 🔌 SSD read-only 전환 사고 — ASMedia UASP 브릿지 의심 (2026-08-16, 조사/해결 진행 중)

**증상**: 재매핑 테스트 중 PC 대시보드가 무응답이 됐지만 **폰의 모터 조종은 계속 동작함** — 즉 완전 hang이 아니라 디스크 쓰기가 필요한 기능만 죽은 상태. 로컬 모니터를 연결해 콘솔 화면을 직접 확인(사진 촬영)하니:
```
Buffer I/O error on dev sda2, logical block ..., lost async page write
EXT4-fs error (device sda2): ext4_journal_check_start:84: ... Detected aborted journal
EXT4-fs (sda2): Remounting filesystem read-only
```
그 후 약 16분 뒤 WiFi 칩(`brcmfmac`)도 `-110`(타임아웃) 에러를 반복하며 응답불능이 됨 — **단, 이 WiFi 단절 이후의 증상(접속 끊김 등)은 SSD 문제의 연쇄가 아니라 이 시점부터 이미 진행되고 있던 배터리 저전압(위 섹션 참고)의 별도 증상으로 판단됨.** SSD read-only 자체의 원인 규명은 이 절 나머지 내용대로 진행.

**중요: 이 로그는 journald/디스크 어디에도 남아있지 않았음** (`journalctl --grep="I/O error|EXT4-fs error"` 전체 이력 검색해도 0건) — 디스크 자체가 고장난 순간이라 그 에러 메시지조차 디스크에 못 썼던 것. **로컬 콘솔 화면을 사진 찍은 게 유일한 증거였음.** 이게 어제 pstore/ramoops를 시도했던 이유와 정확히 같은 시나리오지만, ramoops는 Pi5 미지원으로 못 넣었음 — 재발 시 또 콘솔을 직접 봐야 증거가 남는다는 뜻이므로 유의.

**원인 추정(높은 확신)**: `/dev/sda` 부팅 SSD가 **USB3 SuperSpeed(5000M) + UAS(USB Attached SCSI) 드라이버**로 연결됨 (`lsusb -t` 확인: Bus 003, Ugreen 외장케이스, **idVendor=174c idProduct=55aa = ASMedia 브릿지칩**). ASMedia 계열 USB-SATA/NVMe 브릿지 + 리눅스 `uas` 드라이버 조합은 라즈베리파이 커뮤니티에서 흔히 보고되는 문제로, 특정 명령(TRIM/UNMAP, 링크 파워스테이트 전환 등)에서 브릿지 펌웨어가 명령을 놓치거나 타임아웃 내면서 정확히 이런 "Buffer I/O error → 저널 abort → read-only 강제전환" 패턴을 유발함.
- USB 전원관리는 확인해봤는데 정상(`/sys/bus/usb/devices/3-1/power/control` = `on`, autosuspend 아님 — 이건 원인에서 제외됨).
- `fstrim.timer`가 매주 활성화돼 있음(주기적 TRIM) — UASP 브릿지와 상성이 안 좋은 대표적 트리거라 의심 후보.

**해결법 적용 완료 (2026-08-16)**:
1. **UAS 비활성화, 구식 usb-storage(BOT) 드라이버로 강제 전환 — 적용됨.** `/boot/firmware/cmdline.txt`에 `usb-storage.quirks=174c:55aa:u` 추가 후 재부팅. `lsusb -t`로 `Driver=usb-storage`(uas 아님) 확인됨. 이후 여러 시간 재부팅/매핑 세션 동안 I/O 에러 재발 없음.
2. `fstrim.timer` 비활성화도 같이 적용함(`sudo systemctl disable --now fstrim.timer`).
3. 재발하면 케이블/커넥터 물리 점검, 또는 브릿지칩 자체 결함으로 보고 케이스 교체 고려 — 아직 필요했던 적 없음.

## 🎉 LD06 LiDAR 연결 + SLAM Toolbox 파이프라인 첫 성공 (2026-08-15)

**방학 중간목표(맵핑까지)의 핵심 파이프라인이 처음으로 에러 없이 끝까지 떴음** — `ros2 launch smart_trash_bringup mapping.launch.py` 실행 시 LD06 통신 정상, `/scan` 10Hz 안정적으로 발행, SLAM Toolbox가 Configuring→Activating→"Registering sensor" 까지 전부 정상 완료.

- **LD06은 USB 어댑터가 아니라 Pi5 GPIO UART로 직결됨**: GPIO 2/6/10/12번 핀(5V/GND/RXD/TXD 계열), 장치는 **`/dev/ttyAMA0`**, 보드레이트 **230400**. USB로 연결한다고 가정하고 `/dev/ttyUSB0`를 찾으면 안 됨 — `ttyUSB0`는 모터 드라이버 전용.
- ⚠️ **UART 콘솔 충돌 함정(해결됨)**: 기본 이미지가 `/dev/ttyAMA0`을 로그인 시리얼 콘솔로도 쓰게 설정돼 있어서(`cmdline.txt`의 `console=serial0,115200` + `serial-getty@ttyAMA0.service`), LD06이 데이터를 쏟아내자 getty가 로그인 프롬프트로 착각하고 계속 죽었다 재시작하며 `input overrun` 에러만 남기고 `start-limit-hit`로 포기하는 상태였음. **해결**: `sudo systemctl disable --now serial-getty@ttyAMA0.service` + `sudo systemctl mask serial-getty@ttyAMA0.service` + `/boot/firmware/cmdline.txt`에서 `console=serial0,115200 ` 제거 + 재부팅. 재부팅 후 `/dev/ttyAMA0` 소유 그룹도 자동으로 `dialout`으로 바뀜(`carrotian` 이미 그 그룹이라 추가 조치 불필요). **새 Pi 이미지로 다시 설치하게 되면 이 과정을 처음부터 다시 해야 함.**
- **`smart_trash_bringup/launch/mapping.launch.py` 리팩터링**: 서브모듈이 제공하는 `ld06.launch.py`(포트가 `/dev/ttyUSB0`로 하드코딩됨, 우리 배선과 안 맞음)를 include하는 대신, **`ldlidar_stl_ros2_node`를 직접 구성**해서 `port_name`을 launch 인자(`lidar_port`, 기본값 `/dev/ttyAMA0`)로 뺌. `base_link->base_laser` 정적 tf도 이 launch 파일이 직접 발행(예전과 동일하게 z=0.18, 회전 0).
- ✅ **LD06 상판 거꾸로 마운트 방향 보정 완료 (2026-08-15, 실측 캘리브레이션)**: 로봇을 방 모서리(전면벽+왼쪽벽, 벽에서 40~50cm 이격)에 놓고 `/scan` 원시 데이터에서 두 벽이 찍히는 각도를 직접 분석함. **방법론**: 벽 하나만으로는 회전 오프셋인지 좌우반전인지 구분 안 되므로, 서로 다른 두 방향(전면/왼쪽)의 벽을 동시에 써서 각도差로 판별. `laser_scan_dir=True`(기본 가정)일 땐 왼쪽 벽이 전면(≈0°) 대비 **-90°(270°) 쪽**에 나와서(정상이면 +90°) 좌우가 반전돼 있었음. `laser_scan_dir=False`로 바꾸니 왼쪽 벽이 **+90°대로 정확히 이동**(실측 265°→90°, 미러 변환 예측치 95°와 근접 일치) — 상판 거꾸로 마운트로 인한 반전이 확인/보정됨. **`mapping.launch.py`에 `laser_scan_dir: False`로 반영 완료.** 이 보정이 드라이버 레벨에서 각도 해석 자체를 바로잡아주므로 정적 tf에 별도 롤/피치 180도 회전은 불필요(z높이 0.18m만 아직 placeholder, 실측 필요).
- **다음 단계**: `publish_placeholder_odom`이 아직 identity tf라 실제 이동 시 맵이 안 맞을 것 — Week 3 diff-drive 오도메트리 노드(`smart_trash_motion`) 완성 후 그걸로 교체 필요. 그 전까지는 로봇을 고정해두고 정적 스캔 품질만 확인하거나, 아주 살살 손으로 옮기며 SLAM Toolbox의 스캔매칭만으로 테스트하는 정도로 제한할 것.

## 🧭 오도메트리 설계 — 회전각은 반드시 IMU 자이로 (2026-08-15 실측 확정)

**결론: 전진거리=휠 엔코더, 회전각=IMU 자이로.** 스키드-스티어 4WD는 고정축 4바퀴라 제자리 회전 시 바퀴가 옆으로 미끄러져서, 휠 기반 회전각은 **쓸 수 없는 수준으로 부정확하고 재현성도 없음**.

| 방식 | odom 기록 | 실제 | 오차 |
|---|---|---|---|
| 휠 엔코더 기반 | 373.7° | 193° | **+94%** (게다가 바닥/속도/전압/마모마다 달라짐) |
| **IMU 자이로 기반** | 414.8° | 420° | **-1.2%** ✅ |

- 휠 기반을 쓰려면 실효 트레드를 물리 트레드(0.17m)의 1.94배인 0.329m로 잡아야 겨우 맞는데, 이 값 자체가 바닥 재질·회전 속도·바퀴 마모·습도에 따라 계속 변해서 고정 상수로 쓸 수 없음. 실제로 캘리브레이션 중 바퀴 하나가 마찰로 스톨(돌다 멈췄다 반복)하는 것도 관측됨 — 재현성이 없다는 결정적 증거.
- `use_imu_for_yaw` 파라미터(기본 True)로 자이로 사용. `wheel_separation`은 자이로를 못 쓸 때의 폴백 경로에만 남아있음.
- **자이로 바이어스 보정 필수**: 시작 직후 정지 상태에서 100샘플 평균으로 오프셋을 구해 빼줌(측정값 예: +0.023, -0.007 rad/s로 매번 다름). 안 빼면 가만히 있어도 각도가 계속 돎.
- **센서 융합 계층 구조**(사용자와 합의): `odom→base_link`는 휠+IMU로 부드럽게, `map→odom`은 LiDAR 스캔매칭(SLAM Toolbox)이 드리프트 흡수. 마모·마찰·습도 같은 변수는 이 구조에서 흡수되므로 휠 오도메트리를 완벽하게 만들 필요 없음. Week 4에 `robot_localization` EKF로 정식화 예정.

## 🧊 "다운"은 전원 차단이 아니라 커널 hang일 가능성 (2026-08-16, 진단 방향 전환)

**결정적 관측(사용자)**: 다운 시점에 **LiDAR·모터드라이버 전원이 그대로 공급 중이었고, Pi 녹색 LED도 켜져 있었음.** 전원이 끊겼다면 LED가 전부 꺼졌어야 함 → **전원 차단 가설 대부분 폐기.** 사용자 표현대로 "윈도우 블루스크린" 같은 **커널 패닉/하드 락업**으로 보는 게 맞음.

**이 관점에서 기존 단서들이 재해석됨**:
- 부팅 로그에 있던 `dma dma2chan2: BUG: dma2chan2, IRQ with no descriptors` 와 `ttyAMA ttyAMA0: 40 input overrun(s)` — **ttyAMA0는 LiDAR가 230400bps로 쏟아붓는 포트**. DMA/인터럽트 처리 문제를 시사.
- **가설**: LiDAR 고속 UART(DMA) + SLAM CPU 점유가 겹치면서 인터럽트/DMA 처리가 밀려 커널 hang. → "맵핑 중에만 죽는다"와 정합적이고, CPU 100%인데 안 죽었던 반례(그땐 LiDAR 부하 없음)도 설명됨.
- 로그가 안 남는 것도 자연스러움 — 패닉 시 디스크에 쓰지 못하고 멈추므로.

**다음 검증(우선순위)**:
1. **하드웨어 워치독 활성화** — hang 시 자동 재부팅되면 그 자체가 전원 아닌 hang의 증거.
2. **pstore/ramoops 설정** — 커널 패닉 메시지를 재부팅 후에도 읽도록.
3. LiDAR를 USB 시리얼 어댑터로 옮겨 GPIO UART DMA 경로 자체를 회피 테스트.

## (이전 방향) 전원 다운 — 원인 미확정 (2026-08-16 기준, 가설 단계)

**사용자가 대조 실험으로 범위를 크게 좁힘**:

| 조건 | 결과 |
|---|---|
| 맵핑 ON + 장시간 주행 | **3회 전부 다운** |
| 맵핑 OFF + 장시간 주행 | ✅ **멀쩡** (2026-08-16 확인) |

→ **가설: 방아쇠가 주행(모터)보다 SLAM의 CPU 부하 쪽일 수 있음.** ⚠️ 다만 이건 관측 1회에 기반한 상관관계일 뿐 인과가 확정된 게 아님 — 맵핑 OFF 주행이 더 짧았거나, 배터리 충전 상태·주행 경로·발열 이력 등 통제 안 된 변수가 얼마든지 있음. 반복 재현 전까지 단정하지 말 것. 특히 맵이 커진 뒤의 **루프 클로저**가 순간적으로 CPU를 폭증시키는데, 3회 모두 "한참 돌아다닌 뒤"에 발생한 것과 일치. Claude Code 명령 실행도 같은 방아쇠였던 것으로 보임(부하를 얹는 행위라는 점에서 동일).
- **1초 트레이스에 CPU가 5~12%로 낮게 찍힌 건 모순이 아님** — 수백 ms 스파이크는 1초 평균에서 평탄화됨. 마찬가지로 5V가 5.2V로 보이는 것도 ms 단위 붕괴는 못 잡기 때문.
- **완화 조치 적용(2026-08-16)**: `mapper_params_online_async.yaml`에서 `throttle_scans 1→2`, `map_update_interval 5→10`, `scan_buffer_size 10→5`, `loop_search_maximum_distance 3.0→2.0`.
- **다른 가설도 여전히 살아있음**: 전원 여유 부족(벅컨버터/배터리 내부저항), 커넥터 접촉 불량, 벅컨버터 개체 불량, BMS 보호 동작 등. 아직 어느 것도 배제 못 함.\n- **전원 여유 부족 가설** — CPU 스파이크를 벅컨버터/배터리가 못 받아줌. **다음 세션 검증: Pi를 USB 파워뱅크로 분리 급전하고 맵핑 재현.** 안 죽으면 확정.
- 참고: 주행 직후 벅컨버터(XY-3606) 온도 확인도 아직 안 함.
- ⚠️ **CPU 부하 가설의 반례(2026-08-16)**: `power_trace.csv` 분석 결과 CPU가 **100%까지 튄 구간이 16건** 있었는데(빌드/재시작 시), 그때 5V는 5.15~5.19V로 평상시(5.20)에서 겨우 0.05V만 내려갔고 **다운도 안 났음**. 온도도 최대 57°C로 여유. 즉 CPU 부하만으로는 전원이 무너지지 않는다는 뜻 — SLAM 부하 가설을 단독 원인으로 보기 어려움. (단 1초 평균이라 ms 스파이크는 여전히 관측 불가.)

## (이전 가설) 전원 다운 3회 — 열 스트레스 + CPU 스파이크 (2026-08-15)

**3차 사고가 원인을 크게 좁혀줌**: 한참 주행 후 컴퓨터 옆까지 조종해 와서 **모터 정지 상태로 대기 중**, 사용자가 Claude Code에 말을 걸어 **명령이 실행되는 순간** 다운. 로봇을 만지거나 옮기지 않았음(진동/접촉 아님). 직후 배터리 실측 11.3V로 방전도 아님. **2회 연속 같은 패턴.**

**`power_trace.csv` 마지막 20초는 완벽하게 정상**: 5V 레일 5.16~5.21V, CPU 45~48°C, CPU 5~12%, 메모리 41%, 스로틀 0. 즉 **아무 전조 없이 즉사**.

**최유력 가설(사용자가 제기, 타당함): 누적 열 스트레스 + CPU 전류 스파이크의 조합**
1. 장시간 주행으로 배터리 내부저항 상승 + 벅컨버터 발열 (모터를 멈춰도 즉시 회복 안 됨)
2. 그 취약한 상태에서 SSH 명령 실행 → Pi CPU 순간 부하 → Pi 전류 급증(Pi5는 피크 수 A)
3. 내부저항 × 급증 전류 = 순간 전압 강하 → 벅컨버터 dropout → Pi 리셋
- **1초 샘플링 트레이스에 안 잡히는 게 모순이 아님**: Pi 리셋에는 수 ms 전압 붕괴면 충분한데 트레이스는 1초 간격이라 원리적으로 못 잡음. 배터리 11.3V도 *무부하* 전압이라 정상으로 보이는 게 당연.

**다음 세션에서 할 검증(순서대로)**:
1. **벅컨버터(XY-3606) 온도 확인** — 주행 직후 손으로 만져 뜨거우면 가설 강화.
2. **Pi를 USB 파워뱅크로 분리 급전하고 동일 시나리오 재현** — 이게 가설 검증과 임시 해결을 동시에 함. 안 죽으면 원인이 공유 전원 경로임이 확정됨.
3. 확정되면 근본 대책: 여유 있는 벅컨버터+방열, 또는 모터/Pi 배터리 완전 분리.

**주의**: 원인 미해결 상태이므로 장시간 맵핑 중 언제든 재발 가능. `map_autosave`(60초)가 유일한 방어선.

## (이전 기록) 전원 다운 사고 2건 (2026-08-15)

**2차 사고가 더 심각함**: 집 한 바퀴 맵핑을 마친 직후 Pi가 그냥 죽음. **배터리는 11.3V로 충분했고, `power_monitor.log`에 저전압/스로틀링 경고가 단 한 건도 안 찍혔음.** 시스템 로그도 23:10:14에서 정상 종료 흔적 없이 뚝 끊김 → 점진적 전압 강하가 아니라 **순간적 전원 차단**.

- **가장 유력한 원인: BMS 과전류 보호 작동.** 4WD 동시 가속이나 문턱/카펫에서 스톨에 가까워지면 순간 전류가 급증하는데, BMS는 이때 예고 없이 배터리 출력을 즉시 끊음 — 전압이 서서히 떨어지는 과정이 없으니 로그에 아무것도 안 남는 게 이 패턴과 정확히 일치. 주행 중 진동에 의한 커넥터 순간 단선도 배제 못 함.
- **1차 사고(같은 날 저녁)는 별개**: 그건 실제 배터리 방전이었음.
- **대책으로 넣은 것**: `tools/power_monitor.py`가 이제 매 초 `power_trace.csv`에 전압/온도/CPU/스로틀 비트를 append하고 매번 `fsync` — 전원이 끊겨도 직전 기록이 남도록. **다음 사고가 나면 이 CSV부터 볼 것.**
- **아직 못 밝힘**: 모터 라인 전류를 못 재서 확증이 없음. INA219(3.2A 한계)로는 20A 모터 라인을 못 재므로, 확인하려면 홀 방식 전류센서(ACS712/758 등)가 별도로 필요. 배터리 전압 급락은 INA219로 감지 가능.
- **맵 유실**: 사고로 한 바퀴 돈 맵이 통째로 날아감(저장 전이었음) → `tools/map_autosave.py` + `smart-trash-map-autosave.service` 추가, 60초마다 `~/maps/latest.pgm|yaml`로 자동 저장.
- **조종 문제도 관측됨**: 집 안을 돌아다니는 동안 버튼이 자주 안 먹었고, `cmd_vel 타임아웃 — 정지` 경고가 반복 발생(23:09:04/16/33). 와이파이 신호 약화로 명령이 끊긴 것으로 보임. 타임아웃을 늘리는 건 위험(제어 불능 상태로 계속 주행)하므로, **근본 해결은 네트워크 의존을 줄이는 자율 탐색 쪽**.

## (1차) 배터리 방전 사고 (2026-08-15)

주행 테스트 중 **배터리 방전으로 시스템이 그냥 다운됨**. 파일시스템 손상은 없었지만(ext4 저널링, journald가 저널만 재생성) 반복되면 SSD 손상·리튬이온 수명 저하·주행 중 제어불능 위험.

- **Yahboom 드라이버로는 배터리 전압을 못 읽음** — `$volt:`/`$voltage:`/`$bat:`/`$battery:`/`$vin:`/`$power:`/`$adc:` 전부 `Command not found` 확인(2026-08-15).
- **Pi5에는 자체 ADC가 없음.** 배터리 전압 직접 측정하려면 I2C ADC 모듈(ADS1115 등) + 전압분배 저항 필요 — 미구매 상태.
- 당장 가능한 SW 대책은 Pi 5V 레일 급락 감지뿐인데, 벅컨버터가 마지막까지 5V를 유지하다 급락해서 **예고 시간이 매우 짧음**(조기 경보로는 부족, 파일시스템 보호용 긴급 종료 정도).

## 🖥️ 진단 대시보드 (systemd 서비스로 자동 실행, 2026-08-15)

5개 대시보드 전부 systemd 서비스로 등록, 재부팅해도 자동 시작(`Restart=on-failure`). 허브(8764)에서 전부 카드로 요약 확인, 클릭하면 상세로 이동.

| 서비스 | 포트 | 내용 |
|---|---|---|
| `smart-trash-dashboard-hub` | 8764 | 허브(전체 요약) |
| `smart-trash-power-monitor` | 8765 | Pi5 전원/PMIC |
| `smart-trash-motor-dashboard` | 8766 | 모터 4채널 상태+수동조작(레버로 속도조절) |
| `smart-trash-imu` + `smart-trash-lidar` + `smart-trash-sensor-dashboard` | 8767 | IMU/LiDAR 실시간 상태(sensor_dashboard.py가 앞 둘의 토픽을 구독해서 서빙) |
| `smart-trash-map-dashboard` | 8768 | `/map`(SLAM Toolbox 발행) 실시간 시각화 — **SLAM Toolbox가 안 떠있으면 "맵핑 미실행"이 정상 상태**, `mapping.launch.py` 실행하면 자동으로 그리드가 채워짐. Pillow 등 이미지 라이브러리 없이 `<canvas>`에서 직접 렌더링(base64로 그리드 원시 바이트만 전송). |

⚠️ **`smart-trash-lidar`와 `mapping.launch.py`를 동시에 켜면 `/dev/ttyAMA0` 포트 충돌남** (둘 다 LD06 노드를 띄우려 함) — `mapping.launch.py`로 실제 맵핑 세션 돌릴 땐 먼저 `sudo systemctl stop smart-trash-lidar.service`로 내리고, 끝나면 `start`로 복구할 것. (`smart-trash-map-dashboard`는 `/map` 구독만 하는 별개 프로세스라 이 충돌과 무관, 계속 켜둬도 됨.)

`tools/` 밑 3개 대시보드(`power_monitor.py`, `motor_dashboard.py`, `dashboard_hub.py`)는 **systemd 서비스로 등록**돼 있어 **Pi 재부팅해도 수동으로 안 띄워도 자동 시작**됨. 유닛 파일은 `tools/systemd/*.service`에 버전관리되고, `/etc/systemd/system/`에 복사돼 `enable`된 상태(심볼릭 링크 생성됨). 죽으면 3초 후 자동 재시작(`Restart=on-failure`).

- **접속 주소** (Pi IP는 유동적일 수 있음, `hostname -I`로 확인): 허브 `http://<Pi IP>:8764` (여기서 시작, 카드 클릭하면 상세로 이동), 전원 `:8765`, 모터 `:8766`.
- **관리 명령**: `sudo systemctl status/restart/stop smart-trash-{power-monitor,motor-dashboard,dashboard-hub}.service`, 로그는 `sudo journalctl -u smart-trash-motor-dashboard -f` 식으로.
- **motor_dashboard.py가 시리얼 포트(`/dev/ttyUSB0`)를 독점 소유**함 — 이 서비스가 떠 있는 동안은 다른 스크립트/노드가 같은 포트를 못 엶(`Device or resource busy`). 임시로 직접 `pyserial`로 테스트하려면 먼저 `sudo systemctl stop smart-trash-motor-dashboard`로 내려야 함. 나중에 `smart_trash_motion` ROS2 노드를 만들면 이 대시보드와 포트를 두고 경합하게 되므로, **그때 가서 모터 제어 소유권을 노드 쪽으로 넘기고 대시보드는 노드의 상태 토픽을 구독하는 방식으로 리팩터링 필요** (지금은 임시 진단용 구조).
- 유닛 파일 내용을 바꾸면 `~/smart-trash-bin/tools/systemd/*.service` 원본을 고치고 `sudo cp`로 다시 복사 + `sudo systemctl daemon-reload` + 해당 서비스 `restart`까지 해야 반영됨 (자동 동기화 안 됨).

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
- **회전 방향 확정**: 바퀴 4개 개별로 PWM 양수값 줬을 때(바깥에서 본 기준) M1/M2(왼쪽)는 반시계, M3/M4(오른쪽)는 시계 — 좌우 거울대칭 장착 감안하면 이게 정확히 "전진" 방향에 해당함(차량 바퀴 회전기하학상 왼쪽=반시계·오른쪽=시계가 전진). **결론: `$pwm:` 4채널 전부 양수값 = 전진, 좌우 부호 반전 로직 불필요.** 바닥에 내려놓고 실제 주행 테스트(10초, PWM 600 동일값 4채널)로 4바퀴 전부 같은 방향(전진)으로 굴러가는 것 육안 확인 완료.
- **참고(추가 검증 필요)**: 바닥 접지 상태에서 PWM 500·0.5초 테스트 시 M4(오른쪽앞)만 인코더 변화량이 유독 작았음(-9 vs 나머지 -880~-970대) — 마찰/데드존이 그 바퀴만 살짝 높을 가능성. `smart_trash_motion` 노드에서 채널별 최소 PWM(데드존) 캘리브레이션할 때 M4 우선 확인할 것.
- **바닥 실주행 테스트 전부 통과 (2026-08-14)**: 전진(`$pwm:500,500,500,500#`), 후진(`$pwm:-500,-500,-500,-500#`), 제자리 회전(`$pwm:500,500,-500,-500#` = 왼쪽전진+오른쪽후진 = 시계방향/우회전) 전부 육안으로 의도대로 동작 확인, 정지도 매번 정상. **회전 캘리브레이션 참고값**: PWM ±500, 1.5초 → 약 90도 회전.

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
- **I2C 권한**: `/dev/i2c-1` 소유 그룹이 재부팅마다 `dialout`이었다가 `i2c`였다가 바뀌는 걸 확인함(udev 규칙 이슈로 추정) — 매 세션 `ls -la /dev/i2c-1`로 실제 소유 그룹 확인 후 `carrotian`이 그 그룹에 있는지(`groups` 명령) 체크할 것. 둘 다 이미 추가는 해뒀음(`sudo usermod -aG dialout,i2c carrotian`, 2026-08-01/08-15). **재로그인/재부팅해야 그룹 추가가 적용됨** — 같은 세션 안에서 바로 써야 하면 아래 `sg` 함정 참고.
- ⚠️ **`sg <group> -c "명령어"`로 ROS2 명령을 실행할 때 함정(2026-08-15 발견)**: `sg`가 실행하는 하위 셸은 POSIX `sh`라 `source /opt/ros/jazzy/setup.bash`가 **조용히 실패**(`source: not found`)하거나, 경우에 따라 `LD_LIBRARY_PATH` 등 ROS 환경변수가 하위 프로세스로 안 넘어가서 `rclpy` C 익스텐션 로드 실패(`librcl_action.so: cannot open shared object file`) 또는 **CPU 100%로 아무 로그도 없이 멈추는(hang) 증상**으로 나타남 — 에러 메시지가 애매해서 IMU 하드웨어 문제로 오인하기 쉬움. **반드시 `sg <group> -c "bash -c 'source /opt/ros/jazzy/setup.bash && ...'"` 처럼 `bash -c`로 한 번 더 감싸서 그 안에서 소싱**할 것. (I2C 자체 통신은 순수 파이썬으로는 `sg` 없이도, 있어도 항상 정상 동작했음 — 이 문제는 ROS2/rclpy를 끼고 쓸 때만 재현됨.)
- **IMU 실제 하드웨어 검증 완료 (2026-08-15)**: GPIO 1/3/5/9번 핀(3.3V/SDA/SCL/GND, I2C1 버스)으로 연결, `i2cdetect -y 1`에서 0x69 정상 감지. `imu_publisher_node` 실행 후 `/imu/data` 토픽에 실제 물리적으로 타당한 값 확인됨 — 정지 상태에서 `linear_acceleration.z ≈ 9.88 m/s²`(중력가속도와 거의 일치, 스케일/축 방향 정상 확인), `angular_velocity`는 0에 가까움(정지 상태와 일치). Week 1 IMU 체크리스트 완료.
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
