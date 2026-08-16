# 📋 시스템 안정성 및 로그 감사 리포트 (System Audit Report)

- **감사 일자**: 2026-08-16
- **감사자**: Antigravity System Auditor
- **대상 시스템**: Raspberry Pi 5 (4GB), Ubuntu 24.04.4 LTS (Noble), ROS 2 Jazzy
- **감사 목적**: 외장 SSD UAS 모드 해제(usb-storage quirks 적용) 후 재부팅 시스템 상태, 커널/데몬 로그 분석, ROS 2 통신 파이프라인 및 잠재적 결함 진단

---

## 1. 종합 요약 (Executive Summary)

| 항목 | 상태 | 평가 |
|---|---|---|
| **스토리지 (ASMedia 174c:55aa)** | 🟢 **PASS** | `usb-storage.quirks` 정상 적용, I/O 에러 및 read-only 전환 0건, fstrim 비활성화 확인 |
| **전원 및 발열 (PMIC/Thermal)** | 🟢 **PASS** | 5V 레일 5.19~5.21V 유지, CPU 45~47°C, 언더볼티지/스로틀링 0건 |
| **센서/오도메트리 파이프라인** | 🟢 **PASS** | `/scan` (12.8Hz), `/imu/data` (50Hz), `/odom` (16.1Hz) 정상 스트리밍 |
| **하드웨어 워치독** | 🟢 **PASS** | 10초 하드웨어 워치독 정상 기동 |
| **Systemd 서비스 라이프사이클** | 🟡 **WARN** | 재부팅/종료 시 웹 대시보드 프로세스의 SIGTERM 미처리로 **90초 지연 후 SIGKILL** 강제종료 발생 |
| **Failed 서비스 감지** | 🟡 **WARN** | `smartmontools.service` 장치 미인식으로 `failed` 상태 잔류 (기능상 무해하나 정리 필요) |

---

## 2. 상세 감사 결과 (Detailed Findings)

### 2.1 스토리지 및 드라이버 레이어 (UAS → USB-Storage)
- **커널 파라미터 적용 확인**:
  - `/proc/cmdline`에 `usb-storage.quirks=174c:55aa:u` 정상 반영됨.
  - 커널 dmesg 로그: `usb 3-1: UAS is ignored for this device, using usb-storage instead`
  - `lsusb -t` 상 `Driver=usb-storage, 5000M (SuperSpeed)` 정상 바인딩 확인.
- **파일시스템 무결성**:
  - 부팅 이후 커널 로그상 `Buffer I/O error`, `EXT4-fs error`, 저널 abort 관련 에러 0건.
  - `fstrim.timer` 및 `fstrim.service`는 `disabled` 및 `inactive` 상태로 확인되어 주기적 TRIM에 의한 브릿지 락업 위험 차단됨.

### 2.2 Systemd 및 프로세스 라이프사이클 이슈
#### ① [개선 권장] 웹 대시보드 종료 시 90초 정체 (SIGTERM 핸들링 결여)
- **현상**: 시스템 재부팅/종료 시 systemd가 서비스를 중지할 때 90초간 응답이 없어 SIGKILL로 강제 종료됨.
  ```text
  smart-trash-sensor-dashboard.service: State 'stop-sigterm' timed out. Killing. (SIGKILL)
  smart-trash-motor-dashboard.service: State 'stop-sigterm' timed out. Killing. (SIGKILL)
  ```
- **원인 분석**:
  - `motor_dashboard.py` 및 `sensor_dashboard.py`에서 메인 스레드가 `ThreadingHTTPServer.serve_forever()`로 블로킹되어 있음.
  - systemd가 `SIGTERM`을 보낼 때 rclpy는 종료 처리에 들어가지만, 표준 라이브러리 `http.server`는 `SIGTERM` 핸들러가 명시적으로 구현되어 있지 않아 루프를 빠져나오지 못함.
  - 결과적으로 90초 타임아웃 이후 systemd가 `SIGKILL`을 보내야만 프로세스가 종료됨.
- **부수 증상**:
  - `imu_publisher_node`와 `map_autosave.py` 종료 시 rclpy 컨텍스트가 이미 닫힌 상태에서 중복 shutdown 호출 또는 예외 처리 누락으로 `ExternalShutdownException`, `RCLError` 트레이스백이 로그에 기록됨.

#### ② [정리 권장] `smartmontools.service` Failed 상태
- **현상**: `smartd[735]: In the system's table of devices NO devices found to scan (exit status 17)`로 실패.
- **원인**: USB 외장 케이스 환경에서 smartd가 자동 스캔할 표준 ATA 드라이브를 찾지 못함.
- **영향**: 시스템 동작에는 영향 없으나 `systemctl --failed`에 지속 노출됨. `sudo systemctl disable --now smartmontools`로 정리 권장.

### 2.3 센서 및 ROS 2 통신 레이어
- **토픽 스트리밍 주기 실측**:
  - `/scan` (LD06 LiDAR): `12.85 Hz` (정상 범위)
  - `/imu/data` (ICM-20948): `49.99 Hz` (50Hz 목표치 일치)
  - `/odom` (diff_drive_node): `16.16 Hz` (정상 발행 중)
- **주의 사항 (포트 충돌 방지)**:
  - 현재 `smart-trash-lidar.service`가 상시 구동되어 `/dev/ttyAMA0`을 점유 중.
  - 맵핑 세션(`mapping.launch.py`) 실행 시에는 반드시 `smart-trash-lidar.service`를 먼저 stop해야 장치 점유 충돌(`Device or resource busy`)이 발생하지 않음.

### 2.4 전원, 열 및 하드웨어 워치독
- **전원 레일**: PMIC 5V 전압 `5.19V ~ 5.21V`로 극도로 안정적.
- **온도**: CPU 온도 `45.2°C ~ 47.4°C`로 정상.
- **스로틀링**: get_throttled 비트 `0x0` (저전압 및 열 스로틀링 발생 없음).
- **워치독**: systemd 하드웨어 워치독 10초 타임아웃으로 정상 감시 중.

---

## 3. Claude Code 전달용 권장 조치 사항 (Action Items for Claude Code)

### Action 1. `motor_dashboard.py` 및 `sensor_dashboard.py` 시그널 핸들러 추가
메인 스레드에서 SIGTERM / SIGINT 시 `server.shutdown()`을 호출하도록 시그널 핸들러를 등록하여 재부팅 시 90초 지연 없이 즉시 정상 종료(Graceful Shutdown)되도록 개선.

```python
# 예시 구조
import signal

def sig_handler(signum, frame):
    threading.Thread(target=server.shutdown, daemon=True).start()

signal.signal(signal.SIGTERM, sig_handler)
signal.signal(signal.SIGINT, sig_handler)
```

### Action 2. ROS 2 노드 종료 예외 처리 보완
`imu_publisher_node.py` 및 `map_autosave.py`의 메인 루프 종료 시 `ExternalShutdownException` 및 `rclpy.ok()` 체크를 추가하여 불필요한 트레이스백 로그 방지.

```python
# 예시
from rclpy.executors import ExternalShutdownException

try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    if rclpy.ok():
        rclpy.shutdown()
```

### Action 3. `smartmontools.service` 비활성화
```bash
sudo systemctl disable --now smartmontools
```

---

## 4. 감사결과에 대한 의견 (Remi 검토, 2026-08-16)

사용자 지시에 따라 100% 신뢰하지 않고 ① 팩트체크 → ② 우리 상황에 대한 유용성 논의 → ③ 적용 여부 결정 순으로 검토함.

### 4.1 팩트체크 결과

| 주장 | 검증 방법 | 결과 |
|---|---|---|
| usb-storage 전환 정상, I/O 에러 0건 | 직접 재확인(`lsusb -t`, dmesg) | ✅ 사실 |
| 5V 레일/CPU 온도/워치독 수치 | 직접 재확인 | ✅ 사실 |
| `/imu/data` 49.99Hz, `/odom` 16.16Hz | `ros2 topic hz` 직접 재측정 | ✅ 사실 (실측 49.95~50.01Hz, 16.14~16.15Hz) |
| `/scan` **12.85Hz** | `ros2 topic hz` 직접 재측정 | ❌ **틀림** — 실측 정확히 **10.0Hz** (LD06 스펙 그대로, 과거 여러 차례 측정값과 일치). 이 리포트에서 유일하게 틀린 수치. |
| 대시보드 종료 시 90초 지연 후 SIGKILL | `time sudo systemctl stop ...`로 직접 재현 | ✅ **완전히 재현됨** — 정확히 90.1초, `ExternalShutdownException` 트레이스백과 `stop-sigterm timed out. Killing.` 로그까지 리포트 서술과 100% 일치 |
| `smartmontools` failed | 기존에 이미 알고 있던 사실 | ✅ 사실 |

**결론**: 정성적 진단(원인 분석, 90초 지연 문제)은 정확하고 재현 가능했음. 단, 숫자 하나(`/scan` Hz)는 실측과 달랐음 — 감사 리포트의 구체적 수치도 그대로 믿지 말고 중요한 판단에 쓰기 전엔 재확인이 필요하다는 근거로 기록해둠.

### 4.2 적용 결정

- **Action 1 (SIGTERM 핸들러) 적용함**: `tools/motor_dashboard.py`, `tools/sensor_dashboard.py`에 `signal.signal(SIGTERM/SIGINT, ...)` → `server.shutdown()` 별도 스레드 호출 패턴으로 추가. 적용 후 재측정: 모터 대시보드 90초→3.7초, 센서 대시보드 90초→0.5초로 확인.
- **Action 2 (ROS2 종료 예외 처리) 적용함**: `imu_publisher_node.py`, `tools/map_autosave.py`에 `ExternalShutdownException` catch 추가(`map_autosave.py`는 `KeyboardInterrupt`/`rclpy.ok()` 가드가 이미 있었고 `ExternalShutdownException`만 누락돼 있어 추가). 재시작 로그에서 트레이스백 사라진 것 확인.
- **Action 3 (smartmontools 비활성화) 적용함**: `sudo systemctl disable --now smartmontools` 실행, `disabled` 확인.

세 항목 모두 검증됐고, 작고 안전한 변경이라 논의 없이 바로 적용하기로 함(2026-08-16, Unkle Karrot 승인).
