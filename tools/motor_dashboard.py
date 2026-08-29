#!/usr/bin/env python3
"""4채널 모터 원격 조종 + 상태 대시보드 (ROS2 토픽 기반).

⚠️ 2026-08-15 리팩터링: 예전에는 이 대시보드가 시리얼 포트를 직접 열어 $pwm:을
쐈는데, 그러면 diff_drive_node와 포트를 두고 싸워서 "조종하면서 맵 그리기"가
불가능했다. 이제는 시리얼을 전혀 안 건드리고 ROS2 토픽만 쓴다:

    발행: /cmd_vel        (geometry_msgs/Twist)  — 조종 명령
    구독: /odometry/filtered (nav_msgs/Odometry) — EKF(robot_localization) 융합 후
                              실제 이동/회전 상태 (2026-08-16 도입). /odom(원본,
                              순수 휠 기반)은 회전각 신뢰도가 낮아서 대신 이걸 씀.
    구독: /wheel_encoders (std_msgs/Int32MultiArray) — 바퀴별 회전 표시용

따라서 diff_drive_node가 반드시 떠 있어야 동작한다(그 노드가 시리얼 유일 소유자).
SLAM/맵핑과 동시에 켜둘 수 있어서, 무선조종 하듯 집안을 돌며 맵을 그릴 수 있다.

안전장치:
  - 버튼을 누르고 있는 동안만 주행, 떼면 램프로 감속 정지
  - diff_drive_node의 cmd_vel 타임아웃(0.5초)이 최종 방어선 — 이 대시보드가
    죽거나 와이파이가 끊겨도 로봇은 자동으로 멈춘다
  - 클라이언트 측 최대 홀드 시간(2초) 제한

사용법:
    python3 tools/motor_dashboard.py [--port 8766]
"""
import argparse
import collections
import gzip
import json
import math
import signal
import threading
import time
import urllib.parse
# 비상정지는 맵 대시보드와 **같은 코드**를 쓴다. 복사해 두면 한쪽만 고쳐져
# 어긋나고, 하필 비상정지에서 그러면 위험하다.
from estop_common import kill_autonomous_processes

from dashboard_icon import FAVICON_TAG
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

try:
    import smbus2
except ImportError:
    smbus2 = None

# 조종 상한 — diff_drive_node의 max_wheel_speed(1.0 m/s)보다 보수적으로 잡음.
# 실내 맵핑 주행은 느릴수록 SLAM 품질이 좋아서 일부러 낮게 둔다.
MAX_LINEAR = 1.0   # m/s (diff_drive_node의 max_wheel_speed=1.5 대비 여유)
MAX_ANGULAR = 3.0  # rad/s
STALE_S = 2.0

state_lock = threading.Lock()
state = {
    "connected": False,      # diff_drive_node가 살아있고 토픽이 흐르는지
    "encoder": [0, 0, 0, 0],
    "rate": [0.0, 0.0, 0.0, 0.0],   # counts/sec (바퀴별 회전 표시용)
    "linear": 0.0,           # m/s (odom 실측)
    "angular": 0.0,          # rad/s (odom 실측)
    "commanded": [0.0, 0.0],  # [linear, angular] 마지막 조종 명령
    "last_ts": 0.0,
    "manual_blocked": False,  # 자율주행(controller_server) 중이라 수동 명령이 막혔는지
}

# 배터리 전압(INA219, 0x40). ROS 토픽이 아니라 I2C 직접 읽기 — 이 값이
# 필요한 곳이 대시보드뿐이라 굳이 노드/토픽을 새로 만들 이유가 없다.
# Vin+/Vin-는 묶어서 전압만 잰다. 2026-08-25: SDA/SCL은 정식 Qwiic 데이지체인
# 케이블로 IMU 스페어 포트에 연결됨(이전엔 물리 핀 스플라이스 임시 배선이었음
# — docs/hardware_inventory.md 변경 이력 참고).
BATTERY_I2C_ADDR = 0x40
BATTERY_POLL_S = 2.0        # 배터리는 급변하지 않음 — 이 페이지도 부하를 아낀다
BATTERY_HISTORY_LEN = 1800  # 2초 간격 * 1800 = 1시간

# 3단계 전압 경보 (2026-08-29). 3S Li-ion 기준.
# ⚠️ **이 값은 지금 장착된 알리 SUYIJIA 팩 전용이다.** 이 팩은 셀 불균형 불량품
# (docs/hardware_inventory.md 참고) — 만충이 정상(12.6V)이 아니라 11.9V까지만
# 올라가고, 실제 전원차단 사고 때 **팩 전압 10.5V(셀 불균형으로 최약 셀은 이미
# 2.8V)에서 BMS가 강제 차단**하는 게 실측 확인됐다. 그래서 "건강한 3S팩" 기준
# 표준값(과방전 한계 3.0V/cell=9.0V)을 쓰면 이 팩은 그 문턱에 도달하기 한참
# 전에 이미 하드웨어 BMS가 먼저 끊어버려 소프트웨어 경보가 무의미해진다.
# 대신 **이 팩의 실측 위험 지점(10.5V)보다 여유를 두고** 잡았다:
#   1차(주의) 11.3V, 2차(위험/적색) 10.9V, 3차(자동정지) 10.6V
#   — 10.6V는 실측 차단점(10.5V) 바로 위라, 소프트웨어가 하드웨어 BMS보다
#     먼저 개입해서 "갑작스런 전원차단" 대신 "제어된 정지"를 만드는 게 목적.
# **새 배터리(루나볼트 3S2P 등, 미주문)로 교체하면 이 값 전부 재검토할 것** —
# 건강한 팩이면 표준값(1차 11.1V/2차 10.5V/3차 9.9V, 3.7/3.5/3.3V per cell)
# 으로 다시 잡아야 한다.
BATTERY_WARN1_V = 11.3
BATTERY_WARN2_V = 10.9
BATTERY_CRITICAL_V = 10.6
# 순간 전압은 모터 기동/정지 때 수백mV~1V 이상 출렁인다(제자리 회전이 특히
# 심함 — CLAUDE.md 회전 캘리브레이션 이력 참고). 원시값을 그대로 임계 판정에
# 쓰면 순간 부하 때마다 오탐(false trip)한다. 그래서 **경보/자동정지 판정에는
# 이동평균**을 쓴다 — 정지 상태 실측 잡음이 6초에 48mV뿐이라(2026-08-29
# 확인) 노이즈 자체는 작지만, 부하 중 실제 처짐은 훨씬 크므로 몇 초는
# 걸러야 진짜 하강 추세와 순간 처짐을 구분할 수 있다. 그래프/기록용
# 원시값은 그대로 남긴다(오도메트리 진단에 순간 처짐 자체가 필요할 수 있어서
# — tools/loop_gyro_diag.py 참고).
BATTERY_AVG_WINDOW = 5      # BATTERY_POLL_S=2.0 기준 10초 이동평균

battery_lock = threading.Lock()
battery_state = {"connected": False}
battery_history = collections.deque(maxlen=BATTERY_HISTORY_LEN)


def _read_battery_voltage(bus):
    # INA219 Bus Voltage 레지스터(0x02): 상위 13비트가 4mV 단위 전압,
    # bit0=OVF(연산 오버플로), bit1=CNVR(변환 완료). smbus2는 word를
    # little-endian으로 돌려주는데 칩은 MSB 먼저 보내므로 바이트를 뒤집어야 함.
    raw = bus.read_word_data(BATTERY_I2C_ADDR, 0x02)
    val = ((raw & 0xff) << 8) | (raw >> 8)
    if val & 0x1:
        raise IOError("INA219 OVF (측정 범위 초과)")
    return (val >> 3) * 0.004


def _battery_tier(v):
    """이동평균 전압 -> ('ok'|'warn1'|'warn2'|'critical', 임계값 딕셔너리)."""
    if v <= BATTERY_CRITICAL_V:
        return "critical"
    if v <= BATTERY_WARN2_V:
        return "warn2"
    if v <= BATTERY_WARN1_V:
        return "warn1"
    return "ok"


def battery_poll_loop():
    if smbus2 is None:
        return
    bus = None
    n, total, vmin, vmax = 0, 0.0, None, None
    session_started = time.time()
    recent = collections.deque(maxlen=BATTERY_AVG_WINDOW)
    # 히스테리시스: critical에 "진입"하는 순간 딱 한 번만 자동정지를 걸고,
    # warn2 이상으로 회복하기 전까지는 다시 안 건다 — 안 그러면 임계값
    # 근처에서 값이 오르내릴 때마다 매 폴링 주기(2초)마다 반복 발동한다.
    armed = True
    while True:
        try:
            if bus is None:
                bus = smbus2.SMBus(1)
            v = _read_battery_voltage(bus)
            n += 1
            total += v
            vmin = v if vmin is None else min(vmin, v)
            vmax = v if vmax is None else max(vmax, v)
            now = time.time()
            recent.append(v)
            recent_avg = sum(recent) / len(recent)
            tier = _battery_tier(recent_avg)

            if tier == "critical" and armed:
                armed = False
                try:
                    killed = kill_autonomous_processes()
                except Exception:
                    killed = []
                try:
                    Handler.node.send_cmd(0.0, 0.0)
                except Exception:
                    pass
                print(f'[battery] 🔴 자동정지 발동 — 이동평균 {recent_avg:.2f}V '
                      f'<= {BATTERY_CRITICAL_V}V (프로세스 {killed})', flush=True)
            elif tier in ("ok", "warn1"):
                armed = True  # warn2 문턱 위로 회복해야 재무장(둘 다 정의상 >warn2) — critical 바로 밑에서 진동 방지

            with battery_lock:
                battery_state.update({
                    "connected": True, "voltage": v, "ts": now,
                    "avg": total / n, "min": vmin, "max": vmax,
                    "session_started": session_started, "samples": n,
                    "recent_avg": recent_avg, "tier": tier, "auto_stop_armed": armed,
                    "thresholds": {"warn1": BATTERY_WARN1_V, "warn2": BATTERY_WARN2_V,
                                    "critical": BATTERY_CRITICAL_V},
                })
                battery_history.append({"ts": now, "v": v})
        except Exception as e:
            with battery_lock:
                battery_state["connected"] = False
                battery_state["error"] = str(e)
            bus = None  # 다음 루프에서 재오픈 — 임시 배선이라 접촉 불량 재시도 가치 있음
        time.sleep(BATTERY_POLL_S)


class TeleopNode(Node):
    def __init__(self):
        super().__init__('motor_dashboard')
        # 발행 큐도 1 — 오래된 조종 명령이 쌓여서 뒤늦게 전달되지 않도록
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.create_subscription(Odometry, 'odometry/filtered', self._on_odom, 10)
        self.create_subscription(Int32MultiArray, 'wheel_encoders', self._on_enc, 10)
        self._prev_enc = None
        self._prev_enc_ts = None

    def _on_odom(self, msg: Odometry):
        with state_lock:
            state["connected"] = True
            state["last_ts"] = time.time()
            state["linear"] = msg.twist.twist.linear.x
            state["angular"] = msg.twist.twist.angular.z

    def _on_enc(self, msg: Int32MultiArray):
        counts = list(msg.data)
        now = time.time()
        if len(counts) != 4:
            return
        with state_lock:
            if self._prev_enc is not None and self._prev_enc_ts is not None:
                dt = max(now - self._prev_enc_ts, 1e-3)
                state["rate"] = [(counts[i] - self._prev_enc[i]) / dt for i in range(4)]
            state["encoder"] = counts
        self._prev_enc, self._prev_enc_ts = counts, now

    def autonomous_active(self):
        """Nav2 controller_server가 지금 /cmd_vel에 붙어있는지 직접 조회한다.

        2026-08-21: 자율주행(Nav2) 테스트 중 /cmd_vel 발행자가 이 대시보드,
        map_dashboard, controller_server 셋 다 동시에 있었던 게 실측으로
        확인됨(ros2 topic info --verbose) — 우선순위 중재가 없어서 속도 상한을
        지키던 Nav2 명령 사이에 수동 조종 잔여 명령이 섞여 들어갈 수 있었다.
        새 mux 노드/토픽 리매핑 대신, 발행 직전에 rclpy로 직접 물어봐서
        controller_server가 살아있으면 그냥 안 보낸다 — 상태를 따로 관리할
        필요가 없어 어긋날 일이 없다.
        """
        try:
            infos = self.get_publishers_info_by_topic('/cmd_vel')
        except Exception:
            return False
        return any(i.node_name == 'controller_server' for i in infos)

    def send_cmd(self, linear, angular):
        # 정지(0,0)는 항상 통과시킨다 — 자율주행 중이라도 사람이 세우려는
        # 시도를 막으면 안 된다. 실제 주행 명령만 자율주행 중엔 막는다.
        if (abs(linear) > 1e-9 or abs(angular) > 1e-9) and self.autonomous_active():
            with state_lock:
                state["manual_blocked"] = True
            return
        with state_lock:
            state["manual_blocked"] = False
        msg = Twist()
        msg.linear.x = float(max(-MAX_LINEAR, min(MAX_LINEAR, linear)))
        msg.angular.z = float(max(-MAX_ANGULAR, min(MAX_ANGULAR, angular)))
        self.pub.publish(msg)
        with state_lock:
            state["commanded"] = [msg.linear.x, msg.angular.z]


def staleness_watchdog():
    """diff_drive_node가 죽으면 '연결 안 됨'으로 표시 (가짜 초록불 방지)."""
    while True:
        with state_lock:
            if state["connected"] and time.time() - state["last_ts"] > STALE_S:
                state["connected"] = False
                state["rate"] = [0.0, 0.0, 0.0, 0.0]
        time.sleep(0.5)


PAGE = """<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
__FAVICON__
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>모터 대시보드</title>
<style>
  #estopBtn { width:100%; margin:.5rem 0; padding:.9rem; font-size:1.05rem;
              font-weight:700; border-radius:10px; cursor:pointer;
              background:#c0392b; color:#fff; border:none; }
  #estopBtn:active { background:#96281b; }
  #estopMsg { font-size:.85rem; margin:.25rem 0 .5rem; }
  :root {
    color-scheme: light;
    --surface-1: #fcfcfb; --page: #f9f9f7;
    --text-primary: #0b0b0b; --text-secondary: #52514e; --text-muted: #898781;
    --border: rgba(11,11,11,0.10);
    --accent: #2a78d6; --accent-soft: #cde2fb;
    --good: #0ca30c; --critical: #d03b3b;
    --wheel-idle: #e1e0d9;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --surface-1: #1a1a19; --page: #0d0d0d;
      --text-primary: #ffffff; --text-secondary: #c3c2b7; --text-muted: #898781;
      --border: rgba(255,255,255,0.10);
      --accent: #3987e5; --accent-soft: #184f95;
      --good: #0ca30c; --critical: #e66767;
      --wheel-idle: #2c2c2a;
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --surface-1: #1a1a19; --page: #0d0d0d;
    --text-primary: #ffffff; --text-secondary: #c3c2b7; --text-muted: #898781;
    --border: rgba(255,255,255,0.10);
    --accent: #3987e5; --accent-soft: #184f95;
    --good: #0ca30c; --critical: #e66767;
    --wheel-idle: #2c2c2a;
  }
  body { background: var(--page); color: var(--text-primary);
    font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
    max-width: 720px; margin: 24px auto; padding: 0 16px; }
  h1 { font-size: 1.3rem; display: flex; align-items: center; gap: 8px; }
  .dot { width: 10px; height: 10px; border-radius: 50%; display: inline-block; }
  .card { background: var(--surface-1); border: 1px solid var(--border); border-radius: 12px;
    padding: 16px; margin: 16px 0; }
  #chassis { display: block; margin: 0 auto; }
  .wheel-label { font-size: 11px; fill: var(--text-secondary); }
  .readout { font-size: 12px; fill: var(--text-muted); font-variant-numeric: tabular-nums; }
  .grid2 { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; margin-top: 12px; }
  .pad-btn { padding: 18px 0; border-radius: 10px; border: 1px solid var(--border);
    background: var(--surface-1); color: var(--text-primary); font-size: 1.1rem;
    cursor: pointer; user-select: none; touch-action: none; }
  .pad-btn:active { background: var(--accent-soft); border-color: var(--accent); }
  #stopBtn { grid-column: span 3; background: var(--critical); color: white; border: none;
    padding: 14px 0; border-radius: 10px; font-size: 1rem; font-weight: 600; cursor: pointer; }
  .status-row { display: flex; justify-content: space-between; font-size: 0.85rem;
    color: var(--text-secondary); margin-top: 4px; }
  .warn { font-size: 0.8rem; color: var(--text-muted); margin-top: 10px; }
</style>
</head>
<body>
<h1><span class="dot" id="connDot" style="background:var(--critical)"></span> 모터 대시보드</h1>
<button id="estopBtn">🛑 비상정지 (자율주행 전체 중단)</button>

<a href="/battery" class="card" style="display:block; text-decoration:none; color:inherit;">
  <div class="status-row" style="margin:0;">
    <span id="battBadge">🔋 —</span>
    <span style="color:var(--accent);">상세 →</span>
  </div>
</a>

<div class="card">
  <svg id="chassis" width="260" height="340" viewBox="0 0 260 340"></svg>
  <div class="status-row">
    <span id="connText">연결 확인 중...</span>
    <span id="moveText">정지</span>
  </div>
</div>

<div class="card">
  <div class="grid2">
    <button class="pad-btn" data-cmd="left">↺ 좌회전</button>
    <button class="pad-btn" data-cmd="fwd">↑ 전진</button>
    <button class="pad-btn" data-cmd="right">↻ 우회전</button>
    <div></div>
    <button class="pad-btn" data-cmd="back">↓ 후진</button>
    <div></div>
    <button id="stopBtn">■ 정지</button>
  </div>
  <div class="warn">버튼을 누르고 있는 동안만 움직이고, 떼면 즉시 정지합니다. 바퀴가 바닥에 닿아 있으면 실제로 움직이니 주의하세요.</div>
</div>

<div class="card">
  <div class="status-row" style="margin-bottom:6px;">
    <span>출력 레버</span>
    <span id="speedLabel" style="font-variant-numeric: tabular-nums;"></span>
  </div>
  <input type="range" id="speedSlider" min="10" max="80" step="1" value="40" style="width:100%;">
</div>

<script>
// 서버(motor_dashboard.py)의 상한과 일치. PWM이 아니라 cmd_vel 속도 단위.
const MAX_LINEAR = 1.0;   // m/s — 서버 상한과 일치
const MAX_ANGULAR = 3.0;  // rad/s
const slider = document.getElementById('speedSlider');
const speedLabel = document.getElementById('speedLabel');

function speedFrac() {
  return Number(slider.value) / 100;
}
function updateSpeedLabel() {
  const f = speedFrac();
  speedLabel.textContent =
    `${slider.value}% (${(MAX_LINEAR * f).toFixed(2)} m/s · ${(MAX_ANGULAR * f).toFixed(1)} rad/s)`;
}
slider.addEventListener('input', updateSpeedLabel);
updateSpeedLabel();

function cmdsForSpeed() {
  const f = speedFrac();
  const v = MAX_LINEAR * f;
  const w = MAX_ANGULAR * f;
  return {
    fwd:   {linear:  v, angular: 0},
    back:  {linear: -v, angular: 0},
    left:  {linear: 0, angular:  w},   // ROS 표준: +z = 반시계 = 좌회전
    right: {linear: 0, angular: -w},
  };
}

// 방향 버튼: 누르면 0->목표까지, 떼면 목표->0까지 약 500ms에 걸쳐 부드럽게 램프.
// (■ 정지 버튼은 이 램프를 건너뛰고 즉시 0을 보내는 별도 hardStop — 안전 우선)
const TICK_MS = 50;
// 가속은 부드럽게(기계 충격 방지), 감속은 빠르게(조종 반응성/안전).
// 예전엔 둘 다 0.1이라 버튼을 떼도 0.5초간 더 나가서 조종이 안 맞았음.
const RAMP_UP_STEP = 0.1;    // 약 10틱 = 500ms에 최고속 도달
const RAMP_DOWN_STEP = 0.5;  // 약 2틱 = 100ms 안에 정지
const MAX_HOLD_MS = 5000;  // 뗀 신호(pointerup)를 못 받는 경우를 대비한 안전장치 —
                            // 아무리 눌려 있어도 이 시간 지나면 강제로 놓은 것처럼 처리
                            // (2026-08-16: 2초는 의도적으로 길게 누르는 조종에 너무
                            // 짧아서 5초로 완화 — diff_drive_node의 cmd_vel_timeout
                            // 0.5초가 진짜 최종 안전장치라 이건 여유있게 잡아도 됨)

let currentScale = 0;
let targetScale = 0;
let activeCmd = null;
let tickTimer = null;
let holdStartedAt = 0;
let driveInFlight = false;
let prevScale = 0;  // 이전 /api/drive 응답 오기 전엔 새로 안 쏨 — 요청 밀림 방지

document.getElementById('estopBtn').onclick = async () => {
  // 되돌릴 수 없는 동작은 아니지만 주행을 통째로 끊으므로 확인을 받는다.
  if (!confirm('자율주행 프로세스를 강제 종료합니다. 진행할까요?')) return;
  const b = document.getElementById('estopBtn');
  b.textContent = '중단 중...';
  try {
    const r = await (await fetch('/api/estop', {method:'POST'})).json();
    const n = (r.killed || []).length;
    b.textContent = '🛑 비상정지 (자율주행 전체 중단)';
    alert(n ? ('종료된 프로세스 ' + n + '개:\\n' + r.killed.join('\\n'))
            : '종료할 자율주행 프로세스가 없었습니다.');
  } catch (e) {
    b.textContent = '🛑 비상정지 (자율주행 전체 중단)';
    alert('비상정지 요청 실패: ' + e);
  }
};

async function drive(cmd, force) {
  // 감속/정지 명령(force)은 절대 버리지 않는다 — 예전엔 이것도 스킵돼서
  // 와이파이가 느릴 때 버튼을 떼도 한참 더 나가는 원인이 됐음.
  if (driveInFlight && !force) return;
  driveInFlight = true;
  try {
    await fetch('/api/drive', {method:'POST', body: JSON.stringify(cmd)});
  } finally {
    driveInFlight = false;
  }
}

function driveTick() {
  if (targetScale > 0 && performance.now() - holdStartedAt > MAX_HOLD_MS) {
    targetScale = 0;  // 안전장치 발동 — pointerup을 못 받았어도 강제로 뗀 것처럼 처리
  }

  if (currentScale < targetScale) currentScale = Math.min(currentScale + RAMP_UP_STEP, targetScale);
  else if (currentScale > targetScale) currentScale = Math.max(currentScale - RAMP_DOWN_STEP, targetScale);

  const base = activeCmd ? cmdsForSpeed()[activeCmd] : {linear: 0, angular: 0};
  const decelerating = currentScale < prevScale || targetScale === 0;
  prevScale = currentScale;
  drive({linear: base.linear * currentScale, angular: base.angular * currentScale},
        decelerating);

  if (currentScale === 0 && targetScale === 0) {
    clearInterval(tickTimer);
    tickTimer = null;
    activeCmd = null;
  }
}

function startHold(cmd) {
  activeCmd = cmd;
  targetScale = 1;
  holdStartedAt = performance.now();
  if (!tickTimer) tickTimer = setInterval(driveTick, TICK_MS);
}

function releaseHold() {
  // 감속 램프 없이 즉시 정지. 램프로 서서히 줄이면 조종이 굼떠서 못 쓰겠다는
  // 피드백이 반복됨. 전기적으로 즉시 끊어도 모터 관성 때문에 실제 정지는 부드럽다.
  clearInterval(tickTimer);
  tickTimer = null;
  currentScale = 0;
  targetScale = 0;
  prevScale = 0;
  activeCmd = null;
  drive({linear: 0, angular: 0}, true);  // force=true — 절대 스킵되지 않게
}

async function hardStop() {
  // ⚠️ 여기엔 document.hidden 가드를 넣지 말 것. ■ 정지는 안전상 지연 없이
  // 즉시 반응해야 하는 버튼이고(위 주석 참고), 화면이 잠깐 hidden으로 잡히는
  // 순간(탭 전환 찰나 등)에 눌렀다고 무시되면 안 된다.
  clearInterval(tickTimer);
  tickTimer = null;
  currentScale = 0;
  targetScale = 0;
  activeCmd = null;
  await fetch('/api/stop', {method:'POST'});
}

// Pointer Events로 마우스/터치를 하나로 통합 — mousedown+touchstart를 따로 걸면
// 모바일에서 터치 후 지연된 "유령" mousedown이 한 번 더 들어와 이미 뗀 버튼을
// 다시 누른 것처럼 재실행되는 문제가 실제로 있었음(activeCmd가 의도치 않게
// 덮어써져서 "이전 명령이 계속 기억되어 실행되는" 증상으로 나타남).
document.querySelectorAll('.pad-btn').forEach(btn => {
  const cmd = btn.dataset.cmd;
  btn.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    btn.setPointerCapture(e.pointerId);
    startHold(cmd);
  });
  ['pointerup', 'pointercancel', 'pointerleave'].forEach(ev =>
    btn.addEventListener(ev, releaseHold));
});
// 정지 버튼은 안전상 지연 없이 즉시 반응해야 하므로 click 대신 pointerdown
document.getElementById('stopBtn').addEventListener('pointerdown', (e) => {
  e.preventDefault();
  hardStop();
});

// 차체 SVG 그리기
const svg = document.getElementById('chassis');
const NS = 'http://www.w3.org/2000/svg';
function el(tag, attrs) {
  const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  return e;
}
const cssVar = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

// 차체 사각형
svg.appendChild(el('rect', {x:60, y:60, width:140, height:220, rx:16,
  fill:'none', stroke: cssVar('--text-muted'), 'stroke-width':2}));
const frontLabel = el('text', {x:130, y:50, 'text-anchor':'middle', class:'wheel-label'});
frontLabel.textContent = '앞';
svg.appendChild(frontLabel);

// 바퀴 위치: [x,y] 좌상단 기준, 순서 M1(왼쪽뒤) M2(왼쪽앞) M3(오른쪽뒤) M4(오른쪽앞)
const wheelPos = [
  {x:30,  y:230, name:'M1 왼쪽뒤'},
  {x:30,  y:70,  name:'M2 왼쪽앞'},
  {x:190, y:230, name:'M3 오른쪽뒤'},
  {x:190, y:70,  name:'M4 오른쪽앞'},
];
const wheelRects = [];
const wheelArrows = [];
wheelPos.forEach((w, i) => {
  const g = el('g', {});
  const r = el('rect', {x:w.x, y:w.y, width:40, height:40, rx:8, fill: cssVar('--wheel-idle')});
  const label = el('text', {x:w.x+20, y:w.y+52, 'text-anchor':'middle', class:'wheel-label'});
  label.textContent = w.name;
  const arrow = el('text', {x:w.x+20, y:w.y+26, 'text-anchor':'middle',
    'font-size':'20', fill: cssVar('--text-primary')});
  arrow.textContent = '';
  g.appendChild(r); g.appendChild(label); g.appendChild(arrow);
  svg.appendChild(g);
  wheelRects.push(r);
  wheelArrows.push(arrow);
});

// 중앙 이동방향 화살표
const bodyArrow = el('text', {x:130, y:180, 'text-anchor':'middle',
  'font-size':'40', fill: cssVar('--accent')});
bodyArrow.textContent = '';
svg.appendChild(bodyArrow);

function directionArrow(rate) {
  if (Math.abs(rate) < 30) return '';
  return rate > 0 ? '↻' : '↺';  // 부호 규약: 양수=반시계(CCW) 실측이었으나 표시는 상대적 회전감만 전달
}

async function tick() {
  // 화면이 안 보이면 상태 조회를 쉰다. 폰 화면을 끄거나 다른 앱으로 전환해도
  // 계속 돌면 CPU와 배터리를 낭비한다.
  if (document.hidden) return;
  let res;
  try { res = await (await fetch('/api/status')).json(); }
  catch (e) { document.getElementById('connText').textContent = '연결 끊김'; return; }

  const connected = res.connected;
  document.getElementById('connDot').style.background = connected ? 'var(--good)' : 'var(--critical)';
  document.getElementById('connText').textContent = res.manual_blocked
    ? '⚠️ 자율주행 중 — 수동 조종 차단됨 (정지는 항상 동작)'
    : (connected ? 'diff_drive 연결됨' : 'diff_drive 응답 없음 (노드 실행 필요)');

  const rate = res.rate;
  const cmd = res.commanded || [0, 0];
  rate.forEach((r, i) => {
    const moving = Math.abs(r) > 30;
    wheelRects[i].setAttribute('fill', moving ? cssVar('--accent') : cssVar('--wheel-idle'));
    wheelArrows[i].textContent = directionArrow(r);
    wheelArrows[i].setAttribute('fill', moving ? 'white' : cssVar('--text-primary'));
  });

  // 이동 방향은 odom 실측값으로 판정 (회전은 IMU 자이로 기반이라 엔코더 추정보다 정확)
  const linear = res.linear || 0;
  const angular = res.angular || 0;

  let moveText = '정지';
  let arrow = '';
  if (Math.abs(linear) > 0.03 && Math.abs(angular) < 0.3) {
    if (linear > 0) { moveText = '전진'; arrow = '↑'; }
    else { moveText = '후진'; arrow = '↓'; }
  } else if (Math.abs(angular) > 0.15) {
    if (angular > 0) { moveText = '좌회전'; arrow = '↺'; }
    else { moveText = '우회전'; arrow = '↻'; }
  }
  bodyArrow.textContent = arrow;
  const moving = Math.abs(cmd[0]) > 1e-3 || Math.abs(cmd[1]) > 1e-3;
  document.getElementById('moveText').textContent = moveText +
    (moving ? ` (명령 ${cmd[0].toFixed(2)} m/s, ${cmd[1].toFixed(1)} rad/s)` : '');
}

tick();
setInterval(tick, 200);
document.addEventListener('visibilitychange', () => {
  if (!document.hidden) tick();
});

// 배터리는 급변하지 않으니 메인 화면에서는 5초에 한 번만 확인한다 —
// 부하를 아껴야 할 곳은 여기지 tick()이 아니다.
async function battTick() {
  if (document.hidden) return;
  let b;
  try { b = await (await fetch('/api/battery')).json(); } catch (e) { return; }
  const el = document.getElementById('battBadge');
  if (!b.connected) { el.textContent = '🔋 연결 안 됨'; return; }
  el.textContent = `🔋 ${b.voltage.toFixed(2)} V (세션 최저 ${b.min.toFixed(2)} V)`;
}
battTick();
setInterval(battTick, 5000);
document.addEventListener('visibilitychange', () => {
  if (!document.hidden) battTick();
});
</script>
</body>
</html>
"""

# 파비콘 삽입. PAGE가 일반 문자열이라 f-string 치환을 못 쓰므로 여기서 바꾼다.
PAGE = PAGE.replace("__FAVICON__", FAVICON_TAG)

BATTERY_PAGE = """<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
__FAVICON__
<title>배터리 상세</title>
<style>
  :root {
    color-scheme: light;
    --surface-1: #fcfcfb; --page: #f9f9f7;
    --text-primary: #0b0b0b; --text-secondary: #52514e; --text-muted: #898781;
    --border: rgba(11,11,11,0.10); --accent: #2a78d6;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --surface-1: #1a1a19; --page: #0d0d0d;
      --text-primary: #ffffff; --text-secondary: #c3c2b7; --text-muted: #898781;
      --border: rgba(255,255,255,0.10); --accent: #5b9bea;
    }
  }
  body { background: var(--page); color: var(--text-primary);
    font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
    max-width: 480px; margin: 24px auto; padding: 0 16px; }
  h1 { font-size: 1.2rem; }
  a.back { color: var(--accent); text-decoration: none; font-size: 0.85rem; }
  .card { background: var(--surface-1); border: 1px solid var(--border); border-radius: 12px;
    padding: 16px; margin: 16px 0; }
  .big { font-size: 2.2rem; font-weight: 700; font-variant-numeric: tabular-nums; }
  table { width: 100%; border-collapse: collapse; font-size: 0.9rem; margin-top: 8px;
    font-variant-numeric: tabular-nums; }
  td, th { text-align: left; padding: 4px 6px; border-bottom: 1px solid var(--border); }
  #sparkSvg { display: block; width: 100%; height: 100px; margin-top: 8px; }
  .sub { font-size: 0.8rem; color: var(--text-muted); margin-top: 8px; }
  .tier-badge { display: inline-block; padding: 3px 10px; border-radius: 999px;
    font-size: 0.85rem; font-weight: 600; margin-left: 10px; vertical-align: middle; }
  .tier-ok { background: color-mix(in srgb, #2fa84f 20%, transparent); color: #2fa84f; }
  .tier-warn1 { background: color-mix(in srgb, #d6a300 20%, transparent); color: #b58500; }
  .tier-warn2 { background: color-mix(in srgb, #e05a3c 22%, transparent); color: #e05a3c; }
  .tier-critical { background: color-mix(in srgb, #d63a3a 25%, transparent); color: #d63a3a; }
</style>
</head>
<body>
<a class="back" href="/">← 모터 대시보드</a>
<h1>🔋 배터리 상세</h1>
<div class="card">
  <div class="big"><span id="voltNow">— V</span><span class="tier-badge" id="tierBadge"></span></div>
  <svg id="sparkSvg" viewBox="0 0 300 100" preserveAspectRatio="none"></svg>
  <table id="statTable"></table>
  <div class="sub" id="updated"></div>
</div>
<script>
let lastTs = 0;
let hist = [];
let thresholds = null;  // {warn1, warn2, critical} — 첫 /api/battery 응답에서 채움

const TIER_COLOR = {ok: '#2fa84f', warn1: '#d6a300', warn2: '#e05a3c', critical: '#d63a3a'};

function drawSpark() {
  const svg = document.getElementById('sparkSvg');
  svg.innerHTML = '';
  if (hist.length < 2) return;
  const vs = hist.map(h => h.v);
  let lo = Math.min(...vs), hi = Math.max(...vs);
  // 임계선이 최근 데이터 범위 밖에 있어도(배터리가 넉넉해서 한참 위인 경우 등)
  // 스케일에 강제로 끼워 넣지 않는다 — 그러면 실제 전압 변화가 눌려서 안 보인다.
  // 대신 임계선은 화면 범위 안에 들 때만 그린다(아래 threshLine).
  const pad = Math.max((hi - lo) * 0.15, 0.02);
  const y0 = lo - pad, y1 = hi + pad;
  const W = 300, H = 100;
  const x = i => (i / (hist.length - 1)) * W;
  const y = v => H - ((v - y0) / (y1 - y0)) * H;
  const ns = 'http://www.w3.org/2000/svg';

  if (thresholds) {
    const threshLine = (v, color, label) => {
      if (v < y0 || v > y1) return;  // 화면 범위 밖이면 안 그림
      const ly = y(v);
      const line = document.createElementNS(ns, 'line');
      line.setAttribute('x1', 0); line.setAttribute('x2', W);
      line.setAttribute('y1', ly); line.setAttribute('y2', ly);
      line.setAttribute('stroke', color);
      line.setAttribute('stroke-width', '1');
      line.setAttribute('stroke-dasharray', '4,3');
      svg.appendChild(line);
      const text = document.createElementNS(ns, 'text');
      text.setAttribute('x', W - 2); text.setAttribute('y', ly - 2);
      text.setAttribute('text-anchor', 'end');
      text.setAttribute('font-size', '8');
      text.setAttribute('fill', color);
      text.textContent = label + ' ' + v.toFixed(1) + 'V';
      svg.appendChild(text);
    };
    threshLine(thresholds.warn1, TIER_COLOR.warn1, '1차');
    threshLine(thresholds.warn2, TIER_COLOR.warn2, '2차');
    threshLine(thresholds.critical, TIER_COLOR.critical, '정지');
  }

  const pts = hist.map((h, i) => `${x(i).toFixed(1)},${y(h.v).toFixed(1)}`).join(' ');
  const poly = document.createElementNS(ns, 'polyline');
  poly.setAttribute('points', pts);
  poly.setAttribute('fill', 'none');
  poly.setAttribute('stroke', 'var(--accent)');
  poly.setAttribute('stroke-width', '2');
  svg.appendChild(poly);
}

async function poll() {
  if (document.hidden) return;
  let b;
  try { b = await (await fetch('/api/battery')).json(); } catch (e) { return; }
  if (!b.connected) {
    document.getElementById('voltNow').textContent = '연결 안 됨';
    document.getElementById('tierBadge').textContent = '';
    document.getElementById('updated').textContent = b.error ? ('오류: ' + b.error) : '';
    return;
  }
  thresholds = b.thresholds || thresholds;
  const voltEl = document.getElementById('voltNow');
  voltEl.textContent = b.voltage.toFixed(3) + ' V';
  voltEl.style.color = TIER_COLOR[b.tier] || '';
  const badge = document.getElementById('tierBadge');
  const tierLabel = {ok: '정상', warn1: '1차 경고', warn2: '2차 경고', critical: '자동정지'}[b.tier] || '';
  badge.textContent = tierLabel;
  badge.className = 'tier-badge tier-' + (b.tier || 'ok');
  const started = new Date(b.session_started * 1000).toLocaleTimeString();
  const th = b.thresholds || {};
  document.getElementById('statTable').innerHTML = `
    <tr><th>항목</th><th>값</th></tr>
    <tr><td>10초 이동평균 (판정 기준)</td><td>${(b.recent_avg ?? b.voltage).toFixed(3)} V</td></tr>
    <tr><td>세션 평균</td><td>${b.avg.toFixed(3)} V</td></tr>
    <tr><td>세션 최저</td><td>${b.min.toFixed(3)} V</td></tr>
    <tr><td>세션 최고</td><td>${b.max.toFixed(3)} V</td></tr>
    <tr><td>1차 경고 임계값</td><td style="color:${TIER_COLOR.warn1}">${(th.warn1 ?? 0).toFixed(2)} V</td></tr>
    <tr><td>2차 경고(적색) 임계값</td><td style="color:${TIER_COLOR.warn2}">${(th.warn2 ?? 0).toFixed(2)} V</td></tr>
    <tr><td>자동정지 임계값</td><td style="color:${TIER_COLOR.critical}">${(th.critical ?? 0).toFixed(2)} V</td></tr>
    <tr><td>자동정지 재무장 상태</td><td>${b.auto_stop_armed === false ? '발동됨(회복 대기 중)' : '정상'}</td></tr>
    <tr><td>샘플 수</td><td>${b.samples}</td></tr>
    <tr><td>세션 시작</td><td>${started}</td></tr>`;
  document.getElementById('updated').textContent =
    '마지막 갱신: ' + new Date(b.ts * 1000).toLocaleTimeString();

  let h;
  try { h = await (await fetch('/api/battery/history?since=' + lastTs)).json(); }
  catch (e) { return; }
  if (h.rows && h.rows.length) {
    hist = hist.concat(h.rows).slice(-300);
    lastTs = h.last_ts;
    drawSpark();
  }
}
poll();
setInterval(poll, 2000);
document.addEventListener('visibilitychange', () => { if (!document.hidden) poll(); });
</script>
</body>
</html>
"""
BATTERY_PAGE = BATTERY_PAGE.replace("__FAVICON__", FAVICON_TAG)


class Handler(BaseHTTPRequestHandler):
    node = None

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        # 클라이언트가 받아준다면 압축해서 보낸다. 맵 그리드나 JSON 반복 필드는
        # 10배 가까이 줄어든다. WiFi가 약한 구간에서 이게 응답성을 좌우한다.
        enc = None
        if len(body) > 1024 and "gzip" in (self.headers.get("Accept-Encoding") or ""):
            packed = gzip.compress(body, 6)
            if len(packed) < len(body):
                body, enc = packed, "gzip"
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        if enc:
            self.send_header("Content-Encoding", enc)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            # no-store: JS를 고쳐도 브라우저가 옛 버전을 계속 실행하던 문제 방지
            body = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store, must-revalidate")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/api/status":
            # 허브(dashboard_hub.py)가 이 한 엔드포인트만 보고 모터 카드를
            # 그리므로, 배터리 전압도 여기 곁들여 보낸다(2026-08-25 — 허브
            # 카드의 "드라이버 연결됨" 텍스트를 전압으로 바꾸기 위함. 연결
            # 상태 자체는 카드 색(녹/빨 원)이 이미 표시하므로 중복이었음).
            with state_lock, battery_lock:
                out = dict(state)
                out["battery"] = {"connected": battery_state.get("connected", False),
                                   "voltage": battery_state.get("voltage")}
                self._send(200, json.dumps(out).encode("utf-8"), "application/json")
        elif self.path == "/battery":
            body = BATTERY_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store, must-revalidate")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/api/battery":
            with battery_lock:
                self._send(200, json.dumps(battery_state).encode("utf-8"), "application/json")
        elif self.path.startswith("/api/battery/history"):
            q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            try:
                since = float(q.get("since", ["0"])[0])
            except ValueError:
                since = 0.0
            with battery_lock:
                rows = [h for h in battery_history if h["ts"] > since]
                if not since:
                    rows = rows[-300:]
            body = json.dumps({"rows": rows,
                               "last_ts": rows[-1]["ts"] if rows else since}).encode("utf-8")
            self._send(200, body, "application/json")
        else:
            self._send(404, b"not found", "text/plain")

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        try:
            payload = json.loads(self.rfile.read(length) or b"{}") if length else {}
        except json.JSONDecodeError:
            payload = {}

        if self.path == "/api/estop":
            # ⚠️ 순서가 중요하다. 0 속도 발행을 먼저 하면 DDS가 꼬였을 때
            #    거기서 막혀 프로세스 종료까지 도달하지 못한다(2026-08-16에
            #    비상정지 API가 통째로 응답불능이 된 사례). 소스를 먼저 죽인다.
            killed = kill_autonomous_processes()
            try:
                Handler.node.send_cmd(0.0, 0.0)
            except Exception:
                pass
            self._send(200, json.dumps({"ok": True, "killed": killed}).encode(),
                       "application/json")
            return
        if self.path == "/api/drive":
            Handler.node.send_cmd(payload.get("linear", 0.0), payload.get("angular", 0.0))
            self._send(200, b'{"ok":true}', "application/json")
        elif self.path == "/api/stop":
            # 정지는 유실되면 안 되므로 여러 번 발행
            for _ in range(3):
                Handler.node.send_cmd(0.0, 0.0)
            self._send(200, b'{"ok":true}', "application/json")
        else:
            self._send(404, b"not found", "text/plain")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8766)
    args = ap.parse_args()

    rclpy.init()
    node = TeleopNode()
    Handler.node = node

    threading.Thread(target=staleness_watchdog, daemon=True).start()
    # 2026-08-21: 자율주행 중 비상정지 채널(이 대시보드 HTTP 서버)이 세 번
    # 먹통됐다 — 매번 이 스레드가 유력한 용의자였다(I2C 블로킹 의심, 확정은
    # 안 됐지만 안전이 우선이라 임시로 껐었다). 그 의심의 배경은 당시 배선이
    # INA219 SDA/SCL을 IMU 물리 핀에 스플라이스로 공유한 **임시 배선**이었다는
    # 점 — 접촉 불량이 I2C를 정말로 블로킹시켰을 가능성이 있었다.
    # 2026-08-25: 정식 Qwiic 데이지체인 케이블 도착·연결 후 재검증 —
    # /api/status 132회 연속 요청(20초, 폴링주기 2초 10회 이상 포함) 최대
    # 37.9ms/평균 1.9ms, /api/estop 45.4ms로 전부 정상 응답, 행 없음 확인 후
    # 다시 켬.
    threading.Thread(target=battery_poll_loop, daemon=True).start()
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)

    # systemd가 SIGTERM을 보내도 serve_forever()가 이를 감지 못해 90초 뒤
    # SIGKILL로 강제종료되는 문제(2026-08-16 감사 리포트로 확인) 방지.
    def _sig_handler(signum, frame):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, _sig_handler)
    signal.signal(signal.SIGINT, _sig_handler)

    print(f"모터 조종 대시보드 시작: http://<Pi IP>:{args.port}", flush=True)
    try:
        server.serve_forever()
    finally:
        try:
            node.send_cmd(0.0, 0.0)  # 종료 시 반드시 정지 명령
        except Exception:
            pass
        # rclpy.spin()이 백그라운드 데몬 스레드에서 도는 채로 인터프리터가
        # 그냥 종료되면, 그 스레드가 DDS 콜백 도중 거칠게 죽으면서
        # "terminate called without an active exception"로 core dump 나는
        # 문제가 있었음(2026-08-16, ROS 그래프에 노드가 많아지면서 레이스가
        # 잘 걸림). rclpy.shutdown()으로 spin()을 먼저 정상 반환시키고
        # 스레드가 실제로 끝나길 기다린 뒤에 노드를 정리해야 안전하다.
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == "__main__":
    main()