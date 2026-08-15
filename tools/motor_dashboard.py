#!/usr/bin/env python3
"""4채널 모터 원격 조종 + 상태 대시보드 (ROS2 토픽 기반).

⚠️ 2026-08-15 리팩터링: 예전에는 이 대시보드가 시리얼 포트를 직접 열어 $pwm:을
쐈는데, 그러면 diff_drive_node와 포트를 두고 싸워서 "조종하면서 맵 그리기"가
불가능했다. 이제는 시리얼을 전혀 안 건드리고 ROS2 토픽만 쓴다:

    발행: /cmd_vel        (geometry_msgs/Twist)  — 조종 명령
    구독: /odom           (nav_msgs/Odometry)    — 실제 이동/회전 상태
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
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

# 조종 상한 — diff_drive_node의 max_wheel_speed(1.0 m/s)보다 보수적으로 잡음.
# 실내 맵핑 주행은 느릴수록 SLAM 품질이 좋아서 일부러 낮게 둔다.
MAX_LINEAR = 0.5   # m/s
MAX_ANGULAR = 2.5  # rad/s
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
}


class TeleopNode(Node):
    def __init__(self):
        super().__init__('motor_dashboard')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
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

    def send_cmd(self, linear, angular):
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
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>모터 대시보드</title>
<style>
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
const MAX_LINEAR = 0.5;   // m/s
const MAX_ANGULAR = 2.5;  // rad/s
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
const RAMP_STEP = 0.1;  // tick당 목표치에 이만큼씩 접근 (약 10틱 = 500ms에 도달)
const MAX_HOLD_MS = 2000;  // 뗀 신호(pointerup)를 못 받는 경우를 대비한 안전장치 —
                            // 아무리 눌려 있어도 이 시간 지나면 강제로 놓은 것처럼 처리

let currentScale = 0;
let targetScale = 0;
let activeCmd = null;
let tickTimer = null;
let holdStartedAt = 0;
let driveInFlight = false;  // 이전 /api/drive 응답 오기 전엔 새로 안 쏨 — 요청 밀림 방지

async function drive(cmd) {
  if (driveInFlight) return;  // 응답이 밀려있으면 이번 틱은 건너뜀(요청 적체 방지)
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

  if (currentScale < targetScale) currentScale = Math.min(currentScale + RAMP_STEP, targetScale);
  else if (currentScale > targetScale) currentScale = Math.max(currentScale - RAMP_STEP, targetScale);

  const base = activeCmd ? cmdsForSpeed()[activeCmd] : {linear: 0, angular: 0};
  drive({linear: base.linear * currentScale, angular: base.angular * currentScale});

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
  targetScale = 0;
  if (!tickTimer && currentScale > 0) tickTimer = setInterval(driveTick, TICK_MS);
}

async function hardStop() {
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
  let res;
  try { res = await (await fetch('/api/status')).json(); }
  catch (e) { document.getElementById('connText').textContent = '연결 끊김'; return; }

  const connected = res.connected;
  document.getElementById('connDot').style.background = connected ? 'var(--good)' : 'var(--critical)';
  document.getElementById('connText').textContent =
    connected ? 'diff_drive 연결됨' : 'diff_drive 응답 없음 (노드 실행 필요)';

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
</script>
</body>
</html>
"""

class Handler(BaseHTTPRequestHandler):
    node = None

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif self.path == "/api/status":
            with state_lock:
                self._send(200, json.dumps(state).encode("utf-8"), "application/json")
        else:
            self._send(404, b"not found", "text/plain")

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        try:
            payload = json.loads(self.rfile.read(length) or b"{}") if length else {}
        except json.JSONDecodeError:
            payload = {}

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
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"모터 조종 대시보드 시작: http://<Pi IP>:{args.port}", flush=True)
    try:
        server.serve_forever()
    finally:
        try:
            node.send_cmd(0.0, 0.0)  # 종료 시 반드시 정지 명령
        except Exception:
            pass


if __name__ == "__main__":
    main()
