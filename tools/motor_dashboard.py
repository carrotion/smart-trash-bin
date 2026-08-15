#!/usr/bin/env python3
"""4채널 모터 드라이버 실시간 대시보드 — 바퀴별 상태 + 본체 이동방향 시각화.

Yahboom 드라이버 시리얼 포트를 이 프로세스가 독점 소유하고
(CLAUDE.md에 기록된 검증된 프로토콜: 115200 8N1, $CMD:val# ASCII,
MOTOR_TYPE=1 고정 — TT 프리셋 절대 사용 금지), 주기적으로 엔코더를
읽어 웹 페이지에 실시간으로 보여준다. 조작은 누르고 있는 동안만
구동되는 hold-to-drive 방식(뗴면 즉시 정지 + 클라이언트 하트비트)으로
제한해 오늘 겪은 "정지 명령 무시" 사고가 재발해도 노출 시간을 최소화함.

사용법:
    python3 tools/motor_dashboard.py [--port 8766]
    브라우저에서 http://<Pi IP>:8766 접속
"""
import argparse
import json
import re
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

SERIAL_PORT = "/dev/ttyUSB0"
BAUD = 115200
MAX_PWM = 1200  # 안전 상한 — 오늘 검증에 쓴 값(500~1000)보다 약간 높은 정도로만 제한
STOP_TIMEOUT_S = 0.35  # 이 시간 안에 새 명령/하트비트가 안 오면 서버가 자체적으로 정지

WHEEL_NAMES = ["왼쪽뒤", "왼쪽앞", "오른쪽뒤", "오른쪽앞"]

state_lock = threading.Lock()
state = {
    "connected": False,
    "encoder": [0, 0, 0, 0],
    "rate": [0.0, 0.0, 0.0, 0.0],  # counts/sec, 부호=회전방향
    "commanded_pwm": [0, 0, 0, 0],
    "last_cmd_ts": 0.0,
}

# ThreadingHTTPServer는 요청마다 새 스레드를 띄우고, 백그라운드 폴링 스레드도
# 따로 돌아서 같은 pyserial Serial 객체를 동시에 건드릴 수 있음(스레드 안전 아님).
# 램프 기능으로 요청 빈도가 늘면서 실제로 겹쳐서 SerialException이 났던 버그 수정용 락.
serial_lock = threading.Lock()


def send(ser, cmd: bytes, wait=0.1):
    with serial_lock:
        ser.reset_input_buffer()
        ser.write(cmd)
        time.sleep(wait)
        return ser.read(ser.in_waiting or 1)


_MALL_RE = re.compile(r"^\$MAll:(-?\d+),(-?\d+),(-?\d+),(-?\d+)#$")


def read_encoder(ser):
    with serial_lock:
        ser.reset_input_buffer()
        ser.write(b"$upload:1#")
        time.sleep(0.15)
        raw = ser.read(ser.in_waiting or 1)
        ser.write(b"$upload:0#")
        time.sleep(0.05)
        ser.reset_input_buffer()
    # 잘린 프레임(마지막 줄이 '#'로 안 끝나는 경우)을 걸러내기 위해
    # 끝에서부터 완전한 $MAll:...# 형태와 매치되는 마지막 줄만 사용
    lines = raw.decode(errors="replace").split("\r\n")
    for line in reversed(lines):
        m = _MALL_RE.match(line.strip())
        if m:
            return [int(g) for g in m.groups()]
    return None


def watchdog_and_poll_loop(ser):
    prev_enc = None
    prev_ts = None
    while True:
        enc = read_encoder(ser)
        now = time.time()
        with state_lock:
            if enc is not None:
                state["connected"] = True
                if prev_enc is not None and prev_ts is not None:
                    dt = max(now - prev_ts, 1e-3)
                    state["rate"] = [(enc[i] - prev_enc[i]) / dt for i in range(4)]
                state["encoder"] = enc
                prev_enc, prev_ts = enc, now
            else:
                state["connected"] = False

            # 워치독: 마지막 명령/하트비트가 STOP_TIMEOUT_S 넘게 안 오면 강제 정지
            if state["commanded_pwm"] != [0, 0, 0, 0] and now - state["last_cmd_ts"] > STOP_TIMEOUT_S:
                state["commanded_pwm"] = [0, 0, 0, 0]
                send(ser, b"$pwm:0,0,0,0#", wait=0.02)

        time.sleep(0.15)


def apply_drive(ser, m1, m2, m3, m4):
    vals = [max(-MAX_PWM, min(MAX_PWM, int(v))) for v in (m1, m2, m3, m4)]
    cmd = f"$pwm:{vals[0]},{vals[1]},{vals[2]},{vals[3]}#".encode()
    send(ser, cmd, wait=0.02)
    with state_lock:
        state["commanded_pwm"] = vals
        state["last_cmd_ts"] = time.time()


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
const MAX_PWM = 1200;  // 서버(motor_dashboard.py)의 안전 상한과 일치시킴
const slider = document.getElementById('speedSlider');
const speedLabel = document.getElementById('speedLabel');

function currentSpeed() {
  const pct = Number(slider.value);
  return Math.round(MAX_PWM * pct / 100);
}
function updateSpeedLabel() {
  speedLabel.textContent = `${slider.value}% (PWM ${currentSpeed()})`;
}
slider.addEventListener('input', updateSpeedLabel);
updateSpeedLabel();

function cmdsForSpeed() {
  const s = currentSpeed();
  return {
    fwd:   [ s,  s,  s,  s],
    back:  [-s, -s, -s, -s],
    left:  [-s, -s,  s,  s],
    right: [ s,  s, -s, -s],
  };
}

// 방향 버튼: 누르면 0->목표까지, 떼면 목표->0까지 약 500ms에 걸쳐 부드럽게 램프.
// (■ 정지 버튼은 이 램프를 건너뛰고 즉시 0을 보내는 별도 hardStop — 안전 우선)
const TICK_MS = 50;
const RAMP_STEP = 0.1;  // tick당 목표치에 이만큼씩 접근 (약 10틱 = 500ms에 도달)

let currentScale = 0;
let targetScale = 0;
let activeCmd = null;
let tickTimer = null;

async function drive(vals) {
  await fetch('/api/drive', {method:'POST', body: JSON.stringify({pwm: vals})});
}

function driveTick() {
  if (currentScale < targetScale) currentScale = Math.min(currentScale + RAMP_STEP, targetScale);
  else if (currentScale > targetScale) currentScale = Math.max(currentScale - RAMP_STEP, targetScale);

  const base = activeCmd ? cmdsForSpeed()[activeCmd] : [0, 0, 0, 0];
  drive(base.map(v => Math.round(v * currentScale)));

  if (currentScale === 0 && targetScale === 0) {
    clearInterval(tickTimer);
    tickTimer = null;
    activeCmd = null;
  }
}

function startHold(cmd) {
  activeCmd = cmd;
  targetScale = 1;
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
  document.getElementById('connText').textContent = connected ? '드라이버 연결됨' : '드라이버 응답 없음';

  const rate = res.rate;
  const pwm = res.commanded_pwm;
  rate.forEach((r, i) => {
    const moving = Math.abs(r) > 30;
    wheelRects[i].setAttribute('fill', moving ? cssVar('--accent') : cssVar('--wheel-idle'));
    wheelArrows[i].textContent = directionArrow(r);
    wheelArrows[i].setAttribute('fill', moving ? 'white' : cssVar('--text-primary'));
  });

  // 본체 이동방향 추정: 좌측평균(M1,M2) vs 우측평균(M3,M4)
  const leftAvg = (rate[0] + rate[1]) / 2;
  const rightAvg = (rate[2] + rate[3]) / 2;
  const linear = (leftAvg + rightAvg) / 2;  // 부호는 엔코더 규약(양수=반시계=왼쪽바퀴 전진 방향)
  const angular = rightAvg - leftAvg;

  let moveText = '정지';
  let arrow = '';
  if (Math.abs(linear) > 40 && Math.abs(angular) < Math.abs(linear) * 0.5) {
    // 직진/후진 위주 — 부호는 왼쪽 바퀴 기준(양수=CCW=전진)
    if (linear > 0) { moveText = '전진'; arrow = '↑'; }
    else { moveText = '후진'; arrow = '↓'; }
  } else if (Math.abs(angular) > 40) {
    if (angular > 0) { moveText = '우회전'; arrow = '↻'; }
    else { moveText = '좌회전'; arrow = '↺'; }
  }
  bodyArrow.textContent = arrow;
  document.getElementById('moveText').textContent = moveText +
    (pwm.some(v => v !== 0) ? ` (PWM ${pwm.join(',')})` : '');
}

tick();
setInterval(tick, 200);
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    ser = None

    def log_message(self, *a):
        pass

    def _send_json(self, obj, code=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/api/status":
            with state_lock:
                self._send_json(dict(state))
        else:
            self._send_json({"error": "not found"}, 404)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            payload = json.loads(raw or b"{}")
        except json.JSONDecodeError:
            payload = {}

        if self.path == "/api/drive":
            pwm = payload.get("pwm", [0, 0, 0, 0])
            if isinstance(pwm, list) and len(pwm) == 4:
                apply_drive(Handler.ser, *pwm)
            self._send_json({"ok": True})
        elif self.path == "/api/stop":
            apply_drive(Handler.ser, 0, 0, 0, 0)
            self._send_json({"ok": True})
        else:
            self._send_json({"error": "not found"}, 404)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8766)
    args = ap.parse_args()

    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
    time.sleep(0.3)
    # 검증된 안전 설정 고정 적용 — TT 프리셋(3/4) 절대 사용 금지
    send(ser, b"$mtype:1#")
    send(ser, b"$deadzone:1900#")
    send(ser, b"$wdiameter:67.00#")
    send(ser, b"$pwm:0,0,0,0#")

    Handler.ser = ser
    t = threading.Thread(target=watchdog_and_poll_loop, args=(ser,), daemon=True)
    t.start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"모터 대시보드 시작: http://<Pi IP>:{args.port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
