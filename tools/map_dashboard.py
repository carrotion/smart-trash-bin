#!/usr/bin/env python3
"""SLAM Toolbox가 발행하는 /map(OccupancyGrid)을 실시간으로 그려서 보여주는 대시보드.

실제 맵핑 주행(mapping.launch.py) 시작 전에 미리 띄워두는 용도 — SLAM Toolbox가
아직 안 떠있으면 그냥 "데이터 없음" 상태로 대기하다가, 맵핑을 시작하면 자동으로
그리드가 채워지는 걸 브라우저에서 실시간으로 볼 수 있음.

의존성 추가 없이(Pillow 등 이미지 라이브러리 없이) 그리드 원시 바이트를 base64로
클라이언트에 보내고, 브라우저 <canvas>에서 직접 픽셀을 찍어 그림.

사용법:
    python3 tools/map_dashboard.py [--port 8768]
    (SLAM Toolbox가 /map을 발행하기 전까진 "데이터 없음" 상태로 대기)
"""
import argparse
import base64
import json
import os
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

state_lock = threading.Lock()
state = {"connected": False, "last_ts": 0.0}
STALE_S = 5.0

# 비상정지 대상: 로봇에 cmd_vel을 만들어 보낼 수 있는 프로세스들.
# 2026-08-16: 전부 carrotian 소유라 같은 유저인 대시보드가 권한 없이 신호를 보낼 수 있다.
#
# ⚠️ **실행파일 이름(argv[0]의 basename)으로 정확히 일치**시킨다. 예전엔 명령줄
# 전체에 대한 부분문자열 매칭이었는데, 그러면 "bt_navigator" 같은 흔한 문자열이
# 들어간 **무관한 프로세스까지 죽인다**. 실제로 진단용 셸(`pgrep -f
# 'nav2_bt_navigator|...'`)이 비상정지에 휘말려 죽는 걸 확인했다 — 실전에서는
# 사용자의 SSH 세션이나 모니터링 명령이 같은 이유로 죽을 수 있어서 위험하다.
AUTONOMOUS_EXECUTABLES = {
    "controller_server",   # Nav2 컨트롤러 — cmd_vel을 직접 발행하는 주범
    "planner_server",
    "bt_navigator",
    "behavior_server",     # 복구 동작(회전/후진)도 cmd_vel을 냄
    "explore",             # explore_lite의 실행파일 이름
}
# 런치 프로세스는 argv[0]이 python이라 실행파일명으로는 못 잡는다 — 인자로 판별.
AUTONOMOUS_LAUNCH_FILE = "autonomous_mapping.launch.py"


def kill_autonomous_processes():
    """자율주행 프로세스를 /proc 직접 스캔 + os.kill로 죽인다. 목록을 반환.

    ⚠️ 예전엔 `subprocess.run(["pkill", "-f", p], capture_output=True)`를 썼는데,
    **죽일 프로세스가 실제로 있을 때만 응답이 안 돌아오는** 현상을 재현 확인했다
    (죽일 게 없으면 0.4초, 있으면 무한 대기). 비상정지 경로에서 서브프로세스와
    파이프에 의존하는 것 자체가 위험하다고 판단해, 외부 프로세스를 아예 안 쓰는
    방식으로 바꿨다. /proc 읽기와 os.kill은 블로킹될 여지가 없다.

    자기 자신(대시보드)은 반드시 제외한다 — 패턴이 자기 명령줄에 걸리면
    비상정지를 누르는 순간 대시보드가 자살한다.
    """
    me = os.getpid()
    killed = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                # cmdline은 인자들이 NUL로 구분돼 있음 — 리스트로 복원
                args = f.read().decode('utf-8', 'replace').split('\0')
        except OSError:
            continue  # 그 사이 종료됐거나 권한 없음
        args = [a for a in args if a]
        if not args:
            continue  # 커널 스레드

        # 파이썬 노드는 argv[0]가 python3이고 실제 이름은 argv[1]에 있다 —
        # 지금 대상은 전부 C++ 바이너리지만, 나중에 파이썬 노드가 대상에
        # 추가돼도 조용히 안 잡히는 일이 없도록 여기서도 같이 처리한다.
        exe = os.path.basename(args[0])
        if exe.startswith('python'):
            for a in args[1:]:
                if not a.startswith('-'):
                    exe = os.path.basename(a)
                    break
        if exe in AUTONOMOUS_EXECUTABLES:
            match = exe
        elif any(a.endswith(AUTONOMOUS_LAUNCH_FILE) for a in args):
            match = AUTONOMOUS_LAUNCH_FILE
        else:
            continue

        try:
            os.kill(pid, signal.SIGKILL)
            killed.append(f'{match}({pid})')
        except OSError:
            pass
    return killed


class MapBridge(Node):
    def __init__(self):
        super().__init__('map_dashboard_bridge')
        # /map은 **TRANSIENT_LOCAL(래치)** 로 구독해야 한다. 기본 VOLATILE로 두면
        # 발행자가 이미 낸 마지막 맵을 못 받고, 다음 발행 때까지 "응답 없음"으로 보인다.
        # SLAM은 주기적으로 재발행해서 가려져 있었지만, map_server는 활성화 때 한 번만
        # 내므로 저장된 맵을 띄우면 화면이 영영 비어 있었다(2026-08-17 확인).
        # 대시보드를 재시작해도 즉시 마지막 맵이 뜨는 이점도 있다.
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, map_qos)
        # 비상정지용 — 자율주행이 계속 새 명령을 낼 수도 있으니 여러 번 강제로 0 발행.
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)

    def emergency_stop(self):
        """0 속도를 여러 번 발행. **호출자는 이게 멈출 수 있다고 가정해야 한다.**

        2026-08-16: DDS 상태가 꼬이면(대량 kill -9 / /dev/shm 정리 후 등) 이
        publish가 무한정 안 돌아오는 걸 실제로 겪음 — 그때 비상정지 API 전체가
        응답을 안 했다. 그래서 이건 이제 '보조 수단'이고, 진짜 정지는
        프로세스를 죽이는 쪽(아래 do_POST)이 담당한다.
        """
        for _ in range(5):
            self._cmd_pub.publish(Twist())
            time.sleep(0.05)

    def emergency_stop_async(self, timeout=1.5):
        """emergency_stop을 별도 스레드로 돌려서, 멈춰도 호출자를 막지 않게 한다.

        반환값은 시간 내에 끝났는지 여부(진단용).
        """
        t = threading.Thread(target=self.emergency_stop, daemon=True)
        t.start()
        t.join(timeout=timeout)
        return not t.is_alive()

    def _on_map(self, msg: OccupancyGrid):
        # OccupancyGrid.data는 int8[] (-1=unknown, 0=free, 100=occupied).
        # 파이썬에서 그대로 bytes로 바꾸면 부호 처리가 꼬이므로 unsigned로 변환해서 전송,
        # 클라이언트에서 다시 signed로 해석함(-1은 255로 옴).
        raw = bytes((v & 0xFF) for v in msg.data)
        with state_lock:
            state.update({
                "connected": True,
                "last_ts": time.time(),
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution,
                "origin_x": msg.info.origin.position.x,
                "origin_y": msg.info.origin.position.y,
                "data_b64": base64.b64encode(raw).decode("ascii"),
            })


def staleness_watchdog():
    """맵의 '신선도'를 감시하되 **화면에서 지우지는 않는다.**

    /map은 스트리밍이 아니라 스냅샷이다. SLAM은 주기적으로 재발행하지만
    map_server(저장된 맵 열람)는 활성화 때 **딱 한 번** 래치로 발행한다. 예전엔
    STALE_S초 뒤 connected=False로 바꿔서 **맵이 잠깐 떴다가 사라졌다**(2026-08-17).

    맵은 한 번 받으면 계속 유효하다 — 오래됐다는 표시만 하고 그림은 남긴다.
    (라이다/IMU 같은 실시간 센서와 성격이 다르므로 같은 잣대를 쓰면 안 된다)
    """
    while True:
        with state_lock:
            if state.get("last_ts"):
                state["stale"] = (time.time() - state["last_ts"]) > STALE_S
        time.sleep(1.0)


PAGE = """<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>맵 대시보드</title>
<style>
  :root {
    color-scheme: light;
    --surface-1: #fcfcfb; --page: #f9f9f7;
    --text-primary: #0b0b0b; --text-secondary: #52514e; --text-muted: #898781;
    --border: rgba(11,11,11,0.10);
    --good: #0ca30c; --critical: #d03b3b;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --surface-1: #1a1a19; --page: #0d0d0d;
      --text-primary: #ffffff; --text-secondary: #c3c2b7; --text-muted: #898781;
      --border: rgba(255,255,255,0.10);
      --good: #0ca30c; --critical: #e66767;
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --surface-1: #1a1a19; --page: #0d0d0d;
    --text-primary: #ffffff; --text-secondary: #c3c2b7; --text-muted: #898781;
    --border: rgba(255,255,255,0.10);
    --good: #0ca30c; --critical: #e66767;
  }
  body { background: var(--page); color: var(--text-primary);
    font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
    max-width: 720px; margin: 24px auto; padding: 0 16px; }
  h1 { font-size: 1.3rem; display: flex; align-items: center; gap: 8px; }
  .dot { width: 10px; height: 10px; border-radius: 50%; }
  .card { background: var(--surface-1); border: 1px solid var(--border); border-radius: 12px;
    padding: 16px; margin: 16px 0; }
  #mapCanvas { display: block; margin: 0 auto; background: #808080;
    image-rendering: pixelated; max-width: 100%; border: 1px solid var(--border); }
  .status-row { display: flex; justify-content: space-between; font-size: 0.85rem;
    color: var(--text-secondary); margin-top: 8px; font-variant-numeric: tabular-nums; }
  .empty { text-align: center; color: var(--text-muted); padding: 60px 0; }
  #estopBtn { display: block; width: 100%; padding: 18px 0; margin: 16px 0;
    background: var(--critical); color: white; border: none; border-radius: 12px;
    font-size: 1.2rem; font-weight: 700; letter-spacing: 0.05em; cursor: pointer; }
  #estopBtn:active { filter: brightness(0.85); }
  #estopMsg { text-align: center; font-size: 0.85rem; margin-top: -8px; margin-bottom: 8px;
    color: var(--text-secondary); min-height: 1.2em; }
</style>
</head>
<body>
<h1><span class="dot" id="connDot" style="background:var(--critical)"></span> 맵 대시보드</h1>
<button id="estopBtn">🛑 비상정지 (자율주행 전체 중단)</button>
<div id="estopMsg"></div>
<div class="card">
  <canvas id="mapCanvas" width="10" height="10"></canvas>
  <div id="emptyMsg" class="empty">맵 데이터 없음 — SLAM Toolbox(mapping.launch.py)를 실행하면 여기 실시간으로 채워집니다.</div>
  <div class="status-row">
    <span id="dimText"></span>
    <span id="updatedText"></span>
  </div>
</div>

<script>
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');

// occupancy(-1~100)를 회색조로: -1(미지)=중간회색, 0(빈공간)=흰색, 100(장애물)=검정
function occColor(v) {
  if (v === -1) return 128;
  const clamped = Math.max(0, Math.min(100, v));
  return Math.round(255 * (1 - clamped / 100));
}

async function tick() {
  let d;
  try { d = await (await fetch('/api/map')).json(); } catch (e) { return; }

  document.getElementById('connDot').style.background = d.connected ? 'var(--good)' : 'var(--critical)';

  if (!d.connected || !d.data_b64) {
    document.getElementById('emptyMsg').style.display = 'block';
    canvas.style.display = 'none';
    return;
  }
  document.getElementById('emptyMsg').style.display = 'none';
  canvas.style.display = 'block';

  const bin = atob(d.data_b64);
  const w = d.width, h = d.height;
  canvas.width = w;
  canvas.height = h;
  const imgData = ctx.createImageData(w, h);
  for (let i = 0; i < w * h; i++) {
    const raw = bin.charCodeAt(i);           // 0~255 (unsigned로 옴)
    const v = raw > 100 ? -1 : raw;          // 255였던 -1 복원
    const g = occColor(v);
    // OccupancyGrid는 (0,0)이 좌하단부터 row-major라 캔버스(좌상단 시작)로 옮길 때 y 뒤집기
    const row = Math.floor(i / w);
    const col = i % w;
    const destRow = h - 1 - row;
    const destIdx = (destRow * w + col) * 4;
    imgData.data[destIdx] = g;
    imgData.data[destIdx + 1] = g;
    imgData.data[destIdx + 2] = g;
    imgData.data[destIdx + 3] = 255;
  }
  ctx.putImageData(imgData, 0, 0);

  document.getElementById('dimText').textContent =
    `${w}×${h} 셀, 해상도 ${d.resolution.toFixed(3)}m/셀 (약 ${(w*d.resolution).toFixed(1)}×${(h*d.resolution).toFixed(1)}m)`;
  document.getElementById('updatedText').textContent =
    '마지막 갱신: ' + new Date(d.last_ts * 1000).toLocaleTimeString()
    + (d.stale ? '  (갱신 멈춤 — 저장된 맵이거나 SLAM 정지)' : '');
}

tick();
setInterval(tick, 1500);

document.getElementById('estopBtn').addEventListener('click', async () => {
  const msg = document.getElementById('estopMsg');
  msg.textContent = '정지 명령 전송 중...';
  try {
    const res = await (await fetch('/api/estop', {method: 'POST'})).json();
    const base = res.killed && res.killed.length
      ? `정지됨 — 종료된 프로세스: ${res.killed.join(', ')}`
      : '정지 명령 발행됨 (실행 중이던 자율주행 프로세스는 없었음)';
    // zero_published=false면 ROS 발행이 막힌 것. 프로세스는 죽였으므로
    // diff_drive의 cmd_vel_timeout(0.5초)으로 정지는 되지만, 알려는 준다.
    msg.textContent = res.zero_published === false
      ? base + ' ⚠️ 0속도 발행 실패 — 0.5초 내 자동정지로 대체됨'
      : base;
  } catch (e) {
    msg.textContent = '정지 요청 실패 — 네트워크 확인 필요';
  }
});
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
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
        elif self.path == "/api/map":
            with state_lock:
                self._send(200, json.dumps(state).encode("utf-8"), "application/json")
        else:
            self._send(404, b"not found", "text/plain")

    def do_POST(self):
        if self.path == "/api/estop":
            # ⚠️ 순서가 중요하다. 예전엔 0 속도 발행을 먼저 했는데, DDS가 꼬여서
            # publish가 안 돌아오면 뒤의 pkill까지 영영 도달을 못 했다(실제로
            # 2026-08-16에 비상정지 API가 통째로 응답불능이 된 걸 확인).
            #
            # 진짜 정지는 "명령을 만들어내는 프로세스를 죽이는 것"이다 —
            # diff_drive_node가 cmd_vel이 0.5초만 끊겨도 스스로 정지하므로
            # (cmd_vel_timeout), 소스를 죽이면 0.5초 안에 반드시 선다.
            # 0 속도 발행은 그 0.5초를 더 앞당기는 보조 수단일 뿐이라 나중에,
            # 그리고 멈춰도 응답을 막지 않도록 타임아웃을 걸고 돌린다.
            killed = kill_autonomous_processes()
            zero_ok = Handler.node.emergency_stop_async(timeout=1.5)
            self._send(200, json.dumps({
                "ok": True, "killed": killed,
                # false면 ROS 발행이 멈춘 것 — 그래도 위에서 소스를 죽였으므로
                # cmd_vel_timeout으로 정지는 보장된다. UI에 경고로 표시.
                "zero_published": zero_ok,
            }).encode("utf-8"), "application/json")
        else:
            self._send(404, b"not found", "text/plain")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8768)
    args = ap.parse_args()

    rclpy.init()
    node = MapBridge()
    Handler.node = node

    threading.Thread(target=staleness_watchdog, daemon=True).start()
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)

    # 2026-08-16: 다른 대시보드들과 동일한 이유로 SIGTERM 핸들러 +
    # spin 스레드 정리 추가 (재부팅/재시작 시 지연·core dump 방지).
    def _sig_handler(signum, frame):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, _sig_handler)
    signal.signal(signal.SIGINT, _sig_handler)

    print(f"맵 대시보드 시작: http://<Pi IP>:{args.port}")
    try:
        server.serve_forever()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == "__main__":
    main()
