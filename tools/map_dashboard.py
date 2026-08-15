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
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

state_lock = threading.Lock()
state = {"connected": False, "last_ts": 0.0}
STALE_S = 5.0


class MapBridge(Node):
    def __init__(self):
        super().__init__('map_dashboard_bridge')
        self.create_subscription(OccupancyGrid, 'map', self._on_map, 1)

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
    while True:
        with state_lock:
            if state["connected"] and time.time() - state["last_ts"] > STALE_S:
                state["connected"] = False
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
</style>
</head>
<body>
<h1><span class="dot" id="connDot" style="background:var(--critical)"></span> 맵 대시보드</h1>
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
    '마지막 갱신: ' + new Date(d.last_ts * 1000).toLocaleTimeString();
}

tick();
setInterval(tick, 1500);
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8768)
    args = ap.parse_args()

    rclpy.init()
    node = MapBridge()

    threading.Thread(target=staleness_watchdog, daemon=True).start()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"맵 대시보드 시작: http://<Pi IP>:{args.port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
