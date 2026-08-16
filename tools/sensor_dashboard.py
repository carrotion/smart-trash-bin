#!/usr/bin/env python3
"""IMU(/imu/data) + LiDAR(/scan) 상태를 구독해서 웹으로 보여주는 브릿지.

이 자체는 센서를 직접 안 건드리고, 이미 떠있는 smart_trash_sensors의
imu_publisher_node와 ldlidar_stl_ros2_node가 퍼블리시하는 토픽을 구독만 함
(둘 다 별도 systemd 서비스로 상시 실행됨, tools/systemd/*.service 참고).

사용법:
    python3 tools/sensor_dashboard.py [--port 8767]
    브라우저에서 http://<Pi IP>:8767 접속
"""
import argparse
import json
import math
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan

state_lock = threading.Lock()
state = {
    "imu": {"connected": False, "last_ts": 0.0},
    "lidar": {"connected": False, "last_ts": 0.0},
}
STALE_S = 2.0


class SensorBridge(Node):
    def __init__(self):
        super().__init__('sensor_dashboard_bridge')
        self.create_subscription(Imu, 'imu/data', self._on_imu, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)

    def _on_imu(self, msg: Imu):
        with state_lock:
            state["imu"] = {
                "connected": True,
                "last_ts": time.time(),
                "linear_acceleration": {
                    "x": msg.linear_acceleration.x,
                    "y": msg.linear_acceleration.y,
                    "z": msg.linear_acceleration.z,
                },
                "angular_velocity": {
                    "x": msg.angular_velocity.x,
                    "y": msg.angular_velocity.y,
                    "z": msg.angular_velocity.z,
                },
            }

    def _on_scan(self, msg: LaserScan):
        valid = [r for r in msg.ranges if r and not math.isnan(r) and msg.range_min < r < msg.range_max]
        with state_lock:
            state["lidar"] = {
                "connected": True,
                "last_ts": time.time(),
                "point_count": len(msg.ranges),
                "valid_point_count": len(valid),
                "min_range": min(valid) if valid else None,
                "max_range": max(valid) if valid else None,
                "avg_range": sum(valid) / len(valid) if valid else None,
            }


def staleness_watchdog():
    while True:
        now = time.time()
        with state_lock:
            for key in ("imu", "lidar"):
                if state[key]["connected"] and now - state[key]["last_ts"] > STALE_S:
                    state[key]["connected"] = False
        time.sleep(0.5)


PAGE = """<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>센서 대시보드</title>
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
    max-width: 640px; margin: 24px auto; padding: 0 16px; }
  h1 { font-size: 1.3rem; }
  .card { background: var(--surface-1); border: 1px solid var(--border); border-radius: 12px;
    padding: 16px; margin: 16px 0; }
  .card-head { display: flex; align-items: center; gap: 8px; font-weight: 600; }
  .dot { width: 10px; height: 10px; border-radius: 50%; }
  table { width: 100%; border-collapse: collapse; font-size: 0.85rem; margin-top: 8px;
    font-variant-numeric: tabular-nums; }
  td, th { text-align: left; padding: 3px 6px; border-bottom: 1px solid var(--border); }
</style>
</head>
<body>
<h1>센서 대시보드</h1>

<div class="card">
  <div class="card-head"><span class="dot" id="imuDot"></span> IMU (ICM-20948)</div>
  <table id="imuTable"></table>
</div>

<div class="card">
  <div class="card-head"><span class="dot" id="lidarDot"></span> LiDAR (LD06)</div>
  <table id="lidarTable"></table>
</div>

<script>
async function tick() {
  let d;
  try { d = await (await fetch('/api/status')).json(); } catch (e) { return; }

  const imuDot = document.getElementById('imuDot');
  imuDot.style.background = d.imu.connected ? 'var(--good)' : 'var(--critical)';
  const imuRows = [];
  if (d.imu.connected) {
    const a = d.imu.linear_acceleration, g = d.imu.angular_velocity;
    imuRows.push(['가속도 x/y/z (m/s²)', `${a.x.toFixed(2)} / ${a.y.toFixed(2)} / ${a.z.toFixed(2)}`]);
    imuRows.push(['각속도 x/y/z (rad/s)', `${g.x.toFixed(2)} / ${g.y.toFixed(2)} / ${g.z.toFixed(2)}`]);
  } else {
    imuRows.push(['상태', '토픽 응답 없음']);
  }
  document.getElementById('imuTable').innerHTML =
    imuRows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join('');

  const lidarDot = document.getElementById('lidarDot');
  lidarDot.style.background = d.lidar.connected ? 'var(--good)' : 'var(--critical)';
  const lidarRows = [];
  if (d.lidar.connected) {
    lidarRows.push(['포인트 수 (유효/전체)', `${d.lidar.valid_point_count} / ${d.lidar.point_count}`]);
    lidarRows.push(['거리 최소/평균/최대 (m)',
      `${d.lidar.min_range?.toFixed(2) ?? '-'} / ${d.lidar.avg_range?.toFixed(2) ?? '-'} / ${d.lidar.max_range?.toFixed(2) ?? '-'}`]);
  } else {
    lidarRows.push(['상태', '토픽 응답 없음']);
  }
  document.getElementById('lidarTable').innerHTML =
    lidarRows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join('');
}
tick();
setInterval(tick, 500);
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
        elif self.path == "/api/status":
            with state_lock:
                self._send(200, json.dumps(state).encode("utf-8"), "application/json")
        else:
            self._send(404, b"not found", "text/plain")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8767)
    args = ap.parse_args()

    rclpy.init()
    node = SensorBridge()

    threading.Thread(target=staleness_watchdog, daemon=True).start()
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)

    # systemd가 SIGTERM을 보내도 serve_forever()가 이를 감지 못해 90초 뒤
    # SIGKILL로 강제종료되는 문제(2026-08-16 감사 리포트로 확인) 방지.
    def _sig_handler(signum, frame):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, _sig_handler)
    signal.signal(signal.SIGINT, _sig_handler)

    print(f"센서 대시보드 시작: http://<Pi IP>:{args.port}")
    try:
        server.serve_forever()
    finally:
        # motor_dashboard.py와 동일한 이유(2026-08-16) — spin() 데몬 스레드를
        # 정리 안 하고 인터프리터를 그냥 종료하면 ROS 그래프가 클 때
        # "terminate called without an active exception"로 core dump 남.
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == "__main__":
    main()
