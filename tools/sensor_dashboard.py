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
from sensor_msgs.msg import Imu, LaserScan, MagneticField
import math

state_lock = threading.Lock()
state = {
    "imu": {"connected": False, "last_ts": 0.0},
    "mag": {"connected": False, "last_ts": 0.0},
    "lidar": {"connected": False, "last_ts": 0.0},
}
STALE_S = 2.0


class SensorBridge(Node):
    def __init__(self):
        super().__init__('sensor_dashboard_bridge')
        self.create_subscription(Imu, 'imu/data', self._on_imu, 10)
        self.create_subscription(MagneticField, 'imu/mag', self._on_mag, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)

    def _on_mag(self, msg: MagneticField):
        """지자기(AK09916). 아직 어떤 추정에도 안 쓰고 **표시만** 한다.

        2026-08-17 실측: 정지 시 총 세기 123µT(지구 자기장 25~65µT의 2~5배)로,
        로봇 자신의 모터 자석이 만드는 하드아이언 편향이 크다. 게다가 **모터가
        도는 동안 52~69µT나 요동친다** — 지구 자기장 크기와 맞먹어 주행 중에는
        방위 판단에 못 쓴다.
        결론(사용자와 합의): **정지 상태에서만** 방위 기준으로 쓸 여지가 있다.
        여기 표시하는 건 그 판단을 사용자가 언제든 눈으로 확인하기 위함.
        """
        x = msg.magnetic_field.x * 1e6      # 테슬라 -> µT
        y = msg.magnetic_field.y * 1e6
        z = msg.magnetic_field.z * 1e6
        # 나침반 방위는 **수평 성분(X,Y)만** 쓴다. Z는 방위에 안 들어가므로,
        # 신뢰도도 총 세기가 아니라 수평 세기로 판단해야 한다.
        # 한국의 지구 자기장 수평 성분은 대략 30µT.
        horiz = math.sqrt(x * x + y * y)
        with state_lock:
            state["mag"] = {
                "connected": True,
                "last_ts": time.time(),
                "x": x, "y": y, "z": z,
                "norm": math.sqrt(x * x + y * y + z * z),
                "horiz": horiz,
                # 하드아이언 보정 전이라 절대 방위는 못 믿는다. 다만 수평 세기가
                # 지구 자기장 수준이면 "회전에 따라 방향이 바뀌는 것"은 볼 수 있다.
                "usable": 15.0 <= horiz <= 60.0,
                # 나침반 방위 = -atan2(By, Bx)  ← 부호에 주의.
                # 로봇 좌표계는 x=전방, y=좌측(REP-103). atan2(By,Bx)는 "로봇에서 봐서
                # 북쪽이 어느 방향인가"를 반시계 양수로 잰 값이다. 반면 나침반 방위는
                # "북쪽에서 봐서 로봇이 어디를 보는가"를 시계 양수로 재므로 정확히 반대다.
                # 2026-08-17 실측 검증: 부호 없이 109°가 나왔을 때 폰 나침반은 259°,
                # -109 mod 360 = 251°로 8° 이내 일치(잔차는 편각+미보정 하드아이언).
                "heading_deg": (-math.degrees(math.atan2(y, x))) % 360.0,
            }

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
            for key in ("imu", "mag", "lidar"):
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
  .compass-wrap { display: flex; flex-direction: column; align-items: center; margin: 4px 0 12px; }
  #bearingText { font-size: 20px; font-weight: 700; margin-top: 2px; }
  #compass { width: 200px; height: 200px; }
  /* transform-origin을 쓰면 안 된다: 아래 JS가 거는 SVG rotate(각도 100 100)는
     회전 중심을 인자로 이미 갖고 있어서, CSS transform-origin이 겹치면 중심이
     (200,200)으로 밀려 화살표가 뷰박스 밖으로 날아간다(방위 0°일 때만 보임). */
  #robot { transition: transform 0.25s ease-out; }
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
  <div class="card-head"><span class="dot" id="magDot"></span> 지자기 나침반 (AK09916)</div>
  <div class="compass-wrap">
    <svg id="compass" viewBox="0 0 200 200" role="img" aria-label="로봇 방위 나침반">
      <!-- 북쪽이 항상 위. 로봇 아이콘이 실제 바라보는 방위로 돌아간다. -->
      <circle cx="100" cy="100" r="86" fill="var(--surface-2, rgba(127,127,127,0.06))"
              stroke="var(--border)" stroke-width="1"/>
      <circle cx="100" cy="100" r="70" fill="none"
              stroke="var(--border)" stroke-width="1" stroke-dasharray="2 4"/>
      <g id="ticks"></g>
      <!-- 북쪽 표식: 고정 -->
      <polygon points="100,16 94,30 106,30" fill="var(--critical)"/>
      <text x="100" y="44" text-anchor="middle" font-size="15" font-weight="700"
            fill="var(--critical)">N</text>
      <text x="172" y="106" text-anchor="middle" font-size="13" fill="var(--text-secondary)">E</text>
      <text x="100" y="192" text-anchor="middle" font-size="13" fill="var(--text-secondary)">S</text>
      <text x="28"  y="106" text-anchor="middle" font-size="13" fill="var(--text-secondary)">W</text>
      <!-- 로봇: bearing 만큼 시계방향 회전.
           도형은 회전 중심(100,100)을 기준으로 위아래 균형이 맞아야 한다 —
           전에는 꼭짓점 y=42 / 꼬리 y=116이라 축이 꼬리 쪽에 치우쳐서, 돌 때
           바늘이 한쪽으로 쏠려 도는 것처럼 보였다. -->
      <g id="robot">
        <polygon points="100,38 84,132 100,114 116,132" fill="var(--good)"/>
        <circle cx="100" cy="100" r="5.5" fill="var(--good)"/>
      </g>
    </svg>
    <!-- 방위 텍스트는 SVG 밖에 둔다. 안에 두면 바늘이 원 전체를 쓸고 지나가면서
         글자와 겹쳐 둘 다 안 읽힌다(실사용에서 확인). -->
    <div id="bearingText"></div>
  </div>
  <table id="magTable"></table>
</div>

<div class="card">
  <div class="card-head"><span class="dot" id="lidarDot"></span> LiDAR (LD06)</div>
  <table id="lidarTable"></table>
</div>

<script>
// 나침반 바늘의 누적 회전각(도). 0~360으로 자르지 않고 계속 쌓아간다 — 이유는
// 아래 언랩 주석 참고.
let displayAngle = 0;

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

  const magDot = document.getElementById('magDot');
  magDot.style.background = d.mag && d.mag.connected ? 'var(--good)' : 'var(--critical)';
  const magRows = [];
  if (d.mag && d.mag.connected) {
    magRows.push(['성분 X/Y/Z (µT)',
      `${d.mag.x.toFixed(1)} / ${d.mag.y.toFixed(1)} / ${d.mag.z.toFixed(1)}`]);
    // 총 세기로 정상 여부를 판단하면 안 된다: Z축 하드아이언은 **제자리 수평 회전으로
    // 관측이 불가능**하다(회전해도 Z 성분이 안 변해서, 로봇의 편향인지 지구 자기장의
    // 수직 성분인지 구분이 안 됨. 보려면 로봇을 기울여야 한다). 그래서 Z는 미보정으로
    // 남아 있고 총 세기는 항상 부풀려 나온다. 방위는 수평 성분만 쓰므로 무해하다.
    magRows.push(['총 세기 (µT)',
      `${d.mag.norm.toFixed(1)}  (Z축 미보정분 포함 — 방위와 무관)`]);
    // 방위 정확도를 좌우하는 건 이 값. 한국 수평 성분은 약 30µT.
    const h = d.mag.horiz;
    const hwarn = (h < 15 || h > 60) ? ' ⚠ 지구자기장 범위 벗어남 — 철재/모터 간섭' : '';
    magRows.push(['수평 세기 (µT)', `${h.toFixed(1)}  (방위 계산에 쓰이는 성분, 한국 기준 약 30)${hwarn}`]);
    magRows.push(['신뢰도', d.mag.usable
      ? '✔ 지구 자기장 범위 — 참고 가능'
      : '⚠ 보정 안 됨 — 방향이 실제와 다를 수 있음']);
    magRows.push(['주의', '정지 상태에서만 유효 (주행 중엔 모터 간섭 52~69µT)']);
  } else {
    magRows.push(['상태', '토픽 응답 없음']);
  }
  document.getElementById('magTable').innerHTML =
    magRows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join('');

  // --- 나침반 그리기 ---
  // 북쪽을 항상 위로 고정하고, 로봇 아이콘을 실제 방위만큼 시계방향으로 돌린다.
  // 방위 계산(부호 포함)은 서버쪽 heading_deg에서 이미 끝났다 — 위 파이썬 주석 참고.
  // 여기서는 0~360으로 정규화만 하고 그대로 회전각으로 쓴다.
  const ticks = document.getElementById('ticks');
  if (ticks && !ticks.hasChildNodes()) {
    let html = '';
    for (let a = 0; a < 360; a += 15) {
      const major = a % 45 === 0;
      const r1 = 86, r2 = major ? 76 : 81;
      const rad = (a - 90) * Math.PI / 180;
      html += `<line x1="${100 + r1 * Math.cos(rad)}" y1="${100 + r1 * Math.sin(rad)}"
                     x2="${100 + r2 * Math.cos(rad)}" y2="${100 + r2 * Math.sin(rad)}"
                     stroke="var(--border)" stroke-width="${major ? 1.5 : 1}"/>`;
    }
    ticks.innerHTML = html;
  }
  const robotG = document.getElementById('robot');
  const bearingText = document.getElementById('bearingText');
  if (d.mag && d.mag.connected) {
    const bearing = ((d.mag.heading_deg % 360) + 360) % 360;

    // 각도 언랩. 0~360으로 자른 값을 그대로 넘기면 359°->2° 같은 경계에서 차이가
    // 357°가 되고, CSS transition이 이걸 회전으로 보간하지 못해 **행렬로 분해**한다.
    // 그러면 평행이동 성분이 섞여서 바늘이 회전 중심을 벗어나 크게 휘돈다(실사용에서
    // 관측). 누적각에 -180~180 최단 변화량만 더해 항상 연속으로 만들어 준다.
    let delta = ((bearing - (displayAngle % 360)) % 360 + 540) % 360 - 180;
    displayAngle += delta;
    robotG.setAttribute('transform', `rotate(${displayAngle.toFixed(1)} 100 100)`);

    const dirs = ['북','북동','동','남동','남','남서','서','북서'];
    const name = dirs[Math.round(bearing / 45) % 8];
    bearingText.textContent = `${name} ${bearing.toFixed(0)}°`;
    bearingText.style.opacity = d.mag.usable ? '1' : '0.5';
    robotG.style.opacity = d.mag.usable ? '1' : '0.35';
  } else {
    robotG.style.opacity = '0.2';
    bearingText.textContent = '—';
  }

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
