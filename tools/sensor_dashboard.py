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
import gzip
import json
import math
import signal
import threading
import time
from dashboard_icon import FAVICON_TAG
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

# 자기 편각(자북 -> 진북, 동쪽 양수, 단위 도). --declination 으로 덮어쓴다.
# 0이면 자북 기준. 폰 나침반(진북 표시)과 맞추려면 실측해서 채울 것 —
# 서로 다른 방위 3곳 이상에서 폰과 비교한 차이의 평균이다(한 번만 재면 소프트아이언
# 변동분과 구분이 안 된다). 참고: 한국은 대략 8° 서편(음수) 정도.
DECLINATION_DEG = 0.0


class SensorBridge(Node):
    def __init__(self):
        super().__init__('sensor_dashboard_bridge')
        self.create_subscription(Imu, 'imu/data', self._on_imu, 10)
        self.create_subscription(MagneticField, 'imu/mag', self._on_mag, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)

    def _on_mag(self, msg: MagneticField):
        """지자기(AK09916). 아직 어떤 추정에도 안 쓰고 **표시만** 한다.

        2026-08-17 초기 실측(IMU가 차체 모서리 근처): 총 세기 123µT, 모터 구동 중
        52~69µT 요동 -> 주행 중 사용 불가로 판단했었다.

        같은 날 **IMU를 차체 중앙으로 이설한 뒤 재측정하니 결과가 뒤집혔다**:
        모터를 220~245RPM으로 돌려도 방위 편차가 1° 미만(정지 구간의 노이즈와 같은
        수준)이었다. 모터가 차체 모서리에 있어서 중앙이 자기적으로 가장 조용하다.
        측정: tools/mag_motor_interference.py (도크 위에서 바퀴를 띄워 차체 회전을
        배제 -> 자기장 변화가 순수하게 모터 간섭만 남는다).

        ⚠️ 다만 그 측정은 **무부하**였다. 영구자석 영향은 부하와 무관하지만 전류가
        만드는 자기장은 전류에 비례하므로, 바닥에서 마찰/가속 부하를 받으면 더 커진다.
        바닥 주행 재측정 전까지는 **정지 상태 전용**으로 취급한다.
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
                # 수평 세기가 지구 자기장 수준을 벗어나면 주변 철재/모터 간섭이다.
                "usable": 15.0 <= horiz <= 60.0,
                # 나침반 방위 = -atan2(By, Bx)  ← 부호에 주의.
                # 로봇 좌표계는 x=전방, y=좌측(REP-103). atan2(By,Bx)는 "로봇에서 봐서
                # 북쪽이 어느 방향인가"를 반시계 양수로 잰 값이다. 반면 나침반 방위는
                # "북쪽에서 봐서 로봇이 어디를 보는가"를 시계 양수로 재므로 정확히 반대다.
                # 2026-08-17 실측 검증: 부호 없이 109°가 나왔을 때 폰 나침반은 259°,
                # -109 mod 360 = 251°로 8° 이내 일치(잔차는 편각+미보정 하드아이언).
                #
                # DECLINATION_DEG: 자북 -> 진북 보정(동쪽이 양수). 폰 나침반은 보통
                # **진북**을 표시하므로 이걸 넣어야 폰과 맞는다. 로봇 항법만 생각하면
                # 자북도 고정 기준이라 상관없지만, 사람이 폰과 비교해 읽으려면 필요하다.
                # ⚠️ 이건 **설치 지역** 값이지 기체 값이 아니다 — 나중에 site.yaml로
                #    분리할 것(fleet 문서 참고). 0이면 자북 기준 그대로.
                "heading_deg": (
                    -math.degrees(math.atan2(y, x)) + DECLINATION_DEG) % 360.0,
                "declination_deg": DECLINATION_DEG,
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
__FAVICON__
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
  #rose { transition: transform 0.25s ease-out; }
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
      <!-- 실제 나침반과 같은 방식: **바늘(로봇)은 12시에 고정**하고 방위판(NSEW)이 돈다.
           로봇 실물을 앞에 두고 화면과 나란히 보면, 화면 위쪽이 곧 로봇의 정면이라
           "지금 어느 쪽을 보고 있나"가 바로 읽힌다. -->
      <circle cx="100" cy="100" r="86" fill="var(--surface-2, rgba(127,127,127,0.06))"
              stroke="var(--border)" stroke-width="1"/>
      <circle cx="100" cy="100" r="68" fill="none"
              stroke="var(--border)" stroke-width="1" stroke-dasharray="2 4"/>

      <!-- 돌아가는 방위판. JS가 -방위만큼 돌린다(로봇이 오른쪽으로 돌면 판은 왼쪽으로). -->
      <g id="rose">
        <g id="ticks"></g>
        <!-- 각 글자를 자기 방위각만큼 미리 돌려둔다. 그러면 판이 -방위로 돌 때
             맨 위에 오는 글자의 최종 회전이 0이 되어 **항상 똑바로 선다**.
             (그냥 붙이면 남쪽을 볼 때 S가 거꾸로 뒤집힌다) -->
        <!-- 반지름 배치: 눈금 76~86, 점선 68, 글자 중심 약 59, 바늘 끝 48.
             겹치지 않게 안쪽부터 바늘 > 글자 > 눈금 순으로 벌려 뒀다. -->
        <g transform="rotate(0 100 100)">
          <polygon points="100,26 95.5,35 104.5,35" fill="var(--critical)"/>
          <text x="100" y="52" text-anchor="middle" font-size="17" font-weight="700"
                fill="var(--critical)">N</text>
        </g>
        <g transform="rotate(90 100 100)">
          <text x="100" y="50" text-anchor="middle" font-size="14" font-weight="600"
                fill="var(--text-secondary)">E</text>
        </g>
        <g transform="rotate(180 100 100)">
          <text x="100" y="50" text-anchor="middle" font-size="14" font-weight="600"
                fill="var(--text-secondary)">S</text>
        </g>
        <g transform="rotate(270 100 100)">
          <text x="100" y="50" text-anchor="middle" font-size="14" font-weight="600"
                fill="var(--text-secondary)">W</text>
        </g>
      </g>

      <!-- 읽는 기준선(lubber line). 고정. 이 눈금이 가리키는 방위판 글자가 현재 방위다. -->
      <polygon points="100,6 96,15 104,15" fill="var(--text-primary)"/>

      <!-- 로봇(바늘): 12시 고정. 화면 위 = 로봇 정면. -->
      <g id="robot">
        <polygon points="100,52 87,128 100,112 113,128" fill="var(--good)"/>
        <circle cx="100" cy="100" r="5" fill="var(--good)"/>
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
  // 화면이 안 보이면 폴링하지 않는다. 폰 화면을 끄거나 다른 앱으로 전환해도
  // 계속 돌면 CPU와 배터리를 낭비한다. 이 프로젝트에서 프리즈의 유력한
  // 방아쇠가 CPU 부하였다.
  if (document.hidden) return;
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
    magRows.push(['주의', '정지 상태에서 유효. 무부하 모터 간섭은 1° 미만으로 측정됐으나 '
                        + '바닥 주행(부하) 재측정 전까지는 주행 중 사용 보류']);
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
  const roseG = document.getElementById('rose');
  const robotG = document.getElementById('robot');
  const bearingText = document.getElementById('bearingText');
  if (d.mag && d.mag.connected) {
    const bearing = ((d.mag.heading_deg % 360) + 360) % 360;

    // 각도 언랩. 0~360으로 자른 값을 그대로 넘기면 359°->2° 같은 경계에서 차이가
    // 357°가 되고, CSS transition이 이걸 회전으로 보간하지 못해 **행렬로 분해**한다.
    // 그러면 평행이동 성분이 섞여서 방위판이 회전 중심을 벗어나 크게 휘돈다(실사용에서
    // 관측). 누적각에 -180~180 최단 변화량만 더해 항상 연속으로 만들어 준다.
    let delta = ((bearing - (displayAngle % 360)) % 360 + 540) % 360 - 180;
    displayAngle += delta;
    // 방위판은 로봇과 **반대로** 돈다: 로봇이 오른쪽(시계)으로 돌면 북쪽은 화면에서
    // 왼쪽(반시계)으로 흘러가야 한다. 그래야 12시 고정 바늘이 실제 정면과 맞는다.
    roseG.setAttribute('transform', `rotate(${(-displayAngle).toFixed(1)} 100 100)`);

    const dirs = ['북','북동','동','남동','남','남서','서','북서'];
    const name = dirs[Math.round(bearing / 45) % 8];
    bearingText.textContent = `${name} ${bearing.toFixed(0)}°`;
    bearingText.style.opacity = d.mag.usable ? '1' : '0.5';
    roseG.style.opacity = d.mag.usable ? '1' : '0.35';
    robotG.style.opacity = '1';
  } else {
    roseG.style.opacity = '0.2';
    robotG.style.opacity = '0.4';
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
document.addEventListener('visibilitychange', () => {
  if (!document.hidden) tick();
});
</script>
</body>
</html>
"""

# 파비콘 삽입. PAGE가 일반 문자열이라 f-string 치환을 못 쓰므로 여기서 바꾼다.
PAGE = PAGE.replace("__FAVICON__", FAVICON_TAG)



class Handler(BaseHTTPRequestHandler):
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
            self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif self.path == "/api/status":
            with state_lock:
                self._send(200, json.dumps(state).encode("utf-8"), "application/json")
        else:
            self._send(404, b"not found", "text/plain")


def main():
    # global 선언은 이름을 쓰기 **전에** 와야 한다 (아래 add_argument의 default에서
    # 이미 읽으므로, 뒤에 두면 SyntaxError).
    global DECLINATION_DEG

    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8767)
    ap.add_argument("--declination", type=float, default=DECLINATION_DEG,
                    help="자기 편각(도, 동쪽 양수). 폰 나침반(진북)과 맞출 때 쓴다. "
                         "설치 지역 값이며 기체 값이 아님.")
    args = ap.parse_args()
    DECLINATION_DEG = args.declination

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