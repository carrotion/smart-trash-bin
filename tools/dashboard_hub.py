#!/usr/bin/env python3
"""스마트 쓰레기통 — 전체 모듈 허브 대시보드.

전원(power_monitor.py, :8765), 모터(motor_dashboard.py, :8766), IMU+LiDAR
(sensor_dashboard.py, :8767)의 요약 상태를 카드로 보여주고, 클릭하면 각 상세
대시보드로 이동한다.

브라우저에서 다른 포트로 직접 fetch하면 CORS에 막히므로, 이 허브 서버가
서버 쪽에서 각 서비스의 /api/status를 대신 조회해서 넘겨준다.

사용법:
    python3 tools/dashboard_hub.py [--port 8764]
    (power_monitor.py, motor_dashboard.py, sensor_dashboard.py가 먼저 떠 있어야 요약이 채워짐)
"""
import argparse
import json
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MODULES = {
    "power": {"url": "http://localhost:8765/api/status", "dash_port": 8765},
    "motor": {"url": "http://localhost:8766/api/status", "dash_port": 8766},
    "sensors": {"url": "http://localhost:8767/api/status", "dash_port": 8767},
    "map": {"url": "http://localhost:8768/api/map", "dash_port": 8768},
}


def fetch_json(url, timeout=1.0):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return json.loads(resp.read())
    except Exception:
        return None


PAGE = """<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<title>시스템 대시보드</title>
<style>
  :root {
    color-scheme: light;
    --surface-1: #fcfcfb; --page: #f9f9f7;
    --text-primary: #0b0b0b; --text-secondary: #52514e; --text-muted: #898781;
    --border: rgba(11,11,11,0.10);
    --accent: #2a78d6;
    --good: #0ca30c; --critical: #d03b3b; --idle: #898781;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --surface-1: #1a1a19; --page: #0d0d0d;
      --text-primary: #ffffff; --text-secondary: #c3c2b7; --text-muted: #898781;
      --border: rgba(255,255,255,0.10);
      --accent: #3987e5; --good: #0ca30c; --critical: #e66767; --idle: #6b6a66;
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --surface-1: #1a1a19; --page: #0d0d0d;
    --text-primary: #ffffff; --text-secondary: #c3c2b7; --text-muted: #898781;
    --border: rgba(255,255,255,0.10);
    --accent: #3987e5; --good: #0ca30c; --critical: #e66767; --idle: #6b6a66;
  }
  body { background: var(--page); color: var(--text-primary);
    font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
    max-width: 640px; margin: 24px auto; padding: 0 16px; }
  h1 { font-size: 1.3rem; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-top: 16px; }
  a.card { display: block; background: var(--surface-1); border: 1px solid var(--border);
    border-radius: 14px; padding: 16px; text-decoration: none; color: inherit;
    transition: border-color 0.15s; }
  a.card:hover { border-color: var(--accent); }
  a.card.disabled { opacity: 0.55; pointer-events: none; }
  .card-head { display: flex; align-items: center; gap: 8px; font-weight: 600; }
  .dot { width: 10px; height: 10px; border-radius: 50%; flex-shrink: 0; }
  .metric { font-size: 1.6rem; font-weight: 600; margin-top: 10px;
    font-variant-numeric: tabular-nums; }
  .sub { font-size: 0.8rem; color: var(--text-secondary); margin-top: 2px; }
  .rows { margin-top: 10px; font-size: 0.85rem; font-variant-numeric: tabular-nums; }
  .row { display: flex; justify-content: space-between; gap: 8px; padding: 2px 0; }
  .row .k { color: var(--text-secondary); }
  .row .v { font-weight: 600; }
  .row .v.warn { color: var(--critical); }
  .updated { font-size: 0.7rem; color: var(--text-muted); margin-top: 16px; }
</style>
</head>
<body>
<h1>🗑️ 스마트 쓰레기통 — 시스템 대시보드</h1>
<div class="grid" id="grid"></div>
<div class="updated" id="updated"></div>

<script>
const MODULES = [
  {key:'pi',    icon:'🖥️', label:'Pi', port:8765},
  {key:'motor', icon:'⚙️', label:'모터', port:8766},
  {key:'imu',   icon:'🧭', label:'IMU', port:8767},
  {key:'lidar', icon:'📡', label:'LiDAR', port:8767},
  {key:'map',   icon:'🗺️', label:'맵', port:8768},
];

function card(mod, statusHtml, dotColor, disabled) {
  const href = mod.port ? `http://${location.hostname}:${mod.port}/` : '#';
  return `<a class="card ${disabled ? 'disabled' : ''}" href="${href}">
    <div class="card-head">
      <span class="dot" style="background:${dotColor}"></span>
      <span>${mod.icon} ${mod.label}</span>
    </div>
    ${statusHtml}
  </a>`;
}

// 카드 안에 "항목: 값" 3~4줄로 핵심 정보만. 상세는 클릭해서 각 대시보드로.
function rows(pairs) {
  return `<div class="rows">` + pairs.map(([k, v, warn]) =>
    `<div class="row"><span class="k">${k}</span><span class="v${warn ? ' warn' : ''}">${v}</span></div>`
  ).join('') + `</div>`;
}
const f = (v, d, unit) => (v === null || v === undefined) ? '-' : v.toFixed(d) + unit;

async function tick() {
  let summary;
  try { summary = await (await fetch('/api/summary')).json(); }
  catch (e) { return; }

  const grid = document.getElementById('grid');
  const sensorsAvail = summary.sensors && summary.sensors.available;

  grid.innerHTML = MODULES.map(mod => {
    // Pi 자체 상태 (전원 + CPU 온도/사용률/메모리) — power_monitor.py(:8765)가 서빙
    if (mod.key === 'pi') {
      const d = summary.power;
      if (!d || !d.available) {
        return card(mod, `<div class="sub">대시보드 응답 없음</div>`, 'var(--idle)', true);
      }
      const sys = (d.latest && d.latest.system) || {};
      const rails = (d.latest && d.latest.rails) || {};
      const flags = (d.latest && d.latest.throttled && d.latest.throttled.flags) || {};
      const bad = flags.under_voltage_now || flags.throttled_now;
      const hot = sys.cpu_temp_c != null && sys.cpu_temp_c >= 70;
      const v = rails.EXT5V_V ? rails.EXT5V_V.value : null;
      return card(mod, rows([
        ['5V 입력', f(v, 2, 'V'), bad],
        ['CPU 온도', f(sys.cpu_temp_c, 1, '°C'), hot],
        ['CPU 사용률', f(sys.cpu_percent, 0, '%')],
        ['메모리', f(sys.mem_percent, 0, '%')],
      ]), (bad || hot) ? 'var(--critical)' : 'var(--good)', false);
    }

    if (mod.key === 'motor') {
      const d = summary.motor;
      if (!d || !d.available) {
        return card(mod, `<div class="sub">대시보드 응답 없음</div>`, 'var(--idle)', true);
      }
      const connected = d.connected;
      const pwm = d.commanded_pwm || [0,0,0,0];
      const rate = d.rate || [0,0,0,0];
      const moving = pwm.some(v => v !== 0);
      const movingWheels = rate.filter(r => Math.abs(r) > 30).length;
      return card(mod, rows([
        ['상태', moving ? '주행 중' : '정지'],
        ['드라이버', connected ? '연결됨' : '연결 안 됨', !connected],
        ['회전 중 바퀴', `${movingWheels} / 4`],
        ['PWM', pwm.join(', ')],
      ]), !connected ? 'var(--critical)' : (moving ? 'var(--accent)' : 'var(--good)'), false);
    }

    // imu / lidar는 sensor_dashboard.py(:8767) 하나가 둘 다 서빙
    if (mod.key === 'imu' || mod.key === 'lidar') {
      if (!sensorsAvail) {
        return card(mod, `<div class="sub">대시보드 응답 없음</div>`, 'var(--idle)', true);
      }
      if (mod.key === 'imu') {
        const imu = summary.sensors.imu || {};
        if (!imu.connected) {
          return card(mod, `<div class="sub">토픽 응답 없음</div>`, 'var(--critical)', false);
        }
        const a = imu.linear_acceleration || {};
        const g = imu.angular_velocity || {};
        const accelMag = Math.hypot(a.x ?? 0, a.y ?? 0, a.z ?? 0);
        return card(mod, rows([
          ['가속도 크기', f(accelMag, 2, ' m/s²')],
          ['가속도 Z', f(a.z, 2, '')],
          ['각속도 Z', f(g.z, 3, ' rad/s')],
          ['각속도 X/Y', `${f(g.x, 2, '')} / ${f(g.y, 2, '')}`],
        ]), 'var(--good)', false);
      }
      const lidar = summary.sensors.lidar || {};
      if (!lidar.connected) {
        return card(mod, `<div class="sub">토픽 응답 없음</div>`, 'var(--critical)', false);
      }
      return card(mod, rows([
        ['유효 포인트', `${lidar.valid_point_count ?? '-'} / ${lidar.point_count ?? '-'}`],
        ['최소 거리', f(lidar.min_range, 2, ' m')],
        ['평균 거리', f(lidar.avg_range, 2, ' m')],
        ['최대 거리', f(lidar.max_range, 2, ' m')],
      ]), 'var(--good)', false);
    }

    // map: map_dashboard.py(:8768), SLAM Toolbox가 안 떠있으면 미연결이 정상 상태
    const mapData = summary.map;
    if (!mapData || !mapData.available || !mapData.connected) {
      return card(mod, `<div class="sub">맵핑 미실행 (정상 대기 상태)</div>`, 'var(--idle)', false);
    }
    const areaW = mapData.width * mapData.resolution;
    const areaH = mapData.height * mapData.resolution;
    return card(mod, rows([
      ['그리드', `${mapData.width} × ${mapData.height}`],
      ['실제 크기', `${areaW.toFixed(1)} × ${areaH.toFixed(1)} m`],
      ['해상도', f(mapData.resolution, 2, ' m/셀')],
      ['갱신', new Date(mapData.last_ts * 1000).toLocaleTimeString()],
    ]), 'var(--good)', false);
  }).join('');

  document.getElementById('updated').textContent = '마지막 갱신: ' + new Date().toLocaleTimeString();
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
        elif self.path == "/api/summary":
            out = {}
            for key, cfg in MODULES.items():
                data = fetch_json(cfg["url"])
                out[key] = {"available": data is not None, **(data or {})}
            self._send(200, json.dumps(out).encode("utf-8"), "application/json")
        else:
            self._send(404, b"not found", "text/plain")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8764)
    args = ap.parse_args()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"허브 대시보드 시작: http://<Pi IP>:{args.port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
