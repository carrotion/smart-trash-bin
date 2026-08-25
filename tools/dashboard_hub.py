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
import gzip
import json
import threading
import urllib.request
from collections import deque
from dashboard_icon import FAVICON_TAG
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


# Pi CPU/메모리 스파크라인용 이력. power_monitor.py의 /api/status는 더 이상
# history를 통째로 안 준다(예전엔 그것 때문에 매초 최대 590KB가 나갔다).
# 대신 여기서 /api/history?since=로 증분만 받아 누적한다 — 허브가 클라이언트
# 대신 유지하는 5분치 롤링 버퍼.
PI_HISTORY_LEN = 300
pi_history = deque(maxlen=PI_HISTORY_LEN)
pi_history_lock = threading.Lock()
pi_history_last_ts = 0.0


def poll_pi_history():
    global pi_history_last_ts
    d = fetch_json(f'http://localhost:8765/api/history?since={pi_history_last_ts}', timeout=2.0)
    if not d or not d.get("rows"):
        return
    with pi_history_lock:
        for row in d["rows"]:
            pi_history.append(row)
        pi_history_last_ts = d.get("last_ts", pi_history_last_ts)


PAGE = """<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
__FAVICON__
<title>시스템 대시보드</title>
<style>
  :root {
    color-scheme: light;
    --surface-1: #fcfcfb; --page: #f9f9f7;
    --text-primary: #0b0b0b; --text-secondary: #52514e; --text-muted: #898781;
    --border: rgba(11,11,11,0.10);
    --accent: #2a78d6;
    --good: #0ca30c; --critical: #d03b3b; --warning: #c98500; --idle: #898781;
    --series-cpu: #2a78d6; --series-mem: #eb6834;
    --grid-line: rgba(11,11,11,0.16);
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --surface-1: #1a1a19; --page: #0d0d0d;
      --text-primary: #ffffff; --text-secondary: #c3c2b7; --text-muted: #898781;
      --border: rgba(255,255,255,0.10);
      --accent: #3987e5; --good: #0ca30c; --critical: #e66767; --warning: #eda100; --idle: #6b6a66;
      --series-cpu: #3987e5; --series-mem: #d95926;
      --grid-line: rgba(255,255,255,0.16);
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --surface-1: #1a1a19; --page: #0d0d0d;
    --text-primary: #ffffff; --text-secondary: #c3c2b7; --text-muted: #898781;
    --border: rgba(255,255,255,0.10);
    --accent: #3987e5; --good: #0ca30c; --critical: #e66767; --idle: #6b6a66;
    --series-cpu: #3987e5; --series-mem: #d95926;
    --grid-line: rgba(255,255,255,0.16);
  }
  body { background: var(--page); color: var(--text-primary);
    font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
    max-width: 640px; margin: 24px auto; padding: 0 16px; }
  h1 { font-size: 1.3rem; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-top: 16px; position: relative; }
  a.card { display: block; background: var(--surface-1); border: 1px solid var(--border);
    border-radius: 14px; padding: 16px; text-decoration: none; color: inherit;
    transition: border-color 0.15s; }
  a.card:hover { border-color: var(--accent); }
  a.card.disabled { opacity: 0.55; pointer-events: none; }
  a.card.wide { grid-column: span 2; }
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
  .wide-body { display: flex; gap: 20px; margin-top: 10px; }
  .wide-body .rows { flex: 0 0 auto; min-width: 150px; margin-top: 0; }
  .chart-col { flex: 1 1 auto; min-width: 0; }
  .chart-legend { display: flex; gap: 14px; font-size: 0.75rem; color: var(--text-secondary); margin-bottom: 4px; }
  .chart-legend span { display: inline-flex; align-items: center; gap: 5px; }
  .chart-legend i { width: 8px; height: 8px; border-radius: 50%; display: inline-block; }
  .spark-wrap { position: relative; }
  .spark-wrap svg { display: block; width: 100%; height: 68px; overflow: visible; }
  .chart-tip { position: fixed; display: none; pointer-events: none; z-index: 5;
    background: var(--surface-1); border: 1px solid var(--border); border-radius: 8px;
    padding: 5px 8px; font-size: 0.72rem; color: var(--text-primary);
    box-shadow: 0 2px 8px rgba(0,0,0,0.15); white-space: nowrap; }
  .chart-tip .k { color: var(--text-secondary); }
</style>
</head>
<body>
<h1>🗑️ 스마트 쓰레기통 — 시스템 대시보드</h1>
<div class="grid" id="grid"></div>
<div class="chart-tip" id="chartTip"></div>
<div class="updated" id="updated"></div>

<script>
const MODULES = [
  {key:'pi',    icon:'🖥️', label:'Pi', port:8765},
  {key:'motor', icon:'⚙️', label:'모터', port:8766},
  {key:'imu',   icon:'🧭', label:'IMU', port:8767},
  {key:'lidar', icon:'📡', label:'LiDAR', port:8767},
  {key:'map',   icon:'🗺️', label:'맵', port:8768},
];

function card(mod, statusHtml, dotColor, disabled, wide) {
  const href = mod.port ? `http://${location.hostname}:${mod.port}/` : '#';
  return `<a class="card ${disabled ? 'disabled' : ''} ${wide ? 'wide' : ''}" href="${href}">
    <div class="card-head">
      <span class="dot" style="background:${dotColor}"></span>
      <span>${mod.icon} ${mod.label}</span>
    </div>
    ${statusHtml}
  </a>`;
}

// 최근 5분 CPU/메모리 스파크라인 (history: [{ts, system:{cpu_percent, mem_percent}}, ...])
function sparkChart(history) {
  const pts = (history || []).filter(s => s.system &&
    s.system.cpu_percent != null && s.system.mem_percent != null);
  if (pts.length < 2) {
    return `<div class="sub">이력 쌓는 중…</div>`;
  }
  const W = 240, H = 68;
  const x = i => (i / (pts.length - 1)) * W;
  const y = v => H - Math.max(0, Math.min(100, v)) / 100 * H;
  const line = key => pts.map((s, i) => `${x(i).toFixed(1)},${y(s.system[key]).toFixed(1)}`).join(' ');
  const last = pts[pts.length - 1];
  const dot = (key, color) => {
    const cx = x(pts.length - 1), cy = y(last.system[key]);
    return `<circle cx="${cx.toFixed(1)}" cy="${cy.toFixed(1)}" r="4" style="fill:${color}" />
      <circle cx="${cx.toFixed(1)}" cy="${cy.toFixed(1)}" r="4" style="fill:none;stroke:var(--surface-1);stroke-width:2" />`;
  };
  const histJson = JSON.stringify(pts.map(s => ({ ts: s.ts, cpu: s.system.cpu_percent, mem: s.system.mem_percent })))
    .replace(/"/g, '&quot;');
  return `
    <div class="chart-legend">
      <span><i style="background:var(--series-cpu)"></i>CPU ${last.system.cpu_percent.toFixed(0)}%</span>
      <span><i style="background:var(--series-mem)"></i>메모리 ${last.system.mem_percent.toFixed(0)}%</span>
      <span style="margin-left:auto;color:var(--text-muted)">최근 5분</span>
    </div>
    <div class="spark-wrap" data-history="${histJson}">
      <svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="none">
        <line x1="0" y1="${y(100).toFixed(1)}" x2="${W}" y2="${y(100).toFixed(1)}" stroke="var(--grid-line)" stroke-width="1" />
        <line x1="0" y1="${y(50).toFixed(1)}" x2="${W}" y2="${y(50).toFixed(1)}" stroke="var(--grid-line)" stroke-width="1" />
        <line x1="0" y1="${y(0).toFixed(1)}" x2="${W}" y2="${y(0).toFixed(1)}" stroke="var(--grid-line)" stroke-width="1" />
        <polyline points="${line('mem_percent')}" fill="none" style="stroke:var(--series-mem)" stroke-width="2" stroke-linejoin="round" stroke-linecap="round" />
        <polyline points="${line('cpu_percent')}" fill="none" style="stroke:var(--series-cpu)" stroke-width="2" stroke-linejoin="round" stroke-linecap="round" />
        ${dot('mem_percent', 'var(--series-mem)')}
        ${dot('cpu_percent', 'var(--series-cpu)')}
        <line class="crosshair" x1="0" y1="0" x2="0" y2="${H}" stroke="var(--text-muted)" stroke-width="1" style="display:none" />
      </svg>
    </div>`;
}

// 차트 hover는 #grid가 매 tick마다 innerHTML로 통째로 갈아끼워지므로,
// 개별 svg에 리스너를 다는 대신 #grid에 한 번만 위임 리스너를 건다.
(function setupChartHover() {
  const grid = document.getElementById('grid');
  const tip = document.getElementById('chartTip');
  grid.addEventListener('mousemove', (e) => {
    const wrap = e.target.closest('.spark-wrap');
    if (!wrap) { tip.style.display = 'none';
      grid.querySelectorAll('.crosshair').forEach(c => c.style.display = 'none');
      return; }
    const hist = JSON.parse(wrap.dataset.history);
    const rect = wrap.getBoundingClientRect();
    const frac = Math.max(0, Math.min(1, (e.clientX - rect.left) / rect.width));
    const idx = Math.round(frac * (hist.length - 1));
    const s = hist[idx];
    const cross = wrap.querySelector('.crosshair');
    if (cross) {
      cross.style.display = '';
      const cx = (idx / (hist.length - 1)) * 240;
      cross.setAttribute('x1', cx); cross.setAttribute('x2', cx);
    }
    const time = new Date(s.ts * 1000).toLocaleTimeString();
    tip.innerHTML = `<div>${time}</div>
      <div><span class="k">CPU</span> ${s.cpu.toFixed(1)}%</div>
      <div><span class="k">메모리</span> ${s.mem.toFixed(1)}%</div>`;
    tip.style.left = (e.clientX + 14) + 'px';
    tip.style.top = (e.clientY - 10) + 'px';
    tip.style.display = 'block';
  });
  grid.addEventListener('mouseleave', () => {
    tip.style.display = 'none';
    grid.querySelectorAll('.crosshair').forEach(c => c.style.display = 'none');
  });
})();

// 카드 안에 "항목: 값" 3~4줄로 핵심 정보만. 상세는 클릭해서 각 대시보드로.
// 3번째 값은 boolean(true=critical 빨강, 기존 동작 그대로) 또는 색상 문자열
// (예: 'var(--warning)')을 직접 지정 — WiFi처럼 3단계 이상 등급이 필요할 때 씀.
function rows(pairs) {
  return `<div class="rows">` + pairs.map(([k, v, warn]) => {
    const style = typeof warn === 'string' ? ` style="color:${warn}"` : '';
    const cls = warn === true ? ' warn' : '';
    return `<div class="row"><span class="k">${k}</span><span class="v${cls}"${style}>${v}</span></div>`;
  }).join('') + `</div>`;
}
const f = (v, d, unit) => (v === null || v === undefined) ? '-' : v.toFixed(d) + unit;
// WiFi 신호 등급별 색상 — "얼마가 좋은 숫자냐"는 질문에 답한 기준표 그대로 적용.
function wifiColor(dbm) {
  if (dbm >= -60) return 'var(--good)';       // 매우 좋음/좋음
  if (dbm >= -70) return 'var(--warning)';    // 양호/보통
  return 'var(--critical)';                    // 약함/매우 약함
}

async function tick() {
  // 화면이 안 보이면 폴링하지 않는다. 폰 화면을 끄거나 다른 앱으로 전환해도
  // 계속 돌면 CPU와 배터리를 낭비한다. 이 프로젝트에서 프리즈의 유력한
  // 방아쇠가 CPU 부하였다.
  if (document.hidden) return;
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
      const wifi = (d.latest && d.latest.wifi) || {};
      // -70dBm 아래는 통상 "약함"으로 보는 기준 — 음영지역 감시용
      const wifiRows = Object.values(wifi).map(w =>
        [`WiFi(${w.label})`, `${w.ssid || '-'} · ${w.level_dbm.toFixed(0)}dBm`, wifiColor(w.level_dbm)]);
      const body = `<div class="wide-body">
        ${rows([
          ['5V 입력', f(v, 2, 'V'), bad],
          ['CPU 온도', f(sys.cpu_temp_c, 1, '°C'), hot],
          ['CPU 사용률', f(sys.cpu_percent, 0, '%')],
          ['메모리', f(sys.mem_percent, 0, '%')],
          ...wifiRows,
        ])}
        <div class="chart-col">${sparkChart(d.history)}</div>
      </div>`;
      return card(mod, body, (bad || hot) ? 'var(--critical)' : 'var(--good)', false, true);
    }

    if (mod.key === 'motor') {
      const d = summary.motor;
      if (!d || !d.available) {
        return card(mod, `<div class="sub">대시보드 응답 없음</div>`, 'var(--idle)', true);
      }
      const connected = d.connected;
      const rate = d.rate || [0,0,0,0];
      const cmd = d.commanded || [0,0];
      const movingWheels = rate.filter(r => Math.abs(r) > 30).length;
      const moving = movingWheels > 0;
      const batt = d.battery || {};
      const battLabel = batt.connected && batt.voltage != null
        ? f(batt.voltage, 2, 'V') : '읽기 실패';
      return card(mod, rows([
        ['상태', moving ? '주행 중' : '정지'],
        ['배터리', battLabel, batt.connected === false],
        ['회전 중 바퀴', `${movingWheels} / 4`],
        ['명령 속도', `${f(cmd[0], 2, ' m/s')} / ${f(cmd[1], 2, ' rad/s')}`],
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
// 허브는 요약 화면이라 빠를 필요가 없다. 게다가 이 서버는 매 폴링마다
// 하위 3개 서비스의 /api/status를 **대신 조회**하므로, 1회 폴링이 내부 HTTP
// 요청 3건을 만든다. 1.5초로 두면 초당 6건이었다.
setInterval(tick, 4000);
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
        elif self.path == "/api/summary":
            poll_pi_history()
            out = {}
            for key, cfg in MODULES.items():
                data = fetch_json(cfg["url"])
                out[key] = {"available": data is not None, **(data or {})}
            with pi_history_lock:
                out["power"]["history"] = list(pi_history)
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