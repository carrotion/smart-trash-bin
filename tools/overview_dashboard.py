#!/usr/bin/env python3
"""스마트 쓰레기통 — PC용 한 화면 관제 (2026-10-10).

다섯 대시보드(허브·전원·모터·센서·지도)에 흩어진 정보를 한 화면에 모은다. 새로 모으는 것:
라이다 점(로봇 기준)·근접 레이더·가림 검사(tools/lidar_blockage.py와 같은 기준), 명령 속도 대 실제 속도 이력,
지도 위 로봇 위치, 자율주행 프로세스 생존, 서비스 상태, 엔코더 오독 누적, 최근 목표 결과, 최근 경고.

구성: ROS 노드 1개(스캔·오도메트리·명령·TF, 0.5초마다 요약) + 시스템 수집 스레드(다른 대시보드 API·systemctl·journal·기록 파일)
+ HTTP 서버. 화면은 /api/overview를 0.5초마다, 지도는 2초마다 받는다.
⚠️ 비상정지 버튼은 모터 대시보드의 /api/estop을 그대로 부른다(로직을 여기 복제하지 않는다).

    python3 tools/overview_dashboard.py [--port 8769]     # 서비스: smart-trash-overview
"""
import argparse, glob, gzip, json, math, os, re, subprocess, threading, time, urllib.request
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

import lidar_blockage
from dashboard_icon import FAVICON_TAG

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIDAR_X = 0.054
SERVICES = ['diff-drive', 'ekf', 'imu', 'imu-tf', 'lidar', 'motor-dashboard', 'sensor-dashboard', 'map-dashboard',
            'dashboard-hub', 'power-monitor', 'blackbox', 'disk-watchdog', 'map-autosave']
NAV_NODES = {'slam_toolbox': 'SLAM(지도)', 'planner_server': '경로 계획', 'collision_monitor': '충돌 감시',
             'frontier_explorer': '탐색기', 'step_navigator': '주행기', 'controller_server': '연속 주행(Nav2)'}

S = {'t0': time.time()}          # 수집한 최신 상태(스레드 공유) — 값은 통째로 바꿔 넣는다
LOCK = threading.Lock()


def put(key, val):
    with LOCK:
        S[key] = val


def fetch_json(url, timeout=1.0, data=None):
    try:
        req = urllib.request.Request(url, data=data, method='POST' if data is not None else 'GET')
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())
    except Exception:
        return None


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# ── ROS 수집 ────────────────────────────────────────────────────────────────
def ros_thread():
    rclpy.init()
    n = rclpy.create_node('overview_dashboard')
    R = {'scan': None, 'scan_t': 0.0, 'raw': deque(maxlen=5), 'raw_t': 0.0, 'scan_times': deque(maxlen=20),
         'odom': None, 'cmd': (0.0, 0.0, 0.0), 'nav': (0.0, 0.0, 0.0)}
    hist = deque(maxlen=300)     # 60초(5 Hz): t, v, w, cmd_v, cmd_w
    base = lidar_blockage.load_baseline()

    def on_scan(m):
        R['scan'] = m; R['scan_t'] = time.time(); R['scan_times'].append(R['scan_t'])
        if time.time() - R['raw_t'] > 1.0:      # /scan_raw가 없으면(스택 꺼짐) 서비스의 /scan이 곧 원시 스캔
            R['raw'].append(m)

    def on_raw(m):
        R['raw'].append(m); R['raw_t'] = time.time()

    def on_odom(m):
        p = m.pose.pose
        R['odom'] = (time.time(), m.twist.twist.linear.x, m.twist.twist.angular.z, p.position.x, p.position.y, yaw_of(p.orientation))
    n.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    n.create_subscription(LaserScan, '/scan_raw', on_raw, qos_profile_sensor_data)
    n.create_subscription(Odometry, '/odometry/filtered', on_odom, 10)
    n.create_subscription(Twist, '/cmd_vel', lambda m: R.update(cmd=(time.time(), m.linear.x, m.angular.z)), 10)
    n.create_subscription(Twist, '/cmd_vel_nav', lambda m: R.update(nav=(time.time(), m.linear.x, m.angular.z)), 10)
    tfb = Buffer(); TransformListener(tfb, n)

    def tick_fast():      # 5 Hz: 속도 이력
        o = R['odom']; now = time.time()
        if o and now - o[0] < 1.0:
            c = R['cmd']; hist.append((round(now, 2), round(o[1], 3), round(o[2], 3),
                                       round(c[1], 3) if now - c[0] < 0.6 else 0.0, round(c[2], 3) if now - c[0] < 0.6 else 0.0))

    def tick():           # 2 Hz: 라이다 요약·자세·주행
        now = time.time(); m = R['scan']; lid = dict(ok=False)
        if m is not None and now - R['scan_t'] < 2.0:
            pts = []; near = []; valid = 0
            for i, r in enumerate(m.ranges):
                if not (m.range_min <= r <= m.range_max):
                    continue
                valid += 1
                a = m.angle_min + i * m.angle_increment; x, y = LIDAR_X + r * math.cos(a), r * math.sin(a)
                if r < 1.6:
                    near.append((round(x, 3), round(y, 3)))
                if i % 2 == 0 and r < 9.0:
                    pts.append((round(x, 2), round(y, 2)))
            st = list(R['scan_times']); hz = (len(st) - 1) / (st[-1] - st[0]) if len(st) > 3 and st[-1] > st[0] else 0.0
            raws = list(R['raw'])
            nb = lidar_blockage.new_blocked(lidar_blockage.near_fraction(raws), base) if len(raws) >= 3 else []
            lid = dict(ok=True, pts=pts, near=near, valid=round(valid / max(len(m.ranges), 1), 3), hz=round(hz, 1),
                       blocked=[d for d, _ in nb], blocked_deg=len(nb) * lidar_blockage.BIN,
                       raw_src='scan_raw' if now - R['raw_t'] < 1.5 else 'scan')
        put('lidar', lid)
        pose = dict(map=None, odom=None)
        o = R['odom']
        if o and now - o[0] < 2.0:
            pose['odom'] = (round(o[3], 3), round(o[4], 3), round(o[5], 4))
        try:
            t = tfb.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
            st = tfb.lookup_transform('map', 'odom', rclpy.time.Time())
            age = n.get_clock().now().nanoseconds * 1e-9 - (st.header.stamp.sec + st.header.stamp.nanosec * 1e-9)
            if age < 3.0:
                pose['map'] = (round(t.translation.x, 3), round(t.translation.y, 3), round(yaw_of(t.rotation), 4))
        except Exception:
            pass
        put('pose', pose)
        c, nv = R['cmd'], R['nav']
        put('drive', dict(ok=bool(o and now - o[0] < 2.0), v=round(o[1], 3) if o else 0.0, w=round(o[2], 3) if o else 0.0,
                          cmd_v=c[1] if now - c[0] < 0.6 else 0.0, cmd_w=c[2] if now - c[0] < 0.6 else 0.0, cmd_age=round(now - c[0], 1) if c[0] else None,
                          nav_v=nv[1] if now - nv[0] < 0.6 else 0.0, nav_w=nv[2] if now - nv[0] < 0.6 else 0.0, nav_age=round(now - nv[0], 1) if nv[0] else None,
                          hist=list(hist)))

    def tick_slow():      # 3초: 자율주행 노드 생존
        try:
            names = set(n.get_node_names())
        except Exception:
            names = set()
        put('nav_nodes', {k: (k in names) for k in NAV_NODES})
    n.create_timer(0.2, tick_fast); n.create_timer(0.5, tick); n.create_timer(3.0, tick_slow)
    put('baseline', sorted(base))
    rclpy.spin(n)


# ── 시스템 수집 ─────────────────────────────────────────────────────────────
def recent_goals():
    """최근 목표 결과 6개: 오늘 주행기 서버 기록(jsonl)과 단발 주행 기록(json)에서."""
    out = []
    day = time.strftime('%Y%m%d')
    for f in sorted(glob.glob(os.path.join(REPO, 'docs/calibration', f'step_nav_server_{day}_*.jsonl')))[-2:]:
        try:
            for ln in open(f).read().splitlines()[-8:]:
                r = json.loads(ln); res = r['result']
                out.append(dict(t=r.get('t', 0), goal=r['goal'][:2], ok=res['ok'], msg=res['msg'][:40], secs=round(res.get('secs', 0), 1),
                                err_cm=round(res['final_err_m'] * 100, 1) if res.get('final_err_m') == res.get('final_err_m') else None,
                                src='sim' if f.endswith('_sim.jsonl') else '탐색'))
        except Exception:
            pass
    for f in sorted(glob.glob(os.path.join(REPO, 'docs/calibration', f'step_nav_{day}_*.json')))[-8:]:
        try:
            d = json.load(open(f)); res = d['result']
            out.append(dict(t=os.path.getmtime(f), goal=d['goal'][:2], ok=res['ok'], msg=res['msg'][:40], secs=round(res.get('secs', 0), 1),
                            err_cm=round(res['final_err_m'] * 100, 1) if res.get('final_err_m') == res.get('final_err_m') else None,
                            src=os.path.basename(f).split('_', 4)[-1].replace('.json', '')[:16] or '주행'))
        except Exception:
            pass
    return sorted(out, key=lambda r: r['t'])[-6:]


def sys_thread():
    k = 0; log = deque(maxlen=8)
    while True:
        t0 = time.time()
        put('motor', fetch_json('http://localhost:8766/api/status'))
        put('sensors', fetch_json('http://localhost:8767/api/status'))
        if k % 2 == 0:
            put('power', fetch_json('http://localhost:8765/api/status'))
            put('battery', fetch_json('http://localhost:8766/api/battery'))
        if k % 10 == 0:
            h = fetch_json('http://localhost:8766/api/battery/history', timeout=2.0)
            if h and h.get('rows'):
                put('batt_hist', [(round(r['ts']), round(r['v'], 3)) for r in h['rows']])
            put('goals', recent_goals())
        if k % 5 == 0:
            try:
                units = [f'smart-trash-{s}.service' for s in SERVICES]
                r = subprocess.run(['systemctl', 'is-active'] + units, capture_output=True, text=True, timeout=4).stdout.split()
                put('services', [dict(name=s, active=(r[i] == 'active') if i < len(r) else False) for i, s in enumerate(SERVICES)])
            except Exception:
                pass
            try:
                out = subprocess.run(['journalctl', '-u', 'smart-trash-diff-drive', '-u', 'smart-trash-imu', '-u', 'smart-trash-ekf',
                                      '--since', '-10 min', '--no-pager', '-o', 'short-unix', '-n', '80'],
                                     capture_output=True, text=True, timeout=4).stdout.splitlines()
                rej = None
                for ln in out:
                    m = re.search(r'누적 \[(\d+), (\d+), (\d+), (\d+)\]', ln)
                    if m:
                        rej = [int(x) for x in m.groups()]
                    lv = '[ERROR]' in ln or '[FATAL]' in ln
                    if (lv or '[WARN]' in ln) and '엔코더 값 버림' not in ln and 'Failed to meet update rate' not in ln:
                        ts = float(ln.split()[0]) if ln.split() and ln.split()[0].replace('.', '').isdigit() else time.time()
                        msg = re.sub(r'^.*\]: ', '', ln)[:110]
                        if not any(x['msg'] == msg for x in log):
                            log.append(dict(t=ts, level='error' if lv else 'warn', msg=msg))
                if rej:
                    put('enc_rejects', rej)
                put('log', list(log))
            except Exception:
                pass
        k += 1
        time.sleep(max(0.05, 1.0 - (time.time() - t0)))


# ── 요약 문장·경고 ──────────────────────────────────────────────────────────
def snapshot():
    with LOCK:
        d = dict(S)
    now = time.time(); alerts = []
    batt = d.get('battery') or {}; pw = (d.get('power') or {}).get('latest') or {}
    drv = d.get('drive') or {}; lid = d.get('lidar') or {}; nodes = d.get('nav_nodes') or {}
    tier = batt.get('tier', 'unknown')
    if tier in ('critical', 'warn2'):
        alerts.append(('danger', f'배터리 {batt.get("voltage", 0):.2f} V — {"종료 단계" if tier == "critical" else "구동 정지 단계"}'))
    elif tier == 'warn1':
        alerts.append(('warn', f'배터리 {batt.get("voltage", 0):.2f} V — 충전할 때'))
    if not batt.get('connected', False):
        alerts.append(('warn', '배터리 전압을 못 읽고 있음(모터 대시보드 응답 없음)'))
    th = (pw.get('throttled') or {}).get('flags') or {}
    if th.get('under_voltage_now') or th.get('throttled_now'):
        alerts.append(('danger', '파이 저전압/속도 제한 발생 중'))
    temp = (pw.get('system') or {}).get('cpu_temp_c')
    if temp and temp >= 75:
        alerts.append(('warn', f'파이 온도 {temp:.0f}°C'))
    for s in d.get('services') or []:
        if not s['active'] and s['name'] != 'lidar':
            alerts.append(('danger' if s['name'] in ('diff-drive', 'ekf', 'imu') else 'warn', f'서비스 꺼짐: {s["name"]}'))
    if not lid.get('ok'):
        alerts.append(('warn', '라이다 스캔이 안 들어옴'))
    elif lid.get('blocked_deg', 0) > lidar_blockage.MAX_NEW_DEG:
        b = lid['blocked']; alerts.append(('danger', f'라이다 가림 또는 바로 옆 물체: {b[0]}°~{b[-1]}° ({lid["blocked_deg"]}°)'))
    if not drv.get('ok'):
        alerts.append(('warn', '오도메트리(EKF)가 안 들어옴'))
    slam = nodes.get('slam_toolbox'); auto = nodes.get('collision_monitor') and (nodes.get('step_navigator') or nodes.get('controller_server'))
    moving = abs(drv.get('v', 0)) > 0.01 or abs(drv.get('w', 0)) > 0.05
    nav_cmd = abs(drv.get('nav_v', 0)) > 1e-3 or abs(drv.get('nav_w', 0)) > 1e-3
    man_cmd = (abs(drv.get('cmd_v', 0)) > 1e-3 or abs(drv.get('cmd_w', 0)) > 1e-3) and not nav_cmd
    if nav_cmd:
        act = '자율 주행 중' + (' (탐색)' if nodes.get('frontier_explorer') else '')
    elif man_cmd:
        act = '수동 조종 또는 시험 도구가 구동 중'
    elif moving:
        act = '명령 없이 움직이는 중(관성 또는 손으로 옮김)'
    elif auto:
        act = '자율 스택 켜짐, 목표 대기'
    elif slam:
        act = '지도 작성 스택만 켜짐, 정지'
    else:
        act = '대기 (자율 스택 꺼짐)'
    level = 'danger' if any(a[0] == 'danger' for a in alerts) else ('warn' if alerts else 'ok')
    d.update(now=now, alerts=[dict(level=a, msg=m) for a, m in alerts], state=dict(level=level, act=act),
             node_labels=NAV_NODES, uptime=now - d['t0'])
    return d


PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>스마트 쓰레기통 관제</title>__FAVICON__
<style>
:root{--film:#E6EBEF;--sheet:#F7F9FA;--ink:#17232F;--mute:#5F6E7B;--rule:#C9D2D9;--line:#1E5FA6;--ok:#17855F;--warn:#B47A00;--bad:#C2362B;--ghost:#A9B5BF}
*{box-sizing:border-box}
html,body{margin:0;height:100%;background:var(--film);color:var(--ink);font:14px/1.4 "Pretendard","Noto Sans KR","Apple SD Gothic Neo","Malgun Gothic",system-ui,sans-serif;font-variant-numeric:tabular-nums}
#app{display:grid;height:100vh;gap:10px;padding:10px;grid-template-columns:minmax(0,1.5fr) minmax(0,1fr) minmax(0,1.05fr);grid-template-rows:auto minmax(0,1.35fr) minmax(0,1fr)}
header{grid-column:1/-1;display:flex;align-items:stretch;gap:14px;background:var(--sheet);border-left:8px solid var(--ok);padding:10px 14px}
header.warn{border-left-color:var(--warn)} header.danger{border-left-color:var(--bad)}
#act{font-size:26px;font-weight:700;letter-spacing:-.01em;line-height:1.15}
#sub{color:var(--mute);margin-top:2px}
.say{flex:1;min-width:0}
.gauges{display:flex;gap:18px;align-items:center}
.g b{display:block;font-size:22px;font-weight:700;line-height:1.1} .g span{color:var(--mute);font-size:12px}
.g.warn b{color:var(--warn)} .g.danger b{color:var(--bad)}
#estop{border:0;background:var(--bad);color:#fff;font:700 17px/1 inherit;padding:0 22px;cursor:pointer;border-radius:4px}
#estop:hover{filter:brightness(1.1)} #estop:focus-visible{outline:3px solid var(--ink);outline-offset:2px}
section{background:var(--sheet);padding:10px 12px;min-height:0;min-width:0;display:flex;flex-direction:column;position:relative}
h2{margin:0 0 6px;font-size:14px;font-weight:700;display:flex;justify-content:space-between;gap:8px;align-items:baseline}
h2 small{font-weight:400;color:var(--mute);font-size:12px}
canvas{display:block;width:100%;flex:1;min-height:0}
#map{grid-row:2/4}
.cols{display:grid;grid-template-columns:1fr 1fr;gap:4px 14px}
.kv{display:flex;justify-content:space-between;gap:8px;border-bottom:1px solid var(--rule);padding:2px 0}
.kv span{color:var(--mute)} .kv b{font-weight:600}
.dots{display:flex;flex-wrap:wrap;gap:4px 12px;margin:2px 0 6px}
.dot{display:inline-flex;align-items:center;gap:5px;font-size:13px}
.dot i{width:9px;height:9px;border-radius:50%;background:var(--ghost);display:inline-block}
.dot.on i{background:var(--ok)} .dot.off.need i{background:var(--bad)}
.wheels{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin:4px 0 2px}
.wh{border:1px solid var(--rule);padding:3px 6px;font-size:12px} .wh b{display:block;font-size:15px}
.wh .bar{height:5px;background:var(--rule);margin-top:3px;position:relative} .wh .bar i{position:absolute;top:0;bottom:0;background:var(--line)}
.wh em{font-style:normal;color:var(--mute)} .wh em.bad{color:var(--bad)}
table{border-collapse:collapse;width:100%;font-size:13px} td{padding:2px 4px;border-bottom:1px solid var(--rule);white-space:nowrap;overflow:hidden;text-overflow:ellipsis;max-width:0}
td.ok{color:var(--ok);font-weight:700;width:1.4em} td.no{color:var(--bad);font-weight:700;width:1.4em} td.r{text-align:right;width:4.2em} td.t{width:3.6em;color:var(--mute)}
ul{margin:0;padding:0;list-style:none;font-size:13px;overflow:auto}
li{padding:2px 0 2px 10px;border-left:3px solid var(--rule);margin-bottom:3px} li.warn{border-left-color:var(--warn)} li.danger,li.error{border-left-color:var(--bad)}
li time{color:var(--mute);margin-right:6px}
.stale{opacity:.45}
.legend{position:absolute;right:12px;bottom:8px;font-size:12px;color:var(--mute);background:rgba(247,249,250,.85);padding:2px 6px}
.split{display:grid;grid-template-rows:auto minmax(0,1fr) auto minmax(0,.8fr);min-height:0;flex:1}
@media (max-width:1100px){#app{height:auto;grid-template-columns:1fr;grid-template-rows:none} #map{grid-row:auto} section{min-height:320px}}
@media (prefers-reduced-motion:reduce){*{transition:none!important}}
</style></head><body>
<div id="app">
<header id="hdr">
  <div class="say"><div id="act">연결 중…</div><div id="sub"></div></div>
  <div class="gauges">
    <div class="g" id="gBatt"><b>–</b><span>배터리</span></div>
    <div class="g" id="gTemp"><b>–</b><span>파이 온도</span></div>
    <div class="g" id="gCpu"><b>–</b><span>CPU / 메모리</span></div>
    <div class="g" id="gWifi"><b>–</b><span>WiFi</span></div>
    <div class="g" id="gClock"><b>–</b><span>켜진 지</span></div>
  </div>
  <button id="estop" title="자율주행·시험 도구를 모두 종료하고 정지 명령을 보냅니다">비상정지</button>
</header>

<section id="map"><h2><span id="mapTitle">지도와 로봇 위치</span><small id="mapInfo"></small></h2><canvas id="cMap"></canvas>
  <div class="legend">파란 점 = 지금 라이다가 보는 것 · 화살표 = 로봇 · 한 칸 1 m</div></section>

<section><h2>근접 레이더 <small id="radInfo"></small></h2><canvas id="cRad"></canvas>
  <div class="legend">실선 원 22 cm = 회전에 필요한 반경 · 점선 원 = 라이다가 못 읽는 13.5 cm</div></section>

<section><h2>주행 <small id="drvInfo"></small></h2>
  <div class="split">
    <div><div class="cols">
      <div class="kv"><span>실제 속도</span><b id="vAct">–</b></div><div class="kv"><span>명령 속도</span><b id="vCmd">–</b></div>
      <div class="kv"><span>실제 회전</span><b id="wAct">–</b></div><div class="kv"><span>명령 회전</span><b id="wCmd">–</b></div>
    </div></div>
    <canvas id="cSpd"></canvas>
    <div><div class="wheels" id="wheels"></div><div class="cols">
      <div class="kv"><span>방위(자기)</span><b id="hdg">–</b></div><div class="kv"><span>자이로 z</span><b id="gz">–</b></div></div></div>
    <canvas id="cBat"></canvas>
  </div></section>

<section><h2>자율주행과 서비스 <small id="navInfo"></small></h2>
  <div class="dots" id="navDots"></div>
  <div class="dots" id="svcDots"></div>
  <div class="cols" id="piKv"></div></section>

<section><h2>최근 목표와 경고 <small id="logInfo"></small></h2>
  <table id="goals"></table>
  <ul id="alerts" style="margin-top:8px;flex:1"></ul></section>
</div>
<script>
const $=id=>document.getElementById(id), css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const C={line:css('--line'),ink:css('--ink'),mute:css('--mute'),rule:css('--rule'),ok:css('--ok'),warn:css('--warn'),bad:css('--bad'),ghost:css('--ghost'),sheet:css('--sheet')};
let D=null, MAP=null, mapImg=null, mapTs=0;
function fit(cv){const r=cv.getBoundingClientRect(),d=devicePixelRatio||1;if(cv.width!==Math.round(r.width*d)||cv.height!==Math.round(r.height*d)){cv.width=Math.round(r.width*d);cv.height=Math.round(r.height*d)}const g=cv.getContext('2d');g.setTransform(d,0,0,d,0,0);g.clearRect(0,0,r.width,r.height);return [g,r.width,r.height]}
const f=(v,n=2)=>v==null||isNaN(v)?'–':(+v).toFixed(n), sgn=(v,n=2)=>v==null?'–':(v>=0?'+':'')+(+v).toFixed(n);
function dur(s){s=Math.max(0,s|0);const h=s/3600|0,m=(s%3600)/60|0;return h?`${h}시간 ${m}분`:`${m}분`}
function dur2(s){return s<90?f(s,0)+'초':dur(s)}
function hhmm(t){const d=new Date(t*1000);return String(d.getHours()).padStart(2,'0')+':'+String(d.getMinutes()).padStart(2,'0')}

function robot(g,sc){ // 로봇 몸체(길이 19 × 폭 21 cm)와 뒤 케이블 영역 — 로봇 좌표: x 앞(위), y 왼쪽
  g.fillStyle='rgba(23,35,47,.10)';g.strokeStyle=C.ink;g.lineWidth=1.5;
  g.fillRect(-0.105*sc,-0.095*sc,0.21*sc,0.19*sc);g.strokeRect(-0.105*sc,-0.095*sc,0.21*sc,0.19*sc);
  g.setLineDash([3,3]);g.strokeStyle=C.mute;g.lineWidth=1;g.strokeRect(-0.06*sc,0.095*sc,0.12*sc,0.09*sc);g.setLineDash([]);
  g.beginPath();g.moveTo(0,-0.15*sc);g.lineTo(-0.035*sc,-0.085*sc);g.lineTo(0.035*sc,-0.085*sc);g.closePath();g.fillStyle=C.ink;g.fill();
}
function drawRadar(){
  const [g,W,H]=fit($('cRad')); const L=D.lidar||{}; const R=1.25, sc=Math.min(W,H)/2/R;
  g.save();g.translate(W/2,H/2+H*0.04);
  // 눈금 원(0.5, 1.0 m)
  g.strokeStyle=C.rule;g.lineWidth=1;g.fillStyle=C.mute;g.font='11px sans-serif';
  for(const r of [0.5,1.0]){g.beginPath();g.arc(0,0,r*sc,0,7);g.stroke();g.fillText(r.toFixed(1)+' m',4,-r*sc-3)}
  // 자기 구조물 방향(기준)과 새로 가려진 방향 — 라이다 위치 기준 2° 쐐기
  const lx=0, ly=-0.054*sc;
  const wedge=(deg,col,len)=>{const a0=(deg)*Math.PI/180,a1=(deg+2)*Math.PI/180;g.beginPath();g.moveTo(lx,ly);
    g.lineTo(lx-Math.sin(a0)*len,ly-Math.cos(a0)*len);g.lineTo(lx-Math.sin(a1)*len,ly-Math.cos(a1)*len);g.closePath();g.fillStyle=col;g.fill()};
  for(const d of (D.baseline||[])) wedge(d,'rgba(95,110,123,.13)',0.30*sc);
  for(const d of (L.blocked||[])) wedge(d,'rgba(194,54,43,.38)',R*sc);
  // 회전 반경 22 cm, 라이다 근접 한계 13.5 cm
  g.strokeStyle=C.line;g.lineWidth=1.5;g.beginPath();g.arc(0,0,0.22*sc,0,7);g.stroke();
  g.setLineDash([4,4]);g.strokeStyle=C.mute;g.lineWidth=1;g.beginPath();g.arc(lx,ly,0.135*sc,0,7);g.stroke();g.setLineDash([]);
  robot(g,sc);
  // 점: 회전 반경 안이면 빨강
  for(const p of (L.near||[])){const x=p[0],y=p[1];const inb=Math.abs(x)<=0.115&&Math.abs(y)<=0.115;if(inb)continue;
    const d=Math.hypot(x,y);if(d>R*1.2)continue;g.fillStyle=d<0.22?C.bad:C.line;g.fillRect(-y*sc-1.5,-x*sc-1.5,3,3)}
  g.restore();
  let nearest=null;for(const p of (L.near||[])){if(Math.abs(p[0])<=0.115&&Math.abs(p[1])<=0.115)continue;const d=Math.hypot(p[0],p[1]);if(nearest==null||d<nearest)nearest=d}
  $('radInfo').textContent=L.ok?`가장 가까운 물체 ${nearest==null?'없음':f(nearest*100,0)+' cm'} · 유효 ${f(L.valid*100,0)}% · ${f(L.hz,1)} Hz`:'스캔 없음';
  $('cRad').parentElement.classList.toggle('stale',!L.ok);
}
function b64grid(b){const s=atob(b),a=new Int8Array(s.length);for(let i=0;i<s.length;i++)a[i]=s.charCodeAt(i);return a}
function drawMap(){
  const [g,W,H]=fit($('cMap')); const L=D.lidar||{}, P=D.pose||{}; const haveMap=MAP&&MAP.connected&&MAP.data_b64&&P.map;
  if(haveMap){
    if(MAP.last_ts!==mapTs){const a=b64grid(MAP.data_b64),w=MAP.width,h=MAP.height;const oc=document.createElement('canvas');oc.width=w;oc.height=h;
      const og=oc.getContext('2d'),im=og.createImageData(w,h);
      for(let j=0;j<h;j++)for(let i=0;i<w;i++){const v=a[j*w+i],k=((h-1-j)*w+i)*4;const c=v<0?[214,221,226]:(v>=65?[23,35,47]:[247,249,250]);im.data[k]=c[0];im.data[k+1]=c[1];im.data[k+2]=c[2];im.data[k+3]=255}
      og.putImageData(im,0,0);mapImg=oc;mapTs=MAP.last_ts}
    const mw=MAP.width*MAP.resolution,mh=MAP.height*MAP.resolution,sc=Math.min((W-16)/mw,(H-16)/mh),ox=(W-mw*sc)/2,oy=(H-mh*sc)/2;
    g.imageSmoothingEnabled=false;g.drawImage(mapImg,ox,oy,mw*sc,mh*sc);
    const X=x=>ox+(x-MAP.origin_x)*sc, Y=y=>oy+(mh-(y-MAP.origin_y))*sc;
    g.strokeStyle='rgba(30,95,166,.18)';g.lineWidth=1;
    for(let x=Math.ceil(MAP.origin_x);x<MAP.origin_x+mw;x++){g.beginPath();g.moveTo(X(x),oy);g.lineTo(X(x),oy+mh*sc);g.stroke()}
    for(let y=Math.ceil(MAP.origin_y);y<MAP.origin_y+mh;y++){g.beginPath();g.moveTo(ox,Y(y));g.lineTo(ox+mw*sc,Y(y));g.stroke()}
    const [rx,ry,th]=P.map,c=Math.cos(th),s=Math.sin(th);
    g.fillStyle=C.line;for(const p of (L.pts||[])){g.fillRect(X(rx+c*p[0]-s*p[1])-1,Y(ry+s*p[0]+c*p[1])-1,2,2)}
    g.save();g.translate(X(rx),Y(ry));g.rotate(-th+Math.PI/2);const kr=Math.max(1,14/(0.21*sc));g.scale(kr,kr);robot(g,sc);g.restore();
    g.fillStyle=C.ok;g.beginPath();g.arc(X(0),Y(0),4,0,7);g.fill();
    $('mapTitle').textContent='지도와 로봇 위치';
    $('mapInfo').textContent=`로봇 (${sgn(rx)}, ${sgn(ry)}) m, ${f(th*180/Math.PI,0)}° · 지도 ${f(mw,1)}×${f(mh,1)} m · 초록 점 = 출발점`;
  }else{
    // 지도가 없으면(SLAM 꺼짐) 로봇 중심으로 주변 스캔만
    const R=6.5,sc=Math.min(W,H)/2/R;g.save();g.translate(W/2,H/2);
    g.strokeStyle='rgba(30,95,166,.18)';g.lineWidth=1;g.fillStyle=C.mute;g.font='11px sans-serif';
    for(let r=1;r<=6;r++){g.beginPath();g.arc(0,0,r*sc,0,7);g.stroke();if(r%2==0)g.fillText(r+' m',4,-r*sc-3)}
    g.fillStyle=C.line;for(const p of (L.pts||[])){g.fillRect(-p[1]*sc-1,-p[0]*sc-1,2,2)}
    g.save();const k=Math.max(1,12/(0.21*sc));g.scale(k,k);robot(g,sc);g.restore();g.restore();
    $('mapTitle').textContent='주변 스캔 (지도 없음)';
    $('mapInfo').textContent='SLAM이 꺼져 있어 지도가 없습니다. 시험 스택이나 자율 매핑을 켜면 지도 위에 표시됩니다';
  }
  $('cMap').parentElement.classList.toggle('stale',!L.ok);
}
function chart(cv,series,opt){ // series: [{pts:[[t,v]],col,dash}], opt:{min,max,lines:[{v,col,label}],unit,span}
  const [g,W,H]=fit(cv),pl=34,pr=6,pt=8,pb=16; const t1=opt.t1,t0=t1-opt.span;
  const X=t=>pl+(t-t0)/(t1-t0)*(W-pl-pr), Y=v=>pt+(opt.max-v)/(opt.max-opt.min)*(H-pt-pb);
  g.font='11px sans-serif';g.fillStyle=C.mute;g.strokeStyle=C.rule;g.lineWidth=1;
  for(const v of opt.ticks){g.beginPath();g.moveTo(pl,Y(v));g.lineTo(W-pr,Y(v));g.stroke();g.fillText(v.toFixed(opt.dec),2,Y(v)+4)}
  for(const l of (opt.lines||[])){g.strokeStyle=l.col;g.setLineDash([5,4]);g.beginPath();g.moveTo(pl,Y(l.v));g.lineTo(W-pr,Y(l.v));g.stroke();g.setLineDash([]);g.fillStyle=l.col;g.fillText(l.label,W-pr-g.measureText(l.label).width-2,Y(l.v)-3)}
  for(const s of series){g.strokeStyle=s.col;g.lineWidth=s.w||1.6;g.setLineDash(s.dash||[]);g.beginPath();let on=false;
    for(const p of s.pts){if(p[0]<t0)continue;const x=X(p[0]),y=Y(Math.max(opt.min,Math.min(opt.max,p[1])));on?g.lineTo(x,y):g.moveTo(x,y);on=true}g.stroke();g.setLineDash([])}
  g.fillStyle=C.mute;g.fillText(opt.label,pl+2,H-3);
}
function render(){
  const s=D.state, B=D.battery||{}, pw=((D.power||{}).latest)||{}, sys=pw.system||{}, drv=D.drive||{}, M=D.motor||{}, SE=D.sensors||{};
  $('hdr').className=s.level==='ok'?'':s.level; $('act').textContent=s.act;
  const al=D.alerts||[]; $('sub').textContent=al.length?al.map(a=>a.msg).join(' / '):'이상 없음';
  const tier=B.tier||'unknown', gb=$('gBatt'); gb.className='g'+(tier==='ok'?'':(tier==='warn1'?' warn':' danger'));
  gb.querySelector('b').textContent=B.voltage!=null?f(B.voltage,2)+' V':'–';
  gb.querySelector('span').textContent='배터리'+(B.thresholds?` (주의 ${B.thresholds.warn1} / 정지 ${B.thresholds.warn2} / 종료 ${B.thresholds.critical})`:'');
  $('gTemp').querySelector('b').textContent=sys.cpu_temp_c!=null?f(sys.cpu_temp_c,0)+'°C':'–';
  $('gCpu').querySelector('b').textContent=sys.cpu_percent!=null?f(sys.cpu_percent,0)+'% / '+f(sys.mem_percent,0)+'%':'–';
  const wf=Object.values(pw.wifi||{})[0]; $('gWifi').querySelector('b').textContent=wf&&wf.level_dbm!=null?f(wf.level_dbm,0)+' dBm':'–';
  $('gClock').querySelector('b').textContent=sys.uptime_s!=null?dur(sys.uptime_s):'–';
  // 주행
  $('vAct').textContent=f(drv.v,3)+' m/s'; $('vCmd').textContent=f(drv.cmd_v,3)+' m/s'; $('wAct').textContent=sgn(drv.w,2)+' rad/s'; $('wCmd').textContent=sgn(drv.cmd_w,2)+' rad/s';
  $('drvInfo').textContent=drv.ok?(Math.abs(drv.nav_v)>1e-3||Math.abs(drv.nav_w)>1e-3?'자율 명령이 들어오는 중':(drv.cmd_age==null?'이 화면을 켠 뒤 명령 없음':'마지막 명령 '+dur2(drv.cmd_age)+' 전')):'오도메트리 없음';
  const h=drv.hist||[], t1=D.now;
  chart($('cSpd'),[{pts:h.map(r=>[r[0],r[3]]),col:C.mute,dash:[4,3]},{pts:h.map(r=>[r[0],r[1]]),col:C.line,w:2}],
    {t1,span:60,min:-0.1,max:0.3,ticks:[0,0.15,0.3],dec:2,label:'속도 m/s · 최근 60초 (실선 실제, 점선 명령)',lines:[{v:0.15,col:C.ghost,label:'주행기 순항 0.15'}]});
  const rej=D.enc_rejects, names=['M1 왼앞','M2 오앞','M3 왼뒤','M4 오뒤'], rate=M.rate||[0,0,0,0], enc=M.encoder||[];
  $('wheels').innerHTML=names.map((n,i)=>{const r=rate[i]||0,w=Math.min(50,Math.abs(r)/60),left=r>=0?50:50-w;
    return `<div class="wh">${n}<b>${f(r,0)}</b><div class="bar"><i style="left:${left}%;width:${w}%"></i></div><em class="${rej&&rej[i]>0?'bad':''}">오독 ${rej?rej[i]:'–'}</em></div>`}).join('');
  const mag=SE.mag||{}, imu=SE.imu||{}; $('hdg').textContent=mag.heading_deg!=null?f(mag.heading_deg,0)+'°'+(mag.usable?'':' (불안정)'):'–';
  $('gz').textContent=imu.angular_velocity?sgn(imu.angular_velocity.z,3)+' rad/s':'–';
  const bh=D.batt_hist||[], th=B.thresholds||{warn1:11.3,warn2:11.0,critical:10.7};
  chart($('cBat'),[{pts:bh,col:C.line,w:2}],{t1,span:Math.max(600,bh.length?t1-bh[0][0]:600),min:10.5,max:12.7,ticks:[11,12],dec:0,
    label:'배터리 V · 최근 '+(bh.length?dur(t1-bh[0][0]):'–'),lines:[{v:th.warn1,col:C.warn,label:'주의'},{v:th.warn2,col:C.bad,label:'정지'}]});
  // 자율주행·서비스
  const nn=D.nav_nodes||{}, lab=D.node_labels||{};
  $('navDots').innerHTML=Object.keys(lab).map(k=>`<span class="dot ${nn[k]?'on':'off'}"><i></i>${lab[k]}</span>`).join('');
  $('svcDots').innerHTML=(D.services||[]).map(x=>`<span class="dot ${x.active?'on':'off need'}"><i></i>${x.name}</span>`).join('');
  const on=Object.values(nn).filter(Boolean).length; $('navInfo').textContent=on?`자율 프로세스 ${on}개 켜짐`:'자율 프로세스 꺼짐';
  const rails=pw.rails||{}, v5=(rails.EXT5V_V||{}).value, flags=((pw.throttled||{}).flags)||{};
  const kv=[['파이 5 V 입력',v5!=null?f(v5,2)+' V':'–'],['저전압 이력',flags.under_voltage_occurred?'있음':'없음'],['디스크',sys.disk_percent!=null?f(sys.disk_percent,0)+'%':'–'],
    ['라이다 원시 스캔 출처',(D.lidar||{}).raw_src||'–'],['지도 위 위치',(D.pose||{}).map?'잡힘':'없음'],['이 화면 켜진 지',dur(D.uptime)]];
  $('piKv').innerHTML=kv.map(r=>`<div class="kv"><span>${r[0]}</span><b>${r[1]}</b></div>`).join('');
  // 목표·경고
  const gs=(D.goals||[]).slice().reverse();
  $('goals').innerHTML=gs.length?gs.map(r=>`<tr><td class="t">${hhmm(r.t)}</td><td class="${r.ok?'ok':'no'}">${r.ok?'○':'×'}</td><td>${r.msg} <span style="color:var(--mute)">(${sgn(r.goal[0],1)}, ${sgn(r.goal[1],1)}) ${r.src}</span></td><td class="r">${r.err_cm==null?'–':f(r.err_cm,1)+' cm'}</td><td class="r">${f(r.secs,0)} s</td></tr>`).join('')
    :'<tr><td style="color:var(--mute)">오늘 기록된 목표가 없습니다</td></tr>';
  const lg=(D.log||[]).slice().reverse();
  $('alerts').innerHTML=al.map(a=>`<li class="${a.level}">${a.msg}</li>`).join('')+lg.map(a=>`<li class="${a.level}"><time>${hhmm(a.t)}</time>${a.msg.replace(/</g,'&lt;')}</li>`).join('')||'<li>최근 10분 동안 구동·IMU·EKF 경고가 없습니다</li>';
  $('logInfo').textContent=`목표 ${gs.length}건 · 경고 ${al.length+lg.length}건`;
  drawRadar(); drawMap();
}
async function poll(){try{D=await (await fetch('/api/overview')).json();render()}catch(e){$('act').textContent='관제 서버와 연결이 끊겼습니다';$('sub').textContent='파이 전원과 WiFi를 확인하세요';$('hdr').className='danger'}}
async function pollMap(){try{const m=await (await fetch('/api/map')).json();if(m&&m.connected)MAP=m;else MAP=null}catch(e){}}
$('estop').onclick=async()=>{$('estop').textContent='정지 보내는 중…';try{const r=await (await fetch('/api/estop',{method:'POST'})).json();$('estop').textContent=r&&r.ok?`정지함 (${(r.killed||[]).length}개 종료)`:'실패 — 모터 대시보드 확인'}catch(e){$('estop').textContent='실패 — 연결 확인'}setTimeout(()=>$('estop').textContent='비상정지',4000)};
poll();pollMap();setInterval(poll,500);setInterval(pollMap,2000);addEventListener('resize',()=>D&&render());
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        enc = None
        if len(body) > 1024 and 'gzip' in (self.headers.get('Accept-Encoding') or ''):
            packed = gzip.compress(body, 5)
            if len(packed) < len(body):
                body, enc = packed, 'gzip'
        self.send_response(code); self.send_header('Content-Type', ctype)
        if enc:
            self.send_header('Content-Encoding', enc)
        self.send_header('Content-Length', str(len(body))); self.send_header('Cache-Control', 'no-store'); self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self._send(200, PAGE.replace('__FAVICON__', FAVICON_TAG).encode('utf-8'), 'text/html; charset=utf-8')
        elif self.path == '/api/overview':
            self._send(200, json.dumps(snapshot(), ensure_ascii=False).encode('utf-8'), 'application/json')
        elif self.path == '/api/map':
            d = fetch_json('http://localhost:8768/api/map', timeout=2.0) or {'connected': False}
            if d.get('stale'):
                d = {'connected': False}
            self._send(200, json.dumps(d).encode('utf-8'), 'application/json')
        else:
            self._send(404, b'not found', 'text/plain')

    def do_POST(self):
        if self.path == '/api/estop':
            d = fetch_json('http://localhost:8766/api/estop', timeout=5.0, data=b'') or {'ok': False}
            self._send(200, json.dumps(d).encode('utf-8'), 'application/json')
        else:
            self._send(404, b'not found', 'text/plain')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8769)
    a = ap.parse_args()
    threading.Thread(target=ros_thread, daemon=True).start()
    threading.Thread(target=sys_thread, daemon=True).start()
    print(f'관제 화면: http://<Pi IP>:{a.port}', flush=True)
    ThreadingHTTPServer(('0.0.0.0', a.port), Handler).serve_forever()


if __name__ == '__main__':
    main()
