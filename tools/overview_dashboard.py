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
import argparse, glob, gzip, json, math, os, re, signal, subprocess, threading, time, urllib.request
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
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


def is_self(x, y):
    """로봇 자신에게서 나온 점인가: 차체(±0.115) 또는 뒤로 나온 케이블 영역."""
    return (abs(x) <= 0.115 and abs(y) <= 0.115) or (-0.19 <= x <= -0.09 and abs(y) <= 0.065)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# ── ROS 수집 ────────────────────────────────────────────────────────────────
def ros_thread():
    # 종료 신호는 main이 받는다 — rclpy가 가로채면 HTTP 서버가 남아 systemd가 90초 뒤 강제 종료했다(10-10)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
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
    # 금지 구역: 구역 서버가 실제로 적용한 것(파일이 아니라) — 지도에 겹쳐 그리고 개수를 자율주행 칸에 보인다
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    KZ = {'zones': [], 'version': None}

    def on_zones(m):
        try:
            d = json.loads(m.data); KZ.update(zones=[z['poly'] for z in d.get('zones', [])], version=d.get('version'))
        except (ValueError, KeyError, TypeError):
            pass
    n.create_subscription(String, '/keepout_zones', on_zones,
                          QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
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
        server = 'keepout_server' in names
        put('keepout', dict(server=server, zones=KZ['zones'] if server else [], version=KZ['version']))
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


def last_enc_rejects():
    try:
        out = subprocess.run(['journalctl', '-u', 'smart-trash-diff-drive', '-b', '--no-pager', '-g', '누적 \\[', '-n', '1'],
                             capture_output=True, text=True, timeout=10).stdout
        m = re.findall(r'누적 \[(\d+), (\d+), (\d+), (\d+)\]', out)
        return [int(x) for x in m[-1]] if m else None
    except Exception:
        return None


def sys_thread():
    k = 0; log = deque(maxlen=8)
    rej0 = last_enc_rejects()
    if rej0:
        put('enc_rejects', rej0)
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
                    with LOCK:
                        prev = S.get('enc_rejects')
                    if prev is not None and prev != rej:      # 처음 읽은 값은 '늘었다'로 치지 않는다
                        put('enc_rejects_changed', time.time())
                    put('enc_rejects', rej)
                put('log', list(log))
            except Exception:
                pass
        k += 1
        time.sleep(max(0.05, 1.0 - (time.time() - t0)))


# ── 요약 문장·부위별 점검 ───────────────────────────────────────────────────
PART_OWNER = {'diff-drive': '구동', 'ekf': '위치 추정', 'imu': '위치 추정', 'imu-tf': '위치 추정'}   # 그 밖의 서비스는 '파이'


def snapshot():
    """화면에 보낼 한 덩어리. 부위 6곳(전원·구동·라이다·위치 추정·자율주행·파이)마다 판정과 핵심 수치를 만들고,
    맨 위 문장의 '이유'는 그 판정에서 나온 문제만 모은다(같은 사실을 두 군데서 따로 판단하지 않는다)."""
    with LOCK:
        d = dict(S)
    now = time.time(); parts = []; alerts = []

    def part(name, value, detail, problems=(), notes=(), off=False):
        # problems: 맨 위 문장까지 올라가는 문제. notes: 알려진 만성 문제 — 칸만 '주의'로 바꾸고 맨 위는 건드리지 않는다.
        level = 'danger' if any(p[0] == 'danger' for p in problems) else ('warn' if (problems or notes) else ('off' if off else 'ok'))
        parts.append(dict(name=name, level=level, value=value, detail=' · '.join(x for x in list(detail) + list(notes) if x)))
        alerts.extend(problems)

    batt = d.get('battery') or {}; pw = (d.get('power') or {}).get('latest') or {}; sysm = pw.get('system') or {}
    drv = d.get('drive') or {}; lid = d.get('lidar') or {}; nodes = d.get('nav_nodes') or {}
    motor = d.get('motor') or {}; sens = d.get('sensors') or {}
    svc = d.get('services') or []
    off_by = {}
    for s in svc:
        if not s['active'] and s['name'] != 'lidar':        # lidar 서비스는 시험 스택이 자기 드라이버를 띄울 때 꺼진다 — 스캔 유무로 판단
            off_by.setdefault(PART_OWNER.get(s['name'], '파이'), []).append(s['name'])

    # 전원
    pr = []; tier = batt.get('tier', 'unknown'); v = batt.get('voltage') or 0.0; th = batt.get('thresholds') or {}
    if tier in ('critical', 'warn2'):
        pr.append(('danger', f'배터리 {v:.2f} V — {"종료 단계" if tier == "critical" else "구동 정지 단계"}'))
    elif tier == 'warn1':
        pr.append(('warn', f'배터리 {v:.2f} V — 충전할 때'))
    if not batt.get('connected', False):
        pr.append(('warn', '배터리 전압을 못 읽고 있음(모터 대시보드 응답 없음)'))
    flags = (pw.get('throttled') or {}).get('flags') or {}
    if flags.get('under_voltage_now') or flags.get('throttled_now'):
        pr.append(('danger', '파이 저전압/속도 제한 발생 중'))
    v5 = ((pw.get('rails') or {}).get('EXT5V_V') or {}).get('value')
    part('전원', f'{v:.2f} V' if batt.get('connected') else '읽지 못함',
         [f'주의 {th.get("warn1", 11.3)} · 정지 {th.get("warn2", 11.0)} · 종료 {th.get("critical", 10.7)} V', f'파이 입력 {v5:.2f} V' if v5 else ''],
         pr, ['부팅 뒤 저전압 이력 있음'] if flags.get('under_voltage_occurred') else [])

    # 구동
    pr = [('danger', f'서비스 꺼짐: {n}') for n in off_by.get('구동', [])]
    if not motor.get('connected', False):
        pr.append(('warn', '모터 보드 상태를 못 읽고 있음'))
    enc_age = motor.get('enc_age')
    if enc_age is not None and enc_age > 1.0:
        # ⚠️ 11-07 사고: 보드가 멈춘 2분 동안 이 화면은 마지막 바퀴 값을 실시간처럼 보여 줬고, 비상정지는 바퀴를 못 세웠다
        pr.append(('danger', f'모터 보드 응답 없음 {enc_age:.0f}초 — 바퀴 값이 멈춰 있음, 정지 명령이 안 먹을 수 있다'))
    rej = d.get('enc_rejects'); notes = []
    if rej and sum(rej):
        bad = ' · '.join(f'M{i + 1} {n:,}' for i, n in enumerate(rej) if n)
        growing = now - d.get('enc_rejects_changed', 0) < 300
        notes.append(f'엔코더 오독 {"늘어나는 중" if growing else "누적"}: {bad}')
    moving = abs(drv.get('v', 0)) > 0.01 or abs(drv.get('w', 0)) > 0.05
    enc_dead = enc_age is not None and enc_age > 1.0
    value = '모름(보드 무응답)' if enc_dead else ('정지' if not moving else (f'{drv.get("v", 0):+.2f} m/s' if abs(drv.get('v', 0)) > 0.01 else '제자리 회전'))
    part('구동', value, ['' if enc_dead else ('모터 보드 응답 정상' if motor.get('connected') else ''), '엔코더 오독 없음' if rej and not sum(rej) else ''], pr, notes)

    # 라이다
    pr = []; near_cm = None
    if lid.get('ok'):
        ds = [math.hypot(x, y) for x, y in lid.get('near', []) if not is_self(x, y)]
        near_cm = round(min(ds) * 100) if ds else None
        if lid.get('blocked_deg', 0) > lidar_blockage.MAX_NEW_DEG:
            b = lid['blocked']; pr.append(('danger', f'라이다 가림 또는 바로 옆 물체: {b[0]}°~{b[-1]}° ({lid["blocked_deg"]}°)'))
        part('라이다', f'{lid.get("hz", 0):.0f} Hz', [f'유효 {lid.get("valid", 0) * 100:.0f}%', '가림 없음' if not pr else '',
                                                    f'가장 가까운 물체 {near_cm} cm' if near_cm is not None else '1.6 m 안에 물체 없음'], pr)
    else:
        part('라이다', '스캔 없음', ['라이다 서비스와 USB를 확인'], [('warn', '라이다 스캔이 안 들어옴')])
    d['near_cm'] = near_cm

    # 위치 추정
    pr = [('danger' if n in ('ekf', 'imu') else 'warn', f'서비스 꺼짐: {n}') for n in off_by.get('위치 추정', [])]
    if not drv.get('ok'):
        pr.append(('warn', '오도메트리(EKF)가 안 들어옴'))
    imu_ok = (sens.get('imu') or {}).get('connected', False)
    if not imu_ok:
        pr.append(('warn', 'IMU 값을 못 읽고 있음'))
    pm = (d.get('pose') or {}).get('map'); slam = nodes.get('slam_toolbox')
    part('위치 추정', f'({pm[0]:+.2f}, {pm[1]:+.2f}) m' if pm else '지도 없음',
         ['EKF 정상' if drv.get('ok') else '', 'IMU 정상' if imu_ok else '',
          f'방향 {math.degrees(pm[2]):.0f}°' if pm else ('SLAM 켜짐, 위치 잡는 중' if slam else 'SLAM 꺼짐')], pr)

    # 자율주행
    nav_cmd = abs(drv.get('nav_v', 0)) > 1e-3 or abs(drv.get('nav_w', 0)) > 1e-3
    man_cmd = (abs(drv.get('cmd_v', 0)) > 1e-3 or abs(drv.get('cmd_w', 0)) > 1e-3) and not nav_cmd
    driver = nodes.get('step_navigator') or nodes.get('controller_server')
    auto = nodes.get('collision_monitor') and driver
    on = [NAV_NODES[k] for k in NAV_NODES if nodes.get(k)]
    pr = [('danger', '충돌 감시 없이 주행기가 켜져 있음')] if driver and not nodes.get('collision_monitor') else []
    if nav_cmd:
        act = '자율 주행 중' + (' (탐색)' if nodes.get('frontier_explorer') else ''); value = '탐색 중' if nodes.get('frontier_explorer') else '주행 중'
    elif man_cmd:
        act = '수동 조종 또는 시험 도구가 구동 중'; value = '수동·시험'
    elif moving:
        act = '명령 없이 움직이는 중(관성 또는 손으로 옮김)'; value = '명령 없음'
    elif auto:
        act = '자율 스택 켜짐, 목표 대기'; value = '목표 대기'
    elif slam:
        act = '지도 작성 스택만 켜짐, 정지'; value = '지도 작성만'
    else:
        act = '대기 (자율 스택 꺼짐)'; value = '꺼짐'
    kz = d.get('keepout') or {}
    if on and not kz.get('server'):
        pr.append(('warn', '금지 구역 서버가 꺼져 있음 — 주행기는 구역 정보를 받기 전에는 움직이지 않는다'))
    part('자율주행', value, ['켜진 것: ' + ', '.join(on) if on else 'SLAM·경로 계획·충돌 감시·주행기 모두 꺼짐',
                          f'금지 구역 {len(kz.get("zones") or [])}개 적용' if kz.get('server') else ''], pr, off=not on)

    # 파이
    pr = [('warn', f'서비스 꺼짐: {n}') for n in off_by.get('파이', [])]
    temp = sysm.get('cpu_temp_c')
    if temp and temp >= 75:
        pr.append(('warn', f'파이 온도 {temp:.0f}°C'))
    wifi = next(iter((pw.get('wifi') or {}).values()), None) or {}
    part('파이', f'{temp:.0f}°C' if temp else '–',
         [f'CPU {sysm["cpu_percent"]:.0f}%' if sysm.get('cpu_percent') is not None else '', f'메모리 {sysm["mem_percent"]:.0f}%' if sysm.get('mem_percent') is not None else '',
          f'WiFi {wifi["level_dbm"]:.0f} dBm' if wifi.get('level_dbm') is not None else '', f'디스크 {sysm["disk_percent"]:.0f}%' if sysm.get('disk_percent') is not None else '',
          f'서비스 {sum(1 for s in svc if s["active"])}/{len(svc)}' if svc else ''], pr)

    level = 'danger' if any(a[0] == 'danger' for a in alerts) else ('warn' if alerts else 'ok')
    for k in ('power', 'sensors', 'services', 'nav_nodes', 'enc_rejects_changed'):     # 화면이 안 쓰는 큰 덩어리는 보내지 않는다
        d.pop(k, None)
    d.update(now=now, alerts=[dict(level=a, msg=m) for a, m in alerts], state=dict(level=level, act=act), parts=parts)
    return d


PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>스마트 쓰레기통 관제</title>__FAVICON__
<style>
:root{--film:#E6EBEF;--sheet:#F7F9FA;--ink:#17232F;--mute:#5F6E7B;--rule:#C9D2D9;--line:#1E5FA6;--ok:#17855F;--warn:#B47A00;--bad:#C2362B;--ghost:#A9B5BF}
*{box-sizing:border-box}
html,body{margin:0;background:var(--film);color:var(--ink);font:14px/1.45 "Pretendard","Noto Sans KR","Apple SD Gothic Neo","Malgun Gothic",system-ui,sans-serif;font-variant-numeric:tabular-nums;word-break:keep-all}
#app{display:grid;gap:10px;padding:10px;min-width:0}
header{display:flex;align-items:center;gap:16px;background:var(--sheet);border-left:8px solid var(--ok);padding:10px 14px}
header.warn{border-left-color:var(--warn)} header.danger{border-left-color:var(--bad)}
.say{flex:1;min-width:0}
#act{font-size:24px;font-weight:700;letter-spacing:-.01em;line-height:1.2}
#sub{color:var(--mute);margin-top:2px} header.warn #sub{color:var(--warn)} header.danger #sub{color:var(--bad)}
#clock{color:var(--mute);font-size:13px;white-space:nowrap;text-align:right}
#nav{display:flex;flex-wrap:wrap;gap:6px;justify-content:flex-end;max-width:46%}
#nav a{color:var(--line);text-decoration:none;font-size:13px;padding:5px 10px;border:1px solid var(--rule);border-radius:4px;white-space:nowrap}
#nav a:first-child{font-weight:700} #nav a:hover{background:var(--film)} #nav a:focus-visible{outline:2px solid var(--line);outline-offset:1px}
#estop{align-self:stretch;min-height:46px;border:0;background:var(--bad);color:#fff;font:700 17px/1 inherit;padding:0 24px;cursor:pointer;border-radius:4px;white-space:nowrap}
#estop:hover{filter:brightness(1.1)} #estop:focus-visible{outline:3px solid var(--ink);outline-offset:2px}

/* 부위별 점검: 한 줄에 여섯 칸. 칸 위의 띠 색이 판정 */
#parts{display:grid;grid-template-columns:repeat(6,minmax(0,1fr));gap:10px}
.part{background:var(--sheet);padding:8px 12px 9px;border-top:4px solid var(--ok);min-width:0}
.part h3{margin:0;font-size:13px;font-weight:600;color:var(--mute);display:flex;justify-content:space-between;gap:6px}
.part h3 em{font-style:normal;font-weight:700;color:var(--ok)}
.part b{display:block;font-size:22px;font-weight:700;line-height:1.3;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.part p{margin:1px 0 0;font-size:12.5px;line-height:1.4;color:var(--mute)}
.part.warn{border-top-color:var(--warn)} .part.warn h3 em{color:var(--warn)}
.part.danger{border-top-color:var(--bad)} .part.danger h3 em,.part.danger b{color:var(--bad)}
.part.off{border-top-color:var(--ghost)} .part.off h3 em,.part.off b{color:var(--mute)}

#main{display:grid;gap:10px;grid-template-columns:minmax(0,1.55fr) minmax(0,1fr) minmax(0,1fr);align-items:stretch}
.col{display:grid;gap:10px;align-content:start;min-width:0}
section{background:var(--sheet);padding:10px 12px 12px;min-width:0}
h2{margin:0 0 6px;font-size:14px;font-weight:700;display:flex;flex-wrap:wrap;justify-content:space-between;gap:2px 10px;align-items:baseline}
h2 small{font-weight:400;color:var(--mute);font-size:12.5px}
.cap{margin:6px 0 0;font-size:12.5px;line-height:1.4;color:var(--mute)}
/* 캔버스는 높이가 정해진 틀 안에 띄운다 — 캔버스 자체 크기가 배치를 밀지 않게 */
.cv{position:relative;min-height:0} .cv canvas{position:absolute;inset:0;width:100%;height:100%;display:block}
#map{display:flex;flex-direction:column} #map .cv{flex:1;min-height:clamp(300px,50vh,900px)}
#rad .cv{height:clamp(210px,24vh,400px)}
#spd{height:clamp(90px,11vh,140px)} #bat{height:clamp(110px,16vh,190px)}
.stale .cv{opacity:.4}

table{border-collapse:collapse;width:100%;font-size:13px}
th{font-weight:400;color:var(--mute);font-size:12.5px;text-align:left;padding:2px 6px 2px 0;border-bottom:1px solid var(--rule);white-space:nowrap}
td{padding:3px 6px 3px 0;border-bottom:1px solid var(--rule);vertical-align:top}
th.r,td.r{text-align:right;padding-right:0;padding-left:10px;white-space:nowrap} td.n{white-space:nowrap} td.m{color:var(--mute)}
td.ok{color:var(--ok);font-weight:700} td.no{color:var(--bad);font-weight:700}
#speeds td b{font-size:17px;font-weight:700}
#wheels td.bar{width:38%;vertical-align:middle;padding-left:10px} #wheels .bar i{display:block;height:6px;background:var(--rule);position:relative}
#wheels .bar i u{position:absolute;top:0;bottom:0;background:var(--line)} #wheels .bar i::after{content:"";position:absolute;left:50%;top:-2px;bottom:-2px;width:1px;background:var(--mute)}
#wheels td.bad{color:var(--bad);font-weight:700}
#spd{margin:8px 0 0} #drv .cap{margin:2px 0 8px}
ul{margin:0;padding:0;list-style:none;font-size:13px;max-height:clamp(120px,22vh,300px);overflow:auto}
li{padding:2px 0 2px 10px;border-left:3px solid var(--rule);margin-bottom:4px;overflow-wrap:anywhere}
li.warn{border-left-color:var(--warn)} li.danger,li.error{border-left-color:var(--bad)} li.none{border-left-color:var(--ok);color:var(--mute)}
li time{color:var(--mute);margin-right:6px}

@media (max-width:1500px){ /* 창이 좁으면 2단 + 아래 줄 */
  #parts{grid-template-columns:repeat(3,minmax(0,1fr))}
  #main{grid-template-columns:minmax(0,1.35fr) minmax(0,1fr)}
  #colC{grid-column:1/-1;grid-template-columns:repeat(3,minmax(0,1fr));align-items:start}
}
@media (max-width:900px){
  #parts{grid-template-columns:repeat(2,minmax(0,1fr))}
  #main,#colC{grid-template-columns:minmax(0,1fr)} #colC{grid-column:auto}
  header{flex-wrap:wrap} #clock{display:none} #nav{max-width:100%;order:3;justify-content:flex-start}
}
@media (prefers-reduced-motion:reduce){*{transition:none!important}}
</style></head><body>
<div id="app">
<header id="hdr">
  <div class="say"><div id="act">연결 중…</div><div id="sub"></div></div>
  <nav id="nav" aria-label="다른 대시보드">
    <a href="#" data-port="8764">← 시스템 대시보드(홈)</a>
    <a href="#" data-port="8766">모터</a><a href="#" data-port="8767">센서</a><a href="#" data-port="8768">지도·금지 구역</a><a href="#" data-port="8765">전원</a>
  </nav>
  <div id="clock"></div>
  <button id="estop" title="자율주행·시험 도구를 모두 종료하고 정지 명령을 보냅니다">비상정지</button>
</header>

<div id="parts"></div>

<div id="main">
  <section id="map"><h2><span id="mapTitle">지도와 로봇 위치</span><small id="mapInfo"></small></h2>
    <div class="cv"><canvas id="cMap"></canvas></div>
    <p class="cap" id="mapCap"></p></section>

  <div class="col">
    <section id="rad"><h2>로봇 바로 주변 <small id="radInfo"></small></h2>
      <div class="cv"><canvas id="cRad"></canvas></div>
      <p class="cap">파란 원(22 cm) 안은 회전에 필요한 자리, 점선 원(13.5 cm) 안은 라이다가 못 읽는 거리입니다.</p></section>
    <section id="drv"><h2>주행 <small id="drvInfo"></small></h2>
      <table id="speeds"><tr><th></th><th class="r">실제</th><th class="r">명령</th></tr>
        <tr><td>직진 m/s</td><td class="r"><b id="vAct">–</b></td><td class="r"><b id="vCmd">–</b></td></tr>
        <tr><td>회전 rad/s</td><td class="r"><b id="wAct">–</b></td><td class="r"><b id="wCmd">–</b></td></tr></table>
      <div class="cv" id="spd"><canvas id="cSpd"></canvas></div>
      <p class="cap">최근 60초 직진 속도(m/s). 굵은 선이 실제, 점선이 명령입니다.</p>
      <table id="wheels"></table></section>
  </div>

  <div class="col" id="colC">
    <section><h2>최근 목표 <small id="goalInfo"></small></h2><table id="goals"></table></section>
    <section><h2>경고 기록 <small id="logInfo"></small></h2><ul id="alerts"></ul></section>
    <section><h2>배터리 추이 <small id="batInfo"></small></h2><div class="cv" id="bat"><canvas id="cBat"></canvas></div></section>
  </div>
</div>
</div>
<script>
const $=id=>document.getElementById(id), css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const C={line:css('--line'),ink:css('--ink'),mute:css('--mute'),rule:css('--rule'),ok:css('--ok'),warn:css('--warn'),bad:css('--bad'),ghost:css('--ghost'),sheet:css('--sheet')};
const WORD={ok:'정상',warn:'주의',danger:'이상',off:'꺼짐'};
let D=null, MAP=null, mapImg=null, mapTs=0;
function fit(cv){const r=cv.getBoundingClientRect(),d=devicePixelRatio||1;if(cv.width!==Math.round(r.width*d)||cv.height!==Math.round(r.height*d)){cv.width=Math.round(r.width*d);cv.height=Math.round(r.height*d)}const g=cv.getContext('2d');g.setTransform(d,0,0,d,0,0);g.clearRect(0,0,r.width,r.height);return [g,r.width,r.height]}
const f=(v,n=2)=>v==null||isNaN(v)?'–':(+v).toFixed(n), sgn=(v,n=2)=>v==null?'–':(v>=0?'+':'−')+Math.abs(+v).toFixed(n);
const esc=s=>String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;');
function dur(s){s=Math.max(0,s|0);const h=s/3600|0,m=(s%3600)/60|0;return h?`${h}시간 ${m}분`:`${m}분`}
function dur2(s){return s<90?f(s,0)+'초':dur(s)}
function hhmm(t,sec){const d=new Date(t*1000),p=n=>String(n).padStart(2,'0');return p(d.getHours())+':'+p(d.getMinutes())+(sec?':'+p(d.getSeconds()):'')}

function robot(g,sc){ // 로봇 몸체(길이 19 × 폭 21 cm)와 뒤 케이블 영역 — 화면 위쪽이 로봇 앞
  g.fillStyle='rgba(23,35,47,.10)';g.strokeStyle=C.ink;g.lineWidth=1.5;
  g.fillRect(-0.105*sc,-0.095*sc,0.21*sc,0.19*sc);g.strokeRect(-0.105*sc,-0.095*sc,0.21*sc,0.19*sc);
  g.setLineDash([3,3]);g.strokeStyle=C.mute;g.lineWidth=1;g.strokeRect(-0.06*sc,0.095*sc,0.12*sc,0.09*sc);g.setLineDash([]);
  g.beginPath();g.moveTo(0,-0.15*sc);g.lineTo(-0.035*sc,-0.085*sc);g.lineTo(0.035*sc,-0.085*sc);g.closePath();g.fillStyle=C.ink;g.fill();
}
function drawRadar(){
  const [g,W,H]=fit($('cRad')); const L=D.lidar||{}; const R=0.82, sc=Math.min(W,H)/2/R;
  g.save();g.translate(W/2,H/2);
  g.strokeStyle=C.rule;g.lineWidth=1;g.fillStyle=C.mute;g.font='11px sans-serif';
  for(const r of [0.4,0.8]){g.beginPath();g.arc(0,0,r*sc,0,7);g.stroke();g.fillText(r*100+' cm',4,r*sc-4)}
  // 자기 구조물 방향(기준)과 새로 가려진 방향 — 라이다 위치 기준 2° 쐐기
  const lx=0, ly=-0.054*sc;
  const wedge=(deg,col,len)=>{const a0=(deg)*Math.PI/180,a1=(deg+2)*Math.PI/180;g.beginPath();g.moveTo(lx,ly);
    g.lineTo(lx-Math.sin(a0)*len,ly-Math.cos(a0)*len);g.lineTo(lx-Math.sin(a1)*len,ly-Math.cos(a1)*len);g.closePath();g.fillStyle=col;g.fill()};
  for(const d of (D.baseline||[])) wedge(d,'rgba(95,110,123,.13)',0.30*sc);
  for(const d of (L.blocked||[])) wedge(d,'rgba(194,54,43,.38)',R*sc);
  g.strokeStyle=C.line;g.lineWidth=1.5;g.beginPath();g.arc(0,0,0.22*sc,0,7);g.stroke();
  g.setLineDash([4,4]);g.strokeStyle=C.mute;g.lineWidth=1;g.beginPath();g.arc(lx,ly,0.135*sc,0,7);g.stroke();g.setLineDash([]);
  robot(g,sc);
  for(const p of (L.near||[])){const x=p[0],y=p[1];if((Math.abs(x)<=0.115&&Math.abs(y)<=0.115)||(x>=-0.19&&x<=-0.09&&Math.abs(y)<=0.065))continue;
    const d=Math.hypot(x,y);if(d>R*1.5)continue;g.fillStyle=d<0.22?C.bad:C.line;g.fillRect(-y*sc-1.5,-x*sc-1.5,3,3)}
  g.restore();
  g.fillStyle=C.mute;g.font='11px sans-serif';g.fillText('▲ 앞',W/2-13,11);
  $('radInfo').textContent=L.ok?(D.near_cm==null?'가까운 물체 없음':`가장 가까운 물체 ${D.near_cm} cm`):'스캔 없음';
  $('rad').classList.toggle('stale',!L.ok);
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
    for(const poly of ((D.keepout||{}).zones||[])){g.beginPath();poly.forEach((q,i)=>i?g.lineTo(X(q[0]),Y(q[1])):g.moveTo(X(q[0]),Y(q[1])));g.closePath();
      g.fillStyle='rgba(194,54,43,.22)';g.fill();g.strokeStyle=C.bad;g.lineWidth=2;g.stroke()}
    const [rx,ry,th]=P.map,c=Math.cos(th),s=Math.sin(th);
    g.fillStyle=C.line;for(const p of (L.pts||[])){g.fillRect(X(rx+c*p[0]-s*p[1])-1,Y(ry+s*p[0]+c*p[1])-1,2,2)}
    g.save();g.translate(X(rx),Y(ry));g.rotate(-th+Math.PI/2);const kr=Math.max(1,14/(0.21*sc));g.scale(kr,kr);robot(g,sc);g.restore();
    g.fillStyle=C.ok;g.beginPath();g.arc(X(0),Y(0),4,0,7);g.fill();
    $('mapTitle').textContent='지도와 로봇 위치';
    $('mapInfo').textContent=`지도 ${f(mw,1)} × ${f(mh,1)} m`;
    if($('mapCap').dataset.k!=='map'){$('mapCap').dataset.k='map';$('mapCap').innerHTML='검은 칸은 벽, 회색은 아직 모르는 곳입니다. 파란 점은 지금 라이다가 보는 것, 초록 점은 출발점, 모눈 한 칸은 1 m입니다. 빨간 칸은 금지 구역입니다 — <a href="#" onclick="location.port=8768;return false">지도 대시보드에서 그리고 고칩니다</a>.';}
  }else{
    // 지도가 없으면(SLAM 꺼짐) 로봇 중심으로 주변 스캔만
    const R=6.5,sc=Math.min(W,H)/2/R;g.save();g.translate(W/2,H/2);
    g.strokeStyle='rgba(30,95,166,.18)';g.lineWidth=1;g.fillStyle=C.mute;g.font='11px sans-serif';
    for(let r=1;r<=6;r++){g.beginPath();g.arc(0,0,r*sc,0,7);g.stroke();if(r%2==0)g.fillText(r+' m',4,-r*sc+12)}
    g.fillStyle=C.line;for(const p of (L.pts||[])){g.fillRect(-p[1]*sc-1,-p[0]*sc-1,2,2)}
    g.save();const k=Math.max(1,12/(0.21*sc));g.scale(k,k);robot(g,sc);g.restore();g.restore();
    $('mapTitle').textContent='라이다가 보는 주변 (지도 없음)';
    $('mapInfo').textContent='화면 위쪽이 로봇 앞';
    $('mapCap').dataset.k='scan';$('mapCap').textContent='SLAM이 꺼져 있어 지도 대신 지금 스캔만 그립니다. 시험 스택이나 자율 매핑을 켜면 이 자리에 지도와 로봇 위치가 나옵니다.';
  }
  $('map').classList.toggle('stale',!L.ok);
}
function chart(cv,series,opt){ // series: [{pts:[[t,v]],col,dash}], opt:{t1,span,min,max,ticks,dec,lines:[{v,col,label}]}
  const [g,W,H]=fit(cv),pl=30,pr=4,pt=8,pb=6; const t1=opt.t1,t0=t1-opt.span;
  const X=t=>pl+(t-t0)/(t1-t0)*(W-pl-pr), Y=v=>pt+(opt.max-v)/(opt.max-opt.min)*(H-pt-pb);
  g.font='11px sans-serif';g.fillStyle=C.mute;g.strokeStyle=C.rule;g.lineWidth=1;
  for(const v of opt.ticks){g.beginPath();g.moveTo(pl,Y(v));g.lineTo(W-pr,Y(v));g.stroke();g.fillText(v.toFixed(opt.dec),0,Y(v)+4)}
  for(const l of (opt.lines||[])){g.strokeStyle=l.col;g.setLineDash([5,4]);g.beginPath();g.moveTo(pl,Y(l.v));g.lineTo(W-pr,Y(l.v));g.stroke();g.setLineDash([]);g.fillStyle=l.col;g.fillText(l.label,W-pr-g.measureText(l.label).width-2,Y(l.v)+(l.below?12:-4))}
  for(const s of series){g.strokeStyle=s.col;g.lineWidth=s.w||1.6;g.setLineDash(s.dash||[]);g.beginPath();let on=false;
    for(const p of s.pts){if(p[0]<t0)continue;const x=X(p[0]),y=Y(Math.max(opt.min,Math.min(opt.max,p[1])));on?g.lineTo(x,y):g.moveTo(x,y);on=true}g.stroke();g.setLineDash([])}
}
function render(){
  const s=D.state, B=D.battery||{}, drv=D.drive||{}, M=D.motor||{};
  $('hdr').className=s.level==='ok'?'':s.level; $('act').textContent=s.act;
  const al=D.alerts||[]; $('sub').textContent=al.length?al.map(a=>a.msg).join(' / '):'여섯 부위 모두 문제 없음';
  $('clock').textContent=hhmm(D.now,true)+' 갱신';
  $('parts').innerHTML=(D.parts||[]).map(p=>`<div class="part ${p.level==='ok'?'':p.level}"><h3>${p.name}<em>${WORD[p.level]}</em></h3><b>${esc(p.value)}</b><p>${esc(p.detail)}</p></div>`).join('');
  // 주행
  $('vAct').textContent=sgn(drv.v,3); $('vCmd').textContent=sgn(drv.cmd_v,3); $('wAct').textContent=sgn(drv.w,2); $('wCmd').textContent=sgn(drv.cmd_w,2);
  $('drvInfo').textContent=drv.ok?(Math.abs(drv.nav_v)>1e-3||Math.abs(drv.nav_w)>1e-3?'자율 명령이 들어오는 중':(drv.cmd_age==null?'이 화면을 켠 뒤 명령 없음':'마지막 명령 '+dur2(drv.cmd_age)+' 전')):'오도메트리 없음';
  const h=drv.hist||[], t1=D.now;
  chart($('cSpd'),[{pts:h.map(r=>[r[0],r[3]]),col:C.mute,dash:[4,3]},{pts:h.map(r=>[r[0],r[1]]),col:C.line,w:2}],
    {t1,span:60,min:-0.1,max:0.3,ticks:[0,0.15,0.3],dec:2,lines:[{v:0.15,col:C.mute,label:'순항 0.15'}]});
  const rej=D.enc_rejects, names=['왼쪽 앞 M1','오른쪽 앞 M2','왼쪽 뒤 M3','오른쪽 뒤 M4'], rate=(M.enc_age!=null&&M.enc_age>1)?[NaN,NaN,NaN,NaN]:(M.rate||[0,0,0,0]);
  $('wheels').innerHTML='<tr><th>바퀴</th><th class="r">카운트/초</th><th></th><th class="r">오독 누적</th></tr>'+names.map((n,i)=>{const r=Number.isNaN(rate[i])?NaN:(rate[i]||0),w=Number.isNaN(r)?0:Math.min(50,Math.abs(r)/60),left=r>=0?50:50-w;
    return `<tr><td class="n">${n}</td><td class="r">${Number.isNaN(r)?'—':f(r,0)}</td><td class="bar"><i><u style="left:${left}%;width:${w}%"></u></i></td><td class="r ${rej&&rej[i]>0?'bad':''}">${rej?rej[i].toLocaleString():'–'}</td></tr>`}).join('');
  // 배터리
  const bh=D.batt_hist||[], th=B.thresholds||{warn1:11.3,warn2:11.0,critical:10.7};
  chart($('cBat'),[{pts:bh,col:C.line,w:2}],{t1,span:Math.max(600,bh.length?t1-bh[0][0]:600),min:10.4,max:12.8,ticks:[11,12],dec:0,
    lines:[{v:th.warn1,col:C.warn,label:'주의 '+f(th.warn1,1)+' V'},{v:th.warn2,col:C.bad,label:'구동 정지 '+f(th.warn2,1)+' V',below:true}]});
  $('batInfo').textContent=bh.length?`최근 ${dur(t1-bh[0][0])} · 지금 ${f(B.voltage,2)} V`:'기록 없음';
  // 목표·경고
  const gs=(D.goals||[]).slice().reverse();
  $('goals').innerHTML=gs.length?'<tr><th>시각</th><th>결과</th><th>목표</th><th class="r">오차</th><th class="r">걸린 시간</th></tr>'+gs.map(r=>`<tr><td class="n m">${hhmm(r.t)}</td><td class="${r.ok?'ok n':'no'}">${r.ok?'도착':esc(r.msg)}</td><td>${esc(r.src)} <span style="color:var(--mute);white-space:nowrap">(${sgn(r.goal[0],1)}, ${sgn(r.goal[1],1)})</span></td><td class="r">${r.err_cm==null?'–':f(r.err_cm,1)+' cm'}</td><td class="r">${f(r.secs,0)}초</td></tr>`).join('')
    :'<tr><td class="m">오늘 기록된 목표가 없습니다.</td></tr>';
  const nOk=gs.filter(r=>r.ok).length; $('goalInfo').textContent=gs.length?`오늘 최근 ${gs.length}건 중 ${nOk}건 도착`:'';
  const lg=(D.log||[]).slice().reverse();
  $('alerts').innerHTML=al.map(a=>`<li class="${a.level}"><time>지금</time>${esc(a.msg)}</li>`).join('')+lg.map(a=>`<li class="${a.level}"><time>${hhmm(a.t)}</time>${esc(a.msg)}</li>`).join('')||'<li class="none">구동·IMU·EKF에서 나온 경고가 없습니다.</li>';
  $('logInfo').textContent=al.length?`지금 ${al.length}건`:(lg.length?`지난 기록 ${lg.length}건`:'');
  drawRadar(); drawMap();
}
async function poll(){try{D=await (await fetch('/api/overview')).json();render()}catch(e){$('act').textContent='관제 서버와 연결이 끊겼습니다';$('sub').textContent='파이 전원과 WiFi를 확인하세요';$('hdr').className='danger'}}
async function pollMap(){try{const m=await (await fetch('/api/map')).json();if(m&&m.connected)MAP=m;else MAP=null}catch(e){}}
$('estop').onclick=async()=>{$('estop').textContent='정지 보내는 중…';try{const r=await (await fetch('/api/estop',{method:'POST'})).json();$('estop').textContent=r&&r.ok?`정지함 (${(r.killed||[]).length}개 종료)`:'실패 — 모터 대시보드 확인'}catch(e){$('estop').textContent='실패 — 연결 확인'}setTimeout(()=>$('estop').textContent='비상정지',4000)};
$('nav').addEventListener('click',ev=>{const p=ev.target.dataset.port;if(p){ev.preventDefault();location.href=location.protocol+'//'+location.hostname+':'+p+'/'}});
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
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: os._exit(0))
    threading.Thread(target=ros_thread, daemon=True).start()
    threading.Thread(target=sys_thread, daemon=True).start()
    print(f'관제 화면: http://<Pi IP>:{a.port}', flush=True)
    ThreadingHTTPServer(('0.0.0.0', a.port), Handler).serve_forever()


if __name__ == '__main__':
    main()
