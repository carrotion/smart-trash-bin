#!/usr/bin/env python3
"""로봇 ↔ 서버 연결 — 로봇 쪽 1단계 (2026-11-15, `docs/server_protocol.md` 0.2판 5절 구현 순서 1).

하는 일:
  - 켤 때 `hello`(능력 목록). `status`를 5초마다. 409면 `hello`를 보내고 같은 `status`를 한 번만 다시 보낸다.
  - 명령을 long-poll로 받는다. 받는 명령은 `get_status`, `cancel`뿐이다. 그 밖의 명령은 `rejected/invalid`로 거절한다.
    로봇을 움직이는 명령은 2단계에서 넣는다.
  - `cancel`(target mission·all)은 돌고 있는 `tools/mission_go.py`에 SIGINT를 보낸다. mission_go는 SIGINT를 받으면 주행 목표를
    취소하고 끝난다(29e4ef2). 비상정지가 아니다. 탐색 멈춤은 1단계 밖이라 `failed`로 알린다.
  - `ack`·`event`는 보내지 못하면 파일에 쌓아 두었다가, 연결되면 일련번호 순서대로 보낸다(엣지 우선 7절 "음영 지역").
    로봇은 서버를 기다리지 않는다. 연결이 끊겨도 이 프로그램 밖의 일은 아무것도 멈추지 않는다.
  - 같은 명령 번호가 다시 오면 다시 하지 않고, 그 명령의 마지막 단계 알림만 다시 보낸다.
    `received`를 다시 보내면 서버가 끝난 명령을 받음 상태로 덮어 버리기 때문이다.
  - 시작 전에 `ttl_s`가 지난 명령은 하지 않고 `finished/expired`로 알린다.

설정(저장소에 쓰지 않음): 기기 토큰 ~/.config/smart-trash-bin/robot_token(권한 600), 서버 주소 ~/.config/smart-trash-bin/server_url.
쌓아 두는 파일: ~/.local/state/smart-trash-bin/server_outbox.jsonl(최대 OUTBOX_MAX건, 넘치면 오래된 것부터 버리고 그 수를 status에 싣는다).

    python3 tools/server_link.py                  # 실행(Ctrl-C로 끝)
    python3 tools/test_server_link.py             # 하드웨어·서버 없이 시험
"""
import base64, collections, hashlib, json, os, signal, stat, statistics, subprocess, sys, threading, time, urllib.error, urllib.request
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import killproc

CONF = os.path.expanduser('~/.config/smart-trash-bin')
STATE = os.path.expanduser('~/.local/state/smart-trash-bin')
PROTOCOL = '0.2'
STATUS_S = 5.0
POLL_S = 20
OUTBOX_MAX = 2000       # 잠정: 사건은 드물다(배터리 단계·미션 결과). 음영 지역에 몇 시간 있어도 넘지 않을 양. Wi-Fi 지도를 보고 다시 정한다
SEEN_MAX = 200
MAP_S = 10.0           # 그리는 중인 지도를 올리는 주기(바뀌었을 때만) — 규약 3.9
POSE_S = 0.5           # 움직이는 동안 자리를 올리는 주기
NOT_FOUND_BACKOFF_S = 60.0   # 받는 곳이 아직 없으면(404) 그만큼 쉰다
SAVED_DIR = os.path.expanduser('~/maps')
SAVED_SCAN_S = 30.0          # 저장 지도 폴더를 살피는 주기
COMMANDS = {
    'get_status': {'args': {}, 'rejects': []},
    'cancel': {'args': {'id': '명령 번호(없으면 지금 하는 것)', 'target': 'mission|speech|all'}, 'rejects': []},
    'go_to': {'args': {'place': '장소 이름', 'x': 'm', 'y': 'm'}, 'rejects': ['no_stack', 'battery', 'busy', 'invalid']},
    'return_home': {'args': {}, 'rejects': ['no_stack', 'battery', 'busy']},
}


def software_version():
    try:
        return subprocess.run(['git', '-C', os.path.dirname(os.path.abspath(__file__)), 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True, timeout=3).stdout.strip() or 'unknown'
    except Exception:
        return 'unknown'


def alive_pid(pid):
    """끝나고 아직 거둬지지 않은(좀비) 프로세스는 끝난 것으로 본다."""
    try:
        return open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()[0] != 'Z'
    except (OSError, IndexError):
        return False


class Outbox:
    """보내지 못한 ack·event. 순서를 지키고, 파일에 남겨 다시 켜도 이어 보낸다."""

    def __init__(self, path, cap=OUTBOX_MAX):
        self.path, self.cap, self.dropped = path, cap, 0
        self.items = collections.deque()
        if path and os.path.exists(path):
            for line in open(path):
                try:
                    self.items.append(json.loads(line))
                except ValueError:
                    pass

    def _save(self):
        if not self.path:
            return
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            for it in self.items:
                f.write(json.dumps(it, ensure_ascii=False) + '\n')
        os.replace(tmp, self.path)

    def push(self, item):
        self.items.append(item)
        while len(self.items) > self.cap:
            self.items.popleft(); self.dropped += 1
        self._save()

    def pop_front(self):
        self.items.popleft(); self._save()


class Link:
    def __init__(self, base, device, token, outbox_path=None, status_source=None, boot_id=None,
                 clock=time.monotonic, wall=time.time, geo_source=None, mission=None):
        self.base, self.device, self.token = base.rstrip('/'), device, token
        self.clock, self.wall = clock, wall
        self.status_source = status_source or (lambda: {})
        self.boot = (boot_id or open('/proc/sys/kernel/random/boot_id').read().strip())[:8]
        self.caps_version = hashlib.sha1(json.dumps(COMMANDS, sort_keys=True).encode()).hexdigest()[:8]
        self.software = software_version()
        self.outbox = Outbox(outbox_path)
        self.seq = max([int(i['body'].get('event_id', '-0').rsplit('-', 1)[1]) for i in self.outbox.items
                        if i['kind'] == 'event' and i['body'].get('event_id', '').startswith(self.boot)] or [0])
        self.last_phase = collections.OrderedDict()     # 명령 번호 → 마지막으로 보낸 ack 본문
        self.rtts = collections.deque(maxlen=20); self.fails = collections.deque(maxlen=20)
        self.lock = threading.Lock(); self.stop = threading.Event(); self.doing = None
        self.geo = geo_source                 # 지도·자리(map_snapshot(), moving(), pose()) — 규약 3.9. 없으면 지도·자리를 올리지 않는다
        self.map_sent = {}                    # map_id → (map_version, 내용 해시)
        self.backoff = {}                     # 받는 곳 이름 → 이 시각(clock)까지 쉼
        self.started_at = self.wall()          # 이 프로세스가 켜진 때(저장 지도는 그 뒤에 바뀐 것만 올린다)
        self.saved_sent = set(); self.saved_seed = False
        self.mission = mission                # 미션 띄우기: mission(x, y, label) → pid (2단계). 없으면 go_to를 거절한다
        self.mission_pid = None
        self.mission_timeout_s = 600.0        # 미션 하나의 상한(주행기 자체 상한과 별개로 여기서도 끊는다)
        self.status_battery = None            # 마지막으로 본 배터리(단계로 go_to를 거절할 때 쓴다)
        self.navigator_present = None          # 시험에서 주행기 서버 유무를 정해 준다(실제로는 killproc로 본다)
        self.home = self._read_home()

    # ---- HTTP
    def call(self, method, path, body=None, timeout=10.0):
        data = json.dumps(body, ensure_ascii=False).encode() if body is not None else None
        rq = urllib.request.Request(self.base + path, data=data, method=method,
                                    headers={'Authorization': f'Bearer {self.token}', 'Content-Type': 'application/json'})
        t0 = self.clock()
        try:
            with urllib.request.urlopen(rq, timeout=timeout) as r:
                code, raw = r.status, r.read()
        except urllib.error.HTTPError as e:
            code, raw = e.code, e.read()
        except Exception:
            code, raw = None, b''
        if method != 'GET' or '/commands' not in path:      # long-poll은 기다리는 시간이라 왕복 통계에서 뺀다
            self.fails.append(code is None)
            if code is not None:
                self.rtts.append((self.clock() - t0) * 1000)
        try:
            return code, (json.loads(raw) if raw else {})
        except ValueError:
            return code, {}

    def places(self):
        """지금 지도에 붙인 장소 이름 목록. 장소는 지도 좌표라 그 지도 위에서만 뜻이 있다(`~/maps/<지도>.places.json`)."""
        mid = getattr(self.geo, 'map_id', None) if self.geo else None
        if not mid or mid.startswith('live-'):
            return []
        try:
            d = json.load(open(os.path.join(SAVED_DIR, mid + '.places.json')))
            return sorted(p['name'] for p in d.get('places', []))
        except Exception:
            return []

    def place_xy(self, name):
        mid = getattr(self.geo, 'map_id', None) if self.geo else None
        try:
            d = json.load(open(os.path.join(SAVED_DIR, mid + '.places.json')))
            for p in d['places']:
                if p['name'] == name:
                    return float(p['x']), float(p['y'])
        except Exception:
            pass
        return None

    @staticmethod
    def _read_home():
        """출발점(충전 자리). 설정 파일이 없으면 지금 지도의 원점(0, 0) — 지도를 그리기 시작한 자리다."""
        try:
            d = json.load(open(os.path.join(CONF, 'home_pose.json')))
            return float(d['x']), float(d['y'])
        except Exception:
            return 0.0, 0.0

    def path(self, leaf):
        return f'/v1/robots/{self.device}/{leaf}'

    # ---- 보내기
    def hello(self):
        return self.call('POST', self.path('hello'), {
            'protocol': PROTOCOL, 'software': self.software, 'caps_version': self.caps_version, 'name': '쓰레기통',
            'devices': {'mic': False, 'speaker': False, 'camera': False}, 'commands': COMMANDS, 'places': self.places(),
            'status_interval_s': STATUS_S})[0]

    def status_body(self):
        rt = sorted(self.rtts)
        body = {'protocol': PROTOCOL, 'software': self.software, 'caps_version': self.caps_version, 'boot_id': self.boot,
                'uptime_s': round(self.clock(), 1), 'robot_time': datetime.now().astimezone().isoformat(timespec='seconds'),
                'mode': 'idle', 'doing': self.doing, 'speaking': False,
                'devices': {'mic': False, 'speaker': False, 'camera': False},
                'link': {'rtt_ms_p50': round(statistics.median(rt), 1) if rt else None,
                         'lost_pct': round(100 * sum(self.fails) / len(self.fails), 1) if self.fails else None,
                         'outbox': len(self.outbox.items), 'outbox_dropped': self.outbox.dropped}}
        try:
            body.update(self.status_source())
            self.status_battery = body.get('battery')
        except Exception as e:                       # 상태 읽기가 실패해도 보내기는 계속한다
            body['source_error'] = type(e).__name__
        if 'link_wifi_dbm' in body:
            body['link']['wifi_dbm'] = body.pop('link_wifi_dbm')
        return body

    def status(self):
        body = self.status_body()
        code = self.call('POST', self.path('status'), body)[0]
        if code == 409:                               # 규약 0.2: hello 뒤 같은 status를 한 번만 다시
            self.hello(); code = self.call('POST', self.path('status'), body)[0]
        if code == 200:
            self.flush()
        return code

    def flush(self):
        """쌓인 ack·event를 순서대로. 하나라도 실패하면 거기서 멈추고 다음에 다시."""
        with self.lock:
            while self.outbox.items:
                it = self.outbox.items[0]; body = dict(it['body'])
                if it['kind'] == 'event':
                    body['ago_ms'] = int((self.wall() - it['t_wall']) * 1000)
                code = self.call('POST', self.path(it['kind']), body)[0]
                if code is None or code >= 500:
                    return False
                self.outbox.pop_front()                 # 4xx는 다시 보내도 같으니 버린다
            return True

    def _send(self, kind, body):
        with self.lock:
            self.outbox.push({'kind': kind, 'body': body, 't_wall': self.wall()})
        self.flush()

    def ack(self, cid, phase, **kw):
        body = {'command_id': cid, 'phase': phase, **kw}
        self.last_phase[cid] = body
        while len(self.last_phase) > SEEN_MAX:
            self.last_phase.popitem(last=False)
        self._send('ack', body)

    def event(self, kind, data=None):
        with self.lock:
            self.seq += 1; eid = f'{self.boot}-{self.seq:06d}'
        self._send('event', {'event_id': eid, 'kind': kind, 'data': data or {}})
        return eid

    # ---- 명령
    def handle(self, cmd, received_at=None):
        received_at = self.clock() if received_at is None else received_at
        cid = cmd.get('id', ''); typ = cmd.get('type')
        if cid in self.last_phase:                    # 다시 온 명령: 하지 않고 마지막 단계만 다시 알린다
            self._send('ack', self.last_phase[cid]); return 'duplicate'
        if typ not in COMMANDS:
            self.ack(cid, 'rejected', reason='invalid'); return 'rejected'
        self.ack(cid, 'received')
        ttl = cmd.get('ttl_s')
        if ttl is not None and self.clock() - received_at > float(ttl):
            self.ack(cid, 'finished', result={'outcome': 'expired', 'ok': False}); return 'expired'
        self.ack(cid, 'started'); self.doing = cid
        try:
            if typ == 'get_status':
                self.status(); result = {'outcome': 'done', 'ok': True}
            elif typ in ('go_to', 'return_home'):
                result = self.go(typ, cmd.get('args') or {})
            else:
                result = self.cancel(cmd.get('args') or {})
        finally:
            self.doing = None
        self.ack(cid, 'finished', result=result)
        return result['outcome']

    def reject_reason(self, typ):
        """움직이는 명령을 받을 수 있는 상태인가(규약 3.2 거절 사유). keepout·자리 검사는 주행기가 직접 거절한다."""
        if self.mission is None or not self.geo or not self.geo.map_ok():
            return 'no_stack'            # 지도·자리가 없거나(자율 스택 꺼짐) 미션을 띄울 수단이 없다
        nav = self.navigator_present if self.navigator_present is not None else bool(killproc.find(['step_nav_action_server'], include_services=True))
        if not nav:
            return 'no_stack'            # 주행기 서버가 없으면(비상정지가 껐거나 스택이 안 떴다) 움직이지 않는다
        st = (self.status_battery or {}).get('stage')
        if st in ('warn2', 'critical'):
            return 'battery'
        if self.mission_pid is not None and alive_pid(self.mission_pid):
            return 'busy'
        return None

    def go(self, typ, args):
        """go_to·return_home: 주행기 도구(tools/mission_go.py)로 미션을 띄우고, 끝날 때까지 지켜본다."""
        if typ == 'go_to':
            if args.get('place'):
                xy = self.place_xy(str(args['place']))
                if xy is None:
                    return {'outcome': 'failed', 'ok': False, 'reason': 'invalid'}
                x, y = xy
            else:
                try:
                    x, y = float(args['x']), float(args['y'])
                except (KeyError, TypeError, ValueError):
                    return {'outcome': 'failed', 'ok': False, 'reason': 'invalid'}
        else:
            x, y = float(self.home[0]), float(self.home[1])
        why = self.reject_reason(typ)
        if why:
            return {'outcome': 'failed', 'ok': False, 'reason': why}
        label = '서버' + ('목표' if typ == 'go_to' else '출발점')
        self.mission_pid = self.mission(x, y, label)
        if not self.mission_pid:
            return {'outcome': 'failed', 'ok': False, 'reason': 'no_stack'}
        ok, msg = self.wait_mission()
        return {'outcome': 'done', 'ok': True} if ok else {'outcome': 'failed', 'ok': False, 'reason': msg}

    def wait_mission(self):
        """미션이 끝날 때까지(최대 mission_timeout_s) 기다린다. 취소·비상정지로 죽으면 그 사실을 돌려준다."""
        end = self.clock() + self.mission_timeout_s
        while self.clock() < end and self.mission_pid and alive_pid(self.mission_pid):
            time.sleep(0.2)
        if self.mission_pid and alive_pid(self.mission_pid):
            return False, 'mission_timeout'
        return True, ''

    def cancel(self, args):
        target = args.get('target', 'all')
        if target == 'speech':
            return {'outcome': 'done', 'ok': True, 'note': '스피커 없음'}
        missions = killproc.find(['mission_go'])
        for pid, _, _ in missions:
            try:
                os.kill(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        end = self.clock() + 8.0
        while self.clock() < end and any(alive_pid(p) for p, _, _ in missions):
            time.sleep(0.1)
        alive = [p for p, _, _ in missions if alive_pid(p)]
        if alive:
            return {'outcome': 'failed', 'ok': False, 'reason': f'미션이 8초 안에 끝나지 않음(pid {alive})'}
        if killproc.find(['frontier_explorer'], include_services=True):
            return {'outcome': 'failed', 'ok': False, 'reason': '탐색 멈춤은 1단계 밖 — 지도 대시보드의 [탐색 멈춤]'}
        return {'outcome': 'done', 'ok': True, 'stopped_missions': len(missions)}

    # ---- 지도·자리(규약 3.9) — 쌓아 두지 않는다. 끊겨 있으면 버리고 다음 것을 보낸다
    def _post_live(self, leaf, body):
        if self.clock() < self.backoff.get(leaf, 0):
            return 'backoff'
        code = self.call('POST', self.path(leaf), body)[0]
        if code == 404:
            self.backoff[leaf] = self.clock() + NOT_FOUND_BACKOFF_S
        return code

    def send_map(self, snap):
        """snap = {map_id, kind, resolution, origin, width, height, png}. 내용이 같으면 보내지 않는다."""
        h = hashlib.sha1(snap['png']).hexdigest(); prev = self.map_sent.get(snap['map_id'])
        if prev and prev[1] == h:
            return 'same'
        ver = prev[0] + 1 if prev else 1
        body = {k: snap[k] for k in ('map_id', 'kind', 'resolution', 'origin', 'width', 'height')}
        body.update(map_version=ver, encoding='png_gray_occupancy', image_b64=base64.b64encode(snap['png']).decode())
        code = self._post_live('map', body)
        if code == 200:
            self.map_sent[snap['map_id']] = (ver, h)
        return code

    def send_pose(self):
        if not self.geo.moving():
            return 'still'
        p = self.geo.pose()
        return self._post_live('pose', dict(p, moving=True)) if p else 'no_map'

    def saved_map(self, name):
        """~/maps/<이름>.pgm|.yaml을 map_server 형식 그대로 읽어 서버 형식으로. 없거나 깨졌으면 None."""
        import cv2
        try:
            import yaml
            d = yaml.safe_load(open(os.path.join(SAVED_DIR, name + '.yaml')))
            img = cv2.imread(os.path.join(SAVED_DIR, name + '.pgm'), cv2.IMREAD_GRAYSCALE)
            if img is None or not d.get('resolution'):
                return None
            ok, png = cv2.imencode('.png', img)
            if not ok:
                return None
            return {'map_id': name, 'kind': 'saved', 'resolution': round(float(d['resolution']), 4),
                    'origin': [round(float(d['origin'][0]), 4), round(float(d['origin'][1]), 4), 0.0],
                    'width': int(img.shape[1]), 'height': int(img.shape[0]), 'png': png.tobytes()}
        except Exception:
            return None

    def scan_saved_maps(self):
        """새로 저장된 지도(이름.pgm 이 이 프로세스가 켜진 뒤에 바뀐 것)를 찾아 올린다."""
        try:
            names = {f[:-4] for f in os.listdir(SAVED_DIR) if f.endswith('.pgm')}
        except OSError:
            return 0
        n = 0
        if not self.saved_seed and names:       # 켤 때 한 번은 가장 최근 저장 지도를 올린다(서버에 지도가 없으면 화면이 빈다)
            self.saved_seed = True
            pick = {n for n in names if n != 'latest'} or names      # 자동 저장본(latest)보다 이름 붙인 지도를 먼저
            newest = max(pick, key=lambda x: os.path.getmtime(os.path.join(SAVED_DIR, x + '.pgm')))
            snap = self.saved_map(newest)
            if snap and self.send_map(snap) == 200:
                self.saved_sent.add(newest); n += 1
                print(f'저장 지도 올림(최근 것): {newest} {snap["width"]}x{snap["height"]}', flush=True)
        for name in sorted(names):
            try:
                mt = os.path.getmtime(os.path.join(SAVED_DIR, name + '.pgm'))
            except OSError:
                continue
            if name in self.saved_sent or (mt <= self.started_at and name != newest):
                continue
            snap = self.saved_map(name)
            if snap and self.send_map(snap) == 200:
                self.saved_sent.add(name); n += 1
                print(f'저장 지도 올림: {name} {snap["width"]}x{snap["height"]} {len(snap["png"])}바이트', flush=True)
        return n

    def map_loop(self):
        last_saved = -1e9
        while not self.stop.is_set():
            try:
                snap = self.geo.map_snapshot()
                if snap:
                    self.send_map(snap)
                if self.clock() - last_saved > SAVED_SCAN_S:
                    self.scan_saved_maps(); last_saved = self.clock()
            except Exception as e:
                print(f'지도 올리기 실패: {type(e).__name__} {e}', flush=True)
            self.stop.wait(MAP_S)

    def pose_loop(self):
        while not self.stop.is_set():
            try:
                self.send_pose()
            except Exception as e:
                print(f'자리 올리기 실패: {type(e).__name__} {e}', flush=True)
            self.stop.wait(POSE_S)

    # ---- 돌리기
    def status_loop(self):
        while not self.stop.is_set():
            self.status(); self.stop.wait(STATUS_S)

    def poll_loop(self):
        back = 1.0
        while not self.stop.is_set():
            code, body = self.call('GET', self.path(f'commands?timeout={POLL_S}'), timeout=POLL_S + 10)
            if code == 200 and body.get('command'):
                c = body['command']; r = self.handle(c); back = 1.0
                print(f"명령 {c.get('id')} {c.get('type')} {json.dumps(c.get('args') or {}, ensure_ascii=False)} → {r}", flush=True)
            elif code in (200, 204):
                back = 1.0
            else:
                self.stop.wait(back); back = min(back * 2, 10.0)

    def run(self):
        print(f'hello → {self.hello()}  (caps {self.caps_version}, 쌓인 것 {len(self.outbox.items)}건)', flush=True)
        loops = [self.status_loop, self.poll_loop] + ([self.map_loop, self.pose_loop] if self.geo else [])
        threads = [threading.Thread(target=f, daemon=True) for f in loops]
        for t in threads:
            t.start()
        return threads


# ---- 로봇 상태 읽기(실행할 때만. 시험은 이 부분 없이 Link만 쓴다)
class RobotState:
    """배터리(모터 대시보드 API), 모드(돌고 있는 프로세스), 지도 위 자세(TF map→base_link, 자율 스택이 켜져 있을 때만), Wi-Fi."""

    def __init__(self, on_battery_stage=None):
        self.on_battery_stage = on_battery_stage; self.stage = None; self.tf = None; self.node = None; self.spin = None
        self.boot = open('/proc/sys/kernel/random/boot_id').read().strip()[:8]
        self.map_msg = None; self.map_t = 0.0; self.session = 0; self.odom = (0.0, 0.0, 0.0)   # (v, w, 받은 시각)
        try:
            import rclpy, tf2_ros
            from rclpy.signals import SignalHandlerOptions
            from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
            from nav_msgs.msg import OccupancyGrid, Odometry
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO); self.node = rclpy.create_node('server_link')
            self.tf = tf2_ros.Buffer(); tf2_ros.TransformListener(self.tf, self.node)
            self.node.create_subscription(OccupancyGrid, '/map', self._on_map, QoSProfile(
                depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
            self.node.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)
            self.spin = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True); self.spin.start()
        except Exception as e:
            print(f'ROS 없이 실행(자세 없음): {e}', flush=True)

    def close(self):
        """rclpy를 정리하지 않고 끝내면 'terminate called without an active exception'으로 죽는다(CLAUDE.md 대시보드 사례와 같음).
        순서: shutdown → spin 스레드 join → 노드 정리."""
        if not self.node:
            return
        import rclpy
        try:
            rclpy.shutdown()
        except Exception:
            pass
        if self.spin:
            self.spin.join(timeout=3)
        self.node.destroy_node()

    # 지도: SLAM이 그리는 /map(탐색 중 5초마다). 20초 넘게 안 오다가 다시 오면 새 지도(새 좌표계)로 본다 → map_id가 바뀐다
    def _on_map(self, m):
        now = time.monotonic()
        if now - self.map_t > 20.0:
            self.session += 1
        self.map_msg, self.map_t = m, now

    def _on_odom(self, m):
        self.odom = (m.twist.twist.linear.x, m.twist.twist.angular.z, time.monotonic())

    @property
    def map_id(self):
        return f'live-{self.boot}-{self.session:02d}'

    def map_ok(self):
        return self.map_msg is not None and time.monotonic() - self.map_t < 30.0

    def map_snapshot(self):
        """규약 3.9 지도: 8비트 회색 PNG(0 벽, 254 빈칸, 205 모름 — map_server 저장과 같은 문턱), 행 0이 맨 위."""
        if not self.map_ok():
            return None
        import cv2, numpy as np
        m = self.map_msg; w, h = m.info.width, m.info.height
        d = np.frombuffer(bytes(m.data), np.int8).reshape(h, w).astype(np.int16)
        img = np.full((h, w), 205, np.uint8); img[(d >= 0) & (d <= 19)] = 254; img[d >= 65] = 0
        ok, png = cv2.imencode('.png', img[::-1])
        o = m.info.origin.position
        return {'map_id': self.map_id, 'kind': 'live', 'resolution': round(m.info.resolution, 4),
                'origin': [round(o.x, 4), round(o.y, 4), 0.0], 'width': w, 'height': h, 'png': png.tobytes()} if ok else None

    def moving(self):
        v, w, t = self.odom
        return time.monotonic() - t < 1.0 and (abs(v) > 0.02 or abs(w) > 0.05)

    def pose(self):
        if not self.tf or not self.map_ok():
            return None
        try:
            import math, rclpy.time
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time()); q = t.transform.rotation
            age = (self.node.get_clock().now() - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds / 1e6
            return {'map_id': self.map_id, 'x': round(t.transform.translation.x, 2), 'y': round(t.transform.translation.y, 2),
                    'yaw_deg': round(math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))), 1),
                    'age_ms': int(max(0, age))}
        except Exception:
            return None

    def __call__(self):
        out = {}
        try:
            b = json.load(urllib.request.urlopen('http://127.0.0.1:8766/api/battery', timeout=2))
            out['battery'] = {'v': round(b.get('avg') or b.get('voltage') or 0, 2), 'stage': b.get('tier')}
            if b.get('tier') != self.stage:
                if self.stage is not None and self.on_battery_stage:
                    self.on_battery_stage(self.stage, b.get('tier'), out['battery']['v'])
                self.stage = b.get('tier')
        except Exception:
            out['battery'] = None
        out['mode'] = ('mission' if killproc.find(['mission_go']) else
                       'explore' if killproc.find(['frontier_explorer'], include_services=True) else 'idle')
        out['pose'] = self.pose()
        try:
            la = os.getloadavg(); mem = {l.split(':')[0]: int(l.split()[1]) for l in open('/proc/meminfo')}
            out['cpu_load1'] = round(la[0], 2); out['mem_avail_mb'] = mem['MemAvailable'] // 1024
            w = [l for l in open('/proc/net/wireless').read().splitlines()[2:] if l.split(':')[0].strip() == 'wlan0']
            out['link_wifi_dbm'] = float(w[0].split()[3].rstrip('.')) if w else None
        except Exception:
            pass
        return out


def read_conf():
    tok = os.path.join(CONF, 'robot_token'); url = os.path.join(CONF, 'server_url')
    for p in (tok, url):
        if not os.path.exists(p):
            sys.exit(f'설정 파일이 없어요: {p}')
    if os.stat(tok).st_mode & (stat.S_IRWXG | stat.S_IRWXO):
        sys.exit(f'{tok} 권한이 600이 아니에요.')
    return open(url).read().strip(), open(tok).read().strip()


def main():
    base, token = read_conf()
    link = None

    def battery_stage(old, new, v):
        if link:
            link.event('battery_stage', {'from': old, 'to': new, 'v': v})

    def spawn_mission(x, y, label):
        """주행기 도구로 미션 하나. 반환 pid가 끝나면 미션이 끝난 것이다(취소는 SIGINT — mission_go가 스스로 목표를 취소한다)."""
        os.makedirs(STATE, exist_ok=True)
        p = subprocess.Popen([sys.executable, '-u', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mission_go.py'),
                              f'{label}:{x:.2f},{y:.2f}'], stdout=open(os.path.join(STATE, 'mission.log'), 'ab'),
                             stderr=subprocess.STDOUT)
        print(f'미션 띄움: {label} ({x:.2f}, {y:.2f}) pid {p.pid}', flush=True)
        return p.pid

    state = RobotState(battery_stage)
    link = Link(base, 'trashbin', token, os.path.join(STATE, 'server_outbox.jsonl'), state, geo_source=state, mission=spawn_mission)
    signal.signal(signal.SIGTERM, lambda *_: link.stop.set()); signal.signal(signal.SIGINT, lambda *_: link.stop.set())
    link.run()
    while not link.stop.is_set():
        link.stop.wait(1)
    state.close()
    print('멈춤', flush=True)


if __name__ == '__main__':
    main()
