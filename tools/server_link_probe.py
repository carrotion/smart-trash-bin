#!/usr/bin/env python3
"""서버 받는 곳과의 연결을 재는 측정용 로봇 쪽 클라이언트 (2026-11-15, 규약 0.2판 4절 "정할 값"용).

로봇을 움직이지 않는다. 능력 목록에는 get_status와 cancel만 올리고, 그 밖의 명령은 `rejected/invalid`로 거절한다.
하는 일: hello → status 5초마다 → 명령 long-poll → get_status를 받으면 received·started·status·finished로 답한다.
서버 쪽 도구(서버 저장소 tools/measure_robot_delivery.py)가 get_status를 무작위 간격으로 넣고 "넣은 시각 → received 도착"을
서버 시계로 잰다. 이 도구는 로봇 시계로 따로 남긴다: 요청마다 HTTP 왕복, 명령을 받은 순간 그 long-poll이 걸려 있던 시간
(짧으면 다시 거는 틈에 쌓여 있던 명령), Wi-Fi 세기.

설정(저장소에 쓰지 않음):
  기기 토큰: ~/.config/smart-trash-bin/robot_token (권한 600 — 아니면 시작하지 않는다. 값은 화면·기록에 내지 않는다)
  서버 주소: --server 또는 환경 변수 ROBOT_SERVER_URL 또는 ~/.config/smart-trash-bin/server_url
기록: --out 폴더에 requests.csv(요청마다), commands.csv(명령마다). 주소·토큰은 쓰지 않는다.

    python3 tools/server_link_probe.py --device trashbin --minutes 10 --out logs/server_link_20261115
"""
import argparse, csv, hashlib, json, os, stat, statistics, sys, threading, time, urllib.error, urllib.request
from datetime import datetime

CONF = os.path.expanduser('~/.config/smart-trash-bin')
STATUS_S = 5.0
POLL_S = 20


def read_token():
    p = os.path.join(CONF, 'robot_token')
    if not os.path.exists(p):
        sys.exit(f'기기 토큰 파일이 없어요: {p} (서버에서 만든 토큰을 사용자가 직접 넣고 chmod 600)')
    st = os.stat(p)
    if st.st_mode & (stat.S_IRWXG | stat.S_IRWXO):
        sys.exit(f'{p} 권한이 600이 아니에요(다른 사용자가 읽을 수 있음). chmod 600 하고 다시 실행하세요.')
    tok = open(p).read().strip()
    if not tok:
        sys.exit(f'{p}가 비어 있어요.')
    return tok


def server_url(arg):
    if arg:
        return arg.rstrip('/')
    if os.environ.get('ROBOT_SERVER_URL'):
        return os.environ['ROBOT_SERVER_URL'].rstrip('/')
    p = os.path.join(CONF, 'server_url')
    if os.path.exists(p):
        return open(p).read().strip().rstrip('/')
    sys.exit('서버 주소가 없어요: --server, ROBOT_SERVER_URL, 또는 ~/.config/smart-trash-bin/server_url')


def wifi_dbm():
    """가장 센 무선 연결의 신호(dBm). 없으면 None."""
    try:
        best = None
        for line in open('/proc/net/wireless').read().splitlines()[2:]:
            v = float(line.split()[3].rstrip('.'))
            best = v if best is None else max(best, v)
        return best
    except Exception:
        return None


def pct(v, q):
    return sorted(v)[min(len(v) - 1, int(round(q * (len(v) - 1))))] if v else float('nan')


class Probe:
    def __init__(self, base, device, token, out):
        self.base, self.device, self.token = base, device, token
        self.boot_id = open('/proc/sys/kernel/random/boot_id').read().strip()[:8]
        self.commands = {'get_status': {'args': {}, 'rejects': []},
                         'cancel': {'args': {'id': '명령 번호', 'target': 'mission|speech|all'}, 'rejects': []}}
        self.caps_version = hashlib.sha1(json.dumps(self.commands, sort_keys=True).encode()).hexdigest()[:8]
        self.stop = threading.Event(); self.lock = threading.Lock(); self.seen = []
        os.makedirs(out, exist_ok=True)
        self.req_f = open(os.path.join(out, 'requests.csv'), 'w', newline=''); self.req = csv.writer(self.req_f)
        self.req.writerow(['robot_time', 'kind', 'http', 'rtt_ms', 'wifi_dbm'])
        self.cmd_f = open(os.path.join(out, 'commands.csv'), 'w', newline=''); self.cmd = csv.writer(self.cmd_f)
        self.cmd.writerow(['robot_time', 'command_id', 'type', 'ttl_s', 'poll_age_ms', 'received_ack_rtt_ms', 'duplicate'])
        self.rtts = {}

    def call(self, method, path, body=None, timeout=10.0, kind=None):
        data = json.dumps(body, ensure_ascii=False).encode() if body is not None else None
        rq = urllib.request.Request(self.base + path, data=data, method=method,
                                    headers={'Authorization': f'Bearer {self.token}', 'Content-Type': 'application/json'})
        t0 = time.monotonic()
        try:
            with urllib.request.urlopen(rq, timeout=timeout) as r:
                code, raw = r.status, r.read()
        except urllib.error.HTTPError as e:
            code, raw = e.code, e.read()
        except Exception as e:
            code, raw = f'err:{type(e).__name__}', b''
        dt = (time.monotonic() - t0) * 1000
        with self.lock:
            self.req.writerow([datetime.now().isoformat(timespec='milliseconds'), kind or path.rsplit('/', 1)[-1], code, round(dt, 1), wifi_dbm()])
            self.req_f.flush()
            if kind != 'poll':
                self.rtts.setdefault(kind or 'other', []).append(dt) if code == 200 else None
        try:
            return code, (json.loads(raw) if raw else {}), dt
        except ValueError:
            return code, {}, dt

    def hello(self):
        body = {'protocol': '0.2', 'software': 'server_link_probe', 'caps_version': self.caps_version, 'name': self.device,
                'devices': {'mic': False, 'speaker': False, 'camera': False}, 'commands': self.commands, 'places': [],
                'status_interval_s': STATUS_S}
        return self.call('POST', f'/v1/robots/{self.device}/hello', body, kind='hello')[0]

    def status(self):
        body = {'protocol': '0.2', 'software': 'server_link_probe', 'caps_version': self.caps_version, 'boot_id': self.boot_id,
                'uptime_s': float(open('/proc/uptime').read().split()[0]), 'robot_time': datetime.now().astimezone().isoformat(timespec='seconds'),
                'mode': 'idle', 'pose': None, 'battery': None, 'doing': None, 'speaking': False,
                'devices': {'mic': False, 'speaker': False, 'camera': False},
                'link': {'wifi_dbm': wifi_dbm()}, 'note': '측정 모드 — 로봇을 움직이지 않음'}
        code = self.call('POST', f'/v1/robots/{self.device}/status', body, kind='status')[0]
        if code == 409:                       # 규약 0.2: hello를 보내고 같은 status를 한 번만 다시
            self.hello(); code = self.call('POST', f'/v1/robots/{self.device}/status', body, kind='status')[0]
        return code

    def ack(self, cid, phase, **kw):
        return self.call('POST', f'/v1/robots/{self.device}/ack', {'command_id': cid, 'phase': phase, **kw}, kind='ack_' + phase)

    def status_loop(self):
        while not self.stop.is_set():
            self.status(); self.stop.wait(STATUS_S)

    def run(self, minutes):
        print(f'hello → {self.hello()}  (caps {self.caps_version}, 토큰은 표시하지 않음)')
        th = threading.Thread(target=self.status_loop, daemon=True); th.start()
        end = time.monotonic() + minutes * 60; n = 0
        try:
            while time.monotonic() < end:
                t_poll = time.monotonic()
                code, body, _ = self.call('GET', f'/v1/robots/{self.device}/commands?timeout={POLL_S}', timeout=POLL_S + 10, kind='poll')
                if code == 204:
                    continue
                if code != 200:
                    time.sleep(1); continue
                age = (time.monotonic() - t_poll) * 1000
                c = body.get('command') or {}; cid = c.get('id', ''); dup = cid in self.seen
                if not dup:
                    self.seen = (self.seen + [cid])[-200:]
                _, _, rtt = self.ack(cid, 'received')
                with self.lock:
                    self.cmd.writerow([datetime.now().isoformat(timespec='milliseconds'), cid, c.get('type'), c.get('ttl_s'), round(age, 1), round(rtt, 1), dup])
                    self.cmd_f.flush()
                n += 1
                if dup:
                    continue
                if c.get('type') == 'get_status':
                    self.ack(cid, 'started'); self.status(); self.ack(cid, 'finished', result={'outcome': 'done', 'ok': True})
                elif c.get('type') == 'cancel':
                    self.ack(cid, 'finished', result={'outcome': 'done', 'ok': True})
                else:
                    self.ack(cid, 'rejected', reason='invalid')     # 측정 모드: 움직이는 명령은 받지 않는다
        except KeyboardInterrupt:
            print('\n멈춤')
        finally:
            self.stop.set(); th.join(timeout=3)
        print(f'받은 명령 {n}개')
        for k, v in sorted(self.rtts.items()):
            print(f'  {k:<14} {len(v):4d}건  왕복 중앙 {statistics.median(v):6.1f} ms  90% {pct(v, 0.9):6.1f}  99% {pct(v, 0.99):6.1f}  최대 {max(v):6.1f}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--device', default='trashbin'); ap.add_argument('--server'); ap.add_argument('--minutes', type=float, default=10)
    ap.add_argument('--out', default=f'logs/server_link_{datetime.now():%Y%m%d_%H%M}')
    a = ap.parse_args()
    Probe(server_url(a.server), a.device, read_token(), a.out).run(a.minutes)


if __name__ == '__main__':
    main()
