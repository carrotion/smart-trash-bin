#!/usr/bin/env python3
"""tools/server_link.py 시험 — 로봇·ROS·실제 서버 없이 (2026-11-15).

가짜 서버(같은 프로세스 안 HTTP)로 확인하는 것:
  ① hello·status, 409면 hello 뒤 같은 status를 한 번만 다시(서버가 계속 409여도 되풀이하지 않음)
  ② get_status → received·started·(status)·finished(done) 순서
  ③ 같은 번호가 다시 오면 다시 하지 않고 마지막 단계(finished)만 다시 알림 — received로 덮지 않음
  ④ 모르는 명령(go_to 등 움직이는 것)은 rejected/invalid
  ⑤ 받은 뒤 ttl_s가 지나 시작하면 finished/expired
  ⑥ 서버가 없을 때 ack·event를 파일에 쌓고, 다시 켠 Link가 이어서 순서대로 보냄. event_id = 부팅 번호-일련번호, ago_ms는 일이 난 때부터
  ⑦ 쌓는 수 상한을 넘으면 오래된 것부터 버리고 그 수를 status에 싣는다
  ⑧ cancel이 돌고 있는 mission_go.py에 SIGINT를 보내고, 끝난 것을 확인한 뒤 done

    python3 tools/test_server_link.py
"""
import json, os, subprocess, sys, tempfile, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from server_link import Link


class Fake:
    def __init__(self):
        self.log = []; self.caps = None; self.force_409 = False
        fake = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])) or b'{}')
                leaf = self.path.rsplit('/', 1)[-1]
                fake.log.append((leaf, body, self.headers.get('Authorization')))
                code = 200
                if leaf == 'hello':
                    fake.caps = body['caps_version']
                elif leaf == 'status' and (fake.force_409 or body.get('caps_version') != fake.caps):
                    code = 409
                self.send_response(code); self.send_header('Content-Length', '2'); self.end_headers(); self.wfile.write(b'{}')

            def do_GET(self):
                self.send_response(204); self.end_headers()
        self.srv = ThreadingHTTPServer(('127.0.0.1', 0), H)
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        self.url = f'http://127.0.0.1:{self.srv.server_address[1]}'

    def kinds(self, leaf):
        return [b for l, b, _ in self.log if l == leaf]

    def close(self):
        self.srv.shutdown(); self.srv.server_close()


fails = []


def check(name, ok, detail=''):
    print(f'{"✅" if ok else "❌"} {name}' + (f' — {detail}' if detail and not ok else ''))
    if not ok:
        fails.append(name)


def mk(url, path=None, **kw):
    return Link(url, 'trashbin', 'TOKEN', outbox_path=path, boot_id='abcdef0123', **kw)


def main():
    tmp = tempfile.mkdtemp()
    f = Fake(); L = mk(f.url)

    # ①
    L.hello(); c = L.status()
    check('① hello 뒤 status 200, 토큰이 Bearer로 실림', c == 200 and f.log[0][2] == 'Bearer TOKEN')
    f.log.clear(); f.force_409 = True; c = L.status(); f.force_409 = False
    seq = [l for l, _, _ in f.log]
    check('① 409면 hello + status 한 번만 다시', seq == ['status', 'hello', 'status'] and c == 409, seq)

    # ②
    f.log.clear(); r = L.handle({'id': 'c1', 'type': 'get_status', 'ttl_s': 30})
    seq = [(l, b.get('phase')) for l, b, _ in f.log]
    check('② get_status 순서', r == 'done' and seq == [('ack', 'received'), ('ack', 'started'), ('status', None), ('ack', 'finished')], seq)
    fin = f.kinds('ack')[-1]
    check('② finished 결과에 outcome·ok', fin['result'] == {'outcome': 'done', 'ok': True}, fin)

    # ③
    f.log.clear(); r = L.handle({'id': 'c1', 'type': 'get_status', 'ttl_s': 30})
    acks = f.kinds('ack')
    check('③ 다시 온 명령은 하지 않고 finished만 다시', r == 'duplicate' and len(acks) == 1 and acks[0]['phase'] == 'finished'
          and not f.kinds('status'), acks)

    # ④
    f.log.clear(); r = L.handle({'id': 'c2', 'type': 'go_to', 'args': {'x': 1, 'y': 2}, 'ttl_s': 30})
    acks = f.kinds('ack')
    check('④ 움직이는 명령은 rejected/invalid', r == 'rejected' and [(a['phase'], a.get('reason')) for a in acks] == [('rejected', 'invalid')], acks)

    # ⑤
    f.log.clear(); r = L.handle({'id': 'c3', 'type': 'get_status', 'ttl_s': 0.5}, received_at=time.monotonic() - 2)
    acks = f.kinds('ack')
    check('⑤ 유효 시간이 지난 명령은 하지 않고 expired', r == 'expired' and [a['phase'] for a in acks] == ['received', 'finished']
          and acks[-1]['result']['outcome'] == 'expired' and not f.kinds('status'), acks)
    f.close()

    # ⑥ 서버 없음 → 쌓기 → 새 Link가 이어 보내기
    box = os.path.join(tmp, 'outbox.jsonl'); dead = 'http://127.0.0.1:9'          # 닫힌 포트
    D = mk(dead, box)
    e1 = D.event('battery_stage', {'from': 'ok', 'to': 'warn1'}); D.ack('c9', 'received'); time.sleep(0.3); e2 = D.event('mission_failed')
    lines = open(box).read().splitlines()
    check('⑥ 서버가 없으면 파일에 3건 쌓임', len(lines) == 3, len(lines))
    check('⑥ event_id = 부팅 번호 8자-일련번호', (e1, e2) == ('abcdef01-000001', 'abcdef01-000002'), (e1, e2))
    f2 = Fake(); D2 = mk(f2.url, box)
    check('⑥ 다시 켜면 쌓인 것을 읽고 일련번호를 이어 감', len(D2.outbox.items) == 3 and D2.seq == 2, (len(D2.outbox.items), D2.seq))
    time.sleep(0.2); D2.flush()
    sent = [(l, b.get('event_id') or b.get('phase')) for l, b, _ in f2.log]
    check('⑥ 연결되면 순서대로 보내고 파일을 비움', sent == [('event', e1), ('ack', 'received'), ('event', e2)] and open(box).read() == '', sent)
    ago = [b['ago_ms'] for l, b, _ in f2.log if l == 'event']
    check('⑥ ago_ms는 일이 난 때부터(먼저 난 일이 더 큼)', ago[0] > ago[1] >= 200, ago)
    f2.close()

    # ⑦
    server_link_cap = Link(dead, 'trashbin', 'T', outbox_path=os.path.join(tmp, 'cap.jsonl'), boot_id='x' * 8)
    server_link_cap.outbox.cap = 5
    for i in range(8):
        server_link_cap.event('e', {'i': i})
    st = server_link_cap.status_body()['link']
    left = [it['body']['data']['i'] for it in server_link_cap.outbox.items]
    check('⑦ 상한을 넘으면 오래된 것부터 버리고 status에 그 수', left == [3, 4, 5, 6, 7] and st['outbox'] == 5 and st['outbox_dropped'] == 3, (left, st))

    # ⑧ cancel → mission_go.py에 SIGINT
    mg = os.path.join(tmp, 'mission_go.py'); mark = os.path.join(tmp, 'got_sigint')
    open(mg, 'w').write('import signal,sys,time\n'
                        f'def h(*_):\n    open({mark!r},"w").write("1"); sys.exit(0)\n'
                        'signal.signal(signal.SIGINT, h)\nwhile True: time.sleep(0.1)\n')
    p = subprocess.Popen([sys.executable, mg]); time.sleep(0.5)
    f3 = Fake(); L3 = mk(f3.url); L3.hello()
    r = L3.handle({'id': 'k1', 'type': 'cancel', 'args': {'target': 'all'}, 'ttl_s': 30})
    p.wait(timeout=5)
    fin = f3.kinds('ack')[-1]
    check('⑧ cancel이 mission_go에 SIGINT를 보내고 끝난 뒤 done', r == 'done' and os.path.exists(mark)
          and fin['result'].get('stopped_missions') == 1, (r, fin))
    r = L3.handle({'id': 'k2', 'type': 'cancel', 'args': {'target': 'speech'}, 'ttl_s': 30})
    check('⑧ cancel target=speech는 스피커가 없어 그냥 done', r == 'done')
    f3.close()

    print('합격' if not fails else f'불합격 {len(fails)}건: {fails}')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
