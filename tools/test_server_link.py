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
import server_link
from server_link import Link


class Fake:
    def __init__(self):
        self.log = []; self.caps = None; self.force_409 = False; self.missing = set()   # missing: 아직 없는 받는 곳(404)
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
                elif leaf in fake.missing:
                    code = 404
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
    f.log.clear(); r = L.handle({'id': 'c2', 'type': 'say', 'args': {'text': '안녕'}, 'ttl_s': 30})   # 11-22부터 go_to는 실제 명령이라 '아직 없는 명령'으로 바꿈
    acks = f.kinds('ack')
    check('④ 아직 없는 명령(say 등)은 rejected/invalid', r == 'rejected' and [(a['phase'], a.get('reason')) for a in acks] == [('rejected', 'invalid')], acks)

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

    # ⑨~⑪ 지도·자리(규약 3.9) — ROS 없이 가짜 원천으로
    class Geo:
        def __init__(self):
            self.mov = False; self.png = b'PNG-A'
        def map_snapshot(self):
            return {'map_id': 'live-abcdef01-01', 'kind': 'live', 'resolution': 0.05, 'origin': [-1.0, -2.0, 0.0],
                    'width': 3, 'height': 2, 'png': self.png}
        def moving(self):
            return self.mov
        def pose(self):
            return {'map_id': 'live-abcdef01-01', 'x': 1.0, 'y': 2.0, 'yaw_deg': 10.0, 'age_ms': 30}
    import base64
    f4 = Fake(); geo = Geo(); L4 = mk(f4.url, geo_source=geo); L4.hello(); f4.log.clear()
    r1 = L4.send_map(geo.map_snapshot()); r2 = L4.send_map(geo.map_snapshot()); geo.png = b'PNG-B'; r3 = L4.send_map(geo.map_snapshot())
    maps = f4.kinds('map')
    check('⑨ 지도: 바뀌었을 때만 보내고 map_version이 는다', (r1, r2, r3) == (200, 'same', 200) and [m['map_version'] for m in maps] == [1, 2]
          and maps[0]['encoding'] == 'png_gray_occupancy' and base64.b64decode(maps[1]['image_b64']) == b'PNG-B'
          and maps[0]['origin'] == [-1.0, -2.0, 0.0] and maps[0]['kind'] == 'live', (r1, r2, r3, [m.get('map_version') for m in maps]))
    p1 = L4.send_pose(); geo.mov = True; p2 = L4.send_pose(); poses = f4.kinds('pose')
    check('⑩ 자리: 서 있으면 보내지 않고, 움직이면 map_id·도·age_ms와 moving을 보냄', p1 == 'still' and p2 == 200 and len(poses) == 1
          and poses[0] == {'map_id': 'live-abcdef01-01', 'x': 1.0, 'y': 2.0, 'yaw_deg': 10.0, 'age_ms': 30, 'moving': True}, (p1, p2, poses))
    f4.missing = {'pose'}; f4.log.clear(); q1 = L4.send_pose(); q2 = L4.send_pose(); n_posts = len(f4.kinds('pose'))
    check('⑪ 받는 곳이 아직 없으면(404) 1분 쉬고 쌓지 않음', q1 == 404 and q2 == 'backoff' and n_posts == 1
          and not any(i['kind'] == 'pose' for i in L4.outbox.items), (q1, q2, n_posts))
    f4.close()

    # ⑫~⑮ 2단계: go_to·return_home(주행기 미션) + 저장 지도 올리기
    f5 = Fake(); calls = []
    def spawn(x, y, label):
        calls.append((round(x, 2), round(y, 2), label)); return subprocess.Popen([sys.executable, '-c', 'pass'])   # 종료 코드 0 = 성공
    class Geo2(Geo):
        def map_ok(self):
            return True
    g5 = Geo2(); g5.mov = False
    L5 = mk(f5.url, geo_source=g5, mission=spawn); L5.navigator_present = True; L5.hello(); f5.log.clear()
    r = L5.handle({'id': 'g1', 'type': 'go_to', 'args': {'x': 1.5, 'y': -2.0}, 'ttl_s': 30})
    fin = f5.kinds('ack')[-1]
    check('⑫ go_to가 주행기 미션을 띄우고 끝나면 done', r == 'done' and calls == [(1.5, -2.0, '서버목표')] and fin['result']['outcome'] == 'done', (r, calls, fin))
    r = L5.handle({'id': 'g2', 'type': 'return_home', 'args': {}, 'ttl_s': 30})
    check('⑬ return_home은 출발점(설정 없으면 지도 원점 0,0)으로', r == 'done' and calls[-1] == (0.0, 0.0, '서버출발점'), (r, calls[-1]))
    r = L5.handle({'id': 'g3', 'type': 'go_to', 'args': {'x': 'abc'}, 'ttl_s': 30})
    fin = f5.kinds('ack')[-1]
    check('⑭ 좌표가 틀린 go_to는 failed(invalid)', r == 'failed' and fin['result']['reason'] == 'invalid' and len(calls) == 2, (r, fin))
    L5.status_battery = {'stage': 'warn2'}
    r = L5.handle({'id': 'g4', 'type': 'go_to', 'args': {'x': 1, 'y': 1}, 'ttl_s': 30})
    check('⑮ 배터리 2차 단계면 go_to를 battery로 거절', r == 'failed' and f5.kinds('ack')[-1]['result']['reason'] == 'battery' and len(calls) == 2)
    L5.status_battery = {'stage': 'ok'}
    L6 = mk(f5.url, geo_source=Geo2()); L6.navigator_present = True   # 미션 띄울 수단 없음
    r = L6.handle({'id': 'g5', 'type': 'go_to', 'args': {'x': 1, 'y': 1}, 'ttl_s': 30})
    check('⑮ 미션 수단이 없으면 no_stack', r == 'failed' and f5.kinds('ack')[-1]['result']['reason'] == 'no_stack')
    f5.close()

    # ⑯ 저장 지도: ~/maps의 새 .pgm|.yaml을 saved로 올린다
    import numpy as np, cv2, yaml
    d = tempfile.mkdtemp(); img = np.full((4, 5), 254, np.uint8); img[0, 0] = 0
    cv2.imwrite(os.path.join(d, 'testmap.pgm'), img)
    yaml.safe_dump({'image': 'testmap.pgm', 'resolution': 0.05, 'origin': [-1.0, -2.0, 0.0]}, open(os.path.join(d, 'testmap.yaml'), 'w'))
    os.utime(os.path.join(d, 'testmap.pgm'), (time.time() + 5, time.time() + 5))
    server_link.SAVED_DIR = d
    f6 = Fake(); L7 = mk(f6.url, geo_source=Geo2()); L7.hello(); f6.log.clear()
    n = L7.scan_saved_maps(); maps = f6.kinds('map')
    check('⑯ 저장 지도(saved)를 이름·해상도·원점 그대로 올리고 두 번은 안 올림', n == 1 and len(maps) == 1
          and maps[0]['map_id'] == 'testmap' and maps[0]['kind'] == 'saved' and maps[0]['width'] == 5 and maps[0]['height'] == 4
          and maps[0]['origin'] == [-1.0, -2.0, 0.0] and L7.scan_saved_maps() == 0, (n, maps[:1]))
    f6.close()

    # ⑰ 장소 이름: 저장 지도에 붙인 이름으로 go_to, hello의 places에도 실린다
    d2 = tempfile.mkdtemp()
    json.dump({'places': [{'name': '주방', 'x': -1.71, 'y': 4.73}, {'name': '거실', 'x': 4.14, 'y': 2.73}]},
              open(os.path.join(d2, 'mymap.places.json'), 'w'), ensure_ascii=False)
    server_link.SAVED_DIR = d2
    class Geo3(Geo2):
        map_id = 'mymap'
    f7 = Fake(); calls2 = []
    L8 = mk(f7.url, geo_source=Geo3(), mission=lambda x, y, l: (calls2.append((round(x, 2), round(y, 2))), subprocess.Popen([sys.executable, '-c', 'pass']))[1])
    L8.navigator_present = True; L8.hello()
    pl = f7.kinds('hello')[-1]['places']
    r = L8.handle({'id': 'p1', 'type': 'go_to', 'args': {'place': '주방'}, 'ttl_s': 30})
    r2 = L8.handle({'id': 'p2', 'type': 'go_to', 'args': {'place': '없는방'}, 'ttl_s': 30})
    check('⑰ hello에 장소 목록, go_to는 이름으로 좌표를 찾고 모르는 이름은 invalid', pl == ['거실', '주방'] and r == 'done'
          and calls2 == [(-1.71, 4.73)] and r2 == 'failed' and f7.kinds('ack')[-1]['result']['reason'] == 'invalid', (pl, r, calls2, r2))
    class Geo4(Geo2):
        map_id = 'live-abcd-01'
    server_link.ACTIVE_PLACES = os.path.join(d2, 'active_places.json')
    L10 = mk(f7.url, geo_source=Geo4())
    empty = L10.places()
    json.dump({'places': [{'name': '주방', 'x': 9.0, 'y': 9.0}]}, open(server_link.ACTIVE_PLACES, 'w'), ensure_ascii=False)
    check("⑱ live 지도면 옮겨 둔 active_places.json을 읽는다(없으면 빈 목록)", empty == [] and L10.places() == ['주방'])
    L11 = mk(f7.url, geo_source=Geo3(), mission=lambda x, y, l: subprocess.Popen([sys.executable, '-c', 'raise SystemExit(1)']))
    L11.navigator_present = True; L11.status_battery = {'stage': 'ok'}
    r = L11.handle({'id': 'f1', 'type': 'go_to', 'args': {'place': '주방'}, 'ttl_s': 30})
    check('⑲ 미션이 실패(종료 코드 1)면 done이 아니라 failed로 알린다', r == 'failed'
          and f7.kinds('ack')[-1]['result']['reason'].startswith('mission_failed'), (r, f7.kinds('ack')[-1]))
    f7.close()

    # ⑳ seen: 사람이 보이면 얼굴 부분(잘린 JPEG)과 상자를 올리고, 안 보이면 올리지 않는다
    f8 = Fake(); calls3 = []
    def seen_src():
        calls3.append(1)
        return (b'JPEGCROP', {'x1': 10, 'y1': 20, 'x2': 110, 'y2': 320, 'score': 0.8, 't': time.monotonic()}) if len(calls3) % 2 else None
    L12 = mk(f8.url, geo_source=Geo3(), seen_source=seen_src); L12.hello(); f8.log.clear()
    r1 = L12.send_seen(); r2 = L12.send_seen(); seen = f8.kinds('seen')
    check('⑳ seen: 사람이 보일 때만 얼굴 부분(잘린 JPEG)과 상자를 올린다', r1 == 200 and r2 == 'none' and len(seen) == 1
          and seen[0]['jpeg_b64'] == base64.b64encode(b'JPEGCROP').decode() and seen[0]['box']['score'] == 0.8
          and seen[0]['camera'] == 'logitech_c270', (r1, r2, seen[:1]))
    f8.close()

    print('합격' if not fails else f'불합격 {len(fails)}건: {fails}')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
