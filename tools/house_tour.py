#!/usr/bin/env python3
"""집 한 바퀴 기록 — 방 이름 장소를 차례로 돌며 장소마다 제자리에서 90°씩 둘러보고, 카메라 화면·자세·라이다를 남긴다 (2026-11-28).

왜: 주행 AR의 3D 장면과 그 장면-지도 자동 맞춤 확인. 사용자 결정(11-28): 휴대폰 대신 로봇이 돌며 찍는다.
    서버 쪽은 스택이 켜진 동안 `scene`(1초)으로 맞춤을 다듬고, 이 기록(화면 + 화면마다 지도 자세)으로 3D를 만든다.

남기는 것(집 안에만 — 공개 저장소에 넣지 않는다, 저장소 밖 ~/ar_scan/):
    ~/ar_scan/tour_<시각>/frames/000123.jpg   카메라 원본(상자 없음, 640×480)
    ~/ar_scan/<시각>/frames.jsonl        화면마다 {i, t, x, y, yaw_deg, moving, place}
    ~/ar_scan/<시각>/bag/                /scan·/tf·/tf_static·/odometry/filtered 녹화
    ~/ar_scan/<시각>/tour.json           장소마다 결과(도착·시간·점까지 거리), 둘러보기 결과
올리기: 장마다 집 안 서버 POST /v1/robots/<기기>/scan/frame {session, seq, t, place, moving, pose{x,y,yaw_deg}, jpeg_b64}
      (서버 주소·토큰은 server_link와 같은 ~/.config/smart-trash-bin/). 실패한 장은 건너뛴다 — 로봇에는 다 남는다.
카메라 기하: tools/camera_detect.GEOM (f=644 px, 높이 14 cm, 몸통 가운데, 위 기울기 — 11-25 9.5°, 11-28 카메라를 올려 다시 잼).

    python3 tools/house_tour.py                 # 지금 장소 목록 전부, 가까운 순서
    python3 tools/house_tour.py 거실 주방 식탁   # 고른 장소만, 적은 순서대로
    python3 tools/house_tour.py --no-look        # 둘러보기 없이 지나가기만

스택이 저장 지도로 켜져 있어야 한다(MAP=… tools/nav_test_stack.sh start). 두 번 연속 실패하면 멈춘다. 배터리는 mission_go가 본다(11.4 V).
"""
import argparse, json, math, os, signal, subprocess, sys, threading, time, urllib.request

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))
CAM = 'http://127.0.0.1:8767'
PLACE_AREA_M = 0.30
TOPICS = ['/scan', '/tf', '/tf_static', '/odometry/filtered']


def order_nearest(start, places):
    """가까운 곳부터(욕심 순서). 9곳이면 충분하다 — 최적 순서를 찾을 만큼 길지 않다."""
    left = list(places); out = []; here = start
    while left:
        p = min(left, key=lambda q: math.hypot(q[1] - here[0], q[2] - here[1]))
        out.append(p); left.remove(p); here = (p[1], p[2])
    return out


class Uploader(threading.Thread):
    """장마다 집 안 서버로 한 장씩 올린다 — POST /v1/robots/<기기>/scan/frame (11-28 서버 쪽과 정한 형식).
    실패(5xx·시간 초과)면 그 장은 건너뛴다(로봇에는 남아 있다). 줄이 넘치면 가장 새 장을 버린다 — 주행을 기다리게 하지 않는다."""

    def __init__(self, session):
        super().__init__(daemon=True)
        import queue, server_link as S
        base, token = S.read_conf()
        self.link = S.Link(base, 'trashbin', token); self.session = session
        self.q = queue.Queue(maxsize=200); self.ok = 0; self.fail = 0; self.dropped = 0; self.last_err = None

    def put(self, seq, t, pose, moving, place, jpg):
        import base64, queue
        body = dict(session=self.session, seq=seq, t=round(t, 3), place=place, moving=moving,
                    pose=dict(x=pose['x'], y=pose['y'], yaw_deg=pose['yaw_deg']), jpeg_b64=base64.b64encode(jpg).decode())
        try:
            self.q.put_nowait(body)
        except queue.Full:
            self.dropped += 1

    def run(self):
        while True:
            body = self.q.get()
            if body is None:
                return
            code, _ = self.link.call('POST', f'/v1/robots/{self.link.device}/scan/frame', body, timeout=5.0)
            if code == 200:
                self.ok += 1
            else:
                self.fail += 1; self.last_err = code

    def finish(self, wait_s=60.0):
        """남은 장을 다 보낼 때까지(최대 wait_s) 기다린다."""
        end = time.time() + wait_s
        while not self.q.empty() and time.time() < end:
            time.sleep(0.5)
        self.q.put(None); self.join(10)


class Recorder(threading.Thread):
    """frame_s마다 카메라 원본 한 장과 그때 자세를 남긴다. 카메라는 요청이 오는 동안 켜져 있다(20초 안 찾으면 꺼짐)."""

    def __init__(self, state, out, frame_s, uploader=None):
        super().__init__(daemon=True)
        self.state, self.out, self.frame_s, self.up = state, out, frame_s, uploader
        self.place = None; self.stop = threading.Event(); self.n = 0; self.fail = 0
        os.makedirs(os.path.join(out, 'frames'), exist_ok=True)
        self.log = open(os.path.join(out, 'frames.jsonl'), 'a')

    def run(self):
        while not self.stop.is_set():
            t0 = time.time()
            try:
                jpg = urllib.request.urlopen(CAM + '/camera.jpg?raw=1', timeout=8).read()
                p = self.state.pose()
                if p and jpg[:2] == b'\xff\xd8':
                    name = f'{self.n:06d}.jpg'
                    with open(os.path.join(self.out, 'frames', name), 'wb') as f:
                        f.write(jpg)
                    self.log.write(json.dumps(dict(i=self.n, t=round(t0, 3), x=p['x'], y=p['y'], yaw_deg=p['yaw_deg'],
                                                   moving=self.state.moving(), place=self.place), ensure_ascii=False) + '\n')
                    self.log.flush()
                    if self.up:
                        self.up.put(self.n, t0, p, self.state.moving(), self.place, jpg)
                    self.n += 1
            except Exception:
                self.fail += 1
            self.stop.wait(max(0.0, self.frame_s - (time.time() - t0)))


def mission(spec, log):
    """mission_go 하나를 돌려 끝날 때까지 기다린다. 성공 = 종료 코드 0."""
    p = subprocess.Popen([sys.executable, '-u', os.path.join(REPO, 'tools', 'mission_go.py'), spec], stdout=log, stderr=subprocess.STDOUT)
    try:
        return p.wait() == 0
    except BaseException:
        p.send_signal(signal.SIGTERM); p.wait(20); raise


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('places', nargs='*'); ap.add_argument('--no-look', action='store_true')
    ap.add_argument('--frame-s', type=float, default=0.5); ap.add_argument('--no-bag', action='store_true')
    ap.add_argument('--no-return', action='store_true', help='끝나고 출발 자리로 돌아오지 않음')
    ap.add_argument('--no-upload', action='store_true', help='집 안 서버로 올리지 않음(로봇에만 남김)')
    a = ap.parse_args()
    import server_link as S
    allp = [(p['name'], p['x'], p['y']) for p in json.load(open(os.path.expanduser('~/maps/active_places.json')))['places']]
    # 11-28 사용자: "안방 화장실은 금지구역이잖아, 단차가 있어서 들어가면 안 된다." 장소 점은 구역 밖(앞 0.28 m)이라 막히지 않았다.
    #   금지 구역과 이름이 같은 장소(공백 무시)는 "들어가면 안 되는 방"이라 둘러볼 대상에서 뺀다. 거리로 거르면 안 된다 —
    #   훙게방 점은 침대 구역에서 0.19 m라 함께 빠진다
    import keepout
    zn = {str(q.get('name', '')).replace(' ', '') for q in keepout.load().get('zones', [])}
    banned = [p[0] for p in allp if p[0].replace(' ', '') in zn]
    if banned:
        print('금지 구역 방이라 뺌:', banned, flush=True)
        allp = [p for p in allp if p[0] not in banned]
    if a.places:
        byname = {p[0]: p for p in allp}
        miss = [n for n in a.places if n not in byname]
        if miss:
            print('❌ 모르는 장소:', miss, '— 있는 것:', [p[0] for p in allp]); return 2
    state = S.RobotState()
    try:
        for _ in range(40):
            if state.pose():
                break
            time.sleep(0.5)
        p0 = state.pose()
        if not p0:
            print('❌ 지도 위 자세가 없다 — 스택을 저장 지도로 켰나?'); return 2
        # 11-28: 스택을 `start`(주행기 서버 없음)로 켜고 돌렸다 → 목표가 전부 "주행기 액션 서버 없음"으로 실패했는데 화면 48장은 올라갔다.
        #   주행기 서버가 없으면 기록도 시작하지 않는다(`MAP=… tools/nav_test_stack.sh start explore`로 켜야 뜬다)
        procs = subprocess.run(['ps', '-eo', 'args'], capture_output=True, text=True, timeout=5).stdout
        if 'step_nav_action_server.py' not in procs:
            print('❌ 주행기 서버(step_nav_action_server)가 없다 — `MAP=… tools/nav_test_stack.sh start explore`로 켤 것'); return 2
        route = [byname[n] for n in a.places] if a.places else order_nearest((p0['x'], p0['y']), allp)
        session = 'tour_' + time.strftime('%Y%m%d_%H%M%S')
        out = os.path.expanduser('~/ar_scan/' + session); os.makedirs(out, exist_ok=True)
        up = None
        if not a.no_upload:
            up = Uploader(session); up.start()
        mlog = open(os.path.join(out, 'missions.log'), 'ab')
        bag = None
        if not a.no_bag:
            bag = subprocess.Popen(['ros2', 'bag', 'record', '-s', 'mcap', '-o', os.path.join(out, 'bag')] + TOPICS,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        rec = Recorder(state, out, a.frame_s, up); rec.start()
        print(f'세션 {session} — 기록: {out}' + ('' if up else ' (올리기 끔)') + f'\n순서: {" → ".join(p[0] for p in route)}', flush=True)
        res = []; fails = 0; t_all = time.time()
        try:
            for name, x, y in route:
                rec.place = name; t0 = time.time()
                ok = mission(f'{name}:{x:.2f},{y:.2f},{PLACE_AREA_M:.2f}', mlog)
                p = state.pose() or {}
                r = dict(place=name, ok=ok, secs=round(time.time() - t0, 1),
                         to_point_cm=round(math.hypot(x - p.get('x', math.nan), y - p.get('y', math.nan)) * 100, 1), looks=[])
                if ok and not a.no_look:
                    for k in (1, 2, 3, 4):                      # 지금 자리에서 90°씩 — 네 번째로 처음 방향에 돌아온다
                        q = state.pose()
                        if not q:
                            break
                        yaw = q['yaw_deg'] + 90
                        r['looks'].append(mission(f'{name}-둘러보기{k}:{q["x"]:.2f},{q["y"]:.2f},,{yaw:.1f}', mlog))
                        time.sleep(1.0)                          # 선 채로 몇 장 더(흔들림 없는 화면)
                res.append(r)
                print(f'{time.strftime("%H:%M:%S")} {name}: {"도착" if ok else "실패"} {r["secs"]:.0f}초, 점까지 {r["to_point_cm"]:.0f} cm, '
                      f'둘러보기 {sum(r["looks"])}/{len(r["looks"])}, 화면 {rec.n}장', flush=True)
                fails = 0 if ok else fails + 1
                if fails >= 2:
                    print('두 번 연속 실패 — 멈춤', flush=True); break
            if not a.no_return and fails < 2:
                rec.place = '출발 자리'
                back = mission(f'출발자리:{p0["x"]:.2f},{p0["y"]:.2f}', mlog)
                print(f'출발 자리로: {"도착" if back else "실패"}', flush=True)
        finally:
            rec.stop.set(); rec.join(10)
            if up:
                up.finish()
            if bag:
                bag.send_signal(signal.SIGINT)
                try:
                    bag.wait(20)
                except subprocess.TimeoutExpired:
                    bag.kill()
            json.dump(dict(session=session, upload=None if not up else dict(ok=up.ok, fail=up.fail, dropped=up.dropped, last_err=up.last_err), start=p0, route=[p[0] for p in route], results=res, frames=rec.n, frame_fail=rec.fail,
                           secs=round(time.time() - t_all, 1), map=os.path.basename(os.readlink(os.path.expanduser('~/maps/active_places.json')))
                           if os.path.islink(os.path.expanduser('~/maps/active_places.json')) else None,
                           camera_geometry='docs/calibration/camera_geometry_20261125.json'),
                      open(os.path.join(out, 'tour.json'), 'w'), ensure_ascii=False, indent=1)
            print(f'끝: 장소 {sum(r["ok"] for r in res)}/{len(route)}, 화면 {rec.n}장(실패 {rec.fail})'
                  + (f', 올림 {up.ok}장(실패 {up.fail}, 버림 {up.dropped})' if up else '') + f', {(time.time() - t_all) / 60:.1f}분 → {out}', flush=True)
    finally:
        state.close()
    return 0 if res and all(r['ok'] for r in res) else 1


if __name__ == '__main__':
    sys.exit(main())
