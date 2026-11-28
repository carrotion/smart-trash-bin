#!/usr/bin/env python3
"""주행 AR 확인 화면(A: 카메라 위 덧그리기) — 로봇 쪽 기준 화면 (2026-11-28).

깔끔한 카메라 화면(왜곡 폄·롤 바로잡음, camera_detect.rectify) 위에 그린다:
  ① 갈 길: 계획 경로(~/.local/state/smart-trash-bin/path.json)를 바닥 위 띠(로봇 폭)로, 목적지는 바닥 고리로
  ② 라이다 점: 라이다 높이(8.5 cm)의 점을 제자리에 — 점이 벽·가구 밑단에 붙어 있으면 투영이 맞다는 뜻(맞춤 확인용)
  ③ 인식 상자: 원본 화면의 상자를 깔끔한 화면 좌표로 옮기고, 거리는 상자 방향의 라이다 점으로(카메라 줄로 재지 않음)
  ④ 상태(모드·목적지·남은 거리·배터리·지연)는 화면 위가 아니라 웹 페이지 글자로(/api/ar) — 한글 글꼴은 브라우저가 낫다
관제소(서버 쪽)가 최종 화면을 그릴 때 이 화면을 기준(정답 그림)으로 쓴다. 투영식은 아래 project()가 정본이다.

    python3 tools/ar_view.py                  # http://<로봇>:8770/
    python3 tools/ar_view.py --port 8770 --fps 8

투영(로봇 기준 앞 X, 왼쪽 Y, 위 Z, 카메라는 몸통 가운데 높이 h, 위로 θ 든 핀홀):
    z_c = X·cosθ + (Z−h)·sinθ,  y_c = X·sinθ − (Z−h)·cosθ,  x_c = −Y
    u = cx + f·x_c/z_c,  v = cy + f·y_c/z_c   (z_c > 0.05일 때만)
"""
import argparse, json, math, os, sys, threading, time, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_detect as C

CAM = 'http://127.0.0.1:8767'
PATH_FILE = os.path.expanduser('~/.local/state/smart-trash-bin/path.json')
PLACES = os.path.expanduser('~/maps/active_places.json')
LX, LIDAR_Z = 0.054, 0.085          # base_link → 라이다(앞 5.4 cm, 높이 8.5 cm — hardware_inventory)
BAND_HALF = 0.16                    # 경로 띠 반폭(m) — 몸통 폭 약 0.32 m

G = C.rect_geom()
TH = math.radians(G['pitch_up_deg'])
K0 = np.array([[C.GEOM.get('fx', C.GEOM['f']), 0, C.GEOM['cx']], [0, C.GEOM['f'], C.GEOM['cy']], [0, 0, 1.0]])
KN = np.array([[G['fx'], 0, G['cx']], [0, G['f'], G['cy']], [0, 0, 1.0]])
_t = math.radians(C.ROLL_FIX_DEG)
RZ = np.array([[math.cos(_t), -math.sin(_t), 0], [math.sin(_t), math.cos(_t), 0], [0, 0, 1.0]])
DIST = np.array(C.GEOM.get('dist') or [0] * 5, float)


def project(X, Y, Z=0.0):
    """로봇 기준 점 → 깔끔한 화면 (u, v) 실수. 카메라 뒤면 None."""
    h = G['h']
    zc = X * math.cos(TH) + (Z - h) * math.sin(TH)
    if zc <= 0.05:
        return None
    yc = X * math.sin(TH) - (Z - h) * math.cos(TH)
    return G['cx'] + G['fx'] * (-Y) / zc, G['cy'] + G['f'] * yc / zc


def to_robot(px, py, pose):
    th = math.radians(pose['yaw_deg']); dx, dy = px - pose['x'], py - pose['y']
    return dx * math.cos(th) + dy * math.sin(th), -dx * math.sin(th) + dy * math.cos(th)


def raw_box_to_rect(q):
    """원본 화면 상자 → 깔끔한 화면 상자(네 귀퉁이를 옮겨 감싸는 사각형)."""
    pts = np.array([[[q['x1'], q['y1']]], [[q['x2'], q['y1']]], [[q['x1'], q['y2']]], [[q['x2'], q['y2']]]], np.float64)
    r = cv2.undistortPoints(pts, K0, DIST, R=RZ, P=KN).reshape(-1, 2)
    return r[:, 0].min(), r[:, 1].min(), r[:, 0].max(), r[:, 1].max()


class State:
    def __init__(self):
        import server_link as S
        self.rs = S.RobotState()
        self.lock = threading.Lock(); self.jpg = None; self.n = 0; self.info = {}
        self.lat = []          # (찍힘 → 그림 완성) ms
        self.places = []
        try:
            self.places = [(p['name'], p['x'], p['y']) for p in json.load(open(PLACES))['places']]
        except Exception:
            pass

    def path(self):
        try:
            d = json.load(open(PATH_FILE))
            if time.time() - d.get('t', 0) < 5.0:
                return d.get('goal'), d.get('path') or []
        except Exception:
            pass
        return None, []

    def battery(self):
        try:
            b = json.load(urllib.request.urlopen('http://127.0.0.1:8769/api/overview', timeout=1)).get('battery', {})
            return b.get('voltage')
        except Exception:
            return None

    def dets(self):
        try:
            d = json.load(urllib.request.urlopen(CAM + '/api/detect?peek=1', timeout=1))
            return d.get('dets', []) if d.get('age_s', 9) < 1.5 else []
        except Exception:
            return []

    def frame(self):
        r = urllib.request.urlopen(CAM + '/camera.jpg?raw=1', timeout=3)
        t_fr = float(r.headers.get('X-Frame-T') or 0) or None
        img = cv2.imdecode(np.frombuffer(r.read(), np.uint8), cv2.IMREAD_COLOR)
        return img, t_fr

    def draw(self, img, pose, scan, goal, path, dets):
        out = C.rectify(img)
        H, W = out.shape[:2]
        info = {'pose': pose, 'goal': None, 'remaining_m': None, 'dets': []}
        # ② 라이다 점(로봇 기준, 라이다 높이)
        pts_lidar = []
        if scan is not None:
            r = np.asarray(scan.ranges, float); a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
            ok = np.isfinite(r) & (r > 0.12) & (r < 8.0)
            X = LX + r[ok] * np.cos(a[ok]); Y = r[ok] * np.sin(a[ok])
            pts_lidar = np.c_[X, Y]
            for x, y in pts_lidar[::2]:
                p = project(x, y, LIDAR_Z)
                if p and 0 <= p[0] < W and 0 <= p[1] < H:
                    d = math.hypot(x, y); c = (0, int(min(255, 60 + d * 50)), int(max(0, 255 - d * 60)))
                    cv2.circle(out, (int(p[0]), int(p[1])), 2, c, -1, cv2.LINE_AA)
        # ① 갈 길 띠 + 목적지 고리
        if pose and path:
            rel = [to_robot(px, py, pose) for px, py in path]
            dense = []
            for (x1, y1), (x2, y2) in zip(rel, rel[1:]):
                n = max(1, int(math.hypot(x2 - x1, y2 - y1) / 0.1))
                dense += [(x1 + (x2 - x1) * k / n, y1 + (y2 - y1) * k / n) for k in range(n)]
            dense.append(rel[-1])
            info['remaining_m'] = round(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(rel, rel[1:])), 2)
            ov = out.copy(); drawn = 0
            for (x1, y1), (x2, y2) in zip(dense, dense[1:]):
                if min(x1, x2) < 0.3:
                    continue
                L = math.hypot(x2 - x1, y2 - y1) or 1e-6; nx, ny = -(y2 - y1) / L * BAND_HALF, (x2 - x1) / L * BAND_HALF
                q = [project(x1 + nx, y1 + ny), project(x2 + nx, y2 + ny), project(x2 - nx, y2 - ny), project(x1 - nx, y1 - ny)]
                if all(q):
                    cv2.fillPoly(ov, [np.int32(q)], (40, 170, 255), cv2.LINE_AA); drawn += 1     # BGR 주황(11-28 첫 시험은 RGB로 넣어 파랗게 나왔다)
            cv2.addWeighted(ov, 0.55, out, 0.45, 0, out)
            if goal:
                gx, gy = to_robot(goal[0], goal[1], pose)
                ring = [project(gx + 0.25 * math.cos(k), gy + 0.25 * math.sin(k)) for k in np.linspace(0, 2 * math.pi, 40)]
                if all(ring):
                    cv2.polylines(out, [np.int32(ring)], True, (120, 220, 255), 2, cv2.LINE_AA)
                nm = min(self.places, key=lambda p: math.hypot(p[1] - goal[0], p[2] - goal[1]), default=None)
                info['goal'] = nm[0] if nm and math.hypot(nm[1] - goal[0], nm[2] - goal[1]) < 0.6 else f'({goal[0]:.1f}, {goal[1]:.1f})'
            info['band_segments'] = drawn
        # ③ 인식 상자 — 거리는 상자 방향 라이다 점으로
        for q in dets:
            if q.get('score', 0) < 0.45:
                continue
            x1, y1, x2, y2 = raw_box_to_rect(q)
            dist = None
            if len(pts_lidar):
                b1 = math.atan((G['cx'] - (x1 + 0.2 * (x2 - x1))) / G['fx']); b2 = math.atan((G['cx'] - (x2 - 0.2 * (x2 - x1))) / G['fx'])
                brg = np.arctan2(pts_lidar[:, 1], pts_lidar[:, 0]); sel = pts_lidar[(brg <= b1) & (brg >= b2)]
                if len(sel):
                    dd = np.hypot(sel[:, 0], sel[:, 1]); dist = float(np.median(dd[dd <= dd.min() + 0.1]))
            col = (80, 220, 80) if q.get('name') == 'person' else (220, 200, 80)
            cv2.rectangle(out, (int(x1), int(y1)), (int(x2), int(y2)), col, 2, cv2.LINE_AA)
            if dist:
                cv2.putText(out, f'{dist:.1f}m', (int(x1) + 3, int(y1) + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
            info['dets'].append({'name': q.get('name'), 'score': round(q.get('score', 0), 2), 'dist_m': None if dist is None else round(dist, 2)})
        self.inset(out, pts_lidar, pose, path, goal)
        return out, info

    def inset(self, out, pts_lidar, pose, path, goal, size=150, rng=4.0):
        """오른쪽 아래 작은 위에서 본 창(로봇 기준, 앞이 위, 반경 4 m). 이 카메라 각도에선 바닥이 3 m 앞부터라
        가까운 길은 카메라 화면에 안 나온다(11-28 시험: 띠가 화면 맨 아래 몇 줄에만 보임) — 그 부분을 여기서 보인다."""
        H, W = out.shape[:2]; x0, y0 = W - size - 8, H - size - 8
        roi = out[y0:y0 + size, x0:x0 + size]; roi[:] = (roi * 0.35).astype(np.uint8)
        k = size / (2 * rng); cx, cy = size // 2, int(size * 0.8)
        def P(x, y):
            return int(cx - y * k), int(cy - x * k)
        for x, y in (pts_lidar[::2] if len(pts_lidar) else []):
            u, v = P(x, y)
            if 0 <= u < size and 0 <= v < size:
                cv2.circle(roi, (u, v), 1, (80, 230, 80), -1)
        if pose and path:
            rel = [P(*to_robot(px, py, pose)) for px, py in path]
            cv2.polylines(roi, [np.int32(rel)], False, (40, 170, 255), max(2, int(2 * BAND_HALF * k)), cv2.LINE_AA)
            if goal:
                cv2.circle(roi, P(*to_robot(goal[0], goal[1], pose)), 5, (120, 220, 255), 2, cv2.LINE_AA)
        cv2.fillPoly(roi, [np.int32([(cx, cy - 7), (cx - 5, cy + 5), (cx + 5, cy + 5)])], (255, 255, 255))
        # 카메라가 보는 바닥 범위(3.07 m부터, 좌우 ±19°)
        a = math.atan(G['cx'] / G['fx'])
        for s_ in (-1, 1):
            cv2.line(roi, P(3.07, s_ * 3.07 * math.tan(a)), P(rng * 1.2, s_ * rng * 1.2 * math.tan(a)), (200, 200, 200), 1, cv2.LINE_AA)
        cv2.rectangle(out, (x0, y0), (x0 + size - 1, y0 + size - 1), (200, 200, 200), 1)

    def loop(self, fps):
        while True:
            t0 = time.time()
            try:
                img, t_fr = self.frame()
                pose = self.rs.pose(); goal, path = self.path()
                out, info = self.draw(img, pose, self.rs.scan_msg, goal, path, self.dets())
                ok, buf = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 85])
                t1 = time.time()
                if t_fr:
                    self.lat = (self.lat + [(t1 - t_fr) * 1000])[-50:]
                info.update(battery_v=self.battery(), moving=self.rs.moving(), t=round(t1, 3),
                            latency_ms=round(float(np.median(self.lat)), 0) if self.lat else None, draw_ms=round((t1 - t0) * 1000))
                with self.lock:
                    self.jpg = buf.tobytes(); self.n += 1; self.info = info
            except Exception as e:
                with self.lock:
                    self.info = dict(self.info, error=f'{type(e).__name__}: {e}')
                time.sleep(1.0)
            time.sleep(max(0.0, 1.0 / fps - (time.time() - t0)))


PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>주행 AR 확인</title><style>
:root{--bg:#0e1116;--fg:#e8edf2;--dim:#8b96a3;--acc:#ffb347}
body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.45 system-ui,sans-serif}
.wrap{max-width:960px;margin:0 auto;padding:12px 16px}
.view{position:relative;width:100%;aspect-ratio:4/3;background:#000;border-radius:10px;overflow:hidden}
.view img{width:100%;height:100%;display:block}
.hud{position:absolute;left:10px;top:10px;background:rgba(0,0,0,.55);padding:8px 12px;border-radius:8px;font-size:14px}
.hud b{color:var(--acc);font-size:17px}
.foot{position:absolute;right:10px;bottom:10px;background:rgba(0,0,0,.55);padding:4px 10px;border-radius:6px;font-size:12px;color:var(--dim)}
p{color:var(--dim);font-size:13px}
</style></head><body><div class="wrap">
<div class="view"><img src="/ar.mjpg" alt="주행 AR 카메라 화면"><div class="hud" id="hud">연결 중…</div><div class="foot" id="foot"></div></div>
<p>주황 띠 = 갈 길(바닥), 고리 = 목적지, 점 = 라이다(8.5 cm 높이), 상자 = 인식(거리는 라이다). 카메라에는 바닥이 약 3 m 앞부터 보여서, 가까운 길은 오른쪽 아래 위에서 본 작은 창(반경 4 m, 회색 선 = 카메라가 보는 바닥)에 보인다.</p>
</div><script>
async function tick(){try{const d=await (await fetch('/api/ar')).json();
const g=d.goal?`<b>${d.goal}</b>까지 ${d.remaining_m!=null?d.remaining_m.toFixed(1)+' m':''}`:'<b>대기</b>';
const ppl=(d.dets||[]).filter(x=>x.name==='person').length;
document.getElementById('hud').innerHTML=`${g}<br>${d.moving?'이동 중':'멈춤'} · 배터리 ${d.battery_v?d.battery_v.toFixed(2)+' V':'?'}${ppl?' · 사람 '+ppl:''}${d.pose?'':' · 위치 모름'}`;
document.getElementById('foot').textContent=`지연 ${d.latency_ms??'?'} ms · 그리기 ${d.draw_ms??'?'} ms${d.error?' · '+d.error:''}`;
}catch(e){document.getElementById('hud').textContent='연결 끊김'}}
setInterval(tick,500);tick();
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, default=8770); ap.add_argument('--fps', type=float, default=8.0)
    a = ap.parse_args()
    st = State()
    threading.Thread(target=st.loop, args=(a.fps,), daemon=True).start()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *x):
            pass

        def _send(self, code, body, ctype):
            self.send_response(code); self.send_header('Content-Type', ctype); self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body))); self.end_headers(); self.wfile.write(body)

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self._send(200, PAGE.encode(), 'text/html; charset=utf-8')
            elif self.path.startswith('/api/ar'):
                with st.lock:
                    self._send(200, json.dumps(st.info, ensure_ascii=False).encode(), 'application/json')
            elif self.path.startswith('/ar.jpg'):
                with st.lock:
                    jpg = st.jpg
                self._send(200, jpg, 'image/jpeg') if jpg else self._send(503, b'no frame', 'text/plain')
            elif self.path.startswith('/ar.mjpg'):
                self.send_response(200); self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store'); self.end_headers(); last = None
                try:
                    while True:
                        with st.lock:
                            jpg, n = st.jpg, st.n
                        if jpg and n != last:
                            last = n
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
                            self.wfile.flush()
                        time.sleep(0.03)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self._send(404, b'not found', 'text/plain')

    srv = ThreadingHTTPServer(('0.0.0.0', a.port), H)
    print(f'주행 AR 확인 화면: http://0.0.0.0:{a.port}/', flush=True)
    try:
        srv.serve_forever()
    finally:
        st.rs.close()


if __name__ == '__main__':
    main()
