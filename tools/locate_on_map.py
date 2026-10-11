#!/usr/bin/env python3
"""저장된 지도에서 "지금 로봇이 어디인가"를 스캔 한 장으로 찾는다 (2026-10-11).

왜: 저장한 지도로 다시 시작하려면 시작 자세를 알려 줘야 하는데(slam_toolbox 위치 잡기는 0.5 m·25° 안이어야 제자리를 찾고
1 m 어긋나면 못 찾는다 — localize_replay.py), 사람이 놓은 자리는 그보다 어긋날 수 있고 방향은 통째로 반대일 수도 있다.
지금 스캔을 지도의 벽에 포개어 가장 잘 맞는 자세를 찾아 그 값을 시작 자세로 쓴다.

방법: 지도의 벽 칸까지 거리장을 만들고, 자세 후보(위치 격자 × 방향 한 바퀴)마다 스캔 점이 벽 가까이 떨어지는 정도를 점수로 매긴다
(점마다 exp(−(거리/7 cm)²)의 평균). 거칠게(10 cm·3°) 훑고 가장 좋은 몇 곳을 촘촘히(2 cm·0.5°) 다시 본다.
⚠️ 비슷하게 생긴 자리가 여러 곳이면 틀린 곳을 고를 수 있다 → 둘째로 좋은 자리와의 점수 차를 같이 낸다. 찾는 범위(--around, --radius)를
   아는 만큼 좁혀 줄 것. 결과는 추정이다 — 사람이 지도 화면에서 한 번 본다.

    python3 tools/locate_on_map.py --map ~/maps/home_20261011 [--around 0,0 --radius 1.5]     # 스캔은 관제 대시보드(8769)에서 읽는다
"""
import argparse, json, math, os, sys, urllib.request

import numpy as np


def load_map(base):
    y = dict(l.split(':', 1) for l in open(base + '.yaml').read().splitlines() if ':' in l)
    res = float(y['resolution']); ox, oy = (float(v) for v in y['origin'].strip(' []').split(',')[:2])
    raw = open(base + '.pgm', 'rb').read(); parts = raw.split(None, 4); w, h = int(parts[1]), int(parts[2])
    img = np.frombuffer(parts[4][-w * h:], np.uint8).reshape(h, w)[::-1]        # 행 0 = 지도 아래쪽(y 작은 쪽)
    return img, res, ox, oy


def dist_field(occ, res, rmax=0.5):
    """벽 칸까지 거리(m), rmax에서 자름."""
    h, w = occ.shape; D = np.full((h, w), rmax, np.float32); k = int(rmax / res)
    for dj in range(-k, k + 1):
        for di in range(-k, k + 1):
            d = math.hypot(di, dj) * res
            if d >= rmax:
                continue
            src = occ[max(0, -dj):h - max(0, dj), max(0, -di):w - max(0, di)]
            dst = D[max(0, dj):h - max(0, -dj), max(0, di):w - max(0, -di)]
            np.minimum(dst, np.where(src, d, rmax), out=dst)
    return D


def score_grid(D, res, ox, oy, pts, xs, ys, yaws, sigma=0.07):
    """자세 격자의 점수 [yaw, y, x]."""
    h, w = D.shape; out = np.zeros((len(yaws), len(ys), len(xs)), np.float32)
    X, Y = np.meshgrid(xs, ys)
    for a, th in enumerate(yaws):
        c, s = math.cos(th), math.sin(th); px = c * pts[:, 0] - s * pts[:, 1]; py = s * pts[:, 0] + c * pts[:, 1]
        acc = np.zeros(X.shape, np.float32)
        for qx, qy in zip(px, py):
            i = ((X + qx - ox) / res).astype(np.int32); j = ((Y + qy - oy) / res).astype(np.int32)
            ok = (i >= 0) & (i < w) & (j >= 0) & (j < h)
            d = np.where(ok, D[np.clip(j, 0, h - 1), np.clip(i, 0, w - 1)], 0.5)
            acc += np.exp(-(d / sigma) ** 2)
        out[a] = acc / len(pts)
    return out


def locate(base, pts, around=(0.0, 0.0), radius=1.5):
    img, res, ox, oy = load_map(base); D = dist_field(img < 50, res)
    pts = np.array([p for p in pts if 0.25 < math.hypot(*p) < 8.0], float)
    if len(pts) < 40:
        raise SystemExit(f'❌ 쓸 수 있는 스캔 점이 너무 적다({len(pts)}개)')
    coarse = pts[::2]
    xs = np.arange(around[0] - radius, around[0] + radius + 1e-6, 0.10); ys = np.arange(around[1] - radius, around[1] + radius + 1e-6, 0.10)
    yaws = np.radians(np.arange(0, 360, 3.0)); S = score_grid(D, res, ox, oy, coarse, xs, ys, yaws)
    order = np.argsort(S.ravel())[::-1]; cands = []
    for idx in order[:4000]:                      # 서로 떨어진(0.4 m 또는 20°) 좋은 자리 몇 개
        a, j, i = np.unravel_index(idx, S.shape); p = (xs[i], ys[j], yaws[a])
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 0.4 or abs((p[2] - q[2] + math.pi) % (2 * math.pi) - math.pi) > math.radians(20) for q in cands):
            cands.append(p)
        if len(cands) >= 4:
            break
    fine = []
    for cx, cy, cth in cands:
        fx = np.arange(cx - 0.12, cx + 0.121, 0.02); fy = np.arange(cy - 0.12, cy + 0.121, 0.02); fa = cth + np.radians(np.arange(-4, 4.01, 0.5))
        F = score_grid(D, res, ox, oy, pts, fx, fy, fa); a, j, i = np.unravel_index(int(F.argmax()), F.shape)
        th = (fa[a] + math.pi) % (2 * math.pi) - math.pi; c, s = math.cos(th), math.sin(th)
        gi = ((fx[i] + c * pts[:, 0] - s * pts[:, 1] - ox) / res).astype(int); gj = ((fy[j] + s * pts[:, 0] + c * pts[:, 1] - oy) / res).astype(int)
        ok = (gi >= 0) & (gi < D.shape[1]) & (gj >= 0) & (gj < D.shape[0]); d = np.where(ok, D[np.clip(gj, 0, D.shape[0] - 1), np.clip(gi, 0, D.shape[1] - 1)], 0.5)
        fine.append(dict(x=float(fx[i]), y=float(fy[j]), yaw=float(th), score=float(F.max()), on_wall=float((d <= 0.10).mean()), n=len(pts)))
    fine.sort(key=lambda r: -r['score'])
    return fine


def whole_area(base):
    """지도에서 아는 범위 전체를 덮는 (가운데, 반경) — 어디 놓였는지 전혀 모를 때의 찾는 범위."""
    img, res, ox, oy = load_map(os.path.expanduser(base)); jj, ii = np.nonzero(img != 205)
    return (ox + (ii.min() + ii.max()) / 2 * res, oy + (jj.min() + jj.max()) / 2 * res), max(ii.max() - ii.min(), jj.max() - jj.min()) / 2 * res + 0.3


def scan_from_ros(topic='/scan', lidar_x=0.054, timeout=10.0):
    """/scan 한 장을 로봇 기준 점 목록으로(차체 안은 뺌)."""
    import rclpy, time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    rclpy.init(); n = rclpy.create_node('locate_on_map'); got = []
    n.create_subscription(LaserScan, topic, got.append, qos_profile_sensor_data)
    t0 = time.time()
    while not got and time.time() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.2)
    rclpy.shutdown()
    if not got:
        return None
    m = got[-1]; out = []
    for i, r in enumerate(m.ranges):
        if m.range_min <= r <= m.range_max:
            a = m.angle_min + i * m.angle_increment; x, y = lidar_x + r * math.cos(a), r * math.sin(a)
            if not (abs(x) <= 0.115 and abs(y) <= 0.115):
                out.append((x, y))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='저장된 지도 이름(확장자 없이)')
    ap.add_argument('--around', default='0,0'); ap.add_argument('--radius', type=float, default=1.5)
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--whole', action='store_true', help='지도에서 아는 범위 전체에서 찾는다(--around/--radius 무시)')
    ap.add_argument('--ros', action='store_true', help='관제 대시보드 대신 지금 ROS 도메인의 /scan을 직접 읽는다(시뮬, 대시보드가 없을 때)')
    a = ap.parse_args()
    if a.ros:
        pts = scan_from_ros()
    else:
        d = json.load(urllib.request.urlopen('http://localhost:8769/api/overview', timeout=5))
        pts = d['lidar']['pts'] if d['lidar'].get('ok') else None
    if not pts:
        print('❌ 스캔이 안 들어온다'); return 2
    around, radius = tuple(float(v) for v in a.around.split(',')), a.radius
    if a.whole:
        around, radius = whole_area(a.map)
    r = locate(os.path.expanduser(a.map), pts, around, radius)
    if a.json:
        print(json.dumps(r)); return 0
    b = r[0]
    print(f'지도 위 자세 추정: ({b["x"]:+.2f}, {b["y"]:+.2f}) m, {math.degrees(b["yaw"]):+.1f}°  — 스캔 점 {b["n"]}개 중 {b["on_wall"] * 100:.0f}%가 지도의 벽 10 cm 안, 점수 {b["score"]:.3f}')
    for k, c in enumerate(r[1:], 2):
        print(f'  {k}번째 후보: ({c["x"]:+.2f}, {c["y"]:+.2f}) m, {math.degrees(c["yaw"]):+.1f}° — 벽 위 {c["on_wall"] * 100:.0f}%, 점수 {c["score"]:.3f}')
    if len(r) > 1 and r[1]['score'] > 0.85 * b['score']:
        print('⚠️ 둘째 후보와 점수 차가 작다 — 비슷하게 생긴 자리일 수 있으니 지도 화면에서 확인할 것')
    return 0


if __name__ == '__main__':
    sys.exit(main())
