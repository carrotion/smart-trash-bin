#!/usr/bin/env python3
""""벽 줄 틈 너머 얕은 빈칸"을 탐색 경계로 치지 않는 규칙을 녹화로 시험한다 (2026-11-14, 설계 시험 — 탐색 코드에는 아직 안 넣음).

배경(docs/motion_foundation.md): 9번째 탐색의 옷장 문 너머 목표는 곧은 벽 줄(문 면)에 난 틈 너머 8~16 cm였다. 비스듬히 보면 레이저가 덮개를 지나
14~21 cm 안쪽에서 돌아와 문 면 뒤에 얕은 빈칸 띠가 생긴다. 진짜 열린 문이면 틈 너머로 빈칸이 방 안까지 깊게 이어진다.
"확인된 벽"(tools/confirmed_wall_replay.py)은 그 문 면을 정면에서 본 적이 없어 못 막았다.

규칙(안), 경계 칸 f마다:
  1. f에서 LINE_R 안의 벽 칸으로 곧은 선을 맞춘다(RANSAC, 선에서 2.5 cm 안 칸이 LINE_MIN개 이상 = 길이 약 LINE_MIN×5 cm). 없으면 규칙 밖(남김).
  2. 로봇과 f가 그 선의 서로 반대쪽이고, f가 선에서 NEAR 안이면 "벽 줄 너머"다.
  3. f와 이어진 빈칸(f에서 1 m 안) 중 선 너머에 있는 것이 선에서 가장 멀리 간 거리 = 깊이. 깊이 < DEPTH면 얕은 틈 → 경계에서 뺀다.
판정: 목표 14개마다 보낸 순간의 /map에서, 목표 둘레 0.3 m의 닿는 경계 칸이 규칙 뒤에도 남는가(옷장 너머 3번은 0, 다른 11개는 그대로여야).
그리고 모든 /map 스냅샷(30초마다)에서 규칙이 뺀 경계 덩어리를 전부 늘어놓아 다른 곳(열린 방문 등)을 잘못 뺐는지 그림으로 본다.

    python3 tools/shallow_frontier_replay.py bags/20261114_104750_explore9 --server docs/calibration/step_nav_server_20261114_104731.jsonl --out DIR
"""
import argparse, bisect, collections, json, math, os, sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from explore_map_filter import reachable
from confirmed_wall_replay import GOALS

LINE_R, LINE_TOL, NEAR = 0.6, 0.025, 0.4


def read_maps(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import OccupancyGrid
    from tf2_msgs.msg import TFMessage
    meta = open(os.path.join(bag, 'metadata.yaml')).read(); rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap' if 'storage_identifier: mcap' in meta else 'sqlite3'), rosbag2_py.ConverterOptions('', ''))
    rd.set_filter(rosbag2_py.StorageFilter(topics=['/map']))
    out = []
    while rd.has_next():
        _, data, tns = rd.read_next(); m = deserialize_message(data, OccupancyGrid)
        out.append((tns * 1e-9, np.frombuffer(bytes(m.data), np.int8).reshape(m.info.height, m.info.width).copy(),
                    (m.info.resolution, m.info.origin.position.x, m.info.origin.position.y)))
    return out


def frontier_mask(g, info, start, zones):
    res, ox, oy = info; reach, keep = reachable(g, res, ox, oy, start, zones)
    free = (g >= 0) & (g < 65); unk = g < 0; nb = np.zeros(g.shape, bool)
    nb[1:, :] |= unk[:-1, :]; nb[:-1, :] |= unk[1:, :]; nb[:, 1:] |= unk[:, :-1]; nb[:, :-1] |= unk[:, 1:]
    return free & nb & (keep if keep is not None else True)


def fit_line(P, rng, iters=60):
    """점 P(n×2)에 RANSAC 선: (점 하나, 단위 법선, 안쪽 점 수)."""
    best = (None, None, 0)
    for _ in range(iters):
        i, j = rng.choice(len(P), 2, replace=False); d = P[j] - P[i]; L = np.hypot(*d)
        if L < 0.1:
            continue
        n = np.array([-d[1], d[0]]) / L; k = int((np.abs((P - P[i]) @ n) < LINE_TOL).sum())
        if k > best[2]:
            best = (P[i], n, k)
    return best


def shallow(g, info, f_ij, robot, depth_min, line_min, rng):
    """경계 칸 f가 '벽 줄 틈 너머 얕은 빈칸'이면 (True, 깊이), 규칙 밖이면 (False, None), 깊으면 (False, 깊이)."""
    res, ox, oy = info; j, i = f_ij; fx, fy = ox + (i + 0.5) * res, oy + (j + 0.5) * res; k = int(LINE_R / res)
    j0, i0 = max(0, j - k), max(0, i - k); sub = g[j0:j + k + 1, i0:i + k + 1]
    oj, oi = np.nonzero(sub >= 65)
    if len(oj) < line_min:
        return False, None
    P = np.c_[ox + (oi + i0 + 0.5) * res, oy + (oj + j0 + 0.5) * res]
    P = P[np.hypot(P[:, 0] - fx, P[:, 1] - fy) <= LINE_R]
    if len(P) < line_min:
        return False, None
    p0, n, cnt = fit_line(P, rng)
    if p0 is None or cnt < line_min:
        return False, None
    sf = (np.array([fx, fy]) - p0) @ n; sr = (np.array(robot) - p0) @ n
    if sf * sr >= 0 or abs(sf) > NEAR:                 # 같은 쪽이거나 선에서 멀다
        return False, None
    sgn = np.sign(sf)
    # f와 이어진 빈칸(1 m 안, 선 너머 쪽)으로 번져 선에서 가장 먼 거리
    free = (g >= 0) & (g < 65); kk = int(1.0 / res); J0, I0 = max(0, j - kk), max(0, i - kk)
    F = free[J0:j + kk + 1, I0:i + kk + 1].copy(); h, w = F.shape
    yy, xx = np.mgrid[0:h, 0:w]; X = ox + (xx + I0 + 0.5) * res; Y = oy + (yy + J0 + 0.5) * res
    side = sgn * ((X - p0[0]) * n[0] + (Y - p0[1]) * n[1]); F &= side > 0
    seed = (j - J0, i - I0)
    if not F[seed]:
        return True, abs(sf)
    seen = np.zeros_like(F); seen[seed] = True; q = collections.deque([seed]); dmax = 0.0
    while q:
        a, b = q.popleft(); dmax = max(dmax, side[a, b])
        for da in (-1, 0, 1):
            for db in (-1, 0, 1):
                c, d = a + da, b + db
                if 0 <= c < h and 0 <= d < w and F[c, d] and not seen[c, d]:
                    seen[c, d] = True; q.append((c, d))
    return dmax < depth_min, dmax


def filtered_frontier(g, info, start, zones, robot, depth_min, line_min, seed=0):
    """규칙을 적용한 경계 칸, 뺀 칸, 뺀 칸마다 (x, y, 깊이)."""
    rng = np.random.default_rng(seed); fr = frontier_mask(g, info, start, zones); drop = np.zeros_like(fr); info_drop = []
    res, ox, oy = info
    for j, i in np.argwhere(fr):
        s, d = shallow(g, info, (j, i), robot, depth_min, line_min, rng)
        if s:
            drop[j, i] = True; info_drop.append((ox + (i + 0.5) * res, oy + (j + 0.5) * res, d))
    return fr & ~drop, drop, info_drop


def near_count(mask, info, goal, rad=0.3):
    res, ox, oy = info; i, j = int((goal[0] - ox) / res), int((goal[1] - oy) / res); k = int(rad / res)
    return int(mask[max(0, j - k):j + k + 1, max(0, i - k):i + k + 1].sum())


def robot_at(tf_poses, t):
    T = [p[0] for p in tf_poses]; k = max(0, bisect.bisect(T, t) - 1); return tf_poses[k][1:]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag'); ap.add_argument('--server', required=True)
    ap.add_argument('--depth', default='0.3'); ap.add_argument('--line-min', default='8')
    ap.add_argument('--zones', default=os.path.expanduser('~/maps/home_20261114_explore9.keepout.json.bak_20261114_1349'))
    ap.add_argument('--every', type=float, default=30.0, help='경계 덩어리 조사용 /map 간격(초)')
    ap.add_argument('--out')
    a = ap.parse_args()
    import keepout
    from confirmed_wall_replay import read_bag
    scans, maps = read_bag(a.bag)                       # 로봇 자세(그때의 map 좌표)를 함께 쓰려고 스캔도 읽는다
    poses = [(s[0], s[1][0], s[1][1]) for s in scans]
    zones = keepout.load(a.zones)['zones'] if a.zones and os.path.exists(a.zones) else []
    goals = [(r['t'] - r['result']['secs'], r['start'][:2], r['goal'][:2], r['result']['msg']) for r in map(json.loads, open(a.server))]
    mt = [m[0] for m in maps]; start0 = poses[0][1:]
    wardrobe = [min(math.hypot(gl[0] - x, gl[1] - y) for x, y in GOALS) < 0.05 for _, _, gl, _ in goals]
    rows = []
    for depth, lm in [(float(d), int(l)) for d in a.depth.split(',') for l in a.line_min.split(',')]:
        print(f'== 깊이 < {depth:.2f} m, 벽 줄 ≥ {lm}칸')
        res_goals = []
        for (t, st, gl, msg), wd in zip(goals, wardrobe):
            _, g, info = maps[max(0, bisect.bisect(mt, t) - 1)]
            keep, drop, dl = filtered_frontier(g, info, st, zones, st, depth, lm)
            n0 = near_count(keep | drop, info, gl); n1 = near_count(keep, info, gl)
            ds = [d for x, y, d in dl if math.hypot(x - gl[0], y - gl[1]) <= 0.3]
            res_goals.append((gl, wd, n0, n1))
            print(f'  ({gl[0]:+.2f},{gl[1]:+.2f}) 경계 칸 {n0:3d} → {n1:3d}  {"[옷장 너머] " if wd else ""}' + (f'뺀 칸 깊이 {min(ds):.2f}~{max(ds):.2f} m ' if ds else '') + f'실제: {msg[:20]}')
        survey = []; t_last = -1e9
        for t, g, info in maps:
            if t - t_last < a.every:
                continue
            t_last = t; rb = robot_at(poses, t)
            _, drop, dl = filtered_frontier(g, info, start0, zones, rb, depth, lm)
            for x, y, d in dl:
                survey.append((round(t - maps[0][0]), round(x, 2), round(y, 2), round(d, 2), [round(v, 2) for v in rb]))
        spots = collections.OrderedDict()
        for t, x, y, d, rb in survey:                    # 0.5 m 칸으로 묶어 자리별로
            spots.setdefault((round(x * 2) / 2, round(y * 2) / 2), []).append((t, d))
        print(f'  30초마다 /map {len([1 for t, *_ in maps])}장 중 조사, 뺀 경계 칸이 나온 자리(0.5 m로 묶음):')
        for (x, y), v in spots.items():
            print(f'    ({x:+.1f},{y:+.1f}) {len(v):3d}번, 처음 {v[0][0]} s, 깊이 {min(d for _, d in v):.2f}~{max(d for _, d in v):.2f} m')
        rows.append(dict(depth=depth, line_min=lm,
                         goals=[dict(goal=gl, wardrobe=wd, before=n0, after=n1) for gl, wd, n0, n1 in res_goals],
                         dropped_spots=[dict(x=x, y=y, n=len(v), first_s=v[0][0]) for (x, y), v in spots.items()]))
    if a.out:
        os.makedirs(a.out, exist_ok=True); json.dump(rows, open(os.path.join(a.out, 'summary.json'), 'w'), ensure_ascii=False, indent=1)
        save_image(a.out, maps[-1], rows[0], goals)
        print('저장:', a.out)


def save_image(out, last, row, goals):
    """마지막 /map 위에 규칙이 경계를 뺀 자리(빨강 원)와 목표(초록 + 도착, 보라 + 옷장 너머, 주황 + 그 밖 실패)."""
    import cv2
    _, g, (res, ox, oy) = last; h, w = g.shape
    c = np.full((h, w, 3), 128, np.uint8); c[g == 0] = 255; c[g >= 65] = 0; c = cv2.resize(c[::-1].copy(), None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)

    def px(x, y):
        return int((x - ox) / res * 3), int((h - 1 - (y - oy) / res) * 3)
    for s in row['dropped_spots']:
        cv2.circle(c, px(s['x'], s['y']), 10, (0, 0, 255), 2)
    for (t, st, gl, msg), gr in zip(goals, row['goals']):
        col = (255, 0, 255) if gr['wardrobe'] else ((0, 170, 0) if msg.startswith('도착') else (0, 140, 255))
        cv2.drawMarker(c, px(*gl), col, cv2.MARKER_CROSS, 14, 2)
    cv2.imwrite(os.path.join(out, 'dropped.png'), c)


if __name__ == '__main__':
    main()
