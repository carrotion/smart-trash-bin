#!/usr/bin/env python3
""""정면에서 확인된 벽" 규칙을 녹화로 시험한다 (2026-11-14, 설계 시험 — 주행 코드에는 아직 안 넣음).

배경(docs/motion_foundation.md "반사·투과면의 근본 대책"): 옷장 문은 비스듬히(입사 45°+) 보면 레이저가 덮개를 지나 안쪽에서 돌아와,
지나가며 볼 때마다 문 면 칸이 빈칸으로 지워졌다. 움직이는 물체는 시간에 따라, 반사·투과면은 각도에 따라 달라진다는 것을 쓴다.

규칙(덧씌우기): SLAM 지도는 건드리지 않는다. 따로 칸마다 센다 —
  Fh = 정면에 가깝게(입사 < θ) 벽으로 찍힌 횟수, Fp = 그 칸의 벽 방향에 정면으로 가깝게 빔이 지나간(빈칸이라고 한) 횟수.
  Fh ≥ N 이고 Fh ≥ ratio·(Fh+Fp) 이면 "확인된 벽" → 탐색기용 지도와 코스트맵에 벽으로 덧씌운다. 비스듬한 빔은 아무것도 바꾸지 않는다.
  정면에서 다시 비어 보이면(물건을 치움) Fp가 늘어 풀린다. 벽 방향은 같은 스캔 이웃 끝점으로 맞춘 선.
  --min-span: 이웃 끝점이 그 길이 이상 곧게 이어질 때만 정면 증거로 센다(다리·의자 다리처럼 작은 것은 굳히지 않으려는 것).
  스캔은 SLAM처럼 0.2 m / 0.2 rad / 0.5 s 넘게 움직였을 때만 센다 — N은 "서로 다른 자리에서 본 횟수"다.

  먼저 해 본 "확인된 벽을 비스듬히 지나는 빔을 끊기"(SLAM 지도 자체를 바꿈)는 9번째 탐색에서 정상 빔 19~22%를 끊어
  벽을 따라 빈칸이 모름으로 바뀌었다(빈칸→모름 731~1230칸) — 버림. 기록: docs/motion_foundation.md.

판정(녹화의 실제 /map을 그대로 쓴다):
  1. 목표마다 보낸 순간의 /map(+덧씌우기)에서 목표 둘레 0.3 m에 "닿을 수 있는 경계 칸"(탐색기 지도 필터를 지난 것)이 남았나.
     옷장 문 너머 목표는 0이 되어야 하고, 다른 목표는 그대로여야 한다.
  2. 마지막 지도에서 덧씌운 벽 칸이 어디에 얼마나 생겼나, 로봇이 설 수 있는 닿는 영역이 얼마나 줄었나(통로가 좁아지지 않았나).
  3. 사람 시험(--people K): 다리(지름 12 cm 둘, 20 cm 간격)를 K곳에 30초씩 넣었다 뺀다. 넣는 동안 로봇이 0.8~2.5 m에서 보고,
     빼고 60초 넘게 지나 다시 3 m 안으로 오는 빈 칸에서 고른다. 끝까지 확인된 벽으로 남은 곳의 수.

    python3 tools/confirmed_wall_replay.py bags/20261114_104750_explore9 --server docs/calibration/step_nav_server_20261114_104731.jsonl
    python3 tools/confirmed_wall_replay.py BAG --server … --theta 35,45 --count 2,3,5 --min-span 0,0.2 --people 20 --out DIR
"""
import argparse, bisect, collections, itertools, json, math, os, sys, warnings

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

LX = 0.054                     # base_link → base_laser (tf_static)
MAX_RANGE = 12.0               # mapper_params_online_async.yaml
KEY_D, KEY_A, KEY_T = 0.2, 0.2, 0.5

# 옷장 문 면(11-14 옷장 시험에서 맞춘 것): 첫 문짝 앞 자세, 문 면 법선 방향, 라이다에서 문 면까지
DOOR_ROBOT = (5.34, -1.88, math.radians(-55.0)); DOOR_D = 0.489
GOALS = ((6.68, -1.70), (7.03, -1.55))        # 9번째 탐색에서 왕복한 목표(문 너머)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from nav_msgs.msg import OccupancyGrid
    meta = open(os.path.join(bag, 'metadata.yaml')).read(); rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap' if 'storage_identifier: mcap' in meta else 'sqlite3'), rosbag2_py.ConverterOptions('', ''))
    rd.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/scan', '/map']))
    mo, ob, scans, maps = [], [], [], []
    while rd.has_next():
        tp, data, tns = rd.read_next()
        if tp == '/tf':
            for t in deserialize_message(data, TFMessage).transforms:
                st = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9; tr = t.transform.translation
                v = (st, tr.x, tr.y, yaw_of(t.transform.rotation))
                if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                    mo.append(v)
                elif t.header.frame_id == 'odom' and t.child_frame_id == 'base_link':
                    ob.append(v)
        elif tp == '/scan':
            m = deserialize_message(data, LaserScan)
            scans.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.angle_min, m.angle_increment, np.array(m.ranges, np.float32)))
        else:
            m = deserialize_message(data, OccupancyGrid)
            g = np.frombuffer(bytes(m.data), np.int8).reshape(m.info.height, m.info.width).copy()
            maps.append((tns * 1e-9, g, (m.info.resolution, m.info.origin.position.x, m.info.origin.position.y)))
    mo.sort(); ob.sort(); mt = [v[0] for v in mo]; bt = [v[0] for v in ob]

    def near(L, T, t):
        i = bisect.bisect(T, t); c = [j for j in (i - 1, i) if 0 <= j < len(L)]; j = min(c, key=lambda j: abs(T[j] - t)); return L[j], abs(T[j] - t)

    def comp(a, b):
        c, s = math.cos(a[2]), math.sin(a[2]); return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])

    out = []
    for st, a0, da, rg in scans:
        (m, dm), (o, do) = near(mo, mt, st), near(ob, bt, st)
        if do > 0.06 or dm > 5.0:
            continue
        out.append((st, comp(comp(m[1:], o[1:]), (LX, 0, 0)), a0, da, rg))
    return out, maps


def keyframes(scans):
    """SLAM이 쓰는 스캔만: 마지막 쓴 스캔에서 0.2 m 또는 0.2 rad 넘게, 0.5 s 넘게 지났을 때."""
    keep, last = [], None
    for s in scans:
        t, P = s[0], s[1]
        if last is None or (t - last[0] >= KEY_T and (math.hypot(P[0] - last[1][0], P[1] - last[1][1]) >= KEY_D
                                                     or abs(math.remainder(P[2] - last[1][2], math.tau)) >= KEY_A)):
            keep.append(s); last = s
    return keep


def beam_geometry(P, a0, da, rg, nb=6, gap=0.15):
    """끝점, 빔 방향, 이웃 끝점으로 맞춘 선의 법선 각, 그 선이 곧게 이어진 길이(m). 법선을 못 맞추면 nan."""
    n = len(rg); ang = P[2] + a0 + da * np.arange(n); r = rg.astype(np.float64)
    ok = np.isfinite(r) & (r > 0.02) & (r <= MAX_RANGE)
    x = np.where(ok, P[0] + r * np.cos(ang), np.nan); y = np.where(ok, P[1] + r * np.sin(ang), np.nan)
    K = np.arange(-nb, nb + 1)[:, None]; idx = (np.arange(n)[None, :] + K) % n
    X, Y = x[idx], y[idx]; near_ = np.isfinite(X) & (np.hypot(X - x, Y - y) < gap)
    cnt = near_.sum(0); mx = np.where(near_, X, 0).sum(0) / np.maximum(cnt, 1); my = np.where(near_, Y, 0).sum(0) / np.maximum(cnt, 1)
    dx, dy = np.where(near_, X - mx, 0), np.where(near_, Y - my, 0)
    cxx, cyy, cxy = (dx * dx).sum(0), (dy * dy).sum(0), (dx * dy).sum(0)
    line = 0.5 * np.arctan2(2 * cxy, cxx - cyy); nrm = line + math.pi / 2
    proj = np.where(near_, dx * np.cos(line) + dy * np.sin(line), np.nan)
    with np.errstate(all='ignore'), warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        span = np.nanmax(proj, 0) - np.nanmin(proj, 0)
        perp = np.where(near_, -dx * np.sin(line) + dy * np.cos(line), np.nan); rms = np.sqrt(np.nanmean(perp ** 2, 0))
    good = ok & (cnt >= 3) & (rms < 0.015)
    return ok, r, ang, x, y, np.where(good, nrm, np.nan), np.where(good, span, 0.0)


def add_people(scans, spots):
    """spots = [(x, y, t0, t1)] — 그 시간 동안 다리 둘(반지름 6 cm, 중심 간격 20 cm, 로봇을 마주 보게)을 스캔에 넣는다."""
    out = []
    for t, P, a0, da, rg in scans:
        rg2 = None
        for (px, py, t0, t1) in spots:
            if not (t0 <= t <= t1):
                continue
            if rg2 is None:
                rg2 = rg.copy(); ang = P[2] + a0 + da * np.arange(len(rg)); ux, uy = np.cos(ang), np.sin(ang)
            b = math.atan2(py - P[1], px - P[0])
            for side in (-0.1, 0.1):
                cx, cy = px - side * math.sin(b), py + side * math.cos(b)
                ox_, oy_ = cx - P[0], cy - P[1]; tc = ox_ * ux + oy_ * uy; d2 = ox_ * ox_ + oy_ * oy_ - tc * tc
                hit = (tc > 0) & (d2 < 0.06 ** 2); th = tc - np.sqrt(np.maximum(0.06 ** 2 - d2, 0))
                cur = np.where(np.isfinite(rg2) & (rg2 > 0.02), rg2, np.inf)
                rg2 = np.where(hit & (th < cur), th, rg2).astype(np.float32)
        out.append((t, P, a0, da, rg if rg2 is None else rg2))
    return out


def confirm_layer(geo, frame, rule, snaps=()):
    """확인된 벽 층. geo = [(t, (x, y), beam_geometry)], frame = (h, w, res, ox, oy), rule = (θ°, N, min_span, ratio).
    (마지막 bool 격자, {snaps 시각: 그때까지의 격자})"""
    h, w, res, ox, oy = frame; th, N, span_min, ratio = rule; cth = math.cos(math.radians(th))
    Fh = np.zeros(h * w, np.int32); Fp = np.zeros(h * w, np.int32); C2 = np.zeros(h * w, np.float32); S2 = np.zeros(h * w, np.float32)
    step = res * 0.5; snaps = sorted(snaps); out = {}

    def mask():
        return ((Fh >= N) & (Fh >= ratio * (Fh + Fp))).reshape(h, w)
    for t, (sx, sy), (ok, r, ang, x, y, nrm, span) in geo:
        while snaps and snaps[0] <= t:
            out[snaps.pop(0)] = mask()
        r_ok = r[ok]; a_ok = ang[ok]; nseg = np.ceil(r_ok / step).astype(np.int64)
        ray = np.repeat(np.arange(len(r_ok)), nseg); loc = np.arange(nseg.sum()) - (np.cumsum(nseg) - nseg)[ray]
        d = loc * step; ci = np.floor((sx + d * np.cos(a_ok[ray]) - ox) / res).astype(np.int64); cj = np.floor((sy + d * np.sin(a_ok[ray]) - oy) / res).astype(np.int64)
        cell = np.where((ci >= 0) & (ci < w) & (cj >= 0) & (cj < h), cj * w + ci, -1)
        ei, ej = np.floor((x[ok] - ox) / res).astype(np.int64), np.floor((y[ok] - oy) / res).astype(np.int64)
        e_in = (ei >= 0) & (ei < w) & (ej >= 0) & (ej < h); ecell = np.where(e_in, ej * w + ei, -1)
        first = np.r_[True, (cell[1:] != cell[:-1]) | (ray[1:] != ray[:-1])]
        passm = first & (cell >= 0) & (cell != ecell[ray]); c = cell[passm]
        seen = Fh[c] > 0
        cosi = np.abs(np.cos(a_ok[ray[passm]][seen] - 0.5 * np.arctan2(S2[c[seen]], C2[c[seen]])))
        np.add.at(Fp, c[seen][cosi >= cth], 1)                      # 정면에 가깝게 지나감 = 빈칸이라는 증거
        n_ok = nrm[ok]; cosi_e = np.abs(np.cos(a_ok - n_ok))
        fr = e_in & np.isfinite(n_ok) & (cosi_e >= cth) & (span[ok] >= span_min)
        ec = ecell[fr]; nf = n_ok[fr]
        np.add.at(Fh, ec, 1); np.add.at(C2, ec, np.cos(2 * nf).astype(np.float32)); np.add.at(S2, ec, np.sin(2 * nf).astype(np.float32))
    for s in snaps:
        out[s] = mask()
    return mask(), out


def paste(g, info, m, frame):
    """지도 g(info = res, ox, oy)에 층 m(frame 좌표)의 칸을 벽(100)으로 덧씌운 사본."""
    res, ox, oy = info; h, w, _, fx, fy = frame; out = g.copy()
    jj, ii = np.nonzero(m); X = fx + (ii + 0.5) * res; Y = fy + (jj + 0.5) * res
    i2, j2 = np.floor((X - ox) / res).astype(int), np.floor((Y - oy) / res).astype(int)
    k = (i2 >= 0) & (i2 < g.shape[1]) & (j2 >= 0) & (j2 < g.shape[0]); out[j2[k], i2[k]] = 100
    return out


def door_frame():
    rx, ry, th = DOOR_ROBOT; lx, ly = rx + LX * math.cos(th), ry + LX * math.sin(th)
    n = (math.cos(th), math.sin(th)); t = (-n[1], n[0]); return (lx + n[0] * DOOR_D, ly + n[1] * DOOR_D), n, t


def door_face(g, info):
    """문 면(−5~+5 cm), 문을 따라 0~2.0 m를 5 cm마다: 벽 칸이 있는 자리 수 / 전체 — 문 면 벽이 끊기지 않았나."""
    res, ox, oy = info; foot, n, t = door_frame(); h, w = g.shape; wall = tot = 0
    for s in np.arange(0.0, 2.01, 0.05):
        tot += 1; hit = False
        for dd in (-0.05, 0.0, 0.05):
            x = foot[0] + t[0] * s + n[0] * dd; y = foot[1] + t[1] * s + n[1] * dd; i, j = int((x - ox) / res), int((y - oy) / res)
            hit |= 0 <= i < w and 0 <= j < h and g[j, i] >= 65
        wall += hit
    return wall, tot


def frontier_near(g, info, start, goal, zones, rad=0.3):
    """목표 둘레 rad 안의 '닿을 수 있는 경계 칸'(빈 칸이면서 모르는 칸과 맞닿음, 탐색기 지도 필터를 지난 것) 수. 0이면 탐색기가 거기를 고를 수 없다."""
    from explore_map_filter import reachable
    res, ox, oy = info; reach, keep = reachable(g, res, ox, oy, start, zones)
    free = (g >= 0) & (g < 65); unk = g < 0; nb = np.zeros(g.shape, bool)
    nb[1:, :] |= unk[:-1, :]; nb[:-1, :] |= unk[1:, :]; nb[:, 1:] |= unk[:, :-1]; nb[:, :-1] |= unk[:, 1:]
    front = free & nb & (keep if keep is not None else True)
    i, j = int((goal[0] - ox) / res), int((goal[1] - oy) / res); k = int(rad / res)
    return int(front[max(0, j - k):j + k + 1, max(0, i - k):i + k + 1].sum())


def reach_area(g, info, start, zones):
    from explore_map_filter import reachable
    res, ox, oy = info; reach, _ = reachable(g, res, ox, oy, start, zones); return 0 if reach is None else int(reach.sum())


def pick_people(scans, g, info, k, seed=1):
    """사람을 넣을 자리와 시간: 벽에서 35 cm 넘는 빈 칸, 넣는 30초 동안 로봇이 0.8~2.5 m, 빼고 60초 넘게 지나 다시 3 m 안으로 오는 곳. 시간이 겹치지 않게."""
    from locate_on_map import dist_field
    res, ox, oy = info; rng = np.random.default_rng(seed); T = np.array([s[0] for s in scans]); XY = np.array([s[1][:2] for s in scans])
    D = dist_field(g >= 65, res, rmax=0.5); cand = np.argwhere((g == 0) & (D >= 0.35)); rng.shuffle(cand)
    spots, busy = [], []
    for j, i in cand:
        if len(spots) >= k:
            break
        x, y = ox + (i + 0.5) * res, oy + (j + 0.5) * res; dist = np.hypot(XY[:, 0] - x, XY[:, 1] - y)
        for t0 in T[::20]:
            t1 = t0 + 30; win = (T >= t0) & (T <= t1); later = T > t1 + 60
            if any(not (t1 < a or t0 > b) for a, b in busy) or not win.any():
                continue
            if dist[win].min() > 0.8 and dist[win].max() < 2.5 and later.any() and dist[later].min() < 3.0:
                spots.append((x, y, float(t0), float(t1))); busy.append((t0 - 5, t1 + 5)); break
    return spots


def spots_left(m, frame, spots):
    """넣었던 다리 자리(가운데에서 0.15 m 안)에 확인된 벽 칸이 남은 곳의 수, 칸 수."""
    h, w, res, ox, oy = frame; n = cells = 0
    for x, y, *_ in spots:
        i, j = int((x - ox) / res), int((y - oy) / res); c = int(m[max(0, j - 3):j + 4, max(0, i - 3):i + 4].sum()); cells += c; n += c > 0
    return n, cells


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag'); ap.add_argument('--server', required=True, help='그 탐색의 step_nav_server_*.jsonl')
    ap.add_argument('--theta', default='45'); ap.add_argument('--count', default='3'); ap.add_argument('--min-span', default='0'); ap.add_argument('--ratio', default='0.5')
    ap.add_argument('--people', type=int, default=0); ap.add_argument('--zones', default=os.path.expanduser('~/maps/home_20261114_explore9.keepout.json.bak_20261114_1349'),
                                                                          help='그 탐색 때의 금지 구역(9번째 탐색은 드레스룸 구역 추가 전 8개)')
    ap.add_argument('--out')
    a = ap.parse_args()
    import keepout
    scans, maps = read_bag(a.bag); ks = keyframes(scans)
    zones = keepout.load(a.zones)['zones'] if a.zones and os.path.exists(a.zones) else []
    _, final, finfo = maps[-1]; frame = (final.shape[0] + 40, final.shape[1] + 40, finfo[0], finfo[1] - 20 * finfo[0], finfo[2] - 20 * finfo[0])
    goals = [(r['t'] - r['result']['secs'], r['start'][:2], r['goal'][:2], r['result']['msg']) for r in map(json.loads, open(a.server))]
    mt = [m[0] for m in maps]
    gmap = [maps[max(0, bisect.bisect(mt, t) - 1)] for t, *_ in goals]
    start = ks[0][1][:2]
    print(f'스캔 {len(scans)}장 중 {len(ks)}장 사용(서로 다른 자리), /map {len(maps)}장, 구역 {len(zones)}개, 목표 {len(goals)}개')
    geo = [(t, (P[0], P[1]), beam_geometry(P, a0, da, rg)) for t, P, a0, da, rg in ks]
    base_front = [frontier_near(m[1], m[2], st, gl, zones) for (t, st, gl, _), m in zip(goals, gmap)]
    wardrobe = [min(math.hypot(gl[0] - x, gl[1] - y) for x, y in GOALS) < 0.05 for _, _, gl, _ in goals]
    print('목표를 보낸 순간의 /map에서 목표 둘레 0.3 m의 닿는 경계 칸(규칙 없음):')
    for (t, st, gl, msg), n, wd in zip(goals, base_front, wardrobe):
        print(f'  ({gl[0]:+.2f},{gl[1]:+.2f}) {n:3d}칸  {"[옷장 문 너머] " if wd else ""}실제: {msg[:22]}')
    area0 = reach_area(final, finfo, start, zones); f0 = door_face(final, finfo)
    print(f'마지막 /map: 문 면 벽 {f0[0]}/{f0[1]}, 닿는 영역 {area0}칸')
    print(f'{"규칙":<26} 옷장 너머 목표 경계칸   다른 목표 중 경계 잃음   문 면 벽   덧씌운 칸(빈칸 위)   닿는 영역 변화')
    combos = [c for c in itertools.product([float(v) for v in a.theta.split(',')], [int(v) for v in a.count.split(',')],
                                           [float(v) for v in a.min_span.split(',')], [float(v) for v in a.ratio.split(',')])]
    rows, layers = [], {}
    for rule in combos:
        m, sn = confirm_layer(geo, frame, rule, snaps=[g[0] for g in goals]); key = f'θ{rule[0]:.0f} N{rule[1]} 길이{rule[2]:.2f} 비율{rule[3]:.1f}'
        layers[key] = m
        wf, lost = [], []
        for (t, st, gl, msg), mp, n0, wd in zip(goals, gmap, base_front, wardrobe):
            n = frontier_near(paste(mp[1], mp[2], sn[t], frame), mp[2], st, gl, zones)
            if wd:
                wf.append(n)
            elif n0 > 0 and n == 0:
                lost.append(f'({gl[0]:+.1f},{gl[1]:+.1f})')
        fo = paste(final, finfo, m, frame); f1 = door_face(fo, finfo); added = int(((final >= 0) & (final < 65) & (fo >= 65)).sum())
        area1 = reach_area(fo, finfo, start, zones)
        rows.append(dict(rule=key, wardrobe_goal_frontier=wf, other_goals_lost=lost, door_face=f'{f1[0]}/{f1[1]}', added_on_free=added, reach_area=[area0, area1]))
        print(f'{key:<26} {str(wf):<20} {len(lost):2d} {" ".join(lost):<20} {f1[0]:2d}/{f1[1]}     {added:5d}            {area1 - area0:+6d} ({(area1 - area0) / area0 * 100:+.1f}%)')
    if a.people:
        spots = pick_people(ks, final, finfo, a.people)
        geo_p = [(t, (P[0], P[1]), beam_geometry(P, a0, da, rg)) for t, P, a0, da, rg in add_people(ks, spots)]
        print(f'사람 시험: {len(spots)}곳에 30초씩 넣었다 뺌 — 끝까지 확인된 벽으로 남은 곳')
        for rule in combos:
            m, _ = confirm_layer(geo_p, frame, rule); n1, c1 = spots_left(m, frame, spots); key = f'θ{rule[0]:.0f} N{rule[1]} 길이{rule[2]:.2f} 비율{rule[3]:.1f}'
            print(f'  {key:<26} {n1}/{len(spots)}곳 ({c1}칸)')
            next(r for r in rows if r['rule'] == key).update(people=[n1, len(spots), c1])
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        json.dump(rows, open(os.path.join(a.out, 'summary.json'), 'w'), ensure_ascii=False, indent=1)
        save_images(a.out, final, finfo, layers, frame)
        print('저장:', a.out)


def save_images(out, final, info, layers, frame):
    """마지막 /map 위에 확인된 벽(빨강: 빈칸·모름 위에 덧씌운 것)과 옷장 문 면(파랑 선), 문 너머 목표(보라 +). 규칙마다 옷장 쪽 확대 + 전체."""
    import cv2
    res, ox, oy = info; h, w = final.shape

    def px(x, y):
        return int((x - ox) / res), int(h - 1 - (y - oy) / res)
    foot, n, t = door_frame(); x0, y0 = px(4.3, -0.3); x1, y1 = px(8.0, -3.4); crops = []; whole = []
    for key, m in [('none', None)] + list(layers.items())[:3]:
        c = np.full((h, w, 3), 128, np.uint8); c[final == 0] = 255; c[final >= 65] = 0
        if m is not None:
            o = paste(np.zeros_like(final), info, m, frame) == 100; c[o & (final < 65)] = (0, 0, 255)
        c = c[::-1].copy(); whole.append(cv2.resize(c, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
        cr = cv2.resize(c[y0:y1, x0:x1], None, fx=6, fy=6, interpolation=cv2.INTER_NEAREST)
        p1, p2 = px(foot[0] - t[0] * 0.5, foot[1] - t[1] * 0.5), px(foot[0] + t[0] * 2.2, foot[1] + t[1] * 2.2)
        cv2.line(cr, ((p1[0] - x0) * 6, (p1[1] - y0) * 6), ((p2[0] - x0) * 6, (p2[1] - y0) * 6), (255, 160, 0), 1)
        for gx, gy in GOALS:
            q = px(gx, gy); cv2.drawMarker(cr, ((q[0] - x0) * 6 + 3, (q[1] - y0) * 6 + 3), (255, 0, 255), cv2.MARKER_CROSS, 14, 2)
        crops.append(cr)
    cv2.imwrite(os.path.join(out, 'door_panels.png'), np.hstack(crops)); cv2.imwrite(os.path.join(out, 'whole.png'), np.hstack(whole[:2]))


if __name__ == '__main__':
    main()
