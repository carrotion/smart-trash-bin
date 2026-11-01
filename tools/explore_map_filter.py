#!/usr/bin/env python3
"""탐색기에게 "닿을 수 있는 곳의 경계만" 보여 주는 지도 필터 (2026-10-11).

왜: 탐색기(frontier_exploration_ros2)는 지도에서 "아는 빈 칸과 모르는 칸의 경계"를 목표로 고르는데, 거기까지 갈 길이 있는지는
보지 않는다. 금지 구역이나 좁은 틈 너머로 들여다본 자리(문 앞을 구역으로 막은 화장실 안, 닫힌 문틈 뒤)는 경계로 남아 있어서
10-11 첫 실물 탐색에서 같은 자리를 3~6번씩 "빈 경로"로 되풀이했고, 다 돌고도 끝났다는 것을 몰랐다.

방법: 로봇 자리에서 시작해 "차체 중심이 설 수 있는 빈 칸"(벽·구역에서 INSCRIBED보다 먼 칸)만 따라 번져 나가 닿는 영역을 구한다.
그 영역(과 거기서 EDGE 안의 빈 칸 — 벽 바로 옆 칸은 중심이 못 서도 경계로는 유효하다)이 아닌 빈 칸은 벽으로 바꾼 지도를
/map_explore로 낸다. 탐색기의 map_topic을 이 토픽으로 돌리면 닿지 못하는 경계가 후보에서 사라진다. 원래 /map은 건드리지 않는다
(SLAM·경로 계획·주행기는 그대로). 문이 열리거나 구역이 지워지면 다음 지도에서 그 경계가 다시 살아난다.

    python3 tools/explore_map_filter.py                      # 노드(자율 스택 런치가 띄운다)
    python3 tools/explore_map_filter.py --check-bag bags/… --server docs/calibration/step_nav_server_….jsonl --zones 파일
        # 녹화로 확인: 그때 보낸 목표마다 "닿는 곳이었나"를 실제 결과와 견준다
"""
import argparse, collections, json, os, sys, time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import keepout
from locate_on_map import dist_field

EDGE = 0.20        # 닿는 영역에서 이만큼까지의 빈 칸은 남긴다(벽에서 INSCRIBED 안쪽 띠 + 여유). 구역·벽은 넘지 못한다
OCC = 65


def zone_mask(shape, res, ox, oy, zones):
    """칸 중심이 구역 안인 칸."""
    h, w = shape; m = np.zeros(shape, bool)
    for z in zones:
        p = np.asarray(z['poly'], float)
        i0, i1 = max(0, int((p[:, 0].min() - ox) / res)), min(w, int((p[:, 0].max() - ox) / res) + 2)
        j0, j1 = max(0, int((p[:, 1].min() - oy) / res)), min(h, int((p[:, 1].max() - oy) / res) + 2)
        if i0 >= i1 or j0 >= j1:
            continue
        X, Y = np.meshgrid(ox + (np.arange(i0, i1) + 0.5) * res, oy + (np.arange(j0, j1) + 0.5) * res); ins = np.zeros(X.shape, bool)
        for k in range(len(p)):                  # 홀짝 규칙
            (x1, y1), (x2, y2) = p[k], p[(k + 1) % len(p)]
            if y1 == y2:
                continue
            c = ((y1 > Y) != (y2 > Y)) & (X < (x2 - x1) * (Y - y1) / (y2 - y1) + x1); ins ^= c
        m[j0:j1, i0:i1] |= ins
    return m


def _grow(seed, allowed, steps=None):
    """seed에서 allowed 칸만 따라 8방향으로 번진 영역. steps가 있으면 그 걸음 수까지만."""
    out = seed.copy(); front = seed; n = 0
    while front.any() and (steps is None or n < steps):
        g = np.zeros_like(front)
        g[1:, :] |= front[:-1, :]; g[:-1, :] |= front[1:, :]; g[:, 1:] |= front[:, :-1]; g[:, :-1] |= front[:, 1:]
        g[1:, 1:] |= front[:-1, :-1]; g[1:, :-1] |= front[:-1, 1:]; g[:-1, 1:] |= front[1:, :-1]; g[:-1, :-1] |= front[1:, 1:]
        front = g & allowed & ~out; out |= front; n += 1
    return out


# 차체 중심이 설 수 있는 칸: 벽·구역에서 이만큼 넘게 떨어진 칸. 처음엔 차체 반폭(0.11)이었으나 11-01 실물에서 닫힌 문 옆 틈(코스트 84~99, 실제 못 지나감) 너머가
#   "닿음"으로 남아 왕복했다. 두 녹화로 고른 값: 0.15면 도착한 목표 13개 모두 유지, 왕복 쪽 목표 6개 중 4개 제외. 0.16이면 실제로 도착한 A방 목표가 빠진다
#   ⚠️ 여유가 0.01뿐이고 왕복 쪽 2개는 남는다 — 이것만으로 왕복이 사라진다고 보지 말 것. 실물로 다시 잴 것
STAND = 0.15


def reachable(grid, res, ox, oy, robot_xy, zones=(), inscribed=STAND, edge=EDGE):
    """(닿는 영역, 남길 빈 칸) 두 장의 bool 격자. grid는 OccupancyGrid 값(−1 모름, 0 빈 칸, 100 벽), 행 0 = 아래쪽.
    로봇 자리가 설 수 없는 칸이면(벽 옆에 바짝) 0.3 m 안의 가장 가까운 설 수 있는 칸에서 시작한다. 그것도 없으면 (None, None)."""
    occ = grid >= OCC; free = (grid >= 0) & ~occ; zm = zone_mask(grid.shape, res, ox, oy, zones) if zones else np.zeros(grid.shape, bool)
    D = dist_field(occ | zm, res, rmax=0.3); stand = free & ~zm & (D > inscribed)
    h, w = grid.shape; i, j = int((robot_xy[0] - ox) / res), int((robot_xy[1] - oy) / res)
    if not (0 <= i < w and 0 <= j < h):
        return None, None
    if not stand[j, i]:
        k = int(0.3 / res); j0, i0 = max(0, j - k), max(0, i - k); sub = stand[j0:j + k + 1, i0:i + k + 1]; jj, ii = np.nonzero(sub)
        if not len(jj):
            return None, None
        b = np.argmin((jj + j0 - j) ** 2 + (ii + i0 - i) ** 2); j, i = jj[b] + j0, ii[b] + i0
    seed = np.zeros(grid.shape, bool); seed[j, i] = True
    reach = _grow(seed, stand)
    keep = _grow(reach, free & ~zm, steps=int(round(edge / res)))
    return reach, keep


def filtered(grid, res, ox, oy, robot_xy, zones=()):
    """탐색기에게 줄 지도: 닿지 못하는 빈 칸을 벽(100)으로 바꾼 것. 시작 칸을 못 찾으면 원래 지도 그대로(탐색을 막지 않는다)."""
    reach, keep = reachable(grid, res, ox, oy, robot_xy, zones)
    if reach is None:
        return grid, None
    out = grid.copy(); out[(grid >= 0) & (grid < OCC) & ~keep] = 100
    return out, reach


def frontier_cells(grid):
    """모르는 칸과 맞닿은 빈 칸(4방향)의 수 — 탐색 후보가 얼마나 남았는지의 대략적인 척도."""
    free = (grid >= 0) & (grid < OCC); unk = grid < 0; n = np.zeros(grid.shape, bool)
    n[1:, :] |= unk[:-1, :]; n[:-1, :] |= unk[1:, :]; n[:, 1:] |= unk[:, :-1]; n[:, :-1] |= unk[:, 1:]
    return int((free & n).sum())


def check_bag(bag, server, zones_file):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import OccupancyGrid
    R = [json.loads(l) for l in open(server)]; zones = keepout.load(zones_file)['zones'] if zones_file else []
    starts = sorted((r['t'] - r['result']['secs'], k) for k, r in enumerate(R)); maps = {}
    meta = open(os.path.join(bag, 'metadata.yaml')).read(); rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap' if 'storage_identifier: mcap' in meta else 'sqlite3'), rosbag2_py.ConverterOptions('', ''))
    rd.set_filter(rosbag2_py.StorageFilter(topics=['/map'])); last = None; q = collections.deque(starts)
    while rd.has_next():
        _, data, tns = rd.read_next(); t = tns * 1e-9
        while q and q[0][0] < t:
            maps[q.popleft()[1]] = last
        last = data
    while q:
        maps[q.popleft()[1]] = last
    print(f'구역 {len(zones)}개(마지막 상태를 전 구간에 적용 — 실제로는 주행 중에 하나씩 그려졌다), 목표 {len(R)}개')
    tally = collections.Counter()
    for k, r in enumerate(R):
        if maps.get(k) is None:
            print(f'{k + 1:2d} 지도 없음'); continue
        m = deserialize_message(maps[k], OccupancyGrid); g = np.frombuffer(bytes(m.data), np.int8).reshape(m.info.height, m.info.width)
        res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
        t0 = time.time(); reach, keep = reachable(g, res, ox, oy, r['start'][:2], zones); dt = time.time() - t0
        gi, gj = int((r['goal'][0] - ox) / res), int((r['goal'][1] - oy) / res); rad = 3
        if reach is None:
            ok = None
        else:
            ok = bool(keep[max(0, gj - rad):gj + rad + 1, max(0, gi - rad):gi + rad + 1].any()) if 0 <= gi < m.info.width and 0 <= gj < m.info.height else False
        out, _ = filtered(g, res, ox, oy, r['start'][:2], zones); res_ok = r['result']['ok']; msg = r['result']['msg']
        kind = '도착' if res_ok else ('빈 경로' if '빈 경로' in msg else ('취소' if '취소' in msg else '그 밖의 실패'))
        tally[(kind, ok)] += 1
        print(f'{k + 1:2d} 목표({r["goal"][0]:+.1f},{r["goal"][1]:+.1f}) 필터 판단: {"닿음" if ok else ("시작 칸 없음" if ok is None else "못 닿음")}  실제: {msg[:30]:<30} '
              f'| 경계 칸 {frontier_cells(g)} → {frontier_cells(out)} ({dt * 1000:.0f} ms)')
    print('집계(실제 결과, 필터 판단) → 건수:'); [print(f'  {a} / {"닿음" if b else "못 닿음" if b is False else "판단 불가"}: {n}') for (a, b), n in sorted(tally.items(), key=str)]


def self_test():
    """만든 격자로 규칙 확인(ROS 없이): 방 A(로봇) | 벽 x=3 m, 문 | 방 B. B의 오른쪽 끝과 A의 왼쪽 끝이 모르는 칸과 맞닿은 경계."""
    res = 0.05; ok_all = True

    def world(door_cells):
        g = np.zeros((80, 122), np.int8); g[:, 0] = -1; g[:, 121] = -1          # 양 끝 한 줄은 모르는 칸
        g[0, :] = g[79, :] = 100; g[:, 60] = 100; j0 = 40 - door_cells // 2; g[j0:j0 + door_cells, 60] = 0
        return g

    def right_frontier(g):                      # 방 B 쪽 경계 칸 수
        return frontier_cells(g[:, 61:]) if g.shape[1] > 61 else 0

    def case(name, cond, detail):
        nonlocal ok_all
        ok_all &= bool(cond); print(f'{"✅" if cond else "❌"} {name}: {detail}')
    g = world(10); out, reach = filtered(g, res, 0.0, 0.0, (1.0, 2.0))
    case('문 0.5 m, 구역 없음 → 옆방의 경계가 남는다', right_frontier(out) == right_frontier(g) > 0 and frontier_cells(out[:, :60]) > 0, f'옆방 경계 칸 {right_frontier(g)} → {right_frontier(out)}')
    z = [dict(id=1, name='문', poly=[[2.8, 1.6], [3.3, 1.6], [3.3, 2.4], [2.8, 2.4]])]
    out, reach = filtered(g, res, 0.0, 0.0, (1.0, 2.0), z)
    case('문을 구역으로 막음 → 옆방의 경계가 사라지고 이쪽 방은 그대로', right_frontier(out) == 0 and frontier_cells(out[:, :60]) == frontier_cells(g[:, :60]) and (g == out)[g != 0].all(),
         f'옆방 경계 칸 {right_frontier(g)} → {right_frontier(out)}, 이쪽 {frontier_cells(g[:, :60])} → {frontier_cells(out[:, :60])}')
    n20 = right_frontier(filtered(world(6), res, 0.0, 0.0, (1.0, 2.0))[0]); n30 = right_frontier(filtered(world(8), res, 0.0, 0.0, (1.0, 2.0))[0])
    case('틈 0.30 m는 못 지나감(주행기 통로 0.32 m) / 0.40 m는 지나감', n20 == 0 and n30 > 0, f'옆방 경계 칸: 0.30 m {n20}, 0.40 m {n30}')
    out, reach = filtered(g, res, 0.0, 0.0, (2.96, 1.0))
    case('로봇이 벽에 바짝(설 수 없는 칸) → 가까운 설 수 있는 칸에서 시작', reach is not None and right_frontier(out) > 0, f'닿는 칸 {0 if reach is None else int(reach.sum())}')
    out, reach = filtered(g, res, 0.0, 0.0, (30.0, 30.0))
    case('로봇이 지도 밖 → 거르지 않고 그대로', reach is None and (out == g).all(), '원래 지도와 같음')
    return 0 if ok_all else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check-bag'); ap.add_argument('--server'); ap.add_argument('--zones'); ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--out-topic', default='/map_explore')
    a, _ = ap.parse_known_args()
    if a.self_test:
        return self_test()
    if a.check_bag:
        check_bag(a.check_bag, a.server, a.zones); return 0
    import rclpy
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    rclpy.init(); n = rclpy.create_node('explore_map_filter'); latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    S = dict(map=None, zones=[], dirty=False, warned=0.0); pub = n.create_publisher(OccupancyGrid, a.out_topic, latched); tfb = Buffer(); TransformListener(tfb, n)

    def on_zones(m):
        try:
            S['zones'] = keepout.clean_zones(json.loads(m.data).get('zones', [])); S['dirty'] = True
        except (ValueError, AttributeError) as e:
            n.get_logger().warn(f'금지 구역 메시지 오류 — 이전 구역 유지: {e}')
    n.create_subscription(OccupancyGrid, '/map', lambda m: S.update(map=m, dirty=True), latched)
    n.create_subscription(String, '/keepout_zones', on_zones, latched)

    def tick():
        m = S['map']
        if m is None or not S['dirty']:
            return
        try:
            tr = tfb.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.0))
        except Exception:
            return                               # 자세를 아직 모른다 — 다음 차례에
        S['dirty'] = False
        g = np.frombuffer(bytes(m.data), np.int8).reshape(m.info.height, m.info.width)
        out, reach = filtered(g, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y,
                              (tr.transform.translation.x, tr.transform.translation.y), S['zones'])
        if reach is None and time.time() - S['warned'] > 30:
            S['warned'] = time.time(); n.get_logger().warn('로봇 둘레에 설 수 있는 칸이 없어 지도를 거르지 않고 그대로 낸다')
        o = OccupancyGrid(); o.header = m.header; o.info = m.info; o.data = out.astype(np.int8).ravel().tolist()
        pub.publish(o)
    n.create_timer(1.0, tick)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
