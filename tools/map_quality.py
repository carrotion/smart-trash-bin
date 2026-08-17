#!/usr/bin/env python3
"""맵 품질을 숫자로 잰다 — 벽 방향의 일관성.

왜 이 지표인가
--------------
실내는 대부분 직각이라, 잘 그려진 맵의 벽은 **몇 개의 지배적 방향(보통 0°/90°)에
몰린다**. 오도메트리의 회전 오차가 누적되면 나중에 그린 방이 통째로 기울고, 벽 방향이
여러 갈래로 흩어진다. 즉 **방향 분산이 곧 회전 오차의 흔적**이다.

눈으로 "좀 이상하다"라고 하면 개선 여부를 판정할 수 없다. rf2o 같은 걸 넣었을 때
좋아졌는지 말하려면 같은 잣대로 전후를 재야 한다.

방법
----
점유 셀마다 반경 r 안의 이웃으로 PCA를 해서 국소 벽 방향을 구한다. 벽은 180° 주기이고
직각 구조는 90° 주기라, 각도를 90°로 접어서(mod 90) 히스토그램을 만든다.
잘 정렬된 맵이면 한 봉우리에 몰리고, 기울어진 방이 섞이면 봉우리가 갈라진다.

    python3 tools/map_quality.py --pgm 맵.pgm
    python3 tools/map_quality.py            # 실행 중인 /map을 구독
"""
import argparse, math, sys
from collections import Counter


def wall_angles(occ_cells, radius_cells=4, min_neighbors=5):
    """점유 셀들의 국소 벽 방향(도, 0~90)을 반환."""
    pts = set(occ_cells)
    out = []
    r = radius_cells
    for (x, y) in occ_cells:
        nb = [(nx, ny) for nx in range(x - r, x + r + 1)
                       for ny in range(y - r, y + r + 1)
              if (nx, ny) in pts and (nx - x) ** 2 + (ny - y) ** 2 <= r * r]
        if len(nb) < min_neighbors:
            continue
        mx = sum(p[0] for p in nb) / len(nb)
        my = sum(p[1] for p in nb) / len(nb)
        sxx = syy = sxy = 0.0
        for px, py in nb:
            dx, dy = px - mx, py - my
            sxx += dx * dx; syy += dy * dy; sxy += dx * dy
        # 2x2 공분산의 주축 각도
        theta = 0.5 * math.atan2(2 * sxy, sxx - syy)
        # 직선성이 낮으면(모서리/잡음) 버린다
        tr = sxx + syy
        det = sxx * syy - sxy * sxy
        if tr <= 0: continue
        disc = max(0.0, tr * tr / 4 - det)
        l1 = tr / 2 + math.sqrt(disc); l2 = tr / 2 - math.sqrt(disc)
        if l1 <= 0 or l2 / l1 > 0.15:   # 충분히 선형이 아니면 제외
            continue
        out.append(math.degrees(theta) % 90.0)
    return out


def report(angles, label=''):
    if len(angles) < 30:
        print(f'  {label}표본 부족({len(angles)}개)')
        return None
    # 5도 빈 히스토그램
    h = Counter(int(a // 5) for a in angles)
    total = len(angles)
    peak_bin, peak_n = h.most_common(1)[0]
    # 지배 방향 주변 ±10도에 몰린 비율 = 정렬도
    aligned = sum(n for b, n in h.items()
                  if min(abs(b - peak_bin), 18 - abs(b - peak_bin)) <= 2)
    print(f'  {label}표본 {total}개, 지배 방향 {peak_bin*5}~{peak_bin*5+5}°')
    print(f'  {label}정렬도(지배방향 ±10° 안) {100*aligned/total:.1f}%   <- 높을수록 좋다')
    bars = []
    for b in range(18):
        n = h.get(b, 0)
        bars.append(f'{b*5:>3}°|' + '#' * int(40 * n / max(1, peak_n)))
    print('  ' + label + '분포:')
    for line in bars:
        print('    ' + line)
    return 100.0 * aligned / total


def from_topic():
    import rclpy, time
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                           QoSReliabilityPolicy, QoSHistoryPolicy)
    rclpy.init()
    n = rclpy.create_node('map_quality')
    if '--sim-time' in sys.argv:
        from rclpy.parameter import Parameter
        n.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    got = []
    q = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                   history=QoSHistoryPolicy.KEEP_LAST,
                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(OccupancyGrid, '/map', lambda m: got.append(m), q)
    end = time.time() + 15
    while time.time() < end and not got:
        rclpy.spin_once(n, timeout_sec=0.1)
    rclpy.shutdown()
    if not got:
        print('맵 수신 실패'); sys.exit(1)
    m = got[-1]
    w = m.info.width
    occ = [(i % w, i // w) for i, v in enumerate(m.data) if v >= 65]
    return occ, w, m.info.height


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sim-time', action='store_true',
                    help='가방 재생 중이면 지정 (모의 시간 사용)')
    ap.add_argument('--split', action='store_true',
                    help='좌상단 사분면과 나머지를 나눠서 비교')
    args = ap.parse_args()

    occ, w, h = from_topic()
    print(f'점유 셀 {len(occ)}개, 맵 {w}x{h}\n')
    print('=== 전체 ===')
    overall = report(wall_angles(occ))
    if args.split:
        # ROS 좌표는 아래에서 위로 증가하므로 "위쪽"은 y가 큰 쪽
        tl = [(x, y) for x, y in occ if x < w * 0.5 and y > h * 0.5]
        rest = [(x, y) for x, y in occ if not (x < w * 0.5 and y > h * 0.5)]
        print('\n=== 좌상단 사분면 ===')
        a = report(wall_angles(tl), '')
        print('\n=== 나머지 ===')
        b = report(wall_angles(rest), '')
        if a is not None and b is not None:
            print(f'\n정렬도 차이: 좌상단 {a:.1f}% vs 나머지 {b:.1f}%'
                  f'  ({a-b:+.1f}%p)')


if __name__ == '__main__':
    main()
