#!/usr/bin/env python3
"""저장된 지도의 금지 구역을 "새로 그릴 지도"의 좌표로 옮긴다 (2026-11-01).

왜: 새 지도를 그리면 좌표계가 새로 생겨 지난 구역은 엉뚱한 자리를 가리킨다(서버가 치워 둠). 그러면 구역을 다시 그리기 전까지
소파·침대 밑이 뚫려 있다 — 10-11 첫 탐색에서 막아 두지 않은 곳(A방)에 시작 1.3분 만에 들어갔다. 출발 전에 막혀 있어야 한다.

방법: 같은 로봇의 자세를 두 좌표로 안다 — 저장 지도 위(스캔으로 찾음, locate_on_map) / 새 지도 위(새 지도의 원점 = 오도메트리 원점이므로
지금 오도메트리 자세. 10-11 녹화의 첫 map→odom이 0). 구역 점을 "저장 지도 → 로봇 기준 → 새 지도"로 옮긴다.
스캔 추정 오차만큼 구역도 어긋난다 → 자리를 확신 못 하면 쓰지 않고, 시작 뒤 사람이 화면에서 본다.

    python3 tools/carry_zones.py --map home_20261011          # 계산만
    python3 tools/carry_zones.py --map home_20261011 --write  # 지금 구역 파일에 씀(nav_test_stack.sh의 ZONES_FROM이 쓴다)
    python3 tools/carry_zones.py --self-test
"""
import argparse, json, math, os, sys, time, urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import keepout


def carry(zones, old, new):
    """zones의 점을 저장 지도 위 자세 old에서 본 로봇 기준을 거쳐, 같은 로봇의 새 지도 위 자세 new로 옮긴 사본."""
    co, so, cn, sn = math.cos(old[2]), math.sin(old[2]), math.cos(new[2]), math.sin(new[2]); out = []
    for z in zones:
        poly = []
        for x, y in z['poly']:
            dx, dy = x - old[0], y - old[1]; rx, ry = co * dx + so * dy, -so * dx + co * dy
            poly.append([round(new[0] + cn * rx - sn * ry, 3), round(new[1] + sn * rx + cn * ry, 3)])
        out.append(dict(z, poly=poly))
    return out


def overview():
    return json.load(urllib.request.urlopen('http://localhost:8769/api/overview', timeout=5))


def self_test():
    z = [dict(id=1, name='a', poly=[[1.0, 0.0], [2.0, 0.0], [2.0, 1.0]])]
    same = carry(z, (0.3, -0.2, 0.7), (0.3, -0.2, 0.7))[0]['poly'] == [[1.0, 0.0], [2.0, 0.0], [2.0, 1.0]]
    turn = carry(z, (0.0, 0.0, 0.0), (5.0, 0.0, math.pi / 2))[0]['poly'][0]           # 앞 1 m 점 → 새 지도에서 로봇이 +y를 보므로 (5, 1)
    old, new = (1.0, 2.0, 0.5), (-3.0, 0.5, -2.0); c = carry(z, old, new)[0]['poly']   # 로봇에서 각 점까지 거리는 그대로
    d = max(abs(math.hypot(p[0] - old[0], p[1] - old[1]) - math.hypot(q[0] - new[0], q[1] - new[1])) for p, q in zip(z[0]['poly'], c))
    ok = same and abs(turn[0] - 5.0) < 1e-3 and abs(turn[1] - 1.0) < 1e-3 and d < 2e-3
    print(f'{"✅" if ok else "❌"} 구역 옮기기: 같은 자세 그대로 {same}, 90° {turn}, 거리 차 최대 {d * 1000:.1f} mm')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map'); ap.add_argument('--write', action='store_true'); ap.add_argument('--self-test', action='store_true')
    a = ap.parse_args()
    if a.self_test:
        return self_test()
    from locate_on_map import locate, whole_area
    base = os.path.expanduser(a.map if '/' in a.map else f'~/maps/{a.map}')
    d0 = overview(); time.sleep(1.0); d = overview(); o0, new = d0['pose']['odom'], d['pose']['odom']
    if math.hypot(new[0] - o0[0], new[1] - o0[1]) > 0.01 or abs(new[2] - o0[2]) > 0.01:
        print('❌ 로봇이 움직이는 중이다 — 서 있을 때만 옮긴다'); return 1
    if not d['lidar'].get('ok'):
        print('❌ 스캔이 안 들어온다'); return 1
    r = locate(base, d['lidar']['pts'], *whole_area(base)); b = r[0]
    print(f'저장 지도 위 자리 ({b["x"]:+.2f}, {b["y"]:+.2f}) m, {math.degrees(b["yaw"]):+.1f}° — 벽 위 {b["on_wall"] * 100:.0f}%, 점수 {b["score"]:.2f}'
          + (f' (2등 {r[1]["score"]:.2f})' if len(r) > 1 else '') + f' | 새 지도 위 자리(오도메트리) ({new[0]:+.2f}, {new[1]:+.2f}) m, {math.degrees(new[2]):+.1f}°')
    if b['on_wall'] < 0.85 or (len(r) > 1 and r[1]['score'] > 0.85 * b['score']):
        print('❌ 자리를 확신하지 못해 구역을 옮기지 않는다 — 벽이 잘 보이는 곳(도크 밖)에 두고 다시'); return 1
    zones = carry(keepout.load(base + '.keepout.json')['zones'], (b['x'], b['y'], b['yaw']), new)
    print(f'구역 {len(zones)}개를 옮김: ' + ', '.join(z['name'] for z in zones))
    if a.write:
        keepout.save(dict(keepout.load(), zones=keepout.clean_zones(zones), prev=[]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
