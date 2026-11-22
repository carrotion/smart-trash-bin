#!/usr/bin/env python3
"""저장 지도에 붙인 방 이름(장소)을 "새로 그릴 지도"의 좌표로 옮긴다 (2026-11-22, 금지 구역 carry_zones와 같은 방법).

왜: 장소는 그 지도 위의 좌표라, 새 지도를 그리면 좌표계가 새로 생겨 이름이 엉뚱한 자리를 가리킨다.
     그러면 "주방으로 가"가 벽 속을 가리킬 수 있다. 출발 전에 저장 지도 → 로봇 기준 → 새 지도(오도메트리)로 옮겨 둔다.
옮긴 결과는 `~/maps/active_places.json`에 둔다 — 탐색 중 지도(live-…)의 장소 파일 이름을 미리 알 수 없기 때문이다.
`tools/server_link.py`는 지금 지도가 live면 이 파일을 읽고, 저장 지도면 `<지도>.places.json`을 읽는다.

    python3 tools/carry_places.py --map home_20261122_explore12 --write     # nav_test_stack.sh가 ZONES_FROM과 함께 부른다
    python3 tools/carry_places.py --self-test
"""
import argparse, json, math, os, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
ACTIVE = os.path.expanduser('~/maps/active_places.json')


def carry(places, old, new):
    """장소 점을 저장 지도 위 자세 old에서 본 로봇 기준을 거쳐, 같은 로봇의 새 지도 위 자세 new로 옮긴 사본."""
    co, so, cn, sn = math.cos(old[2]), math.sin(old[2]), math.cos(new[2]), math.sin(new[2]); out = []
    for p in places:
        dx, dy = p['x'] - old[0], p['y'] - old[1]; rx, ry = co * dx + so * dy, -so * dx + co * dy
        out.append(dict(p, x=round(new[0] + cn * rx - sn * ry, 3), y=round(new[1] + sn * rx + cn * ry, 3)))
    return out


def load_places(path):
    try:
        return json.load(open(path)).get('places', [])
    except Exception:
        return []


def self_test():
    ps = [dict(name='주방', x=1.0, y=0.0), dict(name='안방', x=-1.0, y=2.0)]
    same = carry(ps, (0.3, -0.2, 0.7), (0.3, -0.2, 0.7))
    ok1 = all(abs(a['x'] - b['x']) < 1e-6 and abs(a['y'] - b['y']) < 1e-6 for a, b in zip(ps, same))
    turn = carry(ps, (0.0, 0.0, 0.0), (5.0, 0.0, math.pi / 2))       # 앞 1 m → 로봇이 +y를 보므로 (5, 1)
    ok2 = abs(turn[0]['x'] - 5.0) < 1e-3 and abs(turn[0]['y'] - 1.0) < 1e-3
    old, new = (1.0, 2.0, 0.5), (-3.0, 0.5, -2.0); c = carry(ps, old, new)
    d = max(abs(math.hypot(p['x'] - old[0], p['y'] - old[1]) - math.hypot(q['x'] - new[0], q['y'] - new[1])) for p, q in zip(ps, c))
    print(f'{"✅" if ok1 and ok2 and d < 2e-3 else "❌"} 장소 옮기기: 같은 자세 그대로 {ok1}, 90° {turn[0]}, 거리 차 최대 {d * 1000:.1f} mm')
    return 0 if (ok1 and ok2 and d < 2e-3) else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map'); ap.add_argument('--write', action='store_true'); ap.add_argument('--self-test', action='store_true')
    a = ap.parse_args()
    if a.self_test:
        return self_test()
    if not a.map:
        print('--map 이 필요하다'); return 2
    src = os.path.expanduser(a.map if '/' in a.map else f'~/maps/{a.map}.places.json')
    places = load_places(src)
    if not places:
        print(f'장소 없음: {src} — 옮길 것이 없다(그대로 시작해도 된다)')
        if a.write:
            try:
                os.remove(ACTIVE)
            except OSError:
                pass
        return 0
    from locate_on_map import locate, whole_area
    import urllib.request

    def overview():
        return json.load(urllib.request.urlopen('http://localhost:8769/api/overview', timeout=5))
    base = src[:-len('.places.json')]
    d0 = overview(); time.sleep(1.0); d = overview(); o0, new = d0['pose']['odom'], d['pose']['odom']
    if math.hypot(new[0] - o0[0], new[1] - o0[1]) > 0.01 or abs(new[2] - o0[2]) > 0.01:
        print('❌ 로봇이 움직이는 중이다 — 서 있을 때만 옮긴다'); return 1
    if not d['lidar'].get('ok'):
        print('❌ 스캔이 안 들어온다'); return 1
    area = whole_area(base); R = [locate(base, d['lidar']['pts'], *area)]
    for _ in range(2):
        time.sleep(0.5); R.append(locate(base, overview()['lidar']['pts'], *area))
    b = R[-1][0]; spread = max(math.hypot(p[0]['x'] - b['x'], p[0]['y'] - b['y']) for p in R)
    if spread > 0.05:
        print(f'❌ 저장 지도 위 자리가 흔들린다({spread * 100:.0f} cm) — 옮기지 않음'); return 1
    out = carry(places, (b['x'], b['y'], b['yaw']), tuple(new))
    print(f'장소 {len(out)}개를 옮김: 저장 지도 ({b["x"]:+.2f}, {b["y"]:+.2f}, {math.degrees(b["yaw"]):+.1f}°) → 새 지도 ({new[0]:+.2f}, {new[1]:+.2f}, {math.degrees(new[2]):+.1f}°)')
    for p in out:
        print(f'  {p["name"]}: ({p["x"]:+.2f}, {p["y"]:+.2f})')
    if a.write:
        json.dump({'from': os.path.basename(base), 'carried_at': time.strftime('%Y-%m-%d %H:%M'), 'places': out},
                  open(ACTIVE, 'w'), ensure_ascii=False, indent=1)
        print('저장:', ACTIVE)
    else:
        print('(--write 없이 미리보기만)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
