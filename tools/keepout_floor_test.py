#!/usr/bin/env python3
"""금지 구역 바닥 판정 시험 (todo 11a) — 빈 바닥에서 로봇 앞에 구역을 하나 만들고, 그 너머로 가게 한다.

순서(로봇은 ②와 ③에서 스스로 움직인다 — 사람이 옆에서, 비상정지에 손이 닿게):
  ① 구역 안의 목표 → 움직이지 않고 "금지 구역 안"으로 거절해야 한다
  ② 구역 너머의 목표 → 구역을 돌아서 도착해야 한다. 가는 동안 차체(±0.11 m 사각형)와 구역 선의 거리를 계속 잰다
  ③ 출발점으로 복귀(다시 구역을 돌아서)
합격: ①거절·이동 0, ②③도착, **차체가 선을 넘은 표본 0**. 끝나면 시험 구역을 지운다(--keep-zone이면 남김).

준비: 로봇 앞 (goal + 0.5) m, 좌우 (half-width + side) m가 비어 있어야 한다 — 스캔으로 확인하고 아니면 시작하지 않는다.
       실물: tools/nav_test_stack.sh start (탐색기 없음, 구역 서버 포함) 뒤
    python3 tools/keepout_floor_test.py
시뮬(도메인 42): --odom-topic /odom --gyro-from-odom --label sim --side 0.55
"""
import argparse, json, math, os, sys, time

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import keepout
from keepout_watch import body_gap
from step_navigator import StepNavigator

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def blocked_points(scan, x0, x1, y0, y1, lidar_x=0.054):
    """로봇 기준 사각형 안의 스캔 점(차체·뒤 부착물 제외)."""
    out = []
    for i, r in enumerate(scan.ranges):
        if not (scan.range_min <= r <= scan.range_max):
            continue
        a = scan.angle_min + i * scan.angle_increment; x, y = lidar_x + r * math.cos(a), r * math.sin(a)
        if (abs(x) <= 0.13 and abs(y) <= 0.13) or (-0.21 < x < -0.11 and abs(y) <= 0.11):
            continue
        if x0 <= x <= x1 and y0 <= y <= y1:
            out.append((round(x, 2), round(y, 2)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ahead', type=float, default=0.9, help='구역 가까운 변까지(로봇 중심에서, m)')
    ap.add_argument('--depth', type=float, default=0.5, help='구역 깊이(m)')
    ap.add_argument('--half-width', type=float, default=0.35, help='구역 반폭(m)')
    ap.add_argument('--goal', type=float, default=2.1, help='구역 너머 목표(로봇 앞, m)')
    ap.add_argument('--side', type=float, default=0.75, help='구역 옆으로 비어 있어야 하는 폭(m) — 돌아갈 자리')
    ap.add_argument('--keep-zone', action='store_true')
    ap.add_argument('--label', default='', help="기록 파일 이름 꼬리(시뮬은 'sim')")
    ap.add_argument('--odom-topic', default='/odometry/filtered'); ap.add_argument('--gyro-from-odom', action='store_true')
    a = ap.parse_args()
    rclpy.init(); n = rclpy.create_node('keepout_floor_test')
    nav = StepNavigator(n, odom_topic=a.odom_topic, gyro_from_odom=a.gyro_from_odom)
    t0 = time.time()
    while (nav.S['pose'] is None or nav.S['scan'] is None or nav.map_pose() is None or not nav._zones_seen) and time.time() - t0 < 15:
        nav.spin(0.2)
    here = nav.map_pose()
    if here is None or not nav._zones_seen:
        print('❌ 시작 못 함: ' + ('지도 위 위치 없음(SLAM)' if here is None else '금지 구역 서버의 말을 못 들음(keepout_server)')); return 2
    bad = blocked_points(nav.S['scan'], -0.3, a.goal + 0.5, -(a.half_width + a.side), a.half_width + a.side)
    if len(bad) > 3:
        print(f'❌ 시험 자리가 비어 있지 않다(점 {len(bad)}개, 로봇 기준 앞·왼쪽 m): {bad[:8]}\n'
              f'   필요한 빈 자리: 앞 {a.goal + 0.5:.1f} m, 좌우 각 {a.half_width + a.side:.2f} m'); return 2
    c, s = math.cos(here[2]), math.sin(here[2])
    to_map = lambda x, y: [round(here[0] + c * x - s * y, 3), round(here[1] + s * x + c * y, 3)]
    poly = [to_map(a.ahead, -a.half_width), to_map(a.ahead + a.depth, -a.half_width), to_map(a.ahead + a.depth, a.half_width), to_map(a.ahead, a.half_width)]
    d = keepout.load(); zid = max([z['id'] for z in d['zones']] + [0]) + 1
    keepout.save(dict(d, zones=d['zones'] + [dict(id=zid, name='바닥시험', poly=poly)]))
    t0 = time.time()
    while not any(z.get('name') == '바닥시험' for z in nav._zones) and time.time() - t0 < 6:
        nav.spin(0.2)
    if not any(z.get('name') == '바닥시험' for z in nav._zones):
        print('❌ 구역을 썼는데 주행기에 도착하지 않음(구역 서버 확인)'); return 2
    print(f'구역 만듦: 로봇 앞 {a.ahead:.2f}~{a.ahead + a.depth:.2f} m, 좌우 ±{a.half_width:.2f} m', flush=True)
    pts = keepout.boundary_points([dict(poly=poly)], 0.01)
    W = dict(n=0, cross=0, min_gap=float('inf'), max_lat=0.0, last=0.0)
    spin0 = nav.spin

    def spin(t):          # 주행기가 기다리는 모든 순간에 자세를 본다(약 10 Hz)
        spin0(t)
        if time.time() - W['last'] < 0.1:
            return
        W['last'] = time.time(); p = nav.map_pose()
        if p:
            g = body_gap(p, pts); W['n'] += 1; W['min_gap'] = min(W['min_gap'], g); W['cross'] += g < -0.005
            W['max_lat'] = max(W['max_lat'], abs(-s * (p[0] - here[0]) + c * (p[1] - here[1])))
    nav.spin = spin
    res = {}
    try:
        # ① 구역 안 목표
        gin = to_map(a.ahead + a.depth / 2, 0.0); r = nav.go((gin[0], gin[1], None)); p = nav.map_pose() or here
        moved = math.hypot(p[0] - here[0], p[1] - here[1])
        res['inside'] = dict(ok=(not r['ok']) and '금지 구역' in r['msg'] and moved < 0.03, msg=r['msg'], moved_cm=round(moved * 100, 1))
        print(f'① 구역 안 목표: {"✅" if res["inside"]["ok"] else "❌"} {r["msg"]} (이동 {moved * 100:.1f} cm)', flush=True)
        # ② 구역 너머
        gout = to_map(a.goal, 0.0); r = nav.go((gout[0], gout[1], None), timeout=240.0)
        res['beyond'] = dict(ok=r['ok'], msg=r['msg'], err_cm=round(r['final_err_m'] * 100, 1), secs=round(r['secs'], 1), steps=r['steps'],
                             detour_cm=round(W['max_lat'] * 100))
        print(f'② 구역 너머 목표: {"✅" if r["ok"] else "❌"} {r["msg"]} | 오차 {r["final_err_m"] * 100:.1f} cm, {r["secs"]:.0f}초, 옆으로 최대 {W["max_lat"] * 100:.0f} cm 돌아감', flush=True)
        # ③ 복귀
        r = nav.go((here[0], here[1], here[2]), timeout=240.0)
        res['back'] = dict(ok=r['ok'], msg=r['msg'], err_cm=round(r['final_err_m'] * 100, 1), secs=round(r['secs'], 1))
        print(f'③ 출발점 복귀: {"✅" if r["ok"] else "❌"} {r["msg"]} | 오차 {r["final_err_m"] * 100:.1f} cm', flush=True)
    except KeyboardInterrupt:
        print('중단')
    finally:
        from geometry_msgs.msg import Twist
        for _ in range(5):
            nav.pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.02)
        if not a.keep_zone:
            d = keepout.load(); keepout.save(dict(d, zones=[z for z in d['zones'] if z.get('name') != '바닥시험']))
    ok = all(res.get(k, {}).get('ok') for k in ('inside', 'beyond', 'back')) and W['cross'] == 0 and W['n'] > 0
    print(f'{"✅ 합격" if ok else "❌ 불합격"} | 차체가 구역 선을 넘은 표본 {W["cross"]}/{W["n"]}, 차체-선 최소 {W["min_gap"] * 100:+.1f} cm')
    out = os.path.join(REPO, 'docs/calibration', f'keepout_floor_{time.strftime("%Y%m%d_%H%M%S")}{("_" + a.label) if a.label else ""}.json')
    json.dump(dict(start=here, zone=poly, result=res, watch={k: v for k, v in W.items() if k != 'last'}, ok=ok, log=nav.log), open(out, 'w'),
              ensure_ascii=False, indent=1, default=str)
    print('기록:', out)
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
