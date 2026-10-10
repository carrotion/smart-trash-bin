#!/usr/bin/env python3
"""금지 구역 감시 기록 — 로봇 차체(±0.11 m 사각형)와 구역 선 사이 거리를 지도 위 자세(map→base_link)로 계속 잰다.

시뮬·실물 시험의 판정 도구: 끝나면 "차체가 선을 넘은 횟수, 가장 가까웠던 거리"를 낸다(넘으면 종료 코드 1).
구역은 /keepout_zones(구역 서버가 실제로 적용한 것)를 받는다 — 파일이 아니라 적용된 것을 기준으로.

    python3 tools/keepout_watch.py [--duration 300] [--sim-time] [--out 기록.csv]
"""
import argparse, json, math, os, sys, time

import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import keepout

HALF = 0.11


def body_gap(pose, pts):
    """차체 사각형에서 구역 선(점들)까지 가장 가까운 거리. 음수 = 선이 차체 안(넘었다)."""
    c, s = math.cos(pose[2]), math.sin(pose[2]); best = float('inf')
    for wx, wy in pts:
        dx, dy = wx - pose[0], wy - pose[1]
        best = min(best, max(abs(c * dx + s * dy), abs(-s * dx + c * dy)) - HALF)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=0.0, help='0이면 Ctrl+C까지')
    ap.add_argument('--sim-time', action='store_true')
    ap.add_argument('--out')
    a = ap.parse_args()
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'] if a.sim_time else None)
    n = rclpy.create_node('keepout_watch')
    Z = {'zones': [], 'pts': [], 'ver': None}

    def on_zones(m):
        d = json.loads(m.data); Z.update(zones=d['zones'], pts=keepout.boundary_points(d['zones'], 0.01), ver=d.get('version'))
        print(f'구역 {len(d["zones"])}개(version {d.get("version")})', flush=True)
    n.create_subscription(String, '/keepout_zones', on_zones,
                          QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
    tfb = Buffer(); TransformListener(tfb, n)
    st = dict(n=0, cross=0, inside=0, min_gap=float('inf'), dist=0.0, last=None)
    f = open(a.out, 'w') if a.out else None
    if f:
        f.write('t,x,y,yaw,body_gap,center_dist\n')
    t0 = time.time(); last_print = 0.0
    try:
        while rclpy.ok() and (not a.duration or time.time() - t0 < a.duration):
            rclpy.spin_once(n, timeout_sec=0.1)
            try:
                t = tfb.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
            except Exception:
                continue
            q = t.rotation; pose = (t.translation.x, t.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            if st['last']:
                st['dist'] += math.hypot(pose[0] - st['last'][0], pose[1] - st['last'][1])
            st['last'] = pose; st['n'] += 1
            if not Z['pts']:
                continue
            g = body_gap(pose, Z['pts']); cd = keepout.nearest(pose[0], pose[1], Z['zones'])[0]
            st['min_gap'] = min(st['min_gap'], g); st['cross'] += g < -0.005; st['inside'] += cd == 0.0
            if f:
                f.write(f'{time.time() - t0:.2f},{pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f},{g:.3f},{cd:.3f}\n')
            if time.time() - last_print > 10:
                last_print = time.time()
                print(f'  {time.time() - t0:5.0f}초: 차체-구역 선 {g * 100:+.0f} cm (최소 {st["min_gap"] * 100:+.0f} cm), 이동 {st["dist"]:.1f} m', flush=True)
    except KeyboardInterrupt:
        pass
    ok = st['cross'] == 0 and st['inside'] == 0
    print(f'{"✅" if ok else "❌"} 표본 {st["n"]}개, 이동 {st["dist"]:.1f} m | 차체가 구역 선을 넘은 표본 {st["cross"]}, 중심이 구역 안인 표본 {st["inside"]}, '
          f'차체-구역 선 최소 {st["min_gap"] * 100:+.1f} cm' if Z['pts'] else '구역이 없어 판정하지 않음', flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
