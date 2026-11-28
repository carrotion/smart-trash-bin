#!/usr/bin/env python3
"""벽 다가가기 시험 — 충돌 감시(collision_monitor)가 벽 앞에서 제때 멈추는지, 0.08 m 가짜 점이 나오는지 (2026-11-28).

왜: 11-28 10:24 충돌 감시가 한 번 멈췄다. 정면 상자 안에 0.08 m 점 3개가 딱 한 장만 찍혔다(이 라이다는 13.5 cm보다
    가까우면 0.08로 읽는다). 하루치 녹화에서 3번, 모두 한 장짜리. "0.08 점은 두 장 연속일 때만 멈춤"으로 바꿀지 정하는
    시험이다. 바꾸기 전 기준값(지금 설정)을 먼저 잰다.

동작(바닥 전용, 정면에 벽이 0.35~2.0 m):
    /cmd_vel_nav로 v(기본 0.15 m/s) 전진 명령을 계속 보낸다 → 충돌 감시가 거르고 /cmd_vel로 내보낸다(Nav2 주행과 같은 길).
    충돌 감시가 STOP을 알리고 바퀴가 멈추면 그 자리의 앞 여유(차체 앞면 → 벽)를 잰다. 닿음 = 여유 1 cm 이하.
    장마다 정면 상자(base 기준 x 0~0.20, |y|≤0.14 — nav2_params translation_forward와 같다) 안 점 수와 0.08 점 수를 남긴다.
    끝나면 /cmd_vel로 직접 뒤로 물러나 다음 회차.
보호: 실제 남은 거리(시작 여유 − 바퀴 거리)가 2 cm 아래면 /cmd_vel에 0을 직접 보내고 중단한다(감시를 믿지 않는 두 번째 줄).
     거리 한도(시작 여유 + 0.10 m)를 넘게 달리면 중단.

    python3 tools/wall_approach_test.py --reps 5            # 결과: docs/calibration/wall_approach_<시각>.json
"""
import argparse, json, math, os, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from speed_floor_sweep import corridor_clearance, LIDAR_X, HALF
from wheel_config import wheel_distances

BOX_X = (0.0, 0.20); BOX_Y = 0.14; GHOST = 0.085
LANE = BOX_Y - HALF   # 앞 여유를 정지 상자와 같은 폭(±0.14)으로만 본다


def box_points(msg):
    """정면 정지 상자 안 점: (전체 수, 0.08 점 수, 가장 가까운 x)."""
    n = g = 0; near = None
    for i, r in enumerate(msg.ranges):
        if not (msg.range_min <= r <= msg.range_max):
            continue
        a = msg.angle_min + i * msg.angle_increment
        x, y = LIDAR_X + r * math.cos(a), r * math.sin(a)
        if abs(x) <= HALF + 0.005 and abs(y) <= HALF + 0.005:
            continue
        if BOX_X[0] <= x <= BOX_X[1] and abs(y) <= BOX_Y:
            n += 1; g += r <= GHOST
            near = x if near is None else min(near, x)
    return n, g, near


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--v', type=float, default=0.15)
    ap.add_argument('--reps', type=int, default=5)
    ap.add_argument('--back', type=float, default=0.40, help='회차 사이 물러날 거리(m)')
    a = ap.parse_args()
    rclpy.init(); n = rclpy.create_node('wall_approach_test')
    S = {'scan': None, 'enc': [], 'state': None, 'log': None}

    def on_scan(m):
        S['scan'] = m
        if S['log'] is not None:
            bn, bg, near = box_points(m)
            S['log'].append(dict(t=round(time.time(), 3), box=bn, ghost=bg, near_x=near,
                                 clear=round(corridor_clearance(m, True, corridor=LANE), 3)))

    def on_state(m):
        S['state'] = (time.time(), m.action_type, m.polygon_name)
        if S['log'] is not None and m.action_type == CollisionMonitorState.STOP and S.get('t_stopmsg') is None:
            S['t_stopmsg'] = time.time()

    n.create_subscription(LaserScan, '/scan', on_scan, 10)
    n.create_subscription(Int32MultiArray, '/wheel_encoders', lambda m: S['enc'].append((time.time(), list(m.data))), 50)
    n.create_subscription(CollisionMonitorState, '/collision_monitor_state', on_state, 10)
    nav = n.create_publisher(Twist, '/cmd_vel_nav', 10)
    raw = n.create_publisher(Twist, '/cmd_vel', 10)

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(n, timeout_sec=0.005)

    def halt():
        for _ in range(5):
            nav.publish(Twist()); raw.publish(Twist()); spin(0.02)

    def enc_moved(since):
        e = [d for t, d in S['enc'] if t >= since]
        return len(e) >= 2 and any(abs(e[-1][i] - e[0][i]) >= 2 for i in (0, 1))

    t0 = time.time()
    while (S['scan'] is None or not S['enc']) and time.time() - t0 < 10:
        spin(0.1)
    if S['scan'] is None or not S['enc']:
        print('❌ 스캔/엔코더가 안 들어옴'); return 1
    t1 = time.time()
    while n.count_subscribers('/cmd_vel_nav') == 0 and time.time() - t1 < 8:   # 발견(discovery)이 늦게 끝남
        spin(0.2)
    if n.count_subscribers('/cmd_vel_nav') == 0:
        print('❌ /cmd_vel_nav를 받는 충돌 감시가 없음 — 스택이 켜져 있나?'); return 1
    rows = []
    try:
        for rep in range(1, a.reps + 1):
            spin(0.5)
            c0 = corridor_clearance(S['scan'], True, corridor=LANE)
            if not 0.35 <= c0 <= 2.0:
                print(f'❌ {rep}회: 앞 여유 {c0:.2f} m — 0.35~2.0 m에 벽이 있어야 함. 중단'); break
            S['log'] = []; S['t_stopmsg'] = None
            e_start = S['enc'][-1][1]; ts = time.time(); limit = c0 + 0.10
            why = None; t_still = None
            while True:
                c = Twist(); c.linear.x = a.v; nav.publish(c); spin(0.05)
                d = sum(wheel_distances(e_start, S['enc'][-1][1])) / 4
                # 실제 남은 거리 = 시작 때 제대로 읽은 여유 − 바퀴로 간 거리. 라이다 여유는 못 믿는다 — 라이다에서 약 16 cm보다
                # 가까우면 0.08 근처로 읽혀 몸통 앞 2 cm처럼 보인다(11-28 1회차: 실제 5~6 cm였는데 2.1 cm로 읽힘)
                if c0 - d < 0.02:
                    why = f'보호 정지: 실제 남은 거리 {(c0 - d) * 100:.1f} cm'; break
                if d > limit:
                    why = f'보호 정지: {d:.2f} m 달림(한도 {limit:.2f})'; break
                if time.time() - ts > limit / a.v + 6:
                    why = '시간 초과(충돌 감시가 안 멈춤?)'; break
                if S['t_stopmsg'] is not None and time.time() - ts > 1.0:
                    if not enc_moved(time.time() - 0.4):
                        t_still = t_still or time.time()
                        if time.time() - t_still > 0.6:
                            break
                    else:
                        t_still = None
            halt(); spin(0.5)
            d = sum(wheel_distances(e_start, S['enc'][-1][1])) / 4
            c1 = corridor_clearance(S['scan'], True, corridor=LANE)
            log = S['log']; S['log'] = None
            ghost_frames = [r for r in log if r['ghost'] > 0]
            # 0.08 점이 상자 안 점수 기준(3) 이상이었던 장과 그중 한 장짜리(앞뒤 장에 0.08 없음)
            g3 = [i for i, r in enumerate(log) if r['ghost'] >= 3]
            lone = [i for i in g3 if not any(0 <= j < len(log) and log[j]['ghost'] > 0 for j in (i - 1, i + 1))]
            first_box = next((r for r in log if r['box'] >= 3), None)
            r = dict(rep=rep, v=a.v, start_clear_m=round(c0, 3), end_clear_lidar_m=round(c1, 3), end_clear_true_m=round(c0 - d, 3), drove_m=round(d, 3),
                     contact=c0 - d <= 0.005, guard=why,
                     stop_msg_after_s=round(S['t_stopmsg'] - ts, 2) if S['t_stopmsg'] else None,
                     box_first_t_s=round(first_box['t'] - ts, 2) if first_box else None,
                     ghost_frames=len(ghost_frames), ghost3_frames=len(g3), ghost3_lone=len(lone), scans=len(log),
                     log=log)
            rows.append(r)
            print(f'{rep}회: 시작 여유 {c0 * 100:.0f} cm → 멈춘 실제 여유 {(c0 - d) * 100:.1f} cm(라이다 {c1 * 100:.1f}), {d * 100:.0f} cm 달림, '
                  f'{"닿음 ❌" if r["contact"] else "안 닿음"}'
                  f'{", " + why if why else ""} | 0.08 점 있던 장 {len(ghost_frames)}/{len(log)}, 3개 이상 {len(g3)}(한 장짜리 {len(lone)})',
                  flush=True)
            if why:
                break
            # 물러나기 — 직접 /cmd_vel(감시를 안 거침), 뒤 여유 확인
            if corridor_clearance(S['scan'], False, 0.1) < a.back + 0.1:
                print('뒤 여유 부족 — 물러나지 않고 끝'); break
            e_b = S['enc'][-1][1]; tb = time.time()
            while -sum(wheel_distances(e_b, S['enc'][-1][1])) / 4 < a.back and time.time() - tb < a.back / 0.1 + 3:
                c = Twist(); c.linear.x = -0.10; raw.publish(c); spin(0.05)
            halt(); spin(1.0)
    finally:
        halt()
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(repo, 'docs/calibration', f'wall_approach_{time.strftime("%Y%m%d_%H%M")}.json')
    json.dump(rows, open(out, 'w'), ensure_ascii=False, indent=1); print('저장:', out)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
