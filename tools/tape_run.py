#!/usr/bin/env python3
"""줄자 대조용 한 번 이동 — 거리 배율 판정(todo #5, docs/motion_foundation.md "거리 배율 2% 차").

drive_distance로 d m 가고, 그동안의 EKF 거리·바퀴 엔코더 거리(앞바퀴 평균)·라이다 스캔 정합 거리를 출력·기록한다.
사람이 출발 전 로봇 기준점 아래 바닥에 표시하고, 멈춘 뒤 줄자로 실제 이동을 잰다 → --tape로 넣으면 기록에 합쳐짐.

    python3 tools/tape_run.py 1.0            # 전진 1 m
    python3 tools/tape_run.py -1.0           # 후진 1 m
    python3 tools/tape_run.py --record 0.987 # 직전 이동의 줄자 값(m) 기록
"""
import argparse, json, math, os, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG = os.path.join(REPO, 'docs/calibration', f'tape_runs_{time.strftime("%Y%m%d")}.json')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dist', type=float, nargs='?')
    ap.add_argument('--record', type=float, help='직전 이동의 줄자 측정값(m, 양수)')
    a = ap.parse_args()
    hist = json.load(open(LOG)) if os.path.exists(LOG) else []
    if a.record is not None:
        hist[-1]['tape_m'] = a.record
        r = hist[-1]; d = abs(r['cmd'])
        print(f"기록: 줄자 {a.record:.3f} m | EKF {abs(r['ekf_m']):.3f} (줄자 대비 {abs(r['ekf_m'])/a.record:.4f}) | 엔코더 {abs(r['enc_m']):.3f} ({abs(r['enc_m'])/a.record:.4f}) | 라이다 {abs(r['icp_m']):.3f} ({abs(r['icp_m'])/a.record:.4f})")
        json.dump(hist, open(LOG, 'w'), ensure_ascii=False, indent=1)
        return 0
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Int32MultiArray
    from drive_distance import DriveDistance
    from turn_to_angle import yaw_of
    from wheel_config import wheel_distances
    from speed_floor_sweep import corridor_clearance
    from drive_distance_test import icp_motion
    rclpy.init(); n = rclpy.create_node('tape_run')
    S = {'pose': None, 'v': 0.0, 'scan': None, 'enc': None}

    def on_odom(m):
        p = m.pose.pose; S['pose'] = (p.position.x, p.position.y, yaw_of(p.orientation)); S['v'] = m.twist.twist.linear.x
    n.create_subscription(Odometry, '/odometry/filtered', on_odom, 20)
    n.create_subscription(LaserScan, '/scan', lambda m: S.update(scan=m), 5)
    n.create_subscription(Int32MultiArray, '/wheel_encoders', lambda m: S.update(enc=list(m.data)), 20)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(n, timeout_sec=0.005)
    t0 = time.time()
    while (S['pose'] is None or S['scan'] is None or S['enc'] is None) and time.time() - t0 < 10:
        spin(0.1)
    sgn = 1 if a.dist > 0 else -1
    room = corridor_clearance(S['scan'], sgn > 0, 0.0 if sgn > 0 else 0.1)
    if room < abs(a.dist) + 0.3:
        print(f'❌ {"앞" if sgn > 0 else "뒤"} 여유 {room:.2f} m < {abs(a.dist) + 0.3:.2f}'); return 1
    spin(1.0)
    p0, s0, e0 = S['pose'], S['scan'], list(S['enc'])
    dd = DriveDistance(pub, spin, lambda: S['pose'], lambda: S['v'], clear_fn=lambda s: corridor_clearance(S['scan'], s > 0, 0.0 if s > 0 else 0.1) > 0.2)
    r = dd.drive_by(a.dist)
    spin(1.5)
    p1 = S['pose']; c, s_ = math.cos(p0[2]), math.sin(p0[2]); ex, ey = p1[0] - p0[0], p1[1] - p0[1]
    g = (c * ex + s_ * ey, -s_ * ex + c * ey, (p1[2] - p0[2] + math.pi) % (2 * math.pi) - math.pi)
    tx, ty, th, resid = icp_motion(s0, S['scan'], g)
    d = wheel_distances(e0, S['enc']); enc = (d[0] + d[1]) / 2
    rec = dict(time=time.strftime('%H:%M:%S'), cmd=a.dist, ekf_m=g[0], enc_m=enc, enc_rear_m=(d[2] + d[3]) / 2, icp_m=tx,
               icp_resid_cm=resid * 100, ok=r['ok'], msg=r['msg'])
    hist.append(rec); json.dump(hist, open(LOG, 'w'), ensure_ascii=False, indent=1)
    print(f"이동 끝({'정상' if r['ok'] else r['msg']}): EKF {g[0]:+.3f} m | 앞바퀴 엔코더 {enc:+.3f} m | 뒷바퀴 {rec['enc_rear_m']:+.3f} m | 라이다 {tx:+.3f} m (잔차 {resid*100:.1f} cm)")
    print('→ 줄자로 잰 값을 알려 주세요(기록: tools/tape_run.py --record 값)')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
