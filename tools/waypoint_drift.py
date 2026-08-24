#!/usr/bin/env python3
"""알려진 실제 목표 지점(예: `our_home` 지도 대비 계산한 거리)까지 직진시켜서
휠 단독(/odom)과 EKF(/odometry/filtered)가 목표까지의 변위를 얼마나 정확히
추정하는지 잰다 — Track 0 "웨이포인트 대조" 첫 실측.

round_trip_drift.py는 왕복 후 원점(시작 위치)과의 드리프트를 재지만, 이건
**절대 목표 지점**(지도에서 미리 계산한 진짜 거리)까지 편도로 이동시켜서
"목표에 도달했다고 믿는 시점"의 오도메트리 추정이 실제로 맞는지 사용자의
육안/실측 확인과 대조하는 용도.

정지 기준은 **EKF의 실시간 누적 이동거리**로 건다(시간 기반이 아님) — "EKF가
목표거리 도달을 선언한 시점"이 곧 시험 대상이므로, 그 판단 자체로 멈춰야
의미가 있다. 동시에 /odom 기준 거리도 같이 기록해서 두 채널의 이견을 비교한다.

⚠️ 반드시 사용자가 로봇을 목표 방향으로 미리 정확히 조준해둔 뒤 실행할 것
   (지도 좌표계와 로봇 실제 방위를 소프트웨어로 정합할 방법이 아직 없음 —
   round_trip_drift.py와 같은 한계). 이 스크립트는 순수 직진만 명령한다.

사용법:
    python3 tools/waypoint_drift.py --target-dist 5.06 --speed 0.2 \
        --target-name "A(거실 옆방)" --json docs/calibration/waypoint_A.json
"""
import argparse, json, math, signal, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Runner:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node('waypoint_drift')
        self.odom_raw = None
        self.odom_ekf = None
        self.n.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.n.create_subscription(Odometry, '/odometry/filtered', self._on_ekf, 10)
        self.pub = self.n.create_publisher(Twist, '/cmd_vel', 10)

    def _on_odom(self, msg):
        p = msg.pose.pose
        self.odom_raw = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def _on_ekf(self, msg):
        p = msg.pose.pose
        self.odom_ekf = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def drive(self, v):
        t = Twist(); t.linear.x = float(v); t.angular.z = 0.0
        self.pub.publish(t)

    def stop(self):
        for _ in range(6):
            self.drive(0.0); time.sleep(0.04)


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--target-dist', type=float, required=True,
                     help='목표까지 지도로 계산한 진짜 직선거리(m)')
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--target-name', default='')
    ap.add_argument('--max-sec', type=float, default=120,
                     help='안전 상한 — EKF가 안 붙거나 이상하면 이 시간에 강제 정지')
    ap.add_argument('--json')
    args = ap.parse_args()

    r = Runner()
    d = time.time() + 10
    while time.time() < d and (r.odom_raw is None or r.odom_ekf is None):
        rclpy.spin_once(r.n, timeout_sec=0.02)
    if r.odom_raw is None or r.odom_ekf is None:
        print('❌ /odom 또는 /odometry/filtered 수신 실패'); return 1

    start_raw, start_ekf = r.odom_raw, r.odom_ekf
    print(f'목표: {args.target_name or "(이름 없음)"}  진짜 거리 {args.target_dist:.2f}m')
    print(f'시작 — /odom: ({start_raw[0]:+.3f}, {start_raw[1]:+.3f})   '
          f'/odometry/filtered: ({start_ekf[0]:+.3f}, {start_ekf[1]:+.3f})\n')

    flag = {'v': False}
    signal.signal(signal.SIGINT, lambda *a: flag.update(v=True))
    signal.signal(signal.SIGTERM, lambda *a: flag.update(v=True))

    rows = []
    t_end = time.time() + args.max_sec
    last_print = 0.0
    try:
        while time.time() < t_end and not flag['v']:
            rclpy.spin_once(r.n, timeout_sec=0.02)
            d_ekf = dist(r.odom_ekf, start_ekf)
            d_raw = dist(r.odom_raw, start_raw)
            rows.append({'t': time.time(), 'd_ekf': d_ekf, 'd_raw': d_raw,
                         'ekf': r.odom_ekf, 'raw': r.odom_raw})
            if d_ekf >= args.target_dist:
                break
            r.drive(args.speed)
            now = time.time()
            if now - last_print > 1.0:
                print(f'  진행 — EKF {d_ekf:.2f}m / {args.target_dist:.2f}m   '
                      f'(휠 {d_raw:.2f}m)', end='\r')
                last_print = now
    finally:
        r.stop()

    d_ekf = dist(r.odom_ekf, start_ekf)
    d_raw = dist(r.odom_raw, start_raw)
    print(f'\n\n=== 정지 — EKF가 목표거리 도달을 선언한 시점 ===')
    print(f'  EKF 이동거리 : {d_ekf:.2f}m (목표 {args.target_dist:.2f}m, '
          f'오차 {(d_ekf-args.target_dist)*100:+.1f}cm)')
    print(f'  휠 단독 이동거리: {d_raw:.2f}m (목표 대비 {(d_raw-args.target_dist)*100:+.1f}cm, '
          f'EKF 대비 {(d_raw-d_ekf)*100:+.1f}cm)')
    print(f'\n  EKF 최종 위치: ({r.odom_ekf[0]:+.3f}, {r.odom_ekf[1]:+.3f})')
    print(f'  휠 최종 위치  : ({r.odom_raw[0]:+.3f}, {r.odom_raw[1]:+.3f})')
    print('\n👉 지금 로봇의 실제 위치를 눈으로/실측으로 확인해서, 목표 지점과'
          '\n   얼마나 떨어져 있는지 알려줘 — 그게 이 시험의 진짜 정답이다.')

    if args.json:
        # end_raw/end_ekf 키명은 tools/waypoint_error.py(Issue #3, Resie)의
        # load_log()가 기대하는 스키마와 맞춤 — 이 스크립트로 찍은 로그를
        # 바로 그 도구로 채점할 수 있게.
        json.dump({'date': time.strftime('%Y-%m-%d %H:%M'),
                   'target_name': args.target_name, 'target_dist': args.target_dist,
                   'speed': args.speed, 'start_raw': start_raw, 'start_ekf': start_ekf,
                   'end_raw': r.odom_raw, 'end_ekf': r.odom_ekf,
                   'd_raw_final': d_raw, 'd_ekf_final': d_ekf,
                   'rows': rows}, open(args.json, 'w'), ensure_ascii=False, indent=1)
        print(f'\n저장: {args.json}')

    r.n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
