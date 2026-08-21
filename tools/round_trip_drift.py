#!/usr/bin/env python3
"""전진-후진을 N회 반복해서 **원위치 대비 실제 위치 드리프트**를 잰다.

straight_check.py는 자이로 적분으로 요(yaw) 이탈각만 본다. 이건 그보다 한 단계
더 나가서, 오도메트리가 보고하는 **위치(x,y)**가 왕복을 반복할수록 원점에서
얼마나 벌어지는지를 직접 잰다 — 맵핑 전에 위치 보정이 실사용 수준인지 확인하는
용도.

/odom(순수 휠)과 /odometry/filtered(EKF, 휠+자이로 융합) **둘 다** 동시에 기록해서
비교한다 — 자이로 융합이 실제로 도움되는지, 아니면 반대로 오차를 더 키우는지
이번 기회에 같이 보인다.

⚠️ 바닥에서, 앞이 1m 정도 트인 곳. 1회 왕복(전진+후진) 후 이론상 원위치로
   돌아와야 한다 — 실제로 얼마나 못 돌아오는지가 이 측정의 핵심.

사용법:
    python3 tools/round_trip_drift.py --cycles 20 --speed 0.2 --sec 2.5
"""
import argparse, json, math, signal, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    # z축 회전만 있다고 가정(2D 평면 주행)
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Runner:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node('round_trip_drift')
        self.odom_raw = None      # 최신 /odom (x, y, yaw)
        self.odom_ekf = None      # 최신 /odometry/filtered (x, y, yaw)
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

    def spin(self, sec, v=None, flag=None):
        end = time.time() + sec
        while time.time() < end:
            if flag and flag['v']: return
            if v is not None: self.drive(v)
            rclpy.spin_once(self.n, timeout_sec=0.01)


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--sec', type=float, default=2.5)
    ap.add_argument('--cycles', type=int, default=20)
    ap.add_argument('--battery-v', type=float)
    ap.add_argument('--json')
    args = ap.parse_args()

    r = Runner()
    # ⚠️ 구독 확인 — 붙기 전 구간은 위치가 안 잡혀서 원점 기준이 틀어진다
    # (2026-08-19에 이 확인이 없어서 "1회차만 성적이 좋다"는 유령현상을 쫓을 뻔함).
    d = time.time() + 10
    while time.time() < d and (r.odom_raw is None or r.odom_ekf is None):
        rclpy.spin_once(r.n, timeout_sec=0.02)
    if r.odom_raw is None or r.odom_ekf is None:
        print('❌ /odom 또는 /odometry/filtered 수신 실패'); return 1

    start_raw, start_ekf = r.odom_raw, r.odom_ekf
    print(f'시작 위치 — /odom: ({start_raw[0]:+.3f}, {start_raw[1]:+.3f}) '
          f'{math.degrees(start_raw[2]):+.1f}°   '
          f'/odometry/filtered: ({start_ekf[0]:+.3f}, {start_ekf[1]:+.3f}) '
          f'{math.degrees(start_ekf[2]):+.1f}°\n')

    flag = {'v': False}
    signal.signal(signal.SIGINT, lambda *a: flag.update(v=True))
    signal.signal(signal.SIGTERM, lambda *a: flag.update(v=True))

    rows = []
    try:
        for i in range(args.cycles):
            if flag['v']: break
            r.spin(0.8, None, flag)
            r.spin(args.sec, args.speed, flag)
            r.stop()
            r.spin(0.8, None, flag)
            r.spin(args.sec, -args.speed, flag)
            r.stop()
            r.spin(0.8, None, flag)

            d_raw = dist(r.odom_raw, start_raw)
            d_ekf = dist(r.odom_ekf, start_ekf)
            yaw_raw = math.degrees(r.odom_raw[2] - start_raw[2])
            yaw_ekf = math.degrees(r.odom_ekf[2] - start_ekf[2])
            rows.append({'cycle': i + 1,
                        'odom_raw': r.odom_raw, 'odom_ekf': r.odom_ekf,
                        'drift_raw_m': d_raw, 'drift_ekf_m': d_ekf,
                        'yaw_raw_deg': yaw_raw, 'yaw_ekf_deg': yaw_ekf})
            print(f'  {i+1:>2}/{args.cycles}  '
                  f'휠 드리프트 {d_raw*100:>5.1f}cm (요 {yaw_raw:>+6.1f}°)   '
                  f'EKF 드리프트 {d_ekf*100:>5.1f}cm (요 {yaw_ekf:>+6.1f}°)')
    finally:
        r.stop()

    if rows:
        last = rows[-1]
        print(f'\n=== {len(rows)}회 왕복 후 최종 드리프트 ===')
        print(f'  휠 단독(/odom)            : {last["drift_raw_m"]*100:.1f}cm, '
              f'요 {last["yaw_raw_deg"]:+.1f}°')
        print(f'  EKF(휠+자이로 융합)        : {last["drift_ekf_m"]*100:.1f}cm, '
              f'요 {last["yaw_ekf_deg"]:+.1f}°')
        per_cycle_raw = last['drift_raw_m'] / len(rows) * 100
        per_cycle_ekf = last['drift_ekf_m'] / len(rows) * 100
        print(f'\n  왕복 1회당 평균 드리프트: 휠 {per_cycle_raw:.2f}cm/회, '
              f'EKF {per_cycle_ekf:.2f}cm/회')
        if last['drift_ekf_m'] < last['drift_raw_m']:
            print('  -> 자이로 융합이 순수 휠보다 낫다.')
        else:
            print('  -> ⚠️ 자이로 융합이 순수 휠보다 오히려 나쁘다 — 확인 필요.')

    if args.json:
        json.dump({'date': time.strftime('%Y-%m-%d %H:%M'), 'speed': args.speed,
                   'sec': args.sec, 'battery_v': args.battery_v,
                   'start_raw': start_raw, 'start_ekf': start_ekf,
                   'rows': rows}, open(args.json, 'w'), ensure_ascii=False, indent=1)
        print(f'\n저장: {args.json}')

    r.n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
