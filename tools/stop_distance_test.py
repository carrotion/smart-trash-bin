#!/usr/bin/env python3
"""정지거리 측정 — 정지 명령 후 더 간 거리·걸린 시간(충돌 감시 정지 구역 크기의 근거, todo #12).

동작: 속도 v로 run초 달려 안정시킨 뒤 cmd_vel 0 → 바퀴 엔코더(앞바퀴 평균)와 EKF로 정지까지 이동량 측정.
앞/뒤 번갈아(제자리 근처 유지). 첫 동작 뒤 도크 검사. ⚠️ 바닥 전용.

    python3 tools/stop_distance_test.py --vs 0.15 0.3 --reps 2
"""
import argparse, json, math, os, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from speed_floor_sweep import corridor_clearance
from dock_guard import looks_on_dock
from wheel_config import wheel_distances


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--vs', type=float, nargs='+', default=[0.15, 0.3])
    ap.add_argument('--reps', type=int, default=2)
    ap.add_argument('--run', type=float, default=1.5)
    a = ap.parse_args()
    rclpy.init(); n = rclpy.create_node('stop_distance_test')
    S = {'scan': None, 'enc': [], 'ekf': None}
    n.create_subscription(LaserScan, '/scan', lambda m: S.update(scan=m), 5)
    n.create_subscription(Int32MultiArray, '/wheel_encoders', lambda m: S['enc'].append((time.time(), list(m.data))), 50)
    n.create_subscription(Odometry, '/odometry/filtered', lambda m: S.update(
        ekf=(m.pose.pose.position.x, m.pose.pose.position.y)), 20)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(n, timeout_sec=0.005)

    t0 = time.time()
    while (S['scan'] is None or not S['enc'] or S['ekf'] is None) and time.time() - t0 < 10:
        spin(0.1)
    rows = []; first = True
    try:
        for v in a.vs:
            for rep in range(a.reps):
                for sv in (1, -1):
                    need = v * a.run + 0.5
                    clear = corridor_clearance(S['scan'], sv > 0, 0.0 if sv > 0 else 0.1)
                    if clear < need:
                        print(f'❌ {"앞" if sv > 0 else "뒤"} 여유 {clear:.2f} m < {need:.2f} — 중단'); return 1
                    scan0, e_start = S['scan'], S['enc'][-1][1]
                    spin(1.0)
                    ts = time.time()
                    while time.time() - ts < a.run:
                        c = Twist(); c.linear.x = sv * v; pub.publish(c); spin(0.02)
                    # 정지 순간
                    t_stop = time.time(); e_stop = S['enc'][-1][1]; k_stop = S['ekf']
                    for _ in range(5):
                        pub.publish(Twist()); spin(0.02)
                    spin(1.5)
                    # 멈춘 시각: 엔코더 앞바퀴 평균이 더 안 바뀌는 첫 시각
                    tail = [(t, d) for t, d in S['enc'] if t >= t_stop]
                    last_move = t_stop
                    for (ta, da), (tb, db) in zip(tail, tail[1:]):
                        if any(abs(db[i] - da[i]) >= 2 for i in (0, 1)):
                            last_move = tb
                    d_enc = wheel_distances(e_stop, S['enc'][-1][1])
                    d = sv * (d_enc[0] + d_enc[1]) / 2
                    k1 = S['ekf']; d_ekf = math.hypot(k1[0] - k_stop[0], k1[1] - k_stop[1])
                    if first:
                        dd = wheel_distances(e_start, S['enc'][-1][1])
                        bad, msg = looks_on_dock((dd[0] + dd[1]) / 2, scan0, S['scan']); print(msg)
                        if bad:
                            return 1
                        first = False
                    r = dict(v=sv * v, stop_dist_enc_m=d, stop_dist_ekf_m=d_ekf, stop_time_s=last_move - t_stop)
                    rows.append(r)
                    print(f'v {sv * v:+.2f}: 정지 명령 후 {d * 100:.1f} cm(엔코더) / {d_ekf * 100:.1f} cm(EKF), '
                          f'{r["stop_time_s"]:.2f}초에 정지', flush=True)
    finally:
        for _ in range(5):
            pub.publish(Twist()); spin(0.02)
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(repo, 'docs/calibration', f'stop_distance_{time.strftime("%Y%m%d_%H%M")}.json')
    json.dump(rows, open(out, 'w'), ensure_ascii=False, indent=1); print('저장:', out)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
