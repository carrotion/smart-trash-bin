#!/usr/bin/env python3
"""직진 중 작은 회전 명령이 실제 회전으로 얼마나 이어지는지(곡선 주행 이득) 측정.

왜 (2026-09-26): 정사각형 시험에서 직진 구간마다 방향이 1~2°씩 흘렀고, 구동 노드의 자이로 직진
루프가 좌우 속도 차를 요구해도 실제 좌우 바퀴 속도는 같았다. 스키드 조향은 옆으로 미끄러져야
돌기 때문에, 달리는 중 작은 회전 명령엔 반응하지 않는 구간(요 데드밴드)이 있는지 확인한다.

동작: v로 달리며 w를 일정하게 준다(앞/뒤 번갈아 dur초). 1~dur초 구간의 자이로 평균을 명령과 비교.
⚠️ 바닥 전용, 앞뒤 0.3 m + 이동 거리 여유 검사, 첫 동작 뒤 도크 검사.

    python3 tools/curve_gain_test.py --v 0.15 --ws 0.05 0.1 0.2 0.3 --dur 3
"""
import argparse, json, os, sys, time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from speed_floor_sweep import corridor_clearance
from dock_guard import looks_on_dock
from wheel_config import wheel_distances


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--v', type=float, default=0.15)
    ap.add_argument('--ws', type=float, nargs='+', default=[0.05, 0.1, 0.2, 0.3])
    ap.add_argument('--dur', type=float, default=3.0)
    ap.add_argument('--label', default='')
    a = ap.parse_args()
    rclpy.init(); n = rclpy.create_node('curve_gain_test')
    S = {'g': [], 'scan': None, 'enc': None}
    n.create_subscription(Imu, '/imu/data', lambda m: S['g'].append((time.time(), m.angular_velocity.z)), 100)
    n.create_subscription(LaserScan, '/scan', lambda m: S.update(scan=m), 5)
    n.create_subscription(Int32MultiArray, '/wheel_encoders', lambda m: S.update(enc=list(m.data)), 20)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(n, timeout_sec=0.005)

    def stop():
        for _ in range(5):
            pub.publish(Twist()); spin(0.03)

    t0 = time.time()
    while (S['scan'] is None or S['enc'] is None) and time.time() - t0 < 10:
        spin(0.1)
    rows = []; first = True
    need = a.v * a.dur + 0.3
    try:
        for w in a.ws:
            for sw in (1, -1):              # 좌/우 곡선
                for sv in (1, -1):          # 앞/뒤(제자리 근처 유지)
                    clear = corridor_clearance(S['scan'], sv > 0, 0.0 if sv > 0 else 0.1)
                    if clear < need:
                        print(f'❌ {"앞" if sv > 0 else "뒤"} 여유 {clear:.2f} m < {need:.2f} — 중단'); return 1
                    scan0, e0 = S['scan'], list(S['enc'])
                    spin(1.0)
                    S['g'] = []; ts = time.time()
                    while time.time() - ts < a.dur:
                        c = Twist(); c.linear.x = sv * a.v; c.angular.z = sw * w; pub.publish(c); spin(0.02)
                    stop(); spin(0.8)
                    if first:
                        d = wheel_distances(e0, S['enc'])
                        bad, msg = looks_on_dock((d[0] + d[1]) / 2, scan0, S['scan']); print(msg)
                        if bad:
                            return 1
                        first = False
                    g = [v for t, v in S['g'] if ts + 1.0 <= t <= ts + a.dur]
                    meas = sum(g) / len(g) if g else float('nan')
                    r = dict(v=sv * a.v, w_cmd=sw * w, w_meas=meas, ratio=meas / (sw * w))
                    rows.append(r)
                    print(f'v {sv * a.v:+.2f} w {sw * w:+.2f} → 실제 {meas:+.3f} rad/s (비율 {r["ratio"]:.2f})', flush=True)
    finally:
        stop()
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(repo, 'docs/calibration', f'curve_gain_{time.strftime("%Y%m%d_%H%M")}{a.label}.json')
    json.dump(rows, open(out, 'w'), ensure_ascii=False, indent=1); print('저장:', out)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
