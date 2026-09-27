#!/usr/bin/env python3
"""2층 위치 추정 — 정지 상태 방향 흐름 측정. 로봇을 움직이지 않고 N초 동안 자이로(/imu/data z)와 EKF 방향을 기록.
정지 중엔 IMU 노드의 영점 추적(ZRU)이 돌아야 하므로 EKF 방향 흐름은 거의 0이어야 한다.
도크 위(충전 중)여도 된다 — 자이로는 파이 3.3 V로 동작해 배터리 전압과 무관.

    python3 tools/still_drift_test.py [초, 기본 300]
"""
import json, math, os, sys, time
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from turn_to_angle import yaw_of


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    rclpy.init(); n = rclpy.create_node('still_drift_test')
    G = []; E = []
    n.create_subscription(Imu, '/imu/data', lambda m: G.append((time.time(), m.angular_velocity.z)), 100)
    n.create_subscription(Odometry, '/odometry/filtered', lambda m: E.append((time.time(), yaw_of(m.pose.pose.orientation))), 20)
    t0 = time.time(); last = t0
    while time.time() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - last >= 60 and G and E:
            last = time.time()
            g = [x for t, x in G if t > last - 60]
            ey = E[-1][1] - E[0][1]
            print(f'[{last - t0:4.0f}s] 최근 1분 자이로 평균 {sum(g)/len(g):+.5f} rad/s | EKF 방향 누적 {math.degrees((ey + math.pi) % (2*math.pi) - math.pi):+.2f}°', flush=True)
    gi = 0.0
    for (ta, a), (tb, b) in zip(G, G[1:]):
        gi += a * (tb - ta)
    ey = (E[-1][1] - E[0][1] + math.pi) % (2 * math.pi) - math.pi
    mean = sum(x for _, x in G) / len(G)
    res = dict(dur=dur, gyro_mean=mean, gyro_int_deg=math.degrees(gi), ekf_drift_deg=math.degrees(ey),
               ekf_rate_deg_per_min=math.degrees(ey) / dur * 60)
    print(f'{dur:.0f}초 정지: 자이로 평균 {mean:+.5f} rad/s, 자이로 적분 {res["gyro_int_deg"]:+.2f}°, EKF 방향 흐름 {res["ekf_drift_deg"]:+.2f}° ({res["ekf_rate_deg_per_min"]:+.2f}°/분)')
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(repo, 'docs/calibration', f'still_drift_{time.strftime("%Y%m%d_%H%M")}.json')
    json.dump(res, open(out, 'w'), indent=1); print('저장:', out)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
